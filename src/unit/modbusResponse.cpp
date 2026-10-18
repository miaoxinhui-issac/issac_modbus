#include "issac_modbus/unit/modbusResponse.h"

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "issac_modbus/unit/crc.h"
#include "issac_modbus/unit/modbusProtocol.h"


namespace issac_modbus
{
/// @brief 根据结构化字段创建并校验 Modbus 响应。
ModbusResponse::ModbusResponse(uint8_t unitId, MBfunctionCode functionCode, uint16_t address,
                               uint16_t registersNumber, std::vector<ModbusCell> values)
    : unitId_(unitId), functionCode_(functionCode), address_(address),
      registersNumber_(registersNumber), values_(std::move(values))
{
    const MBFunctionType type = issac_modbus::functionType(functionCode_);

    if (functionCode_ == MBfunctionCode::MBReportSlaveID)
    {
        if (registersNumber_ == 0 && !values_.empty())
        {
            registersNumber_ = static_cast<uint16_t>(values_.size());
        }
        normalizeValues(functionCode_, values_);
        return;
    }

    if (functionCode_ == MBfunctionCode::MBMaskWriteRegister)
    {
        if (values_.size() != 2)
        {
            throw std::invalid_argument("Mask write response requires AND and OR values.");
        }
        registersNumber_ = 2;
        normalizeValues(functionCode_, values_);
        return;
    }

    if (type == Read || type == ReadWrite)
    {
        if (registersNumber_ == 0)
        {
            registersNumber_ = static_cast<uint16_t>(values_.size());
        }
        if (values_.empty() && registersNumber_ != 0)
        {
            values_.resize(registersNumber_);
        }
        normalizeValues(functionCode_, values_);
        return;
    }

    if (type == WriteSingle)
    {
        if (values_.size() != 1)
        {
            throw std::invalid_argument("Write single response requires exactly one value.");
        }
        registersNumber_ = 1;
        normalizeValues(functionCode_, values_);
        return;
    }

    if (type == WriteMultiple)
    {
        if (registersNumber_ == 0)
        {
            registersNumber_ = static_cast<uint16_t>(values_.size());
        }
        // A write-multiple response only echoes the address and quantity.
        values_.clear();
    }
}

/// @brief 从原始响应字节中解析功能码、地址和值。
ModbusResponse::ModbusResponse(const std::vector<uint8_t>& inputData, bool verifyCRC)
{
    const std::size_t crcLength = verifyCRC ? 2 : 0;

    if (inputData.size() < 2 + crcLength)
    {
        throw std::invalid_argument("Modbus response is too short.");
    }

    const std::size_t payloadLength = inputData.size() - crcLength;

    if (verifyCRC)
    {
        const uint16_t receivedCRC = static_cast<uint16_t>(inputData[payloadLength]) |
                                     static_cast<uint16_t>(inputData[payloadLength + 1] << 8u);
        const uint16_t calculatedCRC = calculateCRC(inputData.data(), payloadLength);

        if (receivedCRC != calculatedCRC)
        {
            throw std::invalid_argument("Modbus response CRC is invalid.");
        }
    }

    unitId_ = inputData[0];
    issac_modbus::validateUnitId(unitId_);
    functionCode_ = static_cast<MBfunctionCode>(inputData[1]);

    if ((static_cast<uint8_t>(functionCode_) & 0x80u) != 0)
    {
        throw std::invalid_argument("Modbus exception response is not supported.");
    }

    switch (functionCode_)
    {
    case MBfunctionCode::MBReadCoils:
    case MBfunctionCode::MBReadDiscreteInputs:
    {
        if (payloadLength < 3)
        {
            throw std::invalid_argument("Invalid coil response.");
        }

        const std::size_t byteCount = inputData[2];
        if (payloadLength != 3 + byteCount)
        {
            throw std::invalid_argument("Invalid coil response byte count.");
        }

        registersNumber_ = static_cast<uint16_t>(byteCount * 8u);
        values_.clear();
        values_.reserve(registersNumber_);
        for (std::size_t i = 0; i < registersNumber_; ++i)
        {
            const bool enabled = (inputData[3 + i / 8] & (1u << (i % 8))) != 0;
            values_.emplace_back(enabled);
        }
        break;
    }

    case MBfunctionCode::MBReadHoldingRegisters:
    case MBfunctionCode::MBReadInputRegisters:
    case MBfunctionCode::MBReadWriteMultipleRegisters:
    {
        if (payloadLength < 3)
        {
            throw std::invalid_argument("Invalid register response.");
        }

        const std::size_t byteCount = inputData[2];
        if (byteCount % 2u != 0 || payloadLength != 3 + byteCount)
        {
            throw std::invalid_argument("Invalid register response byte count.");
        }

        registersNumber_ = static_cast<uint16_t>(byteCount / 2u);
        values_.clear();
        values_.reserve(registersNumber_);
        for (std::size_t i = 0; i < registersNumber_; ++i)
        {
            values_.emplace_back(readUint16(
                inputData, 3 + i * 2, payloadLength,
                "Modbus response contains incomplete data."));
        }
        break;
    }

    case MBfunctionCode::MBWriteSingleCoil:
    {
        if (payloadLength != 6)
        {
            throw std::invalid_argument("Invalid write single coil response.");
        }
        address_ = readUint16(inputData, 2, payloadLength,
                              "Modbus response contains incomplete data.");
        const uint16_t value = readUint16(inputData, 4, payloadLength,
                                          "Modbus response contains incomplete data.");
        if (value != 0x0000u && value != 0xFF00u)
        {
            throw std::invalid_argument("Invalid write single coil value.");
        }
        registersNumber_ = 1;
        values_ = {ModbusCell(value == 0xFF00u)};
        break;
    }

    case MBfunctionCode::MBWriteSingleRegister:
        if (payloadLength != 6)
        {
            throw std::invalid_argument("Invalid write single register response.");
        }
        address_ = readUint16(inputData, 2, payloadLength,
                              "Modbus response contains incomplete data.");
        registersNumber_ = 1;
        values_ = {ModbusCell(readUint16(inputData, 4, payloadLength,
                                         "Modbus response contains incomplete data."))};
        break;

    case MBfunctionCode::MBWriteMultipleCoils:
    case MBfunctionCode::MBWriteMultipleRegisters:
        if (payloadLength != 6)
        {
            throw std::invalid_argument("Invalid write multiple response.");
        }
        address_ = readUint16(inputData, 2, payloadLength,
                              "Modbus response contains incomplete data.");
        registersNumber_ = readUint16(inputData, 4, payloadLength,
                                      "Modbus response contains incomplete data.");
        if (registersNumber_ == 0)
        {
            throw std::invalid_argument("Write response count cannot be zero.");
        }
        values_.clear();
        break;

    case MBfunctionCode::MBReportSlaveID:
    {
        if (payloadLength < 3)
        {
            throw std::invalid_argument("Invalid report slave ID response.");
        }

        const std::size_t byteCount = inputData[2];
        if (payloadLength != 3 + byteCount)
        {
            throw std::invalid_argument("Invalid report slave ID response byte count.");
        }

        registersNumber_ = static_cast<uint16_t>(byteCount);
        values_.clear();
        values_.reserve(byteCount);
        for (std::size_t i = 0; i < byteCount; ++i)
        {
            values_.emplace_back(static_cast<uint16_t>(inputData[3 + i]));
        }
        break;
    }

    case MBfunctionCode::MBMaskWriteRegister:
        if (payloadLength != 8)
        {
            throw std::invalid_argument("Invalid mask write response.");
        }
        address_ = readUint16(inputData, 2, payloadLength,
                              "Modbus response contains incomplete data.");
        registersNumber_ = 2;
        values_ = {ModbusCell(readUint16(inputData, 4, payloadLength,
                                         "Modbus response contains incomplete data.")),
                   ModbusCell(readUint16(inputData, 6, payloadLength,
                                         "Modbus response contains incomplete data."))};
        break;
    default:
        throw std::invalid_argument("Unsupported Modbus response function.");
    }
}

/// @brief 根据请求创建可继续填充的响应骨架。
ModbusResponse ModbusResponse::from(const ModbusRequest& request)
{
    if (request.functionCode() == MBfunctionCode::MBReadWriteMultipleRegisters)
    {
        return ModbusResponse(request.unitId(), request.functionCode(), request.registerAddress(),
                              request.numberOfRegisters(), {});
    }

    return ModbusResponse(request.unitId(), request.functionCode(), request.registerAddress(),
                          request.numberOfRegisters(), request.registerValues());
}

/// @brief 生成便于日志和调试的响应描述文本。
std::string ModbusResponse::toString() const
{
    std::ostringstream result;

    result << functionCodeToString(functionCode_) << ", from unit "
           << static_cast<unsigned int>(unitId_);

    if (functionCode_ == MBfunctionCode::MBReportSlaveID)
    {
        result << ", data = { ";
        for (std::size_t i = 0; i < values_.size(); ++i)
        {
            if (i != 0)
            {
                result << ", ";
            }
            result << values_[i].toString();
        }
        result << " }";
        return result.str();
    }

    result << ", address " << address_;

    if (functionCode_ == MBfunctionCode::MBMaskWriteRegister)
    {
        if (values_.size() != 2)
        {
            result << ", invalid mask values";
        }
        else
        {
            result << ", AND mask " << values_[0].toString() << ", OR mask "
                   << values_[1].toString();
        }
        return result.str();
    }

    if (functionType() == WriteMultiple)
    {
        result << ", count " << registersNumber_;
        return result.str();
    }

    if (functionType() == WriteSingle)
    {
        if (values_.empty())
        {
            result << ", value unavailable";
        }
        else
        {
            result << ", value " << values_.front().toString();
        }
        return result.str();
    }

    result << ", count " << registersNumber_ << ", values = { ";
    for (std::size_t i = 0; i < values_.size(); ++i)
    {
        if (i != 0)
        {
            result << ", ";
        }
        result << values_[i].toString();
    }
    result << " }";

    return result.str();
}

/// @brief 计算响应中随功能码变化的数据字节数量。
uint16_t ModbusResponse::numberOfBytesToFollow() const
{
    if (functionCode_ == MBfunctionCode::MBReportSlaveID)
    {
        return static_cast<uint16_t>(1 + values_.size());
    }

    if (functionCode_ == MBfunctionCode::MBMaskWriteRegister)
    {
        return 6;
    }

    if (functionType() == WriteSingle || functionType() == WriteMultiple)
    {
        return 4;
    }

    if (functionRegisters() == OutputCoils || functionRegisters() == InputCoils)
    {
        return static_cast<uint16_t>((values_.size() + 7u) / 8u);
    }

    return static_cast<uint16_t>(values_.size() * 2u);
}

/// @brief 序列化不含 CRC 的 Modbus 响应字节。
std::vector<uint8_t> ModbusResponse::toRaw() const
{
    std::vector<uint8_t> result;
    result.reserve(7 + values_.size() * 2);

    result.push_back(unitId_);
    result.push_back(static_cast<uint8_t>(functionCode_));

    if (functionCode_ == MBfunctionCode::MBReadCoils ||
        functionCode_ == MBfunctionCode::MBReadDiscreteInputs)
    {
        if (values_.empty())
        {
            throw std::invalid_argument("Coil response data is empty.");
        }

        const uint8_t byteCount = static_cast<uint8_t>((values_.size() + 7u) / 8u);
        result.push_back(byteCount);
        const std::size_t start = result.size();
        result.resize(start + byteCount, 0);

        for (std::size_t i = 0; i < values_.size(); ++i)
        {
            if (!values_[i].isCoil())
            {
                throw std::invalid_argument("Coil response requires coil values.");
            }
            if (values_[i].coil())
            {
                result[start + i / 8] |= static_cast<uint8_t>(1u << (i % 8));
            }
        }

        return result;
    }

    if (functionCode_ == MBfunctionCode::MBReadHoldingRegisters ||
        functionCode_ == MBfunctionCode::MBReadInputRegisters ||
        functionCode_ == MBfunctionCode::MBReadWriteMultipleRegisters)
    {
        if (values_.empty())
        {
            throw std::invalid_argument("Register response data is empty.");
        }

        const uint16_t byteCount = static_cast<uint16_t>(values_.size() * 2u);
        if (byteCount > 0xFFu)
        {
            throw std::invalid_argument("Register response exceeds the byte count field.");
        }
        result.push_back(static_cast<uint8_t>(byteCount));

        for (const ModbusCell& value : values_)
        {
            if (!value.isReg())
            {
                throw std::invalid_argument("Register response requires register values.");
            }
            appendUint16(result, value.reg());
        }

        return result;
    }

    if (functionCode_ == MBfunctionCode::MBWriteSingleCoil)
    {
        if (values_.size() != 1 || !values_.front().isCoil())
        {
            throw std::invalid_argument("Write single coil response requires one coil value.");
        }
        appendUint16(result, address_);
        appendUint16(result, values_.front().coil() ? 0xFF00u : 0x0000u);
        return result;
    }

    if (functionCode_ == MBfunctionCode::MBWriteSingleRegister)
    {
        if (values_.size() != 1 || !values_.front().isReg())
        {
            throw std::invalid_argument(
                "Write single register response requires one register value.");
        }
        appendUint16(result, address_);
        appendUint16(result, values_.front().reg());
        return result;
    }

    if (functionCode_ == MBfunctionCode::MBWriteMultipleCoils ||
        functionCode_ == MBfunctionCode::MBWriteMultipleRegisters)
    {
        appendUint16(result, address_);
        appendUint16(result, registersNumber_);
        return result;
    }

    if (functionCode_ == MBfunctionCode::MBReportSlaveID)
    {
        if (values_.size() > 0xFFu)
        {
            throw std::invalid_argument("Report slave ID response exceeds the byte count field.");
        }

        result.push_back(static_cast<uint8_t>(values_.size()));
        for (const ModbusCell& value : values_)
        {
            if (!value.isReg() || value.reg() > 0xFFu)
            {
                throw std::invalid_argument("Report slave ID requires byte-sized register values.");
            }
            result.push_back(static_cast<uint8_t>(value.reg()));
        }
        return result;
    }

    if (functionCode_ == MBfunctionCode::MBMaskWriteRegister)
    {
        if (values_.size() != 2 || !values_[0].isReg() || !values_[1].isReg())
        {
            throw std::invalid_argument("Mask write response requires AND and OR values.");
        }
        appendUint16(result, address_);
        appendUint16(result, values_[0].reg());
        appendUint16(result, values_[1].reg());
        return result;
    }

    throw std::invalid_argument("Unsupported Modbus response function.");
}

/// @brief 返回当前功能码对应的操作类型。
MBFunctionType ModbusResponse::functionType() const
{
    return issac_modbus::functionType(functionCode_);
}

/// @brief 返回当前功能码对应的数据区类型。
MBFunctionRegisters ModbusResponse::functionRegisters() const
{
    return issac_modbus::functionRegister(functionCode_);
}


} // namespace issac_modbus
