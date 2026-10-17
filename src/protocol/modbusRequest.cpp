#include "issac/modbus/protocol/modbusRequest.h"

#include "issac/modbus/protocol/crc.h"
#include "issac/modbus/protocol/modbusProtocol.h"

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

/// @brief 向字节容器追加一个 Modbus 大端 16 位值。
static void appendUint16(std::vector<uint8_t>& buffer, uint16_t value)
{
    buffer.push_back(static_cast<uint8_t>(value >> 8u));
    buffer.push_back(static_cast<uint8_t>(value & 0xFFu));
}

/// @brief 从字节容器读取一个 Modbus 大端 16 位值。
/// @param buffer 输入报文字节。
/// @param offset 读取起始位置。
/// @param limit 可读取的最大位置，不包括 CRC。
/// @return 读取到的 16 位值。
/// @throws std::invalid_argument 数据越界时抛出。
static uint16_t readUint16(const std::vector<uint8_t>& buffer, std::size_t offset,
                           std::size_t limit)
{
    if (offset > limit || limit - offset < 2)
    {
        throw std::invalid_argument("Modbus request contains incomplete data.");
    }

    return bigEndianConv(buffer.data() + offset, 2);
}

/// @brief 将功能码转换为可读文本。
static std::string_view functionCodeToString(MBfunctionCode functionCode)
{
    switch (functionCode)
    {
    case MBfunctionCode::MBReadCoils:
        return "Read Coils";
    case MBfunctionCode::MBReadDiscreteInputs:
        return "Read Discrete Inputs";
    case MBfunctionCode::MBReadHoldingRegisters:
        return "Read Holding Registers";
    case MBfunctionCode::MBReadInputRegisters:
        return "Read Input Registers";
    case MBfunctionCode::MBWriteSingleCoil:
        return "Write Single Coil";
    case MBfunctionCode::MBWriteSingleRegister:
        return "Write Single Register";
    case MBfunctionCode::MBWriteMultipleCoils:
        return "Write Multiple Coils";
    case MBfunctionCode::MBWriteMultipleRegisters:
        return "Write Multiple Registers";
    case MBfunctionCode::MBReportSlaveID:
        return "Report Slave ID";
    case MBfunctionCode::MBMaskWriteRegister:
        return "Mask Write Register";
    case MBfunctionCode::MBReadWriteMultipleRegisters:
        return "Read/Write Multiple Registers";
    }

    return "Unknown Function";
}

/// @brief 将值列表转换为当前功能码要求的线圈或寄存器类型。
static void normalizeValues(MBfunctionCode functionCode, std::vector<ModbusCell>& values)
{
    const MBFunctionRegisters registerType = functionRegister(functionCode);

    if (registerType == OutputCoils || registerType == InputCoils)
    {
        for (ModbusCell& value : values)
        {
            value.coil();
        }
    }
    else if (registerType == HoldingRegisters || registerType == InputRegisters)
    {
        for (ModbusCell& value : values)
        {
            value.Reg();
        }
    }
}

/// @brief 根据结构化字段创建并校验 Modbus 请求。
ModbusRequest::ModbusRequest(uint8_t slaveId, MBfunctionCode functionCode, uint16_t address,
                             uint16_t registersNumber, std::vector<ModbusCell> values,
                             uint16_t writeAddress)
    : slaveID_(slaveId), functionCode_(functionCode), address_(address),
      registersNumber_(registersNumber), writeAddress_(writeAddress), values_(std::move(values))
{
    MB::validateUnitId(slaveId);
    const MBFunctionType type = ::functionType(functionCode_);

    if (functionCode_ == MBfunctionCode::MBReadWriteMultipleRegisters)
    {
        if (registersNumber_ == 0 || values_.empty())
        {
            throw std::invalid_argument(
                "Read/write multiple registers requires read and write data.");
        }
    }
    else if (functionCode_ == MBfunctionCode::MBMaskWriteRegister)
    {
        if (values_.size() != 2)
        {
            throw std::invalid_argument("Mask write register requires AND and OR values.");
        }
        registersNumber_ = 2;
    }
    else if (type == Read)
    {
        if (!values_.empty())
        {
            throw std::invalid_argument("Read request cannot contain values.");
        }
        if (functionCode_ != MBfunctionCode::MBReportSlaveID && registersNumber_ == 0)
        {
            throw std::invalid_argument("Read request requires a non-zero register count.");
        }
    }
    else if (type == WriteSingle)
    {
        if (values_.size() != 1)
        {
            throw std::invalid_argument("Write single request requires exactly one value.");
        }
        registersNumber_ = 1;
    }
    else if (type == WriteMultiple)
    {
        if (registersNumber_ == 0)
        {
            registersNumber_ = static_cast<uint16_t>(values_.size());
        }
        if (values_.size() != registersNumber_)
        {
            throw std::invalid_argument("Register count does not match the number of values.");
        }
    }

    normalizeValues(functionCode_, values_);
    MB::validateRequestLimits(*this);
}

/// @brief 从原始请求字节中解析功能码、地址和值。
ModbusRequest::ModbusRequest(const std::vector<uint8_t>& inputData, bool verifyCRC)
{
    const std::size_t crcLength = verifyCRC ? 2 : 0;

    if (inputData.size() < 2 + crcLength)
    {
        throw std::invalid_argument("Modbus request is too short.");
    }

    const std::size_t payloadLength = inputData.size() - crcLength;

    if (verifyCRC)
    {
        const uint16_t receivedCRC = static_cast<uint16_t>(inputData[payloadLength]) |
                                     static_cast<uint16_t>(inputData[payloadLength + 1] << 8u);
        const uint16_t calculatedCRC = calculateCRC(inputData.data(), payloadLength);

        if (receivedCRC != calculatedCRC)
        {
            throw std::invalid_argument("Modbus request CRC is invalid.");
        }
    }

    slaveID_ = inputData[0];
    MB::validateUnitId(slaveID_);
    functionCode_ = static_cast<MBfunctionCode>(inputData[1]);

    switch (functionCode_)
    {
    case MBfunctionCode::MBReadCoils:
    case MBfunctionCode::MBReadDiscreteInputs:
    case MBfunctionCode::MBReadHoldingRegisters:
    case MBfunctionCode::MBReadInputRegisters:
        if (payloadLength != 6)
        {
            throw std::invalid_argument("Invalid read request length.");
        }
        address_ = readUint16(inputData, 2, payloadLength);
        registersNumber_ = readUint16(inputData, 4, payloadLength);
        if (registersNumber_ == 0)
        {
            throw std::invalid_argument("Read request count cannot be zero.");
        }
        values_.clear();
        break;

    case MBfunctionCode::MBReportSlaveID:
        if (payloadLength != 2)
        {
            throw std::invalid_argument("Invalid report slave ID request.");
        }
        address_ = 0;
        registersNumber_ = 0;
        values_.clear();
        break;

    case MBfunctionCode::MBWriteSingleCoil:
    {
        if (payloadLength != 6)
        {
            throw std::invalid_argument("Invalid write single coil request.");
        }
        address_ = readUint16(inputData, 2, payloadLength);
        const uint16_t value = readUint16(inputData, 4, payloadLength);
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
            throw std::invalid_argument("Invalid write single register request.");
        }
        address_ = readUint16(inputData, 2, payloadLength);
        registersNumber_ = 1;
        values_ = {ModbusCell(readUint16(inputData, 4, payloadLength))};
        break;

    case MBfunctionCode::MBWriteMultipleCoils:
    case MBfunctionCode::MBWriteMultipleRegisters:
    {
        if (payloadLength < 7)
        {
            throw std::invalid_argument("Invalid write multiple request.");
        }

        address_ = readUint16(inputData, 2, payloadLength);
        registersNumber_ = readUint16(inputData, 4, payloadLength);
        const std::size_t byteCount = inputData[6];

        if (registersNumber_ == 0)
        {
            throw std::invalid_argument("Write request count cannot be zero.");
        }

        values_.clear();
        values_.reserve(registersNumber_);

        if (functionCode_ == MBfunctionCode::MBWriteMultipleCoils)
        {
            const std::size_t expectedByteCount =
                (static_cast<std::size_t>(registersNumber_) + 7u) / 8u;
            if (byteCount != expectedByteCount || payloadLength != 7 + byteCount)
            {
                throw std::invalid_argument("Invalid coil byte count.");
            }

            for (std::size_t i = 0; i < registersNumber_; ++i)
            {
                const bool enabled = (inputData[7 + i / 8] & (1u << (i % 8))) != 0;
                values_.emplace_back(enabled);
            }
        }
        else
        {
            const std::size_t expectedByteCount = static_cast<std::size_t>(registersNumber_) * 2u;
            if (byteCount != expectedByteCount || payloadLength != 7 + byteCount)
            {
                throw std::invalid_argument("Invalid register byte count.");
            }

            for (std::size_t i = 0; i < registersNumber_; ++i)
            {
                values_.emplace_back(readUint16(inputData, 7 + i * 2, payloadLength));
            }
        }
        break;
    }

    case MBfunctionCode::MBMaskWriteRegister:
        if (payloadLength != 8)
        {
            throw std::invalid_argument("Invalid mask write request length.");
        }
        address_ = readUint16(inputData, 2, payloadLength);
        registersNumber_ = 2;
        values_ = {ModbusCell(readUint16(inputData, 4, payloadLength)),
                   ModbusCell(readUint16(inputData, 6, payloadLength))};
        break;

    case MBfunctionCode::MBReadWriteMultipleRegisters:
    {
        if (payloadLength < 11)
        {
            throw std::invalid_argument("Invalid read/write multiple registers request.");
        }

        address_ = readUint16(inputData, 2, payloadLength);
        registersNumber_ = readUint16(inputData, 4, payloadLength);
        writeAddress_ = readUint16(inputData, 6, payloadLength);
        const uint16_t writeCount = readUint16(inputData, 8, payloadLength);
        const std::size_t byteCount = inputData[10];

        if (registersNumber_ == 0 || writeCount == 0)
        {
            throw std::invalid_argument("Read/write counts must be greater than zero.");
        }

        const std::size_t expectedByteCount = static_cast<std::size_t>(writeCount) * 2u;
        if (byteCount != expectedByteCount || payloadLength != 11 + byteCount)
        {
            throw std::invalid_argument("Invalid read/write register bytes.");
        }

        values_.clear();
        values_.reserve(writeCount);
        for (std::size_t i = 0; i < writeCount; ++i)
        {
            values_.emplace_back(readUint16(inputData, 11 + i * 2, payloadLength));
        }
        break;
    }
    default:
        throw std::invalid_argument("Unsupported Modbus request function.");
    }

    MB::validateRequestLimits(*this);
}

/// @brief 生成便于日志和调试的请求描述文本。
std::string ModbusRequest::toString() const
{
    std::ostringstream result;

    result << functionCodeToString(functionCode_) << ", from slave "
           << static_cast<unsigned int>(slaveID_);

    if (functionCode_ == MBfunctionCode::MBReportSlaveID)
    {
        return result.str();
    }

    result << ", starting from address " << address_;

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

    if (functionCode_ == MBfunctionCode::MBReadWriteMultipleRegisters)
    {
        result << ", read count " << registersNumber_ << ", write from " << writeAddress_
               << ", write count " << values_.size() << ", values = { ";
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

    if (functionType() == Read)
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

/// @brief 序列化不含 CRC 的 Modbus 请求字节。
std::vector<uint8_t> ModbusRequest::toRaw() const
{
    std::vector<uint8_t> result;
    result.reserve(7 + values_.size() * 2);

    result.push_back(slaveID_);
    result.push_back(static_cast<uint8_t>(functionCode_));

    switch (functionCode_)
    {
    case MBfunctionCode::MBReadCoils:
    case MBfunctionCode::MBReadDiscreteInputs:
    case MBfunctionCode::MBReadHoldingRegisters:
    case MBfunctionCode::MBReadInputRegisters:
        appendUint16(result, address_);
        appendUint16(result, registersNumber_);
        break;

    case MBfunctionCode::MBReportSlaveID:
        break;

    case MBfunctionCode::MBWriteSingleCoil:
        if (values_.size() != 1 || !values_.front().isCoil())
        {
            throw std::invalid_argument("Write single coil requires one coil value.");
        }
        appendUint16(result, address_);
        appendUint16(result, values_.front().coil() ? 0xFF00u : 0x0000u);
        break;

    case MBfunctionCode::MBWriteSingleRegister:
        if (values_.size() != 1 || !values_.front().isReg())
        {
            throw std::invalid_argument("Write single register requires one register value.");
        }
        appendUint16(result, address_);
        appendUint16(result, values_.front().reg());
        break;

    case MBfunctionCode::MBWriteMultipleCoils:
    {
        if (values_.size() != registersNumber_)
        {
            throw std::invalid_argument("Register count does not match the number of values.");
        }

        appendUint16(result, address_);
        appendUint16(result, registersNumber_);
        const uint8_t byteCount = static_cast<uint8_t>((registersNumber_ + 7u) / 8u);
        result.push_back(byteCount);

        const std::size_t start = result.size();
        result.resize(start + byteCount, 0);
        for (std::size_t i = 0; i < values_.size(); ++i)
        {
            if (!values_[i].isCoil())
            {
                throw std::invalid_argument("Write multiple coils requires coil values.");
            }
            if (values_[i].coil())
            {
                result[start + i / 8] |= static_cast<uint8_t>(1u << (i % 8));
            }
        }
        break;
    }

    case MBfunctionCode::MBWriteMultipleRegisters:
    {
        if (values_.size() != registersNumber_)
        {
            throw std::invalid_argument("Register count does not match the number of values.");
        }

        appendUint16(result, address_);
        appendUint16(result, registersNumber_);
        result.push_back(static_cast<uint8_t>(registersNumber_ * 2u));

        for (const ModbusCell& value : values_)
        {
            if (!value.isReg())
            {
                throw std::invalid_argument("Write multiple registers requires register values.");
            }
            appendUint16(result, value.reg());
        }
        break;
    }

    case MBfunctionCode::MBMaskWriteRegister:
        if (values_.size() != 2 || !values_[0].isReg() || !values_[1].isReg())
        {
            throw std::invalid_argument("Mask write register requires AND and OR values.");
        }
        appendUint16(result, address_);
        appendUint16(result, values_[0].reg());
        appendUint16(result, values_[1].reg());
        break;

    case MBfunctionCode::MBReadWriteMultipleRegisters:
    {
        if (registersNumber_ == 0 || values_.empty())
        {
            throw std::invalid_argument(
                "Read/write multiple registers requires read and write data.");
        }
        if (values_.size() > 127)
        {
            throw std::invalid_argument(
                "Read/write multiple registers exceeds the byte count field.");
        }

        appendUint16(result, address_);
        appendUint16(result, registersNumber_);
        appendUint16(result, writeAddress_);
        appendUint16(result, static_cast<uint16_t>(values_.size()));
        result.push_back(static_cast<uint8_t>(values_.size() * 2u));

        for (const ModbusCell& value : values_)
        {
            if (!value.isReg())
            {
                throw std::invalid_argument(
                    "Read/write multiple registers requires register values.");
            }
            appendUint16(result, value.reg());
        }
        break;
    }
    default:
        throw std::invalid_argument("Unsupported Modbus request function.");
    }

    return result;
}

/// @brief 返回当前功能码对应的操作类型。
MBFunctionType ModbusRequest::functionType() const { return ::functionType(functionCode_); }

/// @brief 返回当前功能码对应的数据区类型。
MBFunctionRegisters ModbusRequest::functionRegisters() const
{
    return functionRegister(functionCode_);
}
