#include "issac_modbus/util/modbusException.h"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "issac_modbus/util/crc.h"


namespace issac_modbus
{
/// @brief 从 TCP 或 RTU 异常响应字节中解析异常信息。
ModbusException::ModbusException(const std::vector<uint8_t>& inputData, bool checkCRC) noexcept
{
    constexpr std::size_t packetSizeWithoutCRC = 3;
    constexpr std::size_t packetSizeWithCRC = 5;
    const std::size_t expectedSize = checkCRC ? packetSizeWithCRC : packetSizeWithoutCRC;

    // 1. 长度校验：最基础的拦截
    if (inputData.size() != expectedSize)
    {
        unitId_ = 0xFF;
        validUnit_ = false;
        functionCode_ = std::nullopt;
        errorCode_ = MBErrorCode::MBInvalidByteOrder; // 或定义一个 MBInvalidLength 错误
        return;
    }

    // 2. 提前进行 CRC 校验：拒绝解析任何脏数据
    if (checkCRC)
    {
        const uint16_t receivedCRC =
            static_cast<uint16_t>(inputData[packetSizeWithoutCRC]) |
            static_cast<uint16_t>(inputData[packetSizeWithoutCRC + 1] << 8u);
        const uint16_t calculatedCRC = calculateCRC(inputData.data(), packetSizeWithoutCRC);

        if (receivedCRC != calculatedCRC)
        {
            // CRC校验失败说明整帧数据都是不可信的垃圾数据，不能提取任何字段
            unitId_ = 0xFF;
            validUnit_ = false;
            functionCode_ = std::nullopt;
            errorCode_ = MBErrorCode::MBErrorCodeCRCError;
            return;
        }
    }

    // 3. 此时数据已被证明物理层完好，可以安全地提取业务字段
    unitId_ = inputData[0];
    validUnit_ = true;

    const uint8_t functionCodeByte = inputData[1];

    // 4. 验证这是否真的是一个“异常响应”报文
    if ((functionCodeByte & 0x80u) == 0)
    {
        // 走到这里说明 CRC 正常，但最高位为 0。
        // 这意味着你把一个“正常的响应报文”错误地喂给了 ModbusException 类。
        functionCode_ = static_cast<MBfunctionCode>(functionCodeByte);
        errorCode_ = MBErrorCode::MBProtocolError; // 此处表示：类型不匹配错误
        return;
    }

    // 5. 最终解析合法的异常报文
    functionCode_ = static_cast<MBfunctionCode>(functionCodeByte & 0x7Fu);
    errorCode_ = static_cast<MBErrorCode>(inputData[2]);
}

/// @brief 根据错误码、从站地址和功能码创建异常对象。
ModbusException::ModbusException(MBErrorCode errorCode, uint8_t unitId,
                                 std::optional<MBfunctionCode> functionCode) noexcept
    : unitId_(unitId), validUnit_(true), errorCode_(errorCode), functionCode_(functionCode)
{
}

/// @brief 判断字节报文是否带有异常功能码标志位。
bool ModbusException::exist(const std::vector<uint8_t>& inputData) noexcept
{
    constexpr std::size_t minimumExceptionSize = 3;

    if (inputData.size() < minimumExceptionSize)
    {
        return false;
    }

    return (inputData[1] & 0x80u) != 0;
}

/// @brief 生成包含从站、错误码和功能码的异常描述。
std::string ModbusException::toString() const
{
    std::string result = "Error on unit ";

    if (validUnit_)
    {
        result += std::to_string(unitId_);
    }
    else
    {
        result += "Unknown";
    }

    result += " - ";
    result += getMBErrorString(errorCode_);

    if (functionCode_.has_value())
    {
        result += " ( on function: ";
        result += functionCodeToString(*functionCode_);
        result += " )";
    }

    return result;
}

/// @brief 返回线程安全缓存后的 std::exception 兼容文本。
const char* ModbusException::what() const noexcept
{
    static thread_local std::string message;

    try
    {
        message = toString();
        return message.c_str();
    }
    catch (...)
    {
        return "ModbusException";
    }
}

/// @brief 序列化不含 CRC 的异常响应字节。
std::vector<uint8_t> ModbusException::toRaw() const
{
    if (!functionCode_.has_value())
    {
        throw std::logic_error("Modbus exception function code is not set.");
    }

    return {unitId_, static_cast<uint8_t>(static_cast<uint8_t>(*functionCode_) | 0x80u),
            static_cast<uint8_t>(errorCode_)};
}


} // namespace issac_modbus
