#ifndef MODBUS_UTILS_H
#define MODBUS_UTILS_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "issac_modbus/unit/crc.h"
#include "issac_modbus/unit/modbusCell.h"

namespace issac_modbus
{
enum class MBErrorCode : uint8_t
{
    MBIllegalFunction = 0x01,        // 非法功能码
    MBIllegalDataAddress = 0x02,     // 非法数据地址
    MBIllegalDataValue = 0x03,       // 非法数据值
    MBSlaveDeviceFailure = 0x04,     // 从站设备故障
    MBAcknowledge = 0x05,            // 请求已确认
    MBSlaveDeviceBusy = 0x06,        // 从站设备忙
    MBNegativeAcknowledge = 0x07,    // 否定确认
    MBMemoryParityError = 0x08,      // 存储器奇偶校验错误
    MBGatewayPathUnavailable = 0x10, // 网关路径不可用
    MBGatewayTargetFailed = 0x11,    // 网关目标设备失败

    /// Custom modbus errors for Modbus for C++ library
    MBErrorCodeCRCError = 0b0111111,         // CRC 校验错误
    MBInvalidCRC = 0b01111110,               // 无效 CRC
    MBInvalidByteOrder = 0b01111101,         // 无效字节序
    MBInvalidMessageID = 0b01111100,         // 无效消息 ID
    MBProtocolError = 0b01111011,            // 协议错误
    MBConnectionClosed = 0b01111010,         // 连接已关闭
    MBTimeout = 0b01111001,                  // 超时
    MBNumberOfRegistersInvalid = 0b01111000, // 寄存器数量无效
    MBNumberOfValuesInvalid = 0b01110111,    // 值数量无效
    MBInvalidSlaveId = 0b01110110,
    MBFunctionMismatch = 0b01110101,
    MBTransactionMismatch = 0b01110100,
    MBBroadcastNoResponse = 0b01110011,
    MBFrameTimeout = 0b01110010,
    MBBadFrame = 0b01110001,
    MBSerialConfiguration = 0b01110000,
    MBRetryExhausted = 0b01101111,
    MBConcurrentOperation = 0b01101110,
};

/// @brief 判断错误码是否属于 Modbus 标准异常码。
/// @param errorCode 待判断的错误码。
/// @return 标准异常码返回 true，否则返回 false。
inline bool isStandardErrorCode(MBErrorCode errorCode)
{
    switch (errorCode)
    {
    case MBErrorCode::MBIllegalFunction:
    case MBErrorCode::MBIllegalDataAddress:
    case MBErrorCode::MBIllegalDataValue:
    case MBErrorCode::MBSlaveDeviceFailure:
    case MBErrorCode::MBAcknowledge:
    case MBErrorCode::MBSlaveDeviceBusy:
    case MBErrorCode::MBNegativeAcknowledge:
    case MBErrorCode::MBMemoryParityError:
    case MBErrorCode::MBGatewayPathUnavailable:
    case MBErrorCode::MBGatewayTargetFailed:
        return true;
    default:
        return false;
    }
}
/// @brief 获取 Modbus 错误码对应的文本描述。
/// @param error_code 待转换的错误码。
/// @return 错误码对应的描述文本。
inline std::string_view getMBErrorString(MBErrorCode error_code)
{
    switch (error_code)
    {
    case MBErrorCode::MBIllegalFunction:
        return "Illegal Function";
    case MBErrorCode::MBIllegalDataAddress:
        return "Illegal Data Address";
    case MBErrorCode::MBIllegalDataValue:
        return "Illegal Data Value";
    case MBErrorCode::MBSlaveDeviceFailure:
        return "Slave Device Failure";
    case MBErrorCode::MBAcknowledge:
        return "Acknowledge";
    case MBErrorCode::MBSlaveDeviceBusy:
        return "Slave Device Busy";
    case MBErrorCode::MBNegativeAcknowledge:
        return "Negative Acknowledge";
    case MBErrorCode::MBMemoryParityError:
        return "Memory Parity Error";
    case MBErrorCode::MBGatewayPathUnavailable:
        return "Gateway Path Unavailable";
    case MBErrorCode::MBGatewayTargetFailed:
        return "Gateway Target Failed";
    case MBErrorCode::MBErrorCodeCRCError:
        return "CRC Error";
    case MBErrorCode::MBInvalidCRC:
        return "Invalid CRC";
    case MBErrorCode::MBInvalidByteOrder:
        return "Invalid Byte Order";
    case MBErrorCode::MBInvalidMessageID:
        return "Invalid Message ID";
    case MBErrorCode::MBProtocolError:
        return "Protocol Error";
    case MBErrorCode::MBConnectionClosed:
        return "Connection is closed";
    case MBErrorCode::MBTimeout:
        return "Time out";
    case MBErrorCode::MBNumberOfRegistersInvalid:
        return "Number of registers in response is too big - cannot be "
               "serialized";
    case MBErrorCode::MBNumberOfValuesInvalid:
        return "Number of values is not valid";
    case MBErrorCode::MBInvalidSlaveId:
        return "Invalid unit ID";
    case MBErrorCode::MBFunctionMismatch:
        return "Response function does not match request";
    case MBErrorCode::MBTransactionMismatch:
        return "Response transaction does not match request";
    case MBErrorCode::MBBroadcastNoResponse:
        return "Broadcast has no response";
    case MBErrorCode::MBFrameTimeout:
        return "Incomplete Modbus frame timed out";
    case MBErrorCode::MBBadFrame:
        return "Invalid Modbus frame";
    case MBErrorCode::MBSerialConfiguration:
        return "Serial configuration failed";
    case MBErrorCode::MBRetryExhausted:
        return "Modbus retry limit exhausted";
    case MBErrorCode::MBConcurrentOperation:
        return "Another Modbus operation is already in progress";
    }
    return "Unknown";
}
enum class MBfunctionCode : uint8_t

{
    MBReadCoils = 0x01,                  // 读可写线圈
    MBReadDiscreteInputs = 0x02,         // 读离散输入
    MBReadHoldingRegisters = 0x03,       // 读保持寄存器
    MBReadInputRegisters = 0x04,         // 读输入寄存器
    MBWriteSingleCoil = 0x05,            // 写单个线圈
    MBWriteSingleRegister = 0x06,        // 写单个寄存器
    MBWriteMultipleCoils = 0x0F,         // 写多个线圈
    MBWriteMultipleRegisters = 0x10,     // 写多个寄存器
    MBReportSlaveID = 0x11,              // 报告从站 ID
    MBMaskWriteRegister = 0x16,          // 掩码按位写保持寄存器
    MBReadWriteMultipleRegisters = 0x17, // 读写多个寄存器
};

enum MBFunctionType
{
    Read,
    WriteSingle,
    WriteMultiple,
    ReadWrite
};

/// @brief 根据功能码判断操作类型。
/// @param code Modbus 功能码。
/// @return 功能码对应的读、写或读写类型。
/// @throws std::runtime_error 功能码未定义时抛出。
inline MBFunctionType functionType(const MBfunctionCode code)
{
    switch (code)
    {
    case MBfunctionCode::MBReadCoils:
    case MBfunctionCode::MBReadDiscreteInputs:
    case MBfunctionCode::MBReadHoldingRegisters:
    case MBfunctionCode::MBReadInputRegisters:
    case MBfunctionCode::MBReportSlaveID:
        return Read;
    case MBfunctionCode::MBWriteSingleCoil:
    case MBfunctionCode::MBWriteSingleRegister:
    case MBfunctionCode::MBMaskWriteRegister:
        return WriteSingle;
    case MBfunctionCode::MBWriteMultipleCoils:
    case MBfunctionCode::MBWriteMultipleRegisters:
        return WriteMultiple;
    case MBfunctionCode::MBReadWriteMultipleRegisters:
        return ReadWrite;
    }
    throw std::runtime_error("The function code is undefined");
}

//! Simplified register types
enum MBFunctionRegisters
{
    OutputCoils,
    InputCoils,
    HoldingRegisters,
    InputRegisters,
    NoRegisters
};

/// @brief 获取功能码操作对应的寄存器类型。
/// @param code Modbus 功能码。
/// @return 功能码对应的寄存器类型。
/// @throws std::runtime_error 功能码未定义时抛出。
inline MBFunctionRegisters functionRegister(const MBfunctionCode code)
{
    switch (code)
    {
    case MBfunctionCode::MBReadCoils:
    case MBfunctionCode::MBWriteSingleCoil:
    case MBfunctionCode::MBWriteMultipleCoils:
        return OutputCoils;
    case MBfunctionCode::MBReadDiscreteInputs:
        return InputCoils;
    case MBfunctionCode::MBReadHoldingRegisters:
    case MBfunctionCode::MBWriteSingleRegister:
    case MBfunctionCode::MBWriteMultipleRegisters:
    case MBfunctionCode::MBMaskWriteRegister:
    case MBfunctionCode::MBReadWriteMultipleRegisters:
        return HoldingRegisters;
    case MBfunctionCode::MBReadInputRegisters:
        return InputRegisters;
    case MBfunctionCode::MBReportSlaveID:
        return NoRegisters;
    }
    throw std::runtime_error("The function code is undefined");
}

/// @brief 向字节容器追加一个 Modbus 大端 16 位值。
inline void appendUint16(std::vector<uint8_t>& buffer, uint16_t value)
{
    buffer.push_back(static_cast<uint8_t>(value >> 8u));
    buffer.push_back(static_cast<uint8_t>(value & 0xFFu));
}

/// @brief 从字节容器读取一个 Modbus 大端 16 位值。
/// @param buffer 输入报文字节。
/// @param offset 读取起始位置。
/// @param limit 可读取的最大位置，不包括 CRC。
/// @param incompleteMessageError 数据不完整时使用的错误信息。
/// @return 读取到的 16 位值。
/// @throws std::invalid_argument 数据越界时抛出。
inline uint16_t readUint16(const std::vector<uint8_t>& buffer, std::size_t offset,
                           std::size_t limit, std::string_view incompleteMessageError)
{
    if (offset > limit || limit - offset < 2)
    {
        throw std::invalid_argument(std::string(incompleteMessageError));
    }

    return static_cast<uint16_t>(buffer[offset + 1]) |
           (static_cast<uint16_t>(buffer[offset]) << 8u);
}

/// @brief 将功能码转换为可读文本。
inline std::string_view functionCodeToString(MBfunctionCode functionCode)
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
inline void normalizeValues(MBfunctionCode functionCode, std::vector<ModbusCell>& values)
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

enum class MBMaxDataSize : uint16_t
{
    MBMaxReadCoils = 2000,                 // 最大读线圈数量
    MBMaxReadDiscreteInputs = 2000,        // 最大读离散输入数量
    MBMaxReadHoldingRegisters = 125,       // 最大读保持寄存器数量
    MBMaxReadInputRegisters = 125,         // 最大读输入寄存器数量
    MBMaxWriteSingleCoil = 1,              // 最大写单个线圈数量
    MBMaxWriteSingleRegister = 1,          // 最大写单个寄存器数量
    MBMaxWriteMultipleCoils = 1968,        // 最大写多个线圈数量
    MBMaxWriteMultipleRegisters = 123,     // 最大写多个寄存器数量
    MBMaxReportSlaveID = 0,                // 报告从站 ID 最大数据量
    MBMaxMaskWriteRegister = 1,            // 最大屏蔽写寄存器数量
    MBMaxReadWriteMultipleRegisters = 121, // 最大读写多个寄存器数量
};

/// @brief 将两字节大端数据转换为 16 位无符号数。
/// @param buf 输入缓冲区，至少包含两个字节。
/// @param length 输入缓冲区长度。
/// @return 转换后的 16 位无符号数。
/// @throws std::invalid_argument 缓冲区为空或长度不足时抛出。
inline uint16_t bigEndianConv(const uint8_t* buf, size_t length)
{
    if (length < 2 || buf == nullptr)
    {
        throw std::invalid_argument("Buffer length must be at least 2 bytes.");
    }

    return static_cast<uint16_t>(buf[1]) | (static_cast<uint16_t>(buf[0]) << 8u);
}
using Registers = std::array<uint16_t, 2>;

/// @brief Modbus 双寄存器浮点数的字节和字排列顺序。
///
/// 顺序 | 设备或场景例子 | 说明
/// ABCD | 西门子 SIWAREX WP521/WP522 称重模块，在相关交换选项关闭时 |
///      默认高字节、高字在前；注意这是称重模块，不能据此推断所有西门子 PLC
/// CDAB | 施耐德 Modicon M340、M580 的常规双字映射 |
///      官方要求配套 HMI 选择 Low word first，低字在前
enum class FloatOrder
{
    /// ABCD：高字节、高字在前。
    ABCD,
    /// DCBA：字节和字均逆序。
    DCBA,
    /// BADC：字节交换，字顺序保持不变。
    BADC,
    /// CDAB：低字在前。
    CDAB
};

/// @brief 将打包字节展开为独立的 0/1，写入 dest 的指定位置。
/// @param dest 输出容器，函数从 dest[idx] 开始写入。
/// @param idx 写入起始位置。
/// @param nb_bits 需要展开的位数。
/// @param bytes 打包后的位数据。
/// @throws std::out_of_range 缓冲区空间不足时抛出。
/// @note Modbus 位数据按低位在前展开。
inline void modbus_set_bits_from_bytes(std::vector<uint8_t>& dest, size_t idx, size_t nb_bits,
                                       const std::vector<uint8_t>& bytes)
{
    const size_t required = nb_bits / 8 + (nb_bits % 8 != 0);

    if (idx > dest.size() || nb_bits > dest.size() - idx || required > bytes.size())
    {
        throw std::out_of_range("Insufficient buffer size");
    }

    for (size_t i = 0; i < nb_bits; ++i)
    {
        dest[idx + i] = (bytes[i / 8] >> (i % 8)) & 1u;
    }
}

/// @brief 将独立的 0/1 打包成一个字节。
/// @param src 输入位数据。
/// @param idx 读取起始位置。
/// @param nb_bits 需要打包的位数，最多使用 8 位。
/// @return 打包后的字节。
/// @throws std::out_of_range 输入数据空间不足时抛出。
inline uint8_t modbus_get_byte_from_bits(const std::vector<uint8_t>& src, size_t idx,
                                         size_t nb_bits)
{
    nb_bits = std::min(nb_bits, size_t{8});

    if (idx > src.size() || nb_bits > src.size() - idx)
    {
        throw std::out_of_range("Insufficient buffer size");
    }

    uint8_t value = 0;

    for (size_t i = 0; i < nb_bits; ++i)
    {
        if (src[idx + i] != 0)
        {
            value |= static_cast<uint8_t>(1u << i);
        }
    }

    return value;
}

/// @brief 按指定顺序重排 32 位数据。
/// @param bits 待重排的 32 位数据。
/// @param order 目标排列顺序。
/// @return 重排后的 32 位数据。
/// @throws std::invalid_argument 排列顺序无效时抛出。
/// @note 四种排列均执行两次后可以恢复原值，因此编码和解码可以共用。
inline uint32_t reorder(uint32_t bits, FloatOrder order)
{
    switch (order)
    {
    case FloatOrder::ABCD:
        return bits;

    case FloatOrder::BADC:
        return ((bits & 0xFF00FF00u) >> 8u) | ((bits & 0x00FF00FFu) << 8u);

    case FloatOrder::CDAB:
        return (bits >> 16u) | (bits << 16u);

    case FloatOrder::DCBA:
        return ((bits & 0xFF000000u) >> 24u) | ((bits & 0x00FF0000u) >> 8u) |
               ((bits & 0x0000FF00u) << 8u) | ((bits & 0x000000FFu) << 24u);
    }

    throw std::invalid_argument("Invalid float order");
}

static_assert(sizeof(float) == sizeof(uint32_t), "Requires 32-bit float");

static_assert(std::numeric_limits<float>::is_iec559, "Requires IEEE 754 float");

/// @brief 将两个 Modbus 寄存器转换为 float。
/// @param src 输入寄存器，src[0] 为高字，src[1] 为低字。
/// @param order 寄存器中的浮点数排列顺序。
/// @return 转换后的 float 值。
/// @note ABCD 适用于西门子 SIWAREX WP521/WP522 等默认排列场景。
/// @note CDAB 适用于施耐德 Modicon M340/M580 的 Low word first 映射。
inline float getFloat(const Registers& src, FloatOrder order)
{
    uint32_t bits = (static_cast<uint32_t>(src[0]) << 16u) | static_cast<uint32_t>(src[1]);

    bits = reorder(bits, order);

    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

/// @brief 将 float 转换为两个 Modbus 寄存器。
/// @param value 待转换的 float 值。
/// @param order 目标寄存器排列顺序。
/// @return 转换后的两个寄存器，返回值的 [0] 为高字，[1] 为低字。
/// @note ABCD 适用于西门子 SIWAREX WP521/WP522 等默认排列场景。
/// @note CDAB 适用于施耐德 Modicon M340/M580 的 Low word first 映射。
inline Registers setFloat(float value, FloatOrder order)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));

    bits = reorder(bits, order);

    return {static_cast<uint16_t>(bits >> 16u), static_cast<uint16_t>(bits)};
}

/// @brief 按 ABCD 顺序从两个寄存器读取 float。
/// @param src 输入寄存器。
/// @return 转换后的 float 值。
inline float modbus_get_float_abcd(const Registers& src) { return getFloat(src, FloatOrder::ABCD); }

/// @brief 按 DCBA 顺序从两个寄存器读取 float。
/// @param src 输入寄存器。
/// @return 转换后的 float 值。
inline float modbus_get_float_dcba(const Registers& src) { return getFloat(src, FloatOrder::DCBA); }

/// @brief 按 BADC 顺序从两个寄存器读取 float。
/// @param src 输入寄存器。
/// @return 转换后的 float 值。
inline float modbus_get_float_badc(const Registers& src) { return getFloat(src, FloatOrder::BADC); }

/// @brief 按 CDAB 顺序从两个寄存器读取 float。
/// @param src 输入寄存器。
/// @return 转换后的 float 值。
inline float modbus_get_float_cdab(const Registers& src) { return getFloat(src, FloatOrder::CDAB); }

/// @brief 按 ABCD 顺序将 float 写入两个寄存器。
/// @param value 待写入的 float 值。
/// @param dest 输出寄存器。
inline void modbus_set_float_abcd(float value, Registers& dest)
{
    dest = setFloat(value, FloatOrder::ABCD);
}

/// @brief 按 DCBA 顺序将 float 写入两个寄存器。
/// @param value 待写入的 float 值。
/// @param dest 输出寄存器。
inline void modbus_set_float_dcba(float value, Registers& dest)
{
    dest = setFloat(value, FloatOrder::DCBA);
}

/// @brief 按 BADC 顺序将 float 写入两个寄存器。
/// @param value 待写入的 float 值。
/// @param dest 输出寄存器。
inline void modbus_set_float_badc(float value, Registers& dest)
{
    dest = setFloat(value, FloatOrder::BADC);
}

/// @brief 按 CDAB 顺序将 float 写入两个寄存器。
/// @param value 待写入的 float 值。
/// @param dest 输出寄存器。
inline void modbus_set_float_cdab(float value, Registers& dest)
{
    dest = setFloat(value, FloatOrder::CDAB);
}

} // namespace issac_modbus

#endif // MODBUS_UTILS_H
