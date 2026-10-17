#ifndef MODBUS_EXCEPTION_H
#define MODBUS_EXCEPTION_H

#include "issac/modbus/protocol/modbusUtils.h"

#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <vector>

class ModbusException : public std::exception
{
  public:
    /// @brief 禁止默认构造异常。
    ModbusException() = delete;

    /// @brief 从原始 Modbus 异常响应报文构造对象。
    /// @param inputData 原始报文字节。
    /// @param checkCRC 是否校验报文末尾的 CRC16。
    explicit ModbusException(const std::vector<uint8_t>& inputData, bool checkCRC = false) noexcept;

    /// @brief 根据错误码、从站地址和功能码构造异常。
    /// @param errorCode Modbus 错误码。
    /// @param slaveId 从站地址，默认 0xFF 表示未知。
    /// @param functionCode 发生异常的功能码，默认未指定。
    explicit ModbusException(MBErrorCode errorCode, uint8_t slaveId = 0xFF,
                             std::optional<MBfunctionCode> functionCode = std::nullopt) noexcept;

    /// @brief 检查原始报文是否为 Modbus 异常响应。
    /// @param inputData 原始报文字节。
    /// @return 功能码最高位为 1 时返回 true。
    [[nodiscard]] static bool exist(const std::vector<uint8_t>& inputData) noexcept;

    /// @brief 获取从站地址。
    [[nodiscard]] uint8_t slaveID() const noexcept { return slaveId_; }

    /// @brief 判断从站地址是否有效。
    [[nodiscard]] bool isSlaveValid() const noexcept { return validSlave_; }

    /// @brief 设置从站地址并标记为有效。
    void setSlaveID(uint8_t slaveId) noexcept
    {
        slaveId_ = slaveId;
        validSlave_ = true;
    }

    /// @brief 获取 Modbus 错误码。
    [[nodiscard]] MBErrorCode getErrorCode() const noexcept { return errorCode_; }

    /// @brief 获取发生异常的功能码。
    [[nodiscard]] std::optional<MBfunctionCode> functionCode() const noexcept
    {
        return functionCode_;
    }

    /// @brief 设置发生异常的功能码。
    void setFunctionCode(std::optional<MBfunctionCode> functionCode) noexcept
    {
        functionCode_ = functionCode;
    }

    /// @brief 获取异常文本。
    /// @return 包含从站、错误码和功能码的描述。
    [[nodiscard]] std::string toString() const;

    /// @brief 获取兼容 std::exception 的错误文本。
    [[nodiscard]] const char* what() const noexcept override;

    /// @brief 返回不含 CRC16 的 Modbus 异常响应报文。
    /// @return 异常响应报文字节。
    /// @throws std::logic_error 功能码未指定时抛出。
    [[nodiscard]] std::vector<uint8_t> toRaw() const;

  private:
    uint8_t slaveId_ = 0xFF;
    bool validSlave_ = false;
    MBErrorCode errorCode_ = MBErrorCode::MBInvalidByteOrder;
    std::optional<MBfunctionCode> functionCode_;
};

#endif // MODBUS_EXCEPTION_H
