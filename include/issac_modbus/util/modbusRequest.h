#ifndef MODBUS_REQUEST_H
#define MODBUS_REQUEST_H

#include <cstdint>
#include <string>
#include <vector>

#include "issac_modbus/util/modbusCell.h"
#include "issac_modbus/util/modbusUtils.h"

namespace issac_modbus
{
class ModbusRequest
{
  public:
    /// @brief 从不含 CRC 的原始报文构造请求。
    /// @param inputData 原始报文字节。
    /// @return 解析后的请求对象。
    [[nodiscard]] static ModbusRequest fromRaw(const std::vector<uint8_t>& inputData)
    {
        return ModbusRequest(inputData);
    }

    /// @brief 从包含 CRC 的原始报文构造请求。
    /// @param inputData 原始报文字节，最后两个字节为 CRC16。
    /// @return 解析后的请求对象。
    [[nodiscard]] static ModbusRequest fromRawCRC(const std::vector<uint8_t>& inputData)
    {
        return ModbusRequest(inputData, true);
    }

    /// @brief 创建一个空请求。
    ModbusRequest() = default;

    /// @brief 使用编译器生成的拷贝构造。
    ModbusRequest(const ModbusRequest&) = default;

    /// @brief 使用编译器生成的拷贝赋值。
    ModbusRequest& operator=(const ModbusRequest&) = default;

    /// @brief 使用编译器生成的移动构造。
    ModbusRequest(ModbusRequest&&) noexcept = default;

    /// @brief 使用编译器生成的移动赋值。
    ModbusRequest& operator=(ModbusRequest&&) noexcept = default;

    /// @brief 从原始 Modbus 请求报文构造对象。
    /// @param inputData 原始报文字节。
    /// @param verifyCRC 是否校验报文末尾的 CRC16。
    /// @throws std::invalid_argument 报文格式或 CRC 无效时抛出。
    explicit ModbusRequest(const std::vector<uint8_t>& inputData, bool verifyCRC = false);

    /// @brief 手动创建一个 Modbus 请求。
    /// @param unitId Modbus 单元地址。
    /// @param functionCode Modbus 功能码。
    /// @param address 寄存器或线圈起始地址；0x17 时为读取起始地址。
    /// @param registersNumber 寄存器或线圈数量；0x17 时为读取数量。
    /// @param values 写入值；读请求应保持为空，0x17 时表示写入寄存器值。
    /// @param writeAddress 读写多寄存器请求的写入起始地址。
    explicit ModbusRequest(uint8_t unitId, MBfunctionCode functionCode, uint16_t address = 0,
                           uint16_t registersNumber = 0, std::vector<ModbusCell> values = {},
                           uint16_t writeAddress = 0);

    /// @brief 返回请求的可读文本表示。
    /// @return 请求描述文本。
    [[nodiscard]] std::string toString() const;

    /// @brief 返回不含 CRC16 的 Modbus 请求报文。
    /// @return 原始报文字节。
    [[nodiscard]] std::vector<uint8_t> toRaw() const;

    /// @brief 获取当前功能码对应的操作类型。
    /// @return 读、单写、多写或读写类型。
    [[nodiscard]] MBFunctionType functionType() const;

    /// @brief 获取当前功能码对应的寄存器类型。
    /// @return 线圈、离散输入、保持寄存器或输入寄存器类型。
    [[nodiscard]] MBFunctionRegisters functionRegisters() const;

    /// @brief 获取 Modbus 单元地址。
    [[nodiscard]] uint8_t unitId() const { return unitId_; }

    /// @brief 获取 Modbus 单元地址（兼容旧版从站命名）。
    [[nodiscard]] uint8_t slaveID() const { return unitId(); }

    /// @brief 获取功能码。
    [[nodiscard]] MBfunctionCode functionCode() const { return functionCode_; }

    /// @brief 获取寄存器或线圈起始地址。
    [[nodiscard]] uint16_t registerAddress() const { return address_; }

    /// @brief 获取寄存器或线圈数量。
    [[nodiscard]] uint16_t numberOfRegisters() const { return registersNumber_; }

    /// @brief 获取写入值列表。
    [[nodiscard]] const std::vector<ModbusCell>& registerValues() const { return values_; }

    /// @brief 获取读写多寄存器请求的写入起始地址。
    [[nodiscard]] uint16_t writeAddress() const { return writeAddress_; }

    /// @brief 获取读写多寄存器请求的写入寄存器数量。
    [[nodiscard]] uint16_t numberOfWriteRegisters() const
    {
        return static_cast<uint16_t>(values_.size());
    }

    /// @brief 设置 Modbus 单元地址。
    void setUnitId(uint8_t unitId) { unitId_ = unitId; }

    /// @brief 设置 Modbus 单元地址（兼容旧版从站命名）。
    void setSlaveId(uint8_t slaveId) { setUnitId(slaveId); }

    /// @brief 设置功能码。
    void setFunctionCode(MBfunctionCode functionCode) { functionCode_ = functionCode; }

    /// @brief 设置寄存器或线圈起始地址。
    void setAddress(uint16_t address) { address_ = address; }

    /// @brief 设置寄存器或线圈数量并调整写入值列表大小。
    void setRegistersNumber(uint16_t registersNumber)
    {
        registersNumber_ = registersNumber;
        if (functionCode_ != MBfunctionCode::MBReadWriteMultipleRegisters)
        {
            values_.resize(registersNumber);
        }
    }

    /// @brief 设置读写多寄存器请求的写入起始地址。
    void setWriteAddress(uint16_t writeAddress) { writeAddress_ = writeAddress; }

    /// @brief 设置写入值列表。
    void setValues(const std::vector<ModbusCell>& values) { values_ = values; }

  private:
    uint8_t unitId_ = 0;
    MBfunctionCode functionCode_ = MBfunctionCode::MBReadCoils;
    uint16_t address_ = 0;
    uint16_t registersNumber_ = 0;
    uint16_t writeAddress_ = 0;
    std::vector<ModbusCell> values_;
};

} // namespace issac_modbus

#endif // MODBUS_REQUEST_H
