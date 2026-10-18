#ifndef MODBUS_RESPONSE_H
#define MODBUS_RESPONSE_H

#include <cstdint>
#include <string>
#include <vector>

#include "issac_modbus/unit/modbusCell.h"
#include "issac_modbus/unit/modbusRequest.h"
#include "issac_modbus/unit/modbusUtils.h"

namespace issac_modbus
{
class ModbusResponse
{
  public:
    /// @brief 从不含 CRC 的原始报文构造响应。
    /// @param inputData 原始报文字节。
    /// @return 解析后的响应对象。
    [[nodiscard]] static ModbusResponse fromRaw(const std::vector<uint8_t>& inputData)
    {
        return ModbusResponse(inputData);
    }

    /// @brief 从包含 CRC 的原始报文构造响应。
    /// @param inputData 原始报文字节，最后两个字节为 CRC16。
    /// @return 解析后的响应对象。
    [[nodiscard]] static ModbusResponse fromRawCRC(const std::vector<uint8_t>& inputData)
    {
        return ModbusResponse(inputData, true);
    }

    /// @brief 根据请求创建一个待填充的响应对象。
    /// @param request 对应的 Modbus 请求。
    /// @return 响应对象。
    [[nodiscard]] static ModbusResponse from(const ModbusRequest& request);

    /// @brief 创建一个空响应。
    ModbusResponse() = default;

    /// @brief 使用编译器生成的拷贝构造。
    ModbusResponse(const ModbusResponse&) = default;

    /// @brief 使用编译器生成的拷贝赋值。
    ModbusResponse& operator=(const ModbusResponse&) = default;

    /// @brief 使用编译器生成的移动构造。
    ModbusResponse(ModbusResponse&&) noexcept = default;

    /// @brief 使用编译器生成的移动赋值。
    ModbusResponse& operator=(ModbusResponse&&) noexcept = default;

    /// @brief 从原始 Modbus 响应报文构造对象。
    /// @param inputData 原始报文字节。
    /// @param verifyCRC 是否校验报文末尾的 CRC16。
    /// @throws std::invalid_argument 报文格式或 CRC 无效时抛出。
    explicit ModbusResponse(const std::vector<uint8_t>& inputData, bool verifyCRC = false);

    /// @brief 手动创建一个 Modbus 响应。
    /// @param unitId Modbus 单元地址。
    /// @param functionCode Modbus 功能码。
    /// @param address 寄存器或线圈地址。
    /// @param registersNumber 寄存器或线圈数量。
    /// @param values 响应数据。
    explicit ModbusResponse(uint8_t unitId, MBfunctionCode functionCode, uint16_t address = 0,
                            uint16_t registersNumber = 0, std::vector<ModbusCell> values = {});

    /// @brief 返回响应的可读文本表示。
    /// @return 响应描述文本。
    [[nodiscard]] std::string toString() const;

    /// @brief 返回不含 CRC16 的 Modbus 响应报文。
    /// @return 原始报文字节。
    [[nodiscard]] std::vector<uint8_t> toRaw() const;

    /// @brief 获取当前功能码对应的操作类型。
    [[nodiscard]] MBFunctionType functionType() const;

    /// @brief 获取当前功能码对应的寄存器类型。
    [[nodiscard]] MBFunctionRegisters functionRegisters() const;

    /// @brief 获取 Modbus 单元地址。
    [[nodiscard]] uint8_t unitId() const { return unitId_; }

    /// @brief 获取 Modbus 单元地址（兼容旧版从站命名）。
    [[nodiscard]] uint8_t slaveID() const { return unitId(); }

    /// @brief 获取功能码。
    [[nodiscard]] MBfunctionCode functionCode() const { return functionCode_; }

    /// @brief 获取寄存器或线圈地址。
    [[nodiscard]] uint16_t registerAddress() const { return address_; }

    /// @brief 获取寄存器或线圈数量。
    [[nodiscard]] uint16_t numberOfRegisters() const { return registersNumber_; }

    /// @brief 获取响应数据。
    [[nodiscard]] const std::vector<ModbusCell>& registerValues() const { return values_; }

    /// @brief 获取响应中随功能码变化的字节数量。
    [[nodiscard]] uint16_t numberOfBytesToFollow() const;

    /// @brief 设置 Modbus 单元地址。
    void setUnitId(uint8_t unitId) { unitId_ = unitId; }

    /// @brief 设置 Modbus 单元地址（兼容旧版从站命名）。
    void setSlaveId(uint8_t slaveId) { setUnitId(slaveId); }

    /// @brief 设置功能码。
    void setFunctionCode(MBfunctionCode functionCode) { functionCode_ = functionCode; }

    /// @brief 设置寄存器或线圈地址。
    void setAddress(uint16_t address) { address_ = address; }

    /// @brief 设置寄存器或线圈数量并调整响应数据大小。
    void setRegistersNumber(uint16_t registersNumber)
    {
        registersNumber_ = registersNumber;
        if (functionCode_ != MBfunctionCode::MBReadWriteMultipleRegisters)
        {
            values_.resize(registersNumber);
        }
    }

    /// @brief 设置响应数据。
    void setValues(const std::vector<ModbusCell>& values) { values_ = values; }

  private:
    uint8_t unitId_ = 0;
    MBfunctionCode functionCode_ = MBfunctionCode::MBReadCoils;
    uint16_t address_ = 0;
    uint16_t registersNumber_ = 0;
    std::vector<ModbusCell> values_;
};

} // namespace issac_modbus

#endif // MODBUS_RESPONSE_H
