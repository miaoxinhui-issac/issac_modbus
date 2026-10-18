#ifndef MODBUSCELL_H
#define MODBUSCELL_H

#include <cstdint>
#include <string>
#include <variant>

namespace issac_modbus
{
class ModbusCell
{
  public:
    /// @brief 使用寄存器值创建单元。
    [[nodiscard]] static ModbusCell initReg(uint16_t reg) { return ModbusCell(reg); }

    /// @brief 使用布尔值创建寄存器类型的单元。
    [[nodiscard]] static ModbusCell initReg(bool reg) { return ModbusCell(reg); }

    /// @brief 创建一个默认寄存器单元，初始值为零。
    constexpr ModbusCell() = default;

    /// @brief 使用 16 位寄存器值创建单元。
    constexpr ModbusCell(uint16_t reg) : value_{reg} {}

    /// @brief 使用布尔线圈值创建单元。
    constexpr ModbusCell(bool reg) : value_{reg} {}

    /// @brief 复制构造一个单元。
    ModbusCell(const ModbusCell& other) noexcept = default;

    /// @brief 复制赋值一个单元。
    ModbusCell& operator=(const ModbusCell& other) noexcept = default;

    /// @brief 移动赋值一个单元。
    ModbusCell& operator=(ModbusCell&& other) noexcept = default;

    /// @brief 判断当前单元是否为线圈类型。
    [[nodiscard]] bool isCoil() const { return std::holds_alternative<bool>(value_); }

    /// @brief 判断当前单元是否为寄存器类型。
    [[nodiscard]] bool isReg() const { return std::holds_alternative<uint16_t>(value_); }

    /// @brief 获取可修改的线圈值，必要时把寄存器值转换为布尔值。
    bool& coil()
    {
        if (isReg())
            value_ = static_cast<bool>(std::get<uint16_t>(value_));
        return std::get<bool>(value_);
    };
    /// @brief 获取可修改的寄存器值，必要时把线圈值转换为 0 或 1。
    uint16_t& Reg()
    {
        if (isCoil())
            value_ = static_cast<uint16_t>(std::get<bool>(value_));
        return std::get<uint16_t>(value_);
    }
    /// @brief 获取只读线圈值。
    [[nodiscard]] const bool& coil() const { return std::get<bool>(value_); };

    /// @brief 获取只读寄存器值。
    [[nodiscard]] const uint16_t& reg() const { return std::get<uint16_t>(value_); }

    /// @brief 把当前值转换为便于日志输出的文本。
    [[nodiscard]] std::string toString() const noexcept
    {
        return isCoil() ? ((coil()) ? "true" : "false") : std::to_string(reg());
    }

  private:
    std::variant<uint16_t, bool> value_;
};

} // namespace issac_modbus

#endif
