#include "issac_modbus/unit/modbusError.h"

#include <string>

namespace issac_modbus
{

namespace
{

class ModbusErrorCategory final : public std::error_category
{
  public:
    /// @brief 返回 Modbus 错误类别的名称。
    const char* name() const noexcept override { return "modbus"; }

    /// @brief 返回指定错误码对应的可读描述文本。
    std::string message(int value) const override
    {
        return std::string(getMBErrorString(static_cast<MBErrorCode>(value)));
    }
};

} // namespace

/// @brief 获取进程内唯一的 Modbus 错误类别实例。
const std::error_category& modbusErrorCategory() noexcept
{
    static const ModbusErrorCategory category;
    return category;
}

} // namespace issac_modbus
