#ifndef MODBUS_ERROR_H
#define MODBUS_ERROR_H

#include <system_error>
#include <type_traits>

#include "issac_modbus/util/modbusUtils.h"

namespace issac_modbus
{

/// @brief 用于传输层和协议层错误的 Modbus 专用错误类别。
const std::error_category& modbusErrorCategory() noexcept;

/// @brief 根据 Modbus 错误码创建标准错误码对象。
inline std::error_code makeErrorCode(MBErrorCode code) noexcept
{
    return {static_cast<int>(code), modbusErrorCategory()};
}

/// @brief 提供给 std::error_code 基础设施使用的 Modbus 错误码转换函数。
inline std::error_code make_error_code(MBErrorCode code) noexcept
{
    return makeErrorCode(code);
}

} // namespace issac_modbus

namespace std
{
template <> struct is_error_code_enum<issac_modbus::MBErrorCode> : std::true_type
{
};
} // namespace std

#endif // MODBUS_ERROR_H
