#ifndef MODBUS_TRANSACTION_H
#define MODBUS_TRANSACTION_H

#include <chrono>
#include <cstddef>
#include <functional>
#include <optional>
#include <system_error>

#include "issac_modbus/util/modbusException.h"
#include "issac_modbus/util/modbusResponse.h"

namespace issac_modbus
{

struct RetryPolicy
{
    std::size_t maxAttempts = 1;
    std::chrono::milliseconds retryDelay{0};
    bool retryOnTimeout = true;
    bool retryOnCrcError = true;
    bool retryOnConnectionError = true;
};

using TransactionHandler =
    std::function<void(const std::error_code&, ModbusResponse, std::optional<ModbusException>)>;

} // namespace issac_modbus

#endif // MODBUS_TRANSACTION_H
