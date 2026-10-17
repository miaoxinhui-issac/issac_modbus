#ifndef MODBUS_TRANSACTION_H
#define MODBUS_TRANSACTION_H

#include "issac/modbus/protocol/modbusException.h"
#include "issac/modbus/protocol/modbusResponse.h"

#include <chrono>
#include <cstddef>
#include <functional>
#include <optional>
#include <system_error>

namespace MB
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

} // namespace MB

#endif // MODBUS_TRANSACTION_H
