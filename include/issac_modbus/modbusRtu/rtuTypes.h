#ifndef ISSAC_MODBUS_RTU_TYPES_H
#define ISSAC_MODBUS_RTU_TYPES_H

#include "issac_modbus/util/modbusCell.h"
#include "issac_modbus/util/modbusException.h"
#include "issac_modbus/util/modbusTransaction.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <system_error>
#include <vector>

namespace issac_modbus
{

using RtuBytes = std::vector<std::uint8_t>;
using RtuCompletionHandler = std::function<void(const std::error_code&)>;
using RtuWriteHandler = std::function<void(const std::error_code&, std::size_t)>;
using RtuRawHandler = std::function<void(const std::error_code&, RtuBytes)>;
using RtuValuesHandler = std::function<void(const std::error_code&, std::vector<ModbusCell>,
                                            std::optional<ModbusException>)>;

enum class Parity
{
    None,
    Even,
    Odd
};

enum class StopBits
{
    One,
    Two
};

enum class DataBits
{
    Five = 5,
    Six = 6,
    Seven = 7,
    Eight = 8
};

enum class FlowControl
{
    None,
    Software,
    Hardware
};

enum class SerialMode
{
    RS232,
    RS485
};

enum class RtuFrameType
{
    Request,
    Response
};

enum class SerialState
{
    Closed,
    Open,
    Closing
};

enum class ServiceState
{
    Idle,
    Running,
    Stopping
};

struct Rs485Options
{
    /// Enables Linux kernel RS485 direction control. USB adapters with automatic
    /// direction control normally leave this false.
    bool enabled = false;
    bool rtsOnSend = true;
    bool rtsAfterSend = false;
    std::uint32_t delayBeforeSendMs = 0;
    std::uint32_t delayAfterSendMs = 0;
    bool receiveDuringSend = false;
};

inline constexpr std::size_t MaxRtuAduSize = 256;
inline constexpr std::chrono::milliseconds DefaultResponseTimeout{100};
inline constexpr std::chrono::milliseconds DefaultFrameTimeout{1000};
inline constexpr std::chrono::milliseconds DefaultWriteTimeout{1000};
inline constexpr std::chrono::milliseconds DefaultRecoveryDelay{100};

struct SerialOptions
{
    unsigned int baudRate = 9600;
    Parity parity = Parity::None;
    StopBits stopBits = StopBits::One;
    DataBits dataBits = DataBits::Eight;
    FlowControl flowControl = FlowControl::None;
    SerialMode serialMode = SerialMode::RS232;
    Rs485Options rs485{};
    std::chrono::milliseconds frameTimeout = DefaultFrameTimeout;
    std::chrono::milliseconds writeTimeout = DefaultWriteTimeout;
    std::chrono::microseconds byteTimeout{0};
    std::chrono::microseconds frameGap{0};
    std::chrono::milliseconds recoveryDelay = DefaultRecoveryDelay;
    std::size_t maxPendingOperations = 128;
};

struct MasterOptions
{
    std::chrono::milliseconds responseTimeout = DefaultResponseTimeout;
    std::chrono::milliseconds transactionTimeout{5000};
    std::chrono::milliseconds broadcastDelay = DefaultResponseTimeout;
    bool discardInputBeforeRequest = true;
    RetryPolicy retryPolicy{};
};

struct SlaveOptions
{
    std::uint8_t unitId = 1;
};

} // namespace issac_modbus

#endif // ISSAC_MODBUS_RTU_TYPES_H
