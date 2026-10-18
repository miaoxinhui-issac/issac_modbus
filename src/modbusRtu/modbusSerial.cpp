#include "issac_modbus/modbusRtu/modbusSerial.h"

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <asio/error.hpp>

#include "issac_modbus/unit/crc.h"
#include "issac_modbus/unit/modbusError.h"
#include "issac_modbus/unit/modbusProtocol.h"

namespace
{

/// @brief 向 RTU 帧追加低字节在前的 CRC16。
void appendCRC(std::vector<std::uint8_t>& frame)
{
    const std::uint16_t crc = calculateCRC(frame);
    frame.push_back(static_cast<std::uint8_t>(crc & 0xFFu));
    frame.push_back(static_cast<std::uint8_t>(crc >> 8u));
}

/// @brief 根据响应头部计算完整 RTU 响应帧长度。
std::optional<std::size_t> responseFrameLength(const std::vector<std::uint8_t>& data)
{
    if (data.size() < 2)
    {
        return std::nullopt;
    }

    const auto functionCode = static_cast<MBfunctionCode>(data[1]);
    if ((data[1] & 0x80u) != 0)
    {
        return 5;
    }

    switch (functionCode)
    {
    case MBfunctionCode::MBReadCoils:
    case MBfunctionCode::MBReadDiscreteInputs:
    case MBfunctionCode::MBReadHoldingRegisters:
    case MBfunctionCode::MBReadInputRegisters:
    case MBfunctionCode::MBReportSlaveID:
    case MBfunctionCode::MBReadWriteMultipleRegisters:
        if (data.size() < 3)
        {
            return std::nullopt;
        }
        return 3u + static_cast<std::size_t>(data[2]) + 2u;

    case MBfunctionCode::MBWriteSingleCoil:
    case MBfunctionCode::MBWriteSingleRegister:
    case MBfunctionCode::MBWriteMultipleCoils:
    case MBfunctionCode::MBWriteMultipleRegisters:
        return 8;

    case MBfunctionCode::MBMaskWriteRegister:
        return 10;
    }

    return std::nullopt;
}

/// @brief 根据请求头部计算完整 RTU 请求帧长度。
std::optional<std::size_t> requestFrameLength(const std::vector<std::uint8_t>& data)
{
    if (data.size() < 2)
    {
        return std::nullopt;
    }

    const auto functionCode = static_cast<MBfunctionCode>(data[1]);

    switch (functionCode)
    {
    case MBfunctionCode::MBReadCoils:
    case MBfunctionCode::MBReadDiscreteInputs:
    case MBfunctionCode::MBReadHoldingRegisters:
    case MBfunctionCode::MBReadInputRegisters:
    case MBfunctionCode::MBWriteSingleCoil:
    case MBfunctionCode::MBWriteSingleRegister:
        return 8;

    case MBfunctionCode::MBWriteMultipleCoils:
    case MBfunctionCode::MBWriteMultipleRegisters:
        if (data.size() < 7)
        {
            return std::nullopt;
        }
        return 9u + static_cast<std::size_t>(data[6]);

    case MBfunctionCode::MBReportSlaveID:
        return 4;

    case MBfunctionCode::MBMaskWriteRegister:
        return 10;

    case MBfunctionCode::MBReadWriteMultipleRegisters:
        if (data.size() < 11)
        {
            return std::nullopt;
        }
        return 13u + static_cast<std::size_t>(data[10]);
    }

    return std::nullopt;
}

} // namespace

namespace issac_modbus
{

struct RtuConnection::ReadState
{
    Bytes data;
    std::uint8_t byte = 0;
    FrameLengthResolver resolver;
    RawHandler handler;
    std::shared_ptr<asio::steady_timer> timer;
    std::shared_ptr<asio::steady_timer> byteTimer;
    bool receivedFirstByte = false;
    bool completed = false;
};

/// @brief 创建 RTU 串口连接并保存串口参数。
RtuConnection::RtuConnection(asio::io_context& ioContext, std::string devicePath, unsigned int baudRate)
    : ioContext_(ioContext), serialPort_(ioContext), devicePath_(std::move(devicePath)),
      baudRate_(baudRate)
{
}

/// @brief 销毁连接并关闭串口。
RtuConnection::~RtuConnection() { close(); }

/// @brief 打开串口并应用全部串口选项。
void RtuConnection::open()
{
    if (serialPort_.is_open())
    {
        return;
    }

    std::error_code error;
    serialPort_.open(devicePath_, error);
    if (error)
    {
        throw std::system_error(error);
    }

    applySerialOptions();
}

/// @brief 关闭串口并取消所有排队中的写入。
void RtuConnection::close() noexcept
{
    std::error_code error;
    serialPort_.cancel(error);
    serialPort_.close(error);
    if (writeInProgress_ && !writeQueue_.empty())
    {
        auto active = std::next(writeQueue_.begin());
        while (active != writeQueue_.end())
        {
            auto pending = std::move(*active);
            active = writeQueue_.erase(active);
            pending.handler(asio::error::operation_aborted, 0);
        }
    }
    else
    {
        while (!writeQueue_.empty())
        {
            auto pending = std::move(writeQueue_.front());
            writeQueue_.pop_front();
            pending.handler(asio::error::operation_aborted, 0);
        }
    }
}

/// @brief 判断串口是否已经打开。
bool RtuConnection::isOpen() const noexcept { return serialPort_.is_open(); }

/// @brief 取消异步操作并清空串口输入缓冲。
void RtuConnection::clearInput() noexcept
{
    std::error_code error;
    serialPort_.cancel(error);
    flushInput();
}

/// @brief 使用 Linux 串口接口清空输入队列。
void RtuConnection::flushInput() noexcept
{
    if (serialPort_.is_open())
    {
        ::tcflush(serialPort_.native_handle(), TCIFLUSH);
    }
}

/// @brief 把波特率、数据位、校验位、停止位和流控应用到串口。
void RtuConnection::applySerialOptions()
{
    serialPort_.set_option(asio::serial_port_base::baud_rate(baudRate_));
    serialPort_.set_option(asio::serial_port_base::character_size(8));
    serialPort_.set_option(asio::serial_port_base::flow_control(
        flowControl_ == FlowControl::Hardware   ? asio::serial_port_base::flow_control::hardware
        : flowControl_ == FlowControl::Software ? asio::serial_port_base::flow_control::software
                                                : asio::serial_port_base::flow_control::none));

    serialPort_.set_option(
        asio::serial_port_base::character_size(static_cast<unsigned int>(dataBits_)));

    switch (parity_)
    {
    case Parity::None:
        serialPort_.set_option(
            asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        break;
    case Parity::Even:
        serialPort_.set_option(
            asio::serial_port_base::parity(asio::serial_port_base::parity::even));
        break;
    case Parity::Odd:
        serialPort_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::odd));
        break;
    }

    switch (stopBits_)
    {
    case StopBits::One:
        serialPort_.set_option(
            asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        break;
    case StopBits::Two:
        serialPort_.set_option(
            asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::two));
        break;
    }

    // A normal UART/RS232 port does not implement the Linux RS485 ioctls.
    // Only touch them when RS485 was explicitly requested.
    if (serialMode_ == SerialMode::RS485 || rs485_.enabled)
    {
        serial_rs485 config{};
        const int getRs485Result = ::ioctl(serialPort_.native_handle(), TIOCGRS485, &config);
        const bool rs485Unsupported = getRs485Result < 0 && errno == ENOTTY;
        if (getRs485Result < 0 && !rs485Unsupported)
        {
            throw std::system_error(std::error_code(errno, std::generic_category()));
        }
        if (rs485_.enabled || serialMode_ == SerialMode::RS485)
        {
            config.flags |= SER_RS485_ENABLED;
            if (rs485_.rtsOnSend)
                config.flags |= SER_RS485_RTS_ON_SEND;
            else
                config.flags &= ~SER_RS485_RTS_ON_SEND;
            if (rs485_.rtsAfterSend)
                config.flags |= SER_RS485_RTS_AFTER_SEND;
            else
                config.flags &= ~SER_RS485_RTS_AFTER_SEND;
            if (rs485_.receiveDuringSend)
                config.flags |= SER_RS485_RX_DURING_TX;
            else
                config.flags &= ~SER_RS485_RX_DURING_TX;
            config.delay_rts_before_send = rs485_.delayBeforeSendMs;
            config.delay_rts_after_send = rs485_.delayAfterSendMs;
            if (::ioctl(serialPort_.native_handle(), TIOCSRS485, &config) < 0 && errno != ENOTTY)
            {
                throw std::system_error(std::error_code(errno, std::generic_category()));
            }
        }
        else if (!rs485Unsupported)
        {
            config.flags &= ~SER_RS485_ENABLED;
            (void)::ioctl(serialPort_.native_handle(), TIOCSRS485, &config);
        }
    }
}

/// @brief 设置波特率，串口已打开时立即重新应用配置。
void RtuConnection::setBaudRate(unsigned int baudRate)
{
    baudRate_ = baudRate;
    if (serialPort_.is_open())
    {
        applySerialOptions();
    }
}

/// @brief 设置校验方式，串口已打开时立即重新应用配置。
void RtuConnection::setParity(Parity parity)
{
    parity_ = parity;
    if (serialPort_.is_open())
    {
        applySerialOptions();
    }
}

/// @brief 设置停止位数量，串口已打开时立即重新应用配置。
void RtuConnection::setStopBits(StopBits stopBits)
{
    stopBits_ = stopBits;
    if (serialPort_.is_open())
    {
        applySerialOptions();
    }
}

/// @brief 设置数据位数量，串口已打开时立即重新应用配置。
void RtuConnection::setDataBits(DataBits dataBits)
{
    dataBits_ = dataBits;
    if (serialPort_.is_open())
        applySerialOptions();
}

/// @brief 设置流控模式，串口已打开时立即重新应用配置。
void RtuConnection::setFlowControl(FlowControl flowControl)
{
    flowControl_ = flowControl;
    if (serialPort_.is_open())
        applySerialOptions();
}

/// @brief 设置 RS232 或 RS485 模式，串口已打开时立即重新应用配置。
void RtuConnection::setSerialMode(SerialMode mode)
{
    serialMode_ = mode;
    if (serialPort_.is_open())
        applySerialOptions();
}

/// @brief 设置 RS485 方向控制和发送延时参数。
void RtuConnection::setRs485Options(const Rs485Options& options)
{
    rs485_ = options;
    if (serialPort_.is_open())
        applySerialOptions();
}

/// @brief 序列化请求、追加 CRC 并异步发送 RTU 帧。
void RtuConnection::asyncSendRequest(const ModbusRequest& request, WriteHandler handler)
{
    try
    {
        Bytes frame = request.toRaw();
        appendCRC(frame);
        asyncSend(std::move(frame), std::move(handler));
    }
    catch (...)
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::invalid_argument, 0); });
    }
}

/// @brief 序列化响应、追加 CRC 并异步发送 RTU 帧。
void RtuConnection::asyncSendResponse(const ModbusResponse& response, WriteHandler handler)
{
    try
    {
        Bytes frame = response.toRaw();
        appendCRC(frame);
        asyncSend(std::move(frame), std::move(handler));
    }
    catch (...)
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::invalid_argument, 0); });
    }
}

/// @brief 序列化异常响应、追加 CRC 并异步发送 RTU 帧。
void RtuConnection::asyncSendException(const ModbusException& exception, WriteHandler handler)
{
    try
    {
        Bytes frame = exception.toRaw();
        appendCRC(frame);
        asyncSend(std::move(frame), std::move(handler));
    }
    catch (...)
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::invalid_argument, 0); });
    }
}

/// @brief 异步发送已经包含 CRC 的完整 RTU 帧。
void RtuConnection::asyncSend(Bytes data, WriteHandler handler)
{
    if (!isOpen())
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::bad_descriptor, 0); });
        return;
    }

    writeQueue_.push_back({std::move(data), std::move(handler)});
    startNextWrite();
}

/// @brief 依次发送写队列中的下一个 RTU 帧。
void RtuConnection::startNextWrite()
{
    if (writeInProgress_ || writeQueue_.empty() || !isOpen())
        return;
    writeInProgress_ = true;
    asio::async_write(serialPort_, asio::buffer(writeQueue_.front().data),
                      [this](const std::error_code& error, std::size_t bytes)
                      {
                          auto pending = std::move(writeQueue_.front());
                          writeQueue_.pop_front();
                          writeInProgress_ = false;
                          pending.handler(error, bytes);
                          startNextWrite();
                      });
}

/// @brief 异步等待一个完整 RTU 响应帧。
void RtuConnection::asyncAwaitRawMessage(RawHandler handler)
{
    asyncAwaitFrame(responseFrameLength, std::move(handler));
}

/// @brief 异步接收、校验并解析一个 RTU 请求。
void RtuConnection::asyncAwaitRequest(RequestHandler handler)
{
    asyncAwaitFrame(requestFrameLength,
                    [handler = std::move(handler)](const std::error_code& error, Bytes data) mutable
                    {
                        if (error)
                        {
                            handler(error, ModbusRequest{});
                            return;
                        }

                        try
                        {
                            ModbusRequest request = ModbusRequest::fromRawCRC(data);
                            issac_modbus::validateRtuUnitId(request.unitId());
                            handler(std::error_code{}, std::move(request));
                        }
                        catch (...)
                        {
                            handler(asio::error::invalid_argument, ModbusRequest{});
                        }
                    });
}

/// @brief 异步接收、校验并解析一个 RTU 响应。
void RtuConnection::asyncAwaitResponse(ResponseHandler handler)
{
    asyncAwaitRawMessage(
        [handler = std::move(handler)](const std::error_code& error, Bytes data) mutable
        {
            if (error)
            {
                handler(error, ModbusResponse{});
                return;
            }

            if (ModbusException::exist(data))
            {
                handler(asio::error::invalid_argument, ModbusResponse{});
                return;
            }

            try
            {
                handler(std::error_code{}, ModbusResponse::fromRawCRC(data));
            }
            catch (...)
            {
                handler(asio::error::invalid_argument, ModbusResponse{});
            }
        });
}

/// @brief 发送 RTU 请求并处理响应、超时、异常和重试。
void RtuConnection::asyncTransaction(const ModbusRequest& request, TransactionHandler handler,
                                  RetryPolicy policy)
{
    if (transactionInProgress_)
    {
        asio::post(ioContext_,
                   [handler = std::move(handler)]() mutable
                   {
                       handler(issac_modbus::makeErrorCode(MBErrorCode::MBConcurrentOperation),
                               ModbusResponse{}, std::nullopt);
                   });
        return;
    }

    try
    {
        issac_modbus::validateRequestLimits(request);
        issac_modbus::validateUnitId(request.unitId());
    }
    catch (...)
    {
        asio::post(ioContext_,
                   [handler = std::move(handler)]() mutable
                   {
                       handler(issac_modbus::makeErrorCode(MBErrorCode::MBNumberOfValuesInvalid),
                               ModbusResponse{}, std::nullopt);
                   });
        return;
    }

    transactionInProgress_ = true;
    auto requestState = std::make_shared<ModbusRequest>(request);
    auto attempt = std::make_shared<std::size_t>(0);
    auto run = std::make_shared<std::function<void()>>();
    *run = [this, requestState, attempt, policy, handler = std::move(handler), run]() mutable
    {
        ++*attempt;
        try
        {
            Bytes frame = requestState->toRaw();
            appendCRC(frame);
            asyncSend(
                std::move(frame),
                [this, requestState, attempt, policy, handler, run](const ErrorCode& writeError,
                                                                    std::size_t) mutable
                {
                    if (writeError)
                    {
                        if (*attempt < std::max<std::size_t>(1, policy.maxAttempts) &&
                            policy.retryOnConnectionError)
                        {
                            auto timer = std::make_shared<asio::steady_timer>(ioContext_);
                            timer->expires_after(policy.retryDelay);
                            timer->async_wait([run, timer](const ErrorCode&) { (*run)(); });
                            return;
                        }
                        transactionInProgress_ = false;
                        handler(writeError, ModbusResponse{}, std::nullopt);
                        return;
                    }

                    if (issac_modbus::isBroadcast(*requestState))
                    {
                        transactionInProgress_ = false;
                        handler(ErrorCode{}, ModbusResponse{}, std::nullopt);
                        return;
                    }

                    asyncAwaitRawMessage(
                        [this, requestState, attempt, policy, handler,
                         run](const ErrorCode& readError, Bytes data) mutable
                        {
                            ErrorCode result = readError;
                            std::optional<ModbusException> exception;
                            ModbusResponse response;
                            try
                            {
                                if (!result)
                                {
                                    if (ModbusException::exist(data))
                                    {
                                        exception.emplace(data, true);
                                        result = issac_modbus::makeErrorCode(exception->getErrorCode());
                                    }
                                    else
                                    {
                                        const std::uint16_t receivedCrc =
                                            data.size() >= 2 ? static_cast<std::uint16_t>(
                                                                   data[data.size() - 2]) |
                                                                   static_cast<std::uint16_t>(
                                                                       data[data.size() - 1] << 8u)
                                                             : 0;
                                        if (data.size() < 2 ||
                                            calculateCRC(data.data(), data.size() - 2) !=
                                                receivedCrc)
                                        {
                                            result =
                                                issac_modbus::makeErrorCode(MBErrorCode::MBErrorCodeCRCError);
                                        }
                                        if (!result)
                                        {
                                            response = ModbusResponse::fromRawCRC(data);
                                            issac_modbus::validateResponseIdentity(*requestState, response);
                                            issac_modbus::normalizeResponseForRequest(*requestState,
                                                                            response);
                                        }
                                    }
                                }
                            }
                            catch (...)
                            {
                                result = issac_modbus::makeErrorCode(MBErrorCode::MBBadFrame);
                            }

                            const bool isTimeout =
                                result == asio::error::timed_out ||
                                result == issac_modbus::makeErrorCode(MBErrorCode::MBFrameTimeout);
                            const bool isCrcError =
                                result == issac_modbus::makeErrorCode(MBErrorCode::MBErrorCodeCRCError);
                            const bool isBadFrame =
                                result == issac_modbus::makeErrorCode(MBErrorCode::MBBadFrame);
                            const bool retryable = (isTimeout && policy.retryOnTimeout) ||
                                                   (isCrcError && policy.retryOnCrcError) ||
                                                   (isBadFrame && policy.retryOnCrcError);
                            if (result && retryable &&
                                *attempt < std::max<std::size_t>(1, policy.maxAttempts))
                            {
                                auto timer = std::make_shared<asio::steady_timer>(ioContext_);
                                timer->expires_after(policy.retryDelay);
                                timer->async_wait([run, timer](const ErrorCode&) { (*run)(); });
                                return;
                            }

                            transactionInProgress_ = false;
                            handler(result, std::move(response), std::move(exception));
                        });
                });
        }
        catch (...)
        {
            transactionInProgress_ = false;
            handler(issac_modbus::makeErrorCode(MBErrorCode::MBBadFrame), ModbusResponse{}, std::nullopt);
        }
    };
    (*run)();
}

namespace
{
template <typename Build>
/// @brief 使用请求构造器执行一次 RTU 值操作。
void serialValues(RtuConnection& connection, Build build, RtuConnection::ValuesHandler handler,
                  RetryPolicy policy)
{
    ModbusRequest request = build();
    connection.asyncTransaction(
        request,
        [handler = std::move(handler)](const RtuConnection::ErrorCode& error, ModbusResponse response,
                                       std::optional<ModbusException> exception) mutable
        {
            if (error)
            {
                handler(error, {}, std::move(exception));
                return;
            }
            handler({}, response.registerValues(), std::move(exception));
        },
        policy);
}
} // namespace

/// @brief 构造并执行读取线圈事务。
void RtuConnection::asyncReadCoils(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                ValuesHandler handler, RetryPolicy policy)
{
    serialValues(
        *this, [=] { return ModbusRequest(unit, MBfunctionCode::MBReadCoils, address, count); },
        std::move(handler), policy);
}

/// @brief 构造并执行读取离散输入事务。
void RtuConnection::asyncReadDiscreteInputs(std::uint8_t unit, std::uint16_t address,
                                         std::uint16_t count, ValuesHandler handler,
                                         RetryPolicy policy)
{
    serialValues(
        *this,
        [=] { return ModbusRequest(unit, MBfunctionCode::MBReadDiscreteInputs, address, count); },
        std::move(handler), policy);
}

/// @brief 构造并执行读取保持寄存器事务。
void RtuConnection::asyncReadHoldingRegisters(std::uint8_t unit, std::uint16_t address,
                                           std::uint16_t count, ValuesHandler handler,
                                           RetryPolicy policy)
{
    serialValues(
        *this,
        [=] { return ModbusRequest(unit, MBfunctionCode::MBReadHoldingRegisters, address, count); },
        std::move(handler), policy);
}

/// @brief 构造并执行读取输入寄存器事务。
void RtuConnection::asyncReadInputRegisters(std::uint8_t unit, std::uint16_t address,
                                         std::uint16_t count, ValuesHandler handler,
                                         RetryPolicy policy)
{
    serialValues(
        *this,
        [=] { return ModbusRequest(unit, MBfunctionCode::MBReadInputRegisters, address, count); },
        std::move(handler), policy);
}

/// @brief 构造并执行写单个线圈事务。
void RtuConnection::asyncWriteSingleCoil(std::uint8_t unit, std::uint16_t address, bool value,
                                      ValuesHandler handler, RetryPolicy policy)
{
    serialValues(
        *this,
        [=] {
            return ModbusRequest(unit, MBfunctionCode::MBWriteSingleCoil, address, 1,
                                 {ModbusCell(value)});
        },
        std::move(handler), policy);
}

/// @brief 构造并执行写单个保持寄存器事务。
void RtuConnection::asyncWriteSingleRegister(std::uint8_t unit, std::uint16_t address,
                                          std::uint16_t value, ValuesHandler handler,
                                          RetryPolicy policy)
{
    serialValues(
        *this,
        [=]
        {
            return ModbusRequest(unit, MBfunctionCode::MBWriteSingleRegister, address, 1,
                                 {ModbusCell(value)});
        },
        std::move(handler), policy);
}

/// @brief 构造并执行写多个保持寄存器事务。
void RtuConnection::asyncWriteMultipleRegisters(std::uint8_t unit, std::uint16_t address,
                                             const std::vector<std::uint16_t>& values,
                                             ValuesHandler handler, RetryPolicy policy)
{
    std::vector<ModbusCell> cells;
    for (auto value : values)
        cells.emplace_back(value);
    serialValues(
        *this,
        [=, cells = std::move(cells)]() mutable
        {
            return ModbusRequest(unit, MBfunctionCode::MBWriteMultipleRegisters, address,
                                 static_cast<std::uint16_t>(cells.size()), cells);
        },
        std::move(handler), policy);
}

/// @brief 构造并执行写多个线圈事务。
void RtuConnection::asyncWriteMultipleCoils(std::uint8_t unit, std::uint16_t address,
                                         const std::vector<bool>& values, ValuesHandler handler,
                                         RetryPolicy policy)
{
    std::vector<ModbusCell> cells;
    for (bool value : values)
        cells.emplace_back(value);
    serialValues(
        *this,
        [=, cells = std::move(cells)]() mutable
        {
            return ModbusRequest(unit, MBfunctionCode::MBWriteMultipleCoils, address,
                                 static_cast<std::uint16_t>(cells.size()), cells);
        },
        std::move(handler), policy);
}

/// @brief 构造并执行保持寄存器掩码写事务。
void RtuConnection::asyncMaskWriteRegister(std::uint8_t unit, std::uint16_t address,
                                        std::uint16_t andMask, std::uint16_t orMask,
                                        ValuesHandler handler, RetryPolicy policy)
{
    serialValues(
        *this,
        [=]
        {
            return ModbusRequest(unit, MBfunctionCode::MBMaskWriteRegister, address, 2,
                                 {ModbusCell(andMask), ModbusCell(orMask)});
        },
        std::move(handler), policy);
}

/// @brief 构造并执行读写多个保持寄存器事务。
void RtuConnection::asyncReadWriteMultipleRegisters(std::uint8_t unit, std::uint16_t readAddress,
                                                 std::uint16_t readCount,
                                                 std::uint16_t writeAddress,
                                                 const std::vector<std::uint16_t>& values,
                                                 ValuesHandler handler, RetryPolicy policy)
{
    std::vector<ModbusCell> cells;
    for (auto value : values)
        cells.emplace_back(value);
    serialValues(
        *this,
        [=, cells = std::move(cells)]() mutable
        {
            return ModbusRequest(unit, MBfunctionCode::MBReadWriteMultipleRegisters, readAddress,
                                 readCount, cells, writeAddress);
        },
        std::move(handler), policy);
}

/// @brief 构造并执行读取从站标识事务。
void RtuConnection::asyncReportSlaveId(std::uint8_t unit, ValuesHandler handler, RetryPolicy policy)
{
    serialValues(
        *this, [=] { return ModbusRequest(unit, MBfunctionCode::MBReportSlaveID); },
        std::move(handler), policy);
}

/// @brief 在没有业务钩子时持续处理 RTU 请求。
void RtuConnection::asyncServe(Mapping& mapping, std::function<void(const ErrorCode&)> handler)
{
    asyncServe(mapping, {}, std::move(handler));
}

/// @brief 在业务钩子参与下持续接收、处理并回复 RTU 请求。
void RtuConnection::asyncServe(Mapping& mapping, MappingHooks hooks,
                            std::function<void(const ErrorCode&)> handler)
{
    auto receive = std::make_shared<std::function<void()>>();
    *receive = [this, &mapping, hooks, receive, handler]() mutable
    {
        asyncAwaitRequest(
            [this, &mapping, hooks, receive, handler](const ErrorCode& error,
                                                      ModbusRequest request) mutable
            {
                if (error)
                {
                    const bool recoverable =
                        error == asio::error::timed_out ||
                        error == issac_modbus::makeErrorCode(MBErrorCode::MBFrameTimeout) ||
                        error == asio::error::invalid_argument ||
                        error == asio::error::message_size;
                    if (recoverable && isOpen())
                    {
                        // RTU bus idle and a discarded/broken frame are normal runtime
                        // conditions.  Flush the partial frame and keep serving.
                        clearInput();
                        (*receive)();
                        return;
                    }
                    if (handler)
                        handler(error);
                    return;
                }
                auto process = [this, &mapping, hooks, receive, handler, request]() mutable
                {
                    try
                    {
                        if (request.unitId() == 0)
                        {
                            issac_modbus::handleRequest(request, mapping, hooks);
                            (*receive)();
                            return;
                        }
                        ModbusResponse response = issac_modbus::handleRequest(request, mapping, hooks);
                        asyncSendResponse(
                            response,
                            [receive, handler](const ErrorCode& sendError, std::size_t) mutable
                            {
                                if (sendError)
                                {
                                    if (handler)
                                        handler(sendError);
                                    return;
                                }
                                (*receive)();
                            });
                    }
                    catch (const ModbusException& exception)
                    {
                        asyncSendException(
                            exception,
                            [receive, handler](const ErrorCode& sendError, std::size_t) mutable
                            {
                                if (sendError)
                                {
                                    if (handler)
                                        handler(sendError);
                                    return;
                                }
                                (*receive)();
                            });
                    }
                };
                if (hooks.dispatch)
                {
                    hooks.dispatch(std::move(process));
                }
                else
                {
                    process();
                }
            });
    };
    (*receive)();
}

/// @brief 逐字节读取数据并等待解析器确认完整 RTU 帧。
void RtuConnection::asyncAwaitFrame(const FrameLengthResolver& resolver, RawHandler handler)
{
    if (!isOpen())
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::bad_descriptor, {}); });
        return;
    }

    auto state = std::make_shared<ReadState>();
    state->resolver = resolver;
    state->handler = std::move(handler);
    state->data.reserve(256);
    state->timer = std::make_shared<asio::steady_timer>(ioContext_);
    state->byteTimer = std::make_shared<asio::steady_timer>(ioContext_);
    state->timer->expires_after(timeout_);

    state->timer->async_wait(
        [this, state](const std::error_code& error)
        {
            if (error || state->completed)
            {
                return;
            }

            state->completed = true;
            std::error_code ignored;
            serialPort_.cancel(ignored);
            state->byteTimer->cancel();
            state->handler(state->receivedFirstByte ? issac_modbus::makeErrorCode(MBErrorCode::MBFrameTimeout)
                                                    : asio::error::timed_out,
                           std::move(state->data));
        });

    readNextByte(state);
}

/// @brief 读取下一个串口字节并检查帧完成、超时和长度上限。
void RtuConnection::readNextByte(const std::shared_ptr<ReadState>& state)
{
    if (state->completed)
    {
        return;
    }

    serialPort_.async_read_some(
        asio::buffer(&state->byte, sizeof(state->byte)),
        [this, state](const std::error_code& error, std::size_t bytesTransferred)
        {
            if (state->completed)
            {
                return;
            }

            if (error)
            {
                state->completed = true;
                state->timer->cancel();
                state->handler(error, std::move(state->data));
                return;
            }

            if (bytesTransferred == 1)
            {
                state->data.push_back(state->byte);
                if (!state->receivedFirstByte)
                {
                    state->receivedFirstByte = true;
                    state->timer->cancel();
                }
                state->byteTimer->expires_after(byteTimeout_);
                state->byteTimer->async_wait(
                    [this, state](const std::error_code& timeoutError)
                    {
                        if (timeoutError || state->completed)
                            return;
                        state->completed = true;
                        std::error_code ignored;
                        serialPort_.cancel(ignored);
                        state->handler(issac_modbus::makeErrorCode(MBErrorCode::MBFrameTimeout),
                                       std::move(state->data));
                    });
            }

            std::optional<std::size_t> frameLength;
            try
            {
                frameLength = state->resolver(state->data);
            }
            catch (...)
            {
                state->completed = true;
                state->timer->cancel();
                state->byteTimer->cancel();
                state->handler(asio::error::invalid_argument, std::move(state->data));
                return;
            }

            if (frameLength.has_value() && state->data.size() >= *frameLength)
            {
                state->data.resize(*frameLength);
                state->completed = true;
                state->timer->cancel();
                state->byteTimer->cancel();
                state->handler(std::error_code{}, std::move(state->data));
                return;
            }

            if (state->data.size() >= 512)
            {
                state->completed = true;
                state->timer->cancel();
                state->byteTimer->cancel();
                state->handler(asio::error::message_size, std::move(state->data));
                return;
            }

            readNextByte(state);
        });
}

} // namespace issac_modbus
