#include "serialTransport.h"

#include "rtuCodec.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <linux/serial.h>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <sys/ioctl.h>
#include <termios.h>
#include <tuple>
#include <unistd.h>
#include <utility>

namespace issac_modbus::detail
{
namespace
{

using Clock = std::chrono::steady_clock;
using Microseconds = std::chrono::microseconds;
using Milliseconds = std::chrono::milliseconds;
using Error = std::error_code;

struct TimerSlot
{
    explicit TimerSlot(asio::io_context& io) : timer(io) {}

    asio::steady_timer timer;
    std::uint64_t generation = 0;
};

Microseconds charTime(const SerialOptions& options)
{
    const std::uint64_t bits = 1u + 8u + (options.parity != Parity::None ? 1u : 0u) +
                               (options.stopBits == StopBits::Two ? 2u : 1u);
    return Microseconds((bits * 1000000u + options.baudRate - 1u) / options.baudRate);
}

Microseconds byteGap(const SerialOptions& options, Microseconds explicitTimeout)
{
    if (explicitTimeout.count() > 0)
        return explicitTimeout;
    return options.baudRate > 19200 ? Microseconds(750)
                                    : Microseconds((charTime(options).count() * 3 + 1) / 2);
}

Microseconds frameGap(const SerialOptions& options)
{
    const auto automatic = options.baudRate > 19200
                               ? Microseconds(1750)
                               : Microseconds((charTime(options).count() * 7 + 1) / 2);
    return std::max(automatic, options.frameGap);
}

void validateOptions(const SerialOptions& options)
{
    if (options.baudRate == 0 || options.baudRate > 4000000u)
        throw std::invalid_argument("Invalid RTU baud rate");
    if (options.dataBits != DataBits::Eight)
        throw std::invalid_argument("Modbus RTU requires eight data bits");
    if (options.flowControl == FlowControl::Software)
        throw std::invalid_argument("Software flow control is unsafe for RTU binary data");
    if (options.flowControl != FlowControl::None && options.flowControl != FlowControl::Hardware)
        throw std::invalid_argument("Invalid RTU flow control");
    if (options.parity != Parity::None && options.parity != Parity::Even &&
        options.parity != Parity::Odd)
    {
        throw std::invalid_argument("Invalid RTU parity");
    }
    if (options.stopBits != StopBits::One && options.stopBits != StopBits::Two)
        throw std::invalid_argument("Invalid RTU stop bits");
    if (options.serialMode != SerialMode::RS232 && options.serialMode != SerialMode::RS485)
        throw std::invalid_argument("Invalid RTU serial mode");
    if (options.serialMode == SerialMode::RS485 && options.flowControl != FlowControl::None)
        throw std::invalid_argument("RS485 direction control must not use RTS/CTS flow control");
    if (options.rs485.enabled && options.serialMode != SerialMode::RS485)
        throw std::invalid_argument("Kernel RS485 control requires RS485 serial mode");
    if (options.rs485.enabled && options.rs485.rtsOnSend == options.rs485.rtsAfterSend)
        throw std::invalid_argument("RS485 RTS direction levels must differ");
    if (options.frameTimeout.count() <= 0 || options.writeTimeout.count() <= 0 ||
        options.byteTimeout.count() < 0 || options.frameGap.count() < 0 ||
        options.recoveryDelay.count() < 0)
    {
        throw std::invalid_argument("Invalid RTU timeout");
    }
    constexpr auto maxTime = std::chrono::hours(24);
    if (options.frameTimeout > maxTime || options.writeTimeout > maxTime ||
        options.byteTimeout > maxTime || options.frameGap > maxTime ||
        options.recoveryDelay > maxTime || options.rs485.delayBeforeSendMs > 86400000u ||
        options.rs485.delayAfterSendMs > 86400000u)
    {
        throw std::invalid_argument("RTU timeout exceeds 24 hours");
    }
}

} // namespace

struct SerialTransport::State final : std::enable_shared_from_this<SerialTransport::State>
{
    enum class Kind
    {
        Write,
        Read
    };

    struct Operation final
    {
        Operation(asio::io_context& io, Kind value)
            : kind(value), deadline(io), byteDeadline(io), phase(io)
        {
        }

        Kind kind;
        RtuBytes input;
        RtuBytes output;
        RtuWriteHandler writeHandler;
        RtuRawHandler readHandler;
        TimerSlot deadline;
        TimerSlot byteDeadline;
        TimerSlot phase;
        std::uint8_t byte = 0;
        std::size_t written = 0;
        bool ioPending = false;
        bool boundary = false;
        bool receivedFirstByte = false;
        bool stopping = false;
        bool discardInputBeforeWrite = false;
        RtuFrameType frameType = RtuFrameType::Response;
        Milliseconds firstByteTimeout{0};
        Milliseconds frameTimeout{0};
        Microseconds byteTimeout{0};
        Error ioError;
        Error stopError;
    };

    State(asio::io_context& context, std::string pathValue, SerialOptions optionsValue)
        : io(context), strand(asio::make_strand(context)), port(context),
          path(std::move(pathValue)), config(optionsValue)
    {
        validateOptions(config);
    }

    asio::io_context& io;
    asio::strand<asio::io_context::executor_type> strand;
    asio::serial_port port;
    const std::string path;
    mutable std::mutex controlMutex;
    SerialOptions config;
    std::atomic<bool> alive{true};
    std::atomic<bool> openFlag{false};
    std::atomic<bool> busyFlag{false};
    std::atomic<bool> activeFlag{false};
    std::atomic<std::uint64_t> submissionEpoch{0};
    bool rs485Managed = false;
    std::shared_ptr<Operation> active;
    Clock::time_point nextTransmit = Clock::now();

    template <typename Handler, typename... Args> void notify(Handler handler, Args... args)
    {
        if (!handler)
            return;
        auto self = shared_from_this();
        asio::post(strand, [self, handler = std::move(handler),
                            values = std::make_tuple(std::move(args)...)]() mutable
                   { std::apply(handler, std::move(values)); });
    }

    static void cancelTimer(TimerSlot& slot)
    {
        ++slot.generation;
        Error ignored;
        slot.timer.cancel(ignored);
    }

    static void cancelTimers(Operation& operation)
    {
        cancelTimer(operation.deadline);
        cancelTimer(operation.byteDeadline);
        cancelTimer(operation.phase);
    }

    template <typename Duration, typename Function>
    void arm(TimerSlot& slot, const std::shared_ptr<Operation>& operation, Duration delay,
             Function function)
    {
        const auto armedGeneration = ++slot.generation;
        slot.timer.expires_after(delay);
        auto self = shared_from_this();
        slot.timer.async_wait(asio::bind_executor(
            strand,
            [self, operation, slotPointer = &slot, armedGeneration,
             function = std::move(function)](const Error& error) mutable
            {
                if (error || !self->alive.load() || self->active != operation ||
                    operation->stopping || slotPointer->generation != armedGeneration)
                {
                    return;
                }
                function();
            }));
    }

    Error flushInput()
    {
        if (!port.is_open())
            return asio::error::bad_descriptor;
        if (::tcflush(port.native_handle(), TCIFLUSH) < 0)
            return systemError();
        return {};
    }

    void apply(const SerialOptions& options)
    {
        using Base = asio::serial_port_base;
        port.set_option(Base::baud_rate(options.baudRate));
        port.set_option(Base::character_size(
            options.dataBits == DataBits::Eight ? 8 : static_cast<unsigned int>(options.dataBits)));
        port.set_option(Base::parity(options.parity == Parity::Even  ? Base::parity::even
                                     : options.parity == Parity::Odd ? Base::parity::odd
                                                                     : Base::parity::none));
        port.set_option(Base::stop_bits(options.stopBits == StopBits::Two ? Base::stop_bits::two
                                                                          : Base::stop_bits::one));
        port.set_option(Base::flow_control(options.flowControl == FlowControl::Hardware
                                               ? Base::flow_control::hardware
                                               : Base::flow_control::none));

        if (!options.rs485.enabled && !rs485Managed)
            return;

        serial_rs485 settings{};
        if (::ioctl(port.native_handle(), TIOCGRS485, &settings) < 0)
            throw std::system_error(systemError(), "TIOCGRS485");
        constexpr std::uint32_t controlled = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND |
                                             SER_RS485_RTS_AFTER_SEND | SER_RS485_RX_DURING_TX;
        settings.flags &= ~controlled;
        settings.delay_rts_before_send = 0;
        settings.delay_rts_after_send = 0;
        if (options.rs485.enabled)
        {
            settings.flags |= SER_RS485_ENABLED;
            if (options.rs485.rtsOnSend)
                settings.flags |= SER_RS485_RTS_ON_SEND;
            if (options.rs485.rtsAfterSend)
                settings.flags |= SER_RS485_RTS_AFTER_SEND;
            if (options.rs485.receiveDuringSend)
                settings.flags |= SER_RS485_RX_DURING_TX;
            settings.delay_rts_before_send = options.rs485.delayBeforeSendMs;
            settings.delay_rts_after_send = options.rs485.delayAfterSendMs;
        }
        if (::ioctl(port.native_handle(), TIOCSRS485, &settings) < 0)
            throw std::system_error(systemError(), "TIOCSRS485");
        rs485Managed = options.rs485.enabled;
    }

    void reject(const std::shared_ptr<Operation>& operation, Error error)
    {
        cancelTimers(*operation);
        complete(operation, error);
    }

    void complete(const std::shared_ptr<Operation>& operation, Error error)
    {
        if (operation->kind == Kind::Write)
            notify(std::move(operation->writeHandler), error, operation->written);
        else
            notify(std::move(operation->readHandler), error, std::move(operation->input));
    }

    void finish(const std::shared_ptr<Operation>& operation, Error error)
    {
        if (active != operation)
            return;
        cancelTimers(*operation);
        active.reset();
        activeFlag.store(false);
        complete(operation, error);
        if (!openFlag.load())
            finishClose();
        else
            busyFlag.store(false);
    }

    void fail(const std::shared_ptr<Operation>& operation, Error error)
    {
        if (active != operation)
            return;
        finish(operation, error);
    }

    void cancelActive(Error error)
    {
        if (!active)
            return;
        auto operation = active;
        operation->stopping = true;
        operation->stopError = error;
        cancelTimers(*operation);
        if (operation->ioPending)
        {
            Error ignored;
            port.cancel(ignored);
        }
        if (!operation->ioPending)
            finish(operation, error);
    }

    void admitWrite(const std::shared_ptr<Operation>& operation, std::uint64_t submittedEpoch)
    {
        if (submittedEpoch != submissionEpoch.load())
        {
            reject(operation, asio::error::operation_aborted);
            return;
        }
        if (!alive.load() || !openFlag.load())
        {
            reject(operation, asio::error::bad_descriptor);
            return;
        }
        if (active)
        {
            reject(operation, mbError(MBErrorCode::MBConcurrentOperation));
            return;
        }
        active = operation;
        activeFlag.store(true);
        busyFlag.store(true);
        beginWrite(operation);
    }

    void admitRead(const std::shared_ptr<Operation>& operation, std::uint64_t submittedEpoch)
    {
        if (submittedEpoch != submissionEpoch.load())
        {
            reject(operation, asio::error::operation_aborted);
            return;
        }
        if (!alive.load() || !openFlag.load())
        {
            reject(operation, asio::error::bad_descriptor);
            return;
        }
        if (active)
        {
            reject(operation, mbError(MBErrorCode::MBConcurrentOperation));
            return;
        }
        active = operation;
        activeFlag.store(true);
        busyFlag.store(true);
        beginRead(operation);
    }

    void beginWrite(const std::shared_ptr<Operation>& operation)
    {
        const auto now = Clock::now();
        if (nextTransmit > now)
        {
            arm(operation->phase, operation, nextTransmit - now,
                [self = shared_from_this(), operation] { self->beginWrite(operation); });
            return;
        }

        if (operation->discardInputBeforeWrite)
        {
            if (auto error = flushInput())
            {
                fail(operation, error);
                return;
            }
        }

        operation->ioPending = true;
        arm(operation->deadline, operation, config.writeTimeout,
            [self = shared_from_this(), operation]
            {
                if (operation->ioPending)
                {
                    operation->ioError = asio::error::timed_out;
                    Error ignored;
                    self->port.cancel(ignored);
                }
                else
                {
                    self->fail(operation, asio::error::timed_out);
                }
            });
        auto self = shared_from_this();
        asio::async_write(port, asio::buffer(operation->output),
                          asio::bind_executor(strand,
                                              [self, operation](Error error, std::size_t count)
                                              {
                                                  operation->ioPending = false;
                                                  operation->written = count;
                                                  if (self->active != operation)
                                                      return;
                                                  if (operation->stopping)
                                                  {
                                                      self->finish(operation, operation->stopError);
                                                      return;
                                                  }
                                                  if (operation->ioError)
                                                      error = operation->ioError;
                                                  if (error)
                                                  {
                                                      self->fail(operation, error);
                                                      return;
                                                  }
                                                  self->beginDrain(operation);
                                              }));
    }

    void beginDrain(const std::shared_ptr<Operation>& operation)
    {
        int queued = 0;
        if (::ioctl(port.native_handle(), TIOCOUTQ, &queued) < 0)
        {
            if (errno != ENOTTY && errno != EINVAL)
            {
                fail(operation, systemError());
                return;
            }
            queued = 0;
        }
        if (queued > 0)
        {
            arm(operation->phase, operation, Milliseconds(1),
                [self = shared_from_this(), operation] { self->beginDrain(operation); });
            return;
        }

        const auto guard =
            std::chrono::duration_cast<Milliseconds>(charTime(config) * operation->output.size()) +
            Milliseconds(1) +
            (config.rs485.enabled
                 ? Milliseconds(config.rs485.delayBeforeSendMs + config.rs485.delayAfterSendMs)
                 : Milliseconds(0));
        arm(operation->phase, operation, guard,
            [self = shared_from_this(), operation]
            {
                cancelTimer(operation->deadline);
                self->nextTransmit = Clock::now() + frameGap(self->config);
                self->finish(operation, {});
            });
    }

    void beginRead(const std::shared_ptr<Operation>& operation)
    {
        operation->input.clear();
        operation->input.reserve(MaxRtuAduSize);
        operation->receivedFirstByte = false;
        operation->boundary = false;
        operation->ioError.clear();
        if (operation->firstByteTimeout.count() > 0)
        {
            arm(operation->deadline, operation, operation->firstByteTimeout,
                [self = shared_from_this(), operation]
                { self->cancelRead(operation, asio::error::timed_out); });
        }
        readByte(operation);
    }

    void cancelRead(const std::shared_ptr<Operation>& operation, Error error, bool boundary = false)
    {
        if (operation->ioError || operation->boundary)
            return;
        operation->ioError = error;
        operation->boundary = boundary;
        Error ignored;
        port.cancel(ignored);
    }

    void readByte(const std::shared_ptr<Operation>& operation)
    {
        operation->ioPending = true;
        auto self = shared_from_this();
        port.async_read_some(
            asio::buffer(&operation->byte, 1),
            asio::bind_executor(
                strand,
                [self, operation](Error error, std::size_t count)
                {
                    operation->ioPending = false;
                    if (self->active != operation)
                        return;
                    if (operation->stopping)
                    {
                        self->finish(operation, operation->stopError);
                        return;
                    }
                    if (operation->boundary)
                    {
                        operation->boundary = false;
                        if (count == 0)
                        {
                            self->frameReceived(operation);
                            return;
                        }
                    }
                    if (operation->ioError)
                    {
                        self->fail(operation, operation->ioError);
                        return;
                    }
                    if (error && count == 0)
                    {
                        self->fail(operation, error);
                        return;
                    }
                    if (count == 0)
                    {
                        self->readByte(operation);
                        return;
                    }
                    if (operation->input.size() >= MaxRtuAduSize)
                    {
                        self->fail(operation, asio::error::message_size);
                        return;
                    }

                    operation->input.push_back(operation->byte);
                    self->nextTransmit =
                        std::max(self->nextTransmit, Clock::now() + frameGap(self->config));
                    if (!operation->receivedFirstByte)
                    {
                        operation->receivedFirstByte = true;
                        self->arm(
                            operation->deadline, operation, operation->frameTimeout,
                            [self, operation]
                            { self->cancelRead(operation, mbError(MBErrorCode::MBFrameTimeout)); });
                    }

                    const bool request = operation->frameType == RtuFrameType::Request;
                    const auto length = frameLength(operation->input, request);
                    if (length && (*length < 4 || *length > MaxRtuAduSize))
                    {
                        self->fail(operation, asio::error::message_size);
                        return;
                    }
                    if (length && operation->input.size() == *length)
                    {
                        self->frameReceived(operation);
                        return;
                    }

                    const bool unknown = operation->input.size() >= 2 &&
                                         !supportedFunction(operation->input[1]) &&
                                         (request || (operation->input[1] & 0x80u) == 0);
                    const auto gap = unknown
                                         ? std::max(frameGap(self->config),
                                                    byteGap(self->config, operation->byteTimeout))
                                         : byteGap(self->config, operation->byteTimeout);
                    self->arm(operation->byteDeadline, operation, gap,
                              [self, operation, unknown]
                              {
                                  self->cancelRead(operation,
                                                   unknown ? Error{}
                                                           : mbError(MBErrorCode::MBFrameTimeout),
                                                   unknown);
                              });
                    self->readByte(operation);
                }));
    }

    void frameReceived(const std::shared_ptr<Operation>& operation)
    {
        cancelTimer(operation->deadline);
        cancelTimer(operation->byteDeadline);
        if (!validCrc(operation->input))
        {
            fail(operation, mbError(MBErrorCode::MBErrorCodeCRCError));
            return;
        }
        finish(operation, {});
    }

    void closeOnStrand()
    {
        openFlag.store(false);
        busyFlag.store(true);
        cancelActive(asio::error::operation_aborted);
        if (!active)
            finishClose();
    }

    void finishClose()
    {
        if (active)
            return;
        Error ignored;
        port.cancel(ignored);
        port.close(ignored);
        busyFlag.store(false);
    }
};

SerialTransport::SerialTransport(asio::io_context& io, std::string device, SerialOptions options)
    : state_(std::make_shared<State>(io, std::move(device), options))
{
}

SerialTransport::~SerialTransport()
{
    state_->alive.store(false);
    close();
}

void SerialTransport::open()
{
    std::lock_guard<std::mutex> lock(state_->controlMutex);
    if (state_->openFlag.load() || state_->busyFlag.load())
        throw std::system_error(mbError(MBErrorCode::MBConcurrentOperation),
                                "RTU transport is not idle");
    Error error;
    state_->port.open(state_->path, error);
    if (error)
        throw std::system_error(error);
    try
    {
        state_->apply(state_->config);
    }
    catch (...)
    {
        Error ignored;
        state_->port.close(ignored);
        throw;
    }
    state_->openFlag.store(true);
    state_->nextTransmit = Clock::now();
}

void SerialTransport::setOptions(const SerialOptions& options)
{
    std::lock_guard<std::mutex> lock(state_->controlMutex);
    if (state_->openFlag.load() || state_->busyFlag.load())
        throw std::system_error(mbError(MBErrorCode::MBConcurrentOperation),
                                "RTU configuration is allowed only while closed");
    validateOptions(options);
    state_->config = options;
}

SerialOptions SerialTransport::options() const
{
    std::lock_guard<std::mutex> lock(state_->controlMutex);
    return state_->config;
}

void SerialTransport::close() noexcept
{
    {
        std::lock_guard<std::mutex> lock(state_->controlMutex);
        ++state_->submissionEpoch;
        state_->openFlag.store(false);
        state_->busyFlag.store(true);
    }
    auto self = state_;
    asio::dispatch(state_->strand, [self] { self->closeOnStrand(); });
}

void SerialTransport::cancel() noexcept
{
    ++state_->submissionEpoch;
    auto self = state_;
    asio::dispatch(state_->strand, [self] { self->cancelActive(asio::error::operation_aborted); });
}

bool SerialTransport::isOpen() const noexcept { return state_->openFlag.load(); }

bool SerialTransport::busy() const noexcept { return state_->busyFlag.load(); }

bool SerialTransport::operationActive() const noexcept { return state_->activeFlag.load(); }

std::error_code SerialTransport::flushInput() { return state_->flushInput(); }

asio::io_context& SerialTransport::io() noexcept { return state_->io; }

const asio::strand<asio::io_context::executor_type>& SerialTransport::strand() const noexcept
{
    return state_->strand;
}

void SerialTransport::asyncWrite(RtuBytes adu, bool discardInputBeforeWrite,
                                 RtuWriteHandler handler)
{
    if (adu.size() < 4 || adu.size() > MaxRtuAduSize)
    {
        state_->notify(std::move(handler), Error(asio::error::message_size), std::size_t{0});
        return;
    }
    if (!validCrc(adu))
    {
        state_->notify(std::move(handler), mbError(MBErrorCode::MBErrorCodeCRCError),
                       std::size_t{0});
        return;
    }

    auto operation = std::make_shared<State::Operation>(state_->io, State::Kind::Write);
    operation->output = std::move(adu);
    operation->discardInputBeforeWrite = discardInputBeforeWrite;
    operation->writeHandler = std::move(handler);
    auto self = state_;
    const auto submittedEpoch = state_->submissionEpoch.load();
    asio::dispatch(state_->strand, [self, operation, submittedEpoch]
                   { self->admitWrite(operation, submittedEpoch); });
}

void SerialTransport::asyncRead(RtuFrameType type, Milliseconds firstByteTimeout,
                                Milliseconds frameTimeout, Microseconds byteTimeout,
                                RtuRawHandler handler)
{
    if (firstByteTimeout.count() < 0 || firstByteTimeout > std::chrono::hours(24) ||
        frameTimeout.count() <= 0 || frameTimeout > std::chrono::hours(24) ||
        byteTimeout.count() < 0 || byteTimeout > std::chrono::hours(24))
    {
        state_->notify(std::move(handler), Error(asio::error::invalid_argument), RtuBytes{});
        return;
    }

    auto operation = std::make_shared<State::Operation>(state_->io, State::Kind::Read);
    operation->frameType = type;
    operation->firstByteTimeout = firstByteTimeout;
    operation->frameTimeout = frameTimeout;
    operation->byteTimeout = byteTimeout;
    operation->readHandler = std::move(handler);
    auto self = state_;
    const auto submittedEpoch = state_->submissionEpoch.load();
    asio::dispatch(state_->strand, [self, operation, submittedEpoch]
                   { self->admitRead(operation, submittedEpoch); });
}

} // namespace issac_modbus::detail
