#include "issac_modbus/modbusRtu/rtuSlave.h"

#include "detail/rtuCodec.h"
#include "detail/serialTransport.h"
#include "issac_modbus/util/modbusProtocol.h"

#include <algorithm>
#include <asio/error.hpp>
#include <atomic>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace issac_modbus
{
namespace
{

using detail::badFrame;
using detail::mbError;

void validateSlaveOptions(const SlaveOptions& options)
{
    if (options.unitId == 0 || options.unitId > 247)
        throw std::invalid_argument("RTU slave address must be in range 1..247");
}

} // namespace

struct RtuSlave::Impl final : std::enable_shared_from_this<RtuSlave::Impl>
{
    Impl(asio::io_context& io, std::string device, std::shared_ptr<Mapping> mappingValue,
         SerialOptions serial, SlaveOptions slave)
        : transport(std::make_shared<detail::SerialTransport>(io, std::move(device), serial)),
          options(slave), mapping(std::move(mappingValue)), phase(io)
    {
        validateSlaveOptions(options);
        if (!mapping)
            throw std::invalid_argument("RTU slave Mapping must not be null");
    }

    std::shared_ptr<detail::SerialTransport> transport;
    const SlaveOptions options;
    const std::shared_ptr<Mapping> mapping;
    bool businessPending = false;
    std::atomic<bool> alive{true};
    std::atomic<ServiceState> serviceState{ServiceState::Idle};
    std::atomic<std::uint64_t> serviceEpoch{0};
    std::uint64_t serveGeneration = 0;
    MappingHooks hooks;
    RtuCompletionHandler handler;
    std::error_code stopError;
    asio::steady_timer phase;
    std::optional<asio::executor_work_guard<asio::io_context::executor_type>> workGuard;
    std::vector<RtuCompletionHandler> closeWaiters;
    bool closeInProgress = false;
    bool transportClosing = false;
    std::optional<detail::SerialTransport::ClosePhase> transportClosePhase;

    template <typename Handler, typename... Args> void notify(Handler handlerValue, Args... args)
    {
        if (!handlerValue)
            return;
        auto self = shared_from_this();
        asio::post(transport->strand(), [self, handlerValue = std::move(handlerValue),
                                         values = std::make_tuple(std::move(args)...)]() mutable
                   { std::apply(handlerValue, std::move(values)); });
    }

    void submitServe(MappingHooks nextHooks, RtuCompletionHandler nextHandler)
    {
        const auto requestedEpoch = serviceEpoch.load();
        auto self = shared_from_this();
        asio::dispatch(transport->strand(),
                       [self, requestedEpoch, nextHooks = std::move(nextHooks),
                        nextHandler = std::move(nextHandler)]() mutable
                       {
                           if (requestedEpoch != self->serviceEpoch.load())
                           {
                               self->notify(std::move(nextHandler), asio::error::operation_aborted);
                               return;
                           }
                           if (!self->alive.load() || !self->transport->isOpen())
                           {
                               self->notify(std::move(nextHandler), asio::error::bad_descriptor);
                               return;
                           }
                           if (self->serviceState.load() != ServiceState::Idle)
                           {
                               self->notify(std::move(nextHandler),
                                            mbError(MBErrorCode::MBConcurrentOperation));
                               return;
                           }
                           self->hooks = std::move(nextHooks);
                           self->handler = std::move(nextHandler);
                           self->serviceState.store(ServiceState::Running);
                           self->businessPending = false;
                           const auto generation = ++self->serveGeneration;
                           self->workGuard.emplace(asio::make_work_guard(self->transport->io()));
                           self->beginRead(generation);
                       });
    }

    void beginRead(std::uint64_t generation)
    {
        if (serviceState.load() != ServiceState::Running || businessPending)
            return;
        const auto serial = transport->options();
        auto self = shared_from_this();
        transport->asyncRead(RtuFrameType::Request, std::chrono::milliseconds(0),
                             serial.frameTimeout, serial.byteTimeout,
                             [self, generation](const std::error_code& error, RtuBytes data)
                             {
                                 if (self->serviceState.load() == ServiceState::Idle ||
                                     generation != self->serveGeneration)
                                     return;
                                 if (self->serviceState.load() == ServiceState::Stopping)
                                 {
                                     self->finish(self->stopError);
                                     return;
                                 }
                                 if (error)
                                 {
                                     self->recoverOrFinish(error, generation);
                                     return;
                                 }
                                 self->handleFrame(std::move(data), generation);
                             });
    }

    void recoverOrFinish(const std::error_code& error, std::uint64_t generation)
    {
        const bool recoverable =
            error == asio::error::timed_out || error == mbError(MBErrorCode::MBFrameTimeout) ||
            error == mbError(MBErrorCode::MBErrorCodeCRCError) || error == badFrame() ||
            error == asio::error::message_size || error == asio::error::invalid_argument;
        if (!recoverable)
        {
            finish(error);
            return;
        }

        const auto delay = std::max(
            transport->options().recoveryDelay,
            std::chrono::duration_cast<std::chrono::milliseconds>(transport->options().frameGap));
        phase.expires_after(delay);
        auto self = shared_from_this();
        phase.async_wait(asio::bind_executor(
            transport->strand(),
            [self, generation](const std::error_code& timerError)
            {
                if (timerError || self->serviceState.load() == ServiceState::Idle ||
                    generation != self->serveGeneration)
                    return;
                if (self->serviceState.load() == ServiceState::Stopping)
                {
                    self->finish(self->stopError);
                    return;
                }
                if (auto error = self->transport->flushInput())
                {
                    self->finish(error);
                    return;
                }
                self->beginRead(generation);
            }));
    }

    static RtuBytes exceptionFrame(std::uint8_t unit, std::uint8_t functionCode, MBErrorCode code)
    {
        return detail::withCrc({unit, static_cast<std::uint8_t>(functionCode | 0x80u),
                                static_cast<std::uint8_t>(code)});
    }

    void handleFrame(RtuBytes data, std::uint64_t generation)
    {
        if (data.size() < 4)
        {
            finish(badFrame());
            return;
        }

        const auto unit = data[0];
        const auto functionCode = data[1];
        if ((unit != options.unitId && unit != 0) ||
            (unit == 0 && !detail::broadcastWrite(functionCode)))
        {
            beginRead(generation);
            return;
        }

        if (!detail::supportedFunction(functionCode))
        {
            if (unit == 0)
            {
                beginRead(generation);
                return;
            }
            sendResponse(exceptionFrame(unit, functionCode, MBErrorCode::MBIllegalFunction),
                         generation);
            return;
        }

        ModbusRequest request;
        try
        {
            request = ModbusRequest::fromRawCRC(data);
            detail::checkRequest(request, data);
        }
        catch (const std::exception&)
        {
            if (unit == 0)
            {
                beginRead(generation);
                return;
            }
            sendResponse(exceptionFrame(unit, functionCode, MBErrorCode::MBIllegalDataValue),
                         generation);
            return;
        }

        businessPending = true;
        auto claimed = std::make_shared<std::atomic<bool>>(false);
        auto hooksCopy = hooks;
        auto self = shared_from_this();
        auto deliver = [self, generation](RtuBytes output) mutable
        {
            asio::post(self->transport->strand(),
                       [self, generation, output = std::move(output)]() mutable
                       {
                           self->businessPending = false;
                           if (self->serviceState.load() == ServiceState::Idle ||
                               generation != self->serveGeneration)
                               return;
                           if (self->serviceState.load() == ServiceState::Stopping)
                           {
                               self->finish(self->stopError);
                               return;
                           }
                           if (output.empty())
                               self->beginRead(generation);
                           else
                               self->sendResponse(std::move(output), generation);
                       });
        };

        auto deliverForWork = deliver;
        auto work = [self, request = std::move(request), claimed, hooksCopy,
                     deliver = std::move(deliverForWork)]() mutable
        {
            if (claimed->exchange(true))
                return;
            RtuBytes output;
            if (self->serviceState.load() != ServiceState::Stopping)
            {
                try
                {
                    auto response = handleRequest(request, *self->mapping, hooksCopy);
                    if (request.unitId() != 0)
                    {
                        if (request.functionCode() == MBfunctionCode::MBReportSlaveID &&
                            response.registerValues().empty())
                        {
                            output = detail::withCrc(
                                {request.unitId(), 0x11, 2, self->options.unitId, 0xFF});
                        }
                        else
                        {
                            output = detail::withCrc(response.toRaw());
                        }
                    }
                }
                catch (const ModbusException& exception)
                {
                    if (request.unitId() != 0)
                    {
                        auto code = static_cast<std::uint8_t>(exception.getErrorCode());
                        if (!detail::validExceptionCode(code))
                            code = static_cast<std::uint8_t>(MBErrorCode::MBSlaveDeviceFailure);
                        output = exceptionFrame(request.unitId(),
                                                static_cast<std::uint8_t>(request.functionCode()),
                                                static_cast<MBErrorCode>(code));
                    }
                }
                catch (...)
                {
                    if (request.unitId() != 0)
                    {
                        output = exceptionFrame(request.unitId(),
                                                static_cast<std::uint8_t>(request.functionCode()),
                                                MBErrorCode::MBSlaveDeviceFailure);
                    }
                }
            }
            auto deliverCopy = deliver;
            deliverCopy(std::move(output));
        };

        if (!hooksCopy.dispatch)
        {
            work();
            return;
        }

        try
        {
            hooksCopy.dispatch(std::move(work));
        }
        catch (...)
        {
            if (!claimed->exchange(true))
            {
                RtuBytes output;
                if (unit != 0)
                    output = exceptionFrame(unit, functionCode, MBErrorCode::MBSlaveDeviceFailure);
                deliver(std::move(output));
            }
        }
    }

    void sendResponse(RtuBytes output, std::uint64_t generation)
    {
        auto self = shared_from_this();
        transport->asyncWrite(std::move(output), false,
                              [self, generation](const std::error_code& error, std::size_t)
                              {
                                  if (self->serviceState.load() == ServiceState::Idle ||
                                      generation != self->serveGeneration)
                                      return;
                                  if (self->serviceState.load() == ServiceState::Stopping)
                                  {
                                      self->finish(self->stopError);
                                      return;
                                  }
                                  if (error)
                                  {
                                      self->finish(error);
                                      return;
                                  }
                                  self->beginRead(generation);
                              });
    }

    void finish(std::error_code error)
    {
        if (serviceState.load() == ServiceState::Idle)
            return;
        phase.cancel();
        serviceState.store(ServiceState::Idle);
        businessPending = false;
        stopError.clear();
        workGuard.reset();
        notify(std::move(handler), error);
        if (closeInProgress)
            beginTransportClose();
    }

    void requestStop(const std::error_code& error)
    {
        if (serviceState.load() == ServiceState::Idle)
            return;
        serviceState.store(ServiceState::Stopping);
        stopError = error;
        transport->cancel();
        if (businessPending)
            return;
        if (transport->operationActive())
            return;
        finish(error);
    }

    void beginTransportClose()
    {
        if (!closeInProgress || transportClosing || serviceState.load() != ServiceState::Idle)
            return;
        transportClosing = true;
        auto self = shared_from_this();
        const auto phase =
            transportClosePhase.value_or(detail::SerialTransport::ClosePhase::Waiting);
        transport->continueClose(phase,
                                 [self](const std::error_code& error)
                                 {
                                     self->transportClosing = false;
                                     self->closeInProgress = false;
                                     self->transportClosePhase.reset();
                                     auto waiters = std::move(self->closeWaiters);
                                     self->closeWaiters.clear();
                                     for (auto& waiter : waiters)
                                         self->notify(std::move(waiter), error);
                                 });
    }

    void requestClose(detail::SerialTransport::ClosePhase closePhase,
                      RtuCompletionHandler closeHandler)
    {
        if (closeHandler)
            closeWaiters.push_back(std::move(closeHandler));
        if (closeInProgress)
            return;
        closeInProgress = true;
        transportClosePhase = closePhase;
        requestStop(asio::error::operation_aborted);
        if (serviceState.load() == ServiceState::Idle)
            beginTransportClose();
    }

    void shutdown()
    {
        alive.store(false);
        ++serviceEpoch;
        const auto closePhase = transport->beginClose();
        auto self = shared_from_this();
        asio::dispatch(transport->strand(),
                       [self, closePhase] { self->requestClose(closePhase, {}); });
    }
};

RtuSlave::RtuSlave(asio::io_context& io, std::string device, std::shared_ptr<Mapping> mapping,
                   SerialOptions serial, SlaveOptions slave)
    : impl_(std::make_shared<Impl>(io, std::move(device), std::move(mapping), serial, slave))
{
}

RtuSlave::~RtuSlave() { impl_->shutdown(); }

void RtuSlave::open() { impl_->transport->open(); }

void RtuSlave::close() noexcept
{
    try
    {
        asyncClose();
    }
    catch (...)
    {
    }
}

void RtuSlave::asyncClose(RtuCompletionHandler handler)
{
    ++impl_->serviceEpoch;
    const auto closePhase = impl_->transport->beginClose();
    auto impl = impl_;
    asio::dispatch(impl_->transport->strand(),
                   [impl, closePhase, handler = std::move(handler)]() mutable
                   { impl->requestClose(closePhase, std::move(handler)); });
}

void RtuSlave::cancel() noexcept
{
    ++impl_->serviceEpoch;
    auto impl = impl_;
    asio::dispatch(impl_->transport->strand(),
                   [impl] { impl->requestStop(asio::error::operation_aborted); });
}

bool RtuSlave::isOpen() const { return impl_->transport->isOpen(); }

SerialState RtuSlave::serialState() const { return impl_->transport->serialState(); }

bool RtuSlave::busy() const
{
    return impl_->serviceState.load() != ServiceState::Idle || impl_->transport->busy();
}

SerialOptions RtuSlave::serialOptions() const { return impl_->transport->options(); }

void RtuSlave::setSerialOptions(const SerialOptions& options)
{
    impl_->transport->setOptions(options);
}

void RtuSlave::asyncServe(RtuCompletionHandler handler)
{
    impl_->submitServe({}, std::move(handler));
}

void RtuSlave::asyncServe(MappingHooks hooks, RtuCompletionHandler handler)
{
    impl_->submitServe(std::move(hooks), std::move(handler));
}

void RtuSlave::stopServe() noexcept
{
    ++impl_->serviceEpoch;
    auto impl = impl_;
    asio::dispatch(impl_->transport->strand(),
                   [impl] { impl->requestStop(asio::error::operation_aborted); });
}

} // namespace issac_modbus
