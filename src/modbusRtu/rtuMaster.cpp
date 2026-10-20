#include "issac_modbus/modbusRtu/rtuMaster.h"

#include "detail/rtuCodec.h"
#include "detail/serialTransport.h"
#include "issac_modbus/util/modbusError.h"
#include "issac_modbus/util/modbusProtocol.h"

#include <algorithm>
#include <asio/error.hpp>
#include <atomic>
#include <chrono>
#include <deque>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace issac_modbus
{
namespace
{

using detail::badFrame;
using detail::mbError;

void validateMasterOptions(const MasterOptions& options)
{
    constexpr auto maxTime = std::chrono::hours(24);
    if (options.responseTimeout.count() <= 0 || options.responseTimeout > maxTime ||
        options.transactionTimeout.count() <= 0 || options.transactionTimeout > maxTime ||
        options.broadcastDelay.count() < 0 || options.broadcastDelay > maxTime ||
        options.retryPolicy.maxAttempts == 0 || options.retryPolicy.retryDelay.count() < 0 ||
        options.retryPolicy.retryDelay > maxTime)
    {
        throw std::invalid_argument("Invalid RTU master options");
    }
}

} // namespace

struct RtuMaster::Impl final : std::enable_shared_from_this<RtuMaster::Impl>
{
    struct Transaction final
    {
        Transaction(asio::io_context& io, ModbusRequest value, RetryPolicy retryValue,
                    TransactionHandler done)
            : request(std::move(value)), policy(retryValue), handler(std::move(done)), phase(io),
              deadline(io)
        {
        }

        ModbusRequest request;
        RetryPolicy policy;
        TransactionHandler handler;
        asio::steady_timer phase;
        asio::steady_timer deadline;
        RtuBytes requestFrame;
        std::size_t attempts = 0;
        std::uint64_t submittedEpoch = 0;
        bool timedOut = false;
        bool canceled = false;
        ModbusResponse response;
        std::optional<ModbusException> exception;
    };

    Impl(asio::io_context& io, std::string device, SerialOptions serial, MasterOptions master)
        : transport(std::make_shared<detail::SerialTransport>(io, std::move(device), serial)),
          options(std::move(master))
    {
        validateMasterOptions(options);
    }

    std::shared_ptr<detail::SerialTransport> transport;
    const MasterOptions options;
    std::deque<std::shared_ptr<Transaction>> queue;
    std::shared_ptr<Transaction> active;
    std::atomic<bool> busyFlag{false};
    std::atomic<bool> alive{true};
    std::atomic<std::uint64_t> cancellationEpoch{0};

    template <typename Handler, typename... Args> void notify(Handler handler, Args... args)
    {
        if (!handler)
            return;
        auto self = shared_from_this();
        asio::post(transport->strand(), [self, handler = std::move(handler),
                                         values = std::make_tuple(std::move(args)...)]() mutable
                   { std::apply(handler, std::move(values)); });
    }

    void enqueue(const std::shared_ptr<Transaction>& transaction)
    {
        if (transaction->submittedEpoch != cancellationEpoch.load())
        {
            notify(std::move(transaction->handler), asio::error::operation_aborted,
                   ModbusResponse{}, std::optional<ModbusException>{});
            return;
        }
        if (!alive.load() || !transport->isOpen())
        {
            notify(std::move(transaction->handler), asio::error::bad_descriptor, ModbusResponse{},
                   std::optional<ModbusException>{});
            return;
        }
        if (queue.size() + (active ? 1u : 0u) >= transport->options().maxPendingOperations)
        {
            notify(std::move(transaction->handler), asio::error::no_buffer_space, ModbusResponse{},
                   std::optional<ModbusException>{});
            return;
        }
        queue.push_back(transaction);
        busyFlag.store(true);
        pump();
    }

    void pump()
    {
        if (active || queue.empty())
            return;
        active = std::move(queue.front());
        queue.pop_front();
        auto transaction = active;
        auto self = shared_from_this();
        transaction->deadline.expires_after(options.transactionTimeout);
        transaction->deadline.async_wait(asio::bind_executor(
            transport->strand(),
            [self, transaction](const std::error_code& error)
            {
                if (error || self->active != transaction || transaction->timedOut)
                    return;
                transaction->timedOut = true;
                if (self->transport->operationActive())
                    self->transport->cancel();
                else
                    self->finish(transaction, asio::error::timed_out);
            }));
        startAttempt(transaction);
    }

    void startAttempt(const std::shared_ptr<Transaction>& transaction)
    {
        ++transaction->attempts;
        auto self = shared_from_this();
        transport->asyncWrite(transaction->requestFrame, options.discardInputBeforeRequest,
                              [self, transaction](const std::error_code& error, std::size_t)
                              {
                                  if (self->active != transaction)
                                      return;
                                  if (transaction->canceled || !self->alive.load())
                                  {
                                      self->finish(transaction, asio::error::operation_aborted);
                                      return;
                                  }
                                  if (transaction->timedOut)
                                  {
                                      self->finish(transaction, asio::error::timed_out);
                                      return;
                                  }
                                  if (error)
                                  {
                                      self->attemptFailed(transaction, error);
                                      return;
                                  }
                                  if (transaction->request.unitId() == 0)
                                  {
                                      self->armPhase(transaction, self->options.broadcastDelay,
                                                     [self, transaction]
                                                     { self->finish(transaction, {}); });
                                      return;
                                  }
                                  self->readResponse(transaction);
                              });
    }

    void readResponse(const std::shared_ptr<Transaction>& transaction)
    {
        const auto serial = transport->options();
        auto self = shared_from_this();
        transport->asyncRead(
            RtuFrameType::Response, options.responseTimeout, serial.frameTimeout,
            serial.byteTimeout,
            [self, transaction](const std::error_code& readError, RtuBytes data)
            {
                if (self->active != transaction)
                    return;
                if (transaction->canceled || !self->alive.load())
                {
                    self->finish(transaction, asio::error::operation_aborted);
                    return;
                }
                if (transaction->timedOut)
                {
                    self->finish(transaction, asio::error::timed_out);
                    return;
                }
                if (readError)
                {
                    self->attemptFailed(transaction, readError);
                    return;
                }

                std::error_code error = detail::matchResponse(transaction->requestFrame, data);
                if (!error)
                {
                    try
                    {
                        if ((data[1] & 0x80u) != 0)
                        {
                            transaction->exception.emplace(static_cast<MBErrorCode>(data[2]),
                                                           transaction->request.unitId(),
                                                           transaction->request.functionCode());
                            error = mbError(transaction->exception->getErrorCode());
                        }
                        else
                        {
                            transaction->response = ModbusResponse::fromRawCRC(data);
                            normalizeResponseForRequest(transaction->request,
                                                        transaction->response);
                        }
                    }
                    catch (const std::exception&)
                    {
                        error = badFrame();
                    }
                }
                if (error)
                {
                    self->attemptFailed(transaction, error);
                    return;
                }
                self->finish(transaction, {});
            });
    }

    static bool retryable(const std::error_code& error, const RetryPolicy& policy)
    {
        const bool timeout =
            error == asio::error::timed_out || error == mbError(MBErrorCode::MBFrameTimeout);
        const bool corrupt = error == mbError(MBErrorCode::MBErrorCodeCRCError) ||
                             error == badFrame() ||
                             error == mbError(MBErrorCode::MBInvalidSlaveId) ||
                             error == mbError(MBErrorCode::MBFunctionMismatch);
        const bool aborted = error == asio::error::operation_aborted;
        const bool localError = error == asio::error::invalid_argument ||
                                error == asio::error::message_size ||
                                error == asio::error::no_buffer_space;
        const bool protocolException =
            error.category() == makeErrorCode(MBErrorCode::MBIllegalFunction).category();
        const bool connection =
            !timeout && !corrupt && !aborted && !localError && !protocolException;
        return (timeout && policy.retryOnTimeout) || (corrupt && policy.retryOnCrcError) ||
               (connection && policy.retryOnConnectionError);
    }

    void attemptFailed(const std::shared_ptr<Transaction>& transaction,
                       const std::error_code& error)
    {
        if (transaction->attempts < transaction->policy.maxAttempts &&
            retryable(error, transaction->policy))
        {
            const auto delay =
                std::max(transaction->policy.retryDelay, transport->options().recoveryDelay);
            armPhase(transaction, delay,
                     [self = shared_from_this(), transaction] { self->startAttempt(transaction); });
            return;
        }
        finish(transaction, error);
    }

    void armPhase(const std::shared_ptr<Transaction>& transaction, std::chrono::milliseconds delay,
                  std::function<void()> function)
    {
        transaction->phase.expires_after(delay);
        auto self = shared_from_this();
        transaction->phase.async_wait(
            asio::bind_executor(transport->strand(),
                                [self, transaction, function = std::move(function)](
                                    const std::error_code& error) mutable
                                {
                                    if (error || !self->alive.load() ||
                                        self->active != transaction || transaction->timedOut)
                                        return;
                                    function();
                                }));
    }

    void finish(const std::shared_ptr<Transaction>& transaction, std::error_code error)
    {
        if (active != transaction)
            return;
        transaction->phase.cancel();
        transaction->deadline.cancel();
        active.reset();
        notify(std::move(transaction->handler), error, std::move(transaction->response),
               std::move(transaction->exception));
        pump();
        if (!active && queue.empty())
            busyFlag.store(false);
    }

    void requestCancel()
    {
        auto pending = std::move(queue);
        queue.clear();
        for (auto& transaction : pending)
            notify(std::move(transaction->handler), asio::error::operation_aborted,
                   ModbusResponse{}, std::optional<ModbusException>{});

        if (active)
        {
            auto transaction = active;
            transaction->canceled = true;
            transaction->phase.cancel();
            transaction->deadline.cancel();
            if (!transport->operationActive())
                finish(transaction, asio::error::operation_aborted);
        }
        if (!active && queue.empty())
            busyFlag.store(false);
    }

    void shutdown()
    {
        alive.store(false);
        ++cancellationEpoch;
        auto self = shared_from_this();
        asio::dispatch(transport->strand(), [self] { self->requestCancel(); });
        transport->close();
    }

    void values(std::function<ModbusRequest()> build, ValuesHandler handler,
                std::optional<RetryPolicy> policy)
    {
        ModbusRequest request;
        try
        {
            request = build();
        }
        catch (const std::exception&)
        {
            notify(std::move(handler), asio::error::invalid_argument, std::vector<ModbusCell>{},
                   std::optional<ModbusException>{});
            return;
        }
        submit(
            request,
            [handler = std::move(handler)](const std::error_code& error, ModbusResponse response,
                                           std::optional<ModbusException> exception) mutable
            {
                if (handler)
                {
                    handler(error, error ? std::vector<ModbusCell>{} : response.registerValues(),
                            std::move(exception));
                }
            },
            policy);
    }

    void submit(const ModbusRequest& request, TransactionHandler handler,
                std::optional<RetryPolicy> policy)
    {
        RtuBytes frame;
        std::error_code error;
        const auto selectedPolicy = policy.value_or(options.retryPolicy);
        try
        {
            if (selectedPolicy.maxAttempts == 0 || selectedPolicy.retryDelay.count() < 0 ||
                selectedPolicy.retryDelay > std::chrono::hours(24))
            {
                throw std::invalid_argument("Invalid retry policy");
            }
            frame = detail::withCrc(request.toRaw());
            detail::checkRequest(request, frame);
        }
        catch (const std::exception&)
        {
            error = request.unitId() > 247 ? mbError(MBErrorCode::MBInvalidSlaveId)
                                           : asio::error::invalid_argument;
        }
        if (error)
        {
            notify(std::move(handler), error, ModbusResponse{}, std::optional<ModbusException>{});
            return;
        }

        auto transaction = std::make_shared<Transaction>(transport->io(), request, selectedPolicy,
                                                         std::move(handler));
        transaction->requestFrame = std::move(frame);
        transaction->submittedEpoch = cancellationEpoch.load();
        auto self = shared_from_this();
        asio::dispatch(transport->strand(), [self, transaction] { self->enqueue(transaction); });
    }
};

RtuMaster::RtuMaster(asio::io_context& io, std::string device, SerialOptions serial,
                     MasterOptions master)
    : impl_(std::make_shared<Impl>(io, std::move(device), serial, master))
{
}

RtuMaster::~RtuMaster() { impl_->shutdown(); }

void RtuMaster::open() { impl_->transport->open(); }

void RtuMaster::close() noexcept
{
    ++impl_->cancellationEpoch;
    auto impl = impl_;
    asio::dispatch(impl_->transport->strand(), [impl] { impl->requestCancel(); });
    impl_->transport->close();
}

void RtuMaster::cancel() noexcept
{
    ++impl_->cancellationEpoch;
    impl_->transport->cancel();
    auto impl = impl_;
    asio::dispatch(impl_->transport->strand(), [impl] { impl->requestCancel(); });
}

bool RtuMaster::isOpen() const { return impl_->transport->isOpen(); }

bool RtuMaster::busy() const { return impl_->busyFlag.load(); }

SerialOptions RtuMaster::serialOptions() const { return impl_->transport->options(); }

void RtuMaster::setSerialOptions(const SerialOptions& options)
{
    impl_->transport->setOptions(options);
}

void RtuMaster::asyncTransaction(const ModbusRequest& request, TransactionHandler handler,
                                 std::optional<RetryPolicy> policy)
{
    impl_->submit(request, std::move(handler), policy);
}

void RtuMaster::asyncReadCoils(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                               ValuesHandler handler, std::optional<RetryPolicy> policy)
{
    impl_->values([=] { return ModbusRequest(unit, MBfunctionCode::MBReadCoils, address, count); },
                  std::move(handler), policy);
}

void RtuMaster::asyncReadDiscreteInputs(std::uint8_t unit, std::uint16_t address,
                                        std::uint16_t count, ValuesHandler handler,
                                        std::optional<RetryPolicy> policy)
{
    impl_->values(
        [=] { return ModbusRequest(unit, MBfunctionCode::MBReadDiscreteInputs, address, count); },
        std::move(handler), policy);
}

void RtuMaster::asyncReadHoldingRegisters(std::uint8_t unit, std::uint16_t address,
                                          std::uint16_t count, ValuesHandler handler,
                                          std::optional<RetryPolicy> policy)
{
    impl_->values(
        [=] { return ModbusRequest(unit, MBfunctionCode::MBReadHoldingRegisters, address, count); },
        std::move(handler), policy);
}

void RtuMaster::asyncReadInputRegisters(std::uint8_t unit, std::uint16_t address,
                                        std::uint16_t count, ValuesHandler handler,
                                        std::optional<RetryPolicy> policy)
{
    impl_->values(
        [=] { return ModbusRequest(unit, MBfunctionCode::MBReadInputRegisters, address, count); },
        std::move(handler), policy);
}

void RtuMaster::asyncWriteSingleCoil(std::uint8_t unit, std::uint16_t address, bool value,
                                     ValuesHandler handler, std::optional<RetryPolicy> policy)
{
    impl_->values(
        [=] {
            return ModbusRequest(unit, MBfunctionCode::MBWriteSingleCoil, address, 1,
                                 {ModbusCell(value)});
        },
        std::move(handler), policy);
}

void RtuMaster::asyncWriteSingleRegister(std::uint8_t unit, std::uint16_t address,
                                         std::uint16_t value, ValuesHandler handler,
                                         std::optional<RetryPolicy> policy)
{
    impl_->values(
        [=]
        {
            return ModbusRequest(unit, MBfunctionCode::MBWriteSingleRegister, address, 1,
                                 {ModbusCell(value)});
        },
        std::move(handler), policy);
}

void RtuMaster::asyncWriteMultipleRegisters(std::uint8_t unit, std::uint16_t address,
                                            const std::vector<std::uint16_t>& values,
                                            ValuesHandler handler,
                                            std::optional<RetryPolicy> policy)
{
    impl_->values(
        [=]
        {
            if (values.empty() || values.size() > 123)
                throw std::invalid_argument("Register count must be 1..123");
            std::vector<ModbusCell> cells;
            cells.reserve(values.size());
            for (auto value : values)
                cells.emplace_back(value);
            return ModbusRequest(unit, MBfunctionCode::MBWriteMultipleRegisters, address,
                                 static_cast<std::uint16_t>(cells.size()), cells);
        },
        std::move(handler), policy);
}

void RtuMaster::asyncWriteMultipleCoils(std::uint8_t unit, std::uint16_t address,
                                        const std::vector<bool>& values, ValuesHandler handler,
                                        std::optional<RetryPolicy> policy)
{
    impl_->values(
        [=]
        {
            if (values.empty() || values.size() > 1968)
                throw std::invalid_argument("Coil count must be 1..1968");
            std::vector<ModbusCell> cells;
            cells.reserve(values.size());
            for (bool value : values)
                cells.emplace_back(value);
            return ModbusRequest(unit, MBfunctionCode::MBWriteMultipleCoils, address,
                                 static_cast<std::uint16_t>(cells.size()), cells);
        },
        std::move(handler), policy);
}

void RtuMaster::asyncMaskWriteRegister(std::uint8_t unit, std::uint16_t address,
                                       std::uint16_t andMask, std::uint16_t orMask,
                                       ValuesHandler handler, std::optional<RetryPolicy> policy)
{
    impl_->values(
        [=]
        {
            return ModbusRequest(unit, MBfunctionCode::MBMaskWriteRegister, address, 2,
                                 {ModbusCell(andMask), ModbusCell(orMask)});
        },
        std::move(handler), policy);
}

void RtuMaster::asyncReadWriteMultipleRegisters(std::uint8_t unit, std::uint16_t readAddress,
                                                std::uint16_t readCount, std::uint16_t writeAddress,
                                                const std::vector<std::uint16_t>& values,
                                                ValuesHandler handler,
                                                std::optional<RetryPolicy> policy)
{
    impl_->values(
        [=]
        {
            if (values.empty() || values.size() > 121)
                throw std::invalid_argument("Write count must be 1..121");
            std::vector<ModbusCell> cells;
            cells.reserve(values.size());
            for (auto value : values)
                cells.emplace_back(value);
            return ModbusRequest(unit, MBfunctionCode::MBReadWriteMultipleRegisters, readAddress,
                                 readCount, cells, writeAddress);
        },
        std::move(handler), policy);
}

void RtuMaster::asyncReportSlaveId(std::uint8_t unit, ValuesHandler handler,
                                   std::optional<RetryPolicy> policy)
{
    impl_->values([=] { return ModbusRequest(unit, MBfunctionCode::MBReportSlaveID); },
                  std::move(handler), policy);
}

} // namespace issac_modbus
