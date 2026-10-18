#include "issac_modbus/modbusTcp/modbusTcpConnection.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "issac_modbus/util/modbusError.h"
#include "issac_modbus/util/modbusProtocol.h"

namespace
{

// 在当前项目的数据模型中，单元标识是 PDU 的第一个字节。
// 它前面的三个 MBAP 字段在网络上共占六个字节。
constexpr std::size_t MbapHeaderSize = 6;
constexpr std::uint16_t ModbusProtocolId = 0;
constexpr std::uint16_t MinimumMbapLength = 2; // 单元标识 + 功能码
constexpr std::uint16_t MaximumMbapLength = 253;

/// @brief 按网络字节序向缓冲区追加 16 位整数。
void appendBigEndian16(std::vector<std::uint8_t>& data, std::uint16_t value)
{
    data.push_back(static_cast<std::uint8_t>(value >> 8u));
    data.push_back(static_cast<std::uint8_t>(value & 0xFFu));
}

/// @brief 从网络字节序缓冲区读取 16 位整数。
std::uint16_t readBigEndian16(const std::uint8_t* data)
{
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[0]) << 8u) | data[1]);
}

} // namespace

namespace issac_modbus
{

struct TcpConnection::ReadState
{
    std::array<std::uint8_t, MbapHeaderSize> header{};
    Bytes frame;
    RawHandler handler;
    std::shared_ptr<asio::steady_timer> timer;
    bool completed = false;
};

/// @brief 创建尚未连接的 TCP 客户端连接。
TcpConnection::TcpConnection(asio::io_context& ioContext, std::string host, std::uint16_t port)
    : ioContext_(ioContext), socket_(ioContext), host_(std::move(host)), port_(port)
{
}

/// @brief 使用已建立的 TCP socket 创建连接。
TcpConnection::TcpConnection(asio::io_context& ioContext, Socket socket)
    : ioContext_(ioContext), socket_(std::move(socket))
{
}

/// @brief 销毁连接并关闭底层 socket。
TcpConnection::~TcpConnection() { close(); }

/// @brief 异步解析主机地址并建立 TCP 连接。
void TcpConnection::asyncConnect(ConnectHandler handler)
{
    if (socket_.is_open())
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::already_connected); });
        return;
    }

    auto resolver = std::make_shared<asio::ip::tcp::resolver>(ioContext_);
    auto timer = std::make_shared<asio::steady_timer>(ioContext_);
    timer->expires_after(connectTimeout_);
    timer->async_wait(
        [this, timer, resolver](const ErrorCode& error)
        {
            if (error)
                return;
            ErrorCode ignored;
            resolver->cancel();
            socket_.cancel(ignored);
        });
    resolver->async_resolve(
        host_, std::to_string(port_),
        [this, resolver, timer, handler = std::move(handler)](
            const ErrorCode& error, asio::ip::tcp::resolver::results_type endpoints) mutable
        {
            if (error)
            {
                timer->cancel();
                handler(error);
                return;
            }

            asio::async_connect(socket_, endpoints,
                                [timer, handler = std::move(handler)](const ErrorCode& connectError,
                                                                      const auto&) mutable
                                {
                                    timer->cancel();
                                    handler(connectError);
                                });
        });
}

/// @brief 取消异步操作、关闭 socket 并终止所有排队写入。
void TcpConnection::close() noexcept
{
    ErrorCode ignored;
    socket_.cancel(ignored);
    socket_.shutdown(Socket::shutdown_both, ignored);
    socket_.close(ignored);
    while (!writeQueue_.empty())
    {
        auto pending = std::move(writeQueue_.front());
        writeQueue_.pop_front();
        pending.handler(asio::error::operation_aborted, 0);
    }
    writeInProgress_ = false;
}

/// @brief 判断底层 TCP socket 是否已经打开。
bool TcpConnection::isOpen() const noexcept { return socket_.is_open(); }

/// @brief 取消当前 socket 上等待中的异步操作。
void TcpConnection::clearInput() noexcept
{
    ErrorCode ignored;
    socket_.cancel(ignored);
}

/// @brief 在 PDU 前添加 MBAP 头生成完整 TCP 帧。
TcpConnection::Bytes TcpConnection::makeFrame(const Bytes& pdu, std::uint16_t transactionId) const
{
    if (pdu.size() < MinimumMbapLength || pdu.size() > MaximumMbapLength)
    {
        throw std::invalid_argument("Invalid Modbus TCP PDU length.");
    }

    Bytes frame;
    frame.reserve(MbapHeaderSize + pdu.size());
    appendBigEndian16(frame, transactionId);
    appendBigEndian16(frame, ModbusProtocolId);
    appendBigEndian16(frame, static_cast<std::uint16_t>(pdu.size()));
    frame.insert(frame.end(), pdu.begin(), pdu.end());
    return frame;
}

/// @brief 校验 MBAP 头并提取单元标识及 PDU 数据。
TcpConnection::Bytes TcpConnection::extractPdu(const Bytes& frame)
{
    if (frame.size() < MbapHeaderSize)
    {
        throw std::invalid_argument("Modbus TCP frame is too short.");
    }

    const std::uint16_t protocolId = readBigEndian16(frame.data() + 2);
    const std::uint16_t length = readBigEndian16(frame.data() + 4);
    if (protocolId != ModbusProtocolId || length < MinimumMbapLength ||
        length > MaximumMbapLength || frame.size() != MbapHeaderSize + length)
    {
        throw std::invalid_argument("Invalid Modbus TCP MBAP header.");
    }

    return Bytes(frame.begin() + MbapHeaderSize, frame.end());
}

/// @brief 异步发送已经包含 MBAP 头的完整 TCP 帧。
void TcpConnection::asyncSend(Bytes data, WriteHandler handler)
{
    if (!isOpen())
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::bad_descriptor, 0); });
        return;
    }

    if (data.size() < MbapHeaderSize)
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::invalid_argument, 0); });
        return;
    }

    const std::uint16_t length = readBigEndian16(data.data() + 4);
    if (readBigEndian16(data.data() + 2) != ModbusProtocolId || length < MinimumMbapLength ||
        length > MaximumMbapLength || data.size() != MbapHeaderSize + length)
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::invalid_argument, 0); });
        return;
    }

    writeQueue_.push_back({std::move(data), std::move(handler)});
    startNextWrite();
}

/// @brief 依次发送写队列中的下一个缓冲区。
void TcpConnection::startNextWrite()
{
    if (writeInProgress_ || writeQueue_.empty() || !isOpen())
        return;
    writeInProgress_ = true;
    auto frame = std::make_shared<Bytes>(std::move(writeQueue_.front().data));
    auto handler = std::move(writeQueue_.front().handler);
    writeQueue_.pop_front();
    asio::async_write(socket_, asio::buffer(*frame),
                      [this, frame, handler = std::move(handler)](const ErrorCode& error,
                                                                  std::size_t bytes) mutable
                      {
                          writeInProgress_ = false;
                          handler(error, bytes);
                          startNextWrite();
                      });
}

/// @brief 为 PDU 添加 MBAP 头并异步发送。
void TcpConnection::asyncSendPdu(Bytes pdu, WriteHandler handler)
{
    if (!isOpen())
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::bad_descriptor, 0); });
        return;
    }

    Bytes frame;
    try
    {
        const std::uint16_t transactionId = messageId_++;
        frame = makeFrame(pdu, transactionId);
    }
    catch (...)
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::invalid_argument, 0); });
        return;
    }

    asyncSend(std::move(frame), std::move(handler));
}

/// @brief 序列化 Modbus 请求并异步发送。
void TcpConnection::asyncSendRequest(const ModbusRequest& request, WriteHandler handler)
{
    try
    {
        asyncSendPdu(request.toRaw(), std::move(handler));
    }
    catch (...)
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::invalid_argument, 0); });
    }
}

/// @brief 序列化 Modbus 响应并沿用最近一次事务 ID 发送。
void TcpConnection::asyncSendResponse(const ModbusResponse& response, WriteHandler handler)
{
    Bytes frame;
    try
    {
        frame = makeFrame(response.toRaw(), lastReceivedMessageId_);
    }
    catch (...)
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::invalid_argument, 0); });
        return;
    }

    asyncSend(std::move(frame), std::move(handler));
}

/// @brief 序列化 Modbus 异常响应并沿用最近一次事务 ID 发送。
void TcpConnection::asyncSendException(const ModbusException& exception, WriteHandler handler)
{
    Bytes frame;
    try
    {
        frame = makeFrame(exception.toRaw(), lastReceivedMessageId_);
    }
    catch (...)
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::invalid_argument, 0); });
        return;
    }

    asyncSend(std::move(frame), std::move(handler));
}

/// @brief 异步等待一个完整 TCP 帧。
void TcpConnection::asyncAwaitRawMessage(RawHandler handler) { asyncAwaitFrame(std::move(handler)); }

/// @brief 异步接收并解析一个 Modbus 请求。
void TcpConnection::asyncAwaitRequest(RequestHandler handler)
{
    asyncAwaitFrame(
        [handler = std::move(handler)](const ErrorCode& error, Bytes frame) mutable
        {
            if (error)
            {
                handler(error, ModbusRequest{});
                return;
            }

            try
            {
                handler(ErrorCode{}, ModbusRequest::fromRaw(TcpConnection::extractPdu(frame)));
            }
            catch (...)
            {
                handler(asio::error::invalid_argument, ModbusRequest{});
            }
        });
}

/// @brief 异步接收并解析一个 Modbus 响应。
void TcpConnection::asyncAwaitResponse(ResponseHandler handler)
{
    asyncAwaitFrame(
        [handler = std::move(handler)](const ErrorCode& error, Bytes frame) mutable
        {
            if (error)
            {
                handler(error, ModbusResponse{});
                return;
            }

            try
            {
                const Bytes pdu = TcpConnection::extractPdu(frame);
                if (ModbusException::exist(pdu))
                {
                    handler(asio::error::invalid_argument, ModbusResponse{});
                    return;
                }
                handler(ErrorCode{}, ModbusResponse::fromRaw(pdu));
            }
            catch (...)
            {
                handler(asio::error::invalid_argument, ModbusResponse{});
            }
        });
}

/// @brief 发送请求并处理响应、超时、异常和重试。
void TcpConnection::asyncTransaction(const ModbusRequest& request, TransactionHandler handler,
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
        issac_modbus::validateUnitId(request.unitId());
        issac_modbus::validateRequestLimits(request);
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
        const std::uint16_t expectedTransaction = messageId_;
        try
        {
            asyncSendPdu(
                requestState->toRaw(),
                [this, requestState, attempt, policy, handler, run,
                 expectedTransaction](const ErrorCode& writeError, std::size_t) mutable
                {
                    if (writeError)
                    {
                        if (*attempt < std::max<std::size_t>(1, policy.maxAttempts) &&
                            policy.retryOnConnectionError)
                        {
                            auto timer = std::make_shared<asio::steady_timer>(ioContext_);
                            timer->expires_after(policy.retryDelay);
                            timer->async_wait(
                                [run, timer](const ErrorCode& error)
                                {
                                    if (!error)
                                        (*run)();
                                });
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
                        [this, requestState, attempt, policy, handler, run,
                         expectedTransaction](const ErrorCode& readError, Bytes frame) mutable
                        {
                            ErrorCode result = readError;
                            std::optional<ModbusException> exception;
                            ModbusResponse response;
                            try
                            {
                                if (!result)
                                {
                                    const std::uint16_t actualTransaction =
                                        readBigEndian16(frame.data());
                                    if (actualTransaction != expectedTransaction)
                                    {
                                        result =
                                            issac_modbus::makeErrorCode(MBErrorCode::MBTransactionMismatch);
                                    }
                                    else
                                    {
                                        const Bytes pdu = extractPdu(frame);
                                        if (ModbusException::exist(pdu))
                                        {
                                            exception.emplace(pdu, false);
                                            result = issac_modbus::makeErrorCode(exception->getErrorCode());
                                        }
                                        else
                                        {
                                            response = ModbusResponse::fromRaw(pdu);
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

                            const bool isTimeout = result == asio::error::timed_out;
                            const bool isTransactionMismatch =
                                result == issac_modbus::makeErrorCode(MBErrorCode::MBTransactionMismatch);
                            const bool isBadFrame =
                                result == issac_modbus::makeErrorCode(MBErrorCode::MBBadFrame);
                            const bool retryable = (isTimeout && policy.retryOnTimeout) ||
                                                   isTransactionMismatch ||
                                                   (isBadFrame && policy.retryOnConnectionError);
                            if (result && retryable &&
                                *attempt < std::max<std::size_t>(1, policy.maxAttempts))
                            {
                                auto timer = std::make_shared<asio::steady_timer>(ioContext_);
                                timer->expires_after(policy.retryDelay);
                                timer->async_wait(
                                    [run, timer](const ErrorCode& error)
                                    {
                                        if (!error)
                                            (*run)();
                                    });
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

/// @brief 构造并执行读取保持寄存器事务。
void TcpConnection::asyncReadHoldingRegisters(std::uint8_t unit, std::uint16_t address,
                                           std::uint16_t count, ValuesHandler handler,
                                           RetryPolicy policy)
{
    ModbusRequest request(unit, MBfunctionCode::MBReadHoldingRegisters, address, count);
    asyncTransaction(
        request,
        [handler = std::move(handler)](const ErrorCode& error, ModbusResponse response,
                                       std::optional<ModbusException> exception) mutable
        { handler(error, response.registerValues(), std::move(exception)); },
        policy);
}

/// @brief 构造并执行写单个保持寄存器事务。
void TcpConnection::asyncWriteSingleRegister(std::uint8_t unit, std::uint16_t address,
                                          std::uint16_t value, ValuesHandler handler,
                                          RetryPolicy policy)
{
    ModbusRequest request(unit, MBfunctionCode::MBWriteSingleRegister, address, 1,
                          {ModbusCell(value)});
    asyncTransaction(
        request,
        [handler = std::move(handler)](const ErrorCode& error, ModbusResponse response,
                                       std::optional<ModbusException> exception) mutable
        { handler(error, response.registerValues(), std::move(exception)); },
        policy);
}

namespace
{
template <typename Build>
/// @brief 使用请求构造器执行一次 TCP 值操作。
void tcpValues(TcpConnection& connection, Build build, TcpConnection::ValuesHandler handler,
               RetryPolicy policy)
{
    ModbusRequest request = build();
    connection.asyncTransaction(
        request,
        [handler = std::move(handler)](const TcpConnection::ErrorCode& error, ModbusResponse response,
                                       std::optional<ModbusException> exception) mutable
        { handler(error, response.registerValues(), std::move(exception)); },
        policy);
}
} // namespace

/// @brief 构造并执行读取线圈事务。
void TcpConnection::asyncReadCoils(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                ValuesHandler handler, RetryPolicy policy)
{
    tcpValues(
        *this, [=] { return ModbusRequest(unit, MBfunctionCode::MBReadCoils, address, count); },
        std::move(handler), policy);
}

/// @brief 构造并执行读取离散输入事务。
void TcpConnection::asyncReadDiscreteInputs(std::uint8_t unit, std::uint16_t address,
                                         std::uint16_t count, ValuesHandler handler,
                                         RetryPolicy policy)
{
    tcpValues(
        *this,
        [=] { return ModbusRequest(unit, MBfunctionCode::MBReadDiscreteInputs, address, count); },
        std::move(handler), policy);
}

/// @brief 构造并执行读取输入寄存器事务。
void TcpConnection::asyncReadInputRegisters(std::uint8_t unit, std::uint16_t address,
                                         std::uint16_t count, ValuesHandler handler,
                                         RetryPolicy policy)
{
    tcpValues(
        *this,
        [=] { return ModbusRequest(unit, MBfunctionCode::MBReadInputRegisters, address, count); },
        std::move(handler), policy);
}

/// @brief 构造并执行写单个线圈事务。
void TcpConnection::asyncWriteSingleCoil(std::uint8_t unit, std::uint16_t address, bool value,
                                      ValuesHandler handler, RetryPolicy policy)
{
    tcpValues(
        *this,
        [=] {
            return ModbusRequest(unit, MBfunctionCode::MBWriteSingleCoil, address, 1,
                                 {ModbusCell(value)});
        },
        std::move(handler), policy);
}

/// @brief 构造并执行写多个保持寄存器事务。
void TcpConnection::asyncWriteMultipleRegisters(std::uint8_t unit, std::uint16_t address,
                                             const std::vector<std::uint16_t>& values,
                                             ValuesHandler handler, RetryPolicy policy)
{
    std::vector<ModbusCell> cells;
    for (auto value : values)
        cells.emplace_back(value);
    tcpValues(
        *this,
        [=, cells = std::move(cells)]() mutable
        {
            return ModbusRequest(unit, MBfunctionCode::MBWriteMultipleRegisters, address,
                                 static_cast<std::uint16_t>(cells.size()), cells);
        },
        std::move(handler), policy);
}

/// @brief 构造并执行写多个线圈事务。
void TcpConnection::asyncWriteMultipleCoils(std::uint8_t unit, std::uint16_t address,
                                         const std::vector<bool>& values, ValuesHandler handler,
                                         RetryPolicy policy)
{
    std::vector<ModbusCell> cells;
    for (bool value : values)
        cells.emplace_back(value);
    tcpValues(
        *this,
        [=, cells = std::move(cells)]() mutable
        {
            return ModbusRequest(unit, MBfunctionCode::MBWriteMultipleCoils, address,
                                 static_cast<std::uint16_t>(cells.size()), cells);
        },
        std::move(handler), policy);
}

/// @brief 构造并执行保持寄存器掩码写事务。
void TcpConnection::asyncMaskWriteRegister(std::uint8_t unit, std::uint16_t address,
                                        std::uint16_t andMask, std::uint16_t orMask,
                                        ValuesHandler handler, RetryPolicy policy)
{
    tcpValues(
        *this,
        [=]
        {
            return ModbusRequest(unit, MBfunctionCode::MBMaskWriteRegister, address, 2,
                                 {ModbusCell(andMask), ModbusCell(orMask)});
        },
        std::move(handler), policy);
}

/// @brief 构造并执行读写多个保持寄存器事务。
void TcpConnection::asyncReadWriteMultipleRegisters(std::uint8_t unit, std::uint16_t readAddress,
                                                 std::uint16_t readCount,
                                                 std::uint16_t writeAddress,
                                                 const std::vector<std::uint16_t>& values,
                                                 ValuesHandler handler, RetryPolicy policy)
{
    std::vector<ModbusCell> cells;
    for (auto value : values)
        cells.emplace_back(value);
    tcpValues(
        *this,
        [=, cells = std::move(cells)]() mutable
        {
            return ModbusRequest(unit, MBfunctionCode::MBReadWriteMultipleRegisters, readAddress,
                                 readCount, cells, writeAddress);
        },
        std::move(handler), policy);
}

/// @brief 构造并执行读取从站标识事务。
void TcpConnection::asyncReportSlaveId(std::uint8_t unit, ValuesHandler handler, RetryPolicy policy)
{
    tcpValues(
        *this, [=] { return ModbusRequest(unit, MBfunctionCode::MBReportSlaveID); },
        std::move(handler), policy);
}

/// @brief 设置底层 socket 的 KeepAlive 选项。
void TcpConnection::setKeepAlive(bool enabled)
{
    ErrorCode error;
    socket_.set_option(asio::socket_base::keep_alive(enabled), error);
}

/// @brief 设置底层 socket 的 Nagle 算法选项。
void TcpConnection::setNoDelay(bool enabled)
{
    ErrorCode error;
    socket_.set_option(asio::ip::tcp::no_delay(enabled), error);
}

/// @brief 先读取 MBAP 头，再按长度字段读取完整 TCP 帧。
void TcpConnection::asyncAwaitFrame(RawHandler handler)
{
    if (!isOpen())
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::bad_descriptor, {}); });
        return;
    }

    auto state = std::make_shared<ReadState>();
    state->handler = std::move(handler);
    state->timer = std::make_shared<asio::steady_timer>(ioContext_);
    state->timer->expires_after(timeout_);

    state->timer->async_wait(
        [this, state](const ErrorCode& error)
        {
            if (error || state->completed)
            {
                return;
            }

            state->completed = true;
            ErrorCode ignored;
            socket_.cancel(ignored);
            state->handler(asio::error::timed_out, std::move(state->frame));
        });

    asio::async_read(socket_, asio::buffer(state->header),
                     [this, state](const ErrorCode& error, std::size_t)
                     {
                         if (state->completed)
                         {
                             return;
                         }

                         if (error)
                         {
                             state->completed = true;
                             state->timer->cancel();
                             state->handler(error, std::move(state->frame));
                             return;
                         }

                         const std::uint16_t protocolId = readBigEndian16(state->header.data() + 2);
                         const std::uint16_t length = readBigEndian16(state->header.data() + 4);
                         if (protocolId != ModbusProtocolId || length < MinimumMbapLength ||
                             length > MaximumMbapLength)
                         {
                             state->completed = true;
                             state->timer->cancel();
                             state->handler(asio::error::invalid_argument, {});
                             return;
                         }

                         // A server must reply with the transaction identifier from the
                         // request it just received.  Keep it separate from messageId_,
                         // which is the next ID allocated by a client request.
                         lastReceivedMessageId_ = readBigEndian16(state->header.data());

                         state->frame.assign(state->header.begin(), state->header.end());
                         state->frame.resize(MbapHeaderSize + length);
                         asio::async_read(
                             socket_, asio::buffer(state->frame.data() + MbapHeaderSize, length),
                             [this, state](const ErrorCode& bodyError, std::size_t)
                             {
                                 if (state->completed)
                                 {
                                     return;
                                 }

                                 state->completed = true;
                                 state->timer->cancel();
                                 if (bodyError)
                                 {
                                     state->handler(bodyError, std::move(state->frame));
                                     return;
                                 }

                                 lastReceivedMessageId_ = readBigEndian16(state->frame.data());
                                 state->handler(ErrorCode{}, std::move(state->frame));
                             });
                     });
}

} // namespace issac_modbus
