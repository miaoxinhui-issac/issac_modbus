#ifndef MODBUS_TCP_CONNECTION_H
#define MODBUS_TCP_CONNECTION_H

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <system_error>
#include <vector>

#include <asio.hpp>

#include "issac_modbus/util/modbusException.h"
#include "issac_modbus/util/modbusRequest.h"
#include "issac_modbus/util/modbusResponse.h"
#include "issac_modbus/util/modbusTransaction.h"

namespace issac_modbus
{

/// @brief 基于 standalone Asio 的 Modbus TCP 连接。
///
/// ModbusRequest/ModbusResponse 负责序列化包含单元地址的 Modbus PDU，
/// 本类负责在 PDU 外层添加和解析 Modbus TCP 的事务、协议和长度字段。
class TcpConnection
{
  public:
    using Bytes = std::vector<std::uint8_t>;
    using ErrorCode = std::error_code;
    using WriteHandler = std::function<void(const ErrorCode&, std::size_t)>;
    using RawHandler = std::function<void(const ErrorCode&, Bytes)>;
    using RequestHandler = std::function<void(const ErrorCode&, ModbusRequest)>;
    using ResponseHandler = std::function<void(const ErrorCode&, ModbusResponse)>;
    using ConnectHandler = std::function<void(const ErrorCode&)>;
    using ValuesHandler = std::function<void(const ErrorCode&, std::vector<ModbusCell>,
                                             std::optional<ModbusException>)>;

    using Socket = asio::ip::tcp::socket;

    static constexpr std::chrono::milliseconds DefaultTcpTimeout{500};
    static constexpr std::chrono::milliseconds DefaultConnectTimeout{5000};

    /// @brief 创建一个尚未连接的客户端连接。
    explicit TcpConnection(asio::io_context& ioContext, std::string host, std::uint16_t port);

    /// @brief 使用已经接受的 TCP socket 创建连接。
    explicit TcpConnection(asio::io_context& ioContext, Socket socket);

    /// @brief 关闭连接并释放资源。
    ~TcpConnection();

    /// @brief 禁止复制构造连接。
    TcpConnection(const TcpConnection&) = delete;

    /// @brief 禁止复制赋值连接。
    TcpConnection& operator=(const TcpConnection&) = delete;

    /// @brief 使用编译器生成的移动构造。
    TcpConnection(TcpConnection&&) = default;

    /// @brief 使用编译器生成的移动赋值。
    TcpConnection& operator=(TcpConnection&&) = default;

    /// @brief 异步连接到客户端地址。
    void asyncConnect(ConnectHandler handler);

    /// @brief 设置连接超时时间。
    void setConnectTimeout(std::chrono::milliseconds timeout) noexcept
    {
        connectTimeout_ = timeout;
    }
    /// @brief 获取连接超时时间。
    [[nodiscard]] std::chrono::milliseconds connectTimeout() const noexcept
    {
        return connectTimeout_;
    }

    /// @brief 关闭 TCP 连接并取消所有未完成的异步操作。
    void close() noexcept;

    /// @brief 判断 TCP socket 是否已经连接。
    [[nodiscard]] bool isOpen() const noexcept;

    /// @brief 取消当前异步读写。
    void clearInput() noexcept;

    /// @brief 异步发送完整的 Modbus TCP ADU（包含 MBAP 头）。
    void asyncSend(Bytes data, WriteHandler handler);

    /// @brief 异步发送一个不含 MBAP 头的 Modbus PDU。
    void asyncSendPdu(Bytes pdu, WriteHandler handler);

    /// @brief 异步发送 Modbus 请求。
    void asyncSendRequest(const ModbusRequest& request, WriteHandler handler);

    /// @brief 异步发送 Modbus 响应，使用最近一次收到的事务 ID。
    void asyncSendResponse(const ModbusResponse& response, WriteHandler handler);

    /// @brief 异步发送 Modbus 异常响应，使用最近一次收到的事务 ID。
    void asyncSendException(const ModbusException& exception, WriteHandler handler);

    /// @brief 异步接收一个完整的 Modbus TCP ADU（包含 MBAP 头）。
    void asyncAwaitRawMessage(RawHandler handler);

    /// @brief 异步接收并解析一个 Modbus 请求。
    void asyncAwaitRequest(RequestHandler handler);

    /// @brief 异步接收并解析一个 Modbus 响应。
    void asyncAwaitResponse(ResponseHandler handler);

    /// @brief 执行一次带校验、超时和重试策略的异步事务。
    void asyncTransaction(const ModbusRequest& request, TransactionHandler handler,
                          RetryPolicy policy = {});

    /// @brief 异步读取保持寄存器。
    void asyncReadHoldingRegisters(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                   ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步写入单个保持寄存器。
    void asyncWriteSingleRegister(std::uint8_t unit, std::uint16_t address, std::uint16_t value,
                                  ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步读取线圈。
    void asyncReadCoils(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                        ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步读取离散输入。
    void asyncReadDiscreteInputs(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                 ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步读取输入寄存器。
    void asyncReadInputRegisters(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                 ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步写入单个线圈。
    void asyncWriteSingleCoil(std::uint8_t unit, std::uint16_t address, bool value,
                              ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步写入多个保持寄存器。
    void asyncWriteMultipleRegisters(std::uint8_t unit, std::uint16_t address,
                                     const std::vector<std::uint16_t>& values,
                                     ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步写入多个线圈。
    void asyncWriteMultipleCoils(std::uint8_t unit, std::uint16_t address,
                                 const std::vector<bool>& values, ValuesHandler handler,
                                 RetryPolicy policy = {});

    /// @brief 异步执行保持寄存器掩码写。
    void asyncMaskWriteRegister(std::uint8_t unit, std::uint16_t address, std::uint16_t andMask,
                                std::uint16_t orMask, ValuesHandler handler,
                                RetryPolicy policy = {});

    /// @brief 异步执行读写多个保持寄存器操作。
    void asyncReadWriteMultipleRegisters(std::uint8_t unit, std::uint16_t readAddress,
                                         std::uint16_t readCount, std::uint16_t writeAddress,
                                         const std::vector<std::uint16_t>& values,
                                         ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步读取从站标识信息。
    void asyncReportSlaveId(std::uint8_t unit, ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 设置 TCP 连接是否启用 KeepAlive。
    void setKeepAlive(bool enabled);

    /// @brief 设置 TCP 连接是否禁用 Nagle 算法。
    void setNoDelay(bool enabled);

    /// @brief 设置接收超时。
    void setTimeout(std::chrono::milliseconds timeout) noexcept { timeout_ = timeout; }

    /// @brief 获取接收超时。
    [[nodiscard]] std::chrono::milliseconds timeout() const noexcept { return timeout_; }

    /// @brief 获取最近一次发送或接收使用的事务 ID。
    [[nodiscard]] std::uint16_t messageId() const noexcept { return messageId_; }

    /// @brief 设置下一个客户端请求使用的事务 ID。
    void setMessageId(std::uint16_t messageId) noexcept { messageId_ = messageId; }

    /// @brief 获取底层 socket。
    [[nodiscard]] Socket& socket() noexcept { return socket_; }

  private:
    struct ReadState;
    struct PendingWrite
    {
        Bytes data;
        WriteHandler handler;
    };

    /// @brief 为 Modbus PDU 添加 MBAP 头并生成完整 TCP 帧。
    [[nodiscard]] Bytes makeFrame(const Bytes& pdu, std::uint16_t transactionId) const;

    /// @brief 从完整 TCP 帧中校验 MBAP 头并提取单元标识和 PDU。
    [[nodiscard]] static Bytes extractPdu(const Bytes& frame);

    /// @brief 异步等待一个完整 TCP 帧。
    void asyncAwaitFrame(RawHandler handler);

    /// @brief 启动写队列中的下一个发送任务。
    void startNextWrite();

    asio::io_context& ioContext_;
    Socket socket_;
    std::string host_;
    std::uint16_t port_ = 0;
    std::uint16_t messageId_ = 0;
    std::uint16_t lastReceivedMessageId_ = 0;
    std::chrono::milliseconds timeout_ = DefaultTcpTimeout;
    std::chrono::milliseconds connectTimeout_ = DefaultConnectTimeout;
    bool transactionInProgress_ = false;
    std::deque<PendingWrite> writeQueue_;
    bool writeInProgress_ = false;
};

} // namespace issac_modbus

#endif // MODBUS_TCP_CONNECTION_H
