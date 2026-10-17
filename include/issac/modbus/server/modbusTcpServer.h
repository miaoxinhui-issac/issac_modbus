#ifndef MODBUS_TCP_SERVER_H
#define MODBUS_TCP_SERVER_H

#include "issac/modbus/core/modbusMapping.h"
#include "issac/modbus/transport/modbusTcpConnection.h"

#include <asio.hpp>
#include <cstdint>
#include <functional>
#include <memory>

namespace MB::TCP
{

/// @brief Modbus TCP 服务端监听器。
class Server
{
  public:
    using ErrorCode = std::error_code;
    using ConnectionPtr = std::shared_ptr<Connection>;
    using AcceptHandler = std::function<void(const ErrorCode&, ConnectionPtr)>;

    /// @brief 创建并绑定 TCP 监听端口。
    Server(asio::io_context& ioContext, std::uint16_t port,
           const asio::ip::address& address = asio::ip::address_v4::any());

    /// @brief 关闭监听器并释放资源。
    ~Server();

    /// @brief 禁止复制构造监听器。
    Server(const Server&) = delete;

    /// @brief 禁止复制赋值监听器。
    Server& operator=(const Server&) = delete;

    /// @brief 禁止移动构造监听器。
    Server(Server&&) = delete;

    /// @brief 禁止移动赋值监听器。
    Server& operator=(Server&&) = delete;

    /// @brief 异步接受一个客户端连接。
    void asyncAccept(AcceptHandler handler);

    /// @brief 接受客户端连接，并持续使用 Mapping 处理请求。
    void asyncServe(Mapping& mapping, std::function<void(const ErrorCode&)> handler = {});
    /// @brief 使用业务钩子持续处理客户端连接。
    void asyncServe(Mapping& mapping, MappingHooks hooks,
                    std::function<void(const ErrorCode&)> handler = {});

    /// @brief 停止监听并取消等待中的 accept。
    void close() noexcept;

    /// @brief 判断监听器是否处于打开状态。
    [[nodiscard]] bool isOpen() const noexcept;

    /// @brief 获取底层 Asio 监听器。
    [[nodiscard]] asio::ip::tcp::acceptor& acceptor() noexcept { return acceptor_; }

  private:
    asio::io_context& ioContext_;
    asio::ip::tcp::acceptor acceptor_;
};

} // namespace MB::TCP

#endif // MODBUS_TCP_SERVER_H
