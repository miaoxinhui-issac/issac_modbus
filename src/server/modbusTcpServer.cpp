#include "issac/modbus/server/modbusTcpServer.h"

#include <memory>
#include <utility>

namespace MB::TCP
{

/// @brief 创建、绑定并监听 Modbus TCP 服务端口。
Server::Server(asio::io_context& ioContext, std::uint16_t port, const asio::ip::address& address)
    : ioContext_(ioContext), acceptor_(ioContext)
{
    std::error_code error;
    const auto endpoint = asio::ip::tcp::endpoint(address, port);
    acceptor_.open(endpoint.protocol(), error);
    if (error)
    {
        throw std::system_error(error);
    }

    acceptor_.set_option(asio::socket_base::reuse_address(true), error);
    if (error)
    {
        throw std::system_error(error);
    }

    acceptor_.bind(endpoint, error);
    if (error)
    {
        throw std::system_error(error);
    }

    acceptor_.listen(asio::socket_base::max_listen_connections, error);
    if (error)
    {
        throw std::system_error(error);
    }
}

/// @brief 销毁服务端并关闭监听器。
Server::~Server() { close(); }

/// @brief 异步接受一个客户端并返回新建连接。
void Server::asyncAccept(AcceptHandler handler)
{
    if (!acceptor_.is_open())
    {
        asio::post(ioContext_, [handler = std::move(handler)]() mutable
                   { handler(asio::error::bad_descriptor, nullptr); });
        return;
    }

    auto socket = std::make_shared<Connection::Socket>(ioContext_);
    acceptor_.async_accept(
        *socket,
        [this, socket, handler = std::move(handler)](const ErrorCode& error) mutable
        {
            if (error)
            {
                handler(error, nullptr);
                return;
            }
            handler(ErrorCode{}, std::make_shared<Connection>(ioContext_, std::move(*socket)));
        });
}

/// @brief 在没有业务钩子时持续处理客户端连接。
void Server::asyncServe(Mapping& mapping, std::function<void(const ErrorCode&)> handler)
{
    asyncServe(mapping, {}, std::move(handler));
}

/// @brief 在业务钩子参与下持续接收请求、处理映射并发送响应。
void Server::asyncServe(Mapping& mapping, MappingHooks hooks,
                        std::function<void(const ErrorCode&)> handler)
{
    auto acceptNext = std::make_shared<std::function<void()>>();
    *acceptNext = [this, &mapping, hooks, handler, acceptNext]() mutable
    {
        asyncAccept(
            [this, &mapping, hooks, handler, acceptNext](const ErrorCode& acceptError,
                                                         ConnectionPtr connection) mutable
            {
                if (acceptError)
                {
                    if (handler)
                        handler(acceptError);
                    return;
                }

                auto receive = std::make_shared<std::function<void()>>();
                *receive = [connection, &mapping, hooks, receive, handler]() mutable
                {
                    connection->asyncAwaitRequest(
                        [connection, &mapping, hooks, receive,
                         handler](const ErrorCode& error, ModbusRequest request) mutable
                        {
                            if (error)
                            {
                                if (error == asio::error::timed_out && connection->isOpen())
                                {
                                    // A TCP slave may stay connected while the master is
                                    // idle.  The connection timeout is useful for clients,
                                    // but must not terminate a server receive loop.
                                    (*receive)();
                                    return;
                                }
                                if (handler)
                                    handler(error);
                                return;
                            }

                            auto process =
                                [connection, &mapping, hooks, receive, handler, request]() mutable
                            {
                                try
                                {
                                    if (request.slaveID() == 0)
                                    {
                                        MB::handleRequest(request, mapping, hooks);
                                        (*receive)();
                                        return;
                                    }
                                    ModbusResponse response =
                                        MB::handleRequest(request, mapping, hooks);
                                    connection->asyncSendResponse(
                                        response,
                                        [receive, handler](const ErrorCode& sendError,
                                                           std::size_t) mutable
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
                                    connection->asyncSendException(
                                        exception,
                                        [receive, handler](const ErrorCode& sendError,
                                                           std::size_t) mutable
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
                (*acceptNext)();
            });
    };
    (*acceptNext)();
}

/// @brief 取消并关闭 TCP 监听器。
void Server::close() noexcept
{
    ErrorCode ignored;
    acceptor_.cancel(ignored);
    acceptor_.close(ignored);
}

/// @brief 判断 TCP 监听器当前是否已经打开。
bool Server::isOpen() const noexcept { return acceptor_.is_open(); }

} // namespace MB::TCP
