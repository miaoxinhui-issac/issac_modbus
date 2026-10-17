#include "demo_protocol.h"

#include <csignal>
#include <cstdlib>
#include <iostream>

int main(int argc, char* argv[])
{
    const auto port = static_cast<std::uint16_t>(argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 1502);

    try
    {
        asio::io_context io;
        issac::modbus::core::Mapping mapping =
            issac::modbus::demo::soft_protocol::makeMapping();
        issac::modbus::server::tcp::Server server(io, port);
        asio::signal_set signals(io, SIGINT, SIGTERM);
        signals.async_wait([&](const std::error_code&, int)
                           {
                               server.close();
                               io.stop();
                           });

        std::cout << "TCP slave listening on 0.0.0.0:" << port << ", unit "
                  << static_cast<unsigned int>(
                         issac::modbus::demo::soft_protocol::UnitId)
                  << std::endl;
        server.asyncServe(mapping, issac::modbus::demo::soft_protocol::makeHooks(),
                          [](const std::error_code& error)
                          {
                              if (error)
                                  std::cerr << "[slave] " << error.message() << std::endl;
                          });
        io.run();
    }
    catch (const std::exception& error)
    {
        std::cerr << "TCP slave: " << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
