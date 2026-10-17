#include "demo_protocol.h"

#include <csignal>
#include <cstdlib>
#include <iostream>

int main(int argc, char* argv[])
{
    const std::string device = argc > 1 ? argv[1] : "/dev/ttyS2";
    const auto baud =
        static_cast<unsigned int>(argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 9600);
    const bool useRs485 = argc > 3 && std::string(argv[3]) == "rs485";

    try
    {
        asio::io_context io;
        issac::modbus::core::Mapping mapping = issac::modbus::demo::soft_protocol::makeMapping();
        issac::modbus::transport::rtu::Connection connection(io, device, baud);
        connection.setSerialMode(
            useRs485 ? issac::modbus::transport::rtu::Connection::SerialMode::RS485
                     : issac::modbus::transport::rtu::Connection::SerialMode::RS232);
        connection.open();
        asio::signal_set signals(io, SIGINT, SIGTERM);
        signals.async_wait(
            [&](const std::error_code&, int)
            {
                connection.close();
                io.stop();
            });

        std::cout << "RTU slave listening on " << device << " at " << baud
                  << " baud, mode " << (useRs485 ? "RS485" : "RS232") << ", unit "
                  << static_cast<unsigned int>(issac::modbus::demo::soft_protocol::UnitId)
                  << std::endl;
        connection.asyncServe(mapping, issac::modbus::demo::soft_protocol::makeHooks(),
                              [](const std::error_code& error)
                              {
                                  if (error)
                                      std::cerr << "[slave] " << error.message() << std::endl;
                              });
        io.run();
    }
    catch (const std::exception& error)
    {
        std::cerr << "RTU slave: " << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
