#include "demo_protocol.h"

#include <cstdlib>
#include <iostream>
#include <memory>

int main(int argc, char* argv[])
{
    const std::string host = argc > 1 ? argv[1] : "127.0.0.1";
    const auto port = static_cast<std::uint16_t>(argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 1502);

    asio::io_context io;
    auto connection = std::make_shared<issac::modbus::transport::tcp::Connection>(io, host, port);
    connection->setTimeout(std::chrono::seconds(2));
    connection->setMessageId(1);
    connection->asyncConnect(
        [connection, &io](const std::error_code& error)
        {
            if (error)
            {
                std::cerr << "TCP master connect: " << error.message() << std::endl;
                io.stop();
                return;
            }
            issac::modbus::demo::runMasterFlow(*connection, io);
        });
    io.run();
    return EXIT_SUCCESS;
}
