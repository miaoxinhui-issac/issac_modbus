#include "demo_protocol.h"

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
        issac_modbus::SerialOptions serial;
        serial.baudRate = baud;
        serial.serialMode =
            useRs485 ? issac_modbus::SerialMode::RS485 : issac_modbus::SerialMode::RS232;

        issac_modbus::MasterOptions master;
        master.responseTimeout = std::chrono::seconds(2);
        issac_modbus::RtuMaster connection(io, device, serial, master);
        connection.open();
        std::cout << "RTU master connected to " << device << " at " << baud << " baud, mode "
                  << (useRs485 ? "RS485" : "RS232") << std::endl;
        issac_modbus_demo::runMasterFlow(connection, io);
        io.run();
    }
    catch (const std::exception& error)
    {
        std::cerr << "RTU master: " << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
