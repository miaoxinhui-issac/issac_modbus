#ifndef ISSAC_MODBUS_TRANSPORT_H
#define ISSAC_MODBUS_TRANSPORT_H

#include "issac/modbus/transport/modbusSerial.h"
#include "issac/modbus/transport/modbusTcpConnection.h"

namespace issac::modbus::transport
{
namespace rtu
{
using Connection = ::MB::Serial::Connection;
}
namespace tcp
{
using Connection = ::MB::TCP::Connection;
}
} // namespace issac::modbus::transport

#endif // ISSAC_MODBUS_TRANSPORT_H
