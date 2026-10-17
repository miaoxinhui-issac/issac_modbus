#ifndef ISSAC_MODBUS_MODBUS_H
#define ISSAC_MODBUS_MODBUS_H

#include "issac/modbus/core/modbusError.h"
#include "issac/modbus/core/modbusMapping.h"
#include "issac/modbus/core/modbusTransaction.h"
#include "issac/modbus/protocol/crc.h"
#include "issac/modbus/protocol/modbusCell.h"
#include "issac/modbus/protocol/modbusException.h"
#include "issac/modbus/protocol/modbusProtocol.h"
#include "issac/modbus/protocol/modbusRequest.h"
#include "issac/modbus/protocol/modbusResponse.h"
#include "issac/modbus/protocol/modbusUtils.h"
#include "issac/modbus/server/modbusTcpServer.h"
#include "issac/modbus/transport/modbusSerial.h"
#include "issac/modbus/transport/modbusTcpConnection.h"

namespace issac
{
namespace modbus
{

namespace protocol
{
using ::bigEndianConv;
using ::calculateCRC;
using ::FloatOrder;
using ::functionRegister;
using ::functionType;
using ::getMBErrorString;
using ::isStandardErrorCode;
using ::MBErrorCode;
using ::MBfunctionCode;
using ::MBFunctionRegisters;
using ::MBFunctionType;
using ::MBMaxDataSize;
using ::modbus_get_byte_from_bits;
using ::modbus_get_float_abcd;
using ::modbus_get_float_badc;
using ::modbus_get_float_cdab;
using ::modbus_get_float_dcba;
using ::modbus_set_bits_from_bytes;
using ::modbus_set_float_abcd;
using ::modbus_set_float_badc;
using ::modbus_set_float_cdab;
using ::modbus_set_float_dcba;
using ::ModbusCell;
using ::ModbusException;
using ::ModbusRequest;
using ::ModbusResponse;
using ::Registers;
using ::reorder;
using ::setFloat;
} // namespace protocol

namespace core
{
using ::MB::copyMapping;
using ::MB::handleRequest;
using ::MB::makeErrorCode;
using ::MB::Mapping;
using ::MB::MappingHooks;
using ::MB::modbusErrorCategory;
using ::MB::RetryPolicy;
using ::MB::TransactionHandler;
} // namespace core

namespace transport
{
namespace rtu
{
using Connection = ::MB::Serial::Connection;
}

namespace tcp
{
using Connection = ::MB::TCP::Connection;
}
} // namespace transport

namespace server
{
namespace tcp
{
using Server = ::MB::TCP::Server;
}
} // namespace server

} // namespace modbus
} // namespace issac

#endif // ISSAC_MODBUS_MODBUS_H
