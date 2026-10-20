#ifndef ISSAC_MODBUS_RTU_CODEC_H
#define ISSAC_MODBUS_RTU_CODEC_H

#include "issac_modbus/modbusRtu/rtuTypes.h"
#include "issac_modbus/util/modbusError.h"
#include "issac_modbus/util/modbusRequest.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <system_error>

namespace issac_modbus::detail
{

std::error_code mbError(MBErrorCode code);
std::error_code systemError();
std::error_code badFrame();

bool supportedFunction(std::uint8_t functionCode);
bool validExceptionCode(std::uint8_t code);
bool broadcastWrite(std::uint8_t functionCode);
bool validCrc(const RtuBytes& data);
RtuBytes withCrc(RtuBytes data);
std::uint16_t word(const RtuBytes& data, std::size_t offset);
void checkRange(std::uint16_t address, std::size_t count);
void checkRequest(const ModbusRequest& request, const RtuBytes& frame);
std::optional<std::size_t> frameLength(const RtuBytes& data, bool request);
std::error_code matchResponse(const RtuBytes& request, const RtuBytes& response);

} // namespace issac_modbus::detail

#endif // ISSAC_MODBUS_RTU_CODEC_H
