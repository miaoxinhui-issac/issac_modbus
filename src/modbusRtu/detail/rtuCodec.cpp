#include "rtuCodec.h"

#include "issac_modbus/util/crc.h"
#include "issac_modbus/util/modbusProtocol.h"

#include <algorithm>
#include <cerrno>
#include <stdexcept>

namespace issac_modbus::detail
{

std::error_code mbError(MBErrorCode code) { return makeErrorCode(code); }

std::error_code systemError() { return {errno, std::generic_category()}; }

std::error_code badFrame() { return mbError(MBErrorCode::MBBadFrame); }

bool supportedFunction(std::uint8_t functionCode)
{
    switch (functionCode)
    {
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x0F:
    case 0x10:
    case 0x11:
    case 0x16:
    case 0x17:
        return true;
    default:
        return false;
    }
}

bool validExceptionCode(std::uint8_t code)
{
    return (code >= 1 && code <= 8) || code == 0x0A || code == 0x0B;
}

bool broadcastWrite(std::uint8_t functionCode)
{
    return functionCode == 0x05 || functionCode == 0x06 || functionCode == 0x0F ||
           functionCode == 0x10 || functionCode == 0x16;
}

bool validCrc(const RtuBytes& data)
{
    if (data.size() < 4 || data.size() > MaxRtuAduSize)
        return false;
    const auto last = data.size();
    const auto received = static_cast<std::uint16_t>(data[last - 2]) |
                          (static_cast<std::uint16_t>(data[last - 1]) << 8u);
    return calculateCRC(data.data(), last - 2) == received;
}

RtuBytes withCrc(RtuBytes data)
{
    if (data.size() < 2 || data.size() > MaxRtuAduSize - 2)
        throw std::invalid_argument("RTU ADU payload must fit in 256 bytes");
    const auto crc = calculateCRC(data);
    data.push_back(static_cast<std::uint8_t>(crc));
    data.push_back(static_cast<std::uint8_t>(crc >> 8u));
    return data;
}

std::uint16_t word(const RtuBytes& data, std::size_t offset)
{
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(data.at(offset)) << 8u) |
                                      data.at(offset + 1));
}

void checkRange(std::uint16_t address, std::size_t count)
{
    if (count == 0 || count > 65536u - static_cast<std::size_t>(address))
        throw std::invalid_argument("Modbus address range exceeds 0..65535");
}

void checkRequest(const ModbusRequest& request, const RtuBytes& frame)
{
    validateRtuUnitId(request.unitId());
    validateRequestLimits(request);
    if (!supportedFunction(static_cast<std::uint8_t>(request.functionCode())))
        throw std::invalid_argument("Unsupported function");
    if (request.unitId() == 0 && !broadcastWrite(frame.at(1)))
        throw std::invalid_argument("Broadcast accepts write-only functions");

    auto parsed = ModbusRequest::fromRawCRC(frame);
    validateRequestLimits(parsed);
    const auto functionCode = frame[1];
    if (functionCode != 0x11)
    {
        checkRange(parsed.registerAddress(), functionCode == 0x16 ? 1 : parsed.numberOfRegisters());
    }
    if (functionCode == 0x17)
        checkRange(parsed.writeAddress(), parsed.numberOfWriteRegisters());
    if (parsed.toRaw() != request.toRaw())
        throw std::invalid_argument("Request object and encoded frame differ");
}

std::optional<std::size_t> frameLength(const RtuBytes& data, bool request)
{
    if (data.size() < 2)
        return std::nullopt;

    const auto functionCode = data[1];
    if (!request && (functionCode & 0x80u))
        return 5;
    if (!supportedFunction(functionCode))
        return std::nullopt;

    if (request)
    {
        switch (functionCode)
        {
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x04:
        case 0x05:
        case 0x06:
            return 8;
        case 0x0F:
        case 0x10:
            if (data.size() >= 7)
                return 9u + data[6];
            break;
        case 0x11:
            return 4;
        case 0x16:
            return 10;
        case 0x17:
            if (data.size() >= 11)
                return 13u + data[10];
            break;
        }
    }
    else
    {
        switch (functionCode)
        {
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x04:
        case 0x11:
        case 0x17:
            if (data.size() >= 3)
                return 5u + data[2];
            break;
        case 0x05:
        case 0x06:
        case 0x0F:
        case 0x10:
            return 8;
        case 0x16:
            return 10;
        }
    }
    return std::nullopt;
}

std::error_code matchResponse(const RtuBytes& request, const RtuBytes& response)
{
    if (response.size() < 5 || response.size() > MaxRtuAduSize)
        return badFrame();
    if (response[0] != request[0])
        return mbError(MBErrorCode::MBInvalidSlaveId);
    if ((response[1] & 0x7Fu) != request[1])
        return mbError(MBErrorCode::MBFunctionMismatch);

    if ((response[1] & 0x80u) != 0)
    {
        if (response.size() != 5 || !validExceptionCode(response[2]))
            return badFrame();
        return {};
    }

    switch (request[1])
    {
    case 0x01:
    case 0x02:
        if (response[2] != (word(request, 4) + 7u) / 8u)
            return badFrame();
        break;
    case 0x03:
    case 0x04:
    case 0x17:
        if (response[2] != 2u * word(request, 4))
            return badFrame();
        break;
    case 0x05:
    case 0x06:
    case 0x16:
        if (response.size() != request.size() ||
            !std::equal(request.begin(), request.end() - 2, response.begin()))
        {
            return badFrame();
        }
        break;
    case 0x0F:
    case 0x10:
        if (response.size() != 8 ||
            !std::equal(request.begin(), request.begin() + 6, response.begin()))
        {
            return badFrame();
        }
        break;
    case 0x11:
        if (response[2] < 2)
            return badFrame();
        break;
    default:
        return badFrame();
    }
    return {};
}

} // namespace issac_modbus::detail
