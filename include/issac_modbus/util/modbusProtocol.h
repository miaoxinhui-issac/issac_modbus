#ifndef MODBUS_PROTOCOL_H
#define MODBUS_PROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "issac_modbus/util/modbusRequest.h"
#include "issac_modbus/util/modbusResponse.h"
#include "issac_modbus/util/modbusUtils.h"

namespace issac_modbus
{

/// @brief 判断请求是否为广播请求。
inline bool isBroadcast(const ModbusRequest& request) noexcept { return request.unitId() == 0; }

/// @brief 校验请求数量、广播规则和功能码取值范围。
inline void validateRequestLimits(const ModbusRequest& request)
{
    if (request.unitId() == 0 &&
        (functionType(request.functionCode()) == Read ||
         request.functionCode() == MBfunctionCode::MBReadWriteMultipleRegisters))
    {
        throw std::invalid_argument("Broadcast is only valid for write requests.");
    }
    const auto count = request.numberOfRegisters();
    switch (request.functionCode())
    {
    case MBfunctionCode::MBReadCoils:
        if (count == 0 || count > 2000)
            throw std::invalid_argument("Invalid coil count.");
        break;
    case MBfunctionCode::MBReadDiscreteInputs:
        if (count == 0 || count > 2000)
            throw std::invalid_argument("Invalid discrete input count.");
        break;
    case MBfunctionCode::MBReadHoldingRegisters:
    case MBfunctionCode::MBReadInputRegisters:
        if (count == 0 || count > 125)
            throw std::invalid_argument("Invalid register count.");
        break;
    case MBfunctionCode::MBWriteMultipleCoils:
        if (count == 0 || count > 1968)
            throw std::invalid_argument("Invalid coil write count.");
        break;
    case MBfunctionCode::MBWriteMultipleRegisters:
        if (count == 0 || count > 123)
            throw std::invalid_argument("Invalid register write count.");
        break;
    case MBfunctionCode::MBReadWriteMultipleRegisters:
        if (count == 0 || count > 125 || request.numberOfWriteRegisters() == 0 ||
            request.numberOfWriteRegisters() > 121)
        {
            throw std::invalid_argument("Invalid read/write register count.");
        }
        break;
    case MBfunctionCode::MBWriteSingleCoil:
    case MBfunctionCode::MBWriteSingleRegister:
    case MBfunctionCode::MBReportSlaveID:
    case MBfunctionCode::MBMaskWriteRegister:
        break;
    }
}

/// @brief 校验通用 Modbus 单元地址是否在 0 到 255 之间。
inline void validateUnitId(std::uint8_t id)
{
    // Unit IDs are transmitted as uint8_t, so the 0..255 range is enforced by
    // the type itself. Keep the function for callers that use a common API.
    (void)id;
}

/// @brief 校验 RTU 单元地址是否在 0 到 247 之间。
inline void validateRtuUnitId(std::uint8_t id)
{
    if (id > 247)
    {
        throw std::invalid_argument("Modbus RTU unit ID must be in range 0..247.");
    }
}

/// @brief 校验响应从站地址和功能码是否与请求一致。
inline void validateResponseIdentity(const ModbusRequest& request, const ModbusResponse& response)
{
    if (request.unitId() != response.unitId())
    {
        throw std::invalid_argument("Modbus response unit ID mismatch.");
    }
    if (request.functionCode() != response.functionCode())
    {
        throw std::invalid_argument("Modbus response function mismatch.");
    }
}

/// @brief 按请求数量裁剪线圈响应中的补齐位。
inline void normalizeResponseForRequest(const ModbusRequest& request, ModbusResponse& response)
{
    if (request.functionCode() == MBfunctionCode::MBReadCoils ||
        request.functionCode() == MBfunctionCode::MBReadDiscreteInputs)
    {
        auto values = response.registerValues();
        if (values.size() < request.numberOfRegisters())
        {
            throw std::invalid_argument("Modbus response contains too few bits.");
        }
        values.resize(request.numberOfRegisters());
        response.setValues(values);
        response.setRegistersNumber(request.numberOfRegisters());
    }
}

} // namespace issac_modbus

#endif // MODBUS_PROTOCOL_H
