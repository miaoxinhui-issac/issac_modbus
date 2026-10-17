#include "issac/modbus/core/modbusMapping.h"

#include <stdexcept>
#include <string>

namespace MB
{

/// @brief 按四类数据区容量创建从地址零开始的映射。
Mapping::Mapping(std::size_t coilsCount, std::size_t discreteInputsCount,
                 std::size_t holdingRegistersCount, std::size_t inputRegistersCount)
    : coils(coilsCount), discreteInputs(discreteInputsCount),
      holdingRegisters(holdingRegistersCount), inputRegisters(inputRegistersCount)
{
}

/// @brief 创建带四类起始地址并校验地址范围的映射。
Mapping Mapping::newStartAddress(unsigned int startBits, unsigned int bitCount,
                                 unsigned int startInputBits, unsigned int inputBitCount,
                                 unsigned int startRegisters, unsigned int registerCount,
                                 unsigned int startInputRegisters, unsigned int inputRegisterCount)
{
    constexpr unsigned int addressSpaceSize = 0x10000u;
    const auto checkedCount = [addressSpaceSize](unsigned int start, unsigned int count,
                                                 const char* name) -> std::size_t
    {
        if (start >= addressSpaceSize || count > addressSpaceSize - start)
        {
            throw std::invalid_argument(std::string(name) +
                                        " mapping range exceeds the Modbus address space.");
        }
        return static_cast<std::size_t>(count);
    };

    Mapping mapping;
    mapping.startBits = static_cast<std::uint16_t>(startBits);
    mapping.coils.resize(checkedCount(startBits, bitCount, "Coil"));
    mapping.startInputBits = static_cast<std::uint16_t>(startInputBits);
    mapping.discreteInputs.resize(checkedCount(startInputBits, inputBitCount, "Discrete input"));
    mapping.startRegisters = static_cast<std::uint16_t>(startRegisters);
    mapping.holdingRegisters.resize(
        checkedCount(startRegisters, registerCount, "Holding register"));
    mapping.startInputRegisters = static_cast<std::uint16_t>(startInputRegisters);
    mapping.inputRegisters.resize(
        checkedCount(startInputRegisters, inputRegisterCount, "Input register"));
    return mapping;
}

/// @brief 返回包含起始地址和数据的完整 Mapping 副本。
Mapping copyMapping(const Mapping& mapping) { return mapping; }

namespace
{

template <typename Container>
/// @brief 把 Modbus 绝对地址转换为容器下标并检查访问范围。
std::size_t mappingOffset(const Container& container, std::uint16_t startAddress,
                          std::uint16_t address, std::size_t count)
{
    if (address < startAddress)
    {
        throw std::out_of_range("Modbus mapping address is out of range.");
    }

    const std::size_t offset = static_cast<std::size_t>(address - startAddress);
    if (offset > container.size() || count > container.size() - offset)
    {
        throw std::out_of_range("Modbus mapping address is out of range.");
    }
    return offset;
}

/// @brief 根据请求信息创建对应的 Modbus 异常。
ModbusException mappingError(const ModbusRequest& request, MBErrorCode code)
{
    return ModbusException(code, request.slaveID(), request.functionCode());
}

} // namespace

/// @brief 根据功能码读取或修改 Mapping 并生成响应。
ModbusResponse handleRequest(const ModbusRequest& request, Mapping& mapping)
{
    try
    {
        const auto address = request.registerAddress();
        const auto count = request.numberOfRegisters();
        std::vector<ModbusCell> values;

        switch (request.functionCode())
        {
        case MBfunctionCode::MBReadCoils:
        {
            const std::size_t offset =
                mappingOffset(mapping.coils, mapping.startBits, address, count);
            for (std::size_t i = 0; i < count; ++i)
                values.emplace_back(mapping.coils[offset + i] != 0);
            return ModbusResponse(request.slaveID(), request.functionCode(), address, count,
                                  std::move(values));
        }
        case MBfunctionCode::MBReadDiscreteInputs:
        {
            const std::size_t offset =
                mappingOffset(mapping.discreteInputs, mapping.startInputBits, address, count);
            for (std::size_t i = 0; i < count; ++i)
                values.emplace_back(mapping.discreteInputs[offset + i] != 0);
            return ModbusResponse(request.slaveID(), request.functionCode(), address, count,
                                  std::move(values));
        }
        case MBfunctionCode::MBReadHoldingRegisters:
        {
            const std::size_t offset =
                mappingOffset(mapping.holdingRegisters, mapping.startRegisters, address, count);
            for (std::size_t i = 0; i < count; ++i)
                values.emplace_back(mapping.holdingRegisters[offset + i]);
            return ModbusResponse(request.slaveID(), request.functionCode(), address, count,
                                  std::move(values));
        }
        case MBfunctionCode::MBReadInputRegisters:
        {
            const std::size_t offset =
                mappingOffset(mapping.inputRegisters, mapping.startInputRegisters, address, count);
            for (std::size_t i = 0; i < count; ++i)
                values.emplace_back(mapping.inputRegisters[offset + i]);
            return ModbusResponse(request.slaveID(), request.functionCode(), address, count,
                                  std::move(values));
        }
        case MBfunctionCode::MBWriteSingleCoil:
        case MBfunctionCode::MBWriteMultipleCoils:
        {
            const std::size_t offset =
                mappingOffset(mapping.coils, mapping.startBits, address, count);
            for (std::size_t i = 0; i < count; ++i)
                mapping.coils[offset + i] = request.registerValues()[i].coil();
            return ModbusResponse(request.slaveID(), request.functionCode(), address, count,
                                  request.registerValues());
        }
        case MBfunctionCode::MBWriteSingleRegister:
        case MBfunctionCode::MBWriteMultipleRegisters:
        {
            const std::size_t offset =
                mappingOffset(mapping.holdingRegisters, mapping.startRegisters, address, count);
            for (std::size_t i = 0; i < count; ++i)
                mapping.holdingRegisters[offset + i] = request.registerValues()[i].reg();
            return ModbusResponse(request.slaveID(), request.functionCode(), address, count,
                                  request.registerValues());
        }
        case MBfunctionCode::MBMaskWriteRegister:
        {
            const std::size_t offset =
                mappingOffset(mapping.holdingRegisters, mapping.startRegisters, address, 1);
            mapping.holdingRegisters[offset] = static_cast<std::uint16_t>(
                (mapping.holdingRegisters[offset] & request.registerValues()[0].reg()) |
                (request.registerValues()[1].reg() & ~request.registerValues()[0].reg()));
            return ModbusResponse(request.slaveID(), request.functionCode(), address, 2,
                                  request.registerValues());
        }
        case MBfunctionCode::MBReadWriteMultipleRegisters:
        {
            const std::size_t readOffset =
                mappingOffset(mapping.holdingRegisters, mapping.startRegisters, address, count);
            const std::size_t writeOffset =
                mappingOffset(mapping.holdingRegisters, mapping.startRegisters,
                              request.writeAddress(), request.numberOfWriteRegisters());
            for (std::size_t i = 0; i < request.numberOfWriteRegisters(); ++i)
                mapping.holdingRegisters[writeOffset + i] = request.registerValues()[i].reg();
            for (std::size_t i = 0; i < count; ++i)
                values.emplace_back(mapping.holdingRegisters[readOffset + i]);
            return ModbusResponse(request.slaveID(), request.functionCode(), address, count,
                                  std::move(values));
        }
        case MBfunctionCode::MBReportSlaveID:
            return ModbusResponse(request.slaveID(), request.functionCode(), 0, 0, {});
        }
    }
    catch (const std::out_of_range&)
    {
        throw mappingError(request, MBErrorCode::MBIllegalDataAddress);
    }
    catch (const std::exception&)
    {
        throw mappingError(request, MBErrorCode::MBIllegalDataValue);
    }

    throw mappingError(request, MBErrorCode::MBIllegalFunction);
}

/// @brief 在请求前后执行设备同步钩子后处理请求。
ModbusResponse handleRequest(const ModbusRequest& request, Mapping& mapping,
                             const MappingHooks& hooks)
{
    if (hooks.beforeRequest)
    {
        hooks.beforeRequest(mapping);
    }

    Mapping before = copyMapping(mapping);
    ModbusResponse response = handleRequest(request, mapping);

    if (hooks.afterRequest)
    {
        hooks.afterRequest(before, mapping);
    }

    return response;
}

} // namespace MB
