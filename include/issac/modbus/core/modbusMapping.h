#ifndef MODBUS_MAPPING_H
#define MODBUS_MAPPING_H

#include "issac/modbus/protocol/modbusException.h"
#include "issac/modbus/protocol/modbusRequest.h"
#include "issac/modbus/protocol/modbusResponse.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace MB
{

/// @brief Modbus 服务端地址空间，语义兼容 libmodbus 的 Mapping。
class Mapping
{
  public:
    /// @brief 按四类数据区的容量创建从地址零开始的映射。
    Mapping(std::size_t coils = 0, std::size_t discreteInputs = 0, std::size_t holdingRegisters = 0,
            std::size_t inputRegisters = 0);

    /// @brief 创建带起始地址的 Mapping，参数顺序兼容
    /// modbus_mapping_new_start_address()。
    /// @throws std::invalid_argument 起始地址或数量超出 Modbus 地址范围时抛出。
    [[nodiscard]] static Mapping
    newStartAddress(unsigned int startBits, unsigned int bitCount, unsigned int startInputBits,
                    unsigned int inputBitCount, unsigned int startRegisters,
                    unsigned int registerCount, unsigned int startInputRegisters,
                    unsigned int inputRegisterCount);

    std::uint16_t startBits = 0;
    std::uint16_t startInputBits = 0;
    std::uint16_t startRegisters = 0;
    std::uint16_t startInputRegisters = 0;

    std::vector<std::uint8_t> coils;
    std::vector<std::uint8_t> discreteInputs;
    std::vector<std::uint16_t> holdingRegisters;
    std::vector<std::uint16_t> inputRegisters;
};

/// @brief 设备型 Modbus 从站使用的生命周期钩子。
///
/// beforeRequest 在应用请求前调用；afterRequest 接收请求前的快照和请求处理后的 Mapping。
struct MappingHooks
{
    std::function<void(Mapping&)> beforeRequest;
    std::function<void(const Mapping&, Mapping&)> afterRequest;
    // 可选执行器，用于承载可能阻塞设备业务操作的任务。
    std::function<void(std::function<void()>)> dispatch;
};

/// @brief 深拷贝一份完整的 Mapping，包括起始地址和全部数据。
[[nodiscard]] Mapping copyMapping(const Mapping& mapping);

/// @brief 根据 Mapping 执行请求并返回响应。
/// @throws ModbusException 请求无法处理时抛出。
ModbusResponse handleRequest(const ModbusRequest& request, Mapping& mapping);

/// @brief 按照 libmodbus 风格的生命周期处理请求。
ModbusResponse handleRequest(const ModbusRequest& request, Mapping& mapping,
                             const MappingHooks& hooks);

} // namespace MB

#endif // MODBUS_MAPPING_H
