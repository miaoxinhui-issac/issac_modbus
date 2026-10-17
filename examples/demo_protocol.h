#ifndef ISSAC_MODBUS_DEMO_PROTOCOL_H
#define ISSAC_MODBUS_DEMO_PROTOCOL_H

#include <issac/modbus/modbus.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

namespace issac::modbus::demo
{

namespace soft_protocol
{

constexpr std::uint8_t UnitId = 1;

constexpr std::uint16_t CoilStart = 0x0000;
constexpr std::uint16_t DiscreteInputStart = 0x0100;
constexpr std::uint16_t HoldingRegisterStart = 0x0200;
constexpr std::uint16_t InputRegisterStart = 0x0300;
constexpr std::uint16_t PointCount = 8;

// The application protocol used by all four examples:
//   0000..0003 coils: pump, valve, alarm reset, automatic mode
//   0100..0103 discrete inputs: ready, running, fault, emergency stop
//   0200..0203 holding registers: target RPM, target temperature (0.1 C),
//                                 control flags, command
//   0300..0303 input registers: actual RPM, temperature (0.1 C), pressure (kPa),
//                               telemetry counter
constexpr std::uint16_t CoilPumpEnable = CoilStart + 0;
constexpr std::uint16_t CoilValveOpen = CoilStart + 1;

constexpr std::uint16_t DiscreteReady = DiscreteInputStart + 0;
constexpr std::uint16_t DiscreteRunning = DiscreteInputStart + 1;
constexpr std::uint16_t DiscreteFault = DiscreteInputStart + 2;
constexpr std::uint16_t DiscreteEmergencyStop = DiscreteInputStart + 3;

constexpr std::uint16_t HoldingTargetRpm = HoldingRegisterStart + 0;
constexpr std::uint16_t HoldingTargetTemperature = HoldingRegisterStart + 1;
constexpr std::uint16_t HoldingControlFlags = HoldingRegisterStart + 2;
constexpr std::uint16_t HoldingCommand = HoldingRegisterStart + 3;

constexpr std::uint16_t InputActualRpm = InputRegisterStart + 0;
constexpr std::uint16_t InputTemperature = InputRegisterStart + 1;
constexpr std::uint16_t InputPressure = InputRegisterStart + 2;
constexpr std::uint16_t InputTelemetryCounter = InputRegisterStart + 3;

inline core::Mapping makeMapping()
{
    auto mapping = core::Mapping::newStartAddress(
        CoilStart, PointCount, DiscreteInputStart, PointCount, HoldingRegisterStart, PointCount,
        InputRegisterStart, PointCount);

    mapping.coils[0] = 0;
    mapping.coils[1] = 0;
    mapping.coils[2] = 0;
    mapping.coils[3] = 1;

    mapping.discreteInputs[0] = 1;
    mapping.discreteInputs[1] = 0;
    mapping.discreteInputs[2] = 0;
    mapping.discreteInputs[3] = 0;

    mapping.holdingRegisters[0] = 1200;
    mapping.holdingRegisters[1] = 235;
    mapping.holdingRegisters[2] = 0x0001;
    mapping.holdingRegisters[3] = 0;

    mapping.inputRegisters[0] = 0;
    mapping.inputRegisters[1] = 235;
    mapping.inputRegisters[2] = 1013;
    mapping.inputRegisters[3] = 0;
    return mapping;
}

inline core::MappingHooks makeHooks()
{
    auto counter = std::make_shared<std::uint16_t>(0);
    core::MappingHooks hooks;
    hooks.beforeRequest = [counter](core::Mapping& mapping)
    {
        const auto targetRpm = mapping.holdingRegisters[0];
        mapping.discreteInputs[0] = 1; // ready
        mapping.discreteInputs[1] = mapping.coils[0] != 0; // running follows pump
        mapping.discreteInputs[2] = 0; // no simulated fault
        mapping.discreteInputs[3] = 0; // emergency stop released
        mapping.inputRegisters[0] = mapping.coils[0] != 0 ? targetRpm : 0;
        mapping.inputRegisters[1] = mapping.holdingRegisters[1];
        mapping.inputRegisters[2] = 1013;
        mapping.inputRegisters[3] = ++*counter;
    };
    hooks.afterRequest = [](const core::Mapping& before, core::Mapping& after)
    {
        if (before.coils != after.coils || before.holdingRegisters != after.holdingRegisters)
        {
            std::cout << "[slave] writable data changed" << std::endl;
        }
    };
    return hooks;
}

} // namespace soft_protocol

inline void printValues(const char* name,
                        const std::vector<::issac::modbus::protocol::ModbusCell>& values)
{
    std::cout << "[master] " << name << ":";
    for (const auto& value : values)
        std::cout << ' ' << value.toString();
    std::cout << std::endl;
}

template <typename Connection>
class MasterFlow final : public std::enable_shared_from_this<MasterFlow<Connection>>
{
  public:
    using ErrorCode = typename Connection::ErrorCode;
    using Values = std::vector<::issac::modbus::protocol::ModbusCell>;
    using Exception = ::issac::modbus::protocol::ModbusException;

    MasterFlow(Connection& connection, asio::io_context& io) : connection_(connection), io_(io) {}

    void start() { readCoils(); }

  private:
    bool handleResult(const char* operation, const ErrorCode& error, const Values& values,
                      const std::optional<Exception>& exception)
    {
        if (error)
        {
            std::cerr << "[master] " << operation << " failed: " << error.message() << std::endl;
            if (exception)
                std::cerr << "[master] exception: " << exception->what() << std::endl;
            finish(error);
            return false;
        }

        printValues(operation, values);
        return true;
    }

    void finish(const ErrorCode& error)
    {
        if (error)
            std::cerr << "[master] failed: " << error.message() << std::endl;
        else
            std::cout << "[master] protocol exchange completed" << std::endl;
        connection_.close();
        io_.stop();
    }

    void readCoils()
    {
        auto self = this->shared_from_this();
        connection_.asyncReadCoils(
            soft_protocol::UnitId, soft_protocol::CoilStart, 4,
            [self](const ErrorCode& error, Values values, std::optional<Exception> exception)
            {
                if (!self->handleResult("coils", error, values, exception))
                    return;
                self->readDiscreteInputs();
            });
    }

    void readDiscreteInputs()
    {
        auto self = this->shared_from_this();
        connection_.asyncReadDiscreteInputs(
            soft_protocol::UnitId, soft_protocol::DiscreteInputStart, 4,
            [self](const ErrorCode& error, Values values, std::optional<Exception> exception)
            {
                if (!self->handleResult("discrete inputs", error, values, exception))
                    return;
                self->readHoldingRegisters();
            });
    }

    void readHoldingRegisters()
    {
        auto self = this->shared_from_this();
        connection_.asyncReadHoldingRegisters(
            soft_protocol::UnitId, soft_protocol::HoldingRegisterStart, 4,
            [self](const ErrorCode& error, Values values, std::optional<Exception> exception)
            {
                if (!self->handleResult("holding registers", error, values, exception))
                    return;
                self->readInputRegisters();
            });
    }

    void readInputRegisters()
    {
        auto self = this->shared_from_this();
        connection_.asyncReadInputRegisters(
            soft_protocol::UnitId, soft_protocol::InputRegisterStart, 4,
            [self](const ErrorCode& error, Values values, std::optional<Exception> exception)
            {
                if (!self->handleResult("input registers", error, values, exception))
                    return;
                self->writePumpCoil();
            });
    }

    void writePumpCoil()
    {
        auto self = this->shared_from_this();
        connection_.asyncWriteSingleCoil(
            soft_protocol::UnitId, soft_protocol::CoilPumpEnable, true,
            [self](const ErrorCode& error, Values values, std::optional<Exception> exception)
            {
                if (!self->handleResult("write pump coil", error, values, exception))
                    return;
                self->writeHoldingRegisters();
            });
    }

    void writeHoldingRegisters()
    {
        auto self = this->shared_from_this();
        connection_.asyncWriteMultipleRegisters(
            soft_protocol::UnitId, soft_protocol::HoldingRegisterStart,
            {1500, 240, 0x0003, 0x0055},
            [self](const ErrorCode& error, Values values, std::optional<Exception> exception)
            {
                if (!self->handleResult("write holding registers", error, values, exception))
                    return;
                self->verifyHoldingRegisters();
            });
    }

    void verifyHoldingRegisters()
    {
        auto self = this->shared_from_this();
        connection_.asyncReadHoldingRegisters(
            soft_protocol::UnitId, soft_protocol::HoldingRegisterStart, 4,
            [self](const ErrorCode& error, Values values, std::optional<Exception> exception)
            {
                if (!self->handleResult("holding registers after write", error, values,
                                        exception))
                    return;
                self->finish({});
            });
    }

    Connection& connection_;
    asio::io_context& io_;
};

template <typename Connection>
void runMasterFlow(Connection& connection, asio::io_context& io)
{
    std::make_shared<MasterFlow<Connection>>(connection, io)->start();
}

} // namespace issac::modbus::demo

#endif // ISSAC_MODBUS_DEMO_PROTOCOL_H
