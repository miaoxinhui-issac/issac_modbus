#ifndef ISSAC_MODBUS_RTU_MASTER_H
#define ISSAC_MODBUS_RTU_MASTER_H

#include "issac_modbus/modbusRtu/rtuTypes.h"
#include "issac_modbus/util/modbusRequest.h"

#include <asio.hpp>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace issac_modbus
{

/// Asynchronous RTU master. It owns one serial link and exposes only master
/// transactions; manual frame I/O belongs to RtuConnection, slave service to
/// RtuSlave.
class RtuMaster final
{
  public:
    using ErrorCode = std::error_code;
    using ValuesHandler = RtuValuesHandler;

    RtuMaster(asio::io_context& io, std::string device, SerialOptions serial = {},
              MasterOptions master = {});
    ~RtuMaster();

    RtuMaster(const RtuMaster&) = delete;
    RtuMaster& operator=(const RtuMaster&) = delete;
    RtuMaster(RtuMaster&&) = delete;
    RtuMaster& operator=(RtuMaster&&) = delete;

    void open();
    /// Initiates close without a completion boundary.
    void close() noexcept;
    /// Completion runs after transaction cleanup and port close.
    void asyncClose(RtuCompletionHandler handler = {});
    void cancel() noexcept;
    [[nodiscard]] SerialState serialState() const;
    [[nodiscard]] bool isOpen() const;
    [[nodiscard]] bool busy() const;
    [[nodiscard]] SerialOptions serialOptions() const;
    void setSerialOptions(const SerialOptions& options);

    /// Transactions are queued FIFO. An omitted policy uses MasterOptions.
    void asyncTransaction(const ModbusRequest& request, TransactionHandler handler,
                          std::optional<RetryPolicy> policy = std::nullopt);
    void asyncReadCoils(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                        ValuesHandler handler, std::optional<RetryPolicy> policy = std::nullopt);
    void asyncReadDiscreteInputs(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                 ValuesHandler handler,
                                 std::optional<RetryPolicy> policy = std::nullopt);
    void asyncReadHoldingRegisters(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                   ValuesHandler handler,
                                   std::optional<RetryPolicy> policy = std::nullopt);
    void asyncReadInputRegisters(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                 ValuesHandler handler,
                                 std::optional<RetryPolicy> policy = std::nullopt);
    void asyncWriteSingleCoil(std::uint8_t unit, std::uint16_t address, bool value,
                              ValuesHandler handler,
                              std::optional<RetryPolicy> policy = std::nullopt);
    void asyncWriteSingleRegister(std::uint8_t unit, std::uint16_t address, std::uint16_t value,
                                  ValuesHandler handler,
                                  std::optional<RetryPolicy> policy = std::nullopt);
    void asyncWriteMultipleRegisters(std::uint8_t unit, std::uint16_t address,
                                     const std::vector<std::uint16_t>& values,
                                     ValuesHandler handler,
                                     std::optional<RetryPolicy> policy = std::nullopt);
    void asyncWriteMultipleCoils(std::uint8_t unit, std::uint16_t address,
                                 const std::vector<bool>& values, ValuesHandler handler,
                                 std::optional<RetryPolicy> policy = std::nullopt);
    void asyncMaskWriteRegister(std::uint8_t unit, std::uint16_t address, std::uint16_t andMask,
                                std::uint16_t orMask, ValuesHandler handler,
                                std::optional<RetryPolicy> policy = std::nullopt);
    void asyncReadWriteMultipleRegisters(std::uint8_t unit, std::uint16_t readAddress,
                                         std::uint16_t readCount, std::uint16_t writeAddress,
                                         const std::vector<std::uint16_t>& values,
                                         ValuesHandler handler,
                                         std::optional<RetryPolicy> policy = std::nullopt);
    void asyncReportSlaveId(std::uint8_t unit, ValuesHandler handler,
                            std::optional<RetryPolicy> policy = std::nullopt);

  private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

} // namespace issac_modbus

#endif // ISSAC_MODBUS_RTU_MASTER_H
