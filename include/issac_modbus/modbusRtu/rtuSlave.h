#ifndef ISSAC_MODBUS_RTU_SLAVE_H
#define ISSAC_MODBUS_RTU_SLAVE_H

#include "issac_modbus/modbusRtu/rtuTypes.h"
#include "issac_modbus/util/modbusMapping.h"

#include <asio.hpp>
#include <memory>
#include <string>

namespace issac_modbus
{

/// Asynchronous RTU slave service. It owns one serial link, filters requests
/// by its configured unit address, handles broadcast writes, and keeps the
/// Mapping alive until the service completion callback has been dispatched.
class RtuSlave final
{
  public:
    using ErrorCode = std::error_code;

    RtuSlave(asio::io_context& io, std::string device, std::shared_ptr<Mapping> mapping,
             SerialOptions serial = {}, SlaveOptions slave = {});
    ~RtuSlave();

    RtuSlave(const RtuSlave&) = delete;
    RtuSlave& operator=(const RtuSlave&) = delete;
    RtuSlave(RtuSlave&&) = delete;
    RtuSlave& operator=(RtuSlave&&) = delete;

    void open();
    /// Initiates close without a completion boundary.
    void close() noexcept;
    /// Completion runs after service/business cleanup and port close.
    void asyncClose(RtuCompletionHandler handler = {});
    void cancel() noexcept;
    [[nodiscard]] SerialState serialState() const;
    [[nodiscard]] bool isOpen() const;
    [[nodiscard]] bool busy() const;
    [[nodiscard]] SerialOptions serialOptions() const;
    void setSerialOptions(const SerialOptions& options);

    /// The completion handler runs once when the service exits.
    void asyncServe(RtuCompletionHandler handler = {});
    void asyncServe(MappingHooks hooks, RtuCompletionHandler handler);
    void stopServe() noexcept;

  private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

} // namespace issac_modbus

#endif // ISSAC_MODBUS_RTU_SLAVE_H
