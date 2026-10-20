#ifndef ISSAC_MODBUS_SERIAL_H
#define ISSAC_MODBUS_SERIAL_H

#include "issac_modbus/modbusRtu/rtuTypes.h"

#include <asio.hpp>
#include <chrono>
#include <memory>
#include <string>

namespace issac_modbus
{
namespace detail
{
class SerialTransport;
}

/// Manual RTU frame channel. It performs one write or one receive at a time;
/// master transactions and slave serving belong to RtuMaster and RtuSlave.
class RtuConnection final
{
  public:
    using Bytes = RtuBytes;
    using ErrorCode = std::error_code;
    using WriteHandler = RtuWriteHandler;
    using RawHandler = RtuRawHandler;
    using Parity = issac_modbus::Parity;
    using StopBits = issac_modbus::StopBits;
    using DataBits = issac_modbus::DataBits;
    using FlowControl = issac_modbus::FlowControl;
    using SerialMode = issac_modbus::SerialMode;
    using Rs485Options = issac_modbus::Rs485Options;
    using FrameType = issac_modbus::RtuFrameType;

    explicit RtuConnection(asio::io_context& io, std::string device, SerialOptions options = {});
    ~RtuConnection();

    RtuConnection(const RtuConnection&) = delete;
    RtuConnection& operator=(const RtuConnection&) = delete;
    RtuConnection(RtuConnection&&) = delete;
    RtuConnection& operator=(RtuConnection&&) = delete;

    void open();
    void close() noexcept;
    void cancel() noexcept;
    [[nodiscard]] bool isOpen() const noexcept;
    [[nodiscard]] bool busy() const noexcept;
    void setOptions(const SerialOptions& options);
    [[nodiscard]] SerialOptions options() const;

    /// Sends one complete RTU ADU including CRC.
    void asyncSend(Bytes adu, WriteHandler handler);

    /// Receives one complete request or response ADU and validates CRC.
    void asyncAwaitRawMessage(FrameType type, RawHandler handler,
                              std::chrono::milliseconds firstByteTimeout = DefaultResponseTimeout);

    /// Compatibility convenience for response frames.
    void asyncAwaitRawMessage(RawHandler handler,
                              std::chrono::milliseconds firstByteTimeout = DefaultResponseTimeout);

  private:
    std::shared_ptr<detail::SerialTransport> transport_;
};

} // namespace issac_modbus

#endif // ISSAC_MODBUS_SERIAL_H
