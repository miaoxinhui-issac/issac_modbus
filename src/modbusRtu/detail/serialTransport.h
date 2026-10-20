#ifndef ISSAC_MODBUS_SERIAL_TRANSPORT_H
#define ISSAC_MODBUS_SERIAL_TRANSPORT_H

#include "issac_modbus/modbusRtu/rtuTypes.h"

#include <asio.hpp>
#include <chrono>
#include <memory>
#include <string>

namespace issac_modbus::detail
{

/// One serial device with at most one active write or read. The owning role
/// state machine is responsible for sequencing operations.
class SerialTransport final
{
  public:
    enum class ClosePhase
    {
        Initiated,
        Waiting,
        AlreadyClosed
    };

    SerialTransport(asio::io_context& io, std::string device, SerialOptions options);
    ~SerialTransport();

    SerialTransport(const SerialTransport&) = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;
    SerialTransport(SerialTransport&&) = delete;
    SerialTransport& operator=(SerialTransport&&) = delete;

    /// Open and configuration are synchronous and allowed only while closed.
    void open();
    void setOptions(const SerialOptions& options);
    [[nodiscard]] SerialOptions options() const;

    /// Close and cancellation post their work to the transport strand.
    void close() noexcept;
    void asyncClose(RtuCompletionHandler handler = {});
    [[nodiscard]] ClosePhase beginClose() noexcept;
    void continueClose(ClosePhase phase, RtuCompletionHandler handler = {});
    void cancel() noexcept;

    [[nodiscard]] SerialState serialState() const noexcept;
    [[nodiscard]] bool isOpen() const noexcept;
    [[nodiscard]] bool busy() const noexcept;
    [[nodiscard]] bool operationActive() const noexcept;
    /// Must be called while closed or from the transport strand with no active
    /// operation. Used by the slave recovery path.
    [[nodiscard]] std::error_code flushInput();
    [[nodiscard]] asio::io_context& io() noexcept;
    [[nodiscard]] const asio::strand<asio::io_context::executor_type>& strand() const noexcept;

    /// Complete CRC-bearing RTU ADU.
    void asyncWrite(RtuBytes adu, bool discardInputBeforeWrite, RtuWriteHandler handler);

    /// Receives one complete RTU ADU and validates CRC before success.
    void asyncRead(RtuFrameType type, std::chrono::milliseconds firstByteTimeout,
                   std::chrono::milliseconds frameTimeout, std::chrono::microseconds byteTimeout,
                   RtuRawHandler handler);

  private:
    struct State;
    std::shared_ptr<State> state_;
};

} // namespace issac_modbus::detail

#endif // ISSAC_MODBUS_SERIAL_TRANSPORT_H
