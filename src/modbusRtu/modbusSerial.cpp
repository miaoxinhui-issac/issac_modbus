#include "issac_modbus/modbusRtu/modbusSerial.h"

#include "detail/serialTransport.h"

#include <utility>

namespace issac_modbus
{

RtuConnection::RtuConnection(asio::io_context& io, std::string device, SerialOptions options)
    : transport_(std::make_shared<detail::SerialTransport>(io, std::move(device), options))
{
}

RtuConnection::~RtuConnection() = default;

void RtuConnection::open() { transport_->open(); }

void RtuConnection::close() noexcept
{
    try
    {
        transport_->asyncClose();
    }
    catch (...)
    {
    }
}

void RtuConnection::asyncClose(RtuCompletionHandler handler)
{
    transport_->asyncClose(std::move(handler));
}

void RtuConnection::cancel() noexcept { transport_->cancel(); }

SerialState RtuConnection::serialState() const noexcept { return transport_->serialState(); }

bool RtuConnection::isOpen() const noexcept { return transport_->isOpen(); }

bool RtuConnection::busy() const noexcept { return transport_->busy(); }

void RtuConnection::setOptions(const SerialOptions& options) { transport_->setOptions(options); }

SerialOptions RtuConnection::options() const { return transport_->options(); }

void RtuConnection::asyncSend(Bytes adu, WriteHandler handler)
{
    transport_->asyncWrite(std::move(adu), false, std::move(handler));
}

void RtuConnection::asyncAwaitRawMessage(FrameType type, RawHandler handler,
                                         std::chrono::milliseconds firstByteTimeout)
{
    const auto serial = transport_->options();
    transport_->asyncRead(type, firstByteTimeout, serial.frameTimeout, serial.byteTimeout,
                          std::move(handler));
}

void RtuConnection::asyncAwaitRawMessage(RawHandler handler,
                                         std::chrono::milliseconds firstByteTimeout)
{
    asyncAwaitRawMessage(FrameType::Response, std::move(handler), firstByteTimeout);
}

} // namespace issac_modbus
