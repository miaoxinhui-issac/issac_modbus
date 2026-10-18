#ifndef MODBUS_SERIAL_H
#define MODBUS_SERIAL_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#include <asio.hpp>

#include "issac_modbus/util/modbusException.h"
#include "issac_modbus/util/modbusMapping.h"
#include "issac_modbus/util/modbusRequest.h"
#include "issac_modbus/util/modbusResponse.h"
#include "issac_modbus/util/modbusTransaction.h"

namespace issac_modbus
{

/// @brief 基于 standalone Asio 的 Modbus RTU 串口连接。
class RtuConnection
{
  public:
    using Bytes = std::vector<std::uint8_t>;
    using ErrorCode = std::error_code;
    using WriteHandler = std::function<void(const ErrorCode&, std::size_t)>;
    using RawHandler = std::function<void(const ErrorCode&, Bytes)>;
    using RequestHandler = std::function<void(const ErrorCode&, ModbusRequest)>;
    using ResponseHandler = std::function<void(const ErrorCode&, ModbusResponse)>;
    using ValuesHandler = std::function<void(const ErrorCode&, std::vector<ModbusCell>,
                                             std::optional<ModbusException>)>;

    /// @brief 串口校验方式。
    enum class Parity
    {
        None,
        Even,
        Odd
    };

    /// @brief 串口停止位数量。
    enum class StopBits
    {
        One,
        Two
    };

    enum class DataBits
    {
        Five = 5,
        Six = 6,
        Seven = 7,
        Eight = 8
    };
    enum class FlowControl
    {
        None,
        Software,
        Hardware
    };
    enum class SerialMode
    {
        RS232,
        RS485
    };

    struct Rs485Options
    {
        bool enabled = false;
        bool rtsOnSend = false;
        bool rtsAfterSend = true;
        std::uint32_t delayBeforeSendMs = 0;
        std::uint32_t delayAfterSendMs = 0;
        bool receiveDuringSend = false;
    };

    static constexpr std::chrono::milliseconds DefaultSerialTimeout{100};
    static constexpr std::chrono::milliseconds DefaultByteTimeout{20};

    /// @brief 创建一个 RTU 串口连接。
    /// @param ioContext Asio 事件循环，由调用方负责运行。
    /// @param devicePath 串口设备路径，例如 /dev/ttyS0。
    /// @param baudRate 波特率。
    explicit RtuConnection(asio::io_context& ioContext, std::string devicePath,
                        unsigned int baudRate = 9600);

    /// @brief 关闭串口。
    ~RtuConnection();

    /// @brief 禁止复制构造串口连接。
    RtuConnection(const RtuConnection&) = delete;

    /// @brief 禁止复制赋值串口连接。
    RtuConnection& operator=(const RtuConnection&) = delete;

    /// @brief 禁止移动构造串口连接。
    RtuConnection(RtuConnection&&) = delete;

    /// @brief 禁止移动赋值串口连接。
    RtuConnection& operator=(RtuConnection&&) = delete;

    /// @brief 打开串口并应用配置。
    /// @throws std::system_error 打开或配置失败时抛出。
    void open();

    /// @brief 关闭串口。
    void close() noexcept;

    /// @brief 判断串口是否已经打开。
    [[nodiscard]] bool isOpen() const noexcept;
    /// @brief 获取串口原生文件描述符，未打开时返回 -1。
    [[nodiscard]] int nativeHandle() noexcept
    {
        return serialPort_.is_open() ? serialPort_.native_handle() : -1;
    }

    /// @brief 取消当前异步读写，并丢弃尚未处理的数据。
    void clearInput() noexcept;

    /// @brief 异步发送 Modbus 请求。
    /// @param request 请求对象。
    /// @param handler 完成回调，参数为错误码和写入字节数。
    void asyncSendRequest(const ModbusRequest& request, WriteHandler handler);

    /// @brief 异步发送 Modbus 响应。
    void asyncSendResponse(const ModbusResponse& response, WriteHandler handler);

    /// @brief 异步发送 Modbus 异常响应。
    void asyncSendException(const ModbusException& exception, WriteHandler handler);

    /// @brief 异步发送已经构造好的完整 RTU 帧。
    void asyncSend(Bytes data, WriteHandler handler);

    /// @brief 异步接收一个完整 RTU 响应帧，包含 CRC。
    void asyncAwaitRawMessage(RawHandler handler);

    /// @brief 异步接收并解析一个 Modbus 请求。
    void asyncAwaitRequest(RequestHandler handler);

    /// @brief 异步接收并解析一个 Modbus 响应。
    void asyncAwaitResponse(ResponseHandler handler);

    /// @brief 发送一个请求，并严格匹配一个响应。
    void asyncTransaction(const ModbusRequest& request, TransactionHandler handler,
                          RetryPolicy policy = {});

    /// @brief 异步读取线圈。
    void asyncReadCoils(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                        ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步读取离散输入。
    void asyncReadDiscreteInputs(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                 ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步读取保持寄存器。
    void asyncReadHoldingRegisters(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                   ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步读取输入寄存器。
    void asyncReadInputRegisters(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                                 ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步写入单个线圈。
    void asyncWriteSingleCoil(std::uint8_t unit, std::uint16_t address, bool value,
                              ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步写入单个保持寄存器。
    void asyncWriteSingleRegister(std::uint8_t unit, std::uint16_t address, std::uint16_t value,
                                  ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步写入多个保持寄存器。
    void asyncWriteMultipleRegisters(std::uint8_t unit, std::uint16_t address,
                                     const std::vector<std::uint16_t>& values,
                                     ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步写入多个线圈。
    void asyncWriteMultipleCoils(std::uint8_t unit, std::uint16_t address,
                                 const std::vector<bool>& values, ValuesHandler handler,
                                 RetryPolicy policy = {});

    /// @brief 异步执行保持寄存器掩码写。
    void asyncMaskWriteRegister(std::uint8_t unit, std::uint16_t address, std::uint16_t andMask,
                                std::uint16_t orMask, ValuesHandler handler,
                                RetryPolicy policy = {});

    /// @brief 异步执行读写多个保持寄存器操作。
    void asyncReadWriteMultipleRegisters(std::uint8_t unit, std::uint16_t readAddress,
                                         std::uint16_t readCount, std::uint16_t writeAddress,
                                         const std::vector<std::uint16_t>& values,
                                         ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 异步读取从站标识信息。
    void asyncReportSlaveId(std::uint8_t unit, ValuesHandler handler, RetryPolicy policy = {});

    /// @brief 在已经打开的串口上运行基于 Mapping 的 RTU 从站。
    void asyncServe(Mapping& mapping, std::function<void(const ErrorCode&)> handler = {});

    /// @brief 使用业务钩子在已经打开的串口上运行 RTU 从站。
    void asyncServe(Mapping& mapping, MappingHooks hooks,
                    std::function<void(const ErrorCode&)> handler = {});

    /// @brief 设置读写超时。
    void setTimeout(std::chrono::milliseconds timeout) noexcept { timeout_ = timeout; }

    /// @brief 获取读写超时。
    [[nodiscard]] std::chrono::milliseconds timeout() const noexcept { return timeout_; }

    /// @brief 设置波特率；串口已打开时立即生效。
    void setBaudRate(unsigned int baudRate);

    /// @brief 设置校验方式；串口已打开时立即生效。
    void setParity(Parity parity);

    /// @brief 设置停止位；串口已打开时立即生效。
    void setStopBits(StopBits stopBits);

    /// @brief 设置数据位；串口已打开时立即生效。
    void setDataBits(DataBits dataBits);

    /// @brief 设置流控模式；串口已打开时立即生效。
    void setFlowControl(FlowControl flowControl);

    /// @brief 设置串口电气模式；串口已打开时立即生效。
    void setSerialMode(SerialMode mode);

    /// @brief 设置 RS485 方向和延时参数；串口已打开时立即生效。
    void setRs485Options(const Rs485Options& options);

    /// @brief 获取当前数据位配置。
    [[nodiscard]] DataBits dataBits() const noexcept { return dataBits_; }

    /// @brief 获取当前流控配置。
    [[nodiscard]] FlowControl flowControl() const noexcept { return flowControl_; }

    /// @brief 获取当前串口电气模式。
    [[nodiscard]] SerialMode serialMode() const noexcept { return serialMode_; }

    /// @brief 获取当前 RS485 配置。
    [[nodiscard]] const Rs485Options& rs485Options() const noexcept { return rs485_; }

    /// @brief 清理 Linux 串口驱动中的输入缓冲区。
    void flushInput() noexcept;

    /// @brief 设置帧内字节间超时时间。
    void setByteTimeout(std::chrono::milliseconds timeout) noexcept { byteTimeout_ = timeout; }

    /// @brief 获取帧内字节间超时时间。
    [[nodiscard]] std::chrono::milliseconds byteTimeout() const noexcept { return byteTimeout_; }

  private:
    using FrameLengthResolver = std::function<std::optional<std::size_t>(const Bytes&)>;

    struct ReadState;

    struct PendingWrite
    {
        Bytes data;
        WriteHandler handler;
    };

    /// @brief 把当前串口参数应用到原生串口。
    void applySerialOptions();

    /// @brief 根据解析器确定帧长度并等待一个完整帧。
    void asyncAwaitFrame(const FrameLengthResolver& resolver, RawHandler handler);

    /// @brief 读取 RTU 帧的下一个字节。
    void readNextByte(const std::shared_ptr<ReadState>& state);

    /// @brief 启动写队列中的下一个发送任务。
    void startNextWrite();

    asio::io_context& ioContext_;
    asio::serial_port serialPort_;
    std::string devicePath_;
    unsigned int baudRate_;
    Parity parity_ = Parity::None;
    StopBits stopBits_ = StopBits::One;
    DataBits dataBits_ = DataBits::Eight;
    FlowControl flowControl_ = FlowControl::None;
    SerialMode serialMode_ = SerialMode::RS232;
    Rs485Options rs485_{};
    std::chrono::milliseconds timeout_ = DefaultSerialTimeout;
    std::chrono::milliseconds byteTimeout_ = DefaultByteTimeout;
    RetryPolicy retryPolicy_{};
    bool transactionInProgress_ = false;
    std::deque<PendingWrite> writeQueue_;
    bool writeInProgress_ = false;
};

} // namespace issac_modbus

#endif // MODBUS_SERIAL_H
