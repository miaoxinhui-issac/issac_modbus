#include "issac_modbus.h"
#include "issac_modbus/util/crc.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <poll.h>
#include <pty.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace
{

using namespace std::chrono_literals;
using issac_modbus::RtuBytes;

int failures = 0;

void require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << std::endl;
        ++failures;
    }
}

RtuBytes withCrc(RtuBytes data)
{
    const auto crc = issac_modbus::calculateCRC(data);
    data.push_back(static_cast<std::uint8_t>(crc));
    data.push_back(static_cast<std::uint8_t>(crc >> 8u));
    return data;
}

class Pty final
{
  public:
    Pty()
    {
        if (::openpty(&master_, &slave_, nullptr, nullptr, nullptr) < 0)
        {
            std::perror("openpty");
            std::abort();
        }
        char path[256]{};
        if (::ptsname_r(master_, path, sizeof(path)) != 0)
        {
            std::perror("ptsname_r");
            std::abort();
        }
        path_ = path;
    }

    ~Pty()
    {
        if (master_ >= 0)
            ::close(master_);
        if (slave_ >= 0)
            ::close(slave_);
    }

    [[nodiscard]] const std::string& path() const { return path_; }
    [[nodiscard]] int masterFd() const { return master_; }

    bool readable(int timeoutMs) const
    {
        pollfd descriptor{master_, POLLIN, 0};
        int result = 0;
        do
        {
            result = ::poll(&descriptor, 1, timeoutMs);
        } while (result < 0 && errno == EINTR);
        return result > 0 && (descriptor.revents & POLLIN) != 0;
    }

    RtuBytes read(std::size_t size) const
    {
        RtuBytes data(size);
        std::size_t offset = 0;
        while (offset < size)
        {
            const auto count = ::read(master_, data.data() + offset, size - offset);
            if (count <= 0)
            {
                require(false, "pty read failed");
                break;
            }
            offset += static_cast<std::size_t>(count);
        }
        return data;
    }

    void write(const RtuBytes& data) const
    {
        std::size_t offset = 0;
        while (offset < data.size())
        {
            const auto count = ::write(master_, data.data() + offset, data.size() - offset);
            if (count <= 0)
            {
                require(false, "pty write failed");
                return;
            }
            offset += static_cast<std::size_t>(count);
        }
    }

  private:
    int master_ = -1;
    int slave_ = -1;
    std::string path_;
};

class Actor final
{
  public:
    explicit Actor(std::function<void()> function)
        : thread_([function = std::move(function)] { function(); })
    {
    }

    ~Actor()
    {
        if (thread_.joinable())
            thread_.join();
    }

    void join()
    {
        if (thread_.joinable())
            thread_.join();
    }

  private:
    std::thread thread_;
};

class Cable final
{
  public:
    Cable(Pty& left, Pty& right) : left_(left), right_(right), thread_([this] { run(); }) {}

    ~Cable()
    {
        stop_.store(true);
        if (thread_.joinable())
            thread_.join();
    }

  private:
    void run()
    {
        while (!stop_.load())
        {
            pollfd descriptors[2]{{left_.masterFd(), POLLIN, 0}, {right_.masterFd(), POLLIN, 0}};
            const auto ready = ::poll(descriptors, 2, 5);
            if (ready <= 0)
                continue;
            std::uint8_t buffer[512];
            for (int index = 0; index < 2; ++index)
            {
                if ((descriptors[index].revents & POLLIN) == 0)
                    continue;
                const auto count = ::read(descriptors[index].fd, buffer, sizeof(buffer));
                if (count <= 0)
                    continue;
                auto& destination = index == 0 ? right_ : left_;
                destination.write(RtuBytes(buffer, buffer + count));
            }
        }
    }

    Pty& left_;
    Pty& right_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

issac_modbus::SerialOptions serialOptions()
{
    issac_modbus::SerialOptions options;
    options.baudRate = 115200;
    options.frameTimeout = 100ms;
    options.writeTimeout = 200ms;
    options.recoveryDelay = 5ms;
    return options;
}

void masterRetryAndMatch()
{
    Pty pty;
    auto serial = serialOptions();
    issac_modbus::MasterOptions masterOptions;
    masterOptions.responseTimeout = 20ms;
    masterOptions.transactionTimeout = 500ms;
    masterOptions.retryPolicy.maxAttempts = 2;
    masterOptions.retryPolicy.retryDelay = 5ms;

    asio::io_context io;
    issac_modbus::RtuMaster master(io, pty.path(), serial, masterOptions);
    master.open();

    std::error_code result = asio::error::would_block;
    std::vector<issac_modbus::ModbusCell> values;
    Actor peer(
        [&]
        {
            const auto first = pty.read(8);
            require(first[1] == 0x03, "first master request is read holding registers");
            const auto second = pty.read(8);
            require(second == first, "retry sends the same request");
            pty.write(withCrc({1, 3, 4, 0, 10, 0, 20}));
        });

    master.asyncReadHoldingRegisters(1, 0, 2,
                                     [&](const std::error_code& error,
                                         std::vector<issac_modbus::ModbusCell> resultValues,
                                         std::optional<issac_modbus::ModbusException>)
                                     {
                                         result = error;
                                         values = std::move(resultValues);
                                     });
    io.run();
    peer.join();

    require(!result, "master retry succeeds");
    require(values.size() == 2 && values[0].reg() == 10 && values[1].reg() == 20,
            "master parses returned registers");
}

void masterRejectsMismatchedPayload()
{
    Pty pty;
    auto serial = serialOptions();
    issac_modbus::MasterOptions masterOptions;
    masterOptions.responseTimeout = 50ms;
    masterOptions.transactionTimeout = 300ms;

    asio::io_context io;
    issac_modbus::RtuMaster master(io, pty.path(), serial, masterOptions);
    master.open();

    std::error_code result;
    Actor peer(
        [&]
        {
            (void)pty.read(8);
            pty.write(withCrc({1, 3, 2, 0, 10}));
        });

    master.asyncReadHoldingRegisters(
        1, 0, 2,
        [&](const std::error_code& error, std::vector<issac_modbus::ModbusCell>,
            std::optional<issac_modbus::ModbusException>) { result = error; });
    io.run();
    peer.join();

    require(result == issac_modbus::makeErrorCode(issac_modbus::MBErrorCode::MBBadFrame),
            "master rejects a response with the wrong register count");
}

void masterQueuesTransactionsFifo()
{
    Pty pty;
    auto serial = serialOptions();
    issac_modbus::MasterOptions masterOptions;
    masterOptions.responseTimeout = 50ms;
    masterOptions.transactionTimeout = 500ms;

    asio::io_context io;
    issac_modbus::RtuMaster master(io, pty.path(), serial, masterOptions);
    master.open();

    std::vector<std::uint16_t> order;
    std::error_code firstError = asio::error::would_block;
    std::error_code secondError = asio::error::would_block;
    std::uint16_t firstValue = 0;
    std::uint16_t secondValue = 0;
    Actor peer(
        [&]
        {
            const auto first = pty.read(8);
            require(first[2] == 0 && first[3] == 0, "first queued request uses address 0");
            pty.write(withCrc({1, 3, 2, 0, 9}));
            const auto second = pty.read(8);
            require(second[2] == 0 && second[3] == 1, "second queued request uses address 1");
            pty.write(withCrc({1, 3, 2, 0, 17}));
        });

    master.asyncReadHoldingRegisters(1, 0, 1,
                                     [&](const std::error_code& error,
                                         std::vector<issac_modbus::ModbusCell> values,
                                         std::optional<issac_modbus::ModbusException>)
                                     {
                                         firstError = error;
                                         if (!values.empty())
                                             firstValue = values[0].reg();
                                         order.push_back(1);
                                     });
    master.asyncReadHoldingRegisters(1, 1, 1,
                                     [&](const std::error_code& error,
                                         std::vector<issac_modbus::ModbusCell> values,
                                         std::optional<issac_modbus::ModbusException>)
                                     {
                                         secondError = error;
                                         if (!values.empty())
                                             secondValue = values[0].reg();
                                         order.push_back(2);
                                     });
    io.run();
    peer.join();

    require(!firstError && !secondError, "both queued transactions succeed");
    require(firstValue == 9 && secondValue == 17,
            "queued transactions receive their own responses");
    require(order == std::vector<std::uint16_t>{1, 2}, "master completion order follows FIFO");
}

void masterCancelAndReuse()
{
    Pty pty;
    auto serial = serialOptions();
    issac_modbus::MasterOptions masterOptions;
    masterOptions.responseTimeout = 500ms;
    masterOptions.transactionTimeout = 1s;

    asio::io_context io;
    auto workGuard = asio::make_work_guard(io);
    issac_modbus::RtuMaster master(io, pty.path(), serial, masterOptions);
    master.open();

    std::atomic<bool> firstRequestRead{false};
    std::atomic<bool> firstDone{false};
    std::error_code firstError = asio::error::would_block;
    std::error_code secondError = asio::error::would_block;
    std::uint16_t secondValue = 0;
    Actor peer(
        [&]
        {
            (void)pty.read(8);
            firstRequestRead.store(true, std::memory_order_release);
            const auto second = pty.read(8);
            require(second[1] == 0x03, "reused master sends a second request");
            pty.write(withCrc({1, 3, 2, 0, 77}));
        });
    std::thread ioThread([&] { io.run(); });

    master.asyncReadHoldingRegisters(1, 0, 1,
                                     [&](const std::error_code& error,
                                         std::vector<issac_modbus::ModbusCell>,
                                         std::optional<issac_modbus::ModbusException>)
                                     {
                                         firstError = error;
                                         firstDone.store(true, std::memory_order_release);
                                     });

    const auto waitStart = std::chrono::steady_clock::now();
    while (!firstRequestRead.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() - waitStart < 500ms)
    {
        std::this_thread::sleep_for(1ms);
    }
    require(firstRequestRead.load(std::memory_order_acquire),
            "master sends the request before cancellation");
    master.cancel();

    while (!firstDone.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() - waitStart < 1s)
    {
        std::this_thread::sleep_for(1ms);
    }
    require(firstDone.load(std::memory_order_acquire),
            "master cancellation completes the active transaction");
    require(firstError == asio::error::operation_aborted,
            "master cancellation reports operation_aborted");

    master.asyncReadHoldingRegisters(1, 0, 1,
                                     [&](const std::error_code& error,
                                         std::vector<issac_modbus::ModbusCell> values,
                                         std::optional<issac_modbus::ModbusException>)
                                     {
                                         secondError = error;
                                         if (!values.empty())
                                             secondValue = values[0].reg();
                                         workGuard.reset();
                                     });

    peer.join();
    if (ioThread.joinable())
        ioThread.join();

    require(!secondError, "master accepts a transaction after cancellation");
    require(secondValue == 77, "transaction after cancellation receives its response");
}

void masterCancelBeforeIoRuns()
{
    Pty pty;
    auto serial = serialOptions();
    issac_modbus::MasterOptions masterOptions;
    masterOptions.responseTimeout = 50ms;

    asio::io_context io;
    issac_modbus::RtuMaster master(io, pty.path(), serial, masterOptions);
    master.open();

    std::error_code result = asio::error::would_block;
    std::atomic<bool> peerReceived{false};
    Actor peer(
        [&]
        {
            if (pty.readable(80))
            {
                peerReceived.store(true, std::memory_order_release);
                (void)pty.read(8);
            }
        });

    master.asyncReadHoldingRegisters(
        1, 0, 1,
        [&](const std::error_code& error, std::vector<issac_modbus::ModbusCell>,
            std::optional<issac_modbus::ModbusException>) { result = error; });
    master.cancel();
    io.run_for(100ms);
    peer.join();

    require(result == asio::error::operation_aborted,
            "pre-run master cancellation completes the transaction");
    require(!peerReceived.load(std::memory_order_acquire),
            "pre-run master cancellation prevents the posted write from starting");
}

void masterTransactionDeadlineCoversRetryDelay()
{
    Pty pty;
    auto serial = serialOptions();
    issac_modbus::MasterOptions masterOptions;
    masterOptions.responseTimeout = 15ms;
    masterOptions.transactionTimeout = 40ms;
    masterOptions.retryPolicy.maxAttempts = 5;
    masterOptions.retryPolicy.retryDelay = 500ms;

    asio::io_context io;
    issac_modbus::RtuMaster master(io, pty.path(), serial, masterOptions);
    master.open();

    std::error_code result;
    Actor peer([&] { (void)pty.read(8); });
    const auto start = std::chrono::steady_clock::now();
    master.asyncReadHoldingRegisters(
        1, 0, 1,
        [&](const std::error_code& error, std::vector<issac_modbus::ModbusCell>,
            std::optional<issac_modbus::ModbusException>) { result = error; });
    io.run();
    peer.join();
    const auto elapsed = std::chrono::steady_clock::now() - start;

    require(result == asio::error::timed_out, "transaction deadline fires while waiting to retry");
    require(elapsed < 200ms, "transaction deadline is not postponed by retry delay");
}

void masterCloseDuringRetryDelay()
{
    Pty pty;
    auto serial = serialOptions();
    issac_modbus::MasterOptions masterOptions;
    masterOptions.responseTimeout = 15ms;
    masterOptions.transactionTimeout = 2s;
    masterOptions.retryPolicy.maxAttempts = 5;
    masterOptions.retryPolicy.retryDelay = 500ms;

    asio::io_context io;
    auto workGuard = asio::make_work_guard(io);
    issac_modbus::RtuMaster master(io, pty.path(), serial, masterOptions);
    master.open();

    std::atomic<bool> requestRead{false};
    std::error_code result;
    Actor peer(
        [&]
        {
            (void)pty.read(8);
            requestRead.store(true, std::memory_order_release);
        });
    std::thread ioThread([&] { io.run(); });

    master.asyncReadHoldingRegisters(1, 0, 1,
                                     [&](const std::error_code& error,
                                         std::vector<issac_modbus::ModbusCell>,
                                         std::optional<issac_modbus::ModbusException>)
                                     {
                                         result = error;
                                         workGuard.reset();
                                     });

    const auto start = std::chrono::steady_clock::now();
    while (!requestRead.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() - start < 500ms)
    {
        std::this_thread::sleep_for(1ms);
    }
    require(requestRead.load(std::memory_order_acquire),
            "master request is sent before close during retry wait");
    std::this_thread::sleep_for(30ms);
    master.close();

    peer.join();
    if (ioThread.joinable())
        ioThread.join();
    require(result == asio::error::operation_aborted,
            "master close completes a transaction waiting for retry");
}

void masterAsyncCloseBoundary()
{
    Pty pty;
    asio::io_context io;
    issac_modbus::RtuMaster master(io, pty.path());
    master.open();

    std::vector<int> completionOrder;
    std::atomic<bool> peerReceived{false};
    bool closedInCallback = false;
    bool reopenedInCallback = false;
    std::error_code closeError = asio::error::would_block;
    Actor peer(
        [&]
        {
            if (pty.readable(80))
            {
                peerReceived.store(true, std::memory_order_release);
                (void)pty.read(8);
            }
        });

    master.asyncReadHoldingRegisters(
        1, 0, 1,
        [&](const std::error_code&, std::vector<issac_modbus::ModbusCell>,
            std::optional<issac_modbus::ModbusException>) { completionOrder.push_back(1); });
    master.asyncClose(
        [&](const std::error_code& error)
        {
            closeError = error;
            completionOrder.push_back(2);
            closedInCallback = master.serialState() == issac_modbus::SerialState::Closed;
            master.open();
            reopenedInCallback = master.serialState() == issac_modbus::SerialState::Open;
            io.stop();
        });
    require(master.serialState() == issac_modbus::SerialState::Closing,
            "master enters Closing when asyncClose is requested");
    io.run();
    peer.join();

    require(!closeError, "master asyncClose completes successfully");
    require(!peerReceived.load(std::memory_order_acquire),
            "master asyncClose prevents a pending request from starting");
    require(completionOrder == std::vector<int>{1, 2},
            "master transaction completion precedes asyncClose completion");
    require(closedInCallback, "master asyncClose callback observes Closed state");
    require(reopenedInCallback, "master can reopen from asyncClose completion");
}

void multipleAsyncCloseCompletions()
{
    Pty pty;
    asio::io_context io;
    issac_modbus::RtuMaster master(io, pty.path());
    master.open();

    int completions = 0;
    bool bothSawClosed = true;
    bool reopened = false;
    master.asyncClose(
        [&](const std::error_code& error)
        {
            require(!error, "first master asyncClose succeeds");
            bothSawClosed =
                bothSawClosed && master.serialState() == issac_modbus::SerialState::Closed;
            if (++completions == 2)
            {
                master.open();
                reopened = master.serialState() == issac_modbus::SerialState::Open;
                io.stop();
            }
        });
    master.asyncClose(
        [&](const std::error_code& error)
        {
            require(!error, "second master asyncClose succeeds");
            bothSawClosed =
                bothSawClosed && master.serialState() == issac_modbus::SerialState::Closed;
            if (++completions == 2)
            {
                master.open();
                reopened = master.serialState() == issac_modbus::SerialState::Open;
                io.stop();
            }
        });
    io.run();

    require(completions == 2, "all master asyncClose callers complete");
    require(bothSawClosed, "all close callbacks observe Closed state");
    require(reopened, "master can reopen after coalesced close completion");
}

void slaveAddressBroadcastAndResponse()
{
    Pty pty;
    auto mapping = std::make_shared<issac_modbus::Mapping>(0, 0, 2, 0);
    mapping->holdingRegisters[0] = 1;

    asio::io_context io;
    issac_modbus::SlaveOptions slaveOptions;
    slaveOptions.unitId = 7;
    issac_modbus::RtuSlave slave(io, pty.path(), mapping, serialOptions(), slaveOptions);
    slave.open();

    std::atomic<int> completion{0};
    std::error_code completionError;
    slave.asyncServe(
        [&](const std::error_code& error)
        {
            completionError = error;
            completion.fetch_add(1);
        });

    std::atomic<bool> ioDone{false};
    std::thread ioThread(
        [&]
        {
            io.run();
            ioDone.store(true);
        });

    Actor peer(
        [&]
        {
            pty.write(withCrc({8, 3, 0, 0, 0, 1}));
            require(!pty.readable(30), "slave ignores another unit address");

            pty.write(withCrc({7, 6, 0, 0, 0, 42}));
            require(pty.read(8) == withCrc({7, 6, 0, 0, 0, 42}),
                    "slave echoes a single-register write");

            pty.write(withCrc({0, 6, 0, 1, 0, 55}));
            require(!pty.readable(30), "slave does not answer a broadcast");

            pty.write(withCrc({7, 0x45}));
            require(pty.read(5) == withCrc({7, 0xC5, 0x01}),
                    "slave reports an unsupported function");
        });

    peer.join();
    std::this_thread::sleep_for(20ms);
    require(mapping->holdingRegisters[0] == 42, "addressed write updates the mapping");
    require(mapping->holdingRegisters[1] == 55, "broadcast write updates the mapping");
    slave.stopServe();
    if (ioThread.joinable())
        ioThread.join();

    require(ioDone.load(), "slave io_context stops after service cancellation");
    require(completion.load() == 1, "slave completion is delivered exactly once");
    require(completionError == asio::error::operation_aborted,
            "slave cancellation returns operation_aborted");
}

void slaveStopAndRestartBeforeIoRuns()
{
    Pty pty;
    auto mapping = std::make_shared<issac_modbus::Mapping>(0, 0, 1, 0);
    asio::io_context io;
    issac_modbus::SlaveOptions slaveOptions;
    slaveOptions.unitId = 1;
    issac_modbus::RtuSlave slave(io, pty.path(), mapping, serialOptions(), slaveOptions);
    slave.open();

    std::atomic<int> firstCompletions{0};
    std::error_code firstError;
    slave.asyncServe(
        [&](const std::error_code& error)
        {
            firstError = error;
            firstCompletions.fetch_add(1);
        });
    slave.stopServe();
    io.run_for(100ms);

    require(firstCompletions.load() == 1, "pre-run slave stop completes the first service");
    require(firstError == asio::error::operation_aborted,
            "pre-run slave stop reports operation_aborted");
    require(!slave.busy(), "slave is not busy after the first service exits");

    io.restart();
    std::atomic<int> secondCompletions{0};
    std::error_code secondError;
    slave.asyncServe(
        [&](const std::error_code& error)
        {
            secondError = error;
            secondCompletions.fetch_add(1);
        });
    asio::steady_timer timer(io, 10ms);
    timer.async_wait([&](const std::error_code&) { slave.stopServe(); });
    io.run();

    require(secondCompletions.load() == 1, "slave can be restarted after pre-run cancellation");
    require(secondError == asio::error::operation_aborted,
            "restarted slave stops with operation_aborted");
}

void slaveAsyncCloseBoundary()
{
    Pty pty;
    auto mapping = std::make_shared<issac_modbus::Mapping>(0, 0, 1, 0);
    asio::io_context io;
    issac_modbus::RtuSlave slave(io, pty.path(), mapping);
    slave.open();

    std::vector<int> completionOrder;
    bool closedInCallback = false;
    bool reopenedInCallback = false;
    std::error_code closeError = asio::error::would_block;
    slave.asyncServe([&](const std::error_code&) { completionOrder.push_back(1); });
    slave.asyncClose(
        [&](const std::error_code& error)
        {
            closeError = error;
            completionOrder.push_back(2);
            closedInCallback = slave.serialState() == issac_modbus::SerialState::Closed;
            slave.open();
            reopenedInCallback = slave.serialState() == issac_modbus::SerialState::Open;
            io.stop();
        });
    require(slave.serialState() == issac_modbus::SerialState::Closing,
            "slave enters Closing when asyncClose is requested");
    io.run();

    require(!closeError, "slave asyncClose completes successfully");
    require(completionOrder == std::vector<int>{1, 2},
            "slave service completion precedes asyncClose completion");
    require(closedInCallback, "slave asyncClose callback observes Closed state");
    require(reopenedInCallback, "slave can reopen from asyncClose completion");
}

void frameGapDelaysNormalResponse()
{
    Pty pty;
    auto serial = serialOptions();
    serial.frameGap = 200ms;
    auto mapping = std::make_shared<issac_modbus::Mapping>(0, 0, 1, 0);
    mapping->holdingRegisters[0] = 12;

    asio::io_context io;
    issac_modbus::RtuSlave slave(io, pty.path(), mapping, serial);
    slave.open();
    std::atomic<int> completions{0};
    slave.asyncServe([&](const std::error_code&) { completions.fetch_add(1); });
    std::thread ioThread([&] { io.run(); });

    const auto start = std::chrono::steady_clock::now();
    pty.write(withCrc({1, 3, 0, 0, 0, 1}));
    require(pty.readable(1000), "slave responds after the configured frame gap");
    const auto response = pty.read(7);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    require(response == withCrc({1, 3, 2, 0, 12}), "frame-gap response contains the register");
    require(elapsed >= 180ms, "normal response waits for frameGap after receiving the request");

    slave.stopServe();
    if (ioThread.joinable())
        ioThread.join();
    require(completions.load() == 1, "frame-gap slave service exits once");
}

void slaveDestructionWhileServing()
{
    Pty pty;
    auto mapping = std::make_shared<issac_modbus::Mapping>(0, 0, 1, 0);
    asio::io_context io;
    auto slave = std::make_unique<issac_modbus::RtuSlave>(io, pty.path(), mapping, serialOptions());
    slave->open();

    std::atomic<int> completions{0};
    std::error_code completionError;
    slave->asyncServe(
        [&](const std::error_code& error)
        {
            completionError = error;
            completions.fetch_add(1);
        });
    std::thread ioThread([&] { io.run(); });
    std::this_thread::sleep_for(10ms);
    slave.reset();
    if (ioThread.joinable())
        ioThread.join();

    require(completions.load() == 1, "slave destruction completes the service once");
    require(completionError == asio::error::operation_aborted,
            "slave destruction reports operation_aborted");
}

void slaveWaitsForDispatchedBusinessOnStop()
{
    Pty pty;
    auto mapping = std::make_shared<issac_modbus::Mapping>(0, 0, 1, 0);
    mapping->holdingRegisters[0] = 5;

    asio::io_context io;
    issac_modbus::SlaveOptions slaveOptions;
    slaveOptions.unitId = 4;
    issac_modbus::RtuSlave slave(io, pty.path(), mapping, serialOptions(), slaveOptions);
    slave.open();

    std::atomic<bool> dispatched{false};
    std::function<void()> pendingWork;
    issac_modbus::MappingHooks hooks;
    hooks.dispatch = [&](std::function<void()> work)
    {
        pendingWork = std::move(work);
        dispatched.store(true, std::memory_order_release);
    };

    std::atomic<int> completions{0};
    std::error_code completionError;
    slave.asyncServe(std::move(hooks),
                     [&](const std::error_code& error)
                     {
                         completionError = error;
                         completions.fetch_add(1);
                     });
    std::thread ioThread([&] { io.run(); });

    pty.write(withCrc({4, 3, 0, 0, 0, 1}));
    const auto waitStart = std::chrono::steady_clock::now();
    while (!dispatched.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() - waitStart < 500ms)
    {
        std::this_thread::sleep_for(1ms);
    }
    require(dispatched.load(std::memory_order_acquire), "slave dispatches business work");

    slave.stopServe();
    std::this_thread::sleep_for(20ms);
    require(completions.load() == 0,
            "slave completion waits while dispatched business is still pending");

    require(static_cast<bool>(pendingWork), "dispatched slave work is available");
    pendingWork();
    if (ioThread.joinable())
        ioThread.join();

    require(completions.load() == 1, "slave completes after dispatched work returns");
    require(completionError == asio::error::operation_aborted,
            "dispatched slave stop preserves the cancellation error");
}

void masterSlaveRoundTrip()
{
    Pty left;
    Pty right;
    auto mapping = std::make_shared<issac_modbus::Mapping>(0, 0, 4, 0);
    mapping->holdingRegisters = {11, 22, 33, 44};

    asio::io_context io;
    auto serial = serialOptions();
    issac_modbus::SlaveOptions slaveOptions;
    slaveOptions.unitId = 1;
    issac_modbus::RtuSlave slave(io, left.path(), mapping, serial, slaveOptions);
    issac_modbus::MasterOptions masterOptions;
    masterOptions.responseTimeout = 100ms;
    masterOptions.transactionTimeout = 1s;
    issac_modbus::RtuMaster master(io, right.path(), serial, masterOptions);
    slave.open();
    master.open();
    Cable cable(left, right);

    int slaveCompletions = 0;
    std::error_code slaveError;
    slave.asyncServe(
        [&](const std::error_code& error)
        {
            slaveError = error;
            ++slaveCompletions;
        });

    std::error_code masterError = asio::error::would_block;
    std::vector<issac_modbus::ModbusCell> masterValues;
    master.asyncReadHoldingRegisters(1, 0, 4,
                                     [&](const std::error_code& error,
                                         std::vector<issac_modbus::ModbusCell> values,
                                         std::optional<issac_modbus::ModbusException>)
                                     {
                                         masterError = error;
                                         masterValues = std::move(values);
                                         slave.stopServe();
                                     });
    io.run();

    require(!masterError, "typed master and slave exchange a request");
    require(masterValues.size() == 4 && masterValues[0].reg() == 11 && masterValues[3].reg() == 44,
            "typed master receives the slave registers");
    require(slaveCompletions == 1, "typed slave exits once");
    require(slaveError == asio::error::operation_aborted,
            "typed slave stop reports operation_aborted");
}

void manualTotalFrameDeadline()
{
    Pty pty;
    auto serial = serialOptions();
    serial.frameTimeout = 25ms;
    serial.byteTimeout = 10ms;

    asio::io_context io;
    issac_modbus::RtuConnection manual(io, pty.path(), serial);
    manual.open();

    std::error_code result;
    Actor peer(
        [&]
        {
            for (int i = 0; i < 16; ++i)
            {
                pty.write({static_cast<std::uint8_t>(1)});
                std::this_thread::sleep_for(5ms);
            }
        });

    manual.asyncAwaitRawMessage([&](const std::error_code& error, RtuBytes) { result = error; },
                                100ms);
    io.run();
    peer.join();

    require(result == issac_modbus::makeErrorCode(issac_modbus::MBErrorCode::MBFrameTimeout),
            "manual receive enforces a total frame deadline");
}

void manualRejectsOversizeHeader()
{
    Pty pty;
    asio::io_context io;
    issac_modbus::RtuConnection manual(io, pty.path());
    manual.open();

    std::error_code result;
    Actor peer([&] { pty.write({1, 3, 252}); });
    manual.asyncAwaitRawMessage([&](const std::error_code& error, RtuBytes) { result = error; });
    io.run();
    peer.join();

    require(result == asio::error::message_size,
            "manual receive rejects a header larger than one RTU ADU");
}

void manualReceivesRequestFrame()
{
    Pty pty;
    asio::io_context io;
    issac_modbus::RtuConnection manual(io, pty.path());
    manual.open();

    const auto request = withCrc({1, 3, 0, 0, 0, 1});
    RtuBytes received;
    std::error_code result = asio::error::would_block;
    Actor peer([&] { pty.write(request); });
    manual.asyncAwaitRawMessage(issac_modbus::RtuConnection::FrameType::Request,
                                [&](const std::error_code& error, RtuBytes data)
                                {
                                    result = error;
                                    received = std::move(data);
                                });
    io.run();
    peer.join();

    require(!result, "manual request receive succeeds");
    require(received == request, "manual request receive returns all eight request bytes");
}

void manualRejectsConcurrentOperations()
{
    Pty pty;
    asio::io_context io;
    auto workGuard = asio::make_work_guard(io);
    issac_modbus::RtuConnection manual(io, pty.path());
    manual.open();

    std::error_code sendError = asio::error::would_block;
    std::error_code receiveError = asio::error::would_block;
    manual.asyncAwaitRawMessage(
        [&](const std::error_code& error, RtuBytes)
        {
            receiveError = error;
            workGuard.reset();
        },
        500ms);
    std::thread ioThread([&] { io.run(); });
    std::this_thread::sleep_for(10ms);

    manual.asyncSend(withCrc({1, 3, 0, 0, 0, 1}),
                     [&](const std::error_code& error, std::size_t)
                     {
                         sendError = error;
                         manual.cancel();
                     });

    if (ioThread.joinable())
        ioThread.join();
    require(sendError ==
                issac_modbus::makeErrorCode(issac_modbus::MBErrorCode::MBConcurrentOperation),
            "manual channel rejects a second concurrent operation");
    require(receiveError == asio::error::operation_aborted,
            "manual receive can be cancelled after a rejected concurrent write");
}

} // namespace

int main()
{
    const std::pair<const char*, void (*)()> tests[] = {
        {"master retry and match", masterRetryAndMatch},
        {"master rejects mismatched payload", masterRejectsMismatchedPayload},
        {"master queues transactions FIFO", masterQueuesTransactionsFifo},
        {"master cancel and reuse", masterCancelAndReuse},
        {"master cancel before IO runs", masterCancelBeforeIoRuns},
        {"master transaction deadline", masterTransactionDeadlineCoversRetryDelay},
        {"master close during retry", masterCloseDuringRetryDelay},
        {"master async close boundary", masterAsyncCloseBoundary},
        {"multiple async close completions", multipleAsyncCloseCompletions},
        {"slave address and broadcast", slaveAddressBroadcastAndResponse},
        {"slave stop and restart", slaveStopAndRestartBeforeIoRuns},
        {"slave async close boundary", slaveAsyncCloseBoundary},
        {"frame gap response timing", frameGapDelaysNormalResponse},
        {"slave destruction while serving", slaveDestructionWhileServing},
        {"slave dispatched business stop", slaveWaitsForDispatchedBusinessOnStop},
        {"master slave round trip", masterSlaveRoundTrip},
        {"manual total frame deadline", manualTotalFrameDeadline},
        {"manual oversize header", manualRejectsOversizeHeader},
        {"manual request frame", manualReceivesRequestFrame},
        {"manual concurrent rejection", manualRejectsConcurrentOperations},
    };
    for (const auto& [name, test] : tests)
    {
        std::cout << "RUN " << name << std::endl;
        test();
    }

    if (failures != 0)
    {
        std::cerr << failures << " RTU test(s) failed" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "All RTU role tests passed" << std::endl;
    return EXIT_SUCCESS;
}
