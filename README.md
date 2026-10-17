# issac_modbus

基于 C++17 和 standalone Asio 的异步 Modbus RTU/TCP 库，包含：

- Modbus RTU 串口异步收发和 CRC16 校验
- Linux 串口参数、RS485 ioctl 和输入缓冲刷新
- Modbus TCP MBAP 头、事务 ID 和多客户端服务
- 请求/响应校验、异常响应、超时和重试策略
- Coils、Discrete Inputs、Holding Registers、Input Registers 四类 Mapping
- 基于 Mapping 的异步 RTU/TCP 从站服务

## 目录与命名空间

公共头文件和实现文件按职责分层，并保持目录对应：

```text
include/issac/modbus/
  protocol/  Modbus 报文、功能码、CRC 和数据类型
  core/      Mapping、错误码和事务策略
  transport/ TCP/RTU 客户端连接
  server/    TCP 从站服务端

src/
  protocol/  Modbus 报文、CRC 和异常响应实现
  core/      Mapping 和错误码实现
  transport/ TCP/RTU 客户端连接实现
  server/    TCP 从站服务实现
```

新代码建议使用统一入口和分层命名空间：

```cpp
#include <issac/modbus/modbus.h>

issac::modbus::protocol::ModbusRequest request;
issac::modbus::core::Mapping mapping;
issac::modbus::transport::tcp::Connection connection(ioContext, host, port);
issac::modbus::server::tcp::Server server(ioContext, port);
```

旧版 `MB` 和全局协议类型仍保留为兼容别名，便于逐步迁移。

## Mapping 地址空间

`Mapping` 对齐 libmodbus 的 `modbus_mapping_t`，支持四类数据区各自的起始地址。
原有构造函数保持从地址 0 开始，使用起始地址时使用
`newStartAddress()`，参数顺序与 `modbus_mapping_new_start_address()` 一致：

```cpp
auto mapping = issac::modbus::core::Mapping::newStartAddress(
    100, 1000,  // coils: start 100, count 1000
    0, 1024,    // discrete inputs: start 0, count 1024
    400, 5000,  // holding registers: start 400, count 5000
    0, 1200);   // input registers: start 0, count 1200

mapping.holdingRegisters[4] = 42; // Modbus 地址 404
```

请求中的地址是 Modbus 绝对地址，`handleRequest()` 会先减去对应起始地址，
再访问底层数组。地址低于起始地址或超过映射范围时返回非法数据地址异常。

## 构建

依赖：

- CMake 3.16 或更高版本
- C++17 编译器
- standalone Asio
- pthreads

```bash
cmake -S . -B build
cmake --build build -j2
```

## Linux 主站/从站示例

开启默认的 `ISSAC_MODBUS_BUILD_EXAMPLES` 后会生成四个程序：

```text
issac_modbus_tcp_slave   TCP 从站，默认监听 0.0.0.0:1502
issac_modbus_tcp_master  TCP 主站，默认连接 127.0.0.1:1502
issac_modbus_rtu_slave   RTU 从站，默认 /dev/ttyUSB0 9600 8N1
issac_modbus_rtu_master  RTU 主站，默认 /dev/ttyUSB0 9600 8N1
```

构建并运行 TCP 对测：

```bash
cmake -S . -B build -DISSAC_MODBUS_ASIO_INCLUDE_DIR=/path/to/asio/include
cmake --build build -j2
./build/issac_modbus_tcp_slave 1502
./build/issac_modbus_tcp_master 127.0.0.1 1502
```

RTU 需要两个串口端点，或者 Linux 下的虚拟串口对；两个进程使用相同波特率，示例默认
Modbus unit ID 为 `1`：

```bash
./build/issac_modbus_rtu_slave /dev/ttyS2 9600
./build/issac_modbus_rtu_master /dev/ttyS2 9600
```

RTU 示例默认使用普通 `RS232` 模式，不调用 Linux RS485 ioctl，适合普通
`/dev/ttyS*` 或外置自动收发转换器。如果串口驱动明确支持内核 RS485 方向控制，
在命令末尾增加 `rs485`：

```bash
./build/issac_modbus_rtu_slave /dev/ttyS2 9600 rs485
./build/issac_modbus_rtu_master /dev/ttyS2 9600 rs485
```

示例软协议固定了四类数据区，地址使用 Modbus 绝对地址：

| 数据区 | 地址 | 约定 |
| --- | ---: | --- |
| Coils | `0000..0007` | 泵使能、阀门、报警复位、自动模式 |
| Discrete Inputs | `0100..0107` | 就绪、运行、故障、急停 |
| Holding Registers | `0200..0207` | 目标转速、目标温度、控制标志、命令 |
| Input Registers | `0300..0307` | 实际转速、温度、压力、遥测计数 |

主站会依次读取四个数据区，写入泵线圈和四个保持寄存器，再读回保持寄存器验证写入结果；
从站的 `beforeRequest` 钩子会刷新遥测值，`afterRequest` 钩子会记录可写数据变化。

如果只需要库而不需要示例，可以使用 `-DISSAC_MODBUS_BUILD_EXAMPLES=OFF`。

## 安装

```bash
cmake --install build --prefix /usr/local
```

安装后可以使用：

```cmake
find_package(issac_modbus CONFIG REQUIRED)
target_link_libraries(app PRIVATE issac_modbus::issac_modbus)
```

## 代码格式化

仓库提供 `.clang-format` 和 `.clang-tidy`：

```bash
find include src -type f \( -name '*.h' -o -name '*.cpp' \) -print0 \
  | xargs -0 clang-format -i
cmake -S . -B build-tidy -DISSAC_MODBUS_ENABLE_CLANG_TIDY=ON
cmake --build build-tidy -j2
```
