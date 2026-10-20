# RTU 角色拆分验证

验证日期：2026-10-20。环境：Linux、GCC、C++17、standalone Asio。

## 已执行

- 默认构建成功，包含库、TCP 示例和 RTU 示例。
- `-Wall -Wextra -Wpedantic` 构建成功，无项目代码警告。
- `ctest --test-dir build --output-on-failure` 通过。
- ASan/UBSan 构建成功，RTU 测试在
  `ASAN_OPTIONS=detect_leaks=0:abort_on_error=1` 和
  `UBSAN_OPTIONS=halt_on_error=1` 下通过。

## 测试覆盖

测试使用 Linux `openpty()`，覆盖：

- 主站首轮响应超时后重试，成功解析两个保持寄存器。
- 主站拒绝寄存器数量与请求不一致的响应。
- 主站并发提交两笔事务时按 FIFO 发送并分别完成。
- 主站取消当前事务后可以继续提交下一笔事务。
- 主站在事件循环启动前取消时，对端不会收到已经投递的请求。
- 事务总期限覆盖重试等待阶段，不会因为较长的重试间隔而失效。
- 主站在重试等待期间关闭时，活动事务以 `operation_aborted` 完成。
- 从站过滤其他单元地址。
- 从站执行定向写并正确回显。
- 从站执行广播写且不发送响应。
- 从站对未知功能码返回非法功能异常。
- `RtuMaster` 与 `RtuSlave` 经两组伪终端和转发线程完成读事务。
- 从站在事件循环启动前启动并停止后，可以立即重新启动服务。
- 从站正常响应遵守配置的 `frameGap`，不会在接收请求后立即发送。
- 从站服务期间跨线程析构时，完成回调仍恰好调用一次。
- 手动通道在字节持续到达时仍执行整帧总期限。
- 手动通道在帧头声明超过 256 字节时返回 `message_size`。
- 手动通道可以用 `FrameType::Request` 正确接收完整请求帧。
- 手动通道拒绝同时启动读和写，并在取消后正常完成读回调。
- 从站停止后完成回调恰好调用一次。
- 从站停止时若外部派发业务尚未返回，服务完成回调会等待该业务完成。

## 未覆盖

- 真实 RS485 电平、RTS 接线、内核驱动 ioctl 和 USB 芯片内部缓存。
- 严格物理线路 t1.5 / t3.5 时序认证。
- Windows、macOS。
- ThreadSanitizer 可以编译，但当前运行环境在启动测试时报告
  `unexpected memory mapping`，因此没有得到有效的 TSan 运行结果。
- 完整泄漏检测；当前 ASan 运行关闭了 LeakSanitizer。

伪终端测试不能替代真实设备验收。
