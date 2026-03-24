# Fix 02: RESET 命令响应帧在复位前未发送

## 问题描述

`handle_reset()` 的执行流程如下：

```
process_byte(byte)
  └─ handle_command(cmd, data, response)
       └─ handle_reset(data, response)
            1. build_response(CMD_RESET, STATUS_OK, ...) → 写入 response buffer
            2. for _ in 0..10000 { nop() }               → 延时约 0.6ms
            3. SCB::sys_reset()                           → 立即复位，永不返回
```

**`process_byte` 返回后，`bootloader.rs` 的主循环才会调用 `uart::write_bytes()` 发送 response buffer。但 `handle_reset` 调用 `sys_reset()` 永远不会返回，主循环的发送代码根本不会执行。**

```rust
// bootloader.rs main loop - 这段代码在 RESET 命令时永远不会执行
let resp_len = handler.process_byte(byte, &mut response_buf);
if resp_len > 0 {
    uart::write_bytes(&response_buf[..resp_len]);  // ← 永远不会到达这里
}
```

函数内的 10000 NOP 延时是无意义的——它在发送之前等待，而发送永远不会发生。

## 问题影响

OTA Tool 在发送 CMD_RESET 后会等待响应（`send_command` 有 10 秒超时）。由于响应从未被发送，工具会一直等待直到超时，然后判断为错误。

实际上 OTA Tool 的 `reset()` 函数已经做了容错处理（忽略错误），所以功能上不影响复位本身，但会产生 10 秒的无效等待，体验差。

## 根本原因

架构上，`handle_reset` 既需要构建响应帧，也需要在复位前确保帧已发送完毕。但 response buffer 的发送职责在调用者（主循环）手中，而 `handle_reset` 在返回之前就执行了复位。

## 修复方案

在 `handle_reset` 内部直接调用 `uart::write_bytes()` 发送响应，然后等待 USART2 发送完成（TC - Transmission Complete 标志位，ISR[6]），再执行复位。

需要在 `uart.rs` 中增加一个 `wait_tx_complete()` 函数：

```rust
/// Wait until UART shift register is empty (all bits physically transmitted)
pub fn wait_tx_complete() {
    unsafe {
        let isr = (USART2_BASE + off::ISR) as *const u32;
        while core::ptr::read_volatile(isr) & isr::TC == 0 {
            cortex_m::asm::nop();
        }
    }
}
```

**TXE（TX Empty）vs TC（Transmission Complete）的区别：**
- `TXE`（bit 7）：发送数据寄存器为空，可以写入下一个字节，但当前字节可能还在移位寄存器里
- `TC`（bit 6）：移位寄存器也已完成发送，所有位已物理发出

等待 TC 才能保证最后一个字节的所有位都已经发到线上，此后复位不会截断帧。

## 修改文件

- `src/bootloader/uart.rs`：在 `isr` 模块增加 `TC` 位定义，增加 `wait_tx_complete()` 函数
- `src/bootloader/ota.rs`：`handle_reset` 直接发送响应 + 等待 TC，然后复位
