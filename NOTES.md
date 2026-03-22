# STM32G4 UART Shell 开发笔记

## 项目概述

基于 Embassy 异步运行时的 STM32G4 UART Shell 实现，支持 DMA 传输和交互式命令行。

## 遇到的问题与解决方案

### 1. Flash 空间溢出

**问题描述：**
Debug 模式下编译后代码大小超过 STM32G431CB 的 128KB Flash，导致链接错误：
```
rust-lld: error: section '.rodata' will not fit in region 'FLASH': overflowed by 3344 bytes
```

**原因分析：**
- 使用 `heapless::String<CMD_MAX_LEN>` 等泛型类型会产生大量代码实例化
- 复杂的 escape sequence 解析逻辑
- 大量字符串常量

**解决方案：**
- 使用原生 `[u8; N]` 数组替代 `heapless::String`
- 简化状态机，移除复杂的 escape sequence 处理
- 减少字符串常量，使用简短的提示信息

**优化效果：**
| 模式 | 优化前 | 优化后 |
|------|--------|--------|
| Debug | 溢出 (131KB+) | 108KB |
| Release | 42KB | 28KB |

### 2. DMA 中断绑定名称错误

**问题描述：**
```rust
bind_interrupts!(struct Irqs {
    DMA1_CH0 => dma::InterruptHandler<peripherals::DMA1_CH0>;  // 错误
});
```

**原因：**
STM32G4 系列的 DMA 中断在 Embassy 中命名为 `DMA1_CHANNEL1` 而非 `DMA1_CH1`。

**解决方案：**
```rust
bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});
```

### 3. Embassy Executor Spawn Token

**问题描述：**
```rust
spawner.spawn(shell_task(tx, rx)).ok();  // 错误
```

**原因：**
`#[embassy_executor::task]` 宏生成的函数返回 `Result<SpawnToken<_>, SpawnError>`，需要 unwrap。

**解决方案：**
```rust
spawner.spawn(unwrap!(shell_task(tx, rx)));
```

### 4. embedded-io Write Trait 返回值

**问题描述：**
```rust
tx.write(b"data").await?;  // 返回 Result<usize, E>
```

**原因：**
`embedded_io::Write::write()` 返回 `Result<usize, E>`，不是 `Result<(), E>`。

**解决方案：**
```rust
// 方案1: 使用分号忽略返回值
tx.write(b"data").await?;
Ok(())

// 方案2: 使用块表达式
match args {
    b"on" => { tx.write(b"LED ON\r\n").await?; }
    _ => { tx.write(b"Usage: led on|off|toggle\r\n").await?; }
}
```

### 5. Uart::new 参数顺序

**问题描述：**
参数顺序错误导致编译失败。

**正确的参数顺序：**
```rust
Uart::new(
    usart,    // USART 外设
    rx_pin,   // RX 引脚
    tx_pin,   // TX 引脚
    tx_dma,   // TX DMA 通道
    rx_dma,   // RX DMA 通道
    irq,      // 中断绑定
    config,   // 配置
)
```

注意：TX DMA 在 RX DMA 之前，与直觉相反。

## 硬件配置

- MCU: STM32G431CBUx (128KB Flash, 32KB RAM)
- UART: USART2
  - TX: PB3
  - RX: PB4
  - Baud: 115200
  - DMA: TX=DMA1_CH2, RX=DMA1_CH1
- Clock: 170MHz (HSE 8MHz + PLL)

## 可用命令

| 命令 | 说明 |
|------|------|
| `help` | 显示命令列表 |
| `hello [name]` | 问候 |
| `clear` | 清屏 |
| `version` | 显示版本 |
| `echo [text]` | 回显文本 |
| `led on\|off\|toggle` | LED 控制 |
| `system info` | 显示系统信息 |

## 使用方法

```bash
# Debug 模式运行
cargo run

# Release 模式运行 (更小体积)
cargo run --release

# 查看代码大小
cargo size
cargo size --release

# 分析代码体积
cargo bloat --release
```

## 依赖版本

- embassy-stm32: 0.6.0
- embassy-executor: 0.10.0
- embassy-time: 0.5.1
- defmt: 1.0.1
- heapless: 0.9