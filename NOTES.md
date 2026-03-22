# STM32G4 UART Shell 开发笔记

## 项目概述

基于 Embassy 异步运行时的 STM32G4 UART Shell 实现，使用 embedded-cli 库提供交互式命令行界面。

## 遇到的问题与解决方案

### 1. UART RX Frame Error

**问题描述：**
上电后首次连接终端时触发 RX frame error。

**原因分析：**
- UART 初始化时序问题
- 终端连接时的电气噪声
- 缓冲区配置导致的偶发问题

**解决方案：**
实现错误处理机制，区分瞬态错误和持续错误：
```rust
let mut error_count: u8 = 0;
let startup_grace_period = embassy_time::Instant::now();

match rx.read(&mut buf).await {
    Ok(()) => {
        error_count = 0;
        shell.process(buf[0]);
    }
    Err(e) => {
        error_count = error_count.saturating_add(1);
        let elapsed_ms = startup_grace_period.elapsed().as_millis();
        if elapsed_ms > 100 || error_count > 3 {
            error!("RX error: {:?} (count: {})", e, error_count);
        }
    }
}
```

**处理策略：**
- 启动宽限期：前 100ms 内的偶发错误不报告
- 错误计数：允许最多 3 次错误
- 成功读取后重置计数
- 持续错误通过 `error!` 报告

### 2. DMA 中断绑定名称

**问题描述：**
```rust
DMA1_CH0 => dma::InterruptHandler<peripherals::DMA1_CH0>;  // 错误
```

**解决方案：**
STM32G4 系列的 DMA 中断命名为 `DMA1_CHANNEL1`：
```rust
bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});
```

### 3. embedded-io 版本冲突

**问题描述：**
`embedded-cli` 使用 `embedded-io` 0.7.x，但部分依赖仍使用 0.6.x。

**解决方案：**
使用 `StaticCell` 存储 UART TX，避免泛型类型跨越版本边界：
```rust
pub static SHELL_TX: StaticCell<UartTx<'static, Async>> = StaticCell::new();

pub fn init_shell_tx(tx: UartTx<'static, Async>) {
    let tx_ref = SHELL_TX.init(tx);
    // ...
}
```

### 4. Uart::new 参数顺序

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
  - Baud: 921600
  - DMA: TX=DMA1_CH2, RX=DMA1_CH1
- Clock: 170MHz (HSE 8MHz + PLL)

## Shell 特性

基于 embedded-cli 库：

| 特性 | 说明 |
|------|------|
| Tab 补全 | 自动补全命令 |
| 历史记录 | 上/下键浏览历史 |
| 帮助系统 | `help` 命令 |
| 光标移动 | 左/右键移动光标 |

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

# Release 模式运行
cargo run --release

# 终端连接
tio /dev/ttyUSB0 -b 921600
```

## 编译优化

Release 配置已优化以减少 Flash/RAM 占用：

```toml
[profile.release]
opt-level = "z"      # 优化代码大小
lto = "fat"          # 链接时优化
codegen-units = 1    # 单代码生成单元
panic = "abort"      # 不展开 panic
```

**优化效果：**
| 指标 | 优化前 | 优化后 |
|------|--------|--------|
| Flash | 44KB | 27KB |
| RAM | 1.9KB | 1.7KB |

**已移除未使用的依赖：**
- `embassy-embedded-hal`
- `usbd-hid`
- `embedded-hal`
- `embedded-io-async`
- `heapless`
- `critical-section`

## 依赖版本

| 库 | 版本 |
|---|---|
| embassy-stm32 | 0.6.0 |
| embassy-executor | 0.10.0 |
| embassy-time | 0.5.1 |
| embedded-cli | 0.2.1 (git) |
| defmt | 1.0.1 |