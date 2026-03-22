# Embassy STM32G4 UART Shell

基于 Embassy 框架的 STM32G4 UART Shell 实现，使用 embedded-cli 库提供交互式命令行界面。

## 特性

- 基于 Embassy 异步运行时
- UART DMA 传输
- 使用 embedded-cli 库
- 支持命令自动补全 (Tab)
- 支持命令历史 (上/下键)
- 支持帮助系统
- 支持光标移动 (左/右键)

## 项目结构

```
stm32g4/
├── src/
│   ├── bin/
│   │   └── main.rs        # 主程序入口
│   ├── driver/
│   │   ├── mod.rs         # 驱动模块
│   │   └── shell.rs       # Shell 实现
│   ├── bsp/
│   │   └── mod.rs         # BSP 初始化
│   └── lib.rs
├── build.rs               # 构建脚本
└── Cargo.toml             # 项目配置
```

## 依赖库

| 库 | 版本 | 用途 |
|---|---|---|
| `embassy-stm32` | 0.6.0 | STM32 HAL 支持 |
| `embassy-executor` | 0.10.0 | 异步任务执行器 |
| `embassy-time` | 0.5.1 | 异步定时器 |
| `embedded-cli` | 0.2.1 | CLI 库 (git) |

## Shell 命令

启动后显示提示符 `G431> `，支持以下命令：

| 命令 | 说明 |
|------|------|
| `help` | 显示命令列表 |
| `hello [name]` | 问候 |
| `clear` | 清屏 |
| `version` | 显示版本 |
| `echo [text]` | 回显文本 |
| `led on\|off\|toggle` | LED 控制 |
| `system info` | 显示系统信息 |

### 使用示例

```
G431> help
G431> hello
Hello, World!
G431> hello Rust
Hello, Rust!
G431> led on
LED ON
G431> system info
MCU: STM32G431CB
Clock: 170MHz
```

### 快捷键

| 按键 | 功能 |
|------|------|
| Tab | 自动补全命令 |
| 上/下 | 浏览历史命令 |
| 左/右 | 移动光标 |
| Backspace | 删除字符 |

## 硬件配置

- MCU: STM32G431CBUx (128KB Flash, 32KB RAM)
- UART: USART2
  - TX: PB3
  - RX: PB4
  - Baud: 115200
  - DMA: TX=DMA1_CH2, RX=DMA1_CH1
- Clock: 170MHz (HSE 8MHz + PLL)

## 构建与运行

```bash
# Debug 模式构建
cargo build

# Release 模式构建 (更小体积)
cargo build --release

# 运行 (需要连接调试器)
cargo run

# 查看代码大小
cargo size --release
```

## 终端连接

使用任意串口终端工具连接：

```bash
# 使用 tio
tio /dev/ttyUSB0

# 使用 minicom
minicom -D /dev/ttyUSB0

# 使用 picocom
picocom /dev/ttyUSB0 -b 115200
```

确保终端配置为：
- 波特率: 115200
- 数据位: 8
- 停止位: 1
- 无校验