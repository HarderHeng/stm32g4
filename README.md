# Embassy STM32G4 异步定时器示例

基于 Embassy 框架的 STM32G4 异步开发示例项目。

## 项目结构

```
stm32g4/
├── src/
│   └── bin/
│       └── main.rs        # 主程序入口
├── examples/
│   └── bin/               # 示例代码（已忽略）
├── build.rs               # 构建脚本
├── Cargo.toml             # 项目配置
└── .cargo/                # Cargo 配置
```

## 依赖库

### 核心库

| 库 | 版本 | 用途 |
|---|---|---|
| `embassy-stm32` | 0.6.0 | STM32 HAL 支持 |
| `embassy-executor` | 0.10.0 | 异步任务执行器 |
| `embassy-time` | 0.5.1 | 异步定时器 |
| `embassy-sync` | 0.8.0 | 异步同步原语 |

### 调试库

| 库 | 版本 | 用途 |
|---|---|---|
| `defmt` | 1.0.1 | 高效日志格式化 |
| `defmt-rtt` | 1.0.0 | RTT 日志传输 |
| `panic-probe` | 1.0.0 | Panic 处理 |

## Embassy 核心概念

### 1. 异步任务 (Task)

使用 `#[embassy_executor::task]` 宏定义可独立运行的任务：

```rust
#[embassy_executor::task]
async fn my_task() {
    loop {
        // 异步操作
        Timer::after_secs(1).await;
    }
}
```

### 2. 任务启动 (Spawn)

在 `main` 函数中通过 `Spawner` 启动任务：

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 启动任务
    spawner.spawn(unwrap!(my_task()));
}
```

### 3. 异步定时器

Embassy 提供多种定时方式：

```rust
// 秒级定时
Timer::after_secs(1).await;

// 毫秒级定时
Timer::after_millis(500).await;

// 微秒级定时
Timer::after_micros(100).await;

// 自定义频率
Timer::after(Duration::from_hz(100)).await;
```

## 主程序说明

### 时钟配置

STM32G431CB 使用外部 8MHz 晶振，通过 PLL 配置到 170MHz：

```
HSE (8MHz) → PREDIV/2 (4MHz) → PLL MUL85 (340MHz) → PLLR/2 (170MHz)
```

### 异步任务示例

项目实现了两个并行运行的定时器任务：

- **1秒定时器**: 每秒打印一次计数
- **2秒定时器**: 每两秒打印一次计数

运行效果：
```
0.000000 [INFO ] Embassy 异步定时器示例启动!
1.000518 [INFO ] [1s定时器] 第 1 次触发
2.000488 [INFO ] [2s定时器] 第 1 次触发
2.000701 [INFO ] [1s定时器] 第 2 次触发
3.000885 [INFO ] [1s定时器] 第 3 次触发
...
```

## 构建与运行

```bash
# 构建
cargo build --bin main

# 运行（需要连接调试器）
cargo run --bin main

# 发布构建
cargo build --release --bin main
```

## 目标芯片

- MCU: STM32G431CBUx
- Flash: 128KB
- RAM: 32KB
- 主频: 170MHz