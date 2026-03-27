# STM32G4 FOC 开发规划

> **注意**: 详细的移植计划已更新到 [MIGRATION_PLAN.md](./MIGRATION_PLAN.md)

## 项目概述

基于 Embassy 异步运行时的 STM32G4 FOC (Field Oriented Control) 电机实现，使用 Rust 实现。

## 当前状态 (2026-03-27)

| 层级 | 模块 | 状态 | 备注 |
|------|------|------|------|
| **Bootloader** | UART OTA + Flash | ✅ 完成 (90%) | 支持 IWDG 检测 |
| **驱动层** | PWM/ADC/OPAMP/COMP | ✅ 完成 (85%) | R3_2 拓扑 |
| **FOC 算法** | 变换/PI/SVPWM | ✅ 基础完成 (60%) | 待 STO/PLL |
| **位置反馈** | STO/PLL | ❌ 未开始 (0%) | **下一阶段重点** |
| **保护功能** | OCP/PVD | ⚠️ 部分 (50%) | 待母线/NTC |

### 最近修复 (2026-03-27)

- ✅ 修复 TIM1_UP 中断处理程序 W1C 语义 (HardFault 预防)
- ✅ 添加 ADC 校准超时计数器 (死锁预防)
- ✅ 添加 Vbus 零除保护 (SVPWM)
- ✅ 优化 SVPWM 栈分配 (控制循环)

---

## 硬件平台

### MCU
- **型号**: STM32G431CBUx
- **Flash**: 128KB
- **RAM**: 32KB
- **主频**: 170MHz

### 开发板
- **参考**: B-G431B-ESC1 (ESC 开发板)
- **栅极驱动**: L6387E
- **电流采样**: 三电阻分流 (Three Shunt)
- **额定电压**: 24V
- **最大电流**: 5A

## 硬件资源配置

### 电机控制外设

| 外设 | 功能 | 引脚 | 说明 |
|------|------|------|------|
| **TIM1** | PWM 生成 | PA8(UH), PA9(VH), PA10(WH), PC13(UL), PA12(VL), PB15(WL) | 高级定时器，产生 6 路 PWM |
| **ADC1** | 电流采样 | PA1(U 相), ADC12(W 相) | 注入通道，与 PWM 同步 |
| **ADC2** | 电流采样 | ADC2(V 相) | 注入通道，与 PWM 同步 |
| **OPAMP1** | U 相放大 | PA1(输入), PA2(输出), PA3(增益) | 内部运放，PGA 模式 |
| **OPAMP2** | V 相放大 | PA7(输入), PA6(输出), PA5(增益) | 内部运放，PGA 模式 |
| **OPAMP3** | W 相放大 | PB0(输入), PB1(输出), PB2(增益) | 内部运放，PGA 模式 |
| **COMP1/2/4** | 过流保护 | 内部 DAC 基准 | 硬件过流保护 |
| **DAC3** | 过流阈值 | 内部连接 | 过流比较器基准 |
| **CORDIC** | 三角函数 | - | 硬件加速 sin/cos |

### 采样参数

| 参数 | 值 | 说明 |
|------|-----|------|
| PWM 频率 | 20kHz | 电机控制频率 |
| 死区时间 | 800ns | 硬件死区 |
| 采样电阻 | 3mΩ | 分流电阻 |
| 放大增益 | 9.14 | 运放增益 |
| ADC 分辨率 | 12 位 | 左对齐 |

### 通信接口

| 外设 | 功能 | 引脚 |
|------|------|------|
| USART2 | Shell | PB3(TX), PB4(RX) |

## 软件架构

### 分层设计

```
┌─────────────────────────────────────────────┐
│                 Application                 │
│  (Shell Commands, Motor Control Logic)     │
├─────────────────────────────────────────────┤
│                 FOC Library                 │
│  (Park/Clarke Transform, PID, SVPWM)        │
├─────────────────────────────────────────────┤
│              Hardware Abstraction           │
│  (PWM Driver, ADC Driver, OPAMP Config)     │
├─────────────────────────────────────────────┤
│              Embassy HAL                   │
│  (Async Peripheral Drivers)                │
└─────────────────────────────────────────────┘
```

### 模块划分

```
src/
├── bin/
│   └── main.rs           # 主程序
├── bsp/
│   ├── mod.rs
│   └── board.rs          # 板级配置
├── driver/
│   ├── mod.rs
│   ├── shell.rs          # Shell 实现
│   ├── pwm.rs            # PWM 驱动 (TIM1)
│   ├── adc.rs            # ADC 驱动 (电流采样)
│   ├── opamp.rs          # 运放配置
│   ├── comp.rs           # 比较器保护
│   └── traits.rs         # 硬件抽象 trait
└── foc/
    ├── mod.rs
    ├── motor.rs          # FOC 控制器
    ├── transforms.rs     # Clarke/Park 变换
    ├── pi.rs             # PI 控制器
    └── svpwm.rs          # SVPWM 调制
```

## 开发阶段

### 阶段 0: Bootloader (✅ 已完成)
- [x] Bootloader 框架
- [x] Flash 读/写/擦除
- [x] OTA 协议实现
- [x] Python OTA 工具
- [x] APP 跳转
- [x] IWDG 复位检测

### 阶段 1: 基础外设初始化 (✅ 已完成)
- [x] 时钟配置 (170MHz)
- [x] UART Shell
- [x] PWM 初始化 (TIM1, 6 路输出，死区)
- [x] ADC 初始化 (注入通道，触发同步)
- [x] OPAMP 初始化 (内部运放，PGA 模式)
- [x] COMP 过流保护

### 阶段 2: 电流采样 (✅ 已完成)
- [x] ADC 注入通道配置
- [x] PWM 触发 ADC 同步
- [x] 电流校准
- [x] R3_2 双 ADC 方案

### 阶段 3: FOC 算法集成 (⚠️ 部分完成)
- [x] Clarke/Park 变换 (`transforms.rs`)
- [x] 逆 Park 变换 (`transforms.rs`)
- [x] SVPWM 调制 (`svpwm.rs`)
- [x] PI 控制器 (`pi.rs`)
- [x] 电流环控制 (`motor.rs`)
- [ ] 速度环控制 (`motor.rs` 框架完成，待调优)
- [ ] **STO 状态观测器** (下一阶段)
- [ ] **STO-PLL** (下一阶段)

### 阶段 4: 无传感器控制 (❌ 待开始)
- [ ] 反电动势观测器 (`foc/sto.rs`)
- [ ] 速度估计 (`foc/pll.rs`)
- [ ] 启动算法 (`foc/revup.rs`)

### 阶段 5: 保护与监测功能 (⚠️ 部分完成)
- [x] 硬件过流保护 (COMP + DAC3)
- [x] PVD 欠压检测
- [ ] 母线电压检测 (`driver/bus_voltage.rs`)
- [ ] NTC 温度检测 (`driver/ntc.rs`)
- [ ] 弱磁控制 (`foc/flux_weakening.rs`)
- [ ] Profiler 参数辨识 (`foc/profiler.rs`)

---

## 依赖库

| 库 | 版本 | 用途 |
|---|---|---|
| `embassy-stm32` | 0.6.0 | STM32 HAL |
| `libm` | * | 数学库 (sin/cos/sqrt) |
| `embedded-cli` | 0.2.1 | Shell |
| `defmt` | * | 调试输出 |

## 电机参数 (参考电机)

```rust
// 主程序中的配置
const POLE_PAIRS: u8 = 7;        // 7 对极
const RS: f32 = 2.55;            // 定子电阻 2.55Ω
const LS: f32 = 0.00086;         // 定子电感 0.86mH
const KE: f32 = 0.05;            // 反电动势常数
const MAX_CURRENT: f32 = 10.0;   // 最大相电流 10A
const VBUS_NOMINAL: f32 = 24.0;  // 标称母线电压 24V
```

## 预期功能

### Shell 命令
```
G431> motor start        # 启动电机
G431> motor stop         # 停止电机
G431> motor speed 3000   # 设置转速
G431> motor info         # 显示电机状态
G431> foc params         # 显示 FOC 参数
G431> foc tune <param> <value>  # 调整参数
G431> system ota         # 进入 OTA 模式
```

### 安全保护
- [x] 硬件过流保护 (COMP + DAC)
- [x] PVD 欠压/过压检测
- [ ] 软件过流保护
- [ ] 过温保护
- [ ] 启动按钮控制

## 参考资源

1. [STM32G4 参考手册](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
2. [B-G431B-ESC1 用户手册](https://www.st.com/resource/en/user_manual/um2523-discovery-kit-with-stm32g431cb-mcu-stmicroelectronics.pdf)
3. [ST MCSDK v6.4.1](~/study/G431/G431/MCSDK_v6.4.1-Full) - 参考实现
4. [Embassy 文档](https://docs.embassy.dev/embassy-stm32/)

## 文档索引

- [MIGRATION_PLAN.md](./MIGRATION_PLAN.md) - 详细移植计划 (MCSDK 模块评估)
- [BOOTLOADER_DESIGN.md](./BOOTLOADER_DESIGN.md) - Bootloader 设计文档
- [OTA_PROTOCOL.md](./OTA_PROTOCOL.md) - OTA 协议说明
- [PROBLEM_SOLVING.md](./PROBLEM_SOLVING.md) - 问题解决记录
