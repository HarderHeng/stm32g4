# STM32G4 FOC 开发规划

## 项目概述

基于 Embassy 异步运行时的 STM32G4 FOC (Field Oriented Control) 电机控制实现，使用 Rust FOC 库。

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
| **ADC1** | 电流采样 | PA1(U相), ADC12(W相) | 注入通道，与 PWM 同步 |
| **ADC2** | 电流采样 | ADC2(V相) | 注入通道，与 PWM 同步 |
| **OPAMP1** | U相放大 | PA1(输入), PA2(输出), PA3(增益) | 内部运放，PGA 模式 |
| **OPAMP2** | V相放大 | PA7(输入), PA6(输出), PA5(增益) | 内部运放，PGA 模式 |
| **OPAMP3** | W相放大 | PB0(输入), PB1(输出), PB2(增益) | 内部运放，PGA 模式 |
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
| ADC 分辨率 | 12位 | 左对齐 |

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
│   └── foc.rs            # FOC 集成
└── foc/
    ├── mod.rs
    ├── motor.rs          # 电机参数
    ├── controller.rs     # FOC 控制器
    └── sensor.rs         # 传感器处理
```

## 开发阶段

### 阶段 0: Bootloader (已完成)
- [x] Bootloader 框架
- [x] Flash 读/写/擦除
- [x] OTA 协议实现
- [x] Python OTA 工具
- [x] APP 跳转

### 阶段 1: 基础外设初始化 (已完成)
- [x] 时钟配置 (170MHz)
- [x] UART Shell
- [x] PWM 初始化 (TIM1, 6路输出, 死区)
- [x] ADC 初始化 (注入通道, 触发同步)
- [x] OPAMP 初始化 (内部运放, PGA 模式)

### 阶段 2: 电流采样
- [ ] ADC 注入通道配置
- [ ] PWM 触发 ADC 同步
- [ ] 电流校准

### 阶段 3: FOC 算法集成
- [ ] 集成 foc 库
- [ ] Park/Clarke 变换
- [ ] SVPWM 调制
- [ ] PID 控制器

### 阶段 4: 无传感器控制
- [ ] 反电动势观测器
- [ ] 速度估计
- [ ] 启动算法

### 阶段 5: 功能完善
- [ ] 过流保护
- [ ] 温度监控
- [ ] 母线电压监测
- [ ] Shell 命令扩展

## 依赖库

| 库 | 版本 | 用途 |
|---|---|---|
| `embassy-stm32` | 0.6.0 | STM32 HAL |
| `foc` | 0.3.0 | FOC 算法 |
| `fixed` | * | 定点数运算 |
| `embedded-cli` | 0.2.1 | Shell |

## 电机参数 (示例)

```rust
pub struct MotorParams {
    /// 极对数
    pub pole_pairs: u8 = 4,
    /// 定子电阻 (Ω)
    pub rs: f32 = 0.32,
    /// 电感 (H)
    pub ls: f32 = 0.00047,
    /// 额定电压 (V)
    pub nominal_voltage: f32 = 24.0,
    /// 额定电流 (A)
    pub nominal_current: f32 = 5.0,
    /// 最大转速 (RPM)
    pub max_speed: u32 = 6420,
}
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
```

### 安全保护
- 硬件过流保护 (COMP + DAC)
- 软件过流保护
- 过压/欠压保护
- 过温保护
- 启动按钮控制

## 参考资源

1. [STM32G4 参考手册](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
2. [B-G431B-ESC1 用户手册](https://www.st.com/resource/en/user_manual/um2523-discovery-kit-with-stm32g431cb-mcu-stmicroelectronics.pdf)
3. [foc-rs 库](https://github.com/calebfletcher/foc)
4. [Embassy 文档](https://docs.embassy.dev/embassy-stm32/)