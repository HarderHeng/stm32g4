# STM32G4 FOC Motor Controller

基于 Embassy 框架的 STM32G4 FOC 电机控制器，支持 OTA 固件升级。

## 功能特性

- **FOC 电机控制**：基于 Rust FOC 库的无刷电机控制
- **Bootloader**：支持 UART OTA 固件升级
- **交互式 Shell**：命令行调试接口
- **硬件加速**：CORDIC 三角函数加速

## 硬件平台

- **MCU**: STM32G431CBUx (128KB Flash, 32KB RAM, 170MHz)
- **开发板**: B-G431B-ESC1 (ESC 开发板)
- **栅极驱动**: L6387E
- **电流采样**: 三电阻分流

## 项目结构

```
stm32g4/
├── src/
│   ├── bin/
│   │   ├── main.rs         # APP 主程序
│   │   └── bootloader.rs   # Bootloader 程序
│   ├── bootloader/         # OTA Bootloader 模块
│   │   ├── boot_flag.rs    # 启动标志管理
│   │   ├── flash.rs        # Flash 操作
│   │   ├── ota.rs          # OTA 状态机
│   │   ├── protocol.rs     # 通信协议
│   │   └── uart.rs         # UART 传输
│   ├── driver/
│   │   ├── shell.rs        # Shell 实现
│   │   ├── pwm.rs          # PWM 驱动 (TIM1)
│   │   ├── adc.rs          # ADC 驱动 (电流采样)
│   │   └── opamp.rs        # 运放配置
│   ├── foc/
│   │   ├── motor.rs        # 电机参数
│   │   └── ...
│   ├── bsp/                # 板级支持包
│   └── lib.rs
├── tools/
│   └── ota-tool/           # OTA 升级工具 (Python)
└── docs/
    ├── FOC_PLAN.md         # FOC 开发计划
    └── OTA_PROTOCOL.md     # OTA 协议文档
```

## 内存布局

| 地址 | 大小 | 描述 |
|------|------|------|
| 0x08000000 | 20KB | Bootloader |
| 0x08005000 | 2KB | Boot Flag |
| 0x08005800 | 106KB | APP Region |

## 构建与运行

```bash
# 构建 APP
cargo build --bin stm32g4 --release

# 构建 Bootloader
cargo build --bin bootloader --release

# 查看代码大小
cargo size --bin stm32g4 --release
cargo size --bin bootloader --release
```

## Shell 命令

| 命令 | 说明 |
|------|------|
| `help` | 显示命令列表 |
| `hello [name]` | 问候 |
| `led on\|off\|toggle` | LED 控制 |
| `system info` | 显示系统信息 |
| `version` | 显示版本 |

## OTA 固件升级

```bash
# 获取 bootloader 信息
python tools/ota-tool/src/main.py --port /dev/ttyUSB0 --info

# 烧写固件
python tools/ota-tool/src/main.py --port /dev/ttyUSB0 --firmware app.bin

# 烧写并验证
python tools/ota-tool/src/main.py --port /dev/ttyUSB0 --firmware app.bin --verify

# 重启到 APP
python tools/ota-tool/src/main.py --port /dev/ttyUSB0 --reset
```

## 开发进度

详见 [docs/FOC_PLAN.md](docs/FOC_PLAN.md)

- [x] 阶段 1: 基础外设初始化 (时钟、UART、PWM、ADC、OPAMP)
- [x] Bootloader + OTA 功能
- [ ] 阶段 2: 电流采样
- [ ] 阶段 3: FOC 算法集成
- [ ] 阶段 4: 无传感器控制
- [ ] 阶段 5: 功能完善

## 终端连接

```bash
tio /dev/ttyUSB0 -b 921600
```

## 依赖库

| 库 | 版本 | 用途 |
|---|---|---|
| `embassy-stm32` | 0.6.0 | STM32 HAL |
| `foc` | 0.3.0 | FOC 算法 |
| `embedded-cli` | 0.2.1 | Shell |

## 参考资源

- [STM32G4 参考手册](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [B-G431B-ESC1 用户手册](https://www.st.com/resource/en/user_manual/um2523-discovery-kit-with-stm32g431cb-mcu-stmicroelectronics.pdf)
- [Embassy 文档](https://docs.embassy.dev/embassy-stm32/)