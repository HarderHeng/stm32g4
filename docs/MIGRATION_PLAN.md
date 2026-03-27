# STM32G4 FOC 移植计划

**文档版本**: v1.0
**最后更新**: 2026-03-27
**参考项目**: ST MCSDK v6.4.1 (~/study/G431/G431/MCSDK_v6.4.1-Full)

---

## 一、项目概述

### 1.1 移植目标

将 ST Motor Control SDK v6.4.1 中的核心 FOC 算法移植到 Rust/Embassy 环境，实现无传感器 FOC 电机控制。

### 1.2 硬件平台

| 项目 | 规格 |
|------|------|
| **MCU** | STM32G431CBUx (128KB Flash, 32KB RAM, 170MHz) |
| **开发板** | B-G431B-ESC1 |
| **电机** | PMSM (7 对极，Rs=2.55Ω, Ls=0.86mH, 24V, 2600 RPM) |
| **电流采样** | 三电阻分流 + 双 ADC 注入 (R3_2 拓扑) |
| **PWM 频率** | 20kHz |

### 1.3 当前状态

| 层级 | 模块 | 状态 |
|------|------|------|
| **Bootloader** | UART OTA + Flash | ✅ 完成 (90%) |
| **驱动层** | PWM/ADC/OPAMP/COMP | ✅ 完成 (85%) |
| **FOC 算法** | 变换/PI/SVPWM | ✅ 基础完成 (60%) |
| **位置反馈** | STO/PLL | ❌ 未开始 (0%) |
| **保护功能** | OCP/PVD | ⚠️ 部分 (50%) |

---

## 二、MCSDK 模块移植价值评估

### 2.1 评估标准

| 等级 | 标准 |
|------|------|
| ⭐⭐⭐⭐⭐ | **必需** - 无替代方案，核心功能 |
| ⭐⭐⭐⭐ | **高价值** - 强烈建议移植 |
| ⭐⭐⭐ | **实用** - 简化实现即可 |
| ⭐⭐ | **可选** - 特定场景需要 |
| ⭐ | **低价值** - 通常不需要 |
| ❌ | **不建议** - 有更好的替代方案 |

---

### 2.2 高移植价值模块 (P0-P1)

#### 🔴 P0: 无传感器 FOC 核心

##### 1. STO 状态观测器 (`sto_speed_pos_fdbk.c/h`)

| 评估项 | 详情 |
|--------|------|
| **移植价值** | ⭐⭐⭐⭐⭐ |
| **源文件** | `MCLib/Any/Src/sto_speed_pos_fdbk.c` (~800 行) |
| **头文件** | `MCLib/Any/Inc/sto_speed_pos_fdbk.h` |
| **目标模块** | `src/foc/sto.rs` |
| **工作量** | 3-5 天 |

**功能**:
- 反电动势估算
- 转子位置估算
- 速度估算
- 收敛检测

**为什么必需**:
- 当前 `motor.rs:356` 中 `self.omega_e = 0.0` 是占位符
- 没有 STO 只能开环运行，无法实际应用
- 无替代方案，必须自己实现

**关键算法**:
```c
// ST C 代码 - 固定点
// 定子磁链估算
pHandle->hFlux_a = pHandle->hFlux_a + pHandle->hC1 * (pHandle->hV_alpha - pHandle->hRs * pHandle->hI_alpha);
pHandle->hFlux_b = pHandle->hFlux_b + pHandle->hC1 * (pHandle->hV_beta - pHandle->hRs * pHandle->hI_beta);

// 反电动势
pHandle->hBemf_a = pHandle->hFlux_a - pHandle->hC2 * pHandle->hI_beta;
pHandle->hBemf_b = pHandle->hFlux_b + pHandle->hC2 * pHandle->hI_alpha;
```

```rust
// Rust 移植目标 - 浮点
pub fn estimate bemf(&mut self, v_alpha: f32, v_beta: f32, i_alpha: f32, i_beta: f32) -> (f32, f32) {
    self.flux_a += self.c1 * (v_alpha - self.rs * i_alpha);
    self.flux_b += self.c1 * (v_beta - self.rs * i_beta);

    let bemf_a = self.flux_a - self.c2 * i_beta;
    let bemf_b = self.flux_b + self.c2 * i_alpha;
    (bemf_a, bemf_b)
}
```

---

##### 2. STO-PLL 锁相环 (`sto_pll_speed_pos_fdbk.c/h`)

| 评估项 | 详情 |
|--------|------|
| **移植价值** | ⭐⭐⭐⭐⭐ |
| **源文件** | `MCLib/Any/Src/sto_pll_speed_pos_fdbk.c` (~500 行) |
| **头文件** | `MCLib/Any/Inc/sto_pll_speed_pos_fdbk.h` |
| **目标模块** | `src/foc/pll.rs` |
| **工作量** | 2-3 天 |

**功能**:
- 从 STO 输出提取精确转子角度
- 速度滤波
- PLL 锁定检测

**为什么必需**:
- STO 输出有噪声，需要 PLL 滤波
- CORDIC 需要硬件支持，PLL 更通用

**关键算法**:
```c
// PLL 误差计算
hError = hBackEMF_beta * cos_theta - hBackEMF_alpha * sin_theta;

// PI 调节器
hSpeedEst = PID_Controller(&pHandle->_Super.PID_speed_est, hError);

// 积分得到角度
pHandle->_Super.hElAngle = pHandle->_Super.hElAngle + hSpeedEst * Ts;
```

---

#### 🟠 P1: 启动与切换

##### 3. 开环启动 (`open_loop.c/h`)

| 评估项 | 详情 |
|--------|------|
| **移植价值** | ⭐⭐⭐⭐ |
| **源文件** | `MCLib/Any/Src/open_loop.c` (~300 行) |
| **头文件** | `MCLib/Any/Inc/open_loop.h` |
| **目标模块** | 完善 `src/foc/motor.rs` 中的 `RampGenerator` |
| **工作量** | 2 天 |

**功能**:
- V/F 曲线控制
- 启动斜坡
- 开环→闭环切换检测

**当前状态**:
```rust
// motor.rs:387-430 - 基础 RampGenerator
pub struct RampGenerator {
    omega: f32,
    target: f32,
    accel: f32,
}
// 缺少 V/F 曲线、收敛检测
```

**需要添加**:
- `V/F` 曲线：`Voltage = Offset + Slope * Frequency`
- 启动状态机
- Convergency 检测

---

##### 4. RevUp 启动控制 (`revup_ctrl.c/h`)

| 评估项 | 详情 |
|--------|------|
| **移植价值** | ⭐⭐⭐⭐ |
| **源文件** | `MCLib/Any/Src/revup_ctrl.c` (~400 行) |
| **头文件** | `MCLib/Any/Inc/revup_ctrl.h` |
| **目标模块** | `src/foc/revup.rs` |
| **工作量** | 2 天 |

**功能**:
- 开环→闭环平滑切换
- 失步检测
- 启动失败重试

---

#### 🟡 P2: 保护与监测

##### 5. 母线电压检测 (`r_divider_bus_voltage_sensor.c/h`)

| 评估项 | 详情 |
|--------|------|
| **移植价值** | ⭐⭐⭐ |
| **源文件** | `MCLib/Any/Src/r_divider_bus_voltage_sensor.c` (~150 行) |
| **头文件** | `MCLib/Any/Inc/r_divider_bus_voltage_sensor.h` |
| **目标模块** | `src/driver/bus_voltage.rs` (新建) |
| **工作量** | 1 天 |

**功能**:
- DC 母线电压采样 (PA0 - ADC1_CH1)
- 欠压保护 (UVLO)
- 过压保护 (OVLO)
- SVPWM 母线补偿

**简化实现**:
```rust
pub struct BusVoltage {
    adc_channel: u8,
    vbus_filtered: f32,
    undervoltage_threshold: f32,
    overvoltage_threshold: f32,
}

impl BusVoltage {
    pub fn read(&mut self) -> f32 {
        let raw = adc_read(self.adc_channel);
        // 分压比 0.09626
        self.vbus_filtered = raw as f32 * 0.09626;
        self.vbus_filtered
    }

    pub fn check_faults(&self) -> Option<Fault> {
        if self.vbus_filtered < self.undervoltage_threshold {
            return Some(Fault::Undervoltage);
        }
        if self.vbus_filtered > self.overvoltage_threshold {
            return Some(Fault::Overvoltage);
        }
        None
    }
}
```

---

##### 6. NTC 温度检测 (`ntc_temperature_sensor.c/h`)

| 评估项 | 详情 |
|--------|------|
| **移植价值** | ⭐⭐⭐ |
| **源文件** | `MCLib/Any/Src/ntc_temperature_sensor.c` (~200 行) |
| **头文件** | `MCLib/Any/Inc/ntc_temperature_sensor.h` |
| **目标模块** | `src/driver/ntc.rs` (新建) |
| **工作量** | 1 天 |

**功能**:
- NTC 温度采样 (PB14 - ADC1_CH5)
- 过温保护
- 降额控制

**硬件配置**:
```
PB14 → ADC1_CH5
NTC 热敏电阻：10kΩ @ 25°C
保护温度：70°C
```

---

##### 7. Profiler 参数辨识 (`profiler.c/h`, `profiler_impedest.c/h`)

| 评估项 | 详情 |
|--------|------|
| **移植价值** | ⭐⭐⭐ |
| **源文件** | `MCLib/Any/Src/profiler*.c` (~600 行) |
| **头文件** | `MCLib/Any/Inc/profiler.h` |
| **目标模块** | `src/foc/profiler.rs` (新建) |
| **工作量** | 3 天 |

**功能**:
- 自动测量 Rs (定子电阻)
- 自动测量 Ls (定子电感)
- 自动测量 Ke (反电动势常数)
- 自动测量极对数

**价值**:
- 免去手动测量 (万用表/LCR 电桥)
- 提高参数精度

---

### 2.3 中移植价值模块 (P3)

##### 8. 弱磁控制 (`flux_weakening_ctrl.c/h`)

| 评估项 | 详情 |
|--------|------|
| **移植价值** | ⭐⭐ |
| **源文件** | `MCLib/Any/Src/flux_weakening_ctrl.c` (~300 行) |
| **头文件** | `MCLib/Any/Inc/flux_weakening_ctrl.h` |
| **目标模块** | `src/foc/flux_weakening.rs` (可选) |
| **工作量** | 2 天 |

**功能**:
- 高速时降低 Id 以扩展速度范围
- 电压饱和时自动弱磁

**建议**: 参考电机最大 2600 RPM，可能不需要弱磁

---

### 2.4 不建议移植模块

| 模块 | 原因 | 替代方案 |
|------|------|---------|
| ❌ `fixpmath.c` | Rust f32 足够快 | `f32` + `libm` |
| ❌ `mcptl.c` | 专有协议 | 自定义简单 UART 协议 |
| ❌ `usart_aspep_driver.c` | 太复杂 | Embassy UART |
| ❌ `datalog.c` | 调试工具 | `defmt` + RTT |
| ❌ `data_scope.c` | 调试工具 | 自定义 |
| ❌ `encoder_speed_pos_fdbk.c` | 无编码器硬件 | - |
| ❌ `hall_speed_pos_fdbk.c` | 无霍尔硬件 | - |
| ❌ `otf_sixstep.c` | 不需要飞行启动 | - |
| ❌ `max_torque_per_ampere.c` | IPM 专用，PMSM 用 Id=0 | - |
| ❌ `circle_limitation.c` | 硬件 OCP 已足够 | - |
| ❌ `inrush_current_limiter.c` | 可简化 | 软启动即可 |
| ❌ `trajectory_ctrl.c` | 过于复杂 | 简单斜坡 |
| ❌ `cordic` 硬件加速 | 增加依赖 | `libm::sinf/cosf` |

---

## 三、移植路线图

### 3.1 阶段划分

```
┌──────────────────────────────────────────────────────────────┐
│  第 1 阶段：无传感器 FOC 核心 (2 周)                            │
│  ┌────────────────┐  ┌────────────────┐                      │
│  │   STO 观测器   │→ │   STO-PLL     │                      │
│  │   (3-5 天)     │  │   (2-3 天)     │                      │
│  └────────────────┘  └────────────────┘                      │
│  里程碑：能够无传感器运行                                     │
└──────────────────────────────────────────────────────────────┘
                              ↓
┌──────────────────────────────────────────────────────────────┐
│  第 2 阶段：启动完善 (1 周)                                    │
│  ┌────────────────┐  ┌────────────────┐                      │
│  │   开环 V/F     │→ │   RevUp 切换    │                      │
│  │   (2 天)       │  │   (2 天)       │                      │
│  └────────────────┘  └────────────────┘                      │
│  里程碑：平滑启动，不失步                                     │
└──────────────────────────────────────────────────────────────┘
                              ↓
┌──────────────────────────────────────────────────────────────┐
│  第 3 阶段：保护功能 (1 周)                                    │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐ │
│  │  母线电压检测  │  │  NTC 温度检测   │  │   Profiler     │ │
│  │   (1 天)       │  │   (1 天)       │  │   (3 天)       │ │
│  └────────────────┘  └────────────────┘  └────────────────┘ │
│  里程碑：完整保护，自动参数辨识                               │
└──────────────────────────────────────────────────────────────┘
                              ↓
┌──────────────────────────────────────────────────────────────┐
│  第 4 阶段：高级功能 (可选)                                    │
│  ┌────────────────┐                                          │
│  │   弱磁控制     │                                          │
│  │   (2 天)       │                                          │
│  └────────────────┘                                          │
│  里程碑：高速扩展能力                                         │
└──────────────────────────────────────────────────────────────┘
```

---

### 3.2 详细计划

#### 第 1 阶段：无传感器 FOC 核心 (2 周)

**周 1: STO 观测器**

| 天数 | 任务 | 输出 |
|------|------|------|
| Day 1 | 阅读 `sto_speed_pos_fdbk.c`，理解算法 | 算法笔记 |
| Day 2 | 创建 `src/foc/sto.rs` 框架 | 模块结构 |
| Day 3 | 实现反电动势估算 | `estimate_bemf()` |
| Day 4 | 实现位置估算 | `estimate_angle()` |
| Day 5 | 单元测试 + 调试 | 测试通过 |

**周 2: STO-PLL**

| 天数 | 任务 | 输出 |
|------|------|------|
| Day 1 | 阅读 `sto_pll_speed_pos_fdbk.c` | 算法笔记 |
| Day 2 | 创建 `src/foc/pll.rs` | 模块结构 |
| Day 3 | 实现 PLL 误差计算 + PI | `update()` |
| Day 4 | 集成 STO+PLL 到 `motor.rs` | 完整无传感器 |
| Day 5 | 实机测试 + 调试 | 电机旋转 |

**验收标准**:
- [ ] 电机能够无传感器启动并运行
- [ ] 速度估算误差 < 10%
- [ ] 能够带载运行

---

#### 第 2 阶段：启动完善 (1 周)

| 天数 | 任务 | 输出 |
|------|------|------|
| Day 1 | 完善 `RampGenerator`，添加 V/F 曲线 | `VfCurve` |
| Day 2 | 实现开环状态机 | `OpenLoopState` |
| Day 3 | 实现 RevUp 收敛检测 | `check_convergency()` |
| Day 4 | 开环→闭环切换逻辑 | `switch_to_closed_loop()` |
| Day 5 | 测试 + 调试 | 平滑启动 |

**验收标准**:
- [ ] 启动不失步
- [ ] 切换平滑无冲击
- [ ] 启动失败能重试

---

#### 第 3 阶段：保护功能 (1 周)

| 天数 | 任务 | 输出 |
|------|------|------|
| Day 1 | 母线电压检测 `bus_voltage.rs` | UVLO/OVLO |
| Day 2 | NTC 温度检测 `ntc.rs` | 过温保护 |
| Day 3-5 | Profiler 参数辨识 | 自动测量 Rs/Ls/Ke |

**验收标准**:
- [ ] 欠压/过压保护触发
- [ ] 过温保护触发 (70°C)
- [ ] Profiler 能自动测量参数

---

#### 第 4 阶段：高级功能 (可选)

| 任务 | 工作量 | 优先级 |
|------|--------|--------|
| 弱磁控制 | 2 天 | 低 |
| 编码器支持 | 3 天 | 低 |
| MC Protocol | 5 天 | 低 |

---

## 四、模块依赖关系

```
                    ┌─────────────────┐
                    │   Application   │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  FocController  │
                    └────────┬────────┘
                             │
           ┌─────────────────┼─────────────────┐
           │                 │                 │
    ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐
    │     STO     │  │    PLL      │  │   RevUp     │
    │  观测器     │  │  锁相环     │  │  启动切换   │
    └──────┬──────┘  └──────┬──────┘  └──────┬──────┘
           │                 │                 │
           └─────────────────┼─────────────────┘
                             │
                    ┌────────▼────────┐
                    │   OpenLoop      │
                    │   开环 V/F      │
                    └────────┬────────┘
                             │
           ┌─────────────────┼─────────────────┐
           │                 │                 │
    ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐
    │  BusVoltage │  │  NTC Temp   │  │  Profiler   │
    │  母线检测   │  │  温度检测   │  │  参数辨识   │
    └─────────────┘  └─────────────┘  └─────────────┘
                             │
                    ┌────────▼────────┐
                    │   PWM/ADC       │
                    │   驱动层        │
                    └─────────────────┘
```

---

## 五、代码量预估

| 阶段 | 模块 | 预估 Rust 代码量 |
|------|------|-----------------|
| **第 1 阶段** | STO 观测器 | ~400 行 |
| | STO-PLL | ~300 行 |
| **第 2 阶段** | 开环 V/F | ~200 行 |
| | RevUp | ~250 行 |
| **第 3 阶段** | 母线电压 | ~100 行 |
| | NTC 温度 | ~100 行 |
| | Profiler | ~400 行 |
| **第 4 阶段** | 弱磁控制 | ~200 行 |
| **总计** | | ~1,950 行 |

---

## 六、风险与缓解

| 风险 | 影响 | 概率 | 缓解措施 |
|------|------|------|----------|
| STO 参数调不准 | 高 | 中 | Profiler 自动辨识 |
| 启动失步 | 高 | 中 | RevUp 收敛检测 |
| 计算量过大 | 中 | 低 | 优化算法，用 `libm` |
| 栈溢出 | 中 | 低 | 检查中断栈使用 |
| 浮点性能 | 低 | 低 | M4F 有 FPU，20kHz 足够 |

---

## 七、测试计划

### 7.1 单元测试

```rust
// sto.rs
#[cfg(test)]
mod tests {
    #[test]
    fn test_bemf_estimation() { }

    #[test]
    fn test_angle_estimation() { }
}

// pll.rs
#[cfg(test)]
mod tests {
    #[test]
    fn test_pll_lock() { }
}
```

### 7.2 硬件在环测试

| 测试项 | 方法 | 通过标准 |
|--------|------|----------|
| 空载启动 | 命令启动 | 电机平稳加速 |
| 带载启动 | 加负载 | 不失步 |
| 速度阶跃 | 突然加速 | 无过流 |
| 欠压保护 | 降低电压 | 安全停机 |
| 过温保护 | 加热 NTC | 70°C 停机 |

---

## 八、当前进度追踪

### 8.1 已完成

- [x] Bootloader (90%)
- [x] PWM 驱动
- [x] ADC 驱动 (R3_2)
- [x] OPAMP 驱动
- [x] COMP 保护
- [x] FOC 基础 (Clarke/Park/PI/SVPWM)
- [x] HardFault 修复

### 8.2 进行中

- [ ] STO 观测器 (0%)
- [ ] STO-PLL (0%)

### 8.3 待开始

- [ ] 开环 V/F
- [ ] RevUp
- [ ] 母线电压
- [ ] NTC 温度
- [ ] Profiler

---

## 九、参考文档

1. ST MCSDK v6.4.1 源代码：`~/study/G431/G431/MCSDK_v6.4.1-Full`
2. STM32G4 参考手册：RM0440
3. B-G431B-ESC1 用户手册：UM2523
4. [FOC 原理教程](https://simplefoc.com/)

---

## 十、变更历史

| 版本 | 日期 | 变更内容 |
|------|------|----------|
| v1.0 | 2026-03-27 | 初始版本，基于 MCSDK 模块评估 |
