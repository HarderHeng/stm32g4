# STM32G4 Bootloader 与 OTA 开发文档

## 当前状态

**已完成**: 基础 Bootloader 框架（无条件跳转到 APP）

| 功能 | 状态 |
|------|------|
| Bootloader 跳转 APP | ✓ 已验证 |
| Boot Flag 机制 | 待实现 |
| APP 校验 (SP/PC/CRC) | 待实现 |
| UART OTA 协议 | 待实现 |

## 验证结果

```
=== Bootloader ===
- 大小: 1.2KB
- 地址: 0x08000000
- 功能: 无条件跳转到 APP

=== APP ===
- 大小: 32KB
- 地址: 0x08008800
- 验证: 跳转成功，APP 正常运行
```

---

## 1. 系统概述

### 1.1 设计目标

- Bootloader占用最小Flash空间
- 使用Flash存储OTA标志位（不使用RAM共享）
- UART2复用：Bootloader用于OTA，APP用于Shell
- 可靠的APP校验与跳转机制
- 简洁高效的OTA协议

### 1.2 启动流程

```
┌─────────────────────────────────────────────────────────────┐
│                        上电启动                              │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│              读取Flash标志位 (BOOT_FLAG区域)                  │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
              ┌───────────────────────┐
              │ 标志位 == OTA_REQUEST? │
              └───────────┬───────────┘
                    │           │
                   YES          NO
                    │           │
                    ▼           ▼
          ┌──────────────┐  ┌─────────────────────────┐
          │ 清除标志位    │  │ 检查APP有效性            │
          │ 进入OTA模式   │  │ - Header Magic          │
          └──────────────┘  │ - Stack Pointer范围     │
                            │ - Reset Handler范围     │
                            └───────────┬─────────────┘
                                        │
                              ┌─────────┴─────────┐
                              │                   │
                           有效                  无效
                              │                   │
                              ▼                   ▼
                    ┌──────────────┐    ┌──────────────┐
                    │ 设置VTOR     │    │ 进入OTA模式   │
                    │ 设置MSP      │    └──────────────┘
                    │ 跳转APP      │
                    └──────────────┘
```

### 1.3 OTA请求流程

```
┌─────────────────────────────────────────────────────────────┐
│                  APP运行中 (UART2 = Shell)                   │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │ 用户输入: system ota  │
              └───────────┬───────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │ 写入OTA_REQUEST标志   │
              │ 到Flash BOOT_FLAG区域 │
              └───────────┬───────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │     软重启 (NVIC_Reset)│
              └───────────┬───────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │   Bootloader接管      │
              │   进入OTA模式         │
              └───────────────────────┘
```

---

## 2. 内存布局设计

### 2.1 Flash分配 (STM32G431CB: 128KB, 64页, 每页2KB)

| 区域 | 起始地址 | 大小 | 页范围 | 用途 |
|------|----------|------|--------|------|
| BOOTLOADER | 0x08000000 | 16KB | 0-7 | Bootloader代码 |
| BOOT_FLAG | 0x08008000 | 2KB | 8 | OTA标志位 (1页) |
| APP | 0x08008800 | 110KB | 9-63 | 应用程序 |

### 2.2 Bootloader区域 (16KB)

- 目标大小: < 12KB (预留空间)
- 包含: 初始化代码 + APP校验 + 跳转 + UART OTA协议

### 2.3 Boot Flag区域 (2KB = 1页)

```c
// 位于 0x08008000
struct BootFlag {
    u32 magic;        // 0x424F4F54 = "BOOT"
    u32 state;        // 状态值
    u32 crc32;        // CRC32校验
    u8  reserved[2044]; // 保留，填充到2KB
};

// 状态值定义
#define BOOT_STATE_NORMAL     0x00000000  // 正常启动
#define BOOT_STATE_OTA_REQUEST 0x4F544131  // "OTA1" - 请求进入OTA模式
#define BOOT_STATE_OTA_UPDATING 0x4F544132 // "OTA2" - 正在更新(可选)
```

### 2.4 APP区域结构

```
0x08008800 ┌────────────────────────────┐
           │   APP Header (256 bytes)    │
           │   - magic: 0x41505031       │
           │   - version                 │
           │   - length                  │
           │   - crc32                   │
           ├────────────────────────────┤
           │   Vector Table              │
           │   - Initial SP              │
           │   - Reset Handler           │
           │   - Exceptions...           │
           ├────────────────────────────┤
           │   Code (.text)              │
           │   ...                       │
           └────────────────────────────┘
```

### 2.5 APP Header定义

```c
struct AppHeader {
    u32 magic;        // 0x41505031 = "APP1"
    u32 version;      // 固件版本
    u32 length;       // 固件长度(不含header)
    u32 reserved1;    // 保留
    u32 crc32;        // 固件CRC32 (IEEE 802.3)
    u32 entry;        // 入口地址 = 0x08008900
    u8  reserved[232]; // 保留到256字节
};
```

---

## 3. Bootloader设计

### 3.1 核心功能模块

```
┌─────────────────────────────────────────────────────────────┐
│                        Bootloader                            │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │   Start     │  │   Flash     │  │      UART/OTA       │  │
│  │   - Clock   │  │   - Unlock  │  │      - Protocol     │  │
│  │   - GPIO    │  │   - Erase   │  │      - CRC          │  │
│  │   - UART    │  │   - Write   │  │      - Commands     │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
│                                                              │
│  ┌─────────────┐  ┌─────────────────────────────────────┐   │
│  │   Boot      │  │           APP Jump                   │   │
│  │   Flag      │  │   - Validate Header                  │   │
│  │   - Read    │  │   - Validate SP (0x20000000-0x20007FFF)│  │
│  │   - Write   │  │   - Set VTOR = 0x08008800            │   │
│  │   - Clear   │  │   - Set MSP = Initial SP             │   │
│  └─────────────┘  │   - Jump to Reset Handler            │   │
│                    └─────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 最小化策略

1. **不使用embassy executor** - 纯裸机循环
2. **不使用defmt** - 减少依赖
3. **最小化HAL使用** - 直接操作寄存器
4. **精简OTA协议** - 仅保留必要命令
5. **使用静态内存** - 避免动态分配

### 3.3 Bootloader伪代码

```rust
#[entry]
fn main() -> ! {
    // 1. 最小初始化
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // 初始化时钟 (使用默认HSI，足够UART使用)
    // 初始化GPIO (UART2 TX/RX)
    // 初始化UART2 (921600 baud)

    // 2. 检查Boot Flag
    let boot_flag = read_boot_flag();

    if boot_flag.state == BOOT_STATE_OTA_REQUEST {
        // 清除标志
        clear_boot_flag();
        // 进入OTA模式
        enter_ota_mode(&mut uart);
    }

    // 3. 检查APP有效性
    if let Some(header) = validate_app() {
        // 校验Stack Pointer
        let sp = read_initial_sp();
        if is_valid_sp(sp) {
            // 跳转APP
            jump_to_app(header);
        }
    }

    // 4. APP无效，进入OTA模式
    enter_ota_mode(&mut uart);
}

fn validate_app() -> Option<AppHeader> {
    let header = read_app_header();

    // 检查magic
    if header.magic != APP_MAGIC { return None; }

    // 检查长度
    if header.length > APP_MAX_SIZE { return None; }

    // 验证CRC32
    if !verify_crc32(header) { return None; }

    Some(header)
}

fn is_valid_sp(sp: u32) -> bool {
    // SP必须在RAM范围内，且8字节对齐
    (sp >= 0x20000000) && (sp <= 0x20007FFF) && (sp & 0x7 == 0)
}

fn jump_to_app(header: AppHeader) -> ! {
    unsafe {
        // 禁用中断
        cortex_m::interrupt::disable();

        // 设置VTOR
        (*SCB::PTR).vtor.write(APP_START as u32);

        // 读取向量表
        let vt = APP_START as *const u32;
        let sp = ptr::read_volatile(vt);
        let pc = ptr::read_volatile(vt.add(1));

        // 设置MSP并跳转
        asm!(
            "msr msp, {sp}",
            "isb",
            "bx {pc}",
            sp = in(reg) sp,
            pc = in(reg) pc,
            options(noreturn)
        );
    }
}
```

---

## 4. OTA协议设计

### 4.1 协议帧格式

```
┌──────┬──────┬──────────┬──────────┬────────┬──────┐
│ SOF  │ CMD  │   LEN    │   DATA   │  CRC16 │ EOF  │
│ 1B   │ 1B   │   2B     │  0-252B  │   2B   │ 1B   │
└──────┴──────┴──────────┴──────────┴────────┴──────┘

SOF = 0x7E
EOF = 0x7F
LEN = 数据长度(大端)
CRC16 = CRC-CCITT (大端)
```

### 4.2 命令定义

| CMD | 名称 | 方向 | DATA | 描述 |
|-----|------|------|------|------|
| 0x01 | PING | 请求/响应 | - | 心跳检测 |
| 0x02 | INFO | 响应 | [ver:2B, app_valid:1B] | 查询信息 |
| 0x10 | ERASE | 请求 | [size:4B] | 擦除APP区域 |
| 0x11 | WRITE | 请求 | [offset:4B, data:0-248B] | 写入数据 |
| 0x12 | VERIFY | 请求 | - | 校验固件 |
| 0x13 | BOOT | 请求 | - | 跳转APP |

### 4.3 响应格式

```
┌──────┬──────┬──────────┬──────────┬────────┬──────┐
│ SOF  │ CMD  │   LEN    │ STATUS+  │  CRC16 │ EOF  │
│      │      │          │  DATA    │        │      │
└──────┴──────┴──────────┴──────────┴────────┴──────┘

STATUS:
  0x00 = OK
  0x01 = ERROR
  0x02 = INVALID_CMD
  0x03 = INVALID_PARAM
  0x04 = FLASH_ERROR
  0x05 = CRC_ERROR
  0x06 = NO_APP
```

### 4.4 OTA流程时序

```
Host                           Device
  │                               │
  │──── PING ────────────────────>│
  │<─── PING (OK) ────────────────│
  │                               │
  │──── INFO ────────────────────>│
  │<─── INFO (ver, app_valid) ────│
  │                               │
  │──── ERASE [size] ────────────>│
  │<─── ERASE (OK) ───────────────│
  │                               │
  │──── WRITE [0, data] ─────────>│
  │<─── WRITE (OK, next_offset) ──│
  │     ... (重复写入)            │
  │                               │
  │──── VERIFY ──────────────────>│
  │<─── VERIFY (OK) ──────────────│
  │                               │
  │──── BOOT ────────────────────>│
  │<─── BOOT (OK) ────────────────│
  │                               │
  │                    [跳转APP]  │
```

---

## 5. APP修改要点

### 5.1 启动设置

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 1. 设置VTOR
    unsafe {
        (*cortex_m::peripheral::SCB::PTR).vtor.write(0x08008800);
    }

    // 2. 正常初始化...
}
```

### 5.2 Shell OTA命令

```rust
Command::System { command } => match command {
    SystemCommand::Ota => {
        writer.write_str("Entering OTA mode...\r\n");
        // 写入OTA请求标志
        set_ota_request_flag();
        // 延时确保输出完成
        delay_ms(100);
        // 软重启
        cortex_m::peripheral::SCB::sys_reset();
    }
}
```

---

## 6. 开发步骤

### 阶段1: 基础Bootloader框架
1. [x] 创建最小化Bootloader项目结构
2. [x] 实现基本的时钟和UART初始化 (不需要，直接跳转)
3. [x] 实现APP Header读取和校验 (跳过，无条件跳转)
4. [x] 实现APP跳转功能

### 阶段2: Boot Flag机制
1. [ ] 定义Boot Flag结构
2. [ ] 实现Flash标志位读写
3. [ ] 集成标志位检查到启动流程

### 阶段3: OTA协议实现
1. [ ] 实现OTA帧解析
2. [ ] 实现PING/INFO命令
3. [ ] 实现Flash擦写操作
4. [ ] 实现ERASE/WRITE命令
5. [ ] 实现VERIFY/BOOT命令

### 阶段4: APP集成
1. [ ] 修改APP内存配置
2. [ ] 添加VTOR设置
3. [ ] 添加OTA Shell命令
4. [ ] 实现标志位写入函数

### 阶段5: 测试与验证
1. [ ] 编写OTA Host工具
2. [ ] 测试正常启动流程
3. [ ] 测试OTA请求流程
4. [ ] 测试固件更新流程

---

## 7. 文件结构

```
stm32g4/
├── Cargo.toml
├── memory-bootloader.x      # Bootloader内存配置
├── memory-app.x             # APP内存配置
├── build.rs
└── src/
    ├── lib.rs
    ├── shared.rs            # 共享常量和类型
    ├── bootloader/
    │   ├── mod.rs
    │   ├── start.rs         # 启动初始化 (裸机)
    │   ├── flash.rs         # Flash操作
    │   ├── boot_flag.rs     # 标志位操作
    │   ├── app.rs           # APP校验跳转
    │   └── ota.rs           # OTA协议
    ├── driver/              # APP驱动
    ├── bsp/                 # APP BSP
    └── bin/
        ├── bootloader.rs    # Bootloader入口
        └── main.rs          # APP入口
```

---

## 8. 关键代码规范

### 8.1 Bootloader代码要求
- 不使用alloc
- 不使用embassy executor
- 不使用defmt (可选简单调试输出)
- 使用静态缓冲区
- 最小化依赖

### 8.2 Flash操作要求
- 写入前必须擦除
- 按页擦除 (2KB)
- 按字写入 (4字节对齐)
- 操作时禁用中断

### 8.3 跳转前要求
- 禁用所有中断
- 清除所有pending中断
- 设置VTOR
- 设置MSP
- 确保ICache/DCache一致性