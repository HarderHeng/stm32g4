# Fix 03: shared.rs 与 bootloader/flash.rs 中 Flash 操作代码重复

## 问题描述

项目中存在两份独立实现的 Flash 底层操作代码：

### 版本 A：`src/bootloader/flash.rs`
- 结构清晰：`unlock()` / `lock()` / `wait_ready()` 三个私有辅助函数
- 提供 `erase_page()` / `write()` 两个公开接口

### 版本 B：`src/shared.rs` 内的 `write_boot_flag()`
- 约 70 行内联的寄存器操作
- 完全复制了版本 A 的逻辑（unlock → erase → write → lock）
- 在 `shared.rs` 是共享模块（bootloader 和 app 都编译），而 `bootloader/flash.rs` 只在 `bootloader` feature 下编译

### 两份代码的不一致风险

| 场景 | 问题 |
|------|------|
| 一处修了 bug，另一处没修 | Fix 01 就是示例：需要同时修两处 |
| 算法细节差异 | 版本 B 中 `wait_ready` 没有 `nop()`（spin loop 裸等）|
| 维护成本翻倍 | 每次修改都需要同步两处 |

## 根本原因

`shared.rs` 需要被 APP 和 Bootloader 双端编译，而 `bootloader/flash.rs` 只在 `bootloader` feature 下可用。APP 调用 `request_ota()` 时无法访问 `bootloader/flash.rs`，所以开发者在 `shared.rs` 里内联了一份。

## 修复方案

新增 `src/flash_raw.rs` 模块，包含纯底层的 Flash 寄存器操作（无业务逻辑），不受任何 feature 限制，始终编译：

```
src/
├── flash_raw.rs        ← 新增：unlock/lock/wait_ready/erase_page/write
├── shared.rs           ← 调用 crate::flash_raw
├── lib.rs              ← pub(crate) mod flash_raw
└── bootloader/
    └── flash.rs        ← 委托给 crate::flash_raw，保留 write_boot_flag
```

`shared.rs::write_boot_flag()` 内联代码删除，改为调用 `flash_raw::erase_page()` + `flash_raw::write()`。

`bootloader/flash.rs` 的 `unlock/lock/wait_ready` 等私有函数也改为委托 `flash_raw`，消除重复。

这样：
- Flash 底层逻辑只有一份，在一处修改即可全局生效
- `flash_raw` 不依赖任何 feature，编译总是成功
- `bootloader/flash.rs` 保留了 `write_boot_flag` 等业务逻辑的正确归属

## 修改文件

- `src/flash_raw.rs`：新建，包含所有 Flash 底层原语
- `src/lib.rs`：增加 `pub(crate) mod flash_raw`
- `src/shared.rs`：`write_boot_flag` 内联代码替换为 `flash_raw` 调用
- `src/bootloader/flash.rs`：私有辅助函数委托给 `flash_raw`
