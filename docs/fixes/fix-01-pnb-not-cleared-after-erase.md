# Fix 01: Flash CR.PNB 位在擦除完成后未清除

## 问题描述

STM32G4 的 Flash 控制器（FLASH_CR）包含以下擦除相关字段：

| 位域 | 名称 | 说明 |
|------|------|------|
| CR[1] | PER | Page Erase 使能位。写 1 选择页擦除模式 |
| CR[9:3] | PNB | Page Number Bits，指定要擦除的页号 |
| CR[16] | STRT | Start，写 1 触发擦除操作 |

在 `erase_page()` 完成后，原代码只清除了 `PER` 位，**没有清除 `PNB` 字段**：

```rust
// 原代码（flash.rs:100-101）
let val = core::ptr::read_volatile(cr);
core::ptr::write_volatile(cr, val & !cr::PER);  // 只清了 PER，PNB 仍保留页号
```

`shared.rs` 中的内联版本存在同样的问题（第155行）。

## 潜在危险

PNB 残留值不会直接导致重复擦除（因为 PER 位已被清除，STRT 触发也不会发生）。但这违反了"使用后恢复寄存器到安全状态"的原则，并且：

1. 若后续代码错误地读 CR 值并用于判断，残留的 PNB 会造成误判
2. STM32 参考手册（RM0440）推荐在操作完成后将 PNB 清零，以避免误触发风险
3. `shared.rs` 中的代码注释还写着 `// Erase page 8`，但实际操作的是 `BOOT_FLAG_PAGE = 10`，注释错误

## 修复方案

在清除 `PER` 的同时，一并清除 `PNB[6:0]`（位 9:3，掩码 `0x7F << 3 = 0x3F8`）：

```rust
// 修复后
let val = core::ptr::read_volatile(cr);
core::ptr::write_volatile(cr, val & !(cr::PER | (0x7F << 3)));
```

同时修正 `shared.rs` 中的错误注释。

## 修改文件

- `src/bootloader/flash.rs`：`erase_page()` 完成后同时清除 PER 和 PNB
- `src/shared.rs`：`write_boot_flag()` 中同步修复，并修正注释
