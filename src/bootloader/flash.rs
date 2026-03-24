//! Flash operations for STM32G4 Bootloader
//!
//! Provides page-erase, data-write, and boot-flag-write operations used
//! by the bootloader.  All low-level register work is delegated to
//! `crate::flash_raw` so the implementation lives in exactly one place.

use crate::flash_raw;
use crate::shared::{BootFlag, BOOT_FLAG_ADDRESS, BOOT_FLAG_PAGE};

/// Erase a Flash page (0–63).
pub fn erase_page(page: u8) -> Result<(), &'static str> {
    flash_raw::erase_page(page)
}

/// Write data to Flash (address and length must be 8-byte aligned).
pub fn write(address: u32, data: &[u8]) -> Result<(), &'static str> {
    flash_raw::write(address, data)
}

/// Erase the Boot Flag page and write a new flag value.
pub fn write_boot_flag(flag: &BootFlag) -> Result<(), &'static str> {
    erase_page(BOOT_FLAG_PAGE)?;

    let flag_bytes = unsafe {
        core::slice::from_raw_parts(
            flag as *const _ as *const u8,
            core::mem::size_of::<BootFlag>(),
        )
    };

    write(BOOT_FLAG_ADDRESS, flag_bytes)
}
