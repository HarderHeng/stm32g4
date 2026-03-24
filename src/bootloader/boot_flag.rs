//! Boot Flag management for STM32G4 Bootloader
//!
//! Boot Flag is stored in a dedicated Flash page at 0x08005000.
//! This allows the APP to request OTA mode by setting the flag
//! before triggering a system reset.
//!
//! ## APP watchdog protection
//!
//! When the IWDG fires in the APP, the MCU resets immediately — there is no
//! opportunity to write a Flash flag beforehand.  Instead the bootloader reads
//! `RCC_CSR.IWDGRSTF` on every boot:
//!   - Set  → APP watchdog fired; enter OTA mode, do NOT jump to APP.
//!   - Clear → normal path (check Boot Flag, validate APP, jump).
//!
//! `RCC_CSR.IWDGRSTF` survives a system reset but is cleared by a power-on
//! reset or when `RCC_CSR.RMVF` is written.  The bootloader clears it after
//! reading so the *next* soft reset (e.g. after OTA) is not misidentified.

use crate::shared::{BootFlag, BOOT_FLAG_ADDRESS, BOOT_STATE_OTA_REQUEST, BOOT_FLAG_MAGIC};

/// RCC_CSR register address (reset status flags)
const RCC_CSR: u32 = 0x4002_1094;
/// IWDG reset flag bit in RCC_CSR
const RCC_CSR_IWDGRSTF: u32 = 1 << 29;
/// Clear reset flags bit in RCC_CSR
const RCC_CSR_RMVF: u32 = 1 << 23;

/// Check if OTA mode is requested via Boot Flag
///
/// Returns true if Boot Flag state == OTA_REQUEST.
pub fn is_ota_requested() -> bool {
    read_boot_flag().map(|f| f.state == BOOT_STATE_OTA_REQUEST).unwrap_or(false)
}

/// Check `RCC_CSR.IWDGRSTF` to detect an APP watchdog timeout.
///
/// If the flag is set, it is **cleared immediately** (via `RMVF`) so that
/// subsequent soft resets (e.g. the RESET command at the end of OTA) are not
/// misidentified as watchdog resets.
///
/// Returns `true` when an IWDG reset is detected — the caller should enter
/// OTA mode and not jump to APP.
pub fn check_and_clear_iwdg_reset() -> bool {
    unsafe {
        let csr_ptr = RCC_CSR as *mut u32;
        let csr = core::ptr::read_volatile(csr_ptr);

        if csr & RCC_CSR_IWDGRSTF != 0 {
            // Write only RMVF to clear all reset flags; other bits are read-only
            core::ptr::write_volatile(csr_ptr, RCC_CSR_RMVF);
            true
        } else {
            false
        }
    }
}

/// Read Boot Flag from Flash
pub fn read_boot_flag() -> Option<BootFlag> {
    unsafe {
        let ptr = BOOT_FLAG_ADDRESS as *const BootFlag;

        // Read the flag (copy to avoid unaligned reference)
        let flag = core::ptr::read_volatile(ptr);

        // Validate magic (copy to local to avoid unaligned reference)
        let magic = flag.magic;
        if magic != BOOT_FLAG_MAGIC {
            return None;
        }

        // Validate CRC (copy to local to avoid unaligned reference)
        let state = flag.state;
        let crc32 = flag.crc32;
        let expected_crc = crate::shared::calculate_crc32(magic, state);
        if crc32 != expected_crc {
            return None;
        }

        Some(flag)
    }
}

/// Clear the OTA request flag (set to NORMAL state)
///
/// This is called by bootloader after detecting OTA request,
/// to prevent boot loop.
pub fn clear_ota_flag() -> Result<(), &'static str> {
    // Create normal flag
    let flag = BootFlag::new(crate::shared::BOOT_STATE_NORMAL);

    // Write to Flash
    crate::bootloader::flash::write_boot_flag(&flag)
}
