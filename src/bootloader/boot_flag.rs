//! Boot Flag management for STM32G4 Bootloader
//!
//! Boot Flag is stored in a dedicated Flash page at 0x08008000.
//! This allows the APP to request OTA mode by setting the flag
//! before triggering a system reset.

use crate::shared::{BootFlag, BOOT_FLAG_ADDRESS, BOOT_STATE_OTA_REQUEST, BOOT_FLAG_MAGIC};

/// Check if OTA mode is requested
///
/// Returns true if:
/// - Boot Flag exists and state == OTA_REQUEST
///
/// Returns false if:
/// - Boot Flag doesn't exist (erased Flash)
/// - Boot Flag state == NORMAL
/// - Boot Flag is corrupted
pub fn is_ota_requested() -> bool {
    read_boot_flag().map(|f| f.state == BOOT_STATE_OTA_REQUEST).unwrap_or(false)
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