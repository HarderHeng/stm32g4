//! Shared functions for Bootloader and APP
//!
//! Provides functions for both bootloader and APP to interact with
//! the Boot Flag stored in Flash.

/// Boot Flag Flash address (Page 10, after 20KB bootloader)
pub const BOOT_FLAG_ADDRESS: u32 = 0x0800_5000;

/// Boot Flag page number
pub const BOOT_FLAG_PAGE: u8 = 10;

/// Boot Flag magic value ("BOOT")
pub const BOOT_FLAG_MAGIC: u32 = 0x424F_4F54;

/// Normal boot - jump to APP if valid
pub const BOOT_STATE_NORMAL: u32 = 0x0000_0000;

/// OTA requested - enter OTA mode
pub const BOOT_STATE_OTA_REQUEST: u32 = 0x4F54_4131; // "OTA1"

/// Boot Flag structure (stored in Flash)
#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub struct BootFlag {
    /// Magic number (0x424F4F54 = "BOOT")
    pub magic: u32,
    /// State value
    pub state: u32,
    /// CRC32 checksum of magic + state
    pub crc32: u32,
    /// Reserved, padding to 2KB
    pub reserved: [u8; 2044],
}

impl BootFlag {
    /// Create a new Boot Flag with the given state
    pub fn new(state: u32) -> Self {
        let mut flag = Self {
            magic: BOOT_FLAG_MAGIC,
            state,
            crc32: 0,
            reserved: [0; 2044],
        };
        flag.crc32 = calculate_crc32(BOOT_FLAG_MAGIC, state);
        flag
    }

    /// Check if OTA mode is requested
    pub fn is_ota_requested(&self) -> bool {
        self.state == BOOT_STATE_OTA_REQUEST
    }
}

/// Calculate CRC32 of magic + state
pub fn calculate_crc32(magic: u32, state: u32) -> u32 {
    let bytes: [u8; 8] = [
        magic as u8,
        (magic >> 8) as u8,
        (magic >> 16) as u8,
        (magic >> 24) as u8,
        state as u8,
        (state >> 8) as u8,
        (state >> 16) as u8,
        (state >> 24) as u8,
    ];

    let mut crc: u32 = 0xFFFF_FFFF;

    for &byte in &bytes {
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }

    !crc
}

/// Request OTA mode
///
/// This function:
/// 1. Erases the Boot Flag page
/// 2. Writes OTA_REQUEST state
/// 3. Triggers system reset
///
/// The bootloader will detect the flag and enter OTA mode.
pub fn request_ota() -> ! {
    let flag = BootFlag::new(BOOT_STATE_OTA_REQUEST);

    let flag_bytes = unsafe {
        core::slice::from_raw_parts(
            &flag as *const _ as *const u8,
            core::mem::size_of::<BootFlag>(),
        )
    };

    // Erase then write via the shared flash_raw primitives.
    // Errors are ignored here — if the write fails the bootloader will
    // not see a valid OTA flag and will simply jump to APP again.
    let _ = crate::flash_raw::erase_page(BOOT_FLAG_PAGE);
    let _ = crate::flash_raw::write(BOOT_FLAG_ADDRESS, flag_bytes);

    // System reset — bootloader will inspect the flag on next boot
    cortex_m::peripheral::SCB::sys_reset();
}