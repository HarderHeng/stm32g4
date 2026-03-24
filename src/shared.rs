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
    // Create OTA request flag
    let flag = BootFlag::new(BOOT_STATE_OTA_REQUEST);

    // Write flag to Flash
    unsafe {
        write_boot_flag(&flag);
    }

    // Small delay to ensure Flash write completes
    for _ in 0..10_000 {
        cortex_m::asm::nop();
    }

    // System reset
    cortex_m::peripheral::SCB::sys_reset();
}

/// Write Boot Flag to Flash (unsafe, direct register access)
///
/// # Safety
///
/// This function directly accesses Flash controller registers.
unsafe fn write_boot_flag(flag: &BootFlag) {
    unsafe {
        // Flash controller base address
        const FLASH_BASE: usize = 0x4002_2000;
        const KEYR: usize = 0x04;
        const SR: usize = 0x0C;
        const CR: usize = 0x10;

        const KEY1: u32 = 0x4567_0123;
        const KEY2: u32 = 0xCDEF_89AB;

        const CR_PER: u32 = 1 << 1;
        const CR_STRT: u32 = 1 << 16;
        const CR_PG: u32 = 1 << 0;
        const CR_LOCK: u32 = 1 << 31;
        const SR_BSY: u32 = 1 << 16;

        // Unlock Flash
        let keyr = (FLASH_BASE + KEYR) as *mut u32;
        core::ptr::write_volatile(keyr, KEY1);
        core::ptr::write_volatile(keyr, KEY2);

        // Erase Boot Flag page (page 10)
        let cr = (FLASH_BASE + CR) as *mut u32;
        let sr = (FLASH_BASE + SR) as *const u32;

        // Set page number and PER bit
        let mut cr_val = core::ptr::read_volatile(cr);
        cr_val &= !(0x7F << 3); // Clear PNB
        cr_val |= (BOOT_FLAG_PAGE as u32) << 3; // Set PNB = BOOT_FLAG_PAGE
        cr_val |= CR_PER; // Page erase
        core::ptr::write_volatile(cr, cr_val);

        // Start erase
        cr_val |= CR_STRT;
        core::ptr::write_volatile(cr, cr_val);

        // Wait for completion
        while core::ptr::read_volatile(sr) & SR_BSY != 0 {}

        // Clear PER and PNB to restore CR to a safe state
        let cr_val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, cr_val & !(CR_PER | (0x7F << 3)));

        // Write flag
        let cr_val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, cr_val | CR_PG);

        // Convert flag to bytes and write
        let flag_bytes = core::slice::from_raw_parts(
            flag as *const _ as *const u8,
            core::mem::size_of::<BootFlag>(),
        );

        // Write in 8-byte chunks
        for (i, chunk) in flag_bytes.chunks(8).enumerate() {
            // Wait if busy
            while core::ptr::read_volatile(sr) & SR_BSY != 0 {}

            let dest = (BOOT_FLAG_ADDRESS as usize + i * 8) as *mut u64;
            let mut padded = [0u8; 8];
            padded[..chunk.len()].copy_from_slice(chunk);
            let value = u64::from_le_bytes(padded);
            core::ptr::write_volatile(dest, value);

            // Wait for completion
            while core::ptr::read_volatile(sr) & SR_BSY != 0 {}
        }

        // Clear PG bit and lock
        let cr_val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, (cr_val & !CR_PG) | CR_LOCK);
    }
}