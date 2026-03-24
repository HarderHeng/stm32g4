//! Flash operations for STM32G4 Bootloader
//!
//! Provides low-level Flash erase and write operations.

use crate::shared::{BootFlag, BOOT_FLAG_ADDRESS, BOOT_FLAG_PAGE};

/// Flash key 1
const KEY1: u32 = 0x4567_0123;
/// Flash key 2
const KEY2: u32 = 0xCDEF_89AB;

/// Flash controller base address
const FLASH_BASE: usize = 0x4002_2000;

/// Flash registers offsets
mod off {
    pub const KEYR: usize = 0x04;
    pub const SR: usize = 0x0C;
    pub const CR: usize = 0x10;
}

/// CR register bits
mod cr {
    pub const PG: u32 = 1 << 0;
    pub const PER: u32 = 1 << 1;
    pub const STRT: u32 = 1 << 16;
    pub const LOCK: u32 = 1 << 31;
}

/// SR register bits
mod sr {
    pub const BSY: u32 = 1 << 16;
}

/// Unlock Flash for write/erase operations
fn unlock() {
    unsafe {
        let keyr = (FLASH_BASE + off::KEYR) as *mut u32;
        core::ptr::write_volatile(keyr, KEY1);
        core::ptr::write_volatile(keyr, KEY2);
    }
}

/// Lock Flash after operations
fn lock() {
    unsafe {
        let cr = (FLASH_BASE + off::CR) as *mut u32;
        let val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, val | cr::LOCK);
    }
}

/// Wait for Flash operation to complete
fn wait_ready() {
    unsafe {
        let sr = (FLASH_BASE + off::SR) as *const u32;
        while core::ptr::read_volatile(sr) & sr::BSY != 0 {
            cortex_m::asm::nop();
        }
    }
}

/// Erase a Flash page
///
/// # Arguments
///
/// * `page` - Page number (0-63 for STM32G431CB)
///
/// # Returns
///
/// Ok(()) on success, Err on failure
pub fn erase_page(page: u8) -> Result<(), &'static str> {
    if page > 63 {
        return Err("Invalid page number");
    }

    unlock();

    unsafe {
        let cr = (FLASH_BASE + off::CR) as *mut u32;

        // Set page number and PER bit
        let mut val = core::ptr::read_volatile(cr);
        val &= !(0x7F << 3); // Clear PNB[5:0]
        val |= (page as u32) << 3;
        val |= cr::PER;
        core::ptr::write_volatile(cr, val);

        // Start erase
        val |= cr::STRT;
        core::ptr::write_volatile(cr, val);
    }

    // Wait for completion
    wait_ready();

    unsafe {
        let cr = (FLASH_BASE + off::CR) as *mut u32;
        // Clear PER and PNB to restore CR to a safe state
        let val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, val & !(cr::PER | (0x7F << 3)));
    }

    lock();

    Ok(())
}

/// Write data to Flash (8-byte aligned)
///
/// # Arguments
///
/// * `address` - Flash address (must be 8-byte aligned)
/// * `data` - Data to write (must be multiple of 8 bytes)
pub fn write(address: u32, data: &[u8]) -> Result<(), &'static str> {
    if (address as usize) % 8 != 0 {
        return Err("Address not 8-byte aligned");
    }
    if data.len() % 8 != 0 {
        return Err("Data not multiple of 8 bytes");
    }

    unlock();

    unsafe {
        let cr = (FLASH_BASE + off::CR) as *mut u32;

        // Set PG bit
        let val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, val | cr::PG);

        // Write 8-byte chunks
        for (i, chunk) in data.chunks(8).enumerate() {
            wait_ready();

            let dest = (address as usize + i * 8) as *mut u64;
            let mut buf = [0u8; 8];
            buf[..chunk.len()].copy_from_slice(chunk);
            let val = u64::from_le_bytes(buf);
            core::ptr::write_volatile(dest, val);

            wait_ready();
        }

        // Clear PG bit
        let val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, val & !cr::PG);
    }

    lock();

    Ok(())
}

/// Write Boot Flag to Flash
///
/// Erases the Boot Flag page and writes the new flag.
pub fn write_boot_flag(flag: &BootFlag) -> Result<(), &'static str> {
    // Erase the page first
    erase_page(BOOT_FLAG_PAGE)?;

    // Convert flag to bytes
    let flag_bytes = unsafe {
        core::slice::from_raw_parts(
            flag as *const _ as *const u8,
            core::mem::size_of::<BootFlag>(),
        )
    };

    // Write the flag
    write(BOOT_FLAG_ADDRESS, flag_bytes)
}