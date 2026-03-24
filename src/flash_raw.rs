//! Raw Flash register primitives for STM32G431CB
//!
//! This module contains the lowest-level Flash controller operations:
//! unlock, lock, wait-ready, page-erase, and double-word write.
//!
//! It is compiled unconditionally (no feature gate) so both the APP
//! (via `shared::request_ota`) and the Bootloader (via `bootloader::flash`)
//! can share a single implementation without duplication.

/// Flash controller base address (RM0440 §3.7)
const FLASH_BASE: usize = 0x4002_2000;

/// Flash unlock keys
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

/// Register offsets
mod off {
    pub const KEYR: usize = 0x04;
    pub const SR:   usize = 0x0C;
    pub const CR:   usize = 0x10;
}

/// FLASH_CR bit definitions
mod cr {
    /// Programming enable
    pub const PG:       u32 = 1 << 0;
    /// Page erase request
    pub const PER:      u32 = 1 << 1;
    /// Page number field mask (PNB[6:0] at bits [9:3])
    pub const PNB_MASK: u32 = 0x7F << 3;
    /// Start erase operation
    pub const STRT:     u32 = 1 << 16;
    /// Lock flash (write 1 to lock, cleared by unlock sequence)
    pub const LOCK:     u32 = 1 << 31;
}

/// FLASH_SR bit definitions
mod sr {
    /// Busy flag
    pub const BSY: u32 = 1 << 16;
}

/// Unlock the Flash controller for write/erase operations.
pub(crate) fn unlock() {
    unsafe {
        let keyr = (FLASH_BASE + off::KEYR) as *mut u32;
        core::ptr::write_volatile(keyr, KEY1);
        core::ptr::write_volatile(keyr, KEY2);
    }
}

/// Lock the Flash controller.
pub(crate) fn lock() {
    unsafe {
        let cr = (FLASH_BASE + off::CR) as *mut u32;
        let val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, val | cr::LOCK);
    }
}

/// Spin until the Flash BSY flag clears.
pub(crate) fn wait_ready() {
    unsafe {
        let sr = (FLASH_BASE + off::SR) as *const u32;
        while core::ptr::read_volatile(sr) & sr::BSY != 0 {
            cortex_m::asm::nop();
        }
    }
}

/// Erase a single 2 KB Flash page.
///
/// `page` must be in the range 0–63 (STM32G431CB has 64 pages of 2 KB each).
///
/// After the erase completes, both PER and PNB are cleared in CR to
/// restore the register to a safe idle state (per RM0440 recommendation).
pub(crate) fn erase_page(page: u8) -> Result<(), &'static str> {
    if page > 63 {
        return Err("Invalid page number");
    }

    unlock();

    unsafe {
        let cr = (FLASH_BASE + off::CR) as *mut u32;

        // Load page number into PNB field and set PER
        let mut val = core::ptr::read_volatile(cr);
        val &= !cr::PNB_MASK;
        val |= (page as u32) << 3;
        val |= cr::PER;
        core::ptr::write_volatile(cr, val);

        // Trigger erase
        val |= cr::STRT;
        core::ptr::write_volatile(cr, val);
    }

    wait_ready();

    unsafe {
        let cr = (FLASH_BASE + off::CR) as *mut u32;
        // Clear PER + PNB together — leaving PNB set is a latent hazard
        let val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, val & !(cr::PER | cr::PNB_MASK));
    }

    lock();
    Ok(())
}

/// Write a block of data to Flash.
///
/// Both `address` and `data.len()` must be multiples of 8 (STM32G4 requires
/// double-word writes).  The caller is responsible for erasing the target
/// pages before writing.
pub(crate) fn write(address: u32, data: &[u8]) -> Result<(), &'static str> {
    if (address as usize) % 8 != 0 {
        return Err("Address not 8-byte aligned");
    }
    if data.len() % 8 != 0 {
        return Err("Data length not a multiple of 8");
    }

    unlock();

    unsafe {
        let cr = (FLASH_BASE + off::CR) as *mut u32;

        // Enable programming mode
        let val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, val | cr::PG);

        for (i, chunk) in data.chunks(8).enumerate() {
            wait_ready();

            let dest = (address as usize + i * 8) as *mut u64;
            let mut buf = [0u8; 8];
            buf[..chunk.len()].copy_from_slice(chunk);
            let word = u64::from_le_bytes(buf);
            core::ptr::write_volatile(dest, word);

            wait_ready();
        }

        // Disable programming mode
        let val = core::ptr::read_volatile(cr);
        core::ptr::write_volatile(cr, val & !cr::PG);
    }

    lock();
    Ok(())
}
