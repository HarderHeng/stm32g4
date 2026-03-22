//! Minimal Bootloader for STM32G4
//!
//! This bootloader:
//! - Initializes minimal hardware
//! - Unconditionally jumps to APP at 0x08008800

#![no_std]
#![no_main]

use core::arch::asm;

// APP memory layout
const APP_START: u32 = 0x0800_8800;

#[cortex_m_rt::entry]
fn main() -> ! {
    // Jump to APP immediately
    unsafe {
        // Disable interrupts
        cortex_m::interrupt::disable();

        // Set VTOR to APP vector table
        let scb = 0xE000_ED00 as *mut u32;
        core::ptr::write_volatile(scb.add(0x08 / 4), APP_START);

        // Read initial SP and Reset Handler from APP vector table
        let vt = APP_START as *const u32;
        let sp = core::ptr::read_volatile(vt);
        let pc = core::ptr::read_volatile(vt.add(1));

        // Set MSP and jump
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

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}