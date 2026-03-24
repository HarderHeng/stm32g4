//! Minimal Bootloader for STM32G4
//!
//! This bootloader:
//! - Checks Boot Flag for OTA request
//! - Validates APP (SP range, Reset Handler range)
//! - Jumps to APP if valid, otherwise enters OTA mode
//! - OTA mode: Simple UART protocol for firmware updates

#![no_std]
#![no_main]

use core::arch::asm;

use embassy_stm32g4_foc::bootloader::{
    is_ota_requested, clear_ota_flag,
    OtaHandler, uart, protocol::MAX_FRAME_SIZE,
};

// APP memory layout
const APP_START: u32 = 0x0800_5800;

// RAM bounds
const RAM_START: u32 = 0x2000_0000;
const RAM_END: u32 = 0x2000_8000;

#[cortex_m_rt::entry]
fn main() -> ! {
    // Check Boot Flag for OTA request
    if is_ota_requested() {
        // Clear the flag to prevent boot loop
        let _ = clear_ota_flag();
        // Enter OTA mode
        enter_ota_mode();
    }

    // Validate APP before jumping
    if let Some((sp, pc)) = validate_app() {
        jump_to_app(sp, pc);
    }

    // No valid APP, enter OTA mode
    enter_ota_mode();
}

/// Enter OTA mode - listen for UART commands
fn enter_ota_mode() -> ! {
    // Initialize UART
    uart::init();

    // Send startup message to confirm OTA mode
    uart::write_bytes(b"OTA\r\n");

    // Create OTA handler
    let mut handler = OtaHandler::new();
    let mut response_buf = [0u8; MAX_FRAME_SIZE];

    // Main OTA loop
    loop {
        // Check for incoming byte
        if uart::can_read() {
            let byte = uart::read_byte();
            let resp_len = handler.process_byte(byte, &mut response_buf);

            // Send response if we have one
            if resp_len > 0 {
                uart::write_bytes(&response_buf[..resp_len]);
            }
        }
        // Busy wait instead of wfi (no interrupts enabled)
    }
}

/// Validate APP and return (SP, PC) if valid
fn validate_app() -> Option<(u32, u32)> {
    unsafe {
        // Read vector table (APP starts directly with vector table)
        let vt = APP_START as *const u32;
        let sp = core::ptr::read_volatile(vt);
        let pc = core::ptr::read_volatile(vt.add(1));

        // Validate SP (must be in RAM, 8-byte aligned)
        if sp < RAM_START || sp > RAM_END {
            return None;
        }
        if sp & 0x7 != 0 {
            return None;
        }

        // Validate PC (must be in APP region, odd for Thumb)
        if pc < APP_START || pc > (APP_START + 0x20000) {
            return None;
        }
        if pc & 1 == 0 {
            return None;
        }

        Some((sp, pc))
    }
}

/// Jump to APP
fn jump_to_app(sp: u32, pc: u32) -> ! {
    unsafe {
        // Disable interrupts
        cortex_m::interrupt::disable();

        // Set VTOR to APP vector table
        let scb = 0xE000_ED00 as *mut u32;
        core::ptr::write_volatile(scb.add(0x08 / 4), APP_START);

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