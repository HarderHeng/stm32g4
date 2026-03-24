//! Simple UART driver for STM32G4 Bootloader
//!
//! Direct register access without Embassy dependencies.

/// USART2 base address
const USART2_BASE: usize = 0x4000_4400;

/// Register offsets
mod off {
    pub const CR1: usize = 0x00;
    pub const CR2: usize = 0x04;
    pub const CR3: usize = 0x08;
    pub const BRR: usize = 0x0C;
    pub const ISR: usize = 0x1C;
    pub const RDR: usize = 0x24;
    pub const TDR: usize = 0x28;
}

/// CR1 bits
mod cr1 {
    pub const UE: u32 = 1 << 0;
    pub const TE: u32 = 1 << 3;
    pub const RE: u32 = 1 << 2;
}

/// ISR bits
mod isr {
    pub const TXE: u32 = 1 << 7;
    pub const RXNE: u32 = 1 << 5;
}

/// Initialize USART2 for bootloader
///
/// Configures USART2 with:
/// - 115200 baud (assumes 16MHz HSI clock, default after reset)
/// - 8N1
/// - TX on PB3, RX on PB4
pub fn init() {
    unsafe {
        // Enable GPIOB and USART2 clocks
        let rcc = 0x4002_1000 as *mut u32;
        // AHB2ENR: GPIOBEN (bit 1)
        let ahb2enr = rcc.add(0x4C / 4);
        core::ptr::write_volatile(ahb2enr, core::ptr::read_volatile(ahb2enr) | (1 << 1));
        // APB1ENR1: USART2EN (bit 17)
        let apb1enr1 = rcc.add(0x40 / 4);
        core::ptr::write_volatile(apb1enr1, core::ptr::read_volatile(apb1enr1) | (1 << 17));

        // Small delay for clock stabilization
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }

        // Configure PB3 (TX) and PB4 (RX) as AF7
        let gpiob = 0x4800_0400 as *mut u32;

        // Set mode to Alternate Function (mode = 10)
        let moder = gpiob.add(0x00 / 4);
        let mut mode_val = core::ptr::read_volatile(moder);
        mode_val &= !(0b11 << (3 * 2)); // Clear PB3 mode
        mode_val &= !(0b11 << (4 * 2)); // Clear PB4 mode
        mode_val |= 0b10 << (3 * 2);    // PB3: AF mode
        mode_val |= 0b10 << (4 * 2);    // PB4: AF mode
        core::ptr::write_volatile(moder, mode_val);

        // Set AF7 for PB3 and PB4
        let afrl = gpiob.add(0x20 / 4);
        let mut af_val = core::ptr::read_volatile(afrl);
        af_val &= !(0xF << (3 * 4));  // Clear PB3 AF
        af_val &= !(0xF << (4 * 4));  // Clear PB4 AF
        af_val |= 7 << (3 * 4);       // PB3: AF7
        af_val |= 7 << (4 * 4);       // PB4: AF7
        core::ptr::write_volatile(afrl, af_val);

        // Set output speed to high
        let ospeedr = gpiob.add(0x08 / 4);
        let mut speed_val = core::ptr::read_volatile(ospeedr);
        speed_val |= 0b11 << (3 * 2); // PB3: High speed
        speed_val |= 0b11 << (4 * 2); // PB4: High speed
        core::ptr::write_volatile(ospeedr, speed_val);

        // Configure USART2
        let usart2 = USART2_BASE as *mut u32;

        // Disable USART first
        let cr1 = usart2.add(off::CR1 / 4);
        core::ptr::write_volatile(cr1, 0);

        // Set baud rate
        // Default clock is HSI 16MHz (not configured by bootloader)
        // BRR = 16_000_000 / 115200 = 138.89 ≈ 139 = 0x8B
        let brr = usart2.add(off::BRR / 4);
        core::ptr::write_volatile(brr, 139);

        // Enable TX and RX
        core::ptr::write_volatile(cr1, cr1::UE | cr1::TE | cr1::RE);
    }
}

/// Check if a byte is available to read
pub fn can_read() -> bool {
    unsafe {
        let isr = (USART2_BASE + off::ISR) as *const u32;
        core::ptr::read_volatile(isr) & isr::RXNE != 0
    }
}

/// Read a byte (blocking)
pub fn read_byte() -> u8 {
    unsafe {
        let isr = (USART2_BASE + off::ISR) as *const u32;
        let rdr = (USART2_BASE + off::RDR) as *const u32;

        // Wait for data
        while core::ptr::read_volatile(isr) & isr::RXNE == 0 {
            cortex_m::asm::nop();
        }

        core::ptr::read_volatile(rdr) as u8
    }
}

/// Write a byte (blocking)
pub fn write_byte(byte: u8) {
    unsafe {
        let isr = (USART2_BASE + off::ISR) as *const u32;
        let tdr = (USART2_BASE + off::TDR) as *mut u32;

        // Wait for TX empty
        while core::ptr::read_volatile(isr) & isr::TXE == 0 {
            cortex_m::asm::nop();
        }

        core::ptr::write_volatile(tdr, byte as u32);
    }
}

/// Write multiple bytes
pub fn write_bytes(data: &[u8]) {
    for &byte in data {
        write_byte(byte);
    }
}