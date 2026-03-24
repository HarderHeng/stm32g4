//! Simple UART driver for STM32G4 Bootloader
//!
//! Direct register access, no Embassy dependencies.
//! Clock: HSE 8MHz → PLL 170MHz, APB1 85MHz
//! UART:  USART2, 921600 8N1, TX=PB3, RX=PB4

/// RCC base address
const RCC: usize = 0x4002_1000;

// RCC register offsets
mod rcc_off {
    pub const CR: usize       = 0x00;
    pub const CFGR: usize     = 0x08;
    pub const PLLCFGR: usize  = 0x0C;
    pub const AHB2ENR: usize  = 0x4C;
    pub const APB1ENR1: usize = 0x58;
}

// CR bits
mod cr {
    pub const HSEON: u32  = 1 << 16;
    pub const HSERDY: u32 = 1 << 17;
    pub const PLLON: u32  = 1 << 24;
    pub const PLLRDY: u32 = 1 << 25;
}

// CFGR bits
mod cfgr {
    /// System clock switch: PLL1_R
    pub const SW_PLL: u32   = 0x3;
    /// System clock switch status mask
    pub const SWS_MASK: u32 = 0x3 << 2;
    /// SWS == PLL
    pub const SWS_PLL: u32  = 0x3 << 2;
    /// APB1 prescaler DIV2 (bits [10:8] = 0b100)
    pub const PPRE1_DIV2: u32 = 0x4 << 8;
}

// PLLCFGR value for HSE(8MHz)/2 × 85 / 2 = 170MHz
// PLLSRC=HSE(3), PLLM=DIV2(1), PLLN=85(0x55), PLLREN=1, PLLR=DIV2(0)
const PLLCFGR_170MHZ: u32 = 0x03          // PLLSRC = HSE
    | (0x01 << 4)  // PLLM   = DIV2
    | (0x55 << 8)  // PLLN   = MUL85
    | (0x01 << 24) // PLLREN = 1 (enable PLLR output → SYSCLK)
    | (0x00 << 25); // PLLR  = DIV2

/// FLASH_ACR base address
const FLASH_ACR: usize = 0x4002_2000;

/// PWR base address
const PWR: usize = 0x4000_7000;

/// USART2 base address
const USART2_BASE: usize = 0x4000_4400;

/// Register offsets
mod off {
    pub const CR1: usize = 0x00;
    pub const BRR: usize = 0x0C;
    pub const ISR: usize = 0x1C;
    pub const RDR: usize = 0x24;
    pub const TDR: usize = 0x28;
}

/// CR1 bits
mod cr1 {
    pub const UE: u32 = 1 << 0;
    pub const RE: u32 = 1 << 2;
    pub const TE: u32 = 1 << 3;
}

/// ISR bits
mod isr {
    pub const RXNE: u32 = 1 << 5;
    pub const TC: u32   = 1 << 6;
    pub const TXE: u32  = 1 << 7;
}

/// Initialize clocks (HSE → PLL @ 170MHz, APB1 @ 85MHz) and
/// USART2 (921600 8N1, TX=PB3, RX=PB4).
///
/// Safe to call once at bootloader startup before any peripheral use.
pub fn init() {
    unsafe {
        init_clocks();
        init_usart();
    }
}

/// Configure HSE + PLL for 170MHz system clock, APB1 at 85MHz.
///
/// Sequence per RM0440 §3.3.3 (range 1 boost mode):
///   1. Enable PWR clock, set VOS = boost (range 1 boost) and wait VOSF clear
///   2. Set Flash latency to 4 wait states and wait readback confirms
///   3. Enable HSE, wait HSERDY
///   4. Ensure PLL is off, write PLLCFGR, enable PLL, wait PLLRDY
///   5. Switch SYSCLK to PLL (with APB1 /2 = 85 MHz), wait SWS confirms
unsafe fn init_clocks() {
    unsafe {
        let rcc_cr     = (RCC + rcc_off::CR)      as *mut u32;
        let rcc_pllcfg = (RCC + rcc_off::PLLCFGR) as *mut u32;
        let rcc_cfgr   = (RCC + rcc_off::CFGR)    as *mut u32;
        let flash_acr  = FLASH_ACR as *mut u32;

        // Step 1: Enable PWR peripheral clock (APB1ENR1 bit 28) and set
        // voltage range 1 boost mode (PWR_CR1.VOS = 0b01, BOOST = bit 8).
        // Required for SYSCLK > 150 MHz on STM32G4 (RM0440 §6.1.5).
        let rcc_apb1enr1 = (RCC + rcc_off::APB1ENR1) as *mut u32;
        core::ptr::write_volatile(rcc_apb1enr1,
            core::ptr::read_volatile(rcc_apb1enr1) | (1 << 28)); // PWREN
        // Small delay for APB clock to reach PWR
        cortex_m::asm::nop(); cortex_m::asm::nop();

        // VOS[9:8] = 0b01 (range 1), BOOSTEN bit 8 (in EPWRCR or CR5 on G4)
        // On STM32G4 the boost bit is CR5.R1MODE (bit 8 of PWR_CR5 at offset 0x80).
        // CR1.VOS[9:8] must be 0b01 (range 1) — reset default is already 0b01.
        // The "boost" is enabled via PWR_CR5.R1MODE = 0 (RM0440 §6.4.5).
        let pwr_cr5 = (PWR + 0x80) as *mut u32;
        // R1MODE = 0 → boost enabled. Mask bit 8 to 0.
        core::ptr::write_volatile(pwr_cr5,
            core::ptr::read_volatile(pwr_cr5) & !(1 << 8));

        // Confirm VOS is range 1: PWR_SR2.VOSF (bit 10) must be 0 (ready)
        let pwr_sr2 = (PWR + 0x14) as *const u32;
        while core::ptr::read_volatile(pwr_sr2) & (1 << 10) != 0 {}

        // Step 2: Set Flash latency to 4 WS before increasing clock frequency.
        // FLASH_ACR[3:0] = latency, also enable prefetch + icache + dcache.
        let acr = core::ptr::read_volatile(flash_acr);
        core::ptr::write_volatile(flash_acr,
            (acr & !0xF) | 4 | (1 << 8) | (1 << 9) | (1 << 10)); // LATENCY=4, PRFTEN, ICEN, DCEN
        // Wait until latency readback confirms
        while core::ptr::read_volatile(flash_acr) & 0xF != 4 {}

        // Step 3: Enable HSE and wait for it to be ready
        core::ptr::write_volatile(rcc_cr,
            core::ptr::read_volatile(rcc_cr) | cr::HSEON);
        while core::ptr::read_volatile(rcc_cr) & cr::HSERDY == 0 {}

        // Step 4: Ensure PLL is off before writing PLLCFGR (register is
        // write-protected when PLLON = 1, RM0440 §7.4.4).
        if core::ptr::read_volatile(rcc_cr) & cr::PLLON != 0 {
            core::ptr::write_volatile(rcc_cr,
                core::ptr::read_volatile(rcc_cr) & !cr::PLLON);
            while core::ptr::read_volatile(rcc_cr) & cr::PLLRDY != 0 {}
        }
        core::ptr::write_volatile(rcc_pllcfg, PLLCFGR_170MHZ);

        // Enable PLL and wait for it to lock
        core::ptr::write_volatile(rcc_cr,
            core::ptr::read_volatile(rcc_cr) | cr::PLLON);
        while core::ptr::read_volatile(rcc_cr) & cr::PLLRDY == 0 {}

        // Step 5: Set APB1 prescaler to DIV2 (85MHz) and switch SYSCLK to PLL.
        // Keep APB2 at DIV1 (170MHz).
        let cfgr = core::ptr::read_volatile(rcc_cfgr);
        let cfgr = (cfgr & !(0x3 | (0x7 << 8)))
            | cfgr::SW_PLL
            | cfgr::PPRE1_DIV2;
        core::ptr::write_volatile(rcc_cfgr, cfgr);
        while core::ptr::read_volatile(rcc_cfgr) & cfgr::SWS_MASK != cfgr::SWS_PLL {}
    }
}

/// Configure USART2 at 921600 baud (APB1 = 85MHz).
unsafe fn init_usart() {
    unsafe {
        let rcc_ahb2enr  = (RCC + rcc_off::AHB2ENR)  as *mut u32;
        let rcc_apb1enr1 = (RCC + rcc_off::APB1ENR1) as *mut u32;

        // Enable GPIOB clock (AHB2ENR bit 1) and USART2 clock (APB1ENR1 bit 17)
        core::ptr::write_volatile(rcc_ahb2enr,
            core::ptr::read_volatile(rcc_ahb2enr) | (1 << 1));
        core::ptr::write_volatile(rcc_apb1enr1,
            core::ptr::read_volatile(rcc_apb1enr1) | (1 << 17));

        // Small delay for clock stabilization
        for _ in 0..100 { cortex_m::asm::nop(); }

        // Configure PB3 (TX) and PB4 (RX) as AF7 (USART2), high speed
        let gpiob = 0x4800_0400 as *mut u32;

        // MODER: set PB3, PB4 to Alternate Function (0b10)
        let moder = gpiob.add(0x00 / 4);
        let mut v = core::ptr::read_volatile(moder);
        v &= !(0b11 << (3 * 2)) & !(0b11 << (4 * 2));
        v |= (0b10 << (3 * 2)) | (0b10 << (4 * 2));
        core::ptr::write_volatile(moder, v);

        // OSPEEDR: set PB3, PB4 to High speed (0b11)
        let ospeedr = gpiob.add(0x08 / 4);
        let mut v = core::ptr::read_volatile(ospeedr);
        v |= (0b11 << (3 * 2)) | (0b11 << (4 * 2));
        core::ptr::write_volatile(ospeedr, v);

        // AFRL: set PB3, PB4 to AF7
        let afrl = gpiob.add(0x20 / 4);
        let mut v = core::ptr::read_volatile(afrl);
        v &= !(0xF << (3 * 4)) & !(0xF << (4 * 4));
        v |= (7 << (3 * 4)) | (7 << (4 * 4));
        core::ptr::write_volatile(afrl, v);

        // Configure USART2
        let usart2 = USART2_BASE as *mut u32;
        let cr1 = usart2.add(off::CR1 / 4);
        let brr = usart2.add(off::BRR / 4);

        // Disable before configuring
        core::ptr::write_volatile(cr1, 0);

        // BRR = APB1_CLK / baud = 85_000_000 / 921_600 ≈ 92
        // Actual rate: 85_000_000 / 92 = 923_913 baud (+0.25% error, within spec)
        core::ptr::write_volatile(brr, 92);

        // Enable USART, TX and RX
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

/// Wait until the UART shift register is fully empty (Transmission Complete).
///
/// Unlike TXE (TX data register empty), TC guarantees that the last byte's
/// bits have all been physically shifted out onto the wire. Call this before
/// any action that would cut power or reset the UART (e.g. sys_reset).
pub fn wait_tx_complete() {
    unsafe {
        let isr = (USART2_BASE + off::ISR) as *const u32;
        while core::ptr::read_volatile(isr) & isr::TC == 0 {
            cortex_m::asm::nop();
        }
    }
}
