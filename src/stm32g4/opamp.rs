//! STM32G4 OPAMP configuration
//!
//! OPAMP initialization and configuration for current sensing.
//! This module provides OPAMP setup that works with the ADC driver.

use embassy_stm32::pac;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{OPAMP1, OPAMP2, OPAMP3};

/// OPAMP gain configurations
///
/// STM32G4 OPAMP uses PGA (Programmable Gain Amplifier) mode.
/// The gain is set by the PGA_GAIN field in OPAMPx_CSR.
///
/// Note: The actual gain may differ from the register value due to
/// external feedback network on the B-G431B-ESC1 board.
#[derive(Clone, Copy, Debug)]
pub enum OpAmpGain {
    /// PGA Gain ×2
    Gain2,
    /// PGA Gain ×4
    Gain4,
    /// PGA Gain ×8
    Gain8,
    /// PGA Gain ×16
    Gain16,
    /// PGA Gain ×32
    Gain32,
    /// PGA Gain ×64
    Gain64,
}

impl OpAmpGain {
    /// Convert to CSR register value
    fn to_csr_value(&self) -> u8 {
        match self {
            OpAmpGain::Gain2 => 0b000,
            OpAmpGain::Gain4 => 0b001,
            OpAmpGain::Gain8 => 0b010,
            OpAmpGain::Gain16 => 0b011,
            OpAmpGain::Gain32 => 0b100,
            OpAmpGain::Gain64 => 0b101,
        }
    }
}

/// B-G431B-ESC1 uses PGA×16 register setting, but actual gain is ≈9.14
/// due to external feedback network. This is the gain used by the ADC driver.
pub const BOARD_OPAMP_GAIN: f32 = 9.14;

/// STM32G4 OPAMP wrapper for configuration
pub struct Stm32G4OpAmp;

impl Stm32G4OpAmp {
    /// Initialize all three OPAMPs for current sensing
    ///
    /// Configures OPAMP1, OPAMP2, OPAMP3 in PGA mode with external output.
    ///
    /// # Arguments
    ///
    /// * `gain` - PGA gain setting (register value, actual gain may differ)
    ///
    /// # Pin Configuration (B-G431B-ESC1)
    ///
    /// | OPAMP | Non-inv Input | Feedback | Output    | Phase |
    /// |-------|---------------|----------|-----------|-------|
    /// | OPAMP1| PA1 (shunt)   | PA3      | PA2→ADC1  | U/Ia  |
    /// | OPAMP2| PA7 (shunt)   | PA5      | PA6→ADC2  | V/Ib  |
    /// | OPAMP3| PB0 (shunt)   | PB2      | PB1→ADC1  | W/Ic  |
    pub fn init_all(_gain: OpAmpGain) -> Self {
        // Enable OPAMP clock
        pac::RCC.apb2enr().modify(|w| w.set_opampen(true));

        // Configure each OPAMP
        Self::configure_opamp1();
        Self::configure_opamp2();
        Self::configure_opamp3();

        // Wait for OPAMPs to stabilize (typical startup time is 10µs)
        cortex_m::asm::delay(1700); // ~10µs at 170MHz

        Self
    }

    /// Configure OPAMP1 for U-phase current sensing
    ///
    /// PGA mode, non-inverting input PA1, output PA2
    fn configure_opamp1() {
        // OPAMP1_CSR:
        // - OPAMP1EN = 1 (enable)
        // - PGA_GAIN = 011 (PGA×16)
        // - VM_SEL = 01 (PGA feedback)
        // - OPALPM = 0 (normal power mode)
        // - OPA_CALON = 0 (calibration off)
        pac::OPAMP1.csr().write(|w| {
            w.set_opampen(true);
            w.set_pga_gain(0b011); // PGA×16
            w.set_vm_sel(0b01); // PGA feedback
        });
    }

    /// Configure OPAMP2 for V-phase current sensing
    ///
    /// PGA mode, non-inverting input PA7, output PA6
    fn configure_opamp2() {
        pac::OPAMP2.csr().write(|w| {
            w.set_opampen(true);
            w.set_pga_gain(0b011); // PGA×16
            w.set_vm_sel(0b01); // PGA feedback
        });
    }

    /// Configure OPAMP3 for W-phase current sensing
    ///
    /// PGA mode, non-inverting input PB0, output PB1
    fn configure_opamp3() {
        pac::OPAMP3.csr().write(|w| {
            w.set_opampen(true);
            w.set_pga_gain(0b011); // PGA×16
            w.set_vm_sel(0b01); // PGA feedback
        });
    }

    /// Enable all OPAMPs
    pub fn enable_all() {
        pac::OPAMP1.csr().modify(|w| w.set_opampen(true));
        pac::OPAMP2.csr().modify(|w| w.set_opampen(true));
        pac::OPAMP3.csr().modify(|w| w.set_opampen(true));
    }

    /// Disable all OPAMPs
    pub fn disable_all() {
        pac::OPAMP1.csr().modify(|w| w.set_opampen(false));
        pac::OPAMP2.csr().modify(|w| w.set_opampen(false));
        pac::OPAMP3.csr().modify(|w| w.set_opampen(false));
    }

    /// Check if all OPAMPs are enabled
    pub fn are_enabled() -> bool {
        pac::OPAMP1.csr().read().opampen()
            && pac::OPAMP2.csr().read().opampen()
            && pac::OPAMP3.csr().read().opampen()
    }

    /// Get the actual board gain (for use in current calculations)
    #[inline(always)]
    pub fn actual_gain() -> f32 {
        BOARD_OPAMP_GAIN
    }
}