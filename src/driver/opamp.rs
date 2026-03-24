//! OPAMP driver for FOC motor current sensing
//!
//! Configures OPAMP1/2/3 in PGA mode (gain ×16, actual ≈9.14 due to resistor ratio)
//! with **external** GPIO outputs so ADC can sample via regular pins.
//!
//! # Routing
//!
//! | OPAMP  | +IN  | −IN(PGA) | External output | ADC channel         |
//! |--------|------|----------|-----------------|---------------------|
//! | OPAMP1 | PA1  | internal | PA2             | ADC1_CH3 (U phase)  |
//! | OPAMP2 | PA7  | internal | PA6             | ADC2_CH3 (V phase)  |
//! | OPAMP3 | PB0  | internal | PB1 + internal  | ADC1_CH12 / ADC2_CH18 (W phase) |
//!
//! OPAMP3 enables both external (PB1 → ADC1_CH12) and internal (OPAINTOEN=1 → ADC2_CH18)
//! outputs simultaneously.
//!
//! # PAC-level configuration
//!
//! Embassy's `pga_ext` returns an `OpAmpOutput<'_, T>` that borrows the `OpAmp<'_, T>`.
//! Keeping both alive in a struct would require a self-referential type.  Instead, we
//! configure the OPAMPs directly through the PAC (the same registers Embassy would write)
//! and set the GPIO pins to analog mode ourselves.
//!
//! CSR bit-field encoding (STM32G4 opamp_v5, RM0440 §25):
//!  VM_SEL[1:0] = 0b10 → PGA mode
//!  VP_SEL[1:0] = channel of the +IN pin (0 = IO1 for each OPAMP)
//!  PGA_GAIN[4:0] = 0b00011 (GAIN16)
//!  OPAINTOEN = 0 (OPAMP1/2 external only) / 1 (OPAMP3 both)
//!  OPAMPEN = 1

use embassy_stm32::gpio::Flex;
use embassy_stm32::pac;
use embassy_stm32::pac::opamp::vals::{PgaGain, VmSel, VpSel};
use embassy_stm32::Peri;

// ── PAC constants ─────────────────────────────────────────────────────────────

/// VM_SEL = 0b10 → PGA (inverting input connected to gain resistor array)
const VM_SEL_PGA: u8 = 0b10;

/// VP_SEL = 0 → IO1 for all three OPAMPs (PA1, PA7, PB0 respectively)
const VP_SEL_IO1: u8 = 0;

/// PGA_GAIN encoding for ×16 in opamp_v5 (RM0440 Table 206):
/// 0b00011 = GAIN16
const PGA_GAIN_X16: u8 = 0b00011;

/// OPAMP driver token.  Dropped at shutdown to disable all amplifiers.
pub struct CurrentSenseAmp {
    _private: (),
}

impl CurrentSenseAmp {
    /// Initialise OPAMP1/2/3 in PGA ×16 mode with external output pins.
    ///
    /// # Arguments
    /// * `opamp1`/`opamp2`/`opamp3` — peripheral tokens (consumed to model ownership)
    /// * `pa1`/`pa7`/`pb0` — non-inverting (+IN) input pins
    /// * `pa2`/`pa6`/`pb1` — external output pins (set to analog mode)
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        _opamp1: Peri<'static, embassy_stm32::peripherals::OPAMP1>,
        _opamp2: Peri<'static, embassy_stm32::peripherals::OPAMP2>,
        _opamp3: Peri<'static, embassy_stm32::peripherals::OPAMP3>,
        pa1:  Peri<'static, embassy_stm32::peripherals::PA1>,
        pa7:  Peri<'static, embassy_stm32::peripherals::PA7>,
        pb0:  Peri<'static, embassy_stm32::peripherals::PB0>,
        pa2:  Peri<'static, embassy_stm32::peripherals::PA2>,
        pa6:  Peri<'static, embassy_stm32::peripherals::PA6>,
        pb1:  Peri<'static, embassy_stm32::peripherals::PB1>,
    ) -> Self {
        // Set all OPAMP I/O pins to analog mode (Hi-Z, no digital path).
        // Flex::new() takes ownership and set_as_analog() configures the MODER register.
        // The Flex tokens are then dropped (the mode register setting persists).
        Flex::new(pa1).set_as_analog();
        Flex::new(pa7).set_as_analog();
        Flex::new(pb0).set_as_analog();
        Flex::new(pa2).set_as_analog();
        Flex::new(pa6).set_as_analog();
        Flex::new(pb1).set_as_analog();

        // ── OPAMP1: PA1 (+IN IO1) → PA2 (external out) ───────────────────
        pac::OPAMP1.csr().write(|w| {
            w.set_vp_sel(VpSel::from_bits(VP_SEL_IO1));
            w.set_vm_sel(VmSel::from_bits(VM_SEL_PGA));
            w.set_pga_gain(PgaGain::from_bits(PGA_GAIN_X16));
            w.set_opaintoen(false);
            w.set_opampen(true);
        });

        // ── OPAMP2: PA7 (+IN IO1) → PA6 (external out) ───────────────────
        pac::OPAMP2.csr().write(|w| {
            w.set_vp_sel(VpSel::from_bits(VP_SEL_IO1));
            w.set_vm_sel(VmSel::from_bits(VM_SEL_PGA));
            w.set_pga_gain(PgaGain::from_bits(PGA_GAIN_X16));
            w.set_opaintoen(false);
            w.set_opampen(true);
        });

        // ── OPAMP3: PB0 (+IN IO1) → PB1 (external) + ADC2_CH18 (internal)
        pac::OPAMP3.csr().write(|w| {
            w.set_vp_sel(VpSel::from_bits(VP_SEL_IO1));
            w.set_vm_sel(VmSel::from_bits(VM_SEL_PGA));
            w.set_pga_gain(PgaGain::from_bits(PGA_GAIN_X16));
            w.set_opaintoen(true);  // enable internal output → ADC2_CH18
            w.set_opampen(true);
        });

        Self { _private: () }
    }

    /// Nominal register gain (×16). Actual current-sense gain ≈9.14.
    pub fn nominal_gain() -> u8 {
        16
    }
}

impl Drop for CurrentSenseAmp {
    fn drop(&mut self) {
        pac::OPAMP1.csr().modify(|w| w.set_opampen(false));
        pac::OPAMP2.csr().modify(|w| w.set_opampen(false));
        pac::OPAMP3.csr().modify(|w| w.set_opampen(false));
    }
}
