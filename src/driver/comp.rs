//! Hardware over-current protection driver
//!
//! Configures COMP1, COMP2, and COMP4 with DAC3 thresholds to trip the
//! TIM1 break (BDTR.BKE) when any phase current exceeds the OCP threshold.
//!
//! # Configuration (B-G431B-ESC1)
//!
//! | Comparator | +IN (shunt)  | −IN (DAC)    | Trip signal    |
//! |------------|--------------|--------------|----------------|
//! | COMP1      | PA1 (U phase)| DAC3_CH1     | TIM1 BRK       |
//! | COMP2      | PA7 (V phase)| DAC3_CH2     | TIM1 BRK       |
//! | COMP4      | PB0 (W phase)| DAC3_CH2     | TIM1 BRK       |
//!
//! The TIM1 break connection is established by `MotorPwm` via
//! `set_break_comparator_enable(n, true)`.  This module only configures
//! the comparator inputs and DAC threshold — it does **not** touch BDTR.
//!
//! # DAC threshold
//!
//! DAC3 is an internal DAC (no external pin).  Both channels are set to
//! the same threshold value.
//!
//! Reference threshold (from reference design): 2893 LSB on a 12-bit DAC
//! → V_threshold = 2893 / 4095 × 3.3 V ≈ 2.33 V
//!
//! The OCP assert level is calculated as:
//!   I_trip = V_threshold / (R_shunt × G_opamp) = 2.33 / (0.003 × 9.14) ≈ 85 A (peak)
//!
//! This is set well above the rated current so only a genuine fault trips it.

use embassy_stm32::pac;
use embassy_stm32::pac::comp::vals::Inm;

/// Reference OCP threshold (12-bit DAC counts, 2893 ≈ 2.33 V @ 3.3 V VDDA)
pub const OCP_THRESHOLD_DEFAULT: u16 = 2893;

/// INMSEL → DAC3_CH1 for COMP1 (RM0440 Table 197 — INMSEL[3:0] = 0100 = `Inm::DACA`)
const INMSEL_DAC3_CH1: Inm = Inm::DACA;

/// INMSEL → DAC3_CH2 for COMP2/COMP4 (RM0440 Table 197 — INMSEL[3:0] = 0101 = `Inm::DACB`)
const INMSEL_DAC3_CH2: Inm = Inm::DACB;

/// INPSEL = 0 → IO1 pin (PA1 for COMP1, PA7 for COMP2, PB0 for COMP4)
const INPSEL_IO1: bool = false;

/// Over-current protection driver.
///
/// Holds the configuration in registers; there is no additional heap state.
/// Drop this struct to disable the comparators (not normally done at runtime).
pub struct OverCurrentProtection {
    _private: (),
}

impl OverCurrentProtection {
    /// Configure COMP1/COMP2/COMP4 and DAC3 for hardware OCP.
    ///
    /// # Arguments
    ///
    /// * `threshold` — 12-bit DAC value (0–4095). Use [`OCP_THRESHOLD_DEFAULT`]
    ///   for the reference design default of 2893 LSB.
    pub fn new(threshold: u16) -> Self {
        let threshold = threshold.min(4095);

        // ── DAC3 clock ────────────────────────────────────────────────────
        // DAC3 is on AHB2; its clock must be enabled before accessing its
        // registers (default after reset is disabled).
        pac::RCC.ahb2enr().modify(|w| w.set_dac3en(true));

        // ── DAC3 ──────────────────────────────────────────────────────────
        // Enable DAC3 channels (internal, no GPIO required).
        // TEN=0 (no trigger — direct write from DHR takes effect immediately).
        pac::DAC3.cr().modify(|w| {
            w.set_en(0, true); // CH1 → COMP1 −IN
            w.set_en(1, true); // CH2 → COMP2/COMP4 −IN
            w.set_ten(0, false);
            w.set_ten(1, false);
        });

        // Write threshold to both channels.
        pac::DAC3.dhr12r(0).write(|w| w.set_dhr(threshold));
        pac::DAC3.dhr12r(1).write(|w| w.set_dhr(threshold));

        // ── COMP1 (PA1 vs DAC3_CH1) ──────────────────────────────────────
        pac::COMP1.csr().modify(|w| {
            w.set_inpsel(INPSEL_IO1);       // +IN = IO1 (PA1)
            w.set_inmsel(INMSEL_DAC3_CH1);  // −IN = DAC3_CH1
            w.set_en(true);
        });

        // ── COMP2 (PA7 vs DAC3_CH2) ──────────────────────────────────────
        pac::COMP2.csr().modify(|w| {
            w.set_inpsel(INPSEL_IO1);       // +IN = IO1 (PA7)
            w.set_inmsel(INMSEL_DAC3_CH2);  // −IN = DAC3_CH2
            w.set_en(true);
        });

        // ── COMP4 (PB0 vs DAC3_CH2) ──────────────────────────────────────
        pac::COMP4.csr().modify(|w| {
            w.set_inpsel(INPSEL_IO1);       // +IN = IO1 (PB0)
            w.set_inmsel(INMSEL_DAC3_CH2);  // −IN = DAC3_CH2
            w.set_en(true);
        });

        Self { _private: () }
    }
}
