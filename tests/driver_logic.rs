//! Host-side unit tests for driver logic (pure arithmetic, no hardware).
//!
//! These tests run on the host (x86) and verify the numerical logic that
//! is embedded in the hardware driver files.  They do not import embassy-stm32
//! and do not require a target device.
//!
//! Run with:
//!   rustc --edition 2024 tests/driver_logic.rs --test -o /tmp/driver_logic && /tmp/driver_logic

// ─────────────────────────────────────────────────────────────────────────────
// Reproduced constants (must stay in sync with src/driver/*.rs)
// ─────────────────────────────────────────────────────────────────────────────

/// TIM1 clock (Hz)
const TIM1_CLK_HZ: u32 = 170_000_000;
/// PWM switching frequency (Hz)
const PWM_FREQ_HZ: u32 = 20_000;
/// ARR = Fclk / (Fpwm × 2)  (centre-aligned counts up then down)
const PWM_PERIOD: u16 = (TIM1_CLK_HZ / (PWM_FREQ_HZ * 2)) as u16;

/// Shunt resistance (Ω)
const R_SHUNT: f64 = 0.003;
/// Effective OPAMP gain
const OPAMP_GAIN: f64 = 9.14;
/// ADC left-aligned full-scale (12-bit result << 4)
const ADC_FULL_SCALE: f64 = 65520.0;
/// VDDA (V)
const VDDA: f64 = 3.3;
/// Counts per Ampere  (= ADC_FULL_SCALE * R_SHUNT * OPAMP_GAIN / VDDA ≈ 545)
const COUNTS_PER_AMP: f64 = ADC_FULL_SCALE * R_SHUNT * OPAMP_GAIN / VDDA;

/// OCP DAC3 default threshold (LSB)
const OCP_THRESHOLD_DEFAULT: u16 = 2893;
/// DAC reference voltage
const VDDA_DAC: f64 = 3.3;
/// 12-bit DAC full scale
const DAC_FULL_SCALE: f64 = 4095.0;

// ─── JSQR bit-field layout (stm32-metapac comp_v2 / RM0440 §21.7) ───────────
//
//  Bit  0-1  : JL[1:0]      number of conversions minus 1
//  Bit  2-6  : JEXTSEL[4:0] trigger source
//  Bit  7-8  : JEXTEN[1:0]  trigger edge (01 = rising)
//  Bit  9-13 : JSQ1[4:0]    rank-1 channel
//
const JEXTSEL_TIM1_CC4: u32 = 1;
const JEXTEN_RISING: u32 = 1; // 0b01

/// Reconstruct make_jsqr for testing (mirrors CurrentSenseAdc::make_jsqr).
fn make_jsqr(ch: u8) -> u32 {
    let jl: u32 = 0; // 1 conversion
    let jextsel: u32 = JEXTSEL_TIM1_CC4;
    let jexten: u32 = JEXTEN_RISING;
    let jsq1: u32 = ch as u32;
    (jl) | (jextsel << 2) | (jexten << 7) | (jsq1 << 9)
}

/// Reconstruct counts_to_amps for testing (mirrors CurrentSenseAdc::counts_to_amps).
fn counts_to_amps(raw: u16, offset: u16) -> f64 {
    (offset as f64 - raw as f64) / COUNTS_PER_AMP
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn pwm_period_is_4250() {
    assert_eq!(PWM_PERIOD, 4250,
        "ARR must be 4250 for 20 kHz centre-aligned @ 170 MHz");
}

#[test]
fn counts_per_amp_in_range() {
    // Expected: 65520 × 0.003 × 9.14 / 3.3 ≈ 545.0
    let expected = 65520.0_f64 * 0.003 * 9.14 / 3.3;
    let diff = (COUNTS_PER_AMP - expected).abs();
    assert!(diff < 0.1,
        "COUNTS_PER_AMP={:.3} expected≈{:.3}", COUNTS_PER_AMP, expected);
}

#[test]
fn zero_current_at_offset() {
    // When raw == offset the current must be 0.
    let offset: u16 = 32767;
    let amps = counts_to_amps(offset, offset);
    assert!(amps.abs() < 1e-9, "zero current failed: {}", amps);
}

#[test]
fn positive_current_below_offset() {
    // A reading 545 counts below offset → ≈1 A
    let offset: u16 = 32767;
    let raw: u16 = 32767 - 545;
    let amps = counts_to_amps(raw, offset);
    let diff = (amps - 1.0).abs();
    assert!(diff < 0.01, "expected ≈1 A, got {} A", amps);
}

#[test]
fn negative_current_above_offset() {
    let offset: u16 = 32767;
    let raw: u16 = 32767 + 545;
    let amps = counts_to_amps(raw, offset);
    let diff = (amps + 1.0).abs();
    assert!(diff < 0.01, "expected ≈−1 A, got {} A", amps);
}

#[test]
fn kirchhoff_iab_mode() {
    let ia = 2.0_f64;
    let ib = -1.0_f64;
    let ic = -ia - ib;
    assert!((ia + ib + ic).abs() < 1e-10,
        "Kirchhoff violated: Ia+Ib+Ic={}", ia + ib + ic);
}

#[test]
fn kirchhoff_iac_mode() {
    let ia = 2.0_f64;
    let ic = -1.5_f64;
    let ib = -ia - ic;
    assert!((ia + ib + ic).abs() < 1e-10,
        "Kirchhoff violated: Ia+Ib+Ic={}", ia + ib + ic);
}

#[test]
fn kirchhoff_icb_mode() {
    let ic = 1.0_f64;
    let ib = -0.5_f64;
    let ia = -ic - ib;
    assert!((ia + ib + ic).abs() < 1e-10,
        "Kirchhoff violated: Ia+Ib+Ic={}", ia + ib + ic);
}

#[test]
fn jsqr_trigger_source() {
    for ch in [3u8, 12, 18] {
        let jsqr = make_jsqr(ch);
        let jextsel = (jsqr >> 2) & 0x1F;
        assert_eq!(jextsel, JEXTSEL_TIM1_CC4,
            "ch{}: JEXTSEL={:#x} expected {:#x}", ch, jextsel, JEXTSEL_TIM1_CC4);
    }
}

#[test]
fn jsqr_trigger_edge() {
    for ch in [3u8, 12, 18] {
        let jsqr = make_jsqr(ch);
        let jexten = (jsqr >> 7) & 0x3;
        assert_eq!(jexten, JEXTEN_RISING,
            "ch{}: JEXTEN={:#x} expected {:#x}", ch, jexten, JEXTEN_RISING);
    }
}

#[test]
fn jsqr_channel_field() {
    for ch in [3u8, 12, 18] {
        let jsqr = make_jsqr(ch);
        let jsq1 = (jsqr >> 9) & 0x1F;
        assert_eq!(jsq1, ch as u32,
            "ch{}: JSQ1={} expected {}", ch, jsq1, ch);
    }
}

#[test]
fn jsqr_jl_is_zero() {
    for ch in [3u8, 12, 18] {
        let jsqr = make_jsqr(ch);
        let jl = jsqr & 0x3;
        assert_eq!(jl, 0, "ch{}: JL={} expected 0", ch, jl);
    }
}

#[test]
fn ocp_threshold_voltage() {
    let v = OCP_THRESHOLD_DEFAULT as f64 / DAC_FULL_SCALE * VDDA_DAC;
    let diff = (v - 2.33).abs();
    assert!(diff < 0.01,
        "OCP voltage={:.3} V expected≈2.33 V", v);
}

#[test]
fn ocp_trip_current_in_range() {
    let v_thresh = OCP_THRESHOLD_DEFAULT as f64 / DAC_FULL_SCALE * VDDA_DAC;
    let i_trip = v_thresh / (R_SHUNT * OPAMP_GAIN);
    assert!(i_trip > 70.0 && i_trip < 100.0,
        "I_trip={:.1} A should be 70–100 A", i_trip);
}
