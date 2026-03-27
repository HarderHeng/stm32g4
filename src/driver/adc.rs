//! ADC driver for R3_2 (3-shunt dual-ADC) current sensing
//!
//! Uses ADC1 (master) + ADC2 (slave) in injected simultaneous mode,
//! hardware-triggered by TIM1 CH4 CC4 event.
//!
//! # Sector → channel mapping
//!
//! Each FOC sector selects a different pair of ADC channels to sample two
//! phases directly; the third is reconstructed by Kirchhoff's law (Ia+Ib+Ic=0).
//!
//! | Sector | ADC1 ch | ADC2 ch  | Direct phases |
//! |--------|---------|----------|---------------|
//! |   0    |   3=Ia  |  3=Ib    | A, B → Ic     |
//! |   1    |   3=Ia  | 18=Ic    | A, C → Ib     |
//! |   2    |   3=Ia  | 18=Ic    | A, C → Ib     |
//! |   3    |   3=Ia  |  3=Ib    | A, B → Ic     |
//! |   4    |   3=Ia  |  3=Ib    | A, B → Ic     |
//! |   5    |  12=Ic  |  3=Ib    | C, B → Ia     |
//!
//! ADC results are left-aligned (16-bit); zero current → ≈32767.
//!
//! # Implementation notes
//! Embassy's `InjectedAdc::setup_injected_conversions()` writes JSQR once at build
//! time and provides no way to change the channel sequence at runtime.  R3_2 requires
//! re-writing JSQR every PWM cycle (50 µs) to switch channel pairs.  We therefore
//! bypass the Embassy injected-ADC API and operate the ADC directly through the PAC
//! after Embassy's `Adc::new()` has performed the voltage regulator bring-up,
//! calibration, and clock configuration.

use embassy_stm32::adc::{Adc, AdcConfig, Dual};
use embassy_stm32::pac;
use embassy_stm32::pac::adc::vals::Exten;
use embassy_stm32::pac::adc::regs::Jsqr;
use embassy_stm32::peripherals::{ADC1, ADC2};
use embassy_stm32::Peri;

use super::traits::CurrentSampler;

// ─── Physical constants ──────────────────────────────────────────────────────

/// Shunt resistance (Ω)
const R_SHUNT: f32 = 0.003;

/// Effective OPAMP gain (actual, not register value)
const OPAMP_GAIN: f32 = 9.14;

/// ADC left-aligned full-scale (2^16 − 1 quantised to even steps)
const ADC_FULL_SCALE: f32 = 65520.0;

/// VDDA reference voltage (V)
const VDDA: f32 = 3.3;

/// Current scale factor: counts per Ampere
/// = ADC_FULL_SCALE * R_SHUNT * OPAMP_GAIN / VDDA
/// = 65520 * 0.003 * 9.14 / 3.3 ≈ 545.0
const COUNTS_PER_AMP: f32 =
    ADC_FULL_SCALE * R_SHUNT * OPAMP_GAIN / VDDA;

// ─── JEXTSEL constants ───────────────────────────────────────────────────────

/// JEXTSEL = 0b00001 → TIM1_CC4 (RM0440 Table 167)
const JEXTSEL_TIM1_CC4: u8 = 1;

/// JEXTEN = rising edge (0b01)
const JEXTEN_RISING: Exten = Exten::RISING_EDGE;

/// ADC sample time register value for injected channels (2.5 cycles)
/// ADC_SMP = 0b000 = CYCLES2_5
const SAMPLE_TIME: u8 = 0; // SampleTime::CYCLES2_5

// ─── ADC channel numbers ─────────────────────────────────────────────────────

/// OPAMP1 external output → ADC1_CH3  (U / Ia)
const CH_IA: u8 = 3;
/// OPAMP2 external output → ADC2_CH3  (V / Ib)
const CH_IB: u8 = 3;
/// OPAMP3 external output → ADC1_CH12 (W / Ic on ADC1)
const CH_IC_ADC1: u8 = 12;
/// OPAMP3 internal output → ADC2_CH18 (W / Ic on ADC2 = VOPAMP3)
const CH_IC_ADC2: u8 = 18;

// ─── Sector-to-ADC-channel table ────────────────────────────────────────────

/// Describes which ADC channels are sampled in a given sector.
#[derive(Clone, Copy)]
enum SectorSampling {
    /// ADC1 reads Ia (ch3), ADC2 reads Ib (ch3) → Ic = −Ia−Ib
    IaIb,
    /// ADC1 reads Ia (ch3), ADC2 reads Ic (ch18) → Ib = −Ia−Ic
    IaIc,
    /// ADC1 reads Ic (ch12), ADC2 reads Ib (ch3) → Ia = −Ic−Ib
    IcIb,
}

const SECTOR_SAMPLING: [SectorSampling; 6] = [
    SectorSampling::IaIb, // sector 0
    SectorSampling::IaIc, // sector 1
    SectorSampling::IaIc, // sector 2
    SectorSampling::IaIb, // sector 3
    SectorSampling::IaIb, // sector 4
    SectorSampling::IcIb, // sector 5
];

// ─── Driver struct ────────────────────────────────────────────────────────────

/// R3_2 double-ADC current-sense driver.
pub struct CurrentSenseAdc {
    /// Per-sector JSQR pre-computed values for ADC1
    jsqr1: [u32; 6],
    /// Per-sector JSQR pre-computed values for ADC2
    jsqr2: [u32; 6],
    /// Zero-current offset per phase (left-aligned ADC counts ≈ 32767)
    ia_offset: u16,
    ib_offset: u16,
    ic_offset: u16,
    /// Last injected sample, updated in `update_sector` / read by `read_phase_currents`.
    raw_adc1: u16,
    raw_adc2: u16,
    /// Active sector at the time of the last sample
    sector: u8,
}

impl CurrentSenseAdc {
    /// Initialise and calibrate the dual-ADC current-sense driver.
    ///
    /// Performs Embassy ADC bring-up (voltage regulator, calibration, clocks)
    /// via `Adc::new`, then drops the Embassy handles and takes direct PAC control
    /// so that we can update JSQR every PWM cycle.
    pub fn new(
        adc1: Peri<'static, ADC1>,
        adc2: Peri<'static, ADC2>,
    ) -> Self {
        // Use Embassy to bring up ADC1 (clocks, voltage regulator, calibration).
        let cfg1 = AdcConfig {
            dual_mode: Some(Dual::DUAL_J),
            ..Default::default()
        };
        let _adc1 = Adc::new(adc1, cfg1);

        // Bring up ADC2 similarly (slave in dual mode).
        let _adc2 = Adc::new(adc2, AdcConfig::default());

        // From here we own the ADC peripherals via the PAC.
        // Embassy Adc<T> does NOT disable the ADC or the clock when dropped;
        // it only resets the conversion state, so we are safe to continue.

        // Configure sample times on the channels we will use.
        // SMPR1: ch1–ch9, SMPR2: ch10–ch18
        // ADC1: ch3 (bits[11:9] of SMPR1), ch12 (bits[8:6] of SMPR2 for ch10+2)
        pac::ADC1.smpr().modify(|w| w.set_smp(3, SAMPLE_TIME.into()));
        pac::ADC1.smpr2().modify(|w| w.set_smp(2, SAMPLE_TIME.into())); // ch12 = ch10+2

        // ADC2: ch3 (bits[11:9] of SMPR1), ch18 (bits[26:24] of SMPR2 for ch10+8)
        pac::ADC2.smpr().modify(|w| w.set_smp(3, SAMPLE_TIME.into()));
        pac::ADC2.smpr2().modify(|w| w.set_smp(8, SAMPLE_TIME.into())); // ch18 = ch10+8

        // Pre-compute JSQR lookup tables for each of the 6 sectors.
        let jsqr1 = Self::build_jsqr_table_adc1();
        let jsqr2 = Self::build_jsqr_table_adc2();

        // Write initial JSQR (sector 0) and arm the hardware trigger.
        pac::ADC1.jsqr().write_value(Jsqr(jsqr1[0]));
        pac::ADC2.jsqr().write_value(Jsqr(jsqr2[0]));

        // Enable injected conversions on both ADCs (they will wait for TIM1_CC4).
        pac::ADC1.cr().modify(|w| w.set_jadstart(true));
        pac::ADC2.cr().modify(|w| w.set_jadstart(true));

        Self {
            jsqr1,
            jsqr2,
            ia_offset: 32767,
            ib_offset: 32767,
            ic_offset: 32767,
            raw_adc1: 32767,
            raw_adc2: 32767,
            sector: 0,
        }
    }

    // ── JSQR construction ───────────────────────────────────────────────────

    /// Build a single JSQR word for JL=1 (1 conversion), the given channel
    /// in rank-1, and TIM1_CC4 rising-edge trigger.
    fn make_jsqr(ch: u8) -> u32 {
        let mut jsqr = Jsqr(0);
        jsqr.set_jl(0);                      // JL=0 → 1 conversion
        jsqr.set_jextsel(JEXTSEL_TIM1_CC4);
        jsqr.set_jexten(JEXTEN_RISING);
        jsqr.set_jsq(0, ch);
        jsqr.0
    }

    fn build_jsqr_table_adc1() -> [u32; 6] {
        [0, 1, 2, 3, 4, 5].map(|s| match SECTOR_SAMPLING[s] {
            SectorSampling::IaIb => Self::make_jsqr(CH_IA),
            SectorSampling::IaIc => Self::make_jsqr(CH_IA),
            SectorSampling::IcIb => Self::make_jsqr(CH_IC_ADC1),
        })
    }

    fn build_jsqr_table_adc2() -> [u32; 6] {
        [0, 1, 2, 3, 4, 5].map(|s| match SECTOR_SAMPLING[s] {
            SectorSampling::IaIb => Self::make_jsqr(CH_IB),
            SectorSampling::IaIc => Self::make_jsqr(CH_IC_ADC2),
            SectorSampling::IcIb => Self::make_jsqr(CH_IB),
        })
    }

    // ── Raw ADC reading ─────────────────────────────────────────────────────

    /// Read the injected data registers after a conversion completes.
    /// Should be called from the ADC or PWM interrupt handler after JEOC/JEOS.
    ///
    /// # Safety
    /// This function assumes it is called after JEOS (End of Sequence) is set.
    /// In hardware-triggered mode, this is guaranteed by the TIM1_CC4 trigger
    /// and RCR configuration. If called prematurely, may read stale data.
    #[inline(always)]
    pub fn latch_injected(&mut self) {
        // Read JDR before clearing flags — hardware guarantees data validity
        // after JEOS is set. In dual injected mode, both ADCs convert
        // simultaneously, so we read both even if only one triggered.
        self.raw_adc1 = pac::ADC1.jdr(0).read().jdata();
        self.raw_adc2 = pac::ADC2.jdr(0).read().jdata();
        // Clear JEOS + JEOC flags (W1C — must use write(), not modify())
        // RM0440 §24.9.2: ISR flags are cleared by writing 1
        pac::ADC1.isr().write(|w| {
            w.set_jeos(true);
            w.set_jeoc(true);  // Also clear EOC to prevent overrun
        });
        pac::ADC2.isr().write(|w| {
            w.set_jeos(true);
            w.set_jeoc(true);
        });
    }

    // ── Current conversion ──────────────────────────────────────────────────

    #[inline(always)]
    fn counts_to_amps(raw: u16, offset: u16) -> f32 {
        // Left-aligned: positive current → raw < offset
        (offset as f32 - raw as f32) / COUNTS_PER_AMP
    }
}

impl CurrentSampler for CurrentSenseAdc {
    #[inline(always)]
    fn update_sector(&mut self, sector: u8) {
        // 1. Latch results from the previous cycle's conversion.
        //    They were sampled with self.sector's channel pair — read *before*
        //    changing self.sector so that read_phase_currents decodes correctly.
        self.latch_injected();
        // 2. Program next sector's channels into JSQR (takes effect on next CC4 trigger).
        pac::ADC1.jsqr().write_value(Jsqr(self.jsqr1[sector as usize]));
        pac::ADC2.jsqr().write_value(Jsqr(self.jsqr2[sector as usize]));
        // 3. Advance sector index to match the data we just latched.
        self.sector = sector;
    }

    fn read_phase_currents(&mut self) -> (f32, f32, f32) {
        let (ia, ib, ic) = match SECTOR_SAMPLING[self.sector as usize] {
            SectorSampling::IaIb => {
                let ia = Self::counts_to_amps(self.raw_adc1, self.ia_offset);
                let ib = Self::counts_to_amps(self.raw_adc2, self.ib_offset);
                let ic = -ia - ib;
                (ia, ib, ic)
            }
            SectorSampling::IaIc => {
                let ia = Self::counts_to_amps(self.raw_adc1, self.ia_offset);
                let ic = Self::counts_to_amps(self.raw_adc2, self.ic_offset);
                let ib = -ia - ic;
                (ia, ib, ic)
            }
            SectorSampling::IcIb => {
                let ic = Self::counts_to_amps(self.raw_adc1, self.ic_offset);
                let ib = Self::counts_to_amps(self.raw_adc2, self.ib_offset);
                let ia = -ic - ib;
                (ia, ib, ic)
            }
        };
        (ia, ib, ic)
    }

    fn calibrate(&mut self) {
        const N: u32 = 128;
        /// Timeout counter limit to prevent deadlock if ADC clock fails
        const TIMEOUT_LIMIT: u32 = 1_000_000;

        // Switch to software trigger by writing JSQR with JEXTEN=disabled.
        let make_sw = |ch: u8| -> u32 {
            let mut jsqr = Jsqr(0);
            jsqr.set_jl(0);
            jsqr.set_jextsel(0);
            jsqr.set_jexten(Exten::DISABLED);
            jsqr.set_jsq(0, ch);
            jsqr.0
        };

        let mut sum_a: u32 = 0;
        let mut sum_b: u32 = 0;
        let mut sum_c: u32 = 0;

        for _ in 0..N {
            // Sample Ia: ADC1 ch3
            pac::ADC1.jsqr().write_value(Jsqr(make_sw(CH_IA)));
            pac::ADC1.cr().modify(|w| w.set_jadstart(true));
            let mut timeout = 0;
            while !pac::ADC1.isr().read().jeos() {
                timeout += 1;
                if timeout > TIMEOUT_LIMIT {
                    cortex_m::asm::bkpt();
                    break;
                }
            }
            pac::ADC1.isr().write(|w| w.set_jeos(true));
            sum_a += pac::ADC1.jdr(0).read().jdata() as u32;

            // Sample Ib: ADC2 ch3
            pac::ADC2.jsqr().write_value(Jsqr(make_sw(CH_IB)));
            pac::ADC2.cr().modify(|w| w.set_jadstart(true));
            let mut timeout = 0;
            while !pac::ADC2.isr().read().jeos() {
                timeout += 1;
                if timeout > TIMEOUT_LIMIT {
                    cortex_m::asm::bkpt();
                    break;
                }
            }
            pac::ADC2.isr().write(|w| w.set_jeos(true));
            sum_b += pac::ADC2.jdr(0).read().jdata() as u32;

            // Sample Ic: ADC1 ch12
            pac::ADC1.jsqr().write_value(Jsqr(make_sw(CH_IC_ADC1)));
            pac::ADC1.cr().modify(|w| w.set_jadstart(true));
            let mut timeout = 0;
            while !pac::ADC1.isr().read().jeos() {
                timeout += 1;
                if timeout > TIMEOUT_LIMIT {
                    cortex_m::asm::bkpt();
                    break;
                }
            }
            pac::ADC1.isr().write(|w| w.set_jeos(true));
            sum_c += pac::ADC1.jdr(0).read().jdata() as u32;
        }

        self.ia_offset = (sum_a / N) as u16;
        self.ib_offset = (sum_b / N) as u16;
        self.ic_offset = (sum_c / N) as u16;

        // Restore hardware-trigger JSQR for sector 0 and re-arm.
        pac::ADC1.jsqr().write_value(Jsqr(self.jsqr1[0]));
        pac::ADC2.jsqr().write_value(Jsqr(self.jsqr2[0]));
        pac::ADC1.cr().modify(|w| w.set_jadstart(true));
        pac::ADC2.cr().modify(|w| w.set_jadstart(true));
        self.sector = 0;
    }
}

impl CurrentSenseAdc {
    /// Get Ia offset (for debugging/logging)
    pub fn ia_offset(&self) -> u16 {
        self.ia_offset
    }

    /// Get Ib offset (for debugging/logging)
    pub fn ib_offset(&self) -> u16 {
        self.ib_offset
    }

    /// Get Ic offset (for debugging/logging)
    pub fn ic_offset(&self) -> u16 {
        self.ic_offset
    }
}
