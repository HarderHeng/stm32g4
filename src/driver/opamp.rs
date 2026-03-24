//! OPAMP driver for FOC motor control
//!
//! Configures internal operational amplifiers in PGA (Programmable Gain Amplifier) mode
//! for current sensing on each motor phase.
//!
//! # Pin Mapping (B-G431B-ESC1)
//!
//! | OPAMP   | Input Pin | ADC Channel | Phase |
//! |---------|-----------|-------------|-------|
//! | OPAMP1  | PA1       | ADC1_CH13   | U     |
//! | OPAMP2  | PA7       | ADC2_CH16   | V     |
//! | OPAMP3  | PB0       | ADC2_CH18   | W     |

use embassy_stm32::opamp::{OpAmp, OpAmpGain, OpAmpSpeed};
use embassy_stm32::peripherals::{OPAMP1, OPAMP2, OPAMP3};
use embassy_stm32::Peri;

/// OPAMP gain setting
/// The internal PGA supports gains of 2, 4, 8, or 16
/// For B-G431B-ESC1 with 3mΩ shunt and 5A max current:
/// - Max voltage across shunt: 5A * 0.003Ω = 15mV
/// - With gain 8: 15mV * 8 = 120mV swing
/// - This provides good resolution while staying within ADC range
const CURRENT_SENSE_GAIN: OpAmpGain = OpAmpGain::Mul8;

/// OPAMP driver for three-phase current sensing
///
/// Configures all three internal OPAMPs in PGA mode with internal output
/// connected to ADC channels.
pub struct CurrentSenseAmp {
    /// U phase amplifier (OPAMP1 -> ADC1_CH13)
    #[allow(dead_code)]
    opamp1: OpAmp<'static, OPAMP1>,
    /// V phase amplifier (OPAMP2 -> ADC2_CH16)
    #[allow(dead_code)]
    opamp2: OpAmp<'static, OPAMP2>,
    /// W phase amplifier (OPAMP3 -> ADC2_CH18)
    #[allow(dead_code)]
    opamp3: OpAmp<'static, OPAMP3>,
}

impl CurrentSenseAmp {
    /// Create a new OPAMP driver instance
    ///
    /// Configures all three OPAMPs in high-speed PGA mode with internal
    /// output routing to ADC channels.
    ///
    /// # Arguments
    ///
    /// * `opamp1` - OPAMP1 peripheral
    /// * `opamp2` - OPAMP2 peripheral
    /// * `opamp3` - OPAMP3 peripheral
    /// * `pa1` - U phase current sense input (PA1)
    /// * `pa7` - V phase current sense input (PA7)
    /// * `pb0` - W phase current sense input (PB0)
    ///
    /// # Returns
    ///
    /// Configured OPAMP driver with all amplifiers enabled
    pub fn new(
        opamp1: Peri<'static, OPAMP1>,
        opamp2: Peri<'static, OPAMP2>,
        opamp3: Peri<'static, OPAMP3>,
        pa1: Peri<'static, embassy_stm32::peripherals::PA1>,
        pa7: Peri<'static, embassy_stm32::peripherals::PA7>,
        pb0: Peri<'static, embassy_stm32::peripherals::PB0>,
    ) -> Self {
        // Configure OPAMP1 for U phase current sensing
        // PGA mode with internal output to ADC1_CH13
        let mut amp1 = OpAmp::new(opamp1, OpAmpSpeed::HighSpeed);
        amp1.pga_int(pa1, CURRENT_SENSE_GAIN);

        // Configure OPAMP2 for V phase current sensing
        // PGA mode with internal output to ADC2_CH16
        let mut amp2 = OpAmp::new(opamp2, OpAmpSpeed::HighSpeed);
        amp2.pga_int(pa7, CURRENT_SENSE_GAIN);

        // Configure OPAMP3 for W phase current sensing
        // PGA mode with internal output to ADC2_CH18
        let mut amp3 = OpAmp::new(opamp3, OpAmpSpeed::HighSpeed);
        amp3.pga_int(pb0, CURRENT_SENSE_GAIN);

        Self {
            opamp1: amp1,
            opamp2: amp2,
            opamp3: amp3,
        }
    }

    /// Get the configured gain value
    pub fn gain() -> f32 {
        match CURRENT_SENSE_GAIN {
            OpAmpGain::Mul2 => 2.0,
            OpAmpGain::Mul4 => 4.0,
            OpAmpGain::Mul8 => 8.0,
            OpAmpGain::Mul16 => 16.0,
            _ => 1.0, // Default non-PGA mode
        }
    }
}

/// ADC channel assignments for current sensing
/// These are the internal connections from OPAMP outputs
pub mod adc_channels {
    /// U phase current ADC channel (OPAMP1 output)
    pub const U_PHASE_ADC1_CH: u8 = 13;
    /// V phase current ADC channel (OPAMP2 output)
    pub const V_PHASE_ADC2_CH: u8 = 16;
    /// W phase current ADC channel (OPAMP3 output)
    pub const W_PHASE_ADC2_CH: u8 = 18;
}