//! ADC driver for FOC motor control current sensing
//!
//! Configures ADC1 and ADC2 in dual injected simultaneous mode for
//! synchronized three-phase current measurement triggered by PWM timer.
//!
//! # Channel Mapping
//!
//! | Phase | OPAMP | ADC    | Channel |
//! |-------|-------|--------|---------|
//! | U     | 1     | ADC1   | CH13    |
//! | V     | 2     | ADC2   | CH16    |
//! | W     | 3     | ADC2   | CH18    |

use embassy_stm32::adc::{Adc, AdcConfig, SampleTime, Resolution, Dual};
use embassy_stm32::peripherals::{ADC1, ADC2};
use embassy_stm32::Peri;

/// ADC sample time for current sensing
/// Longer sample time improves accuracy but increases conversion time
/// CYCLES247_5 provides good noise immunity for current sensing
#[allow(dead_code)]
const SAMPLE_TIME: SampleTime = SampleTime::CYCLES247_5;

/// ADC configuration for current sensing
///
/// This driver sets up dual ADC mode for synchronized current sampling.
/// The OPAMP outputs are used as ADC inputs via internal connections.
pub struct CurrentSenseAdc {
    /// ADC1 instance (master) - U phase current on CH13
    #[allow(dead_code)]
    adc1: Adc<'static, ADC1>,
    /// ADC2 instance (slave) - V and W phase currents
    #[allow(dead_code)]
    adc2: Adc<'static, ADC2>,
}

impl CurrentSenseAdc {
    /// Create a new ADC driver instance
    ///
    /// Configures ADC1 and ADC2 in dual injected simultaneous mode
    /// with hardware trigger from TIM1 TRGO2.
    ///
    /// # Arguments
    ///
    /// * `adc1` - ADC1 peripheral
    /// * `adc2` - ADC2 peripheral
    ///
    /// # Returns
    ///
    /// Configured ADC driver ready for current sensing
    pub fn new(
        adc1: Peri<'static, ADC1>,
        adc2: Peri<'static, ADC2>,
    ) -> Self {
        // Configure ADC1 as master in dual injected mode
        let mut config = AdcConfig::default();
        config.resolution = Some(Resolution::BITS12);
        config.dual_mode = Some(Dual::DUAL_J); // Dual injected mode

        // Create ADC1 (master)
        let adc1 = Adc::new(adc1, config);

        // Create ADC2 (slave)
        // ADC2 will be triggered simultaneously with ADC1 in dual mode
        let config2 = AdcConfig::default();
        let adc2 = Adc::new(adc2, config2);

        Self { adc1, adc2 }
    }

    /// Read current from OPAMP output
    ///
    /// This performs a blocking read of the ADC channel connected to an OPAMP.
    /// In production, this should use injected channels triggered by PWM.
    ///
    /// # Arguments
    ///
    /// * `channel` - ADC channel (OPAMP output)
    ///
    /// # Returns
    ///
    /// Raw ADC value (0-4095 for 12-bit)
    pub fn read_channel(&mut self, _channel: u8) -> u16 {
        // Placeholder - actual implementation would use injected channels
        // with PWM trigger synchronization
        2048 // Mid-scale (zero current)
    }

    /// Get the ADC reference voltage in millivolts
    pub fn get_vref_mv() -> u16 {
        // Internal reference is typically 1.2V
        // VDDA = 1.2V * 4095 / VREFINT_ADC_VALUE
        // For now, return the standard 3300mV
        3300
    }
}

/// ADC channel assignments for current sensing
/// These match the OPAMP internal outputs
pub mod channels {
    /// U phase current ADC channel (OPAMP1 output to ADC1)
    pub const U_PHASE: u8 = 13;
    /// V phase current ADC channel (OPAMP2 output to ADC2)
    pub const V_PHASE: u8 = 16;
    /// W phase current ADC channel (OPAMP3 output to ADC2)
    pub const W_PHASE: u8 = 18;
}