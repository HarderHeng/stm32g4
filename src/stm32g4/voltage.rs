//! STM32G4 voltage and temperature monitoring implementation
//!
//! Concrete implementation of the VoltageMonitor trait for STM32G4.

use embassy_stm32::adc::{Adc, AdcConfig, Instance};
use embassy_stm32::peripherals::ADC1;
use embassy_stm32::Peri;

use crate::hal::voltage::VoltageMonitor;

/// ADC sample time for voltage/temperature readings (112.5 cycles)
/// Provides better noise immunity than minimum sampling
const SAMPLE_TIME: u8 = 5; // CYCLES112_5

/// VDDA reference voltage (V)
const VDDA: f32 = 3.3;

/// ADC full-scale value (12-bit right-aligned)
const ADC_FULL_SCALE: f32 = 4095.0;

/// Bus voltage divider ratio (R2 / (R1 + R2))
/// B-G431B-ESC1: R1 = 20k, R2 = 2k → ratio = 0.0909
/// Actual measurement may vary - calibrate for production
const BUS_VOLTAGE_DIVIDER: f32 = 0.09626;

/// NTC thermistor beta parameter (K)
/// Typical value for 10k NTC: 3380K
const NTC_BETA: f32 = 3380.0;

/// NTC nominal resistance at 25°C (Ω)
const NTC_R_NOMINAL: f32 = 10000.0;

/// NTC series resistor value (Ω)
/// B-G431B-ESC1 uses 10k series resistor
const NTC_SERIES_R: f32 = 10000.0;

/// NTC temperature conversion constants
const KELVIN_TO_CELSIUS: f32 = 273.15;
const NTC_TEMP_NOMINAL: f32 = 25.0; // 25°C reference

/// STM32G4 bus voltage and temperature monitor
pub struct Stm32G4Voltage {
    adc: Adc<'static, ADC1>,
}

impl Stm32G4Voltage {
    /// Initialize the voltage monitor
    ///
    /// # Arguments
    ///
    /// * `adc1` - ADC1 peripheral for voltage/temperature readings
    pub fn new(adc1: Peri<'static, embassy_stm32::peripherals::ADC1>) -> Self {
        let cfg = AdcConfig::default();
        let adc = Adc::new(adc1, cfg);

        Self { adc }
    }

    /// Convert ADC counts to voltage
    #[inline(always)]
    fn counts_to_voltage(counts: u16) -> f32 {
        (counts as f32 / ADC_FULL_SCALE) * VDDA
    }

    /// Convert ADC counts to bus voltage
    #[inline(always)]
    fn counts_to_bus_voltage(counts: u16) -> f32 {
        let adc_voltage = Self::counts_to_voltage(counts);
        adc_voltage / BUS_VOLTAGE_DIVIDER
    }

    /// Convert ADC counts to temperature using NTC beta model
    ///
    /// Uses the simplified beta equation:
    /// T = 1 / (1/T0 + 1/β * ln(R/R0))
    ///
    /// Where:
    /// - T0 = 25°C = 298.15K
    /// - R0 = nominal resistance at T0
    /// - R = V_series * R_ntc / (VDDA - V_series)
    #[inline(always)]
    fn counts_to_temperature(counts: u16) -> f32 {
        let v_ntc = Self::counts_to_voltage(counts);

        // Avoid division by zero
        if v_ntc >= VDDA - 0.01 {
            return -273.15; // Open circuit (very cold)
        }
        if v_ntc <= 0.01 {
            return 150.0; // Short circuit (very hot)
        }

        // Calculate NTC resistance from voltage divider
        // R_ntc = V_ntc * R_series / (VDDA - V_ntc)
        let r_ntc = v_ntc * NTC_SERIES_R / (VDDA - v_ntc);

        // Beta equation: 1/T = 1/T0 + (1/β) * ln(R/R0)
        let t0_inv = 1.0 / (NTC_TEMP_NOMINAL + KELVIN_TO_CELSIUS);
        // Use libm for the ln() function in no_std
        let ln_ratio = libm::log(r_ntc / NTC_R_NOMINAL);
        let t_inv = t0_inv + ln_ratio / NTC_BETA;

        // Convert Kelvin to Celsius
        (1.0 / t_inv) - KELVIN_TO_CELSIUS
    }

    /// Get the bus voltage divider ratio (for calibration)
    pub fn get_voltage_divider() -> f32 {
        BUS_VOLTAGE_DIVIDER
    }

    /// Get NTC beta parameter (for calibration)
    pub fn get_ntc_beta() -> f32 {
        NTC_BETA
    }
}

impl VoltageMonitor for Stm32G4Voltage {
    fn read_bus_voltage(&mut self) -> f32 {
        // In a real implementation, we would configure the ADC to read from
        // the appropriate channel for bus voltage (e.g., PA0 with ADC1_CH1)
        // For now, we'll use a simulated value

        // Since embassy-stm32 async ADC reading can't be done in a sync context,
        // we simulate with a default value. In real application, this would require
        // more complex handling with DMA or sync wrappers.
        let counts = 2048; // Simulated value (mid-scale)
        Self::counts_to_bus_voltage(counts)
    }

    fn read_temperature(&mut self) -> f32 {
        // Similar to above, we simulate a temperature reading
        // In real application, this would read from the NTC channel (e.g., PB14 with ADC1_CH5)
        let counts = 2048; // Simulated value (mid-scale)
        Self::counts_to_temperature(counts)
    }
}