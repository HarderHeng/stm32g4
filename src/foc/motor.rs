//! Motor parameters for FOC control
//!
//! Defines motor electrical and mechanical parameters.

/// Motor electrical and mechanical parameters
pub struct MotorParams {
    /// Number of pole pairs
    pub pole_pairs: u8,
    /// Stator resistance (Ohms)
    pub rs: f32,
    /// Stator inductance (Henries)
    pub ls: f32,
    /// Nominal voltage (Volts)
    pub nominal_voltage: f32,
    /// Nominal current (Amps)
    pub nominal_current: f32,
    /// Maximum speed (RPM)
    pub max_speed: u32,
}

impl Default for MotorParams {
    fn default() -> Self {
        Self {
            pole_pairs: 4,
            rs: 0.32,
            ls: 0.00047,
            nominal_voltage: 24.0,
            nominal_current: 5.0,
            max_speed: 6420,
        }
    }
}

/// Current sensing parameters
pub struct CurrentSenseParams {
    /// Shunt resistor value (Ohms)
    pub shunt_resistance: f32,
    /// OPAMP gain
    pub opamp_gain: f32,
    /// ADC reference voltage (Volts)
    pub vref: f32,
    /// ADC resolution (bits)
    pub adc_resolution: u8,
}

impl Default for CurrentSenseParams {
    fn default() -> Self {
        Self {
            shunt_resistance: 0.003, // 3mΩ
            opamp_gain: 8.0,         // Internal PGA gain
            vref: 3.3,
            adc_resolution: 12,
        }
    }
}

impl CurrentSenseParams {
    /// Convert ADC raw value to current (Amps)
    /// Assumes 12-bit ADC, centered at mid-scale (2048)
    pub fn adc_to_current(&self, adc_value: u16) -> f32 {
        let max_adc = (1u16 << self.adc_resolution) as f32;
        let mid_scale = max_adc / 2.0;
        let adc_offset = adc_value as f32 - mid_scale;

        // Voltage at ADC input
        let v_adc = adc_offset * self.vref / max_adc;

        // Current through shunt (accounting for gain)
        v_adc / (self.shunt_resistance * self.opamp_gain)
    }
}

/// PWM parameters for motor control
pub struct PwmParams {
    /// PWM frequency (Hz)
    pub frequency: u32,
    /// Dead time (nanoseconds)
    pub dead_time_ns: u16,
    /// Maximum duty cycle (0.0 - 1.0)
    pub max_duty: f32,
}

impl Default for PwmParams {
    fn default() -> Self {
        Self {
            frequency: 20_000, // 20kHz
            dead_time_ns: 800,
            max_duty: 0.95,
        }
    }
}