//! Driver trait interfaces for FOC motor control
//!
//! Decouples upper-layer FOC algorithms from concrete hardware drivers.

/// PWM output abstraction for 3-phase motor control.
pub trait PwmOutput {
    /// Set raw CCR values for phases U, V, W.
    /// Values are in timer ticks (0 .. period()).
    fn set_duties_raw(&mut self, u: u16, v: u16, w: u16);

    /// Set CCR4 compare value used to trigger the injected ADC conversion.
    fn set_adc_trigger(&mut self, ccr4: u16);

    /// Timer period (ARR value). Maximum valid raw duty value.
    fn period() -> u16;

    /// Enable all PWM output channels (MOE + CH1-3).
    fn enable_outputs(&mut self);

    /// Disable all PWM output channels.
    fn disable_outputs(&mut self);
}

/// Current sensing abstraction.
pub trait CurrentSampler {
    /// Read corrected phase currents from the previous PWM cycle's injected ADC result.
    /// Returns (Ia, Ib, Ic) in Amperes.
    fn read_phase_currents(&mut self) -> (f32, f32, f32);

    /// Called inside the PWM update interrupt to pre-program the next ADC injected
    /// channel sequence for the given FOC sector (0-5).
    fn update_sector(&mut self, sector: u8);

    /// Zero-current offset calibration. Must be called with PWM outputs disabled.
    /// Performs multiple injected conversions in software-trigger mode and averages
    /// the results to determine the per-channel ADC mid-point offsets.
    fn calibrate(&mut self);
}

/// Bus voltage and temperature monitoring.
pub trait BusMonitor {
    /// Bus voltage in Volts.
    fn bus_voltage_v(&mut self) -> f32;

    /// Heat-sink / NTC temperature in degrees Celsius.
    fn temperature_c(&mut self) -> f32;
}
