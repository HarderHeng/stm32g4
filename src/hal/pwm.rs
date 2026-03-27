//! PWM output trait for 3-phase motor control
//!
//! This trait abstracts the hardware-specific PWM generation details,
//! providing a clean interface for the FOC algorithm layer.

/// PWM output abstraction for 3-phase motor control.
///
/// This trait provides the interface for setting duty cycles and configuring
/// the PWM hardware for FOC motor control applications.
///
/// # Implementation Notes
///
/// - Duty values are in timer ticks (0 to `period()`)
/// - For center-aligned PWM, the effective switching frequency is doubled
/// - Dead time is handled by the implementation, not the caller
pub trait PwmOutput {
    /// Timer period (ARR value) in ticks.
    ///
    /// This is the maximum valid raw duty value. For center-aligned PWM
    /// at 20kHz with a 170MHz timer clock, this is typically 4250.
    fn period(&self) -> u16;

    /// Set raw CCR values for phases U, V, W.
    ///
    /// Values are in timer ticks (0 .. period()).
    ///
    /// # Safety
    ///
    /// This function is designed for performance.
    /// The caller should ensure values are within valid range.
    fn set_duties_raw(&mut self, u: u16, v: u16, w: u16);

    /// Set duty cycles in SVPWM-normalized range.
    ///
    /// # Arguments
    ///
    /// * `duty_u` - Phase U duty cycle (-1.0 to 1.0, where 0 = 50%)
    /// * `duty_v` - Phase V duty cycle (-1.0 to 1.0, where 0 = 50%)
    /// * `duty_w` - Phase W duty cycle (-1.0 to 1.0, where 0 = 50%)
    ///
    /// # Implementation Notes
    ///
    /// Default implementation converts normalized duty to raw ticks.
    /// SVPWM implementations may override for optimized calculation.
    fn set_duties(&mut self, duty_u: f32, duty_v: f32, duty_w: f32) {
        let period = self.period() as f32;
        let half = period / 2.0;

        let u = ((duty_u * half) + half) as u16;
        let v = ((duty_v * half) + half) as u16;
        let w = ((duty_w * half) + half) as u16;

        self.set_duties_raw(u.min(period as u16), v.min(period as u16), w.min(period as u16));
    }

    /// Set CCR4 compare value used to trigger the injected ADC conversion.
    ///
    /// This controls when during the PWM cycle the ADC samples the current.
    /// For center-aligned PWM, optimal sampling is at the valley (period/4).
    fn set_adc_trigger(&mut self, ccr: u16);

    /// Enable all PWM output channels (MOE + CH1-3).
    ///
    /// After calling this, the PWM signals appear on the output pins.
    fn enable_outputs(&mut self);

    /// Disable all PWM output channels.
    ///
    /// After calling this, no PWM signals appear on the output pins.
    fn disable_outputs(&mut self);

    /// Enable UPDATE interrupt for FOC control loop.
    ///
    /// This enables the hardware-triggered interrupt that fires once per
    /// PWM cycle, used to execute the FOC algorithm at a precise frequency.
    fn enable_update_interrupt(&mut self);

    /// Disable UPDATE interrupt.
    fn disable_update_interrupt(&mut self);

    /// Check if PWM outputs are enabled.
    ///
    /// Default implementation always returns true. Override if hardware status
    /// can be read.
    fn is_enabled(&self) -> bool {
        true
    }
}
