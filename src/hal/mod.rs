//! HAL trait definitions for FOC motor control
//!
//! This module defines the hardware abstraction layer traits that decouple
//! FOC algorithms from concrete hardware implementations. The goal is to
//! enable portability across different MCUs and HAL crates.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │         FOC Algorithm Layer             │
//! │    FocController<H: FocHal>             │
//! └─────────────────┬───────────────────────┘
//!                   │ uses
//! ┌─────────────────▼───────────────────────┐
//! │         HAL Trait Layer (this)          │
//! │    ┌─────────────────────────────┐      │
//! │    │    FocHal (super-trait)     │      │
//! │    │  + PwmOutput                │      │
//! │    │  + CurrentSense             │      │
//! │    │  + VoltageMonitor           │      │
//! │    │  + MotorParameters          │      │
//! │    └─────────────────────────────┘      │
//! └─────────────────┬───────────────────────┘
//!                   │ implemented by
//! ┌─────────────────▼───────────────────────┐
//! │    STM32G4 HAL Implementation           │
//! │    Stm32G4Hal struct                    │
//! └─────────────────────────────────────────┘
//! ```

pub mod pwm;
pub mod adc;
pub mod voltage;
pub mod params;

pub use pwm::PwmOutput;
pub use adc::CurrentSense;
pub use voltage::VoltageMonitor;
pub use params::MotorParameters;

/// FOC hardware abstraction layer combining all required interfaces.
///
/// This is the main trait that the FOC controller depends on.
/// It aggregates all hardware interfaces needed for field-oriented control.
pub trait FocHal:
    PwmOutput +
    CurrentSense +
    VoltageMonitor +
    Sized
{
    /// Motor electrical parameters type
    type MotorParams: MotorParameters;

    /// Get motor parameters
    fn motor_params(&self) -> Self::MotorParams;

    /// Get mutable reference to PWM output
    fn pwm_mut(&mut self) -> &mut impl PwmOutput;

    /// Get mutable reference to current sense ADC
    fn adc_mut(&mut self) -> &mut impl CurrentSense;

    /// Get mutable reference to voltage monitor
    fn voltage_mut(&mut self) -> &mut impl VoltageMonitor;
}

/// Calibration error types
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CalibrateError {
    /// ADC clock or hardware failure
    Timeout,
    /// Voltage too low for calibration
    Undervoltage,
    /// Motor not stopped
    MotorRunning,
}
