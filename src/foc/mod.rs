//! FOC (Field Oriented Control) motor control module
//!
//! This module provides the core FOC implementation including:
//! - Motor parameters
//! - Controller logic
//! - Sensor processing
//!
//! # Layer Architecture
//! The FOC layer sits above the Driver layer and uses traits for hardware abstraction:
//!
//! ```text
//! ┌─────────────────────────────────────┐
//! │           Application               │  (main.rs - wires everything)
//! ├─────────────────────────────────────┤
//! │         FOC Algorithm Layer         │  (this module)
//! │  ┌──────────┬──────────┬─────────┐  │
//! │  │transforms│    PI    │  SVPWM  │  │  Pure math, no hardware deps
//! │  └──────────┴──────────┴─────────┘  │
//! │              │                      │
//! │         Motor Controller            │  State machine using traits
//! ├─────────────────────────────────────┤
//! │         Driver Trait Layer          │  (traits.rs)
//! │   PwmOutput │ CurrentSampler        │
//! ├─────────────────────────────────────┤
//! │      Driver Implementation Layer    │  (pwm.rs, adc.rs, etc.)
//! │   TIM1 PWM │ ADC │ OPAMP │ COMP     │  Direct hardware access
//! ├─────────────────────────────────────┤
//! │              BSP Layer              │  (bsp/* - clocks, power)
//! └─────────────────────────────────────┘
//! ```

pub mod transforms;
pub mod pi;
pub mod svpwm;
pub mod controller;

// Legacy re-export for backward compatibility
// New code should use `controller::FocController` instead
pub use controller::{FocController, FocMode, MotorState, RampGenerator};
pub use transforms::{clarke_transform, park_transform, inv_park_transform, inv_clarke_transform, ElectricalAngle};
pub use pi::{PiController, DualCurrentController};
pub use svpwm::{SvpwmModulator, Sector};