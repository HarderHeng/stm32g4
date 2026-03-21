//! Driver layer for STM32G4 peripherals
//!
//! This module provides business-level abstractions over Embassy HAL,
//! simplifying peripheral usage for application code.

mod serial;

pub use serial::*;