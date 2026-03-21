//! Board Support Package (BSP) for STM32G4
//!
//! This module provides board-level configuration including:
//! - Clock configuration (PLL setup for 170MHz)
//! - Power configuration
//! - GPIO initialization

mod board;

pub use board::*;