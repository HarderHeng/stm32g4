//! Board Support Package (BSP) for STM32G4
//!
//! This module provides board-level configuration including:
//! - Clock configuration (PLL setup for 170MHz)
//! - Power configuration
//! - GPIO initialization
//!
//! # Board Support
//!
//! - B-G431B-ESC1: ST motor control discovery board
//! - Custom boards: implement your own board configuration

pub mod board;
pub mod config;
pub mod b_g431b_esc1;

pub use board::*;
pub use config::*;
pub use b_g431b_esc1::*;

// Re-export the specific board configuration
pub use config::BG431bEsc1Config;