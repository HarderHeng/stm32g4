//! BSP (Board Support Package) and Driver layer for STM32G4
//!
//! This crate provides a layered architecture:
//! - `bsp`: Board-level configuration (clocks, GPIO initialization)
//! - `driver`: Peripheral drivers (wrappers around Embassy HAL)

#![no_std]

pub mod bsp;
pub mod driver;