//! BSP (Board Support Package) and Driver layer for STM32G4
//!
//! This crate provides a layered architecture for the APP:
//! - `bsp`: Board-level configuration (clocks, GPIO initialization)
//! - `driver`: Peripheral drivers (wrappers around Embassy HAL)
//! - `foc`: FOC motor control implementation
//!
//! Note: Bootloader is a standalone binary and does not use this library.
//! The bootloader is built with the "bootloader" feature which disables this library.

#![no_std]

// Only compile these modules for APP (not bootloader)
#[cfg(not(feature = "bootloader"))]
pub mod bsp;

#[cfg(not(feature = "bootloader"))]
pub mod driver;

#[cfg(not(feature = "bootloader"))]
pub mod foc;