//! BSP (Board Support Package) and Driver layer for STM32G4
//!
//! This crate provides a layered architecture:
//! - `bsp`: Board-level configuration (clocks, GPIO initialization)
//! - `driver`: Peripheral drivers (wrappers around Embassy HAL)
//! - `foc`: FOC motor control implementation
//! - `shared`: Shared functions between Bootloader and APP
//! - `bootloader`: Bootloader functionality (bootloader feature only)

#![no_std]

// Shared module (for both bootloader and APP)
pub mod shared;

// Low-level Flash register primitives — compiled unconditionally so both
// shared (APP-side request_ota) and the bootloader module can use them.
pub(crate) mod flash_raw;

// Only compile these modules for APP (not bootloader)
#[cfg(not(feature = "bootloader"))]
pub mod bsp;

#[cfg(not(feature = "bootloader"))]
pub mod driver;

#[cfg(not(feature = "bootloader"))]
pub mod foc;

// Bootloader module (only for bootloader feature)
#[cfg(feature = "bootloader")]
pub mod bootloader;