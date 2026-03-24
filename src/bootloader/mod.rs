//! Bootloader module for STM32G4
//!
//! Provides:
//! - Boot Flag storage and management
//! - Flash operations
//! - Simple OTA protocol over UART

mod boot_flag;
mod flash;
mod ota;
pub mod protocol;
pub mod uart;

pub use boot_flag::*;
pub use flash::*;
pub use ota::*;
pub use protocol::*;
pub use uart::*;

// Re-export shared types
pub use crate::shared::BootFlag;