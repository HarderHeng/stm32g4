//! Driver layer for STM32G4 peripherals

mod serial;
mod shell;

pub use serial::*;
pub use shell::*;