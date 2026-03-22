//! USART Serial Driver for STM32G4
//!
//! This module re-exports embassy-stm32 usart types for convenience.
//! For DMA-based UART, use the embassy Uart directly in your application.

pub use embassy_stm32::usart::{
    Config, Error, Instance, InterruptHandler, Parity, RxPin, StopBits, TxPin, Uart, UartRx,
    UartTx,
};

/// Default baud rate for USART
pub const DEFAULT_BAUD_RATE: u32 = 115200;