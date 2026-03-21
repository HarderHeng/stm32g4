//! USART Serial Driver for STM32G4
//!
//! Provides interrupt-driven asynchronous serial communication using Embassy's BufferedUart.
//!
//! # Features
//!
//! - Interrupt-driven TX/RX
//! - Internal ring buffers for reliable communication
//! - Async/await API for non-blocking operation
//! - Support for split() to separate TX and RX for concurrent use

use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::usart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx, Config, Instance,
    Parity, RxPin, StopBits, TxPin,
};
use embassy_stm32::{peripherals, Peri};
use embedded_io_async::{Read, Write};

/// Default baud rate for USART
pub const DEFAULT_BAUD_RATE: u32 = 115200;

/// TX buffer size in bytes
pub const TX_BUF_SIZE: usize = 256;

/// RX buffer size in bytes
pub const RX_BUF_SIZE: usize = 256;

/// Serial driver wrapping Embassy's BufferedUart
///
/// Provides async read/write operations with internal buffering.
pub struct Serial<'d> {
    uart: BufferedUart<'d>,
}

/// Split serial handle into separate TX and RX halves
///
/// Useful for concurrent transmission and reception in different tasks.
pub struct SerialSplit<'d> {
    pub tx: BufferedUartTx<'d>,
    pub rx: BufferedUartRx<'d>,
}

/// Create a default USART config with specified baud rate
fn make_config(baudrate: u32) -> Config {
    let mut config = Config::default();
    config.baudrate = baudrate;
    config.parity = Parity::ParityNone;
    config.stop_bits = StopBits::STOP1;
    config
}

impl<'d> Serial<'d> {
    /// Create a new USART2 instance on PB3 (TX) and PB4 (RX)
    ///
    /// # Arguments
    ///
    /// * `usart` - USART2 peripheral
    /// * `rx_pin` - RX pin (PB4)
    /// * `tx_pin` - TX pin (PB3)
    /// * `tx_buffer` - Static TX buffer
    /// * `rx_buffer` - Static RX buffer
    /// * `irq` - Interrupt binding struct created by `bind_interrupts!`
    ///
    /// # Example
    ///
    /// ```ignore
    /// use embassy_stm32::{bind_interrupts, usart, peripherals};
    ///
    /// bind_interrupts!(struct Irqs {
    ///     USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
    /// });
    ///
    /// static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    /// static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    ///
    /// let serial = Serial::new_usart2(
    ///     p.USART2,
    ///     p.PB4,  // RX
    ///     p.PB3,  // TX
    ///     TX_BUF.init([0u8; 256]),
    ///     RX_BUF.init([0u8; 256]),
    ///     Irqs,
    /// );
    /// ```
    pub fn new_usart2<R, Tx, I>(
        usart: Peri<'d, peripherals::USART2>,
        rx_pin: Peri<'d, R>,
        tx_pin: Peri<'d, Tx>,
        tx_buffer: &'d mut [u8],
        rx_buffer: &'d mut [u8],
        irq: I,
    ) -> Self
    where
        R: RxPin<peripherals::USART2>,
        Tx: TxPin<peripherals::USART2>,
        I: Binding<
            <peripherals::USART2 as Instance>::Interrupt,
            BufferedInterruptHandler<peripherals::USART2>,
        > + 'd,
    {
        let config = make_config(DEFAULT_BAUD_RATE);

        let uart = BufferedUart::new(usart, rx_pin, tx_pin, tx_buffer, rx_buffer, irq, config)
            .expect("USART2 config should not fail");

        Self { uart }
    }

    /// Create a new USART2 instance with custom baud rate
    pub fn new_usart2_with_baud<R, Tx, I>(
        usart: Peri<'d, peripherals::USART2>,
        rx_pin: Peri<'d, R>,
        tx_pin: Peri<'d, Tx>,
        tx_buffer: &'d mut [u8],
        rx_buffer: &'d mut [u8],
        irq: I,
        baudrate: u32,
    ) -> Self
    where
        R: RxPin<peripherals::USART2>,
        Tx: TxPin<peripherals::USART2>,
        I: Binding<
            <peripherals::USART2 as Instance>::Interrupt,
            BufferedInterruptHandler<peripherals::USART2>,
        > + 'd,
    {
        let config = make_config(baudrate);

        let uart = BufferedUart::new(usart, rx_pin, tx_pin, tx_buffer, rx_buffer, irq, config)
            .expect("USART2 config should not fail");

        Self { uart }
    }

    /// Write data asynchronously
    ///
    /// Returns the number of bytes written.
    pub async fn write(&mut self, data: &[u8]) -> Result<usize, embassy_stm32::usart::Error> {
        Write::write(&mut self.uart, data).await
    }

    /// Read data asynchronously
    ///
    /// Returns the number of bytes read into the buffer.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, embassy_stm32::usart::Error> {
        Read::read(&mut self.uart, buf).await
    }

    /// Write a single byte
    pub async fn write_byte(&mut self, byte: u8) -> Result<(), embassy_stm32::usart::Error> {
        Write::write(&mut self.uart, &[byte]).await?;
        Ok(())
    }

    /// Flush any pending write data
    pub async fn flush(&mut self) -> Result<(), embassy_stm32::usart::Error> {
        Write::flush(&mut self.uart).await
    }

    /// Split the serial into separate TX and RX halves
    ///
    /// This allows concurrent transmission and reception in different async tasks.
    pub fn split(self) -> SerialSplit<'d> {
        let (tx, rx) = self.uart.split();
        SerialSplit { tx, rx }
    }
}

impl<'d> SerialSplit<'d> {
    /// Write data asynchronously (TX half)
    pub async fn write(&mut self, data: &[u8]) -> Result<usize, embassy_stm32::usart::Error> {
        Write::write(&mut self.tx, data).await
    }

    /// Read data asynchronously (RX half)
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, embassy_stm32::usart::Error> {
        Read::read(&mut self.rx, buf).await
    }
}