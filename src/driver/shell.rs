//! Shell module using embedded-cli
//!
//! Provides a shell interface using the embedded-cli library.

pub use embedded_cli::cli::Cli;
pub use embedded_cli::Command;

use core::convert::Infallible;

use embassy_stm32::usart::{BufferedUartTx, BufferedInterruptHandler, Config};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::peripherals::USART2;
use embassy_stm32::usart::{TxPin, RxPin, Instance};
use embassy_stm32::Peri;
use embedded_cli::cli::{CliBuilder, CliHandle};
use embedded_cli::codes;
use embedded_io::Write;
use static_cell::StaticCell;
use ufmt::uwrite;

/// LED subcommands
#[derive(Command)]
pub enum LedCommand {
    /// Turn LED on
    On,
    /// Turn LED off
    Off,
    /// Toggle LED
    Toggle,
}

/// Motor subcommands (placeholder for future implementation)
#[derive(Command)]
pub enum MotorCommand {
    /// Show motor status
    Status,
    /// Start motor
    Start,
    /// Stop motor
    Stop,
}

/// System subcommands
#[derive(Command)]
pub enum SystemCommand {
    /// Show system info
    Info,
    /// Request OTA update (reboot to bootloader)
    Ota,
}

/// All commands
#[derive(Command)]
pub enum Command<'a> {
    /// Say hello
    Hello {
        /// Name to greet
        name: Option<&'a str>,
    },
    /// Clear screen
    Clear,
    /// Show version
    Version,
    /// Echo text
    Echo {
        /// Text to echo
        text: Option<&'a str>,
    },
    /// Control LED
    Led {
        #[command(subcommand)]
        command: LedCommand,
    },
    /// Motor control
    Motor {
        #[command(subcommand)]
        command: MotorCommand,
    },
    /// System commands
    System {
        #[command(subcommand)]
        command: SystemCommand,
    },
}

/// Process command callback
fn process_command<W>(cli: &mut CliHandle<'_, W, W::Error>, cmd: Command<'_>)
where
    W: Write,
{
    let writer = cli.writer();
    let _ = match cmd {
        Command::Hello { name } => {
            uwrite!(writer, "Hello, {}!\r\n", name.unwrap_or("World"))
        }
        Command::Clear => writer.write_str("\x1b[2J\x1b[H"),
        Command::Version => writer.write_str("STM32G4 FOC v0.6.0\r\n"),
        Command::Echo { text } => uwrite!(writer, "{}\r\n", text.unwrap_or("")),
        Command::Led { command } => match command {
            LedCommand::On => writer.write_str("LED ON\r\n"),
            LedCommand::Off => writer.write_str("LED OFF\r\n"),
            LedCommand::Toggle => writer.write_str("LED TOGGLE\r\n"),
        },
        Command::Motor { command } => match command {
            MotorCommand::Status => writer.write_str("Motor: initialized (FOC ready)\r\nPWM: 20kHz\r\nADC: dual mode\r\n"),
            MotorCommand::Start => writer.write_str("Motor start (not implemented)\r\n"),
            MotorCommand::Stop => writer.write_str("Motor stop (not implemented)\r\n"),
        },
        Command::System { command } => match command {
            SystemCommand::Info => writer.write_str("MCU: STM32G431CB\r\nClock: 170MHz\r\nFOC: enabled\r\nBootloader: v1.0\r\n"),
            SystemCommand::Ota => {
                let _ = writer.write_str("Entering OTA mode...\r\n");
                // Request OTA mode (will reset)
                crate::shared::request_ota();
            }
        },
    };
}

/// TX buffer size
const TX_BUF_SIZE: usize = 256;

/// RX buffer size
const RX_BUF_SIZE: usize = 256;

/// Static TX buffer
static TX_BUF: StaticCell<[u8; TX_BUF_SIZE]> = StaticCell::new();

/// Static RX buffer
static RX_BUF: StaticCell<[u8; RX_BUF_SIZE]> = StaticCell::new();

/// Static UART TX storage
static SHELL_TX: StaticCell<BufferedUartTx<'static>> = StaticCell::new();

/// Global mutable reference to the TX (set by init_shell)
static mut SHELL_TX_REF: *mut BufferedUartTx<'static> = core::ptr::null_mut();

/// Initialize shell with BufferedUart TX/RX
/// Returns the RX half for use in shell_task
pub fn init_shell<Tx, Rx, I>(
    usart: Peri<'static, USART2>,
    tx_pin: Peri<'static, Tx>,
    rx_pin: Peri<'static, Rx>,
    irq: I,
    config: Config,
) -> embassy_stm32::usart::BufferedUartRx<'static>
where
    Tx: TxPin<USART2>,
    Rx: RxPin<USART2>,
    I: Binding<<USART2 as Instance>::Interrupt, BufferedInterruptHandler<USART2>> + 'static,
{
    let tx_buf = TX_BUF.init([0u8; TX_BUF_SIZE]);
    let rx_buf = RX_BUF.init([0u8; RX_BUF_SIZE]);

    let uart = embassy_stm32::usart::BufferedUart::new(
        usart, rx_pin, tx_pin, tx_buf, rx_buf, irq, config
    ).expect("BufferedUart config");

    let (tx, rx) = uart.split();

    let tx_ref = SHELL_TX.init(tx);
    unsafe {
        SHELL_TX_REF = tx_ref;
    }

    rx
}

/// Get the initialized TX reference
/// Panics if init_shell has not been called
pub fn get_shell_tx() -> &'static mut BufferedUartTx<'static> {
    unsafe {
        if SHELL_TX_REF.is_null() {
            panic!("Shell TX not initialized");
        }
        &mut *SHELL_TX_REF
    }
}

/// Writer that uses a static mutable BufferedUartTx reference
/// Implements embedded_io::Write for use with embedded-cli
/// Uses blocking_write which writes to a ring buffer (safe in async context
/// because it doesn't block on hardware, just on buffer space)
pub struct ShellWriter {
    tx: &'static mut BufferedUartTx<'static>,
}

impl ShellWriter {
    /// Create a new shell writer
    pub fn new(tx: &'static mut BufferedUartTx<'static>) -> Self {
        Self { tx }
    }
}

impl embedded_io::ErrorType for ShellWriter {
    type Error = Infallible;
}

impl Write for ShellWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // Use embedded_io::Write trait method which calls blocking_write internally
        // This is safe in async context - it only blocks if buffer is full,
        // and the interrupt handler will drain the buffer
        match <BufferedUartTx<'_> as Write>::write(self.tx, buf) {
            Ok(len) => Ok(len),
            Err(_) => Ok(buf.len()), // Ignore errors, report all bytes written
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        let _ = <BufferedUartTx<'_> as Write>::flush(self.tx);
        Ok(())
    }
}

impl ufmt::uWrite for ShellWriter {
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_all(s.as_bytes())
    }
}

/// Shell wrapper using embedded-cli
pub struct Shell {
    cli: Cli<ShellWriter, Infallible, [u8; 64], [u8; 64]>,
}

impl Shell {
    /// Create new shell with the given writer
    pub fn new(writer: ShellWriter) -> Self {
        let cli = CliBuilder::default()
            .writer(writer)
            .prompt("G431> ")
            .command_buffer([0u8; 64])
            .history_buffer([0u8; 64])
            .build()
            .expect("CLI build");
        Self { cli }
    }

    /// Process input byte
    /// Handles terminal compatibility (e.g., DEL vs Backspace)
    pub fn process(&mut self, byte: u8) {
        // Many terminals send DEL (0x7F) instead of Backspace (0x08)
        // Convert DEL to Backspace for compatibility
        let byte = if byte == 0x7F { codes::BACKSPACE } else { byte };

        let _ = self.cli.process_byte::<Command, _>(
            byte,
            &mut Command::processor(|cli, cmd| {
                process_command(cli, cmd);
                Ok(())
            }),
        );
    }

    /// Print welcome message
    pub fn print_welcome(&mut self) {
        let _ = self.cli.write(|writer| {
            writer.write_str("STM32G4 FOC v0.6.0\r\n")?;
            writer.write_str("Type 'help' for commands\r\n")
        });
    }
}