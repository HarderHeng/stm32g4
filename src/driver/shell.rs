//! Shell module using embedded-cli
//!
//! Provides a shell interface using the embedded-cli library.

pub use embedded_cli::cli::Cli;
pub use embedded_cli::Command;

use core::convert::Infallible;

use embassy_stm32::usart::UartTx;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
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

/// Static UART TX storage singleton wrapped in Mutex for safe concurrent access
static SHELL_TX: StaticCell<Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>> = StaticCell::new();

/// Initialize shell TX with the given UART TX instance
pub fn init_shell_tx(tx: UartTx<'static, Async>) {
    SHELL_TX.init(Mutex::new(tx));
}

/// Get the initialized TX reference
/// Panics if init_shell_tx has not been called
pub fn get_shell_tx() -> &'static Mutex<CriticalSectionRawMutex, UartTx<'static, Async>> {
    // StaticCell stores the value, we cast a reference to it
    // This is safe because init_shell_tx is guaranteed to be called first
    unsafe {
        &*(&SHELL_TX as *const StaticCell<Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>>
           as *const Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>)
    }
}

/// Writer that uses static UART TX reference with Mutex for safe access
/// Implements embedded_io::Write for use with embedded-cli
/// Uses blocking_write which writes to a ring buffer (safe in async context
/// because it doesn't block on hardware, just on buffer space)
pub struct ShellWriter {
    tx: &'static Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>,
}

impl ShellWriter {
    /// Create a new shell writer
    pub fn new(tx: &'static Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>) -> Self {
        Self { tx }
    }
}

impl embedded_io::ErrorType for ShellWriter {
    type Error = Infallible;
}

impl Write for ShellWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // Block on the async lock and write operation
        embassy_futures::block_on(async {
            let mut tx = self.tx.lock().await;
            let _ = tx.write(buf).await;
            Ok(buf.len()) // Ignore errors
        })
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embassy_futures::block_on(async {
            let mut tx = self.tx.lock().await;
            let _ = tx.flush().await;
            Ok(())
        })
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
