//! STM32G4 OTA Tool
//!
//! Simple command-line tool for firmware updates over UART.

use anyhow::{anyhow, Result};
use clap::{Parser, Subcommand};
use serialport::{SerialPort, SerialPortType};
use std::io::{Read, Write};
use std::path::PathBuf;
use std::time::Duration;

// Protocol constants
const SOF: u8 = 0x7E;
const EOF: u8 = 0x7F;

// Commands
const CMD_ERASE: u8 = 0x01;
const CMD_WRITE: u8 = 0x02;
const CMD_READ: u8 = 0x03;
const CMD_RESET: u8 = 0x04;
const CMD_INFO: u8 = 0x05;
const CMD_RESPONSE: u8 = 0x80;

// Status codes
const STATUS_OK: u8 = 0x00;

// APP region
const APP_START: u32 = 0x0800_5800;
const PAGE_SIZE: usize = 2048;

/// STM32G4 OTA Tool
#[derive(Parser)]
#[command(name = "ota-tool")]
#[command(about = "STM32G4 OTA firmware update tool")]
struct Cli {
    /// Serial port (e.g., /dev/ttyUSB0)
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    port: String,

    /// Baud rate
    #[arg(short, long, default_value = "921600")]
    baud: u32,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Get bootloader info
    Info,

    /// Flash firmware
    Flash {
        /// Firmware binary file
        #[arg(short, long)]
        firmware: PathBuf,

        /// Verify after flashing
        #[arg(short, long)]
        verify: bool,

        /// Reset to APP after flashing
        #[arg(short, long)]
        reset: bool,
    },

    /// Reset device to APP
    Reset,

    /// List available serial ports
    List,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Info => {
            let mut ota = OtaClient::new(&cli.port, cli.baud)?;
            let info = ota.get_info()?;
            println!("Bootloader version: {}.{}", info.version_major, info.version_minor);
            println!("APP region: 0x{:08X} - 0x{:08X}", info.app_start, info.app_end);
            println!("Page size: {} bytes", info.page_size);
            println!("Page count: {}", info.page_count);
        }
        Commands::Flash { firmware, verify, reset } => {
            let mut ota = OtaClient::new(&cli.port, cli.baud)?;

            // Read firmware
            let data = std::fs::read(&firmware)?;
            println!("Firmware size: {} bytes", data.len());

            // Flash
            println!("Flashing...");
            ota.flash_firmware(&data, true)?;

            // Verify
            if verify {
                println!("Verifying...");
                ota.verify_firmware(&data)?;
                println!("Verification passed!");
            }

            // Reset
            if reset {
                println!("Resetting to APP...");
                ota.reset()?;
            }

            println!("Done!");
        }
        Commands::Reset => {
            let mut ota = OtaClient::new(&cli.port, cli.baud)?;
            println!("Resetting to APP...");
            ota.reset()?;
            println!("Done!");
        }
        Commands::List => {
            let ports = serialport::available_ports()?;
            println!("Available ports:");
            for p in ports {
                match p.port_type {
                    SerialPortType::UsbPort(info) => {
                        println!("  {} (USB: {:?} {:?})", p.port_name,
                            info.vid, info.pid);
                    }
                    _ => {
                        println!("  {}", p.port_name);
                    }
                }
            }
        }
    }

    Ok(())
}

/// OTA Client
struct OtaClient {
    port: Box<dyn SerialPort>,
    rx_buf: Vec<u8>,
}

/// Bootloader info
struct BootloaderInfo {
    version_major: u8,
    version_minor: u8,
    app_start: u32,
    app_end: u32,
    page_size: u16,
    page_count: u8,
}

impl OtaClient {
    fn new(port: &str, baud: u32) -> Result<Self> {
        let port = serialport::new(port, baud)
            .timeout(Duration::from_secs(5))
            .open()?;

        Ok(Self {
            port,
            rx_buf: Vec::with_capacity(512),
        })
    }

    fn send_command(&mut self, cmd: u8, data: &[u8]) -> Result<(u8, Vec<u8>)> {
        // Build frame
        let frame = build_frame(cmd, data);

        // Clear RX buffer
        self.rx_buf.clear();

        // Send
        self.port.write_all(&frame)?;
        self.port.flush()?;

        // Receive response
        let start = std::time::Instant::now();
        let mut state = ParseState::Idle;
        let mut resp_cmd = 0u8;
        let mut resp_len = 0u16;
        let mut resp_data = Vec::new();
        let mut crc = 0u16;

        loop {
            if start.elapsed() > Duration::from_secs(10) {
                return Err(anyhow!("Timeout waiting for response"));
            }

            let mut byte = [0u8; 1];
            match self.port.read(&mut byte) {
                Ok(1) => {}
                Ok(_) => continue,
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => return Err(e.into()),
            }

            let b = byte[0];

            match state {
                ParseState::Idle => {
                    if b == SOF {
                        state = ParseState::GotSof;
                        resp_data.clear();
                    }
                }
                ParseState::GotSof => {
                    resp_cmd = b;
                    state = ParseState::GotCmd;
                }
                ParseState::GotCmd => {
                    resp_len = (b as u16) << 8;
                    state = ParseState::GotLenHigh;
                }
                ParseState::GotLenHigh => {
                    resp_len |= b as u16;
                    if resp_len == 0 {
                        state = ParseState::GotCrcHigh;
                    } else {
                        state = ParseState::Data;
                    }
                }
                ParseState::Data => {
                    resp_data.push(b);
                    if resp_data.len() >= resp_len as usize {
                        state = ParseState::GotCrcHigh;
                    }
                }
                ParseState::GotCrcHigh => {
                    crc = (b as u16) << 8;
                    state = ParseState::GotCrcLow;
                }
                ParseState::GotCrcLow => {
                    crc |= b as u16;
                    state = ParseState::GotEof;
                }
                ParseState::GotEof => {
                    if b == EOF {
                        // Verify CRC
                        let mut crc_data = vec![resp_cmd];
                        crc_data.extend_from_slice(&[(resp_len >> 8) as u8, resp_len as u8]);
                        crc_data.extend_from_slice(&resp_data);
                        let expected = calculate_crc16(&crc_data);
                        if crc != expected {
                            return Err(anyhow!("CRC error in response"));
                        }

                        // Check status
                        if resp_data.is_empty() {
                            return Err(anyhow!("Empty response"));
                        }

                        let status = resp_data[0];
                        let data = resp_data[1..].to_vec();
                        return Ok((status, data));
                    } else {
                        // Invalid EOF, reset
                        state = ParseState::Idle;
                    }
                }
            }
        }
    }

    fn get_info(&mut self) -> Result<BootloaderInfo> {
        let (status, data) = self.send_command(CMD_INFO, &[])?;

        if status != STATUS_OK {
            return Err(anyhow!("INFO command failed: status {}", status));
        }

        if data.len() < 13 {
            return Err(anyhow!("Invalid INFO response length"));
        }

        Ok(BootloaderInfo {
            version_major: data[0],
            version_minor: data[1],
            app_start: u32::from_be_bytes([data[2], data[3], data[4], data[5]]),
            app_end: u32::from_be_bytes([data[6], data[7], data[8], data[9]]),
            page_size: u16::from_be_bytes([data[10], data[11]]),
            page_count: data[12],
        })
    }

    fn erase_pages(&mut self, pages: &[u8]) -> Result<()> {
        let mut data = vec![pages.len() as u8];
        data.extend_from_slice(pages);

        let (status, _) = self.send_command(CMD_ERASE, &data)?;

        if status != STATUS_OK {
            return Err(anyhow!("ERASE command failed: status {}", status));
        }

        Ok(())
    }

    fn write_data(&mut self, address: u32, data: &[u8]) -> Result<()> {
        let mut req = vec![];
        req.extend_from_slice(&address.to_be_bytes());
        req.extend_from_slice(data);

        let (status, _) = self.send_command(CMD_WRITE, &req)?;

        if status != STATUS_OK {
            return Err(anyhow!("WRITE command failed: status {}", status));
        }

        Ok(())
    }

    fn read_data(&mut self, address: u32, length: u16) -> Result<Vec<u8>> {
        let mut req = vec![];
        req.extend_from_slice(&address.to_be_bytes());
        req.extend_from_slice(&length.to_be_bytes());

        let (status, data) = self.send_command(CMD_READ, &req)?;

        if status != STATUS_OK {
            return Err(anyhow!("READ command failed: status {}", status));
        }

        // Remove trailing CRC16
        if data.len() < 2 {
            return Err(anyhow!("READ response too short"));
        }

        Ok(data[..data.len() - 2].to_vec())
    }

    fn reset(&mut self) -> Result<()> {
        // Magic number required for reset
        let magic: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

        // Ignore response as device may reset before sending
        let _ = self.send_command(CMD_RESET, &magic);

        Ok(())
    }

    fn flash_firmware(&mut self, firmware: &[u8], progress: bool) -> Result<()> {
        // Calculate pages to erase (starting from page 11)
        let start_page: u8 = 11;
        let num_pages = (firmware.len() + PAGE_SIZE - 1) / PAGE_SIZE;
        let pages_to_erase: Vec<u8> = (start_page..start_page + num_pages as u8).collect();

        if progress {
            println!("  Erasing {} pages...", pages_to_erase.len());
        }

        // Erase in chunks of 10 pages
        for chunk in pages_to_erase.chunks(10) {
            self.erase_pages(chunk)?;
            if progress {
                print!(".");
                std::io::stdout().flush()?;
            }
        }
        if progress {
            println!(" done");
        }

        // Write firmware in chunks
        if progress {
            print!("  Writing: ");
            std::io::stdout().flush()?;
        }

        let chunk_size = 128;
        let mut offset = 0;

        while offset < firmware.len() {
            let end = std::cmp::min(offset + chunk_size, firmware.len());
            let chunk = &firmware[offset..end];

            // Pad to 8-byte alignment
            let mut padded = chunk.to_vec();
            while padded.len() % 8 != 0 {
                padded.push(0xFF);
            }

            self.write_data(APP_START + offset as u32, &padded)?;

            offset = end;

            if progress && offset % 4096 == 0 {
                print!(".");
                std::io::stdout().flush()?;
            }
        }

        if progress {
            println!(" done ({} bytes)", firmware.len());
        }

        Ok(())
    }

    fn verify_firmware(&mut self, firmware: &[u8]) -> Result<()> {
        let chunk_size = 128;
        let mut offset = 0;

        while offset < firmware.len() {
            let len = std::cmp::min(chunk_size, firmware.len() - offset);
            let expected = &firmware[offset..offset + len];

            let read = self.read_data(APP_START + offset as u32, len as u16)?;

            if read.len() < len || &read[..len] != expected {
                return Err(anyhow!("Verification failed at offset {}", offset));
            }

            offset += len;
        }

        Ok(())
    }
}

/// Build a protocol frame
fn build_frame(cmd: u8, data: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(7 + data.len());

    frame.push(SOF);
    frame.push(cmd);
    frame.push((data.len() >> 8) as u8);
    frame.push(data.len() as u8);
    frame.extend_from_slice(data);

    // CRC over cmd + len + data
    let mut crc_data = vec![cmd];
    crc_data.push((data.len() >> 8) as u8);
    crc_data.push(data.len() as u8);
    crc_data.extend_from_slice(data);
    let crc = calculate_crc16(&crc_data);

    frame.push((crc >> 8) as u8);
    frame.push(crc as u8);
    frame.push(EOF);

    frame
}

/// Calculate CRC16-CCITT (MODBUS variant)
fn calculate_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ParseState {
    Idle,
    GotSof,
    GotCmd,
    GotLenHigh,
    Data,
    GotCrcHigh,
    GotCrcLow,
    GotEof,
}