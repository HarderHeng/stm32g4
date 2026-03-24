//! Simple OTA Protocol for STM32G4 Bootloader
//!
//! Frame format:
//! | SOF (1) | CMD (1) | LEN (2, BE) | DATA (N) | CRC16 (2, BE) | EOF (1) |
//!
//! SOF = 0x7E, EOF = 0x7F
//!
//! Commands:
//! - ERASE (0x01): Erase flash pages
//! - WRITE (0x02): Write data to flash
//! - READ (0x03): Read data from flash (for verification)
//! - RESET (0x04): Reset and jump to APP
//! - INFO (0x05): Get bootloader info
//!
//! Response has CMD | 0x80

#![allow(dead_code)]

use core::ops::Range;

/// Start of frame
pub const SOF: u8 = 0x7E;
/// End of frame
pub const EOF: u8 = 0x7F;

/// Maximum frame data size
pub const MAX_DATA_SIZE: usize = 256;

/// Maximum frame size (SOF + CMD + LEN + DATA + CRC + EOF)
pub const MAX_FRAME_SIZE: usize = 1 + 1 + 2 + MAX_DATA_SIZE + 2 + 1;

/// Command: Erase flash pages
pub const CMD_ERASE: u8 = 0x01;
/// Command: Write data to flash
pub const CMD_WRITE: u8 = 0x02;
/// Command: Read data from flash
pub const CMD_READ: u8 = 0x03;
/// Command: Reset and jump to APP
pub const CMD_RESET: u8 = 0x04;
/// Command: Get bootloader info
pub const CMD_INFO: u8 = 0x05;

/// Response flag (OR'd with command)
pub const CMD_RESPONSE: u8 = 0x80;

/// Status: Success
pub const STATUS_OK: u8 = 0x00;
/// Status: Invalid command
pub const STATUS_INVALID_CMD: u8 = 0x01;
/// Status: Invalid parameters
pub const STATUS_INVALID_PARAM: u8 = 0x02;
/// Status: Flash error
pub const STATUS_FLASH_ERROR: u8 = 0x03;
/// Status: CRC error
pub const STATUS_CRC_ERROR: u8 = 0x04;
/// Status: Invalid frame
pub const STATUS_INVALID_FRAME: u8 = 0x05;
/// Status: Invalid address
pub const STATUS_INVALID_ADDR: u8 = 0x06;
/// Status: App not valid
pub const STATUS_APP_INVALID: u8 = 0x07;

/// APP memory region
pub const APP_START: u32 = 0x0800_5800;
pub const APP_END: u32 = 0x0802_0000;
pub const APP_RANGE: Range<u32> = APP_START..APP_END;

/// Bootloader version
pub const VERSION_MAJOR: u8 = 1;
pub const VERSION_MINOR: u8 = 0;

/// Frame parser state
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParseState {
    Idle,
    GotSof,
    GotCmd,
    GotLenHigh,
    Data,
    GotCrcHigh,
    GotCrcLow,
    GotEof,
}

/// Frame parser
pub struct FrameParser {
    state: ParseState,
    cmd: u8,
    len: u16,
    data: [u8; MAX_DATA_SIZE],
    data_idx: usize,
    crc: u16,
}

impl FrameParser {
    pub const fn new() -> Self {
        Self {
            state: ParseState::Idle,
            cmd: 0,
            len: 0,
            data: [0; MAX_DATA_SIZE],
            data_idx: 0,
            crc: 0,
        }
    }

    /// Reset parser state
    pub fn reset(&mut self) {
        self.state = ParseState::Idle;
        self.cmd = 0;
        self.len = 0;
        self.data_idx = 0;
        self.crc = 0;
    }

    /// Parse incoming byte, returns Some((cmd, data)) when frame is complete
    pub fn parse(&mut self, byte: u8) -> Option<(u8, &[u8])> {
        match self.state {
            ParseState::Idle => {
                if byte == SOF {
                    self.state = ParseState::GotSof;
                    self.cmd = 0;
                    self.len = 0;
                    self.data_idx = 0;
                    self.crc = 0;
                }
                None
            }
            ParseState::GotSof => {
                self.cmd = byte;
                self.state = ParseState::GotCmd;
                None
            }
            ParseState::GotCmd => {
                self.len = (byte as u16) << 8;
                self.state = ParseState::GotLenHigh;
                None
            }
            ParseState::GotLenHigh => {
                self.len |= byte as u16;
                if self.len as usize > MAX_DATA_SIZE {
                    self.reset();
                    return None;
                }
                if self.len == 0 {
                    self.state = ParseState::GotCrcHigh;
                } else {
                    self.state = ParseState::Data;
                }
                None
            }
            ParseState::Data => {
                self.data[self.data_idx] = byte;
                self.data_idx += 1;
                if self.data_idx >= self.len as usize {
                    self.state = ParseState::GotCrcHigh;
                }
                None
            }
            ParseState::GotCrcHigh => {
                self.crc = (byte as u16) << 8;
                self.state = ParseState::GotCrcLow;
                None
            }
            ParseState::GotCrcLow => {
                self.crc |= byte as u16;
                self.state = ParseState::GotEof;
                None
            }
            ParseState::GotEof => {
                // This byte should be EOF
                if byte != EOF {
                    self.reset();
                    return None;
                }

                // Verify CRC
                let mut crc_data = [0u8; MAX_DATA_SIZE + 3];
                crc_data[0] = self.cmd;
                crc_data[1] = (self.len >> 8) as u8;
                crc_data[2] = (self.len & 0xFF) as u8;
                crc_data[3..3 + self.data_idx].copy_from_slice(&self.data[..self.data_idx]);
                let expected = calculate_crc16(&crc_data[..3 + self.data_idx]);

                if self.crc != expected {
                    self.reset();
                    return None;
                }

                let cmd = self.cmd;
                let len = self.data_idx;
                self.state = ParseState::Idle;
                Some((cmd, &self.data[..len]))
            }
        }
    }
}

/// Build a response frame
pub fn build_response(cmd: u8, status: u8, data: &[u8], out: &mut [u8]) -> usize {
    let total_len = 1 + 1 + 2 + 1 + data.len() + 2 + 1;
    if out.len() < total_len {
        return 0;
    }

    let mut idx = 0;
    out[idx] = SOF;
    idx += 1;
    out[idx] = cmd | CMD_RESPONSE;
    idx += 1;

    // LEN = status (1) + data.len()
    let len = 1 + data.len() as u16;
    out[idx] = (len >> 8) as u8;
    idx += 1;
    out[idx] = (len & 0xFF) as u8;
    idx += 1;

    // DATA = status + data
    out[idx] = status;
    idx += 1;
    out[idx..idx + data.len()].copy_from_slice(data);
    idx += data.len();

    // CRC16
    let crc = calculate_crc16(&out[1..idx]);
    out[idx] = (crc >> 8) as u8;
    idx += 1;
    out[idx] = (crc & 0xFF) as u8;
    idx += 1;

    out[idx] = EOF;
    idx += 1;

    idx
}

/// Build an info response frame
pub fn build_info_response(out: &mut [u8]) -> usize {
    // Info data: version (2), app_start (4), app_end (4), page_size (2), page_count (1)
    // APP region: 0x08005800 to 0x08020000 = 106KB = 53 pages
    let info: [u8; 13] = [
        VERSION_MAJOR,
        VERSION_MINOR,
        (APP_START >> 24) as u8,
        (APP_START >> 16) as u8,
        (APP_START >> 8) as u8,
        APP_START as u8,
        (APP_END >> 24) as u8,
        (APP_END >> 16) as u8,
        (APP_END >> 8) as u8,
        APP_END as u8,
        0x08, 0x00, // Page size: 2048 bytes (big endian)
        53, // Page count for APP region (from page 11 to 63)
    ];
    build_response(CMD_INFO, STATUS_OK, &info, out)
}

/// Calculate CRC16-CCITT
pub fn calculate_crc16(data: &[u8]) -> u16 {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_response() {
        let mut buf = [0u8; 64];
        let len = build_response(CMD_ERASE, STATUS_OK, &[], &mut buf);
        assert!(len > 0);
        assert_eq!(buf[0], SOF);
        assert_eq!(buf[1], CMD_ERASE | CMD_RESPONSE);
        assert_eq!(buf[len - 1], EOF);
    }
}