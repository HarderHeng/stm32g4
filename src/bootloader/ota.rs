//! OTA Handler for STM32G4 Bootloader
//!
//! Handles OTA commands and Flash operations.

use crate::bootloader::flash;
use crate::bootloader::protocol::*;

/// OTA Handler
pub struct OtaHandler {
    parser: FrameParser,
}

impl OtaHandler {
    pub const fn new() -> Self {
        Self {
            parser: FrameParser::new(),
        }
    }

    /// Process incoming byte, returns response frame length if a complete command was processed
    pub fn process_byte(&mut self, byte: u8, response: &mut [u8]) -> usize {
        let parsed = self.parser.parse(byte);
        if let Some((cmd, data)) = parsed {
            // Clone data to avoid borrow issues
            let data_len = data.len();
            let mut data_copy = [0u8; MAX_DATA_SIZE];
            data_copy[..data_len].copy_from_slice(data);
            self.handle_command(cmd, &data_copy[..data_len], response)
        } else {
            0
        }
    }

    /// Handle a parsed command
    pub fn handle_command(&mut self, cmd: u8, data: &[u8], response: &mut [u8]) -> usize {
        match cmd {
            CMD_ERASE => self.handle_erase(data, response),
            CMD_WRITE => self.handle_write(data, response),
            CMD_READ => self.handle_read(data, response),
            CMD_RESET => self.handle_reset(data, response),
            CMD_INFO => build_info_response(response),
            _ => build_response(cmd, STATUS_INVALID_CMD, &[], response),
        }
    }

    /// Handle ERASE command
    /// Data format: [page_count: 1 byte] [page_numbers: N bytes]
    fn handle_erase(&mut self, data: &[u8], response: &mut [u8]) -> usize {
        if data.is_empty() {
            return build_response(CMD_ERASE, STATUS_INVALID_PARAM, &[], response);
        }

        let page_count = data[0] as usize;
        if page_count == 0 || data.len() < 1 + page_count {
            return build_response(CMD_ERASE, STATUS_INVALID_PARAM, &[], response);
        }

        // Erase each page
        for i in 0..page_count {
            let page = data[1 + i];
            if let Err(_) = flash::erase_page(page) {
                return build_response(CMD_ERASE, STATUS_FLASH_ERROR, &[page], response);
            }
        }

        build_response(CMD_ERASE, STATUS_OK, &[], response)
    }

    /// Handle WRITE command
    /// Data format: [address: 4 bytes BE] [data: N bytes]
    fn handle_write(&mut self, data: &[u8], response: &mut [u8]) -> usize {
        if data.len() < 4 {
            return build_response(CMD_WRITE, STATUS_INVALID_PARAM, &[], response);
        }

        // Parse address (big endian)
        let address = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        let write_data = &data[4..];

        // Validate address is in APP region
        if !is_valid_app_address(address, write_data.len()) {
            return build_response(CMD_WRITE, STATUS_INVALID_ADDR, &[], response);
        }

        // Pad data to 8-byte alignment if needed
        let mut padded = [0u8; MAX_DATA_SIZE + 8];
        let padded_len = (write_data.len() + 7) & !7; // Round up to 8
        padded[..write_data.len()].copy_from_slice(write_data);

        // Write to flash
        match flash::write(address, &padded[..padded_len]) {
            Ok(()) => {
                // Return written address and length
                let resp_data: [u8; 8] = [
                    (address >> 24) as u8,
                    (address >> 16) as u8,
                    (address >> 8) as u8,
                    address as u8,
                    (write_data.len() >> 24) as u8,
                    (write_data.len() >> 16) as u8,
                    (write_data.len() >> 8) as u8,
                    write_data.len() as u8,
                ];
                build_response(CMD_WRITE, STATUS_OK, &resp_data, response)
            }
            Err(_) => build_response(CMD_WRITE, STATUS_FLASH_ERROR, &[], response),
        }
    }

    /// Handle READ command
    /// Data format: [address: 4 bytes BE] [length: 2 bytes BE]
    fn handle_read(&mut self, data: &[u8], response: &mut [u8]) -> usize {
        if data.len() < 6 {
            return build_response(CMD_READ, STATUS_INVALID_PARAM, &[], response);
        }

        // Parse address and length (big endian)
        let address = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
        let length = u16::from_be_bytes([data[4], data[5]]) as usize;

        // Validate address and length
        if length > MAX_DATA_SIZE - 8 || !is_valid_app_address(address, length) {
            return build_response(CMD_READ, STATUS_INVALID_ADDR, &[], response);
        }

        // Read from flash
        let read_data = unsafe {
            core::slice::from_raw_parts(address as *const u8, length)
        };

        // Calculate CRC of read data
        let crc = calculate_crc16(read_data);
        let mut resp_data = [0u8; MAX_DATA_SIZE + 2];
        resp_data[..length].copy_from_slice(read_data);
        resp_data[length] = (crc >> 8) as u8;
        resp_data[length + 1] = crc as u8;

        build_response(CMD_READ, STATUS_OK, &resp_data[..length + 2], response)
    }

    /// Handle RESET command
    /// Data format: [magic: 4 bytes = 0xDEADBEEF]
    fn handle_reset(&mut self, data: &[u8], response: &mut [u8]) -> usize {
        // Require magic number to prevent accidental reset
        const RESET_MAGIC: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

        if data.len() < 4 || data[..4] != RESET_MAGIC {
            return build_response(CMD_RESET, STATUS_INVALID_PARAM, &[], response);
        }

        // Response before reset
        let len = build_response(CMD_RESET, STATUS_OK, &[], response);

        // Small delay to ensure response is sent
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }

        // System reset
        cortex_m::peripheral::SCB::sys_reset();
    }
}

/// Check if address range is valid for APP
fn is_valid_app_address(address: u32, len: usize) -> bool {
    if address < APP_START {
        return false;
    }
    let end = address.checked_add(len as u32);
    match end {
        Some(e) => e <= APP_END,
        None => false,
    }
}

/// Get the page number for a given address
pub fn address_to_page(address: u32) -> u8 {
    // STM32G4 has 2KB pages, Flash starts at 0x0800_0000
    ((address - 0x0800_0000) / 2048) as u8
}

/// Get the start address of a page
pub fn page_to_address(page: u8) -> u32 {
    0x0800_0000 + (page as u32) * 2048
}