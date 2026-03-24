# STM32G4 OTA Protocol

## Overview

Simple binary protocol for firmware updates over UART.

## Frame Format

```
| SOF (1) | CMD (1) | LEN (2, BE) | DATA (N) | CRC16 (2, BE) | EOF (1) |
```

- **SOF**: Start of frame, `0x7E`
- **CMD**: Command byte
- **LEN**: Data length (big-endian, max 256 bytes)
- **DATA**: Command data
- **CRC16**: CRC16-CCITT (MODBUS variant) of CMD + LEN + DATA
- **EOF**: End of frame, `0x7F`

## Commands

| Command | Code | Description |
|---------|------|-------------|
| ERASE | 0x01 | Erase flash pages |
| WRITE | 0x02 | Write data to flash |
| READ | 0x03 | Read data from flash |
| RESET | 0x04 | Reset and jump to APP |
| INFO | 0x05 | Get bootloader info |

Response has CMD | 0x80.

## Status Codes

| Status | Code | Description |
|--------|------|-------------|
| OK | 0x00 | Success |
| INVALID_CMD | 0x01 | Unknown command |
| INVALID_PARAM | 0x02 | Invalid parameters |
| FLASH_ERROR | 0x03 | Flash operation failed |
| CRC_ERROR | 0x04 | CRC mismatch |
| INVALID_FRAME | 0x05 | Frame parsing error |
| INVALID_ADDR | 0x06 | Address out of range |

## Memory Layout

```
Address         Size    Description
0x08000000      20KB    Bootloader
0x08005000      2KB     Boot Flag
0x08005800      106KB   APP Region
0x08020000      -       Flash End (128KB total)
```

## Command Details

### INFO (0x05)

Request bootloader info.

**Response Data (13 bytes)**:
- version_major (1 byte)
- version_minor (1 byte)
- app_start (4 bytes, BE)
- app_end (4 bytes, BE)
- page_size (2 bytes, BE)
- page_count (1 byte)

### ERASE (0x01)

Erase flash pages.

**Request Data**:
- page_count (1 byte): Number of pages to erase
- page_numbers (N bytes): Page numbers to erase

**Example**: Erase pages 11-20
```
Data: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
```

### WRITE (0x02)

Write data to flash. Data must be 8-byte aligned.

**Request Data**:
- address (4 bytes, BE): Flash address
- data (N bytes): Data to write

**Response Data**:
- address (4 bytes, BE)
- length (4 bytes, BE)

### READ (0x03)

Read data from flash.

**Request Data**:
- address (4 bytes, BE)
- length (2 bytes, BE)

**Response Data**:
- data (N bytes)
- crc16 (2 bytes, BE)

### RESET (0x04)

Reset and jump to APP.

**Request Data**:
- magic (4 bytes): `0xDEADBEEF` (required to prevent accidental reset)

## CRC16 Calculation

MODBUS variant (polynomial 0xA001):

```python
def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc
```

## Usage

### Python Tool

```bash
# Get bootloader info
python tools/ota_tool.py --port /dev/ttyUSB0 --info

# Flash firmware
python tools/ota_tool.py --port /dev/ttyUSB0 --firmware app.bin

# Flash and verify
python tools/ota_tool.py --port /dev/ttyUSB0 --firmware app.bin --verify

# Reset to APP
python tools/ota_tool.py --port /dev/ttyUSB0 --reset
```

### Manual Example (INFO command)

```
Request:
7E 05 00 00 21 16 7F

Response:
7E 85 00 0E 00 01 00 08 00 58 00 08 02 00 08 00 08 00 2A XX XX 7F
     ^  ^     ^
     |  |     +-- Data (version, addresses, page info)
     |  +-- Status (OK)
     +-- Response CMD (0x05 | 0x80)
```