//! Embassy STM32G4 Demo - BSP + Driver Architecture
//!
//! This example demonstrates:
//! - BSP layer for board initialization (clocks, GPIO)
//! - Driver layer for USART2 serial communication
//! - Simple echo functionality on USART2 (PB3/PB4)

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, peripherals, usart};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// Bind USART2 interrupt to the BufferedUart interrupt handler
bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

/// Echo task - receives data from USART2 and echoes it back
///
/// This demonstrates the split TX/RX capability for concurrent operation.
#[embassy_executor::task]
async fn echo_task(mut serial: embassy_stm32g4_foc::driver::Serial<'static>) {
    info!("Echo task started - waiting for data on USART2...");

    let mut buf = [0u8; 64];

    loop {
        match serial.read(&mut buf).await {
            Ok(len) if len > 0 => {
                // 打印接收到的原始字节（十六进制）
                for i in 0..len {
                    info!("Received byte: 0x{:02X} ('{}')", buf[i], buf[i] as char);
                }

                if let Err(e) = serial.write(&buf[..len]).await {
                    error!("Write error: {:?}", e);
                }
            }
            Ok(_) => {
                // Empty read, continue
            }
            Err(e) => {
                error!("Read error: {:?}", e);
            }
        }
    }
}

/// 1-second heartbeat task
#[embassy_executor::task]
async fn heartbeat() {
    let mut count = 0u32;
    loop {
        embassy_time::Timer::after_secs(1).await;
        count += 1;
        info!("[heartbeat] {} seconds elapsed", count);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize board using BSP layer
    let p = embassy_stm32g4_foc::bsp::init();

    info!("========================================");
    info!("STM32G4 BSP + Driver Demo");
    info!("========================================");
    info!("USART2 Configuration:");
    info!("  TX: PB3");
    info!("  RX: PB4");
    info!("  Baud: 115200");
    info!("========================================");

    // Set up USART2 with static buffers for BufferedUart
    static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();

    let tx_buf = TX_BUF.init([0u8; 256]);
    let rx_buf = RX_BUF.init([0u8; 256]);

    // Create serial driver instance
    let serial = embassy_stm32g4_foc::driver::Serial::new_usart2(
        p.USART2,
        p.PB4,  // RX
        p.PB3,  // TX
        tx_buf,
        rx_buf,
        Irqs,
    );

    // Spawn tasks
    spawner.spawn(echo_task(serial).unwrap());
    spawner.spawn(heartbeat().unwrap());
}