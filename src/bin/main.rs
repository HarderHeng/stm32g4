//! Embassy STM32G4 Demo - Shell over UART with DMA

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, dma, peripherals, usart};
use embassy_stm32::usart::{Uart, UartTx, UartRx};
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32g4_foc::driver::{execute_command, print_welcome, Shell};

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

#[embassy_executor::task]
async fn shell_task(
    mut tx: UartTx<'static, embassy_stm32::mode::Async>,
    mut rx: UartRx<'static, embassy_stm32::mode::Async>,
) {
    let mut shell = Shell::new();
    let _ = print_welcome(&mut tx).await;

    let mut buf = [0u8; 1];
    loop {
        match rx.read(&mut buf).await {
            Ok(()) => {
                let byte = buf[0];
                if shell.process_byte(byte).is_some() {
                    tx.write(b"\r\n").await.ok();
                    execute_command(&mut tx, shell.line()).await.ok();
                    shell.clear();
                    tx.write(b"> ").await.ok();
                } else if byte == 0x7f || byte == b'\x08' {
                    shell.redraw(&mut tx).await.ok();
                }
            }
            Err(e) => error!("RX error: {:?}", e),
        }
    }
}

#[embassy_executor::task]
async fn heartbeat() {
    let mut count = 0u32;
    loop {
        embassy_time::Timer::after_secs(1).await;
        count += 1;
        info!("heartbeat {}", count);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32g4_foc::bsp::init();
    info!("STM32G4 Shell Demo");

    let uart = Uart::new(p.USART2, p.PB4, p.PB3, p.DMA1_CH2, p.DMA1_CH1, Irqs, Default::default())
        .expect("UART");
    let (tx, rx) = uart.split();

    spawner.spawn(unwrap!(shell_task(tx, rx)));
    spawner.spawn(unwrap!(heartbeat()));
}