//! Embassy STM32G4 Demo - Shell over UART with DMA using embedded-cli

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, dma, peripherals, usart};
use embassy_stm32::usart::{Uart, UartRx, Config};
use embassy_stm32::mode::Async;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32g4_foc::driver::{init_shell_tx, get_shell_tx, ShellWriter, Shell};

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

#[embassy_executor::task]
async fn shell_task(mut rx: UartRx<'static, Async>) {
    // Create shell with the initialized TX
    let writer = ShellWriter::new(get_shell_tx());
    let mut shell = Shell::new(writer);

    shell.print_welcome();

    let mut buf = [0u8; 1];
    loop {
        match rx.read(&mut buf).await {
            Ok(()) => shell.process(buf[0]),
            Err(_) => {
                // Ignore RX errors (frame error, overrun, etc.)
                // These can occur during startup or connection
            }
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
    info!("STM32G4 Shell (embedded-cli)");

    // Configure UART with 921600 baud rate
    let mut config = Config::default();
    config.baudrate = 921600;

    let uart = Uart::new(p.USART2, p.PB4, p.PB3, p.DMA1_CH2, p.DMA1_CH1, Irqs, config)
        .expect("UART");
    let (tx, rx) = uart.split();

    // Initialize TX storage before spawning shell task
    init_shell_tx(tx);

    spawner.spawn(unwrap!(shell_task(rx)));
    spawner.spawn(unwrap!(heartbeat()));
}