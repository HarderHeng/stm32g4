//! Embassy STM32G4 FOC Demo - Motor control peripheral initialization
//!
//! This demo initializes:
//! - UART Shell (USART2)
//! - OPAMP for current sensing (OPAMP1, 2, 3)
//! - PWM for motor control (TIM1, 6-channel complementary)
//! - ADC for current sampling (ADC1, ADC2 dual mode)

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, dma, peripherals, usart};
use embassy_stm32::usart::{Uart, UartRx, Config};
use embassy_stm32::mode::Async;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32g4_foc::driver::{init_shell_tx, get_shell_tx, ShellWriter, Shell};
use embassy_stm32g4_foc::driver::{MotorPwm, CurrentSenseAmp, CurrentSenseAdc};

/// APP start address (after bootloader 16KB + boot flag 2KB)
const APP_START: u32 = 0x0800_8800;

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

    // Error tracking: ignore occasional errors during startup
    // but report if errors persist after initial period
    let mut error_count: u8 = 0;
    let startup_grace_period = embassy_time::Instant::now();

    let mut buf = [0u8; 1];
    loop {
        match rx.read(&mut buf).await {
            Ok(()) => {
                // Reset error count on successful read
                error_count = 0;
                shell.process(buf[0]);
            }
            Err(e) => {
                error_count = error_count.saturating_add(1);

                // Allow a few errors during startup grace period (first 100ms)
                // Otherwise report persistent errors
                let elapsed_ms = startup_grace_period.elapsed().as_millis();
                if elapsed_ms > 100 || error_count > 3 {
                    error!("RX error: {:?} (count: {})", e, error_count);
                }
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
    // Set VTOR to APP address (required when running from bootloader)
    unsafe {
        (*cortex_m::peripheral::SCB::PTR).vtor.write(APP_START);
    }

    let p = embassy_stm32g4_foc::bsp::init();
    info!("STM32G4 FOC Peripheral Init");

    // 1. Configure UART with 921600 baud rate
    let mut config = Config::default();
    config.baudrate = 921600;

    let uart = Uart::new(p.USART2, p.PB4, p.PB3, p.DMA1_CH2, p.DMA1_CH1, Irqs, config)
        .expect("UART");
    let (tx, rx) = uart.split();

    // Initialize TX storage before spawning shell task
    init_shell_tx(tx);

    // 2. Initialize OPAMP for current sensing (must be before ADC)
    // OPAMP internal outputs are connected to ADC channels:
    // - OPAMP1 (PA1) -> ADC1_CH13 (U phase)
    // - OPAMP2 (PA7) -> ADC2_CH16 (V phase)
    // - OPAMP3 (PB0) -> ADC2_CH18 (W phase)
    let _opamp = CurrentSenseAmp::new(
        p.OPAMP1, p.OPAMP2, p.OPAMP3,
        p.PA1, p.PA7, p.PB0,
    );
    info!("OPAMP initialized (gain: {})", CurrentSenseAmp::gain());

    // 3. Initialize PWM for motor control
    // TIM1 generates 6-channel complementary PWM at 20kHz
    // Dead time is set to 800ns for L6387E gate driver
    let _pwm = MotorPwm::new(
        p.TIM1,
        p.PA8, p.PC13,   // CH1 (U phase): PA8 high-side, PC13 low-side
        p.PA9, p.PA12,   // CH2 (V phase): PA9 high-side, PA12 low-side
        p.PA10, p.PB15,  // CH3 (W phase): PA10 high-side, PB15 low-side
    );
    info!("PWM initialized (20kHz, 800ns dead time)");

    // 4. Initialize ADC for current sampling
    // ADC1 and ADC2 in dual injected mode for synchronized sampling
    let _adc = CurrentSenseAdc::new(p.ADC1, p.ADC2);
    info!("ADC initialized (dual injected mode)");

    info!("All peripherals initialized successfully");

    // Spawn tasks
    spawner.spawn(unwrap!(shell_task(rx)));
    spawner.spawn(unwrap!(heartbeat()));
}