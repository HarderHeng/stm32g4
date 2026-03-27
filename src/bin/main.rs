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
use embassy_stm32::{bind_interrupts, peripherals, usart, wdg::IndependentWatchdog};
use embassy_stm32::usart::Config;
use embedded_io_async::Read;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32g4_foc::driver::{init_shell, get_shell_tx, ShellWriter, Shell};
use embassy_stm32g4_foc::driver::{MotorPwm, CurrentSenseAmp, CurrentSenseAdc};

/// APP start address (after bootloader 20KB + boot flag 2KB)
const APP_START: u32 = 0x0800_5800;

/// IWDG timeout: 5 seconds. Heartbeat feeds every 1 s, so 5x margin.
const IWDG_TIMEOUT_US: u32 = 5_000_000;

bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

#[embassy_executor::task]
async fn shell_task(mut rx: embassy_stm32::usart::BufferedUartRx<'static>) {
    // Create shell with the initialized TX
    let writer = ShellWriter::new(get_shell_tx());
    let mut shell = Shell::new(writer);

    shell.print_welcome();

    let mut buf = [0u8; 1];
    loop {
        match rx.read(&mut buf).await {
            Ok(_) => {
                shell.process(buf[0]);
            }
            Err(e) => {
                error!("RX error: {:?}", e);
            }
        }
    }
}

/// Heartbeat task: feeds the IWDG every second.
///
/// This task owns the watchdog. If the Embassy executor stalls (e.g. due to
/// a spinloop, panic, or runaway interrupt), this task won't run, the IWDG
/// will expire, and the bootloader will block the APP until new firmware is
/// flashed via OTA.
#[embassy_executor::task]
async fn watchdog_task(mut wdg: IndependentWatchdog<'static, peripherals::IWDG>) {
    let mut count = 0u32;
    loop {
        embassy_time::Timer::after_secs(1).await;
        count += 1;
        info!("heartbeat {}", count);
        wdg.pet();
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

    // Start IWDG before any other init. unleash() activates the countdown;
    // the watchdog_task will pet() it every second.
    let mut wdg = IndependentWatchdog::new(p.IWDG, IWDG_TIMEOUT_US);
    wdg.unleash();

    // 1. Configure UART with 921600 baud rate using BufferedUart
    let mut config = Config::default();
    config.baudrate = 921600;

    let rx = init_shell(p.USART2, p.PB3, p.PB4, Irqs, config);

    // 2. Initialize OPAMP for current sensing (must be before ADC)
    // OPAMP internal outputs are connected to ADC channels:
    // - OPAMP1 (PA1) -> ADC1_CH13 (U phase)
    // - OPAMP2 (PA7) -> ADC2_CH16 (V phase)
    // - OPAMP3 (PB0) -> ADC2_CH18 (W phase)
    static OPAMP: StaticCell<CurrentSenseAmp> = StaticCell::new();
    let _opamp = OPAMP.init(CurrentSenseAmp::new(
        p.OPAMP1, p.OPAMP2, p.OPAMP3,
        p.PA1, p.PA7, p.PB0,
    ));
    info!("OPAMP initialized (gain: {})", CurrentSenseAmp::gain());

    // 3. Initialize PWM for motor control
    // TIM1 generates 6-channel complementary PWM at 20kHz
    // Dead time is set to 800ns for L6387E gate driver
    static PWM: StaticCell<MotorPwm> = StaticCell::new();
    let _pwm = PWM.init(MotorPwm::new(
        p.TIM1,
        p.PA8, p.PC13,   // CH1 (U phase): PA8 high-side, PC13 low-side
        p.PA9, p.PA12,   // CH2 (V phase): PA9 high-side, PA12 low-side
        p.PA10, p.PB15,  // CH3 (W phase): PA10 high-side, PB15 low-side
    ));
    info!("PWM initialized (20kHz, 800ns dead time)");

    // 4. Initialize ADC for current sampling
    // ADC1 and ADC2 in dual injected mode for synchronized sampling
    static ADC: StaticCell<CurrentSenseAdc> = StaticCell::new();
    let _adc = ADC.init(CurrentSenseAdc::new(p.ADC1, p.ADC2));
    info!("ADC initialized (dual injected mode)");

    info!("All peripherals initialized successfully");

    // Spawn tasks
    spawner.spawn(unwrap!(shell_task(rx)));
    spawner.spawn(unwrap!(watchdog_task(wdg)));
}