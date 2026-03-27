//! Embassy STM32G4 FOC — Main Application
//!
//! # Architecture Layers
//! ```text
//! ┌─────────────────────────────────────────┐
//! │           Application                   │  ← This file
//! │  - Peripheral initialization            │
//! │  - Interrupt configuration              │
//! │  - Task scheduling                      │
//! ├─────────────────────────────────────────┤
//! │         FOC Algorithm Layer             │  ← foc/*
//! │  - Clarke/Park transforms               │
//! │  - PI controllers                       │
//! │  - SVPWM modulation                     │
//! │  - Motor state machine                  │
//! ├─────────────────────────────────────────┤
//! │       Driver Trait Layer                │  ← driver/traits.rs
//! │  - PwmOutput                            │
//! │  - CurrentSampler                       │
//! │  - BusMonitor                           │
//! ├─────────────────────────────────────────┤
//! │    Driver Implementation Layer          │  ← driver/*.rs
//! │  - MotorPwm (TIM1)                      │
//! │  - CurrentSenseAdc (ADC1/ADC2)          │
//! │  - CurrentSenseAmp (OPAMP1/2/3)         │
//! │  - OverCurrentProtection (COMP+DAC3)    │
//! ├─────────────────────────────────────────┤
//! │           BSP Layer                     │  ← bsp/*
//! │  - Clock configuration (170MHz)         │
//! │  - Power configuration                  │
//! └─────────────────────────────────────────┘
//! ```
//!
//! # Initialization Order
//!  1. `bsp::init()`             — clocks, power
//!  2. `OverCurrentProtection`   — COMP + DAC3 (protection first)
//!  3. `CurrentSenseAmp`         — OPAMP1/2/3 PGA
//!  4. `MotorPwm`                — TIM1 (CH1-4, RCR, dead time, break)
//!  5. `CurrentSenseAdc`         — ADC1/ADC2 injected mode
//!  6. `adc.calibrate()`         — zero-current offset
//!  7. `foc.init()`              — initialize FOC controller
//!  8. Enable TIM1_UP interrupt  — hardware-triggered 20kHz FOC loop
//!  9. Enable PVD low voltage detection
//! 10. Start UART + WDG tasks

#![no_std]
#![no_main]

use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, dma, peripherals, usart, wdg::IndependentWatchdog};
use embassy_stm32::usart::{Uart, UartRx, Config};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32g4_foc::driver::{init_shell_tx, get_shell_tx, ShellWriter, Shell};
use embassy_stm32g4_foc::driver::{
    OverCurrentProtection, OCP_THRESHOLD_DEFAULT,
    CurrentSenseAmp,
    MotorPwm,
    CurrentSenseAdc,
};
use embassy_stm32g4_foc::driver::traits::CurrentSampler;
use embassy_stm32g4_foc::foc::{FocController, FocMode};

/// APP start address (bootloader 20 KB + boot-flag 2 KB = 0x0800_5800)
const APP_START: u32 = 0x0800_5800;

/// IWDG timeout 5 s, heartbeat every 1 s → 5× margin
const IWDG_TIMEOUT_US: u32 = 5_000_000;

/// PWM frequency (must match MotorPwm configuration)
const PWM_FREQ_HZ: u32 = 20_000;

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

// ── Global FOC state (shared between tasks) ─────────────────────────────────
/// Global FOC controller wrapped in a Mutex for safe concurrent access
static FOC_CTRL: StaticCell<Mutex<CriticalSectionRawMutex, FocController<MotorPwm, CurrentSenseAdc>>> = StaticCell::new();

/// Signal to wake up FOC control task from TIM1_UP interrupt
static FOC_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, ()>> = StaticCell::new();

/// Get reference to FOC controller from any context
#[inline(always)]
fn with_foc_controller<F, R>(f: F) -> R
where
    F: FnOnce(&mut FocController<MotorPwm, CurrentSenseAdc>) -> R,
{
    embassy_futures::block_on(async {
        let mutex = unsafe {
            &*(&FOC_CTRL as *const StaticCell<Mutex<CriticalSectionRawMutex, FocController<MotorPwm, CurrentSenseAdc>>>
               as *const Mutex<CriticalSectionRawMutex, FocController<MotorPwm, CurrentSenseAdc>>)
        };
        let mut ctrl = mutex.lock().await;
        f(&mut *ctrl)
    })
}

/// FOC control task — triggered by TIM1_UP hardware interrupt via signal
#[embassy_executor::task]
async fn foc_interrupt_task() {
    use embassy_stm32::interrupt::{Interrupt, InterruptExt};

    // Get signal reference
    let signal = unsafe {
        &*(&FOC_SIGNAL as *const StaticCell<Signal<CriticalSectionRawMutex, ()>>
           as *const Signal<CriticalSectionRawMutex, ()>)
    };

    // Enable TIM1_UP interrupt in NVIC
    unsafe {
        let irq = Interrupt::TIM1_UP_TIM16;
        irq.set_priority(core::mem::transmute::<u8, embassy_stm32::interrupt::Priority>(0x80));
        irq.enable();
    }

    defmt::info!("TIM1_UP interrupt handler ready");

    // Main FOC loop - waits for signal from hardware interrupt
    loop {
        signal.wait().await;
        signal.reset();

        // Execute FOC control cycle
        with_foc_controller(|ctrl| {
            ctrl.control_cycle();
        });
    }
}

#[embassy_executor::task]
async fn shell_task(mut rx: UartRx<'static, Async>) {
    let writer = ShellWriter::new(get_shell_tx());
    let mut shell = Shell::new(writer);

    shell.print_welcome();

    let mut error_count: u8 = 0;
    let startup_grace_period = embassy_time::Instant::now();
    let mut buf = [0u8; 1];
    loop {
        match rx.read(&mut buf).await {
            Ok(()) => {
                error_count = 0;
                shell.process(buf[0]);
            }
            Err(e) => {
                error_count = error_count.saturating_add(1);
                let elapsed_ms = startup_grace_period.elapsed().as_millis();
                if elapsed_ms > 100 || error_count > 3 {
                    defmt::error!("RX error: {:?} (count: {})", e, error_count);
                }
            }
        }
    }
}

/// Heartbeat task: feeds the IWDG every second.
#[embassy_executor::task]
async fn watchdog_task(mut wdg: IndependentWatchdog<'static, peripherals::IWDG>) {
    let mut count = 0u32;
    loop {
        embassy_time::Timer::after_secs(1).await;
        count += 1;
        defmt::info!("heartbeat {}", count);
        wdg.pet();
    }
}

/// Motor control task — handles slow control loop (speed reference, status)
#[embassy_executor::task]
async fn motor_control_task() {
    embassy_time::Timer::after_millis(100).await;
    defmt::info!("Motor control task started");

    loop {
        embassy_time::Timer::after_millis(500).await;

        with_foc_controller(|ctrl| {
            if !ctrl.state().enabled {
                defmt::info!("Enabling motor");
                ctrl.enable();
            }

            let current_speed = ctrl.measured_speed();
            let target_speed = 100.0;

            if current_speed < target_speed {
                ctrl.set_speed_ref(current_speed + 10.0);
            }
        });

        embassy_time::Timer::after_millis(1000).await;

        let state = with_foc_controller(|ctrl| ctrl.state());
        let (i_d, i_q) = with_foc_controller(|ctrl| ctrl.measured_dq_currents());
        let omega = with_foc_controller(|ctrl| ctrl.measured_speed());

        defmt::info!("FOC state: enabled={}, spinning={}, fault={}", state.enabled, state.spinning, state.fault);
        defmt::info!("  Id={}A, Iq={}A, omega={} rad/s", defmt::Debug2Format(&i_d), defmt::Debug2Format(&i_q), defmt::Debug2Format(&omega));
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    unsafe {
        (*cortex_m::peripheral::SCB::PTR).vtor.write(APP_START);
    }

    let p = embassy_stm32g4_foc::bsp::init();
    defmt::info!("STM32G4 FOC — peripheral init");

    let mut wdg = IndependentWatchdog::new(p.IWDG, IWDG_TIMEOUT_US);
    wdg.unleash();

    // ── 1. UART ──────────────────────────────────────────────────────────────
    let mut uart_cfg = Config::default();
    uart_cfg.baudrate = 921600;
    let uart = Uart::new(p.USART2, p.PB4, p.PB3, p.DMA1_CH2, p.DMA1_CH1, Irqs, uart_cfg)
        .expect("UART init");
    let (tx, rx) = uart.split();
    init_shell_tx(tx);

    // ── 2. Over-current protection ───────────────────────────────────────────
    let _ocp = OverCurrentProtection::new(OCP_THRESHOLD_DEFAULT);
    defmt::info!("OCP initialised (DAC3 threshold: {} LSB)", OCP_THRESHOLD_DEFAULT);

    // ── 3. OPAMP current-sense amplifiers ────────────────────────────────────
    let _opamp = CurrentSenseAmp::new(
        p.OPAMP1, p.OPAMP2, p.OPAMP3,
        p.PA1, p.PA7, p.PB0,
        p.PA2, p.PA6, p.PB1,
    );
    defmt::info!("OPAMP initialised (gain x{}, effective ≈9.14)", CurrentSenseAmp::nominal_gain());

    // ── 4. TIM1 PWM ──────────────────────────────────────────────────────────
    let pwm = MotorPwm::new(
        p.TIM1,
        p.PA8,  p.PC13,
        p.PA9,  p.PA12,
        p.PA10, p.PB15,
        p.PA11,
    );
    defmt::info!("PWM initialised (20 kHz, 800 ns dead time, break from COMP1/2/4)");

    // ── 5. ADC current sensing ───────────────────────────────────────────────
    let adc = CurrentSenseAdc::new(p.ADC1, p.ADC2);
    defmt::info!("ADC initialised (R3_2 dual-injected mode)");

    // ── 6. Zero-current calibration ─────────────────────────────────────────
    let mut adc_calibrated = adc;
    adc_calibrated.calibrate();
    defmt::info!("ADC offset calibration complete: Ia={}, Ib={}, Ic={}",
          adc_calibrated.ia_offset(), adc_calibrated.ib_offset(), adc_calibrated.ic_offset());

    // ── 7. FOC Controller ────────────────────────────────────────────────────
    const POLE_PAIRS: u8 = 7;
    const RS: f32 = 2.55;
    const LS: f32 = 0.00086;
    const KE: f32 = 0.05;
    const MAX_CURRENT: f32 = 10.0;
    const VBUS_NOMINAL: f32 = 24.0;

    let foc = FocController::new(
        pwm,
        adc_calibrated,
        POLE_PAIRS,
        RS,
        LS,
        KE,
        MAX_CURRENT,
        VBUS_NOMINAL,
    );

    defmt::info!("FOC controller created");

    let foc_mutex = FOC_CTRL.init(Mutex::new(foc));

    {
        let mut ctrl = foc_mutex.lock().await;
        ctrl.init();
        ctrl.set_mode(FocMode::CurrentControl);
    }

    defmt::info!("FOC initialized (mode: current control, Id=0)");

    // ── 8. Enable TIM1_UP interrupt for hardware-triggered FOC loop ─────────
    {
        use embassy_stm32::interrupt::{Interrupt, InterruptExt};

        // Enable TIM1_UP interrupt in NVIC
        unsafe {
            let irq = Interrupt::TIM1_UP_TIM16;
            irq.set_priority(core::mem::transmute::<u8, embassy_stm32::interrupt::Priority>(0x80));
            irq.enable();
        }

        // Enable UPDATE interrupt in TIM1 DIER
        embassy_stm32::pac::TIM1.dier().modify(|w| w.set_uie(true));
    }

    defmt::info!("TIM1_UP interrupt enabled — hardware FOC loop at {} kHz", PWM_FREQ_HZ / 1000);

    // ── 9. Start tasks ───────────────────────────────────────────────────────
    spawner.spawn(unwrap!(foc_interrupt_task()));
    spawner.spawn(unwrap!(shell_task(rx)));
    spawner.spawn(unwrap!(watchdog_task(wdg)));
    spawner.spawn(unwrap!(motor_control_task()));

    defmt::info!("All tasks started");
}

// ── TIM1_UP Interrupt Handler ──────────────────────────────────────────────
/// This function is called by the hardware when TIM1 UPDATE interrupt fires.
/// It signals the FOC task to execute the control cycle.
///
/// # Safety Notes
/// - Uses `write(|w| w.set_uif(true))` because UIF is W1C (Write-1-to-Clear)
/// - Do NOT use `modify()` which could inadvertently clear other status bits
#[unsafe(no_mangle)]
fn tim1_up_irqhandler() {
    use embassy_stm32::pac;

    // Clear the UPDATE interrupt flag (W1C — write 1 to clear)
    // RM0440 §26.9.3: SR register bits are cleared by writing 1
    pac::TIM1.sr().write(|w| w.set_uif(true));

    // Signal the FOC task to run
    let signal = unsafe {
        &*(&FOC_SIGNAL as *const StaticCell<Signal<CriticalSectionRawMutex, ()>>
           as *const Signal<CriticalSectionRawMutex, ()>)
    };
    signal.signal(());
}
