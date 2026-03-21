#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

/// 1秒定时任务 - 展示 Embassy 异步能力
#[embassy_executor::task]
async fn timer_1s() {
    let mut count = 0u32;
    loop {
        Timer::after_secs(1).await;
        count += 1;
        info!("[1s定时器] 第 {} 次触发", count);
    }
}

/// 2秒定时任务 - 展示 Embassy 异步能力
#[embassy_executor::task]
async fn timer_2s() {
    let mut count = 0u32;
    loop {
        Timer::after_secs(2).await;
        count += 1;
        info!("[2s定时器] 第 {} 次触发", count);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(8),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            // Main system clock at 170 MHz: HSE(8MHz) / DIV2 * MUL85 / DIV2 = 170MHz
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
    }
    let _p = embassy_stm32::init(config);

    info!("Embassy 异步定时器示例启动!");
    info!("=====================================");
    info!("将同时运行两个定时器:");
    info!("  - 1秒定时器: 每1秒打印一次");
    info!("  - 2秒定时器: 每2秒打印一次");
    info!("=====================================");

    // spawn 两个任务，它们将并行执行
    spawner.spawn(unwrap!(timer_1s()));
    spawner.spawn(unwrap!(timer_2s()));
}
