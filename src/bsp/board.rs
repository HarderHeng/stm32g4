//! Board-level configuration for STM32G4
//!
//! Provides clock and power configuration for the STM32G431CB.

use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;

/// Board configuration structure
pub struct BoardConfig {
    /// Embassy peripheral configuration
    pub config: Config,
}

impl Default for BoardConfig {
    fn default() -> Self {
        let mut config = Config::default();

        // Configure HSE (External High-Speed Oscillator)
        // Using 8MHz external crystal
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(8),
            mode: HseMode::Oscillator,
        });

        // Configure PLL for 170MHz system clock
        // PLL: HSE(8MHz) / DIV2 * MUL85 / DIV2 = 170MHz
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });

        // Select PLL as system clock
        config.rcc.sys = Sysclk::PLL1_R;

        // Configure APB prescalers
        // STM32G4 specs: APB1 max 85MHz, APB2 max 170MHz
        // System clock is 170MHz, so APB1 needs DIV2 (85MHz), APB2 can be DIV1 (170MHz)
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;

        Self { config }
    }
}

/// Initialize the board with default configuration
///
/// Returns the Embassy peripherals for use by drivers.
///
/// # Example
///
/// ```ignore
/// let p = bsp::init();
/// // Use peripherals...
/// ```
pub fn init() -> embassy_stm32::Peripherals {
    init_with_config(BoardConfig::default())
}

/// Initialize the board with custom configuration
///
/// # Arguments
///
/// * `board_config` - Custom board configuration
///
/// # Returns
///
/// Embassy peripherals for use by drivers
pub fn init_with_config(board_config: BoardConfig) -> embassy_stm32::Peripherals {
    embassy_stm32::init(board_config.config)
}