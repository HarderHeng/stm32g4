//! Board-level configuration for STM32G4
//!
//! Provides clock and power configuration for the STM32G431CB.

use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_stm32::pac::rcc::vals::Adcsel;

/// BOR (Brown-out Reset) threshold levels
///
/// STM32G4 supports programmable BOR levels:
/// - BOR Level 1: ~2.0V (lowest protection)
/// - BOR Level 2: ~2.2V
/// - BOR Level 3: ~2.4V (default, recommended for 3.3V systems)
/// - BOR Level 4: ~2.8V (highest protection)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BorLevel {
    /// BOR Level 1 - Reset at ~2.0V
    Level1,
    /// BOR Level 2 - Reset at ~2.2V
    Level2,
    /// BOR Level 3 - Reset at ~2.4V (default)
    Level3,
    /// BOR Level 4 - Reset at ~2.8V
    Level4,
}

impl Default for BorLevel {
    fn default() -> Self {
        // Level 3 (2.4V) is recommended for 3.3V systems
        // Provides good protection while allowing normal operation
        Self::Level3
    }
}

/// Board configuration structure
pub struct BoardConfig {
    /// Embassy peripheral configuration
    pub config: Config,
    /// BOR threshold level
    pub bor_level: BorLevel,
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
        // PLL P output: 8MHz / 2 * 85 / 17 = 20MHz (for ADC, max 60MHz)
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL85,
            divp: Some(PllPDiv::DIV17), // ADC clock: 20MHz
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

        // Configure ADC12 clock mux to use PLL1_P
        config.rcc.mux.adc12sel = Adcsel::PLL1_P;

        Self {
            config,
            bor_level: BorLevel::default(),
        }
    }
}

/// Configure BOR (Brown-out Reset) threshold
///
/// BOR provides hardware reset when VDD drops below the threshold.
/// This is configured via option bytes. For runtime low-voltage detection,
/// we use PVD (Programmable Voltage Detector) which provides early warning
/// via software-configurable thresholds.
///
/// # Arguments
/// * `level` - BOR threshold level
pub fn configure_bor(level: BorLevel) {
    // Configure PVD which provides interrupt-based low voltage detection
    configure_pvd(level);
}

/// Configure PVD (Programmable Voltage Detector)
///
/// PVD generates an interrupt when VDD drops below the threshold.
/// Unlike BOR, PVD does not reset the MCU - it provides early warning.
///
/// # Arguments
/// * `level` - PVD threshold level
fn configure_pvd(level: BorLevel) {
    use embassy_stm32::pac;

    // PVD threshold encoding for PWR CR2 register (STM32G4):
    // 000: 1.7V, 001: 2.0V, 010: 2.2V, 011: 2.5V
    // 100: 2.8V, 101: 2.9V, 110: 3.0V, 111: 3.3V
    let pvd_level = match level {
        BorLevel::Level1 => 0b001, // 2.0V
        BorLevel::Level2 => 0b010, // 2.2V
        BorLevel::Level3 => 0b011, // 2.5V
        BorLevel::Level4 => 0b100, // 2.8V
    };

    // Configure PVD level and enable
    pac::PWR.cr2().modify(|w| {
        w.set_pls(pvd_level);
        w.set_pvde(true);
    });

    // PVD interrupt (EXTI line 16) configuration would go here
    // but is omitted for brevity - the PVD flag can be polled
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
    // Configure BOR/PVD before initializing peripherals
    configure_bor(board_config.bor_level);

    embassy_stm32::init(board_config.config)
}
