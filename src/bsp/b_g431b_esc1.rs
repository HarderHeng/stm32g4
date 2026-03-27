//! B-G431B-ESC1 board-specific configuration
//!
//! This module contains all hardware-specific configuration
//! for the STMicroelectronics B-G431B-ESC1 motor control discovery board.
//!
//! # Hardware Overview
//!
//! - MCU: STM32G431CBUx (128KB Flash, 32KB RAM, 170MHz)
//! - Gate Driver: STSPIN32G4 (integrated gate driver + op-amps)
//! - Current Sensing: 3-shunt R3_2 topology
//! - OPAMPs: 3x integrated op-amps for current amplification
//! - Comparators: 3x comparators for hardware overcurrent protection
//!
//! # Pin Mapping
//!
//! ## TIM1 PWM Outputs (20kHz complementary)
//! | Pin  | Signal    | Function           |
//! |------|-----------|--------------------|
//! | PA8  | TIM1_CH1  | U high-side        |
//! | PC13 | TIM1_CH1N | U low-side         |
//! | PA9  | TIM1_CH2  | V high-side        |
//! | PA12 | TIM1_CH2N | V low-side         |
//! | PA10 | TIM1_CH3  | W high-side        |
//! | PB15 | TIM1_CH3N | W low-side         |
//! | PA11 | TIM1_CH4  | ADC trigger (no output) |
//!
//! ## Current Sensing (R3_2, 3-shunt)
//! | Shunt | OPAMP | ADC Channel | Gain |
//! |-------|-------|-------------|------|
//! | U     | OPAMP1| ADC1_CH3    | 9.14 |
//! | V     | OPAMP2| ADC2_CH3    | 9.14 |
//! | W     | OPAMP3| ADC1_CH12 / ADC2_CH18 | 9.14 |
//!
//! ## Protection
//! | Comparator | Positive Input | Negative Input | Trigger |
//! |------------|----------------|----------------|---------|
//! | COMP1      | OPAMP1 output  | DAC3_CH1       | TIM1_BRK|
//! | COMP2      | OPAMP2 output  | DAC3_CH2       | TIM1_BRK|
//! | COMP4      | OPAMP3 output  | DAC3_CH2       | TIM1_BRK|
//!
//! ## Analog Signals
//! | Pin   | Signal      | ADC Channel | Purpose        |
//! |-------|-------------|-------------|----------------|
//! | PA0   | VBUS_DIV    | ADC1_CH1    | Bus voltage    |
//! | PB14  | NTC_DIV     | ADC1_CH5    | Temperature    |

use crate::config::{Config, BoardConfig, MotorConfig, ControlConfig, ProtectionConfig, BorLevel};

/// Shunt resistance value (Ohms)
pub const R_SHUNT: f32 = 0.003;

/// OPAMP gain (actual, not register setting)
pub const OPAMP_GAIN: f32 = 9.14;

/// PWM period in timer ticks
/// 170MHz / (20kHz * 2) = 4250 (center-aligned mode)
pub const PWM_PERIOD: u16 = 4250;

/// Dead time in nanoseconds
pub const DEAD_TIME_NS: u32 = 847;

/// Default configuration for B-G431B-ESC1
pub fn default_config() -> Config {
    Config {
        board: BoardConfig {
            sysclk_hz: 170_000_000,
            pwm_frequency_hz: 20_000,
            adc_frequency_hz: 42_000_000,
            bor_level: BorLevel::Level3,
        },
        motor: MotorConfig::for_b_g431b_esc1_motor(),
        control: ControlConfig {
            current_loop_bandwidth_hz: 1000.0,
            speed_loop_bandwidth_hz: 200.0,
            dead_time_compensation_ns: DEAD_TIME_NS,
            field_weakening_enabled: false,
        },
        protection: ProtectionConfig::for_b_g431b_esc1(),
    }
}

/// Get board name string
pub const fn board_name() -> &'static str {
    "B-G431B-ESC1"
}

/// Get MCU name string
pub const fn mcu_name() -> &'static str {
    "STM32G431CBUx"
}
