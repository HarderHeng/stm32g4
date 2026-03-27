//! Unified configuration system for FOC motor control
//!
//! This module provides a centralized configuration structure using the
//! Config pattern. All configuration parameters are defined here for
//! easy tuning and board-specific customization.

/// Top-level configuration structure
///
/// Groups all configuration parameters into a single structure
/// that can be passed to initialization functions.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Board-specific configuration
    pub board: BoardConfig,
    /// Motor configuration
    pub motor: MotorConfig,
    /// Control loop configuration
    pub control: ControlConfig,
    /// Protection configuration
    pub protection: ProtectionConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            board: BoardConfig::default(),
            motor: MotorConfig::default(),
            control: ControlConfig::default(),
            protection: ProtectionConfig::default(),
        }
    }
}

impl Config {
    /// Create configuration for B-G431B-ESC1 board with default motor
    pub fn for_b_g431b_esc1() -> Self {
        Self {
            board: BoardConfig::for_b_g431b_esc1(),
            motor: MotorConfig::for_b_g431b_esc1_motor(),
            control: ControlConfig::default(),
            protection: ProtectionConfig::default(),
        }
    }
}

/// Board-specific configuration
#[derive(Clone, Copy, Debug)]
pub struct BoardConfig {
    /// System clock frequency (Hz)
    pub sysclk_hz: u32,
    /// PWM frequency (Hz)
    pub pwm_frequency_hz: u32,
    /// ADC clock frequency (Hz)
    pub adc_frequency_hz: u32,
    /// BOR (Brown-out Reset) level
    pub bor_level: BorLevel,
}

impl Default for BoardConfig {
    fn default() -> Self {
        Self {
            sysclk_hz: 170_000_000,
            pwm_frequency_hz: 20_000,
            adc_frequency_hz: 42_000_000,
            bor_level: BorLevel::default(),
        }
    }
}

impl BoardConfig {
    /// Configuration for B-G431B-ESC1 discovery board
    pub fn for_b_g431b_esc1() -> Self {
        Self {
            sysclk_hz: 170_000_000,
            pwm_frequency_hz: 20_000,
            adc_frequency_hz: 42_000_000,
            bor_level: BorLevel::Level3,
        }
    }
}

/// BOR (Brown-out Reset) threshold levels
///
/// STM32G4 supports programmable BOR levels via option bytes.
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
        Self::Level3
    }
}

/// Motor electrical parameters configuration
#[derive(Clone, Copy, Debug)]
pub struct MotorConfig {
    /// Number of pole pairs
    pub pole_pairs: u8,
    /// Stator resistance (Ohms)
    pub rs: f32,
    /// Stator inductance (Henrys)
    pub ls: f32,
    /// Back-EMF constant (V·s/rad electrical)
    pub ke: f32,
    /// Maximum phase current (Amperes, peak)
    pub max_current: f32,
    /// Nominal bus voltage (Volts)
    pub vbus_nominal: f32,
}

impl Default for MotorConfig {
    fn default() -> Self {
        // Default values for typical NEMA 17 PMSM
        Self {
            pole_pairs: 7,
            rs: 2.55,
            ls: 0.000_86,
            ke: 0.01,
            max_current: 10.0,
            vbus_nominal: 24.0,
        }
    }
}

impl MotorConfig {
    /// Motor parameters for B-G431B-ESC1 reference motor
    ///
    /// This is the motor included with the ST Discovery Kit:
    /// - Pole pairs: 7
    /// - Stator resistance: 2.55 Ω
    /// - Stator inductance: 0.86 mH
    /// - Max current: 10A (conservative for continuous operation)
    /// - Nominal voltage: 24V
    pub fn for_b_g431b_esc1_motor() -> Self {
        Self {
            pole_pairs: 7,
            rs: 2.55,
            ls: 0.000_86,
            ke: 0.01, // TODO: measure from back-EMF
            max_current: 10.0, // Conservative for continuous operation
            vbus_nominal: 24.0,
        }
    }

    /// Calculate torque constant from Ke
    ///
    /// For 3-phase PMSM with sinusoidal FOC:
    /// `Kt = 1.5 * Ke` (N·m/A peak)
    #[inline(always)]
    pub fn torque_constant(&self) -> f32 {
        1.5 * self.ke
    }

    /// Calculate maximum torque at max current
    #[inline(always)]
    pub fn max_torque(&self) -> f32 {
        self.torque_constant() * self.max_current
    }

    /// Get number of poles (not pole pairs)
    #[inline(always)]
    pub fn num_poles(&self) -> u8 {
        self.pole_pairs * 2
    }
}

/// Control loop configuration
#[derive(Clone, Copy, Debug)]
pub struct ControlConfig {
    /// Current loop bandwidth (Hz)
    pub current_loop_bandwidth_hz: f32,
    /// Speed loop bandwidth (Hz)
    pub speed_loop_bandwidth_hz: f32,
    /// Dead time compensation (nanoseconds)
    pub dead_time_compensation_ns: u32,
    /// Use field weakening above base speed
    pub field_weakening_enabled: bool,
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            current_loop_bandwidth_hz: 1000.0,
            speed_loop_bandwidth_hz: 200.0,
            dead_time_compensation_ns: 800,
            field_weakening_enabled: false,
        }
    }
}

/// Protection configuration
#[derive(Clone, Copy, Debug)]
pub struct ProtectionConfig {
    /// Undervoltage lockout threshold (Volts)
    pub undervoltage_threshold: f32,
    /// Overvoltage lockout threshold (Volts)
    pub overvoltage_threshold: f32,
    /// Overtemperature threshold (Celsius)
    pub overtemperature_threshold: f32,
    /// Overcurrent protection threshold (Amperes, peak)
    pub ocp_threshold: f32,
    /// Hardware OCP threshold (set by DAC/COMP)
    pub hardware_ocp_threshold: f32,
}

impl Default for ProtectionConfig {
    fn default() -> Self {
        Self {
            undervoltage_threshold: 10.0,
            overvoltage_threshold: 30.0,
            overtemperature_threshold: 70.0,
            ocp_threshold: 30.0,
            hardware_ocp_threshold: 35.0,
        }
    }
}

impl ProtectionConfig {
    /// Protection thresholds for B-G431B-ESC1
    pub fn for_b_g431b_esc1() -> Self {
        Self {
            undervoltage_threshold: 10.0, // 10V UVLO
            overvoltage_threshold: 30.0,  // 30V OVLO
            overtemperature_threshold: 70.0, // 70°C thermal shutdown
            ocp_threshold: 30.0, // Software OCP at 30A
            hardware_ocp_threshold: 35.0, // Hardware OCP slightly higher
        }
    }
}
