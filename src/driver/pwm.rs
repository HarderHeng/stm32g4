//! PWM driver for FOC motor control
//!
//! Provides 6-channel complementary PWM output using TIM1.
//! Configured for center-aligned mode with hardware dead time insertion.
//!
//! # Pin Mapping (B-G431B-ESC1)
//!
//! | Channel | Pin  | Function |
//! |---------|------|----------|
//! | CH1     | PA8  | U phase high-side |
//! | CH1N    | PC13 | U phase low-side |
//! | CH2     | PA9  | V phase high-side |
//! | CH2N    | PA12 | V phase low-side |
//! | CH3     | PA10 | W phase high-side |
//! | CH3N    | PB15 | W phase low-side |

use embassy_stm32::timer::complementary_pwm::ComplementaryPwm;
use embassy_stm32::timer::complementary_pwm::ComplementaryPwmPin;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::{Channel, Ch1, Ch2, Ch3};
use embassy_stm32::time::Hertz;
use embassy_stm32::gpio::OutputType;

/// PWM frequency in Hz (20kHz for motor control)
const PWM_FREQ_HZ: u32 = 20_000;

/// Dead time in nanoseconds (800ns typical for MOSFET gate drivers)
const DEAD_TIME_NS: u16 = 800;

/// PWM driver for 3-phase motor control
pub struct MotorPwm {
    pwm: ComplementaryPwm<'static, embassy_stm32::peripherals::TIM1>,
}

impl MotorPwm {
    /// Create a new PWM driver instance
    ///
    /// # Arguments
    ///
    /// * `tim1` - TIM1 peripheral
    /// * `pa8` - CH1 pin (U high-side)
    /// * `pc13` - CH1N pin (U low-side)
    /// * `pa9` - CH2 pin (V high-side)
    /// * `pa12` - CH2N pin (V low-side)
    /// * `pa10` - CH3 pin (W high-side)
    /// * `pb15` - CH3N pin (W low-side)
    ///
    /// # Returns
    ///
    /// A configured PWM driver with all channels enabled
    pub fn new(
        tim1: embassy_stm32::Peri<'static, embassy_stm32::peripherals::TIM1>,
        pa8: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA8>,
        pc13: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PC13>,
        pa9: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA9>,
        pa12: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA12>,
        pa10: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA10>,
        pb15: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PB15>,
    ) -> Self {
        // Configure PWM pins with push-pull output
        let ch1: PwmPin<'_, embassy_stm32::peripherals::TIM1, Ch1> =
            PwmPin::new(pa8, OutputType::PushPull);
        let ch1n: ComplementaryPwmPin<'_, embassy_stm32::peripherals::TIM1, Ch1> =
            ComplementaryPwmPin::new(pc13, OutputType::PushPull);

        let ch2: PwmPin<'_, embassy_stm32::peripherals::TIM1, Ch2> =
            PwmPin::new(pa9, OutputType::PushPull);
        let ch2n: ComplementaryPwmPin<'_, embassy_stm32::peripherals::TIM1, Ch2> =
            ComplementaryPwmPin::new(pa12, OutputType::PushPull);

        let ch3: PwmPin<'_, embassy_stm32::peripherals::TIM1, Ch3> =
            PwmPin::new(pa10, OutputType::PushPull);
        let ch3n: ComplementaryPwmPin<'_, embassy_stm32::peripherals::TIM1, Ch3> =
            ComplementaryPwmPin::new(pb15, OutputType::PushPull);

        // Create complementary PWM in center-aligned mode
        // Center-aligned mode is preferred for motor control as it reduces EMI
        let mut pwm = ComplementaryPwm::new(
            tim1,
            Some(ch1),
            Some(ch1n),
            Some(ch2),
            Some(ch2n),
            Some(ch3),
            Some(ch3n),
            None,  // CH4 not used
            None,  // CH4N not used
            Hertz::hz(PWM_FREQ_HZ),
            CountingMode::CenterAlignedUpInterrupts,
        );

        // Configure dead time
        pwm.set_dead_time(Self::calculate_dead_time_value(DEAD_TIME_NS) as u16);

        // Enable all channels
        pwm.enable(Channel::Ch1);
        pwm.enable(Channel::Ch2);
        pwm.enable(Channel::Ch3);

        // Set initial 50% duty cycle (motor stopped)
        let max_duty = pwm.get_max_duty();
        pwm.set_duty(Channel::Ch1, max_duty / 2);  // CH1 (U)
        pwm.set_duty(Channel::Ch2, max_duty / 2);  // CH2 (V)
        pwm.set_duty(Channel::Ch3, max_duty / 2);  // CH3 (W)

        Self { pwm }
    }

    /// Calculate dead time register value for TIM1
    ///
    /// For STM32G4 at 170MHz:
    /// - T_dts = 1 / 170MHz ≈ 5.88ns
    /// - DTG[7:0] encoding:
    ///   - If DTG[7] = 0: DT = DTG[6:0] * T_dts (max 136 * 5.88 = 800ns)
    ///   - If DTG[7] = 1 and DTG[6] = 0: DT = (64 + DTG[5:0]) * 2 * T_dts
    ///   - If DTG[7:6] = 10: DT = (32 + DTG[4:0]) * 8 * T_dts
    ///   - If DTG[7:6] = 11: DT = (32 + DTG[4:0]) * 16 * T_dts
    fn calculate_dead_time_value(dead_time_ns: u16) -> u8 {
        // T_dts = 1 / 170MHz ≈ 5.88ns
        const T_DTS_NS: f32 = 1000.0 / 170_000.0; // ~5.88ns

        let dt = dead_time_ns as f32;
        let dt_units = dt / T_DTS_NS;

        if dt_units <= 127.0 {
            // DTG[7] = 0: DT = DTG[6:0] * T_dts
            dt_units as u8
        } else if dt_units <= 254.0 {
            // DTG[7] = 1, DTG[6] = 0: DT = (64 + DTG[5:0]) * 2 * T_dts
            let value = ((dt / (2.0 * T_DTS_NS)) - 64.0) as u8;
            0x80 | (value & 0x3F)
        } else if dt_units <= 1008.0 {
            // DTG[7:6] = 10: DT = (32 + DTG[4:0]) * 8 * T_dts
            let value = ((dt / (8.0 * T_DTS_NS)) - 32.0) as u8;
            0xC0 | (value & 0x1F)
        } else {
            // DTG[7:6] = 11: DT = (32 + DTG[4:0]) * 16 * T_dts
            let value = ((dt / (16.0 * T_DTS_NS)) - 32.0) as u8;
            0xE0 | (value & 0x1F)
        }
    }

    /// Set duty cycle for all three phases
    ///
    /// # Arguments
    ///
    /// * `u` - U phase duty (0.0 - 1.0)
    /// * `v` - V phase duty (0.0 - 1.0)
    /// * `w` - W phase duty (0.0 - 1.0)
    pub fn set_duty(&mut self, u: f32, v: f32, w: f32) {
        let max_duty = self.pwm.get_max_duty() as f32;

        // Clamp duty cycles to valid range
        let u_duty = ((u.clamp(0.0, 1.0) * max_duty) as u32).min(max_duty as u32);
        let v_duty = ((v.clamp(0.0, 1.0) * max_duty) as u32).min(max_duty as u32);
        let w_duty = ((w.clamp(0.0, 1.0) * max_duty) as u32).min(max_duty as u32);

        self.pwm.set_duty(Channel::Ch1, u_duty);
        self.pwm.set_duty(Channel::Ch2, v_duty);
        self.pwm.set_duty(Channel::Ch3, w_duty);
    }

    /// Get the maximum duty cycle value
    pub fn get_max_duty(&self) -> u16 {
        self.pwm.get_max_duty() as u16
    }

    /// Enable PWM output
    pub fn enable(&mut self) {
        self.pwm.enable(Channel::Ch1);
        self.pwm.enable(Channel::Ch2);
        self.pwm.enable(Channel::Ch3);
    }

    /// Disable PWM output
    pub fn disable(&mut self) {
        self.pwm.disable(Channel::Ch1);
        self.pwm.disable(Channel::Ch2);
        self.pwm.disable(Channel::Ch3);
    }
}