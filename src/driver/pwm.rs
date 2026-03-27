//! PWM driver for 3-phase FOC motor control
//!
//! Uses TIM1 in centre-aligned mode with complementary outputs on 3 phase channels
//! and CH4 as the ADC injection trigger.
//!
//! # Key configuration
//!
//! | Parameter | Value | Notes |
//! |-----------|-------|-------|
//! | Frequency | 20 kHz | ARR = 4250 @ 170 MHz ÷ 2 (centre) |
//! | Dead time | 800 ns | DTG ≈ 136 ticks @ 170 MHz |
//! | RCR | 1 | One UPDATE interrupt per full PWM cycle |
//! | CH4 | PWM2, CCR4 = period/4 | ADC injection trigger via CC4 event |
//! | Break | COMP1/2/4 via AF | Hardware over-current protection |
//!
//! # Pin mapping (B-G431B-ESC1)
//!
//! | Pin   | Signal   | Phase |
//! |-------|----------|-------|
//! | PA8   | TIM1_CH1  | U high-side |
//! | PC13  | TIM1_CH1N | U low-side  |
//! | PA9   | TIM1_CH2  | V high-side |
//! | PA12  | TIM1_CH2N | V low-side  |
//! | PA10  | TIM1_CH3  | W high-side |
//! | PB15  | TIM1_CH3N | W low-side  |
//! | PA11  | TIM1_CH4  | ADC trigger (no complementary) |

use embassy_stm32::timer::Channel;
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4};
use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::Hertz;
use embassy_stm32::pac;

use super::traits::PwmOutput;

/// Timer frequency (170 MHz, same as APB2 × 2 with no prescaler on TIM1).
const TIM1_CLK_HZ: u32 = 170_000_000;

/// Desired switching frequency
const PWM_FREQ_HZ: u32 = 20_000;

/// ARR = Fclk / (Fpwm * 2)  because centre-aligned mode counts up then down.
/// 170_000_000 / (20_000 * 2) = 4250
pub const PWM_PERIOD: u16 = (TIM1_CLK_HZ / (PWM_FREQ_HZ * 2)) as u16;

/// Dead-time raw value passed to Embassy `set_dead_time()`.
/// Embassy's `compute_dead_time_value()` encodes it per RM0440 §26.6.18 DTG[7:0].
/// 136 > 127 → bit7 is set → uses DTG[7:5]=100 formula: DT = (64 + DTG[5:0]) × 2 × t_DTS
/// = (64 + 8) × 2 / 170 MHz ≈ 847 ns  (closest achievable above 800 ns target)
const DEAD_TIME_RAW: u16 = 136;

/// PWM driver wrapping `ComplementaryPwm<TIM1>`.
pub struct MotorPwm {
    pwm: ComplementaryPwm<'static, embassy_stm32::peripherals::TIM1>,
}

impl MotorPwm {
    /// Initialise TIM1 for 3-phase complementary PWM + ADC trigger on CH4.
    ///
    /// # Arguments
    /// * Peripheral tokens for TIM1, the six phase pins, and PA11 (CH4 trigger).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        tim1: embassy_stm32::Peri<'static, embassy_stm32::peripherals::TIM1>,
        pa8:  embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA8>,
        pc13: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PC13>,
        pa9:  embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA9>,
        pa12: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA12>,
        pa10: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA10>,
        pb15: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PB15>,
        pa11: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA11>,
    ) -> Self {
        let ch1  = PwmPin::<_, Ch1>::new(pa8,  OutputType::PushPull);
        let ch1n = ComplementaryPwmPin::<_, Ch1>::new(pc13, OutputType::PushPull);
        let ch2  = PwmPin::<_, Ch2>::new(pa9,  OutputType::PushPull);
        let ch2n = ComplementaryPwmPin::<_, Ch2>::new(pa12, OutputType::PushPull);
        let ch3  = PwmPin::<_, Ch3>::new(pa10, OutputType::PushPull);
        let ch3n = ComplementaryPwmPin::<_, Ch3>::new(pb15, OutputType::PushPull);
        let ch4  = PwmPin::<_, Ch4>::new(pa11, OutputType::PushPull);

        let mut pwm = ComplementaryPwm::new(
            tim1,
            Some(ch1),
            Some(ch1n),
            Some(ch2),
            Some(ch2n),
            Some(ch3),
            Some(ch3n),
            Some(ch4),
            None, // CH4 has no complementary output
            Hertz::hz(PWM_FREQ_HZ),
            CountingMode::CenterAlignedUpInterrupts,
        );

        // RCR = 1: UPDATE interrupt fires once per full (up+down) PWM cycle.
        pwm.set_repetition_counter(1);

        // Dead time ≈ 800 ns @ 170 MHz.
        pwm.set_dead_time(DEAD_TIME_RAW);

        // CH4: PWM2 mode so CC4 fires on the down-count at CCR4.
        // CH4 is enabled but never drives a load — it only produces the CC4 event
        // that triggers ADC injected conversions.
        // ccmr_output(1) = CCMR2 (covers CH3 + CH4)
        // set_ocm(1, ...) sets the mode for CH4 (index 1 inside CCMR2)
        pac::TIM1.ccmr_output(1).modify(|w| {
            w.set_ocm(1, embassy_stm32::pac::timer::vals::Ocm::PWM_MODE2);
        });
        pwm.enable(Channel::Ch4);

        // Initial ADC trigger position: at PWM valley (center of period)
        // where all low-side FETs are conducting for valid current sampling.
        // CCR4 = period/4 = 1062 places the trigger at 25% of the cycle,
        // which aligns with the center of the down-count for center-aligned PWM.
        pwm.set_duty(Channel::Ch4, (PWM_PERIOD / 4) as u32);

        // Connect COMP1/2/4 outputs to TIM1 BRK (hardware OCP).
        // comp_index 0 = COMP1, 1 = COMP2, 3 = COMP4 (0-based).
        pwm.set_break_comparator_enable(0, true); // COMP1 → BRK
        pwm.set_break_comparator_enable(1, true); // COMP2 → BRK
        pwm.set_break_comparator_enable(3, true); // COMP4 → BRK

        // Enable break (BDTR.BKE). All COMP outputs active-high by default.
        pwm.set_break_enable(true);

        // Leave motor phases disabled at startup — caller must call enable_outputs()
        // after calibration is complete.

        // Initial 50 % duty on phases (used while outputs are disabled).
        let half = PWM_PERIOD / 2;
        pwm.set_duty(Channel::Ch1, half as u32);
        pwm.set_duty(Channel::Ch2, half as u32);
        pwm.set_duty(Channel::Ch3, half as u32);

        Self { pwm }
    }

    /// Direct access to the underlying Embassy PWM (for interrupt configuration etc.).
    pub fn inner_mut(&mut self) -> &mut ComplementaryPwm<'static, embassy_stm32::peripherals::TIM1> {
        &mut self.pwm
    }

    /// Enable TIM1 UPDATE interrupt for FOC control loop.
    ///
    /// Call this after PWM initialization to enable hardware-triggered
    /// 20kHz FOC execution. The interrupt will fire once per full PWM cycle
    /// (configured via RCR = 1).
    pub fn enable_update_interrupt(&mut self) {
        use embassy_stm32::interrupt::{Interrupt, InterruptExt};

        // Enable TIM1 UPDATE interrupt (TIM1_UP_TIM16 on STM32G4)
        unsafe {
            let irq = Interrupt::TIM1_UP_TIM16;
            irq.set_priority(into_priority(0x80));
            irq.enable();
        }

        // Enable UPDATE interrupt in TIM1 DIER register
        pac::TIM1.dier().modify(|w| w.set_uie(true));
    }

    /// Disable TIM1 UPDATE interrupt.
    pub fn disable_update_interrupt(&mut self) {
        use embassy_stm32::interrupt::{Interrupt, InterruptExt};

        // Disable UPDATE interrupt in TIM1 DIER register
        pac::TIM1.dier().modify(|w| w.set_uie(false));

        // Disable NVIC interrupt
        let irq = Interrupt::TIM1_UP_TIM16;
        irq.disable();
    }
}

/// Convert raw priority value to Priority type
fn into_priority(priority: u8) -> embassy_stm32::interrupt::Priority {
    unsafe { core::mem::transmute(priority) }
}

impl PwmOutput for MotorPwm {
    #[inline(always)]
    fn set_duties_raw(&mut self, u: u16, v: u16, w: u16) {
        self.pwm.set_duty(Channel::Ch1, u as u32);
        self.pwm.set_duty(Channel::Ch2, v as u32);
        self.pwm.set_duty(Channel::Ch3, w as u32);
    }

    #[inline(always)]
    fn set_adc_trigger(&mut self, ccr4: u16) {
        self.pwm.set_duty(Channel::Ch4, ccr4 as u32);
    }

    fn period() -> u16 {
        PWM_PERIOD
    }

    fn enable_outputs(&mut self) {
        self.pwm.enable(Channel::Ch1);
        self.pwm.enable(Channel::Ch2);
        self.pwm.enable(Channel::Ch3);
        self.pwm.set_master_output_enable(true);
    }

    fn disable_outputs(&mut self) {
        self.pwm.disable(Channel::Ch1);
        self.pwm.disable(Channel::Ch2);
        self.pwm.disable(Channel::Ch3);
        self.pwm.set_master_output_enable(false);
    }
}
