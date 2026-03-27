//! STM32G4 HAL implementation
//!
//! Concrete implementation of the FOC HAL traits for STM32G4 hardware.

pub mod pwm;
pub mod adc;
pub mod voltage;
pub mod opamp;

pub use pwm::{Stm32G4Pwm, PWM_PERIOD};
pub use adc::Stm32G4Adc;
pub use voltage::Stm32G4Voltage;
pub use opamp::Stm32G4OpAmp;

use crate::hal::{FocHal, PwmOutput, CurrentSense, VoltageMonitor, MotorParameters, CalibrateError};

/// STM32G4 hardware abstraction layer combining all FOC interfaces.
///
/// This struct aggregates all hardware interfaces needed for field-oriented
/// control on STM32G4-based motor drivers like the B-G431B-ESC1.
///
/// # Example
///
/// ```ignore
/// let hal = Stm32G4Hal::new(
///     p.TIM1, p.PA8, p.PC13, p.PA9, p.PA12, p.PA10, p.PB15, p.PA11,
///     p.ADC1, p.ADC2, p.ADC1,
/// );
/// ```
pub struct Stm32G4Hal<MP: MotorParameters + Default> {
    /// PWM output for 3-phase motor control
    pub pwm: Stm32G4Pwm,
    /// Current sensing ADC
    pub adc: Stm32G4Adc,
    /// Bus voltage and temperature monitor
    pub voltage: Stm32G4Voltage,
    /// Motor parameters (phantom type)
    _motor_params: core::marker::PhantomData<MP>,
}

impl<MP: MotorParameters + Default> Stm32G4Hal<MP> {
    /// Create a new STM32G4 HAL instance
    ///
    /// # Arguments
    ///
    /// * `tim1` - TIM1 peripheral for PWM generation
    /// * `pa8` - TIM1_CH1 (U high-side)
    /// * `pc13` - TIM1_CH1N (U low-side)
    /// * `pa9` - TIM1_CH2 (V high-side)
    /// * `pa12` - TIM1_CH2N (V low-side)
    /// * `pa10` - TIM1_CH3 (W high-side)
    /// * `pb15` - TIM1_CH3N (W low-side)
    /// * `pa11` - TIM1_CH4 (ADC trigger)
    /// * `adc1_current` - ADC1 peripheral for current sensing
    /// * `adc2_current` - ADC2 peripheral for current sensing (slave)
    /// * `adc1_voltage` - ADC1 peripheral for voltage/temperature sensing
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        tim1: embassy_stm32::Peri<'static, embassy_stm32::peripherals::TIM1>,
        pa8: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA8>,
        pc13: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PC13>,
        pa9: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA9>,
        pa12: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA12>,
        pa10: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA10>,
        pb15: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PB15>,
        pa11: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA11>,
        adc1_current: embassy_stm32::Peri<'static, embassy_stm32::peripherals::ADC1>,
        adc2_current: embassy_stm32::Peri<'static, embassy_stm32::peripherals::ADC2>,
        adc1_voltage: embassy_stm32::Peri<'static, embassy_stm32::peripherals::ADC1>,
    ) -> Self {
        // Initialize OPAMPs first (required before ADC sampling)
        Stm32G4OpAmp::init_all(opamp::OpAmpGain::Gain16);

        let pwm = Stm32G4Pwm::new(tim1, pa8, pc13, pa9, pa12, pa10, pb15, pa11);
        let adc = Stm32G4Adc::new(adc1_current, adc2_current);
        let voltage = Stm32G4Voltage::new(adc1_voltage);

        Self {
            pwm,
            adc,
            voltage,
            _motor_params: core::marker::PhantomData,
        }
    }

    /// Initialize and calibrate the HAL
    ///
    /// Performs zero-current offset calibration.
    /// Must be called with PWM outputs disabled and motor stationary.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Calibration successful
    /// * `Err(CalibrateError)` - Calibration failed
    pub fn calibrate(&mut self) -> Result<(), CalibrateError> {
        self.adc.calibrate()
    }

    /// Enable all hardware outputs
    ///
    /// Call this after calibration to start PWM generation.
    pub fn enable_outputs(&mut self) {
        self.pwm.enable_outputs();
    }

    /// Disable all hardware outputs
    pub fn disable_outputs(&mut self) {
        self.pwm.disable_outputs();
    }

    /// Get motor parameters
    pub fn motor_params(&self) -> MP {
        MP::default()
    }
}

// Require MotorParameters to implement Default for Stm32G4Hal
impl<MP: MotorParameters + Default> Default for Stm32G4Hal<MP> {
    fn default() -> Self {
        // This is a placeholder - actual initialization requires peripherals
        // Use `new()` method instead
        panic!("Stm32G4Hal::default() is not supported - use Stm32G4Hal::new()")
    }
}

impl<MP: MotorParameters> PwmOutput for Stm32G4Hal<MP> {
    fn period(&self) -> u16 {
        self.pwm.period()
    }

    fn set_duties_raw(&mut self, u: u16, v: u16, w: u16) {
        self.pwm.set_duties_raw(u, v, w);
    }

    fn set_adc_trigger(&mut self, ccr: u16) {
        self.pwm.set_adc_trigger(ccr);
    }

    fn enable_outputs(&mut self) {
        self.pwm.enable_outputs();
    }

    fn disable_outputs(&mut self) {
        self.pwm.disable_outputs();
    }

    fn enable_update_interrupt(&mut self) {
        self.pwm.enable_update_interrupt();
    }

    fn disable_update_interrupt(&mut self) {
        self.pwm.disable_update_interrupt();
    }
}

impl<MP: MotorParameters> CurrentSense for Stm32G4Hal<MP> {
    fn read_currents(&mut self) -> (f32, f32, f32) {
        self.adc.read_currents()
    }

    fn update_sector(&mut self, sector: u8) {
        self.adc.update_sector(sector);
    }

    fn calibrate(&mut self) -> Result<(), CalibrateError> {
        self.adc.calibrate()
    }

    fn ia_offset(&self) -> u16 {
        self.adc.ia_offset()
    }

    fn ib_offset(&self) -> u16 {
        self.adc.ib_offset()
    }

    fn ic_offset(&self) -> u16 {
        self.adc.ic_offset()
    }
}

impl<MP: MotorParameters> VoltageMonitor for Stm32G4Hal<MP> {
    fn read_bus_voltage(&mut self) -> f32 {
        self.voltage.read_bus_voltage()
    }

    fn read_temperature(&mut self) -> f32 {
        self.voltage.read_temperature()
    }
}

impl<MP: MotorParameters + Default> FocHal for Stm32G4Hal<MP> {
    type MotorParams = MP;

    fn motor_params(&self) -> Self::MotorParams {
        MP::default()
    }

    fn pwm_mut(&mut self) -> &mut impl PwmOutput {
        &mut self.pwm
    }

    fn adc_mut(&mut self) -> &mut impl CurrentSense {
        &mut self.adc
    }

    fn voltage_mut(&mut self) -> &mut impl VoltageMonitor {
        &mut self.voltage
    }
}
