//! Complete FOC motor control state machine
//!
//! # Architecture
//! This module integrates all FOC components:
//! - Clarke/Park transforms (transforms.rs)
//! - PI current controllers (pi.rs)
//! - SVPWM modulation (svpwm.rs)
//!
//! # Control Loop (20kHz interrupt)
//! 1. Read phase currents (Ia, Ib, Ic)
//! 2. Clarke transform: Ia,Ib,Ic → Iα,Iβ
//! 3. Park transform: Iα,Iβ,θ → Id,Iq
//! 4. PI controllers: (Id_ref-Id), (Iq_ref-Iq) → Vd,Vq
//! 5. Inverse Park: Vd,Vq,θ → Vα,Vβ
//! 6. SVPWM: Vα,Vβ → duty_u, duty_v, duty_w
//! 7. Update PWM registers
//! 8. Update ADC sector for next cycle
//!
//! # Layer Philosophy
//! - Uses Driver traits (PwmOutput, CurrentSampler) — no direct hardware access
//! - Pure state machine — caller triggers execution
//! - Configurable via MotorParams

use crate::driver::traits::{CurrentSampler, PwmOutput};
use super::transforms::{clarke_park, inv_park_transform, ElectricalAngle};
use super::pi::{PiController, DualCurrentController};
use super::svpwm::{SvpwmModulator, Sector, calculate_adc_trigger};

/// Minimum valid DC bus voltage for SVPWM modulation.
/// Below this value, division by Vbus would cause numerical instability.
const MIN_VBUS_VOLTAGE: f32 = 1.0;

/// FOC operating modes
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FocMode {
    /// Open-loop V/f control (no current feedback)
    OpenLoop,
    /// Closed-loop current control (Id=0, Iq controls torque)
    CurrentControl,
    /// Closed-loop speed control (cascaded speed → current loop)
    SpeedControl,
    /// Position control (cascaded position → speed → current)
    PositionControl,
}

/// Motor state flags
#[derive(Clone, Copy, Debug, Default)]
pub struct MotorState {
    /// Motor is enabled
    pub enabled: bool,
    /// Motor is actively spinning
    pub spinning: bool,
    /// Fault condition detected (OCP, UVLO, etc.)
    pub fault: bool,
    /// Calibration complete
    pub calibrated: bool,
}

/// FOC controller state
pub struct FocController<PWM: PwmOutput, ADC: CurrentSampler> {
    // ── Dependencies (driver layer) ─────────────────────────────────────────
    /// PWM output driver
    pwm: PWM,
    /// Current sampler
    adc: ADC,

    // ── Control parameters ──────────────────────────────────────────────────
    /// Operating mode
    mode: FocMode,
    /// Motor electrical parameters
    pole_pairs: u8,
    /// Stator resistance (Ω)
    rs: f32,
    #[allow(dead_code)]
    /// Stator inductance (H)
    ls: f32,
    /// Back-EMF constant (V·s/rad electrical)
    ke: f32,
    /// Maximum phase current (A)
    max_current: f32,
    /// DC bus voltage (V)
    vbus: f32,

    // ── FOC state variables ─────────────────────────────────────────────────
    /// Electrical angle (for Park transforms)
    electrical_angle: ElectricalAngle,
    /// Electrical angular velocity (rad/s)
    omega_e: f32,
    /// Last SVPWM sector
    last_sector: Sector,

    // ── Current references ──────────────────────────────────────────────────
    /// D-axis current reference (typically 0 for Id=0 control)
    id_ref: f32,
    /// Q-axis current reference (torque-producing)
    iq_ref: f32,
    /// Speed reference (rad/s electrical)
    omega_ref: f32,

    // ── Controllers ─────────────────────────────────────────────────────────
    /// Dual PI current controller (Id, Iq)
    current_ctrl: DualCurrentController,
    /// Speed PI controller (optional, for speed control mode)
    speed_ctrl: Option<PiController>,

    // ── State ───────────────────────────────────────────────────────────────
    /// Motor state flags
    state: MotorState,
    /// Last measured phase currents
    i_a: f32,
    i_b: f32,
    i_c: f32,
    /// Last measured dq currents
    i_d: f32,
    i_q: f32,
    /// Last voltage commands
    v_alpha: f32,
    v_beta: f32,
    /// SVPWM modulator (persistent to avoid stack allocation in control_cycle)
    svpwm: SvpwmModulator,
}

impl<PWM: PwmOutput, ADC: CurrentSampler> FocController<PWM, ADC> {
    /// Create new FOC controller
    ///
    /// # Arguments
    /// * `pwm` - PWM output driver
    /// * `adc` - Current sampler
    /// * `pole_pairs` - Motor pole pairs
    /// * `rs` - Stator resistance (Ω)
    /// * `ls` - Stator inductance (H)
    /// * `ke` - Back-EMF constant (V·s/rad)
    /// * `max_current` - Maximum phase current (A)
    /// * `vbus_nominal` - Nominal DC bus voltage (V)
    pub fn new(
        pwm: PWM,
        adc: ADC,
        pole_pairs: u8,
        rs: f32,
        ls: f32,
        ke: f32,
        max_current: f32,
        vbus_nominal: f32,
    ) -> Self {
        // Current loop bandwidth: typically 1-2 kHz for FOC
        let current_bandwidth = 1000.0;
        // Validate Vbus before using - prevent division by zero
        let vbus = if vbus_nominal >= MIN_VBUS_VOLTAGE {
            vbus_nominal
        } else {
            MIN_VBUS_VOLTAGE // Fallback to minimum valid voltage
        };
        let max_voltage = vbus / 1.732; // Vbus/sqrt(3)

        let current_ctrl = DualCurrentController::from_motor_params(
            current_bandwidth,
            rs,
            ls,
            ke,
            max_voltage,
        );

        let speed_ctrl = PiController::for_speed_loop(
            200.0, // Speed loop bandwidth (Hz)
            0.001, // Inertia (kg·m²) - placeholder
            pole_pairs,
            max_current,
        );

        // Pre-allocate SVPWM modulator to avoid stack allocation in control_cycle
        let svpwm = SvpwmModulator::new(PWM::period(), 0.577, vbus);

        Self {
            pwm,
            adc,
            mode: FocMode::CurrentControl,
            pole_pairs,
            rs,
            ls,
            ke,
            max_current,
            vbus,
            electrical_angle: ElectricalAngle::default(),
            omega_e: 0.0,
            last_sector: Sector::Sector0,
            id_ref: 0.0,
            iq_ref: 0.0,
            omega_ref: 0.0,
            current_ctrl,
            speed_ctrl: Some(speed_ctrl),
            state: MotorState::default(),
            i_a: 0.0,
            i_b: 0.0,
            i_c: 0.0,
            i_d: 0.0,
            i_q: 0.0,
            v_alpha: 0.0,
            v_beta: 0.0,
            svpwm,
        }
    }

    /// Enable motor (start PWM outputs)
    pub fn enable(&mut self) {
        self.pwm.enable_outputs();
        self.state.enabled = true;
        self.state.fault = false;
    }

    /// Disable motor (stop PWM outputs)
    pub fn disable(&mut self) {
        self.pwm.disable_outputs();
        self.state.enabled = false;
        self.state.spinning = false;
        self.current_ctrl.reset();
        if let Some(ref mut ctrl) = self.speed_ctrl {
            ctrl.reset();
        }
    }

    /// Get motor state
    pub fn state(&self) -> MotorState {
        self.state
    }

    /// Set FOC operating mode
    pub fn set_mode(&mut self, mode: FocMode) {
        self.mode = mode;
    }

    /// Set speed reference (rad/s electrical)
    pub fn set_speed_ref(&mut self, omega: f32) {
        self.omega_ref = omega.clamp(-self.max_omega(), self.max_omega());
    }

    /// Set torque reference (Iq current, A)
    pub fn set_torque_ref(&mut self, iq: f32) {
        self.iq_ref = iq.clamp(-self.max_current, self.max_current);
    }

    /// Set current references directly
    pub fn set_current_refs(&mut self, id: f32, iq: f32) {
        self.id_ref = id.clamp(-self.max_current, self.max_current);
        self.iq_ref = iq.clamp(-self.max_current, self.max_current);
    }

    /// Get measured speed (rad/s electrical)
    pub fn measured_speed(&self) -> f32 {
        self.omega_e
    }

    /// Get measured currents (A)
    pub fn measured_currents(&self) -> (f32, f32, f32) {
        (self.i_a, self.i_b, self.i_c)
    }

    /// Get dq currents (A)
    pub fn measured_dq_currents(&self) -> (f32, f32) {
        (self.i_d, self.i_q)
    }

    /// Maximum electrical angular velocity (rad/s)
    fn max_omega(&self) -> f32 {
        // Based on max speed and pole pairs
        2.0 * core::f32::consts::PI * 10000.0 / 60.0 * self.pole_pairs as f32
    }

    /// Execute one FOC control cycle
    ///
    /// Call this at the PWM frequency (20kHz) from the TIM1_UP interrupt.
    ///
    /// # Sequence
    /// 1. Sample currents from previous cycle
    /// 2. Clarke + Park transforms
    /// 3. PI current control
    /// 4. Inverse Park transform
    /// 5. SVPWM duty calculation
    /// 6. Update PWM registers
    /// 7. Program next ADC sector
    #[inline(always)]
    pub fn control_cycle(&mut self) {
        if !self.state.enabled {
            return;
        }

        // ── 1. Sample currents ──────────────────────────────────────────────
        let (i_a, i_b, i_c) = self.adc.read_phase_currents();
        self.i_a = i_a;
        self.i_b = i_b;
        self.i_c = i_c;

        // ── 2. Get electrical angle ─────────────────────────────────────────
        let (sin_theta, cos_theta) = self.electrical_angle.sin_cos();

        // ── 3. Clarke + Park transforms ─────────────────────────────────────
        let (i_d, i_q) = clarke_park(i_a, i_b, sin_theta, cos_theta);
        self.i_d = i_d;
        self.i_q = i_q;

        // ── 4. Current control (PI) ─────────────────────────────────────────
        let (v_d, v_q) = match self.mode {
            FocMode::OpenLoop => {
                // Open-loop: direct Vα, Vβ from speed reference
                self.control_open_loop()
            }
            FocMode::CurrentControl => {
                // Current control: Id=0, Iq from torque ref
                self.current_ctrl.update(self.id_ref, self.iq_ref, i_d, i_q, self.omega_e)
            }
            FocMode::SpeedControl => {
                // Speed control: outer speed loop generates Iq ref
                if let Some(ref mut speed_pi) = self.speed_ctrl {
                    let speed_error = self.omega_ref - self.omega_e;
                    self.iq_ref = speed_pi.update(speed_error);
                    self.iq_ref = self.iq_ref.clamp(-self.max_current, self.max_current);
                }
                self.current_ctrl.update(0.0, self.iq_ref, i_d, i_q, self.omega_e)
            }
            FocMode::PositionControl => {
                // TODO: Position control needs a position sensor
                (0.0, 0.0)
            }
        };

        // ── 5. Inverse Park transform ───────────────────────────────────────
        let (v_alpha, v_beta) = inv_park_transform(v_d, v_q, sin_theta, cos_theta);
        self.v_alpha = v_alpha;
        self.v_beta = v_beta;

        // ── 6. SVPWM modulation ─────────────────────────────────────────────
        // Use persistent svpwm member to avoid stack allocation
        let (duty_u, duty_v, duty_w) = self.svpwm.generate_duties(v_alpha, v_beta);
        self.last_sector = self.svpwm.last_sector();

        // ── 7. Update PWM ───────────────────────────────────────────────────
        self.pwm.set_duties_raw(duty_u, duty_v, duty_w);

        // ── 8. Program next ADC sector ──────────────────────────────────────
        self.adc.update_sector(self.last_sector as u8);

        // ── 9. Update ADC trigger position ──────────────────────────────────
        let trigger = calculate_adc_trigger(PWM::period(), self.last_sector);
        self.pwm.set_adc_trigger(trigger);

        // ── 10. Update electrical angle (open-loop or sensorless) ───────────
        self.update_electrical_angle();
    }

    /// Open-loop V/f control
    fn control_open_loop(&self) -> (f32, f32) {
        // V/f pattern: V = V0 + K * ω
        let v_mag = self.ke * self.omega_e.abs() + self.rs * 0.5;
        (0.0, v_mag.clamp(0.0, self.vbus / 1.732))
    }

    /// Update electrical angle estimator
    ///
    /// For sensorless control, integrate speed:
    /// θ = ∫ ω_e dt
    #[inline(always)]
    fn update_electrical_angle(&mut self) {
        match self.mode {
            FocMode::OpenLoop | FocMode::SpeedControl => {
                // Integrate reference speed (open-loop)
                let dt = 1.0 / 20000.0; // 20kHz PWM
                self.electrical_angle.add(self.omega_ref * dt);
                self.omega_e = self.omega_ref;
            }
            FocMode::CurrentControl => {
                // Use estimated speed from back-EMF (placeholder)
                // TODO: Implement sensorless observer
                self.omega_e = 0.0;
            }
            FocMode::PositionControl => {
                // TODO: Use position sensor
            }
        }
    }

    /// Get mutable reference to PWM driver (for interrupt configuration)
    pub fn pwm_mut(&mut self) -> &mut PWM {
        &mut self.pwm
    }

    /// Initialize FOC controller (call after construction)
    pub fn init(&mut self) {
        // Reset all state
        self.electrical_angle = ElectricalAngle::default();
        self.omega_e = 0.0;
        self.id_ref = 0.0;
        self.iq_ref = 0.0;
        self.omega_ref = 0.0;
        self.current_ctrl.reset();
        if let Some(ref mut ctrl) = self.speed_ctrl {
            ctrl.reset();
        }
        self.state = MotorState::default();
        self.state.calibrated = true;
    }
}

/// Simple open-loop ramp generator
pub struct RampGenerator {
    /// Current position (rad/s electrical)
    omega: f32,
    /// Target position
    target: f32,
    /// Acceleration (rad/s²)
    accel: f32,
}

impl RampGenerator {
    pub fn new(max_accel: f32) -> Self {
        Self {
            omega: 0.0,
            target: 0.0,
            accel: max_accel,
        }
    }

    /// Set target speed
    pub fn set_target(&mut self, target: f32) {
        self.target = target;
    }

    /// Update ramp (call at control frequency)
    /// Returns current ramped speed
    #[inline(always)]
    pub fn update(&mut self, dt: f32) -> f32 {
        let error = self.target - self.omega;

        if error > 0.0 {
            self.omega += self.accel * dt;
            if self.omega > self.target {
                self.omega = self.target;
            }
        } else if error < 0.0 {
            self.omega -= self.accel * dt;
            if self.omega < self.target {
                self.omega = self.target;
            }
        }

        self.omega
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mock driver implementations for testing
    struct MockPwm {
        period: u16,
    }

    impl PwmOutput for MockPwm {
        fn set_duties_raw(&mut self, _u: u16, _v: u16, _w: u16) {}
        fn set_adc_trigger(&mut self, _ccr4: u16) {}
        fn period() -> u16 { 4250 }
        fn enable_outputs(&mut self) {}
        fn disable_outputs(&mut self) {}
    }

    struct MockAdc;

    impl CurrentSampler for MockAdc {
        fn read_phase_currents(&mut self) -> (f32, f32, f32) { (0.0, 0.0, 0.0) }
        fn update_sector(&mut self, _sector: u8) {}
        fn calibrate(&mut self) {}
    }

    #[test]
    fn test_foc_controller_creation() {
        let pwm = MockPwm { period: 4250 };
        let adc = MockAdc;

        let controller = FocController::new(
            pwm,
            adc,
            7,      // pole_pairs
            0.32,   // rs
            0.00047, // ls
            0.05,   // ke
            10.0,   // max_current
            24.0,   // vbus
        );

        assert!(!controller.state.enabled);
        assert_eq!(controller.pole_pairs, 7);
    }

    #[test]
    fn test_ramp_generator() {
        let mut ramp = RampGenerator::new(1000.0);
        ramp.set_target(100.0);

        for _ in 0..10 {
            ramp.update(0.001);
        }

        assert!(ramp.omega > 0.0);
        assert!(ramp.omega <= 100.0);
    }
}
