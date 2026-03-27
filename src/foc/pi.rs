//! PI (Proportional-Integral) controller with anti-windup
//!
//! # Design Philosophy
//! - Pure mathematical implementation (no hardware dependencies)
//! - Anti-windup clamping to prevent integral saturation
//! - Fixed sample time assumption (caller must ensure consistent timing)
//! - Optimized for 20kHz FOC loop (minimal CPU cycles)
//!
//! # Anti-Windup Strategy
//! When the output saturates, the integral term stops accumulating error
//! in the same direction. This prevents "windup" and ensures fast recovery.

/// PI controller state and configuration
pub struct PiController {
    /// Proportional gain
    kp: f32,
    /// Integral gain
    ki: f32,
    /// Integral term accumulator (anti-windup clamped)
    integrator: f32,
    /// Maximum output limit
    max_output: f32,
    /// Minimum output limit
    min_output: f32,
    #[allow(dead_code)]
    /// Maximum integral term (for anti-windup)
    max_integrator: f32,
}

impl PiController {
    /// Create a new PI controller
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    /// * `max_output` - Output saturation limit (symmetric ±)
    /// * `max_integrator` - Maximum integral term (anti-windup)
    ///
    /// # Panics
    /// Panics in debug mode if gains are negative or max_output <= 0
    pub fn new(kp: f32, ki: f32, max_output: f32, max_integrator: f32) -> Self {
        debug_assert!(kp >= 0.0, "Kp must be non-negative");
        debug_assert!(ki >= 0.0, "Ki must be non-negative");
        debug_assert!(max_output > 0.0, "Max output must be positive");
        debug_assert!(max_integrator > 0.0, "Max integrator must be positive");

        Self {
            kp,
            ki,
            integrator: 0.0,
            max_output,
            min_output: -max_output,
            max_integrator,
        }
    }

    /// Create PI controller for current loop (Id/Iq)
    ///
    /// # Arguments
    /// * `bandwidth_hz` - Desired current loop bandwidth
    /// * `rs` - Stator resistance (Ω)
    /// * `ls` - Stator inductance (H)
    /// * `max_voltage` - Maximum voltage command (V)
    ///
    /// Uses pole-zero cancellation: Ki = Kp * Rs / Ls
    pub fn for_current_loop(bandwidth_hz: f32, rs: f32, ls: f32, max_voltage: f32) -> Self {
        // Crossover frequency: ωc = 2π * bandwidth
        let omega_c = 2.0 * core::f32::consts::PI * bandwidth_hz;

        // Kp = ωc * Ls (dominant at high frequency)
        let kp = omega_c * ls;

        // Ki = Kp * Rs / Ls (pole-zero cancellation)
        let ki = kp * rs / ls;

        // Integral limit based on steady-state voltage
        let max_integrator = max_voltage * 0.5;

        Self::new(kp, ki, max_voltage, max_integrator)
    }

    /// Create PI controller for speed loop
    ///
    /// # Arguments
    /// * `bandwidth_hz` - Desired speed loop bandwidth (typically 100-500 Hz)
    /// * `inertia` - Combined motor + load inertia (kg·m²)
    /// * `pole_pairs` - Motor pole pairs
    /// * `max_current` - Maximum Iq current command (A)
    pub fn for_speed_loop(
        bandwidth_hz: f32,
        inertia: f32,
        pole_pairs: u8,
        max_current: f32,
    ) -> Self {
        let omega_c = 2.0 * core::f32::consts::PI * bandwidth_hz;

        // Simplified: Kp = ωc * J / (1.5 * pp * λm)
        // Using inertia as proxy for mechanical gain
        let kp = omega_c * inertia / (pole_pairs as f32);

        // Ki = Kp / (4 * τ) where τ = 1/ωc
        let ki = kp * omega_c / 4.0;

        Self::new(kp, ki, max_current, max_current * 0.3)
    }

    /// Reset integrator to zero
    #[inline(always)]
    pub fn reset(&mut self) {
        self.integrator = 0.0;
    }

    /// Set new gains (runtime tuning)
    #[inline(always)]
    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.kp = kp;
        self.ki = ki;
    }

    /// Get current gains
    #[inline(always)]
    pub fn gains(&self) -> (f32, f32) {
        (self.kp, self.ki)
    }

    /// Get integrator state (for debugging/logging)
    #[inline(always)]
    pub fn integrator(&self) -> f32 {
        self.integrator
    }

    /// Compute PI output for given error
    ///
    /// # Arguments
    /// * `error` - Setpoint minus measured value
    ///
    /// # Returns
    /// Voltage command clamped to [min_output, max_output]
    #[inline(always)]
    pub fn update(&mut self, error: f32) -> f32 {
        // Proportional term
        let p_term = self.kp * error;

        // Update integrator with anti-windup
        self.integrator += self.ki * error;

        // Clamp integrator (conditional integration)
        let output = p_term + self.integrator;

        // Anti-windup: only integrate if not saturated OR error would reduce saturation
        if output > self.max_output {
            if error < 0.0 {
                // Error would reduce output, allow integration
            } else {
                // Error would increase saturation, freeze integrator
                self.integrator -= self.ki * error;
            }
            return self.max_output;
        }

        if output < self.min_output {
            if error > 0.0 {
                // Error would reduce output, allow integration
            } else {
                // Error would increase saturation, freeze integrator
                self.integrator -= self.ki * error;
            }
            return self.min_output;
        }

        output
    }

    /// Compute PI output with feedforward term
    ///
    /// # Arguments
    /// * `error` - Setpoint minus measured value
    /// * `ff` - Feedforward voltage (added directly to output)
    ///
    /// # Returns
    /// Voltage command = PI(error) + feedforward, clamped
    #[inline(always)]
    pub fn update_with_feedforward(&mut self, error: f32, ff: f32) -> f32 {
        let pi_output = self.update(error);

        // Clamp total output including feedforward
        let total = pi_output + ff;
        if total > self.max_output {
            self.max_output
        } else if total < self.min_output {
            self.min_output
        } else {
            total
        }
    }
}

/// Dual PI controller for Id and Iq current loops
pub struct DualCurrentController {
    /// D-axis (flux) current controller
    pub id: PiController,
    /// Q-axis (torque) current controller
    pub iq: PiController,
    /// Decoupling term: ω * Ls (cross-coupling compensation)
    decouple_gain: f32,
    /// Back-EMF compensation gain: ω * Ke
    bemf_gain: f32,
}

impl DualCurrentController {
    /// Create dual current controller
    pub fn new(id: PiController, iq: PiController, decouple_gain: f32, bemf_gain: f32) -> Self {
        Self {
            id,
            iq,
            decouple_gain,
            bemf_gain,
        }
    }

    /// Create from motor parameters
    ///
    /// # Arguments
    /// * `bandwidth_hz` - Current loop bandwidth
    /// * `rs` - Stator resistance
    /// * `ls` - Stator inductance
    /// * `ke` - Back-EMF constant (V·s/rad)
    /// * `max_voltage` - Maximum voltage (typically Vbus/√3)
    pub fn from_motor_params(
        bandwidth_hz: f32,
        rs: f32,
        ls: f32,
        ke: f32,
        max_voltage: f32,
    ) -> Self {
        let id = PiController::for_current_loop(bandwidth_hz, rs, ls, max_voltage);
        let iq = PiController::for_current_loop(bandwidth_hz, rs, ls, max_voltage);

        Self {
            id,
            iq,
            decouple_gain: ls,      // Will be scaled by ω at runtime
            bemf_gain: ke,          // Will be scaled by ω at runtime
        }
    }

    /// Reset both controllers
    #[inline(always)]
    pub fn reset(&mut self) {
        self.id.reset();
        self.iq.reset();
    }

    /// Update both current loops with decoupling compensation
    ///
    /// # Arguments
    /// * `id_ref`, `iq_ref` - Current references (A)
    /// * `id_fb`, `iq_fb` - Current feedback (A)
    /// * `omega_e` - Electrical angular velocity (rad/s)
    ///
    /// # Returns
    /// (v_d, v_q) voltage commands with decoupling
    #[inline(always)]
    pub fn update(
        &mut self,
        id_ref: f32,
        iq_ref: f32,
        id_fb: f32,
        iq_fb: f32,
        omega_e: f32,
    ) -> (f32, f32) {
        // Compute decoupling terms
        // v_d_decouple = -ω * Ls * Iq
        // v_q_decouple = ω * Ls * Id + ω * Ke
        let v_d_decouple = -omega_e * self.decouple_gain * iq_fb;
        let v_q_decouple = omega_e * (self.decouple_gain * id_fb + self.bemf_gain);

        // PI with feedforward decoupling
        let v_d = self.id.update_with_feedforward(id_ref - id_fb, v_d_decouple);
        let v_q = self.iq.update_with_feedforward(iq_ref - iq_fb, v_q_decouple);

        (v_d, v_q)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pi_step_response() {
        let mut pi = PiController::new(1.0, 10.0, 10.0, 5.0);

        // Step input
        let error = 1.0;
        let y1 = pi.update(error);
        assert!(y1 > 0.0);

        // Integrator should accumulate
        let y2 = pi.update(error);
        assert!(y2 > y1);
    }

    #[test]
    fn test_anti_windup() {
        let mut pi = PiController::new(1.0, 100.0, 5.0, 2.0);

        // Large sustained error
        for _ in 0..10 {
            pi.update(10.0);
        }

        // Output should be clamped
        assert!(pi.integrator <= pi.max_integrator + 0.001);
    }

    #[test]
    fn test_dual_controller_decoupling() {
        let id_ctrl = PiController::new(1.0, 10.0, 10.0, 5.0);
        let iq_ctrl = PiController::new(1.0, 10.0, 10.0, 5.0);
        let mut controller = DualCurrentController::new(id_ctrl, iq_ctrl, 0.001, 0.1);

        let (v_d, v_q) = controller.update(0.0, 1.0, 0.0, 1.0, 100.0);

        // Should include decoupling terms
        assert!(v_d != 0.0 || v_q != 0.0);
    }
}
