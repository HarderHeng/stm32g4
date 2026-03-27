//! Clarke and Park transforms for FOC
//!
//! # Mathematical Foundation
//!
//! ## Clarke Transform (abc → αβ)
//! Converts 3-phase stationary currents to 2-phase stationary frame:
//! ```text
//! i_α = i_a
//! i_β = (i_a + 2*i_b) / √3   [using i_a + i_b + i_c = 0]
//! ```
//!
//! ## Park Transform (αβ → dq)
//! Converts stationary frame to rotating reference frame aligned with rotor flux:
//! ```text
//! i_d = i_α * cos(θ) + i_β * sin(θ)
//! i_q = -i_α * sin(θ) + i_β * cos(θ)
//! ```
//!
//! ## Inverse Park Transform (dq → αβ)
//! ```text
//! v_α = v_d * cos(θ) - v_q * sin(θ)
//! v_β = v_d * sin(θ) + v_q * cos(θ)
//! ```
//!
//! # Layer Philosophy
//! Pure mathematical functions — no hardware dependencies.

use libm;

/// Clarke transform: 3-phase currents → αβ stationary frame
///
/// Uses the constraint i_a + i_b + i_c = 0 to simplify:
/// ```text
/// i_α = i_a
/// i_β = (i_a + 2*i_b) / √3
/// ```
#[inline(always)]
pub fn clarke_transform(i_a: f32, i_b: f32, _i_c: f32) -> (f32, f32) {
    // 1/sqrt(3) ≈ 0.57735026919
    const INV_SQRT3: f32 = 0.57735027;

    let i_alpha = i_a;
    let i_beta = INV_SQRT3 * (i_a + 2.0 * i_b);

    (i_alpha, i_beta)
}

/// Inverse Clarke transform: αβ stationary frame → 3-phase currents
///
/// ```text
/// i_a = i_α
/// i_b = (-i_α + √3 * i_β) / 2
/// i_c = (-i_α - √3 * i_β) / 2
/// ```
#[inline(always)]
pub fn inv_clarke_transform(i_alpha: f32, i_beta: f32) -> (f32, f32, f32) {
    const SQRT3: f32 = 1.7320508;

    let i_a = i_alpha;
    let i_b = (-i_alpha + SQRT3 * i_beta) * 0.5;
    let i_c = (-i_alpha - SQRT3 * i_beta) * 0.5;

    (i_a, i_b, i_c)
}

/// Park transform: αβ stationary frame → dq rotating reference frame
///
/// # Arguments
/// * `i_alpha`, `i_beta` — stationary frame currents
/// * `sin_theta`, `cos_theta` — pre-computed sin/cos of electrical angle
///
/// # Returns
/// (i_d, i_q) where:
/// - i_d = direct axis current (flux-producing)
/// - i_q = quadrature axis current (torque-producing)
#[inline(always)]
pub fn park_transform(i_alpha: f32, i_beta: f32, sin_theta: f32, cos_theta: f32) -> (f32, f32) {
    let i_d = i_alpha * cos_theta + i_beta * sin_theta;
    let i_q = -i_alpha * sin_theta + i_beta * cos_theta;

    (i_d, i_q)
}

/// Inverse Park transform: dq rotating frame → αβ stationary frame
///
/// # Arguments
/// * `v_d`, `v_q` — rotating frame voltage commands
/// * `sin_theta`, `cos_theta` — pre-computed sin/cos of electrical angle
///
/// # Returns
/// (v_alpha, v_beta) stationary frame voltages
#[inline(always)]
pub fn inv_park_transform(v_d: f32, v_q: f32, sin_theta: f32, cos_theta: f32) -> (f32, f32) {
    let v_alpha = v_d * cos_theta - v_q * sin_theta;
    let v_beta = v_d * sin_theta + v_q * cos_theta;

    (v_alpha, v_beta)
}

/// Combined Clarke + Park transform for efficiency
#[inline(always)]
pub fn clarke_park(i_a: f32, i_b: f32, sin_theta: f32, cos_theta: f32) -> (f32, f32) {
    let (i_alpha, i_beta) = clarke_transform(i_a, i_b, 0.0);
    park_transform(i_alpha, i_beta, sin_theta, cos_theta)
}

/// Electrical angle wrapper (0 to 2π mapped to 0 to F32)
///
/// Provides convenient angle normalization and sin/cos computation.
#[derive(Clone, Copy, Default)]
pub struct ElectricalAngle {
    /// Electrical angle in radians (normalized to [0, 2π))
    radians: f32,
}

/// Two PI constant for angle normalization
const TWO_PI: f32 = 6.283185307179586; // 2.0 * FRAC_PI_2 * 2.0

impl ElectricalAngle {
    /// Create new electrical angle (auto-normalized to [0, 2π))
    #[inline(always)]
    pub fn new(radians: f32) -> Self {
        // Use fmodf for efficient angle normalization instead of while loops
        let r = libm::fmodf(radians, TWO_PI);
        // Handle negative result from fmodf
        let r = if r < 0.0 { r + TWO_PI } else { r };
        Self { radians: r }
    }

    /// Create from electrical degrees (0-360)
    pub fn from_degrees(deg: f32) -> Self {
        Self::new(deg.to_radians())
    }

    /// Get sin/cos of this angle using libm
    #[inline(always)]
    pub fn sin_cos(&self) -> (f32, f32) {
        (libm::sinf(self.radians), libm::cosf(self.radians))
    }

    /// Get raw radians
    #[inline(always)]
    pub fn radians(&self) -> f32 {
        self.radians
    }

    /// Add electrical angle (auto-normalized)
    #[inline(always)]
    pub fn add(&mut self, delta: f32) {
        self.radians = Self::new(self.radians + delta).radians;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clarke_zero_sum() {
        let (i_alpha, i_beta) = clarke_transform(1.0, 0.0, -1.0);
        assert!((i_alpha - 1.0).abs() < 0.001);
        assert!((i_beta - 0.57735027).abs() < 0.001);
    }

    #[test]
    fn test_park_identity() {
        // At theta=0, dq should equal αβ
        let (i_d, i_q) = park_transform(1.0, 0.5, 0.0, 1.0);
        assert!((i_d - 1.0).abs() < 0.001);
        assert!((i_q - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_inv_park_roundtrip() {
        let v_d = 0.5;
        let v_q = 0.3;
        let (sin_t, cos_t) = (0.6, 0.8);

        let (v_alpha, v_beta) = inv_park_transform(v_d, v_q, sin_t, cos_t);
        let (v_d_back, v_q_back) = park_transform(v_alpha, v_beta, sin_t, cos_t);

        assert!((v_d_back - v_d).abs() < 0.001);
        assert!((v_q_back - v_q).abs() < 0.001);
    }

    #[test]
    fn test_electrical_angle_normalization() {
        let angle = ElectricalAngle::new(7.0); // > 2π
        assert!(angle.radians() >= 0.0);
        assert!(angle.radians() < TWO_PI);
    }
}
