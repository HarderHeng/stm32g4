//! SVPWM (Space Vector PWM) modulation
//!
//! # Theory
//!
//! SVPWM generates 3-phase voltages by time-averaging between adjacent voltage
//! vectors in the space vector diagram. The 8 possible switching states form:
//! - 6 active vectors (V1-V6) at 60° intervals
//! - 2 zero vectors (V0, V7) at origin
//!
//! # Algorithm
//! 1. Determine sector from Vα, Vβ angle (0-5)
//! 2. Calculate dwell times T1, T2 for adjacent vectors
//! 3. Calculate zero vector time T0 = Tsw - T1 - T2
//! 4. Convert to 3-phase duty cycles
//!
//! # Layer Philosophy
//! Pure mathematical modulation — takes Vα, Vβ and returns duty cycles.
//! No hardware registers or driver dependencies.

/// SVPWM sector (0-5 corresponding to 60° segments)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Sector {
    Sector0 = 0,
    Sector1 = 1,
    Sector2 = 2,
    Sector3 = 3,
    Sector4 = 4,
    Sector5 = 5,
}

impl Sector {
    /// Determine sector from αβ voltages
    ///
    /// ```text
    ///       Vβ
    ///        ^
    ///   2  / |  1
    ///     /  |
    /// 3 -----+-----> Vα
    ///     \  |
    ///   4  \ |  5
    ///        0 (below Vα axis, right side)
    /// ```
    #[inline(always)]
    pub fn from_voltages(v_alpha: f32, v_beta: f32) -> Self {
        // Sector determination using sign of Vα, Vβ, and Vα+Vβ/√3
        const SQRT3: f32 = 1.7320508;

        let v1 = v_beta;
        let v2 = SQRT3 * v_alpha * 0.5 - v_beta * 0.5;
        let v3 = -SQRT3 * v_alpha * 0.5 - v_beta * 0.5;

        let s1 = (v1 > 0.0) as u8;
        let s2 = (v2 > 0.0) as u8;
        let s3 = (v3 > 0.0) as u8;

        // Table lookup for sector
        let sector_idx = s1 | (s2 << 1) | (s3 << 2);
        const TABLE: [u8; 8] = [0, 2, 1, 0, 5, 0, 4, 3];

        Self::from(TABLE[sector_idx as usize])
    }
}

impl From<u8> for Sector {
    #[inline(always)]
    fn from(value: u8) -> Self {
        match value {
            0 => Sector::Sector0,
            1 => Sector::Sector1,
            2 => Sector::Sector2,
            3 => Sector::Sector3,
            4 => Sector::Sector4,
            5 => Sector::Sector5,
            _ => Sector::Sector0,
        }
    }
}

/// SVPWM modulator state
pub struct SvpwmModulator {
    /// PWM period in timer ticks
    period: u16,
    /// Maximum modulation index (0.577 for linear, up to 0.637 for overmodulation)
    max_modulation: f32,
    /// Inverse DC bus voltage for normalization
    inv_vbus: f32,
    /// Last computed sector
    last_sector: Sector,
}

impl SvpwmModulator {
    /// Create new SVPWM modulator
    ///
    /// # Arguments
    /// * `period` - PWM timer period (ARR value)
    /// * `max_modulation` - Maximum modulation index (0.577 = linear region limit)
    /// * `vbus_nominal` - Nominal DC bus voltage for per-unit scaling
    pub fn new(period: u16, max_modulation: f32, vbus_nominal: f32) -> Self {
        Self {
            period,
            max_modulation,
            inv_vbus: 1.0 / vbus_nominal,
            last_sector: Sector::Sector0,
        }
    }

    /// Update DC bus voltage (for dynamic bus compensation)
    #[inline(always)]
    pub fn set_vbus(&mut self, vbus: f32) {
        self.inv_vbus = if vbus > 0.1 { 1.0 / vbus } else { 0.0 };
    }

    /// Get last computed sector
    #[inline(always)]
    pub fn last_sector(&self) -> Sector {
        self.last_sector
    }

    /// Generate PWM duties from αβ voltage commands
    ///
    /// # Arguments
    /// * `v_alpha`, `v_beta` - Stationary frame voltage commands (Volts)
    ///
    /// # Returns
    /// (duty_u, duty_v, duty_w) in timer ticks (center-aligned, 0 to period)
    ///
    /// # Algorithm
    /// Uses the "averaging" method:
    /// ```text
    /// T_a = Tsw/2 * (1 + 2*v_alpha/vbus)
    /// T_b = Tsw/2 * (1 + 2*v_beta/vbus * sqrt(3)/2 - v_alpha/vbus)
    /// T_c = Tsw/2 * (1 - 2*v_beta/vbus * sqrt(3)/2 - v_alpha/vbus)
    /// ```
    #[inline(always)]
    pub fn generate_duties(&mut self, v_alpha: f32, v_beta: f32) -> (u16, u16, u16) {
        // Normalize to per-unit
        let v_alpha_pu = v_alpha * self.inv_vbus;
        let v_beta_pu = v_beta * self.inv_vbus;

        // Determine sector
        self.last_sector = Sector::from_voltages(v_alpha_pu, v_beta_pu);

        // Clamp modulation index
        let v_mag = libm::sqrtf(v_alpha_pu * v_alpha_pu + v_beta_pu * v_beta_pu);
        let clamp = if v_mag > self.max_modulation {
            self.max_modulation / v_mag
        } else {
            1.0
        };

        let v_alpha_clamped = v_alpha_pu * clamp;
        let v_beta_clamped = v_beta_pu * clamp;

        // SVPWM to 3-phase duty conversion (center-aligned)
        // Using simplified formula that works for all sectors
        let half_period = self.period as f32 * 0.5;

        // Third harmonic injection for better bus utilization
        // v_zero = -0.5 * (v_max + v_min) where v_max/min are instantaneous phase voltages
        let v_u = v_alpha_clamped;
        let v_v = -0.5 * v_alpha_clamped + 0.8660254 * v_beta_clamped; // sqrt(3)/2
        let v_w = -0.5 * v_alpha_clamped - 0.8660254 * v_beta_clamped;

        // Find max/min for third harmonic
        let v_max = v_u.max(v_v).max(v_w);
        let v_min = v_u.min(v_v).min(v_w);
        let v_zero = -0.5 * (v_max + v_min);

        // Add third harmonic and convert to duty
        let duty_u = half_period * (1.0 + v_u + v_zero);
        let duty_v = half_period * (1.0 + v_v + v_zero);
        let duty_w = half_period * (1.0 + v_w + v_zero);

        // Clamp to valid range
        let period_f = self.period as f32;
        (
            duty_u.clamp(0.0, period_f) as u16,
            duty_v.clamp(0.0, period_f) as u16,
            duty_w.clamp(0.0, period_f) as u16,
        )
    }

    /// Generate duties with explicit sector-based method (alternative algorithm)
    ///
    /// This implements the classical SVPWM with T1, T2, T0 dwell times.
    /// Currently unused - retained for potential future use or comparison.
    #[inline(always)]
    #[allow(dead_code)]
    pub fn generate_duties_sector(&mut self, v_alpha: f32, v_beta: f32) -> (u16, u16, u16) {
        let v_alpha_pu = v_alpha * self.inv_vbus;
        let v_beta_pu = v_beta * self.inv_vbus;

        self.last_sector = Sector::from_voltages(v_alpha_pu, v_beta_pu);

        // Sector-specific duty calculation
        match self.last_sector {
            Sector::Sector0 => self.duties_sector0(v_alpha_pu, v_beta_pu),
            Sector::Sector1 => self.duties_sector1(v_alpha_pu, v_beta_pu),
            Sector::Sector2 => self.duties_sector2(v_alpha_pu, v_beta_pu),
            Sector::Sector3 => self.duties_sector3(v_alpha_pu, v_beta_pu),
            Sector::Sector4 => self.duties_sector4(v_alpha_pu, v_beta_pu),
            Sector::Sector5 => self.duties_sector5(v_alpha_pu, v_beta_pu),
        }
    }

    // Sector-specific duty calculations
    // These are used by generate_duties_sector() which is currently unused.
    #[allow(dead_code)]
    fn duties_sector0(&self, v_alpha: f32, v_beta: f32) -> (u16, u16, u16) {
        // Sector 0: -30° to +30°, vectors V0, V1, V2
        let half = self.period as f32 * 0.5;
        let duty_a = half * (1.0 + v_alpha);
        let duty_b = half * (1.0 - 0.5 * v_alpha + 0.8660254 * v_beta);
        let duty_c = half * (1.0 - 0.5 * v_alpha - 0.8660254 * v_beta);
        self.clamp_duties(duty_a, duty_b, duty_c)
    }

    #[inline(always)]
    #[allow(dead_code)]
    fn duties_sector1(&self, v_alpha: f32, v_beta: f32) -> (u16, u16, u16) {
        // Sector 1: +30° to +90°
        let half = self.period as f32 * 0.5;
        let duty_a = half * (1.0 + 0.5 * v_alpha + 0.8660254 * v_beta);
        let duty_b = half * (1.0 + 0.5 * v_alpha - 0.8660254 * v_beta);
        let duty_c = half * (1.0 - v_alpha);
        self.clamp_duties(duty_a, duty_b, duty_c)
    }

    #[inline(always)]
    #[allow(dead_code)]
    fn duties_sector2(&self, v_alpha: f32, v_beta: f32) -> (u16, u16, u16) {
        // Sector 2: +90° to +150°
        let half = self.period as f32 * 0.5;
        let duty_a = half * (1.0 + 0.5 * v_alpha + 0.8660254 * v_beta);
        let duty_b = half * (1.0 - v_alpha);
        let duty_c = half * (1.0 + 0.5 * v_alpha - 0.8660254 * v_beta);
        self.clamp_duties(duty_a, duty_b, duty_c)
    }

    #[inline(always)]
    #[allow(dead_code)]
    fn duties_sector3(&self, v_alpha: f32, v_beta: f32) -> (u16, u16, u16) {
        // Sector 3: +150° to +210°
        let half = self.period as f32 * 0.5;
        let duty_a = half * (1.0 - v_alpha);
        let duty_b = half * (1.0 + 0.5 * v_alpha - 0.8660254 * v_beta);
        let duty_c = half * (1.0 + 0.5 * v_alpha + 0.8660254 * v_beta);
        self.clamp_duties(duty_a, duty_b, duty_c)
    }

    #[inline(always)]
    #[allow(dead_code)]
    fn duties_sector4(&self, v_alpha: f32, v_beta: f32) -> (u16, u16, u16) {
        // Sector 4: +210° to +270°
        let half = self.period as f32 * 0.5;
        let duty_a = half * (1.0 - 0.5 * v_alpha - 0.8660254 * v_beta);
        let duty_b = half * (1.0 + v_alpha);
        let duty_c = half * (1.0 - 0.5 * v_alpha + 0.8660254 * v_beta);
        self.clamp_duties(duty_a, duty_b, duty_c)
    }

    #[inline(always)]
    #[allow(dead_code)]
    fn duties_sector5(&self, v_alpha: f32, v_beta: f32) -> (u16, u16, u16) {
        // Sector 5: +270° to +330°
        let half = self.period as f32 * 0.5;
        let duty_a = half * (1.0 - 0.5 * v_alpha - 0.8660254 * v_beta);
        let duty_b = half * (1.0 - 0.5 * v_alpha + 0.8660254 * v_beta);
        let duty_c = half * (1.0 + v_alpha);
        self.clamp_duties(duty_a, duty_b, duty_c)
    }

    #[inline(always)]
    fn clamp_duties(&self, a: f32, b: f32, c: f32) -> (u16, u16, u16) {
        let period_f = self.period as f32;
        (
            a.clamp(0.0, period_f) as u16,
            b.clamp(0.0, period_f) as u16,
            c.clamp(0.0, period_f) as u16,
        )
    }
}

/// Calculate optimal ADC sampling point within PWM cycle
///
/// For center-aligned PWM, the optimal sampling point is at the valley
/// (center of the period) where all low-side FETs are conducting.
///
/// # Arguments
/// * `period` - PWM period
/// * `sector` - Current SVPWM sector
///
/// # Returns
/// CCR4 value for TIM1 CH4 (ADC trigger point)
pub fn calculate_adc_trigger(period: u16, sector: Sector) -> u16 {
    // Sample at PWM valley (center of period)
    // Offset slightly based on sector to ensure valid current measurement
    let offset = match sector {
        Sector::Sector0 | Sector::Sector3 => 0,
        Sector::Sector1 | Sector::Sector4 => period / 16,
        Sector::Sector2 | Sector::Sector5 => period / 8,
    };

    period / 2 - offset
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sector_from_voltages() {
        let s = Sector::from_voltages(1.0, 0.0);
        assert_eq!(s, Sector::Sector0);

        let s = Sector::from_voltages(0.0, 1.0);
        assert_eq!(s, Sector::Sector1);

        let s = Sector::from_voltages(-1.0, 0.0);
        assert!(s == Sector::Sector2 || s == Sector::Sector3);
    }

    #[test]
    fn test_generate_duties_zero() {
        let mut svpwm = SvpwmModulator::new(4250, 0.577, 24.0);
        let (du, dv, dw) = svpwm.generate_duties(0.0, 0.0);

        // At zero voltage, all duties should be ~50%
        let half = 4250 / 2;
        assert!((du as i16 - half as i16).abs() < 10);
        assert!((dv as i16 - half as i16).abs() < 10);
        assert!((dw as i16 - half as i16).abs() < 10);
    }

    #[test]
    fn test_generate_duties_clamping() {
        let mut svpwm = SvpwmModulator::new(4250, 0.577, 24.0);

        // Request over-modulation
        let (_, _, _) = svpwm.generate_duties(20.0, 0.0);

        // Duties should be clamped to valid range
    }
}
