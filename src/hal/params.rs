//! Motor parameters trait
//!
//! This trait defines the electrical parameters of the motor,
//! used by the FOC algorithm for control calculations.

/// Motor electrical parameters configuration.
///
/// This trait provides type-safe access to motor parameters needed
/// by the FOC algorithm. Parameters are defined as associated methods
/// for flexibility.
///
/// # Example
///
/// ```rust
/// struct MyMotorParams;
///
/// impl MotorParameters for MyMotorParams {
///     fn pole_pairs(&self) -> u8 { 7 }
///     fn rs(&self) -> f32 { 2.55 }
///     fn ls(&self) -> f32 { 0.00086 }
///     fn ke(&self) -> f32 { 0.01 }
///     fn max_current(&self) -> f32 { 60.0 }
///     fn vbus_nominal(&self) -> f32 { 24.0 }
/// }
/// ```
pub trait MotorParameters: Clone + Copy {
    /// Number of pole pairs in the motor.
    ///
    /// Used to convert between electrical and mechanical angles:
    /// `angle_electrical = angle_mechanical * pole_pairs`
    fn pole_pairs(&self) -> u8;

    /// Stator resistance in Ohms.
    ///
    /// Used for feed-forward compensation and flux weakening.
    /// Typically measured with a DC injection test.
    fn rs(&self) -> f32;

    /// Stator inductance in Henries.
    ///
    /// Used for current loop bandwidth calculation and feed-forward.
    /// Typically the Ld = Lq value for SPMSM, or Ld/Lq for IPMSM.
    fn ls(&self) -> f32;

    /// Back-EMF constant in V·s/rad (electrical).
    ///
    /// Also known as Ke. Related to torque constant Kt by:
    /// `Kt (N·m/A peak) = 1.5 * Ke (V·s/rad)` for 3-phase PMSM.
    ///
    /// Can be calculated from motor specs:
    /// `Ke = Vrms_line / (omega_mech * pole_pairs)`
    fn ke(&self) -> f32;

    /// Maximum continuous phase current in Amperes (peak, not RMS).
    ///
    /// This is the maximum current the motor can handle continuously
    /// without overheating. Peak currents may exceed this for short
    /// durations.
    fn max_current(&self) -> f32;

    /// Nominal bus voltage in Volts.
    ///
    /// The rated operating voltage. Used for per-unit calculations
    /// and voltage limiting.
    fn vbus_nominal(&self) -> f32;

    /// Calculate torque constant from Ke.
    ///
    /// For 3-phase PMSM with sinusoidal FOC:
    /// `Kt = 1.5 * Ke` (N·m/A peak)
    #[inline(always)]
    fn torque_constant(&self) -> f32 {
        1.5 * self.ke()
    }

    /// Calculate maximum torque at max current.
    ///
    /// `Tmax = Kt * Imax`
    #[inline(always)]
    fn max_torque(&self) -> f32 {
        self.torque_constant() * self.max_current()
    }

    /// Get number of poles (not pole pairs).
    #[inline(always)]
    fn num_poles(&self) -> u8 {
        self.pole_pairs() * 2
    }
}

/// Default motor parameters for B-G431B-ESC1 reference motor.
///
/// These are the parameters of the motor included with the ST Discovery Kit:
/// - Pole pairs: 7
/// - Stator resistance: 2.55 Ω
/// - Stator inductance: 0.86 mH
/// - Max current: 60A peak (limited by hardware)
/// - Nominal voltage: 24V
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DefaultMotorParams;

impl MotorParameters for DefaultMotorParams {
    fn pole_pairs(&self) -> u8 { 7 }
    fn rs(&self) -> f32 { 2.55 }
    fn ls(&self) -> f32 { 0.000_86 }
    fn ke(&self) -> f32 { 0.01 } // TODO: measure from back-EMF
    fn max_current(&self) -> f32 { 60.0 } // Hardware limited
    fn vbus_nominal(&self) -> f32 { 24.0 }
}
