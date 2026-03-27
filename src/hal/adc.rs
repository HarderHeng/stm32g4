//! Current sensing trait for FOC motor control
//!
//! This trait abstracts the hardware-specific current sampling details,
//! providing a clean interface for reading phase currents.

/// Current sensing abstraction for FOC.
///
/// This trait provides the interface for reading motor phase currents
/// through ADC sampling. It supports the R3_2 (3-shunt) topology where
/// two phases are measured directly and the third is reconstructed.
///
/// # Implementation Notes
///
/// - Currents are returned in Amperes
/// - Zero-current offset calibration must be performed with PWM disabled
/// - The implementation handles the sector-based channel switching internally
pub trait CurrentSense {
    /// Read corrected phase currents from the previous PWM cycle's ADC result.
    ///
    /// Returns (Ia, Ib, Ic) in Amperes.
    ///
    /// # Implementation Notes
    ///
    /// This function should be called in the PWM update interrupt,
    /// after the ADC has completed its injected conversion.
    fn read_currents(&mut self) -> (f32, f32, f32);

    /// Update sector for next ADC sampling.
    ///
    /// Called inside the PWM update interrupt to pre-program the next ADC
    /// injected channel sequence for the given FOC sector (0-5).
    ///
    /// # Arguments
    ///
    /// * `sector` - FOC sector (0-5) for the upcoming PWM cycle
    fn update_sector(&mut self, sector: u8);

    /// Zero-current offset calibration.
    ///
    /// Must be called with PWM outputs disabled and motor stationary.
    /// Performs multiple injected conversions in software-trigger mode and
    /// averages the results to determine the per-channel ADC mid-point offsets.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Calibration successful
    /// * `Err(CalibrateError)` - Calibration failed
    fn calibrate(&mut self) -> Result<(), CalibrateError>;

    /// Get Ia offset (for debugging).
    ///
    /// Default implementation returns 0. Override if offset tracking is available.
    fn ia_offset(&self) -> u16 {
        0
    }

    /// Get Ib offset (for debugging).
    ///
    /// Default implementation returns 0. Override if offset tracking is available.
    fn ib_offset(&self) -> u16 {
        0
    }

    /// Get Ic offset (for debugging).
    ///
    /// Default implementation returns 0. Override if offset tracking is available.
    fn ic_offset(&self) -> u16 {
        0
    }

    /// Read Ia only (optimized path).
    ///
    /// Default implementation reads all currents and returns Ia.
    /// Implementations may override for more efficient single-phase reads.
    fn read_ia(&mut self) -> f32 {
        let (ia, _, _) = self.read_currents();
        ia
    }

    /// Read Ib only (optimized path).
    ///
    /// Default implementation reads all currents and returns Ib.
    /// Implementations may override for more efficient single-phase reads.
    fn read_ib(&mut self) -> f32 {
        let (_, ib, _) = self.read_currents();
        ib
    }

    /// Read Ic only (optimized path).
    ///
    /// Default implementation reads all currents and returns Ic.
    /// Implementations may override for more efficient single-phase reads.
    fn read_ic(&mut self) -> f32 {
        let (_, _, ic) = self.read_currents();
        ic
    }
}

// Re-export CalibrateError from parent module
pub use super::CalibrateError;
