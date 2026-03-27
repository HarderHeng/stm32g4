//! Voltage and temperature monitoring trait
//!
//! This trait abstracts the hardware-specific bus voltage and temperature
//! sensing details, providing a clean interface for protection monitoring.

/// Bus voltage and temperature monitoring.
///
/// This trait provides the interface for reading bus voltage and temperature,
/// used for protection and monitoring in FOC motor control applications.
///
/// # Implementation Notes
///
/// - Voltage is returned in Volts
/// - Temperature is returned in degrees Celsius
/// - Implementations typically use ADC channels with voltage dividers
pub trait VoltageMonitor {
    /// Read bus voltage in Volts.
    ///
    /// This is the main DC link voltage powering the motor driver.
    /// Typically measured through a resistive divider into an ADC channel.
    fn read_bus_voltage(&mut self) -> f32;

    /// Read heat-sink / NTC temperature in degrees Celsius.
    ///
    /// Temperature is measured through an NTC thermistor in a voltage
    /// divider configuration, converted using the Steinhart-Hart equation
    /// or a simplified beta parameter model.
    fn read_temperature(&mut self) -> f32;

    /// Check for undervoltage condition.
    ///
    /// # Arguments
    ///
    /// * `threshold` - Undervoltage threshold in Volts
    ///
    /// # Returns
    ///
    /// `true` if bus voltage is below the threshold
    fn is_undervoltage(&mut self, threshold: f32) -> bool {
        self.read_bus_voltage() < threshold
    }

    /// Check for overvoltage condition.
    ///
    /// # Arguments
    ///
    /// * `threshold` - Overvoltage threshold in Volts
    ///
    /// # Returns
    ///
    /// `true` if bus voltage is above the threshold
    fn is_overvoltage(&mut self, threshold: f32) -> bool {
        self.read_bus_voltage() > threshold
    }

    /// Check for overtemperature condition.
    ///
    /// # Arguments
    ///
    /// * `threshold` - Overtemperature threshold in degrees Celsius
    ///
    /// # Returns
    ///
    /// `true` if temperature is above the threshold
    fn is_overtemperature(&mut self, threshold: f32) -> bool {
        self.read_temperature() > threshold
    }

    /// Read bus voltage with filtering (moving average).
    ///
    /// Default implementation takes multiple samples and averages them.
    /// Implementations may override for more efficient filtered reads.
    fn read_bus_voltage_filtered(&mut self, samples: usize) -> f32 {
        let mut sum = 0.0;
        for _ in 0..samples {
            sum += self.read_bus_voltage();
        }
        sum / samples as f32
    }

    /// Read temperature with filtering (moving average).
    ///
    /// Default implementation takes multiple samples and averages them.
    /// Implementations may override for more efficient filtered reads.
    fn read_temperature_filtered(&mut self, samples: usize) -> f32 {
        let mut sum = 0.0;
        for _ in 0..samples {
            sum += self.read_temperature();
        }
        sum / samples as f32
    }
}
