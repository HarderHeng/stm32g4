//! Board Support Package configuration module
//!
//! This module provides board-specific configuration structures
//! that extend the base Config with hardware-specific details.

/// Board configuration for B-G431B-ESC1
///
/// Contains all configuration specific to the ST B-G431B-ESC1
/// motor control discovery board.
#[derive(Clone, Copy, Debug)]
pub struct BG431bEsc1Config {
    /// Base configuration
    pub base: crate::config::Config,
}

impl Default for BG431bEsc1Config {
    fn default() -> Self {
        Self {
            base: crate::config::Config::for_b_g431b_esc1(),
        }
    }
}

impl BG431bEsc1Config {
    /// Create default configuration for B-G431B-ESC1
    pub fn new() -> Self {
        Self::default()
    }
}

/// Get the default board configuration
pub fn get_board_config() -> BG431bEsc1Config {
    BG431bEsc1Config::default()
}
