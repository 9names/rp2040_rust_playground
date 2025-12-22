#![cfg_attr(not(test), no_std)]

/// This is a port of the PWM frequency generation code from Micropython for use with rp-hal
/// https://github.com/micropython/micropython/blob/634125820744efa33679fb95a6e441dadaa4f6a7/ports/rp2/machine_pwm.c#L212C13-L212C36
///
/// This code remains under the MIT license specified in machine_pwm.c - see LICENSE.txt for the full copyright notice and attribution.

/// Maximum "top" value, set at 65534 to be able to achieve 100% duty with 65535.
const TOP_MAX: u32 = 65534;

/// Threshold for overflow protection in get_slice_hz.
/// When source_hz + offset/16 exceeds this value, use 64-bit integer arithmetic to avoid overflow.
const OVERFLOW_THRESHOLD: u32 = 268_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmFreq {
    pwm_config: PwmConfig,
    requested_freq: u32,
    source_hz: u32,
}

impl PwmFreq {
    pub fn new(source_hz: u32, freq: u32) -> Result<Self, PwmError> {
        let pwm_config = calculate_pwm_freq(freq, source_hz)?;
        Ok(Self {
            pwm_config,
            requested_freq: freq,
            source_hz,
        })
    }

    pub fn set_freq(&mut self, freq: u32) -> Result<(), PwmError> {
        self.pwm_config = calculate_pwm_freq(freq, self.source_hz)?;
        Ok(())
    }

    pub fn get_requested_freq(&self) -> u32 {
        self.requested_freq
    }

    pub fn get_actual_freq(&self) -> f64 {
        pwm_config_to_freq(self.pwm_config, self.source_hz)
    }

    pub fn get_config(&self) -> PwmConfig {
        self.pwm_config
    }
}

/// PWM configuration containing the divider and top values.
/// div is (integer_part, fractional_part) where the actual divider is integer_part + fractional_part/16
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmConfig {
    pub top: u16,
    pub div: (u8, u8),
}

/// Errors that can occur when calculating PWM frequency.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmError {
    /// Frequency is too large (div16 < 16)
    FreqTooLarge,
    /// Frequency is too small (div16 >= 256 * 16)
    FreqTooSmall,
}

/// Returns floor((16*F + offset) / div16) where F is source_hz.
/// Avoids overflow in the numerator that would occur if
///   16*F + offset > 2**32
///   F + offset/16 > 2**28 = 268435456 (approximately, due to flooring)
fn get_slice_hz(offset: u32, div16: u32, source_hz: u32) -> u32 {
    if source_hz + (offset / 16) > OVERFLOW_THRESHOLD {
        // Use 64-bit arithmetic to avoid overflow
        let (source_hz, offset, div16) = (source_hz as u64, offset as u64, div16 as u64);
        ((16 * source_hz + offset) / div16) as u32
    } else {
        (16 * source_hz + offset) / div16
    }
}

/// Returns 16*F / div16, rounded.
fn get_slice_hz_round(div16: u32, source_hz: u32) -> u32 {
    get_slice_hz(div16 / 2, div16, source_hz)
}

/// Returns ceil(16*F / div16).
fn get_slice_hz_ceil(div16: u32, source_hz: u32) -> u32 {
    get_slice_hz(div16 - 1, div16, source_hz)
}

/// Calculate PWM frequency settings (div16 and top) for a given target frequency.
///
/// # Arguments
/// * `freq` - Target frequency in Hz
/// * `source_hz` - Source clock frequency in Hz
///
/// # Returns
/// * `Ok(PwmConfig)` - Configuration with div16 and top values
/// * `Err(PwmError)` - Error if frequency is too large or too small.
///   According to the datasheet, the lower limit is ~7.5hz when sysclk == 125Mhz.
pub fn calculate_pwm_freq(freq: u32, source_hz: u32) -> Result<PwmConfig, PwmError> {
    if freq == 0 {
        return Err(PwmError::FreqTooSmall);
    }

    let (div16, top) = if (source_hz + (freq / 2)) / freq < TOP_MAX {
        // If possible (based on the formula for TOP below), use a DIV of 1.
        // This also prevents overflow in the DIV calculation.
        let div16 = 16;
        // Same as get_slice_hz_round() below but canceling the 16s
        // to avoid overflow for high freq.
        let top = ((source_hz + (freq / 2)) / freq) - 1;
        (div16, top)
    } else {
        // Otherwise, choose the smallest possible DIV for maximum
        // duty cycle resolution.
        // Constraint: 16*F/(div16*freq) < TOP_MAX
        // So:
        let div16 = get_slice_hz_ceil(TOP_MAX * freq, source_hz);
        // Set TOP as accurately as possible using rounding.
        let top = get_slice_hz_round(div16 * freq, source_hz) - 1;
        (div16, top)
    };

    if div16 < 16 {
        Err(PwmError::FreqTooLarge)
    } else if div16 >= 256 * 16 {
        Err(PwmError::FreqTooSmall)
    } else {
        // Convert div16 (in units of 1/16) to (integer_part, fractional_part)
        let div_int = (div16 / 16) as u8;
        let div_frac = (div16 % 16) as u8;

        let top = top as u16;
        Ok(PwmConfig {
            div: (div_int, div_frac),
            top,
        })
    }
}

/// Calculate the achieved PWM frequency from a PwmConfig.
///
/// # Arguments
/// * `config` - PWM configuration
/// * `source_hz` - Source clock frequency in Hz
///
/// # Returns
/// The actual frequency in Hz, as an f64 (as it's fractional we usually can't get an integer representation back)
pub fn pwm_config_to_freq(config: PwmConfig, source_hz: u32) -> f64 {
    // Convert div tuple back to div16 (in units of 1/16)
    // convert all the values to f64 to make the calculations more terse
    let div0 = config.div.0 as f64;
    let div1 = config.div.1 as f64;
    let src_hz = source_hz as f64;
    let top = config.top as f64;

    let div16 = (div0 * 16.0) + div1;
    // Calculate frequency: freq = (16 * source_hz) / (div16 * (top + 1))
    (16.0 * src_hz) / (div16 * (top + 1.0))
}
