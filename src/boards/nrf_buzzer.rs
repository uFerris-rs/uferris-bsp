//! The buzzer PWM channel shared by the nRF board adapters.
//!
//! Every nRF part `embassy-nrf` supports clocks its PWM peripheral from the
//! same 16 MHz `PWM_CLK`, and every uFerris board drives the buzzer at the same
//! frequency, so the nRF52840 and nRF54L15 adapters would otherwise carry two
//! copies of the same countertop and rescaling arithmetic. They live here
//! instead, and both adapters build their buzzer out of [`NrfBuzzerChannel`].

use embassy_nrf::pwm::{DutyCycle, SimplePwm};
use embedded_hal::pwm::{ErrorType as PwmErrorType, SetDutyCycle};

/// Buzzer PWM output frequency, matching the other boards.
const BUZZER_FREQ_HZ: u32 = 2_700;

/// Frequency of `PWM_CLK` with the prescaler set to
/// [`Prescaler::Div1`](embassy_nrf::pwm::Prescaler::Div1).
///
/// `embassy-nrf` exposes this as [`embassy_nrf::pwm::PWM_CLK_HZ`] and uses the
/// one value for every chip it supports, the nRF54L family included: the PWM
/// peripheral is fed from a fixed 16 MHz clock and is not affected by the
/// nRF54L core clock selection.
const PWM_CLK_HZ: u32 = embassy_nrf::pwm::PWM_CLK_HZ;

/// nRF `COUNTERTOP` that produces [`BUZZER_FREQ_HZ`] from [`PWM_CLK_HZ`].
///
///     COUNTERTOP = 16_000_000 / 2700 = 5925.9 -> 5926
///     f_pwm      = 16_000_000 / 5926 = 2700.0 Hz
pub const BUZZER_COUNTERTOP: u16 = (PWM_CLK_HZ / BUZZER_FREQ_HZ) as u16;

/// Duty cycle resolution reported to application code.
///
/// The LEDC based boards run 14 bit duty cycles, so every board reports the
/// same 16384 and a duty value means the same thing everywhere. See
/// [`NrfBuzzerChannel`] for how it is mapped onto the nRF countertop.
const BUZZER_MAX_DUTY: u16 = 16384;

/// The buzzer PWM channel, presented with the duty cycle resolution the rest of
/// the boards use.
///
/// The nRF PWM period is
///     f_pwm = PWM_CLK / COUNTERTOP
/// where `PWM_CLK` is 16 MHz divided by the prescaler. There is no fractional
/// divider, so the only way to reach 2700 Hz is to pick the countertop, and
/// 16 MHz / 2700 Hz gives 5926 rather than a round power of two. Every other
/// board reports a 14 bit `max_duty_cycle()` of 16384, so this wrapper keeps
/// reporting 16384 and rescales incoming duty values onto the countertop:
///
///     raw = round(duty * 5926 / 16384)
///
/// The rounding is done in `u32` (the largest product is
/// `16384 * 5926 = 97_083_392`, well inside the range) by adding half the
/// divisor before dividing. `duty` 0 maps to 0 and `duty` 16384 maps to 5926,
/// so fully off and fully on stay exact and the worst case error in between is
/// half a countertop step, about 0.008% of the period.
pub struct NrfBuzzerChannel {
    pwm: SimplePwm<'static>,
}

impl NrfBuzzerChannel {
    /// Wrap a one channel [`SimplePwm`] configured with
    /// [`BUZZER_COUNTERTOP`] as its `max_duty`.
    pub fn new(pwm: SimplePwm<'static>) -> Self {
        Self { pwm }
    }
}

impl PwmErrorType for NrfBuzzerChannel {
    type Error = core::convert::Infallible;
}

impl SetDutyCycle for NrfBuzzerChannel {
    fn max_duty_cycle(&self) -> u16 {
        BUZZER_MAX_DUTY
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let duty = duty.min(BUZZER_MAX_DUTY);

        // Round to nearest; see the type level comment for the ranges.
        let raw = (u32::from(duty) * u32::from(BUZZER_COUNTERTOP) + u32::from(BUZZER_MAX_DUTY) / 2)
            / u32::from(BUZZER_MAX_DUTY);

        // Bit 15 of an nRF PWM sequence word selects the polarity of the
        // compare match, and `DutyCycle` is that word. `DutyCycle::normal`
        // leaves the bit clear, which drives the output high once the counter
        // reaches the value, i.e. the value counts the *low* part of the
        // period. `DutyCycle::inverted` sets it, so the value is the high time
        // and a duty of 0 leaves the buzzer pin low. The buzzer is active high,
        // so `inverted` is the one that means what the caller expects.
        self.pwm.set_duty(0, DutyCycle::inverted(raw as u16));

        Ok(())
    }
}
