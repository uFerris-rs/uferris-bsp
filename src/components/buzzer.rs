/// The buzzer, driven by a PWM channel.
///
/// The `SetDutyCycle` bound lives on the board `impl` blocks that drive the
/// channel rather than on this struct, so that a board type can be named
/// without it.
pub struct Buzzer<PIN> {
    pub pin: PIN,
}
