use embedded_hal::pwm::SetDutyCycle;

pub struct Buzzer<PIN>
where
    PIN: SetDutyCycle,
{
    pub pin: PIN,
}
