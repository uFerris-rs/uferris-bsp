use embedded_hal::digital::OutputPin;

pub struct Led<PIN>
where
    PIN: OutputPin,
{
    pub pin: PIN,
}
