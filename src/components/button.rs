use embedded_hal::digital::InputPin;

pub struct Button<PIN>
where
    PIN: InputPin,
{
    pub pin: PIN,
}
