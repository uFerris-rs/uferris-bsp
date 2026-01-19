pub trait OneShot {
    fn read_raw(&mut self) -> nb::Result<u16, ()>;
}

#[cfg(feature = "xiao-esp32c3")]
use esp_hal::{
    Blocking,
    analog::adc::{Adc, AdcChannel, AdcPin},
    gpio::AnalogPin,
    peripherals::ADC1,
};
#[cfg(feature = "xiao-esp32c3")]
use nb;

#[cfg(feature = "xiao-esp32c3")]
pub struct Ldr<'d, P>
where
    P: AnalogPin + AdcChannel,
{
    pub adc: Adc<'d, ADC1<'d>, Blocking>,
    pub pin: AdcPin<P, ADC1<'d>>,
}

#[cfg(feature = "xiao-esp32c3")]
impl<'d, P> OneShot for Ldr<'d, P>
where
    P: AnalogPin + AdcChannel,
{
    fn read_raw(&mut self) -> Result<u16, nb::Error<()>> {
        self.adc.read_oneshot(&mut self.pin)
    }
}
