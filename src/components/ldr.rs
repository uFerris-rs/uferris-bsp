/// A trait for reading a raw ADC value.
pub trait OneShot {
    fn read_raw(&mut self) -> u16;
}
