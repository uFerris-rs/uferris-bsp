use embedded_hal::i2c::I2c;

const RTC_ADDR: u8 = 0x68; // Standard DS3231/PCF8523 address

pub struct Rtc<I2C> {
    i2c: I2C,
}

impl<I2C: I2c> Rtc<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Set the RTC time
    pub fn set_time(&mut self, hours: u8, minutes: u8, seconds: u8) -> Result<(), I2C::Error> {
        // Implementation dependent on specific RTC chip (e.g. PCF8563 vs DS3231)
        // This is just a placeholder example
        let data = [
            0x00, // Start register (seconds usually)
            to_bcd(seconds),
            to_bcd(minutes),
            to_bcd(hours),
        ];
        self.i2c.write(RTC_ADDR, &data)
    }

    /// Read the RTC time
    pub fn read_time(&mut self) -> Result<(u8, u8, u8), I2C::Error> {
        let mut buf = [0u8; 3];
        self.i2c.write_read(RTC_ADDR, &[0x00], &mut buf)?;

        let sec = from_bcd(buf[0] & 0x7F);
        let min = from_bcd(buf[1] & 0x7F);
        let hr = from_bcd(buf[2] & 0x3F);

        Ok((hr, min, sec))
    }
}

// Simple BCD helpers
fn to_bcd(val: u8) -> u8 {
    ((val / 10) << 4) | (val % 10)
}
fn from_bcd(val: u8) -> u8 {
    10 * (val >> 4) + (val & 0x0F)
}
