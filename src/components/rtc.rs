use core::marker::PhantomData;

use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

#[cfg(feature = "async")]
use crate::Async;
use crate::{Blocking, Mode};

const RTC_ADDR: u8 = 0x68;

/// The board's real time clock.
///
/// `M` selects the board mode: see [`Mode`]. Both modes expose the same two
/// methods, they just differ in whether the I2C traffic is awaited.
pub struct Rtc<I2C, M = Blocking> {
    i2c: I2C,
    _mode: PhantomData<M>,
}

impl<I2C, M: Mode> Rtc<I2C, M> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            _mode: PhantomData,
        }
    }
}

impl<I2C: I2c> Rtc<I2C, Blocking> {
    /// Set the RTC time
    pub fn set_time(
        &mut self,
        year: u16,
        month: u8,
        day: u8,
        hour: u8,
        min: u8,
        sec: u8,
    ) -> Result<(), I2C::Error> {
        let year_u8 = (year - 2000) as u8;
        let sec_bcd = to_bcd(sec);
        let min_bcd = to_bcd(min);
        let hour_bcd = to_bcd(hour);
        let day_bcd = to_bcd(day);
        let weekday_bcd = to_bcd(5);
        let month_bcd = to_bcd(month);
        let year_bcd = to_bcd(year_u8);
        self.i2c.write(
            RTC_ADDR,
            &[
                0x00,
                sec_bcd,
                min_bcd,
                hour_bcd,
                weekday_bcd,
                day_bcd,
                month_bcd,
                year_bcd,
            ],
        )
    }

    /// Read the RTC time
    pub fn read_time(&mut self) -> Result<(u16, u8, u8, u8, u8, u8), I2C::Error> {
        // read time from RTC
        let mut buf = [0u8; 7];
        self.i2c.write_read(RTC_ADDR, &[0x00], &mut buf)?;
        let sec = from_bcd(buf[0] & 0x7F);
        let min = from_bcd(buf[1] & 0x7F);
        let hour = from_bcd(buf[2] & 0x3F);
        let day = from_bcd(buf[4] & 0x3F);
        let month = from_bcd(buf[5] & 0x1F);
        let year = from_bcd(buf[6]) as u16 + 2000;
        Ok((year, month, day, hour, min, sec))
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> Rtc<I2C, Async> {
    /// Set the RTC time
    pub async fn set_time(
        &mut self,
        year: u16,
        month: u8,
        day: u8,
        hour: u8,
        min: u8,
        sec: u8,
    ) -> Result<(), I2C::Error> {
        let year_u8 = (year - 2000) as u8;
        let sec_bcd = to_bcd(sec);
        let min_bcd = to_bcd(min);
        let hour_bcd = to_bcd(hour);
        let day_bcd = to_bcd(day);
        let weekday_bcd = to_bcd(5);
        let month_bcd = to_bcd(month);
        let year_bcd = to_bcd(year_u8);
        self.i2c
            .write(
                RTC_ADDR,
                &[
                    0x00,
                    sec_bcd,
                    min_bcd,
                    hour_bcd,
                    weekday_bcd,
                    day_bcd,
                    month_bcd,
                    year_bcd,
                ],
            )
            .await
    }

    /// Read the RTC time
    pub async fn read_time(&mut self) -> Result<(u16, u8, u8, u8, u8, u8), I2C::Error> {
        // read time from RTC
        let mut buf = [0u8; 7];
        self.i2c.write_read(RTC_ADDR, &[0x00], &mut buf).await?;
        let sec = from_bcd(buf[0] & 0x7F);
        let min = from_bcd(buf[1] & 0x7F);
        let hour = from_bcd(buf[2] & 0x3F);
        let day = from_bcd(buf[4] & 0x3F);
        let month = from_bcd(buf[5] & 0x1F);
        let year = from_bcd(buf[6]) as u16 + 2000;
        Ok((year, month, day, hour, min, sec))
    }
}

// Simple BCD helpers
fn to_bcd(val: u8) -> u8 {
    ((val / 10) << 4) | (val % 10)
}
fn from_bcd(val: u8) -> u8 {
    10 * (val >> 4) + (val & 0x0F)
}
