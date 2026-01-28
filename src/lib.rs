#![no_std]
//! # uFerris Board Support Package Crate
//!
//! uFerris is a flexible Rust embedded learning kit that can accomodate several SeeedStudio Xiao controllers.
//! The `uferris-bsp` crate provides a generic Board Support Package for the uFerris carrier board.

use core::fmt;
use core::marker::PhantomData; // Added this import
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;
use embedded_hal::pwm::SetDutyCycle;

#[cfg(feature = "power-board")]
use embedded_sdmmc::{BlockDevice, Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
#[cfg(feature = "power-board")]
use ina219::{SyncIna219, address::Address, calibration::IntCalibration};

// Export generic components
mod components;
pub use components::io_expander::SwPos;

// Export the specific board implementation based on features
pub mod boards;

// ------------------------------------------
// Feature-Gated Trait Alias
// ------------------------------------------

#[cfg(feature = "power-board")]
pub trait PowerConstraints: BlockDevice {}
#[cfg(feature = "power-board")]
impl<T: BlockDevice> PowerConstraints for T {}

#[cfg(not(feature = "power-board"))]
pub trait PowerConstraints {}
#[cfg(not(feature = "power-board"))]
impl<T> PowerConstraints for T {}

// ------------------------------------------
// Constants & Errors
// ------------------------------------------
const INA219_ADDR: u8 = 0x42;

#[derive(Debug, Clone)]
pub struct InitError;

impl fmt::Display for InitError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "uFerris initialization error")
    }
}

// ------------------------------------------
// Generic Board Struct
// ------------------------------------------

/// The Main Carrier Board Driver.
pub struct Uferris<LED, BTN, BUZZ, I2C, ADC, BD>
where
    LED: OutputPin,
    BTN: InputPin,
    BUZZ: SetDutyCycle,
    I2C: I2c,
    ADC: components::ldr::OneShot,
    BD: PowerConstraints,
{
    pub led1: components::led::Led<LED>,
    pub sw_btn5: components::button::Button<BTN>,
    pub buzzer: components::buzzer::Buzzer<BUZZ>,
    pub ldr: ADC,
    pub expander: components::io_expander::IoExpander<I2C>,
    pub rtc: components::rtc::Rtc<I2C>,
    pub i2c: I2C,
    #[cfg(feature = "power-board")]
    pub vol_mgr: Option<VolumeManager<BD, DummyTimeSource>>,
    #[cfg(feature = "power-board")]
    pub power_monitor: SyncIna219<I2C, IntCalibration>,
    // This phantom field "uses" BD when the power-board feature is disabled
    pub _phantom: PhantomData<BD>,
}

impl<LED, BTN, BUZZ, I2C, ADC, BD> Uferris<LED, BTN, BUZZ, I2C, ADC, BD>
where
    LED: OutputPin,
    BTN: InputPin,
    BUZZ: SetDutyCycle,
    I2C: I2c,
    ADC: components::ldr::OneShot,
    BD: PowerConstraints,
{
    /// Create a new generic uFerris board instance.
    pub fn new(
        led1_pin: LED,
        sw_btn5_pin: BTN,
        pwm_pin: BUZZ,
        ldr_driver: ADC,
        expander_i2c: I2C,
        rtc_i2c: I2C,
        raw_i2c: I2C,
        #[cfg(feature = "power-board")] vol_mgr: Option<VolumeManager<BD, DummyTimeSource>>,
        #[cfg(feature = "power-board")] ina_i2c: I2C,
    ) -> Result<Self, InitError> {
        let mut expander = components::io_expander::IoExpander::new(expander_i2c);
        let rtc = components::rtc::Rtc::new(rtc_i2c);

        expander.init().map_err(|_| InitError)?;

        let led1 = components::led::Led { pin: led1_pin };
        let sw_btn5 = components::button::Button { pin: sw_btn5_pin };
        let buzzer = components::buzzer::Buzzer { pin: pwm_pin };

        #[cfg(feature = "power-board")]
        let power_monitor = {
            let calib =
                IntCalibration::new(ina219::calibration::MicroAmpere(1000), 1000_000).unwrap();

            SyncIna219::new_calibrated(ina_i2c, Address::from_byte(INA219_ADDR).unwrap(), calib)
                .map_err(|_| InitError)?
        };

        Ok(Self {
            led1,
            sw_btn5,
            buzzer,
            ldr: ldr_driver,
            expander,
            rtc,
            i2c: raw_i2c,
            #[cfg(feature = "power-board")]
            vol_mgr,
            #[cfg(feature = "power-board")]
            power_monitor,
            _phantom: PhantomData,
        })
    }

    // --- Peripheral Methods ---
    pub fn led1_on(&mut self) {
        let _ = self.led1.pin.set_high();
    }
    pub fn led1_off(&mut self) {
        let _ = self.led1.pin.set_low();
    }
    pub fn led2_on(&mut self) -> Result<(), I2C::Error> {
        self.expander.led2_on()
    }
    pub fn led2_off(&mut self) -> Result<(), I2C::Error> {
        self.expander.led2_off()
    }
    pub fn read_sw5(&mut self) -> bool {
        self.sw_btn5.pin.is_high().unwrap_or(false)
    }
    pub fn read_ldr(&mut self) -> u16 {
        self.ldr.read_raw()
    }
    pub fn buzz_on(&mut self, duty: u16) {
        let _ = self.buzzer.pin.set_duty_cycle(duty);
    }
    pub fn buzz_off(&mut self) {
        let _ = self.buzzer.pin.set_duty_cycle_fully_off();
    }

    #[cfg(feature = "power-board")]
    pub fn read_system_voltage(&mut self) -> Option<u16> {
        self.power_monitor
            .bus_voltage()
            .ok()
            .map(|v| v.voltage_mv())
    }
}

// ------------------------------------------
// Helper Types
// ------------------------------------------
#[cfg(feature = "power-board")]
#[derive(Default, Clone, Copy)]
pub struct DummyTimeSource;

#[cfg(feature = "power-board")]
impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
