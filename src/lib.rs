#![no_std]
//! # uFerris Board Support Package Crate
//!
//! uFerris is a flexible Rust embedded learning kit that can accomodate several SeeedStudio Xiao controllers.
//! uFerris is essentially a carrier board that can accomodate mutliple different controllers.
//!
//! The `uferris-bsp` crate provides a generic Board Support Package for the uFerris carrier board. As such, `uferris-bsp` is architecture-agnostic and can support for several MCUs (ESP32, RP2040...etc.)
//! Controller support is provided via feature flags.
//!
//! In summary, this crate is meant to provide a software abstraction to easily drive the uFerris board with any supported Xiao Controller.
//!
//! ## Crate Architechture
//! The uFerris BSP architechture follows the layer scheme shown in the figure below. The upper uFerris board logic layer is meant to provide a hardware agnositc uniform interface across all Xiao controllers.
//! The second adapter layer is introduced to create the mappings between the logic and the individual device HALs. The adapter layer also utilizes the `embedded-hal` traits where possible. In most cases the controller HALs provide implementations for `embedded-hal` traits.
//!
//! ![BSP Architechture Diagam](https://raw.githubusercontent.com/uFerris-rs/uferris-bsp/master/images/uFerrisBSP.png)
//!
//! ## Current Supported Xiaos:
//! - Xiao ESP32-C3
//!
//! ## `async` Support
//! This crate does not support `async` yet. Support would entail adding board `async` method calls for the different functions in `lib.rs`.
//!
//! ## Contributing to the uFerris BSP - Adding Xiao Support:
//! Adding support for a new Xiao entails two things:
//! 1. **Adding a Feature Flag**: This entails adding a feature flag in `Cargo.toml` that imports the new device HAL.
//! 2. **Creating a Board Adapter**: This entails adding a new board definition (adapter layer) under the crate `boards/` folder.
//!
//! Other files in the crate should remain unchanged.
//! It is recommended to view the existing board implementations for guidance on creating an adapter layer.
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
//!
//! ## Usage Example
//! Placeholder for usage example

// ------------------------------------------
// Imports
// ------------------------------------------
use core::fmt;
use core::marker::PhantomData;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;
#[cfg(feature = "power-board")]
use ina219::{SyncIna219, address::Address, calibration::IntCalibration};

// Export generic components so users can access types (like SevenSegDigit)
mod components;
use components::io_expander::SwPos;

// Export the specific board implementation based on features
pub mod boards;

use embedded_hal::pwm::SetDutyCycle;
#[cfg(feature = "power-board")]
use embedded_sdmmc::{BlockDevice, Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};

// ------------------------------------------
// Constants
// ------------------------------------------
const INA219_ADDR: u8 = 0x42;

// ------------------------------------------
// Error Types
// ------------------------------------------
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
///
/// This struct owns all the drivers on the carrier board.
/// It is generic over the specific hardware implementation.
///
/// - `PIN1`: LED Pin (Implments the embedded_hal::digital::OutputPin Trait)
/// - `PIN2`: Button Pin (Implments the embedded_hal::digital::OutputPin Trait)
/// - `BUZZ`: PWM Pin (Implments the embedded_hal::pwm::SetDutyCycle Trait)
/// - `I2C`: The I2C Bus type (Implments the embedded_hal::i2c::I2c Trait)
/// - `ADC`: The LDR Driver type (Implments the uferris_bsp::components::ldr::OneShot Trait)
pub struct Uferris<LED, BTN, BUZZ, I2C, ADC, #[cfg(feature = "power-board")] BD>
where
    LED: embedded_hal::digital::OutputPin,
    BTN: embedded_hal::digital::InputPin,
    BUZZ: embedded_hal::pwm::SetDutyCycle,
    I2C: embedded_hal::i2c::I2c,
    ADC: components::ldr::OneShot,
    BD: BlockDevice,
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
}

impl<LED, BTN, BUZZ, I2C, ADC, #[cfg(feature = "power-board")] BD>
    Uferris<LED, BTN, BUZZ, I2C, ADC, BD>
where
    LED: OutputPin,
    BTN: InputPin,
    BUZZ: SetDutyCycle,
    I2C: I2c,
    ADC: components::ldr::OneShot,
    BD: BlockDevice,
{
    /// Create a new generic uFerris board instance.
    /// This function consumes the hardware resources and returns an initialized board driver.
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
        // Instantiate the IO Expander
        let mut expander = components::io_expander::IoExpander::new(expander_i2c);

        // Instantiate the RTC
        let rtc = components::rtc::Rtc::new(rtc_i2c);

        // Map the generic I2C error to a generic InitError
        expander.init().map_err(|_| InitError)?;

        // Wrap Pins in Component Drivers
        let led1 = components::led::Led { pin: led1_pin };
        let sw_btn5 = components::button::Button { pin: sw_btn5_pin };
        let buzzer = components::buzzer::Buzzer { pin: pwm_pin };

        #[cfg(feature = "power-board")]
        let power_monitor = {
            // Use 1mA resolution and 100mOhm shunt (from your original code)
            let calib = IntCalibration::new(
                ina219::calibration::MicroAmpere(1000),
                1000_000, // 100mOhm in micro-ohms
            )
            .unwrap();

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
        })
    }

    /// Turn on LED 1
    pub fn led1_on(&mut self) {
        self.led1.pin.set_high().unwrap();
    }

    /// Turn off LED 1
    pub fn led1_off(&mut self) {
        self.led1.pin.set_low().unwrap();
    }

    /// Turn on LED 2
    pub fn led2_on(&mut self) -> Result<(), I2C::Error> {
        self.expander.led2_on()
    }

    /// Turn off LED 2
    pub fn led2_off(&mut self) -> Result<(), I2C::Error> {
        self.expander.led2_off()
    }

    /// Turn on LED 3
    pub fn led3_on(&mut self) -> Result<(), I2C::Error> {
        self.expander.led3_on()
    }

    /// Turn off LED 3
    pub fn led3_off(&mut self) -> Result<(), I2C::Error> {
        self.expander.led3_off()
    }

    /// Read Button Switch 1
    pub fn read_sw1(&mut self) -> Result<bool, I2C::Error> {
        self.expander.read_sw1()
    }

    /// Read Button Switch 2
    pub fn read_sw2(&mut self) -> Result<bool, I2C::Error> {
        self.expander.read_sw2()
    }

    /// Read Button Switch 3
    pub fn read_sw3(&mut self) -> Result<bool, I2C::Error> {
        self.expander.read_sw3()
    }

    /// Read Button Switch 4
    pub fn read_sw4(&mut self) -> Result<bool, I2C::Error> {
        self.expander.read_sw4()
    }

    /// Read Button Switch 5
    pub fn read_sw5(&mut self) -> bool {
        self.sw_btn5.pin.is_high().unwrap_or(false)
    }

    /// Read Slide Switch 6
    pub fn read_sw6(&mut self) -> Result<SwPos, I2C::Error> {
        self.expander.read_slide_sw6_position()
    }

    /// Read Slide Switch 7
    pub fn read_sw7(&mut self) -> Result<SwPos, I2C::Error> {
        self.expander.read_slide_sw7_position()
    }

    /// Read LDR Voltage
    pub fn read_ldr(&mut self) -> u16 {
        self.ldr.read_raw()
    }

    /// Turn on the buzzer with specified duty cycle
    pub fn buzz_on(&mut self, duty: u16) {
        self.buzzer.pin.set_duty_cycle(duty).unwrap();
    }

    /// Turn off the buzzer
    pub fn buzz_off(&mut self) {
        self.buzzer.pin.set_duty_cycle_fully_off().unwrap();
    }

    /// Perform a raw I2C write operation
    pub fn i2c_write(&mut self, addr: u8, data: &[u8]) -> Result<(), I2C::Error> {
        self.i2c.write(addr, data)
    }

    /// Perform a raw I2C read operation
    pub fn i2c_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.read(addr, buffer)
    }

    /// Perform a raw I2C write-read operation
    pub fn i2c_write_read(
        &mut self,
        addr: u8,
        data: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), I2C::Error> {
        self.i2c.write_read(addr, data, buffer)
    }

    /// Set the uFerris RTC time
    pub fn set_rtc_time(
        &mut self,
        year: u16,
        month: u8,
        day: u8,
        hour: u8,
        min: u8,
        sec: u8,
    ) -> Result<(), I2C::Error> {
        self.rtc.set_time(year, month, day, hour, min, sec)
    }

    /// Read the uFerris RTC time
    pub fn read_rtc_time(&mut self) -> Result<(u16, u8, u8, u8, u8, u8), I2C::Error> {
        self.rtc.read_time()
    }

    /// Read the System Voltage in milliVolts
    #[cfg(feature = "power-board")]
    pub fn read_system_voltage(&mut self) -> Option<u16> {
        self.power_monitor
            .bus_voltage()
            .ok()
            .map(|v| v.voltage_mv())
    }

    /// Read the System Current in microAmps
    #[cfg(feature = "power-board")]
    pub fn read_system_current(&mut self) -> Option<i32> {
        match self.power_monitor.next_measurement() {
            Ok(measurement) => {
                if let Some(value) = measurement {
                    Some(value.current.0)
                } else {
                    None
                }
            }
            Err(_) => None,
        };
        None
    }

    /// Read the System Power in milliWatts
    #[cfg(feature = "power-board")]
    pub fn read_system_power(&mut self) -> Option<i32> {
        match self.power_monitor.next_measurement() {
            Ok(measurement) => {
                if let Some(value) = measurement {
                    Some(value.power.0)
                } else {
                    None
                }
            }
            Err(_) => None,
        };
        None
    }

    /// Initialize / Check SD Card
    /// Returns the size of the SD card in bytes if detected.
    pub fn init_sd_card(&mut self) -> Result<u64, InitError> {
        let mgr = self.vol_mgr.as_mut().ok_or(InitError)?;

        // Variable to capture the card size result
        let mut card_size_result = Err(InitError);

        // Access underying device (SD Card Block Device). TimeSource return value is ignored.
        let _ = mgr.device(|dev| {
            // Initialize device
            let blocks_res = dev.num_blocks().map_err(|_| InitError);

            // Update size
            card_size_result = blocks_res.map(|b| b.0 as u64 * 512);

            // Return time source as required by API
            // Strange behaviour from API, not sure why this is required.
            crate::DummyTimeSource::default()
        });

        card_size_result
    }

    /// Read a file from the root directory in chunks
    pub fn read_file_chunked<F>(&mut self, filename: &str, mut func: F)
    where
        F: FnMut(&[u8]),
    {
        // Unwrap the Manager
        let mgr = match self.vol_mgr.as_mut() {
            Some(m) => m,
            None => return, // Or panic, depending on preference
        };

        // Open Volume (Partition 0)
        let volume = match mgr.open_volume(VolumeIdx(0)) {
            Ok(v) => v,
            Err(_) => return,
        };

        // Open Root Directory
        let root_dir = match volume.open_root_dir() {
            Ok(d) => d,
            Err(_) => return,
        };

        // Open File
        let file = match root_dir.open_file_in_dir(filename, Mode::ReadOnly) {
            Ok(f) => f,
            Err(_) => return,
        };

        // 5. Read Loop
        let mut buffer = [0u8; 32];
        while !file.is_eof() {
            if let Ok(bytes_read) = file.read(&mut buffer) {
                if bytes_read > 0 {
                    func(&buffer[0..bytes_read]);
                }
            } else {
                break;
            }
        }

        // File, Dir, and Vol are dropped here, closing handles automatically
    }

    /// Write data to a file in the root directory
    /// Creates the file if it doesn't exist, or truncates it if it does.
    pub fn write_to_file_in_root(&mut self, filename: &str, data: &[u8]) {
        let mgr = match self.vol_mgr.as_mut() {
            Some(m) => m,
            None => return,
        };

        let volume = match mgr.open_volume(VolumeIdx(0)) {
            Ok(v) => v,
            Err(_) => return,
        };

        let root_dir = match volume.open_root_dir() {
            Ok(d) => d,
            Err(_) => return,
        };

        let file = match root_dir.open_file_in_dir(filename, Mode::ReadWriteCreateOrTruncate) {
            Ok(f) => f,
            Err(_) => return,
        };

        if file.write(data).is_ok() {
            let _ = file.flush();
        }
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
