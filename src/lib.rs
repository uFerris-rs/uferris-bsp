#![no_std]
//! # uFerris Board Support Package Crate
//!
//! uFerris is a flexible Rust embedded learning kit that can accomodate several SeeedStudio Xiao controllers.
//!
//! This crate is meant to provide a software abstraction to easily drive the uFerris board with any supported Xiao Controller.
//!
//! ## Crate Architechture
//! The uFerris BSP architechture follows the architechture shown in the figure below. The BSP utilizes the `embedded-hal` traits at the lower level as much as possible. In most cases the controller HALs provide implementations for `embedded-hal` traits. These traits are then used to define the board’s components.
//! These components then are used as members in the board struct with methods to control the different board features. The reasoning behind this architechture is to ease the addition of Xiao baord support by keeping changes between different boards to a minimum.
//! In some cases where embedded-hal support is not available (ex. ADCs), the trait implmentation needs to be added for the particular controller.
//! Finally, board support crates typically take ownership of the peripherals struct. In cases where this is challenging, macros can be used to pass only the needed peripherals to the board instance.
//!
//! ![BSP Architechture Diagam](/images/uFerrisBSP.png)
//!
//! ## Current Supported Xiaos:
//! - Xiao ESP32-C3
//!
//! ## `async` Support
//! This crate does not support `async` yet.
//!
//! ## Contributing to the uFerris BSP - Adding Xiao Support:
//! If you'd like to add support please follow the contribution instructions.
//! How to Contribute
//! 1. Add feature in `Cargo.toml` for the new board importing the board HAL.
//! 2. In `pins.rs`, add a `Pins` struct definition for new Device
//! 3. Add a trait implmentation for reading the ADC LDR pin.
//! 4. In `lib.rs` create a feature gated board driver struct & add needed imports
//! 5. Under devices/, create a [board_name].rs and add a `new` implementation for the device.
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
//!
//! ## Usage Example
//! Placeholder for usage example
//!

// Crate Imports
use bitmask_enum::bitmask;
use core::fmt;

use embedded_hal::i2c::I2c;
use embedded_hal::pwm::SetDutyCycle;

// Power Board Extension Imports
pub mod pwr_brd {
    pub use embedded_hal_bus::spi::RefCellDevice;
    pub use embedded_sdmmc::SdCard;
    pub use embedded_sdmmc::VolumeManager;
}

#[cfg(feature = "power-board")]
use pwr_brd::*;

// Component Imports
use crate::components::button::Button;
use crate::components::buzzer::Buzzer;
use crate::components::ldr::Ldr;
use crate::components::led::Led;

// Module Definitions
pub mod components;
pub mod devices;
pub mod pins;

// Device Specific HAL imports
#[cfg(feature = "xiao-esp32c3")]
pub mod esp_hal {
    pub use esp_hal::Blocking;
    pub use esp_hal::analog::adc::{Adc, AdcConfig};
    pub use esp_hal::delay::Delay;
    pub use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
    pub use esp_hal::i2c::master::I2c;
    pub use esp_hal::ledc::LowSpeed;
    pub use esp_hal::ledc::channel::Channel;
    pub use esp_hal::peripherals::GPIO2;
    pub use esp_hal::spi::master::Spi;
    pub use esp_hal::time::Rate;
}

// Device Specific Component Imports
#[cfg(feature = "xiao-esp32c3")]
pub mod esp_components {
    pub use crate::components::i2cdevices::I2cDevices;
    #[cfg(feature = "power-board")]
    pub use crate::components::spidevices::SpiDevices;
}

use esp_components::*;
use esp_hal::*;

// uFerris Baseboard Struct Definition
/// Driver struct for the uFerris baseboard
#[cfg(feature = "xiao-esp32c3")]
pub struct Uferris<'d> {
    led1: Led<Output<'d>>,
    button: Button<Input<'d>>,
    ldr: Ldr<'d, GPIO2<'d>>,
    buzzer_channel: Buzzer<Channel<'d, LowSpeed>>,
    i2c_devices: I2cDevices<'d>,
    #[cfg(feature = "power-board")]
    spi_devices: SpiDevices<'d>,
    #[cfg(feature = "power-board")]
    vol_mgr: Option<
        VolumeManager<
            SdCard<RefCellDevice<'d, Spi<'d, Blocking>, Output<'d>, Delay>, Delay>,
            DummyTimeSource,
        >,
    >,
}

// TCA6424 Commands
const IN_PORT0: u8 = 0x80; // Input Port 0 Register
// const IN_PORT1: u8 = 0x81; // Input Port 1 Register
const IN_PORT2: u8 = 0x82; // Input Port 2 Register
const OUT_PORT0: u8 = 0x84; // Output Port 0 Register
const OUT_PORT1: u8 = 0x85; // Output Port 1 Register
const OUT_PORT2: u8 = 0x86; // Output Port 2 Register
// const POL_INV_PORT0: u8 = 0x88; // Polarity Inversion Port 0 Register
// const POL_INV_PORT1: u8 = 0x89; // Polarity Inversion Port 1 Register
// const POL_INV_PORT2: u8 = 0x8A; // Polarity Inversion Port 2 Register
const CONFIG_PORT0: u8 = 0x8C; // Configuration Port 0 Register
// const CONFIG_PORT1: u8 = 0x8D; // Configuration Port 1 Register
// const CONFIG_PORT2: u8 = 0x8E; // Configuration Port 2 Register

// I2C Addresses
const TCA6424_ADDR: u8 = 0x22;
const RTC_ADDR: u8 = 0x68;
const INA219_ADDR: u8 = 0x42;

#[bitmask(u8)]
enum IoExpPort0 {
    Sw7Pos1, // Port 0 Pin 0
    Sw7Pos2, // Port 0 Pin 1
    P02,     // Port 0 Pin 2 (Unused)
    Sw4,     // Port 0 Pin 3
    Sw3,     // Port 0 Pin 4
    Sw2,     // Port 0 Pin 5
    Sw1,     // Port 0 Pin 6
    SegG,    // Port 0 Pin 7
}

#[bitmask(u8)]
enum IoExpPort1 {
    Dp,   // Port 1 Pin 0
    SegA, // Port 1 Pin 1
    SegB, // Port 1 Pin 2
    SegC, // Port 1 Pin 3
    SegD, // Port 1 Pin 4
    SegE, // Port 1 Pin 5
    SegF, // Port 1 Pin 6
    P17,  // Port 1 Pin 7 (Unused)
}

#[bitmask(u8)]
enum IoExpPort2 {
    Digit4,  // Port 2 Pin 0
    Digit3,  // Port 2 Pin 1
    Digit2,  // Port 2 Pin 2
    Digit1,  // Port 2 Pin 3
    Led3,    // Port 2 Pin 4
    Led2,    // Port 2 Pin 5
    Sw6Pos1, // Port 2 Pin 6
    Sw6Pos2, // Port 2 Pin 7
}

// Direction Configuration for TCA6424
// Port 0: All outputs except for SW7 and SW4
// Port 1: All outputs except for DP, SegA, SegB, SegC
// Port 2: All outputs except for Digit4, Digit3, Digit2, Digit
const PORT0_DIR: u8 = 0x7F;
const PORT1_DIR: u8 = 0x00;
const PORT2_DIR: u8 = 0xC0;

#[derive(PartialEq, Debug, Clone, Copy)]
pub enum SwPos {
    Down,
    Up,
    Undefined,
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub enum SevenSegDigit {
    Digit1,
    Digit2,
    Digit3,
    Digit4,
}

// Time Source for SD Card
#[cfg(feature = "power-board")]
#[derive(Default)]
pub struct DummyTimeSource;

#[cfg(feature = "power-board")]
impl embedded_sdmmc::TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct InitError;

impl fmt::Display for InitError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "uFerris initialization error")
    }
}

// Function to convert to BCD from decimal (for RTC)
fn to_bcd(val: u8) -> u8 {
    ((val / 10) << 4) | (val % 10)
}

// Function to convert from BCD
fn from_bcd(val: u8) -> u8 {
    10 * (val >> 4) + (val & 0x0F)
}

impl<'d> Uferris<'d> {
    /// Initialize uFerris System
    /// This method initializes the I/O expander, SD card, and INA219 power monitor (if enabled).
    /// It must be called only once after creating a new Uferris instance
    pub fn init_system(&mut self) -> Result<(), InitError> {
        self.init_io_expander()?;

        #[cfg(feature = "power-board")]
        {
            self.init_sd_card();
        }

        Ok(())
    }

    /// Initialize I/O Expander
    /// This method initializes the I/O expander on the board
    /// It must be called once after creating a new Uferris instance
    fn init_io_expander(&mut self) -> Result<(), InitError> {
        // Configure I/O Expander pins before moving I2C to shared state
        // Write I/O Direction to TCA6424 Config Registers
        self.i2c_write(
            TCA6424_ADDR,
            &[CONFIG_PORT0, PORT0_DIR, PORT1_DIR, PORT2_DIR],
        );

        // Reset all output ports to low
        self.i2c_write(TCA6424_ADDR, &[OUT_PORT0, 0, 0, 0]);

        Ok(())
    }

    /// Turn on LED 1
    pub fn led1_on(&mut self) {
        self.led1.pin.set_high();
    }

    /// Turn off LED 1
    pub fn led1_off(&mut self) {
        self.led1.pin.set_low();
    }

    /// Turn on LED 2
    pub fn led2_on(&mut self) {
        let led_bits = IoExpPort2::Led2.bits();
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[OUT_PORT2], &mut rbuf);
        let current = rbuf[0];
        let new = current | led_bits;
        self.i2c_write(TCA6424_ADDR, &[OUT_PORT2, new]);
    }

    /// Turn off LED 2
    pub fn led2_off(&mut self) {
        let led_bits = IoExpPort2::Led2.bits();
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[OUT_PORT2], &mut rbuf);
        let current = rbuf[0];
        let new = current & !led_bits;
        self.i2c_write(TCA6424_ADDR, &[OUT_PORT2, new]);
    }

    /// Turn on LED 3
    pub fn led3_on(&mut self) {
        let led_bits = IoExpPort2::Led3.bits();
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[OUT_PORT2], &mut rbuf);
        let current = rbuf[0];
        let new = current | led_bits;
        self.i2c_write(TCA6424_ADDR, &[OUT_PORT2, new]);
    }

    /// Turn off LED 3
    pub fn led3_off(&mut self) {
        let led_bits = IoExpPort2::Led3.bits();
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[OUT_PORT2], &mut rbuf);
        let current = rbuf[0];
        let new = current & !led_bits;
        self.i2c_write(TCA6424_ADDR, &[OUT_PORT2, new]);
    }

    /// Read the state of Switch Button 1
    pub fn read_sw1(&mut self) -> bool {
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf);
        let port0 = rbuf[0];
        if port0 & IoExpPort0::Sw1.bits() == 0 {
            true
        } else {
            false
        }
    }

    /// Read the state of Switch Button 2
    pub fn read_sw2(&mut self) -> bool {
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf);
        let port0 = rbuf[0];
        if port0 & IoExpPort0::Sw2.bits() == 0 {
            true
        } else {
            false
        }
    }

    /// Read the state of Switch Button 3
    pub fn read_sw3(&mut self) -> bool {
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf);
        let port0 = rbuf[0];
        if port0 & IoExpPort0::Sw3.bits() == 0 {
            true
        } else {
            false
        }
    }

    /// Read the state of Switch Button 4
    pub fn read_sw4(&mut self) -> bool {
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf);
        let port0 = rbuf[0];
        if port0 & IoExpPort0::Sw4.bits() == 0 {
            true
        } else {
            false
        }
    }

    /// Read the state of Switch Button 5
    pub fn read_sw5(&self) -> bool {
        if self.button.pin.is_high() {
            true
        } else {
            false
        }
    }

    /// Turn on the buzzer with specified duty cycle
    pub fn buzz_on(&mut self, duty: u16) {
        self.buzzer_channel.pin.set_duty_cycle(duty).unwrap();
    }

    /// Turn off the buzzer
    pub fn buzz_off(&mut self) {
        self.buzzer_channel.pin.set_duty_cycle_fully_off().unwrap();
    }

    /// Read the position of Slide Switch 6
    pub fn read_slide_sw6_position(&mut self) -> SwPos {
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[IN_PORT2], &mut rbuf);
        let port0 = rbuf[0];
        if port0 & IoExpPort2::Sw6Pos1.bits() == 0 {
            SwPos::Up
        } else if port0 & IoExpPort2::Sw6Pos2.bits() == 0 {
            SwPos::Down
        } else {
            SwPos::Undefined
        }
    }

    /// Read the position of Slide Switch 7
    pub fn read_slide_sw7_position(&mut self) -> SwPos {
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[IN_PORT0], &mut rbuf);
        let port0 = rbuf[0];
        if port0 & IoExpPort0::Sw7Pos1.bits() == 0 {
            SwPos::Up
        } else if port0 & IoExpPort0::Sw7Pos2.bits() == 0 {
            SwPos::Down
        } else {
            SwPos::Undefined
        }
    }

    /// Set the uFerris RTC time
    pub fn set_rtc_time(&mut self, year: u16, month: u8, day: u8, hour: u8, min: u8, sec: u8) {
        let year_u8 = (year - 2000) as u8;
        let sec_bcd = to_bcd(sec);
        let min_bcd = to_bcd(min);
        let hour_bcd = to_bcd(hour);
        let day_bcd = to_bcd(day);
        let weekday_bcd = to_bcd(5);
        let month_bcd = to_bcd(month);
        let year_bcd = to_bcd(year_u8);
        self.i2c_write(
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
        );
    }

    /// Read the uFerris RTC time
    pub fn read_rtc_time(&mut self) -> (u16, u8, u8, u8, u8, u8) {
        // read time from RTC
        let mut buf = [0u8; 7];
        self.i2c_write_read(RTC_ADDR, &[0x00], &mut buf);
        let sec = from_bcd(buf[0] & 0x7F);
        let min = from_bcd(buf[1] & 0x7F);
        let hour = from_bcd(buf[2] & 0x3F);
        let day = from_bcd(buf[4] & 0x3F);
        let month = from_bcd(buf[5] & 0x1F);
        let year = from_bcd(buf[6]) as u16 + 2000;
        (year, month, day, hour, min, sec)
    }

    /// Write a value to a specific digit on the seven-segment display
    /// Passing a `None` value will turn off/blank the digit
    pub fn write_seven_segment_digit(&mut self, digit: SevenSegDigit, value: Option<u8>) {
        // Activate the Specified Digit
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[OUT_PORT2], &mut rbuf);
        let current = rbuf[0];

        let digits = match digit {
            SevenSegDigit::Digit1 => IoExpPort2::Digit1.bits(),
            SevenSegDigit::Digit2 => IoExpPort2::Digit2.bits(),
            SevenSegDigit::Digit3 => IoExpPort2::Digit3.bits(),
            SevenSegDigit::Digit4 => IoExpPort2::Digit4.bits(),
        };

        let new = current | digits;
        self.i2c_write(TCA6424_ADDR, &[OUT_PORT2, new]);

        match value {
            Some(v) => {
                // Write value to segments A-G
                let segment_map = match v {
                    0 => 0b00111111,
                    1 => 0b00000110,
                    2 => 0b01011011,
                    3 => 0b01001111,
                    4 => 0b01100110,
                    5 => 0b01101101,
                    6 => 0b01111101,
                    7 => 0b00000111,
                    8 => 0b01111111,
                    9 => 0b01101111,
                    _ => 0b00000000, // Blank for invalid values
                };

                self.i2c_write(TCA6424_ADDR, &[OUT_PORT1, segment_map]);
            }
            None => {
                // Turn off all segments
                self.i2c_write(TCA6424_ADDR, &[OUT_PORT1, 0x00]);
            }
        }
    }

    /// Enables the colon on the seven-segment display
    pub fn seven_segment_display_colon_en(&mut self, enable: bool) {
        // Activate the Relevant Digits
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[OUT_PORT2], &mut rbuf);
        let current = rbuf[0];
        let digits = IoExpPort2::Digit2.or(IoExpPort2::Digit4).bits();
        let new = current | digits;
        self.i2c_write(TCA6424_ADDR, &[OUT_PORT2, new]);

        // Activate Colons
        let mut rbuf = [0u8];
        self.i2c_write_read(TCA6424_ADDR, &[OUT_PORT1], &mut rbuf);
        let current = rbuf[0];
        let new = if enable {
            current | IoExpPort1::Dp.bits()
        } else {
            current & !IoExpPort1::Dp.bits()
        };
        self.i2c_write(TCA6424_ADDR, &[OUT_PORT1, new]);
    }

    /// Perform a raw I2C write operation
    pub fn i2c_write(&mut self, addr: u8, data: &[u8]) {
        self.i2c_devices.shared_device.write(addr, data).unwrap();
    }

    /// Perform a raw I2C read operation
    pub fn i2c_read(&mut self, addr: u8, buffer: &mut [u8]) {
        self.i2c_devices.shared_device.read(addr, buffer).unwrap();
    }

    /// Perform a raw I2C write-read operation
    pub fn i2c_write_read(&mut self, addr: u8, data: &[u8], buffer: &mut [u8]) {
        self.i2c_devices
            .shared_device
            .write_read(addr, data, buffer)
            .unwrap();
    }

    /// Perform a raw SPI write operation
    /// This method is only available if the `power-board` feature is enabled
    #[cfg(feature = "power-board")]
    pub fn spi_write(&mut self, _data: &[u8]) {}

    /// Perform a raw SPI read operation
    /// This method is only available if the `power-board` feature is enabled
    #[cfg(feature = "power-board")]
    pub fn spi_read(&mut self, _buffer: &mut [u8]) {
        // read data from SPI device
    }

    /// Initialize the SD card
    /// Method returns the size of the SD Card in bytes
    /// This method is only available if the `power-board` feature is enabled
    #[cfg(feature = "power-board")]
    fn init_sd_card(&mut self) -> u64 {
        // Take ownership of the SdCard from spi_devices
        // This leaves 'self.spi_devices.sd_card_device' as None
        let sd_card = self.spi_devices.sd_card_device.take().unwrap();

        // Initialize card
        let num_bytes = sd_card.num_bytes().unwrap();

        // Instantiate VolumeManager
        let vol_mgr = VolumeManager::new(sd_card, DummyTimeSource::default());

        // Store VolumeManager in the uFerris struct
        self.vol_mgr = Some(vol_mgr);

        num_bytes
    }

    /// Read a file from the root directory of the SD card
    /// This method is only available if the `power-board` feature is enabled
    #[cfg(feature = "power-board")]
    pub fn read_file_chunked<F>(&mut self, filename: &str, mut func: F)
    where
        F: FnMut(&[u8]), // closure trait
    {
        use embedded_sdmmc::{Mode, VolumeIdx};

        // Get the Volume Manager
        let vol_mgr = self.vol_mgr.as_mut().unwrap();

        // Open Volume and Directory
        let volume = vol_mgr.open_volume(VolumeIdx(0)).unwrap();
        let root_dir = volume.open_root_dir().unwrap();

        // Open the file
        let my_file = root_dir.open_file_in_dir(filename, Mode::ReadOnly).unwrap();

        // Read chunks and feed them to the closure
        let mut buffer = [0u8; 32];
        while !my_file.is_eof() {
            let bytes_read = my_file.read(&mut buffer).unwrap();

            // Pass the bytes that were read (0..bytes_read)
            if bytes_read > 0 {
                func(&buffer[0..bytes_read]);
            }
        }
    }

    /// Write to a file in the root directory of the SD card
    /// This method is only available if the `power-board` feature is enabled
    #[cfg(feature = "power-board")]
    pub fn write_to_file_in_root(&mut self, filename: &str, data: &[u8]) {
        // Methods to check if SD card is inserted

        use embedded_sdmmc::VolumeIdx;

        // Get the Volume Manager
        let vol_mgr = self.vol_mgr.as_mut().unwrap();

        // Open Volume and Directory
        let volume = vol_mgr.open_volume(VolumeIdx(0)).unwrap();
        let root_dir = volume.open_root_dir().unwrap();

        let my_file = root_dir
            .open_file_in_dir(filename, embedded_sdmmc::Mode::ReadWriteCreateOrTruncate)
            .unwrap();

        if let Ok(()) = my_file.write(data) {
            my_file.flush().unwrap();
        }
    }

    /// Read the System Voltage from the INA219
    /// This method is only available if the `power-board` feature is enabled
    #[cfg(feature = "power-board")]
    pub fn read_system_voltage(&mut self) -> Option<u16> {
        match self.i2c_devices.ina219_device.next_measurement() {
            Ok(measurement) => {
                if let Some(value) = measurement {
                    Some(value.bus_voltage.voltage_mv())
                } else {
                    None
                }
            }
            Err(_) => None,
        };
        None
    }

    /// Read the System Current from the INA219
    /// This method is only available if the `power-board` feature is enabled
    #[cfg(feature = "power-board")]
    pub fn read_system_current(&mut self) -> Option<i64> {
        match self.i2c_devices.ina219_device.next_measurement() {
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

    /// Read the System Power from the INA219
    /// This method is only available if the `power-board` feature is enabled
    #[cfg(feature = "power-board")]
    pub fn read_system_power(&mut self) -> Option<i64> {
        match self.i2c_devices.ina219_device.next_measurement() {
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
}
