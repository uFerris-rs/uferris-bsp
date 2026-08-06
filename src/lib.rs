#![no_std]
#![doc(html_logo_url = "https://i.imgur.com/gAPf1TI.png")]
#![doc(html_favicon_url = "https://i.imgur.com/L8Y0m57.png")]

//! # uFerris Board Support Package Crate
//!
//! <div align="center">
//!     <img src="https://i.imgur.com/KcvXhPw.png"
//!         width="300"
//!         style="margin-top: 40px; margin-bottom: 40px;"
//!     />
//! </div>
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
//! The uFerris BSP architechture follows the layered scheme shown in the figure below. The upper uFerris board logic layer is meant to provide a hardware agnositc uniform interface across all Xiao controllers.
//! The second adapter layer is introduced to create the mappings between the logic and the individual device HALs. The adapter layer also utilizes the `embedded-hal` traits where possible. In most cases the controller HALs provide implementations for `embedded-hal` traits.
//!
//! <div align="center">
//!     <img src="https://i.imgur.com/SD77pGl.png"
//!         width="500"
//!         style="margin-top: 40px; margin-bottom: 40px;"
//!     />
//! </div>
//!
//! ## Currently Supported Xiaos:
//! - Xiao ESP32-C3
//! - Xiao ESP32-C5 (buzzer stubbed - no PWM driver in `esp-hal` yet)
//! - Xiao ESP32-C6
//! - Xiao ESP32-S3
//! - Xiao nRF52840 (and nRF52840 Sense)
//! - Xiao nRF54L15 (and nRF54L15 Sense)
//! - Xiao RP2040
//! - Xiao RP2350
//!
//! ## `async` Support
//! The `async` feature enables the `async` board API. [`Uferris`] carries a
//! [`Mode`] type parameter that selects which set of methods it exposes:
//! [`Blocking`], the default, is the API described above and is what every
//! existing program already gets, and `Async` is the same board with the I2C
//! and ADC operations turned into `async fn`s plus `Uferris::wait_for_sw5`,
//! which suspends until button 5 is pressed instead of spinning on it.
//!
//! The executor and the time driver are the application's responsibility. The
//! BSP starts neither: it hands back a board whose methods are futures, and the
//! program decides what runs them and where its delays come from. `async`
//! support is rolling out per board: a board opts in by exposing an
//! `uferris_init_async` alongside its blocking `uferris_init`, and the Xiao
//! nRF52840, nRF54L15, RP2040 and RP2350 have one. The ESP boards are
//! blocking-only for now.
//!
//! ## Contributing to the uFerris BSP - Adding a New Xiao Board Support:
//! Adding support for a new Xiao board entails two parts:
//! 1. **Device Feature Flag in `Cargo.toml`**: A feature flag that imports the new device HAL needs to be added.
//! 2. **Device Board Adapter**: This entails adding a new board definition (adapter layer) under the crate `boards/` folder.
//!
//! Other files in the crate should remain unchanged.
//! It is recommended to view the existing board implementations for guidance on creating an adapter layer.
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
//!
//! ## Usage
//! The abstractions in this crate are designed in a way where they are common for any Xiao device.
//! The only difference is that the correct controller board needs to be chosen as a feature.
//! The steps to use this crate include the following:
//!
//! 1- Import the board init function:
//! ```
//! use uferris_bsp::uferris_init;
//! ```
//!
//! 2- Acquire the controller peripherals and pass them to initialize the board:
//! ```
//! let mut uferris = uferris_init(peripherals);
//! ```
//!
//! 3- Use the board methods:
//! ```
//! // Turn on LED 1 on the board
//! uferris.led1_on();
//! ```
//!
//! If no device feature is enabled, only the generic board API is available —
//! `uferris_init` requires selecting your Xiao's feature. This is what code
//! written against the board API alone, without a controller in the picture,
//! builds against.
//!

use core::fmt;
use core::marker::PhantomData; // Added this import
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;
use embedded_hal::pwm::SetDutyCycle;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

// `Mode` is the crate's own board mode typestate, so the `embedded-sdmmc` file
// open mode is brought in under a name of its own.
#[cfg(feature = "power-board")]
use embedded_sdmmc::{
    BlockDevice, Mode as FileMode, TimeSource, Timestamp, VolumeIdx, VolumeManager,
};
#[cfg(feature = "power-board")]
use ina219::{SyncIna219, address::Address, calibration::IntCalibration};

// Export generic components
pub mod components;
pub use components::io_expander::SwPos;

// Export the specific board implementation based on features
pub mod boards;

// Re-exports
pub use crate::components::io_expander::SevenSegDigit;
#[cfg(feature = "xiao-esp32c3")]
pub use boards::xiao_esp32c3::uferris_init;
#[cfg(feature = "xiao-esp32c5")]
pub use boards::xiao_esp32c5::uferris_init;
#[cfg(feature = "xiao-esp32c6")]
pub use boards::xiao_esp32c6::uferris_init;
#[cfg(feature = "xiao-esp32s3")]
pub use boards::xiao_esp32s3::uferris_init;
#[cfg(feature = "xiao-nrf54l15")]
pub use boards::xiao_nrf54l15::uferris_init;
#[cfg(all(feature = "xiao-nrf54l15", feature = "async"))]
pub use boards::xiao_nrf54l15::uferris_init_async;
#[cfg(feature = "xiao-nrf52840")]
pub use boards::xiao_nrf52840::uferris_init;
#[cfg(all(feature = "xiao-nrf52840", feature = "async"))]
pub use boards::xiao_nrf52840::uferris_init_async;
#[cfg(feature = "xiao-rp2040")]
pub use boards::xiao_rp2040::uferris_init;
#[cfg(all(feature = "xiao-rp2040", feature = "async"))]
pub use boards::xiao_rp2040::uferris_init_async;
#[cfg(feature = "xiao-rp2350")]
pub use boards::xiao_rp2350::uferris_init;
#[cfg(all(feature = "xiao-rp2350", feature = "async"))]
pub use boards::xiao_rp2350::uferris_init_async;

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
// Board Mode Typestate
// ------------------------------------------

mod sealed {
    pub trait Sealed {}
}

/// Which flavour of the board API a [`Uferris`] exposes.
///
/// This is a typestate: it carries no data and exists only as the last type
/// parameter of [`Uferris`], where it selects between the two `impl` blocks.
/// [`Blocking`] is the default, so `Uferris<..>` written without it means what
/// it has always meant, and `Async` turns every operation that talks to the
/// I2C bus or the ADC into an `async fn`.
///
/// The trait is sealed: the two modes below are the only ones there are, and a
/// board is only ever built by a board adapter.
///
/// The associated types park the power board fields per mode. The `async` power
/// board API is not implemented yet, so under `Async` both of them are `()`
/// and the fields are present but empty; under [`Blocking`] they are the
/// concrete driver types they have always been, which is what keeps the
/// blocking API unchanged.
pub trait Mode: sealed::Sealed {
    /// Type of the [`Uferris::vol_mgr`] field in this mode.
    #[cfg(feature = "power-board")]
    type VolMgr<BD: PowerConstraints>;

    /// Type of the [`Uferris::power_monitor`] field in this mode.
    #[cfg(feature = "power-board")]
    type Ina<I2C>;
}

/// The blocking board API: every method returns its result directly.
///
/// This is the default mode and the one every board adapter's `uferris_init`
/// hands back.
pub struct Blocking;

impl sealed::Sealed for Blocking {}

impl Mode for Blocking {
    #[cfg(feature = "power-board")]
    type VolMgr<BD: PowerConstraints> = Option<VolumeManager<BD, DummyTimeSource>>;

    #[cfg(feature = "power-board")]
    type Ina<I2C> = SyncIna219<I2C, IntCalibration>;
}

/// The `async` board API: the I2C and ADC operations are `async fn`s.
///
/// A board in this mode is built by a board adapter's `uferris_init_async`. The
/// executor that polls the resulting futures, and the time driver behind any
/// delay the program uses, are the application's to bring.
#[cfg(feature = "async")]
pub struct Async;

#[cfg(feature = "async")]
impl sealed::Sealed for Async {}

#[cfg(feature = "async")]
impl Mode for Async {
    // The power board is blocking-only for now: see [`Mode`].
    #[cfg(feature = "power-board")]
    type VolMgr<BD: PowerConstraints> = ();

    #[cfg(feature = "power-board")]
    type Ina<I2C> = ();
}

// ------------------------------------------
// Constants & Errors
// ------------------------------------------
#[cfg(feature = "power-board")]
const INA219_ADDR: u8 = 0x45;

#[derive(Debug, Clone)]
pub enum InitError {
    IoExpander,
    Rtc,
    PowerMonitor,
    SdCard,
}

impl fmt::Display for InitError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            InitError::IoExpander => write!(f, "I/O expander init failed (I2C)"),
            InitError::Rtc => write!(f, "RTC init failed (I2C)"),
            InitError::PowerMonitor => write!(f, "INA219 power monitor init failed (I2C)"),
            InitError::SdCard => write!(f, "SD card init failed (SPI)"),
        }
    }
}

// ------------------------------------------
// Generic Board Struct
// ------------------------------------------

/// The uFerris Board Driver.
///
/// `M` selects the board API: see [`Mode`]. It defaults to [`Blocking`], so
/// `Uferris<LED, BTN, BUZZ, I2C, ADC, BD>` still names the blocking board.
///
/// The struct itself carries no bounds beyond the ones its fields need — the
/// `OutputPin`/`InputPin`/`I2c`/... bounds live on the `impl` blocks, where they
/// describe what each mode's methods actually require rather than gating the
/// type as a whole. `BD: PowerConstraints` is the exception and has to stay: it
/// is what makes the `M::VolMgr<BD>` field type well formed under
/// `power-board`, where [`PowerConstraints`] is `BlockDevice`. Without that
/// feature it is a blanket-implemented marker and constrains nothing.
pub struct Uferris<LED, BTN, BUZZ, I2C, ADC, BD, M: Mode = Blocking>
where
    BD: PowerConstraints,
{
    pub led1: components::led::Led<LED>,
    pub sw_btn5: components::button::Button<BTN, M>,
    pub buzzer: components::buzzer::Buzzer<BUZZ>,
    pub ldr: ADC,
    pub expander: components::io_expander::IoExpander<I2C, M>,
    pub rtc: components::rtc::Rtc<I2C, M>,
    pub i2c: I2C,
    #[cfg(feature = "power-board")]
    pub vol_mgr: M::VolMgr<BD>,
    #[cfg(feature = "power-board")]
    pub power_monitor: M::Ina<I2C>,
    // This phantom member uses BD (block Device) when the power-board feature is
    // disabled, and carries the mode marker in every configuration.
    pub _phantom: PhantomData<(BD, M)>,
}

impl<LED, BTN, BUZZ, I2C, ADC, BD> Uferris<LED, BTN, BUZZ, I2C, ADC, BD, Blocking>
where
    LED: OutputPin,
    BTN: InputPin,
    BUZZ: SetDutyCycle,
    I2C: I2c,
    ADC: components::ldr::OneShot,
    BD: PowerConstraints,
{
    /// Create a new generic uFerris board instance.
    // Only the board adapters call this, so with no device feature enabled
    // there is no caller and the compiler would otherwise flag it as dead.
    #[cfg_attr(
        not(any(
            feature = "xiao-esp32c3",
            feature = "xiao-esp32c5",
            feature = "xiao-esp32c6",
            feature = "xiao-esp32s3",
            feature = "xiao-nrf52840",
            feature = "xiao-nrf54l15",
            feature = "xiao-rp2040",
            feature = "xiao-rp2350"
        )),
        allow(dead_code)
    )]
    fn new(
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

        expander.init().map_err(|_| InitError::IoExpander)?;

        let led1 = components::led::Led { pin: led1_pin };
        let sw_btn5 = components::button::Button::new(sw_btn5_pin);
        let buzzer = components::buzzer::Buzzer { pin: pwm_pin };

        #[cfg(feature = "power-board")]
        let power_monitor = {
            let calib =
                IntCalibration::new(ina219::calibration::MicroAmpere(1000), 100_000).unwrap();

            SyncIna219::new_calibrated(ina_i2c, Address::from_byte(INA219_ADDR).unwrap(), calib)
                .map_err(|_| InitError::PowerMonitor)?
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

    /// Turn on LED 1
    pub fn led1_on(&mut self) {
        let _ = self.led1.pin.set_high();
    }

    /// Turn off LED 1
    pub fn led1_off(&mut self) {
        let _ = self.led1.pin.set_low();
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
        self.sw_btn5.pin.is_low().unwrap_or(false)
    }

    /// Read Slide Switch 6
    pub fn read_sw6(&mut self) -> Result<SwPos, I2C::Error> {
        self.expander.read_slide_sw6_position()
    }

    /// Read Slide Switch 7
    pub fn read_sw7(&mut self) -> Result<SwPos, I2C::Error> {
        self.expander.read_slide_sw7_position()
    }

    /// Write a Digit to the Seven Segment Display
    pub fn write_seven_segment_digit(
        &mut self,
        digit: SevenSegDigit,
        value: Option<u8>,
    ) -> Result<(), I2C::Error> {
        self.expander.write_seven_segment_digit(digit, value)
    }

    /// Activate/Deeactivate the Seven Segment Display Colon
    pub fn seven_segment_display_colon_en(&mut self, enable: bool) -> Result<(), I2C::Error> {
        self.expander.seven_segment_display_colon_en(enable)
    }

    /// Read LDR Value (12-bit Resolution)
    pub fn read_ldr(&mut self) -> u16 {
        self.ldr.read_raw()
    }

    /// Turn on Buzzer wit a Duty Cycle Value (0-100)
    pub fn buzz_on(&mut self, duty: u16) {
        let _ = self.buzzer.pin.set_duty_cycle(duty);
    }

    /// Turn off Buzzer
    pub fn buzz_off(&mut self) {
        let _ = self.buzzer.pin.set_duty_cycle_fully_off();
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
    pub fn read_system_current(&mut self) -> Option<i64> {
        match self.power_monitor.next_measurement() {
            Ok(measurement) => measurement.map(|value| value.current.0),
            Err(_) => None,
        }
    }

    /// Read the System Power in milliWatts
    #[cfg(feature = "power-board")]
    pub fn read_system_power(&mut self) -> Option<i64> {
        match self.power_monitor.next_measurement() {
            Ok(measurement) => measurement.map(|value| value.power.0),
            Err(_) => None,
        }
    }

    /// Initialize / Check SD Card
    /// Returns the size of the SD card in bytes if detected.
    #[cfg(feature = "power-board")]
    pub fn init_sd_card(&mut self) -> Result<u64, InitError> {
        let mgr = self.vol_mgr.as_mut().ok_or(InitError::SdCard)?;

        // Temporary variable to hold the size
        let mut size_bytes = 0u64;

        mgr.device(|dev| {
            if let Ok(num_blocks) = dev.num_blocks() {
                size_bytes = num_blocks.0 as u64 * 512;
            }
            crate::DummyTimeSource::default()
        });

        if size_bytes == 0 {
            return Err(InitError::SdCard);
        }

        Ok(size_bytes)
    }

    /// Write data to a file in the root directory
    /// Creates the file if it doesn't exist, or truncates it if it does.
    #[cfg(feature = "power-board")]
    pub fn write_to_file_in_root(&mut self, filename: &str, data: &[u8]) {
        let mgr = self.vol_mgr.as_mut().expect("VolMgr missing");

        // Open volume and directory
        if let Ok(volume) = mgr.open_volume(VolumeIdx(0)) {
            if let Ok(root_dir) = volume.open_root_dir() {
                // 'file' is dropped/closed immediately after write
                if let Ok(file) =
                    root_dir.open_file_in_dir(filename, FileMode::ReadWriteCreateOrTruncate)
                {
                    let _ = file.write(data);
                    let _ = file.flush();
                } // end of 'file' scope
            } // end of 'root_dir' scope
        } // end of 'volume' scope
    }

    /// Read a file from the root directory in chunks
    #[cfg(feature = "power-board")]
    pub fn read_file_chunked<F>(&mut self, name: &str, mut f: F) -> Result<(), InitError>
    where
        F: FnMut(&[u8]),
    {
        let mgr = self.vol_mgr.as_mut().ok_or(InitError::SdCard)?;

        let volume = mgr
            .open_volume(VolumeIdx(0))
            .map_err(|_| InitError::SdCard)?;
        let root_dir = volume.open_root_dir().map_err(|_| InitError::SdCard)?;
        let file = root_dir
            .open_file_in_dir(name, FileMode::ReadOnly)
            .map_err(|_| InitError::SdCard)?;

        let mut buffer = [0u8; 64];
        while !file.is_eof() {
            let bytes_read = file.read(&mut buffer).map_err(|_| InitError::SdCard)?;
            if bytes_read > 0 {
                f(&buffer[..bytes_read]);
            }
        }
        Ok(())
    }
}

// ------------------------------------------
// Generic Board Struct - `async` Mode
// ------------------------------------------

/// The `async` board API.
///
/// Every method that has to reach the I2C bus or the ADC is an `async fn` here,
/// and [`Uferris::wait_for_sw5`] replaces the `read_sw5` spin loop with a wait.
/// The methods that drive a pin directly — LED 1 and the buzzer — stay
/// synchronous, because `OutputPin` and `SetDutyCycle` are blocking traits in
/// both worlds: writing a GPIO or a PWM duty cycle is a register store and has
/// nothing to await.
///
/// The power board API is not part of this mode yet. Under `power-board` the
/// `vol_mgr` and `power_monitor` fields are still there, parked as `()` by
/// [`Mode`], and the SD card and INA219 methods are blocking-only.
#[cfg(feature = "async")]
impl<LED, BTN, BUZZ, I2C, ADC, BD> Uferris<LED, BTN, BUZZ, I2C, ADC, BD, Async>
where
    LED: OutputPin,
    BTN: embedded_hal_async::digital::Wait,
    BUZZ: SetDutyCycle,
    I2C: AsyncI2c,
    ADC: components::ldr::OneShotAsync,
    BD: PowerConstraints,
{
    /// Create a new generic uFerris board instance in `async` mode.
    // Only the board adapters with an `uferris_init_async` call this, so unless
    // one of those device features is enabled there is no caller and the
    // compiler would otherwise flag it as dead. `async` support is rolling out
    // per board: add a board to this list when its adapter gains an
    // `uferris_init_async`.
    //
    // Named `new_async` rather than `new`: the mode parameter of the board a
    // call site is building is only pinned by the type it is being assigned or
    // returned into, which is too late for method resolution, so two inherent
    // `new`s would leave every adapter's call ambiguous.
    #[cfg_attr(
        not(any(
            feature = "xiao-nrf52840",
            feature = "xiao-nrf54l15",
            feature = "xiao-rp2040",
            feature = "xiao-rp2350"
        )),
        allow(dead_code)
    )]
    async fn new_async(
        led1_pin: LED,
        sw_btn5_pin: BTN,
        pwm_pin: BUZZ,
        ldr_driver: ADC,
        expander_i2c: I2C,
        rtc_i2c: I2C,
        raw_i2c: I2C,
    ) -> Result<Self, InitError> {
        let mut expander = components::io_expander::IoExpander::new(expander_i2c);
        let rtc = components::rtc::Rtc::new(rtc_i2c);

        expander.init().await.map_err(|_| InitError::IoExpander)?;

        let led1 = components::led::Led { pin: led1_pin };
        let sw_btn5 = components::button::Button::new(sw_btn5_pin);
        let buzzer = components::buzzer::Buzzer { pin: pwm_pin };

        Ok(Self {
            led1,
            sw_btn5,
            buzzer,
            ldr: ldr_driver,
            expander,
            rtc,
            i2c: raw_i2c,
            #[cfg(feature = "power-board")]
            vol_mgr: (),
            #[cfg(feature = "power-board")]
            power_monitor: (),
            _phantom: PhantomData,
        })
    }

    /// Turn on LED 1
    pub fn led1_on(&mut self) {
        let _ = self.led1.pin.set_high();
    }

    /// Turn off LED 1
    pub fn led1_off(&mut self) {
        let _ = self.led1.pin.set_low();
    }

    /// Turn on LED 2
    pub async fn led2_on(&mut self) -> Result<(), I2C::Error> {
        self.expander.led2_on().await
    }

    /// Turn off LED 2
    pub async fn led2_off(&mut self) -> Result<(), I2C::Error> {
        self.expander.led2_off().await
    }

    /// Turn on LED 3
    pub async fn led3_on(&mut self) -> Result<(), I2C::Error> {
        self.expander.led3_on().await
    }

    /// Turn off LED 3
    pub async fn led3_off(&mut self) -> Result<(), I2C::Error> {
        self.expander.led3_off().await
    }

    /// Read Button Switch 1
    pub async fn read_sw1(&mut self) -> Result<bool, I2C::Error> {
        self.expander.read_sw1().await
    }

    /// Read Button Switch 2
    pub async fn read_sw2(&mut self) -> Result<bool, I2C::Error> {
        self.expander.read_sw2().await
    }

    /// Read Button Switch 3
    pub async fn read_sw3(&mut self) -> Result<bool, I2C::Error> {
        self.expander.read_sw3().await
    }

    /// Read Button Switch 4
    pub async fn read_sw4(&mut self) -> Result<bool, I2C::Error> {
        self.expander.read_sw4().await
    }

    /// Wait until Button Switch 5 is pressed.
    ///
    /// Button 5 is the one push button wired straight to the controller rather
    /// than to the I/O expander, so it is the one the board can wait on instead
    /// of poll. The button is active low — the same reading the blocking
    /// `read_sw5` reports as pressed — so this waits for the pin to be low.
    ///
    /// The wait is on the level, not on the edge: if the button is already held
    /// down when this is called it returns immediately, exactly as a
    /// `while !uferris.read_sw5() {}` loop would fall straight through.
    pub async fn wait_for_sw5(&mut self) {
        self.sw_btn5.wait_for_press().await;
    }

    /// Read Slide Switch 6
    pub async fn read_sw6(&mut self) -> Result<SwPos, I2C::Error> {
        self.expander.read_slide_sw6_position().await
    }

    /// Read Slide Switch 7
    pub async fn read_sw7(&mut self) -> Result<SwPos, I2C::Error> {
        self.expander.read_slide_sw7_position().await
    }

    /// Write a Digit to the Seven Segment Display
    pub async fn write_seven_segment_digit(
        &mut self,
        digit: SevenSegDigit,
        value: Option<u8>,
    ) -> Result<(), I2C::Error> {
        self.expander.write_seven_segment_digit(digit, value).await
    }

    /// Activate/Deeactivate the Seven Segment Display Colon
    pub async fn seven_segment_display_colon_en(&mut self, enable: bool) -> Result<(), I2C::Error> {
        self.expander.seven_segment_display_colon_en(enable).await
    }

    /// Read LDR Value (12-bit Resolution)
    pub async fn read_ldr(&mut self) -> u16 {
        self.ldr.read_raw().await
    }

    /// Turn on Buzzer wit a Duty Cycle Value (0-100)
    pub fn buzz_on(&mut self, duty: u16) {
        let _ = self.buzzer.pin.set_duty_cycle(duty);
    }

    /// Turn off Buzzer
    pub fn buzz_off(&mut self) {
        let _ = self.buzzer.pin.set_duty_cycle_fully_off();
    }

    /// Set the uFerris RTC time
    pub async fn set_rtc_time(
        &mut self,
        year: u16,
        month: u8,
        day: u8,
        hour: u8,
        min: u8,
        sec: u8,
    ) -> Result<(), I2C::Error> {
        self.rtc.set_time(year, month, day, hour, min, sec).await
    }

    /// Read the uFerris RTC time
    pub async fn read_rtc_time(&mut self) -> Result<(u16, u8, u8, u8, u8, u8), I2C::Error> {
        self.rtc.read_time().await
    }

    /// Perform a raw I2C write operation
    pub async fn i2c_write(&mut self, addr: u8, data: &[u8]) -> Result<(), I2C::Error> {
        self.i2c.write(addr, data).await
    }

    /// Perform a raw I2C read operation
    pub async fn i2c_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), I2C::Error> {
        self.i2c.read(addr, buffer).await
    }

    /// Perform a raw I2C write-read operation
    pub async fn i2c_write_read(
        &mut self,
        addr: u8,
        data: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), I2C::Error> {
        self.i2c.write_read(addr, data, buffer).await
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
