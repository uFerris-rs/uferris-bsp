use embedded_hal_bus::i2c::RefCellDevice;
#[cfg(feature = "power-board")]
use embedded_hal_bus::spi::RefCellDevice as SpiRefCellDevice;
use esp_hal::Blocking;
use esp_hal::analog::adc::{Adc, AdcConfig};
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::ledc::LowSpeed;
use esp_hal::ledc::channel::Channel;
#[cfg(feature = "power-board")]
use esp_hal::spi::master::Spi;
use ina219::SyncIna219;
use ina219::address::Address;
use ina219::calibration::{IntCalibration, MicroAmpere};

use crate::INA219_ADDR;
use crate::components::button::Button;
use crate::components::buzzer::Buzzer;
use crate::components::i2cdevices::I2cDevices;
use crate::components::ldr::{Ldr, OneShot};
use crate::components::led::Led;
use crate::components::spidevices::SpiDevices;
use crate::pins::XiaoPins;

use embedded_sdmmc::SdCard;

use super::Uferris;

pub struct XiaoPeripherals<'d> {
    adc1: esp_hal::peripherals::ADC1<'d>,
    shared_i2c_device: RefCellDevice<'d, I2c<'d, Blocking>>,
    ina219_i2c_device: RefCellDevice<'d, I2c<'d, Blocking>>,
    buzzer_channel: Channel<'d, LowSpeed>,
    #[cfg(feature = "power-board")]
    shared_spi_device: SpiRefCellDevice<'d, Spi<'d, Blocking>, Output<'d>, Delay>,
    #[cfg(feature = "power-board")]
    sd_card_device: SpiRefCellDevice<'d, Spi<'d, Blocking>, Output<'d>, Delay>,
}

impl<'d> Uferris<'d> {
    /// Create a new instance of the uFerris baseboard
    /// This method configures all the components on the board
    /// If the `power-board` feature is enabled, a SPI peripheral is also required
    pub fn new(pins: XiaoPins<'d>, peripherals: XiaoPeripherals<'d>) -> Self {
        //-----------------------------------
        // LED 1 Config
        //-----------------------------------

        // Configure D1 Pin connected to LED1
        let led = Output::new(pins.d1.unwrap(), Level::Low, OutputConfig::default());

        //-----------------------------------
        // Switch Button 5 Config
        //-----------------------------------

        // Configure D3 Pin connected to Switch Button 5
        let button = Input::new(pins.d3.unwrap(), InputConfig::default());

        //-----------------------------------
        // ADC Config
        //-----------------------------------

        // Configure ADC1 for LDR on D0
        let mut adc1_config = AdcConfig::new();
        let analog_pin =
            adc1_config.enable_pin(pins.d0.unwrap(), esp_hal::analog::adc::Attenuation::_11dB);
        let adc1 = Adc::new(peripherals.adc1, adc1_config);

        //-----------------------------------
        // SDCard Device Config
        //-----------------------------------

        let delay = Delay::new();
        let sd_card_device = SdCard::new(peripherals.sd_card_device, delay);

        //-----------------------------------
        // INA219 I2C Device Config
        //-----------------------------------

        // Resolution of 1mA, and a shunt of 100mOhm
        let calib = IntCalibration::new(MicroAmpere(1_000), 1_000_000).unwrap();

        // Create instance for INA device
        let ina219_device = SyncIna219::new_calibrated(
            peripherals.ina219_i2c_device,
            Address::from_byte(INA219_ADDR).unwrap(),
            calib,
        )
        .unwrap();

        Self {
            led1: Led { pin: led },
            button: Button { pin: button },
            ldr: Ldr {
                adc: adc1,
                pin: analog_pin,
            },
            buzzer_channel: Buzzer {
                pin: peripherals.buzzer_channel,
            },
            i2c_devices: I2cDevices {
                shared_device: peripherals.shared_i2c_device,
                ina219_device,
            },
            #[cfg(feature = "power-board")]
            spi_devices: SpiDevices {
                shared_device: peripherals.shared_spi_device,
                sd_card_device: Some(sd_card_device),
                // // sd_card_vol_mgr: sd_card_vol_mgr,
                // sd_card_vol_mgr_ref_cell,
            },
            vol_mgr: None,
        }
    }

    /// Read the raw LDR value
    pub fn read_ldr_raw(&mut self) -> u16 {
        self.ldr.read_raw().unwrap()
    }
}

/// Helper macro to initialize the pins and peripherals for the Xiao ESP32-C3 board
#[macro_export]
#[cfg(not(feature = "power-board"))]
macro_rules! init_esp32c3_xiao {
    (
        $peripherals:ident,
        $pins_var:ident,
        $xiao_peripherals:ident
    ) => {
        // 1. Setup LEDC driver
        let mut ledc = esp_hal::ledc::Ledc::new($peripherals.LEDC);
        ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);

        // 2. Create & configure timer (for buzzer ~2-4 kHz is usually good)
        let mut buzzer_timer =
            ledc.timer::<esp_hal::ledc::LowSpeed>(esp_hal::ledc::timer::Number::Timer0);

        buzzer_timer
            .configure(esp_hal::ledc::timer::config::Config {
                duty: esp_hal::ledc::timer::config::Duty::Duty10Bit, // 10 bits → 1024 levels
                clock_source: esp_hal::ledc::timer::LSClockSource::APBClk,
                frequency: esp_hal::time::Rate::from_khz(2500), // ← adjust for your buzzer
            })
            .unwrap();

        // 3. Create & configure channel
        let mut buzzer_channel =
            ledc.channel(esp_hal::ledc::channel::Number::Channel0, $peripherals.GPIO4);

        buzzer_channel
            .configure(esp_hal::ledc::channel::config::Config {
                timer: &buzzer_timer,
                duty_pct: 0, // start quiet
                drive_mode: DriveMode::PushPull,
            })
            .unwrap();

        // 4. Pins struct
        let $pins_var = XiaoPins {
            d0: Some($peripherals.GPIO2),
            d1: Some($peripherals.GPIO3),
            d2: None,
            d3: Some($peripherals.GPIO5),
            d4: Some($peripherals.GPIO6),
            d5: Some($peripherals.GPIO7),
            d6: Some($peripherals.GPIO21),
            d7: Some($peripherals.GPIO20),
            d8: Some($peripherals.GPIO8),
            d9: Some($peripherals.GPIO9),
            d10: Some($peripherals.GPIO10),
        };

        // Configure I2C on D4 (SDA) and D5 (SCL)
        // I2C connects to Qwiic conntector, RTC, 7-segment display, and IO Expander
        let i2c = I2c::new(
            peripherals.i2c0,
            esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_scl(pins.d5.unwrap())
        .with_sda(pins.d4.unwrap());

        // Create I2C bus reference
        let i2c_refcell = RefCell::new(i2c);

        // Obtain instance for ina219 i2c device
        let ina219_i2c_device = RefCellDevice::new(&i2c_refcell);

        // Obtain instance for shared i2c device
        let shared_i2c_device = RefCellDevice::new(&i2c_refcell);

        // 5. Xiao peripherals struct – move timer & channel in
        let $xiao_peripherals = XiaoPeripherals {
            adc1: $peripherals.ADC1,
            i2c0: $peripherals.I2C0,
            shared_i2c_device,
            ina219_i2c_device,
            buzzer_channel,
            spi0: $peripherals.SPI0,
        };
    };
}

/// Helper macro to initialize the pins and peripherals for the Xiao ESP32-C3 board
#[macro_export]
#[cfg(feature = "power-board")]
macro_rules! init_esp32c3_xiao {
    (
        $peripherals:ident,
        $pins_var:ident,
        $xiao_peripherals:ident
    ) => {
        // Setup LEDC driver
        let mut ledc = esp_hal::ledc::Ledc::new($peripherals.LEDC);
        ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);

        // Create & configure timer
        let mut buzzer_timer =
            ledc.timer::<esp_hal::ledc::LowSpeed>(esp_hal::ledc::timer::Number::Timer0);

        buzzer_timer
            .configure(esp_hal::ledc::timer::config::Config {
                duty: esp_hal::ledc::timer::config::Duty::Duty10Bit, // 10 bits → 1024 levels
                clock_source: esp_hal::ledc::timer::LSClockSource::APBClk,
                frequency: esp_hal::time::Rate::from_khz(2500), // ← adjust for your buzzer
            })
            .unwrap();

        // Create & configure channel
        let mut buzzer_channel =
            ledc.channel(esp_hal::ledc::channel::Number::Channel0, $peripherals.GPIO4);

        buzzer_channel
            .configure(esp_hal::ledc::channel::config::Config {
                timer: &buzzer_timer,
                duty_pct: 0, // start quiet
                drive_mode: DriveMode::PushPull,
            })
            .unwrap();

        // Pins struct
        let $pins_var = XiaoPins {
            d0: Some($peripherals.GPIO2),
            d1: Some($peripherals.GPIO3),
            d2: None,
            d3: Some($peripherals.GPIO5),
            d4: Some($peripherals.GPIO6),
            d5: Some($peripherals.GPIO7),
            d6: Some($peripherals.GPIO21),
            d7: Some($peripherals.GPIO20),
            d8: Some($peripherals.GPIO8),
            d9: Some($peripherals.GPIO9),
            d10: Some($peripherals.GPIO10),
        };

        // Configure I2C on D4 (SDA) and D5 (SCL)
        // I2C connects to Qwiic conntector, RTC, 7-segment display, and IO Expander
        let i2c = I2c::new(
            peripherals.i2c0,
            esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_scl(pins.d5.unwrap())
        .with_sda(pins.d4.unwrap());

        // Create I2C bus reference
        let i2c_refcell = RefCell::new(i2c);

        // Obtain instance for ina219 i2c device
        let ina219_i2c_device = RefCellDevice::new(&i2c_refcell);

        // Obtain instance for shared i2c device
        let shared_i2c_device = RefCellDevice::new(&i2c_refcell);

        let spi = Spi::new(
            peripherals.spi,
            esp_hal::spi::master::Config::default().with_frequency(Rate::from_mhz(10)),
        )
        .unwrap()
        .with_sck(pins.d8.unwrap())
        .with_miso(pins.d9.unwrap())
        .with_mosi(pins.d10.unwrap());

        // CS Pin for SD Card
        let sd_card_cs = Output::new(pins.d7.unwrap(), Level::High, OutputConfig::default());

        // Delay Implementation
        let delay = Delay::new();

        // Obtain instance for SDCard SPI device
        let sdcard_spi_device = SpiRefCellDevice::new(spi, cs, delay);

        // Optional CS Pin on D6 for Additional Device
        let optional_cs = Output::new(pins.d6.unwrap(), Level::High, OutputConfig::default());

        let shared_spi_device = SpiRefCellDevice::new(spi, optional_cs, delay);

        // // Create the SD Card Driver
        // let sd_card = SdCard::new(sdcard_spi_device, Delay::new());

        // // Create the Volume Manager immediately
        // let mut volume_mgr = VolumeManager::new(sd_card, DummyTimeSource::default());

        // Xiao peripherals struct – move timer & channel in
        let $xiao_peripherals = XiaoPeripherals {
            adc1: $peripherals.ADC1,
            i2c0: $peripherals.I2C0,
            shared_i2c_device,
            ina219_i2c_device,
            buzzer_channel,
            shared_spi_device,
            sdcard_spi_device,
            // sdcard_volume_mgr: volume_mgr,
        };
    };
}
