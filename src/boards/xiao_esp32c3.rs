// src/boards/xiao_esp32c3.rs
#![cfg(feature = "xiao-esp32c3")]

use core::cell::RefCell;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcPin, Attenuation},
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    i2c::master::I2c,
    ledc::{
        LowSpeed,
        channel::{Channel, ChannelIFace},
        timer::{self, Timer, TimerIFace},
    },
    peripherals::{GPIO2, Peripherals},
    time::Rate,
    timer::timg::TimerGroup,
};
use static_cell::StaticCell;

use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;

#[cfg(feature = "power-board")]
use embedded_hal_bus::spi::RefCellDevice as SpiRefCellDevice;
#[cfg(feature = "power-board")]
use esp_hal::spi::master::Spi;

use crate::Uferris;
use crate::components::ldr::OneShot;

// ------------------------------------------
// Static Types
// ------------------------------------------
static BUZZER_TIMER: StaticCell<Timer<'static, LowSpeed>> = StaticCell::new();
static I2C_BUS: StaticCell<RefCell<EspI2c>> = StaticCell::new();
#[cfg(feature = "power-board")]
static SPI_BUS: StaticCell<RefCell<EspSpi>> = StaticCell::new();

// ------------------------------------------
// Type Defs
// ------------------------------------------

// I2C Types
type EspI2c = I2c<'static, esp_hal::Blocking>;
type SharedI2c = I2cRefCellDevice<'static, EspI2c>;

// ADC Types
pub struct LdrAdc<'d> {
    adc: Adc<'d, esp_hal::peripherals::ADC1<'d>, esp_hal::Blocking>,
    pin: AdcPin<GPIO2<'d>, esp_hal::peripherals::ADC1<'d>>,
}

impl<'d> OneShot for LdrAdc<'d> {
    fn read_raw(&mut self) -> u16 {
        nb::block!(self.adc.read_oneshot(&mut self.pin)).unwrap_or(0)
    }
}

// Buzzer Types
pub type EspBuzzerChannel = Channel<'static, LowSpeed>;

// SD/SPI Types
#[cfg(feature = "power-board")]
type EspSpi = Spi<'static, esp_hal::Blocking>;

#[cfg(feature = "power-board")]
// The concrete type for the generic BD parameter
// This entire stack is owned by Uferris, but it holds a reference to SPI_BUS
type SdBlockDevice =
    embedded_sdmmc::SdCard<SpiRefCellDevice<'static, EspSpi, Output<'static>, Delay>, Delay>;

// ------------------------------------------
// uFerris Board Type Alias
// ------------------------------------------
#[cfg(not(feature = "power-board"))]
pub type UferrisEsp32 = Uferris<
    Output<'static>,  // LED (D1)
    Input<'static>,   // Button (D3)
    EspBuzzerChannel, // Buzzer (D4)
    SharedI2c,        // I2C
    LdrAdc<'static>,  // LDR
    (),
>;

#[cfg(feature = "power-board")]
pub type UferrisEsp32 = Uferris<
    Output<'static>,  // LED (D1)
    Input<'static>,   // Button (D3)
    EspBuzzerChannel, // Buzzer (D4)
    SharedI2c,        // I2C
    LdrAdc<'static>,  // LDR
    SdBlockDevice,    // SD Manager
>;

// ------------------------------------------
// Board Initialization Function
// ------------------------------------------
pub fn uferris_init(peripherals: Peripherals) -> UferrisEsp32 {
    // --------------------------------------
    //            Embassy Setup
    // Needs to be feature gated
    // --------------------------------------
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    // --------------------------------------
    //              ADC Setup
    // --------------------------------------
    let mut adc_config = AdcConfig::new();
    let ldr_pin = adc_config.enable_pin(peripherals.GPIO2, Attenuation::_11dB);
    let adc1 = Adc::new(peripherals.ADC1, adc_config);
    let ldr_driver = LdrAdc {
        adc: adc1,
        pin: ldr_pin,
    };

    // --------------------------------------
    //              I2C Setup
    // --------------------------------------
    let i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_scl(peripherals.GPIO7)
    .with_sda(peripherals.GPIO6);

    // Promote I2C Bus to Static
    let i2c_bus_ref = I2C_BUS.init(RefCell::new(i2c));

    // Device Instances
    let expander_i2c = I2cRefCellDevice::new(i2c_bus_ref);
    let rtc_i2c = I2cRefCellDevice::new(i2c_bus_ref);
    let raw_i2c = I2cRefCellDevice::new(i2c_bus_ref);
    #[cfg(feature = "power-board")]
    let ina_i2c = I2cRefCellDevice::new(i2c_bus_ref);

    // --------------------------------------
    //              GPIO Setup
    // --------------------------------------
    let led = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    let button = Input::new(peripherals.GPIO5, InputConfig::default());

    // --------------------------------------
    //            LEDC / PWM Setup
    // --------------------------------------
    let mut ledc = esp_hal::ledc::Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);

    let mut buzzer_timer =
        ledc.timer::<esp_hal::ledc::LowSpeed>(esp_hal::ledc::timer::Number::Timer0);

    buzzer_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(2700u32),
        })
        .unwrap();

    // Promote Timer to Static (Channel borrows this)
    let buzzer_timer_ref = BUZZER_TIMER.init(buzzer_timer);

    let mut buzzer_channel =
        ledc.channel(esp_hal::ledc::channel::Number::Channel0, peripherals.GPIO4);

    buzzer_channel
        .configure(esp_hal::ledc::channel::config::Config {
            timer: buzzer_timer_ref,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .unwrap();

    // --------------------------------------
    //            SPI / SD Setup
    // --------------------------------------
    #[cfg(feature = "power-board")]
    let vol_mgr = {
        let spi = Spi::new(
            peripherals.SPI2,
            esp_hal::spi::master::Config::default().with_frequency(Rate::from_mhz(10)),
        )
        .unwrap()
        .with_sck(peripherals.GPIO8)
        .with_miso(peripherals.GPIO9)
        .with_mosi(peripherals.GPIO10);

        // Promote SPI Bus to Static
        let spi_bus_ref = SPI_BUS.init(RefCell::new(spi));

        // CS Pin
        let sd_cs = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());

        // Create SPI Device (Borrows from SPI_BUS static)
        // We do NOT need to make this device static. SdCard owns it.
        let sd_device = SpiRefCellDevice::new(spi_bus_ref, sd_cs, delay).unwrap();

        // Create SD Card (Owns sd_device)
        let sd_card = embedded_sdmmc::SdCard::new(sd_device, delay);

        Some(embedded_sdmmc::VolumeManager::new(
            sd_card,
            crate::DummyTimeSource::default(),
        ))
    };

    // --------------------------------------
    //          Board Instantiation
    // --------------------------------------
    Uferris::new(
        led,
        button,
        buzzer_channel,
        ldr_driver,
        expander_i2c,
        rtc_i2c,
        raw_i2c,
        #[cfg(feature = "power-board")]
        vol_mgr,
        #[cfg(feature = "power-board")]
        ina_i2c,
    )
    .unwrap()
}
