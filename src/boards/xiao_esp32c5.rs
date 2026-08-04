use core::cell::RefCell;
use embedded_hal::pwm::{ErrorType as PwmErrorType, SetDutyCycle};
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
#[cfg(feature = "embassy")]
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcPin, Attenuation},
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    i2c::master::I2c,
    peripherals::{GPIO1, Peripherals},
    time::Rate,
};
use static_cell::StaticCell;

#[cfg(feature = "power-board")]
use embedded_hal_bus::spi::RefCellDevice as SpiRefCellDevice;
#[cfg(feature = "power-board")]
use esp_hal::delay::Delay;
#[cfg(feature = "power-board")]
use esp_hal::spi::master::Spi;

use crate::Uferris;
use crate::components::ldr::OneShot;

// ------------------------------------------
// Static Types
// ------------------------------------------
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
    pin: AdcPin<GPIO1<'d>, esp_hal::peripherals::ADC1<'d>>,
}

impl<'d> OneShot for LdrAdc<'d> {
    fn read_raw(&mut self) -> u16 {
        nb::block!(self.adc.read_oneshot(&mut self.pin)).unwrap_or(0)
    }
}

// Buzzer Types

/// Duty cycle range reported by [`StubBuzzer`].
///
/// Mirrors the 14-bit LEDC resolution (`2^14`) used by the other supported
/// boards so that application code written against them keeps working.
const STUB_BUZZER_MAX_DUTY: u16 = 16384;

/// Placeholder buzzer driver for the Xiao ESP32-C5.
///
/// Hardware PWM (LEDC/MCPWM) is **not yet supported on the ESP32-C5** in
/// `esp-hal` (see `esp-rs/esp-hal` issues #5161 and #5154). Until a driver
/// lands upstream, the buzzer is non-functional on this board.
///
/// This type keeps the buzzer pin (D2 / GPIO25) held as a plain low output so
/// the buzzer stays silent, and implements [`SetDutyCycle`] as a no-op so that
/// the generic uFerris board API stays identical across boards. Calls to
/// `buzz_on`/`buzz_off` are accepted and silently discarded.
pub struct StubBuzzer {
    _pin: Output<'static>,
}

impl PwmErrorType for StubBuzzer {
    type Error = core::convert::Infallible;
}

impl SetDutyCycle for StubBuzzer {
    fn max_duty_cycle(&self) -> u16 {
        STUB_BUZZER_MAX_DUTY
    }

    fn set_duty_cycle(&mut self, _duty: u16) -> Result<(), Self::Error> {
        // No hardware PWM peripheral driver is available on the ESP32-C5 yet,
        // so the requested duty cycle is discarded and the pin stays low.
        Ok(())
    }
}

// SD/SPI Types
#[cfg(feature = "power-board")]
type EspSpi = Spi<'static, esp_hal::Blocking>;

#[cfg(feature = "power-board")]
type SdBlockDevice =
    embedded_sdmmc::SdCard<SpiRefCellDevice<'static, EspSpi, Output<'static>, Delay>, Delay>;

// ------------------------------------------
// uFerris Board Type Alias
// ------------------------------------------
#[cfg(not(feature = "power-board"))]
pub type UferrisEsp32 = Uferris<
    Output<'static>, // LED (D1)
    Input<'static>,  // Button (D3)
    StubBuzzer,      // Buzzer (D2) - no PWM driver on ESP32-C5 yet
    SharedI2c,       // I2C
    LdrAdc<'static>, // LDR
    (),
>;

#[cfg(feature = "power-board")]
pub type UferrisEsp32 = Uferris<
    Output<'static>, // LED (D1)
    Input<'static>,  // Button (D3)
    StubBuzzer,      // Buzzer (D2) - no PWM driver on ESP32-C5 yet
    SharedI2c,       // I2C
    LdrAdc<'static>, // LDR
    SdBlockDevice,   // SD Manager
>;

// ------------------------------------------
// Board Initialization Function
// ------------------------------------------
pub fn uferris_init(peripherals: Peripherals) -> UferrisEsp32 {
    // --------------------------------------
    //            Embassy Setup
    // --------------------------------------
    #[cfg(feature = "embassy")]
    {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let sw_interrupt =
            esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);
    }

    // --------------------------------------
    //              ADC Setup
    // --------------------------------------
    let mut adc_config = AdcConfig::new();
    let ldr_pin = adc_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
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
    .with_scl(peripherals.GPIO24)
    .with_sda(peripherals.GPIO23);

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
    let led = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
    let button = Input::new(peripherals.GPIO7, InputConfig::default());

    // --------------------------------------
    //              Buzzer Setup
    // --------------------------------------
    // The ESP32-C5 has no LEDC/MCPWM driver in esp-hal yet (esp-rs/esp-hal
    // issues #5161 and #5154), so the buzzer pin is only held low to keep the
    // buzzer silent. See `StubBuzzer`.
    let buzzer_channel = StubBuzzer {
        _pin: Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default()),
    };

    // --------------------------------------
    //            SPI / SD Setup
    // --------------------------------------
    #[cfg(feature = "power-board")]
    let vol_mgr = {
        let spi = Spi::new(
            peripherals.SPI2,
            esp_hal::spi::master::Config::default().with_frequency(Rate::from_khz(400)),
        )
        .unwrap()
        .with_sck(peripherals.GPIO8)
        .with_miso(peripherals.GPIO9)
        .with_mosi(peripherals.GPIO10);

        // Promote SPI Bus to Static
        let spi_bus_ref = SPI_BUS.init(RefCell::new(spi));

        // CS Pin
        let sd_cs = Output::new(peripherals.GPIO12, Level::High, OutputConfig::default());

        // Create Delay Instance
        let delay = Delay::new();

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
