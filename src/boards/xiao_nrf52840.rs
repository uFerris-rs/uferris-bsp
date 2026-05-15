use crate::Uferris;
use crate::components::ldr::OneShot;
use core::cell::RefCell;
use embassy_futures::block_on;
use embassy_nrf::Peripherals;
use embassy_nrf::bind_interrupts;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::peripherals;
use embassy_nrf::pwm::{DutyCycle, SimplePwm};
use embassy_nrf::saadc::{self, ChannelConfig, Saadc};
#[cfg(feature = "power-board")]
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::twim::{self, Twim};
use embedded_hal_bus::i2c::RefCellDevice;
use static_cell::ConstStaticCell;
use static_cell::StaticCell;

bind_interrupts!(struct I2cIrqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

bind_interrupts!(struct AdcIrqs {
    SAADC => saadc::InterruptHandler;
});

#[cfg(feature = "power-board")]
bind_interrupts!(struct SpiIrqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

static I2C0_GLOBAL: StaticCell<RefCell<Twim<'static>>> = StaticCell::new();
static RAM_BUFFER: ConstStaticCell<[u8; 16]> = ConstStaticCell::new([0; 16]);

#[cfg(feature = "power-board")]
static SPI_BUS: StaticCell<RefCell<Spim<'static>>> = StaticCell::new();

#[cfg(feature = "power-board")]
use embedded_hal_bus::spi::RefCellDevice as SpiRefCellDevice;

pub struct Adc {
    _adc: Saadc<'static, 1>,
}

impl OneShot for Adc {
    fn read_raw(&mut self) -> u16 {
        let mut buf = [0i16; 1];
        block_on(self._adc.sample(&mut buf));
        buf[0].max(0) as u16
    }
}

pub struct NoDelay;
impl embedded_hal::delay::DelayNs for NoDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

#[cfg(feature = "power-board")]
type NrfSpi = Spim<'static>;
#[cfg(feature = "power-board")]
type SdCs = Output<'static>;
#[cfg(feature = "power-board")]
type SdBlockDevice =
    embedded_sdmmc::SdCard<SpiRefCellDevice<'static, NrfSpi, SdCs, NoDelay>, NoDelay>;

use embedded_hal::pwm::{ErrorType, SetDutyCycle};

pub struct NrfPwmChannel {
    pwm: SimplePwm<'static>,
    channel: usize,
    max_duty: u16,
}

impl NrfPwmChannel {
    pub fn new(pwm: SimplePwm<'static>, channel: usize) -> Self {
        let max_duty = pwm.max_duty();
        Self {
            pwm,
            channel,
            max_duty,
        }
    }

    pub fn set_max_duty(&mut self, max_duty: u16) {
        self.max_duty = max_duty;
        self.pwm.set_max_duty(max_duty);
    }
}

impl ErrorType for NrfPwmChannel {
    type Error = core::convert::Infallible;
}

impl SetDutyCycle for NrfPwmChannel {
    fn max_duty_cycle(&self) -> u16 {
        self.max_duty
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let clamped = duty.min(self.max_duty);
        self.pwm.set_duty(self.channel, DutyCycle::normal(clamped));
        Ok(())
    }
}

#[cfg(not(feature = "power-board"))]
pub type Uferrisnrf52840 = Uferris<
    Output<'static>,
    Input<'static>,
    NrfPwmChannel,
    RefCellDevice<'static, Twim<'static>>,
    Adc,
    (),
>;

#[cfg(feature = "power-board")]
pub type Uferrisnrf52840 = Uferris<
    Output<'static>,
    Input<'static>,
    NrfPwmChannel,
    RefCellDevice<'static, Twim<'static>>,
    Adc,
    SdBlockDevice,
>;

pub fn uferris_init(p: Peripherals) -> Uferrisnrf52840 {
    let led1 = Output::new(p.P0_03, Level::Low, OutputDrive::Standard);
    let sw5 = Input::new(p.P0_29, Pull::Up);

    let pwm = SimplePwm::new_1ch(p.PWM0, p.P0_28, &Default::default());
    pwm.set_prescaler(embassy_nrf::pwm::Prescaler::Div1);
    pwm.set_max_duty(5926);
    pwm.enable();

    let pwm_channel = NrfPwmChannel::new(pwm, 0);

    let mut i2c_config = twim::Config::default();
    i2c_config.frequency = twim::Frequency::K100;
    let i2c = Twim::new(
        p.TWISPI0,
        I2cIrqs,
        p.P0_04,
        p.P0_05,
        i2c_config,
        RAM_BUFFER.take(),
    );

    let i2c_bus = I2C0_GLOBAL.init(RefCell::new(i2c));

    let expander_i2c = RefCellDevice::new(i2c_bus);
    let rtc_i2c = RefCellDevice::new(i2c_bus);
    let raw_i2c = RefCellDevice::new(i2c_bus);
    #[cfg(feature = "power-board")]
    let ina_i2c = RefCellDevice::new(i2c_bus);

    let adc_channel = ChannelConfig::single_ended(p.P0_02);
    let mut adc_config = saadc::Config::default();
    adc_config.resolution = saadc::Resolution::_12BIT;
    let adc_driver = Saadc::new(p.SAADC, AdcIrqs, adc_config, [adc_channel]);

    #[cfg(feature = "power-board")]
    let vol_mgr = {
        let mut spi_config = spim::Config::default();
        spi_config.frequency = spim::Frequency::K500;
        spi_config.mode = spim::MODE_0;
        spi_config.orc = 0xFF;

        let spi = Spim::new(
            p.SPI3, SpiIrqs, p.P1_13, // sck
            p.P1_14, // miso
            p.P1_15, // mosi
            spi_config,
        );

        let spi_bus = SPI_BUS.init(RefCell::new(spi));

        let sd_cs = Output::new(p.P1_12, Level::High, OutputDrive::Standard);

        let sd_device = SpiRefCellDevice::new(spi_bus, sd_cs, NoDelay).unwrap();
        let sd_card = embedded_sdmmc::SdCard::new(sd_device, NoDelay);

        Some(embedded_sdmmc::VolumeManager::new(
            sd_card,
            crate::DummyTimeSource::default(),
        ))
    };

    Uferris::new(
        led1,
        sw5,
        pwm_channel,
        Adc { _adc: adc_driver },
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
