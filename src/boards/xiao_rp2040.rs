use core::cell::RefCell;
use embassy_rp::{
    Peripherals,
    adc::{Adc, Blocking as AdcBlocking, Channel as AdcChannel, Config as AdcConfig},
    gpio::{Input, Level, Output, Pull},
    i2c::{Blocking as I2cBlocking, Config as I2cConfig, I2c},
    peripherals::I2C1,
    pwm::{Config as PwmConfig, Pwm},
};
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use static_cell::StaticCell;

#[cfg(feature = "power-board")]
use embassy_rp::{
    peripherals::SPI0,
    spi::{Blocking as SpiBlocking, Config as SpiConfig, Spi},
};
#[cfg(feature = "power-board")]
use embedded_hal_bus::spi::RefCellDevice as SpiRefCellDevice;

use crate::Uferris;
use crate::components::ldr::OneShot;

// ------------------------------------------
// Second Stage Bootloader
// ------------------------------------------

// The RP2040 has no internal flash. Its boot ROM copies the first 256 bytes of
// the external QSPI flash into SRAM and runs them, and that second stage is
// what configures the flash for XIP.
//
// `embassy-rp` supplies that stage itself: with the `rp2040` feature on and no
// `boot2-*` feature selected it places `BOOT_LOADER_W25Q080` in `.boot2`, which
// is the correct second stage for the Winbond W25Q class part the Xiao RP2040
// carries. The BSP therefore declares no bootloader of its own, and the section
// is placed by `link-rp.x`, the linker script fragment `embassy-rp` emits for
// the RP2040. Applications add it to their rustflags alongside `link.x`.

// ------------------------------------------
// Board Constants
// ------------------------------------------

/// System clock frequency produced by [`embassy_rp::init`] with the default
/// [`embassy_rp::config::Config`].
///
/// That configuration runs the system PLL off the 12 MHz crystal fitted on the
/// Xiao RP2040 with `refdiv` 1, `fbdiv` 125 and post dividers 6 and 2, so
/// `12 MHz * 125 / 12 = 125 MHz`, and the system clock divider is 1.
const SYS_CLOCK_HZ: u32 = 125_000_000;

/// I2C bus frequency shared by the io expander, the RTC and the power monitor.
const I2C_FREQ_HZ: u32 = 100_000;

/// SPI bus frequency used to talk to the SD card on the power board.
#[cfg(feature = "power-board")]
const SPI_FREQ_HZ: u32 = 400_000;

// ------------------------------------------
// Static Types
// ------------------------------------------
static I2C_BUS: StaticCell<RefCell<Rp2040I2c>> = StaticCell::new();
#[cfg(feature = "power-board")]
static SPI_BUS: StaticCell<RefCell<Rp2040Spi>> = StaticCell::new();

// ------------------------------------------
// Type Defs
// ------------------------------------------

// I2C Types
type Rp2040I2c = I2c<'static, I2C1, I2cBlocking>;
type SharedI2c = I2cRefCellDevice<'static, Rp2040I2c>;

// GPIO Types
type LedPin = Output<'static>;
type ButtonPin = Input<'static>;

// ADC Types
pub struct LdrAdc {
    adc: Adc<'static, AdcBlocking>,
    channel: AdcChannel<'static>,
}

impl OneShot for LdrAdc {
    /// `embassy-rp` surfaces the conversion errors the RP2040 ADC can report.
    /// [`OneShot::read_raw`] has no error channel, so a failed conversion reads
    /// as 0, the same value a saturated-dark LDR produces.
    fn read_raw(&mut self) -> u16 {
        self.adc.blocking_read(&mut self.channel).unwrap_or(0)
    }
}

// Buzzer Types
pub type Rp2040BuzzerChannel = Pwm<'static>;

// SD/SPI Types
#[cfg(feature = "power-board")]
type Rp2040Spi = Spi<'static, SPI0, SpiBlocking>;

#[cfg(feature = "power-board")]
type SdCsPin = Output<'static>;

#[cfg(feature = "power-board")]
type SdBlockDevice =
    embedded_sdmmc::SdCard<SpiRefCellDevice<'static, Rp2040Spi, SdCsPin, CycleDelay>, CycleDelay>;

// ------------------------------------------
// uFerris Board Type Alias
// ------------------------------------------
#[cfg(not(feature = "power-board"))]
pub type UferrisRp2040 = Uferris<
    LedPin,              // LED (D1)
    ButtonPin,           // Button (D3)
    Rp2040BuzzerChannel, // Buzzer (D2)
    SharedI2c,           // I2C
    LdrAdc,              // LDR
    (),
>;

#[cfg(feature = "power-board")]
pub type UferrisRp2040 = Uferris<
    LedPin,              // LED (D1)
    ButtonPin,           // Button (D3)
    Rp2040BuzzerChannel, // Buzzer (D2)
    SharedI2c,           // I2C
    LdrAdc,              // LDR
    SdBlockDevice,       // SD Manager
>;

// ------------------------------------------
// Delay Provider
// ------------------------------------------

/// A busy loop delay, usable as an `embedded-hal` delay provider.
///
/// `embassy-rp` only exposes the RP2040 timer through the `embassy-time`
/// driver, which needs an executor to be useful, so the blocking board adapter
/// counts CPU cycles instead. [`cortex_m::asm::delay`] runs one loop iteration
/// per requested cycle and every iteration costs at least one cycle, so the
/// requested time is a lower bound: on the Cortex-M0+ the loop takes about
/// three cycles per iteration, and a delay can therefore run up to roughly
/// three times long. That is the right side to err on for the SD card timings
/// that need it, but it does mean the delays are coarse. An accurate timer
/// backed delay arrives with `async` support, when `embassy-time` comes along
/// anyway.
///
/// The type is a zero sized `Copy` marker, so every caller gets its own handle
/// and it can back both delay slots of a shared SPI device at once.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CycleDelay;

impl DelayNs for CycleDelay {
    fn delay_ns(&mut self, ns: u32) {
        // `u32::MAX` ns at 125 MHz is about 5.4e8 cycles, so the count always
        // fits back into the `u32` `asm::delay` takes.
        let cycles = u64::from(ns) * u64::from(SYS_CLOCK_HZ) / 1_000_000_000;
        cortex_m::asm::delay(cycles as u32);
    }
}

/// A [`CycleDelay`] handle.
///
/// It needs no board state, so it is callable before [`uferris_init`].
pub const fn delay() -> CycleDelay {
    CycleDelay
}

// ------------------------------------------
// Board Initialization Function
// ------------------------------------------

/// Initialize the uFerris board.
///
/// `embassy-rp` is used in blocking mode: no executor, no time driver and no
/// `async` anywhere. [`embassy_rp::init`] sets up the clock tree and hands back
/// the peripheral singletons, and every driver built here is one of the
/// crate's blocking constructors, all of which implement the `embedded-hal`
/// 1.0 blocking traits the board logic is written against.
///
/// There is no console on this board yet. The Xiao RP2040 has no USB-to-UART
/// bridge, so printing would mean driving a USB CDC device, and the
/// `embassy-rp` USB driver is `async` only. Applications therefore run silent
/// until `async` support lands.
pub fn uferris_init(peripherals: Peripherals) -> UferrisRp2040 {
    // --------------------------------------
    //              ADC Setup
    // --------------------------------------
    let adc = Adc::new_blocking(peripherals.ADC, AdcConfig::default());
    // Floating, matching the other boards: the LDR sits in a divider on the
    // uFerris carrier board, so no internal pull is configured here.
    let ldr_channel = AdcChannel::new_pin(peripherals.PIN_26, Pull::None);
    let ldr_driver = LdrAdc {
        adc,
        channel: ldr_channel,
    };

    // --------------------------------------
    //              I2C Setup
    // --------------------------------------
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = I2C_FREQ_HZ;

    let i2c = I2c::new_blocking(
        peripherals.I2C1,
        peripherals.PIN_7,
        peripherals.PIN_6,
        i2c_config,
    );

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
    let led = Output::new(peripherals.PIN_27, Level::Low);
    // Floating input, matching the other boards: the button is wired active low
    // against a pull-up on the uFerris carrier board, so no internal pull is
    // configured here.
    let button = Input::new(peripherals.PIN_29, Pull::None);

    // --------------------------------------
    //              PWM Setup
    // --------------------------------------

    // The buzzer sits on D2 / GPIO28, which is slice 6 channel A.
    //
    // The RP2040 PWM output frequency is
    //     f_pwm = f_sys / (DIV * (TOP + 1))
    // where DIV is an 8.4 fixed point value.
    //
    // `embassy-rp` reports `max_duty_cycle()` as TOP rather than TOP + 1, so
    // TOP is 16384 here to keep the 14 bit resolution the LEDC based boards
    // use. Duty values written by application code therefore mean the same
    // thing on every board.
    //
    // The default clock configuration leaves the RP2040 system clock at
    // 125 MHz, so hitting the 2700 Hz used by the other boards needs
    //     DIV = 125_000_000 / (2700 * 16385) = 2.825
    // and the nearest representable value is 2 + 13/16 = 2.8125, giving
    //     f_pwm = 125_000_000 / (2.8125 * 16385) = 2713 Hz.
    let mut pwm_config = PwmConfig::default();
    pwm_config.top = 16384;
    // `divider` is the 8.4 fixed point value above and defaults to 1, so it is
    // scaled by 45/16 to reach 2 + 13/16 without naming the `fixed` types.
    pwm_config.divider = pwm_config.divider * 45 / 16;

    let buzzer_channel = Pwm::new_output_a(peripherals.PWM_SLICE6, peripherals.PIN_28, pwm_config);

    // --------------------------------------
    //            SPI / SD Setup
    // --------------------------------------
    #[cfg(feature = "power-board")]
    let vol_mgr = {
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = SPI_FREQ_HZ;
        // `SpiConfig` defaults to mode 0, which is what the SD card wants.

        let spi = Spi::new_blocking(
            peripherals.SPI0,
            peripherals.PIN_2,
            peripherals.PIN_3,
            peripherals.PIN_4,
            spi_config,
        );

        // Promote SPI Bus to Static
        let spi_bus_ref = SPI_BUS.init(RefCell::new(spi));

        // CS Pin
        let sd_cs = Output::new(peripherals.PIN_1, Level::High);

        // Create SPI Device (Borrows from SPI_BUS static)
        // We do NOT need to make this device static. SdCard owns it.
        // `CycleDelay` is `Copy`, so the same instance backs both delay slots.
        let sd_device = SpiRefCellDevice::new(spi_bus_ref, sd_cs, delay()).unwrap();

        // Create SD Card (Owns sd_device)
        let sd_card = embedded_sdmmc::SdCard::new(sd_device, delay());

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
