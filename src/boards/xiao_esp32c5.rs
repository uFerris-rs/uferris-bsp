use core::cell::RefCell;
use embedded_hal::pwm::{ErrorType as PwmErrorType, SetDutyCycle};
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcPin, Attenuation},
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    i2c::master::I2c,
    peripherals::{GPIO1, Peripherals},
    time::Rate,
};
use static_cell::StaticCell;

#[cfg(feature = "async")]
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice as I2cAsyncDevice;
#[cfg(feature = "async")]
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex as AsyncMutex};
#[cfg(feature = "async")]
use esp_hal::peripherals::{FROM_CPU_INTR0, TIMG0};

#[cfg(feature = "power-board")]
use embedded_hal_bus::spi::RefCellDevice as SpiRefCellDevice;
#[cfg(feature = "power-board")]
use esp_hal::delay::Delay;
#[cfg(feature = "power-board")]
use esp_hal::spi::master::Spi;

#[cfg(feature = "async")]
use crate::Async;
use crate::Uferris;
#[cfg(feature = "async")]
use crate::components::io_expander::IoExpander;
use crate::components::ldr::OneShot;
#[cfg(feature = "async")]
use crate::components::ldr::OneShotAsync;

// ------------------------------------------
// Static Types
// ------------------------------------------
static I2C_BUS: StaticCell<RefCell<EspI2c>> = StaticCell::new();
#[cfg(feature = "power-board")]
static SPI_BUS: StaticCell<RefCell<EspSpi>> = StaticCell::new();

/// The `async` board's I2C bus.
///
/// A second cell rather than a second use of [`I2C_BUS`], because the two board
/// modes share the bus differently: the blocking one hands out `RefCell`
/// borrows, the `async` one hands out futures that have to be able to suspend
/// mid-transaction and so needs an async-aware mutex. Only one of the two init
/// functions can ever run — both consume the [`Peripherals`] singleton — so the
/// cell the other mode would have used simply stays uninitialized.
#[cfg(feature = "async")]
static ASYNC_I2C_BUS: StaticCell<AsyncMutex<NoopRawMutex, EspAsyncI2c>> = StaticCell::new();

// ------------------------------------------
// Type Defs
// ------------------------------------------

// I2C Types
type EspI2c = I2c<'static, esp_hal::Blocking>;
type SharedI2c = I2cRefCellDevice<'static, EspI2c>;

/// The `async` board's I2C driver.
///
/// A different type from [`EspI2c`], not just a different way of using it:
/// `esp-hal` puts the mode in the driver's type parameter, and only
/// `I2c<.., Async>` implements `embedded_hal_async::i2c::I2c`. The driver is
/// built blocking and moved over with `I2c::into_async`, which installs the
/// peripheral's interrupt handler.
#[cfg(feature = "async")]
type EspAsyncI2c = I2c<'static, esp_hal::Async>;

/// One handle onto the `async` board's shared I2C bus.
///
/// [`NoopRawMutex`] is the right raw mutex here: every one of the board's I2C
/// devices is driven from the same executor on the one core the board uses, so
/// the bus is never contended from an interrupt or from a second core.
#[cfg(feature = "async")]
type SharedAsyncI2c = I2cAsyncDevice<'static, NoopRawMutex, EspAsyncI2c>;

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

/// The LDR channel of the ADC, sampled one conversion at a time, for the
/// `async` board.
///
/// `Adc<.., Async>` is a distinct type from the blocking one — the mode is a
/// type parameter on the driver — so this is a wrapper of its own rather than a
/// second `impl` on [`LdrAdc`]. The pin type is shared between the two.
#[cfg(feature = "async")]
pub struct LdrAdcAsync<'d> {
    adc: Adc<'d, esp_hal::peripherals::ADC1<'d>, esp_hal::Async>,
    pin: AdcPin<GPIO1<'d>, esp_hal::peripherals::ADC1<'d>>,
}

#[cfg(feature = "async")]
impl OneShotAsync for LdrAdcAsync<'_> {
    /// The `async` counterpart of the blocking [`OneShot`] implementation:
    /// `Adc::read_oneshot` starts a single conversion and suspends on the
    /// SARADC interrupt instead of spinning on the ready flag. It hands back
    /// the reading directly rather than through `nb`, so there is no error case
    /// to fold away here.
    async fn read_raw(&mut self) -> u16 {
        self.adc.read_oneshot(&mut self.pin).await
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

/// The `async` uFerris board on this controller, as returned by
/// [`uferris_init_async`].
#[cfg(all(feature = "async", not(feature = "power-board")))]
pub type UferrisEsp32Async = Uferris<
    Output<'static>,      // LED (D1)
    Input<'static>,       // Button (D3)
    StubBuzzer,           // Buzzer (D2) - no PWM driver on ESP32-C5 yet
    SharedAsyncI2c,       // I2C
    LdrAdcAsync<'static>, // LDR
    (),
    Async,
>;

/// The `async` uFerris board on this controller, as returned by
/// [`uferris_init_async`].
///
/// The power board is blocking-only for now, so the block device parameter here
/// only names the type the blocking board would have used: under `Async` the
/// `vol_mgr` and `power_monitor` fields are parked as `()` and
/// [`uferris_init_async`] never touches the SPI bus or the INA219. See
/// [`crate::Mode`].
#[cfg(all(feature = "async", feature = "power-board"))]
pub type UferrisEsp32Async = Uferris<
    Output<'static>,      // LED (D1)
    Input<'static>,       // Button (D3)
    StubBuzzer,           // Buzzer (D2) - no PWM driver on ESP32-C5 yet
    SharedAsyncI2c,       // I2C
    LdrAdcAsync<'static>, // LDR
    SdBlockDevice,        // SD Manager (unused in `async` mode)
    Async,
>;

// ------------------------------------------
// Scheduler Peripherals
// ------------------------------------------

/// The peripherals an `esp-rtos` application starts its scheduler from, handed
/// back by [`uferris_init_async`].
///
/// The runtime belongs to the application on every uFerris board, and on the
/// ESP boards that runs into an ownership problem the others do not have.
/// `esp_rtos::start` is fed a timer and a software interrupt, both of which
/// come out of the one [`Peripherals`] struct that [`uferris_init_async`] takes
/// by value, and a struct that has had a field moved out of it can no longer be
/// passed on. An application therefore cannot start the scheduler first. It
/// gets these two back instead, and starts it immediately afterwards:
///
/// ```ignore
/// let (mut uferris, rtos_parts) = uferris_init_async(peripherals);
///
/// let timg0 = TimerGroup::new(rtos_parts.timg0);
/// esp_rtos::start(timg0.timer0, rtos_parts.sw_interrupt);
/// ```
///
/// The BSP does not depend on `esp-rtos` and never will: these are plain
/// `esp-hal` peripherals, and what an application builds out of them is its own
/// business. `esp-rtos` is only the runtime the uFerris examples happen to use.
#[cfg(feature = "async")]
pub struct EspRtosParts {
    /// Timer group 0. `TimerGroup::new(timg0).timer0` is the tick
    /// `esp_rtos::start` wants; the board itself uses no timer group.
    pub timg0: TIMG0<'static>,
    /// Software interrupt 0. It is the context switch signal
    /// `esp_rtos::start` wants; the board uses no software interrupt.
    pub sw_interrupt: FROM_CPU_INTR0<'static>,
}

// ------------------------------------------
// Board Initialization Function
// ------------------------------------------
pub fn uferris_init(peripherals: Peripherals) -> UferrisEsp32 {
    // The `async` feature is a pure code gate: it selects the `async` board API
    // and starts nothing. Whatever scheduler and time driver an application
    // wants — `esp-rtos` and its embassy integration, say — is the
    // application's to start. This is the blocking board, and it is the same
    // board whether or not that feature is on; `uferris_init_async` at the
    // bottom of this file builds the other one.

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

// ------------------------------------------
// Board Initialization Function - `async`
// ------------------------------------------

/// Initialize the uFerris board in `async` mode.
///
/// The counterpart of [`uferris_init`]. It takes the same [`Peripherals`] and
/// wires up the same pins, but moves the I2C driver and the ADC into `esp-hal`'s
/// `async` mode, so that the board's I2C and ADC operations are futures:
/// both peripherals are driven through their interrupts rather than polled,
/// and the button is an [`Input`] the board can wait on. The two init functions
/// are mutually exclusive — each consumes the peripheral singletons — so a
/// program calls one or the other.
///
/// The executor is the application's, and so is the time driver: this adapter
/// hands out no `async` delay, and a program that wants one takes
/// `embassy_time`'s, backed by whatever its runtime installs.
///
/// # Why this one is not a future
///
/// Every non-ESP uFerris board's `uferris_init_async` is an `async fn`. This
/// one is not, and it hands back an [`EspRtosParts`] next to the board. Both
/// come from the same constraint: on the ESP boards the runtime is started from
/// peripherals that live in the [`Peripherals`] struct this function consumes
/// whole, so it cannot already be running when this is called, and `esp-rtos`'s
/// thread mode executor will not suspend a task before its scheduler is
/// started. Nothing here may await, therefore, and an application has to be
/// able to start the scheduler before its own first `.await`. See
/// [`EspRtosParts`] for the two lines that does.
///
/// The one thing the other boards await during init is configuring the I/O
/// expander, which is done here over the blocking I2C driver before it is moved
/// into `async` mode — the same registers with the same values, polled rather
/// than awaited, and over a few hundred microseconds of bus traffic at 100 kHz.
///
/// Waiting on the button needs nothing extra: `esp-hal` implements
/// `embedded_hal_async::digital::Wait` for [`Input`] unconditionally, and
/// [`esp_hal::init`] binds the GPIO interrupt's default handler itself.
///
/// The power board is not part of the `async` board yet: the SPI bus and the
/// INA219 are left alone here and the corresponding fields are parked. See
/// [`UferrisEsp32Async`].
#[cfg(feature = "async")]
pub fn uferris_init_async(peripherals: Peripherals) -> (UferrisEsp32Async, EspRtosParts) {
    // --------------------------------------
    //         Scheduler Peripherals
    // --------------------------------------

    // Held back first, while `peripherals` is still whole. Neither of these is
    // used by the board.
    let rtos_parts = EspRtosParts {
        timg0: peripherals.TIMG0,
        sw_interrupt: peripherals.FROM_CPU_INTR0,
    };

    // --------------------------------------
    //              ADC Setup
    // --------------------------------------

    // The same pin and the same attenuation as in `uferris_init`.
    // `Adc::into_async` installs the SARADC interrupt handler, which is what
    // lets a conversion suspend rather than spin.
    let mut adc_config = AdcConfig::new();
    let ldr_pin = adc_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
    let adc1 = Adc::new(peripherals.ADC1, adc_config).into_async();
    let ldr_driver = LdrAdcAsync {
        adc: adc1,
        pin: ldr_pin,
    };

    // --------------------------------------
    //              I2C Setup
    // --------------------------------------
    let mut i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_scl(peripherals.GPIO24)
    .with_sda(peripherals.GPIO23);

    // Configure the I/O expander while the driver is still blocking and still
    // exclusively owned: see this function's docs for why it cannot be done
    // afterwards. `IoExpander` borrows the bus for the duration rather than
    // taking it, so the driver is intact when this returns.
    IoExpander::<_, crate::Blocking>::new(&mut i2c)
        .init()
        .unwrap();

    // Move the driver into `async` mode. This installs the I2C peripheral's
    // interrupt handler; the bus itself, and the expander configuration just
    // written over it, are untouched.
    let i2c = i2c.into_async();

    // Promote I2C Bus to Static
    let i2c_bus_ref = ASYNC_I2C_BUS.init(AsyncMutex::new(i2c));

    // Device Instances
    let expander_i2c = I2cAsyncDevice::new(i2c_bus_ref);
    let rtc_i2c = I2cAsyncDevice::new(i2c_bus_ref);
    let raw_i2c = I2cAsyncDevice::new(i2c_bus_ref);

    // --------------------------------------
    //              GPIO Setup
    // --------------------------------------
    let led = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
    // This is also the `Input` `wait_for_sw5` waits on.
    let button = Input::new(peripherals.GPIO7, InputConfig::default());

    // --------------------------------------
    //              Buzzer Setup
    // --------------------------------------
    // Identical to `uferris_init`: `SetDutyCycle` is a blocking trait in both
    // modes, and there is no PWM driver to reach for on this chip either way.
    // See `StubBuzzer`.
    let buzzer_channel = StubBuzzer {
        _pin: Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default()),
    };

    // --------------------------------------
    //          Board Instantiation
    // --------------------------------------

    // `new_async_preinit` rather than `new_async`: the expander was configured
    // above, over the blocking driver.
    let uferris = Uferris::new_async_preinit(
        led,
        button,
        buzzer_channel,
        ldr_driver,
        expander_i2c,
        rtc_i2c,
        raw_i2c,
    );

    (uferris, rtos_parts)
}
