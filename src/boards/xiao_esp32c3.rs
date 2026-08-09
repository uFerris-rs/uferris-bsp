use core::cell::RefCell;
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcPin, Attenuation},
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    i2c::master::I2c,
    ledc::{
        LowSpeed,
        channel::{Channel, ChannelIFace},
        timer::{self, Timer, TimerIFace},
    },
    peripherals::{GPIO2, Peripherals},
    time::Rate,
};
use static_cell::StaticCell;

#[cfg(feature = "async")]
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice as I2cAsyncDevice;
#[cfg(feature = "async")]
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex as AsyncMutex};
#[cfg(feature = "async")]
use esp_hal::peripherals::{SW_INTERRUPT, TIMG0};

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
static BUZZER_TIMER: StaticCell<Timer<'static, LowSpeed>> = StaticCell::new();
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
/// cell the other mode would have used simply stays uninitialized. The LEDC
/// timer cell above is shared between them for the same reason.
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
    pin: AdcPin<GPIO2<'d>, esp_hal::peripherals::ADC1<'d>>,
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
    pin: AdcPin<GPIO2<'d>, esp_hal::peripherals::ADC1<'d>>,
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
pub type EspBuzzerChannel = Channel<'static, LowSpeed>;

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

/// The `async` uFerris board on this controller, as returned by
/// [`uferris_init_async`].
#[cfg(all(feature = "async", not(feature = "power-board")))]
pub type UferrisEsp32Async = Uferris<
    Output<'static>,      // LED (D1)
    Input<'static>,       // Button (D3)
    EspBuzzerChannel,     // Buzzer (D4)
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
    EspBuzzerChannel,     // Buzzer (D4)
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
/// let sw_interrupt = SoftwareInterruptControl::new(rtos_parts.sw_interrupt);
/// esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);
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
    /// The software interrupt block.
    /// `SoftwareInterruptControl::new(sw_interrupt).software_interrupt0` is the
    /// context switch signal `esp_rtos::start` wants; the board uses none of
    /// the four.
    pub sw_interrupt: SW_INTERRUPT<'static>,
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
    .unwrap_or_else(|e| panic!("I2C bus init failed: {:?}", e))
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
        .unwrap_or_else(|e| panic!("Buzzer timer config failed: {:?}", e));

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
        .unwrap_or_else(|e| panic!("Buzzer channel config failed: {:?}", e));

    // --------------------------------------
    //            SPI / SD Setup
    // --------------------------------------
    #[cfg(feature = "power-board")]
    let vol_mgr = {
        let spi = Spi::new(
            peripherals.SPI2,
            esp_hal::spi::master::Config::default().with_frequency(Rate::from_khz(400)),
        )
        .unwrap_or_else(|e| panic!("SPI bus init failed: {:?}", e))
        .with_sck(peripherals.GPIO8)
        .with_miso(peripherals.GPIO9)
        .with_mosi(peripherals.GPIO10);

        // Promote SPI Bus to Static
        let spi_bus_ref = SPI_BUS.init(RefCell::new(spi));

        // CS Pin
        let sd_cs = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());

        // Create Delay Instance
        let delay = Delay::new();

        // Create SPI Device (Borrows from SPI_BUS static)
        // We do NOT need to make this device static. SdCard owns it.
        let sd_device = SpiRefCellDevice::new(spi_bus_ref, sd_cs, delay)
            .unwrap_or_else(|e| panic!("SPI SD card device init failed: {:?}", e));

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
    .unwrap_or_else(|e| panic!("uFerris init failed: {}", e))
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
        sw_interrupt: peripherals.SW_INTERRUPT,
    };

    // --------------------------------------
    //              ADC Setup
    // --------------------------------------

    // The same pin and the same attenuation as in `uferris_init`.
    // `Adc::into_async` installs the SARADC interrupt handler, which is what
    // lets a conversion suspend rather than spin.
    let mut adc_config = AdcConfig::new();
    let ldr_pin = adc_config.enable_pin(peripherals.GPIO2, Attenuation::_11dB);
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
    .unwrap_or_else(|e| panic!("I2C bus init failed: {:?}", e))
    .with_scl(peripherals.GPIO7)
    .with_sda(peripherals.GPIO6);

    // Configure the I/O expander while the driver is still blocking and still
    // exclusively owned: see this function's docs for why it cannot be done
    // afterwards. `IoExpander` borrows the bus for the duration rather than
    // taking it, so the driver is intact when this returns.
    IoExpander::<_, crate::Blocking>::new(&mut i2c)
        .init()
        .unwrap_or_else(|e| panic!("I/O expander init failed: {:?}", e));

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
    let led = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    // This is also the `Input` `wait_for_sw5` waits on.
    let button = Input::new(peripherals.GPIO5, InputConfig::default());

    // --------------------------------------
    //            LEDC / PWM Setup
    // --------------------------------------

    // Identical to `uferris_init`: `SetDutyCycle` is a blocking trait in both
    // modes, so the buzzer is the same driver on the same channel.
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
        .unwrap_or_else(|e| panic!("Buzzer timer config failed: {:?}", e));

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
        .unwrap_or_else(|e| panic!("Buzzer channel config failed: {:?}", e));

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
