use core::cell::RefCell;
use embassy_rp::{
    Peripherals,
    adc::{Adc, Blocking as AdcBlocking, Channel as AdcChannel, Config as AdcConfig},
    block::ImageDef,
    gpio::{Input, Level, Output, Pull},
    i2c::{Blocking as I2cBlocking, Config as I2cConfig, I2c},
    peripherals::I2C1,
    pwm::{Config as PwmConfig, Pwm},
};
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use static_cell::StaticCell;

#[cfg(feature = "async")]
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice as I2cAsyncDevice;
#[cfg(feature = "async")]
use embassy_rp::{
    adc::{self, Async as AdcAsync},
    bind_interrupts,
    i2c::{self, Async as I2cAsync},
};
#[cfg(feature = "async")]
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex as AsyncMutex};

#[cfg(feature = "power-board")]
use embassy_rp::{
    peripherals::SPI0,
    spi::{Blocking as SpiBlocking, Config as SpiConfig, Spi},
};
#[cfg(feature = "power-board")]
use embedded_hal_bus::spi::RefCellDevice as SpiRefCellDevice;

#[cfg(feature = "async")]
use crate::Async;
use crate::Uferris;
use crate::components::ldr::OneShot;
#[cfg(feature = "async")]
use crate::components::ldr::OneShotAsync;

// ------------------------------------------
// Interrupt Bindings
// ------------------------------------------

#[cfg(feature = "async")]
bind_interrupts!(
    /// The `async` drivers this board builds will not be handed out without
    /// proof that the matching interrupt vector is bound, and it is the `async`
    /// paths that actually arm them: the I2C driver suspends on `I2C1_IRQ` and
    /// the ADC on `ADC_IRQ_FIFO`, and the handlers below — `embassy-rp`'s own —
    /// wake the task that is waiting. Applications must therefore not bind
    /// `I2C1_IRQ` or `ADC_IRQ_FIFO` themselves: those two peripherals belong to
    /// the board.
    ///
    /// The blocking board needs none of this, which is why there are no
    /// bindings without the `async` feature: `I2c::new_blocking` and
    /// `Adc::new_blocking` poll the peripheral registers and never enable an
    /// interrupt.
    ///
    /// The pin waits behind `wait_for_sw5` need no binding either.
    /// `embassy-rp` claims `IO_IRQ_BANK0` itself under its `rt` feature and
    /// [`embassy_rp::init`] enables it, so GPIO wakeups are already wired up.
    struct Irqs {
        I2C1_IRQ => i2c::InterruptHandler<I2C1>;
        ADC_IRQ_FIFO => adc::InterruptHandler;
    }
);

// ------------------------------------------
// Image Definition Block
// ------------------------------------------

/// The RP2350 has no second stage bootloader. Its boot ROM instead scans the
/// first 4 KB of flash for an IMAGE_DEF block that describes the image, and
/// refuses to run anything it cannot find one for. The `.start_block` section
/// is placed directly after the vector table by the application's `memory.x`,
/// which is what keeps the block inside that window.
///
/// The uFerris images are plain secure Arm executables, so
/// [`ImageDef::secure_exe`] is the correct descriptor. Providing it is the
/// BSP's job, so applications do not have to declare one themselves.
///
/// `embassy-rp` emits a block of its own unless told not to, so the
/// `xiao-rp2350` feature turns on its `imagedef-none` feature. Only one block
/// may sit in `.start_block`, and this is the one applications can reach by
/// name from their `memory.x`.
#[unsafe(link_section = ".start_block")]
#[unsafe(no_mangle)]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// ------------------------------------------
// Board Constants
// ------------------------------------------

/// System clock frequency produced by [`embassy_rp::init`] with the default
/// [`embassy_rp::config::Config`].
///
/// That configuration runs the system PLL off the 12 MHz crystal fitted on the
/// Xiao RP2350 with `refdiv` 1, `fbdiv` 125 and post dividers 5 and 2, so
/// `12 MHz * 125 / 10 = 150 MHz`, and the system clock divider is 1.
const SYS_CLOCK_HZ: u32 = 150_000_000;

/// I2C bus frequency shared by the io expander, the RTC and the power monitor.
const I2C_FREQ_HZ: u32 = 100_000;

/// SPI bus frequency used to talk to the SD card on the power board.
#[cfg(feature = "power-board")]
const SPI_FREQ_HZ: u32 = 400_000;

// ------------------------------------------
// Static Types
// ------------------------------------------
static I2C_BUS: StaticCell<RefCell<Rp2350I2c>> = StaticCell::new();
#[cfg(feature = "power-board")]
static SPI_BUS: StaticCell<RefCell<Rp2350Spi>> = StaticCell::new();

/// The `async` board's I2C bus.
///
/// A second cell rather than a second use of [`I2C_BUS`], because the two board
/// modes share the bus differently: the blocking one hands out `RefCell`
/// borrows, the `async` one hands out futures that have to be able to suspend
/// mid-transaction and so needs an async-aware mutex. Only one of the two init
/// functions can ever run — both consume the [`Peripherals`] singleton — so the
/// cell the other mode would have used simply stays uninitialized.
#[cfg(feature = "async")]
static ASYNC_I2C_BUS: StaticCell<AsyncMutex<NoopRawMutex, Rp2350AsyncI2c>> = StaticCell::new();

// ------------------------------------------
// Type Defs
// ------------------------------------------

// I2C Types
type Rp2350I2c = I2c<'static, I2C1, I2cBlocking>;
type SharedI2c = I2cRefCellDevice<'static, Rp2350I2c>;

/// The `async` board's I2C driver.
///
/// A different type from [`Rp2350I2c`], not just a different way of using it:
/// `embassy-rp` puts the mode in the driver's type parameter, and only
/// `I2c<.., Async>` implements `embedded_hal_async::i2c::I2c`.
#[cfg(feature = "async")]
type Rp2350AsyncI2c = I2c<'static, I2C1, I2cAsync>;

/// One handle onto the `async` board's shared I2C bus.
///
/// [`NoopRawMutex`] is the right raw mutex here: every one of the board's I2C
/// devices is driven from the same executor on the one core the board uses, so
/// the bus is never contended from an interrupt or from the second core.
#[cfg(feature = "async")]
type SharedAsyncI2c = I2cAsyncDevice<'static, NoopRawMutex, Rp2350AsyncI2c>;

// GPIO Types
type LedPin = Output<'static>;
type ButtonPin = Input<'static>;

// ADC Types
pub struct LdrAdc {
    adc: Adc<'static, AdcBlocking>,
    channel: AdcChannel<'static>,
}

impl OneShot for LdrAdc {
    /// `embassy-rp` surfaces the conversion errors the RP2350 ADC can report,
    /// which the RP2040 driver had no way of signalling. [`OneShot::read_raw`]
    /// has no error channel, so a failed conversion reads as 0, the same value
    /// a saturated-dark LDR produces.
    fn read_raw(&mut self) -> u16 {
        self.adc.blocking_read(&mut self.channel).unwrap_or(0)
    }
}

/// The LDR channel of the ADC, sampled one conversion at a time, for the
/// `async` board.
///
/// [`Adc<Async>`][Adc] is a distinct type from the blocking one — the mode is a
/// type parameter on the driver — so this is a wrapper of its own rather than a
/// second `impl` on [`LdrAdc`]. The channel type is shared between the two.
#[cfg(feature = "async")]
pub struct LdrAdcAsync {
    adc: Adc<'static, AdcAsync>,
    channel: AdcChannel<'static>,
}

#[cfg(feature = "async")]
impl OneShotAsync for LdrAdcAsync {
    /// The `async` counterpart of the blocking [`OneShot`] implementation:
    /// `Adc::read` starts a single conversion, suspends on the FIFO interrupt
    /// instead of spinning on the ready flag, and reports the same conversion
    /// errors. [`OneShotAsync::read_raw`] has no error channel either, so a
    /// failed conversion reads as 0, the same value a saturated-dark LDR
    /// produces.
    async fn read_raw(&mut self) -> u16 {
        self.adc.read(&mut self.channel).await.unwrap_or(0)
    }
}

// Buzzer Types
pub type Rp2350BuzzerChannel = Pwm<'static>;

// SD/SPI Types
#[cfg(feature = "power-board")]
type Rp2350Spi = Spi<'static, SPI0, SpiBlocking>;

#[cfg(feature = "power-board")]
type SdCsPin = Output<'static>;

#[cfg(feature = "power-board")]
type SdBlockDevice =
    embedded_sdmmc::SdCard<SpiRefCellDevice<'static, Rp2350Spi, SdCsPin, CycleDelay>, CycleDelay>;

// ------------------------------------------
// uFerris Board Type Alias
// ------------------------------------------
#[cfg(not(feature = "power-board"))]
pub type UferrisRp2350 = Uferris<
    LedPin,              // LED (D1)
    ButtonPin,           // Button (D3)
    Rp2350BuzzerChannel, // Buzzer (D2)
    SharedI2c,           // I2C
    LdrAdc,              // LDR
    (),
>;

#[cfg(feature = "power-board")]
pub type UferrisRp2350 = Uferris<
    LedPin,              // LED (D1)
    ButtonPin,           // Button (D3)
    Rp2350BuzzerChannel, // Buzzer (D2)
    SharedI2c,           // I2C
    LdrAdc,              // LDR
    SdBlockDevice,       // SD Manager
>;

/// The `async` uFerris board on this controller, as returned by
/// [`uferris_init_async`].
#[cfg(all(feature = "async", not(feature = "power-board")))]
pub type UferrisRp2350Async = Uferris<
    LedPin,              // LED (D1)
    ButtonPin,           // Button (D3)
    Rp2350BuzzerChannel, // Buzzer (D2)
    SharedAsyncI2c,      // I2C
    LdrAdcAsync,         // LDR
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
pub type UferrisRp2350Async = Uferris<
    LedPin,              // LED (D1)
    ButtonPin,           // Button (D3)
    Rp2350BuzzerChannel, // Buzzer (D2)
    SharedAsyncI2c,      // I2C
    LdrAdcAsync,         // LDR
    SdBlockDevice,       // SD Manager (unused in `async` mode)
    Async,
>;

// ------------------------------------------
// Delay Provider
// ------------------------------------------

/// A busy loop delay, usable as an `embedded-hal` delay provider.
///
/// `embassy-rp` only exposes the RP2350 timers through the `embassy-time`
/// driver, which needs an executor to be useful, so the blocking board adapter
/// counts CPU cycles instead. [`cortex_m::asm::delay`] runs one loop iteration
/// per requested cycle and every iteration costs at least one cycle, so the
/// requested time is a lower bound: on the Cortex-M33 the loop takes about
/// three cycles per iteration, and a delay can therefore run up to roughly
/// three times long. That is the right side to err on for the SD card timings
/// that need it, but it does mean the delays are coarse. An `async` program has
/// an accurate timer backed delay available instead: it brings `embassy-time`
/// for its own runtime anyway, and `uferris_init_async` leaves the choice of
/// delay to it.
///
/// The type is a zero sized `Copy` marker, so every caller gets its own handle
/// and it can back both delay slots of a shared SPI device at once.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CycleDelay;

impl DelayNs for CycleDelay {
    fn delay_ns(&mut self, ns: u32) {
        // `u32::MAX` ns at 150 MHz is about 6.4e8 cycles, so the count always
        // fits back into the `u32` `asm::delay` takes.
        let cycles = u64::from(ns) * u64::from(SYS_CLOCK_HZ) / 1_000_000_000;
        cortex_m::asm::delay(cycles as u32);
    }
}

/// A [`CycleDelay`] handle, mirroring the timer getter the RP2040 board
/// exposes.
///
/// Unlike the RP2040 one this needs no board state, so it is callable before
/// [`uferris_init`].
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
/// There is no console on this board yet. The Xiao RP2350 has no USB-to-UART
/// bridge, so printing would mean driving a USB CDC device, and the
/// `embassy-rp` USB driver is `async` only. That driver is reachable now that
/// `uferris_init_async` exists, but nothing here drives it yet, so applications
/// on both board modes still run silent.
pub fn uferris_init(peripherals: Peripherals) -> UferrisRp2350 {
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
    let button = Input::new(peripherals.PIN_5, Pull::None);

    // --------------------------------------
    //              PWM Setup
    // --------------------------------------

    // The buzzer sits on D2 / GPIO28, which is slice 6 channel A.
    //
    // The RP2350 PWM output frequency is
    //     f_pwm = f_sys / (DIV * (TOP + 1))
    // where DIV is an 8.4 fixed point value.
    //
    // `embassy-rp` reports `max_duty_cycle()` as TOP rather than TOP + 1, so
    // TOP is 16384 here to keep the 14 bit resolution the LEDC based boards
    // use. Duty values written by application code therefore mean the same
    // thing on every board.
    //
    // The default clock configuration leaves the RP2350 system clock at
    // 150 MHz rather than the RP2040's 125 MHz, so hitting the 2700 Hz used by
    // the other boards needs
    //     DIV = 150_000_000 / (2700 * 16385) = 3.390
    // and the nearest representable value is 3 + 6/16 = 3.375, giving
    //     f_pwm = 150_000_000 / (3.375 * 16385) = 2713 Hz.
    let mut pwm_config = PwmConfig::default();
    pwm_config.top = 16384;
    // `divider` is the 8.4 fixed point value above and defaults to 1, so it is
    // scaled by 54/16 to reach 3 + 6/16 without naming the `fixed` types.
    pwm_config.divider = pwm_config.divider * 54 / 16;

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

// ------------------------------------------
// Board Initialization Function - `async`
// ------------------------------------------

/// Initialize the uFerris board in `async` mode.
///
/// The counterpart of [`uferris_init`]. It takes the same [`Peripherals`] and
/// wires up the same pins, but reaches for `embassy-rp`'s `async` constructors
/// instead of its blocking ones, so that the board's I2C and ADC operations are
/// futures: the I2C peripheral and the ADC are driven through the interrupts
/// bound in `Irqs` rather than polled, and the button is an [`Input`] the
/// board can wait on. The two init functions are mutually exclusive — each
/// consumes the peripheral singletons — so a program calls one or the other.
///
/// The executor is the application's. So is the time driver: this adapter hands
/// out no `async` delay, and a program that wants one takes `embassy_time`'s
/// (`embassy-rp`'s `time-driver` feature backs it, on the RP2350 timer) rather
/// than the blocking board's cycle counting [`CycleDelay`].
///
/// Everything [`uferris_init`] documents about [`embassy_rp::init`], the image
/// definition block and the absent console holds here unchanged. Waiting on the
/// button needs nothing extra either: [`Input`] implements
/// `embedded_hal_async::digital::Wait` unconditionally on this HAL, and
/// `embassy-rp` claims the `IO_IRQ_BANK0` vector itself.
///
/// The power board is not part of the `async` board yet: the SPI bus and the
/// INA219 are left alone here and the corresponding fields are parked. See
/// [`UferrisRp2350Async`].
#[cfg(feature = "async")]
pub async fn uferris_init_async(peripherals: Peripherals) -> UferrisRp2350Async {
    // --------------------------------------
    //              ADC Setup
    // --------------------------------------

    // `Adc<Async>` is built by `new` rather than `new_blocking`, and takes the
    // interrupt binding the blocking driver has no use for. The channel is the
    // same type and the same pin as in `uferris_init`.
    let adc = Adc::new(peripherals.ADC, Irqs, AdcConfig::default());
    // Floating, matching the blocking board: the LDR sits in a divider on the
    // uFerris carrier board, so no internal pull is configured here.
    let ldr_channel = AdcChannel::new_pin(peripherals.PIN_26, Pull::None);
    let ldr_driver = LdrAdcAsync {
        adc,
        channel: ldr_channel,
    };

    // --------------------------------------
    //              I2C Setup
    // --------------------------------------
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = I2C_FREQ_HZ;

    // D5 / GPIO7 and D4 / GPIO6 on I2C1, as in `uferris_init`, but through the
    // `async` constructor: it takes the `I2C1_IRQ` binding and enables the
    // vector, which is what lets a transaction suspend rather than spin.
    let i2c = I2c::new_async(
        peripherals.I2C1,
        peripherals.PIN_7,
        peripherals.PIN_6,
        Irqs,
        i2c_config,
    );

    // Promote I2C Bus to Static
    let i2c_bus_ref = ASYNC_I2C_BUS.init(AsyncMutex::new(i2c));

    // Device Instances
    let expander_i2c = I2cAsyncDevice::new(i2c_bus_ref);
    let rtc_i2c = I2cAsyncDevice::new(i2c_bus_ref);
    let raw_i2c = I2cAsyncDevice::new(i2c_bus_ref);

    // --------------------------------------
    //              GPIO Setup
    // --------------------------------------
    let led = Output::new(peripherals.PIN_27, Level::Low);
    // Floating input, matching the blocking board: the button is wired active
    // low against a pull-up on the uFerris carrier board, so no internal pull is
    // configured here. This is also the `Input` `wait_for_sw5` waits on.
    let button = Input::new(peripherals.PIN_5, Pull::None);

    // --------------------------------------
    //              PWM Setup
    // --------------------------------------

    // The buzzer sits on D2 / GPIO28, slice 6 channel A, with the same TOP and
    // divider as in `uferris_init` — see there for the frequency maths.
    // `SetDutyCycle` is a blocking trait in both modes, so this is unchanged.
    let mut pwm_config = PwmConfig::default();
    pwm_config.top = 16384;
    pwm_config.divider = pwm_config.divider * 54 / 16;

    let buzzer_channel = Pwm::new_output_a(peripherals.PWM_SLICE6, peripherals.PIN_28, pwm_config);

    // --------------------------------------
    //          Board Instantiation
    // --------------------------------------
    Uferris::new_async(
        led,
        button,
        buzzer_channel,
        ldr_driver,
        expander_i2c,
        rtc_i2c,
        raw_i2c,
    )
    .await
    .unwrap()
}
