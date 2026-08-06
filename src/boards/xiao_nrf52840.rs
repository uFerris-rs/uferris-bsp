use core::cell::RefCell;
use core::sync::atomic::{Ordering, compiler_fence};
use embassy_nrf::{
    Peripherals, bind_interrupts,
    gpio::{Input, Level, Output, OutputDrive, Pull},
    pac,
    pwm::{Prescaler, SimpleConfig, SimplePwm},
    saadc::{self, ChannelConfig, Config as SaadcConfig, Saadc},
    twim::{self, Config as TwimConfig, Frequency as TwimFrequency, Twim},
};
use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use static_cell::StaticCell;

#[cfg(feature = "async")]
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice as I2cAsyncDevice;
#[cfg(feature = "async")]
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex as AsyncMutex};

#[cfg(feature = "power-board")]
use embassy_nrf::spim::{self, Config as SpimConfig, Frequency as SpimFrequency, Spim};
#[cfg(feature = "power-board")]
use embedded_hal_bus::spi::RefCellDevice as SpiRefCellDevice;

#[cfg(feature = "async")]
use crate::Async;
use crate::Uferris;
use crate::boards::nrf_buzzer::{BUZZER_COUNTERTOP, NrfBuzzerChannel};
use crate::components::ldr::OneShot;
#[cfg(feature = "async")]
use crate::components::ldr::OneShotAsync;

// ------------------------------------------
// Interrupt Bindings
// ------------------------------------------

bind_interrupts!(
    /// `embassy-nrf` will not hand out a TWIM, SPIM or SAADC driver without
    /// proof that the matching interrupt vector is bound, even for the blocking
    /// constructors. A binding is only a vector table entry: the handlers below
    /// are `embassy-nrf`'s own, they do nothing but wake a task, and nothing
    /// here ever arms them. Every driver clears `INTENSET` while it is being
    /// built and the blocking code paths poll the event registers instead, so
    /// these vectors stay dormant for the whole life of the program. No
    /// executor is involved.
    ///
    /// Applications must therefore not bind `TWISPI0`, `SPIM3` or `SAADC`
    /// themselves: those three peripherals belong to the board.
    struct Irqs {
        SAADC => saadc::InterruptHandler;
        TWISPI0 => twim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
        #[cfg(feature = "power-board")]
        SPIM3 => spim::InterruptHandler<embassy_nrf::peripherals::SPI3>;
    }
);

// ------------------------------------------
// Board Constants
// ------------------------------------------

/// CPU clock frequency of the nRF52840.
///
/// The core runs from a fixed 64 MHz clock regardless of which HFCLK source
/// [`embassy_nrf::init`] leaves selected, so this is a constant rather than
/// something read back from a clock tree.
const SYS_CLOCK_HZ: u32 = 64_000_000;

/// I2C bus frequency shared by the io expander, the RTC and the power monitor.
const I2C_FREQ: TwimFrequency = TwimFrequency::K100;

/// SPI bus frequency used to talk to the SD card on the power board.
///
/// The SPIM only offers a fixed ladder of bit rates, and SD cards have to be
/// initialized at 400 kHz or less, so the 400 kHz the other boards use rounds
/// *down* to the 250 kHz rung rather than up to 500 kHz.
#[cfg(feature = "power-board")]
const SPI_FREQ: SpimFrequency = SpimFrequency::K250;

/// Scratch buffer the TWIM driver copies non-RAM write payloads into.
///
/// The TWIM is a DMA peripheral and can only read from RAM, but several of the
/// board's I2C writes are constant slices the compiler is free to promote into
/// flash. The longest of them is the RTC time write, at 8 bytes.
const TWIM_RAM_BUFFER_LEN: usize = 16;

// ------------------------------------------
// Static Types
// ------------------------------------------
static I2C_BUS: StaticCell<RefCell<Nrf52840I2c>> = StaticCell::new();
static TWIM_RAM_BUFFER: StaticCell<[u8; TWIM_RAM_BUFFER_LEN]> = StaticCell::new();
#[cfg(feature = "power-board")]
static SPI_BUS: StaticCell<RefCell<Nrf52840Spi>> = StaticCell::new();

/// The `async` board's I2C bus.
///
/// A second cell rather than a second use of [`I2C_BUS`], because the two board
/// modes share the bus differently: the blocking one hands out `RefCell`
/// borrows, the `async` one hands out futures that have to be able to suspend
/// mid-transaction and so needs an async-aware mutex. Only one of the two init
/// functions can ever run — both consume the [`Peripherals`] singleton — so the
/// cell the other mode would have used simply stays uninitialized. The TWIM
/// scratch buffer is shared between them for the same reason.
#[cfg(feature = "async")]
static ASYNC_I2C_BUS: StaticCell<AsyncMutex<NoopRawMutex, Nrf52840I2c>> = StaticCell::new();

// ------------------------------------------
// Type Defs
// ------------------------------------------

// I2C Types
type Nrf52840I2c = Twim<'static>;
type SharedI2c = I2cRefCellDevice<'static, Nrf52840I2c>;

/// One handle onto the `async` board's shared I2C bus.
///
/// [`NoopRawMutex`] is the right raw mutex here: every one of the board's I2C
/// devices is driven from the same executor on the one core, so the bus is never
/// contended from an interrupt or a second core.
#[cfg(feature = "async")]
type SharedAsyncI2c = I2cAsyncDevice<'static, NoopRawMutex, Nrf52840I2c>;

// GPIO Types
type LedPin = Output<'static>;
type ButtonPin = Input<'static>;

// ADC Types

/// The LDR channel of the SAADC, sampled one conversion at a time.
pub struct LdrAdc {
    /// The configured SAADC. Only held so that the peripheral stays claimed and
    /// configured for as long as the board is alive; the conversion itself is
    /// driven through [`pac::SAADC`] by [`OneShot::read_raw`].
    _saadc: Saadc<'static, 1>,
}

impl OneShot for LdrAdc {
    /// Run a single conversion and return the result.
    ///
    /// `embassy-nrf` only exposes SAADC sampling through `async` methods, which
    /// would need an executor, so the conversion is driven straight through the
    /// registers here. The sequence is the blocking equivalent of
    /// `embassy_nrf::saadc::Saadc::sample`: point `RESULT.PTR` at a one entry
    /// buffer, fire `START` and `SAMPLE`, spin on `EVENTS_END`, then `STOP` and
    /// spin on `EVENTS_STOPPED`. Stopping matters because an SAADC left running
    /// keeps drawing current. No interrupt is ever enabled, so this cannot race
    /// the bound vector.
    ///
    /// The SAADC is a signed converter: the board configures the channel as
    /// single ended at 12 bit resolution, which yields results in `0..=4095`,
    /// but noise around the bottom of the range can still push a conversion
    /// slightly negative. [`OneShot::read_raw`] is unsigned, so negative
    /// results are clamped to 0 — the same value a saturated-dark LDR produces.
    /// The effective resolution is therefore 12 bits in a `u16`, matching the
    /// other boards.
    fn read_raw(&mut self) -> u16 {
        let r = pac::SAADC;

        // The result buffer is a stack local, so it is in RAM as EasyDMA
        // requires, and it outlives the conversion because this function does
        // not return until `EVENTS_END` has been observed.
        let mut result: i16 = 0;

        r.result().ptr().write_value(&raw mut result as u32);
        r.result().maxcnt().write(|w| w.set_maxcnt(1));

        r.events_end().write_value(0);

        // Do not let the sampling tasks be reordered before the DMA setup.
        compiler_fence(Ordering::SeqCst);

        r.tasks_start().write_value(1);
        r.tasks_sample().write_value(1);

        while r.events_end().read() == 0 {}
        r.events_end().write_value(0);

        // Stop sampling, so the SAADC does not keep burning current.
        compiler_fence(Ordering::SeqCst);

        r.events_stopped().write_value(0);
        r.tasks_stop().write_value(1);

        while r.events_stopped().read() == 0 {}
        r.events_stopped().write_value(0);

        compiler_fence(Ordering::SeqCst);

        result.max(0) as u16
    }
}

/// The LDR channel of the SAADC, sampled one conversion at a time, for the
/// `async` board.
///
/// The blocking [`LdrAdc`] has to drive the converter through the registers by
/// hand because `embassy-nrf` only exposes sampling as a future. Here that
/// future is exactly what is wanted, so this holds the driver and awaits it.
#[cfg(feature = "async")]
pub struct LdrAdcAsync {
    saadc: Saadc<'static, 1>,
}

#[cfg(feature = "async")]
impl OneShotAsync for LdrAdcAsync {
    /// Run a single conversion and return the result.
    ///
    /// [`Saadc::sample`] points the DMA at the buffer, fires the conversion,
    /// suspends on `EVENTS_END` and stops the converter again before it
    /// returns, so an SAADC left idle does not keep drawing current.
    ///
    /// The channel is configured exactly as the blocking board configures it —
    /// single ended, 12 bit, 0..3.6 V — and the same clamp applies: the SAADC
    /// is a signed converter and noise at the bottom of the range can push a
    /// reading slightly negative, so negative results are clamped to 0.
    async fn read_raw(&mut self) -> u16 {
        let mut result = [0i16; 1];
        self.saadc.sample(&mut result).await;
        result[0].max(0) as u16
    }
}

// SD/SPI Types
#[cfg(feature = "power-board")]
type Nrf52840Spi = Spim<'static>;

#[cfg(feature = "power-board")]
type SdCsPin = Output<'static>;

#[cfg(feature = "power-board")]
type SdBlockDevice =
    embedded_sdmmc::SdCard<SpiRefCellDevice<'static, Nrf52840Spi, SdCsPin, CycleDelay>, CycleDelay>;

// ------------------------------------------
// uFerris Board Type Alias
// ------------------------------------------
#[cfg(not(feature = "power-board"))]
pub type UferrisNrf52840 = Uferris<
    LedPin,           // LED (D1)
    ButtonPin,        // Button (D3)
    NrfBuzzerChannel, // Buzzer (D2)
    SharedI2c,        // I2C
    LdrAdc,           // LDR
    (),
>;

#[cfg(feature = "power-board")]
pub type UferrisNrf52840 = Uferris<
    LedPin,           // LED (D1)
    ButtonPin,        // Button (D3)
    NrfBuzzerChannel, // Buzzer (D2)
    SharedI2c,        // I2C
    LdrAdc,           // LDR
    SdBlockDevice,    // SD Manager
>;

/// The `async` uFerris board on this controller, as returned by
/// [`uferris_init_async`].
#[cfg(all(feature = "async", not(feature = "power-board")))]
pub type UferrisNrf52840Async = Uferris<
    LedPin,           // LED (D1)
    ButtonPin,        // Button (D3)
    NrfBuzzerChannel, // Buzzer (D2)
    SharedAsyncI2c,   // I2C
    LdrAdcAsync,      // LDR
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
pub type UferrisNrf52840Async = Uferris<
    LedPin,           // LED (D1)
    ButtonPin,        // Button (D3)
    NrfBuzzerChannel, // Buzzer (D2)
    SharedAsyncI2c,   // I2C
    LdrAdcAsync,      // LDR
    SdBlockDevice,    // SD Manager (unused in `async` mode)
    Async,
>;

// ------------------------------------------
// Delay Provider
// ------------------------------------------

/// A busy loop delay, usable as an `embedded-hal` delay provider.
///
/// `embassy-nrf` only exposes the nRF timers through the `embassy-time` driver,
/// which needs an executor to be useful, so the blocking board adapter counts
/// CPU cycles instead. [`cortex_m::asm::delay`] runs one loop iteration per
/// requested cycle and every iteration costs at least one cycle, so the
/// requested time is a lower bound: on the Cortex-M4 the loop takes about three
/// cycles per iteration, and a delay can therefore run up to roughly three
/// times long. That is the right side to err on for the SD card timings that
/// need it, but it does mean the delays are coarse. An `async` program has an
/// accurate timer backed delay available instead: it brings `embassy-time` for
/// its own runtime anyway, and `uferris_init_async` leaves the choice of delay
/// to it.
///
/// The type is a zero sized `Copy` marker, so every caller gets its own handle
/// and it can back both delay slots of a shared SPI device at once.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CycleDelay;

impl DelayNs for CycleDelay {
    fn delay_ns(&mut self, ns: u32) {
        // `u32::MAX` ns at 64 MHz is about 2.7e8 cycles, so the count always
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
/// `embassy-nrf` is used in blocking mode: no executor, no time driver and no
/// `async` anywhere. [`embassy_nrf::init`] applies the chip errata workarounds
/// and hands back the peripheral singletons, and every driver built here is
/// driven through its blocking path, all of which implement the
/// `embedded-hal` 1.0 blocking traits the board logic is written against.
///
/// The `xiao-nrf52840` feature supplies the critical section implementation
/// `embassy-nrf` needs through `cortex-m`'s `critical-section-single-core`,
/// which masks interrupts on the calling core. That is sound here because the
/// nRF52840 has a single core *and* because this crate never enables the
/// SoftDevice: a running SoftDevice reserves the highest interrupt priorities
/// for itself and keeps executing through a `BASEPRI`-style mask, which would
/// break the mutual exclusion the single core implementation assumes. The stock
/// Xiao ships with an S140 image resident in flash, but nothing here starts it.
///
/// There is no console on this board yet. The Xiao nRF52840 has no
/// USB-to-UART bridge, so printing would mean driving a USB CDC device, and the
/// `embassy-nrf` USB driver is `async` only. That driver is reachable now that
/// `uferris_init_async` exists, but nothing here drives it yet, so applications
/// on both board modes still run silent.
pub fn uferris_init(peripherals: Peripherals) -> UferrisNrf52840 {
    // --------------------------------------
    //              ADC Setup
    // --------------------------------------

    // The LDR sits on D0 / P0.02, which is SAADC analog input AIN0.
    //
    // `ChannelConfig::single_ended` picks the internal 0.6 V reference with a
    // gain of 1/6, so the input range is 0..3.6 V, and `SaadcConfig::default`
    // is 12 bit with oversampling bypassed. That matches the 12-bit-in-a-`u16`
    // reading the other boards return.
    let ldr_channel = ChannelConfig::single_ended(peripherals.P0_02);
    let saadc = Saadc::new(
        peripherals.SAADC,
        Irqs,
        SaadcConfig::default(),
        [ldr_channel],
    );
    let ldr_driver = LdrAdc { _saadc: saadc };

    // --------------------------------------
    //              I2C Setup
    // --------------------------------------
    let mut i2c_config = TwimConfig::default();
    i2c_config.frequency = I2C_FREQ;
    // The uFerris carrier board fits the bus pull-ups, so the internal ones
    // stay off.

    let i2c = Twim::new(
        peripherals.TWISPI0,
        Irqs,
        peripherals.P0_04,
        peripherals.P0_05,
        i2c_config,
        TWIM_RAM_BUFFER.init([0u8; TWIM_RAM_BUFFER_LEN]),
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
    let led = Output::new(peripherals.P0_03, Level::Low, OutputDrive::Standard);
    // Floating input, matching the other boards: the button is wired active low
    // against a pull-up on the uFerris carrier board, so no internal pull is
    // configured here.
    let button = Input::new(peripherals.P0_29, Pull::None);

    // --------------------------------------
    //              PWM Setup
    // --------------------------------------

    // The buzzer sits on D2 / P0.28, driven by channel 0 of PWM0. See
    // `NrfBuzzerChannel` for the countertop and duty cycle maths.
    let mut pwm_config = SimpleConfig::default();
    pwm_config.prescaler = Prescaler::Div1;
    pwm_config.max_duty = BUZZER_COUNTERTOP;

    let pwm = SimplePwm::new_1ch(peripherals.PWM0, peripherals.P0_28, &pwm_config);
    let mut buzzer_channel = NrfBuzzerChannel::new(pwm);
    // `SimplePwm` does not start a sequence until a duty cycle is written, so
    // write one to put the pin in a defined, silent state.
    let Ok(()) = buzzer_channel.set_duty_cycle_fully_off();

    // --------------------------------------
    //            SPI / SD Setup
    // --------------------------------------
    #[cfg(feature = "power-board")]
    let vol_mgr = {
        let mut spi_config = SpimConfig::default();
        spi_config.frequency = SPI_FREQ;
        // `SpimConfig` defaults to mode 0, which is what the SD card wants.

        let spi = Spim::new(
            peripherals.SPI3,
            Irqs,
            peripherals.P1_13,
            peripherals.P1_14,
            peripherals.P1_15,
            spi_config,
        );

        // Promote SPI Bus to Static
        let spi_bus_ref = SPI_BUS.init(RefCell::new(spi));

        // CS Pin
        let sd_cs = Output::new(peripherals.P1_12, Level::High, OutputDrive::Standard);

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
/// wires up the same pins, but builds the drivers so that the board's I2C and
/// ADC operations are futures: the TWIM and the SAADC are driven through their
/// interrupts rather than polled, and the button is an [`Input`] the board can
/// wait on. The two are mutually exclusive — each consumes the peripheral
/// singletons — so a program calls one or the other.
///
/// The executor is the application's. So is the time driver: this adapter hands
/// out no `async` delay, and a program that wants one takes `embassy_time`'s
/// (`embassy-nrf`'s `time-driver-rtc1` feature backs it on this part) rather
/// than the blocking board's cycle counting [`CycleDelay`].
///
/// Everything [`uferris_init`] documents about [`embassy_nrf::init`], the
/// critical section implementation and the absent console holds here unchanged.
/// The one addition is GPIOTE: `embedded_hal_async::digital::Wait`, which
/// `wait_for_sw5` waits on, is only implemented for [`Input`] when
/// `embassy-nrf`'s `gpiote` feature is on, so the BSP's `async` feature enables
/// it. `embassy_nrf::init` then also initializes the GPIOTE interrupt, and
/// `embassy-nrf` provides that vector's handler itself — there is nothing extra
/// for an application to bind.
///
/// The power board is not part of the `async` board yet: the SPI bus and the
/// INA219 are left alone here and the corresponding fields are parked. See
/// [`UferrisNrf52840Async`].
#[cfg(feature = "async")]
pub async fn uferris_init_async(peripherals: Peripherals) -> UferrisNrf52840Async {
    // --------------------------------------
    //              ADC Setup
    // --------------------------------------

    // The LDR sits on D0 / P0.02, which is SAADC analog input AIN0, configured
    // exactly as the blocking board configures it. See `uferris_init`.
    let ldr_channel = ChannelConfig::single_ended(peripherals.P0_02);
    let saadc = Saadc::new(
        peripherals.SAADC,
        Irqs,
        SaadcConfig::default(),
        [ldr_channel],
    );
    let ldr_driver = LdrAdcAsync { saadc };

    // --------------------------------------
    //              I2C Setup
    // --------------------------------------
    let mut i2c_config = TwimConfig::default();
    i2c_config.frequency = I2C_FREQ;
    // The uFerris carrier board fits the bus pull-ups, so the internal ones
    // stay off.

    // D4 / P0.04 and D5 / P0.05 on `TWISPI0`, as in `uferris_init`. The
    // constructor is the same one: `Twim::new` binds the interrupt either way,
    // and it is the caller that decides whether to poll the driver or await it.
    let i2c = Twim::new(
        peripherals.TWISPI0,
        Irqs,
        peripherals.P0_04,
        peripherals.P0_05,
        i2c_config,
        TWIM_RAM_BUFFER.init([0u8; TWIM_RAM_BUFFER_LEN]),
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
    let led = Output::new(peripherals.P0_03, Level::Low, OutputDrive::Standard);
    // Floating input, matching the blocking board: the button is wired active
    // low against a pull-up on the uFerris carrier board, so no internal pull is
    // configured here. With `gpiote` on, this `Input` is also what
    // `wait_for_sw5` waits on.
    let button = Input::new(peripherals.P0_29, Pull::None);

    // --------------------------------------
    //              PWM Setup
    // --------------------------------------

    // The buzzer sits on D2 / P0.28, driven by channel 0 of PWM0, exactly as in
    // `uferris_init`. `SetDutyCycle` is a blocking trait in both modes, so this
    // is unchanged.
    let mut pwm_config = SimpleConfig::default();
    pwm_config.prescaler = Prescaler::Div1;
    pwm_config.max_duty = BUZZER_COUNTERTOP;

    let pwm = SimplePwm::new_1ch(peripherals.PWM0, peripherals.P0_28, &pwm_config);
    let mut buzzer_channel = NrfBuzzerChannel::new(pwm);
    // `SimplePwm` does not start a sequence until a duty cycle is written, so
    // write one to put the pin in a defined, silent state.
    let Ok(()) = buzzer_channel.set_duty_cycle_fully_off();

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
