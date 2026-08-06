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
    /// The nRF54L names its serial peripherals after the instance rather than
    /// the protocol: `SERIAL22` is the TWIM the board uses for I2C and
    /// `SERIAL00` is the SPIM it uses for the SD card. Applications must
    /// therefore not bind `SERIAL22`, `SERIAL00` or `SAADC` themselves: those
    /// three peripherals belong to the board.
    struct Irqs {
        SAADC => saadc::InterruptHandler;
        SERIAL22 => twim::InterruptHandler<embassy_nrf::peripherals::SERIAL22>;
        #[cfg(feature = "power-board")]
        SERIAL00 => spim::InterruptHandler<embassy_nrf::peripherals::SERIAL00>;
    }
);

// ------------------------------------------
// Board Constants
// ------------------------------------------

/// CPU clock frequency of the nRF54L15.
///
/// The nRF54L can run its core at either 64 MHz or 128 MHz, selected by
/// [`embassy_nrf::config::Config::clock_speed`]. [`uferris_init`] leaves the
/// `embassy-nrf` default of [`ClockSpeed::CK64`][clk] in place, so the core
/// runs at 64 MHz and this is a constant rather than something read back from
/// the clock tree.
///
/// [clk]: embassy_nrf::config::ClockSpeed::CK64
const SYS_CLOCK_HZ: u32 = 64_000_000;

/// I2C bus frequency shared by the io expander, the RTC and the power monitor.
const I2C_FREQ: TwimFrequency = TwimFrequency::K100;

/// SPI bus frequency requested for the SD card on the power board.
///
/// The SD specification wants a card initialized at 400 kHz or less, which is
/// what the other boards ask for, but this bus cannot go that slow. The SD card
/// pins land on port 2, and on the nRF54L the port 2 pads are wired to the fast
/// peripherals only, which leaves `SERIAL00` as the one SPIM that can reach
/// them. `SERIAL00` is clocked from the core PLL rather than the fixed 16 MHz
/// the `SERIAL2x` instances run at, and its `PRESCALER.DIVISOR` field is seven
/// bits wide, so the slowest bit rate it can produce is
///
///     64 MHz / 127 = 504 kHz
///
/// `embassy-nrf` still models the bit rate as the classic nRF ladder and turns
/// it into a divisor at run time, so anything below 1 Mbps computes a divisor
/// that does not fit the field: `K500` asks for 128 and `K250` for 256, and
/// both are truncated to 0 on the way into the register. [`Frequency::M1`][f]
/// is therefore the slowest rung that survives the conversion, and
/// [`uferris_init`] narrows the divisor to [`SPI_DIVISOR`] afterwards to get
/// back down to the hardware floor.
///
/// [f]: embassy_nrf::spim::Frequency::M1
#[cfg(feature = "power-board")]
const SPI_FREQ: SpimFrequency = SpimFrequency::M1;

/// `SERIAL00` `PRESCALER.DIVISOR` value written over the one [`SPI_FREQ`]
/// produces, giving the slowest SD card clock this board can reach.
///
///     64_000_000 / 126 = 507_936 Hz
///
/// 126 rather than the 127 the field would hold, because the nRF54L SPIM
/// divisor is documented as an even value.
#[cfg(feature = "power-board")]
const SPI_DIVISOR: u8 = 126;

/// Scratch buffer the TWIM driver copies non-RAM write payloads into.
///
/// The TWIM is a DMA peripheral and can only read from RAM, but several of the
/// board's I2C writes are constant slices the compiler is free to promote into
/// flash. The longest of them is the RTC time write, at 8 bytes.
const TWIM_RAM_BUFFER_LEN: usize = 16;

// ------------------------------------------
// Static Types
// ------------------------------------------
static I2C_BUS: StaticCell<RefCell<Nrf54l15I2c>> = StaticCell::new();
static TWIM_RAM_BUFFER: StaticCell<[u8; TWIM_RAM_BUFFER_LEN]> = StaticCell::new();
#[cfg(feature = "power-board")]
static SPI_BUS: StaticCell<RefCell<Nrf54l15Spi>> = StaticCell::new();

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
static ASYNC_I2C_BUS: StaticCell<AsyncMutex<NoopRawMutex, Nrf54l15I2c>> = StaticCell::new();

// ------------------------------------------
// Type Defs
// ------------------------------------------

// I2C Types
type Nrf54l15I2c = Twim<'static>;
type SharedI2c = I2cRefCellDevice<'static, Nrf54l15I2c>;

/// One handle onto the `async` board's shared I2C bus.
///
/// [`NoopRawMutex`] is the right raw mutex here: every one of the board's I2C
/// devices is driven from the same executor on the one application core, so the
/// bus is never contended from an interrupt or a second core.
#[cfg(feature = "async")]
type SharedAsyncI2c = I2cAsyncDevice<'static, NoopRawMutex, Nrf54l15I2c>;

// GPIO Types
type LedPin = Output<'static>;
type ButtonPin = Input<'static>;

// ADC Types

/// Number of `RESULT.MAXCNT` units one conversion occupies.
///
/// `MAXCNT` counts 16 bit samples on the nRF52 SAADC but bytes on the nRF54L
/// one, so a single conversion has to be announced as two units here where the
/// nRF52840 board announces one.
const SAADC_CNT_UNIT: u16 = 2;

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
    /// The one nRF54L specific part is `RESULT.MAXCNT`, which counts bytes
    /// rather than samples here. See [`SAADC_CNT_UNIT`].
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
        r.result().maxcnt().write(|w| w.set_maxcnt(SAADC_CNT_UNIT));

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
    /// returns, so an SAADC left idle does not keep drawing current. The
    /// nRF54L's `RESULT.MAXCNT`-counts-bytes quirk, which [`LdrAdc`] has to
    /// handle itself, is the driver's business here.
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
type Nrf54l15Spi = Spim<'static>;

#[cfg(feature = "power-board")]
type SdCsPin = Output<'static>;

#[cfg(feature = "power-board")]
type SdBlockDevice =
    embedded_sdmmc::SdCard<SpiRefCellDevice<'static, Nrf54l15Spi, SdCsPin, CycleDelay>, CycleDelay>;

// ------------------------------------------
// uFerris Board Type Alias
// ------------------------------------------
#[cfg(not(feature = "power-board"))]
pub type UferrisNrf54l15 = Uferris<
    LedPin,           // LED (D1)
    ButtonPin,        // Button (D3)
    NrfBuzzerChannel, // Buzzer (D2)
    SharedI2c,        // I2C
    LdrAdc,           // LDR
    (),
>;

#[cfg(feature = "power-board")]
pub type UferrisNrf54l15 = Uferris<
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
pub type UferrisNrf54l15Async = Uferris<
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
pub type UferrisNrf54l15Async = Uferris<
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
/// `async` anywhere. [`embassy_nrf::init`] selects the core clock, clears any
/// leftover FLPR program and hands back the peripheral singletons, and every
/// driver built here is driven through its blocking path, all of which
/// implement the `embedded-hal` 1.0 blocking traits the board logic is written
/// against.
///
/// The board expects [`embassy_nrf::init`] to have been called with the default
/// configuration, which keeps the core at 64 MHz — the frequency [`CycleDelay`]
/// is scaled to — and resets the FLPR coprocessor, which a debugger reset
/// otherwise leaves running and which can stall `init` if it is left alone.
///
/// The `xiao-nrf54l15` feature supplies the critical section implementation
/// `embassy-nrf` needs through `cortex-m`'s `critical-section-single-core`,
/// which masks interrupts on the calling core. That is sound here because the
/// nRF54L15 application core is a single core and because nothing on this part
/// plays the role the nRF52840 SoftDevice does: there is no SoftDevice for the
/// nRF54L, so no higher priority code keeps executing through the mask. The
/// FLPR coprocessor shares no memory with the peripherals the BSP touches, and
/// `init` stops it in any case.
///
/// Applications print over RTT: the nRF54L15 has no USB peripheral at all, so
/// the onboard CMSIS-DAP debugger is the console, and `probe-rs run` carries
/// both the image and the RTT stream over the one USB-C cable.
pub fn uferris_init(peripherals: Peripherals) -> UferrisNrf54l15 {
    // --------------------------------------
    //              ADC Setup
    // --------------------------------------

    // The LDR sits on D0 / P1.04. The nRF54L SAADC selects its input by port
    // and pin rather than by an `AIN` channel number, so the pin is handed to
    // `ChannelConfig` directly; only P1.04-P1.07 and P1.11-P1.14 are wired to
    // the converter.
    //
    // `ChannelConfig::single_ended` picks the internal 0.9 V reference with a
    // gain of 2/8, so the input range is 0..3.6 V, and `SaadcConfig::default`
    // is 12 bit with oversampling bypassed. That is the same 0..3.6 V window
    // and the same 12-bit-in-a-`u16` reading the nRF52840 board produces from
    // its 0.6 V reference and 1/6 gain, and it clears the 3.3 V the LDR divider
    // can reach without clipping. The next gain up, 2/7, would fold the range
    // down to 3.15 V and clip a bright reading.
    let ldr_channel = ChannelConfig::single_ended(peripherals.P1_04);
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

    // D4 / P1.10 and D5 / P1.11. `SERIAL22` is the instance behind them:
    // `SERIAL00`, the other one that could be reached from here, has no TWIM.
    let i2c = Twim::new(
        peripherals.SERIAL22,
        Irqs,
        peripherals.P1_10,
        peripherals.P1_11,
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
    let led = Output::new(peripherals.P1_05, Level::Low, OutputDrive::Standard);
    // Floating input, matching the other boards: the button is wired active low
    // against a pull-up on the uFerris carrier board, so no internal pull is
    // configured here.
    let button = Input::new(peripherals.P1_07, Pull::None);

    // --------------------------------------
    //              PWM Setup
    // --------------------------------------

    // The buzzer sits on D2 / P1.06, driven by channel 0 of PWM20. See
    // `NrfBuzzerChannel` for the countertop and duty cycle maths. The nRF54L
    // PWM instances live in the same power domain as the port 1 pads, which is
    // where D2 is, so PWM20 can drive it.
    let mut pwm_config = SimpleConfig::default();
    pwm_config.prescaler = Prescaler::Div1;
    pwm_config.max_duty = BUZZER_COUNTERTOP;

    let pwm = SimplePwm::new_1ch(peripherals.PWM20, peripherals.P1_06, &pwm_config);
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

        // D8 / P2.01, D9 / P2.04 and D10 / P2.02.
        let spi = Spim::new(
            peripherals.SERIAL00,
            Irqs,
            peripherals.P2_01,
            peripherals.P2_04,
            peripherals.P2_02,
            spi_config,
        );

        // Slow the bus the rest of the way down to the 508 kHz floor. See
        // `SPI_FREQ` for why this cannot be asked for through `SpimConfig`.
        // Nothing has been transferred yet, so the write lands between the
        // driver's own configuration and the first clock edge.
        pac::SPIM00
            .prescaler()
            .write(|w| w.set_divisor(SPI_DIVISOR));

        // Promote SPI Bus to Static
        let spi_bus_ref = SPI_BUS.init(RefCell::new(spi));

        // CS Pin
        let sd_cs = Output::new(peripherals.P2_07, Level::High, OutputDrive::Standard);

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
/// (`embassy-nrf`'s `time-driver-grtc` feature backs it on this part) rather
/// than the blocking board's cycle counting [`CycleDelay`].
///
/// Everything [`uferris_init`] documents about [`embassy_nrf::init`], the
/// critical section implementation and printing over RTT holds here unchanged.
/// The one addition is GPIOTE: `embedded_hal_async::digital::Wait`, which
/// `wait_for_sw5` waits on, is only implemented for [`Input`] when
/// `embassy-nrf`'s `gpiote` feature is on, so the BSP's `async` feature enables
/// it. `embassy_nrf::init` then also initializes the GPIOTE interrupt, and
/// `embassy-nrf` provides that vector's handler itself — there is nothing extra
/// for an application to bind.
///
/// The power board is not part of the `async` board yet: the SPI bus and the
/// INA219 are left alone here and the corresponding fields are parked. See
/// [`UferrisNrf54l15Async`].
#[cfg(feature = "async")]
pub async fn uferris_init_async(peripherals: Peripherals) -> UferrisNrf54l15Async {
    // --------------------------------------
    //              ADC Setup
    // --------------------------------------

    // The LDR sits on D0 / P1.04, configured exactly as the blocking board
    // configures it. See `uferris_init`.
    let ldr_channel = ChannelConfig::single_ended(peripherals.P1_04);
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

    // D4 / P1.10 and D5 / P1.11 on `SERIAL22`, as in `uferris_init`. The
    // constructor is the same one: `Twim::new` binds the interrupt either way,
    // and it is the caller that decides whether to poll the driver or await it.
    let i2c = Twim::new(
        peripherals.SERIAL22,
        Irqs,
        peripherals.P1_10,
        peripherals.P1_11,
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
    let led = Output::new(peripherals.P1_05, Level::Low, OutputDrive::Standard);
    // Floating input, matching the blocking board: the button is wired active
    // low against a pull-up on the uFerris carrier board, so no internal pull is
    // configured here. With `gpiote` on, this `Input` is also what
    // `wait_for_sw5` waits on.
    let button = Input::new(peripherals.P1_07, Pull::None);

    // --------------------------------------
    //              PWM Setup
    // --------------------------------------

    // The buzzer sits on D2 / P1.06, driven by channel 0 of PWM20, exactly as
    // in `uferris_init`. `SetDutyCycle` is a blocking trait in both modes, so
    // this is unchanged.
    let mut pwm_config = SimpleConfig::default();
    pwm_config.prescaler = Prescaler::Div1;
    pwm_config.max_duty = BUZZER_COUNTERTOP;

    let pwm = SimplePwm::new_1ch(peripherals.PWM20, peripherals.P1_06, &pwm_config);
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
