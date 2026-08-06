use core::cell::RefCell;
use core::sync::atomic::{Ordering, compiler_fence};
use embassy_nrf::{
    Peripherals, bind_interrupts,
    gpio::{Input, Level, Output, OutputDrive, Pull},
    pac,
    pwm::{DutyCycle, Prescaler, SimpleConfig, SimplePwm},
    saadc::{self, ChannelConfig, Config as SaadcConfig, Saadc},
    twim::{self, Config as TwimConfig, Frequency as TwimFrequency, Twim},
};
use embedded_hal::delay::DelayNs;
use embedded_hal::pwm::{ErrorType as PwmErrorType, SetDutyCycle};
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use static_cell::StaticCell;

#[cfg(feature = "power-board")]
use embassy_nrf::spim::{self, Config as SpimConfig, Frequency as SpimFrequency, Spim};
#[cfg(feature = "power-board")]
use embedded_hal_bus::spi::RefCellDevice as SpiRefCellDevice;

use crate::Uferris;
use crate::components::ldr::OneShot;

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

/// Buzzer PWM output frequency, matching the other boards.
const BUZZER_FREQ_HZ: u32 = 2_700;

/// Frequency of `PWM_CLK` with the prescaler set to [`Prescaler::Div1`].
const PWM_CLK_HZ: u32 = 16_000_000;

/// nRF `COUNTERTOP` that produces [`BUZZER_FREQ_HZ`] from [`PWM_CLK_HZ`].
///
///     COUNTERTOP = 16_000_000 / 2700 = 5925.9 -> 5926
///     f_pwm      = 16_000_000 / 5926 = 2700.0 Hz
const BUZZER_COUNTERTOP: u16 = (PWM_CLK_HZ / BUZZER_FREQ_HZ) as u16;

/// Duty cycle resolution reported to application code.
///
/// The LEDC based boards run 14 bit duty cycles, so every board reports the
/// same 16384 and a duty value means the same thing everywhere. See
/// [`Nrf52840BuzzerChannel`] for how it is mapped onto the nRF countertop.
const BUZZER_MAX_DUTY: u16 = 16384;

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

// ------------------------------------------
// Type Defs
// ------------------------------------------

// I2C Types
type Nrf52840I2c = Twim<'static>;
type SharedI2c = I2cRefCellDevice<'static, Nrf52840I2c>;

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

// Buzzer Types

/// The buzzer PWM channel, presented with the duty cycle resolution the rest of
/// the boards use.
///
/// The nRF PWM period is
///     f_pwm = PWM_CLK / COUNTERTOP
/// where `PWM_CLK` is 16 MHz divided by the prescaler. There is no fractional
/// divider, so the only way to reach 2700 Hz is to pick the countertop, and
/// 16 MHz / 2700 Hz gives 5926 rather than a round power of two. Every other
/// board reports a 14 bit `max_duty_cycle()` of 16384, so this wrapper keeps
/// reporting 16384 and rescales incoming duty values onto the countertop:
///
///     raw = round(duty * 5926 / 16384)
///
/// The rounding is done in `u32` (the largest product is
/// `16384 * 5926 = 97_083_392`, well inside the range) by adding half the
/// divisor before dividing. `duty` 0 maps to 0 and `duty` 16384 maps to 5926,
/// so fully off and fully on stay exact and the worst case error in between is
/// half a countertop step, about 0.008% of the period.
pub struct Nrf52840BuzzerChannel {
    pwm: SimplePwm<'static>,
}

impl PwmErrorType for Nrf52840BuzzerChannel {
    type Error = core::convert::Infallible;
}

impl SetDutyCycle for Nrf52840BuzzerChannel {
    fn max_duty_cycle(&self) -> u16 {
        BUZZER_MAX_DUTY
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        let duty = duty.min(BUZZER_MAX_DUTY);

        // Round to nearest; see the type level comment for the ranges.
        let raw = (u32::from(duty) * u32::from(BUZZER_COUNTERTOP) + u32::from(BUZZER_MAX_DUTY) / 2)
            / u32::from(BUZZER_MAX_DUTY);

        // Bit 15 of an nRF PWM sequence word selects the polarity of the
        // compare match, and `DutyCycle` is that word. `DutyCycle::normal`
        // leaves the bit clear, which drives the output high once the counter
        // reaches the value, i.e. the value counts the *low* part of the
        // period. `DutyCycle::inverted` sets it, so the value is the high time
        // and a duty of 0 leaves the buzzer pin low. The buzzer is active high,
        // so `inverted` is the one that means what the caller expects.
        self.pwm.set_duty(0, DutyCycle::inverted(raw as u16));

        Ok(())
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
    LedPin,                // LED (D1)
    ButtonPin,             // Button (D3)
    Nrf52840BuzzerChannel, // Buzzer (D2)
    SharedI2c,             // I2C
    LdrAdc,                // LDR
    (),
>;

#[cfg(feature = "power-board")]
pub type UferrisNrf52840 = Uferris<
    LedPin,                // LED (D1)
    ButtonPin,             // Button (D3)
    Nrf52840BuzzerChannel, // Buzzer (D2)
    SharedI2c,             // I2C
    LdrAdc,                // LDR
    SdBlockDevice,         // SD Manager
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
/// need it, but it does mean the delays are coarse. An accurate timer backed
/// delay arrives with `async` support, when `embassy-time` comes along anyway.
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
/// `embassy-nrf` USB driver is `async` only. Applications therefore run silent
/// until `async` support lands.
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
    // `Nrf52840BuzzerChannel` for the countertop and duty cycle maths.
    let mut pwm_config = SimpleConfig::default();
    pwm_config.prescaler = Prescaler::Div1;
    pwm_config.max_duty = BUZZER_COUNTERTOP;

    let pwm = SimplePwm::new_1ch(peripherals.PWM0, peripherals.P0_28, &pwm_config);
    let mut buzzer_channel = Nrf52840BuzzerChannel { pwm };
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
