use core::cell::{Cell, RefCell};
use critical_section::Mutex;
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use rp2040_hal::{
    Sio, Watchdog,
    adc::{Adc, AdcPin},
    clocks::init_clocks_and_plls,
    fugit::RateExtU32,
    gpio::{
        FunctionI2c, FunctionSio, Pin, Pins, PullDown, PullNone, PullUp, SioInput, SioOutput,
        bank0::{Gpio6, Gpio7, Gpio26, Gpio27, Gpio29},
    },
    i2c::I2C,
    pac::{I2C1, Peripherals},
    pwm::{A, Channel, FreeRunning, Pwm6, Slice, Slices},
    timer::Timer,
};
use static_cell::StaticCell;

#[cfg(feature = "power-board")]
use embedded_hal_bus::spi::RefCellDevice as SpiRefCellDevice;
#[cfg(feature = "power-board")]
use rp2040_hal::{
    Spi,
    gpio::{
        FunctionSpi,
        bank0::{Gpio1, Gpio2, Gpio3, Gpio4},
    },
    pac::SPI0,
    spi::Enabled,
};

use crate::Uferris;
use crate::components::ldr::OneShot;

// ------------------------------------------
// Second Stage Bootloader
// ------------------------------------------

/// The RP2040 has no internal flash. Its boot ROM copies the first 256 bytes of
/// the external QSPI flash into SRAM and runs them, and that second stage is
/// what configures the flash for XIP. The `.boot2` section is placed at the
/// very start of flash by the application's `memory.x`.
///
/// The Xiao RP2040 carries a Winbond W25Q flash part, so `BOOT_LOADER_W25Q080`
/// is the correct second stage. Providing it is the BSP's job, so applications
/// do not have to depend on `rp2040-boot2` themselves.
#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// ------------------------------------------
// Board Constants
// ------------------------------------------

/// Crystal frequency fitted on the Xiao RP2040.
const XOSC_CRYSTAL_FREQ_HZ: u32 = 12_000_000;

/// System clock frequency produced by [`init_clocks_and_plls`].
const SYS_CLOCK_HZ: u32 = 125_000_000;

// ------------------------------------------
// Static Types
// ------------------------------------------
static I2C_BUS: StaticCell<RefCell<Rp2040I2c>> = StaticCell::new();
#[cfg(feature = "power-board")]
static SPI_BUS: StaticCell<RefCell<Rp2040Spi>> = StaticCell::new();

/// Copy of the free running timer minted during board init, handed out by
/// [`timer`].
static TIMER: Mutex<Cell<Option<Timer>>> = Mutex::new(Cell::new(None));

// ------------------------------------------
// Type Defs
// ------------------------------------------

// I2C Types
type I2cSdaPin = Pin<Gpio6, FunctionI2c, PullUp>;
type I2cSclPin = Pin<Gpio7, FunctionI2c, PullUp>;
type Rp2040I2c = I2C<I2C1, (I2cSdaPin, I2cSclPin)>;
type SharedI2c = I2cRefCellDevice<'static, Rp2040I2c>;

// GPIO Types
type LedPin = Pin<Gpio27, FunctionSio<SioOutput>, PullDown>;
type ButtonPin = Pin<Gpio29, FunctionSio<SioInput>, PullNone>;

// ADC Types
pub struct LdrAdc {
    adc: Adc,
    pin: AdcPin<Pin<Gpio26, FunctionSio<SioInput>, PullNone>>,
}

impl OneShot for LdrAdc {
    fn read_raw(&mut self) -> u16 {
        self.adc.read(&mut self.pin).unwrap_or(0)
    }
}

// Buzzer Types
pub type Rp2040BuzzerChannel = Channel<Slice<Pwm6, FreeRunning>, A>;

// SD/SPI Types
#[cfg(feature = "power-board")]
type SpiSckPin = Pin<Gpio2, FunctionSpi, PullDown>;
#[cfg(feature = "power-board")]
type SpiTxPin = Pin<Gpio3, FunctionSpi, PullDown>;
#[cfg(feature = "power-board")]
type SpiRxPin = Pin<Gpio4, FunctionSpi, PullDown>;
#[cfg(feature = "power-board")]
type Rp2040Spi = Spi<Enabled, SPI0, (SpiTxPin, SpiRxPin, SpiSckPin)>;

#[cfg(feature = "power-board")]
type SdCsPin = Pin<Gpio1, FunctionSio<SioOutput>, PullDown>;

#[cfg(feature = "power-board")]
type SdBlockDevice =
    embedded_sdmmc::SdCard<SpiRefCellDevice<'static, Rp2040Spi, SdCsPin, Timer>, Timer>;

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

/// The board's free running 1 MHz timer, usable as an `embedded-hal` delay
/// provider.
///
/// [`Timer`] is `Copy` and only ever reads the hardware counter, so every
/// caller gets its own handle onto the same timer.
///
/// # Panics
///
/// Panics if the board has not been initialized yet. Call [`uferris_init`]
/// first.
pub fn timer() -> Timer {
    critical_section::with(|cs| TIMER.borrow(cs).get())
        .expect("uferris_init must be called before timer()")
}

// ------------------------------------------
// Board Initialization Function
// ------------------------------------------

/// Initialize the uFerris board.
///
/// With the `usb-console` feature enabled this also brings up the USB CDC
/// console. Unlike the ESP32 Xiaos, the Xiao RP2040 has no USB-to-UART bridge,
/// so applications print over a USB CDC device the RP2040 provides itself.
/// Building one needs the USB controller *and* the `UsbClock` token minted
/// while the clock tree is set up, neither of which is reachable once
/// `Peripherals` has been moved in here, so the console is built from inside
/// the initializer. Printing then goes through the crate's `println!` macro,
/// and the device is serviced from `USBCTRL_IRQ` rather than by application
/// code.
pub fn uferris_init(mut peripherals: Peripherals) -> UferrisRp2040 {
    // --------------------------------------
    //             Clock Setup
    // --------------------------------------
    let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ_HZ,
        peripherals.XOSC,
        peripherals.CLOCKS,
        peripherals.PLL_SYS,
        peripherals.PLL_USB,
        &mut peripherals.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(peripherals.SIO);
    let pins = Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    let timer = Timer::new(peripherals.TIMER, &mut peripherals.RESETS, &clocks);
    critical_section::with(|cs| TIMER.borrow(cs).set(Some(timer)));

    // --------------------------------------
    //              ADC Setup
    // --------------------------------------
    let adc = Adc::new(peripherals.ADC, &mut peripherals.RESETS);
    let ldr_pin = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();
    let ldr_driver = LdrAdc { adc, pin: ldr_pin };

    // --------------------------------------
    //              I2C Setup
    // --------------------------------------
    let i2c = I2C::i2c1(
        peripherals.I2C1,
        pins.gpio6.reconfigure(),
        pins.gpio7.reconfigure(),
        100.kHz(),
        &mut peripherals.RESETS,
        SYS_CLOCK_HZ.Hz(),
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
    let led = pins.gpio27.into_push_pull_output();
    // Floating input, matching the other boards: the button is wired active low
    // against a pull-up on the uFerris carrier board, so no internal pull is
    // configured here.
    let button = pins.gpio29.into_floating_input();

    // --------------------------------------
    //              PWM Setup
    // --------------------------------------
    let pwm_slices = Slices::new(peripherals.PWM, &mut peripherals.RESETS);

    // The buzzer sits on D2 / GPIO28, which is slice 6 channel A.
    //
    // The RP2040 PWM output frequency is
    //     f_pwm = f_sys / (DIV * (TOP + 1))
    // where DIV is an 8.4 fixed point value (`div_int` + `div_frac` / 16).
    //
    // TOP is 16383 so that `max_duty_cycle()` reports 16384, i.e. the same
    // 14-bit resolution the LEDC based boards use. Duty values written by
    // application code therefore mean the same thing on every board.
    //
    // Hitting the 2700 Hz used by the other boards then needs
    //     DIV = 125_000_000 / (2700 * 16384) = 2.827
    // and the nearest representable value is 2 + 13/16 = 2.8125, giving
    //     f_pwm = 125_000_000 / (2.8125 * 16384) = 2713 Hz.
    let mut buzzer_slice = pwm_slices.pwm6;
    buzzer_slice.default_config();
    buzzer_slice.set_top(16383);
    buzzer_slice.set_div_int(2);
    buzzer_slice.set_div_frac(13);
    buzzer_slice.enable();

    let mut buzzer_channel = buzzer_slice.channel_a;
    buzzer_channel.output_to(pins.gpio28);

    // --------------------------------------
    //            SPI / SD Setup
    // --------------------------------------
    #[cfg(feature = "power-board")]
    let vol_mgr = {
        let spi_tx: SpiTxPin = pins.gpio3.reconfigure();
        let spi_rx: SpiRxPin = pins.gpio4.reconfigure();
        let spi_sck: SpiSckPin = pins.gpio2.reconfigure();

        let spi = Spi::<_, _, _, 8>::new(peripherals.SPI0, (spi_tx, spi_rx, spi_sck)).init(
            &mut peripherals.RESETS,
            SYS_CLOCK_HZ.Hz(),
            400.kHz(),
            embedded_hal::spi::MODE_0,
        );

        // Promote SPI Bus to Static
        let spi_bus_ref = SPI_BUS.init(RefCell::new(spi));

        // CS Pin
        let sd_cs = pins
            .gpio1
            .into_push_pull_output_in_state(rp2040_hal::gpio::PinState::High);

        // Create SPI Device (Borrows from SPI_BUS static)
        // We do NOT need to make this device static. SdCard owns it.
        // `Timer` is `Copy`, so the same instance backs both delay slots.
        let sd_device = SpiRefCellDevice::new(spi_bus_ref, sd_cs, timer).unwrap();

        // Create SD Card (Owns sd_device)
        let sd_card = embedded_sdmmc::SdCard::new(sd_device, timer);

        Some(embedded_sdmmc::VolumeManager::new(
            sd_card,
            crate::DummyTimeSource::default(),
        ))
    };

    // --------------------------------------
    //          Board Instantiation
    // --------------------------------------
    let uferris = Uferris::new(
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
    .unwrap();

    // --------------------------------------
    //            Console Setup
    // --------------------------------------
    #[cfg(feature = "usb-console")]
    crate::console::rp2040::init(
        peripherals.USBCTRL_REGS,
        peripherals.USBCTRL_DPRAM,
        clocks.usb_clock,
        &mut peripherals.RESETS,
    );

    uferris
}
