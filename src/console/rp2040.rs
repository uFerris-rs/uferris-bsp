//! RP2040 USB CDC console backend.
//!
//! The Xiao RP2040 has no USB-to-UART bridge, so printing goes over a USB CDC
//! device the RP2040 provides itself. The device is serviced from the
//! `USBCTRL_IRQ` handler below rather than from application code, so a demo can
//! sit in a plain `while !uferris.read_sw1()? {}` loop without the host
//! dropping the connection.
//!
//! The handler and the print path share the device through a
//! [`critical_section::Mutex`], which is multicore safe on the RP2040 because
//! `rp2040-hal`'s `critical-section-impl` takes a hardware spinlock.

use core::cell::RefCell;

use critical_section::{CriticalSection, Mutex};
use embedded_hal::delay::DelayNs;
use rp2040_hal::clocks::UsbClock;
use rp2040_hal::pac::{Interrupt, NVIC, RESETS, USBCTRL_DPRAM, USBCTRL_REGS, interrupt};
use rp2040_hal::usb::UsbBus;
use static_cell::StaticCell;
use usb_device::UsbError;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;

use crate::boards::xiao_rp2040::timer;

// ------------------------------------------
// Console Constants
// ------------------------------------------

/// USB identifiers. `16c0:27dd` is the pid.codes CDC-ACM pair the `rp-hal`
/// examples use for test devices.
const USB_VID_PID: UsbVidPid = UsbVidPid(0x16c0, 0x27dd);

/// Bytes offered to the CDC endpoint per attempt. The bulk endpoint is 64 bytes
/// wide, so handing it more in one go buys nothing.
const CHUNK_LEN: usize = 64;

/// How long the host has to settle after opening the port, in milliseconds.
const HOST_SETTLE_MS: u32 = 100;

/// How many attempts a single chunk gets before the rest of the line is
/// dropped. Each attempt hands a millisecond back with interrupts enabled, so
/// this puts a ceiling of roughly 100 ms on a chunk and keeps a host that
/// stopped reading from wedging the application.
const MAX_STALLS: u32 = 100;

// ------------------------------------------
// Static State
// ------------------------------------------

/// The bus allocator has to outlive the device and the serial class, so it is
/// promoted to `'static` here instead of living on the caller's stack.
static USB_BUS: StaticCell<UsbBusAllocator<UsbBus>> = StaticCell::new();

/// Everything `USBCTRL_IRQ` and the print path share.
struct Console {
    device: UsbDevice<'static, UsbBus>,
    serial: SerialPort<'static, UsbBus>,
}

static CONSOLE: Mutex<RefCell<Option<Console>>> = Mutex::new(RefCell::new(None));

// ------------------------------------------
// Console Initialization
// ------------------------------------------

/// Build the USB CDC device and start servicing it from `USBCTRL_IRQ`.
///
/// Called by [`uferris_init`](crate::boards::xiao_rp2040::uferris_init), which
/// still owns the USB peripherals at that point, so applications never call
/// this themselves.
#[allow(
    clippy::large_stack_frames,
    reason = "the USB device builder holds its descriptors on the stack while the device \
    is assembled"
)]
pub(crate) fn init(
    usb_ctrl_reg: USBCTRL_REGS,
    usb_dpram: USBCTRL_DPRAM,
    usb_clock: UsbClock,
    resets: &mut RESETS,
) {
    let usb_bus = USB_BUS.init(UsbBusAllocator::new(UsbBus::new(
        usb_ctrl_reg,
        usb_dpram,
        usb_clock,
        true,
        resets,
    )));

    // The serial class has to be allocated before the device is built.
    let serial = SerialPort::new(usb_bus);
    let device = UsbDeviceBuilder::new(usb_bus, USB_VID_PID)
        .strings(&[StringDescriptors::default()
            .manufacturer("uFerris")
            .product("µFerris Xiao RP2040")
            .serial_number("uferris-rp2040")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    critical_section::with(|cs| {
        CONSOLE.borrow(cs).replace(Some(Console { device, serial }));
    });

    // Only safe to unmask once the console is in place, since that is the one
    // thing the handler touches.
    unsafe { NVIC::unmask(Interrupt::USBCTRL_IRQ) };
}

// ------------------------------------------
// Interrupt Handler
// ------------------------------------------

/// Services the USB stack.
///
/// The device has to be polled every few milliseconds or the host drops the
/// connection. Doing it here is what frees application code from polling.
#[allow(non_snake_case)]
#[interrupt]
fn USBCTRL_IRQ() {
    critical_section::with(|cs| {
        let mut console = CONSOLE.borrow_ref_mut(cs);
        let Some(console) = console.as_mut() else {
            return;
        };

        if console.device.poll(&mut [&mut console.serial]) {
            // The console is output only, but anything the host types still has
            // to come off the endpoint or the host stops sending.
            let mut discard = [0u8; CHUNK_LEN];
            let _ = console.serial.read(&mut discard);
        }
    });
}

// ------------------------------------------
// Public API
// ------------------------------------------

/// Block until a terminal on the host opens the port, then print the banner.
///
/// Examples call this before their first [`println!`](crate::println) so the
/// opening lines are not lost while the device is still enumerating.
pub fn wait_for_host() {
    while !dtr() {}

    // Give the host terminal a moment to settle before the first line.
    delay_ms(HOST_SETTLE_MS);

    crate::println!("µFerris Xiao RP2040 console connected");
}

/// Write a formatted line, terminated with CRLF for terminal friendliness.
///
/// Implementation detail of the [`println!`](crate::println) macro.
#[doc(hidden)]
pub fn _print(args: core::fmt::Arguments<'_>) {
    let _ = core::fmt::Write::write_fmt(&mut ConsoleWriter, args);
    write_bytes(b"\r\n");
    flush();
}

// ------------------------------------------
// Write Path
// ------------------------------------------

/// `core::fmt` sink that pushes straight into the CDC endpoint.
struct ConsoleWriter;

impl core::fmt::Write for ConsoleWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        write_bytes(s.as_bytes());
        Ok(())
    }
}

/// Push `data` out of the CDC endpoint, one chunk at a time.
///
/// Writing from thread context while `USBCTRL_IRQ` pumps the endpoint is the
/// established `usb-device` arrangement. Each attempt holds the lock for a
/// single chunk and then hands the time back with interrupts enabled, so the
/// handler always gets its chance to run.
///
/// Output is best effort: if no terminal is attached, or the endpoint stays
/// full for [`MAX_STALLS`] attempts, the rest of the data is dropped rather
/// than blocking the application forever.
fn write_bytes(mut data: &[u8]) {
    let mut stalls = 0;

    while !data.is_empty() {
        let chunk = &data[..data.len().min(CHUNK_LEN)];

        match critical_section::with(|cs| write_chunk(cs, chunk)) {
            Some(0) => {
                stalls += 1;
                if stalls > MAX_STALLS {
                    return;
                }
                // Leave interrupts enabled for a moment so `USBCTRL_IRQ` can
                // drain the endpoint before the next attempt.
                delay_ms(1);
            }
            Some(count) => {
                data = &data[count..];
                stalls = 0;
            }
            None => return,
        }
    }
}

/// Offer one chunk to the serial class.
///
/// `Some(0)` means the endpoint was busy and the chunk should be retried,
/// `None` means the output should be dropped.
fn write_chunk(cs: CriticalSection<'_>, chunk: &[u8]) -> Option<usize> {
    let mut console = CONSOLE.borrow_ref_mut(cs);
    let console = console.as_mut()?;

    if !console.serial.dtr() {
        // No terminal attached, drop the output rather than block.
        return None;
    }

    match console.serial.write(chunk) {
        Ok(count) => Some(count),
        Err(UsbError::WouldBlock) => Some(0),
        Err(_) => None,
    }
}

/// Nudge the serial class to hand whatever it is holding to the endpoint.
fn flush() {
    critical_section::with(|cs| {
        if let Some(console) = CONSOLE.borrow_ref_mut(cs).as_mut() {
            let _ = console.serial.flush();
        }
    });
}

/// Whether a terminal on the host currently has the port open.
fn dtr() -> bool {
    critical_section::with(|cs| {
        CONSOLE
            .borrow_ref(cs)
            .as_ref()
            .is_some_and(|console| console.serial.dtr())
    })
}

/// Delay using the free running timer minted during board init.
fn delay_ms(ms: u32) {
    timer().delay_ms(ms);
}
