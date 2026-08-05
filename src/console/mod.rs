//! USB CDC Serial Console
//!
//! Not every Xiao controller carries a USB-to-UART bridge. On the ones that do
//! not, applications print over a USB CDC serial device the controller itself
//! provides, and that device has to be serviced every few milliseconds or the
//! host drops the connection. This module owns that machinery so application
//! code never has to poll anything.
//!
//! Everything here sits behind the `usb-console` feature.
//!
//! ## Layout
//! There is one backend module per controller family, plus the re-exports
//! below. The re-exports are what the [`println!`](crate::println) macro and
//! the examples talk to, so a print means the same thing on every board.
//! Adding a controller is a matter of dropping in another backend module that
//! exposes `_print` and `wait_for_host`, and adding another re-export arm.

#[cfg(feature = "xiao-rp2040")]
pub mod rp2040;

#[cfg(feature = "xiao-rp2040")]
pub use rp2040::{_print, wait_for_host};

/// Print a formatted line to the USB CDC console, `std::println!` style.
///
/// Lines are terminated with CRLF, which is what serial terminals expect.
///
/// Output is best effort: if no terminal has the port open, or the host stops
/// draining the endpoint, the line is dropped rather than blocking the
/// application. Call [`wait_for_host`] first if the opening lines matter.
///
/// ```ignore
/// use uferris_bsp::{println, uferris_init, wait_for_host};
///
/// let mut uferris = uferris_init(peripherals);
/// wait_for_host();
/// println!("LDR Reading: {}", uferris.read_ldr());
/// ```
#[macro_export]
macro_rules! println {
    () => {
        $crate::console::_print(format_args!(""))
    };
    ($($arg:tt)*) => {
        $crate::console::_print(format_args!($($arg)*))
    };
}
