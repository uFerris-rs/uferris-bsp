#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with embassy_nrf types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use panic_halt as _;
use uferris_bsp::boards::xiao_nrf52840::delay;
use uferris_bsp::uferris_init;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[cortex_m_rt::entry]
fn main() -> ! {
    // `embassy-nrf` is driven in blocking mode here: `init` only applies the
    // chip errata workarounds and hands back the peripheral singletons, so a
    // plain `cortex-m-rt` entry point is all this needs.
    let peripherals = embassy_nrf::init(Default::default());

    let mut uferris = uferris_init(peripherals);

    let mut delay = delay();

    // The demo itself is shared by every board: see `examples/uferris-demos`.
    uferris_demos::blinky(&mut uferris, &mut delay)
}
