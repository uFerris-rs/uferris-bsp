#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with embassy_rp types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use panic_halt as _;
use uferris_bsp::boards::xiao_rp2350::delay;
use uferris_bsp::uferris_init;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[cortex_m_rt::entry]
fn main() -> ! {
    // `embassy-rp` is driven in blocking mode here: `init` only sets up the
    // clock tree and hands back the peripheral singletons, so a plain
    // `cortex-m-rt` entry point is all this needs.
    let peripherals = embassy_rp::init(Default::default());

    let mut uferris = uferris_init(peripherals);

    let mut delay = delay();

    // The demo itself is shared by every board: see `examples/uferris-demos`.
    // There is no console on this board yet, so `NoopWriter` swallows the text
    // the demo prints. Every phase still runs and every peripheral is still
    // exercised. See the README.
    uferris_demos::full_board_demo(&mut uferris, &mut delay, &mut uferris_demos::NoopWriter)
}
