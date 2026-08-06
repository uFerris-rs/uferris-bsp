#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with embassy_rp types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embedded_hal::delay::DelayNs;
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

    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    loop {
        uferris.led1_on();
        delay.delay_ms(TEST_DELAY_MS);

        uferris.led1_off();
        delay.delay_ms(TEST_DELAY_MS);
    }
}
