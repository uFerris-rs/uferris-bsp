#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with embassy_nrf types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embedded_hal::delay::DelayNs;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use uferris_bsp::boards::xiao_nrf54l15::delay;
use uferris_bsp::uferris_init;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[cortex_m_rt::entry]
fn main() -> ! {
    // Bring the RTT console up first, so that everything after this point —
    // including a panic — has somewhere to print to.
    rtt_init_print!();

    // `embassy-nrf` is driven in blocking mode here: `init` selects the core
    // clock, resets the FLPR coprocessor and hands back the peripheral
    // singletons, so a plain `cortex-m-rt` entry point is all this needs.
    let peripherals = embassy_nrf::init(Default::default());

    let mut uferris = uferris_init(peripherals);

    let mut delay = delay();

    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    rprintln!("Blinking LED 1");

    loop {
        uferris.led1_on();
        delay.delay_ms(TEST_DELAY_MS);

        uferris.led1_off();
        delay.delay_ms(TEST_DELAY_MS);
    }
}
