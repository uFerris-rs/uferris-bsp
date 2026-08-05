#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with rp2040_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embedded_hal::delay::DelayNs;
use panic_halt as _;
use rp2040_hal::pac;
use uferris_bsp::boards::xiao_rp2040::timer;
use uferris_bsp::{uferris_init, wait_for_host};

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[rp2040_hal::entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let mut uferris = uferris_init(peripherals);

    let mut delay = timer();

    // Hold off until a terminal opens the USB CDC port, so the board is not
    // already blinking by the time the monitor comes up.
    wait_for_host();

    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    loop {
        uferris.led1_on();
        delay.delay_ms(TEST_DELAY_MS);

        uferris.led1_off();
        delay.delay_ms(TEST_DELAY_MS);
    }
}
