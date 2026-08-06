#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with embassy_nrf types, especially those \
    holding buffers for the duration of a data transfer."
)]
// `clippy::large_stack_frames` is denied in the blocking examples and allowed
// on their `main`, because it is normal for `main` to be the one big frame in
// the program. That pair does not work here: `#[embassy_executor::main]` moves
// the whole body into a task function of its own, the lint measures *that*
// function, and neither an outer nor an inner attribute on `main` can reach it.
// The frame it is measuring is the demo's own task, which the thread mode
// executor runs on the main stack as the only task in the program, so it is
// `main`'s frame under another name.

use embassy_executor::Spawner;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use uferris_bsp::uferris_init_async;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    // Bring the RTT console up first, so that everything after this point —
    // including a panic — has somewhere to print to.
    rtt_init_print!();

    // `embassy_nrf::init` does the same job it does in the blocking examples:
    // select the core clock, reset the FLPR coprocessor and hand back the
    // peripheral singletons. What is different here is what the crate was built
    // with — the `async` feature turns on the GRTC time driver behind
    // `embassy-time`, and the GPIOTE interrupt `init` also brings up, which is
    // what lets the board wait on a pin.
    let peripherals = embassy_nrf::init(Default::default());

    let mut uferris = uferris_init_async(peripherals).await;

    // The delay is the application's, not the board's: `embassy-time`'s, backed
    // by the GRTC driver. The blocking examples count CPU cycles instead.
    let mut delay = embassy_time::Delay;

    rprintln!("Blinking LED 1");

    // The demo itself is shared by every board: see `examples/uferris-demos`.
    uferris_demos::asynch::blinky(&mut uferris, &mut delay).await
}
