#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with embassy_rp types, especially those \
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
use panic_halt as _;
use uferris_bsp::uferris_init_async;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    // `embassy_rp::init` does the same job it does in the blocking examples:
    // set up the clock tree and hand back the peripheral singletons. What is
    // different here is what the crate was built with — the `async` feature
    // turns on the timer time driver behind `embassy-time` — and that `init`
    // also enables the `IO_IRQ_BANK0` vector `embassy-rp` claims for itself,
    // which is what lets the board wait on a pin.
    let peripherals = embassy_rp::init(Default::default());

    let mut uferris = uferris_init_async(peripherals).await;

    // The delay is the application's, not the board's: `embassy-time`'s, backed
    // by the RP2040 timer. The blocking examples count CPU cycles instead.
    let mut delay = embassy_time::Delay;

    // The demo itself is shared by every board: see `examples/uferris-demos`.
    // There is still no console on this board, so `NoopWriter` swallows the text
    // the demo prints. Every phase still runs and every peripheral is still
    // exercised. The button 5 phase is the one to watch: it suspends on the pin
    // rather than spinning on it. See the README.
    uferris_demos::asynch::baseboard_demo(&mut uferris, &mut delay, &mut uferris_demos::NoopWriter)
        .await
}
