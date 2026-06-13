#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::init;
use embassy_time::{Duration, Timer};
use panic_halt as _;
use uferris_bsp::uferris_init;

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let p = init(Default::default());
    let mut uferris = uferris_init(p);
    let delay: u64 = 300;
    loop {
        uferris.led1_on();
        Timer::after_millis(delay).await;
    }
}
