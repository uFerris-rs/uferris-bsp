#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::init;
use embassy_time::Timer;
use panic_halt as _;
use uferris_bsp::uferris_init;

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let p = init(Default::default());
    let mut uferris = uferris_init(p);
    let delay: u64 = 300;
    loop {
        uferris.led3_off().unwrap();
        uferris.led1_on();
        Timer::after_millis(delay).await;
        uferris.led1_off();
        uferris.led2_on().unwrap();
        Timer::after_millis(delay).await;
        uferris.led2_off().unwrap();
        uferris.led3_on().unwrap();
        Timer::after_millis(delay).await;
    }
}
