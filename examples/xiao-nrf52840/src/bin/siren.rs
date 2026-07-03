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
    let max_duty = 5926;
    let min_duty = 0;

    loop {
        for duty in min_duty..max_duty {
            uferris.buzz_on(duty);
            Timer::after_millis(1).await;
        }
        for duty in (min_duty..max_duty).rev() {
            uferris.buzz_on(duty);
            Timer::after_millis(1).await;
        }
    }
}
