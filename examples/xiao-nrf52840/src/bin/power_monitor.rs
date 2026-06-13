#![no_std]
#![no_main]

use embassy_executor::{Spawner, main};
use embassy_nrf::init;
use embassy_time::Timer;
use panic_halt as _;
use uferris_bsp::{SevenSegDigit, uferris_init};

#[main]
async fn main(_s: Spawner) {
    let p = init(Default::default());
    let mut uferris = uferris_init(p);
    let mut counter: i64;
    loop {
        counter = uferris.read_system_power().unwrap() / 1000;

        let digits = [
            if counter >= 1000 {
                Some(((counter / 1000) % 10) as u8)
            } else {
                None
            },
            if counter >= 100 {
                Some(((counter / 100) % 10) as u8)
            } else {
                None
            },
            if counter >= 10 {
                Some(((counter / 10) % 10) as u8)
            } else {
                None
            },
            Some((counter % 10) as u8),
        ];

        let seg_digits = [
            SevenSegDigit::Digit1,
            SevenSegDigit::Digit2,
            SevenSegDigit::Digit3,
            SevenSegDigit::Digit4,
        ];

        for _ in 0..50 {
            for (seg, val) in seg_digits.iter().zip(digits.iter()) {
                uferris.write_seven_segment_digit(*seg, *val).unwrap();
                Timer::after_millis(2).await;
                uferris.write_seven_segment_digit(*seg, None).unwrap();
            }
        }
    }
}
