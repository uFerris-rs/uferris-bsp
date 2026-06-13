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

    let mut counter: i32 = 0;

    let mut last_sw3 = false;
    let mut last_sw2 = false;

    loop {
        let sw3_now = uferris.read_sw3().unwrap_or(false);
        let sw2_now = uferris.read_sw2().unwrap_or(false);
        let sw5_now = uferris.read_sw5();

        if sw3_now && !last_sw3 {
            counter = (counter + 1).min(9999);
        }
        if sw2_now && !last_sw2 {
            counter = (counter - 1).max(0);
        }
        if sw5_now {
            counter = 0;
        }

        last_sw3 = sw3_now;
        last_sw2 = sw2_now;

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
