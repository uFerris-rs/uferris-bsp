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
    let mut counter: u16;

    let seg_digits = [
        uferris_bsp::SevenSegDigit::Digit1,
        uferris_bsp::SevenSegDigit::Digit2,
        uferris_bsp::SevenSegDigit::Digit3,
        uferris_bsp::SevenSegDigit::Digit4,
    ];

    loop {
        counter = uferris.read_ldr();

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

        for _ in 0..50 {
            for (seg, val) in seg_digits.iter().zip(digits.iter()) {
                uferris.write_seven_segment_digit(*seg, *val).unwrap();
                Timer::after_millis(2).await;
                uferris.write_seven_segment_digit(*seg, None).unwrap();
            }
        }
    }
}
