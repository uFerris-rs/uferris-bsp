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
use uferris_bsp::{SevenSegDigit, println, uferris_init, wait_for_host};

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[rp2040_hal::entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();

    let mut uferris = uferris_init(peripherals);

    let mut delay = timer();

    // Hold off until a terminal opens the USB CDC port, so none of the output
    // below is lost while the device is still enumerating.
    wait_for_host();

    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    // Set RTC to Known Time for Testing
    uferris.set_rtc_time(2026, 1, 29, 9, 16, 0).unwrap();

    loop {
        // ---------------------------------------
        // LED Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing LED Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );
        uferris.led1_on();
        delay.delay_ms(TEST_DELAY_MS);

        uferris.led1_off();
        delay.delay_ms(TEST_DELAY_MS);

        uferris.led2_on().unwrap();
        delay.delay_ms(TEST_DELAY_MS);

        uferris.led2_off().unwrap();
        delay.delay_ms(TEST_DELAY_MS);

        uferris.led3_on().unwrap();
        delay.delay_ms(TEST_DELAY_MS);

        uferris.led3_off().unwrap();
        delay.delay_ms(TEST_DELAY_MS);

        // ---------------------------------------
        // Seven Segment Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing Seven Segment Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit1, Some(8))
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS);

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit1, None)
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS);

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit2, Some(8))
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS);

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit2, None)
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS);

        uferris.seven_segment_display_colon_en(true).unwrap();

        delay.delay_ms(TEST_DELAY_MS);

        uferris.seven_segment_display_colon_en(false).unwrap();

        delay.delay_ms(TEST_DELAY_MS);

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit3, Some(8))
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS);

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit3, None)
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS);

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit4, Some(8))
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS);

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit4, None)
            .unwrap();

        // ---------------------------------------
        // Buzzer Test
        // ---------------------------------------

        let datetime = uferris.rtc.read_time().unwrap();
        println!(
            "Commencing Buzzer Testing at {}:{}:{} on {}/{}/{}",
            datetime.2, datetime.1, datetime.0, datetime.5, datetime.4, datetime.3
        );

        delay.delay_ms(TEST_DELAY_MS);

        uferris.buzz_on(1024);

        delay.delay_ms(2000);

        uferris.buzz_off();

        // ---------------------------------------
        // LDR Testing
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing LDR Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );
        println!("Taking Three Readings with 3s Delay Between Readings");
        let ldr_value = uferris.read_ldr();
        println!("First LDR Reading: {}", ldr_value);
        println!("Waiting 3 Seconds...");
        delay.delay_ms(3000);
        let ldr_value = uferris.read_ldr();
        println!("Second LDR Reading: {}", ldr_value);
        println!("Waiting 3 Seconds...");
        delay.delay_ms(3000);
        let ldr_value = uferris.read_ldr();
        println!("Third LDR Reading: {}", ldr_value);
        println!("LDR Reading Test Complete.");

        // ---------------------------------------
        // Button Press Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing Button Press Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        println!("Button Press Testing");
        println!("Press Button 1 to Proceed...");
        while !uferris.read_sw1().unwrap() {}
        println!("Button 1 Pressed! Continuing...");
        println!("Press Button 2 to Proceed...");
        while !uferris.read_sw2().unwrap() {}
        println!("Button 2 Pressed! Continuing...");
        println!("Press Button 3 to Proceed...");
        while !uferris.read_sw3().unwrap() {}
        println!("Button 3 Pressed! Continuing...");
        println!("Press Button 4 to Proceed...");
        while !uferris.read_sw4().unwrap() {}
        println!("Button 4 Pressed! Continuing...");
        println!("Press Button 5 to Proceed...");
        while !uferris.read_sw5() {}
        println!("Button 5 Pressed! Continuing...");

        // ---------------------------------------
        // Slide Switch Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        println!(
            "Commencing Slide Switch Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        println!("Slide Switch Testing");
        println!("Starting with Switch 6 (Left Switch)...");
        let current_sw6_state = uferris.read_sw6().unwrap();
        let mut new_sw6_state = uferris.read_sw6().unwrap();
        println!("Current Slide Switch 6 State: {:?}", current_sw6_state);
        println!("Toggle Slide Switch 6 to Proceed...");
        while current_sw6_state == new_sw6_state {
            new_sw6_state = uferris.read_sw6().unwrap();
        }
        println!("Slide Switch 6 State Changed");
        println!("Current Slide Switch 6 State: {:?}", new_sw6_state);

        delay.delay_ms(2000);

        println!("Moving to Switch 7 (Right Switch)...");
        let current_sw7_state = uferris.read_sw7().unwrap();
        let mut new_sw7_state = uferris.read_sw7().unwrap();
        println!("Current Slide Switch 7 State: {:?}", current_sw7_state);
        println!("Toggle Slide Switch 7 to Proceed...");
        while current_sw7_state == new_sw7_state {
            new_sw7_state = uferris.read_sw7().unwrap();
        }
        println!("Slide Switch 7 State Changed");
        println!("Current Slide Switch 7 State: {:?}", new_sw7_state);
        delay.delay_ms(2000);

        println!("Testing Loop Complete. Restarting...");
    }
}
