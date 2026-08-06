#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with embassy_nrf types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embedded_hal::delay::DelayNs;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use uferris_bsp::boards::xiao_nrf54l15::delay;
use uferris_bsp::{SevenSegDigit, uferris_init};

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[cortex_m_rt::entry]
fn main() -> ! {
    // Bring the RTT console up first, so that everything after this point —
    // including a panic — has somewhere to print to.
    rtt_init_print!();

    // `embassy-nrf` is driven in blocking mode here: `init` selects the core
    // clock, resets the FLPR coprocessor and hands back the peripheral
    // singletons, so a plain `cortex-m-rt` entry point is all this needs.
    let peripherals = embassy_nrf::init(Default::default());

    let mut uferris = uferris_init(peripherals);

    let mut delay = delay();

    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    // Set RTC to Known Time for Testing
    uferris.set_rtc_time(2026, 1, 29, 9, 16, 0).unwrap();

    loop {
        // ---------------------------------------
        // LED Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        rprintln!(
            "Commencing LED Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3,
            datetime.4,
            datetime.5,
            datetime.2,
            datetime.1,
            datetime.0
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
        rprintln!(
            "Commencing Seven Segment Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3,
            datetime.4,
            datetime.5,
            datetime.2,
            datetime.1,
            datetime.0
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
        rprintln!(
            "Commencing Buzzer Testing at {}:{}:{} on {}/{}/{}",
            datetime.2,
            datetime.1,
            datetime.0,
            datetime.5,
            datetime.4,
            datetime.3
        );

        delay.delay_ms(TEST_DELAY_MS);

        uferris.buzz_on(1024);

        delay.delay_ms(2000);

        uferris.buzz_off();

        // ---------------------------------------
        // LDR Testing
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        rprintln!(
            "Commencing LDR Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3,
            datetime.4,
            datetime.5,
            datetime.2,
            datetime.1,
            datetime.0
        );
        rprintln!("Taking Three Readings with 3s Delay Between Readings");
        let ldr_value = uferris.read_ldr();
        rprintln!("First LDR Reading: {}", ldr_value);
        rprintln!("Waiting 3 Seconds...");
        delay.delay_ms(3000);
        let ldr_value = uferris.read_ldr();
        rprintln!("Second LDR Reading: {}", ldr_value);
        rprintln!("Waiting 3 Seconds...");
        delay.delay_ms(3000);
        let ldr_value = uferris.read_ldr();
        rprintln!("Third LDR Reading: {}", ldr_value);
        rprintln!("LDR Reading Test Complete.");

        // ---------------------------------------
        // Button Press Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        rprintln!(
            "Commencing Button Press Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3,
            datetime.4,
            datetime.5,
            datetime.2,
            datetime.1,
            datetime.0
        );

        rprintln!("Button Press Testing");
        rprintln!("Press Button 1 to Proceed...");
        while !uferris.read_sw1().unwrap() {}
        rprintln!("Button 1 Pressed! Continuing...");
        rprintln!("Press Button 2 to Proceed...");
        while !uferris.read_sw2().unwrap() {}
        rprintln!("Button 2 Pressed! Continuing...");
        rprintln!("Press Button 3 to Proceed...");
        while !uferris.read_sw3().unwrap() {}
        rprintln!("Button 3 Pressed! Continuing...");
        rprintln!("Press Button 4 to Proceed...");
        while !uferris.read_sw4().unwrap() {}
        rprintln!("Button 4 Pressed! Continuing...");
        rprintln!("Press Button 5 to Proceed...");
        while !uferris.read_sw5() {}
        rprintln!("Button 5 Pressed! Continuing...");

        // ---------------------------------------
        // Slide Switch Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        rprintln!(
            "Commencing Slide Switch Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3,
            datetime.4,
            datetime.5,
            datetime.2,
            datetime.1,
            datetime.0
        );

        rprintln!("Slide Switch Testing");
        rprintln!("Starting with Switch 6 (Left Switch)...");
        let current_sw6_state = uferris.read_sw6().unwrap();
        let mut new_sw6_state = uferris.read_sw6().unwrap();
        rprintln!("Current Slide Switch 6 State: {:?}", current_sw6_state);
        rprintln!("Toggle Slide Switch 6 to Proceed...");
        while current_sw6_state == new_sw6_state {
            new_sw6_state = uferris.read_sw6().unwrap();
        }
        rprintln!("Slide Switch 6 State Changed");
        rprintln!("Current Slide Switch 6 State: {:?}", new_sw6_state);

        delay.delay_ms(2000);

        rprintln!("Moving to Switch 7 (Right Switch)...");
        let current_sw7_state = uferris.read_sw7().unwrap();
        let mut new_sw7_state = uferris.read_sw7().unwrap();
        rprintln!("Current Slide Switch 7 State: {:?}", current_sw7_state);
        rprintln!("Toggle Slide Switch 7 to Proceed...");
        while current_sw7_state == new_sw7_state {
            new_sw7_state = uferris.read_sw7().unwrap();
        }
        rprintln!("Slide Switch 7 State Changed");
        rprintln!("Current Slide Switch 7 State: {:?}", new_sw7_state);
        delay.delay_ms(2000);

        rprintln!("Testing Loop Complete. Restarting...");
    }
}
