//! # Shared uFerris `async` Example Demos
//!
//! The `async` counterparts of the demo bodies in the crate root. Same board,
//! same sequence of phases, same printed text — the difference is that every
//! board operation that has to reach the I2C bus or the ADC is awaited, and
//! that the delays come from [`embedded_hal_async::delay::DelayNs`] rather than
//! the blocking one, so the executor is free to run something else while the
//! demo is waiting.
//!
//! The module is called `asynch` because `async` is a reserved word and cannot
//! name a module.
//!
//! There is no `async` `full_board_demo`: the power board API is blocking-only
//! for now. See [`uferris_bsp::Mode`].

use core::fmt::Write;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use uferris_bsp::components::ldr::OneShotAsync;
use uferris_bsp::{Async, PowerConstraints, SevenSegDigit, Uferris};

/// Blink LED 1 forever.
///
/// This demo prints nothing, so it takes no writer.
pub async fn blinky<LED, BTN, BUZZ, I2C, ADC, BD>(
    uferris: &mut Uferris<LED, BTN, BUZZ, I2C, ADC, BD, Async>,
    delay: &mut impl DelayNs,
) -> !
where
    LED: OutputPin,
    BTN: Wait,
    BUZZ: SetDutyCycle,
    I2C: I2c,
    ADC: OneShotAsync,
    BD: PowerConstraints,
{
    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    loop {
        uferris.led1_on();
        delay.delay_ms(TEST_DELAY_MS).await;

        uferris.led1_off();
        delay.delay_ms(TEST_DELAY_MS).await;
    }
}

/// Exercise every component on the uFerris base board, forever.
///
/// The phases run in order: LEDs, seven segment display, buzzer, LDR, the five
/// push buttons and the two slide switches.
///
/// The button 5 phase is what this whole mode is for. The blocking demo spins
/// on `read_sw5` until somebody presses the button, burning the core for as
/// long as it takes; here the same phase is a single
/// [`Uferris::wait_for_sw5`] await, so the task suspends and the executor is
/// free to run anything else in the meantime. Buttons 1 to 4 hang off the I/O
/// expander on the far side of the I2C bus, with no interrupt line back to the
/// controller, so those phases still poll — they just poll with awaits.
pub async fn baseboard_demo<LED, BTN, BUZZ, I2C, ADC, BD>(
    uferris: &mut Uferris<LED, BTN, BUZZ, I2C, ADC, BD, Async>,
    delay: &mut impl DelayNs,
    w: &mut impl Write,
) -> !
where
    LED: OutputPin,
    BTN: Wait,
    BUZZ: SetDutyCycle,
    I2C: I2c,
    ADC: OneShotAsync,
    BD: PowerConstraints,
{
    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    // Set RTC to Known Time for Testing
    uferris.set_rtc_time(2026, 1, 29, 9, 16, 0).await.unwrap();

    loop {
        // ---------------------------------------
        // LED Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().await.unwrap();
        let _ = writeln!(
            w,
            "Commencing LED Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );
        uferris.led1_on();
        delay.delay_ms(TEST_DELAY_MS).await;

        uferris.led1_off();
        delay.delay_ms(TEST_DELAY_MS).await;

        uferris.led2_on().await.unwrap();
        delay.delay_ms(TEST_DELAY_MS).await;

        uferris.led2_off().await.unwrap();
        delay.delay_ms(TEST_DELAY_MS).await;

        uferris.led3_on().await.unwrap();
        delay.delay_ms(TEST_DELAY_MS).await;

        uferris.led3_off().await.unwrap();
        delay.delay_ms(TEST_DELAY_MS).await;

        // ---------------------------------------
        // Seven Segment Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().await.unwrap();
        let _ = writeln!(
            w,
            "Commencing Seven Segment Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit1, Some(8))
            .await
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit1, None)
            .await
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit2, Some(8))
            .await
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit2, None)
            .await
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS).await;

        uferris.seven_segment_display_colon_en(true).await.unwrap();

        delay.delay_ms(TEST_DELAY_MS).await;

        uferris.seven_segment_display_colon_en(false).await.unwrap();

        delay.delay_ms(TEST_DELAY_MS).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit3, Some(8))
            .await
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit3, None)
            .await
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit4, Some(8))
            .await
            .unwrap();

        delay.delay_ms(TEST_DELAY_MS).await;

        uferris
            .write_seven_segment_digit(SevenSegDigit::Digit4, None)
            .await
            .unwrap();

        // ---------------------------------------
        // Buzzer Test
        // ---------------------------------------

        // The blocking demo reaches through `uferris.rtc` here rather than
        // going through the board method, and prints the tuple in a different
        // order than the other phases do. The odd argument order is kept
        // verbatim so that the two demos print the same thing; the field access
        // is not, because the RTC's mode-specific methods are reached through
        // the board.
        let datetime = uferris.read_rtc_time().await.unwrap();
        let _ = writeln!(
            w,
            "Commencing Buzzer Testing at {}:{}:{} on {}/{}/{}",
            datetime.2, datetime.1, datetime.0, datetime.5, datetime.4, datetime.3
        );

        delay.delay_ms(TEST_DELAY_MS).await;

        uferris.buzz_on(1024);

        delay.delay_ms(2000).await;

        uferris.buzz_off();

        // ---------------------------------------
        // LDR Testing
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().await.unwrap();
        let _ = writeln!(
            w,
            "Commencing LDR Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );
        let _ = writeln!(w, "Taking Three Readings with 3s Delay Between Readings");
        let ldr_value = uferris.read_ldr().await;
        let _ = writeln!(w, "First LDR Reading: {}", ldr_value);
        let _ = writeln!(w, "Waiting 3 Seconds...");
        delay.delay_ms(3000).await;
        let ldr_value = uferris.read_ldr().await;
        let _ = writeln!(w, "Second LDR Reading: {}", ldr_value);
        let _ = writeln!(w, "Waiting 3 Seconds...");
        delay.delay_ms(3000).await;
        let ldr_value = uferris.read_ldr().await;
        let _ = writeln!(w, "Third LDR Reading: {}", ldr_value);
        let _ = writeln!(w, "LDR Reading Test Complete.");

        // ---------------------------------------
        // Button Press Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().await.unwrap();
        let _ = writeln!(
            w,
            "Commencing Button Press Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        let _ = writeln!(w, "Button Press Testing");
        let _ = writeln!(w, "Press Button 1 to Proceed...");
        while !uferris.read_sw1().await.unwrap() {}
        let _ = writeln!(w, "Button 1 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 2 to Proceed...");
        while !uferris.read_sw2().await.unwrap() {}
        let _ = writeln!(w, "Button 2 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 3 to Proceed...");
        while !uferris.read_sw3().await.unwrap() {}
        let _ = writeln!(w, "Button 3 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 4 to Proceed...");
        while !uferris.read_sw4().await.unwrap() {}
        let _ = writeln!(w, "Button 4 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 5 to Proceed...");
        // The one phase that does not have to poll: button 5 is wired straight
        // to the controller, so the board can suspend on the pin going low
        // instead of asking it over and over.
        uferris.wait_for_sw5().await;
        let _ = writeln!(w, "Button 5 Pressed! Continuing...");

        // ---------------------------------------
        // Slide Switch Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().await.unwrap();
        let _ = writeln!(
            w,
            "Commencing Slide Switch Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        let _ = writeln!(w, "Slide Switch Testing");
        let _ = writeln!(w, "Starting with Switch 6 (Left Switch)...");
        let current_sw6_state = uferris.read_sw6().await.unwrap();
        let mut new_sw6_state = uferris.read_sw6().await.unwrap();
        let _ = writeln!(w, "Current Slide Switch 6 State: {:?}", current_sw6_state);
        let _ = writeln!(w, "Toggle Slide Switch 6 to Proceed...");
        while current_sw6_state == new_sw6_state {
            new_sw6_state = uferris.read_sw6().await.unwrap();
        }
        let _ = writeln!(w, "Slide Switch 6 State Changed");
        let _ = writeln!(w, "Current Slide Switch 6 State: {:?}", new_sw6_state);

        delay.delay_ms(2000).await;

        let _ = writeln!(w, "Moving to Switch 7 (Right Switch)...");
        let current_sw7_state = uferris.read_sw7().await.unwrap();
        let mut new_sw7_state = uferris.read_sw7().await.unwrap();
        let _ = writeln!(w, "Current Slide Switch 7 State: {:?}", current_sw7_state);
        let _ = writeln!(w, "Toggle Slide Switch 7 to Proceed...");
        while current_sw7_state == new_sw7_state {
            new_sw7_state = uferris.read_sw7().await.unwrap();
        }
        let _ = writeln!(w, "Slide Switch 7 State Changed");
        let _ = writeln!(w, "Current Slide Switch 7 State: {:?}", new_sw7_state);
        delay.delay_ms(2000).await;

        let _ = writeln!(w, "Testing Loop Complete. Restarting...");
    }
}
