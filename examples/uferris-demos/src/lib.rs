#![no_std]

//! # Shared uFerris Example Demos
//!
//! The demo bodies the per-board example crates run. Every supported Xiao runs
//! the exact same sequence of board operations; only the parts that cannot be
//! written once — bringing up the controller, acquiring a delay provider and
//! opening a console — differ. Those stay in the per-board `src/bin/*.rs`
//! shims, and everything after them lives here.
//!
//! This crate depends on the generic layer of `uferris-bsp` only, with no
//! device feature selected, so it is written entirely against the
//! [`Uferris`] board API and the `embedded-hal` traits.
//!
//! ## Printing
//!
//! Each demo takes a [`core::fmt::Write`] sink instead of printing through a
//! macro, because every board reaches its console differently: the ESP32 boards
//! print through `esp-println`, the Xiao nRF54L15 prints over RTT, and the
//! boards with no console at all pass [`NoopWriter`], which throws the bytes
//! away. Write errors are always discarded — a demo must not stop because a
//! console is not attached.

use core::fmt::Write;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;
use embedded_hal::pwm::SetDutyCycle;
use uferris_bsp::components::ldr::OneShot;
use uferris_bsp::{PowerConstraints, SevenSegDigit, Uferris};

/// A [`core::fmt::Write`] sink that discards everything written to it.
///
/// The boards without a console yet (the Xiao RP2040, RP2350 and nRF52840) pass
/// this, so they run the same demo bodies as everything else: every phase still
/// executes and every peripheral is still exercised, the text just has nowhere
/// to go.
pub struct NoopWriter;

impl Write for NoopWriter {
    fn write_str(&mut self, _s: &str) -> core::fmt::Result {
        Ok(())
    }
}

/// Blink LED 1 forever.
///
/// This demo prints nothing, so it takes no writer.
pub fn blinky<LED, BTN, BUZZ, I2C, ADC, BD>(
    uferris: &mut Uferris<LED, BTN, BUZZ, I2C, ADC, BD>,
    delay: &mut impl DelayNs,
) -> !
where
    LED: OutputPin,
    BTN: InputPin,
    BUZZ: SetDutyCycle,
    I2C: I2c,
    ADC: OneShot,
    BD: PowerConstraints,
{
    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    loop {
        uferris.led1_on();
        delay.delay_ms(TEST_DELAY_MS);

        uferris.led1_off();
        delay.delay_ms(TEST_DELAY_MS);
    }
}

/// Exercise every component on the uFerris base board, forever.
///
/// The phases run in order: LEDs, seven segment display, buzzer, LDR, the five
/// push buttons and the two slide switches. The button and slide switch phases
/// block until the board is touched.
pub fn baseboard_demo<LED, BTN, BUZZ, I2C, ADC, BD>(
    uferris: &mut Uferris<LED, BTN, BUZZ, I2C, ADC, BD>,
    delay: &mut impl DelayNs,
    w: &mut impl Write,
) -> !
where
    LED: OutputPin,
    BTN: InputPin,
    BUZZ: SetDutyCycle,
    I2C: I2c,
    ADC: OneShot,
    BD: PowerConstraints,
{
    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    // Set RTC to Known Time for Testing
    uferris.set_rtc_time(2026, 1, 29, 9, 16, 0).unwrap();

    loop {
        // ---------------------------------------
        // LED Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        let _ = writeln!(
            w,
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
        let _ = writeln!(
            w,
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
        let _ = writeln!(
            w,
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
        let _ = writeln!(
            w,
            "Commencing LDR Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );
        let _ = writeln!(w, "Taking Three Readings with 3s Delay Between Readings");
        let ldr_value = uferris.read_ldr();
        let _ = writeln!(w, "First LDR Reading: {}", ldr_value);
        let _ = writeln!(w, "Waiting 3 Seconds...");
        delay.delay_ms(3000);
        let ldr_value = uferris.read_ldr();
        let _ = writeln!(w, "Second LDR Reading: {}", ldr_value);
        let _ = writeln!(w, "Waiting 3 Seconds...");
        delay.delay_ms(3000);
        let ldr_value = uferris.read_ldr();
        let _ = writeln!(w, "Third LDR Reading: {}", ldr_value);
        let _ = writeln!(w, "LDR Reading Test Complete.");

        // ---------------------------------------
        // Button Press Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        let _ = writeln!(
            w,
            "Commencing Button Press Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        let _ = writeln!(w, "Button Press Testing");
        let _ = writeln!(w, "Press Button 1 to Proceed...");
        while !uferris.read_sw1().unwrap() {}
        let _ = writeln!(w, "Button 1 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 2 to Proceed...");
        while !uferris.read_sw2().unwrap() {}
        let _ = writeln!(w, "Button 2 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 3 to Proceed...");
        while !uferris.read_sw3().unwrap() {}
        let _ = writeln!(w, "Button 3 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 4 to Proceed...");
        while !uferris.read_sw4().unwrap() {}
        let _ = writeln!(w, "Button 4 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 5 to Proceed...");
        while !uferris.read_sw5() {}
        let _ = writeln!(w, "Button 5 Pressed! Continuing...");

        // ---------------------------------------
        // Slide Switch Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        let _ = writeln!(
            w,
            "Commencing Slide Switch Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        let _ = writeln!(w, "Slide Switch Testing");
        let _ = writeln!(w, "Starting with Switch 6 (Left Switch)...");
        let current_sw6_state = uferris.read_sw6().unwrap();
        let mut new_sw6_state = uferris.read_sw6().unwrap();
        let _ = writeln!(w, "Current Slide Switch 6 State: {:?}", current_sw6_state);
        let _ = writeln!(w, "Toggle Slide Switch 6 to Proceed...");
        while current_sw6_state == new_sw6_state {
            new_sw6_state = uferris.read_sw6().unwrap();
        }
        let _ = writeln!(w, "Slide Switch 6 State Changed");
        let _ = writeln!(w, "Current Slide Switch 6 State: {:?}", new_sw6_state);

        delay.delay_ms(2000);

        let _ = writeln!(w, "Moving to Switch 7 (Right Switch)...");
        let current_sw7_state = uferris.read_sw7().unwrap();
        let mut new_sw7_state = uferris.read_sw7().unwrap();
        let _ = writeln!(w, "Current Slide Switch 7 State: {:?}", current_sw7_state);
        let _ = writeln!(w, "Toggle Slide Switch 7 to Proceed...");
        while current_sw7_state == new_sw7_state {
            new_sw7_state = uferris.read_sw7().unwrap();
        }
        let _ = writeln!(w, "Slide Switch 7 State Changed");
        let _ = writeln!(w, "Current Slide Switch 7 State: {:?}", new_sw7_state);
        delay.delay_ms(2000);

        let _ = writeln!(w, "Testing Loop Complete. Restarting...");
    }
}

/// Exercise the uFerris base board and the Megalops power board, forever.
///
/// This is [`baseboard_demo`] with two more phases appended: the INA219 power
/// measurements and an SD card write followed by a read back.
#[cfg(feature = "power-board")]
pub fn full_board_demo<LED, BTN, BUZZ, I2C, ADC, BD>(
    uferris: &mut Uferris<LED, BTN, BUZZ, I2C, ADC, BD>,
    delay: &mut impl DelayNs,
    w: &mut impl Write,
) -> !
where
    LED: OutputPin,
    BTN: InputPin,
    BUZZ: SetDutyCycle,
    I2C: I2c,
    ADC: OneShot,
    BD: PowerConstraints,
{
    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    // Set RTC to Known Time for Testing
    uferris.set_rtc_time(2026, 1, 29, 9, 16, 0).unwrap();

    loop {
        // ---------------------------------------
        // LED Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        let _ = writeln!(
            w,
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
        let _ = writeln!(
            w,
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
        let _ = writeln!(
            w,
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
        let _ = writeln!(
            w,
            "Commencing LDR Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );
        let _ = writeln!(w, "Taking Three Readings with 3s Delay Between Readings");
        let ldr_value = uferris.read_ldr();
        let _ = writeln!(w, "First LDR Reading: {}", ldr_value);
        let _ = writeln!(w, "Waiting 3 Seconds...");
        delay.delay_ms(3000);
        let ldr_value = uferris.read_ldr();
        let _ = writeln!(w, "Second LDR Reading: {}", ldr_value);
        let _ = writeln!(w, "Waiting 3 Seconds...");
        delay.delay_ms(3000);
        let ldr_value = uferris.read_ldr();
        let _ = writeln!(w, "Third LDR Reading: {}", ldr_value);
        let _ = writeln!(w, "LDR Reading Test Complete.");

        // ---------------------------------------
        // Button Press Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        let _ = writeln!(
            w,
            "Commencing Button Press Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        let _ = writeln!(w, "Button Press Testing");
        let _ = writeln!(w, "Press Button 1 to Proceed...");
        while !uferris.read_sw1().unwrap() {}
        let _ = writeln!(w, "Button 1 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 2 to Proceed...");
        while !uferris.read_sw2().unwrap() {}
        let _ = writeln!(w, "Button 2 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 3 to Proceed...");
        while !uferris.read_sw3().unwrap() {}
        let _ = writeln!(w, "Button 3 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 4 to Proceed...");
        while !uferris.read_sw4().unwrap() {}
        let _ = writeln!(w, "Button 4 Pressed! Continuing...");
        let _ = writeln!(w, "Press Button 5 to Proceed...");
        while !uferris.read_sw5() {}
        let _ = writeln!(w, "Button 5 Pressed! Continuing...");

        // ---------------------------------------
        // Slide Switch Tests
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        let _ = writeln!(
            w,
            "Commencing Slide Switch Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        let _ = writeln!(w, "Slide Switch Testing");
        let _ = writeln!(w, "Starting with Switch 6 (Left Switch)...");
        let current_sw6_state = uferris.read_sw6().unwrap();
        let mut new_sw6_state = uferris.read_sw6().unwrap();
        let _ = writeln!(w, "Current Slide Switch 6 State: {:?}", current_sw6_state);
        let _ = writeln!(w, "Toggle Slide Switch 6 to Proceed...");
        while current_sw6_state == new_sw6_state {
            new_sw6_state = uferris.read_sw6().unwrap();
        }
        let _ = writeln!(w, "Slide Switch 6 State Changed");
        let _ = writeln!(w, "Current Slide Switch 6 State: {:?}", new_sw6_state);

        delay.delay_ms(2000);

        let _ = writeln!(w, "Moving to Switch 7 (Right Switch)...");
        let current_sw7_state = uferris.read_sw7().unwrap();
        let mut new_sw7_state = uferris.read_sw7().unwrap();
        let _ = writeln!(w, "Current Slide Switch 7 State: {:?}", current_sw7_state);
        let _ = writeln!(w, "Toggle Slide Switch 7 to Proceed...");
        while current_sw7_state == new_sw7_state {
            new_sw7_state = uferris.read_sw7().unwrap();
        }
        let _ = writeln!(w, "Slide Switch 7 State Changed");
        let _ = writeln!(w, "Current Slide Switch 7 State: {:?}", new_sw7_state);
        delay.delay_ms(2000);

        // ---------------------------------------
        // Powerboard Testing
        // Reading INA219 Voltage, Current, Power
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        let _ = writeln!(
            w,
            "Commencing Power Measurement Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        let voltage = uferris.read_system_voltage();
        let current = uferris.read_system_current();
        let power = uferris.read_system_power();
        let _ = writeln!(w, "Battery Voltage: {:?} mV", voltage);
        let _ = writeln!(w, "Battery Current: {:?} uA", current);
        let _ = writeln!(w, "Battery Power: {:?} uW", power);

        // ---------------------------------------
        // Powerboard Testing
        // Reading from and Writing to SD Card
        // ---------------------------------------

        let datetime = uferris.read_rtc_time().unwrap();
        let _ = writeln!(
            w,
            "Commencing SD Card Testing at {:02}:{:02}:{:02} on {}/{}/{}",
            datetime.3, datetime.4, datetime.5, datetime.2, datetime.1, datetime.0
        );

        let sd_card = uferris.init_sd_card();

        match sd_card {
            Ok(vol_mgr) => {
                let _ = writeln!(w, "SD Card Initialized Successfully.");
                let _ = writeln!(w, "SD Card Size: {} Bytes", vol_mgr);

                uferris.write_to_file_in_root("TEST.TXT", "hello".as_bytes());
                let _ = writeln!(w, "Wrote 'hello' to 'TEXT.TXT' file in root directory.");

                let _ = writeln!(w, "Reading 'TEST.TXT' file from root directory:");
                let result = uferris.read_file_chunked("TEST.TXT", |chunk| {
                    let _ = writeln!(
                        w,
                        "Chunk: {:?}",
                        core::str::from_utf8(chunk).unwrap_or("??")
                    );
                });

                if result.is_err() {
                    let _ = writeln!(w, "Read error occurred!");
                }
            }
            Err(e) => {
                let _ = writeln!(w, "Failed to initialize SD Card: {:?}", e);
            }
        }

        let _ = writeln!(w, "Testing Loop Complete. Restarting...");
    }
}
