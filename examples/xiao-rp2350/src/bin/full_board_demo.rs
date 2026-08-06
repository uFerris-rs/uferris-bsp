#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with embassy_rp types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embedded_hal::delay::DelayNs;
use panic_halt as _;
use uferris_bsp::boards::xiao_rp2350::delay;
use uferris_bsp::{SevenSegDigit, uferris_init};

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[cortex_m_rt::entry]
fn main() -> ! {
    // `embassy-rp` is driven in blocking mode here: `init` only sets up the
    // clock tree and hands back the peripheral singletons, so a plain
    // `cortex-m-rt` entry point is all this needs.
    let peripherals = embassy_rp::init(Default::default());

    let mut uferris = uferris_init(peripherals);

    let mut delay = delay();

    // A Constant to Control the Speed of the test/Demo Loop
    const TEST_DELAY_MS: u32 = 200;

    // Set RTC to Known Time for Testing
    uferris.set_rtc_time(2026, 1, 29, 9, 16, 0).unwrap();

    // There is no console on this board yet, so nothing below prints. Every
    // phase still runs, and the reads that only fed a print statement are kept
    // so the peripherals are still exercised. See the README.
    loop {
        // ---------------------------------------
        // LED Tests
        // ---------------------------------------

        let _ = uferris.read_rtc_time();

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

        let _ = uferris.read_rtc_time();

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

        let _ = uferris.rtc.read_time();

        delay.delay_ms(TEST_DELAY_MS);

        uferris.buzz_on(1024);

        delay.delay_ms(2000);

        uferris.buzz_off();

        // ---------------------------------------
        // LDR Testing
        // ---------------------------------------

        let _ = uferris.read_rtc_time();

        // Three readings with a 3s delay between them.
        let _ = uferris.read_ldr();
        delay.delay_ms(3000);
        let _ = uferris.read_ldr();
        delay.delay_ms(3000);
        let _ = uferris.read_ldr();

        // ---------------------------------------
        // Button Press Tests
        // ---------------------------------------

        let _ = uferris.read_rtc_time();

        // Each loop waits for its button, in order: 1 through 5.
        while !uferris.read_sw1().unwrap() {}
        while !uferris.read_sw2().unwrap() {}
        while !uferris.read_sw3().unwrap() {}
        while !uferris.read_sw4().unwrap() {}
        while !uferris.read_sw5() {}

        // ---------------------------------------
        // Slide Switch Tests
        // ---------------------------------------

        let _ = uferris.read_rtc_time();

        // Switch 6 (Left Switch): wait for a toggle.
        let current_sw6_state = uferris.read_sw6().unwrap();
        let mut new_sw6_state = uferris.read_sw6().unwrap();
        while current_sw6_state == new_sw6_state {
            new_sw6_state = uferris.read_sw6().unwrap();
        }

        delay.delay_ms(2000);

        // Switch 7 (Right Switch): wait for a toggle.
        let current_sw7_state = uferris.read_sw7().unwrap();
        let mut new_sw7_state = uferris.read_sw7().unwrap();
        while current_sw7_state == new_sw7_state {
            new_sw7_state = uferris.read_sw7().unwrap();
        }

        delay.delay_ms(2000);

        // ---------------------------------------
        // Powerboard Testing
        // Reading INA219 Voltage, Current, Power
        // ---------------------------------------

        let _ = uferris.read_rtc_time();

        let _ = uferris.read_system_voltage();
        let _ = uferris.read_system_current();
        let _ = uferris.read_system_power();

        // ---------------------------------------
        // Powerboard Testing
        // Reading from and Writing to SD Card
        // ---------------------------------------

        let _ = uferris.read_rtc_time();

        if uferris.init_sd_card().is_ok() {
            uferris.write_to_file_in_root("TEST.TXT", "hello".as_bytes());

            let _ = uferris.read_file_chunked("TEST.TXT", |_chunk| {});
        }
    }
}
