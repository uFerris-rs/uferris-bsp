# Xiao ESP32-C5 Examples

| Example | Description |
| ------- | ----------- |
| `blinky` | Blink LED 1 on the baseboard |
| `baseboard_demo` | Tour of the baseboard peripherals (LEDs, buttons, buzzer, 7-segment, LDR, RTC) |
| `full_board_demo` | Tour of the baseboard & powerboard peripherals |

Run with `cargo run --bin <example>` from this directory.

The buzzer phase of the demos runs on this board but stays silent: `esp-hal` has
no PWM (LEDC/MCPWM) driver for the ESP32-C5 yet, so `buzz_on`/`buzz_off` are
accepted and do nothing. See the note in the [crate README](../../README.md).