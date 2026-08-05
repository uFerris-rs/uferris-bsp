# Xiao RP2040 Examples

| Example | Description |
| ------- | ----------- |
| `blinky` | Blink LED 1 on the baseboard |
| `baseboard_demo` | Tour of the baseboard peripherals (LEDs, buttons, buzzer, 7-segment, LDR, RTC) |
| `full_board_demo` | Tour of the baseboard & powerboard peripherals |

Run with `cargo run --bin <example>` from this directory.

## Flashing & monitor

The Xiao RP2040 has no USB-to-UART bridge, so the examples print over a USB CDC
serial device that the board itself provides. Each example waits for a terminal
to open that port before it starts, so nothing is missed.

Flashing goes through the RP2040 UF2 bootloader:

1. Install the runner once with `cargo install elf2uf2-rs`.
2. Put the board in bootloader mode: hold **BOOT** while pressing **RESET**, or
   hold **BOOT** while plugging the board in. It appears as a `RPI-RP2` drive.
3. Run `cargo run --bin <example>`. This converts the ELF to a UF2, copies it to
   the board, and then opens the serial monitor on the CDC device.
