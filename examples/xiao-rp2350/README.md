# Xiao RP2350 Examples

| Example | Description |
| ------- | ----------- |
| `blinky` | Blink LED 1 on the baseboard |
| `baseboard_demo` | Tour of the baseboard peripherals (LEDs, buttons, buzzer, 7-segment, LDR, RTC) |
| `full_board_demo` | Tour of the baseboard & powerboard peripherals |

Run with `cargo run --bin <example>` from this directory.

The Xiao RP2350 carries an RP2350A, the 30 GPIO package, and the BSP drives it
through `embassy-rp` in blocking mode: no executor, no `embassy-time`, no
`async`.

## No serial console yet

**These examples produce no serial console output.** The Xiao RP2350 has no
USB-to-UART bridge, so printing would mean driving a USB CDC device the chip
provides itself, and the `embassy-rp` USB driver is `async` only. A console
arrives with `async` support.

Until then the demos are silent but complete: every phase still runs in the
same order and with the same delays, the sensors are still read and the SD card
is still written and read back, the results are just discarded. Nothing waits
for a host either, so a demo starts as soon as the board is flashed. The button
and slide switch phases still block until the matching control is operated, in
the order buttons 1 through 5, then slide switch 6, then slide switch 7.

## Flashing

`elf2uf2-rs` does not support the RP2350, so flashing goes through `picotool`:

1. Install the runner once with `cargo install picotool`, or on macOS with
   `brew install picotool`.
2. Put the board in bootloader mode: hold **BOOT** while pressing **RESET**, or
   hold **BOOT** while plugging the board in. It appears as a `RP2350` drive.
3. Run `cargo run --bin <example>`. This loads the ELF onto the board, verifies
   it, and starts it.

There is no monitor step: with nothing being printed there is nothing to attach
to.
