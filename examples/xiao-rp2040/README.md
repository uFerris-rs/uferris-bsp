# Xiao RP2040 Examples

| Example | Description |
| ------- | ----------- |
| `blinky` | Blink LED 1 on the baseboard |
| `baseboard_demo` | Tour of the baseboard peripherals (LEDs, buttons, buzzer, 7-segment, LDR, RTC) |
| `full_board_demo` | Tour of the baseboard & powerboard peripherals |

Run with `cargo run --bin <example>` from this directory.

The Xiao RP2040 carries an RP2040, and the BSP drives it through `embassy-rp`
in blocking mode: no executor, no `embassy-time`, no `async`.

## No serial console yet

**These examples produce no serial console output.** The Xiao RP2040 has no
USB-to-UART bridge, so printing would mean driving a USB CDC device the chip
provides itself, and the `embassy-rp` USB driver is `async` only. A console
arrives with `async` support.

Until then the demos are silent but complete: every phase still runs in the
same order and with the same delays, the sensors are still read and the SD card
is still written and read back, the results are just discarded. Nothing waits
for a host either, so a demo starts as soon as the board is flashed. The button
and slide switch phases still block until the matching control is operated, in
the order buttons 1 through 5, then slide switch 6, then slide switch 7.

If you have a debug probe wired to the SWD pads on the underside of the board,
printing is available today over RTT with `probe-rs` and an RTT logger such as
`defmt-rtt` — that is a change to your own application, not something these
examples set up.

## Flashing

Flashing goes through the RP2040 UF2 bootloader:

1. Install the runner once with `cargo install elf2uf2-rs`.
2. Put the board in bootloader mode: hold **BOOT** while pressing **RESET**, or
   hold **BOOT** while plugging the board in. It appears as a `RPI-RP2` drive.
3. Run `cargo run --bin <example>`. This converts the ELF to a UF2, copies it to
   the board, and starts it.

There is no monitor step: with nothing being printed there is nothing to attach
to.

The second stage bootloader the RP2040 needs is supplied by `embassy-rp`, which
places `BOOT_LOADER_W25Q080` — the correct one for the Winbond W25Q class flash
the Xiao RP2040 carries — into the `BOOT2` region `memory.x` reserves. The
`.boot2` output section itself comes from `link-rp.x`, a linker script fragment
`embassy-rp` emits for the RP2040 and that `.cargo/config.toml` adds to the link
alongside `link.x`.
