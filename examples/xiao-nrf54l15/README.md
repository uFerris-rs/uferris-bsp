# Xiao nRF54L15 Examples

| Example | Description |
| ------- | ----------- |
| `blinky` | Blink LED 1 on the baseboard |
| `baseboard_demo` | Tour of the baseboard peripherals (LEDs, buttons, buzzer, 7-segment, LDR, RTC) |
| `full_board_demo` | Tour of the baseboard & powerboard peripherals |

Run with `cargo run --bin <example>` from this directory.

These examples cover both the Xiao nRF54L15 and the Xiao nRF54L15 Sense: the
two carry the same chip, and the extra sensors on the Sense are not part of the
carrier board API.

The BSP drives the nRF54L15 through `embassy-nrf` in blocking mode: no executor,
no `embassy-time`, no `async`.

## Flashing and the console

The Xiao nRF54L15 carries its own debugger: the USB-C connector goes to an
onboard SAMD11 running CMSIS-DAP, which is wired to the nRF54L15's SWD lines.
One plain USB-C cable is therefore all this board needs — no bootloader mode,
no double-tap, no external probe, and no UF2 volume.

1. Install the runner once with `cargo install probe-rs-tools`.
2. Plug the board in and run `cargo run --bin <example>`.

`probe-rs run` writes the image into RRAM, resets the chip, and then stays
attached, printing the example's output in the same terminal until you stop it
with Ctrl-C. On a board that has never been programmed the first run also has
to lift the factory access-port protection, which `probe-rs` does on its own.

That printed output arrives over **RTT**: the examples write to it with
`rtt-target`'s `rprintln!`, and `probe-rs` reads the buffer out of RAM over the
debug connection. Panics go the same way, through `panic-rtt-target`, so a
failed `unwrap()` prints its message and location instead of silently hanging.

RTT is not a fallback on this board, it is the console. The nRF54L15 has **no
USB peripheral at all**, so there is no USB CDC device for it to present and no
USB-to-UART bridge on the Xiao either. Unlike the other µFerris boards, nothing
changes here when `async` support lands.

The examples do not use `defmt`. `rprintln!` formats on the target and sends
plain text, which costs a little flash and a little time but keeps the BSP and
the examples free of a second logging stack. A `defmt` feature may be added
later for people who want the smaller, faster encoding.

## Memory layout

`memory.x` links the application at address 0 with 1524 KB of RRAM and the full
256 KB of RAM. RRAM replaces flash on this part: it is byte writable and needs
no page erase, but it is mapped and executed just like flash, so `cortex-m-rt`
treats it no differently.

The top 12 KB of the 1536 KB are left out of the application region. That is
where an FLPR coprocessor image is conventionally placed — it is the split
Zephyr uses between its `cpuapp` and `cpuflpr` partitions — and reserving it now
keeps a later FLPR image from having to move the application. Nothing here loads
one, and `embassy_nrf::init` stops and resets the FLPR on every boot so a
program left over from an earlier session cannot interfere.

The examples are built for the **Secure** state (`nrf54l15-app-s`). A bare image
flashed straight onto the chip boots Secure, so that is the matching target;
the non-secure variant is for images that run behind a secure bootloader.
