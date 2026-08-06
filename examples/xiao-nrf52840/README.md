# Xiao nRF52840 Examples

| Example | Description |
| ------- | ----------- |
| `blinky` | Blink LED 1 on the baseboard |
| `baseboard_demo` | Tour of the baseboard peripherals (LEDs, buttons, buzzer, 7-segment, LDR, RTC) |
| `full_board_demo` | Tour of the baseboard & powerboard peripherals |

Run with `cargo run --bin <example>` from this directory.

These examples cover both the Xiao nRF52840 and the Xiao nRF52840 Sense: the
two carry the same chip, and the extra sensors on the Sense are not part of the
carrier board API.

The BSP drives the nRF52840 through `embassy-nrf` in blocking mode: no executor,
no `embassy-time`, no `async`.

## No serial console yet

**These examples produce no serial console output.** The Xiao nRF52840 has no
USB-to-UART bridge, so printing would mean driving a USB CDC device the chip
provides itself, and the `embassy-nrf` USB driver is `async` only. A console
arrives with `async` support.

Until then the demos are silent but complete: every phase still runs in the
same order and with the same delays, the sensors are still read and the SD card
is still written and read back, the results are just discarded. Nothing waits
for a host either, so a demo starts as soon as the board is flashed. The button
and slide switch phases still block until the matching control is operated, in
the order buttons 1 through 5, then slide switch 6, then slide switch 7.

## Flashing

A stock Xiao nRF52840 ships with the Adafruit nRF52 UF2 bootloader and a
resident Nordic SoftDevice, so flashing is done by handing the bootloader a UF2
file. `memory.x` links the application at `0x27000`, the first address above
the SoftDevice, which is where the bootloader expects it.

1. Install the runner once with `cargo install uf2deploy`.
2. Put the board in bootloader mode: **double-tap the RESET button**. A drive
   appears — the name varies with the shipped bootloader, so expect anything
   from `XIAO-BOOT` to `XIAO-SENSE`. Check the mounted volume and open its
   `INFO_UF2.TXT`: it reports the bootloader and SoftDevice versions. These
   examples are linked for **SoftDevice S140 v7** (application base `0x27000`).
   If your board reports a different SoftDevice, adjust `FLASH : ORIGIN` in
   `memory.x` to match.
3. Run `cargo run --bin <example>`. This converts the ELF into a UF2 with family
   ID `0xADA52840`, finds whichever mounted volume carries an `INFO_UF2.TXT`,
   and copies the file across. The board reboots into the application on its
   own.

There is no monitor step: with nothing being printed there is nothing to attach
to.

### Flashing by hand

If you would rather not install the runner, drop the `runner` line from
`.cargo/config.toml` and do the same three steps yourself. `cargo objcopy` needs
`cargo install cargo-binutils` plus `rustup component add llvm-tools`, and
`uf2conv.py` comes from the [microsoft/uf2](https://github.com/microsoft/uf2)
utilities (`uf2conv.py` and `uf2families.json` from its `utils/` directory,
kept side by side):

```bash
cargo objcopy --release --bin blinky -- -O ihex blinky.hex
uf2conv.py blinky.hex --family 0xADA52840 --output blinky.uf2
cp blinky.uf2 /Volumes/<the mounted bootloader drive>/
```

The Intel HEX carries its own addresses, so no `--base` is needed; the resulting
UF2 targets `0x27000`. On Linux the drive is usually under `/media/<user>/`.

### A warning about SWD

**Do not flash these examples over SWD**, with `probe-rs` or anything else,
unless you know exactly what you are doing and are prepared to restore the
board. Two things go wrong:

- A chip erase wipes the bootloader and the SoftDevice along with everything
  else, and recovering from that needs a debug probe and a fresh bootloader
  image.
- Adafruit nRF52 bootloader 0.6.1 checks the application region before jumping
  into it and can refuse to start an image written over SWD, leaving a board
  that looks bricked but is only sulking. Double-tapping RESET gets the
  bootloader back, and a UF2 write fixes it.

Linking at `0x0` instead of `0x27000` destroys the SoftDevice for the same
reason. The UF2 path avoids all of this, which is why it is the only one
documented here.
