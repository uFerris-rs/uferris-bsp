# Xiao nRF52840 Examples

| Example | Description |
| ------- | ----------- |
| `blinky` | Blink LED 1 on the baseboard |
| `baseboard_demo` | Tour of the baseboard peripherals (LEDs, buttons, buzzer, 7-segment, LDR, RTC) |
| `full_board_demo` | Tour of the baseboard & powerboard peripherals |
| `blinky_async` | Blink LED 1, on the `async` board API |
| `baseboard_demo_async` | The baseboard tour, on the `async` board API |

Run with `cargo run --bin <example>` from this directory. The two `_async`
examples need their feature: `cargo run --bin blinky_async --features async`.

These examples cover both the Xiao nRF52840 and the Xiao nRF52840 Sense: the
two carry the same chip, and the extra sensors on the Sense are not part of the
carrier board API.

The blocking examples drive the nRF52840 through `embassy-nrf` in blocking mode:
no executor, no `embassy-time`, no `async`. The `_async` ones stand up a runtime
— see below.

## No serial console yet

**These examples produce no serial console output**, the `_async` ones
included. The Xiao nRF52840 has no USB-to-UART bridge, so printing would mean
driving a USB CDC device the chip provides itself, and the `embassy-nrf` USB
driver is `async` only. That driver is within reach now that the board has an
`async` API, but nothing here drives it yet: a USB CDC console is a follow-up.

Until then the demos are silent but complete: every phase still runs in the
same order and with the same delays, the sensors are still read and the SD card
is still written and read back, the results are just discarded. Nothing waits
for a host either, so a demo starts as soon as the board is flashed. The button
and slide switch phases still block until the matching control is operated, in
the order buttons 1 through 5, then slide switch 6, then slide switch 7.

## The `async` examples

`blinky_async` and `baseboard_demo_async` run the same demos against the BSP's
`async` board API. Build them with the example crate's `async` feature:

```
cargo run --bin blinky_async --features async
cargo run --bin baseboard_demo_async --features async
```

The board comes up through `uferris_init_async` instead of `uferris_init`. It
wires the same pins to the same peripherals, but builds the drivers so that the
I2C and ADC operations are futures: the TWIM is shared through an
`embassy-sync` mutex rather than a `RefCell`, the SAADC is awaited rather than
polled, and button 5 becomes something the board can wait on. The two init
functions are mutually exclusive — each consumes the peripheral singletons — so
a program calls one or the other.

**The runtime lives here, in the example, not in the BSP.** The BSP's `async`
feature is a pure code gate: it starts no executor and installs no time driver.
These examples bring:

- `embassy-executor`, with `platform-cortex-m` and `executor-thread`. The
  `#[embassy_executor::main]` attribute replaces `#[cortex_m_rt::entry]` — it
  expands to a `cortex-m-rt` entry point that stands up a thread-mode executor
  and spawns `main` on it. `main` still returns `!`, and the demo bodies still
  loop forever, so nothing ever falls off the end of it.
- `embassy-time`, for `embassy_time::Delay`. This is what the demos' delays run
  on, in place of the blocking board's cycle counting `delay()`. It needs a
  driver, which the example's `async` feature turns on in `embassy-nrf` as
  `time-driver-rtc1` — RTC1 is the low-power counter `embassy-nrf` uses for time
  on the nRF52 family, and it is free because nothing else here claims it.

The `async` feature also switches on `embassy-nrf`'s `gpiote`, by way of the
BSP: `embedded_hal_async::digital::Wait` is only implemented for `Input` when
that feature is on, and waiting on the button pin is what makes
`wait_for_sw5()` work. `embassy_nrf::init` initializes the GPIOTE interrupt and
`embassy-nrf` supplies its handler, so there is nothing extra to bind here.

**These demos are still silent**, exactly like the blocking ones — this board
has no console either way. The observable difference is at the button phases of
`baseboard_demo_async`: pressing button 5 advances the demo without the core
having spun on the pin waiting for it, and the LED and 7-segment phases that
follow run off timer interrupts rather than cycle counting.

The power board is not part of the `async` board yet, so there is no
`full_board_demo_async`.

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

The `_async` examples flash the same way. A binary's `required-features` only
decides whether Cargo builds it at all; the runner sees an ELF either way, so
`cargo run --bin blinky_async --features async` goes through `uf2deploy`
unchanged.

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
