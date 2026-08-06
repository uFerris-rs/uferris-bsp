# Xiao nRF54L15 Examples

| Example | Description |
| ------- | ----------- |
| `blinky` | Blink LED 1 on the baseboard |
| `baseboard_demo` | Tour of the baseboard peripherals (LEDs, buttons, buzzer, 7-segment, LDR, RTC) |
| `full_board_demo` | Tour of the baseboard & powerboard peripherals |
| `blinky_async` | Blink LED 1, on the `async` board API |
| `baseboard_demo_async` | The baseboard tour, on the `async` board API |

Run with `cargo run --bin <example>` from this directory. The two `_async`
examples need their feature: `cargo run --bin blinky_async --features async`.

These examples cover both the Xiao nRF54L15 and the Xiao nRF54L15 Sense: the
two carry the same chip, and the extra sensors on the Sense are not part of the
carrier board API.

The blocking examples drive the nRF54L15 through `embassy-nrf` in blocking mode:
no executor, no `embassy-time`, no `async`. The `_async` ones are the first
`async` uFerris examples there are — see below.

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
changed here when `async` support landed.

The examples do not use `defmt`. `rprintln!` formats on the target and sends
plain text, which costs a little flash and a little time but keeps the BSP and
the examples free of a second logging stack. A `defmt` feature may be added
later for people who want the smaller, faster encoding.

## The `async` examples

`blinky_async` and `baseboard_demo_async` run the same demos against the BSP's
`async` board API, which the Xiao nRF54L15 is the first board to support. Build
them with the example crate's `async` feature:

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
  `time-driver-grtc` — the GRTC is the nRF54L's always-on global timer.

The `async` feature also switches on `embassy-nrf`'s `gpiote`, by way of the
BSP: `embedded_hal_async::digital::Wait` is only implemented for `Input` when
that feature is on, and waiting on the button pin is what makes
`wait_for_sw5()` work. `embassy_nrf::init` initializes the GPIOTE interrupt and
`embassy-nrf` supplies its handler, so there is nothing extra to bind here.

The power board is not part of the `async` board yet, so there is no
`full_board_demo_async`.

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
