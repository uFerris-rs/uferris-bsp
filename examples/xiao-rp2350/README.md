# Xiao RP2350 Examples

| Example | Description |
| ------- | ----------- |
| `blinky` | Blink LED 1 on the baseboard |
| `baseboard_demo` | Tour of the baseboard peripherals (LEDs, buttons, buzzer, 7-segment, LDR, RTC) |
| `full_board_demo` | Tour of the baseboard & powerboard peripherals |
| `blinky_async` | Blink LED 1, on the `async` board API |
| `baseboard_demo_async` | The baseboard tour, on the `async` board API |

Run with `cargo run --bin <example>` from this directory. The two `_async`
examples need their feature: `cargo run --bin blinky_async --features async`.

The Xiao RP2350 carries an RP2350A, the 30 GPIO package. The blocking examples
drive it through `embassy-rp` in blocking mode: no executor, no `embassy-time`,
no `async`. The `_async` ones stand up a runtime — see below.

## No serial console yet

**These examples produce no serial console output**, the `_async` ones
included. The Xiao RP2350 has no USB-to-UART bridge, so printing would mean
driving a USB CDC device the chip provides itself, and the `embassy-rp` USB
driver is `async` only. That driver is within reach now that the board has an
`async` API, but nothing here drives it yet: a USB CDC console is a follow-up.

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

## The `async` examples

`blinky_async` and `baseboard_demo_async` run the same demos against the BSP's
`async` board API. Build them with the example crate's `async` feature:

```
cargo run --bin blinky_async --features async
cargo run --bin baseboard_demo_async --features async
```

The board comes up through `uferris_init_async` instead of `uferris_init`. It
wires the same pins to the same peripherals, but reaches for `embassy-rp`'s
`async` constructors, so that the I2C and ADC operations are futures: the I2C
bus is shared through an `embassy-sync` mutex rather than a `RefCell`, the ADC
suspends on its FIFO interrupt rather than spinning on the ready flag, and
button 5 becomes something the board can wait on. The two init functions are
mutually exclusive — each consumes the peripheral singletons — so a program
calls one or the other.

The BSP binds `I2C1_IRQ` and `ADC_IRQ_FIFO` for those two drivers, so an
application must not bind them itself. The pin wait needs no binding:
`embassy-rp` claims `IO_IRQ_BANK0` under its `rt` feature and `embassy_rp::init`
enables it.

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
  driver, which the example's `async` feature turns on in `embassy-rp` as
  `time-driver` — the chip's always-on microsecond timer.

**These demos are still silent**, exactly like the blocking ones — this board
has no console either way. The observable difference is at the button phases of
`baseboard_demo_async`: pressing button 5 advances the demo without the core
having spun on the pin waiting for it, and the LED and 7-segment phases that
follow run off timer interrupts rather than cycle counting.

The power board is not part of the `async` board yet, so there is no
`full_board_demo_async`.

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
