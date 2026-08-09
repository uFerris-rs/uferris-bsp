# Xiao ESP32-C5 Examples

| Example | Description |
| ------- | ----------- |
| `blinky` | Blink LED 1 on the baseboard |
| `baseboard_demo` | Tour of the baseboard peripherals (LEDs, buttons, buzzer, 7-segment, LDR, RTC) |
| `full_board_demo` | Tour of the baseboard & powerboard peripherals |
| `blinky_async` | Blink LED 1, on the `async` board API |
| `baseboard_demo_async` | The baseboard tour, on the `async` board API |

Run with `cargo run --bin <example>` from this directory. The two `_async`
examples need their feature: `cargo run --bin blinky_async --features async`.

The buzzer phase of the demos runs on this board but stays silent: `esp-hal` has
no PWM (LEDC/MCPWM) driver for the ESP32-C5 yet, so `buzz_on`/`buzz_off` are
accepted and do nothing. See the note in the [crate README](../../README.md).

## The `async` examples

`blinky_async` and `baseboard_demo_async` run the same demos against the BSP's
`async` board API. Build them with the example crate's `async` feature:

```
cargo run --bin blinky_async --features async
cargo run --bin baseboard_demo_async --features async
```

**These demos print.** Unlike the nRF and RP boards, the Xiao ESP32-C5 already
has a serial console that `cargo run` attaches to, so the `async` demos stream
their output over the same connection the blocking ones use — no probe and no
extra tooling. They are the first `async` uFerris demos that say anything.

The BSP moves both the I2C driver and the ADC into `esp-hal`'s `async` mode,
so both are driven through their interrupts rather than polled, and button 5
becomes something the board can wait on rather than spin on.

**The runtime lives here, in the example, not in the BSP.** The BSP's `async`
feature is a pure code gate: it starts no executor and installs no time driver.
These examples bring:

- `esp-rtos`, with its `embassy` and `esp-alloc` features. `#[esp_rtos::main]`
  replaces `#[esp_hal::main]` — it stands up an `esp-rtos` embassy executor and
  spawns `main` on it as a task.
- `embassy-time`, for `embassy_time::Delay`. This is what the demos' delays run
  on, in place of the blocking examples' `esp_hal::delay::Delay`. Its driver is
  the time queue `esp-rtos`'s `embassy` feature installs.
- `embassy-executor`, for the `Spawner` type in `main`'s signature. No `arch-*`
  feature is enabled on it: `esp-rtos` supplies the executor itself.

**The board comes up before the scheduler, and that order is not optional.**
`esp_rtos::start` needs `TIMG0` and `SW_INTERRUPT`, which live in the same
`esp_hal::Peripherals` struct `uferris_init_async` takes by value, and a struct
with a field moved out of it cannot be passed on. So the BSP takes the struct
whole and hands those two back next to the board:

```rust
let (mut uferris, rtos_parts) = uferris_init_async(peripherals);

let timg0 = TimerGroup::new(rtos_parts.timg0);
let sw_interrupt = SoftwareInterruptControl::new(rtos_parts.sw_interrupt);
esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);
```

That is also why `uferris_init_async` is not a future on the ESP boards, unlike
every other one: `esp-rtos`'s thread mode executor will not suspend a task
before its scheduler is running, so nothing may await until those three lines
have run.

The power board is not part of the `async` board yet, so there is no
`full_board_demo_async`.