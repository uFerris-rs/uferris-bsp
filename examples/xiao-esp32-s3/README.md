# Xiao ESP32-S3 Examples

| Example | Description |
|---------|-------------|
| `blinky` | Blink LED1 |
| `baseboard_demo` | Exercise every base board peripheral |
| `full_board_demo` | Base board + power board (INA219, SD card) |
| `blinky_async` | `blinky` on the `async` API |
| `baseboard_demo_async` | `baseboard_demo` on the `async` API |

## Run

```bash
cargo run --bin blinky
cargo run --bin baseboard_demo
cargo run --bin full_board_demo --features power-board
cargo run --bin blinky_async --features async
cargo run --bin baseboard_demo_async --features async
```

## Setup

- Xtensa toolchain (once): `cargo install espup && espup install`.
- `cargo install espflash` (once). `cargo run` flashes and opens the serial monitor.
