<p align="center">
  <img src="https://i.imgur.com/gAPf1TI.png" width="50" alt="uFerris logo"/>
  <br/>
</p>

<h1 align="center">µFerris Board Support Package</h1>

<p align="center">
  <strong>A flexible, hardware-agnostic BSP crate for the µFerris Xiao carrier board</strong>
</p>

<p align="center">
  <img src="https://i.imgur.com/KcvXhPw.png" width="250" alt="µFerris board photo"/>
</p>

<br/>


<div align="center">

[![crates.io](https://img.shields.io/crates/v/uferris-bsp.svg)](https://crates.io/crates/uferris-bsp)
[![docs.rs](https://docs.rs/uferris-bsp/badge.svg)](https://docs.rs/uferris-bsp)

</div>

**µFerris** is a Rust embedded learning/experimentation kit that can accept various **[Seeed Studio Xiao](seeedstudio.com/xiao-series-page)** controllers.

The `uferris-bsp` crate provides a **generic Board Support Package** that aims to be mostly MCU-agnostic, allowing the same high-level board API to work across different supported Xiao controllers (ESP32-C3, RP2040, etc.).

Controller-specific support is enabled via **feature flags**.

µFerris is an open-source hardware project. The hardware source is available on the **[uferris-hw](https://github.com/uFerris-rs/uferris-hw)** repo. µFerris can also be acquired from **[The Embedded Rustacean Store](http://shop.theembeddedrustacean.com/)**.

## Architecture Overview

The crate adopts the following layered approach:

<p align="center">
  <img src="https://i.imgur.com/SD77pGl.png" width="520" alt="µFerris BSP architecture diagram"/>
</p>

The architechture adds two layers on top of existing community crates:
- **µFerris Board Logic Layer**: This layer implments the hardware-agnostic µFerris board API. 
- **µFerris Board Adapter Layer**: This layer maps the generic logic to concrete MCU HALs (uses `embedded-hal` traits where possible).

## `async` Support

Although there is an `async` feature flag, async support is not implemented yet. The flag currently only starts the `esp-rtos` embassy scheduler on ESP devices (it is not yet supported on other controllers). Adding full async support would require implementing async versions of the main board methods (most likely using `embedded-hal-async` traits), possibly backed by the embassy HALs used in either async or blocking mode.

## Support Status

| Controller       | Feature flag          | Support Status | `async` Support |
|------------------|-----------------------|:----------------:|:-----------------:|
| [Xiao ESP32-C3](https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)    | `xiao-esp32c3`        | ✅             | ❌              |
| [Xiao ESP32-C5](https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C5-p-6609.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)     | `xiao-esp32c5`       | ✅ [^1]        | ❌              |
| [Xiao ESP32-C6](https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)     | `xiao-esp32c6`       | ✅             | ❌              |
| [Xiao ESP32-S3](https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)     | `xiao-esp32s3`       | ✅             | ❌              |
| [Xiao ESP32-S3 Sense](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris) | `xiao-esp32s3`     | ✅             | ❌              |
| [Xiao nRF52840](https://www.seeedstudio.com/Seeed-XIAO-BLE-nRF52840-p-5201.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)    | `xiao-nrf52840`       | ✅ [^2] [^3]   | ❌              |
| [Xiao nRF52840 Sense](https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris) | `xiao-nrf52840`    | ✅ [^2] [^3]   | ❌              |
| [Xiao nRF54L15](https://www.seeedstudio.com/XIAO-nRF54L15-p-6493.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)    | `xiao-nrf54l15`       | ✅ [^4]        | ❌              |
| [Xiao nRF54L15 Sense](https://www.seeedstudio.com/XIAO-nRF54L15-Sense-p-6494.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris) | `xiao-nrf54l15`    | ✅ [^4]        | ❌              |
| [Xiao RP2350](https://www.seeedstudio.com/Seeed-XIAO-RP2350-p-5944.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)      | `xiao-rp2350`         | ✅ [^2]        | ❌              |
| [Xiao RP2040](https://www.seeedstudio.com/XIAO-RP2040-v1-0-p-5026.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)      | `xiao-rp2040`         | ✅ [^2]        | ❌              |
| [Xiao SAMD21](https://www.seeedstudio.com/Seeeduino-XIAO-Arduino-Microcontroller-SAMD21-Cortex-M0+-p-4426.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)      | -                     | ❌             | ❌              |
| [Xiao RA4M1](https://www.seeedstudio.com/Seeed-XIAO-RA4M1-p-5943.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)       | -                     | ❌             | ❌              |
| Xiao MG24        | -                     | ❌             | ❌              |
| Xiao MG24 Sense  | -                     | ❌             | ❌              |

[^1]: On the Xiao ESP32-C5 the buzzer is currently **stubbed and non-functional**. `esp-hal` does not yet provide a PWM (LEDC/MCPWM) driver for this chip, so the buzzer pin is only held low and `buzz_on`/`buzz_off` are accepted but do nothing. Every other baseboard peripheral works as usual.

[^2]: On the Xiao RP2040, the Xiao RP2350 and the Xiao nRF52840 the examples currently run **without serial console output**. Every board peripheral works, but none of these boards has a USB-to-UART bridge and the `embassy-rp` and `embassy-nrf` USB drivers are `async` only, so there is nothing for `println!` to print over yet. A USB CDC console arrives with `async` support. In the meantime, printing is available over RTT with `probe-rs` if a debug probe is wired to the board's SWD pads.

[^3]: The Xiao nRF52840 is flashed **over its resident UF2 bootloader only**. The examples are linked at `0x27000`, above the SoftDevice a stock board ships with, and `cargo run` hands the bootloader a UF2 file. Flashing over SWD is not supported and can leave the board needing a debug probe to recover — see [`examples/xiao-nrf52840/README.md`](examples/xiao-nrf52840/README.md). The same feature covers the nRF52840 Sense: it is the same chip, and the Sense's extra onboard sensors are not part of the carrier board API.

[^4]: The Xiao nRF54L15 examples **do print**. The board's USB-C connector goes to an onboard CMSIS-DAP debugger rather than to the MCU, so a single `cargo run` flashes the image and then streams the output back over **RTT** in the same terminal — no bootloader mode and no external probe. RTT is the native console here rather than a workaround: the nRF54L15 has no USB peripheral at all, so unlike the other boards nothing changes when `async` support lands. The same feature covers the nRF54L15 Sense: it is the same chip, and the Sense's extra onboard sensors are not part of the carrier board API. See [`examples/xiao-nrf54l15/README.md`](examples/xiao-nrf54l15/README.md).



## Adding Support for a New Xiao Controller

Adding support for a new Xiao board entails two parts:
1. **Adding a Device Feature Flag in `Cargo.toml`**: A feature flag that imports the new device HAL needs to be added.
2. **Adding a Device Board Adapter**: This entails adding a new board definition (adapter layer) under the crate `boards/` folder.

The rest of the crate files should not need to change.

## Feature Flags

Available Cargo features:

- `xiao-esp32c3` — Xiao ESP32-C3 Device Support
- `xiao-esp32c5` — Xiao ESP32-C5 Device Support
- `xiao-nrf52840` — Xiao nRF52840 Device Support
- `xiao-nrf54l15` — Xiao nRF54L15 Device Support
- `xiao-rp2040` — Xiao RP2040 Device Support
- `xiao-rp2350` — Xiao RP2350 Device Support
- `powerboard` — µFerris Megalops Power Board Extension Support
- `async` — `async` Support Feature Flag (currently ESP-only; starts the `esp-rtos` embassy scheduler)

## Quick Start / Usage
Generally, you need to:

- Enable the correct feature flags in your `Cargo.toml`
- Pass the correct HAL peripherals struct to `uferris_init()`

Everything else should stay the same regardless of which Xiao is mounted. A simple example is presented below.


```rust
#![no_std]
#![no_main]

use uferris_bsp::uferris_init;

#[entry]
fn main() -> ! {
    // 1. Get your HAL peripherals (depends on MCU)
    let peripherals = your_hal::take().unwrap();

    // 2. Initialize the µFerris board abstraction
    let mut uferris = uferris_init(peripherals).unwrap();

    // 3. Use the board API
    uferris.led1_on();

    loop {}
}
```

## Examples
Each supported board has a self-contained example project under [`examples/`](examples/).
Each board directory is a self-contained Cargo project with its own target, toolchain, and runner configuration.
See each board's README for the full list of examples it supports.

The demos themselves are written once, in [`examples/uferris-demos/`](examples/uferris-demos/): every board runs the same code against the generic board API.
A board's `src/bin/*.rs` only brings the controller up, opens whatever console it has, and hands both to the shared demo.

| Board | Examples |
| ----- | -------- |
| Xiao ESP32-C3 | [`examples/xiao-esp32-c3/`](examples/xiao-esp32-c3/) |
| Xiao ESP32-C6 | [`examples/xiao-esp32-c6/`](examples/xiao-esp32-c6/) |


## Running Examples

Run any example with:

```
cd examples/<board>
cargo run --bin <example>
```

For example, to run `blinky.rs` on a Xiao ESP32-C3:

```bash
cd examples/xiao-esp32-c3
cargo run --bin blinky
```

Use `cargo build --bin <example>` if you only want to compile without flashing.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

---

Made with 🦀
