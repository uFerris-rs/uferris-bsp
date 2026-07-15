<p align="center">
  <img src="https://i.imgur.com/gAPf1TI.png" width="50" alt="uFerris logo"/>
  <br/>
</p>

<h1 align="center">uFerris Board Support Package</h1>

<p align="center">
  <strong>A flexible, hardware-agnostic BSP crate for the uFerris Xiao carrier board</strong>
</p>

<p align="center">
  <img src="https://i.imgur.com/KcvXhPw.png" width="250" alt="uFerris board photo"/>
</p>

<br/>


<div align="center">

[![crates.io](https://img.shields.io/crates/v/uferris-bsp.svg)](https://crates.io/crates/uferris-bsp)
[![docs.rs](https://docs.rs/uferris-bsp/badge.svg)](https://docs.rs/uferris-bsp)

</div>

**uFerris** is a Rust embedded learning/experimentation kit that can accept various **[Seeed Studio Xiao](seeedstudio.com/xiao-series-page)** controllers.

The `uferris-bsp` crate provides a **generic Board Support Package** that aims to be mostly MCU-agnostic, allowing the same high-level board API to work across different supported Xiao controllers (ESP32-C3, RP2040, etc.).

Controller-specific support is enabled via **feature flags**.

uFerris is an open-source hardware project. The hardware source is available on the **[uferris-hw](https://github.com/uFerris-rs/uferris-hw)** repo. uFerris can also be acquired from **[The Embedded Rustacean Store](http://shop.theembeddedrustacean.com/)**.

## Architecture Overview

The crate adopts the following layered approach:

<p align="center">
  <img src="https://i.imgur.com/SD77pGl.png" width="520" alt="uFerris BSP architecture diagram"/>
</p>

The architechture adds two layers on top of existing community crates:
- **uFerris Board Logic Layer**: This layer implments the hardware-agnostic uFerris board API. 
- **uFerris Board Adapter Layer**: This layer maps the generic logic to concrete MCU HALs (uses `embedded-hal` traits where possible).

## `async` Support

Although there is an `embassy` feature flag, async support is not implemented for all devices. Adding async support would require implementing async versions of the main board methods (most likely using `embedded-hal-async` traits).

## Support Status

| Controller       | Feature flag          | Support Status | `async` Support |
|------------------|-----------------------|----------------|-----------------|
| [Xiao ESP32-C3](https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)    | `xiao-esp32c3`        | ✅             | ❌              |
| [Xiao ESP32-C5](https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C5-p-6609.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)     | -                    | ❌             | ❌              |
| [Xiao ESP32-C6](https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)     | `xiao-esp32c6`       | ✅             | ❌              |
| [Xiao ESP32-S3](https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)     | `xiao-esp32s3`       | ✅             | ❌              |
| [Xiao ESP32-S3 Sense](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris) | `xiao-esp32s3`     | ✅             | ❌              |
| [Xiao nRF52840](https://www.seeedstudio.com/Seeed-XIAO-BLE-nRF52840-p-5201.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)    | -                     | ❌             | ❌              |
| [Xiao nRF52840 Sense](https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris) | -                  | ❌             | ❌              |
| [Xiao nRF54L15](https://www.seeedstudio.com/XIAO-nRF54L15-p-6493.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)    | -                     | ❌             | ❌              |
| [Xiao nRF54L15 Sense](https://www.seeedstudio.com/XIAO-nRF54L15-Sense-p-6494.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris) | -                  | ❌             | ❌              |
| [Xiao RP2350](https://www.seeedstudio.com/Seeed-XIAO-RP2350-p-5944.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)      | -                     | ❌             | ❌              |
| [Xiao RP2040](https://www.seeedstudio.com/XIAO-RP2040-v1-0-p-5026.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)      | -                     | ❌             | ❌              |
| [Xiao SAMD21](https://www.seeedstudio.com/Seeeduino-XIAO-Arduino-Microcontroller-SAMD21-Cortex-M0+-p-4426.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)      | -                     | ❌             | ❌              |
| [Xiao RA4M1](https://www.seeedstudio.com/Seeed-XIAO-RA4M1-p-5943.html?utm_source=blog&utm_medium=TER&utm_campaign=uFerris)       | -                     | ❌             | ❌              |
| Xiao MG24        | -                     | ❌             | ❌              |
| Xiao MG24 Sense  | -                     | ❌             | ❌              |



## Adding Support for a New Xiao Controller

Adding support for a new Xiao board entails two parts:
1. **Adding a Device Feature Flag in `Cargo.toml`**: A feature flag that imports the new device HAL needs to be added.
2. **Adding a Device Board Adapter**: This entails adding a new board definition (adapter layer) under the crate `boards/` folder.

The rest of the crate files should not need to change.

## Feature Flags

Available Cargo features:

- `xiao-esp32c3` — Xiao ESP32-C3 Device Support
- `powerboard` — uFerris Megalops Power Board Extension Support
- `embassy` — embassy Support Feature Flag

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

    // 2. Initialize the uFerris board abstraction
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

Licensed under Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)

---

Made with 🦀
