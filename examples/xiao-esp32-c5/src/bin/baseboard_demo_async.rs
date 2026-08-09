#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use uferris_bsp::uferris_init_async;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The heap comes first: `esp-rtos` is built with `esp-alloc` here and
    // allocates its task stacks out of it.
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    // The board, then the scheduler. That order is forced: both the board and
    // `esp_rtos::start` want peripherals out of the one `Peripherals` struct,
    // and a partially moved struct cannot be passed on, so the BSP takes it
    // whole and hands the scheduler's two back. `uferris_init_async` is not a
    // future for the same reason — nothing may await until the next three lines
    // have run.
    let (mut uferris, rtos_parts) = uferris_init_async(peripherals);

    let timg0 = TimerGroup::new(rtos_parts.timg0);
    let sw_interrupt = SoftwareInterruptControl::new(rtos_parts.sw_interrupt);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    // From here on awaiting is safe.

    // The delay is the application's, not the board's: `embassy-time`'s, on the
    // time queue `esp-rtos`'s `embassy` feature drives. The blocking examples
    // use `esp_hal::delay::Delay` instead.
    let mut delay = embassy_time::Delay;

    // The demo itself is shared by every board: see `examples/uferris-demos`.
    // `esp_println::Printer` is the same console `println!` writes to.
    uferris_demos::asynch::baseboard_demo(&mut uferris, &mut delay, &mut esp_println::Printer).await
}
