use embedded_hal_bus::spi::RefCellDevice;
use embedded_sdmmc::SdCard;
use esp_hal::{Blocking, delay::Delay, gpio::Output, spi::master::Spi};

pub struct SpiDevices<'d> {
    pub shared_device: RefCellDevice<'d, Spi<'d, Blocking>, Output<'d>, Delay>,

    pub sd_card_device:
        Option<SdCard<RefCellDevice<'d, Spi<'d, Blocking>, Output<'d>, Delay>, Delay>>,
}
