use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::{Blocking, i2c::master::I2c};
use ina219::{SyncIna219, calibration::IntCalibration};

pub struct I2cDevices<'d> {
    pub shared_device: RefCellDevice<'d, I2c<'d, Blocking>>,
    pub ina219_device: SyncIna219<RefCellDevice<'d, I2c<'d, Blocking>>, IntCalibration>,
}
