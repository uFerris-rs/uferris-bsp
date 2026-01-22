use embedded_hal::i2c::I2c;
use ina219::{SyncIna219, calibration::IntCalibration};

pub struct I2cDevices<I2C> {
    pub shared_device: I2C, // Used for generic access if needed
    pub ina219: SyncIna219<I2C, IntCalibration>,
}
