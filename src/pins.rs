// Xiao Pin Connections
// D0 - LDR ADC Input
// D1 - Alarm LED Output (LED1)
// D2 - Buzzer Output
// D3 - Button Input (Switch Button 5)
// D4 - I2C SDA
// D5 - I2C SCL
// D6 - I/O Expander Interrupt Input / Expansion Connector / Uart Tx / SPI CS
// D7 - Expansion Connector / Uart Rx
// D8 - Expansion Connector / SPI SCK
// D9 - Expansion Connector / SPI MISO
// D10 - Expansion Connector / SPI MOSI

/// This struct defines the pin mappings for the different Xiao boards.
#[cfg(feature = "xiao-esp32c3")]
pub struct XiaoPins<'d> {
    pub d0: Option<esp_hal::peripherals::GPIO2<'d>>,
    pub d1: Option<esp_hal::peripherals::GPIO3<'d>>,
    pub d2: Option<esp_hal::peripherals::GPIO4<'d>>,
    pub d3: Option<esp_hal::peripherals::GPIO5<'d>>,
    pub d4: Option<esp_hal::peripherals::GPIO6<'d>>,
    pub d5: Option<esp_hal::peripherals::GPIO7<'d>>,
    pub d6: Option<esp_hal::peripherals::GPIO21<'d>>,
    pub d7: Option<esp_hal::peripherals::GPIO20<'d>>,
    pub d8: Option<esp_hal::peripherals::GPIO8<'d>>,
    pub d9: Option<esp_hal::peripherals::GPIO9<'d>>,
    pub d10: Option<esp_hal::peripherals::GPIO10<'d>>,
}
