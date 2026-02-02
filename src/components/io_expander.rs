use core::ops::Not;

use bitmask_enum::bitmask;
use embedded_hal::i2c::I2c;

// ------------------------------------------
// Constants & Registers
// ------------------------------------------
const TCA6424_ADDR: u8 = 0x22;

const IN_PORT0: u8 = 0x80;
const IN_PORT1: u8 = 0x81;
// const IN_PORT2: u8 = 0x82; // Placeholder only. Port 2 is not used as input
const OUT_PORT0: u8 = 0x84;
const OUT_PORT1: u8 = 0x85;
const OUT_PORT2: u8 = 0x86;
const CONFIG_PORT0: u8 = 0x8C;

// Direction Configuration for TCA6424
// 0 = Output, 1 = Input
const PORT0_DIR: u8 = 0xFF; // All inputs
const PORT1_DIR: u8 = 0xC0; // All outputs except P17 and P16
const PORT2_DIR: u8 = 0x00; // All outputs 

// ------------------------------------------
// Enums
// ------------------------------------------

#[bitmask(u8)]
enum IoExpPort0 {
    Sw7Pos1, // Port 0 Pin 0
    Sw7Pos2, // Port 0 Pin 1
    P02,     // Port 0 Pin 2 (Unused)
    P03,     // Port 0 Pin 3 (Unused)
    Sw4,     // Port 0 Pin 4
    Sw3,     // Port 0 Pin 5
    Sw2,     // Port 0 Pin 6
    Sw1,     // Port 0 Pin 7
}

#[bitmask(u8)]
enum IoExpPort1 {
    Digit1,  // Port 1 Pin 0
    Digit2,  // Port 1 Pin 1
    Digit3,  // Port 1 Pin 2
    Digit4,  // Port 1 Pin 3
    Led2,    // Port 1 Pin 4
    Led3,    // Port 1 Pin 5
    Sw6Pos2, // Port 1 Pin 6
    Sw6Pos1, // Port 1 Pin 7
}

#[bitmask(u8)]
enum IoExpPort2 {
    SegA, // Port 2 Pin 0
    SegB, // Port 2 Pin 1
    SegC, // Port 2 Pin 2
    SegD, // Port 2 Pin 3
    SegE, // Port 2 Pin 4
    SegF, // Port 2 Pin 5
    Dp,   // Port 2 Pin 6
    SegG, // Port 2 Pin 7
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub enum SwPos {
    Down,
    Up,
    Undefined,
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub enum SevenSegDigit {
    Digit1,
    Digit2,
    Digit3,
    Digit4,
}

// ------------------------------------------
// Driver Struct
// ------------------------------------------

pub struct IoExpander<I2C> {
    i2c: I2C,
}

impl<I2C: I2c> IoExpander<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    pub fn init(&mut self) -> Result<(), I2C::Error> {
        // Write configuration to each port individually
        // Configure I/O Expander pins before moving I2C to shared state
        // Write I/O Direction to TCA6424 Config Registers
        self.i2c
            .write(
                TCA6424_ADDR,
                &[CONFIG_PORT0, PORT0_DIR, PORT1_DIR, PORT2_DIR],
            )
            .unwrap();
        // Reset all output ports to low
        self.i2c.write(TCA6424_ADDR, &[OUT_PORT0, 0, 0, 0]).unwrap();
        Ok(())
    }

    // Active Low example
    pub fn led2_on(&mut self) -> Result<(), I2C::Error> {
        let port2 = self.read_register(OUT_PORT1)?;
        let new_port2 = port2 | IoExpPort1::Led2.bits();
        self.write_register(OUT_PORT1, new_port2)
    }

    pub fn led2_off(&mut self) -> Result<(), I2C::Error> {
        let port1 = self.read_register(OUT_PORT1)?;
        let new_port1 = port1 & IoExpPort1::Led2.bits().not();
        self.write_register(OUT_PORT1, new_port1)
    }

    pub fn led3_on(&mut self) -> Result<(), I2C::Error> {
        let port1 = self.read_register(OUT_PORT1)?;
        let new_port1 = port1 | IoExpPort1::Led3.bits();
        self.write_register(OUT_PORT1, new_port1)
    }

    pub fn led3_off(&mut self) -> Result<(), I2C::Error> {
        let port1 = self.read_register(OUT_PORT1)?;
        let new_port1 = port1 & IoExpPort1::Led3.bits().not();
        self.write_register(OUT_PORT1, new_port1)
    }

    // ------------------------------------------
    // Switch Inputs
    // ------------------------------------------

    pub fn read_sw1(&mut self) -> Result<bool, I2C::Error> {
        let port0 = self.read_register(IN_PORT0)?;
        if port0 & IoExpPort0::Sw1.bits() == 0 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn read_sw2(&mut self) -> Result<bool, I2C::Error> {
        let port0 = self.read_register(IN_PORT0)?;
        if port0 & IoExpPort0::Sw2.bits() == 0 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn read_sw3(&mut self) -> Result<bool, I2C::Error> {
        let port0 = self.read_register(IN_PORT0)?;
        if port0 & IoExpPort0::Sw3.bits() == 0 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn read_sw4(&mut self) -> Result<bool, I2C::Error> {
        let port0 = self.read_register(IN_PORT0)?;
        if port0 & IoExpPort0::Sw4.bits() == 0 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn read_slide_sw6_position(&mut self) -> Result<SwPos, I2C::Error> {
        let port2 = self.read_register(IN_PORT1)?;

        if port2 & IoExpPort1::Sw6Pos1.bits() == 0 {
            Ok(SwPos::Down)
        } else if port2 & IoExpPort1::Sw6Pos2.bits() == 0 {
            Ok(SwPos::Up)
        } else {
            Ok(SwPos::Undefined)
        }
    }

    pub fn read_slide_sw7_position(&mut self) -> Result<SwPos, I2C::Error> {
        let port0 = self.read_register(IN_PORT0)?;

        if port0 & IoExpPort0::Sw7Pos1.bits() == 0 {
            Ok(SwPos::Down)
        } else if port0 & IoExpPort0::Sw7Pos2.bits() == 0 {
            Ok(SwPos::Up)
        } else {
            Ok(SwPos::Undefined)
        }
    }

    // ------------------------------------------
    // Seven Segment Display
    // ------------------------------------------

    pub fn write_seven_segment_digit(
        &mut self,
        digit: SevenSegDigit,
        value: Option<u8>,
    ) -> Result<(), I2C::Error> {
        // 1. Activate the Specified Digit (Port 1)
        let digit_bits = match digit {
            SevenSegDigit::Digit1 => IoExpPort1::Digit1.bits(),
            SevenSegDigit::Digit2 => IoExpPort1::Digit2.bits(),
            SevenSegDigit::Digit3 => IoExpPort1::Digit3.bits(),
            SevenSegDigit::Digit4 => IoExpPort1::Digit4.bits(),
        };

        let port1 = self.read_register(OUT_PORT1)?;
        // Clear existing digit bits (0-3) then set the new one
        let new_port1 = (port1 & 0xF0) | digit_bits;
        self.write_register(OUT_PORT1, new_port1)?; // Fixed: Was OUT_PORT2

        // 2. Write the Segments (Port 2)
        let segment_map = match value {
            Some(v) => match v {
                0 => 0b00111111,
                1 => 0b00000110,
                2 => 0b10011011,
                3 => 0b10001111,
                4 => 0b10100110,
                5 => 0b10101101,
                6 => 0b10111101,
                7 => 0b00000111,
                8 => 0b10111111,
                9 => 0b10101111,
                _ => 0b00000000,
            },
            None => 0x00,
        };

        self.write_register(OUT_PORT2, segment_map) // Fixed: Was OUT_PORT1
    }

    pub fn seven_segment_display_colon_en(&mut self, enable: bool) -> Result<(), I2C::Error> {
        // 1. Activate Digits 2 & 4 (Port 1)
        let digits = IoExpPort1::Digit2.or(IoExpPort1::Digit4).bits();
        let mut port1 = self.read_register(OUT_PORT1)?;
        if enable {
            port1 |= digits;
        } else {
            port1 &= !digits;
        }
        self.write_register(OUT_PORT1, port1)?; // Fixed: Was OUT_PORT2

        // 2. Activate Colon (DP pin on Port 2)
        let mut port2 = self.read_register(OUT_PORT2)?;
        if enable {
            port2 |= IoExpPort2::Dp.bits();
        } else {
            port2 &= !IoExpPort2::Dp.bits();
        }

        self.write_register(OUT_PORT2, port2) // Fixed: Was OUT_PORT1
    }

    // ------------------------------------------
    // Helper Methods
    // ------------------------------------------

    /// Reads a single byte from a register
    fn read_register(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8];
        self.i2c.write_read(TCA6424_ADDR, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), I2C::Error> {
        self.i2c.write(TCA6424_ADDR, &[reg, value])
    }
}
