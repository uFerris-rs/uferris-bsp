use bitmask_enum::bitmask;
use embedded_hal::i2c::I2c;

// ==========================================
// Constants & Registers
// ==========================================
const TCA6424_ADDR: u8 = 0x22;

const IN_PORT0: u8 = 0x80;
// const IN_PORT1: u8 = 0x81;
const IN_PORT2: u8 = 0x82;
const OUT_PORT0: u8 = 0x84;
const OUT_PORT1: u8 = 0x85;
const OUT_PORT2: u8 = 0x86;
const CONFIG_PORT0: u8 = 0x8C;

// Direction Configuration for TCA6424
// 0 = Output, 1 = Input
const PORT0_DIR: u8 = 0x7F; // All outputs except SW7 (bit 0/1) & SW4 (bit 3) ... wait, check your mask below
const PORT1_DIR: u8 = 0x00; // All outputs
const PORT2_DIR: u8 = 0xC0; // All outputs except SW6 (bit 6/7)

// ==========================================
// Enums
// ==========================================

#[bitmask(u8)]
enum IoExpPort0 {
    Sw7Pos1, // Port 0 Pin 0
    Sw7Pos2, // Port 0 Pin 1
    P02,     // Port 0 Pin 2 (Unused)
    Sw4,     // Port 0 Pin 3
    Sw3,     // Port 0 Pin 4
    Sw2,     // Port 0 Pin 5
    Sw1,     // Port 0 Pin 6
    SegG,    // Port 0 Pin 7
}

#[bitmask(u8)]
enum IoExpPort1 {
    Dp,   // Port 1 Pin 0
    SegA, // Port 1 Pin 1
    SegB, // Port 1 Pin 2
    SegC, // Port 1 Pin 3
    SegD, // Port 1 Pin 4
    SegE, // Port 1 Pin 5
    SegF, // Port 1 Pin 6
    P17,  // Port 1 Pin 7 (Unused)
}

#[bitmask(u8)]
enum IoExpPort2 {
    Digit4,  // Port 2 Pin 0
    Digit3,  // Port 2 Pin 1
    Digit2,  // Port 2 Pin 2
    Digit1,  // Port 2 Pin 3
    Led3,    // Port 2 Pin 4
    Led2,    // Port 2 Pin 5
    Sw6Pos1, // Port 2 Pin 6
    Sw6Pos2, // Port 2 Pin 7
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

// ==========================================
// Driver Struct
// ==========================================

/// Driver for the TCA6424 I/O Expander on the uFerris board.
/// This driver owns the I2C bus handle passed to it.
pub struct IoExpander<I2C> {
    i2c: I2C,
}

impl<I2C: I2c> IoExpander<I2C> {
    /// Create a new instance of the IO Expander driver.
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Initialize the I/O expander.
    /// Configures directions and resets outputs.
    pub fn init(&mut self) -> Result<(), I2C::Error> {
        // Write I/O Direction to TCA6424 Config Registers (Auto-increment is usually supported)
        // Note: Check if your chip supports auto-increment writes for consecutive registers.
        // If not, you might need individual writes. Assuming standard TCA behavior here:
        self.i2c.write(
            TCA6424_ADDR,
            &[CONFIG_PORT0, PORT0_DIR, PORT1_DIR, PORT2_DIR],
        )?;

        // Reset all output ports to low
        self.i2c.write(TCA6424_ADDR, &[OUT_PORT0, 0, 0, 0])?;

        Ok(())
    }

    // ----------------------------------------------------------------
    // LED Control
    // ----------------------------------------------------------------

    pub fn led2_on(&mut self) -> Result<(), I2C::Error> {
        self.modify_port2(|current| current | IoExpPort2::Led2.bits())
    }

    pub fn led2_off(&mut self) -> Result<(), I2C::Error> {
        self.modify_port2(|current| current & !IoExpPort2::Led2.bits())
    }

    pub fn led3_on(&mut self) -> Result<(), I2C::Error> {
        self.modify_port2(|current| current | IoExpPort2::Led3.bits())
    }

    pub fn led3_off(&mut self) -> Result<(), I2C::Error> {
        self.modify_port2(|current| current & !IoExpPort2::Led3.bits())
    }

    // ----------------------------------------------------------------
    // Switch Inputs
    // ----------------------------------------------------------------

    pub fn read_sw1(&mut self) -> Result<bool, I2C::Error> {
        self.read_port0_bit(IoExpPort0::Sw1)
    }

    pub fn read_sw2(&mut self) -> Result<bool, I2C::Error> {
        self.read_port0_bit(IoExpPort0::Sw2)
    }

    pub fn read_sw3(&mut self) -> Result<bool, I2C::Error> {
        self.read_port0_bit(IoExpPort0::Sw3)
    }

    pub fn read_sw4(&mut self) -> Result<bool, I2C::Error> {
        self.read_port0_bit(IoExpPort0::Sw4)
    }

    pub fn read_slide_sw6_position(&mut self) -> Result<SwPos, I2C::Error> {
        let port2 = self.read_register(IN_PORT2)?;

        if port2 & IoExpPort2::Sw6Pos1.bits() == 0 {
            Ok(SwPos::Up)
        } else if port2 & IoExpPort2::Sw6Pos2.bits() == 0 {
            Ok(SwPos::Down)
        } else {
            Ok(SwPos::Undefined)
        }
    }

    pub fn read_slide_sw7_position(&mut self) -> Result<SwPos, I2C::Error> {
        let port0 = self.read_register(IN_PORT0)?;

        if port0 & IoExpPort0::Sw7Pos1.bits() == 0 {
            Ok(SwPos::Up)
        } else if port0 & IoExpPort0::Sw7Pos2.bits() == 0 {
            Ok(SwPos::Down)
        } else {
            Ok(SwPos::Undefined)
        }
    }

    // ----------------------------------------------------------------
    // Seven Segment Display
    // ----------------------------------------------------------------

    pub fn write_seven_segment_digit(
        &mut self,
        digit: SevenSegDigit,
        value: Option<u8>,
    ) -> Result<(), I2C::Error> {
        // 1. Activate the Specified Digit (Port 2)
        // Read-Modify-Write Port 2 to preserve LED states
        let digit_bits = match digit {
            SevenSegDigit::Digit1 => IoExpPort2::Digit1.bits(),
            SevenSegDigit::Digit2 => IoExpPort2::Digit2.bits(),
            SevenSegDigit::Digit3 => IoExpPort2::Digit3.bits(),
            SevenSegDigit::Digit4 => IoExpPort2::Digit4.bits(),
        };

        self.modify_port2(|current| current | digit_bits)?;

        // 2. Write the Segments (Port 1)
        let segment_map = match value {
            Some(v) => match v {
                0 => 0b00111111,
                1 => 0b00000110,
                2 => 0b01011011,
                3 => 0b01001111,
                4 => 0b01100110,
                5 => 0b01101101,
                6 => 0b01111101,
                7 => 0b00000111,
                8 => 0b01111111,
                9 => 0b01101111,
                _ => 0b00000000,
            },
            None => 0x00,
        };

        self.i2c.write(TCA6424_ADDR, &[OUT_PORT1, segment_map])
    }

    pub fn seven_segment_display_colon_en(&mut self, enable: bool) -> Result<(), I2C::Error> {
        // 1. Activate Digits 2 & 4 (Port 2)
        let digits = IoExpPort2::Digit2.or(IoExpPort2::Digit4).bits();
        self.modify_port2(|current| current | digits)?;

        // 2. Activate Colon (Port 1 - Read/Mod/Write needed to preserve segments if active)
        let mut port1 = self.read_register(OUT_PORT1)?;

        if enable {
            port1 |= IoExpPort1::Dp.bits();
        } else {
            port1 &= !IoExpPort1::Dp.bits();
        }

        self.i2c.write(TCA6424_ADDR, &[OUT_PORT1, port1])
    }

    // ----------------------------------------------------------------
    // Helper Methods
    // ----------------------------------------------------------------

    /// Reads a single byte from a register
    fn read_register(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8];
        self.i2c.write_read(TCA6424_ADDR, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    /// Helper for checking a switch bit (Active Low check)
    fn read_port0_bit(&mut self, mask: IoExpPort0) -> Result<bool, I2C::Error> {
        let val = self.read_register(IN_PORT0)?;
        // Assuming Active Low (pressed = 0)
        Ok((val & mask.bits()) == 0)
    }

    /// Read-Modify-Write helper for Port 2 (Used heavily by LEDs and Digits)
    fn modify_port2<F>(&mut self, f: F) -> Result<(), I2C::Error>
    where
        F: FnOnce(u8) -> u8,
    {
        let current = self.read_register(OUT_PORT2)?;
        let new_val = f(current);
        self.i2c.write(TCA6424_ADDR, &[OUT_PORT2, new_val])
    }
}
