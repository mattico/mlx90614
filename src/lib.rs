#![no_std]

extern crate embedded_hal as hal;
use hal::blocking::i2c;
use hal::blocking::delay;
use core::fmt;

#[derive(Copy, Clone)]
#[repr(u8)]
enum Register {
    TA = 0x06,
    TOBJ1 = 0x07,
    TOBJ2 = 0x08,
    TOMAX = 0x20,
    TOMIN = 0x21,
//    PWMCTRL = 0x22,
//    TARANGE = 0x23,
//    KE = 0x24,
//    CONFIG = 0x25,
    ADDRESS = 0x2E,
    ID0 = 0x3C,
    ID1 = 0x3D,
    ID2 = 0x3E,
    ID3 = 0x3F,
//   SLEEP = 0xFF,
}

#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "fmt", derive(Debug))]
pub enum Temperature {
    Raw(i16),
    Kelvin(f32),
    Celsius(f32),
    Farenheit(f32),
}

#[cfg(feature = "fmt")]
impl fmt::Display for Temperature {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Temperature::Raw(v) => write!(f, "{} Raw", v),
            Temperature::Kelvin(v) => write!(f, "{:3.2}K", v),
            Temperature::Celsius(v) => write!(f, "{:3.2}C", v),
            Temperature::Farenheit(v) => write!(f, "{:3.2}F", v),
        }
    }
}

impl Temperature {
    #[inline]
    pub fn into_raw(self) -> i16 {
        match self {
            Temperature::Raw(v) => v,
            Temperature::Kelvin(v) => (v * 50.0) as i16,
            v @ Temperature::Celsius(_) | v @ Temperature::Farenheit(_) => v.as_kelvin().into_raw(),
        }
    }

    #[inline]
    pub fn as_raw(self) -> Self {
        Temperature::Raw(self.into_raw())
    }

    #[inline]
    pub fn into_kelvin(self) -> f32 {
        match self {
            Temperature::Kelvin(v) => v,
            Temperature::Raw(v) => (v as f32) / 50.0,
            Temperature::Celsius(v) => v - 273.15,
            v @ Temperature::Farenheit(_) => v.as_celsius().into_kelvin(),
        }
    }

    #[inline]
    pub fn as_kelvin(self) -> Self {
        Temperature::Kelvin(self.into_kelvin())
    }

    #[inline]
    pub fn into_celsius(self) -> f32 {
        match self {
            Temperature::Celsius(v) => v,
            Temperature::Kelvin(v) => v + 273.15,
            Temperature::Farenheit(v) => (v - 32.0) * 5.0 / 9.0,
            v @ Temperature::Raw(_) => v.as_kelvin().into_celsius(),
        }
    }

    #[inline]
    pub fn as_celsius(self) -> Self {
        Temperature::Celsius(self.into_celsius())
    }

    #[inline]
    pub fn into_farenheit(self) -> f32 {
        match self {
            Temperature::Farenheit(v) => v,
            Temperature::Celsius(v) => v * 9.0 / 5.0 + 32.0,
            v @ Temperature::Raw(_) | v @ Temperature::Kelvin(_) => v.as_celsius().into_farenheit(),
        }
    }

    #[inline]
    pub fn as_farenheit(self) -> Self {
        Temperature::Farenheit(self.into_farenheit())
    }
}

#[cfg_attr(feature = "fmt", derive(Debug))]
pub enum Error<I2CError> {
    I2C(I2CError),
    Flag,
    Crc,
    InvalidAddress(u8),
}

impl<I2CError> From<I2CError> for Error<I2CError> {
    fn from(item: I2CError) -> Self {
        Error::I2C(item)
    }
}

pub struct Mlx90614<I2C, Delay> {
    i2c: I2C,
    address: u8,
    delay: Delay,
}

pub const DEFAULT_ADDRESS: u8 = 0x5A;

impl<I2C, I2CError, Delay> Mlx90614<I2C, Delay> 
where
    I2C: i2c::WriteRead<Error = I2CError> + i2c::Read<Error = I2CError> + i2c::Write<Error = I2CError>,
    Delay: delay::DelayMs<u8>,
{
    pub fn new(i2c: I2C, delay: Delay) -> Result<Self, Error<I2CError>> {
        Self::with_address(i2c, DEFAULT_ADDRESS, delay)
    }

    pub fn with_address(i2c: I2C, address: u8, delay: Delay) -> Result<Self, Error<I2CError>> {
        validate_address(address)?;
        Ok(Mlx90614 { i2c, address, delay })
    }

    pub fn read_object(&mut self) -> Result<Temperature, Error<I2CError>> {
        let raw = self.i2c_read(Register::TOBJ1)?;
        if raw & 0x8000 != 0 {
            Err(Error::Flag)
        } else {
            Ok(Temperature::Raw(raw as i16))
        }
    }

    pub fn read_object2(&mut self) -> Result<Temperature, Error<I2CError>> {
        let raw = self.i2c_read(Register::TOBJ2)?;
        if raw & 0x8000 != 0 {
            Err(Error::Flag)
        } else {
            Ok(Temperature::Raw(raw as i16))
        }
    }

    pub fn read_ambient(&mut self) -> Result<Temperature, Error<I2CError>> {
        let raw = self.i2c_read(Register::TA)?;
        Ok(Temperature::Raw(raw as i16))
    }

    pub fn get_min(&mut self) -> Result<Temperature, Error<I2CError>> {
        let raw = self.i2c_read(Register::TOMIN)?;
        Ok(Temperature::Raw(raw as i16))
    }

    pub fn get_max(&mut self) -> Result<Temperature, Error<I2CError>> {
        let raw = self.i2c_read(Register::TOMAX)?;
        Ok(Temperature::Raw(raw as i16))
    }

    pub fn set_min(&mut self, min: Temperature) -> Result<(), I2CError> {
        self.eeprom_write(Register::TOMIN, min.into_raw() as u16)
    }

    pub fn set_max(&mut self, max: Temperature) -> Result<(), I2CError> {
        self.eeprom_write(Register::TOMAX, max.into_raw() as u16)
    }

    // pub fn sleep(&mut self) -> Result<(), Error<I2CError>> {
    //     let crc = crc8(&[self.address << 1, Register::SLEEP as u8]);

    //     self.i2c.write(self.address, &[Register::SLEEP as u8, crc])?;

    //     // TODO: pull SCL low

    //     Ok(())
    // }

    // pub fn wake(&mut self) -> Result<(), Error<I2CError>> {
    //     // TODO: manual control of pin writes while having i2c peripheral?
    //     Ok(())
    // }

    pub fn get_address(&mut self) -> Result<u8, Error<I2CError>> {
        self.i2c_read(Register::ADDRESS).map(|v| v as u8)
    }

    pub fn set_address(&mut self, address: u8) -> Result<(), Error<I2CError>> {
        validate_address(address)?;

        // We're only supposted to modify the lsbyte
        let mut address_value = self.i2c_read(Register::ADDRESS)?;        

        address_value &= 0xFF00;
        address_value |= address as u16;

        self.eeprom_write(Register::ADDRESS, address_value)?;

        Ok(())
    }

    pub fn get_id(&mut self) -> Result<u64, Error<I2CError>> {
        let id0 = self.i2c_read(Register::ID0)?;
        let id1 = self.i2c_read(Register::ID1)?;
        let id2 = self.i2c_read(Register::ID2)?;
        let id3 = self.i2c_read(Register::ID3)?;
        Ok((id3 as u64) << 48 | (id2 as u64) << 32 | (id1 as u64) << 16 | id0 as u64)
    }

    fn i2c_read(&mut self, reg: Register) -> Result<u16, Error<I2CError>> {
        let reg = reg as  u8;
        let mut data = [0u8; 3];
        self.i2c.write_read(self.address, &[reg], &mut data)?;

        let lsb = data[0];
        let msb = data[1];
        let pec = data[2];
        
        let crc = crc8(&[self.address << 1, reg, (self.address << 1) + 1, lsb, msb]);

        if crc != pec { 
            return Err(Error::Crc);
        }

        Ok(((msb as u16) << 8) | lsb as u16)
    }

    fn i2c_write(&mut self, reg: Register, data: u16) -> Result<(), I2CError> {
        let reg = reg as u8;
        let lsb = (data & 0xFF) as u8;
        let msb = (data >> 8) as u8;

        let crc = crc8(&[self.address << 1, reg, lsb, msb]);

        self.i2c.write(self.address, &[reg, lsb, msb, crc])?;

        Ok(())
    }

    fn eeprom_write(&mut self, reg: Register, data: u16) -> Result<(), I2CError> {
        // zero out EEPROM register
        self.i2c_write(reg, 0)?;

        // wait for write to complete
        self.delay.delay_ms(5);
        
        // write data
        self.i2c_write(reg, data)?;

        // wait for write to complete
        self.delay.delay_ms(5);

        Ok(())
    }
}

fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc <<= 1;
                crc ^= 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Returns an error if the given address is not a valid I2C address.
fn validate_address<T>(address: u8) -> Result<(), Error<T>> {
    if address > 0x80 || address == 0 {
        Err(Error::InvalidAddress(address))
    } else {
        Ok(())
    }
}
