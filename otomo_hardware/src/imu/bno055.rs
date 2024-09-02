use stm32f4xx_hal::{i2c::Error as I2cError, timer::SysDelay};

use super::bno_regs::{OperatingMode, PowerMode, Registers};
use embedded_hal::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
};

const ADDR_0: u8 = 0x28;
const ADDR_1: u8 = 0x29;
const CHIP_ID: u8 = 0xA0;

const REMAP_CONFIG_P0: u8 = 0x21;
const REMAP_CONFIG_P1: u8 = 0x24; // default
const REMAP_CONFIG_P2: u8 = 0x24;
const REMAP_CONFIG_P3: u8 = 0x21;
const REMAP_CONFIG_P4: u8 = 0x24;
const REMAP_CONFIG_P5: u8 = 0x21;
const REMAP_CONFIG_P6: u8 = 0x21;
const REMAP_CONFIG_P7: u8 = 0x24;

const REMAP_SIGN_P0: u8 = 0x04;
const REMAP_SIGN_P1: u8 = 0x00; // default
const REMAP_SIGN_P2: u8 = 0x06;
const REMAP_SIGN_P3: u8 = 0x02;
const REMAP_SIGN_P4: u8 = 0x03;
const REMAP_SIGN_P5: u8 = 0x01;
const REMAP_SIGN_P6: u8 = 0x07;
const REMAP_SIGN_P7: u8 = 0x05;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error<E> {
    Timeout,
    InvalidAddress(u8),
    Inner(E),
}

pub struct Bno055<T> {
    i2c: T,
    addr: u8,
}

impl<T, E> Bno055<T>
where
    T: I2c<SevenBitAddress, Error = E>,
{
    pub fn new(i2c: T, alt_addr: bool) -> Self {
        let addr = if alt_addr { ADDR_0 } else { ADDR_1 };

        Self { i2c, addr }
    }

    fn read_u8(&mut self, reg: Registers) -> Result<u8, E> {
        let mut buf = [0_u8; 1];
        self.i2c.write_read(self.addr, &[reg.to_u8()], &mut buf)?;

        Ok(buf[0])
    }

    fn write_u8(&mut self, reg: Registers, val: u8) -> Result<(), E> {
        let buf = [reg.to_u8(), val];
        self.i2c.write(self.addr, &buf)
    }

    pub fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
        let mut buf = [Registers::ChipId.to_u8()];
        let mut timeout = 850; // According to adafruit datasheet, can take up to 850ms to boot
        'outer: while timeout > 0 {
            if let Ok(b) = self.read_u8(Registers::ChipId) {
                if b == CHIP_ID {
                    break 'outer;
                } else {
                    return Err(Error::InvalidAddress(b));
                }
            } else {
                delay.delay_ms(10);
                timeout -= 10;
            }
        }
        if timeout <= 0 {
            return Err(Error::Timeout);
        }

        self.write_u8(Registers::OperatingMode, OperatingMode::Config.to_u8())
            .map_err(Error::Inner)?;
        delay.delay_ms(30);

        self.write_u8(Registers::SystemTrigger, 0x20)
            .map_err(Error::Inner)?;
        delay.delay_ms(650);

        self.write_u8(Registers::PowerMode, PowerMode::Normal.to_u8())
            .map_err(Error::Inner)?;
        self.write_u8(Registers::SystemTrigger, 0)
            .map_err(Error::Inner)?;

        Ok(())
    }

    pub fn get_temp(&mut self) {}
}
