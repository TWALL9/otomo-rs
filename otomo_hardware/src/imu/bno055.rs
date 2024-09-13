use super::bno_regs::*;
use super::Vector3;
use embedded_hal::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
};

const ADDR_0: u8 = 0x28;
const ADDR_1: u8 = 0x29;
const CHIP_ID: u8 = 0xA0;

const MAG_CONVERT: f32 = 1.0 / 16.0;
const GYRO_CONVERT: f32 = 1.0 / 16.0;
const EULER_CONVERT: f32 = 1.0 / 16.0;
const ACCEL_CONVERT: f32 = 1.0 / 100.0;
const LINEAR_ACCEL_CONVERT: f32 = 1.0 / 100.0;
const GRAVITY_CONVERT: f32 = 1.0 / 100.0;

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
    UnknownMode(u8),
    Inner(E),
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum VectorType {
    Mag,
    Gyro,
    Accel,
    Euler,
    LinearAccel,
    Gravity,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CalibrationStatus {
    pub sys: u8,
    pub gyro: u8,
    pub accel: u8,
    pub mag: u8,
}

pub struct Bno055<T> {
    i2c: T,
    addr: u8,
    current_mode: OperatingMode,
    power_mode: PowerMode,
}

impl<T, E> Bno055<T>
where
    T: I2c<SevenBitAddress, Error = E>,
{
    pub fn new(i2c: T, alt_addr: bool) -> Self {
        let addr = if alt_addr { ADDR_1 } else { ADDR_0 };

        Self {
            i2c,
            addr,
            current_mode: OperatingMode::Config,
            power_mode: PowerMode::Suspend,
        }
    }

    fn read_u8(&mut self, reg: Registers) -> Result<u8, E> {
        let mut buf = [0_u8; 1];
        let write_reg = [reg.to_u8(); 1];

        self.i2c.write_read(self.addr, &write_reg, &mut buf)?;

        Ok(buf[0])
    }

    fn write_u8(&mut self, reg: Registers, val: u8) -> Result<(), E> {
        let buf = [reg.to_u8(), val];
        self.i2c.write(self.addr, &buf)
    }

    pub fn read_vec(&mut self, vec_start: VectorType) -> Result<Vector3, Error<E>> {
        let mut buf = [0_u8; 6];
        let (write_reg, conv_factor) = match vec_start {
            VectorType::Mag => ([Registers::MagXLsb.to_u8(); 1], MAG_CONVERT),
            VectorType::Gyro => ([Registers::GyroXLsb.to_u8(); 1], GYRO_CONVERT),
            VectorType::Accel => ([Registers::AccelXLsb.to_u8(); 1], ACCEL_CONVERT),
            VectorType::Euler => ([Registers::EulerHLsb.to_u8(); 1], EULER_CONVERT),
            VectorType::LinearAccel => {
                ([Registers::LinerAccelXLsb.to_u8(); 1], LINEAR_ACCEL_CONVERT)
            }
            VectorType::Gravity => ([Registers::GravityXLsb.to_u8(); 1], GRAVITY_CONVERT),
        };

        self.i2c
            .write_read(self.addr, &write_reg, &mut buf)
            .map_err(Error::Inner)?;
        let xi: i16 = (buf[0] as i16) | ((buf[1] as i16) << 8);
        let yi: i16 = (buf[2] as i16) | ((buf[3] as i16) << 8);
        let zi: i16 = (buf[4] as i16) | ((buf[5] as i16) << 8);

        Ok(Vector3 {
            x: (xi as f32) * conv_factor,
            y: (yi as f32) * conv_factor,
            z: (zi as f32) * conv_factor,
        })
    }

    fn set_op_mode(&mut self, new_mode: OperatingMode) -> Result<(), E> {
        self.current_mode = new_mode;
        self.write_u8(Registers::OperatingMode, new_mode.to_u8())
    }

    fn get_op_mode(&mut self) -> Result<OperatingMode, Error<E>> {
        let mode = self
            .read_u8(Registers::OperatingMode)
            .map_err(Error::Inner)?;

        OperatingMode::from_u8(mode).map_err(|e| Error::UnknownMode(e))
    }

    fn set_power_mode(&mut self, power_mode: PowerMode) -> Result<(), E> {
        self.power_mode = power_mode;
        self.write_u8(Registers::PowerMode, power_mode.to_u8())
    }

    fn set_page(&mut self, page: Page) -> Result<(), E> {
        if page == Page::Page0 {
            self.write_u8(Registers::PageSelect, 0)
        } else {
            self.write_u8(Registers::PageSelect, 1)
        }
    }

    pub fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
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

        // self.set_page(Page::Page0).map_err(Error::Inner)?;

        self.set_op_mode(OperatingMode::Config)
            .map_err(Error::Inner)?;

        self.write_u8(Registers::SystemTrigger, 0x20)
            .map_err(Error::Inner)?;
        delay.delay_ms(30);

        timeout = 850;
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
        delay.delay_ms(50);

        self.set_power_mode(PowerMode::Normal)
            .map_err(Error::Inner)?;
        delay.delay_ms(10);

        self.set_page(Page::Page0).map_err(Error::Inner)?;

        self.write_u8(Registers::SystemTrigger, 0)
            .map_err(Error::Inner)?;
        delay.delay_ms(20);

        self.set_op_mode(OperatingMode::Ndof)
            .map_err(Error::Inner)?;
        delay.delay_ms(20);

        Ok(())
    }

    pub fn set_external_crystal<D: DelayNs>(
        &mut self,
        delay: &mut D,
        use_crystal: bool,
    ) -> Result<(), Error<E>> {
        let original_mode = self.current_mode;
        self.set_op_mode(OperatingMode::Config)
            .map_err(Error::Inner)?;
        delay.delay_ms(25);

        self.set_page(Page::Page0).map_err(Error::Inner)?;

        if use_crystal {
            self.write_u8(Registers::SystemTrigger, 0x80)
                .map_err(Error::Inner)?;
        } else {
            self.write_u8(Registers::SystemTrigger, 0)
                .map_err(Error::Inner)?;
        }
        delay.delay_ms(10);

        self.set_op_mode(original_mode).map_err(Error::Inner)?;
        delay.delay_ms(20);

        Ok(())
    }

    pub fn get_calibration_status(&mut self) -> Result<CalibrationStatus, Error<E>> {
        let calib = self
            .read_u8(Registers::CalibrationStatus)
            .map_err(Error::Inner)?;

        Ok(CalibrationStatus {
            sys: (calib >> 6) & 0x03,
            gyro: (calib >> 4) & 0x03,
            accel: (calib >> 2) & 0x03,
            mag: calib & 0x03,
        })
    }

    pub fn get_mag(&mut self) -> Result<Vector3, Error<E>> {
        self.read_vec(VectorType::Mag)
    }

    pub fn get_gyro(&mut self) -> Result<Vector3, Error<E>> {
        self.read_vec(VectorType::Gyro)
    }

    pub fn get_accel(&mut self) -> Result<Vector3, Error<E>> {
        self.read_vec(VectorType::Accel)
    }

    pub fn get_whole_shebang(&mut self) -> Result<[Vector3; 3], Error<E>> {
        let mag = self.get_mag()?;
        let gyro = self.get_gyro()?;
        let accel = self.get_accel()?;

        Ok([mag, gyro, accel])
    }

    pub fn get_temp(&mut self) -> Result<i8, Error<E>> {
        let temp = self.read_u8(Registers::Temperature).map_err(Error::Inner)?;
        Ok(temp as i8)
    }
}
