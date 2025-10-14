use super::lsm_regs::Registers;
use super::Vector3;

use embedded_hal::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
};

const DEFAULT_ADDR: u8 = 0x6A;

const GYRO_CONVERT: f32 = 1.0;
const ACCEL_CONVERT: f32 = 1.0;
const TEMP_CONVERT: f32 = 1.0;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum VectorType {
    Gyro,
    Accel,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error<E> {
    Timeout,
    SetupInvalidRead((u8, u8)),
    InvalidAddress(u8),
    UnknownMode(u8),
    Inner(E),
}

pub struct Lsm6dsox<T> {
    i2c: T,
}

impl<T, E> Lsm6dsox<T>
where
    T: I2c<SevenBitAddress, Error = E>,
{
    pub fn new(i2c: T) -> Self {
        Self { i2c }
    }

    pub fn setup<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
        let mut timeout_ms = 35 * 4; // Datasheet table 3, T_on is 35ms

        'outer: while timeout_ms > 0 {
            if let Ok(b) = self.read_u8(Registers::WhoAmI) {
                if b == 0x6C {
                    break 'outer;
                } else {
                    return Err(Error::InvalidAddress(b));
                }
            } else {
                delay.delay_ms(5);
                timeout_ms -= 5;
            }
        }

        if timeout_ms <= 0 {
            return Err(Error::Timeout);
        }

        let init_values = [
            (Registers::Ctrl1Accel, 0b01101100), // 416Hz ODR, +- 8g range, single-stage LPF
            (Registers::Ctrl8Accel, 0b00000000), // No HPF, Disable LPF2 stuff
            (Registers::Ctrl9Accel, 0b11100010), // Default settings but disable I3C
            (Registers::Ctrl2Gyro, 0b01101000), // 416Hz ODR, 1000dps, no UI chaining, 135.9 LPF2 cutoff
            // 0 is default for Ctrl4C, Ctrl6C, Ctrl7Gyro
            (Registers::Ctrl4C, 0b00000000), // Disable gyroscope LPF1, using LPF2
            (Registers::Ctrl6C, 0b00000000), // 136.6 gyro LPF1 bandwidth selection
            (Registers::Ctrl7Gryo, 0b00000000), // Disable HPF, Disable OIS
        ];

        for (register, write_val) in init_values {
            self.write_u8(register, write_val).map_err(Error::Inner)?;
            let read_back = self.read_u8(register).map_err(Error::Inner)?;

            if write_val != read_back {
                return Err(Error::SetupInvalidRead((write_val, read_back)));
            }
        }

        Ok(())
    }

    fn read_u8(&mut self, reg: Registers) -> Result<u8, E> {
        let mut buf = [0_u8; 1];
        let write_reg = [reg.into(); 1];

        self.i2c.write_read(DEFAULT_ADDR, &write_reg, &mut buf)?;

        Ok(buf[0])
    }

    fn write_u8(&mut self, reg: Registers, val: u8) -> Result<(), E> {
        let buf = [reg.into(), val];
        self.i2c.write(DEFAULT_ADDR, &buf)
    }

    fn read_vec(&mut self, vec_start: VectorType) -> Result<Vector3, Error<E>> {
        let mut buf = [0_u8; 6];
        let (write_reg, conv_factor) = match vec_start {
            VectorType::Accel => ([Registers::AccelXLo.into()], ACCEL_CONVERT),
            VectorType::Gyro => ([Registers::GyroXLo.into()], GYRO_CONVERT),
        };

        self.i2c
            .write_read(DEFAULT_ADDR, &write_reg, &mut buf)
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

    pub fn get_gyro(&mut self) -> Result<Vector3, Error<E>> {
        self.read_vec(VectorType::Gyro)
    }

    pub fn get_accel(&mut self) -> Result<Vector3, Error<E>> {
        self.read_vec(VectorType::Accel)
    }

    pub fn get_temp(&mut self) -> Result<f32, Error<E>> {
        let temp_lo = self.read_u8(Registers::TempLo).map_err(Error::Inner)?;
        let temp_hi = self.read_u8(Registers::TempHi).map_err(Error::Inner)?;

        let temp: u16 = ((temp_hi as u16) << 8) | (temp_lo as u16);
        let temp = temp as i16;
        Ok((temp as f32) * TEMP_CONVERT)
    }
}
