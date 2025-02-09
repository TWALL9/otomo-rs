use otomo_hardware::imu::{
    bno055::{Bno055, Error as BnoError},
    Vector3,
};
use stm32f4xx_hal::i2c::{Error as I2cError, I2c1};

const CALIB_COUNT: f32 = 100.0;

#[derive(Clone, Copy, PartialEq, Eq)]
enum ImuCalibrationState {
    Calibrating,
    Operational,
    Error,
}

pub struct ImuDriver<'a> {
    calib_count: u8,
    imu: &'a mut Bno055<I2c1>,
    state: ImuCalibrationState,
    gyro_offset: Vector3,
    accel_offset: Vector3,
    gyro: Vector3,
    accel: Vector3,
}

impl<'a> ImuDriver<'a> {
    pub fn new(imu: &'a mut Bno055<I2c1>) -> Self {
        Self {
            calib_count: 0,
            imu,
            state: ImuCalibrationState::Calibrating,
            gyro_offset: Vector3::default(),
            accel_offset: Vector3::default(),
            gyro: Vector3::default(),
            accel: Vector3::default(),
        }
    }

    pub fn get_state(&self) -> ImuCalibrationState {
        self.state
    }

    pub fn update(&mut self) -> Result<(), BnoError<I2cError>> {
        let current_state = self.state;
        match current_state {
            ImuCalibrationState::Calibrating => {
                let (gyro, accel) = self.get_data_raw()?;
                self.gyro_offset.x += gyro.x / CALIB_COUNT;
                self.gyro_offset.y += gyro.y / CALIB_COUNT;
                self.gyro_offset.z += gyro.z / CALIB_COUNT;

                self.accel_offset.x += accel.x / CALIB_COUNT;
                self.accel_offset.y += accel.y / CALIB_COUNT;
                self.accel_offset.z += accel.z / CALIB_COUNT;

                if self.calib_count >= CALIB_COUNT as u8 {
                    log::info!(
                        "Calib done: {:?}, {:?}",
                        self.gyro_offset,
                        self.accel_offset
                    );
                    self.state = ImuCalibrationState::Operational;
                } else {
                    self.calib_count += 1;
                }
            }
            ImuCalibrationState::Operational => match self.get_data_raw() {
                Ok((g, a)) => {
                    self.gyro = g - self.gyro_offset;
                    // self.accel = a - self.accel_offset;
                    self.accel = a;
                }
                Err(e) => {
                    self.state = ImuCalibrationState::Error;
                    return Err(e);
                }
            },
            ImuCalibrationState::Error => self.calib_count = 0,
        };

        Ok(())
    }

    fn get_data_raw(&mut self) -> Result<(Vector3, Vector3), BnoError<I2cError>> {
        let gyro = self.imu.get_gyro()?;
        let accel = self.imu.get_accel()?;

        Ok((gyro, accel))
    }

    pub fn get_data(&self) -> Option<(Vector3, Vector3)> {
        if self.state == ImuCalibrationState::Operational {
            Some((self.gyro, self.accel))
        } else {
            None
        }
    }
}
