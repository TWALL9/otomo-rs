use stm32f4xx_hal::{
    gpio::ReadPin,
    i2c::{Error as I2cError, I2c, Instance},
};

const SENSOR_DEFAULT_ADDR: u8 = 112;
const READ_COMMAND: u8 = 81;
const UNLOCK_COMMAND_0: u8 = 170;
const UNLOCK_COMMAND_1: u8 = 165;

// Per datasheet, odd values are also bad.
const INVALID_ADDRESSES: [u8; 4] = [0, 80, 164, 170];

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error {
    Busy,
    InvalidAddress(u8),
    Inner(I2cError),
}

impl Into<Error> for I2cError {
    fn into(self) -> Error {
        Error::Inner(self)
    }
}

fn check_addr(addr: u8) -> bool {
    INVALID_ADDRESSES.contains(&addr) || addr % 2 != 0 || addr > 0x7F
}

/// Maxbotix MB704-XYY series ultrasonic sensors
/// I found a couple of these in the work e-waste bin...
pub struct MaxBotix<T: Instance, U: ReadPin> {
    i2c: I2c<T>,
    status_pin: U,
    addr: u8,
}

impl<T: Instance, U: ReadPin> MaxBotix<T, U> {
    /// Realistically, the sensors on this bus should already have different addresses
    pub fn new(i2c: I2c<T>, status_pin: U, addr: Option<u8>) -> Result<Self, Error> {
        let actual_addr = addr.unwrap_or(SENSOR_DEFAULT_ADDR);

        if !check_addr(actual_addr) {
            Err(Error::InvalidAddress(actual_addr))
        } else {
            Ok(Self {
                i2c,
                status_pin,
                addr: actual_addr,
            })
        }
    }

    // I do not know what happens if this process fails, so let's assume the sensor is dead.
    pub fn change_address(self, new_addr: u8) -> Result<Self, Error> {
        if !check_addr(new_addr) {
            return Err(Error::InvalidAddress(new_addr));
        }

        // If in the middle of reading a value, wait
        while self.status_pin.is_high() {}

        let mut temp = self;
        temp.i2c
            .write(temp.addr, &[UNLOCK_COMMAND_0, UNLOCK_COMMAND_1, new_addr])
            .map_err(|e| e.into())?;

        Ok(Self {
            i2c: temp.i2c,
            status_pin: temp.status_pin,
            addr: new_addr,
        })
    }

    pub fn start_read(&mut self) -> Result<(), Error> {
        if self.status_pin.is_low() {
            self.i2c
                .write(self.addr, &[READ_COMMAND])
                .map_err(|e| e.into())
        } else {
            return Err(Error::Busy);
        }
    }

    /// Check datasheet (Range Cycle Interrupt)
    /// NACK will be sent if distance is read before it is calculated,
    /// but will sometimes have a valid value, the previous value, or if no value was ever
    /// determined, the sensor will return alternating 0 and 255
    pub fn read_distance(&mut self) -> Result<f32, Error> {
        let mut buf = [0_u8; 2];

        if self.status_pin.is_low() {
            self.i2c.read(self.addr, &mut buf).map_err(|e| e.into())?;

            let dist_cm = ((buf[0] as u16) << 8) | buf[1] as u16;
            let dist_m = dist_cm as f32 / 100.0;
            Ok(dist_m)
        } else {
            Err(Error::Busy)
        }
    }
}
