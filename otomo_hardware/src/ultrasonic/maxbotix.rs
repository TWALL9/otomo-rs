use stm32f4xx_hal::i2c::{Error, I2c, Instance};

const SENSOR_DEFAULT_ADDR: u8 = 224;
const READ_COMMAND: u8 = 81;
const UNLOCK_COMMAND_0: u8 = 170;
const UNLOCK_COMMAND_1: u8 = 165;

/// Maxbotix MB704-XYY series ultrasonic sensors
/// I found a couple of these in the work e-waste bin...
pub struct MaxBotix<T: Instance> {
    i2c: I2c<T>,
    addr: u8,
}

impl<T: Instance> MaxBotix<T> {
    /// Realistically, the sensors on this bus should already have different addresses
    pub fn new(i2c: I2c<T>, addr: Option<u8>) -> Self {
        Self {
            i2c,
            addr: addr.unwrap_or(SENSOR_DEFAULT_ADDR),
        }
    }

    // I do not know what happens if this process fails, so let's assume the sensor is dead.
    pub fn change_address(self, new_addr: u8) -> Result<Self, Error> {
        let mut temp = self;
        temp.i2c
            .write(temp.addr, &[UNLOCK_COMMAND_0, UNLOCK_COMMAND_1, new_addr])?;

        Ok(Self {
            i2c: temp.i2c,
            addr: new_addr,
        })
    }

    pub fn start_read(&mut self) -> Result<(), Error> {
        self.i2c.write(self.addr, &[READ_COMMAND])
    }

    /// Check datasheet (Range Cycle Interrupt)
    /// NACK will be sent if distance is read before it is calculated,
    /// but will sometimes have a valid value, the previous value, or if no value was ever
    /// determined, the sensor will return alternating 0 and 255
    pub fn read_distance(&mut self) -> Result<f32, Error> {
        let mut buf = [0_u8; 2];
        self.i2c.read(self.addr, &mut buf)?;

        let dist_cm = ((buf[0] as u16) << 8) | buf[1] as u16;
        let dist_m = dist_cm as f32 / 100.0;
        Ok(dist_m)
    }
}
