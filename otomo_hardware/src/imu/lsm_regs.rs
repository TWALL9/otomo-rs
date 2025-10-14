#[repr(u8)]
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Registers {
    /// Configuration
    CfgAccess = 0x01,
    PinCtrl = 0x02,
    WhoAmI = 0x0F,
    Ctrl1Accel = 0x10,
    Ctrl2Gyro = 0x11,
    Ctrl3C = 0x12,
    Ctrl4C = 0x13,
    Ctrl5C = 0x14,
    Ctrl6C = 0x15,
    Ctrl7Gryo = 0x16,
    Ctrl8Accel = 0x17,
    Ctrl9Accel = 0x18,
    Ctrl10C = 0x19,

    /// Status
    Status = 0x1E,

    /// Reading data
    /// Temperature
    TempLo = 0x20,
    TempHi = 0x21,

    /// Gyro
    GyroXLo = 0x22,
    GyroXHi = 0x23,
    GyroYLo = 0x24,
    GyroYHi = 0x25,
    GyroZLo = 0x26,
    GyroZHi = 0x27,

    /// Accel
    AccelXLo = 0x28,
    AccelXHi = 0x29,
    AccelYLo = 0x2A,
    AccelYHi = 0x2B,
    AccelZLo = 0x2C,
    AccelZHi = 0x2D,

    /// Calibration
    AccelXOffset = 0x73,
    AccelYOffset = 0x74,
    AccelZOffset = 0x75,
}

impl Into<u8> for Registers {
    fn into(self) -> u8 {
        self as u8
    }
}
