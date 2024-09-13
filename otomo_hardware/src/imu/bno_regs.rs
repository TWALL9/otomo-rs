#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Page {
    Page0,
    Page1,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Registers {
    /// ID's
    ChipId = 0x00,
    AccelRevId = 0x01,
    MagRevId = 0x02,
    GyroRevId = 0x03,
    SwRevIdLsb = 0x04,
    SwRevIdMsb = 0x05,
    BlRevId = 0x06,
    PageSelect = 0x07,

    /// Accel
    AccelXLsb = 0x08,
    AccelXMsb = 0x09,
    AccelYLsb = 0x0A,
    AccelYMsb = 0x0B,
    AccelZLsb = 0x0C,
    AccelZMsb = 0x0D,

    /// Mag
    MagXLsb = 0x0E,
    MagXMsb = 0x0F,
    MagYLsb = 0x10,
    MagYMsb = 0x11,
    MagZLsb = 0x12,
    MagZMsb = 0x13,

    /// Gyro
    GyroXLsb = 0x14,
    GyroXMsb = 0x15,
    GyroYLsb = 0x16,
    GyroYMsb = 0x17,
    GyroZLsb = 0x18,
    GyroZMsb = 0x19,

    /// Euler
    EulerHLsb = 0x1A,
    EulerHMsb = 0x1B,
    EulerRLsb = 0x1C,
    EulerRMsb = 0x1D,
    EulerPLsb = 0x1E,
    EulerPMsb = 0x1F,

    /// Quaternion
    QuaternionWLsb = 0x20,
    QuaternionWMsb = 0x21,
    QuaternionXLsb = 0x22,
    QuaternionXMsb = 0x23,
    QuaternionYLsb = 0x24,
    QuaternionYMsb = 0x25,
    QuaternionZLsb = 0x26,
    QuaternionZMsb = 0x27,

    /// Linear acceleration
    LinerAccelXLsb = 0x28,
    LinerAccelXMsb = 0x29,
    LinerAccelYLsb = 0x2A,
    LinerAccelYMsb = 0x2B,
    LinerAccelZLsb = 0x2C,
    LinerAccelZMsb = 0x2D,

    /// Gravity
    GravityXLsb = 0x2E,
    GravityXMsb = 0x2F,
    GravityYLsb = 0x30,
    GravityYMsb = 0x31,
    GravityZLsb = 0x32,
    GravityZMsb = 0x33,

    /// Temperature
    Temperature = 0x34,

    /// Status
    CalibrationStatus = 0x35,
    SelftestResult = 0x36,
    IntrStatus = 0x37,
    SystemClockStatus = 0x38,
    SystemStatus = 0x39,
    SystemError = 0x3A,

    /// Unit selection
    UnitSelection = 0x3B,

    /// Mode
    OperatingMode = 0x3D,
    PowerMode = 0x3E,

    SystemTrigger = 0x3F,
    TempSource = 0x40,

    /// Axis mapping
    AxisMapConfig = 0x41,
    AxisMapSign = 0x42,

    /// SIC
    SicMatrix0Lsb = 0x43,
    SicMatrix0Msb = 0x44,
    SicMatrix1Lsb = 0x45,
    SicMatrix1Msb = 0x46,
    SicMatrix2Lsb = 0x47,
    SicMatrix2Msb = 0x48,
    SicMatrix3Lsb = 0x49,
    SicMatrix3Msb = 0x4A,
    SicMatrix4Lsb = 0x4B,
    SicMatrix4Msb = 0x4C,
    SicMatrix5Lsb = 0x4D,
    SicMatrix5Msb = 0x4E,
    SicMatrix6Lsb = 0x4F,
    SicMatrix6Msb = 0x50,
    SicMatrix7Lsb = 0x51,
    SicMatrix7Msb = 0x52,
    SicMatrix8Lsb = 0x53,
    SicMatrix8Msb = 0x54,

    /// Accel offset
    AccelOffsetXLsb = 0x55,
    AccelOffsetXMsb = 0x56,
    AccelOffsetYLsb = 0x57,
    AccelOffsetYMsb = 0x58,
    AccelOffsetZLsb = 0x59,
    AccelOffsetZMsb = 0x5A,

    /// Mag offset
    MagOffsetXLsb = 0x5B,
    MagOffsetXMsb = 0x5C,
    MagOffsetYLsb = 0x5D,
    MagOffsetYMsb = 0x5E,
    MagOffsetZLsb = 0x5F,
    MagOffsetZMsb = 0x60,

    /// Gyro offset
    GyroOffsetXLsb = 0x61,
    GyroOffsetXMsb = 0x62,
    GyroOffsetYLsb = 0x63,
    GyroOffsetYMsb = 0x64,
    GyroOffsetZLsb = 0x65,
    GyroOffsetZMsb = 0x66,

    /// Radius
    AccelRadiusLsb = 0x67,
    AccelRadiusMsb = 0x68,
    MagRadiusLsb = 0x69,
    MagRadiusMsb = 0x6A,
}

impl Registers {
    pub fn to_u8(&self) -> u8 {
        *self as u8
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub(crate) enum OperatingMode {
    Config = 0x00,
    AccelOnly = 0x01,
    MagOnly = 0x02,
    GyroOnly = 0x03,
    AccMag = 0x04,
    AccGyro = 0x05,
    MagGyro = 0x06,
    AccMagGyro = 0x07,
    ImuPlus = 0x08,
    Compass = 0x09,
    Mag4Gyro = 0x0A,
    NdofFmcOff = 0x0B,
    Ndof = 0x0C,
}

impl OperatingMode {
    pub fn to_u8(&self) -> u8 {
        *self as u8
    }

    pub fn from_u8(op: u8) -> Result<Self, u8> {
        match op {
            0x00 => Ok(OperatingMode::Config),
            0x01 => Ok(OperatingMode::AccelOnly),
            0x02 => Ok(OperatingMode::MagOnly),
            0x03 => Ok(OperatingMode::GyroOnly),
            0x04 => Ok(OperatingMode::AccMag),
            0x05 => Ok(OperatingMode::AccGyro),
            0x06 => Ok(OperatingMode::MagGyro),
            0x07 => Ok(OperatingMode::AccMagGyro),
            0x08 => Ok(OperatingMode::ImuPlus),
            0x09 => Ok(OperatingMode::Compass),
            0x0A => Ok(OperatingMode::Mag4Gyro),
            0x0B => Ok(OperatingMode::NdofFmcOff),
            0x0C => Ok(OperatingMode::Ndof),
            e => Err(e),
        }
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub(crate) enum PowerMode {
    Normal = 0x00,
    Low = 0x01,
    Suspend = 0x02,
}

impl PowerMode {
    pub fn to_u8(&self) -> u8 {
        *self as u8
    }
}
