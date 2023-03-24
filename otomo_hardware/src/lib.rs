#![no_std]

pub mod led;
pub mod pwm;
pub mod serial;
pub mod ultrasonic;

use led::{BlueLed, GreenLed, OrangeLed, RedLed};
use pwm::{Pwm3, Pwm4};
use serial::BluetoothSerialPort;
use ultrasonic::Ultrasonics;

pub struct OtomoHardware {
    pub green_led: GreenLed,
    pub orange_led: OrangeLed,
    pub red_led: RedLed,
    pub blue_led: BlueLed,

    pub bt_serial: BluetoothSerialPort,

    pub pwm3: Pwm3,
    pub pwm4: Pwm4,

    pub ultras: Ultrasonics,
}
