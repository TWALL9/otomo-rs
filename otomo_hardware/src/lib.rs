#![no_std]

use stm32f4xx_hal::{
    pac::{TIM2, TIM3, TIM4, TIM9, USART2},
    prelude::*,
    serial::{Rx, Serial, Tx},
    timer::{Counter, MonoTimerUs, SysDelay, Timer3, Timer4},
};

pub mod led;
pub mod serial;
pub mod pwm;

use led::{GreenLed, OrangeLed, RedLed, BlueLed};
use serial::BluetoothSerialPort;
use pwm::{Pwm3, Pwm4};

pub struct OtomoHardware {
    pub green_led: GreenLed,
    pub orange_led: OrangeLed,
    pub red_led: RedLed,
    pub blue_led: BlueLed,
    pub bt_serial: BluetoothSerialPort,
    pwm3: Pwm3,
    pwm4: Pwm4,
}
