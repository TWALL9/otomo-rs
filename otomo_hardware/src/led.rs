#![allow(dead_code)]

use stm32f4xx_hal::gpio::{Output, PushPull, PD12, PD13, PD14, PD15};

pub type GreenLed = PD12<Output<PushPull>>;
pub type OrangeLed = PD13<Output<PushPull>>;
pub type RedLed = PD14<Output<PushPull>>;
pub type BlueLed = PD15<Output<PushPull>>;

// TODO figure out the mess of generics that comprise an LED trait

trait Led {
    fn turn_on(&mut self);
    fn turn_off(&mut self);
    fn toggle(&mut self);
}

impl Led for GreenLed {
    fn turn_on(&mut self) {
        self.set_high();
    }

    fn turn_off(&mut self) {
        self.set_low();
    }

    fn toggle(&mut self) {
        self.toggle();
    }
}
impl Led for OrangeLed {
    fn turn_on(&mut self) {
        self.set_high();
    }

    fn turn_off(&mut self) {
        self.set_low();
    }

    fn toggle(&mut self) {
        self.toggle();
    }
}
impl Led for RedLed {
    fn turn_on(&mut self) {
        self.set_high();
    }

    fn turn_off(&mut self) {
        self.set_low();
    }

    fn toggle(&mut self) {
        self.toggle();
    }
}
impl Led for BlueLed {
    fn turn_on(&mut self) {
        self.set_high();
    }

    fn turn_off(&mut self) {
        self.set_low();
    }

    fn toggle(&mut self) {
        self.toggle();
    }
}
