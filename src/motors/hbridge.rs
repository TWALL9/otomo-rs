use super::MotorDirection;

use stm32f4xx_hal::timer::{pwm::PwmExt, PwmChannel};

pub struct HBridge<P1: PwmExt, P2: PwmExt, const C: u8, const D: u8> {
    input_1: PwmChannel<P1, C>,
    input_2: PwmChannel<P2, D>,
}

impl<P1: PwmExt, P2: PwmExt, const C: u8, const D: u8> HBridge<P1, P2, C, D> {
    pub fn new(input_1: PwmChannel<P1, C>, input_2: PwmChannel<P2, D>) -> Self {
        let mut shield = Self { input_1, input_2 };

        // set an initial state
        shield.input_1.enable();
        shield.input_2.enable();
        shield.run(MotorDirection::Release);

        shield
    }

    pub fn run(&mut self, direction: MotorDirection) {
        let max_1 = self.input_1.get_max_duty();
        let max_2 = self.input_2.get_max_duty();
        let (a_duty, b_duty) = match direction {
            MotorDirection::Forward(d) => {
                let duty_ratio = d.clamp(0.0, 1.0);
                ((max_1 as f32 * duty_ratio) as u16, 0)
            }
            MotorDirection::Backward(d) => {
                let duty_ratio = d.clamp(0.0, 1.0);
                (0, (max_2 as f32 * duty_ratio) as u16)
            }
            MotorDirection::Brake => (max_1, max_2),
            MotorDirection::Release => (0, 0),
        };

        self.input_1.set_duty(a_duty);
        self.input_2.set_duty(b_duty);
    }
}
