use crate::motors::{MotorEffort, OpenLoopDrive};
use stm32f4xx_hal::timer::{pwm::PwmExt, PwmChannel};

pub struct HBridge<P1: PwmExt, P2: PwmExt, const C: u8, const D: u8> {
    input_1: PwmChannel<P1, C>,
    input_2: PwmChannel<P2, D>,
    direction: MotorEffort,
}

impl<P1: PwmExt, P2: PwmExt, const C: u8, const D: u8> HBridge<P1, P2, C, D> {
    pub fn new(input_1: PwmChannel<P1, C>, input_2: PwmChannel<P2, D>) -> Self {
        let mut shield = Self {
            input_1,
            input_2,
            direction: MotorEffort::Release,
        };

        // set an initial state
        shield.input_1.enable();
        shield.input_2.enable();
        shield.drive(MotorEffort::Release);

        shield
    }
}

impl<P1: PwmExt, P2: PwmExt, const C: u8, const D: u8> OpenLoopDrive for HBridge<P1, P2, C, D> {
    fn drive(&mut self, direction: MotorEffort) {
        let max_1 = self.input_1.get_max_duty();
        let max_2 = self.input_2.get_max_duty();
        let (a_duty, b_duty) = match direction {
            MotorEffort::Forward(d) => {
                let duty_ratio = d.clamp(0.0, 1.0);
                ((max_1 as f32 * duty_ratio) as u16, 0)
            }
            MotorEffort::Backward(d) => {
                let duty_ratio = d.clamp(0.0, 1.0);
                (0, (max_2 as f32 * duty_ratio) as u16)
            }
            MotorEffort::Brake => (max_1, max_2),
            MotorEffort::Release => (0, 0),
        };

        self.direction = direction;
        self.input_1.set_duty(a_duty);
        self.input_2.set_duty(b_duty);
    }

    fn current_direction(&self) -> MotorEffort {
        self.direction
    }
}
