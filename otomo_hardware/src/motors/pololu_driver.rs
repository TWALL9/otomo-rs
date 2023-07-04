// Driver for the TB9051FTG Single Brushed DC Motor Driver

use crate::motors::{hbridge::HBridge, MotorEffort, OpenLoopDrive};

use stm32f4xx_hal::{
    gpio::{Input, Output, Pin, PushPull},
    pac::TIM3,
    timer::{pwm::PwmExt, PwmChannel},
};

pub struct MotorDriver<
    P1: PwmExt,
    P2: PwmExt,
    const C: u8,
    const D: u8,
    const EP: char,
    const EN: u8,
    const DP: char,
    const DN: u8,
    const OP: char,
    const ON: u8,
> {
    bridge: HBridge<P1, P2, C, D>,
    enable: Pin<EP, EN, Output<PushPull>>,
    diag: Pin<DP, DN, Input>,
    overcurrent: Pin<OP, ON, Output<PushPull>>,
    // current sense
    // encoders?
}

pub type LeftDrive = MotorDriver<TIM3, TIM3, 0, 3, 'B', 14, 'B', 13, 'B', 15>;
pub type RightDrive = MotorDriver<TIM3, TIM3, 1, 2, 'B', 3, 'D', 6, 'B', 4>;

impl<
        P1: PwmExt,
        P2: PwmExt,
        const C: u8,
        const D: u8,
        const EP: char,
        const EN: u8,
        const DP: char,
        const DN: u8,
        const OP: char,
        const ON: u8,
    > MotorDriver<P1, P2, C, D, EP, EN, DP, DN, OP, ON>
{
    pub fn new(
        input_1: PwmChannel<P1, C>,
        input_2: PwmChannel<P2, D>,
        enable: Pin<EP, EN, Output<PushPull>>,
        diag: Pin<DP, DN, Input>,
        overcurrent: Pin<OP, ON, Output<PushPull>>,
    ) -> Self {
        Self {
            bridge: HBridge::new(input_1, input_2),
            enable,
            diag,
            overcurrent,
        }
    }

    pub fn set_enable(&mut self, enabled: bool) {
        if enabled {
            self.enable.set_high();
        } else {
            self.enable.set_low();
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.enable.is_set_high()
    }

    pub fn set_overcurrent(&mut self, enabled: bool) {
        if enabled {
            self.overcurrent.set_high();
        } else {
            self.overcurrent.set_low();
        }
    }

    pub fn is_in_error(&self) -> bool {
        self.diag.is_low()
    }
}

impl<
        P1: PwmExt,
        P2: PwmExt,
        const C: u8,
        const D: u8,
        const EP: char,
        const EN: u8,
        const DP: char,
        const DN: u8,
        const OP: char,
        const ON: u8,
    > OpenLoopDrive for MotorDriver<P1, P2, C, D, EP, EN, DP, DN, OP, ON>
{
    fn drive(&mut self, direction: MotorEffort) {
        self.bridge.drive(direction)
    }

    fn current_direction(&self) -> MotorEffort {
        self.bridge.current_direction()
    }
}
