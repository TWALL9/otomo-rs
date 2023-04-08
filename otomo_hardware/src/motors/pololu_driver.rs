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
> {
    bridge: HBridge<P1, P2, C, D>,
    enable: Pin<EP, EN, Output<PushPull>>,
    diag: Pin<DP, DN, Input>,
    // current sense
    // encoders?
}

pub type LeftDrive = MotorDriver<TIM3, TIM3, 0, 1, 'A', 10, 'A', 13>;
pub type RightDrive = MotorDriver<TIM3, TIM3, 2, 3, 'D', 9, 'D', 10>;

impl<
        P1: PwmExt,
        P2: PwmExt,
        const C: u8,
        const D: u8,
        const EP: char,
        const EN: u8,
        const DP: char,
        const DN: u8,
    > MotorDriver<P1, P2, C, D, EP, EN, DP, DN>
{
    pub fn new(
        input_1: PwmChannel<P1, C>,
        input_2: PwmChannel<P2, D>,
        enable: Pin<EP, EN, Output<PushPull>>,
        diag: Pin<DP, DN, Input>,
    ) -> Self {
        Self {
            bridge: HBridge::new(input_1, input_2),
            enable,
            diag,
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
    > OpenLoopDrive for MotorDriver<P1, P2, C, D, EP, EN, DP, DN>
{
    fn drive(&mut self, direction: MotorEffort) {
        self.bridge.drive(direction)
    }

    fn current_direction(&self) -> MotorEffort {
        self.bridge.current_direction()
    }
}
