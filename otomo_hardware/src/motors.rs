pub mod encoder;
pub mod hbridge;
pub mod pololu_driver;

use fugit::TimerInstantU32;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
#[allow(dead_code)]
pub enum MotorEffort {
    Forward(f32), // Measured as 0..=1 in terms of effort
    Backward(f32),
    Brake,
    #[default]
    Release,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum MotorOdometry {
    #[default]
    Stationary,
    Moving(f32), // measured in rad/ms
}

pub trait OpenLoopDrive {
    fn drive(&mut self, direction: MotorEffort);
    fn current_direction(&self) -> MotorEffort;
}

pub trait Encoder {
    fn get_velocity(&mut self, now: TimerInstantU32<1_000_000>) -> Option<MotorOdometry>;
    fn get_position(&mut self) -> f32;
}
