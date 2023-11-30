pub mod encoder;
pub mod hbridge;
pub mod pololu_driver;

use fugit::TimerInstantU64;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
#[allow(dead_code)]
pub enum MotorEffort {
    Forward(f32), // Measured as 0..=1 in terms of effort
    Backward(f32),
    Brake,
    #[default]
    Release,
}

pub trait OpenLoopDrive {
    fn drive(&mut self, direction: MotorEffort);
    fn current_direction(&self) -> MotorEffort;
}

pub trait Encoder {
    fn get_velocity(&mut self, now: TimerInstantU64<1_000_000>) -> f32;
    fn get_position(&mut self) -> f32;
}
