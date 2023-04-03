pub mod hbridge;
pub mod pololu_driver;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
#[allow(dead_code)]
pub enum MotorDirection {
    Forward(f32),
    Backward(f32),
    Brake,
    #[default]
    Release,
}

pub trait OpenLoopDrive {
    fn drive(&mut self, direction: MotorDirection);
    fn current_direction(&self) -> MotorDirection;
}
