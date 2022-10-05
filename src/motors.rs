use defmt::Format;

#[derive(Debug, Clone, Copy, Format, Default, PartialEq)]
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
}

/// Returns left/right motor inputs based on a unit-circle joystick.
/// [based on this example]: https://robotics.stackexchange.com/questions/2011/how-to-calculate-the-right-and-left-speed-for-a-tank-like-rover
/// * `speed` - Percentage of radians in a unit circle describing the joystick
/// * `deg_heading` - Heading in degrees, with 0 as north.
pub fn joystick_tank_controls(speed: f32, deg_heading: f32) -> (MotorDirection, MotorDirection) {
    if speed <= 0.0 {
        return (MotorDirection::Release, MotorDirection::Release);
    }

    let normalized_heading = ((deg_heading + 180.0) % 360.0) - 180.0;
    let r = speed.clamp(0.0, 100.0);
    let mut right_falloff = r * (45.0 - normalized_heading % 90.0) / 45.0;
    let mut left_compensation = 100.0_f32
        .min(2.0 * r + right_falloff)
        .min(2.0 * r - right_falloff);

    right_falloff /= 100.0;
    left_compensation /= 100.0;

    match normalized_heading {
        t if t < -90.0 => (
            MotorDirection::Backward(left_compensation),
            MotorDirection::Backward(right_falloff),
        ),
        t if t < 0.0 && t >= -90.0 => (
            MotorDirection::Forward(left_compensation),
            MotorDirection::Backward(right_falloff),
        ),
        t if t < 90.0 && t >= 0.0 => (
            MotorDirection::Forward(left_compensation),
            MotorDirection::Forward(right_falloff),
        ),
        _ => (
            MotorDirection::Backward(left_compensation),
            MotorDirection::Forward(right_falloff),
        ),
    }
}
