use num_traits::float::FloatCore;

use core::f32::consts::PI;
use otomo_hardware::motors::MotorEffort;

const MAX_SPEED_RPM: f32 = 100_f32; // based on no-load speed of https://www.pololu.com/product/4755
const MAX_SPEED_RAD_S: f32 = (MAX_SPEED_RPM * PI * 2.0) / 60.0; // 10.47 rad/s
const ACTUAL_MAX_SPEED_RAD_S: f32 = 17.83;

// Dumb function that assumes linear relationship between rpm and duty cycle
pub fn rad_s_to_duty(rad_s: f32) -> MotorEffort {
    let mut speed = rad_s.clamp(-ACTUAL_MAX_SPEED_RAD_S, ACTUAL_MAX_SPEED_RAD_S);
    speed /= ACTUAL_MAX_SPEED_RAD_S;

    if speed > 0.0 {
        MotorEffort::Forward(speed)
    } else if speed < 0.0 {
        MotorEffort::Backward(speed.abs())
    } else {
        MotorEffort::Release
    }
}
