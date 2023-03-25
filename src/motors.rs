// #[derive(Debug, Clone, Copy, Format, Default, PartialEq)]
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

/// Returns left/right motor inputs based on a unit-circle joystick.
/// [based on this example's drive output graph]: https://robotics.stackexchange.com/questions/2011/how-to-calculate-the-right-and-left-speed-for-a-tank-like-rover
/// Uses a very basic linear equation solver to plot drive power
/// The equations are calibrated on a 50% throttle input, so when turning the speed can be more
/// than 100%.  This function saturates the output speed to prevent that.
/// * `speed` - Percentage of radians in a unit circle describing the joystick
/// * `deg_heading` - Heading in degrees, with 0 as north.
pub fn joystick_tank_controls(speed: f32, deg_heading: f32) -> (MotorDirection, MotorDirection) {
    if speed <= 0.0 {
        return (MotorDirection::Release, MotorDirection::Release);
    }

    let r = speed.clamp(0.0, 100.0) / 100.0;
    let theta = -1.0 * (((deg_heading + 180.0) % 360.0) - 180.0);

    let left_raw = throttle_solver_l(r, theta);
    let right_raw = throttle_solver_r(r, theta);

    // info!("l_raw: {}, r_raw: {}", left_raw, right_raw);

    let left = if left_raw.is_sign_positive() {
        MotorDirection::Forward(left_raw.clamp(0.0, 1.0))
    } else {
        let abs_l = left_raw * -1.0;
        MotorDirection::Backward(abs_l.clamp(0.0, 1.0))
    };

    let right = if right_raw.is_sign_positive() {
        MotorDirection::Forward(right_raw.clamp(0.0, 1.0))
    } else {
        let abs_r = right_raw * -1.0;
        MotorDirection::Backward(abs_r.clamp(0.0, 1.0))
    };

    (left, right)
}

fn throttle_solver_l(throttle: f32, theta: f32) -> f32 {
    let (x_pos, y_intercept) = match theta {
        t if (-135.0..=45.0).contains(&t) => (t, throttle),
        t if (-180.0..-135.0).contains(&t) => (180.0 + t, throttle * -1.0),
        t if (45.0..=180.0).contains(&t) => (-180.0 + t, throttle * -1.0),
        _ => return 0.0,
    };

    let slope = y_intercept / 45.0;

    slope * x_pos + y_intercept
}

fn throttle_solver_r(throttle: f32, theta: f32) -> f32 {
    let (x_pos, y_intercept) = match theta {
        t if (-45.0..=135.0).contains(&t) => (t, throttle),
        t if (135.0..=180.0).contains(&t) => (-180.0 + t, throttle * -1.0),
        t if (-180.0..-45.0).contains(&t) => (180.0 + t, throttle * -1.0),
        _ => return 0.0,
    };

    let slope = -y_intercept / 45.0;

    slope * x_pos + y_intercept
}
