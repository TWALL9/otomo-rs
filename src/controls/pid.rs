#![allow(dead_code)]
use num_traits::Signed;

pub trait Number: PartialOrd + Signed + Copy + Default {}

impl<T: PartialOrd + Signed + Copy + Default> Number for T {}

#[derive(Default)]
pub struct PidCreator<T: Number> {
    kp: T,
    ki: T,
    kd: T,
}

impl<T: Number> PidCreator<T> {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn set_p(self, p: impl Into<T>) -> Self {
        let mut s = self;
        s.kp = p.into();
        s
    }

    pub fn set_i(self, i: impl Into<T>) -> Self {
        let mut s = self;
        s.ki = i.into();
        s
    }

    pub fn set_d(self, d: impl Into<T>) -> Self {
        let mut s = self;
        s.kd = d.into();
        s
    }

    pub fn create_controller(self) -> PidController<T> {
        PidController {
            kp: self.kp,
            ki: self.ki,
            kd: self.kd,
            setpoint: T::zero(),
            setpoint_limit: None,
            prev_measurement: None,
            prev_integral: T::zero(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PidController<T: Number> {
    kp: T,
    ki: T,
    kd: T,
    setpoint: T,
    setpoint_limit: Option<T>,
    prev_measurement: Option<T>,
    prev_integral: T,
}

impl<T: Number> PidController<T> {
    pub fn set_setpoint(&mut self, new_setpoint: impl Into<T>, limit: Option<impl Into<T>>) {
        self.setpoint = new_setpoint.into();
        self.setpoint_limit = limit.map(|l| l.into());
    }

    pub fn update(&mut self, measurement: T) -> T {
        let error = self.setpoint - measurement;
        let p_term = error * self.kp;

        let integral_unbound = self.prev_integral + error * self.ki;

        let integral_term = if let Some(limit) = self.setpoint_limit {
            num_traits::clamp(integral_unbound, -limit.abs(), limit)
        } else {
            integral_unbound
        };
        self.prev_integral = integral_term;

        let prev = self.prev_measurement.replace(measurement);
        let d_term = match prev {
            Some(p) => -((measurement - p) * self.kd),
            None => T::zero(),
        };

        let output_unbound = p_term + self.prev_integral + d_term;

        match self.setpoint_limit {
            Some(limit) => num_traits::clamp(output_unbound, -limit.abs(), limit.abs()),
            None => output_unbound,
        }
    }

    pub fn update_terms(&mut self, new_p: T, new_i: T, new_d: T) {
        self.kp = new_p;
        self.ki = new_i;
        self.kd = new_d;
        self.prev_measurement = None;
        self.prev_integral = T::zero();
    }
}
