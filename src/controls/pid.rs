use num_traits::Signed;

pub trait Number: PartialOrd + Signed + Copy + Default {}

impl <T: PartialOrd + Signed + Copy + Default> Number for T {}

#[derive(Default)]
pub struct PidCreator<T: Number> {
    p: Option<T>,
    i: Option<T>,
    d: Option<T>,
}

impl<T: Number>PidCreator<T> {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn set_p(self, p: impl Into<T>) -> Self {
        let mut s = self;
        s.p.replace(p.into());
        s
    }

    pub fn set_i(self, i: impl Into<T>) -> Self {
        let mut s = self;
        s.i.replace(i.into());
        s
    }

    pub fn set_d(self, d: impl Into<T>) -> Self {
        let mut s = self;
        s.d.replace(d.into());
        s
    }

    pub fn create_controller(self) -> PidController<T> {
        PidController {
            p: self.p.unwrap_or(T::zero()),
            i: self.i.unwrap_or(T::zero()),
            d: self.d.unwrap_or(T::zero()),
            setpoint: T::zero(),
            setpoint_limit: None,
            prev: None,
            prev_integral: T::zero(),
        }
    }
}

#[derive(Debug)]
pub struct PidController<T: Number> {
    p: T,
    i: T,
    d: T,
    setpoint: T,
    setpoint_limit: Option<T>,
    prev: Option<T>,
    prev_integral: T,
}

impl <T: Number> PidController<T> {
    pub fn setpoint(&mut self, setpoint: impl Into<T>, limit: Option<impl Into<T>>) {
        self.setpoint = setpoint.into();
        self.setpoint_limit = limit.map(|l| l.into());
    }

    pub fn update(&mut self, measurement: T) -> T {
        let error = self.setpoint - measurement;
        let p_term = error * self.p;

        self.prev_integral = self.prev_integral + error * self.i;
        // need to prevent integral windup

        let prev = self.prev.replace(measurement);
        let d_term = match prev {
            Some(p) => -((measurement - prev) * self.d),
            None => T::zero()
        };

        let output_unbound = p_term + self.prev_integral + d_term;
        
        let output = match self.setpoint_limit {
            Some(limit) => num_traits::clamp(output_unbound, -limit.abs(), limit.abs()),
            None => output_unbound
        };

        output
    }

    pub fn update_terms(self, p: Option<T>, i: Option<T>, d: Option<T>) -> Self {
        Self {
            p: p.unwrap_or(self.p),
            i: i.unwrap_or(self.i),
            d: d.unwrap_or(self.d),
            setpoint: self.setpoint,
            setpoint_limit: self.setpoint_limit,
            prev: self.prev,
            prev_integral: self.prev_integral,
        }
    }
}