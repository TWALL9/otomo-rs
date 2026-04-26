//! Take the current encoder speed and compare it to previous rolling average to determine if the speed is insane, or should be rejected

use micromath::{statistics::StdDev, F32};

#[derive(Debug, Clone, Copy)]
pub struct EncoderRollingStats<const N: usize> {
    buffer: [F32; N],
    average: F32,
    index: usize,
    std_dev: f32,
    abs_boundary: F32,
    prev: f32,
}

impl<const N: usize> EncoderRollingStats<N> {
    pub fn new(abs_boundary: f32) -> Self {
        Self {
            buffer: [F32(0.0); N],
            average: F32(0.0),
            index: 0,
            std_dev: 0.0,
            abs_boundary: F32(abs_boundary),
            prev: 0.0,
        }
    }

    pub fn update(&mut self, new_vel: f32) -> f32 {
        // If the value is plain insane, just reject it out of hand.
        if new_vel.abs() >= self.abs_boundary {
            return self.prev;
        }

        let new_vel = F32(new_vel);

        self.buffer[self.index] = new_vel;
        self.index = (self.index + 1) % self.buffer.len();

        let mut new_avg = F32(0.0);
        for i in self.buffer.iter() {
            new_avg += *i / (F32(N as f32));
        }

        self.average = new_avg;

        self.std_dev = self.buffer.stddev();

        let new_z = (new_vel - new_avg) / self.std_dev;

        if new_z.abs() <= 1.5 {
            self.prev = new_vel.0;
            self.average.0
        } else {
            self.prev
        }
    }
}
