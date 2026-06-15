//! Take the current encoder speed and compare it to previous rolling average to determine if the speed is insane, or should be rejected

#[derive(Debug, Clone, Copy)]
pub struct RollingAverage<const N: usize> {
    values: [f32; N],
    index: usize,
    sum: f32,
    max_diff: f32,
    prev: f32,
}

impl<const N: usize> RollingAverage<N> {
    pub fn new(max_diff: f32) -> Self {
        Self {
            values: [0.0; N],
            index: 0,
            sum: 0.0,
            max_diff,
            prev: 0.0,
        }
    }

    pub fn update(&mut self, new_val: f32) -> f32 {
        // If the value is plain insane, reject it.
        let val_to_send = if (new_val - self.prev).abs() > self.max_diff {
            self.prev
        } else {
            new_val
        };

        self.prev = val_to_send;

        self.sum += val_to_send - self.values[self.index];
        self.values[self.index] = val_to_send;
        self.index = (self.index + 1) % N;

        self.sum / (N as f32)
    }
}
