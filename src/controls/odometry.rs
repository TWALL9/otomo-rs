//! Take the current encoder speed and smooth it while rejecting single-sample spikes.
//!
//! This filter first performs a small median filter over the last 3 raw samples
//! to remove single-sample spikes, then feeds the median into a rolling average
//! buffer of size `N` to smooth the result.

#[derive(Debug, Clone, Copy)]
pub struct RollingAverage<const N: usize> {
    values: [f32; N],
    index: usize,
    sum: f32,
    max_diff: f32,
    prev: f32,

    // small history for median-of-3 filtering of raw inputs
    raw_hist: [f32; 3],
    raw_index: usize,
    raw_count: usize,
}

impl<const N: usize> RollingAverage<N> {
    pub fn new(max_diff: f32) -> Self {
        Self {
            values: [0.0; N],
            index: 0,
            sum: 0.0,
            max_diff,
            prev: 0.0,
            raw_hist: [0.0; 3],
            raw_index: 0,
            raw_count: 0,
        }
    }

    fn median3(a: [f32; 3]) -> f32 {
        let mut v = a;
        // simple sorting network for 3 values
        if v[0] > v[1] {
            v.swap(0, 1);
        }
        if v[1] > v[2] {
            v.swap(1, 2);
        }
        if v[0] > v[1] {
            v.swap(0, 1);
        }
        v[1]
    }

    pub fn update(&mut self, new_val: f32) -> f32 {
        // Reject plain-insane samples by comparing against last accepted value.
        // If the new sample differs from `prev` by more than `max_diff`, treat
        // it as a transient and use `prev` as the raw input instead.
        let raw = if self.raw_count > 0 && (new_val - self.prev).abs() > self.max_diff {
            self.prev
        } else {
            new_val
        };

        // update raw-history for median-of-3
        self.raw_hist[self.raw_index] = raw;
        self.raw_index = (self.raw_index + 1) % 3;
        if self.raw_count < 3 {
            self.raw_count += 1;
        }

        // compute median of available raw history (use average when <3 samples)
        let median = if self.raw_count < 3 {
            let mut s = 0.0f32;
            for i in 0..self.raw_count {
                s += self.raw_hist[i];
            }
            s / (self.raw_count as f32)
        } else {
            Self::median3(self.raw_hist)
        };

        // store last accepted (median) for next-cycle spike rejection
        self.prev = median;

        // push median into rolling average buffer
        self.sum += median - self.values[self.index];
        self.values[self.index] = median;
        self.index = (self.index + 1) % N;

        // return the average
        self.sum / (N as f32)
    }
}
