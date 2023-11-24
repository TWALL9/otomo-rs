use embedded_hal::{Direction, Qei};
use fugit::TimerInstantU32;
use num_traits::{float::FloatCore, ToPrimitive, WrappingSub};

use super::Encoder;
use core::f32::consts::PI;

trait TimerWidth: PartialOrd + PartialEq + Copy + Default + WrappingSub + ToPrimitive {}
impl<T: PartialOrd + Copy + Default + PartialEq + WrappingSub + ToPrimitive> TimerWidth for T {}

pub struct QuadratureEncoder<Q: Qei> {
    qei: Q,
    rads_per_tick: f32,
    current_count: Option<Q::Count>,
    current_time: Option<TimerInstantU32<10_000>>,
}

impl<Q: Qei> QuadratureEncoder<Q> {
    pub fn new(qei: Q, ticks_per_revolution: usize) -> Self {
        Self {
            qei,
            rads_per_tick: (2.0 * PI) / (ticks_per_revolution as f32),
            current_count: None,
            current_time: None,
        }
    }
}

impl<Q: Qei> Encoder for QuadratureEncoder<Q>
where
    Q::Count: TimerWidth,
{
    fn get_velocity(&mut self, now: TimerInstantU32<10_000>) -> f32 {
        let current_count = self.qei.count();
        let last_count = self.current_count.replace(current_count);
        let last_time = self.current_time.replace(now);

        if last_count.is_none() || last_time.is_none() || last_time >= Some(now) {
            0_f32
        } else if Some(current_count) == last_count {
            0_f32
        } else if let (Some(last_count), Some(last_time)) = (last_count, last_time) {
            let tick_diff = if current_count >= last_count {
                current_count - last_count
            } else {
                last_count - current_count
            };
            let tick_diff = tick_diff.to_f32().unwrap_or(0_f32).abs();
            let rad_diff = self.rads_per_tick * tick_diff;
            let time_diff = (now - last_time).to_millis() as f32;
            let vel = rad_diff / (time_diff / 1000_f32); // to_secs() would have returned 0
            match self.qei.direction() {
                Direction::Downcounting => vel * -1_f32,
                Direction::Upcounting => vel,
            }
        } else {
            0_f32
        }
    }

    fn get_position(&mut self) -> f32 {
        self.qei.count().to_f32().unwrap_or(0.0) * self.rads_per_tick
    }
}
