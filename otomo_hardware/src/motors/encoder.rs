use embedded_hal::{Direction, Qei};
use fugit::TimerInstantU32;
use num_traits::{float::FloatCore, ToPrimitive, WrappingSub};

use super::{Encoder, MotorOdometry};
use core::f32::consts::PI;

trait TimerWidth: PartialOrd + PartialEq + Copy + Default + WrappingSub + ToPrimitive {}
impl<T: PartialOrd + Copy + Default + PartialEq + WrappingSub + ToPrimitive> TimerWidth for T {}

pub struct QuadratureEncoder<Q: Qei> {
    qei: Q,
    rads_per_tick: f32,
    current_count: Option<Q::Count>,
    current_time: Option<TimerInstantU32<1_000_000>>,
}

impl<Q: Qei> QuadratureEncoder<Q> {
    pub fn new(qei: Q, ticks_per_revolution: usize) -> Self {
        let ticks_per_half_rev = (ticks_per_revolution / 2) as f32;
        Self {
            qei,
            rads_per_tick: PI / ticks_per_half_rev,
            current_count: None,
            current_time: None,
        }
    }
}

impl<Q: Qei> Encoder for QuadratureEncoder<Q>
where
    Q::Count: TimerWidth,
{
    fn get_velocity(&mut self, now: TimerInstantU32<1_000_000>) -> Option<MotorOdometry> {
        let current_count = self.qei.count();
        let last_count = self.current_count.replace(current_count);
        let last_time = self.current_time.replace(now);

        if last_count.is_none() || last_time.is_none() || last_time >= Some(now) {
            None
        } else if Some(current_count) == last_count {
            Some(MotorOdometry::Stationary)
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
                Direction::Downcounting => Some(MotorOdometry::Moving(vel * -1_f32)),
                Direction::Upcounting => Some(MotorOdometry::Moving(vel)),
            }
        } else {
            None
        }
    }

    fn get_position(&mut self) -> f32 {
        self.qei.count().to_f32().unwrap_or(0.0)
    }
}
