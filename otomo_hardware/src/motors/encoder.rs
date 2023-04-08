use embedded_hal::{Direction, Qei};
use fugit::TimerInstantU32;

use super::{Encoder, MotorOdometry};
use crate::qei::{LeftQei, RightQei};

use core::f32::consts::PI;

pub struct QuadratureEncoder<Q: Qei> {
    qei: Q,
    rads_per_tick: f32,
    current_count: Option<Q::Count>,
    current_time: Option<TimerInstantU32<1_000_000>>,
}

pub type LeftEncoder = QuadratureEncoder<LeftQei>;
pub type RightEncoder = QuadratureEncoder<RightQei>;

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

impl<Q: Qei<Count = u16>> Encoder for QuadratureEncoder<Q> {
    fn update_speed(&mut self, now: TimerInstantU32<1_000_000>) -> Option<MotorOdometry> {
        let current_count = self.qei.count();
        let last_count = self.current_count.replace(current_count);
        let last_time = self.current_time.replace(now);

        if last_count.is_none() || last_time.is_none() || last_time == Some(now) {
            None
        } else if Some(current_count) == last_count {
            Some(MotorOdometry::Stationary)
        } else if let (Some(last_count), Some(last_time)) = (last_count, last_time) {
            let tick_diff = current_count.wrapping_sub(last_count) as i16 as f32;
            let rad_diff = self.rads_per_tick * tick_diff;
            let vel = rad_diff / (now - last_time).to_millis() as f32;
            match self.qei.direction() {
                Direction::Downcounting => Some(MotorOdometry::Backward(vel)),
                Direction::Upcounting => Some(MotorOdometry::Forward(vel)),
            }
        } else {
            None
        }
    }
}
