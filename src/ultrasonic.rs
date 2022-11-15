use fugit::TimerInstantU32;
use stm32f4xx_hal::gpio::{ExtiPin, Input, Output, Pin, PushPull};

use defmt::Format;

// use defmt::info;

const SPEED_OF_SOUND: f32 = 0.34; // mm/us

#[derive(Debug, Format)]
enum MeasurementState {
    NotStarted,
    Running,
}

pub struct Hcsr04<const TP: char, const TN: u8, const EP: char, const EN: u8> {
    trig_pin: Pin<TP, TN, Output<PushPull>>,
    echo_pin: Pin<EP, EN, Input>,
    start_time: TimerInstantU32<1_000_000>,
    state: MeasurementState,
}

impl<const TP: char, const TN: u8, const EP: char, const EN: u8> Hcsr04<TP, TN, EP, EN> {
    pub fn new(mut trig_pin: Pin<TP, TN, Output<PushPull>>, echo_pin: Pin<EP, EN, Input>) -> Self {
        trig_pin.set_low();
        Self {
            trig_pin,
            echo_pin,
            start_time: TimerInstantU32::from_ticks(0),
            state: MeasurementState::NotStarted,
        }
    }

    pub fn start_trigger(&mut self) {
        self.trig_pin.set_high();
    }

    pub fn finish_trigger(&mut self) {
        self.trig_pin.set_low();
    }

    pub fn check_distance(&mut self, now: TimerInstantU32<1_000_000>) -> Option<f32> {
        // info!("state: {:?}, pin: {}, now: {}", self.state, self.echo_pin.is_high(), now);
        match self.state {
            MeasurementState::NotStarted => {
                if self.echo_pin.is_high() {
                    self.echo_pin.clear_interrupt_pending_bit();
                    self.start_time = now;
                    self.state = MeasurementState::Running;
                }

                None
            }
            MeasurementState::Running => {
                if self.echo_pin.is_low() {
                    self.echo_pin.clear_interrupt_pending_bit();
                    self.state = MeasurementState::NotStarted;
                    now.checked_duration_since(self.start_time).map(|d| {
                        let ticks = d.ticks() as f32;
                        SPEED_OF_SOUND * ticks / 2_f32
                    })
                } else {
                    None
                }
            }
        }
    }
}