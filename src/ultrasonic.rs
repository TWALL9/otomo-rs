use fugit::TimerInstantU32;
use stm32f4xx_hal::gpio::{ExtiPin, Input, Output, Pin, PushPull};

const SPEED_OF_SOUND: f32 = 0.34; // mm/us

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

    pub fn check_distance(&mut self) -> Option<f32> {
        match self.state {
            MeasurementState::NotStarted => {
                if self.echo_pin.is_high() {
                    self.echo_pin.clear_interrupt_pending_bit();
                    self.start_time = TimerInstantU32::from_ticks(0);
                    self.state = MeasurementState::Running;
                }

                None
            }
            MeasurementState::Running => {
                if self.echo_pin.is_low() {
                    self.echo_pin.clear_interrupt_pending_bit();
                    self.state = MeasurementState::NotStarted;
                    let ticks = self.start_time.duration_since_epoch().ticks();
                    Some(SPEED_OF_SOUND * (ticks as f32))
                } else {
                    None
                }
            }
        }
    }
}
