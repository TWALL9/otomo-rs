/// A simple pwm buzzer, use a timer's period to alter the tone
use stm32f4xx_hal::{
    pac::TIM1,
    prelude::*,
    timer::{
        pwm::{ChannelBuilder, PwmHz},
        Channel,
    },
};

use fugit::Rate;

/// Represents middle octave for the most part
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Notes {
    Rest,
    MiddleA,
    MiddleB,
    MiddleC,
    MiddleD,
    MiddleE,
    MiddleF,
    MiddleG,
    HighA,
    HighB,
    HighC,
    HighD,
    HighE,
    HighF,
    HighG,
}

impl Notes {
    pub fn to_tone(&self) -> Rate<u32, 1, 1> {
        match self {
            // Cannot have a 0Hz signal or else there will be a div0 fault, and disabling the
            // speaker is an annoying change of state
            Notes::Rest => 1.Hz(),
            Notes::MiddleA => 440.Hz(),
            Notes::MiddleB => 494.Hz(),
            Notes::MiddleC => 261.Hz(),
            Notes::MiddleD => 294.Hz(),
            Notes::MiddleE => 329.Hz(),
            Notes::MiddleF => 349.Hz(),
            Notes::MiddleG => 392.Hz(),
            Notes::HighA => 880.Hz(),
            Notes::HighB => 987.Hz(),
            Notes::HighC => 523.Hz(),
            Notes::HighD => 587.Hz(),
            Notes::HighE => 659.Hz(),
            Notes::HighF => 698.Hz(),
            Notes::HighG => 784.Hz(),
        }
    }
}

pub type BuzzerType = PwmHz<TIM1, ChannelBuilder<TIM1, 0>>;

pub struct Buzzer {
    tim: BuzzerType,
    channel: Channel,
}

impl Buzzer {
    pub fn new(tim: BuzzerType, channel: Channel) -> Self {
        let mut t = Self { tim, channel };

        t.disable();

        let max = t.tim.get_max_duty();
        t.tim.set_duty(t.channel, max / 2);

        t.enable();

        t
    }

    pub fn enable(&mut self) {
        self.tim.enable(self.channel);
    }

    pub fn disable(&mut self) {
        self.tim.disable(self.channel);
    }

    pub fn set_note(&mut self, note: Notes) {
        self.tim.set_period(note.to_tone());
    }
}
