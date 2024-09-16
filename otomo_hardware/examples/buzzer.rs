#![no_std]
#![no_main]

/// NOTE: adapted from https://dev.to/theembeddedrustacean/stm32f4-embedded-rust-at-the-hal-pwm-buzzer-3f1b
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
    pac::Peripherals,
    prelude::*,
    timer::{Channel, Channel1, Timer1},
};

use otomo_hardware::buzzer::*;

#[entry]
fn main() -> ! {
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();
    let pac = Peripherals::take().unwrap();

    let rcc = pac.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .require_pll48clk()
        .freeze();

    let mut delay = cp.SYST.delay(&clocks);

    let gpioa = pac.GPIOA.split();
    let tim1 = Timer1::new(pac.TIM1, &clocks);
    let tim1_pins = Channel1::new(gpioa.pa8);
    let buzz_pwm = tim1.pwm_hz(tim1_pins, 10.kHz());

    let mut buzzer = Buzzer::new(buzz_pwm, Channel::C1);

    buzzer.enable();

    // Define the notes to be played and the beats per note
    let tune = [
        (Notes::MiddleC, 1),
        (Notes::MiddleC, 1),
        (Notes::MiddleG, 1),
        (Notes::MiddleG, 1),
        (Notes::MiddleA, 1),
        (Notes::MiddleA, 1),
        (Notes::MiddleG, 2),
        (Notes::MiddleF, 1),
        (Notes::MiddleF, 1),
        (Notes::MiddleE, 1),
        (Notes::MiddleE, 1),
        (Notes::MiddleD, 1),
        (Notes::MiddleD, 1),
        (Notes::MiddleC, 2),
        (Notes::HighC, 1),
        (Notes::HighC, 1),
        (Notes::HighG, 1),
        (Notes::HighG, 1),
        (Notes::HighA, 1),
        (Notes::HighA, 1),
        (Notes::HighG, 2),
        (Notes::HighF, 1),
        (Notes::HighF, 1),
        (Notes::HighE, 1),
        (Notes::HighE, 1),
        (Notes::HighD, 1),
        (Notes::HighD, 1),
        (Notes::HighC, 2),
    ];

    let tempo = 300_u32;

    // Application Loop
    loop {
        buzzer.enable();

        for (note, time) in tune {
            buzzer.set_note(note);
            delay.delay_ms(time * tempo);
        }

        buzzer.disable();

        delay.delay_ms(500);
    }
}
