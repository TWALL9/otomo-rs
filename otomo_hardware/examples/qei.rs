#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt::debug;
use defmt_rtt as _;
use fugit::{Instant, TimerDurationU32 as Duration};
use panic_halt as _;
use stm32f4xx_hal::{pac, prelude::*, qei::Qei};

use otomo_hardware::motors::{encoder::QuadratureEncoder, Encoder, MotorOdometry};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("Failed to get device periph");
    let cp = cortex_m::peripheral::Peripherals::take().expect("Failed to get core periph");

    let gpiob = dp.GPIOB.split();
    let gpiod = dp.GPIOD.split();
    let mut led = gpiod.pd12.into_push_pull_output();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let mut delay = cp.SYST.delay(&clocks);

    let rotary_encoder_pins = (gpiob.pb6.into_alternate(), gpiob.pb7.into_alternate());
    let rotary_timer = dp.TIM4;
    let rotary_encoder = Qei::new(rotary_timer, rotary_encoder_pins);

    let mut encoder = QuadratureEncoder::new(rotary_encoder, 6500);

    let mut instant = Instant::<u32, 1, 1_000_000>::from_ticks(1_000_000);

    loop {
        match encoder.update_speed(instant) {
            Some(MotorOdometry::Stationary) => debug!("No movement!"),
            Some(MotorOdometry::Forward(s)) => {
                debug!("forward: {:?}", s);
                led.set_high();
            }
            Some(MotorOdometry::Backward(s)) => {
                debug!("backward: {:?}", s);
                led.set_low();
            }
            None => debug!("odometry not ready"),
        };

        delay.delay_ms(10_u32);
        instant += Duration::millis(10);
    }
}
