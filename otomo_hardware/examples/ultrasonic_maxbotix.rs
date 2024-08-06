#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use defmt::debug;
use defmt_rtt as _;
use stm32f4xx_hal::{gpio::PB10, i2c::I2c, pac, prelude::*};

use otomo_hardware::ultrasonic::maxbotix::MaxBotix;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("Failed to get device periph");
    let cp = cortex_m::peripheral::Peripherals::take().expect("Failed to get core periph");

    let gpiob = dp.GPIOB.split();
    let pb10 = gpiob.pb10.into_input();
    let sda = gpiob.pb7;
    let scl = gpiob.pb8;

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(168.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);
    let i2c1 = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

    let mut ultrasonic = MaxBotix::<pac::I2C1, PB10>::new(i2c1, pb10, None);

    loop {
        delay.delay_ms(100_u32);
        ultrasonic.start_read().expect("couldn't start read");
        delay.delay_ms(100_u32);
        let dist = ultrasonic.read_distance().expect("couldn't read distance");
        debug!("Distance read: {}", dist);
    }
}
