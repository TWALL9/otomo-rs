#![no_main]
#![no_std]

use core::fmt::Write;

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hio;
use stm32f4xx_hal::{i2c::I2c, pac, prelude::*};

use otomo_hardware::imu::{lsm6dsox::*, Vector3};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("Failed to get device periph");
    let cp = cortex_m::peripheral::Peripherals::take().expect("Failed to get core periph");

    let mut stdout = hio::hstdout().map_err(|_| core::fmt::Error).unwrap();

    let gpiob = dp.GPIOB.split();
    let sda = gpiob.pb9;
    let scl = gpiob.pb8;

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(168.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);
    let i2c1 = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

    let mut lsm = Lsm6dsox::new(i2c1);

    delay.delay_ms(100);

    if let Err(e) = lsm.setup(&mut delay) {
        writeln!(stdout, "lsm init error {:?}", e).unwrap();
        panic!()
    }

    loop {
        delay.delay_ms(200);

        let mut accel_vec = Vector3::default();
        let mut gyro_vec = Vector3::default();
        let mut temperature = 0.0;

        match lsm.get_accel() {
            Ok(a) => accel_vec = a,
            Err(e) => writeln!(stdout, "accel err: {:?}", e).unwrap(),
        }

        match lsm.get_gyro() {
            Ok(g) => gyro_vec = g,
            Err(e) => writeln!(stdout, "gyro err: {:?}", e).unwrap(),
        }

        match lsm.get_temp() {
            Ok(t) => temperature = t,
            Err(e) => writeln!(stdout, "temp err: {:?}", e).unwrap(),
        }

        writeln!(
            stdout,
            "gyro: {:?}, accel: {:?}, temp: {}",
            gyro_vec, accel_vec, temperature
        )
        .unwrap();
    }
}
