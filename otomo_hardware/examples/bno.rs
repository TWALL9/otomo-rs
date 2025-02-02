#![no_main]
#![no_std]

use core::fmt::Write;

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hio;
use stm32f4xx_hal::{i2c::I2c, pac, prelude::*};

use otomo_hardware::imu::bno055::*;

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

    let mut bno = Bno055::new(i2c1, false);

    delay.delay_ms(100);

    if let Err(e) = bno.init(&mut delay) {
        writeln!(stdout, "bno init error {:?}", e).unwrap();
        panic!()
    }

    if let Err(e) = bno.set_external_crystal(&mut delay, true) {
        writeln!(stdout, "bno xtal error {:?}", e).unwrap();
        panic!()
    }

    loop {
        match bno.get_calibration_status() {
            // Ok(c) => writeln!(stdout, "calib: {:?}", c).unwrap(),
            Err(e) => writeln!(stdout, "calib error: {:?}", e).unwrap(),
            _ => {}
        }

        delay.delay_ms(200);

        match bno.get_whole_shebang() {
            Ok(v) => writeln!(stdout, "shebang: {:?}", v).unwrap(),
            Err(e) => writeln!(stdout, "gyro err: {:?}", e).unwrap(),
        }

        // match bno.get_temp() {
        //     Ok(v) => writeln!(stdout, "temp: {:?}", v).unwrap(),
        //     Err(e) => writeln!(stdout, "temp err: {:?}", e).unwrap(),
        // }
    }
}
