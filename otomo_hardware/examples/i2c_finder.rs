#![no_main]
#![no_std]

use core::fmt::Write;

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hio;
use stm32f4xx_hal::{
    i2c::{Error, I2c, NoAcknowledgeSource},
    pac,
    prelude::*,
};

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
    let mut i2c1 = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &clocks);
    let mut buf = [0_u8; 1];

    let disallowed_addresses: [u8; 3] = [80, 164, 170];

    delay.delay_ms(100_u32);

    for addr in (0x02..0x7E).step_by(2) {
        if disallowed_addresses.contains(&addr) {
            continue;
        }
        match i2c1.read(addr, &mut buf) {
            Ok(_) => {
                writeln!(stdout, "Address found: {}", addr).unwrap();
                loop {}
            }
            Err(e) if e == Error::NoAcknowledge(NoAcknowledgeSource::Address) => (),
            Err(s) => writeln!(stdout, "Some other error: {:?}", s).unwrap(),
        };
        delay.delay_ms(10_u32);
    }

    writeln!(stdout, "No addresses found :(").unwrap();

    loop {}
}
