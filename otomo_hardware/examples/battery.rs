#![no_main]
#![no_std]

use core::fmt::Write;

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hio;
use stm32f4xx_hal::{
    adc::{
        config::{AdcConfig, SampleTime, Sequence},
        Adc,
    },
    pac,
    prelude::*,
};

use otomo_hardware::battery_monitor::BatteryMonitor;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("Failed to get device periph");
    let cp = cortex_m::peripheral::Peripherals::take().expect("Failed to get core periph");

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(168.MHz()).freeze();

    let mut stdout = hio::hstdout().map_err(|_| core::fmt::Error).unwrap();

    let gpioa = dp.GPIOA.split();
    let cell0_pin = gpioa.pa4.into_analog(); // ADC12_IN4
    let cell1_pin = gpioa.pa5.into_analog(); // ADC12_IN5
    let cell2_pin = gpioa.pa7.into_analog(); // ADC12_IN7

    // Note, ADC2 does not calibrate a reference voltage like ADC1 does.
    // This means that the voltages will frequently be at or near the max.
    // To try and mitigate this, get the reference from ADC1 and pass that to ADC2
    let adc1 = Adc::adc1(dp.ADC1, true, AdcConfig::default());
    let vdda = adc1.reference_voltage();
    let config = AdcConfig::default().reference_voltage(vdda);
    let mut adc2 = Adc::adc2(dp.ADC2, true, config);

    adc2.configure_channel(&cell0_pin, Sequence::One, SampleTime::Cycles_112);
    adc2.configure_channel(&cell1_pin, Sequence::Two, SampleTime::Cycles_112);
    adc2.configure_channel(&cell2_pin, Sequence::Three, SampleTime::Cycles_112);

    writeln!(stdout, "1 ref v: {}", adc1.reference_voltage()).unwrap();
    writeln!(stdout, "2 ref v: {}", adc2.reference_voltage()).unwrap();

    let mut batt_mon = BatteryMonitor::new(adc2, cell0_pin, cell1_pin, cell2_pin);

    let mut delay = cp.SYST.delay(&clocks);

    delay.delay_ms(100);

    loop {
        let cells = batt_mon.get_cell_voltages();

        writeln!(stdout, "{}", cells.0).unwrap();

        delay.delay_ms(200);
    }
}
