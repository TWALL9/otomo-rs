#![no_std]

use stm32f4xx_hal::{
    gpio::Edge,
    pac::{CorePeripherals, Peripherals, TIM2},
    prelude::*,
    serial::Serial,
    timer::{MonoTimerUs, SysDelay, Timer3, Timer4},
};

pub mod led;
pub mod pwm;
pub mod serial;
pub mod ultrasonic;

use led::{BlueLed, GreenLed, OrangeLed, RedLed};
use pwm::{Pwm3, Pwm4};
use serial::BluetoothSerialPort;
use ultrasonic::{Hcsr04, Ultrasonics};

pub type MonoTimer = MonoTimerUs<TIM2>;

pub struct OtomoHardware {
    pub mono: MonoTimer,
    pub delay: SysDelay,
    pub green_led: GreenLed,
    pub orange_led: OrangeLed,
    pub red_led: RedLed,
    pub blue_led: BlueLed,

    pub bt_serial: BluetoothSerialPort,

    pub pwm3: Pwm3,
    pub pwm4: Pwm4,

    pub ultrasonics: Ultrasonics,
}

impl OtomoHardware {
    pub fn init(mut pac: Peripherals, core: CorePeripherals) -> Self {
        let mut syscfg = pac.SYSCFG.constrain();

        let rcc = pac.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();
        let mono = pac.TIM2.monotonic_us(&clocks);
        let delay = core.SYST.delay(&clocks);

        let gpioa = pac.GPIOA.split();
        let gpiob = pac.GPIOB.split();
        let gpioc = pac.GPIOC.split();
        let gpiod = pac.GPIOD.split();

        // Status LED's
        let green_led = gpiod.pd12.into_push_pull_output();
        let orange_led = gpiod.pd13.into_push_pull_output();
        let red_led = gpiod.pd14.into_push_pull_output();
        let blue_led = gpiod.pd15.into_push_pull_output();

        let tx_pin = gpioa.pa2.into_alternate();
        // let tx_pin = stm32f4xx_hal::gpio::NoPin;
        let rx_pin = gpioa.pa3.into_alternate();
        let mut bt_serial =
            Serial::new(pac.USART2, (tx_pin, rx_pin), 38400.bps(), &clocks).unwrap();
        bt_serial.listen(stm32f4xx_hal::serial::Event::Rxne);

        let tim3 = Timer3::new(pac.TIM3, &clocks);
        let tim3_pins = (
            gpioc.pc6.into_alternate(),
            gpioc.pc7.into_alternate(),
            gpioc.pc8.into_alternate(),
            gpioc.pc9.into_alternate(),
        );
        let pwm3 = tim3.pwm_hz(tim3_pins, 10.kHz());

        let tim4 = Timer4::new(pac.TIM4, &clocks);
        let tim4_pins = (
            gpiob.pb6.into_alternate(),
            gpiob.pb7.into_alternate(),
            gpiob.pb8.into_alternate(),
            gpiob.pb9.into_alternate(),
        );
        let pwm4 = tim4.pwm_hz(tim4_pins, 10.kHz());

        // Right ultrasonic
        let trig_pin = gpiob.pb11.into_push_pull_output();
        let mut echo_pin = gpiob.pb10.into_pull_down_input();
        echo_pin.make_interrupt_source(&mut syscfg);
        echo_pin.enable_interrupt(&mut pac.EXTI);
        echo_pin.trigger_on_edge(&mut pac.EXTI, Edge::RisingFalling);

        let right_ultrasonic = Hcsr04::new(trig_pin, echo_pin);

        let trig_pin = gpioc.pc11.into_push_pull_output();
        let mut echo_pin = gpioc.pc10.into_pull_down_input();
        // echo_pin.make_interrupt_source(&mut syscfg);
        echo_pin.enable_interrupt(&mut pac.EXTI);
        echo_pin.trigger_on_edge(&mut pac.EXTI, Edge::RisingFalling);
        let left_ultrasonic = Hcsr04::new(trig_pin, echo_pin);

        let counter = pac.TIM9.counter_us(&clocks);
        let ultrasonics = Ultrasonics {
            counter,
            right: right_ultrasonic,
            left: left_ultrasonic,
        };

        Self {
            mono,
            delay,
            green_led,
            orange_led,
            red_led,
            blue_led,
            bt_serial,
            pwm3,
            pwm4,
            ultrasonics,
        }
    }
}
