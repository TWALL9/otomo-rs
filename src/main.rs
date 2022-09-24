#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

use defmt_rtt as _;
use panic_probe as _;

extern crate alloc;

mod proto;
use proto::{decode_proto_msg, encode_proto};

use alloc::vec::Vec;
use core::{alloc::Layout, fmt::Write};
use embedded_hal::serial::Read;

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    panic!("OOM");
}

const NAME: &str = env!("CARGO_PKG_NAME");
const VERSION: &str = env!("CARGO_PKG_VERSION");

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    use super::*;
    use alloc_cortex_m::CortexMHeap;
    use defmt::{error, info, Format};
    use stm32f4xx_hal::{
        gpio::{Output, PD14, PD15},
        pac::{TIM2, TIM4, USART2},
        prelude::*,
        serial::{Rx, Serial, Tx},
        timer::{pwm::PwmExt, MonoTimerUs, PwmChannel, Timer4},
    };

    #[global_allocator]
    static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

    pub struct Hc05 {
        pub rx: Rx<USART2>,
        pub tx: Tx<USART2>,
    }

    #[derive(Debug, Clone, Copy, Format, Default)]
    #[allow(dead_code)]
    enum MotorDirection {
        Forward(f32),
        Backward(f32),
        Brake,
        #[default]
        Release,
    }
    struct HBridge<P1: PwmExt, P2: PwmExt, const C: u8, const D: u8> {
        input_1: PwmChannel<P1, C>,
        input_2: PwmChannel<P2, D>,
    }

    impl<P1: PwmExt, P2: PwmExt, const C: u8, const D: u8> HBridge<P1, P2, C, D> {
        fn new(input_1: PwmChannel<P1, C>, input_2: PwmChannel<P2, D>) -> Self {
            let mut shield = Self { input_1, input_2 };

            // set an initial state
            shield.input_1.enable();
            shield.input_2.enable();
            shield.run(MotorDirection::Release);

            shield
        }

        fn run(&mut self, direction: MotorDirection) {
            let max_1 = self.input_1.get_max_duty();
            let max_2 = self.input_2.get_max_duty();
            let (a_duty, b_duty) = match direction {
                MotorDirection::Forward(d) => {
                    let duty_ratio = d.clamp(0.0, 1.0);
                    ((max_1 as f32 * duty_ratio) as u16, 0)
                }
                MotorDirection::Backward(d) => {
                    let duty_ratio = d.clamp(0.0, 1.0);
                    (0, (max_2 as f32 * duty_ratio) as u16)
                }
                MotorDirection::Brake => (max_1, max_2),
                MotorDirection::Release => (0, 0),
            };

            self.input_1.set_duty(a_duty);
            self.input_2.set_duty(b_duty);
        }
    }

    #[shared]
    struct Shared {
        hc05: Hc05,
    }

    #[local]
    struct Local {
        red_led: PD14<Output>,
        blue_led: PD15<Output>,
        cmd: Vec<u8>,
        _h_bridge: HBridge<TIM4, TIM4, 0, 1>,
    }

    #[monotonic(binds = TIM2, default = true)]
    type MicrosecMono = MonoTimerUs<TIM2>;

    // type PwmPin = PwmHz<TIM4, (Ch<0>, Ch<1>), (Pin<'D', 12, Alternate<2>>, Pin<'D', 13, Alternate<2>>)>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        info!("{} v{}", NAME, VERSION);

        // Initialize heap
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024;
            static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
        }

        // set DBGMCU to allow wfi in idle function while using defmt
        let dbgmcu = ctx.device.DBGMCU;
        dbgmcu.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        // enabling the dma1 clock keeps one AHB bus master active, which prevents SRAM from reading as 0's
        // https://github.com/probe-rs/probe-rs/issues/350#issuecomment-740550519
        ctx.device.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());

        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

        let gpiod = ctx.device.GPIOD.split();
        let mut red_led = gpiod.pd14.into_push_pull_output();
        let mut blue_led = gpiod.pd15.into_push_pull_output();
        red_led.set_low();
        blue_led.set_low();

        let gpioa = ctx.device.GPIOA.split();
        let tx_pin = gpioa.pa2.into_alternate();
        // let tx_pin = stm32f4xx_hal::gpio::NoPin;
        let rx_pin = gpioa.pa3.into_alternate();
        let mut serial =
            Serial::new(ctx.device.USART2, (tx_pin, rx_pin), 38400.bps(), &clocks).unwrap();
        serial.listen(stm32f4xx_hal::serial::Event::Rxne);
        let (tx, rx) = serial.split();
        let hc05 = Hc05 { tx, rx };

        let tim4 = Timer4::new(ctx.device.TIM4, &clocks);

        let tim4_pins = (gpiod.pd12.into_alternate(), gpiod.pd13.into_alternate());
        let pwm4 = tim4.pwm_hz(tim4_pins, 10.kHz());

        // pwm4.deref_mut().listen(Event::C1);
        // Add C2 event as well?
        let (input_1, input_2) = pwm4.split();
        let _h_bridge = HBridge::new(input_1, input_2);

        let mono = ctx.device.TIM2.monotonic_us(&clocks);
        tick::spawn().ok();
        (
            Shared { hc05 },
            Local {
                red_led,
                blue_led,
                cmd: Vec::new(),
                _h_bridge,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        info!("idle!");
        loop {
            tick::spawn_after(1.secs()).ok();
            rtic::export::wfi();
        }
    }

    #[task(binds = USART2, local=[red_led, cmd], shared = [hc05])]
    fn usart2(mut ctx: usart2::Context) {
        let mut write_back = false;
        ctx.shared.hc05.lock(|h| {
            match h.rx.read() {
                Ok(b) => {
                    let c = b as char;
                    ctx.local.cmd.push(b);
                    ctx.local.red_led.set_low();
                    if c == '\n' {
                        write_back = true;
                    }
                }
                Err(_) => {
                    ctx.local.red_led.set_high();
                }
            };
        });

        if write_back {
            use alloc::string::String;

            if let Err(e) = decode_proto_msg::<proto::TopMsg>(ctx.local.cmd.as_slice()) {
                error!("Proto decode error!: {:?}", e);
            }

            let msg = proto::TopMsg {
                msg: Some(proto::top_msg::Msg::Log(proto::Log {
                    level: 0,
                    handle: String::new(),
                    message: String::from("asdf"),
                })),
            };
            let enc = encode_proto(msg).unwrap();

            ctx.shared.hc05.lock(|h| {
                for b in enc.iter() {
                    h.tx.write(*b).ok();
                }
                write!(h.tx, "\r\n").ok();
            });
            ctx.local.cmd.clear();
        }

        if ctx.local.cmd.len() > 100 {
            ctx.local.cmd.clear();
        }
    }

    #[task(local = [blue_led])]
    fn tick(ctx: tick::Context) {
        ctx.local.blue_led.toggle();
    }

    // #[task(binds = TIM4, local = [pwm])]
    // fn tim4(ctx: tim4::Context) {
    //     // WHY DOES RTIC NOT CLEAR THE TIMER INTERRUPTS???
    //     ctx.local.pwm.deref_mut().clear_interrupt(Event::C1);
    // }
}
