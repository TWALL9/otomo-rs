#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

use defmt_rtt as _;
use panic_probe as _;

extern crate alloc;

mod proto;
use proto::{decode_proto_msg, encode_proto, top_msg::Msg, Joystick};

mod motors;
use motors::{hbridge::HBridge, joystick_tank_controls};

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
    use defmt::{error, info};
    use stm32f4xx_hal::{
        pac::{TIM2, TIM3, TIM4, USART2},
        prelude::*,
        serial::{Rx, Serial, Tx},
        timer::{MonoTimerUs, Timer3, Timer4},
    };

    #[global_allocator]
    static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

    pub struct Hc05 {
        pub rx: Rx<USART2>,
        pub tx: Tx<USART2>,
    }

    #[shared]
    struct Shared {
        hc05: Hc05,
    }

    #[local]
    struct Local {
        cmd: Vec<u8>,
        front_right: HBridge<TIM4, TIM4, 2, 3>,
        rear_right: HBridge<TIM4, TIM4, 0, 1>,
        front_left: HBridge<TIM3, TIM3, 2, 3>,
        rear_left: HBridge<TIM3, TIM3, 0, 1>,
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

        let gpioa = ctx.device.GPIOA.split();
        let gpioc = ctx.device.GPIOC.split();
        let gpiod = ctx.device.GPIOD.split();

        let tx_pin = gpioa.pa2.into_alternate();
        // let tx_pin = stm32f4xx_hal::gpio::NoPin;
        let rx_pin = gpioa.pa3.into_alternate();
        let mut serial =
            Serial::new(ctx.device.USART2, (tx_pin, rx_pin), 38400.bps(), &clocks).unwrap();
        serial.listen(stm32f4xx_hal::serial::Event::Rxne);
        let (tx, rx) = serial.split();
        let hc05 = Hc05 { tx, rx };

        let tim3 = Timer3::new(ctx.device.TIM3, &clocks);
        let tim3_pins = (
            gpioc.pc6.into_alternate(),
            gpioc.pc7.into_alternate(),
            gpioc.pc8.into_alternate(),
            gpioc.pc9.into_alternate(),
        );
        let pwm3 = tim3.pwm_hz(tim3_pins, 10.kHz());
        let (rear_left_a, rear_left_b, front_left_a, front_left_b) = pwm3.split();
        let rear_left = HBridge::new(rear_left_a, rear_left_b);
        let front_left = HBridge::new(front_left_a, front_left_b);

        let tim4 = Timer4::new(ctx.device.TIM4, &clocks);
        let tim4_pins = (
            gpiod.pd12.into_alternate(),
            gpiod.pd13.into_alternate(),
            gpiod.pd14.into_alternate(),
            gpiod.pd15.into_alternate(),
        );
        let pwm4 = tim4.pwm_hz(tim4_pins, 10.kHz());

        // pwm4.deref_mut().listen(Event::C1);
        // Add C2 event as well?
        let (rear_right_a, rear_right_b, front_right_a, front_right_b) = pwm4.split();
        let rear_right = HBridge::new(rear_right_a, rear_right_b);
        let front_right = HBridge::new(front_right_a, front_right_b);

        let mono = ctx.device.TIM2.monotonic_us(&clocks);
        tick::spawn().ok();
        (
            Shared { hc05 },
            Local {
                cmd: Vec::new(),
                front_right,
                rear_right,
                front_left,
                rear_left,
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

    #[task(binds = USART2, local=[cmd], shared = [hc05])]
    fn usart2(mut ctx: usart2::Context) {
        let mut msg_complete = false;
        ctx.shared.hc05.lock(|h| {
            if let Ok(b) = h.rx.read() {
                let c = b as char;
                ctx.local.cmd.push(b);
                if c == '\n' {
                    msg_complete = true;
                }
            }
        });

        if msg_complete {
            use alloc::string::String;

            match decode_proto_msg::<proto::TopMsg>(ctx.local.cmd.as_slice()) {
                Ok(t) => {
                    if let Some(m) = t.msg {
                        if let Msg::Joystick(j) = m {
                            let directions = joystick_tank_controls(j.speed, j.heading);
                            info!("received directions: {:?}", directions);
                        }
                    }
                }
                Err(e) => error!("Proto decode error!: {:?}", e),
            };

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

    #[task(local = [])]
    fn tick(_ctx: tick::Context) {
        // ctx.local.blue_led.toggle();
    }

    // #[task(binds = TIM4, local = [pwm])]
    // fn tim4(ctx: tim4::Context) {
    //     // WHY DOES RTIC NOT CLEAR THE TIMER INTERRUPTS???
    //     ctx.local.pwm.deref_mut().clear_interrupt(Event::C1);
    // }
}
