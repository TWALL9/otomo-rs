#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

use defmt_rtt as _;
use panic_probe as _;

extern crate alloc;

mod proto;
use proto::{decode_proto_msg, encode_proto};

use core::alloc::Layout;
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    use super::*;
    use alloc_cortex_m::CortexMHeap;
    use defmt::{error, info};
    use embedded_hal::serial::Read;
    use stm32f4xx_hal::{
        gpio::{Output, PD12, PD13, PD14, PD15},
        pac,
        prelude::*,
        serial::{Rx, Serial, Tx},
        timer::MonoTimerUs,
    };

    use alloc::vec::Vec;
    use core::fmt::Write;

    #[global_allocator]
    static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

    pub struct Hc05 {
        pub rx: Rx<pac::USART2>,
        pub tx: Tx<pac::USART2>,
    }

    #[shared]
    struct Shared {
        hc05: Hc05,
    }

    #[local]
    struct Local {
        orange_led: PD13<Output>,
        red_led: PD14<Output>,
        green_led: PD12<Output>,
        blue_led: PD15<Output>,
        cmd: Vec<u8>,
    }

    #[monotonic(binds = TIM2, default = true)]
    type MicrosecMono = MonoTimerUs<pac::TIM2>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        info!("Yeet!");

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
        let mut orange_led = gpiod.pd13.into_push_pull_output();
        let mut red_led = gpiod.pd14.into_push_pull_output();
        let mut green_led = gpiod.pd12.into_push_pull_output();
        let mut blue_led = gpiod.pd15.into_push_pull_output();
        orange_led.set_low();
        red_led.set_low();
        green_led.set_low();
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

        let mono = ctx.device.TIM2.monotonic_us(&clocks);
        tick::spawn().ok();
        (
            Shared { hc05 },
            Local {
                orange_led,
                red_led,
                green_led,
                blue_led,
                cmd: Vec::new(),
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

    #[task(binds = USART2, local=[red_led, green_led, blue_led, cmd], shared = [hc05])]
    fn usart2(mut ctx: usart2::Context) {
        ctx.local.blue_led.set_high();
        let mut write_back = false;
        ctx.shared.hc05.lock(|h| {
            match h.rx.read() {
                Ok(b) => {
                    let c = b as char;
                    ctx.local.cmd.push(b);
                    ctx.local.green_led.set_high();
                    ctx.local.red_led.set_low();
                    if c == '\n' {
                        write_back = true;
                    }
                }
                Err(_) => {
                    ctx.local.red_led.set_high();
                    ctx.local.green_led.set_low();
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

    #[task(local = [orange_led])]
    fn tick(ctx: tick::Context) {
        ctx.local.orange_led.toggle();
    }
}
