#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

use defmt_rtt as _;
use panic_probe as _;

// mod asdf;
// use crate::asdf::BigChungus;

extern crate alloc;

use core::alloc::Layout;
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    use alloc_cortex_m::CortexMHeap;
    use defmt::*;
    use embedded_hal::serial::Read;
    use stm32f4xx_hal::{
        gpio::{Output, PD13},
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
        led: PD13<Output>,
        count: i32,
        v: Vec<i32>,
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

        let mut v = Vec::new();
        v.push(4);
        info!("{:?}", v[0]);

        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

        let gpiod = ctx.device.GPIOD.split();
        let led = gpiod.pd13.into_push_pull_output();

        let gpioa = ctx.device.GPIOA.split();
        let tx_pin = gpioa.pa2.into_alternate();
        let rx_pin = gpioa.pa3.into_alternate();
        let mut serial =
            Serial::new(ctx.device.USART2, (tx_pin, rx_pin), 115200.bps(), &clocks).unwrap();
        serial.listen(stm32f4xx_hal::serial::Event::Rxne);
        let (tx, rx) = serial.split();
        let hc05 = Hc05 { tx, rx };

        let mono = ctx.device.TIM2.monotonic_us(&clocks);
        tick::spawn().ok();
        (
            Shared { hc05 },
            Local { led, count: 0, v },
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

    #[task(binds = USART2, shared = [hc05])]
    fn usart2(mut ctx: usart2::Context) {
        ctx.shared.hc05.lock(|h| {
            if let Ok(b) = h.rx.read() {
                let c = char::from(b);
                info!("read {:?}", c);
            };
        });
    }

    #[task(local = [led, count, v], shared = [hc05])]
    fn tick(mut ctx: tick::Context) {
        ctx.local.led.toggle();
        *ctx.local.count += 1;
        ctx.local.v.push(*ctx.local.count);
        info!("{:?}", ctx.local.v.last());
        ctx.shared
            .hc05
            .lock(|h| writeln!(h.tx, "{:?}\r", ctx.local.count).ok());
    }
}
