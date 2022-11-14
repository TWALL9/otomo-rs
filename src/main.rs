#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

//mod encoder;
mod hbridge;
mod motors;
mod proto;
mod ultrasonic;

use alloc::vec::Vec;
use core::alloc::Layout;

use defmt_rtt as _;
use panic_probe as _;

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    panic!("OOM");
}

const NAME: &str = env!("CARGO_PKG_NAME");
const VERSION: &str = env!("CARGO_PKG_VERSION");

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1, USART3])]
mod app {
    use super::*;

    use hbridge::HBridge;
    use motors::{joystick_tank_controls, OpenLoopDrive};
    use proto::{decode_proto_msg, top_msg::Msg, Joystick};
    use ultrasonic::Hcsr04;

    use alloc_cortex_m::CortexMHeap;
    use defmt::{error, info};
    use embedded_hal::serial::Read;
    use heapless::spsc::{Consumer, Producer, Queue};
    use stm32f4xx_hal::{
        gpio::{Edge, Input, Output, PushPull, PC10, PD12, PD13, PD14, PD15},
        pac::{TIM2, TIM3, TIM4, USART2},
        prelude::*,
        serial::{Rx, Serial, Tx},
        timer::{MonoTimerUs, SysDelay, Timer3, Timer4},
    };

    #[global_allocator]
    static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

    pub struct SerialCmd {
        rx: Rx<USART2>,
        _tx: Tx<USART2>,
        cmd: Vec<u8>,
    }

    pub struct Motors {
        pub front_right: HBridge<TIM4, TIM4, 2, 3>,
        pub rear_right: HBridge<TIM4, TIM4, 0, 1>,
        pub front_left: HBridge<TIM3, TIM3, 2, 3>,
        pub rear_left: HBridge<TIM3, TIM3, 0, 1>,
    }

    #[shared]
    struct Shared {
        _green_led: PD12<Output<PushPull>>,
        _orange_led: PD13<Output<PushPull>>,
        _red_led: PD14<Output<PushPull>>,
        _blue_led: PD15<Output<PushPull>>,
        ultrasonic: Hcsr04<'B', 11, 'B', 10>,
    }

    #[local]
    struct Local {
        serial_cmd: SerialCmd,
        motors: Motors,
        cmd_tx: Producer<'static, Joystick, 5>,
        cmd_rx: Consumer<'static, Joystick, 5>,
        button: PC10<Input>,
        delay: SysDelay,
    }

    #[monotonic(binds = TIM2, default = true)]
    type MicrosecMono = MonoTimerUs<TIM2>;

    // type PwmPin = PwmHz<TIM4, (Ch<0>, Ch<1>), (Pin<'D', 12, Alternate<2>>, Pin<'D', 13, Alternate<2>>)>;

    #[init(local = [q: Queue<Joystick, 5> = Queue::new()])]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
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
        let mut syscfg = ctx.device.SYSCFG.constrain();

        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();
        let mono = ctx.device.TIM2.monotonic_us(&clocks);

        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();
        let gpiod = ctx.device.GPIOD.split();

        // Status LED's
        let _green_led = gpiod.pd12.into_push_pull_output();
        let _orange_led = gpiod.pd13.into_push_pull_output();
        let _red_led = gpiod.pd14.into_push_pull_output();
        let _blue_led = gpiod.pd15.into_push_pull_output();

        let tx_pin = gpioa.pa2.into_alternate();
        // let tx_pin = stm32f4xx_hal::gpio::NoPin;
        let rx_pin = gpioa.pa3.into_alternate();
        let mut serial =
            Serial::new(ctx.device.USART2, (tx_pin, rx_pin), 38400.bps(), &clocks).unwrap();
        serial.listen(stm32f4xx_hal::serial::Event::Rxne);
        let (_tx, rx) = serial.split();
        let serial_cmd = SerialCmd {
            _tx,
            rx,
            cmd: Vec::new(),
        };

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
            gpiob.pb6.into_alternate(),
            gpiob.pb7.into_alternate(),
            gpiob.pb8.into_alternate(),
            gpiob.pb9.into_alternate(),
        );
        let pwm4 = tim4.pwm_hz(tim4_pins, 10.kHz());

        // pwm4.deref_mut().listen(Event::C1);
        // Add C2 event as well?
        let (rear_right_a, rear_right_b, front_right_a, front_right_b) = pwm4.split();
        let rear_right = HBridge::new(rear_right_a, rear_right_b);
        let front_right = HBridge::new(front_right_a, front_right_b);

        let motors = Motors {
            front_right,
            rear_right,
            front_left,
            rear_left,
        };

        let (cmd_tx, cmd_rx) = ctx.local.q.split();

        let trig_pin = gpiob.pb11.into_push_pull_output();
        let mut echo_pin = gpiob.pb10.into_floating_input();
        echo_pin.make_interrupt_source(&mut syscfg);
        echo_pin.enable_interrupt(&mut ctx.device.EXTI);
        echo_pin.trigger_on_edge(&mut ctx.device.EXTI, Edge::RisingFalling);

        let mut button = gpioc.pc10.into_pull_up_input();
        button.make_interrupt_source(&mut syscfg);
        button.enable_interrupt(&mut ctx.device.EXTI);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);

        let ultrasonic = Hcsr04::new(trig_pin, echo_pin);
        let delay = ctx.core.SYST.delay(&clocks);

        tick::spawn().ok();

        (
            Shared {
                _green_led,
                _orange_led,
                _red_led,
                _blue_led,
                ultrasonic,
            },
            Local {
                serial_cmd,
                motors,
                cmd_tx,
                cmd_rx,
                button,
                delay,
            },
            init::Monotonics(mono),
        )
    }

    #[idle(local=[cmd_rx, motors])]
    fn idle(ctx: idle::Context) -> ! {
        info!("idle!");
        loop {
            tick::spawn_after(500.millis()).ok();
            if let Some(j) = ctx.local.cmd_rx.dequeue() {
                let (left_drive, right_drive) = joystick_tank_controls(j.speed, j.heading);
                // info!("received directions: ({:?}, {:?})", left_drive, left_drive);
                ctx.local.motors.front_right.drive(right_drive);
                ctx.local.motors.rear_right.drive(right_drive);
                ctx.local.motors.front_left.drive(left_drive);
                ctx.local.motors.rear_left.drive(left_drive);
            }
            rtic::export::wfi();
        }
    }

    #[task(binds = USART2, local=[serial_cmd, cmd_tx])]
    fn usart2(ctx: usart2::Context) {
        let serial_cmd = ctx.local.serial_cmd;
        let cmd_tx = ctx.local.cmd_tx;
        let mut msg_complete = false;
        if let Ok(b) = serial_cmd.rx.read() {
            // info!("rcvd: {:02x}", b);
            if b == 0xFF {
                // info!("complete");
                msg_complete = true;
            } else {
                serial_cmd.cmd.push(b);
            }
        } else {
            error!("uart fail");
            serial_cmd.cmd.clear();
        }

        if msg_complete {
            match decode_proto_msg::<proto::TopMsg>(serial_cmd.cmd.as_slice()) {
                Ok(t) => {
                    // info!("decoded");
                    if let Some(Msg::Joystick(j)) = t.msg {
                        if cmd_tx.ready() {
                            cmd_tx.enqueue(j).unwrap();
                        } else {
                            error!("can't enqueue");
                        }
                    }
                }
                Err(e) => error!("Proto decode error: {}", e),
            };

            serial_cmd.cmd.clear();
        }

        if serial_cmd.cmd.len() > 100 {
            serial_cmd.cmd.clear();
        }
    }

    #[task(shared = [_blue_led, ultrasonic], local = [delay])]
    fn tick(mut ctx: tick::Context) {
        ctx.shared._blue_led.lock(|led| led.toggle());
        ctx.shared.ultrasonic.lock(|us| us.start_trigger());
        //stop_trig::spawn_after(10.micros()).ok();
        ctx.local.delay.delay_us(10_u8);
        ctx.shared.ultrasonic.lock(|us| us.finish_trigger());
    }

    #[task(shared = [ultrasonic])]
    fn stop_trig(mut ctx: stop_trig::Context) {
        ctx.shared.ultrasonic.lock(|us| us.finish_trigger());
    }

    #[task(binds = EXTI15_10, shared = [ultrasonic], local=[button])]
    fn echo_line(mut ctx: echo_line::Context) {
        info!("ISR!");
        let distance = ctx.shared.ultrasonic.lock(|us| us.check_distance());
        if ctx.local.button.is_low() {
            info!("button low");
        }
        ctx.local.button.clear_interrupt_pending_bit();
        if let Some(distance) = distance {
            info!("Distance found: {}", distance);
        }
    }

    // #[task(binds = TIM4, local = [pwm])]
    // fn tim4(ctx: tim4::Context) {
    //     // WHY DOES RTIC NOT CLEAR THE TIMER INTERRUPTS???
    //     ctx.local.pwm.deref_mut().clear_interrupt(Event::C1);
    // }
}
