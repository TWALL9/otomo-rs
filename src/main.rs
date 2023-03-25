#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

//mod encoder;
mod hbridge;
mod motors;
// mod navigation;
mod proto;

use kiss_encoding::decode::{DataFrame, DecodedVal};

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
    use otomo_hardware::{
        led::{BlueLed, GreenLed, OrangeLed, RedLed},
        ultrasonic::Ultrasonics,
        MonoTimer, OtomoHardware, UsbSerial,
    };
    use proto::{decode_proto_msg, top_msg::Msg, Joystick};
    use usb_device::UsbError;

    use alloc_cortex_m::CortexMHeap;
    use defmt::{error, info};
    use embedded_hal::serial::Read;
    use fugit::{Duration, ExtU32, TimerInstantU32};
    use heapless::spsc::{Consumer, Producer, Queue};
    use stm32f4xx_hal::{
        pac::{TIM3, TIM4, USART2},
        prelude::*,
        serial::{Rx, Tx},
        timer::SysDelay,
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
        green_led: GreenLed,
        _orange_led: OrangeLed,
        red_led: RedLed,
        blue_led: BlueLed,
        ultrasonics: Ultrasonics,
        usb_serial: UsbSerial,
    }

    #[local]
    struct Local {
        serial_cmd: SerialCmd,
        decoder: DataFrame,
        motors: Motors,
        cmd_tx: Producer<'static, Joystick, 8>,
        cmd_rx: Consumer<'static, Joystick, 8>,
        delay: SysDelay,
        l_done: bool,
        r_done: bool,
    }

    #[monotonic(binds = TIM2, default = true)]
    type MicrosecMono = MonoTimer;

    // type PwmPin = PwmHz<TIM4, (Ch<0>, Ch<1>), (Pin<'D', 12, Alternate<2>>, Pin<'D', 13, Alternate<2>>)>;

    #[init(local = [q: Queue<Joystick, 8> = Queue::new()])]
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
        // let dbgmcu = ctx.device.DBGMCU;
        // dbgmcu.cr.modify(|_, w| {
        ctx.device.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        // enabling the dma1 clock keeps one AHB bus master active, which prevents SRAM from reading as 0's
        // https://github.com/probe-rs/probe-rs/issues/350#issuecomment-740550519
        ctx.device.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());

        let device = OtomoHardware::init(ctx.device, ctx.core);

        let mut mono = device.mono;

        let (_tx, rx) = device.bt_serial.split();
        let serial_cmd = SerialCmd {
            _tx,
            rx,
            cmd: Vec::new(),
        };

        let (rear_left_a, rear_left_b, front_left_a, front_left_b) = device.pwm3.split();
        let rear_left = HBridge::new(rear_left_a, rear_left_b);
        let front_left = HBridge::new(front_left_a, front_left_b);

        // For capturing PWM cycles
        // pwm4.deref_mut().listen(Event::C1);
        // Add C2 event as well?
        let (rear_right_a, rear_right_b, front_right_a, front_right_b) = device.pwm4.split();
        let rear_right = HBridge::new(rear_right_a, rear_right_b);
        let front_right = HBridge::new(front_right_a, front_right_b);

        let motors = Motors {
            front_right,
            rear_right,
            front_left,
            rear_left,
        };

        let decoder = DataFrame::new();
        let (cmd_tx, cmd_rx) = ctx.local.q.split();

        trigger::spawn_after(1.secs(), mono.now()).unwrap();
        (
            Shared {
                green_led: device.green_led,
                _orange_led: device.orange_led,
                red_led: device.red_led,
                blue_led: device.blue_led,
                ultrasonics: device.ultrasonics,
                usb_serial: device.usb_serial,
            },
            Local {
                serial_cmd,
                decoder,
                motors,
                cmd_tx,
                cmd_rx,
                delay: device.delay,
                l_done: false,
                r_done: false,
            },
            init::Monotonics(mono),
        )
    }

    #[idle(local=[cmd_rx, motors])]
    fn idle(ctx: idle::Context) -> ! {
        info!("idle!");

        let motors = ctx.local.motors;
        loop {
            heartbeat::spawn_after(500.millis()).ok();
            if let Some(j) = ctx.local.cmd_rx.dequeue() {
                let (left_drive, right_drive) = joystick_tank_controls(j.speed, j.heading);
                info!(
                    "received directions: ({:?}, {:?}), ({:?}, {:?})",
                    j.speed, j.heading, left_drive, right_drive
                );
                motors.front_right.drive(right_drive);
                motors.rear_right.drive(right_drive);
                motors.front_left.drive(left_drive);
                motors.rear_left.drive(left_drive);
            }
            rtic::export::wfi();
        }
    }

    #[task(priority = 4, shared = [blue_led])]
    fn heartbeat(mut ctx: heartbeat::Context) {
        // info!("heartbeat!");
        ctx.shared.blue_led.lock(|b| b.toggle());
    }

    // #[task(priority = 5, binds = USART2, local=[serial_cmd, cmd_tx, decoder])]
    #[task(binds = USART2, local=[serial_cmd, cmd_tx, decoder])]
    fn usart2(ctx: usart2::Context) {
        let serial_cmd = ctx.local.serial_cmd;
        let cmd_tx = ctx.local.cmd_tx;
        let decoder = ctx.local.decoder;

        let mut msg_complete = false;
        if let Ok(b) = serial_cmd.rx.read() {
            match decoder.decode_byte(b) {
                Ok(Some(DecodedVal::Data(u))) => serial_cmd.cmd.push(u),
                Ok(Some(DecodedVal::EndFend)) => {
                    msg_complete = true;
                }
                _ => (),
            };
        } else {
            error!("uart fail");
            serial_cmd.cmd.clear();
        }

        if msg_complete {
            match decode_proto_msg::<proto::TopMsg>(serial_cmd.cmd.as_slice()) {
                Ok(t) => {
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

    #[task(priority = 3, shared = [ultrasonics], local = [delay])]
    fn trigger(mut ctx: trigger::Context, now: TimerInstantU32<1_000_000>) {
        let dur: Duration<u32, 1, 1_000_000> = 20000.micros();
        ctx.shared.ultrasonics.lock(|us| {
            us.left.start_trigger();
            us.right.start_trigger();
            ctx.local.delay.delay_us(5_u8);
            us.left.finish_trigger();
            us.right.finish_trigger();
            us.counter.start(dur).unwrap();
        });

        let next_trigger = now + 50.millis();
        trigger::spawn_after(50.millis(), next_trigger).unwrap();
    }

    #[task(priority = 2, binds = EXTI15_10, shared = [ultrasonics, red_led], local = [l_done, r_done])]
    fn echo_line(mut ctx: echo_line::Context) {
        let l_done = ctx.local.l_done;
        let r_done = ctx.local.r_done;

        ctx.shared.red_led.lock(|r| r.toggle());

        let (l_distance, r_distance) = ctx.shared.ultrasonics.lock(|us| {
            let now = us.counter.now();
            let l = us.left.check_distance(now);
            let r = us.right.check_distance(now);
            (l, r)
        });

        if let Some(distance) = l_distance {
            info!("Distance left: {}", distance);
            *l_done = true;
        }

        if let Some(distance) = r_distance {
            info!("Distance right: {}", distance);
            *r_done = true;
        }

        // TODO FIXME: this is an extremely cursed usage of the ISR
        // It constantly spams _this_ ISR at a low priority until all ultrasonics have reported.
        // This means that I'll be burning CPU cycles every ~3ms out of 50ms.
        // the MCU won't go into idle and send commands to the motors.
        // Although reading the RTT logs...it actually looks okay???
        // I think the ultrasonics should be in different ISR's anyway though, just don't want to use
        // TOO many EXTI lines
        if *l_done && *r_done {
            *l_done = false;
            *r_done = false;
            ctx.shared.ultrasonics.lock(|us| {
                us.right.clear_interrupt();
            });
        }
    }

    // #[task(binds=OTG_FS, shared=[usb_serial, green_led])]
    #[task(priority = 5, binds = OTG_FS, shared = [usb_serial, green_led])]
    fn usb_fs(ctx: usb_fs::Context) {
        let usb_fs::SharedResources {
            mut usb_serial,
            mut green_led,
        } = ctx.shared;

        (&mut usb_serial, &mut green_led).lock(|usb_serial, green_led| {
            let mut buf = [0_u8; 64];
            match usb_serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    green_led.set_low();
                    let resp = b"asdf";
                    if usb_serial.write(resp).is_err() {
                        error!("usb write");
                    }
                }
                Ok(_) => (),
                Err(UsbError::WouldBlock) => (),
                Err(_) => {
                    green_led.set_high();
                    error!("USB read");
                }
            }
        });
    }

    // #[task(binds = TIM4, local = [pwm])]
    // fn tim4(ctx: tim4::Context) {
    //     // WHY DOES RTIC NOT CLEAR THE TIMER INTERRUPTS???
    //     ctx.local.pwm.deref_mut().clear_interrupt(Event::C1);
    // }
}
