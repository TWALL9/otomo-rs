#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(type_alias_impl_trait)]

extern crate alloc;

mod controls;
// mod navigation;
mod loggers;
mod proto;

#[cfg(feature = "defmt_logger")]
#[cfg(not(feature = "null_logger"))]
#[cfg(not(feature = "serial_logger"))]
use loggers::defmt_logger as logger;

#[cfg(feature = "null_logger")]
#[cfg(not(feature = "defmt_logger"))]
#[cfg(not(feature = "serial_logger"))]
use loggers::null_logger as logger;

#[cfg(feature = "serial_logger")]
#[cfg(not(feature = "defmt_logger"))]
#[cfg(not(feature = "null_logger"))]
use loggers::serial_logger as logger;

use kiss_encoding::decode::{DataFrame, DecodedVal};

use alloc::vec::Vec;
use core::alloc::Layout;

use embedded_hal_async::delay::DelayUs;
use rtic_monotonics::{systick::*, Monotonic};

#[cfg(feature = "defmt_logger")]
use panic_probe as _;

#[cfg(not(feature = "defmt_logger"))]
use panic_halt as _;

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    panic!("OOM");
}

const NAME: &str = env!("CARGO_PKG_NAME");
const VERSION: &str = env!("CARGO_PKG_VERSION");

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [UART4, UART5])]
mod app {
    use super::*;

    use controls::joystick::joystick_tank_controls;
    use otomo_hardware::{
        led::{BlueLed, GreenLed, OrangeLed, RedLed},
        motors::{
            encoder::QuadratureEncoder,
            pololu_driver::{LeftDrive, RightDrive},
            Encoder, MotorEffort, OpenLoopDrive,
        },
        qei::{LeftQei, RightQei},
        EStopPressed, FanPin, MonoTimer, OtomoHardware, UsbSerial,
    };
    use proto::{decode_proto_msg, top_msg::Msg, DriveResponse, MotorState, RobotState, TopMsg};
    use usb_device::UsbError;

    use alloc_cortex_m::CortexMHeap;
    use fugit::{ExtU32, TimerInstantU32};
    use heapless::mpmc::Q32;

    use log::{error, info, warn};

    #[global_allocator]
    static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

    pub struct Motors {
        pub left: LeftDrive,
        pub right: RightDrive,
    }

    #[shared]
    struct Shared {
        green_led: GreenLed,
        _orange_led: OrangeLed,
        _red_led: RedLed,
        usb_serial: UsbSerial,
        cmd_queue: Q32<TopMsg>,
        motors: Motors,
        fan: FanPin,
    }

    #[local]
    struct Local {
        blue_led: BlueLed,
        usb_cmd: Vec<u8>,
        usb_decoder: DataFrame,
        left_encoder: QuadratureEncoder<LeftQei>,
        right_encoder: QuadratureEncoder<RightQei>,
        e_stop: EStopPressed,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
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

        let mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(device.systick, 168_000_000, mono_token);

        let mut motors = Motors {
            left: device.left_motor,
            right: device.right_motor,
        };

        motors.left.set_enable(true);
        motors.right.set_enable(true);

        #[cfg(feature = "serial_logger")]
        let mut logger = device.dbg_serial;
        #[cfg(not(feature = "serial_logger"))]
        let logger = logger::LoggerType;

        #[cfg(feature = "serial_logger")]
        {
            use core::fmt::Write;
            writeln!(&mut logger, "asdf\r").unwrap();
        }

        logger::init(logger);
        info!("{} v{}", NAME, VERSION);

        heartbeat::spawn().ok();

        (
            Shared {
                green_led: device.green_led,
                _orange_led: device.orange_led,
                _red_led: device.red_led,
                usb_serial: device.usb_serial,
                cmd_queue: Q32::<TopMsg>::new(),
                motors,
                fan: device.fan_motor,
            },
            Local {
                blue_led: device.blue_led,
                usb_cmd: Vec::new(),
                usb_decoder: DataFrame::new(),
                left_encoder: device.left_encoder,
                right_encoder: device.right_encoder,
                e_stop: device.e_stop,
            },
        )
    }

    #[idle()]
    fn idle(_ctx: idle::Context) -> ! {
        info!("idle!");

        loop {
            rtic::export::wfi();
        }
    }

    #[task(priority = 6, local = [left_encoder, right_encoder, blue_led, e_stop], shared = [cmd_queue, usb_serial, motors, fan])]
    async fn heartbeat(ctx: heartbeat::Context) {
        // info!("heartbeat!");

        let heartbeat::SharedResources {
            mut cmd_queue,
            mut usb_serial,
            mut motors,
            mut fan,
            __rtic_internal_marker,
        } = ctx.shared;

        let left_encoder = ctx.local.left_encoder;
        let right_encoder = ctx.local.right_encoder;
        let blue_led = ctx.local.blue_led;
        let e_stop = ctx.local.e_stop;

        let mut last_switch = Systick::now();

        loop {
            let now = Systick::now();

            let left_velocity = left_encoder.get_velocity(now);
            let right_velocity = right_encoder.get_velocity(now);

            let fan_state = (&mut fan).lock(|fan| fan.is_set_high());

            let left_state = MotorState {
                angular_velocity: left_velocity,
                encoder: left_encoder.get_position(),
            };
            let right_state = MotorState {
                angular_velocity: right_velocity,
                encoder: right_encoder.get_position(),
            };

            info!(
                "motor states: left: {:?}, right: {:?}",
                left_state, right_state
            );

            let state_msg = TopMsg {
                msg: Some(Msg::State(RobotState {
                    left_motor: Some(left_state),
                    right_motor: Some(right_state),
                    fan_on: fan_state,
                    e_stop: e_stop.is_low(),
                })),
            };

            (&mut cmd_queue, &mut usb_serial, &mut motors, &mut fan).lock(
                |q, usb_serial, motors, fan| {
                    let dequeued = if let Some(msg) = q.dequeue() {
                        match msg.msg {
                            Some(Msg::Joystick(j)) => {
                                let (left_drive, right_drive) =
                                    joystick_tank_controls(j.speed, j.heading);
                                info!(
                                    "received directions: ({:?}, {:?}), ({:?}, {:?})",
                                    j.speed, j.heading, left_drive, right_drive
                                );
                                motors.right.drive(right_drive);
                                motors.left.drive(left_drive);
                                true
                            }
                            Some(Msg::DiffDrive(d)) => {
                                // info!("dequeued command: {:?}", d);
                                let left_drive = controls::motor_math::rad_s_to_duty(d.left_motor);
                                let right_drive =
                                    controls::motor_math::rad_s_to_duty(d.right_motor);

                                motors.left.drive(left_drive);
                                motors.right.drive(right_drive);
                                true
                            }
                            Some(Msg::Fan(f)) => {
                                if f.on {
                                    fan.set_high();
                                } else {
                                    fan.set_low();
                                }
                                true
                            }
                            Some(_) => false,
                            None => false,
                        }
                    } else {
                        false
                    };

                    // USB serial needs to be free in order to write to it
                    // Note that this triggers the USB ISR???
                    // info!("usb polling");
                    // if usb_serial.poll() {
                    //     info!("poll true");
                    //     return;
                    // } else {
                    //     info!("poll false");
                    // }

                    let mut write_buf = Vec::new();
                    // if dequeued {
                    //     let response = TopMsg {
                    //         msg: Some(Msg::DriveResponse(DriveResponse {
                    //             ok: dequeued
                    //         })),
                    //     };

                    //     let ret_msg = proto::encode_proto(response).unwrap_or_default();
                    //     if let Ok(ret_kiss) = kiss_encoding::encode::encode(0, &ret_msg) {
                    //         write_buf.extend_from_slice(&ret_kiss);
                    //     }
                    // }

                    if let Ok(state_buf) = proto::encode_proto(state_msg) {
                        if let Ok(state_kiss) = kiss_encoding::encode::encode(0, &state_buf) {
                            write_buf.extend_from_slice(&state_kiss);
                        }
                    }

                    let mut offset = 0;
                    let count = write_buf.len();
                    while offset < count {
                        match usb_serial.write(&write_buf[offset..count]) {
                            Ok(len) => {
                                // info!("wrote {} to usb", len);
                                offset += len;
                            }
                            Err(UsbError::WouldBlock) => {}
                            Err(e) => error!("Cannot write to usb: {:?}", e),
                        };
                    }
                },
            );

            if let Some(dur) = now.checked_duration_since(last_switch) {
                if dur.to_millis() >= 1000 {
                    blue_led.toggle();
                    last_switch = now;
                }
            }

            Systick::delay(20.millis()).await;
        }
    }

    #[task(priority = 4, binds = OTG_FS, local = [usb_decoder, usb_cmd], shared = [usb_serial, green_led, cmd_queue])]
    fn usb_fs(ctx: usb_fs::Context) {
        // info!("usb_fs");
        let usb_fs::SharedResources {
            mut usb_serial,
            mut green_led,
            mut cmd_queue,
            __rtic_internal_marker,
        } = ctx.shared;

        let decoder = ctx.local.usb_decoder;
        let usb_cmd = ctx.local.usb_cmd;

        (&mut usb_serial, &mut green_led, &mut cmd_queue).lock(
            |usb_serial, green_led, cmd_queue| {
                let mut buf = [0_u8; 64];
                let mut msg_complete = false;

                // USB serial needs to have an event to be worth reading
                if !usb_serial.poll() {
                    warn!("No event for USB ISR?");
                    return;
                }

                match usb_serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        green_led.set_low();

                        for b in buf[0..count].iter() {
                            match decoder.decode_byte(*b) {
                                Ok(Some(DecodedVal::Data(u))) => usb_cmd.push(u),
                                Ok(Some(DecodedVal::EndFend)) => {
                                    msg_complete = true;
                                }
                                Ok(_) => (),
                                Err(d) => {
                                    usb_cmd.clear();
                                    error!("could not decode: {:?}", d);
                                }
                            };
                        }
                    }
                    Ok(_) => (),
                    Err(UsbError::WouldBlock) => (),
                    Err(_) => {
                        green_led.set_high();
                        usb_cmd.clear();
                        *decoder = DataFrame::new();
                        error!("USB read");
                    }
                }

                if msg_complete {
                    match decode_proto_msg::<TopMsg>(usb_cmd.as_slice()) {
                        Ok(t) => {
                            if cmd_queue.enqueue(t).is_err() {
                                error!("can't enqueue new command");
                            }
                        }
                        Err(e) => error!("Proto decode error: {}", e),
                    };

                    usb_cmd.clear();
                }
            },
        );
    }

    // #[task(priority = 3, shared = [ultrasonics], local = [delay])]
    // fn trigger(mut ctx: trigger::Context, now: TimerInstantU32<1_000_000>) {
    //     let dur: Duration<u32, 1, 1_000_000> = 20000.micros();
    //     ctx.shared.ultrasonics.lock(|us| {
    //         us.left.start_trigger();
    //         us.right.start_trigger();
    //         ctx.local.delay.delay_us(5_u8);
    //         us.left.finish_trigger();
    //         us.right.finish_trigger();
    //         us.counter.start(dur).unwrap();
    //     });

    //     let next_trigger = now + 50.millis();
    //     trigger::spawn_after(50.millis(), next_trigger).unwrap();
    // }

    // #[task(priority = 2, binds = EXTI15_10, shared = [ultrasonics, red_led], local = [l_done, r_done])]
    // fn echo_line(ctx: echo_line::Context) {
    //     let l_done = ctx.local.l_done;
    //     let r_done = ctx.local.r_done;

    //     let echo_line::SharedResources {
    //         mut ultrasonics,
    //         mut red_led,
    //     } = ctx.shared;

    //     (&mut ultrasonics, &mut red_led).lock(|ultrasonics, red_led| {
    //         red_led.toggle();

    //         let now = ultrasonics.counter.now();
    //         let l_distance = ultrasonics.left.check_distance(now);
    //         let r_distance = ultrasonics.right.check_distance(now);

    //         if let Some(distance) = l_distance {
    //             info!("Distance left: {}", distance);
    //             *l_done = true;
    //         }

    //         if let Some(distance) = r_distance {
    //             info!("Distance right: {}", distance);
    //             *r_done = true;
    //         }

    //         // TODO FIXME: this is an extremely cursed usage of the ISR
    //         // It constantly spams _this_ ISR at a low priority until all ultrasonics have reported.
    //         // This means that I'll be burning CPU cycles every ~3ms out of 50ms.
    //         // the MCU won't go into idle and send commands to the motors.
    //         // Although reading the RTT logs...it actually looks okay???
    //         // I think the ultrasonics should be in different ISR's anyway though, just don't want to use
    //         // TOO many EXTI lines
    //         if *l_done && *r_done {
    //             *l_done = false;
    //             *r_done = false;
    //             ultrasonics.right.clear_interrupt();
    //         }
    //     });
    // }

    // #[task(binds = TIM4, local = [pwm])]
    // fn tim4(ctx: tim4::Context) {
    //     // WHY DOES RTIC NOT CLEAR THE TIMER INTERRUPTS???
    //     ctx.local.pwm.deref_mut().clear_interrupt(Event::C1);
    // }
}
