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

use loggers::Level;

use kiss_encoding::decode::{DataFrame, DecodedVal};

use alloc::vec::Vec;
use core::alloc::Layout;

use rtic_monotonics::{stm32::Tim2 as Mono, stm32::*, Monotonic};
use rtic_sync::{
    channel::{Receiver, Sender},
    make_channel,
};

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

    use controls::{joystick::joystick_tank_controls, motor_math::rad_s_to_duty, pid::PidCreator};
    use otomo_hardware::{
        led::{BlueLed, GreenLed, OrangeLed, RedLed},
        motors::{
            current_monitor::DefaultCurrentMonitor,
            encoder::QuadratureEncoder,
            pololu_driver::{LeftDrive, RightDrive},
            Encoder, OpenLoopDrive,
        },
        qei::{LeftQei, RightQei},
        EStopPressed, FanPin, OtomoHardware, TaskToggle0, TaskToggle1, TaskToggle2, UsbSerial,
    };
    use proto::{decode_proto_msg, top_msg::Msg, MotorState, RobotState, TopMsg};
    use usb_device::UsbError;

    use alloc_cortex_m::CortexMHeap;
    use fugit::ExtU64;

    use log::{error, info, warn};

    const CMD_QUEUE_CAP: usize = 5;
    const RESP_QUEUE_CAP: usize = 5;
    const MOTOR_QUEUE_CAP: usize = 2;

    #[global_allocator]
    static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

    pub struct MotorTaskLocal {
        left: LeftDrive,
        right: RightDrive,
        left_encoder: QuadratureEncoder<LeftQei>,
        right_encoder: QuadratureEncoder<RightQei>,
        current_monitor: DefaultCurrentMonitor,
        task_toggle: TaskToggle2,
        motor_cmd_r: Receiver<'static, proto::top_msg::Msg, MOTOR_QUEUE_CAP>,
        feedback_s: Sender<'static, TopMsg, RESP_QUEUE_CAP>,
    }

    pub struct HeartbeatTaskLocal {
        task_toggle: TaskToggle0,
        motor_cmd_s: Sender<'static, proto::top_msg::Msg, MOTOR_QUEUE_CAP>,
        cmd_r: Receiver<'static, TopMsg, CMD_QUEUE_CAP>,
        feedback_r: Receiver<'static, TopMsg, RESP_QUEUE_CAP>,
        e_stop: EStopPressed,
        fan: FanPin,
    }

    pub struct UsbTaskLocal {
        task_toggle: TaskToggle1,
        cmd_s: Sender<'static, TopMsg, CMD_QUEUE_CAP>,
        usb_cmd: Vec<u8>,
        usb_decoder: DataFrame,
    }

    #[shared]
    struct Shared {
        usb_serial: UsbSerial,
        usb_connected: bool,
    }

    #[local]
    struct Local {
        green_led: GreenLed,
        _blue_led: BlueLed,
        _orange_led: OrangeLed,
        red_led: RedLed,
        motor_task_local: MotorTaskLocal,
        heartbeat_task_local: HeartbeatTaskLocal,
        usb_task_local: UsbTaskLocal,
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

        // Initialise queues
        let (cmd_s, cmd_r) = make_channel!(TopMsg, CMD_QUEUE_CAP);
        let (resp_s, resp_r) = make_channel!(TopMsg, RESP_QUEUE_CAP);
        let (motor_cmd_s, motor_cmd_r) = make_channel!(proto::top_msg::Msg, MOTOR_QUEUE_CAP);

        let device = OtomoHardware::init(ctx.device, ctx.core);

        let mono_token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
        Mono::start(84_000_000, mono_token);

        let mut motor_task_local = MotorTaskLocal {
            left: device.left_motor,
            right: device.right_motor,
            left_encoder: device.left_encoder,
            right_encoder: device.right_encoder,
            current_monitor: device.current_monitor,
            task_toggle: device.task_toggle_2,
            motor_cmd_r,
            feedback_s: resp_s.clone(),
        };

        motor_task_local.left.set_enable(true);
        motor_task_local.right.set_enable(true);

        let usb_task_local = UsbTaskLocal {
            task_toggle: device.task_toggle_1,
            cmd_s,
            usb_cmd: Vec::with_capacity(256),
            usb_decoder: DataFrame::new(),
        };

        let heartbeat_task_local = HeartbeatTaskLocal {
            task_toggle: device.task_toggle_0,
            motor_cmd_s,
            cmd_r,
            feedback_r: resp_r,
            e_stop: device.e_stop,
            fan: device.fan_motor,
        };

        #[cfg(feature = "serial_logger")]
        let mut logger = device.dbg_serial;
        #[cfg(not(feature = "serial_logger"))]
        let logger = logger::LoggerType;

        #[cfg(feature = "serial_logger")]
        {
            use core::fmt::Write;
            writeln!(&mut logger, "asdf\r").unwrap();
        }

        logger::init(logger, Level::Debug);
        info!("{} v{}", NAME, VERSION);

        heartbeat::spawn().ok();
        motor_task::spawn().ok();

        (
            Shared {
                usb_serial: device.usb_serial,
                usb_connected: false,
            },
            Local {
                green_led: device.green_led,
                _blue_led: device.blue_led,
                _orange_led: device.orange_led,
                red_led: device.red_led,
                motor_task_local,
                heartbeat_task_local,
                usb_task_local,
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

    #[task(priority = 4, local = [heartbeat_task_local, green_led], shared = [usb_serial, usb_connected])]
    async fn heartbeat(ctx: heartbeat::Context) {
        let heartbeat::SharedResources {
            mut usb_serial,
            mut usb_connected,
            __rtic_internal_marker,
        } = ctx.shared;

        let green_led = ctx.local.green_led;
        let task_toggle = &mut ctx.local.heartbeat_task_local.task_toggle;
        let motor_cmd_s = &mut ctx.local.heartbeat_task_local.motor_cmd_s;
        let cmd_r = &mut ctx.local.heartbeat_task_local.cmd_r;
        let feedback_r = &mut ctx.local.heartbeat_task_local.feedback_r;

        let mut last_switch = Mono::now();

        loop {
            task_toggle.set_high();

            let now = Mono::now();

            let _e_stop_pressed = ctx.local.heartbeat_task_local.e_stop.is_low();
            let fan = &mut ctx.local.heartbeat_task_local.fan;

            if let Ok(msg) = cmd_r.try_recv() {
                match msg.msg {
                    Some(Msg::Joystick(_) | Msg::DiffDrive(_) | Msg::Pid(_)) => {
                        motor_cmd_s.send(msg.msg.unwrap()).await.unwrap();
                    }
                    Some(Msg::Fan(f)) => {
                        if f.on {
                            fan.set_high();
                        } else {
                            fan.set_low();
                        }
                    }
                    Some(_) => warn!("unrecognized message! {:?}", msg.which_msg() as u8),
                    None => (),
                }
            }

            let state_msg = feedback_r.recv().await.unwrap();

            (&mut usb_serial, &mut usb_connected).lock(|usb_serial, usb_connected| {
                // USB serial needs to be free in order to write to it
                // Note that this triggers the USB ISR???

                if *usb_connected == false {
                    return;
                }

                let mut write_buf = Vec::new();

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
            });

            if let Some(dur) = now.checked_duration_since(last_switch) {
                if dur.to_millis() >= 1000 {
                    green_led.toggle();
                    last_switch = now;
                }
            }

            task_toggle.set_low();
            Mono::delay(20.millis()).await;
        }
    }

    #[task(priority = 5, local = [motor_task_local])]
    async fn motor_task(ctx: motor_task::Context) {
        let left_motor = &mut ctx.local.motor_task_local.left;
        let right_motor = &mut ctx.local.motor_task_local.right;
        let left_encoder = &mut ctx.local.motor_task_local.left_encoder;
        let right_encoder = &mut ctx.local.motor_task_local.right_encoder;
        let current_monitor = &mut ctx.local.motor_task_local.current_monitor;
        let task_toggle = &mut ctx.local.motor_task_local.task_toggle;
        let motor_cmd_r = &mut ctx.local.motor_task_local.motor_cmd_r;
        let feedback_s = &mut ctx.local.motor_task_local.feedback_s;

        let mut left_pid = PidCreator::<f32>::new()
            .set_p(0.0)
            .set_i(0.0)
            .set_d(0.0)
            .create_controller();
        let mut right_pid = left_pid.clone();

        let mut prev_left_error_state = false;
        let mut prev_right_error_state = false;

        loop {
            task_toggle.set_high();

            let now = Mono::now();

            let left_error = left_motor.is_in_error();
            let right_error = right_motor.is_in_error();

            if prev_left_error_state != left_error || prev_right_error_state != right_error {
                if left_error || right_error {
                    warn!("Motor fault detected! {}, {}", left_error, right_error);
                    left_motor.set_enable(false);
                    right_motor.set_enable(false);
                } else {
                    warn!("Motor fault cleared! {}, {}", left_error, right_error);
                    left_motor.set_enable(true);
                    right_motor.set_enable(true);
                }

                prev_left_error_state = left_error;
                prev_right_error_state = right_error;
            }

            let left_velocity = left_encoder.get_velocity(now);
            let right_velocity = right_encoder.get_velocity(now);

            // TODO add these to the proto spec
            let (_left_current, _right_current) = current_monitor.get_currents();

            let left_state = MotorState {
                angular_velocity: left_velocity,
                encoder: left_encoder.get_position(),
            };
            let right_state = MotorState {
                angular_velocity: right_velocity,
                encoder: right_encoder.get_position(),
            };

            let state_msg = TopMsg {
                msg: Some(Msg::State(RobotState {
                    left_motor: Some(left_state),
                    right_motor: Some(right_state),
                    fan_on: false,
                    e_stop: (left_error || right_error),
                })),
            };

            task_toggle.set_low();
            feedback_s.send(state_msg).await.ok();

            if let Ok(msg) = motor_cmd_r.try_recv() {
                task_toggle.set_high();
                match msg {
                    Msg::Joystick(j) => {
                        let (left_drive, right_drive) = joystick_tank_controls(j.speed, j.heading);
                        info!(
                            "received directions: ({:?}, {:?}), ({:?}, {:?})",
                            j.speed, j.heading, left_drive, right_drive
                        );
                        right_motor.drive(right_drive);
                        left_motor.drive(left_drive);
                        // if !e_stop_pressed {
                        //     right_motor.drive(right_drive);
                        //     left_motor.drive(left_drive);
                        // }
                    }
                    Msg::DiffDrive(d) => {
                        // info!("dequeued command: {:?}", d);
                        // info!("new setpoint: {}, {}", d.left_motor, d.right_motor);
                        left_pid.set_setpoint(
                            d.left_motor,
                            Some(controls::motor_math::ACTUAL_MAX_SPEED_RAD_S),
                        );
                        right_pid.set_setpoint(
                            d.right_motor,
                            Some(controls::motor_math::ACTUAL_MAX_SPEED_RAD_S),
                        );
                        // if !e_stop_pressed {
                        //     left_pid.set_setpoint(
                        //         d.left_motor,
                        //         Some(controls::motor_math::ACTUAL_MAX_SPEED_RAD_S),
                        //     );
                        //     right_pid.set_setpoint(
                        //         d.right_motor,
                        //         Some(controls::motor_math::ACTUAL_MAX_SPEED_RAD_S),
                        //     );
                        // }
                    }
                    Msg::Pid(pid) => {
                        warn!("updating PID: {:?}", pid);
                        left_pid.update_terms(pid.p, pid.i, pid.d);
                        right_pid.update_terms(pid.p, pid.i, pid.d);
                    }
                    _ => error!("Unrecognized msg!"),
                };

                let next_left = left_pid.update(left_velocity);
                let next_right = right_pid.update(right_velocity);

                info!("left v: {}, next: {}", left_velocity, next_left);
                info!("right v: {}, next: {}", right_velocity, next_right);

                left_motor.drive(rad_s_to_duty(next_left));
                right_motor.drive(rad_s_to_duty(next_right));
            }

            // info!(
            //     "motor states: left: {:?}, right: {:?}",
            //     left_velocity, right_velocity
            // );

            task_toggle.set_low();
            Mono::delay(2.millis()).await;
        }
    }

    #[task(priority = 6, binds = OTG_FS, local = [usb_task_local, red_led], shared = [usb_serial, usb_connected])]
    fn usb_fs(ctx: usb_fs::Context) {
        // info!("usb_fs");
        let usb_fs::SharedResources {
            mut usb_serial,
            mut usb_connected,
            __rtic_internal_marker,
        } = ctx.shared;

        let decoder = &mut ctx.local.usb_task_local.usb_decoder;
        let usb_cmd = &mut ctx.local.usb_task_local.usb_cmd;
        let red_led = ctx.local.red_led;
        let task_toggle = &mut ctx.local.usb_task_local.task_toggle;
        let cmd_s = &mut ctx.local.usb_task_local.cmd_s;

        task_toggle.set_high();

        (&mut usb_serial, &mut usb_connected).lock(|usb_serial, usb_connected| {
            let mut buf = [0_u8; 64];
            let mut msg_complete = false;

            // USB serial needs to have an event to be worth reading
            if !usb_serial.poll() {
                warn!("No event for USB ISR?");
                return;
            }

            match usb_serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    red_led.set_low();

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
                    red_led.set_high();
                    usb_cmd.clear();
                    *decoder = DataFrame::new();
                    error!("USB read");
                    *usb_connected = false;
                }
            }

            if msg_complete {
                match decode_proto_msg::<TopMsg>(usb_cmd.as_slice()) {
                    Ok(t) => {
                        if cmd_s.try_send(t).is_ok() {
                            *usb_connected = true;
                        } else {
                            error!("can't enqueue new command");
                        }
                    }
                    Err(e) => error!("Proto decode error: {}", e),
                };

                usb_cmd.clear();
            }
        });

        task_toggle.set_low();
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
