#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(type_alias_impl_trait)]
#![feature(stdarch_arm_barrier)]
#![feature(stdarch_arm_neon_intrinsics)]

extern crate alloc;

mod controls;
mod drivers;
mod logging;
mod proto;
mod time;

use logging::*;

use kiss_encoding::decode::{DataFrame, DecodedVal};

use core::alloc::Layout;

use rtic_monotonics::stm32::prelude::*;
use rtic_sync::{
    channel::{Receiver, Sender},
    make_channel,
};

use heapless::Vec as StaticVec;

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

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [UART4, UART5, CAN2_TX, CAN2_RX0])]
mod app {
    use super::*;

    rtic_monotonics::stm32_tim2_monotonic!(Mono, 1_000_000);

    use controls::{joystick::joystick_tank_controls, motor_math::rad_s_to_duty, pid::PidCreator};
    use otomo_hardware::{
        battery_monitor::DefaultBatteryMonitor,
        buzzer::{Buzzer, Notes},
        imu::lsm6dsox::Lsm6dsox,
        led::{BlueLed, GreenLed, OrangeLed, RedLed},
        motors::{
            encoder::QuadratureEncoder,
            pololu_driver::{LeftDrive, RightDrive},
            Encoder, OpenLoopDrive,
        },
        qei::{LeftQei, RightQei},
        EStopPressed, FanPin, OtomoHardware, TaskToggle0, TaskToggle1, TaskToggle2, TaskToggle3,
        TaskToggle4, TaskToggle5, UsbSerial,
    };
    use proto::{
        decode_proto_msg, top_msg::Msg, Battery as BatteryMsg, Imu as ImuMsg, MotorState,
        RobotState, TopMsg, Vector3,
    };
    use time::dur_from_millis;

    use usb_device::UsbError;

    use alloc_cortex_m::CortexMHeap;
    use fugit::ExtU64;

    use log::{error, info, warn};

    const CMD_QUEUE_CAP: usize = 5;
    const RESP_QUEUE_CAP: usize = 5;
    const MOTOR_QUEUE_CAP: usize = 2;
    const BUZZ_QUEUE_CAP: usize = 4;

    #[global_allocator]
    static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

    #[derive(Clone, Copy, Debug)]
    pub struct BuzzerTune {
        tone: Notes,
        millis: fugit::Duration<u64, 1, 1000000>,
        repeat: u32,
    }

    #[derive(Clone, Copy, Debug)]
    pub enum BuzzerMsg {
        Tune(BuzzerTune),
        Indef(Notes),
        Background(Option<Notes>),
        Mute,
    }

    impl Default for BuzzerMsg {
        fn default() -> Self {
            BuzzerMsg::Mute
        }
    }

    pub struct MotorTaskLocal {
        left: LeftDrive,
        right: RightDrive,
        e_stop: EStopPressed,
        left_encoder: QuadratureEncoder<LeftQei>,
        right_encoder: QuadratureEncoder<RightQei>,
        task_toggle: TaskToggle2,
        motor_cmd_r: Receiver<'static, proto::top_msg::Msg, MOTOR_QUEUE_CAP>,
        feedback_s: Sender<'static, TopMsg, RESP_QUEUE_CAP>,
        buzzer_s: Sender<'static, BuzzerMsg, BUZZ_QUEUE_CAP>,
        #[cfg(feature = "battery_task")]
        current_monitor: motors::current_monitor::DefaultCurrentMonitor,
    }

    pub struct HeartbeatTaskLocal {
        task_toggle: TaskToggle0,
        motor_cmd_s: Sender<'static, proto::top_msg::Msg, MOTOR_QUEUE_CAP>,
        buzzer_s: Sender<'static, BuzzerMsg, BUZZ_QUEUE_CAP>,
        cmd_r: Receiver<'static, TopMsg, CMD_QUEUE_CAP>,
        feedback_r: Receiver<'static, TopMsg, RESP_QUEUE_CAP>,
        fan: FanPin,
    }

    pub struct UsbTaskLocal {
        task_toggle: TaskToggle1,
        cmd_s: Sender<'static, TopMsg, CMD_QUEUE_CAP>,
        usb_cmd: StaticVec<u8, 256>,
        usb_decoder: DataFrame,
    }

    pub struct BuzzerTaskLocal {
        task_toggle: TaskToggle3,
        buzzer: Buzzer,
        buzzer_r: Receiver<'static, BuzzerMsg, BUZZ_QUEUE_CAP>,
    }

    pub struct BatteryTaskLocal {
        task_toggle: TaskToggle4,
        battery_monitor: DefaultBatteryMonitor,
        feedback_s: Sender<'static, TopMsg, RESP_QUEUE_CAP>,
        buzzer_s: Sender<'static, BuzzerMsg, BUZZ_QUEUE_CAP>,
    }

    pub struct ImuTaskLocal {
        task_toggle: TaskToggle5,
        imu: Lsm6dsox<stm32f4xx_hal::i2c::I2c1>,
        feedback_s: Sender<'static, TopMsg, RESP_QUEUE_CAP>,
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
        buzzer_task_local: BuzzerTaskLocal,
        battery_task_local: BatteryTaskLocal,
        imu_task_local: ImuTaskLocal,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Initialize heap
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024;
            static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe {
                let heap_start = &raw const HEAP as usize;
                ALLOCATOR.init(heap_start as usize, HEAP_SIZE)
            }
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

        let (buzzer_cmd_s, buzzer_cmd_r) = make_channel!(BuzzerMsg, BUZZ_QUEUE_CAP);

        let device = OtomoHardware::init(ctx.device, ctx.core);

        Mono::start(84_000_000);

        let mut motor_task_local = MotorTaskLocal {
            left: device.left_motor,
            right: device.right_motor,
            e_stop: device.e_stop,
            left_encoder: device.left_encoder,
            right_encoder: device.right_encoder,
            task_toggle: device.task_toggle_2,
            motor_cmd_r,
            feedback_s: resp_s.clone(),
            buzzer_s: buzzer_cmd_s.clone(),
            #[cfg(feature = "battery_task")]
            current_monitor: device.current_monitor,
        };

        motor_task_local.left.set_enable(true);
        motor_task_local.right.set_enable(true);

        let usb_task_local = UsbTaskLocal {
            task_toggle: device.task_toggle_1,
            cmd_s,
            usb_cmd: StaticVec::new(),
            usb_decoder: DataFrame::new(),
        };

        let buzzer_task_local = BuzzerTaskLocal {
            task_toggle: device.task_toggle_3,
            buzzer: device.buzzer,
            buzzer_r: buzzer_cmd_r,
        };

        let battery_task_local = BatteryTaskLocal {
            task_toggle: device.task_toggle_4,
            battery_monitor: device.battery_monitor,
            feedback_s: resp_s.clone(),
            buzzer_s: buzzer_cmd_s.clone(),
        };

        let heartbeat_task_local = HeartbeatTaskLocal {
            task_toggle: device.task_toggle_0,
            motor_cmd_s,
            cmd_r,
            feedback_r: resp_r,
            fan: device.fan_motor,
            buzzer_s: buzzer_cmd_s,
        };

        let imu_task_local = ImuTaskLocal {
            task_toggle: device.task_toggle_5,
            imu: device.imu,
            feedback_s: resp_s.clone(),
        };

        #[cfg(feature = "serial_logger")]
        logging::serial_logger::init(device.dbg_serial);

        logging::init(Level::Debug);
        info!("{} v{}", NAME, VERSION);

        heartbeat::spawn().ok();
        motor_task::spawn().ok();
        buzzer_task::spawn().ok();
        #[cfg(feature = "battery_task")]
        battery_task::spawn().ok();
        #[cfg(feature = "imu_task")]
        imu_task::spawn().ok();

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
                buzzer_task_local,
                battery_task_local,
                imu_task_local,
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
        let buzzer_s = &mut ctx.local.heartbeat_task_local.buzzer_s;

        let mut last_switch = Mono::now();

        let initial_buzz_1 = BuzzerMsg::Tune(BuzzerTune {
            tone: Notes::MiddleC,
            millis: dur_from_millis(400),
            repeat: 2,
        });

        let initial_buzz_2 = BuzzerMsg::Tune(BuzzerTune {
            tone: Notes::HighA,
            millis: dur_from_millis(600),
            repeat: 1,
        });

        buzzer_s.send(initial_buzz_1).await.unwrap();
        buzzer_s.send(initial_buzz_2).await.unwrap();

        loop {
            task_toggle.set_high();

            let now = Mono::now();

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
                    Some(Msg::Config(c)) => logging::set_level(c.new_level().into()),
                    Some(_) => warn!("unrecognized message! {:?}", msg.which_msg() as u8),
                    None => (),
                }
            }

            let mut msgs_sent: u8 = 0;
            while let Ok(state_msg) = feedback_r.try_recv() {
                (&mut usb_serial, &mut usb_connected).lock(|usb_serial, usb_connected| {
                    // info!("new msg");
                    // USB serial needs to be free in order to write to it
                    // Note that this triggers the USB ISR???

                    if !(*usb_connected) {
                        return;
                    }

                    let mut write_buf: StaticVec<u8, 256> = StaticVec::new();

                    if let Ok(state_buf) = proto::encode_proto(state_msg) {
                        if let Ok(state_kiss) = kiss_encoding::encode::encode(0, &state_buf) {
                            write_buf.extend_from_slice(&state_kiss).ok();
                        }
                    }

                    let mut offset = 0;
                    let count = write_buf.len();
                    while offset < count {
                        match usb_serial.write(&write_buf[offset..count]) {
                            Ok(len) => {
                                // enhanced_log!("wrote {} to usb", len);
                                offset += len;
                            }
                            Err(UsbError::WouldBlock) => {}
                            Err(e) => error!("Cannot write to usb: {:?}", e),
                        };
                    }
                });

                msgs_sent += 1;
                if msgs_sent >= 5 {
                    break;
                }
            }

            if let Some(dur) = now.checked_duration_since(last_switch) {
                if dur.to_millis() >= 1000 {
                    green_led.toggle();
                    last_switch = now;
                }
            }

            task_toggle.set_low();
            Mono::delay(10.millis()).await;
        }
    }

    #[task(priority = 4, local = [motor_task_local])]
    async fn motor_task(ctx: motor_task::Context) {
        let left_motor = &mut ctx.local.motor_task_local.left;
        let right_motor = &mut ctx.local.motor_task_local.right;
        let left_encoder = &mut ctx.local.motor_task_local.left_encoder;
        let right_encoder = &mut ctx.local.motor_task_local.right_encoder;
        #[cfg(feature = "battery_task")]
        let current_monitor = &mut ctx.local.motor_task_local.current_monitor;
        let task_toggle = &mut ctx.local.motor_task_local.task_toggle;
        let motor_cmd_r = &mut ctx.local.motor_task_local.motor_cmd_r;
        let feedback_s = &mut ctx.local.motor_task_local.feedback_s;
        let buzzer_s = &mut ctx.local.motor_task_local.buzzer_s;

        let mut left_pid = PidCreator::<f32>::new()
            .set_p(0.8)
            .set_i(0.5)
            .set_d(0.55)
            .create_controller();
        let mut right_pid = left_pid;

        let mut prev_stop_motor_state = 0;

        let motor_alert = BuzzerMsg::Background(Some(Notes::MiddleA));
        let motor_alert_cleared = BuzzerMsg::Background(None);

        loop {
            task_toggle.set_high();

            let now = Mono::now();

            let left_error = left_motor.is_in_error();
            let right_error = right_motor.is_in_error();
            let e_stop_pressed = ctx.local.motor_task_local.e_stop.is_high();
            let stop_motors = left_error || right_error || e_stop_pressed;
            let stop_motor_state =
                ((left_error as u8) << 2) | ((right_error as u8) << 1) | (e_stop_pressed as u8);

            if stop_motor_state != prev_stop_motor_state {
                if stop_motors {
                    // Setting the e-stop can cause the motors to enter an error state, re-enabling
                    // the motors when the e-stop can try to clear it
                    let prev_e_stop = (prev_stop_motor_state & 1) == 1;
                    if !e_stop_pressed && prev_e_stop && (left_error || right_error) {
                        warn!("E-stop cleared, resetting motors");
                        left_motor.set_enable(true);
                        right_motor.set_enable(true);
                        buzzer_s.send(motor_alert_cleared).await.unwrap();
                    } else {
                        warn!(
                            "Motor fault detected! {}, {}, {}",
                            left_error, right_error, e_stop_pressed
                        );
                        left_motor.set_enable(false);
                        right_motor.set_enable(false);
                        buzzer_s.send(motor_alert).await.unwrap();
                    }
                } else {
                    warn!(
                        "Motor fault cleared! {}, {}, {}",
                        left_error, right_error, e_stop_pressed
                    );
                    left_motor.set_enable(true);
                    right_motor.set_enable(true);
                    buzzer_s.send(motor_alert_cleared).await.unwrap();
                }

                prev_stop_motor_state = stop_motor_state;
            }

            let left_velocity = left_encoder.get_velocity(now);
            let right_velocity = right_encoder.get_velocity(now);

            #[cfg(feature = "battery_task")]
            {
                // TODO add these to the proto spec
                let (mut left_current, mut right_current) = current_monitor.get_currents();
                left_current = left_current.abs();
                right_current = right_current.abs();

                enhanced_log!("currents: {}, {}", left_current, right_current);
            }

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
                        enhanced_log!(
                            "received directions: ({:?}, {:?}), ({:?}, {:?})",
                            j.speed,
                            j.heading,
                            left_drive,
                            right_drive
                        );
                        if !stop_motors {
                            right_motor.drive(right_drive);
                            left_motor.drive(left_drive);
                        }
                    }
                    Msg::DiffDrive(d) => {
                        enhanced_log!("dequeued command: {:?}", d);
                        enhanced_log!("new setpoint: {}, {}", d.left_motor, d.right_motor);
                        if !stop_motors {
                            left_pid.set_setpoint(
                                d.left_motor,
                                Some(controls::motor_math::ACTUAL_MAX_SPEED_RAD_S),
                            );
                            right_pid.set_setpoint(
                                d.right_motor,
                                Some(controls::motor_math::ACTUAL_MAX_SPEED_RAD_S),
                            );
                        }
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

                enhanced_log!("left v: {}, next: {}", left_velocity, next_left);
                enhanced_log!("right v: {}, next: {}", right_velocity, next_right);

                left_motor.drive(rad_s_to_duty(next_left));
                right_motor.drive(rad_s_to_duty(next_right));
            }

            task_toggle.set_low();
            Mono::delay(2.millis()).await;
        }
    }

    #[task(priority = 6, binds = OTG_FS, local = [usb_task_local, red_led], shared = [usb_serial, usb_connected])]
    fn usb_fs(ctx: usb_fs::Context) {
        // enhanced_log!("usb_fs");
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
                enhanced_warn!("No event for USB ISR?");
                return;
            }

            match usb_serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    red_led.set_low();

                    for b in buf[0..count].iter() {
                        match decoder.decode_byte(*b) {
                            Ok(Some(DecodedVal::Data(u))) => {
                                if let Err(_) = usb_cmd.push(u) {
                                    error!("cmd buffer full");
                                    usb_cmd.clear();
                                    *decoder = DataFrame::new();
                                }
                            }
                            Ok(Some(DecodedVal::EndFend)) => {
                                msg_complete = true;
                            }
                            Ok(_) => (),
                            Err(d) => {
                                usb_cmd.clear();
                                error!("could not decode: {:?}", d);
                                *decoder = DataFrame::new();
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

            if msg_complete && !cmd_s.is_full() {
                match decode_proto_msg::<TopMsg>(usb_cmd.as_slice()) {
                    Ok(t) => {
                        match cmd_s.try_send(t) {
                            Ok(_) => *usb_connected = true,
                            Err(e) => error!("can't enqueue new command: {:?}", e),
                        };
                    }
                    Err(e) => error!("Proto decode error: {}", e),
                };

                usb_cmd.clear();
            }
        });

        task_toggle.set_low();
    }

    #[task(priority = 2, local = [buzzer_task_local])]
    async fn buzzer_task(ctx: buzzer_task::Context) {
        let cmd_receiver = &mut ctx.local.buzzer_task_local.buzzer_r;
        let buzzer = &mut ctx.local.buzzer_task_local.buzzer;
        let task_toggle = &mut ctx.local.buzzer_task_local.task_toggle;

        buzzer.disable();
        task_toggle.set_low();

        let mut background_cmd = Notes::Rest;
        let mut have_background_cmd = false;

        loop {
            let current_cmd = cmd_receiver.recv().await.unwrap();

            buzzer.enable();

            info!("new buzzer: {:?}", current_cmd);

            match current_cmd {
                BuzzerMsg::Tune(t) => {
                    for _ in 0..t.repeat {
                        task_toggle.set_high();
                        buzzer.set_note(t.tone);

                        task_toggle.set_low();
                        Mono::delay(t.millis).await;
                    }
                    buzzer.disable();
                }
                BuzzerMsg::Indef(n) => buzzer.set_note(n),
                BuzzerMsg::Background(b) => {
                    if let Some(bn) = b {
                        background_cmd = bn;
                        have_background_cmd = true;
                    } else {
                        buzzer.disable();
                        have_background_cmd = false;
                    }
                }
                BuzzerMsg::Mute => {
                    buzzer.disable();
                    have_background_cmd = false;
                }
            };

            while cmd_receiver.is_empty() && have_background_cmd {
                task_toggle.set_high();
                buzzer.set_note(background_cmd);
                buzzer.enable();

                task_toggle.set_low();
                Mono::delay(300.millis()).await;

                task_toggle.set_high();
                buzzer.disable();

                task_toggle.set_low();
                Mono::delay(300.millis()).await;
            }
        }
    }

    #[task(priority = 3, local = [battery_task_local])]
    async fn battery_task(ctx: battery_task::Context) {
        let feedback_s = &mut ctx.local.battery_task_local.feedback_s;
        let buzzer_s = &mut ctx.local.battery_task_local.buzzer_s;
        let task_toggle = &mut ctx.local.battery_task_local.task_toggle;
        let battery_monitor = &mut ctx.local.battery_task_local.battery_monitor;

        const MAX_VOLTAGE_DIFF: f32 = 0.25;
        const LOW_VOLTAGE_WARNING: f32 = 3.1;

        let mut warnings_on = false;

        loop {
            task_toggle.set_high();

            let (cell0, cell1, cell2) = battery_monitor.get_cell_voltages();
            info!("cell 0 {}, cell 1 {}, cell 2 {}", cell0, cell1, cell2);

            // TODO add cell voltages message to protobuf
            let cell_diffs = [
                ("01", (cell0 - cell1).abs()),
                ("02", (cell0 - cell2).abs()),
                ("03", (cell1 - cell2).abs()),
            ];

            let cells = [cell0, cell1, cell2];

            let mut low_voltage_warning = false;
            let mut cell_voltage_warning = false;

            let mut i = 0;
            for cell in cells {
                if cell <= LOW_VOLTAGE_WARNING {
                    low_voltage_warning = true;
                    warn!("Cell {} V is low: {}", i, cell);
                }
                i += 1;
            }

            for (cell, diff) in cell_diffs {
                if diff > MAX_VOLTAGE_DIFF {
                    warn!("Diff {} is too large: {}", cell, diff);
                    cell_voltage_warning = true;
                }
            }

            let new_warnings = low_voltage_warning || cell_voltage_warning;

            if new_warnings != warnings_on {
                let msg = if new_warnings {
                    BuzzerMsg::Background(Some(Notes::MiddleB))
                } else {
                    BuzzerMsg::Background(None)
                };

                if let Err(e) = buzzer_s.send(msg).await {
                    error!("no buzzer send: {:?}", e);
                }

                warnings_on = new_warnings;
            }

            let battery_msg = TopMsg {
                msg: Some(Msg::Battery(BatteryMsg {
                    cell0,
                    cell1,
                    cell2,
                })),
            };

            feedback_s.send(battery_msg).await.ok();

            task_toggle.set_low();
            Mono::delay(10000.millis()).await;
        }
    }

    #[task(priority = 5, local = [imu_task_local])]
    async fn imu_task(ctx: imu_task::Context) {
        let task_toggle = &mut ctx.local.imu_task_local.task_toggle;
        let feedback_s = &mut ctx.local.imu_task_local.feedback_s;
        let imu = &mut ctx.local.imu_task_local.imu;

        let mut driver = drivers::imu::ImuDriver::new(imu);

        loop {
            task_toggle.set_high();
            if driver
                .update()
                .inspect_err(|e| error!("IMU Error: {:?}", e))
                .is_ok()
            {
                if let Some((g, a)) = driver.get_data() {
                    // info!("IMU data: {:?}, {:?}", g, a);
                    let msg = TopMsg {
                        msg: Some(Msg::Imu(ImuMsg {
                            gyro: Some(Vector3 {
                                x: g.x,
                                y: g.y,
                                z: g.z,
                            }),
                            accel: Some(Vector3 {
                                x: a.x,
                                y: a.y,
                                z: a.z,
                            }),
                        })),
                    };
                    feedback_s.send(msg).await.ok();
                }
            } else if driver.get_state() == drivers::imu::ImuCalibrationState::Operational {
                driver.reset();
            }
            Mono::delay(2.millis()).await;
            task_toggle.set_low();
        }
    }
}
