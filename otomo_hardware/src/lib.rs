#![no_std]

use stm32f4xx_hal::{
    gpio::{Input, Output, PinState, PushPull, PD0, PD1, PD2, PE7, PE8},
    otg_fs::{UsbBus, UsbBusType, USB},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
    qei::Qei,
    timer::{Channel1, Channel2, Channel3, Channel4, Timer3},
};

use usb_device::{
    bus::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::{SerialPort as UsbSerialPort, UsbError};

pub mod led;
pub mod motors;
pub mod qei;
pub mod serial;
pub mod ultrasonic;

use led::{BlueLed, GreenLed, OrangeLed, RedLed};
use motors::{
    encoder::QuadratureEncoder,
    pololu_driver::{LeftDrive, MotorDriver, RightDrive},
};
use qei::{LeftQei, RightQei};
use serial::DebugSerialPort;

pub type FanPin = PE7<Output<PushPull>>;

pub type TaskToggle0 = PD0<Output<PushPull>>;
pub type TaskToggle1 = PD1<Output<PushPull>>;
pub type TaskToggle2 = PD2<Output<PushPull>>;

pub type EStopPressed = PE8<Input>;

// The absolutely UNHOLY shit I have to do to share a USB port >:(
static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_EP_MEM: [u32; 512] = [0; 512];
pub struct UsbSerial {
    pub device: UsbDevice<'static, UsbBusType>,
    pub serial: UsbSerialPort<'static, UsbBusType>,
}

impl UsbSerial {
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, UsbError> {
        if self.poll() {
            self.serial.read(buf)
        } else {
            Err(UsbError::WouldBlock)
        }
    }

    pub fn write(&mut self, buf: &[u8]) -> Result<usize, UsbError> {
        self.serial.write(buf)
    }

    pub fn poll(&mut self) -> bool {
        self.device.poll(&mut [&mut self.serial])
    }
}

pub struct OtomoHardware {
    pub green_led: GreenLed,
    pub orange_led: OrangeLed,
    pub red_led: RedLed,
    pub blue_led: BlueLed,

    pub task_toggle_0: TaskToggle0,
    pub task_toggle_1: TaskToggle1,
    pub task_toggle_2: TaskToggle2,

    pub dbg_serial: DebugSerialPort,

    pub left_motor: LeftDrive,
    pub right_motor: RightDrive,
    pub left_encoder: QuadratureEncoder<LeftQei>,
    pub right_encoder: QuadratureEncoder<RightQei>,
    pub fan_motor: FanPin,
    pub e_stop: EStopPressed,

    pub usb_serial: UsbSerial,
}

impl OtomoHardware {
    pub fn init(pac: Peripherals, _core: CorePeripherals) -> Self {
        // let syscfg = pac.SYSCFG.constrain();

        let rcc = pac.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).pclk1(8.MHz()).freeze();

        let gpioa = pac.GPIOA.split();
        let gpiob = pac.GPIOB.split();
        let gpioc = pac.GPIOC.split();
        let gpiod = pac.GPIOD.split();
        let gpioe = pac.GPIOE.split();

        // Task toggle pins
        let task_toggle_0 = gpiod.pd0.into_push_pull_output();
        let task_toggle_1 = gpiod.pd1.into_push_pull_output();
        let task_toggle_2 = gpiod.pd2.into_push_pull_output();

        // Status LED's
        let green_led = gpiod.pd12.into_push_pull_output();
        let orange_led = gpiod.pd13.into_push_pull_output();
        let red_led = gpiod.pd14.into_push_pull_output();
        let blue_led = gpiod.pd15.into_push_pull_output();

        let debug_tx_pin = gpioa.pa9.into_alternate();
        let dbg_serial = pac.USART1.tx(debug_tx_pin, 115200.bps(), &clocks).unwrap();

        let tim3 = Timer3::new(pac.TIM3, &clocks);
        let tim3_pins = (
            Channel1::new(gpioc.pc6),
            Channel2::new(gpioc.pc7),
            Channel3::new(gpioc.pc8),
            Channel4::new(gpioc.pc9),
        );
        let pwm3 = tim3.pwm_hz(tim3_pins, 10.kHz());

        // For capturing PWM cycles
        // pwm3.deref_mut().listen(Event::C1);
        // Add C2 event as well?
        let (left_a, right_a, right_b, left_b) = pwm3.split();

        let left_enable = gpiob.pb14.into_push_pull_output();
        let left_diag = gpiob.pb13.into_input();
        let left_overcurrent = gpiob.pb15.into_push_pull_output_in_state(PinState::High);
        let right_enable = gpiob.pb3.into_push_pull_output();
        let right_diag = gpiod.pd6.into_input();
        let right_overcurrent = gpiob.pb4.into_push_pull_output_in_state(PinState::High);

        let left_motor = MotorDriver::new(left_a, left_b, left_enable, left_diag, left_overcurrent);
        let right_motor = MotorDriver::new(
            right_a,
            right_b,
            right_enable,
            right_diag,
            right_overcurrent,
        );

        let left_encoder_pins = (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate());
        let right_encoder_pins = (gpiob.pb6.into_alternate(), gpiob.pb7.into_alternate());

        let left_qei = Qei::new(pac.TIM5, left_encoder_pins);
        let right_qei = Qei::new(pac.TIM4, right_encoder_pins);

        let left_encoder = QuadratureEncoder::new(left_qei, 6500);
        let right_encoder = QuadratureEncoder::new(right_qei, 6500);

        let fan_motor = gpioe.pe7.into_push_pull_output_in_state(PinState::Low);
        let e_stop = gpioe.pe8.into_pull_up_input();

        // Right ultrasonic
        // let trig_pin = gpiob.pb11.into_push_pull_output();
        // let mut echo_pin = gpiob.pb10.into_pull_down_input();
        // echo_pin.make_interrupt_source(&mut syscfg);
        // echo_pin.enable_interrupt(&mut pac.EXTI);
        // echo_pin.trigger_on_edge(&mut pac.EXTI, Edge::RisingFalling);

        // let right_ultrasonic = Hcsr04::new(trig_pin, echo_pin);

        // let trig_pin = gpioc.pc11.into_push_pull_output();
        // let mut echo_pin = gpioc.pc10.into_pull_down_input();
        // // echo_pin.make_interrupt_source(&mut syscfg);
        // echo_pin.enable_interrupt(&mut pac.EXTI);
        // echo_pin.trigger_on_edge(&mut pac.EXTI, Edge::RisingFalling);
        // let left_ultrasonic = Hcsr04::new(trig_pin, echo_pin);

        // let counter = pac.TIM10.counter_us(&clocks);
        // let ultrasonics = Ultrasonics {
        //     counter,
        //     right: right_ultrasonic,
        //     left: left_ultrasonic,
        // };

        let usb = USB::new(
            (pac.OTG_FS_GLOBAL, pac.OTG_FS_DEVICE, pac.OTG_FS_PWRCLK),
            (gpioa.pa11, gpioa.pa12),
            &clocks,
        );

        // FURTHER unholy shit.
        unsafe {
            USB_BUS.replace(UsbBus::new(usb, &mut USB_EP_MEM));
        }

        let usb_serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        // 0x0483: STMicroelectronics, 0x5740: Virtual COM Port
        let vid_pid = UsbVidPid(0x0483, 0x5740);
        let usb_dev = UsbDeviceBuilder::new(unsafe { USB_BUS.as_ref().unwrap() }, vid_pid)
            .manufacturer("idklol")
            .product("Serial port")
            .serial_number("001")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .self_powered(true)
            .build();

        Self {
            green_led,
            orange_led,
            red_led,
            blue_led,
            task_toggle_0,
            task_toggle_1,
            task_toggle_2,
            dbg_serial,
            left_motor,
            right_motor,
            left_encoder,
            right_encoder,
            fan_motor,
            e_stop,
            usb_serial: UsbSerial {
                device: usb_dev,
                serial: usb_serial,
            },
        }
    }
}
