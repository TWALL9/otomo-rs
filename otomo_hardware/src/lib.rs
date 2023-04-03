#![no_std]

use stm32f4xx_hal::{
    gpio::Edge,
    otg_fs::{UsbBus, UsbBusType, USB},
    pac::{CorePeripherals, Peripherals, TIM2},
    prelude::*,
    serial::Serial,
    timer::{MonoTimerUs, SysDelay, Timer3},
};

use usb_device::{
    bus::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::{SerialPort as UsbSerialPort, UsbError};

pub mod led;
pub mod motors;
pub mod pwm;
pub mod serial;
pub mod ultrasonic;

use led::{BlueLed, GreenLed, OrangeLed, RedLed};
use motors::pololu_driver::{LeftDrive, MotorDriver, RightDrive};
use serial::{BluetoothSerialPort, DebugSerialPort};
use ultrasonic::{Hcsr04, Ultrasonics};

pub type MonoTimer = MonoTimerUs<TIM2>;

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

    fn poll(&mut self) -> bool {
        self.device.poll(&mut [&mut self.serial])
    }
}

pub struct OtomoHardware {
    pub mono: MonoTimer,
    pub delay: SysDelay,
    pub green_led: GreenLed,
    pub orange_led: OrangeLed,
    pub red_led: RedLed,
    pub blue_led: BlueLed,

    pub bt_serial: BluetoothSerialPort,
    pub dbg_serial: DebugSerialPort,

    pub left_motor: LeftDrive,
    pub right_motor: RightDrive,

    pub ultrasonics: Ultrasonics,

    pub usb_serial: UsbSerial,
}

impl OtomoHardware {
    pub fn init(mut pac: Peripherals, core: CorePeripherals) -> Self {
        let mut syscfg = pac.SYSCFG.constrain();

        let rcc = pac.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).pclk1(8.MHz()).freeze();
        let mono = pac.TIM2.monotonic_us(&clocks);
        let delay = core.SYST.delay(&clocks);

        let gpioa = pac.GPIOA.split();
        let gpiob = pac.GPIOB.split();
        let gpioc = pac.GPIOC.split();
        let gpiod = pac.GPIOD.split();

        // Status LED's
        let green_led = gpiod.pd12.into_push_pull_output();
        let orange_led = gpiod.pd13.into_push_pull_output();
        let red_led = gpiod.pd14.into_push_pull_output();
        let blue_led = gpiod.pd15.into_push_pull_output();

        let tx_pin = gpioa.pa2.into_alternate();
        // let tx_pin = stm32f4xx_hal::gpio::NoPin;
        let rx_pin = gpioa.pa3.into_alternate();
        let mut bt_serial =
            Serial::new(pac.USART2, (tx_pin, rx_pin), 38400.bps(), &clocks).unwrap();
        bt_serial.listen(stm32f4xx_hal::serial::Event::Rxne);

        let debug_tx_pin = gpioa.pa9.into_alternate();
        let dbg_serial = pac.USART1.tx(debug_tx_pin, 115200.bps(), &clocks).unwrap();

        let tim3 = Timer3::new(pac.TIM3, &clocks);
        let tim3_pins = (
            gpioc.pc6.into_alternate(),
            gpioc.pc7.into_alternate(),
            gpioc.pc8.into_alternate(),
            gpioc.pc9.into_alternate(),
        );
        let pwm3 = tim3.pwm_hz(tim3_pins, 10.kHz());

        // For capturing PWM cycles
        // pwm3.deref_mut().listen(Event::C1);
        // Add C2 event as well?
        let (left_a, left_b, right_a, right_b) = pwm3.split();

        let left_enable = gpioa.pa10.into_push_pull_output();
        let left_diag = gpioa.pa13.into_input();
        let right_enable = gpiod.pd9.into_push_pull_output();
        let right_diag = gpiod.pd10.into_input();

        let left_motor = MotorDriver::new(left_a, left_b, left_enable, left_diag);
        let right_motor = MotorDriver::new(right_a, right_b, right_enable, right_diag);

        // Right ultrasonic
        let trig_pin = gpiob.pb11.into_push_pull_output();
        let mut echo_pin = gpiob.pb10.into_pull_down_input();
        echo_pin.make_interrupt_source(&mut syscfg);
        echo_pin.enable_interrupt(&mut pac.EXTI);
        echo_pin.trigger_on_edge(&mut pac.EXTI, Edge::RisingFalling);

        let right_ultrasonic = Hcsr04::new(trig_pin, echo_pin);

        let trig_pin = gpioc.pc11.into_push_pull_output();
        let mut echo_pin = gpioc.pc10.into_pull_down_input();
        // echo_pin.make_interrupt_source(&mut syscfg);
        echo_pin.enable_interrupt(&mut pac.EXTI);
        echo_pin.trigger_on_edge(&mut pac.EXTI, Edge::RisingFalling);
        let left_ultrasonic = Hcsr04::new(trig_pin, echo_pin);

        let counter = pac.TIM9.counter_us(&clocks);
        let ultrasonics = Ultrasonics {
            counter,
            right: right_ultrasonic,
            left: left_ultrasonic,
        };

        let usb = USB {
            usb_global: pac.OTG_FS_GLOBAL,
            usb_device: pac.OTG_FS_DEVICE,
            usb_pwrclk: pac.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };

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
            mono,
            delay,
            green_led,
            orange_led,
            red_led,
            blue_led,
            bt_serial,
            dbg_serial,
            left_motor,
            right_motor,
            ultrasonics,
            usb_serial: UsbSerial {
                device: usb_dev,
                serial: usb_serial,
            },
        }
    }
}
