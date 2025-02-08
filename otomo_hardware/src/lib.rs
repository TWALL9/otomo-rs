#![no_std]

use core::ptr::addr_of_mut;

use stm32f4xx_hal::{
    adc::{
        config::{AdcConfig, SampleTime, Sequence},
        Adc,
    },
    gpio::{Input, Output, PinState, PushPull, PD0, PD1, PD2, PD3, PD4, PE7, PE8},
    i2c::{I2c, I2c1},
    otg_fs::{UsbBus, UsbBusType, USB},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
    qei::Qei,
    timer::{Channel, Channel1, Channel2, Channel3, Channel4, Timer13, Timer3},
};

use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort as UsbSerialPort, UsbError};

pub mod battery_monitor;
pub mod buzzer;
pub mod imu;
pub mod led;
pub mod motors;
pub mod qei;
pub mod serial;
pub mod ultrasonic;

use led::{BlueLed, GreenLed, OrangeLed, RedLed};
use motors::{
    current_monitor::{CurrentMonitor, DefaultCurrentMonitor},
    encoder::QuadratureEncoder,
    pololu_driver::{LeftDrive, MotorDriver, RightDrive},
};
use qei::{LeftQei, RightQei};
use serial::DebugSerialPort;
// use ultrasonic::maxbotix;

pub type FanPin = PE7<Output<PushPull>>;

pub type TaskToggle0 = PD0<Output<PushPull>>;
pub type TaskToggle1 = PD1<Output<PushPull>>;
pub type TaskToggle2 = PD2<Output<PushPull>>;
pub type TaskToggle3 = PD3<Output<PushPull>>;
pub type TaskToggle4 = PD4<Output<PushPull>>;

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
    pub task_toggle_3: TaskToggle3,
    pub task_toggle_4: TaskToggle4,

    pub dbg_serial: DebugSerialPort,

    pub left_motor: LeftDrive,
    pub right_motor: RightDrive,
    pub left_encoder: QuadratureEncoder<LeftQei>,
    pub right_encoder: QuadratureEncoder<RightQei>,
    pub current_monitor: DefaultCurrentMonitor,
    pub fan_motor: FanPin,
    pub e_stop: EStopPressed,

    pub usb_serial: UsbSerial,

    pub buzzer: buzzer::Buzzer,

    pub battery_monitor: battery_monitor::DefaultBatteryMonitor,
    pub imu: imu::bno055::Bno055<I2c1>,
}

impl OtomoHardware {
    pub fn init(pac: Peripherals, core: CorePeripherals) -> Self {
        let rcc = pac.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(168.MHz())
            .require_pll48clk()
            .freeze();

        let gpioa = pac.GPIOA.split();
        let gpiob = pac.GPIOB.split();
        let gpioc = pac.GPIOC.split();
        let gpiod = pac.GPIOD.split();
        let gpioe = pac.GPIOE.split();

        // Task toggle pins
        let task_toggle_0 = gpiod.pd0.into_push_pull_output();
        let task_toggle_1 = gpiod.pd1.into_push_pull_output();
        let task_toggle_2 = gpiod.pd2.into_push_pull_output();
        let task_toggle_3 = gpiod.pd3.into_push_pull_output();
        let task_toggle_4 = gpiod.pd4.into_push_pull_output();

        // Status LED's
        let green_led = gpiod.pd12.into_push_pull_output();
        let orange_led = gpiod.pd13.into_push_pull_output();
        let red_led = gpiod.pd14.into_push_pull_output();
        let blue_led = gpiod.pd15.into_push_pull_output();

        let debug_tx_pin = gpioa.pa2.into_alternate();
        let dbg_serial = pac.USART2.tx(debug_tx_pin, 115200.bps(), &clocks).unwrap();

        // Buzzer
        let tim13 = Timer13::new(pac.TIM13, &clocks);
        let tim13_pins = Channel1::new(gpioa.pa6);
        let mut pwm13 = tim13.pwm_hz(tim13_pins, 10.kHz());
        pwm13.disable(Channel::C1);

        let buzzer = buzzer::Buzzer::new(pwm13, Channel::C1);

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
        let left_diag = gpiob.pb13.into_pull_up_input();
        let left_overcurrent = gpiob.pb15.into_push_pull_output_in_state(PinState::High);
        let right_enable = gpiob.pb3.into_push_pull_output();
        let right_diag = gpiod.pd6.into_pull_up_input();
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

        let left_current_pin = gpioc.pc2.into_analog();
        let right_current_pin = gpiob.pb0.into_analog();
        let adc1 = Adc::adc1(pac.ADC1, true, AdcConfig::default());
        let vdda = adc1.reference_voltage();

        let current_monitor = CurrentMonitor::new(adc1, left_current_pin, right_current_pin);

        let cell0_pin = gpioa.pa4.into_analog(); // ADC12_IN4
        let cell1_pin = gpioa.pa5.into_analog(); // ADC12_IN5
        let cell2_pin = gpioa.pa7.into_analog(); // ADC12_IN7

        let config = AdcConfig::default().reference_voltage(vdda);
        let mut adc2 = Adc::adc2(pac.ADC2, true, config);

        adc2.configure_channel(&cell0_pin, Sequence::One, SampleTime::Cycles_480);
        adc2.configure_channel(&cell1_pin, Sequence::Two, SampleTime::Cycles_480);
        adc2.configure_channel(&cell2_pin, Sequence::Three, SampleTime::Cycles_480);

        let battery_monitor =
            battery_monitor::BatteryMonitor::new(adc2, cell0_pin, cell1_pin, cell2_pin);

        let fan_motor = gpioe.pe7.into_push_pull_output_in_state(PinState::Low);
        let e_stop = gpioe.pe8.into_pull_up_input();

        #[cfg(feature = "ultrasonic_hcsr04")]
        {
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

            let counter = pac.TIM10.counter_us(&clocks);
            let ultrasonics = Ultrasonics {
                counter,
                right: right_ultrasonic,
                left: left_ultrasonic,
            };
        }

        let usb = USB::new(
            (pac.OTG_FS_GLOBAL, pac.OTG_FS_DEVICE, pac.OTG_FS_PWRCLK),
            (gpioa.pa11, gpioa.pa12),
            &clocks,
        );

        // FURTHER unholy shit.
        unsafe {
            #[allow(static_mut_refs)]
            USB_BUS.replace(UsbBus::new(usb, &mut *addr_of_mut!(USB_EP_MEM)));
        }

        #[allow(static_mut_refs)]
        let usb_serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        // 0x0483: STMicroelectronics, 0x5740: Virtual COM Port
        let vid_pid = UsbVidPid(0x0483, 0x5740);

        #[allow(static_mut_refs)]
        let usb_dev = UsbDeviceBuilder::new(unsafe { USB_BUS.as_ref().unwrap() }, vid_pid)
            .device_class(usbd_serial::USB_CLASS_CDC)
            .self_powered(true)
            .strings(&[StringDescriptors::default()
                .manufacturer("idklol")
                .product("Serial port")
                .serial_number("001")])
            .unwrap()
            .build();

        let sda = gpiob.pb9;
        let scl = gpiob.pb8;

        let i2c1 = I2c::new(pac.I2C1, (scl, sda), 400.kHz(), &clocks);

        let mut delay = core.SYST.delay(&clocks);
        let mut bno = imu::bno055::Bno055::new(i2c1, false);
        delay.delay_ms(100);

        bno.init(&mut delay).unwrap();
        bno.set_external_crystal(&mut delay, true).unwrap();

        Self {
            green_led,
            orange_led,
            red_led,
            blue_led,
            task_toggle_0,
            task_toggle_1,
            task_toggle_2,
            task_toggle_3,
            task_toggle_4,
            dbg_serial,
            left_motor,
            right_motor,
            left_encoder,
            right_encoder,
            current_monitor,
            fan_motor,
            e_stop,
            usb_serial: UsbSerial {
                device: usb_dev,
                serial: usb_serial,
            },
            buzzer,
            battery_monitor,
            imu: bno,
        }
    }
}
