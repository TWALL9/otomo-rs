use stm32f4xx_hal::{
    gpio::{Alternate, Pin},
    pac::USART2,
    serial::Serial,
};

type TxThing = Pin<'A', 2_u8, Alternate<7_u8>>;
type RxThing = Pin<'A', 3_u8, Alternate<7_u8>>;

pub type BluetoothSerialPort = Serial<USART2, (TxThing, RxThing)>;
