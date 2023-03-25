use stm32f4xx_hal::{
    gpio::{Alternate, Pin},
    pac::{USART1, USART2},
    serial::{Serial, Tx},
};

type TxThing = Pin<'A', 2_u8, Alternate<7_u8>>;
type RxThing = Pin<'A', 3_u8, Alternate<7_u8>>;
// type TxSerial = Pin<'A', 9_u8, Alternate<7_u8>>;

pub type DebugSerialPort = Tx<USART1>;
pub type BluetoothSerialPort = Serial<USART2, (TxThing, RxThing)>;
