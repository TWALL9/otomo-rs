use stm32f4xx_hal::{pac::USART2, serial::Tx};

pub type DebugSerialPort = Tx<USART2>;
