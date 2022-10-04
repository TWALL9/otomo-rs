// use embedded_hal::serial::{Read as SerialRead, Write as SerialWrite};
// use stm32f4xx_hal::{pac::USART2, serial::{Rx, Tx}};

// pub struct Hc05 {
//     pub rx: Rx<USART2>,
//     pub tx: Tx<USART2>,
// }

// impl SerialRead<u8> for Hc05 {
//     type Error = ;
//     fn read(&mut self) -> nb::Result<u8, Self::Error> {
//         self.rx.read()
//     }
// }

// impl SerialWrite<u8> for Hc05 {
//     fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
//         self.tx.write
//     }
// }
