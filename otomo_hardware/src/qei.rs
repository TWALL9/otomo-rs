use stm32f4xx_hal::{
    gpio::{Alternate, Pin},
    pac::{TIM4, TIM5},
    qei::Qei,
};

pub type RightQei = Qei<TIM4, (Pin<'B', 6, Alternate<2>>, Pin<'B', 7, Alternate<2>>)>;
pub type LeftQei = Qei<TIM5, (Pin<'A', 0, Alternate<2>>, Pin<'A', 1, Alternate<2>>)>;
