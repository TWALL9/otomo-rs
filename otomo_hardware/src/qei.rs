use stm32f4xx_hal::{
    pac::{TIM4, TIM5},
    qei::Qei,
};

pub type RightQei = Qei<TIM4>;
pub type LeftQei = Qei<TIM5>;
