use stm32f4xx_hal::{
    gpio::{Alternate, Pin},
    pac::{TIM3, TIM4},
    timer::{Ch, PwmHz},
};

type PwmC6 = Pin<'C', 6_u8, Alternate<2>>;
type PwmC7 = Pin<'C', 7_u8, Alternate<2>>;
type PwmC8 = Pin<'C', 8_u8, Alternate<2>>;
type PwmC9 = Pin<'C', 9_u8, Alternate<2>>;
type PwmB6 = Pin<'B', 6_u8, Alternate<2>>;
type PwmB7 = Pin<'B', 7_u8, Alternate<2>>;
type PwmB8 = Pin<'B', 8_u8, Alternate<2>>;
type PwmB9 = Pin<'B', 9_u8, Alternate<2>>;

pub type Pwm3 = PwmHz<TIM3, (Ch<0>, Ch<1>, Ch<2>, Ch<3>), (PwmC6, PwmC7, PwmC8, PwmC9)>;
pub type Pwm4 = PwmHz<TIM4, (Ch<0>, Ch<1>, Ch<2>, Ch<3>), (PwmB6, PwmB7, PwmB8, PwmB9)>;
