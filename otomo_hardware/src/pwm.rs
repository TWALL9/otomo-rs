use stm32f4xx_hal::{
    gpio::{Pin, Alternate},
    pac::{TIM3, TIM4},
    timer::{PwmHz, Ch},
};

type PwmC6 = Pin<'C', 6_u8, Alternate<2>>;
type PwmC7 = Pin<'C', 7_u8, Alternate<2>>;
type PwmC8 = Pin<'C', 8_u8, Alternate<2>>;
type PwmC9 = Pin<'C', 9_u8, Alternate<2>>;
type PwmD6 = Pin<'D', 6_u8, Alternate<2>>;
type PwmD7 = Pin<'D', 7_u8, Alternate<2>>;
type PwmD8 = Pin<'D', 8_u8, Alternate<2>>;
type PwmD9 = Pin<'D', 9_u8, Alternate<2>>;

pub type Pwm3 = PwmHz<TIM3, (Ch<1>, Ch<2>, Ch<3>, Ch<4>), (PwmC6, PwmC7, PwmC8, PwmC9)>;
pub type Pwm4 = PwmHz<TIM4, (Ch<1>, Ch<2>, Ch<3>, Ch<4>), (PwmD6, PwmD7, PwmD8, PwmD9)>;
