/// To monitor current from the pololu driver
use stm32f4xx_hal::{
    adc::{config::SampleTime, Adc},
    gpio::{Analog, Pin},
    pac::ADC1,
};

use embedded_hal::adc::Channel;

pub type DefaultCurrentMonitor = CurrentMonitor<Pin<'C', 2, Analog>, Pin<'B', 0, Analog>>;

const CONVERSION_FACTOR: i16 = 2; // 1000 mA per 500 mV

pub struct CurrentMonitor<C: Channel<ADC1, ID = u8>, D: Channel<ADC1, ID = u8>> {
    adc: Adc<ADC1>,
    left_pin: C,
    right_pin: D,
}

impl<C: Channel<ADC1, ID = u8>, D: Channel<ADC1, ID = u8>> CurrentMonitor<C, D> {
    pub fn new(adc: Adc<ADC1>, left_pin: C, right_pin: D) -> Self {
        Self {
            adc,
            left_pin,
            right_pin,
        }
    }

    /// Reports in milliamps (mA)
    pub fn get_currents(&mut self) -> (i16, i16) {
        let left_sample = self.adc.convert(&self.left_pin, SampleTime::Cycles_480);
        let right_sample = self.adc.convert(&self.right_pin, SampleTime::Cycles_480);
        let left_millivolts = self.adc.sample_to_millivolts(left_sample) as i16;
        let right_millivolts = self.adc.sample_to_millivolts(right_sample) as i16;

        (
            left_millivolts * CONVERSION_FACTOR,
            right_millivolts * CONVERSION_FACTOR,
        )
    }
}
