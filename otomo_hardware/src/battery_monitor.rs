/// Monitor the three cells from the 3S battery running the robot
/// Using voltage dividers, ensure that the input voltage does not exceed the 2.95V
/// the STM32F4-Discovery uses as the Vref for ADC's.
use stm32f4xx_hal::{
    adc::{config::SampleTime, Adc},
    gpio::{Analog, Pin},
    pac::ADC2,
};

use embedded_hal_02::adc::Channel;

pub type DefaultBatteryMonitor =
    BatteryMonitor<Pin<'A', 4, Analog>, Pin<'A', 5, Analog>, Pin<'A', 7, Analog>>;

const CONVERSION_FACTOR: f32 = 0.001 * (4.2 / 3.0); // Max cell voltage / max ADC voltage

pub struct BatteryMonitor<
    C1: Channel<ADC2, ID = u8>,
    C2: Channel<ADC2, ID = u8>,
    C3: Channel<ADC2, ID = u8>,
> {
    adc: Adc<ADC2>,
    cell1: C1,
    cell2: C2,
    cell3: C3,
}

impl<C1: Channel<ADC2, ID = u8>, C2: Channel<ADC2, ID = u8>, C3: Channel<ADC2, ID = u8>>
    BatteryMonitor<C1, C2, C3>
{
    pub fn new(adc: Adc<ADC2>, cell1: C1, cell2: C2, cell3: C3) -> Self {
        Self {
            adc,
            cell1,
            cell2,
            cell3,
        }
    }

    /// Returns cell voltages
    pub fn get_cell_voltages(&mut self) -> (f32, f32, f32) {
        let cell1_sample = self.adc.convert(&self.cell1, SampleTime::Cycles_480);
        let cell2_sample = self.adc.convert(&self.cell2, SampleTime::Cycles_480);
        let cell3_sample = self.adc.convert(&self.cell3, SampleTime::Cycles_480);

        let cell1_millivolts = self.adc.sample_to_millivolts(cell1_sample) as f32;
        let cell2_millivolts = self.adc.sample_to_millivolts(cell2_sample) as f32;
        let cell3_millivolts = self.adc.sample_to_millivolts(cell3_sample) as f32;

        (
            cell1_millivolts * CONVERSION_FACTOR,
            cell2_millivolts * CONVERSION_FACTOR,
            cell3_millivolts * CONVERSION_FACTOR,
        )
    }
}
