use stm32f7::stm32f7x2::*;

pub struct ADC {
    adc1: ADC1
}

impl ADC {
    pub fn new(adc1: ADC1) -> ADC {
        adc1.cr2.modify(|_,w| {w.adon().bit(true)});
        ADC { adc1 }
    }
    pub fn read_blocking(&self, channel: u8) -> u16 {
        self.adc1.sqr3.modify(|_,w| { unsafe { w.sq1().bits(channel) } });
        self.adc1.cr2.modify(|_,w| { w.swstart().set_bit() });
        while !self.adc1.sr.read().eoc().bit() {};
        self.adc1.dr.read().data().bits()
    }
}
