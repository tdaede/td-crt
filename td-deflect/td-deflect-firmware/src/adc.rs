use stm32g4::stm32g474::*;
use cortex_m::asm::delay;

use crate::app::AHB_CLOCK;

pub struct ADC {
    adc1: ADC1
}

impl ADC {
    pub fn new(adc12_common: &ADC12_COMMON, adc1: ADC1, rcc: &RCC, gpioa: &GPIOA) -> ADC {
        rcc.ahb2enr().modify(|_,w| { w.adc12en().enabled() });
        adc1.cr().modify(|_,w| { w.deeppwd().disabled() });
        adc12_common.ccr().write(|w| { w.ckmode().sync_div4() });
        // startup adc voltage regulator
        adc1.cr().modify(|_,w| { w.advregen().enabled() });
        while !adc1.cr().read().advregen().bit_is_set() {}
        delay((20e-6*AHB_CLOCK as f32) as u32);
        adc1.cr().modify(|_,w| { w.adcaldif().clear_bit() });
        adc1.cr().modify(|_,w| { w.adcal().set_bit() });
        while adc1.cr().read().adcal().bit_is_set() {};
        adc1.isr().modify(|_, w| { w.adrdy().set_bit() });
        adc1.cr().modify(|_, w| { w.aden().set_bit() });
        while !adc1.isr().read().adrdy().bit() {};
        adc1.isr().modify(|_, w| { w.adrdy().set_bit() });
        // adc lines
        //gpioa.moder.modify(|_,w| { w.moder0().analog() }); // h current transformer
        //gpioa.moder.modify(|_,w| { w.moder1().analog() }); // hot source current
        gpioa.moder().modify(|_,w| { w.moder2().analog() }); // s cap voltage
        //gpioa.moder.modify(|_,w| { w.moder3().analog() }); // eht voltage
        //gpioa.moder.modify(|_,w| { w.moder5().analog() }); // eht current
        ADC { adc1 }
    }
    pub fn read_blocking(&self, channel: u8) -> u16 {
        self.adc1.sqr1().modify(|_,w| { unsafe { w.sq1().bits(channel) } });
        self.adc1.cr().modify(|_,w| { w.adstart().set_bit() });
        while !self.adc1.isr().read().eoc().bit() {};
        let data = self.adc1.dr().read().rdata().bits();
        self.adc1.isr().modify(|_, w| { w.eoc().set_bit() });
        data
    }
}
