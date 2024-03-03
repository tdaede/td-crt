use stm32g4::stm32g474::*;
use cortex_m::asm::delay;

use crate::app::AHB_CLOCK;

pub struct ADC {
    adc1: ADC1
}

const ADC1_CHANNEL_S_VOLTAGE: u8 = 3;
const ADC12_CHANNEL_VERTICAL_CLASS_D_CURRENT_PRE: u8 = 6;
const _ADC2_CHANNEL_VERTICAL_CLASS_D_CURRENT_POST: u8 = 12;

const REF_VOLTAGE: f32 = 3.3;
const FULL_SCALE: f32 = 4095.0;
const VOLTS_PER_COUNT: f32 = REF_VOLTAGE / FULL_SCALE;

const S_CAP_MULTIPLIER: f32 = 101.0/1.0;
const VERTICAL_CLASS_D_CURRENT_MULTIPLIER: f32 = 1.0/60.0/0.01;

pub struct ADCHorizData {
    pub s_voltage: f32,
    pub vertical_class_d_current_pre: f32,
}

impl ADC {
    pub fn new(adc12_common: &ADC12_COMMON, adc1: ADC1, rcc: &RCC, gpioa: &GPIOA, gpiob: &GPIOB, gpioc: &GPIOC) -> ADC {
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

        // additional config
        adc1.cfgr2().modify(|_,w| { w.rovse().enabled().ovsr().os8().ovss().shift3() });
        // adc lines
        gpioa.moder().modify(|_,w| { w.moder2().analog() }); // s cap voltage
        gpioc.moder().modify(|_,w| { w.moder0().analog() }); // class d current before filter
        gpiob.moder().modify(|_,w| { w.moder2().analog() }); // class d current after filter
        let a = ADC { adc1 };
        a.set_sample_time(ADC12_CHANNEL_VERTICAL_CLASS_D_CURRENT_PRE, 0b000);
        a
    }
    pub fn set_sample_time(&self, channel: u8, sample_time: u8) {
        match channel {
            6 => self.adc1.smpr1().modify(|_,w| { w.smp6().bits(sample_time) }),
            _ => {}
        }
    }
    pub fn read_blocking(&self, channel: u8) -> u16 {
        self.adc1.sqr1().modify(|_,w| { unsafe { w.sq1().bits(channel) } });
        self.adc1.cr().modify(|_,w| { w.adstart().set_bit() });
        while !self.adc1.isr().read().eoc().bit() {};
        let data = self.adc1.dr().read().rdata().bits();
        self.adc1.isr().modify(|_, w| { w.eoc().set_bit() });
        data
    }
    pub fn read_blocking_volts(&self, channel: u8) -> f32 {
        self.read_blocking(channel) as f32 * VOLTS_PER_COUNT
    }

    pub fn read_all_horiz(&self) -> ADCHorizData {
        let s_voltage = self.read_blocking_volts(ADC1_CHANNEL_S_VOLTAGE) * S_CAP_MULTIPLIER;
        let vertical_class_d_current_pre = (self.read_blocking_volts(ADC12_CHANNEL_VERTICAL_CLASS_D_CURRENT_PRE) - 3.3/2.0) *
            VERTICAL_CLASS_D_CURRENT_MULTIPLIER;
        ADCHorizData {
            s_voltage,
            vertical_class_d_current_pre
        }
    }
}
