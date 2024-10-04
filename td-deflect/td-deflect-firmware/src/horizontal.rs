use stm32g4::stm32g474::*;

use crate::app::AHB_CLOCK;
use crate::app::MAX_H_PERIOD;
use crate::app::MAX_H_FREQ;
use crate::app::MIN_H_FREQ;

#[allow(unused)]
pub struct HOTDriver {
    pub tp: TIM1, // horizontal supply buck converter PWM
    thv: TIM4, // hv timer
    pub duty_setpoint: f32,
    current_duty: f32,
    h_pin_period: u16,
    period: f32,
    accumulator: f32,
}

impl HOTDriver {
    pub fn new(tp: TIM1, thv: TIM4) -> HOTDriver {
        // TODO: check TP / THV register size
        // configuration for width
        tp.ccmr1_output().write(|w| { unsafe { w.oc1m().bits(0b110) } }); // PWM1
        tp.ccer().write(|w| { w.cc1e().set_bit().cc1ne().set_bit() });
        // configuration for HOT
        tp.ccmr2_output().write(|w| { unsafe { w.oc3m().bits(0b111) } }); // PWM2
        tp.ccer().modify(|_,w| { w.cc3e().set_bit() });
        tp.ccr3().write(|w| { unsafe { w.ccr().bits(MAX_H_PERIOD*1/2) } });
        // initialize to longest possible period in case synchronization fails
        let h_pin_period = MAX_H_PERIOD as u16;
        // start out with 0 width to slowly ramp it up
        tp.ccr1().write(|w| { unsafe { w.ccr().bits(0) } });
        tp.arr().write(|w| { unsafe { w.arr().bits(h_pin_period.into()) } });
        // with 75 ohm gate resistors, 0x38 is the best
        tp.bdtr().write(|w| { unsafe { w.moe().set_bit().dtg().bits(0x38) }}); // uwu
        tp.cr1().write(|w| { w.cen().set_bit().urs().bit(true) });
        tp.egr().write(|w| { w.ug().set_bit() });
        tp.dier().write(|w| { w.uie().set_bit() });

        // hv driver
        thv.ccmr1_output().write(|w| { unsafe { w.oc1m().bits(0b110) } });
        thv.ccer().modify(|_,w| { w.cc1e().set_bit() });
        thv.ccr1().write(|w| { unsafe { w.ccr().bits(h_pin_period as u32 / 2) } });
        thv.arr().write(|w| { unsafe { w.arr().bits(h_pin_period.into()) } });
        thv.cr1().write(|w| { w.cen().set_bit() });
        HOTDriver {
            //t,
            tp,
            thv,
            duty_setpoint: 0.9,
            current_duty: 0.0,
            h_pin_period,
            period: MAX_H_PERIOD as f32,
            accumulator: 0.0,
        }
    }
    /// call once per horizontal period to update drive
    pub fn update(&mut self) {
        // update error diffusion for h period
        let period_quantized = (self.period + self.accumulator) as u32;
        self.accumulator += self.period - period_quantized as f32;
        self.set_period(period_quantized as u32);
        // update h pin duty
        let max_change_per_iteration = 0.001;
        self.current_duty = self.current_duty + (self.duty_setpoint - self.current_duty).clamp(-1.0*max_change_per_iteration, max_change_per_iteration);
        self.tp.ccr1().write(|w| { unsafe { w.ccr().bits((self.h_pin_period as f32 * self.current_duty) as u32)} });
        // trigger hv
        self.thv.cnt().write(|w| { unsafe { w.cnt().bits(0) } });
    }
    pub fn set_period_float(&mut self, period: f32) {
        self.period = period;
    }
    fn set_period(&mut self, mut period: u32) {
        period = period.clamp(AHB_CLOCK/MAX_H_FREQ, AHB_CLOCK/MIN_H_FREQ);
        let turn_off_time = period * 1 / 4;
        self.tp.arr().write(|w| { unsafe { w.arr().bits(period - 1) } });
        self.tp.ccr3().write(|w| { unsafe { w.ccr().bits(turn_off_time) } });
        self.h_pin_period = period as u16;
    }
    pub fn get_period(&self) -> u32 {
        self.tp.arr().read().bits() + 1
    }
    pub fn set_frequency(&mut self, mut f: u32) {
        f = f.clamp(MIN_H_FREQ, MAX_H_FREQ);
        let timer_freq = AHB_CLOCK;
        let period = timer_freq / f;
        self.set_period(period)
    }
}
