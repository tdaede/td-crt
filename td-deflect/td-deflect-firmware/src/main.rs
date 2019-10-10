#![no_main]
#![no_std]

extern crate panic_halt;
extern crate stm32f4;
extern crate cortex_m;
extern crate stm32f4xx_hal;
extern crate num_traits;
use stm32f4::stm32f401::*;
use cortex_m_rt::entry;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_semihosting::*;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::*;
use num_traits::clamp;

const MIN_H_FREQ: u32 = 15000;
const MAX_H_FREQ: u32 = 33000;

struct HOTDriver {
    t: TIM10,
}

impl HOTDriver {
    fn new(t: TIM10) -> HOTDriver {
        t.ccmr1_output.write(|w| { unsafe { w.oc1m().bits(0b110) }}); // PWM1
        t.ccer.write(|w| { w.cc1e().set_bit() });
        t.ccr1.write(|w| { unsafe { w.ccr().bits(2680) }});
        t.arr.write(|w| { unsafe { w.arr().bits(2680*2) }});
        t.cr1.write(|w| { w.cen().enabled() });
        t.egr.write(|w| { w.ug().set_bit() });
        HOTDriver {
            t
        }
    }
    fn set_period(&self, mut period: u32) {
        period = clamp(period, 84_000_000/MAX_H_FREQ, 84_000_000/MIN_H_FREQ);
        let turn_on_time = period / 2;
        self.t.arr.write(|w| { unsafe { w.arr().bits(period as u16) }});
        self.t.ccr1.write(|w| { unsafe { w.ccr().bits(turn_on_time as u16) }});
    }
    fn set_frequency(&self, mut f: u32) {
        f = clamp(f, MIN_H_FREQ, MAX_H_FREQ);
        let timer_freq = 84_000_000;
        let period = timer_freq / f;
        self.set_period(period)
    }
}

struct CSyncCapture {
    t: TIM3,
}

impl CSyncCapture {
    fn new(t: TIM3) -> CSyncCapture {
        t.ccmr1_output.write(|w| { unsafe { w.bits(0b0000_00_10_0000_00_01)}});
        t.smcr.write(|w| { unsafe { w.ts().bits(0b101).sms().bits(0b100) }});
        t.ccer.write(|w| { w.cc1p().set_bit().cc1e().set_bit().cc2e().set_bit() });
        t.cr1.write(|w| { w.cen().enabled() });
        CSyncCapture {
            t
        }
    }
    fn get_period(&self) -> u32 {
        self.t.ccr1.read().ccr().bits() & 0xFFFF
    }
    fn get_sync_len(&self) -> u32 {
        self.t.ccr2.read().ccr().bits() & 0xFFFF
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    let s = stm32f4::stm32f401::Peripherals::take().unwrap();

    let mut syst = p.SYST;
    let mut nvic = p.NVIC;
    let rcc = s.RCC;
    let gpioa = s.GPIOA;
    let gpiob = s.GPIOB;

    rcc.ahb1enr.write(|w| { w.gpioaen().bit(true) });
    rcc.ahb1enr.write(|w| { w.gpioben().bit(true) });
    rcc.apb2enr.write(|w| { w.tim10en().bit(true) });
    rcc.apb1enr.write(|w| { w.tim3en().bit(true) });
    rcc.constrain().cfgr.sysclk(84.mhz()).freeze();
    gpioa.moder.write(|w| { w.moder5().output() });

    gpiob.split().pb8.into_alternate_af3(); // HOT driver
    gpioa.split().pa6.into_alternate_af2(); // csync input

    nvic.enable(Interrupt::EXTI0);

    let hot_driver = HOTDriver::new(s.TIM10);
    let csync_capture = CSyncCapture::new(s.TIM3);
    hot_driver.set_frequency(15700);

    // configure the system timer to wrap around every second
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(84_000_000); // once a second
    syst.enable_counter();

    loop {
        // busy wait until the timer wraps around
        //while !syst.has_wrapped() {}
        let period = csync_capture.get_period();
        let sync_len = csync_capture.get_sync_len();
        //hprintln!("period: {} sync_len: {}", period, sync_len);
    }
}

