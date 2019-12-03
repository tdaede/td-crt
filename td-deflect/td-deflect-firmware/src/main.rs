#![no_main]
#![no_std]

use panic_halt as _;
use stm32f7::stm32f7x2::*;
use cortex_m_semihosting::*;
use num_traits::clamp;
use rtfm::app;

const AHB_CLOCK: u32 = 216_000_000;

const MIN_H_FREQ: u32 = 15000;
const MAX_H_FREQ: u32 = 33000;

const DAC_MAX: u32 = 4095;
const DAC_MIDPOINT: i32 = 2047;

#[app(device = stm32f7::stm32f7x2, peripherals = true)]
const APP: () = {
    struct Resources {
        dac: DAC,
        hot_driver: HOTDriver
    }
    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let s = cx.device;

        let rcc = s.RCC;
        let gpioa = s.GPIOA;
        let gpiob = s.GPIOB;
        let gpioc = s.GPIOC;
        let dac = s.DAC;
        let flash = s.FLASH;
        let pwr = s.PWR;

        // enable pwr clock
        rcc.apb1enr.write(|w| { w.pwren().bit(true) });

        // set maximum flash delay
        flash.acr.write(|w| {
            w.prften().bit(true).arten().bit(true).latency().bits(0b1111)
        });
        // 16mhz hse / 8 * 216 = 432mhz pll output
        // divide by 2 for 216mhz main system clock
        rcc.pllcfgr.write(|w| { unsafe {
            w.pllsrc().bit(true).pllm().bits(8).plln().bits(216)
        }});
        //enable hse and pll
        rcc.cr.write(|w| {
            w.hseon().set_bit().hsebyp().set_bit().pllon().set_bit()
        });
        // enable power overdrive
        pwr.cr1.write(|w| { w.oden().bit(true) });
        // wait for overdrive to come up
        while !pwr.csr1.read().odrdy().bit() {};
        // switch to overdrive
        pwr.cr1.modify(|_,w| { w.odswen().bit(true) });
        // wait for switch to overdrive
        while !pwr.csr1.read().odswrdy().bit() {};
        // wait for pll lock
        while !rcc.cr.read().pllrdy().bit() {};
        // apb2 108mhz, apb1 54mhz, switch to pll as clock source
        rcc.cfgr.write(|w| { unsafe {
            w.ppre2().bits(0b100).ppre1().bits(0b101).sw().pll()}
        });

        rcc.ahb1enr.modify(|_,w| { w.gpioaen().bit(true) });
        rcc.ahb1enr.modify(|_,w| { w.gpioben().bit(true) });
        rcc.ahb1enr.modify(|_,w| { w.gpiocen().bit(true) });
        rcc.apb2enr.modify(|_,w| { w.tim1en().bit(true) });
        rcc.apb1enr.modify(|_,w| { w.dacen().bit(true) });
        rcc.apb2enr.modify(|_,w| { w.tim10en().bit(true) });
        rcc.apb1enr.modify(|_,w| { w.tim3en().bit(true) });

        gpioa.moder.modify(|_,w| { w.moder5().output() });

        gpiob.afrh.modify(|_,w| { w.afrh8().af3() });
        gpiob.moder.modify(|_,w| { w.moder8().alternate() });
        //gpioa.split().pa6.into_alternate_af2(); // csync input

        // horizontal PWM
        gpioa.afrh.modify(|_,w| {w.afrh8().af1()});
        gpioa.moder.modify(|_,w| {w.moder8().alternate()});
        gpiob.afrh.modify(|_,w| {w.afrh13().af1()});
        gpiob.moder.modify(|_,w| {w.moder13().alternate()});

        // blanking output to m51387
        gpioc.moder.modify(|_,w| {w.moder14().output()});
        gpioc.odr.modify(|_,w| {w.odr14().bit(true)});

        // S-cap lines
        gpioc.odr.modify(|_,w| { w.odr0().bit(true).odr1().bit(true).odr2().bit(true).odr3().bit(true) });
        gpioc.moder.modify(|_,w| { w.moder0().output().moder1().output().moder2().output().moder3().output() });

        // Vertical DAC output
        gpioa.moder.modify(|_,w| { w.moder4().analog() });
        dac.cr.write(|w| { w.en1().enabled().boff1().enabled() });
        dac.dhr12r1.write(|w| { unsafe { w.bits(DAC_MIDPOINT as u32) }});

        //nvic.enable(Interrupt::EXTI0);

        let hot_driver = HOTDriver::new(s.TIM10, s.TIM1);
        let csync_capture = HSyncCapture::new(s.TIM3);
        hot_driver.set_frequency(15700);

        // configure the system timer to wrap around every second
        //syst.set_clock_source(SystClkSource::Core);
        //syst.set_reload(AHB_CLOCK); // once a second
        //syst.enable_counter();
        init::LateResources { dac, hot_driver }
    }

    // internal hsync timer interrupt
    #[task(binds = TIM1_UP_TIM10, resources = [dac, hot_driver])]
    fn tim1_up_tim10(cx: tim1_up_tim10::Context) {
        static mut HLINE: i32 = 0;
        let dac = cx.resources.dac;
        let hot_driver = cx.resources.hot_driver;
        let total_lines = 262;
        let center_line = total_lines / 2;
        let hline_scaler = 1;
        let dac_value = ((*HLINE) - center_line) * hline_scaler + DAC_MIDPOINT;
        dac.dhr12r1.write(|w| { unsafe { w.bits(dac_value as u32) }});
        *HLINE += 1;
        if *HLINE >= total_lines { *HLINE = 0 };
        // clear interrupt flag for tim10
        hot_driver.t.sr.write(|w| w.uif().bit(false));
    }
};

pub struct HOTDriver {
    t: TIM10, // HOT output transistor timer
    tp: TIM1, // horizontal supply buck converter PWM
}

struct CRTState {
    line: u32,
}

impl HOTDriver {
    fn new(t: TIM10, tp: TIM1) -> HOTDriver {
        t.ccmr1_output().write(|w| { unsafe { w.oc1m().bits(0b110) }}); // PWM1
        t.ccer.write(|w| { w.cc1e().set_bit() });
        t.ccr1.write(|w| { unsafe { w.ccr().bits(2680) }});
        t.arr.write(|w| { unsafe { w.arr().bits(2680*2) }});
        t.cr1.write(|w| { w.cen().enabled() });
        t.egr.write(|w| { w.ug().set_bit() });
        t.dier.write(|w| { w.uie().set_bit() });

        tp.ccmr1_output().write(|w| {unsafe{w.oc1m().bits(0b110) }}); // PWM1
        tp.ccer.write(|w| { w.cc1e().set_bit().cc1ne().set_bit() });
        // initialize to 50% duty cycle at 100khz
        tp.ccr1.write(|w| { unsafe { w.ccr().bits((AHB_CLOCK/15700*1/2) as u16) }});
        tp.arr.write(|w| { unsafe { w.arr().bits((AHB_CLOCK/15700) as u16) }});
        // minimum dead time experimentally seems to be 0x0D
        tp.bdtr.write(|w| { unsafe { w.moe().enabled().dtg().bits(0x20) }}); // uwu
        tp.cr1.write(|w| { w.cen().enabled() });
        tp.egr.write(|w| { w.ug().set_bit() });
        HOTDriver {
            t,
            tp
        }
    }
    fn set_period(&self, mut period: u32) {
        period = clamp(period, AHB_CLOCK/MAX_H_FREQ, AHB_CLOCK/MIN_H_FREQ);
        let turn_on_time = period * 3 / 4;
        self.t.arr.write(|w| { unsafe { w.arr().bits(period as u16) }});
        self.t.ccr1.write(|w| { unsafe { w.ccr().bits(turn_on_time as u16) }});
    }
    fn set_frequency(&self, mut f: u32) {
        f = clamp(f, MIN_H_FREQ, MAX_H_FREQ);
        let timer_freq = AHB_CLOCK;
        let period = timer_freq / f;
        self.set_period(period)
    }
}

struct HSyncCapture {
    t: TIM3,
}

impl HSyncCapture {
    fn new(t: TIM3) -> HSyncCapture {
        t.ccmr1_output().write(|w| { unsafe { w.bits(0b0000_00_10_0000_00_01)}});
        t.smcr.write(|w| { unsafe { w.ts().bits(0b101).sms().bits(0b100) }});
        t.ccer.write(|w| { w.cc1p().set_bit().cc1e().set_bit().cc2e().set_bit() });
        t.cr1.write(|w| { w.cen().enabled() });
        HSyncCapture {
            t
        }
    }
    fn get_period(&self) -> u32 {
        (self.t.ccr1.read().ccr().bits() & 0xFFFF).into()
    }
    fn get_sync_len(&self) -> u32 {
        (self.t.ccr2.read().ccr().bits() & 0xFFFF).into()
    }
}
