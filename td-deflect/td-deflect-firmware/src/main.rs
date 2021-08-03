#![no_main]
#![no_std]

use panic_halt as _;
use stm32f7::stm32f7x2::*;
use rtic::app;
use serde::{Serialize};

const AHB_CLOCK: u32 = 216_000_000;
const APB2_CLOCK: u32 = AHB_CLOCK / 2;

const MIN_H_FREQ: u32 = 15000;
const MAX_H_FREQ: u32 = 16000;
#[allow(unused)]
const MAX_H_PERIOD: u32 = AHB_CLOCK / MIN_H_FREQ;
const MIN_H_PERIOD: u32 = AHB_CLOCK / MAX_H_FREQ;

const DAC_MIDPOINT: i32 = 2047;

#[app(device = stm32f7::stm32f7x2, peripherals = true)]
const APP: () = {
    struct Resources {
        dac: DAC,
        gpiob: GPIOB,
        hot_driver: HOTDriver,
        hsync_capture: HSyncCapture,
        crt_state: CRTState,
        crt_stats_live: CRTStats,
        crt_stats: CRTStats,
        serial: Serial,
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
        let usart1 = s.USART1;

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
        rcc.apb2enr.modify(|_,w| { w.usart1en().bit(true) });

        gpioa.moder.modify(|_,w| { w.moder5().output() });

        gpiob.afrh.modify(|_,w| { w.afrh8().af3() });
        gpiob.moder.modify(|_,w| { w.moder8().alternate() });
        gpioa.afrl.modify(|_,w| { w.afrl6().af2() });
        gpioa.moder.modify(|_,w| { w. moder6().alternate() });

        // serial input/output
        gpioa.afrh.modify(|_,w| { w.afrh9().af7() });
        gpioa.moder.modify(|_,w| {w.moder9().alternate() });
        gpioa.afrh.modify(|_,w| {w.afrh10().af7() });
        gpioa.moder.modify(|_,w| {w.moder10().alternate() });
        let serial = Serial::new(usart1);

        // sync inputs
        gpiob.moder.modify(|_,w| {w.moder0().input()});
        gpiob.pupdr.modify(|_,w| {w.pupdr0().pull_up()});

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
        let hsync_capture = HSyncCapture::new(s.TIM3);
        hot_driver.set_frequency(15700);

        // disable blanking
        gpioc.odr.modify(|_,w| {w.odr14().bit(false)});

        // configure the system timer to wrap around every second
        //syst.set_clock_source(SystClkSource::Core);
        //syst.set_reload(AHB_CLOCK); // once a second
        //syst.enable_counter();
        let crt_stats_live = CRTStats::default();
        let crt_stats = CRTStats::default();
        let crt_state = CRTState::default();
        init::LateResources { gpiob, dac, hot_driver, hsync_capture, crt_stats_live, crt_stats, crt_state, serial }
    }

    // internal hsync timer interrupt
    #[task(binds = TIM1_UP_TIM10, resources = [gpiob, dac, hot_driver, hsync_capture, crt_state, crt_stats_live], spawn = [update_double_buffers], priority = 15)]
    fn tim1_up_tim10(cx: tim1_up_tim10::Context) {
        let crt_state = cx.resources.crt_state;
        let current_scanline = &mut crt_state.current_scanline;
        let dac = cx.resources.dac;
        let gpiob = cx.resources.gpiob;
        let hot_driver = cx.resources.hot_driver;
        let hsync_capture = cx.resources.hsync_capture;
        let crt_stats = cx.resources.crt_stats_live;

        // horizontal sync PLL
        // TODO: replace this software capture with a hardware capture
        let cycles_since_sync = hsync_capture.get_cycles_since_sync() as i32;
        let input_period = hsync_capture.get_period() as i32;
        let output_period = hot_driver.get_period() as i32;
        // if we are really far away frequency wise, just reset the period entirely
        if (output_period - input_period).abs() > ((MIN_H_PERIOD as i32) / 10){
            //hot_driver.set_period(input_period as u32);
        } else {
            // this error's magnitude is precisely wrong - values near
            // 0 are the largest error
            let error = if cycles_since_sync < (input_period / 2) {
                cycles_since_sync.max(1)
            } else {
                (cycles_since_sync - input_period).min(-1)
            };
            let error_mag = (error.abs() * 1 / 4).max(2);
            //let error_mag = 2;
            let new_period = if error > 0 {
                input_period - error_mag
            } else if error < 0 {
                input_period + error_mag
            } else {
                input_period
            };
            hot_driver.set_period(new_period as u32);
        }

        let VERTICAL_MAX_AMPS = 0.5;
        let VERTICAL_MAGNITUDE_AMPS = 0.45;
        // vertical advance
        let total_lines = 262;
        let center_line = total_lines / 2;
        // convert scanline to a (+1, -1) range coordinate (+1 is top of screen)
        let horizontal_pos_coordinate = ((*current_scanline) - center_line) as f32 / (total_lines as f32) * -2.0;
        let horizontal_amps = (horizontal_pos_coordinate * VERTICAL_MAGNITUDE_AMPS).clamp(-1.0*VERTICAL_MAX_AMPS, VERTICAL_MAX_AMPS);
        let AMPS_TO_VOLTS = 1.0;
        let VOLTS_TO_DAC_VALUE = 1.0/(3.3/4095.0);
        let dac_value = ((horizontal_amps * AMPS_TO_VOLTS * VOLTS_TO_DAC_VALUE) as i32 + DAC_MIDPOINT).clamp(0, 4095);
        dac.dhr12r1.write(|w| { unsafe { w.bits(dac_value as u32) }});

        // vertical counter
        let vga_vsync = gpiob.idr.read().idr0().bit();
        // negative edge triggered
        if (vga_vsync == false) && (crt_state.previous_vga_vsync == true) {
            *current_scanline = 0;
        } else {
            *current_scanline += 1;
        }
        crt_state.previous_vga_vsync = vga_vsync;
        if *current_scanline >= total_lines { *current_scanline = 0 };
        // clear interrupt flag for tim10
        hot_driver.t.sr.write(|w| w.uif().bit(false));

        // update stats
        crt_stats.h_input_period = input_period;
        crt_stats.h_output_period = output_period;

        // dispatch double buffer updates
        if *current_scanline == 0 {
            cx.spawn.update_double_buffers().unwrap();
        }
    }

    #[task(resources = [crt_stats_live, crt_stats], priority = 14, spawn = [send_stats])]
    fn update_double_buffers(mut c: update_double_buffers::Context) {
        let crt_stats = c.resources.crt_stats;
        c.resources.crt_stats_live.lock(|crt_stats_live| {
            *crt_stats = *crt_stats_live;
        });
        c.spawn.send_stats().unwrap();
    }

    #[task(resources = [crt_stats, serial])]
    fn send_stats(mut c: send_stats::Context) {
        let serial = c.resources.serial;
        let mut json_stats: [u8; 256] = [0; 256];
        let mut json_stats_len = 0;
        c.resources.crt_stats.lock(|crt_stats| {
            match serde_json_core::to_slice(&crt_stats, &mut json_stats) {
                Ok(c) => json_stats_len = c,
                Err(_) => (),
            };
        });
        serial.write_blocking(&json_stats[..json_stats_len]);
        serial.write_blocking(b"\n");
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn EXTI0();
        fn EXTI1();
    }
};

#[allow(unused)]
pub struct HOTDriver {
    t: TIM10, // HOT output transistor timer
    tp: TIM1, // horizontal supply buck converter PWM
}

#[derive(Default)]
pub struct CRTState {
    current_scanline: i32,
    previous_vga_vsync: bool,
}

#[derive(Default, Copy, Clone, Serialize)]
pub struct CRTStats {
    h_output_period: i32,
    h_input_period: i32,
}

impl HOTDriver {
    fn new(t: TIM10, tp: TIM1) -> HOTDriver {
        t.ccmr1_output().write(|w| { unsafe { w.oc1m().bits(0b111) }}); // PWM2
        t.ccer.write(|w| { w.cc1e().set_bit() });
        t.ccr1.write(|w| { unsafe { w.ccr().bits(2680) }});
        t.arr.write(|w| { unsafe { w.arr().bits(2680*2) }});
        t.cr1.write(|w| { w.cen().enabled().urs().bit(true) });
        t.egr.write(|w| { w.ug().set_bit() });
        t.dier.write(|w| { w.uie().set_bit() });

        tp.ccmr1_output().write(|w| {unsafe{w.oc1m().bits(0b110) }}); // PWM1
        tp.ccer.write(|w| { w.cc1e().set_bit().cc1ne().set_bit() });
        // initialize to 50% duty cycle at 100khz
        tp.ccr1.write(|w| { w.ccr().bits((AHB_CLOCK/15700*1/2) as u16) });
        tp.arr.write(|w| { w.arr().bits((AHB_CLOCK/15700) as u16) });
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
        period = period.clamp(AHB_CLOCK/MAX_H_FREQ, AHB_CLOCK/MIN_H_FREQ);
        let turn_off_time = period * 1 / 4;
        self.t.arr.write(|w| { unsafe { w.arr().bits(period as u16 - 1) }});
        self.t.ccr1.write(|w| { unsafe { w.ccr().bits(turn_off_time as u16) }});
    }
    fn get_period(&self) -> u32 {
        self.t.arr.read().bits() + 1
    }
    fn set_frequency(&self, mut f: u32) {
        f = f.clamp(MIN_H_FREQ, MAX_H_FREQ);
        let timer_freq = AHB_CLOCK;
        let period = timer_freq / f;
        self.set_period(period)
    }
}

pub struct HSyncCapture {
    t: TIM3,
}

impl HSyncCapture {
    fn new(t: TIM3) -> HSyncCapture {
        t.ccmr1_output().write(|w| { unsafe { w.bits(0b0000_00_10_0000_00_01)}});
        t.smcr.write(|w| { unsafe { w.ts().bits(0b101).sms().bits(0b0100) }});
        t.ccer.write(|w| { w.cc1p().set_bit().cc1e().set_bit() });
        t.cr1.write(|w| { w.cen().enabled() });
        HSyncCapture {
            t
        }
    }
    // following functions multiply by 2 because of timer clock rate
    fn get_cycles_since_sync(&self) -> u32 {
        ((self.t.cnt.read().cnt().bits() & 0xFFFF) as u32) * 2
    }
    fn get_period(&self) -> u32 {
        ((self.t.ccr1.read().ccr().bits() & 0xFFFF) as u32 + 1) * 2
    }
}

pub struct Serial {
    usart: USART1
}

impl Serial {
    fn new(usart: USART1) -> Serial {
        usart.brr.write(|w| { w.brr().bits((APB2_CLOCK/16/115200*2*8) as u16)});
        usart.cr1.write(|w| { w.te().enabled().re().enabled().ue().enabled() });
        Serial { usart }
    }
    fn write_blocking(&self, b: &[u8]) {
        for c in b {
            while self.usart.isr.read().txe() == false {}
            self.usart.tdr.write(|w| { unsafe { w.tdr().bits((*c).into()) }});
        }
    }
}
