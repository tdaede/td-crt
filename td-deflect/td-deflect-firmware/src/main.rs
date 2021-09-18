#![no_main]
#![no_std]

use stm32f7::stm32f7x2::*;
use rtic::app;
use serde::{Serialize, Deserialize};
use core::sync::atomic::{self, Ordering};
use core::panic::PanicInfo;
use heapless::Vec;
use heapless::spsc::{Queue, Producer, Consumer};
use libm;

const AHB_CLOCK: u32 = 216_000_000;
const APB2_CLOCK: u32 = AHB_CLOCK / 2;

const MIN_H_FREQ: u32 = 15000;
const MAX_H_FREQ: u32 = 16000;
#[allow(unused)]
const MAX_H_PERIOD: u32 = AHB_CLOCK / MIN_H_FREQ;
const MIN_H_PERIOD: u32 = AHB_CLOCK / MAX_H_FREQ;

const DAC_MIDPOINT: i32 = 2047;

/// Immediately blanks the video signal.
/// Used in case of deflection failure to prevent burn-in.
fn fault_blank() {
    // read-modify-write is a bit unfortunate here
    unsafe { (*GPIOC::ptr()).odr.modify(|_,w| {w.odr14().bit(true)}); }
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    fault_blank();
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

#[app(device = stm32f7::stm32f7x2, peripherals = true)]
const APP: () = {
    struct Resources {
        v_drive: VDrive,
        gpioa: GPIOA,
        gpiob: GPIOB,
        gpioc: GPIOC,
        hot_driver: HOTDriver,
        hsync_capture: HSyncCapture,
        crt_state: CRTState,
        crt_stats_live: CRTStats,
        crt_stats: CRTStats,
        serial_protocol: SerialProtocol,
        adc: ADC,
        config: Config,
        config_queue_in: Producer<'static, Config, 2>,
        config_queue_out: Consumer<'static, Config, 2>,
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
        let adc1 = s.ADC1;

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

        rcc.dckcfgr1.write(|w| { w.timpre().set_bit() });

        rcc.ahb1enr.modify(|_,w| { w.gpioaen().bit(true) });
        rcc.ahb1enr.modify(|_,w| { w.gpioben().bit(true) });
        rcc.ahb1enr.modify(|_,w| { w.gpiocen().bit(true) });
        rcc.apb2enr.modify(|_,w| { w.adc1en().bit(true) });
        rcc.apb2enr.modify(|_,w| { w.tim1en().bit(true) });
        rcc.apb1enr.modify(|_,w| { w.dacen().bit(true) });
        rcc.apb2enr.modify(|_,w| { w.tim10en().bit(true) });
        rcc.apb1enr.modify(|_,w| { w.tim3en().bit(true) });
        rcc.apb2enr.modify(|_,w| { w.usart1en().bit(true) });

        gpioa.moder.modify(|_,w| { w.moder5().output() });

        // HOT output
        // old TIM10 connected lines
        //gpiob.afrh.modify(|_,w| { w.afrh8().af3() });
        //gpiob.moder.modify(|_,w| { w.moder8().alternate() });
        // new TIM1 CH4 connected lines
        gpioa.afrh.modify(|_,w| { w.afrh11().af1() });
        gpioa.moder.modify(|_,w| { w.moder11().alternate() });

        // hsync input
        gpioa.afrl.modify(|_,w| { w.afrl6().af2() });
        gpioa.moder.modify(|_,w| { w. moder6().alternate() });
        // loopback signal from H.PIN OUT for phase measurement
        gpiob.afrl.modify(|_,w| {w.afrl1().af2() }); // TIM3_CH4
        gpiob.moder.modify(|_,w| {w.moder1().alternate() });

        // serial input/output
        gpioa.afrh.modify(|_,w| { w.afrh9().af7() });
        gpioa.moder.modify(|_,w| {w.moder9().alternate() });
        gpioa.afrh.modify(|_,w| {w.afrh10().af7() });
        gpioa.moder.modify(|_,w| {w.moder10().alternate() });
        let serial = Serial::new(usart1);
        let serial_protocol = SerialProtocol::new(serial);

        // sync inputs
        gpiob.moder.modify(|_,w| {w.moder0().input()});
        gpiob.pupdr.modify(|_,w| {w.pupdr0().pull_up()});
        gpioa.moder.modify(|_,w| {w.moder7().input()});
        gpioa.pupdr.modify(|_,w| {w.pupdr7().pull_up()});

        // horizontal PWM
        gpioa.afrh.modify(|_,w| {w.afrh8().af1()});
        gpioa.moder.modify(|_,w| {w.moder8().alternate()});
        gpiob.afrh.modify(|_,w| {w.afrh13().af1()});
        gpiob.moder.modify(|_,w| {w.moder13().alternate()});

        // blanking output to m51387
        gpioc.moder.modify(|_,w| {w.moder14().output()});
        gpioc.odr.modify(|_,w| {w.odr14().bit(true)}); // start up with blanking enabled

        // S-cap lines
        // start with all S caps on, will change later in hsync timer interrupt
        gpioc.odr.modify(|_,w| { w.odr0().bit(true).odr1().bit(true).odr2().bit(true).odr3().bit(true) });
        gpioc.moder.modify(|_,w| { w.moder0().output().moder1().output().moder2().output().moder3().output() });

        // Vertical DAC output
        gpioa.moder.modify(|_,w| { w.moder4().analog() });
        gpiob.odr.modify(|_,w| { w.odr14().set_bit() });
        gpiob.moder.modify(|_,w| { w.moder14().output() });

        // adc lines
        gpioa.moder.modify(|_,w| { w.moder0().analog() }); // h current transformer
        gpioa.moder.modify(|_,w| { w.moder1().analog() }); // hot source current
        gpioa.moder.modify(|_,w| { w.moder2().analog() }); // s cap voltage
        gpioa.moder.modify(|_,w| { w.moder3().analog() }); // eht voltage
        gpioa.moder.modify(|_,w| { w.moder5().analog() }); // eht current

        let adc = ADC::new(adc1);

        let v_drive = VDrive::new(dac);

        //nvic.enable(Interrupt::EXTI0);

        let mut hot_driver = HOTDriver::new(s.TIM10, s.TIM1);
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
        let config = Config { crt: CRT_CONFIG_PANASONIC_S901Y, input: InputConfig { h_size: 0.95, h_phase: 0.0 } };

        // config queue for purposes of "double buffering" config
        static mut CONFIG_QUEUE: Queue<Config, 2> = Queue::new();
        // unsafe: we can fix this with rtic 0.6
        let (config_queue_in, config_queue_out) = unsafe { CONFIG_QUEUE.split() };
        init::LateResources {
            gpioa,
            gpiob,
            gpioc,
            v_drive,
            hot_driver,
            hsync_capture,
            crt_stats_live,
            crt_stats,
            crt_state,
            config,
            config_queue_in,
            config_queue_out,
            serial_protocol,
            adc
        }
    }

    // internal hsync timer interrupt
  //  #[inline(never)]
  //  #[link_section = ".data.TIM1_UP_TIM10"]
    #[task(binds = TIM1_UP_TIM10, resources = [gpioa, gpiob, gpioc, v_drive, hot_driver, hsync_capture, crt_state, config, config_queue_out, crt_stats_live, adc], spawn = [update_double_buffers], priority = 15)]
    fn tim1_up_tim10(cx: tim1_up_tim10::Context) {
        let crt_state = cx.resources.crt_state;
        let current_scanline = &mut crt_state.current_scanline;
        let v_drive = cx.resources.v_drive;
        let gpioa = cx.resources.gpioa;
        let _gpiob = cx.resources.gpiob;
        let gpioc = cx.resources.gpioc;
        let hot_driver = cx.resources.hot_driver;
        let hsync_capture = cx.resources.hsync_capture;
        let crt_stats = cx.resources.crt_stats_live;
        let adc = cx.resources.adc;
        let config = cx.resources.config;

        // horizontal sync PLL
        hot_driver.synchronize_h_pin();
        atomic::compiler_fence(Ordering::SeqCst);
        hsync_capture.update();
        atomic::compiler_fence(Ordering::SeqCst);
        v_drive.trigger();
        let input_period = hsync_capture.get_period_averaged();
        let output_period = hot_driver.get_period() as i32;
        // if we are really far away frequency wise, just ignore this sync entirely
        if (output_period - input_period as i32).abs() > ((MIN_H_PERIOD as i32) / 10){
            // yolo
        } else {
            let error = hsync_capture.get_phase_error_averaged();
            let FEEDBACK_GAIN = 0.001;
            let fb = if (*current_scanline < 10) || (*current_scanline > 250) {
                (error * FEEDBACK_GAIN).clamp(-3.0, 3.0)
            } else {
                (error * FEEDBACK_GAIN).clamp(-1.0, 1.0)
                  //0.0
            };
            //let fb_quantized = libm::roundf(fb) as i32;
            //let error_mag = 0;
            let new_period = input_period - fb;
            hot_driver.set_period_float(new_period);
            //hsync_capture.apply_phase_feedforward(fb_quantized);
        }

        hot_driver.update();

        let VERTICAL_MAX_AMPS = 1.0;
        // vertical advance
        // vertical counter
        //let vga_vsync = gpiob.idr.read().idr0().bit();
        let vga_vsync = gpioa.idr.read().idr7().bit();
        // negative edge triggered
        let total_lines = 262;
        let center_line = total_lines / 2;
        if (vga_vsync == false) && (crt_state.previous_vga_vsync == true) {
            //crt_stats.v_lines = *current_scanline as u16;
            crt_stats.v_lines = *current_scanline as u16;
            *current_scanline = 0;
        }
        crt_state.previous_vga_vsync = vga_vsync;
        if *current_scanline >= (total_lines + 10) { *current_scanline = 0 };
        // convert scanline to a (+1, -1) range coordinate (+1 is top of screen)
        let horizontal_pos_coordinate = ((*current_scanline) - center_line) as f32 / (total_lines as f32) * -2.0;
        let vertical_linearity = config.crt.vertical_linearity;
        let vertical_linearity_scale = 1.0 / libm::atanf(1.0 * vertical_linearity);
        let horizontal_coordinate_corrected = if vertical_linearity < 0.1 {
            horizontal_pos_coordinate
        } else {
            libm::atanf(horizontal_pos_coordinate * vertical_linearity) * vertical_linearity_scale
        };
        let horizontal_amps = (horizontal_coordinate_corrected * config.crt.v_mag_amps + config.crt.v_offset_amps).clamp(-1.0*VERTICAL_MAX_AMPS, VERTICAL_MAX_AMPS);
        v_drive.set_current(horizontal_amps);
        v_drive.update();
        // clear interrupt flag for tim10
        hot_driver.tp.sr.write(|w| w.uif().bit(false));

        // update stats
        crt_stats.h_input_period = input_period as i32;
        crt_stats.h_input_period_max = crt_stats.h_input_period_max.max(input_period as i32);
        if crt_stats.h_input_period_min == 0 {
            crt_stats.h_input_period_min = input_period as i32;
        }
        crt_stats.h_input_period_min = crt_stats.h_input_period_min.min(input_period as i32);
        crt_stats.h_output_period = output_period;
        crt_stats.h_output_period_max = crt_stats.h_output_period_max.max(output_period);
        if crt_stats.h_output_period_min == 0 {
            crt_stats.h_output_period_min = output_period;
        }
        crt_stats.h_output_period_min = crt_stats.h_output_period_min.min(output_period);

        // perform analog reads
        // todo: trigger these with timer
        crt_stats.s_voltage = adc.read_blocking(2);

        // update s capacitors
        gpioc.odr.modify(|_,w| { w.odr0().bit((config.crt.s_cap & 0b1000) == 0)
                                 .odr1().bit((config.crt.s_cap & 0b0100) == 0)
                                 .odr2().bit((config.crt.s_cap & 0b0010) == 0)
                                 .odr3().bit((config.crt.s_cap & 0b0001) == 0) });

        // dispatch double buffer updates
        if *current_scanline == 0 {
            let _ = cx.spawn.update_double_buffers();
        }
        *current_scanline += 1;

        // update config if there's one in the queue
        if let Some(new_config) = cx.resources.config_queue_out.dequeue() {
            *config = new_config;
        }
    }

    #[task(resources = [crt_stats_live, crt_stats], priority = 14, spawn = [send_stats])]
    fn update_double_buffers(mut c: update_double_buffers::Context) {
        let crt_stats = c.resources.crt_stats;
        c.resources.crt_stats_live.lock(|crt_stats_live| {
            *crt_stats = *crt_stats_live;
            *crt_stats_live = CRTStats::default(); // reset stats on new field
        });
        let _ = c.spawn.send_stats();
    }

    #[task(binds = USART1, resources = [serial_protocol, config_queue_in])]
    fn serial_interrupt(c: serial_interrupt::Context) {
        let serial_protocol = c.resources.serial_protocol;
        if serial_protocol.serial.usart.isr.read().rxne().bit() {
            serial_protocol.process_byte(c.resources.config_queue_in);
        }
        if serial_protocol.serial.usart.isr.read().txe().bit() {
            if let Some(c) = serial_protocol.serial.send_queue.dequeue() {
                serial_protocol.serial.usart.tdr.write(|w| { unsafe { w.tdr().bits((c).into()) }});
            }
        }
    }

    #[task(resources = [crt_stats, serial_protocol])]
    fn send_stats(mut c: send_stats::Context) {
        let serial = &mut c.resources.serial_protocol.serial;
        let mut json_stats: [u8; 1024] = [0; 1024];
        let mut json_stats_len = 0;
        c.resources.crt_stats.lock(|crt_stats| {
            match serde_json_core::to_slice(&crt_stats, &mut json_stats) {
                Ok(c) => json_stats_len = c,
                Err(_) => (),
            };
        });
        serial.write_queued(&json_stats[..json_stats_len]);
        serial.write_queued(b"\n");
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn EXTI0();
        fn EXTI1();
    }
};

#[derive(Default)]
pub struct CRTState {
    current_scanline: i32,
    previous_vga_vsync: bool,
}

#[derive(Default, Copy, Clone, Serialize)]
pub struct CRTStats {
    h_output_period: i32,
    h_output_period_min: i32,
    h_output_period_max: i32,
    h_input_period: i32,
    h_input_period_min: i32,
    h_input_period_max: i32,
    hot_source_current: u16,
    v_lines: u16,
    s_voltage: u16,
}

/// Configuration to match driver board to a particular tube/yoke
#[allow(unused)]
#[derive(Copy, Clone, Deserialize)]
pub struct CRTConfig {
    v_mag_amps: f32,
    v_offset_amps: f32,
    #[serde(default)]
    vertical_linearity: f32,
    // s-capacitor value, 0 = highest capacitance
    #[serde(default)]
    s_cap: u8,
}

/// Configuration for a particular input
#[allow(unused)]
#[derive(Copy, Clone, Deserialize)]
pub struct InputConfig {
    h_size: f32,
    h_phase: f32,
}

static CRT_CONFIG_PANASONIC_S901Y: CRTConfig = CRTConfig {
    v_mag_amps: 0.414,
    v_offset_amps: 0.0,
    vertical_linearity: 0.55,
    s_cap: 1,
};

/// Complete runtime configuration, both CRT config + input config
#[derive(Copy, Clone, Deserialize)]
pub struct Config {
    crt: CRTConfig,
    input: InputConfig,
}

#[allow(unused)]
pub struct HOTDriver {
    //t: TIM10, // HOT output transistor timer
    tp: TIM1, // horizontal supply buck converter PWM
    duty_setpoint: f32,
    current_duty: f32,
    h_pin_period: u16,
    period: f32,
    accumulator: f32,
}

impl HOTDriver {
    fn new(_t: TIM10, tp: TIM1) -> HOTDriver {
        /*
        t.ccmr1_output().write(|w| { unsafe { w.oc1m().bits(0b111) }}); // PWM2
        t.ccer.write(|w| { w.cc1e().set_bit() });
        t.ccr1.write(|w| { unsafe { w.ccr().bits((AHB_CLOCK/15700*1/2) as u16) }});
        t.arr.write(|w| { unsafe { w.arr().bits((AHB_CLOCK/15700) as u16) }});
        t.cr1.write(|w| { w.cen().enabled().urs().bit(true) });
        t.egr.write(|w| { w.ug().set_bit() });
        t.dier.write(|w| { w.uie().set_bit() });
        */

        tp.ccmr1_output().write(|w| {unsafe{w.oc1m().bits(0b110) }}); // PWM1
        tp.ccer.write(|w| { w.cc1e().set_bit().cc1ne().set_bit() });
        // configuration for HOT
        tp.ccmr2_output().write(|w| { unsafe { w.oc4m().bits(0b111) } }); // PWM2
        tp.ccer.modify(|_,w| { w.cc4e().set_bit() });
        tp.ccr4.write(|w| { w.ccr().bits((MAX_H_PERIOD*1/2) as u16) });
        // configure tp to reset whenever TIM3 (the input capture timer) does
        // ideally we would reset on t (TIM10) but that's not possible
        // this means a bad input can totally screw up our H size, that's pretty bad
        // we now do this in software via synchronize_h_pin() instead
        //tp.smcr.write(|w| { w.sms().bits(0b0100).ts().itr2() });
        // initialize to longest possible period in case synchronization fails
        let h_pin_period = MAX_H_PERIOD as u16;
        tp.ccr1.write(|w| { w.ccr().bits(h_pin_period*0) });
        tp.arr.write(|w| { w.arr().bits(h_pin_period) });
        // with 75 ohm gate resistors, 0x38 is the best
        tp.bdtr.write(|w| { unsafe { w.moe().enabled().dtg().bits(0x38) }}); // uwu
        tp.cr1.write(|w| { w.cen().enabled().urs().bit(true) });
        tp.egr.write(|w| { w.ug().set_bit() });
        tp.dier.write(|w| { w.uie().set_bit() });
        HOTDriver {
            //t,
            tp,
            duty_setpoint: 0.9,
            current_duty: 0.0,
            h_pin_period,
            period: MAX_H_PERIOD as f32,
            accumulator: 0.0,
        }
    }
    /// call once per horizontal period to synchronize tp
    #[inline(always)]
    fn synchronize_h_pin(&mut self) {
        //self.tp.egr.write(|w| { w.ug().set_bit() });
    }
    /// call once per horizontal period to update drive
    fn update(&mut self) {
        // update error diffusion for h period
        let period_quantized = (self.period + self.accumulator) as u32;
        self.accumulator += self.period - period_quantized as f32;
        self.set_period(period_quantized as u32);
        // update h pin duty
        let max_change_per_iteration = 0.001;
        self.current_duty = self.current_duty + (self.duty_setpoint - self.current_duty).clamp(-1.0*max_change_per_iteration, max_change_per_iteration);
        self.tp.ccr1.write(|w| { w.ccr().bits((self.h_pin_period as f32 * self.current_duty) as u16)});
    }
    fn set_period_float(&mut self, period: f32) {
        self.period = period;
    }
    fn set_period(&mut self, mut period: u32) {
        period = period.clamp(AHB_CLOCK/MAX_H_FREQ, AHB_CLOCK/MIN_H_FREQ);
        let turn_off_time = period * 1 / 4;
        self.tp.arr.write(|w| { w.arr().bits(period as u16 - 1) });
        self.tp.ccr4.write(|w| { w.ccr().bits(turn_off_time as u16) });
        self.h_pin_period = period as u16;
    }
    fn get_period(&self) -> u32 {
        self.tp.arr.read().bits() + 1
    }
    fn set_frequency(&mut self, mut f: u32) {
        f = f.clamp(MIN_H_FREQ, MAX_H_FREQ);
        let timer_freq = AHB_CLOCK;
        let period = timer_freq / f;
        self.set_period(period)
    }
}

pub struct HSyncCapture {
    t: TIM3,
    previous_input_periods: [u32; HSyncCapture::AVERAGED_INPUT_PERIODS],
    previous_input_periods_index: usize,
    previous_phase_errors: [i32; HSyncCapture::AVERAGED_PHASE_ERRORS],
    previous_phase_errors_index: usize,
}

impl HSyncCapture {
    const AVERAGED_INPUT_PERIODS: usize = 256;
    const AVERAGED_PHASE_ERRORS: usize = 4;
    fn new(t: TIM3) -> HSyncCapture {
        t.ccmr1_output().write(|w| { unsafe { w.bits(0b0000_00_10_0000_00_01)}});
        t.ccmr2_input().write(|w| { unsafe { w.cc4s().bits(0b01) } });
        t.smcr.write(|w| { unsafe { w.ts().bits(0b101).sms().bits(0b0100) }});
        // CH1 is configured to reset the counter and measure H period
        // CH4 is configured for capture to measure H phase error
        t.ccer.write(|w| { w.cc1p().set_bit().cc1e().set_bit().cc4p().clear_bit().cc4np().clear_bit().cc4e().set_bit() });
        t.cr1.write(|w| { w.cen().enabled() });
        HSyncCapture {
            t,
            previous_input_periods: [MAX_H_PERIOD; HSyncCapture::AVERAGED_INPUT_PERIODS],
            previous_input_periods_index: 0,
            previous_phase_errors: [0; HSyncCapture::AVERAGED_PHASE_ERRORS],
            previous_phase_errors_index: 0,
        }
    }
    #[inline(always)]
    fn get_cycles_since_sync(&self) -> u32 {
        (self.t.ccr4.read().ccr().bits() & 0xFFFF) as u32
        //(self.t.cnt.read().cnt().bits() & 0xFFFF) as u32
    }
    #[inline(always)]
    fn update(&mut self) {
        let cycles_since_sync = self.get_cycles_since_sync() as i32; // important: must be executed first
        atomic::compiler_fence(Ordering::SeqCst);
        let input_period = self.get_period() as i32;
        let previous_input_period = self.previous_input_periods[self.previous_input_periods_index] as i32;
        if (previous_input_period - input_period).abs() > ((MIN_H_PERIOD as i32) / 10) {
            // don't update any averages for outliers
        } else {
            self.previous_input_periods[self.previous_input_periods_index] = input_period as u32;
            self.previous_input_periods_index = (self.previous_input_periods_index + 1) % HSyncCapture::AVERAGED_INPUT_PERIODS;
            let error = if cycles_since_sync < (input_period as i32 / 2) {
                cycles_since_sync.max(64)
            } else {
                (cycles_since_sync - input_period as i32).min(-16)
            };
            self.previous_phase_errors[self.previous_phase_errors_index] = error;
            self.previous_phase_errors_index = (self.previous_phase_errors_index + 1) % HSyncCapture::AVERAGED_PHASE_ERRORS;
        }
    }
    fn _apply_phase_feedforward(&mut self, b: i32) {
        self.previous_phase_errors[self.previous_phase_errors_index] -= b * 1;
    }
    fn get_phase_error_averaged(&self) -> f32 {
        let mut avg: f32 = 0.0;
        for a in self.previous_phase_errors {
            avg += a as f32;
        }
        avg = avg / HSyncCapture::AVERAGED_PHASE_ERRORS as f32;
        return avg;
    }
    fn get_period_averaged(&self) -> f32 {
        let mut avg = 0.0;
        for a in self.previous_input_periods {
            avg += a as f32;
        }
        avg = avg / HSyncCapture::AVERAGED_INPUT_PERIODS as f32;
        return avg;
    }
    fn get_period(&self) -> u32 {
        // not entirely sure why this is +2 but it seems to be better
        (self.t.ccr1.read().ccr().bits() & 0xFFFF) as u32 + 2
    }
}

pub struct VDrive {
    dac: DAC,
    setpoint: f32,
    accumulator: f32,
}

impl VDrive {
    fn new(dac: DAC) -> VDrive {
        dac.cr.write(|w| { w.en1().enabled().boff1().enabled().tsel1().software().ten1().enabled() });
        dac.dhr12r1.write(|w| { unsafe { w.bits(DAC_MIDPOINT as u32) }});
        VDrive { dac, setpoint: 0.0, accumulator: 0.0 }
    }
    fn set_current(&mut self, current: f32) {
        self.setpoint = current;
    }
    /// call as soon as possible on horizontal interrupt
    #[inline(always)]
    fn trigger(&mut self) {
        self.dac.swtrigr.write(|w| { w.swtrig1().enabled() });
    }
    /// call once per horizontal interrupt
    fn update(&mut self) {
        let amps_to_volts = 1.0;
        let volts_to_dac_value = 1.0/(3.3/4095.0);
        let dac_value = self.setpoint * amps_to_volts * volts_to_dac_value;
        let dac_value_quantized = (dac_value + self.accumulator) as i32;
        self.accumulator += dac_value - dac_value_quantized as f32;
        let dac_value_final = (dac_value_quantized as i32 + DAC_MIDPOINT).clamp(0, 4095);
        self.dac.dhr12r1.write(|w| { unsafe { w.bits(dac_value_final as u32) }});
    }
}

pub struct Serial {
    usart: USART1,
    send_queue: Queue<u8, 1024>,
}

impl Serial {
    fn new(usart: USART1) -> Serial {
        usart.brr.write(|w| { w.brr().bits((APB2_CLOCK*2*8/16/230400) as u16)});
        usart.cr1.write(|w| { w.te().enabled().re().enabled().ue().enabled().rxneie().enabled().txeie().enabled() });
        Serial {
            send_queue: Queue::new(),
            usart
        }
    }
    fn _write_blocking(&self, b: &[u8]) {
        for c in b {
            while self.usart.isr.read().txe() == false {}
            self.usart.tdr.write(|w| { unsafe { w.tdr().bits((*c).into()) }});
        }
    }
    fn write_queued(&mut self, b: &[u8]) {
        for c in b {
            let _ = self.send_queue.enqueue(*c);
        }
    }
    fn read_byte(&self) -> u8 {
        return self.usart.rdr.read().bits() as u8
    }
}

pub struct SerialProtocol {
    serial: Serial,
    raw_message: Vec<u8, 4096>,
}

#[allow(dead_code)]
impl SerialProtocol {
    fn new(serial: Serial) -> SerialProtocol {
        SerialProtocol {
            serial,
            raw_message: Vec::new(),
        }
    }
    fn process_byte(&mut self, config_queue_in: &mut Producer<'static, Config, 2>) {
        let b = self.serial.read_byte();
        if b == b'\n' {
            self.process_message(config_queue_in);
        } else {
            let _ = self.raw_message.push(b);
        }
    }
    fn process_message(&mut self, config_queue_in: &mut Producer<'static, Config, 2>) {
        let parsed_config: Result<(CRTConfig, _), serde_json_core::de::Error> = serde_json_core::from_slice(&self.raw_message);
        if let Ok((crt_config, _)) = parsed_config {
            let _ = config_queue_in.enqueue(Config { crt: crt_config, input: InputConfig { h_size: 0.95, h_phase: 0.0 } });
        }
        self.raw_message.clear();
    }
}

pub struct ADC {
    adc1: ADC1
}

impl ADC {
    fn new(adc1: ADC1) -> ADC {
        adc1.cr2.modify(|_,w| {w.adon().bit(true)});
        ADC { adc1 }
    }
    fn read_blocking(&self, channel: u8) -> u16 {
        self.adc1.sqr3.modify(|_,w| { unsafe { w.sq1().bits(channel) } });
        self.adc1.cr2.modify(|_,w| { w.swstart().set_bit() });
        while !self.adc1.sr.read().eoc().bit() {};
        self.adc1.dr.read().data().bits()
    }
}
