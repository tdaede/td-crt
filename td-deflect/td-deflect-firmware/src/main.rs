#![no_main]
#![no_std]

mod adc;
mod horizontal;
mod serial;
mod v_class_d;
mod scan_fault;

use rtic::app;

#[app(device = stm32g4::stm32g474, peripherals = true, dispatchers = [EXTI0, EXTI1])]
mod app {
    use stm32g4::stm32g474::*;
    use cortex_m::asm::delay;
    use serde::{Deserialize};
    use heapless::spsc::{Queue, Producer, Consumer};
    use core::sync::atomic::{self, Ordering, AtomicBool};
    use core::panic::PanicInfo;
    use td_crt_protocol::*;
    use crate::adc::ADC;
    use crate::serial::{Serial, SerialProtocol};
    use crate::v_class_d::VDriveClassD;
    use crate::scan_fault::ScanFault;
    use crate::horizontal::HOTDriver;

    pub const AHB_CLOCK: u32 = 170_000_000;
    pub const APB2_CLOCK: u32 = AHB_CLOCK;

    pub const MIN_H_FREQ: u32 = 15000;
    pub const MAX_H_FREQ: u32 = 16000;
    #[allow(unused)]
    pub const MAX_H_PERIOD: u32 = AHB_CLOCK / MIN_H_FREQ;
    pub const MIN_H_PERIOD: u32 = AHB_CLOCK / MAX_H_FREQ;

    const DAC_MIDPOINT: i32 = 2047;

    static FAULTED: AtomicBool = AtomicBool::new(false);

    /// Immediately blanks the video signal.
    /// Used in case of deflection failure to prevent burn-in.
    fn fault_blank() {
        // read-modify-write is a bit unfortunate here
        unsafe { (*GPIOC::ptr()).odr().modify(|_,w| {w.odr14().bit(true)}); }
    }

    #[inline(never)]
    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        FAULTED.store(true, Ordering::Relaxed);
        fault_blank();
        loop {
            atomic::compiler_fence(Ordering::SeqCst);
        }
    }

    #[shared]
    struct Shared {
        serial_protocol: SerialProtocol,
    }
    #[local]
    struct Local {
        gpioa: GPIOA,
        gpiob: GPIOB,
        gpioc: GPIOC,
        gpioe: GPIOE,
        v_drive: VDrive,
        v_drive_classd: VDriveClassD,
        hot_driver: HOTDriver,
        hsync_capture: HSyncCapture,
        crt_state: CRTState,
        crt_stats_live: CRTStats,
        adc: ADC,
        config: Config,
        config_queue_in: Producer<'static, Config, 2>,
        config_queue_out: Consumer<'static, Config, 2>,
    }
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let s = cx.device;

        let rcc = s.RCC;
        let gpioa = s.GPIOA;
        let gpiob = s.GPIOB;
        let gpioc = s.GPIOC;
        let gpioe = s.GPIOE;
        let dac = s.DAC1;
        let flash = s.FLASH;
        let pwr = s.PWR;
        let usart1 = s.USART1;
        let adc1 = s.ADC1;
        let adc2 = s.ADC2;

        // enable pwr clock
        rcc.apb1enr1().write(|w| { w.pwren().bit(true) });

        // set maximum flash delay
        flash.acr().write(|w| {
            unsafe {
                w.prften().bit(true).dcen().set_bit().icen().set_bit().latency().bits(0b0100)
            }
        });
        // switch to 1.28v core voltage
        pwr.cr5().modify(|_,w| { w.r1mode().clear_bit() });
        // 16mhz hse / 4 * 75 = 300mhz pll output
        // divide by 2 for 170mhz main system clock
        rcc.pllcfgr().write(|w| { unsafe {
            w.pllsrc().bits(0b11).pllm().bits(3).plln().bits(85).pllren().set_bit()
        }});
        //enable hse and pll
        rcc.cr().write(|w| {
            w.hseon().set_bit().hsebyp().set_bit().pllon().set_bit()
        });
        // wait for pll lock
        while !rcc.cr().read().pllrdy().bit() {};
        // apb2 system, apb1 system, switch to pll as clock source
        // temporarily run ahb at clock rate / 2
        rcc.cfgr().write(|w| { unsafe {
            w.ppre2().bits(0b000).ppre1().bits(0b000).hpre().div2().sw().pll()}
        });
        while rcc.cfgr().read().sws().bits() != 0b11 {};
        delay(170); // wait 1us for stabilization
        rcc.cfgr().modify(|_,w| { w.hpre().div1() });

        rcc.ahb2enr().modify(|_,w| { w.gpioaen().bit(true) });
        rcc.ahb2enr().modify(|_,w| { w.gpioben().bit(true) });
        rcc.ahb2enr().modify(|_,w| { w.gpiocen().bit(true) });
        rcc.ahb2enr().modify(|_,w| { w.gpioeen().bit(true) });
        rcc.apb2enr().modify(|_,w| { w.tim1en().bit(true) });
        rcc.ahb2enr().modify(|_,w| { w.dac1en().bit(true) });
        //rcc.apb2enr.modify(|_,w| { w.tim10en().bit(true) });
        rcc.apb1enr1().modify(|_,w| { w.tim3en().bit(true) });
        rcc.apb1enr1().modify(|_,w| { w.tim4en().bit(true) });

        // HOT output
        // new TIM1 CH3 connected lines
        gpioa.afrh().modify(|_,w| { w.afrh10().af6() });
        gpioa.moder().modify(|_,w| { w.moder10().alternate() });

        // hsync input
        gpioa.afrl().modify(|_,w| { w.afrl6().af2() });
        gpioa.moder().modify(|_,w| { w. moder6().alternate() });
        // loopback signal from H.PIN OUT for phase measurement
        //gpiob.afrl.modify(|_,w| {w.afrl1().af2() }); // TIM3_CH4
        //gpiob.moder.modify(|_,w| {w.moder1().alternate() });

        // serial input/output
        let serial = Serial::new(usart1, &gpioc, &rcc);
        let serial_protocol = SerialProtocol::new(serial);

        // vertical sync inputs
        gpiob.moder().modify(|_,w| {w.moder0().input()});
        gpiob.pupdr().modify(|_,w| {w.pupdr0().pull_up()});
        gpioa.moder().modify(|_,w| {w.moder7().input()});
        gpioa.pupdr().modify(|_,w| {w.pupdr7().pull_up()});
        //gpioc.moder.modify(|_,w| {w.moder15().input()}); // oddeven

        // horizontal PWM
        gpioa.afrh().modify(|_,w| {w.afrh8().af6()});
        gpioa.moder().modify(|_,w| {w.moder8().alternate()});
        gpiob.afrh().modify(|_,w| {w.afrh13().af6()});
        gpiob.moder().modify(|_,w| {w.moder13().alternate()});

        // hv flyback transistor output
        gpiob.afrl().modify(|_,w| {w.afrl6().af2()});
        gpiob.moder().modify(|_,w| {w.moder6().alternate()});

        // blanking output to lm1205
        gpioe.moder().modify(|_,w| {w.moder7().output()});
        gpioe.odr().modify(|_,w| {w.odr7().bit(true)}); // start up with blanking enabled

        // S-cap lines
        // start with all S caps on, will change later in hsync timer interrupt
        gpiob.odr().modify(|_,w| { w.odr9().high() });
        gpioc.odr().modify(|_,w| { w.odr1().high().odr2().high().odr3().high() });
        gpiob.moder().modify(|_,w| { w.moder9().output() });
        gpioc.moder().modify(|_,w| { w.moder1().output().moder2().output().moder3().output() });

        // Vertical DAC output
        gpioa.moder().modify(|_,w| { w.moder4().analog() });
        // enable vertical SSR
        gpioc.odr().modify(|_,w| { w.odr13().set_bit() });
        gpioc.moder().modify(|_,w| { w.moder13().output() });

        // heater
        gpiob.odr().modify(|_,w| { w.odr4().high() });
        gpiob.moder().modify(|_,w| { w.moder4().output() });

        let adc = ADC::new(&s.ADC12_COMMON, adc1, adc2, &rcc, &gpioa, &gpiob, &gpioc);
        let v_drive = VDrive::new(dac);

        let v_drive_classd = VDriveClassD::new(&rcc, &gpioc, &s.HRTIM_COMMON, &s.HRTIM_MASTER, s.HRTIM_TIME, s.HRTIM_TIMF);

        let mut hot_driver = HOTDriver::new(s.TIM1, s.TIM4);
        let hsync_capture = HSyncCapture::new(s.TIM3);
        hot_driver.set_frequency(15700);

        // configure the system timer to wrap around every second
        //syst.set_clock_source(SystClkSource::Core);
        //syst.set_reload(AHB_CLOCK); // once a second
        //syst.enable_counter();
        let crt_stats_live = CRTStats::default();
        let crt_state = CRTState::default();
        let config = Config { crt: CRT_CONFIG_PANASONIC_CTN_1061R, input: InputConfig { h_size: 0.90, h_phase: 0.04 } };

        // config queue for purposes of "double buffering" config
        static mut CONFIG_QUEUE: Queue<Config, 2> = Queue::new();
        // unsafe: we can fix this with rtic 0.6
        let (config_queue_in, config_queue_out) = unsafe { CONFIG_QUEUE.split() };
        (Shared {
            serial_protocol,
        }, Local {
            gpioa,
            gpiob,
            gpioc,
            gpioe,
            config_queue_out,
            config_queue_in,
            adc,
            v_drive,
            v_drive_classd,
            hot_driver,
            hsync_capture,
            crt_stats_live,
            crt_state,
            config,
        })
    }

    // internal hsync timer interrupt
    #[inline(never)]
    #[link_section = ".data.TIM1_UP_TIM16"]
    #[task(binds = TIM1_UP_TIM16, local = [v_drive, v_drive_classd, gpioa, gpiob, gpioc, gpioe, hot_driver, hsync_capture,  crt_state, config, crt_stats_live, adc, config_queue_out], priority = 15)]
    fn tim1_up_tim16(cx: tim1_up_tim16::Context) {
        let crt_state = cx.local.crt_state;
        let current_scanline = &mut crt_state.current_scanline;
        let v_drive = cx.local.v_drive;
        let v_drive_classd = cx.local.v_drive_classd;
        let gpioa = cx.local.gpioa;
        let gpiob = cx.local.gpiob;
        let gpioc = cx.local.gpioc;
        let gpioe = cx.local.gpioe;
        let hot_driver = cx.local.hot_driver;
        let hsync_capture = cx.local.hsync_capture;
        let crt_stats = cx.local.crt_stats_live;
        let adc = cx.local.adc;
        let config = cx.local.config;

        hsync_capture.update();
        atomic::compiler_fence(Ordering::SeqCst);
        v_drive.trigger();

        let input_period = hsync_capture.get_period_averaged();
        let output_period = hot_driver.get_period() as i32;
        // if we are really far away frequency wise, just ignore this sync entirely
        if (output_period - input_period as i32).abs() > ((MIN_H_PERIOD as i32) / 10){
            // yolo
        } else {
            let error = hsync_capture.get_phase_error_averaged() + (config.input.h_phase * output_period as f32);
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

        hot_driver.duty_setpoint = config.input.h_size;
        hot_driver.update();

        let VERTICAL_MAX_AMPS = 1.0;
        let VERTICAL_BLANKING_SCANLINES = 20;
        // vertical advance
        // vertical counter
        let vga_vsync = gpiob.idr().read().idr0().bit(); // vga sync input
        let separated_vsync = gpioa.idr().read().idr7().bit(); // composite sync input
        let vsync = vga_vsync & separated_vsync;
        let odd = gpioc.idr().read().idr15().bit();
        // negative edge triggered
        let total_lines = 262;
        let center_line = total_lines / 2;
        if (vsync == false) && (crt_state.previous_vga_vsync == true) {
            crt_stats.v_lines = *current_scanline as u16;
            crt_stats.odd = odd;
            *current_scanline = 0;
        }
        crt_state.previous_vga_vsync = vsync;
        if *current_scanline >= (total_lines + 10) { *current_scanline = 0 };
        // handle vertical blanking
        if *current_scanline >= VERTICAL_BLANKING_SCANLINES {
            if !FAULTED.load(Ordering::Relaxed) &&
                !crt_state.v_scan_fault.get_fault() {
                gpioe.odr().modify(|_,w| {w.odr7().bit(true)}); // show image
            }
        } else {
            gpioe.odr().modify(|_,w| {w.odr7().bit(false)}); // blank image
        }
        // convert scanline to a (+1, -1) range coordinate (+1 is top of screen)
        let horizontal_pos_coordinate = (((*current_scanline) - center_line) as f32 + if odd { 0.0 } else { -0.5 }) / (total_lines as f32) * -2.0;
        let vertical_linearity = config.crt.vertical_linearity;
        let vertical_linearity_scale = 1.0 / libm::atanf(1.0 * vertical_linearity);
        let horizontal_coordinate_corrected = if vertical_linearity < 0.1 {
            horizontal_pos_coordinate
        } else {
            libm::atanf(horizontal_pos_coordinate * vertical_linearity) * vertical_linearity_scale
        };
        let horizontal_amps = (horizontal_coordinate_corrected * config.crt.v_mag_amps + config.crt.v_offset_amps).clamp(-1.0*VERTICAL_MAX_AMPS, VERTICAL_MAX_AMPS);
        v_drive.set_current(horizontal_amps);
        v_drive_classd.set_current(horizontal_amps);
        v_drive.update();
        v_drive_classd.update(1.0/15700.0); // TODO: use real timestep value

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
        let adc_horiz_data = adc.read_all_horiz();
        crt_state.v_scan_fault.push(adc_horiz_data.vertical_class_d_current_post);
        crt_stats.s_voltage = adc_horiz_data.s_voltage;
        let _ = crt_stats.vertical_class_d_current_per_scanline.push(adc_horiz_data.vertical_class_d_current_post);
        let _ = crt_stats.vertical_target_current_per_scanline.push(horizontal_amps);

        // update s capacitors
        gpiob.odr().modify(|_,w| { w.odr9().bit((config.crt.s_cap & 0b1000) == 0) });
        gpioc.odr().modify(|_,w| { w.odr1().bit((config.crt.s_cap & 0b0100) == 0)
                                 .odr2().bit((config.crt.s_cap & 0b0010) == 0)
                                 .odr3().bit((config.crt.s_cap & 0b0001) == 0) });

        crt_stats.faulted = FAULTED.load(Ordering::Relaxed);

        // send stats updates over serial
        if *current_scanline == 0 {
            let _ = send_stats::spawn(crt_stats.clone());
            *crt_stats = CRTStats::default(); // reset for a new frame
        }
        *current_scanline += 1;

        // update config if there's one in the queue
        if let Some(new_config) = cx.local.config_queue_out.dequeue() {
            *config = new_config;
        }

        // clear interrupt flag for tim10
        hot_driver.tp.sr().write(|w| w.uif().bit(false));
    }

    #[task(binds = USART1, local = [config_queue_in], shared = [serial_protocol], priority = 2)]
    fn serial_interrupt(mut c: serial_interrupt::Context) {
        // TODO: convert tx to use fifo interrupts
        // TODO: convert rx to use fifo interrupts
        let config_queue_in = c.local.config_queue_in;
        c.shared.serial_protocol.lock(|serial_protocol| {
            while serial_protocol.serial.usart.isr().read().rxne().bit() {
                serial_protocol.process_byte(config_queue_in);
            }
            if serial_protocol.serial.usart.isr().read().txe().bit() {
                if let Some(c) = serial_protocol.serial.send_queue.pop_front() {
                    serial_protocol.serial.usart.tdr().write(|w| { unsafe{ w.tdr().bits((c).into()) } });
                } else {
                    serial_protocol.serial.usart.cr1().modify(|_,w| { w.txeie().clear_bit() });
                }
            }
            serial_protocol.serial.usart.icr().write(|w| { w.orecf().set_bit() });
        });
    }

    #[task(shared = [serial_protocol], priority = 1)]
    async fn send_stats(mut c: send_stats::Context, crt_stats: CRTStats) {
        let mut json_stats: [u8; 8192] = [0; 8192];
        let mut json_stats_len = 0;
        if let Ok(c) = serde_json_core::to_slice(&crt_stats, &mut json_stats) { json_stats_len = c };
        c.shared.serial_protocol.lock(|serial_protocol| {
            if json_stats_len < json_stats.len() {
                json_stats[json_stats_len] = b'\n';
                let _ = serial_protocol.serial.write_queued(&json_stats[..json_stats_len + 1]);
            }
        });
    }


    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
              rtic::export::wfi()
        }
    }

    #[derive(Default)]
    pub struct CRTState {
        current_scanline: i32,
        previous_vga_vsync: bool, // used to detect negative edge sync
        v_scan_fault: ScanFault,
    }

    #[allow(unused)]
    static CRT_CONFIG_PANASONIC_S901Y: CRTConfig = CRTConfig {
        v_mag_amps: 0.414,
        v_offset_amps: 0.0,
        vertical_linearity: 0.55,
        s_cap: 1,
    };

    static CRT_CONFIG_PANASONIC_CTN_1061R: CRTConfig = CRTConfig {
        v_mag_amps: 0.53,
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
            t.smcr().write(|w| { unsafe { w.ts().bits(0b101).sms().bits(0b0100) }});
            // CH1 is configured to reset the counter and measure H period
            // CH4 is configured for capture to measure H phase error
            t.ccer().write(|w| { w.cc1p().set_bit().cc1e().set_bit().cc4p().clear_bit().cc4np().clear_bit().cc4e().set_bit() });
            t.cr1().write(|w| { w.cen().set_bit() });
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
            // hardware method
            //self.t.ccr4().read().ccr().bits() as u32
            // software method (relies on interrupt latency being exact)
            (self.t.cnt().read().cnt().bits() & 0xFFFF) as u32
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
            avg /= HSyncCapture::AVERAGED_PHASE_ERRORS as f32;
            return avg;
        }
        fn get_period_averaged(&self) -> f32 {
            let mut avg = 0.0;
            for a in self.previous_input_periods {
                avg += a as f32;
            }
            avg /= HSyncCapture::AVERAGED_INPUT_PERIODS as f32;
            return avg;
        }
        fn get_period(&self) -> u32 {
            // not entirely sure why this is +2 but it seems to be better
            self.t.ccr1().read().ccr().bits() as u32 + 2
        }
    }

    pub struct VDrive {
        dac: DAC1,
        setpoint: f32,
        accumulator: f32,
    }

    impl VDrive {
        fn new(dac: DAC1) -> VDrive {
            dac.cr().write(|w| { unsafe { w.en1().enabled().tsel1().bits(0b0000).ten1().enabled() } });
            dac.dhr12r1().write(|w| { unsafe { w.bits(DAC_MIDPOINT as u32) }});
            VDrive { dac, setpoint: 0.0, accumulator: 0.0 }
        }
        fn set_current(&mut self, current: f32) {
            self.setpoint = current;
        }
        /// call as soon as possible on horizontal interrupt
        #[inline(always)]
        fn trigger(&mut self) {
            self.dac.swtrgr().write(|w| { w.swtrig1().set_bit() });
        }
        /// call once per horizontal interrupt
        fn update(&mut self) {
            let amps_to_volts = 1.0;
            let volts_to_dac_value = 1.0/(3.3/4095.0);
            let dac_value = self.setpoint * amps_to_volts * volts_to_dac_value;
            let dac_value_quantized = (dac_value + self.accumulator) as i32;
            self.accumulator += dac_value - dac_value_quantized as f32;
            let dac_value_final = (dac_value_quantized as i32 + DAC_MIDPOINT).clamp(0, 4095);
            self.dac.dhr12r1().write(|w| { unsafe { w.bits(dac_value_final as u32) }});
        }
    }
}
