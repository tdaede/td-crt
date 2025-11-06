use stm32g4::stm32g474::*;

pub struct VDriveClassD {
    hrte: HRTIM_TIME,
    hrtf: HRTIM_TIMF,
    current_setpoint: f32,
    current_previous: f32,
}

const DEADTIME: u16 = 60;

const PERIOD: u16 = 0x2000;
const INPUT_VOLTAGE: f32 = 36.0;
const V_RESISTANCE: f32 = 50.6; //13.243;
const V_INDUCTANCE: f32 =  115.0e-3; //83.35e-3;//27.86e-3;
const MIDPOINT: f32 = (PERIOD as f32) / 2.0;
// maximum duty cycle controlled by bootstrap for high side mosfet
// make min duty cycle symmetrical to avoid creating a dc component
const DUTY_CYCLE_MIN: f32 = 0.20;
const DUTY_CYCLE_MAX: f32 = 0.80;
const COUNTS_MIN: u16 = (DUTY_CYCLE_MIN * PERIOD as f32) as u16;
const COUNTS_MAX: u16 = (DUTY_CYCLE_MAX * PERIOD as f32) as u16;


impl VDriveClassD {
    pub fn new(rcc: &RCC, gpioc: &GPIOC, hrc: &HRTIM_COMMON, hrm: &HRTIM_MASTER, hrte: HRTIM_TIME, hrtf: HRTIM_TIMF) -> VDriveClassD {
        rcc.apb2enr().modify(|_,w| { w.hrtim1en().enabled() });
        // start hrtim calibration
        hrc.dllcr().modify(|_,w| { w.cal().set_bit() });
        while !hrc.isr().read().dllrdy().bit() {};
        hrte.outer().modify(|_,w| { unsafe { w.pol1().clear_bit()
                                             .pol2().clear_bit()
                                             .fault1().bits(0b10)
                                             .fault2().bits(0b10)
                                             .dten().set_bit() } });
        hrtf.outfr().modify(|_,w| { unsafe { w.pol1().clear_bit()
                                             .pol2().clear_bit()
                                             .fault1().bits(0b10)
                                             .fault2().bits(0b10)
                                             .dten().set_bit() } });
        hrte.dter().modify(|_,w| { unsafe { w.dtrx().bits(DEADTIME).dtfx().bits(DEADTIME) } });
        hrtf.dtfr().modify(|_,w| { unsafe { w.dtrx().bits(DEADTIME).dtfx().bits(DEADTIME) } });
        hrte.perer().write(|w| { unsafe { w.perx().bits(PERIOD) } });
        hrtf.perfr().write(|w| { unsafe { w.perx().bits(PERIOD) } });
        hrte.sete1r().modify(|_,w| { w.cmp1().set_bit() });
        hrte.rste1r().modify(|_,w| { w.per().set_bit() });
        hrtf.setf1r().modify(|_,w| { w.cmp1().set_bit() });
        hrtf.rstf1r().modify(|_,w| { w.per().set_bit() });
        hrte.timecr().modify(|_,w| { w.cont().set_bit().preen().set_bit().tx_rstu().set_bit() });
        hrtf.timfcr().modify(|_,w| { w.cont().set_bit().preen().set_bit().tx_rstu().set_bit() });
        hrte.cmp1er().write(|w| { unsafe { w.cmp1x().bits(MIDPOINT as u16) } });
        hrtf.cmp1fr().write(|w| { unsafe { w.cmp1x().bits(MIDPOINT as u16) } });
        hrm.mcr().modify(|_,w| { w.tecen().set_bit().tfcen().set_bit() });
        gpioc.afrl().modify(|_,w| { w.afrl6().af13()
                                    .afrl7().af13() });
        gpioc.afrh().modify(|_,w| { w.afrh8().af3()
                                    .afrh9().af3() });
        gpioc.moder().modify(|_,w| { w
                                     .moder6().alternate()
                                     .moder7().alternate()
                                     .moder8().alternate()
                                     .moder9().alternate()
        });
        hrc.oenr().write(|w| { w.te1oen().set_bit()
                                .te2oen().set_bit()
                                .tf1oen().set_bit()
                                .tf2oen().set_bit() });
        VDriveClassD {
            hrte,
            hrtf,
            current_setpoint: 0.0,
            current_previous: 0.0
        }
    }
    pub fn set_current(&mut self, current: f32) {
        self.current_setpoint = current;
    }
    pub fn update(&mut self, timestep: f32) {
        let current_delta = self.current_setpoint - self.current_previous;
        let target_voltage = self.current_setpoint * V_RESISTANCE + current_delta * V_INDUCTANCE / timestep;
        let target_voltage_a = target_voltage*0.5;
        let target_voltage_b = target_voltage*-0.5;
        let volts_per_count = INPUT_VOLTAGE / PERIOD as f32;
        let target_counts_a = (MIDPOINT + target_voltage_a / volts_per_count) as u16;
        let target_counts_b = (MIDPOINT + target_voltage_b / volts_per_count) as u16;
        let clamped_counts_a = target_counts_a.clamp(COUNTS_MIN, COUNTS_MAX);
        let clamped_counts_b = target_counts_b.clamp(COUNTS_MIN, COUNTS_MAX);
        self.hrte.cmp1er().write(|w| { unsafe { w.cmp1x().bits(clamped_counts_a) } });
        self.hrtf.cmp1fr().write(|w| { unsafe { w.cmp1x().bits(clamped_counts_b) } });
        // because during retrace we are likely duty cycle limited,
        // we compute the state by going backwards
        let real_voltage_a = (clamped_counts_a as f32 - MIDPOINT) * volts_per_count;
        let real_voltage_b = (clamped_counts_b as f32 - MIDPOINT) * volts_per_count;
        let real_voltage = real_voltage_a - real_voltage_b;
        // V = L di/dt
        // V*dt/L = di
        self.current_previous += (real_voltage - self.current_previous * V_RESISTANCE)*timestep/V_INDUCTANCE;
    }
}
