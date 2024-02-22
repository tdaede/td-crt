use stm32g4::stm32g474::*;

pub struct VDriveClassD {
    hrte: HRTIM_TIME,
    hrtf: HRTIM_TIMF
}

const DEADTIME: u16 = 200;

const PERIOD: u16 = 0x4000;
const INPUT_VOLTAGE: f32 = 24.0;
const H_RESISTANCE: f32 = 8.0;
const MIDPOINT: f32 = (PERIOD as f32) / 2.0;

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
            hrtf
        }
    }
    pub fn set_current(&self, current: f32) {
        let target_voltage = current * H_RESISTANCE;
        let target_voltage_low = target_voltage*0.5;
        let target_voltage_high = target_voltage*-0.5;
        let volts_per_count = INPUT_VOLTAGE / PERIOD as f32;
        self.hrte.cmp1er().write(|w| { unsafe { w.cmp1x().bits((MIDPOINT + target_voltage_low / volts_per_count) as u16) } });
        self.hrtf.cmp1fr().write(|w| { unsafe { w.cmp1x().bits((MIDPOINT + target_voltage_high / volts_per_count) as u16) } });
    }
}
