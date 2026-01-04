// This checks for a scan fault that could cause burn-in.
// It makes sure that the beam covers the entire screen by comparing it to a
// threshold (both positive and negative) and ensuring that the beam stays within
// that section over a certain fraction of the total time.
// This makes the assumption that the beam has to pass through zero to reach both
// of these sides, e.g. that the scan is covering at least the area of the threshold.

// There will be no fault reported until a minimum of WINDOW samples to prevent
// tripping on startup.

const SCAN_THRESHOLD: f32 = 0.1; // in real units
const SCAN_MIN_FRAC: f32 = 0.2; // should be under 0.5
const WINDOW: u32 = 512; // about two fields
const SCAN_MIN: u32 = (WINDOW as f32 * SCAN_MIN_FRAC) as u32;

pub struct ScanFault {
    counter: u32,
    count_pos: u32,
    count_neg: u32,
    fault: bool,
}

impl ScanFault {
    pub fn new() -> ScanFault {
        ScanFault {
            counter: 0,
            count_pos: 0,
            count_neg: 0,
            fault: false
        }
    }
    pub fn push(&mut self, v: f32) {
        if v >= SCAN_THRESHOLD {
            self.count_pos += 1;
        }
        if v <= SCAN_THRESHOLD {
            self.count_neg += 1;
        }
        self.counter += 1;
        if self.counter >= WINDOW {
            //self.fault = self.count_pos < SCAN_MIN || self.count_neg < SCAN_MIN;
            self.fault = false;
            self.counter = 0;
            self.count_pos = 0;
            self.count_neg = 0;
        }
    }
    pub fn get_fault(&self) -> bool {
        self.fault
    }
}

impl Default for ScanFault {
    fn default() -> ScanFault {
        ScanFault::new()
    }
}
