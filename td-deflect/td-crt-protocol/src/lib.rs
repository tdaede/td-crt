#![no_std]

use serde::{Serialize, Deserialize};
use heapless::Vec;

/// Debug stats
#[derive(Default, Clone, Serialize, Deserialize)]
pub struct CRTStats {
    pub h_output_period: i32,
    pub h_output_period_min: i32,
    pub h_output_period_max: i32,
    pub h_input_period: i32,
    pub h_input_period_min: i32,
    pub h_input_period_max: i32,
    pub hot_source_current: u16,
    pub v_lines: u16,
    pub s_voltage: f32,
    pub odd: bool,
    pub faulted: bool,
    pub vertical_class_d_current_per_scanline: Vec<f32, 512>,
    pub vertical_target_current_per_scanline: Vec<f32, 512>,
}

/// Configuration to match driver board to a particular tube/yoke
#[allow(unused)]
#[derive(Copy, Clone, Deserialize)]
pub struct CRTConfig {
    pub v_mag_amps: f32,
    pub v_offset_amps: f32,
    #[serde(default)]
    pub vertical_linearity: f32,
    // s-capacitor value, 0 = highest capacitance
    #[serde(default)]
    pub s_cap: u8,
}

/// Configuration for a particular input
#[allow(unused)]
#[derive(Copy, Clone, Deserialize)]
pub struct InputConfig {
    pub h_size: f32,
    pub h_phase: f32,
}
