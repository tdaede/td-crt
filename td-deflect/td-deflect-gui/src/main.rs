mod scope;

use gtk::prelude::*;
use gtk::{Application, ApplicationWindow, Label, Box, Orientation, ComboBoxText, SpinButton, Grid, Frame, Scale};
use glib::MainContext;
use gtk::glib;
use std::{thread, time::Duration};
use std::io::BufReader;
use std::io::BufRead;
use serde::Serialize;
use gtk::pango::{AttrList, AttrFontFeatures};
use std::sync::mpsc::channel;
use std::rc::Rc;
use glib::clone;
use td_crt_protocol::CRTStats;
use scope::Scope;

const SYSTEM_CLOCK: f32 = 170000000.0;

#[derive(Copy, Clone, Serialize)]
#[allow(unused)]
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
#[derive(Copy, Clone, Serialize)]
pub struct InputConfig {
    h_size: f32,
    h_phase: f32,
}

/// Complete runtime configuration, both CRT config + input config
#[derive(Copy, Clone, Serialize)]
pub struct Config {
    crt: CRTConfig,
    input: InputConfig,
}

fn main() {
    // Create a new application
    let app = Application::new(Some("com.thomasdaede.td-deflect-gui"), Default::default());
    app.connect_activate(build_ui);

    // Run the application
    app.run();
}

fn build_ui(app: &Application) {
    // Create a window
    let window = ApplicationWindow::new(app);

    // Set the window title
    window.set_title(Some("TD-Deflect GUI"));

    // widgets that instantly result in a new config message being sent
    let mut config_spin_widgets = Vec::new();
    let mut config_scale_widgets = Vec::new();

    let stats_box = Box::new(Orientation::Vertical, 10);
    let serial_selector = ComboBoxText::new();
    let mut default_index = 0;
    for (index, port) in serialport::available_ports().unwrap().iter().enumerate() {
        match &port.port_type {
            serialport::SerialPortType::UsbPort(usbport) => {
                if usbport.vid == 6790 && usbport.pid == 29987 {
                    default_index = index;
                }
            },
            _ => {}
        }
        serial_selector.append_text(&port.port_name);
    }
    let tnum = AttrList::new();
    tnum.insert(AttrFontFeatures::new("tnum"));
    serial_selector.set_active(Some(default_index as u32));
    stats_box.append(&serial_selector);
    let line_stats_grid = Grid::new();
    line_stats_grid.set_column_spacing(10);
    line_stats_grid.attach(&Label::new(Some("Min")), 1, 0, 1, 1);
    line_stats_grid.attach(&Label::new(Some("Max")), 2, 0, 1, 1);
    line_stats_grid.attach(&Label::new(Some("Input horizontal period")), 0, 1, 1, 1);
    let input_horizontal_period_min_label = Label::new(None);
    line_stats_grid.attach(&input_horizontal_period_min_label, 1, 1, 1, 1);
    let input_horizontal_period_max_label = Label::new(None);
    line_stats_grid.attach(&input_horizontal_period_max_label, 2, 1, 1, 1);
    line_stats_grid.attach(&Label::new(Some("Output horizontal period")), 0, 2, 1, 1);
    let output_horizontal_period_min_label = Label::builder().attributes(&tnum).build();
    line_stats_grid.attach(&output_horizontal_period_min_label, 1, 2, 1, 1);
    let output_horizontal_period_max_label = Label::builder().attributes(&tnum).build();
    line_stats_grid.attach(&output_horizontal_period_max_label, 2, 2, 1, 1);
    stats_box.append(&line_stats_grid);
    let input_horizontal_period_label = Label::new(Some("Input horizontal period: 0us"));
    stats_box.append(&input_horizontal_period_label);
    let sync_freq_label = Label::new(Some("Output horizontal period: 0us"));
    stats_box.append(&sync_freq_label);
    let field_lines_label = Label::new(Some("Lines in last field: 0"));
    stats_box.append(&field_lines_label);
    let vertical_period_label = Label::new(Some("Vertical period: 0 ms"));
    stats_box.append(&vertical_period_label);
    let last_field_phase_label = Label::new(Some("Last field phase: Even"));
    stats_box.append(&last_field_phase_label);
    let s_voltage_label = Label::new(Some("S-cap voltage: 0 V"));
    s_voltage_label.set_attributes(Some(&tnum));
    stats_box.append(&s_voltage_label);

    // CRT Configuration Settings
    let geometry_settings_frame = Frame::new(Some("CRT Configuration"));
    let geometry_settings_grid = Grid::new();
    geometry_settings_grid.set_column_spacing(10);
    let mut y_pos = 0;

    // Vertical Settings
    let vertical_current_magnitude_label = Label::new(Some("Vertical current magnitude (A):"));
    geometry_settings_grid.attach(&vertical_current_magnitude_label, 0, y_pos, 1, 1);
    let vertical_current_magnitude_adj = SpinButton::with_range(0.0, 2.0, 0.01);
    config_spin_widgets.push(&vertical_current_magnitude_adj);
    vertical_current_magnitude_adj.set_value(0.23);
    geometry_settings_grid.attach(&vertical_current_magnitude_adj, 1, y_pos, 1, 1);
    y_pos += 1;

    let vertical_current_offset_label = Label::new(Some("Vertical current offset (A):"));
    geometry_settings_grid.attach(&vertical_current_offset_label, 0, y_pos, 1, 1);
    let vertical_current_offset_adj = SpinButton::with_range(-2.0, 2.0, 0.01);
    config_spin_widgets.push(&vertical_current_offset_adj);
    vertical_current_offset_adj.set_value(0.0);
    geometry_settings_grid.attach(&vertical_current_offset_adj, 1, y_pos, 1, 1);
    geometry_settings_frame.set_child(Some(&geometry_settings_grid));
    y_pos += 1;

    geometry_settings_grid.attach(&Label::new(Some("V. Linearity")), 0, y_pos, 1, 1);
    let v_lin = Scale::with_range(Orientation::Horizontal, 0.0, 1.0, 0.01);
    config_scale_widgets.push(&v_lin);
    v_lin.set_value(0.55);
    v_lin.set_width_request(200);
    geometry_settings_grid.attach(&v_lin, 1, y_pos, 1, 1);
    y_pos += 1;

    // Horizontal Settings
    geometry_settings_grid.attach(&Label::new(Some("S Correction")), 0, y_pos, 1, 1);
    let s_cap = SpinButton::with_range(0.0, 15.0, 1.0);
    config_spin_widgets.push(&s_cap);
    s_cap.set_value(1.0);
    geometry_settings_grid.attach(&s_cap, 1, y_pos, 1, 1);

    stats_box.append(&geometry_settings_frame);

    // Per-input settings
    y_pos = 0;
    let input_settings_frame = Frame::new(Some("Input Configuration"));
    let input_settings_grid = Grid::new();
    input_settings_grid.set_column_spacing(10);

    input_settings_grid.attach(&Label::new(Some("H. Size")), 0, y_pos, 1, 1);
    let input_settings_h_size = Scale::with_range(Orientation::Horizontal, 0.5, 0.95, 0.01);
    config_scale_widgets.push(&input_settings_h_size);
    input_settings_h_size.set_value(0.9);
    input_settings_h_size.set_width_request(200);
    input_settings_grid.attach(&input_settings_h_size, 1, y_pos, 1, 1);
    y_pos += 1;

    input_settings_grid.attach(&Label::new(Some("H. Phase")), 0, y_pos, 1, 1);
    let h_phase = Scale::with_range(Orientation::Horizontal, -0.1, 0.1, 0.01);
    config_scale_widgets.push(&h_phase);
    h_phase.set_value(0.05);
    h_phase.set_width_request(200);
    input_settings_grid.attach(&h_phase, 1, y_pos, 1, 1);

    input_settings_frame.set_child(Some(&input_settings_grid));
    stats_box.append(&input_settings_frame);

    let vertical_scope = Scope::new();
    vertical_scope.set_width_request(200);
    vertical_scope.set_height_request(200);
    stats_box.append(&vertical_scope);

    window.set_child(Some(&stats_box));

    let (sender, receiver) = async_channel::unbounded();

    let (tx_channel_sender, tx_channel_receiver) = async_channel::bounded(1);

    let serial_port_string = String::from(serial_selector.active_text().unwrap());

    let send_crt_config = clone!(
        #[weak] vertical_current_magnitude_adj,
        #[weak] vertical_current_offset_adj,
        #[weak] v_lin,
        #[weak] s_cap,
        #[weak] input_settings_h_size,
        #[weak] h_phase,
        move || {
            let crt_config = CRTConfig {
                v_mag_amps: vertical_current_magnitude_adj.value() as f32,
                v_offset_amps: vertical_current_offset_adj.value() as f32,
                vertical_linearity: v_lin.value() as f32,
                s_cap: s_cap.value() as u8,
            };
            let input_config = InputConfig {
                h_size: input_settings_h_size.value() as f32,
                h_phase: h_phase.value() as f32,
            };
            tx_channel_sender.force_send(Config {crt: crt_config, input: input_config}).unwrap();
    });

    for w in config_spin_widgets {
        w.connect_value_changed(clone!(#[strong] send_crt_config, move |_| {
            send_crt_config();
        }));
    }
    for w in config_scale_widgets {
        w.connect_value_changed(clone!(#[strong] send_crt_config, move |_| {
            send_crt_config();
        }));
    }

    thread::spawn(move || {
        let mut serial = serialport::new(serial_port_string, 1_000_000).open().expect("Failed to open port");
        serial.set_timeout(Duration::from_millis(100)).unwrap();
        let mut reader = BufReader::new(serial.try_clone().unwrap());
        loop {
            let mut l = String::new();
            if reader.read_line(&mut l).is_ok() {
                let parse_result: Result<CRTStats, serde_json::Error> = serde_json::from_str(&l);
                if let Ok(stats) = parse_result {
                    sender.send_blocking(stats).expect("Could not send through channel");
                } else {
                    eprintln!("got corrupted stats message");
                }
            }
            if let Ok(config) = tx_channel_receiver.try_recv() {
                if let Ok(serialized_config) = serde_json::to_vec(&config) {
                    serial.write_all(&serialized_config).unwrap();
                    serial.write_all(&[b'\n']).unwrap();
                }
            }
        }
    });

    let main_context = MainContext::default();
    main_context.spawn_local(
        async move {
            while let Ok(a) = receiver.recv().await {
                let output_horizontal_period_us = a.h_output_period as f32 * 1_000_000.0 / SYSTEM_CLOCK;
                sync_freq_label.set_label(&format!("Output horizontal period: {:.2} us", output_horizontal_period_us));
                input_horizontal_period_label.set_label(&format!("Input horizontal period: {:.2} us", a.h_input_period as f32 * 1_000_000.0 / SYSTEM_CLOCK));
                input_horizontal_period_min_label.set_label(&format!("{:.2} us", a.h_input_period_min as f32 * 1_000_000.0 / SYSTEM_CLOCK));
                input_horizontal_period_max_label.set_label(&format!("{:.2} us", a.h_input_period_max as f32 * 1_000_000.0 / SYSTEM_CLOCK));
                output_horizontal_period_min_label.set_label(&format!("{:.2} us", a.h_output_period_min as f32 * 1_000_000.0 / SYSTEM_CLOCK));
                output_horizontal_period_max_label.set_label(&format!("{:.2} us", a.h_output_period_max as f32 * 1_000_000.0 / SYSTEM_CLOCK));
                field_lines_label.set_label(&format!("Lines in last field: {:.2}", a.v_lines));
                last_field_phase_label.set_label(&format!("Last field phase: {}", if a.odd { "Odd" } else { "Even" }));
                vertical_period_label.set_label(&format!("Vertical period: {:.2} ms", output_horizontal_period_us * a.v_lines as f32 / 1000.0));
                s_voltage_label.set_label(&format!("S-cap voltage: {:.2} V", a.s_voltage));
                vertical_scope.set_values(vec![a.vertical_target_current_per_scanline.to_vec(), a.vertical_class_d_current_per_scanline.to_vec()]);
            }
        }
    );

    window.present();
}
