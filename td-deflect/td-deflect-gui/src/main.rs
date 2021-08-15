use gtk::prelude::*;
use gtk::{Application, ApplicationWindow, Label, Box, Orientation, ComboBoxText, SpinButton, Grid};
use glib::{Continue, MainContext, PRIORITY_DEFAULT};
use gtk::glib;
use std::{thread, time::Duration};
use std::io::BufReader;
use std::io::BufRead;
use serialport;
use serde::{Serialize, Deserialize};
use gtk::pango::{AttrList, Attribute};
use std::sync::mpsc::channel;
use std::rc::Rc;
use glib::clone;

#[derive(Default, Copy, Clone, Deserialize)]
#[allow(unused)]
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

#[derive(Copy, Clone, Serialize)]
#[allow(unused)]
pub struct CRTConfig {
    v_mag_amps: f32,
    v_mag_offset: f32,
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

    let stats_box = Box::new(Orientation::Vertical, 10);
    let serial_selector = ComboBoxText::new();
    for port in serialport::available_ports().unwrap() {
        serial_selector.append_text(&port.port_name);
    }
    let tnum = AttrList::new();
    tnum.insert(Attribute::new_font_features("tnum"));
    serial_selector.set_active(Some(2));
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
    let geometry_settings_grid = Grid::new();
    geometry_settings_grid.set_column_spacing(10);
    let vertical_current_magnitude_label = Label::new(Some("Vertical current magnitude:"));
    geometry_settings_grid.attach(&vertical_current_magnitude_label, 0, 0, 1, 1);
    let vertical_current_magnitude_adj = SpinButton::with_range(0.0, 2.0, 0.01);
    vertical_current_magnitude_adj.set_value(0.45);
    geometry_settings_grid.attach(&vertical_current_magnitude_adj, 1, 0, 1, 1);
    let vertical_current_offset_label = Label::new(Some("Vertical current offset:"));
    geometry_settings_grid.attach(&vertical_current_offset_label, 0, 1, 1, 1);
    let vertical_current_offset_adj = SpinButton::with_range(-2.0, 2.0, 0.01);
    vertical_current_offset_adj.set_value(0.0);
    geometry_settings_grid.attach(&vertical_current_offset_adj, 1, 1, 1, 1);
    stats_box.append(&geometry_settings_grid);

    window.set_child(Some(&stats_box));

    let (sender, receiver) = MainContext::channel::<CRTStats>(PRIORITY_DEFAULT);

    let (tx_channel_sender, tx_channel_receiver) = channel::<CRTConfig>();
    let tx_channel_sender_rc = Rc::new(tx_channel_sender);

    let serial_port_string = String::from(serial_selector.active_text().unwrap());

    let send_crt_config = clone!(
        @strong tx_channel_sender_rc,
        @weak vertical_current_magnitude_adj,
        @weak vertical_current_offset_adj => move || {
        let crt_config = CRTConfig {
            v_mag_amps: vertical_current_magnitude_adj.value() as f32,
            v_mag_offset: vertical_current_offset_adj.value() as f32
        };
        tx_channel_sender_rc.send(crt_config).unwrap();
    });

    vertical_current_magnitude_adj.connect_value_changed(clone!(@strong send_crt_config => move |_| {
        send_crt_config();
    }));

    vertical_current_offset_adj.connect_value_changed(clone!(@strong send_crt_config => move |_|  {
        send_crt_config();
    }));

    thread::spawn(move || {
        let mut serial = serialport::new(serial_port_string, 230400).open().expect("Failed to open port");
        serial.set_timeout(Duration::from_millis(100)).unwrap();
        let mut reader = BufReader::new(serial.try_clone().unwrap());
        loop {
            let mut l = String::new();
            if let Ok(_) = reader.read_line(&mut l) {
                let parse_result: Result<CRTStats, serde_json::Error> = serde_json::from_str(&l);
                if let Ok(stats) = parse_result {
                    sender.send(stats).expect("Could not send through channel");
                }
            }
            if let Ok(crt_config) = tx_channel_receiver.try_recv() {
                if let Ok(serialized_crt_config) = serde_json::to_vec(&crt_config) {
                    serial.write(&serialized_crt_config).unwrap();
                    serial.write(&[b'\n']).unwrap();
                }
            }
        }
    });

    receiver.attach(
        None,
        move |a| {
            let output_horizontal_period_us = a.h_output_period as f32 * 1_000_000.0 / (216000000.0);
            sync_freq_label.set_label(&format!("Output horizontal period: {:.2} us", output_horizontal_period_us));
            input_horizontal_period_label.set_label(&format!("Input horizontal period: {:.2} us", a.h_input_period as f32 * 1_000_000.0 / (216000000.0)));
            input_horizontal_period_min_label.set_label(&format!("{:.2} us", a.h_input_period_min as f32 * 1_000_000.0 / (216000000.0)));
            input_horizontal_period_max_label.set_label(&format!("{:.2} us", a.h_input_period_max as f32 * 1_000_000.0 / (216000000.0)));
            output_horizontal_period_min_label.set_label(&format!("{:.2} us", a.h_output_period_min as f32 * 1_000_000.0 / (216000000.0)));
            output_horizontal_period_max_label.set_label(&format!("{:.2} us", a.h_output_period_max as f32 * 1_000_000.0 / (216000000.0)));
            field_lines_label.set_label(&format!("Lines in last field: {:.2}", a.v_lines));
            vertical_period_label.set_label(&format!("Vertical period: {:.2} ms", output_horizontal_period_us * a.v_lines as f32 / 1000.0));
            s_voltage_label.set_label(&format!("S-cap voltage: {:.2} V", a.s_voltage as f32 * 3.3 / 4096.0 * 101.0));
            Continue(true)
        }
    );

    window.present();
}
