use gtk::prelude::*;
use gtk::{Application, ApplicationWindow, Label, Box, Orientation, ComboBoxText, SpinButton, Grid};
use glib::{Continue, MainContext, PRIORITY_DEFAULT};
use gtk::glib;
use std::{thread, time::Duration};
use serialport;

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
    serial_selector.set_active(Some(0));
    stats_box.append(&serial_selector);
    let sync_freq_label = Label::new(Some("Horizontal line width: 0us"));
    stats_box.append(&sync_freq_label);
    let field_lines_label = Label::new(Some("Lines in last field: 0"));
    stats_box.append(&field_lines_label);
    let last_field_phase_label = Label::new(Some("Last field phase: Even"));
    stats_box.append(&last_field_phase_label);
    let vertical_peak_current_label = Label::new(Some("Peak vertical current: 0 A"));
    stats_box.append(&vertical_peak_current_label);
    let horizontal_peak_current_label = Label::new(Some("Peak horizontal current: 0 A"));
    stats_box.append(&horizontal_peak_current_label);
    let hot_peak_current_label = Label::new(Some("Peak HOT current: 0 A"));
    stats_box.append(&hot_peak_current_label);
    let eht_peak_current_label = Label::new(Some("Peak EHT current: 0 mA"));
    stats_box.append(&eht_peak_current_label);
    let geometry_settings_grid = Grid::new();
    geometry_settings_grid.set_column_spacing(10);
    let vertical_current_magnitude_label = Label::new(Some("Vertical current magnitude:"));
    geometry_settings_grid.attach(&vertical_current_magnitude_label, 0, 0, 1, 1);
    let vertical_current_magnitude_adj = SpinButton::with_range(0.0, 2.0, 0.01);
    geometry_settings_grid.attach(&vertical_current_magnitude_adj, 1, 0, 1, 1);
    let vertical_current_offset_label = Label::new(Some("Vertical current offset:"));
    geometry_settings_grid.attach(&vertical_current_offset_label, 0, 1, 1, 1);
    let vertical_current_offset_adj = SpinButton::with_range(-2.0, 2.0, 0.01);
    vertical_current_offset_adj.set_value(0.0);
    geometry_settings_grid.attach(&vertical_current_offset_adj, 1, 1, 1, 1);
    stats_box.append(&geometry_settings_grid);

    window.set_child(Some(&stats_box));

    let (sender, receiver) = MainContext::channel::<u32>(PRIORITY_DEFAULT);

    thread::spawn(move || {
        let mut counter = 0;
        loop {
            sender.send(counter).expect("Could not send through channel");
            thread::sleep(Duration::from_millis(16));
            counter += 1;
        }
    });

    receiver.attach(
        None,
        move |a| {
            sync_freq_label.set_label(&format!("Horizontal sync width: {}us", a));
            Continue(true)
        }
    );

    window.present();
}
