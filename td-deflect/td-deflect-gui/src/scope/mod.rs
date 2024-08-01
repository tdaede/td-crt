use gtk::glib;
use glib::Object;
use gtk::subclass::prelude::*;

mod imp;

glib::wrapper! {
    pub struct Scope(ObjectSubclass<imp::Scope>) @extends gtk::Widget;
}

impl Scope {
    pub fn new() -> Self {
        Object::builder().build()
    }
    pub fn set_values(&self, v: Vec<Vec<f32>>) {
        self.imp().set_values(v);
    }
}
