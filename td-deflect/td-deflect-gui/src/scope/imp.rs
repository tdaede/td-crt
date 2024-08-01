use gtk::glib;
use gtk::prelude::*;
use gtk::subclass::prelude::*;

use std::error::Error;
use std::cell::RefCell;

use plotters::prelude::*;
use plotters_cairo::CairoBackend;

#[derive(Default)]
pub struct Scope {
    pub values: RefCell<Vec<Vec<f32>>>
}

#[glib::object_subclass]
impl ObjectSubclass for Scope {
    const NAME: &'static str = "Scope";
    type Type = super::Scope;
    type ParentType = gtk::Widget;
}

impl ObjectImpl for Scope {
}

impl WidgetImpl for Scope {
    fn snapshot(&self, snapshot: &gtk::Snapshot) {
        let width = self.obj().width() as u32;
        let height = self.obj().height() as u32;
        if width == 0 || height == 0 {
            return;
        }

        let bounds = gtk::graphene::Rect::new(0.0, 0.0, width as f32, height as f32);
        let cr = snapshot.append_cairo(&bounds);
        let backend = CairoBackend::new(&cr, (width, height)).unwrap();
        self.plot_pdf(backend).unwrap();
    }
}

impl Scope {
    fn plot_pdf<'a, DB: DrawingBackend + 'a>(
        &self,
        backend: DB,
    ) -> Result<(), Box<dyn Error + 'a>> {
        let root = backend.into_drawing_area();
        root.fill(&WHITE)?;
        let mut chart = ChartBuilder::on(&root)
            .margin(5)
            .build_cartesian_2d(0f32..263f32, -2f32..2f32)?;

        chart.configure_mesh().draw()?;

        let colors = [&RED, &BLUE];

        for series in self.values.borrow().iter().enumerate() {
            chart
                .draw_series(LineSeries::new(series.1.iter().enumerate().map(|x| { (x.0 as f32, *x.1 as f32) }),
                                             colors[series.0],
                ))?;
        }

        root.present()?;
        Ok(())
    }
    pub fn set_values(&self, v: Vec<Vec<f32>>) {
        self.values.replace(v);
        self.obj().queue_draw();
    }
}
