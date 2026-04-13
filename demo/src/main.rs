use eframe::egui;
use pinocchio_rs::{Model, SVec};
use std::{collections::VecDeque, f64::consts::PI, time::SystemTime};

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "Double Pendulum",
        native_options,
        Box::new(|_cc| Ok(Box::new(PendulumApp::default()))),
    )
}

struct PendulumApp {
    t: SystemTime,
    q: SVec<2>,
    v: SVec<2>,
    model: Model<2, 2>,
    trail: VecDeque<egui::Pos2>,
}

impl Default for PendulumApp {
    fn default() -> Self {
        let mut model = Model::load(
            &format!("{}/model/double_pendulum.urdf", env!("CARGO_MANIFEST_DIR")),
            false,
        )
        .unwrap();
        let mut q = SVec::default();
        q[0] = PI / 2.0;
        q[1] = PI / 2.0;
        model.forward_kinematics(&q);
        Self {
            t: SystemTime::now(),
            q,
            v: SVec::default(),
            model,
            trail: VecDeque::with_capacity(512),
        }
    }
}

impl PendulumApp {
    pub fn step(&mut self, dt: f64) {
        let t = SVec::default();
        self.model
            .semi_implicit_euler(&mut self.q, &mut self.v, &t, dt);
        self.model.forward_kinematics(&self.q);
    }
}

impl eframe::App for PendulumApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        const TARGET_FREQ: f64 = 60.0;
        // Update time
        let t = SystemTime::now();
        let dt = t.duration_since(self.t).unwrap_or_default().as_secs_f64();
        self.t = t;

        // Step physics
        self.step(dt);

        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.heading("Double Pendulum");

            let painter = ui.painter();
            let center = ui.max_rect().center();

            let a1 = self.q[0] as f32;
            let l1 = 128.0_f32;
            let a2 = a1 + self.q[1] as f32;
            let l2 = 128.0_f32;

            // Calculate positions
            let p1 = center + egui::vec2(a1.sin() * l1, a1.cos() * l1);
            let p2 = p1 + egui::vec2(a2.sin() * l2, a2.cos() * l2);

            // Update trail
            self.trail.push_back(p2);
            if self.trail.len() > 500 {
                self.trail.pop_front();
            }

            if self.trail.len() > 1 {
                let trail_color = egui::Color32::from_rgba_unmultiplied(100, 100, 255, 150);
                let points: Vec<egui::Pos2> = self.trail.iter().cloned().collect();
                painter.add(egui::Shape::line(
                    points,
                    egui::Stroke::new(1.5, trail_color),
                ));
            }

            let arm_stroke = egui::Stroke::new(2.0, egui::Color32::GRAY);
            painter.line_segment([center, p1], arm_stroke);
            painter.line_segment([p1, p2], arm_stroke);
            painter.circle_filled(p1, 5.0, egui::Color32::WHITE);
            painter.circle_filled(p2, 5.0, egui::Color32::LIGHT_BLUE);
        });

        // Request repaint
        let sleep_dt = 1.0 / TARGET_FREQ;
        ui.request_repaint_after_secs(sleep_dt as f32);
    }
}
