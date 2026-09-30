//! Physics Playground : bac à sable de corps rigides (egui) construit sur le moteur
//! `physics` de ce dépôt. Crée, attrape, lance et fais exploser des objets ; relève les
//! défis « Démolition » et « Tour infernale ».

#![cfg_attr(windows, windows_subsystem = "windows")]

mod scenes;

use eframe::egui::{self, Align2, Color32, CornerRadius, FontId, Key, Margin, Pos2, Rect, RichText, Sense, Shape as EShape, Stroke, Vec2 as EVec2};
use physics::{RigidBody, Shape, Vec2, World};
use scenes::{spawn, Built, Material, Meta, Role, Scene, GRAVITY, LAUNCHER, PLATFORM_TOP, STATIC_COLOR, TOWER_PIECES};

const BG_TOP: Color32 = Color32::from_rgb(16, 20, 44);
const BG_BOTTOM: Color32 = Color32::from_rgb(8, 9, 22);
const PANEL: Color32 = Color32::from_rgb(14, 16, 34);
const PANEL_2: Color32 = Color32::from_rgb(24, 27, 54);
const TEXT: Color32 = Color32::from_rgb(230, 234, 255);
const DIM: Color32 = Color32::from_rgb(140, 148, 190);
const ACCENT: Color32 = Color32::from_rgb(255, 170, 60);
const CYAN: Color32 = Color32::from_rgb(76, 201, 240);
const GREEN: Color32 = Color32::from_rgb(114, 239, 150);
const PINK: Color32 = Color32::from_rgb(247, 72, 133);
const GOLD: Color32 = Color32::from_rgb(255, 209, 102);

const DT: f64 = 1.0 / 120.0;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Physics Playground")
            .with_inner_size([1280.0, 760.0])
            .with_min_inner_size([980.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native("Physics Playground", options, Box::new(|cc| Ok(Box::new(Playground::new(&cc.egui_ctx)))))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tool {
    Hand,
    Box,
    Ball,
    Polygon,
    Plank,
    Slingshot,
    Explosion,
    Erase,
}

impl Tool {
    const ALL: [Tool; 8] = [Tool::Hand, Tool::Box, Tool::Ball, Tool::Polygon, Tool::Plank, Tool::Slingshot, Tool::Explosion, Tool::Erase];
    fn label(self) -> &'static str {
        match self {
            Tool::Hand => "✋  Attraper / lancer",
            Tool::Box => "⬛  Boîte",
            Tool::Ball => "⚽  Balle",
            Tool::Polygon => "⬟  Polygone",
            Tool::Plank => "➖  Planche fixe",
            Tool::Slingshot => "🎯  Fronde",
            Tool::Explosion => "💥  Explosion",
            Tool::Erase => "🗑  Effacer",
        }
    }
    fn hint(self) -> &'static str {
        match self {
            Tool::Hand => "Attrape un objet et secoue-le ; relâche en mouvement pour le lancer.",
            Tool::Box => "Clique pour poser une boîte.",
            Tool::Ball => "Clique pour poser une balle.",
            Tool::Polygon => "Clique pour poser une forme à 3 à 6 côtés.",
            Tool::Plank => "Glisse pour tracer une planche fixe (rampe, étagère, mur…).",
            Tool::Slingshot => "Glisse depuis un point : un boulet part dans la direction opposée.",
            Tool::Explosion => "Clique : tout ce qui est proche est soufflé.",
            Tool::Erase => "Clique sur un objet pour le retirer.",
        }
    }
}

struct Ripple {
    pos: Vec2,
    age: f32,
}

struct Playground {
    world: World,
    meta: Vec<Meta>,
    scene: Scene,
    view_center: Vec2,
    view_half_width: f64,
    zoom: f32,

    paused: bool,
    slow_motion: f64,
    accum: f64,
    time: f64,
    gravity_strength: f64,
    gravity_dir: (f64, f64),

    tool: Tool,
    material: Material,
    size: f64,
    show_contacts: bool,
    show_velocity: bool,
    show_aabb: bool,

    grab: Option<(usize, Vec2)>,
    drag_from: Option<Vec2>,
    ripples: Vec<Ripple>,

    // Démolition
    shots_left: u32,
    last_shot_time: f64,
    // Tour infernale
    pieces_left: u32,
    best_height: f64,
    record_height: f64,
    calm_time: f64,

    toast: Option<(String, Color32, f64)>,
    frame_time: f64,
}

impl Playground {
    fn new(ctx: &egui::Context) -> Self {
        style(ctx);
        let mut p = Playground {
            world: World::new(Vec2::new(0.0, GRAVITY)),
            meta: Vec::new(),
            scene: Scene::Sandbox,
            view_center: Vec2::new(0.0, 7.0),
            view_half_width: 15.0,
            zoom: 0.0,
            paused: false,
            slow_motion: 1.0,
            accum: 0.0,
            time: 0.0,
            gravity_strength: 9.81,
            gravity_dir: (0.0, -1.0),
            tool: Tool::Hand,
            material: Material::Wood,
            size: 1.0,
            show_contacts: false,
            show_velocity: false,
            show_aabb: false,
            grab: None,
            drag_from: None,
            ripples: Vec::new(),
            shots_left: 5,
            last_shot_time: 0.0,
            pieces_left: TOWER_PIECES,
            best_height: 0.0,
            record_height: load_record(),
            calm_time: 0.0,
            toast: None,
            frame_time: 1.0 / 60.0,
        };
        let args: Vec<String> = std::env::args().collect();
        let scene = args
            .windows(2)
            .find(|w| w[0] == "--scene")
            .and_then(|w| w[1].parse::<usize>().ok())
            .and_then(|n| Scene::PLAY.iter().chain(Scene::GAMES.iter()).nth(n.wrapping_sub(1)).copied())
            .unwrap_or(Scene::Sandbox);
        p.load(scene);
        p
    }

    fn load(&mut self, scene: Scene) {
        let Built { world, meta, view } = scene.build();
        self.world = world;
        self.meta = meta;
        self.scene = scene;
        self.view_center = view.0;
        self.view_half_width = view.1;
        self.zoom = 0.0;
        self.time = 0.0;
        self.grab = None;
        self.drag_from = None;
        self.ripples.clear();
        self.shots_left = 5;
        self.pieces_left = TOWER_PIECES;
        self.best_height = 0.0;
        self.calm_time = 0.0;
        self.toast = None;
        self.apply_gravity();
        match scene {
            Scene::Demolition => {
                self.tool = Tool::Slingshot;
                self.toast("Glisse vers l'arrière depuis la fronde pour viser, relâche pour tirer !", CYAN, 6.0);
            }
            Scene::Tower => {
                self.tool = Tool::Box;
                self.toast("Clique au-dessus du socle pour poser une pièce. Change de forme à gauche.", CYAN, 6.0);
            }
            Scene::Billiard => self.tool = Tool::Hand,
            _ => {}
        }
    }

    fn apply_gravity(&mut self) {
        if self.scene == Scene::Billiard {
            self.world.gravity = Vec2::ZERO;
            return;
        }
        self.world.gravity = Vec2::new(self.gravity_dir.0, self.gravity_dir.1) * self.gravity_strength;
    }

    fn toast(&mut self, text: &str, color: Color32, seconds: f64) {
        self.toast = Some((text.to_string(), color, self.time + seconds));
    }

    fn remove(&mut self, i: usize) {
        self.world.bodies.remove(i);
        self.meta.remove(i);
        // Les contacts mémorisés référencent des indices : on les oublie.
        self.world.last_manifolds.clear();
        self.grab = None;
    }

    fn add(&mut self, shape: Shape, pos: Vec2, role: Role) -> usize {
        spawn(&mut self.world, &mut self.meta, shape, pos, self.material, role, None)
    }

    /// Indice du corps sous le point `p` (monde).
    fn pick(&self, p: Vec2) -> Option<usize> {
        self.world.bodies.iter().enumerate().rev().find(|(_, b)| contains(b, p)).map(|(i, _)| i)
    }

    fn step(&mut self, frame_dt: f64) {
        if self.paused {
            return;
        }
        self.accum += frame_dt.min(0.05) * self.slow_motion;
        let mut n = 0;
        while self.accum >= DT && n < 8 {
            // Main : ressort amorti vers la souris (l'objet reste soumis aux collisions).
            if let Some((i, target)) = self.grab {
                if let Some(b) = self.world.bodies.get_mut(i) {
                    if !b.is_static() {
                        let desired = (target - b.position) * 14.0;
                        b.velocity = b.velocity * 0.6 + desired * 0.4;
                        b.angular_velocity *= 0.9;
                    }
                }
            }
            self.world.step(DT);
            self.time += DT;
            self.accum -= DT;
            n += 1;
        }
        // Nettoyage : ce qui tombe hors du monde disparaît.
        let mut i = 0;
        while i < self.world.bodies.len() {
            let p = self.world.bodies[i].position;
            if p.y < -40.0 || p.x.abs() > 200.0 || p.y > 400.0 {
                self.remove(i);
            } else {
                i += 1;
            }
        }
        self.update_games();
    }

    fn update_games(&mut self) {
        match self.scene {
            Scene::Demolition => {
                if self.shots_left == 0 && self.time - self.last_shot_time > 5.0 && self.toast.as_ref().is_none_or(|t| t.1 != GREEN && t.1 != GOLD) {
                    let (fallen, total) = self.demolition_score();
                    let stars = if fallen == total { 3 } else if fallen * 10 >= total * 7 { 2 } else if fallen * 10 >= total * 4 { 1 } else { 0 };
                    let text = format!(
                        "{} {} / {} pièces à terre. {}",
                        "★".repeat(stars) + &"☆".repeat(3 - stars),
                        fallen,
                        total,
                        if stars == 3 { "Démolition totale !" } else { "Recommence (R) pour faire mieux !" }
                    );
                    self.toast = Some((text, if stars >= 2 { GREEN } else { GOLD }, f64::INFINITY));
                }
            }
            Scene::Tower => {
                let calm = self.world.bodies.iter().all(|b| b.is_static() || b.velocity.length() < 0.15);
                self.calm_time = if calm { self.calm_time + self.frame_time } else { 0.0 };
                if self.calm_time > 1.0 {
                    let h = self.tower_height();
                    if h > self.best_height + 0.01 {
                        self.best_height = h;
                        if h > self.record_height {
                            self.record_height = h;
                            save_record(h);
                        }
                    }
                }
            }
            _ => {}
        }
    }

    fn demolition_score(&self) -> (usize, usize) {
        let targets: Vec<&RigidBody> = self.world.bodies.iter().zip(&self.meta).filter(|(_, m)| m.role == Role::Target).map(|(b, _)| b).collect();
        let total = self.meta.iter().filter(|m| m.role == Role::Target).count().max(targets.len());
        let standing = targets.iter().filter(|b| b.position.y > PLATFORM_TOP && (b.position.x - 12.0).abs() < 4.6).count();
        // Les pièces tombées hors du monde ont disparu : elles comptent comme à terre.
        (TARGET_COUNT.max(total) - standing, TARGET_COUNT.max(total))
    }

    fn tower_height(&self) -> f64 {
        self.world
            .bodies
            .iter()
            .filter(|b| !b.is_static() && b.position.x.abs() < 6.0)
            .map(|b| b.world_aabb().max.y)
            .fold(1.0, f64::max)
            - 1.0
    }
}

/// Nombre de pièces du château (voir scenes::demolition).
const TARGET_COUNT: usize = 21;

fn contains(b: &RigidBody, p: Vec2) -> bool {
    match &b.shape {
        Shape::Circle { radius } => (p - b.position).length() <= *radius,
        Shape::Polygon { .. } => {
            let v = b.world_vertices();
            (0..v.len()).all(|i| (v[(i + 1) % v.len()] - v[i]).cross(p - v[i]) >= 0.0)
        }
    }
}

// ---------------------------------------------------------------------------- interface

impl eframe::App for Playground {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        let dt = ctx.input(|i| i.unstable_dt).min(0.1) as f64;
        self.frame_time = self.frame_time * 0.9 + dt * 0.1;
        self.shortcuts(&ctx);
        self.step(dt);

        egui::Panel::left("tools")
            .exact_size(290.0)
            .resizable(false)
            .frame(egui::Frame::new().fill(PANEL).inner_margin(Margin::same(18)))
            .show(ui, |ui| self.left_panel(ui));
        egui::Panel::right("world")
            .exact_size(290.0)
            .resizable(false)
            .frame(egui::Frame::new().fill(PANEL).inner_margin(Margin::same(18)))
            .show(ui, |ui| self.right_panel(ui));
        egui::CentralPanel::no_frame().show(ui, |ui| self.canvas(ui));
        ctx.request_repaint();
    }
}

impl Playground {
    fn shortcuts(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let (space, r, n) = ctx.input(|i| (i.key_pressed(Key::Space), i.key_pressed(Key::R), i.key_pressed(Key::N)));
        if space {
            self.paused = !self.paused;
        }
        if r {
            self.load(self.scene);
        }
        if n && self.paused {
            self.world.step(DT);
            self.time += DT;
        }
    }

    fn left_panel(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("PHYSICS").size(30.0).strong().color(TEXT));
        ui.label(RichText::new("PLAYGROUND").size(30.0).strong().color(ACCENT));
        ui.label(RichText::new("Le labo des objets qui tombent").color(DIM));
        ui.add_space(12.0);
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            let full = ui.available_width() - 6.0;
            section(ui, "SCÈNES");
            egui::Grid::new("scenes").num_columns(2).spacing([6.0, 6.0]).show(ui, |ui| {
                for (i, s) in Scene::PLAY.iter().enumerate() {
                    let b = egui::Button::new(RichText::new(format!("{}  {}", s.icon(), s.label())).size(13.0))
                        .min_size(EVec2::new((full - 6.0) / 2.0, 34.0))
                        .selected(self.scene == *s);
                    if ui.add(b).on_hover_text(s.description()).clicked() {
                        self.load(*s);
                    }
                    if i % 2 == 1 {
                        ui.end_row();
                    }
                }
            });
            ui.add_space(10.0);
            section(ui, "DÉFIS");
            for s in Scene::GAMES {
                let b = egui::Button::new(RichText::new(format!("{}  {}", s.icon(), s.label())).size(14.5))
                    .min_size(EVec2::new(full, 36.0))
                    .selected(self.scene == s);
                if ui.add(b).on_hover_text(s.description()).clicked() {
                    self.load(s);
                }
            }

            ui.add_space(10.0);
            section(ui, "OUTILS");
            egui::Grid::new("tools").num_columns(2).spacing([6.0, 6.0]).show(ui, |ui| {
                for (i, t) in Tool::ALL.iter().enumerate() {
                    let b = egui::Button::new(RichText::new(t.label()).size(13.0))
                        .min_size(EVec2::new((full - 6.0) / 2.0, 32.0))
                        .selected(self.tool == *t);
                    if ui.add(b).clicked() {
                        self.tool = *t;
                    }
                    if i % 2 == 1 {
                        ui.end_row();
                    }
                }
            });
            ui.add_space(4.0);
            ui.label(RichText::new(self.tool.hint()).size(12.5).color(DIM));

            ui.add_space(10.0);
            section(ui, "MATÉRIAU");
            ui.horizontal_wrapped(|ui| {
                for m in Material::ALL {
                    let text = RichText::new(m.label()).color(if self.material == m { Color32::BLACK } else { m.color() });
                    let b = egui::Button::new(text).fill(if self.material == m { m.color() } else { PANEL_2 });
                    if ui.add(b).on_hover_text(m.describe()).clicked() {
                        self.material = m;
                    }
                }
            });
            ui.label(RichText::new(self.material.describe()).size(12.5).color(DIM));
            ui.add(egui::Slider::new(&mut self.size, 0.3..=3.0).text("taille (m)"));
        });
    }

    fn right_panel(&mut self, ui: &mut egui::Ui) {
        section(ui, "TEMPS");
        ui.horizontal(|ui| {
            let label = if self.paused { "▶  Reprendre" } else { "⏸  Pause" };
            if ui.add(egui::Button::new(RichText::new(label).size(14.0)).min_size(EVec2::new(120.0, 32.0))).clicked() {
                self.paused = !self.paused;
            }
            if ui.add_enabled(self.paused, egui::Button::new("⏭").min_size(EVec2::new(36.0, 32.0))).on_hover_text("Pas à pas (N)").clicked() {
                self.world.step(DT);
                self.time += DT;
            }
            if ui.add(egui::Button::new("⟲").min_size(EVec2::new(36.0, 32.0))).on_hover_text("Recommencer (R)").clicked() {
                self.load(self.scene);
            }
        });
        ui.add(egui::Slider::new(&mut self.slow_motion, 0.1..=2.0).logarithmic(true).text("vitesse").suffix("×"));

        if self.scene != Scene::Billiard {
            ui.add_space(10.0);
            section(ui, "GRAVITÉ");
            let before = (self.gravity_strength, self.gravity_dir);
            ui.add(egui::Slider::new(&mut self.gravity_strength, 0.0..=30.0).text("m/s²"));
            ui.horizontal(|ui| {
                for (label, dir, tip) in [("⬇", (0.0, -1.0), "Vers le bas"), ("⬆", (0.0, 1.0), "Vers le haut"), ("⬅", (-1.0, 0.0), "Vers la gauche"), ("➡", (1.0, 0.0), "Vers la droite")] {
                    if ui.add(egui::Button::new(label).min_size(EVec2::new(40.0, 30.0)).selected(self.gravity_dir == dir)).on_hover_text(tip).clicked() {
                        self.gravity_dir = dir;
                    }
                }
            });
            ui.horizontal_wrapped(|ui| {
                for (label, g) in [("Lune", 1.62), ("Mars", 3.71), ("Terre", 9.81), ("Jupiter", 24.8)] {
                    if ui.small_button(label).clicked() {
                        self.gravity_strength = g;
                    }
                }
            });
            if before != (self.gravity_strength, self.gravity_dir) {
                self.apply_gravity();
            }
        }

        ui.add_space(10.0);
        section(ui, "RAYONS X");
        ui.checkbox(&mut self.show_contacts, "Points de contact");
        ui.checkbox(&mut self.show_velocity, "Vecteurs vitesse");
        ui.checkbox(&mut self.show_aabb, "Boîtes englobantes");

        ui.add_space(10.0);
        section(ui, "MESURES");
        let dynamic = self.world.bodies.iter().filter(|b| !b.is_static()).count();
        stat(ui, "Objets", format!("{dynamic}"));
        stat(ui, "Contacts", format!("{}", self.world.last_manifolds.iter().map(|m| m.contacts.len()).sum::<usize>()));
        stat(ui, "Énergie cinétique", format!("{:.1} J", self.world.kinetic_energy()));
        stat(ui, "Quantité de mouvement", format!("{:.1}", self.world.total_momentum().length()));
        stat(ui, "Images / s", format!("{:.0}", 1.0 / self.frame_time.max(1e-3)));

        match self.scene {
            Scene::Demolition => {
                ui.add_space(10.0);
                section(ui, "DÉMOLITION");
                let (fallen, total) = self.demolition_score();
                stat(ui, "Boulets restants", format!("{}", self.shots_left));
                stat(ui, "Pièces à terre", format!("{fallen} / {total}"));
                ui.add(egui::ProgressBar::new(fallen as f32 / total.max(1) as f32).fill(ACCENT));
            }
            Scene::Tower => {
                ui.add_space(10.0);
                section(ui, "TOUR INFERNALE");
                stat(ui, "Pièces restantes", format!("{}", self.pieces_left));
                stat(ui, "Hauteur stable", format!("{:.2} m", self.best_height));
                stat(ui, "Record", format!("{:.2} m", self.record_height));
            }
            _ => {}
        }

        ui.with_layout(egui::Layout::bottom_up(egui::Align::Min), |ui| {
            ui.label(RichText::new("Espace : pause · R : recommencer · N : pas à pas · molette : zoom · clic droit : déplacer").size(11.5).color(DIM));
        });
    }

    fn canvas(&mut self, ui: &mut egui::Ui) {
        let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = response.rect;
        if self.zoom <= 0.0 {
            self.zoom = (rect.width() * 0.5 / self.view_half_width as f32).min(rect.height() * 0.5 / (self.view_half_width as f32 * 0.62));
        }
        let (c, z, vc) = (rect.center(), self.zoom, self.view_center);
        let to_screen = move |p: Vec2| Pos2::new(c.x + ((p.x - vc.x) as f32) * z, c.y - ((p.y - vc.y) as f32) * z);
        let to_world = move |s: Pos2| Vec2::new(vc.x + ((s.x - c.x) / z) as f64, vc.y - ((s.y - c.y) / z) as f64);

        // --- entrées
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                if let Some(m) = response.hover_pos() {
                    let before = to_world(m);
                    self.zoom = (self.zoom * (scroll * 0.0018).exp()).clamp(4.0, 300.0);
                    let after = Vec2::new(vc.x + ((m.x - c.x) / self.zoom) as f64, vc.y - ((m.y - c.y) / self.zoom) as f64);
                    self.view_center = self.view_center + (before - after);
                }
            }
        }
        if response.dragged_by(egui::PointerButton::Secondary) || response.dragged_by(egui::PointerButton::Middle) {
            let d = response.drag_delta();
            self.view_center = self.view_center + Vec2::new((-d.x / z) as f64, (d.y / z) as f64);
        }
        let pointer = response.interact_pointer_pos().or(response.hover_pos()).map(to_world);
        self.handle_tools(&response, pointer);

        // --- fond : dégradé + quadrillage d'un mètre
        painter.add(gradient_rect(rect, BG_TOP, BG_BOTTOM));
        let step = z;
        if step > 8.0 {
            let grid = Color32::from_rgba_unmultiplied(120, 130, 220, 16);
            let origin = to_screen(Vec2::ZERO);
            let mut x = origin.x.rem_euclid(step) + rect.left() - rect.left().rem_euclid(step);
            while x < rect.right() {
                painter.line_segment([Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())], Stroke::new(1.0, grid));
                x += step;
            }
            let mut y = origin.y.rem_euclid(step) + rect.top() - rect.top().rem_euclid(step);
            while y < rect.bottom() {
                painter.line_segment([Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)], Stroke::new(1.0, grid));
                y += step;
            }
        }

        // --- décor des défis
        match self.scene {
            Scene::Demolition => {
                let p = to_screen(LAUNCHER);
                painter.circle_stroke(p, 18.0, Stroke::new(2.0, ACCENT));
                painter.text(p + EVec2::new(0.0, -30.0), Align2::CENTER_CENTER, format!("FRONDE · {} boulet{}", self.shots_left, if self.shots_left > 1 { "s" } else { "" }), FontId::proportional(13.0), ACCENT);
            }
            Scene::Tower => {
                let base = to_screen(Vec2::new(0.0, 1.0));
                for (h, color, label) in [(self.best_height, GREEN, "hauteur"), (self.record_height, GOLD, "record")] {
                    if h > 0.05 {
                        let y = to_screen(Vec2::new(0.0, 1.0 + h)).y;
                        painter.line_segment([Pos2::new(rect.left() + 20.0, y), Pos2::new(rect.right() - 20.0, y)], Stroke::new(1.0, color.gamma_multiply(0.6)));
                        painter.text(Pos2::new(rect.right() - 24.0, y - 4.0), Align2::RIGHT_BOTTOM, format!("{label} {h:.2} m"), FontId::proportional(13.0), color);
                    }
                }
                let _ = base;
            }
            _ => {}
        }

        // --- corps
        for (i, (b, m)) in self.world.bodies.iter().zip(&self.meta).enumerate() {
            let grabbed = self.grab.map(|g| g.0) == Some(i);
            let fill = if b.is_static() { STATIC_COLOR } else { m.color };
            let outline = if grabbed { Stroke::new(2.5, Color32::WHITE) } else { Stroke::new(1.5, darken(fill, 0.55)) };
            match &b.shape {
                Shape::Circle { radius } => {
                    let p = to_screen(b.position);
                    let r = *radius as f32 * z;
                    painter.circle(p, r, fill, outline);
                    // Rayon pour voir la rotation
                    let tip = to_screen(b.position + Vec2::new(b.rotation.cos(), b.rotation.sin()) * *radius * 0.8);
                    painter.line_segment([p, tip], Stroke::new(1.5, darken(fill, 0.5)));
                    if !b.is_static() {
                        painter.circle_filled(p + EVec2::new(-r * 0.35, -r * 0.35), r * 0.25, Color32::from_rgba_unmultiplied(255, 255, 255, 70));
                    }
                }
                Shape::Polygon { .. } => {
                    let pts: Vec<Pos2> = b.world_vertices().into_iter().map(to_screen).collect();
                    painter.add(EShape::convex_polygon(pts.clone(), fill, outline));
                    if !b.is_static() && pts.len() >= 2 {
                        // Liseré clair sur l'arête du haut : effet de volume
                        let top = pts.iter().enumerate().min_by(|a, b| a.1.y.total_cmp(&b.1.y)).map(|(k, _)| k).unwrap_or(0);
                        let next = pts[(top + 1) % pts.len()];
                        let prev = pts[(top + pts.len() - 1) % pts.len()];
                        let other = if next.y < prev.y { next } else { prev };
                        painter.line_segment([pts[top], other], Stroke::new(2.0, Color32::from_rgba_unmultiplied(255, 255, 255, 60)));
                    }
                }
            }
            if m.role == Role::Target && !b.is_static() {
                painter.circle_filled(to_screen(b.position), 2.5, darken(fill, 0.4));
            }
            if self.show_velocity && !b.is_static() && b.velocity.length() > 0.05 {
                let p = to_screen(b.position);
                painter.arrow(p, EVec2::new(b.velocity.x as f32, -b.velocity.y as f32) * z * 0.12, Stroke::new(1.5, GREEN));
            }
            if self.show_aabb {
                let a = b.world_aabb();
                let r = Rect::from_two_pos(to_screen(a.min), to_screen(a.max));
                painter.rect_stroke(r, CornerRadius::ZERO, Stroke::new(1.0, CYAN.gamma_multiply(0.5)), egui::StrokeKind::Middle);
            }
        }
        if self.show_contacts {
            for m in &self.world.last_manifolds {
                for p in &m.contacts {
                    let s = to_screen(*p);
                    painter.circle_filled(s, 3.5, PINK);
                    painter.line_segment([s, s + EVec2::new(m.normal.x as f32, -m.normal.y as f32) * 16.0], Stroke::new(1.5, PINK));
                }
            }
        }

        // --- aperçus d'outils
        if let (Some(from), Some(to)) = (self.drag_from, pointer) {
            match self.tool {
                Tool::Plank => {
                    painter.line_segment([to_screen(from), to_screen(to)], Stroke::new((0.3 * z).max(3.0), STATIC_COLOR.gamma_multiply(1.6)));
                    painter.text(to_screen(to) + EVec2::new(12.0, 12.0), Align2::LEFT_TOP, format!("{:.1} m", (to - from).length()), FontId::proportional(13.0), TEXT);
                }
                Tool::Slingshot => {
                    let v = self.sling_velocity(from, to);
                    painter.line_segment([to_screen(from), to_screen(to)], Stroke::new(2.0, ACCENT.gamma_multiply(0.7)));
                    // Trajectoire balistique prévue (sans les collisions)
                    let g = self.world.gravity;
                    let pts: Vec<Pos2> = (0..60).map(|k| {
                        let t = k as f64 * 0.05;
                        to_screen(from + v * t + g * (0.5 * t * t))
                    }).collect();
                    for (k, p) in pts.iter().enumerate().step_by(2) {
                        painter.circle_filled(*p, 2.5, ACCENT.gamma_multiply(1.0 - k as f32 / 70.0));
                    }
                    painter.text(to_screen(to) + EVec2::new(12.0, 12.0), Align2::LEFT_TOP, format!("{:.1} m/s", v.length()), FontId::proportional(13.0), ACCENT);
                }
                _ => {}
            }
        } else if let Some(p) = pointer.filter(|_| response.hovered()) {
            // Fantôme de l'objet à poser
            let ghost = Color32::from_rgba_unmultiplied(255, 255, 255, 40);
            match self.tool {
                Tool::Box => {
                    let h = self.size as f32 * z / 2.0;
                    painter.rect_stroke(Rect::from_center_size(to_screen(p), EVec2::splat(h * 2.0)), CornerRadius::ZERO, Stroke::new(1.5, ghost), egui::StrokeKind::Middle);
                }
                Tool::Ball | Tool::Polygon => {
                    painter.circle_stroke(to_screen(p), self.size as f32 * z / 2.0, Stroke::new(1.5, ghost));
                }
                Tool::Explosion => {
                    painter.circle_stroke(to_screen(p), 4.0 * z, Stroke::new(1.0, PINK.gamma_multiply(0.5)));
                }
                _ => {}
            }
        }

        // --- ondes d'explosion
        let dt = ui.input(|i| i.unstable_dt).min(0.05);
        self.ripples.retain_mut(|r| {
            r.age += dt;
            r.age < 0.6
        });
        for r in &self.ripples {
            let k = r.age / 0.6;
            painter.circle_stroke(to_screen(r.pos), 4.0 * z * (0.3 + k), Stroke::new(4.0 * (1.0 - k), Color32::from_rgba_unmultiplied(255, 140, 60, ((1.0 - k) * 255.0) as u8)));
        }

        // --- bandeau
        let banner = match &self.toast {
            Some((t, color, until)) if self.time < *until => Some((t.clone(), *color)),
            _ => None,
        };
        let (text, color) = banner.unwrap_or((self.scene.description().to_string(), DIM));
        let galley = painter.layout(text, FontId::proportional(15.0), color, rect.width() - 120.0);
        let size = galley.size() + EVec2::new(28.0, 16.0);
        let top = Rect::from_min_size(Pos2::new(rect.center().x - size.x / 2.0, rect.top() + 16.0), size);
        painter.rect_filled(top, CornerRadius::same(12), Color32::from_rgba_unmultiplied(14, 16, 34, 215));
        painter.galley(top.min + EVec2::new(14.0, 8.0), galley, color);
        if self.paused {
            painter.text(rect.center_bottom() + EVec2::new(0.0, -24.0), Align2::CENTER_BOTTOM, "⏸  EN PAUSE", FontId::proportional(18.0), GOLD);
        }
    }

    fn sling_velocity(&self, from: Vec2, to: Vec2) -> Vec2 {
        let v = (from - to) * 3.2;
        let max = 32.0;
        if v.length() > max { v * (max / v.length()) } else { v }
    }

    fn handle_tools(&mut self, response: &egui::Response, pointer: Option<Vec2>) {
        let Some(p) = pointer else { return };
        let primary_start = response.drag_started_by(egui::PointerButton::Primary);
        let primary_stop = response.drag_stopped_by(egui::PointerButton::Primary);
        let clicked = response.clicked();
        match self.tool {
            Tool::Hand => {
                if primary_start {
                    self.grab = self.pick(p).filter(|i| !self.world.bodies[*i].is_static()).map(|i| (i, p));
                }
                if let Some((i, _)) = self.grab {
                    self.grab = Some((i, p));
                }
                if primary_stop {
                    self.grab = None;
                }
            }
            Tool::Plank | Tool::Slingshot => {
                if primary_start {
                    let from = if self.scene == Scene::Demolition && self.tool == Tool::Slingshot { LAUNCHER } else { p };
                    self.drag_from = Some(from);
                }
                if primary_stop {
                    if let Some(from) = self.drag_from.take() {
                        if self.tool == Tool::Plank {
                            let d = p - from;
                            let len = d.length();
                            if len > 0.3 {
                                let mut body = RigidBody::static_body(Shape::rectangle(len, 0.3), (from + p) * 0.5).with_friction(0.7);
                                body.rotation = d.y.atan2(d.x);
                                self.world.add_body(body);
                                self.meta.push(Meta { color: STATIC_COLOR, role: Role::Normal });
                            }
                        } else {
                            self.fire(from, self.sling_velocity(from, p));
                        }
                    }
                }
            }
            Tool::Box | Tool::Ball | Tool::Polygon => {
                if clicked {
                    if self.scene == Scene::Tower {
                        if self.pieces_left == 0 {
                            self.toast("Plus de pièces ! Recommence (R) pour tenter un nouveau record.", GOLD, 4.0);
                            return;
                        }
                        self.pieces_left -= 1;
                    }
                    let s = self.size;
                    let shape = match self.tool {
                        Tool::Box => Shape::rectangle(s, s),
                        Tool::Ball => Shape::circle(s / 2.0),
                        _ => random_polygon(s / 2.0, self.time),
                    };
                    self.add(shape, p, Role::Normal);
                }
            }
            Tool::Explosion => {
                if clicked {
                    for b in self.world.bodies.iter_mut().filter(|b| !b.is_static()) {
                        let d = b.position - p;
                        let dist = d.length().max(0.3);
                        if dist < 4.0 {
                            let strength = 18.0 * (1.0 - dist / 4.0) * b.mass();
                            b.velocity += d.normalized() * (strength / b.mass());
                            b.angular_velocity += (d.x - d.y).signum() * 3.0 * (1.0 - dist / 4.0);
                        }
                    }
                    self.ripples.push(Ripple { pos: p, age: 0.0 });
                }
            }
            Tool::Erase => {
                if clicked {
                    if let Some(i) = self.pick(p) {
                        self.remove(i);
                    }
                }
            }
        }
    }

    fn fire(&mut self, from: Vec2, v: Vec2) {
        if self.scene == Scene::Demolition {
            if self.shots_left == 0 {
                self.toast("Plus de boulets ! Recommence (R) pour une nouvelle démolition.", GOLD, 4.0);
                return;
            }
            self.shots_left -= 1;
            self.last_shot_time = self.time;
        }
        let i = spawn(&mut self.world, &mut self.meta, Shape::circle(0.45), from, Material::Steel, Role::Ammo, Some(Color32::from_rgb(200, 208, 230)));
        self.world.bodies[i].velocity = v;
    }
}

// ---------------------------------------------------------------------------- utilitaires

fn random_polygon(radius: f64, seed: f64) -> Shape {
    let sides = 3 + ((seed * 1000.0) as usize % 4);
    let verts = (0..sides)
        .map(|k| {
            let a = k as f64 / sides as f64 * std::f64::consts::TAU + 0.3;
            Vec2::new(a.cos(), a.sin()) * radius
        })
        .collect();
    Shape::polygon(verts)
}

fn darken(c: Color32, k: f32) -> Color32 {
    Color32::from_rgb((c.r() as f32 * k) as u8, (c.g() as f32 * k) as u8, (c.b() as f32 * k) as u8)
}

fn gradient_rect(rect: Rect, top: Color32, bottom: Color32) -> EShape {
    let mut mesh = egui::Mesh::default();
    mesh.colored_vertex(rect.left_top(), top);
    mesh.colored_vertex(rect.right_top(), top);
    mesh.colored_vertex(rect.right_bottom(), bottom);
    mesh.colored_vertex(rect.left_bottom(), bottom);
    mesh.add_triangle(0, 1, 2);
    mesh.add_triangle(0, 2, 3);
    EShape::mesh(mesh)
}

fn section(ui: &mut egui::Ui, title: &str) {
    ui.label(RichText::new(title).size(11.5).strong().color(ACCENT));
    ui.add_space(2.0);
}

fn stat(ui: &mut egui::Ui, label: &str, value: String) {
    ui.horizontal(|ui| {
        ui.label(RichText::new(label).color(DIM));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(RichText::new(value).color(TEXT));
        });
    });
}

fn style(ctx: &egui::Context) {
    let mut v = egui::Visuals::dark();
    v.panel_fill = PANEL;
    v.window_fill = PANEL_2;
    v.extreme_bg_color = Color32::from_rgb(10, 11, 26);
    v.faint_bg_color = PANEL_2;
    v.selection.bg_fill = Color32::from_rgb(150, 85, 20);
    v.selection.stroke = Stroke::new(1.0, GOLD);
    v.widgets.inactive.bg_fill = PANEL_2;
    v.widgets.inactive.weak_bg_fill = PANEL_2;
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, TEXT);
    v.widgets.hovered.weak_bg_fill = Color32::from_rgb(40, 44, 84);
    v.widgets.hovered.bg_fill = Color32::from_rgb(40, 44, 84);
    v.widgets.active.weak_bg_fill = Color32::from_rgb(150, 85, 20);
    for w in [&mut v.widgets.inactive, &mut v.widgets.hovered, &mut v.widgets.active, &mut v.widgets.open] {
        w.corner_radius = CornerRadius::same(8);
    }
    ctx.set_visuals(v);
    ctx.all_styles_mut(|s| {
        s.spacing.item_spacing = EVec2::new(8.0, 7.0);
        s.spacing.button_padding = EVec2::new(10.0, 6.0);
        s.text_styles.insert(egui::TextStyle::Body, FontId::proportional(14.5));
        s.text_styles.insert(egui::TextStyle::Button, FontId::proportional(14.5));
    });
}

fn record_file() -> Option<std::path::PathBuf> {
    let base = std::env::var_os("APPDATA").or_else(|| std::env::var_os("HOME"))?;
    Some(std::path::PathBuf::from(base).join("PhysicsPlayground").join("tour.txt"))
}

fn load_record() -> f64 {
    record_file().and_then(|p| std::fs::read_to_string(p).ok()).and_then(|t| t.trim().parse().ok()).unwrap_or(0.0)
}

fn save_record(h: f64) {
    if let Some(path) = record_file() {
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        let _ = std::fs::write(path, format!("{h:.3}"));
    }
}
