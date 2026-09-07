//! Frontend « Pixels » : rendu 2D sur CPU via `softbuffer`, sans GPU externe
//! — même construction que le frontend `pixels` de `gravity-simulation`
//! (fenêtre `winit` 0.30 + framebuffer `softbuffer` 0.4).
//!
//! Contrôles :
//! - `1/2/3/4`   changer de scène (rebond / pile / billard / rampe)
//! - `R`         réinitialiser la scène courante
//! - `P`         pause
//! - `+` / `-`   accélérer / ralentir (nombre de pas physiques par frame)
//! - `C`         afficher/masquer les points de contact
//! - molette     zoom ; clic-glisser pan
//! - `Échap`     quitter

use std::num::NonZeroU32;
use std::sync::Arc;

use physics::scenes::Scene;
use physics::world::World;
use physics::Shape;
use softbuffer::Surface;
use winit::application::ApplicationHandler;
use winit::dpi::{LogicalSize, PhysicalSize};
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowAttributes, WindowId};

const WIDTH: u32 = 960;
const HEIGHT: u32 = 720;

const COLOR_BACKGROUND: u32 = 0x12141a;
const COLOR_STATIC: u32 = 0x5a5f6b;
const COLOR_DYNAMIC: u32 = 0x5b8fff;
const COLOR_DYNAMIC_ASLEEP: u32 = 0x35d07f;
const COLOR_CONTACT: u32 = 0xff5c5c;

struct View {
    center_x: f64,
    center_y: f64,
    scale: f64,
    dragging: bool,
    last_mouse: (f64, f64),
}

impl View {
    fn new() -> Self {
        View {
            center_x: 0.0,
            center_y: 2.0,
            scale: 60.0,
            dragging: false,
            last_mouse: (0.0, 0.0),
        }
    }

    fn world_to_screen(&self, p: physics::Vec2, w: u32, h: u32) -> (i64, i64) {
        let sx = w as f64 / 2.0 + (p.x - self.center_x) * self.scale;
        let sy = h as f64 / 2.0 - (p.y - self.center_y) * self.scale;
        (sx.round() as i64, sy.round() as i64)
    }
}

struct App {
    world: World,
    scene: Scene,
    paused: bool,
    steps_per_frame: usize,
    show_contacts: bool,
    view: View,
    window: Option<Arc<Window>>,
    surface: Option<Surface<Arc<Window>, Arc<Window>>>,
}

impl App {
    fn reset(&mut self) {
        self.world = self.scene.build();
    }

    fn draw(&mut self) {
        let (Some(window), Some(surface)) = (&self.window, &mut self.surface) else {
            return;
        };
        let w = window.inner_size().width.max(1);
        let h = window.inner_size().height.max(1);
        let mut buffer = surface.buffer_mut().unwrap();
        let buf: &mut [u32] = &mut buffer;
        buf.fill(COLOR_BACKGROUND);
        let ww = w as usize;
        let wh = h as usize;

        for body in &self.world.bodies {
            let color = if body.is_static() {
                COLOR_STATIC
            } else if body.velocity.length() < 0.05 && body.angular_velocity.abs() < 0.05 {
                COLOR_DYNAMIC_ASLEEP
            } else {
                COLOR_DYNAMIC
            };

            match &body.shape {
                Shape::Circle { radius } => {
                    let (cx, cy) = self.view.world_to_screen(body.position, w, h);
                    let r = (*radius * self.view.scale).max(1.0) as i64;
                    draw_disc(buf, ww, wh, cx, cy, r, color);
                    // Petit trait radial pour visualiser la rotation d'un cercle
                    // (sinon un cercle qui tourne semble immobile à l'écran).
                    let edge =
                        body.position + physics::Vec2::new(*radius, 0.0).rotated(body.rotation);
                    let (ex, ey) = self.view.world_to_screen(edge, w, h);
                    draw_line(buf, ww, wh, cx, cy, ex, ey, COLOR_BACKGROUND);
                }
                Shape::Polygon { .. } => {
                    let verts: Vec<(i64, i64)> = body
                        .world_vertices()
                        .into_iter()
                        .map(|v| self.view.world_to_screen(v, w, h))
                        .collect();
                    fill_convex_polygon(buf, ww, wh, &verts, color);
                    for i in 0..verts.len() {
                        let (x0, y0) = verts[i];
                        let (x1, y1) = verts[(i + 1) % verts.len()];
                        draw_line(buf, ww, wh, x0, y0, x1, y1, COLOR_BACKGROUND);
                    }
                }
            }
        }

        if self.show_contacts {
            for m in &self.world.last_manifolds {
                for &c in &m.contacts {
                    let (cx, cy) = self.view.world_to_screen(c, w, h);
                    draw_disc(buf, ww, wh, cx, cy, 3, COLOR_CONTACT);
                }
            }
        }

        buffer.present().unwrap();
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let attrs = WindowAttributes::default()
            .with_title("Physics Engine — Pixels (CPU)")
            .with_inner_size(LogicalSize::new(WIDTH as f64, HEIGHT as f64));
        let window = Arc::new(event_loop.create_window(attrs).unwrap());
        let context = softbuffer::Context::new(window.clone()).unwrap();
        let surface = Surface::new(&context, window.clone()).unwrap();
        self.window = Some(window);
        self.surface = Some(surface);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),

            WindowEvent::Resized(PhysicalSize { width, height }) => {
                if let Some(surface) = &mut self.surface {
                    let _ = surface.resize(
                        NonZeroU32::new(width.max(1)).unwrap(),
                        NonZeroU32::new(height.max(1)).unwrap(),
                    );
                }
                if let Some(w) = &self.window {
                    w.request_redraw();
                }
            }

            WindowEvent::KeyboardInput { event, .. } => {
                if event.state != ElementState::Released {
                    return;
                }
                match event.logical_key {
                    Key::Named(NamedKey::Escape) => event_loop.exit(),
                    Key::Character(ref c) => match c.as_str() {
                        "1" => {
                            self.scene = Scene::Bounce;
                            self.reset();
                        }
                        "2" => {
                            self.scene = Scene::Stack;
                            self.reset();
                        }
                        "3" => {
                            self.scene = Scene::Billiard;
                            self.reset();
                        }
                        "4" => {
                            self.scene = Scene::Ramp;
                            self.reset();
                        }
                        "r" => self.reset(),
                        "p" => self.paused = !self.paused,
                        "c" => self.show_contacts = !self.show_contacts,
                        "+" | "=" => self.steps_per_frame = (self.steps_per_frame + 1).min(16),
                        "-" => self.steps_per_frame = self.steps_per_frame.saturating_sub(1).max(1),
                        _ => {}
                    },
                    _ => {}
                }
                if let Some(w) = &self.window {
                    w.request_redraw();
                }
            }

            WindowEvent::CursorMoved { position, .. } => {
                let (mx, my) = (position.x, position.y);
                if self.view.dragging {
                    let dx = (mx - self.view.last_mouse.0) / self.view.scale;
                    let dy = (my - self.view.last_mouse.1) / self.view.scale;
                    self.view.center_x -= dx;
                    self.view.center_y += dy;
                }
                self.view.last_mouse = (mx, my);
            }

            WindowEvent::MouseInput {
                state,
                button: MouseButton::Left,
                ..
            } => {
                self.view.dragging = state == ElementState::Pressed;
            }

            WindowEvent::MouseWheel { delta, .. } => {
                let zoom = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(p) => p.y / 60.0,
                };
                self.view.scale = (self.view.scale * (1.0 + 0.1 * zoom)).clamp(5.0, 400.0);
                if let Some(w) = &self.window {
                    w.request_redraw();
                }
            }

            WindowEvent::RedrawRequested => {
                if !self.paused {
                    for _ in 0..self.steps_per_frame {
                        self.world.step(1.0 / 60.0);
                    }
                }
                self.draw();
            }

            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(w) = &self.window {
            w.request_redraw();
        }
    }
}

fn put_pixel(buf: &mut [u32], ww: usize, wh: usize, x: i64, y: i64, color: u32) {
    if x < 0 || y < 0 || x >= ww as i64 || y >= wh as i64 {
        return;
    }
    buf[y as usize * ww + x as usize] = color;
}

fn draw_disc(buf: &mut [u32], ww: usize, wh: usize, cx: i64, cy: i64, radius: i64, color: u32) {
    let r2 = radius * radius;
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            if dx * dx + dy * dy <= r2 {
                put_pixel(buf, ww, wh, cx + dx, cy + dy, color);
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_line(
    buf: &mut [u32],
    ww: usize,
    wh: usize,
    mut x0: i64,
    mut y0: i64,
    x1: i64,
    y1: i64,
    color: u32,
) {
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    loop {
        put_pixel(buf, ww, wh, x0, y0, color);
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

/// Remplissage par balayage de lignes (*scanline*) : pour un polygone
/// convexe, chaque ligne horizontale ne coupe le contour qu'en exactement
/// deux points (à une tangence près) — on trouve ces deux intersections par
/// arête et on remplit entre les deux.
fn fill_convex_polygon(buf: &mut [u32], ww: usize, wh: usize, verts: &[(i64, i64)], color: u32) {
    if verts.len() < 3 {
        return;
    }
    let min_y = verts.iter().map(|p| p.1).min().unwrap().max(0);
    let max_y = verts.iter().map(|p| p.1).max().unwrap().min(wh as i64 - 1);

    for y in min_y..=max_y {
        let mut xs = Vec::new();
        for i in 0..verts.len() {
            let (x0, y0) = verts[i];
            let (x1, y1) = verts[(i + 1) % verts.len()];
            if (y0 <= y && y1 > y) || (y1 <= y && y0 > y) {
                let t = (y - y0) as f64 / (y1 - y0) as f64;
                xs.push(x0 as f64 + t * (x1 - x0) as f64);
            }
        }
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for pair in xs.chunks(2) {
            if let [x_start, x_end] = pair {
                let x_start = x_start.round() as i64;
                let x_end = x_end.round() as i64;
                for x in x_start..=x_end {
                    put_pixel(buf, ww, wh, x, y, color);
                }
            }
        }
    }
}

fn main() {
    let event_loop = EventLoop::new().unwrap();
    let scene = Scene::Bounce;
    let mut app = App {
        world: scene.build(),
        scene,
        paused: false,
        steps_per_frame: 1,
        show_contacts: true,
        view: View::new(),
        window: None,
        surface: None,
    };
    event_loop.run_app(&mut app).unwrap();
}
