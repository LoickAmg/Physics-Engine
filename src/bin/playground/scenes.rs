//! Scènes, matériaux et mini-jeux de Physics Playground.

use eframe::egui::Color32;
use physics::{RigidBody, Shape, Vec2, World};

pub const GRAVITY: f64 = -9.81;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Material {
    Wood,
    Rubber,
    Ice,
    Steel,
    Stone,
}

impl Material {
    pub const ALL: [Material; 5] = [
        Material::Wood,
        Material::Rubber,
        Material::Ice,
        Material::Steel,
        Material::Stone,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Material::Wood => "Bois",
            Material::Rubber => "Caoutchouc",
            Material::Ice => "Glace",
            Material::Steel => "Acier",
            Material::Stone => "Pierre",
        }
    }

    pub fn describe(self) -> &'static str {
        match self {
            Material::Wood => "Léger, rebondit peu, accroche bien.",
            Material::Rubber => "Rebondit énormément.",
            Material::Ice => "Glisse presque sans frottement.",
            Material::Steel => "Très lourd : idéal pour démolir.",
            Material::Stone => "Lourd et rugueux, ne rebondit pas.",
        }
    }

    /// (densité, rebond, frottement)
    pub fn params(self) -> (f64, f64, f64) {
        match self {
            Material::Wood => (0.7, 0.15, 0.6),
            Material::Rubber => (1.1, 0.85, 0.9),
            Material::Ice => (0.9, 0.05, 0.02),
            Material::Steel => (7.8, 0.25, 0.35),
            Material::Stone => (2.6, 0.02, 0.8),
        }
    }

    pub fn color(self) -> Color32 {
        match self {
            Material::Wood => Color32::from_rgb(222, 164, 96),
            Material::Rubber => Color32::from_rgb(247, 72, 133),
            Material::Ice => Color32::from_rgb(150, 225, 255),
            Material::Steel => Color32::from_rgb(160, 172, 196),
            Material::Stone => Color32::from_rgb(140, 132, 150),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    Normal,
    /// Pièce du château à faire tomber (Démolition).
    Target,
    /// Boulet tiré à la fronde.
    Ammo,
}

pub struct Meta {
    pub color: Color32,
    pub role: Role,
}

pub struct Built {
    pub world: World,
    pub meta: Vec<Meta>,
    pub view: (Vec2, f64), // centre et demi-largeur visibles (m)
}

struct Builder {
    world: World,
    meta: Vec<Meta>,
}

pub const STATIC_COLOR: Color32 = Color32::from_rgb(58, 64, 96);

impl Builder {
    fn new(gravity: Vec2) -> Self {
        Builder {
            world: World::new(gravity),
            meta: Vec::new(),
        }
    }
    fn fixed(&mut self, shape: Shape, pos: Vec2, rotation: f64) {
        let mut b = RigidBody::static_body(shape, pos).with_friction(0.7);
        b.rotation = rotation;
        self.world.add_body(b);
        self.meta.push(Meta {
            color: STATIC_COLOR,
            role: Role::Normal,
        });
    }
    fn ground(&mut self, width: f64) {
        self.fixed(Shape::rectangle(width, 2.0), Vec2::new(0.0, -1.0), 0.0);
    }
    fn body(&mut self, shape: Shape, pos: Vec2, mat: Material, role: Role) -> usize {
        let (density, restitution, friction) = mat.params();
        let i = self.world.add_body(
            RigidBody::dynamic(shape, pos, density)
                .with_restitution(restitution)
                .with_friction(friction),
        );
        let color = if role == Role::Target {
            Color32::from_rgb(255, 196, 80)
        } else {
            mat.color()
        };
        self.meta.push(Meta { color, role });
        i
    }
    fn done(self, center: Vec2, half_width: f64) -> Built {
        Built {
            world: self.world,
            meta: self.meta,
            view: (center, half_width),
        }
    }
}

pub fn spawn(
    world: &mut World,
    meta: &mut Vec<Meta>,
    shape: Shape,
    pos: Vec2,
    mat: Material,
    role: Role,
    color: Option<Color32>,
) -> usize {
    let (density, restitution, friction) = mat.params();
    let i = world.add_body(
        RigidBody::dynamic(shape, pos, density)
            .with_restitution(restitution)
            .with_friction(friction),
    );
    meta.push(Meta {
        color: color.unwrap_or(mat.color()),
        role,
    });
    i
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scene {
    Sandbox,
    Pyramid,
    Dominoes,
    Galton,
    Billiard,
    Ramps,
    Demolition,
    Tower,
}

impl Scene {
    pub const PLAY: [Scene; 6] = [
        Scene::Sandbox,
        Scene::Pyramid,
        Scene::Dominoes,
        Scene::Galton,
        Scene::Billiard,
        Scene::Ramps,
    ];
    pub const GAMES: [Scene; 2] = [Scene::Demolition, Scene::Tower];

    pub fn label(self) -> &'static str {
        match self {
            Scene::Sandbox => "Bac à sable",
            Scene::Pyramid => "Pyramide",
            Scene::Dominoes => "Dominos",
            Scene::Galton => "Planche de Galton",
            Scene::Billiard => "Billard",
            Scene::Ramps => "Rampes",
            Scene::Demolition => "Démolition",
            Scene::Tower => "Tour infernale",
        }
    }

    pub fn icon(self) -> &'static str {
        match self {
            Scene::Sandbox => "✏",
            Scene::Pyramid => "🔺",
            Scene::Dominoes => "🎳",
            Scene::Galton => "🔘",
            Scene::Billiard => "🎱",
            Scene::Ramps => "📐",
            Scene::Demolition => "💥",
            Scene::Tower => "🗼",
        }
    }

    pub fn description(self) -> &'static str {
        match self {
            Scene::Sandbox => {
                "Un sol, deux murs : crée, empile, lance et fais exploser ce que tu veux."
            }
            Scene::Pyramid => {
                "55 boîtes en équilibre. Attrape celle du bas… ou lance un boulet d'acier !"
            }
            Scene::Dominoes => "Une bille dévale la rampe et lance la réaction en chaîne.",
            Scene::Galton => {
                "Des billes rebondissent sur des clous : elles dessinent une courbe en cloche."
            }
            Scene::Billiard => "Sans gravité : glisse depuis la bille blanche pour la lancer.",
            Scene::Ramps => "Glace, bois, caoutchouc : qui arrive en bas le premier ?",
            Scene::Demolition => {
                "5 boulets pour faire tomber le château de son socle. Glisse pour viser !"
            }
            Scene::Tower => {
                "20 pièces pour bâtir la tour la plus haute possible, sans qu'elle s'effondre."
            }
        }
    }

    pub fn build(self) -> Built {
        match self {
            Scene::Sandbox => sandbox(),
            Scene::Pyramid => pyramid(),
            Scene::Dominoes => dominoes(),
            Scene::Galton => galton(),
            Scene::Billiard => billiard(),
            Scene::Ramps => ramps(),
            Scene::Demolition => demolition(),
            Scene::Tower => tower(),
        }
    }
}

fn sandbox() -> Built {
    let mut b = Builder::new(Vec2::new(0.0, GRAVITY));
    b.ground(40.0);
    b.fixed(Shape::rectangle(1.0, 30.0), Vec2::new(-14.5, 15.0), 0.0);
    b.fixed(Shape::rectangle(1.0, 30.0), Vec2::new(14.5, 15.0), 0.0);
    for i in 0..4 {
        b.body(
            Shape::rectangle(1.2, 1.2),
            Vec2::new(-4.0 + i as f64 * 2.6, 0.6),
            Material::Wood,
            Role::Normal,
        );
    }
    b.body(
        Shape::circle(0.6),
        Vec2::new(4.0, 6.0),
        Material::Rubber,
        Role::Normal,
    );
    b.done(Vec2::new(0.0, 7.0), 15.5)
}

fn pyramid() -> Built {
    let mut b = Builder::new(Vec2::new(0.0, GRAVITY));
    b.ground(60.0);
    let size = 1.0;
    let levels = 10;
    for row in 0..levels {
        let count = levels - row;
        for i in 0..count {
            let x = (i as f64 - (count - 1) as f64 / 2.0) * (size * 1.02);
            let y = size / 2.0 + row as f64 * size;
            let mat = if (row + i) % 2 == 0 {
                Material::Wood
            } else {
                Material::Stone
            };
            b.body(
                Shape::rectangle(size, size),
                Vec2::new(x, y),
                mat,
                Role::Normal,
            );
        }
    }
    b.done(Vec2::new(0.0, 6.0), 14.0)
}

fn dominoes() -> Built {
    let mut b = Builder::new(Vec2::new(0.0, GRAVITY));
    b.ground(60.0);
    // Rampe de départ
    b.fixed(Shape::rectangle(7.0, 0.3), Vec2::new(-13.0, 4.2), -0.5);
    b.body(
        Shape::circle(0.45),
        Vec2::new(-15.8, 6.3),
        Material::Steel,
        Role::Normal,
    );
    for i in 0..18 {
        let x = -8.8 + i as f64 * 1.15;
        let mat = if i % 2 == 0 {
            Material::Wood
        } else {
            Material::Stone
        };
        b.body(
            Shape::rectangle(0.25, 1.8),
            Vec2::new(x, 0.9),
            mat,
            Role::Normal,
        );
    }
    b.done(Vec2::new(0.0, 5.0), 17.0)
}

fn galton() -> Built {
    let mut b = Builder::new(Vec2::new(0.0, GRAVITY));
    b.ground(40.0);
    // Entonnoir
    b.fixed(Shape::rectangle(8.0, 0.3), Vec2::new(-4.2, 17.5), -0.55);
    b.fixed(Shape::rectangle(8.0, 0.3), Vec2::new(4.2, 17.5), 0.55);
    // Clous en quinconce
    for row in 0..9 {
        let count = 9 + row % 2;
        for i in 0..count {
            let x = (i as f64 - (count - 1) as f64 / 2.0) * 1.2;
            b.fixed(
                Shape::circle(0.13),
                Vec2::new(x, 14.0 - row as f64 * 1.0),
                0.0,
            );
        }
    }
    // Colonnes de réception
    for i in 0..12 {
        let x = -6.6 + i as f64 * 1.2;
        b.fixed(Shape::rectangle(0.12, 4.6), Vec2::new(x, 2.3), 0.0);
    }
    b.fixed(Shape::rectangle(0.6, 30.0), Vec2::new(-7.4, 12.0), 0.0);
    b.fixed(Shape::rectangle(0.6, 30.0), Vec2::new(7.4, 12.0), 0.0);
    for i in 0..90 {
        let x = -2.5 + (i % 10) as f64 * 0.55;
        let y = 20.0 + (i / 10) as f64 * 0.6;
        b.body(
            Shape::circle(0.2),
            Vec2::new(x, y),
            Material::Steel,
            Role::Normal,
        );
    }
    b.done(Vec2::new(0.0, 11.0), 12.0)
}

fn billiard() -> Built {
    let mut b = Builder::new(Vec2::ZERO);
    let (w, h) = (22.0, 11.0);
    b.fixed(Shape::rectangle(w + 1.0, 0.5), Vec2::new(0.0, h / 2.0), 0.0);
    b.fixed(
        Shape::rectangle(w + 1.0, 0.5),
        Vec2::new(0.0, -h / 2.0),
        0.0,
    );
    b.fixed(Shape::rectangle(0.5, h), Vec2::new(w / 2.0, 0.0), 0.0);
    b.fixed(Shape::rectangle(0.5, h), Vec2::new(-w / 2.0, 0.0), 0.0);
    let r = 0.35;
    let colors = [
        Color32::from_rgb(255, 209, 102),
        Color32::from_rgb(76, 201, 240),
        Color32::from_rgb(247, 37, 133),
        Color32::from_rgb(114, 239, 150),
        Color32::from_rgb(157, 115, 255),
    ];
    let mut n = 0;
    for col in 0..5 {
        for k in 0..=col {
            let x = 3.0 + col as f64 * r * 1.8;
            let y = (k as f64 - col as f64 / 2.0) * r * 2.05;
            let i = b.world.add_body(
                RigidBody::dynamic(Shape::circle(r), Vec2::new(x, y), 1.0)
                    .with_restitution(0.92)
                    .with_friction(0.05),
            );
            let _ = i;
            b.meta.push(Meta {
                color: colors[n % colors.len()],
                role: Role::Normal,
            });
            n += 1;
        }
    }
    b.world.add_body(
        RigidBody::dynamic(Shape::circle(r), Vec2::new(-6.0, 0.0), 1.0)
            .with_restitution(0.92)
            .with_friction(0.05),
    );
    b.meta.push(Meta {
        color: Color32::from_rgb(245, 245, 250),
        role: Role::Normal,
    });
    b.done(Vec2::ZERO, 12.5)
}

fn ramps() -> Built {
    let mut b = Builder::new(Vec2::new(0.0, GRAVITY));
    b.ground(60.0);
    for (k, mat) in [Material::Ice, Material::Wood, Material::Rubber]
        .iter()
        .enumerate()
    {
        let y = 4.0 + k as f64 * 4.0;
        b.fixed(Shape::rectangle(12.0, 0.3), Vec2::new(-4.0, y), -0.28);
        b.body(
            Shape::rectangle(1.0, 0.7),
            Vec2::new(-8.6, y + 1.7),
            *mat,
            Role::Normal,
        );
    }
    b.body(
        Shape::circle(0.5),
        Vec2::new(-9.0, 15.5),
        Material::Rubber,
        Role::Normal,
    );
    b.done(Vec2::new(0.0, 7.0), 14.0)
}

/// Démolition : un château sur un socle, à faire tomber avec 5 boulets.
pub const PLATFORM_TOP: f64 = 3.0;
pub const LAUNCHER: Vec2 = Vec2 { x: -13.0, y: 2.0 };

fn demolition() -> Built {
    let mut b = Builder::new(Vec2::new(0.0, GRAVITY));
    b.ground(80.0);
    b.fixed(
        Shape::rectangle(9.0, PLATFORM_TOP),
        Vec2::new(12.0, PLATFORM_TOP / 2.0),
        0.0,
    );
    b.fixed(Shape::rectangle(1.2, 2.0), Vec2::new(LAUNCHER.x, 1.0), 0.0);
    let base = PLATFORM_TOP;
    // Deux tours de blocs reliées par des linteaux, surmontées d'un donjon.
    for col in [9.0, 11.2, 12.8, 15.0] {
        for k in 0..3 {
            b.body(
                Shape::rectangle(0.8, 0.8),
                Vec2::new(col, base + 0.4 + k as f64 * 0.8),
                Material::Stone,
                Role::Target,
            );
        }
    }
    b.body(
        Shape::rectangle(3.2, 0.3),
        Vec2::new(10.1, base + 2.55),
        Material::Wood,
        Role::Target,
    );
    b.body(
        Shape::rectangle(3.2, 0.3),
        Vec2::new(13.9, base + 2.55),
        Material::Wood,
        Role::Target,
    );
    for col in [9.6, 10.6, 13.4, 14.4] {
        b.body(
            Shape::rectangle(0.6, 1.2),
            Vec2::new(col, base + 3.3),
            Material::Wood,
            Role::Target,
        );
    }
    b.body(
        Shape::rectangle(5.8, 0.3),
        Vec2::new(12.0, base + 4.05),
        Material::Wood,
        Role::Target,
    );
    b.body(
        Shape::rectangle(0.8, 0.8),
        Vec2::new(12.0, base + 4.6),
        Material::Stone,
        Role::Target,
    );
    b.body(
        Shape::polygon(vec![
            Vec2::new(-0.6, -0.4),
            Vec2::new(0.6, -0.4),
            Vec2::new(0.0, 0.6),
        ]),
        Vec2::new(12.0, base + 5.4),
        Material::Wood,
        Role::Target,
    );
    b.done(Vec2::new(0.0, 6.0), 17.0)
}

/// Tour infernale : un socle étroit et 20 pièces à empiler.
pub const TOWER_PIECES: u32 = 20;

fn tower() -> Built {
    let mut b = Builder::new(Vec2::new(0.0, GRAVITY));
    b.ground(60.0);
    b.fixed(Shape::rectangle(3.0, 1.0), Vec2::new(0.0, 0.5), 0.0);
    b.done(Vec2::new(0.0, 8.0), 12.0)
}
