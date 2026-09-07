//! Scènes de démonstration prêtes à l'emploi, partagées entre le CLI
//! headless (`physics-cli`) et le frontend visuel (`physics-pixels`) — comme
//! le module `preset` de `gravity-simulation`, pour ne pas dupliquer la
//! même construction de scène à deux endroits.

use crate::rigidbody::RigidBody;
use crate::shape::Shape;
use crate::vec2::Vec2;
use crate::world::World;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scene {
    Bounce,
    Stack,
    Billiard,
    Ramp,
}

impl Scene {
    pub fn label(&self) -> &'static str {
        match self {
            Scene::Bounce => "Rebond",
            Scene::Stack => "Pile de boîtes",
            Scene::Billiard => "Billard (sans gravité)",
            Scene::Ramp => "Rampe avec friction",
        }
    }

    pub fn build(&self) -> World {
        match self {
            Scene::Bounce => bounce(),
            Scene::Stack => stack(5),
            Scene::Billiard => billiard(),
            Scene::Ramp => ramp(),
        }
    }
}

pub fn bounce() -> World {
    let mut world = World::new(Vec2::new(0.0, -9.81));
    world.add_body(RigidBody::static_body(
        Shape::rectangle(20.0, 1.0),
        Vec2::new(0.0, -0.5),
    ));
    world.add_body(
        RigidBody::dynamic(Shape::circle(0.5), Vec2::new(0.0, 8.0), 1.0).with_restitution(0.7),
    );
    world
}

pub fn stack(count: usize) -> World {
    let mut world = World::new(Vec2::new(0.0, -9.81));
    world.add_body(RigidBody::static_body(
        Shape::rectangle(10.0, 1.0),
        Vec2::new(0.0, -0.5),
    ));
    for i in 0..count {
        world.add_body(
            RigidBody::dynamic(
                Shape::rectangle(1.0, 1.0),
                Vec2::new(0.0, i as f64 + 0.5),
                1.0,
            )
            .with_friction(0.7)
            .with_restitution(0.0),
        );
    }
    world
}

pub fn billiard() -> World {
    let mut world = World::without_gravity();
    let cue = world.add_body(
        RigidBody::dynamic(Shape::circle(0.3), Vec2::new(-5.0, 0.0), 1.0)
            .with_restitution(0.9)
            .with_friction(0.05),
    );
    world.bodies[cue].velocity = Vec2::new(6.0, 0.0);
    for i in 0..3 {
        world.add_body(
            RigidBody::dynamic(
                Shape::circle(0.3),
                Vec2::new(0.0, i as f64 * 0.65 - 0.65),
                1.0,
            )
            .with_restitution(0.9)
            .with_friction(0.05),
        );
    }
    world
}

pub fn ramp() -> World {
    let mut world = World::new(Vec2::new(0.0, -9.81));
    let mut ramp = RigidBody::static_body(Shape::rectangle(12.0, 0.5), Vec2::new(0.0, 0.0));
    ramp.rotation = -0.3;
    world.add_body(ramp);
    world.add_body(
        RigidBody::dynamic(Shape::circle(0.4), Vec2::new(-4.0, 2.0), 1.0)
            .with_friction(0.4)
            .with_restitution(0.1),
    );
    world
}
