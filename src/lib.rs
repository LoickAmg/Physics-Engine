//! Moteur physique 2D de corps rigides, écrit "from scratch" — pas de
//! `nalgebra`/`glam` pour l'algèbre vectorielle, pas de `rapier`/`nphysics`
//! pour la simulation. Le cœur (formes, corps, collision, résolution,
//! intégration) est zéro-dépendance ; seul le frontend visuel optionnel
//! (feature `pixels`) importe des crates externes pour l'affichage, jamais
//! pour la physique elle-même.

pub mod broadphase;
pub mod collision;
pub mod manifold;
pub mod resolver;
pub mod rigidbody;
pub mod scenes;
pub mod shape;
pub mod vec2;
pub mod world;

pub use manifold::Manifold;
pub use rigidbody::{Aabb, RigidBody};
pub use shape::Shape;
pub use vec2::Vec2;
pub use world::World;
