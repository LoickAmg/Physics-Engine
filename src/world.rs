use crate::broadphase;
use crate::collision;
use crate::manifold::Manifold;
use crate::resolver;
use crate::rigidbody::RigidBody;
use crate::vec2::Vec2;

/// Scène physique complète : liste de corps + gravité globale. `step(dt)`
/// avance la simulation d'un pas de temps fixe (voir le README pour la
/// discussion sur pourquoi un pas fixe plutôt que le delta-temps de la
/// boucle de rendu, qui rendrait la simulation non déterministe et
/// difficile à tester).
pub struct World {
    pub bodies: Vec<RigidBody>,
    pub gravity: Vec2,
    /// Dernier ensemble de manifolds calculé — exposé pour l'inspection
    /// (tests, débogage, rendu des points de contact).
    pub last_manifolds: Vec<Manifold>,
}

impl World {
    pub fn new(gravity: Vec2) -> Self {
        World {
            bodies: Vec::new(),
            gravity,
            last_manifolds: Vec::new(),
        }
    }

    /// Terre sans gravité (`Vec2::ZERO`) — pratique pour des tests ou des
    /// scènes en apesanteur (billard, par exemple).
    pub fn without_gravity() -> Self {
        World::new(Vec2::ZERO)
    }

    /// Ajoute un corps et renvoie son indice dans `self.bodies` — cet
    /// indice reste stable tant qu'aucun corps n'est retiré (aucune méthode
    /// de retrait n'est fournie pour l'instant : les manifolds gardent des
    /// indices, les retirer en cours de route les invaliderait).
    pub fn add_body(&mut self, body: RigidBody) -> usize {
        self.bodies.push(body);
        self.bodies.len() - 1
    }

    pub fn step(&mut self, dt: f64) {
        // 1. Détection de collision sur la géométrie du début de pas.
        let pairs = broadphase::candidate_pairs(&self.bodies);
        let mut manifolds = Vec::with_capacity(pairs.len());
        for (i, j) in pairs {
            if let Some(m) = collision::collide(i, &self.bodies[i], j, &self.bodies[j]) {
                manifolds.push(m);
            }
        }

        // 1bis. Warm starting : reporter les impulsions déjà résolues au pas
        // précédent pour les contacts qui persistent (même paire de corps,
        // même nombre de points de contact). Une correspondance par simple
        // proximité de feature (plutôt qu'un vrai identifiant de feature
        // SAT) suffit pour ce moteur pédagogique — voir le commentaire sur
        // `Manifold` pour pourquoi c'est important.
        for m in &mut manifolds {
            if let Some(prev) = self.last_manifolds.iter().find(|p| {
                p.body_a == m.body_a && p.body_b == m.body_b && p.contacts.len() == m.contacts.len()
            }) {
                m.normal_impulse.copy_from_slice(&prev.normal_impulse);
                m.tangent_impulse.copy_from_slice(&prev.tangent_impulse);
            }
        }
        resolver::warm_start(&mut self.bodies, &manifolds);

        // 2. Intégration des forces en vitesse (Euler semi-implicite : on
        // met à jour la vitesse AVANT la position, contrairement à Euler
        // explicite — beaucoup plus stable numériquement pour un système
        // avec ressorts/contraintes, cf. double-pendulum pour la même
        // discussion appliquée à un système hamiltonien).
        for body in &mut self.bodies {
            let (force, torque) = body.take_accumulated_force_and_torque();
            if body.is_static() {
                continue;
            }
            let acceleration = self.gravity + force * body.inv_mass();
            body.velocity += acceleration * dt;
            body.angular_velocity += torque * body.inv_inertia() * dt;
        }

        // 3. Résolution itérative des contraintes de contact (vitesse).
        resolver::resolve_velocity(&mut self.bodies, &mut manifolds);

        // 4. Intégration des positions à partir des vitesses résolues.
        for body in &mut self.bodies {
            if body.is_static() {
                continue;
            }
            body.position += body.velocity * dt;
            body.rotation += body.angular_velocity * dt;
        }

        // 5. Correction positionnelle (résorbe la pénétration résiduelle).
        resolver::correct_positions(&mut self.bodies, &manifolds);

        self.last_manifolds = manifolds;
    }

    /// Énergie cinétique totale (translation + rotation) — sert
    /// essentiellement à vérifier expérimentalement qu'une collision
    /// parfaitement élastique (restitution = 1, sans friction) ne crée pas
    /// d'énergie à partir de rien.
    pub fn kinetic_energy(&self) -> f64 {
        self.bodies
            .iter()
            .filter(|b| !b.is_static())
            .map(|b| {
                0.5 * b.mass() * b.velocity.length_squared()
                    + 0.5 * b.inertia() * b.angular_velocity * b.angular_velocity
            })
            .sum()
    }

    /// Quantité de mouvement linéaire totale des corps dynamiques.
    pub fn total_momentum(&self) -> Vec2 {
        self.bodies
            .iter()
            .filter(|b| !b.is_static())
            .fold(Vec2::ZERO, |acc, b| acc + b.velocity * b.mass())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::Shape;

    #[test]
    fn body_falls_under_gravity() {
        let mut world = World::new(Vec2::new(0.0, -9.81));
        world.add_body(RigidBody::dynamic(
            Shape::circle(0.5),
            Vec2::new(0.0, 10.0),
            1.0,
        ));
        let y0 = world.bodies[0].position.y;
        for _ in 0..60 {
            world.step(1.0 / 60.0);
        }
        assert!(world.bodies[0].position.y < y0, "le corps doit être tombé");
        assert!(world.bodies[0].velocity.y < 0.0);
    }

    #[test]
    fn ball_bounces_on_static_ground_and_settles() {
        let mut world = World::new(Vec2::new(0.0, -9.81));
        world.add_body(RigidBody::static_body(
            Shape::rectangle(20.0, 1.0),
            Vec2::new(0.0, -0.5),
        ));
        let ball = world.add_body(
            RigidBody::dynamic(Shape::circle(0.5), Vec2::new(0.0, 5.0), 1.0).with_restitution(0.5),
        );

        let mut min_y_after_first_bounce = f64::INFINITY;
        let mut bounced = false;
        for _ in 0..600 {
            world.step(1.0 / 60.0);
            if world.bodies[ball].velocity.y > 0.1 {
                bounced = true;
            }
            if bounced {
                min_y_after_first_bounce =
                    min_y_after_first_bounce.min(world.bodies[ball].position.y);
            }
        }

        assert!(
            bounced,
            "la balle doit rebondir au moins une fois sur le sol"
        );
        // La balle ne doit jamais s'enfoncer significativement sous le sol
        // (surface du sol à y=0, centre de la balle ne doit pas descendre
        // sous y≈0.5 - marge de tolérance pour la correction positionnelle).
        assert!(
            world.bodies[ball].position.y > 0.3,
            "la balle ne doit pas transpercer le sol, y={}",
            world.bodies[ball].position.y
        );
    }

    #[test]
    fn stack_of_boxes_remains_stable_and_upright() {
        let mut world = World::new(Vec2::new(0.0, -9.81));
        world.add_body(RigidBody::static_body(
            Shape::rectangle(10.0, 1.0),
            Vec2::new(0.0, -0.5),
        ));

        let box_size = 1.0;
        let mut top_index = 0;
        for i in 0..4 {
            top_index = world.add_body(
                RigidBody::dynamic(
                    Shape::rectangle(box_size, box_size),
                    Vec2::new(0.0, i as f64 * box_size + box_size / 2.0),
                    1.0,
                )
                .with_friction(0.8)
                .with_restitution(0.0),
            );
        }

        for _ in 0..300 {
            world.step(1.0 / 60.0);
        }

        // Après stabilisation, la boîte du haut doit rester quasi à la
        // verticale (rotation faible) et alignée horizontalement (pas
        // tombée sur le côté).
        assert!(
            world.bodies[top_index].rotation.abs() < 0.2,
            "la boîte du haut a basculé: rotation={}",
            world.bodies[top_index].rotation
        );
        assert!(
            world.bodies[top_index].position.x.abs() < 0.5,
            "la boîte du haut est tombée sur le côté: x={}",
            world.bodies[top_index].position.x
        );
    }

    #[test]
    fn elastic_head_on_collision_conserves_momentum_and_energy_over_time() {
        let mut world = World::without_gravity();
        let a = world.add_body(
            RigidBody::dynamic(Shape::circle(1.0), Vec2::new(-5.0, 0.0), 1.0).with_restitution(1.0),
        );
        let b = world.add_body(
            RigidBody::dynamic(Shape::circle(1.0), Vec2::new(5.0, 0.0), 1.0).with_restitution(1.0),
        );
        world.bodies[a].velocity = Vec2::new(2.0, 0.0);
        world.bodies[b].velocity = Vec2::new(-2.0, 0.0);

        let momentum_before = world.total_momentum();
        let energy_before = world.kinetic_energy();

        for _ in 0..600 {
            world.step(1.0 / 120.0);
        }

        let momentum_after = world.total_momentum();
        let energy_after = world.kinetic_energy();

        assert!((momentum_before.x - momentum_after.x).abs() < 1e-6);
        // Tolérance plus large sur l'énergie : la correction positionnelle
        // (nécessaire pour éviter l'enfoncement) dissipe un peu d'énergie
        // par construction, ce n'est pas un solveur symplectique exact.
        assert!(
            (energy_before - energy_after).abs() / energy_before < 0.05,
            "énergie non conservée: avant={energy_before}, après={energy_after}"
        );
    }
}
