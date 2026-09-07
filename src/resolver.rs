//! Résolveur de collisions par impulsions séquentielles (*sequential impulse
//! solver*), la technique standard des moteurs physiques temps réel
//! (Box2D, Chipmunk...) : plutôt que de résoudre exactement un système
//! d'équations pour toutes les contraintes de contact simultanément (coûteux
//! et rarement nécessaire), on relâche chaque contact l'un après l'autre, et
//! on répète plusieurs fois — la solution converge sans jamais être exacte,
//! ce qui suffit largement à un pas de temps de simulation.

use crate::manifold::Manifold;
use crate::rigidbody::RigidBody;

/// Nombre de passes de relaxation par pas de temps. Plus il y a de contacts
/// simultanés (une pile d'objets, typiquement), plus il en faut pour que
/// l'effet d'un contact se propage correctement aux autres.
const VELOCITY_ITERATIONS: usize = 10;

/// En dessous de cette pénétration, on ne corrige pas la position — sans
/// cette tolérance, deux corps parfaitement au repos oscilleraient sans fin
/// entre "légèrement séparés" et "légèrement pénétrants" (*jitter*).
const PENETRATION_SLOP: f64 = 0.005;

/// Fraction de la pénétration résiduelle corrigée à chaque pas — corriger
/// 100% d'un coup est instable (voir Baumgarte stabilization), une
/// correction partielle répétée à chaque pas est beaucoup plus stable.
const POSITIONAL_CORRECTION_PERCENT: f64 = 0.2;

/// En dessous de cette vitesse d'approche, on n'applique aucune restitution
/// — sans ce seuil, un contact presque au repos "rebondirait" indéfiniment
/// sur un bruit numérique de l'ordre de 1e-9, ce qui ne s'arrêterait jamais
/// (pratique standard, ex. Box2D `b2_velocityThreshold`).
const RESTITUTION_VELOCITY_THRESHOLD: f64 = 0.5;

/// Applique immédiatement les impulsions déjà accumulées dans chaque
/// manifold (typiquement reportées du pas précédent par `World::step` pour
/// les contacts qui persistent d'un pas à l'autre — voir `Manifold`). Sans
/// cette étape, chaque pas repartirait de zéro et redériverait la même
/// solution "à froid", ce qui converge plus lentement et surtout laisse
/// s'accumuler du bruit numérique — la cause directe de l'instabilité
/// observée sur une pile de boîtes sans warm-starting.
pub fn warm_start(bodies: &mut [RigidBody], manifolds: &[Manifold]) {
    for m in manifolds {
        let tangent = m.normal.perp();
        for (i, &contact) in m.contacts.iter().enumerate() {
            let impulse = m.normal * m.normal_impulse[i] + tangent * m.tangent_impulse[i];
            bodies[m.body_a].apply_impulse_at_point(-impulse, contact);
            bodies[m.body_b].apply_impulse_at_point(impulse, contact);
        }
    }
}

/// Résolveur par impulsions séquentielles **accumulées** : au lieu
/// d'appliquer une impulsion complète à chaque itération (ce qui reviendrait
/// à sur-corriger dès qu'un contact interagit avec un autre via un corps
/// partagé), chaque itération calcule l'ajustement nécessaire par rapport à
/// l'impulsion déjà accumulée pour ce contact, borne le **total** (pas
/// l'ajustement) à rester physiquement valide (jamais négatif pour la
/// normale, dans le cône de friction pour la tangente), puis n'applique que
/// la différence. C'est la construction standard (Box2D, Chipmunk...) : elle
/// converge vers la même solution que la version "naïve" mais reste stable
/// même avec plusieurs contacts couplés par un corps commun (le cas d'une
/// pile d'objets, justement).
pub fn resolve_velocity(bodies: &mut [RigidBody], manifolds: &mut [Manifold]) {
    // Le biais de restitution est dérivé de la vitesse d'approche INITIALE
    // (avant toute résolution) et reste fixe pendant toutes les itérations.
    // Erreur classique à éviter : le recalculer à partir de la vitesse
    // courante à chaque itération fait osciller indéfiniment le contact
    // entre "impulsion pleine" et "impulsion nulle" au lieu de converger,
    // puisque la cible elle-même bougerait à chaque pas de la boucle.
    let biases: Vec<Vec<f64>> = manifolds
        .iter()
        .map(|m| compute_restitution_bias(bodies, m))
        .collect();

    for _ in 0..VELOCITY_ITERATIONS {
        for (m, bias) in manifolds.iter_mut().zip(biases.iter()) {
            resolve_manifold_velocity(bodies, m, bias);
        }
    }
}

fn compute_restitution_bias(bodies: &[RigidBody], m: &Manifold) -> Vec<f64> {
    let restitution = bodies[m.body_a]
        .restitution
        .min(bodies[m.body_b].restitution);
    m.contacts
        .iter()
        .map(|&contact| {
            let rel_vel = bodies[m.body_b].velocity_at_point(contact)
                - bodies[m.body_a].velocity_at_point(contact);
            let vn = rel_vel.dot(m.normal);
            if vn < -RESTITUTION_VELOCITY_THRESHOLD {
                -restitution * vn
            } else {
                0.0
            }
        })
        .collect()
}

fn resolve_manifold_velocity(bodies: &mut [RigidBody], m: &mut Manifold, bias: &[f64]) {
    let inv_mass_a = bodies[m.body_a].inv_mass();
    let inv_mass_b = bodies[m.body_b].inv_mass();
    if inv_mass_a == 0.0 && inv_mass_b == 0.0 {
        return;
    }
    let inv_inertia_a = bodies[m.body_a].inv_inertia();
    let inv_inertia_b = bodies[m.body_b].inv_inertia();
    let friction = (bodies[m.body_a].friction * bodies[m.body_b].friction).sqrt();
    let tangent = m.normal.perp();

    // Plusieurs tableaux parallèles (contacts, bias, impulsions normale et
    // tangentielle) sont indexés ensemble ici — un simple `enumerate()` sur
    // l'un d'eux ne suffirait pas à couvrir les autres proprement.
    #[allow(clippy::needless_range_loop)]
    for i in 0..m.contacts.len() {
        let contact = m.contacts[i];
        let ra = contact - bodies[m.body_a].position;
        let rb = contact - bodies[m.body_b].position;

        // --- Impulsion normale ---
        let rel_vel = bodies[m.body_b].velocity_at_point(contact)
            - bodies[m.body_a].velocity_at_point(contact);
        let vel_along_normal = rel_vel.dot(m.normal);

        let ra_cross_n = ra.cross(m.normal);
        let rb_cross_n = rb.cross(m.normal);
        let k_normal = inv_mass_a
            + inv_mass_b
            + inv_inertia_a * ra_cross_n * ra_cross_n
            + inv_inertia_b * rb_cross_n * rb_cross_n;

        if k_normal > 0.0 {
            // On pousse vel_along_normal vers `bias[i]` (la vitesse de
            // séparation cible, fixée une fois pour toutes plus haut) plutôt
            // que vers 0 — c'est ce qui rend la convergence stable sur
            // plusieurs itérations au lieu d'osciller.
            let delta_impulse = (bias[i] - vel_along_normal) / k_normal;
            // Le total accumulé ne doit jamais devenir négatif : un contact
            // ne peut que pousser (jamais tirer, ce qui collerait les corps).
            let new_impulse = (m.normal_impulse[i] + delta_impulse).max(0.0);
            let applied = new_impulse - m.normal_impulse[i];
            m.normal_impulse[i] = new_impulse;

            let impulse = m.normal * applied;
            bodies[m.body_a].apply_impulse_at_point(-impulse, contact);
            bodies[m.body_b].apply_impulse_at_point(impulse, contact);
        }

        // --- Friction de Coulomb (tangente fixe, dérivée de la normale une
        // fois pour toutes plutôt que recalculée depuis la vitesse relative
        // courante — nécessaire pour que l'impulsion accumulée reste
        // définie le long d'un axe stable d'une itération à l'autre). ---
        let ra_cross_t = ra.cross(tangent);
        let rb_cross_t = rb.cross(tangent);
        let k_tangent = inv_mass_a
            + inv_mass_b
            + inv_inertia_a * ra_cross_t * ra_cross_t
            + inv_inertia_b * rb_cross_t * rb_cross_t;

        if k_tangent > 0.0 {
            let rel_vel_after = bodies[m.body_b].velocity_at_point(contact)
                - bodies[m.body_a].velocity_at_point(contact);
            let vel_along_tangent = rel_vel_after.dot(tangent);

            let delta_impulse = -vel_along_tangent / k_tangent;
            let max_friction_impulse = friction * m.normal_impulse[i];
            let new_impulse = (m.tangent_impulse[i] + delta_impulse)
                .clamp(-max_friction_impulse, max_friction_impulse);
            let applied = new_impulse - m.tangent_impulse[i];
            m.tangent_impulse[i] = new_impulse;

            let friction_impulse = tangent * applied;
            bodies[m.body_a].apply_impulse_at_point(-friction_impulse, contact);
            bodies[m.body_b].apply_impulse_at_point(friction_impulse, contact);
        }
    }
}

/// Corrige directement les positions pour résorber la pénétration
/// résiduelle qui subsiste après résolution des vitesses (la résolution de
/// vitesse seule empêche les corps de s'enfoncer *davantage*, mais ne les
/// sépare pas s'ils étaient déjà en contact enfoncé au début du pas).
pub fn correct_positions(bodies: &mut [RigidBody], manifolds: &[Manifold]) {
    for m in manifolds {
        let inv_mass_a = bodies[m.body_a].inv_mass();
        let inv_mass_b = bodies[m.body_b].inv_mass();
        let total_inv_mass = inv_mass_a + inv_mass_b;
        if total_inv_mass <= 0.0 {
            continue;
        }
        let magnitude = (m.penetration - PENETRATION_SLOP).max(0.0) / total_inv_mass
            * POSITIONAL_CORRECTION_PERCENT;
        let correction = m.normal * magnitude;
        bodies[m.body_a].position -= correction * inv_mass_a;
        bodies[m.body_b].position += correction * inv_mass_b;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rigidbody::RigidBody;
    use crate::shape::Shape;
    use crate::vec2::Vec2;

    #[test]
    fn separating_bodies_receive_no_impulse() {
        let mut a = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(0.0, 0.0), 1.0);
        a.velocity = Vec2::new(-1.0, 0.0);
        let mut b = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(1.9, 0.0), 1.0);
        b.velocity = Vec2::new(1.0, 0.0);
        let mut bodies = vec![a, b];
        let mut m = Manifold::new(0, 1, Vec2::new(1.0, 0.0), 0.1, vec![Vec2::new(0.95, 0.0)]);
        let before = (bodies[0].velocity, bodies[1].velocity);
        resolve_velocity(&mut bodies, std::slice::from_mut(&mut m));
        assert_eq!(bodies[0].velocity, before.0);
        assert_eq!(bodies[1].velocity, before.1);
    }

    #[test]
    fn head_on_elastic_collision_conserves_momentum() {
        let mut a =
            RigidBody::dynamic(Shape::circle(1.0), Vec2::new(0.0, 0.0), 1.0).with_restitution(1.0);
        a.velocity = Vec2::new(1.0, 0.0);
        let mut b =
            RigidBody::dynamic(Shape::circle(1.0), Vec2::new(1.9, 0.0), 1.0).with_restitution(1.0);
        b.velocity = Vec2::ZERO;
        let mut bodies = vec![a, b];

        let momentum_before =
            bodies[0].velocity * bodies[0].mass() + bodies[1].velocity * bodies[1].mass();

        let mut m = Manifold::new(0, 1, Vec2::new(1.0, 0.0), 0.1, vec![Vec2::new(0.95, 0.0)]);
        resolve_velocity(&mut bodies, std::slice::from_mut(&mut m));

        let momentum_after =
            bodies[0].velocity * bodies[0].mass() + bodies[1].velocity * bodies[1].mass();
        assert!((momentum_before.x - momentum_after.x).abs() < 1e-9);

        // Deux masses égales, restitution 1 -> échange complet des vitesses.
        assert!((bodies[0].velocity.x - 0.0).abs() < 1e-6);
        assert!((bodies[1].velocity.x - 1.0).abs() < 1e-6);
    }

    #[test]
    fn static_body_never_moves_after_resolution() {
        let a = RigidBody::static_body(Shape::circle(1.0), Vec2::ZERO);
        let mut b = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(1.9, 0.0), 1.0);
        b.velocity = Vec2::new(-1.0, 0.0);
        let mut bodies = vec![a, b];
        let mut m = Manifold::new(0, 1, Vec2::new(1.0, 0.0), 0.1, vec![Vec2::new(0.95, 0.0)]);
        resolve_velocity(&mut bodies, std::slice::from_mut(&mut m));
        assert_eq!(bodies[0].velocity, Vec2::ZERO);
        assert!(
            bodies[1].velocity.x > -1.0,
            "la boule doit rebondir, pas continuer à s'enfoncer"
        );
    }

    #[test]
    fn positional_correction_moves_bodies_apart_proportionally_to_inverse_mass() {
        let mut a = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(0.0, 0.0), 1.0);
        let mut b = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(1.0, 0.0), 2.0); // 2x plus massif
        a.velocity = Vec2::ZERO;
        b.velocity = Vec2::ZERO;
        let mut bodies = vec![a, b];
        let m = Manifold::new(0, 1, Vec2::new(1.0, 0.0), 0.5, vec![Vec2::new(0.5, 0.0)]);
        correct_positions(&mut bodies, &[m]);
        let moved_a = -bodies[0].position.x; // a a reculé (position initiale 0)
        let moved_b = bodies[1].position.x - 1.0; // b a avancé (position initiale 1)
        assert!(moved_a > 0.0 && moved_b > 0.0);
        // Le corps le plus léger (a, masse 1, inv_mass 1) doit bouger davantage
        // que le corps le plus lourd (b, masse 2, inv_mass 0.5).
        assert!(moved_a > moved_b);
    }

    #[test]
    fn correction_below_slop_is_ignored() {
        let mut a = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(0.0, 0.0), 1.0);
        let mut b = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(1.0, 0.0), 1.0);
        a.velocity = Vec2::ZERO;
        b.velocity = Vec2::ZERO;
        let mut bodies = vec![a, b];
        // Pénétration sous PENETRATION_SLOP (0.005) : ne doit déclencher aucune correction.
        let m = Manifold::new(0, 1, Vec2::new(1.0, 0.0), 0.001, vec![Vec2::new(0.5, 0.0)]);
        let pos_before = (bodies[0].position, bodies[1].position);
        correct_positions(&mut bodies, &[m]);
        assert_eq!(bodies[0].position, pos_before.0);
        assert_eq!(bodies[1].position, pos_before.1);
    }

    #[test]
    fn warm_start_seeds_velocity_from_cached_impulse() {
        let mut a = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(0.0, 0.0), 1.0);
        let mut b = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(1.9, 0.0), 1.0);
        a.velocity = Vec2::ZERO;
        b.velocity = Vec2::ZERO;
        let mut bodies = vec![a, b];
        let mut m = Manifold::new(0, 1, Vec2::new(1.0, 0.0), 0.1, vec![Vec2::new(0.95, 0.0)]);
        m.normal_impulse[0] = 1.0; // impulsion "mémorisée" d'un pas précédent

        warm_start(&mut bodies, &[m]);

        // a est poussé selon -normal, b selon +normal.
        assert!(bodies[0].velocity.x < 0.0);
        assert!(bodies[1].velocity.x > 0.0);
    }
}
