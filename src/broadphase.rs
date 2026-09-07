//! Broad-phase par balayage et découpage (*sweep and prune*) sur l'axe X :
//! on trie les AABB des corps par leur borne minimale en X, puis on balaie
//! la liste en ne comparant que les corps dont les intervalles X se
//! chevauchent — évite le test O(n²) de tous les corps entre eux dès que la
//! scène en contient beaucoup et qu'ils sont dispersés spatialement.
//!
//! Un corps statique contre un autre corps statique est toujours ignoré
//! (deux murs ne peuvent pas entrer en collision l'un avec l'autre, et ça
//! économise du travail dans les scènes avec beaucoup d'obstacles fixes).

use crate::rigidbody::RigidBody;

/// Marge appliquée à chaque AABB avant le test de chevauchement — une paire
/// de corps qui s'approchent mais ne se touchent pas encore sur ce pas de
/// temps peut se toucher au pas suivant ; la marge évite de la manquer si
/// elle traverse la frontière entre deux appels à `candidate_pairs`.
const AABB_MARGIN: f64 = 0.05;

pub fn candidate_pairs(bodies: &[RigidBody]) -> Vec<(usize, usize)> {
    let mut entries: Vec<(usize, f64, f64, f64, f64)> = bodies
        .iter()
        .enumerate()
        .map(|(i, b)| {
            let aabb = b.world_aabb().expanded(AABB_MARGIN);
            (i, aabb.min.x, aabb.max.x, aabb.min.y, aabb.max.y)
        })
        .collect();

    entries.sort_by(|a, b| a.1.partial_cmp(&b.1).expect("bornes AABB non-NaN"));

    let mut pairs = Vec::new();
    for i in 0..entries.len() {
        let (idx_i, _min_x_i, max_x_i, min_y_i, max_y_i) = entries[i];
        for entry_j in entries.iter().skip(i + 1) {
            let (idx_j, min_x_j, _max_x_j, min_y_j, max_y_j) = *entry_j;
            if min_x_j > max_x_i {
                // Trié par min_x : plus aucun candidat possible au-delà.
                break;
            }
            if bodies[idx_i].is_static() && bodies[idx_j].is_static() {
                continue;
            }
            let y_overlap = min_y_i <= max_y_j && max_y_i >= min_y_j;
            if y_overlap {
                pairs.push((idx_i.min(idx_j), idx_i.max(idx_j)));
            }
        }
    }
    pairs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shape::Shape;
    use crate::vec2::Vec2;

    #[test]
    fn far_apart_bodies_produce_no_pairs() {
        let bodies = vec![
            RigidBody::dynamic(Shape::circle(1.0), Vec2::new(0.0, 0.0), 1.0),
            RigidBody::dynamic(Shape::circle(1.0), Vec2::new(100.0, 100.0), 1.0),
        ];
        assert!(candidate_pairs(&bodies).is_empty());
    }

    #[test]
    fn overlapping_aabbs_produce_a_pair() {
        let bodies = vec![
            RigidBody::dynamic(Shape::circle(1.0), Vec2::new(0.0, 0.0), 1.0),
            RigidBody::dynamic(Shape::circle(1.0), Vec2::new(1.0, 0.0), 1.0),
        ];
        let pairs = candidate_pairs(&bodies);
        assert_eq!(pairs, vec![(0, 1)]);
    }

    #[test]
    fn two_static_bodies_never_pair_even_if_overlapping() {
        let bodies = vec![
            RigidBody::static_body(Shape::circle(1.0), Vec2::new(0.0, 0.0)),
            RigidBody::static_body(Shape::circle(1.0), Vec2::new(0.5, 0.0)),
        ];
        assert!(candidate_pairs(&bodies).is_empty());
    }

    #[test]
    fn three_bodies_only_adjacent_ones_pair() {
        // Rayon 0.6 -> AABB [-0.65,0.65] et [0.35,1.65] pour les corps 0 et 1
        // (chevauchement réel), corps 2 loin à x=10 (aucun chevauchement).
        let bodies = vec![
            RigidBody::dynamic(Shape::circle(0.6), Vec2::new(0.0, 0.0), 1.0),
            RigidBody::dynamic(Shape::circle(0.6), Vec2::new(1.0, 0.0), 1.0),
            RigidBody::dynamic(Shape::circle(0.6), Vec2::new(10.0, 0.0), 1.0),
        ];
        let pairs = candidate_pairs(&bodies);
        assert_eq!(pairs, vec![(0, 1)]);
    }
}
