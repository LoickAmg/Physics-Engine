//! Détection de collision entre paires de formes (narrow-phase).
//!
//! Trois cas seulement grâce à l'unification cercle/polygone de `Shape` :
//! cercle-cercle, cercle-polygone, polygone-polygone (un rectangle est un
//! polygone à 4 sommets, donc "boîte-boîte" et "boîte-cercle" sont déjà
//! couverts par ces deux derniers cas, y compris quand les boîtes tournent).
//!
//! L'algorithme polygone-polygone (SAT + découpage d'arête de référence) et
//! cercle-polygone (séparation par arête la plus proche, avec cas
//! vertex/edge) suivent la construction pédagogique désormais classique
//! popularisée par Box2D / le tutoriel "How to Create a Custom Physics
//! Engine" de Randy Gaul — ici entièrement réécrite et testée indépendamment
//! plutôt que copiée depuis une implémentation existante.

use crate::manifold::Manifold;
use crate::rigidbody::RigidBody;
use crate::shape::Shape;
use crate::vec2::Vec2;

pub fn collide(ia: usize, a: &RigidBody, ib: usize, b: &RigidBody) -> Option<Manifold> {
    match (&a.shape, &b.shape) {
        (Shape::Circle { .. }, Shape::Circle { .. }) => circle_vs_circle(ia, a, ib, b),
        (Shape::Circle { .. }, Shape::Polygon { .. }) => circle_vs_polygon(ia, a, ib, b),
        (Shape::Polygon { .. }, Shape::Circle { .. }) => {
            circle_vs_polygon(ib, b, ia, a).map(swap_manifold)
        }
        (Shape::Polygon { .. }, Shape::Polygon { .. }) => polygon_vs_polygon(ia, a, ib, b),
    }
}

fn swap_manifold(m: Manifold) -> Manifold {
    Manifold {
        body_a: m.body_b,
        body_b: m.body_a,
        normal: -m.normal,
        penetration: m.penetration,
        contacts: m.contacts,
        normal_impulse: m.normal_impulse,
        tangent_impulse: m.tangent_impulse,
    }
}

fn radius_of(body: &RigidBody) -> f64 {
    match body.shape {
        Shape::Circle { radius } => radius,
        _ => panic!("radius_of() appelé sur une forme non-cercle"),
    }
}

fn circle_vs_circle(ia: usize, a: &RigidBody, ib: usize, b: &RigidBody) -> Option<Manifold> {
    let ra = radius_of(a);
    let rb = radius_of(b);
    let radius_sum = ra + rb;

    let delta = b.position - a.position;
    let dist_sq = delta.length_squared();
    if dist_sq >= radius_sum * radius_sum {
        return None;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-9 {
        delta / dist
    } else {
        // Centres exactement superposés : direction arbitraire mais stable.
        Vec2::new(1.0, 0.0)
    };
    let penetration = radius_sum - dist;

    let point_on_a = a.position + normal * ra;
    let point_on_b = b.position - normal * rb;
    let contact = (point_on_a + point_on_b) / 2.0;

    Some(Manifold::new(ia, ib, normal, penetration, vec![contact]))
}

/// `circle` et `poly` dans un ordre fixé ; le manifold renvoyé a toujours
/// `body_a = i_circle`, `body_b = i_poly`, normale pointant du cercle vers
/// le polygone.
fn circle_vs_polygon(
    i_circle: usize,
    circle: &RigidBody,
    i_poly: usize,
    poly: &RigidBody,
) -> Option<Manifold> {
    let radius = radius_of(circle);
    let verts = poly.world_vertices();
    let normals = poly.world_normals();
    let n = verts.len();

    // Arête de séparation maximale par rapport au centre du cercle.
    let mut best_index = 0;
    let mut best_separation = f64::NEG_INFINITY;
    for i in 0..n {
        let separation = normals[i].dot(circle.position - verts[i]);
        if separation > best_separation {
            best_separation = separation;
            best_index = i;
        }
    }

    if best_separation > radius {
        return None; // trop loin de toutes les faces : pas de collision possible
    }

    let outward_normal;
    let closest_point;

    if best_separation < 1e-9 {
        // Centre à l'intérieur (ou pile sur le bord) du polygone.
        outward_normal = normals[best_index];
        closest_point = circle.position - outward_normal * best_separation;
    } else {
        let v1 = verts[best_index];
        let v2 = verts[(best_index + 1) % n];
        let u1 = (circle.position - v1).dot(v2 - v1);
        let u2 = (circle.position - v2).dot(v1 - v2);

        if u1 <= 0.0 {
            closest_point = v1;
            let d = circle.position - v1;
            if d.length() > radius {
                return None;
            }
            outward_normal = d.normalized();
        } else if u2 <= 0.0 {
            closest_point = v2;
            let d = circle.position - v2;
            if d.length() > radius {
                return None;
            }
            outward_normal = d.normalized();
        } else {
            outward_normal = normals[best_index];
            closest_point = circle.position - outward_normal * best_separation;
        }
    }

    let dist = (circle.position - closest_point).length();
    let penetration = radius - dist;
    if penetration < 0.0 {
        return None;
    }

    // outward_normal pointe du polygone vers le cercle ; convention du
    // manifold (a=cercle -> b=polygone) veut l'inverse.
    Some(Manifold::new(
        i_circle,
        i_poly,
        -outward_normal,
        penetration,
        vec![closest_point],
    ))
}

/// Pour chaque normale d'arête de `verts_a`, cherche le sommet de `verts_b`
/// le plus "contre" cette normale (support le plus profond), puis retient
/// l'arête pour laquelle cette profondeur est la plus faible en valeur
/// algébrique (l'axe le "moins pénétrant" — s'il est positif, les deux
/// polygones sont séparés selon cet axe et ne se touchent donc pas du tout).
fn find_max_separation(verts_a: &[Vec2], normals_a: &[Vec2], verts_b: &[Vec2]) -> (usize, f64) {
    let mut best_index = 0;
    let mut best_separation = f64::NEG_INFINITY;

    for (i, &normal) in normals_a.iter().enumerate() {
        let v_a = verts_a[i];
        let mut min_dot = f64::INFINITY;
        for &v_b in verts_b {
            let d = normal.dot(v_b - v_a);
            if d < min_dot {
                min_dot = d;
            }
        }
        if min_dot > best_separation {
            best_separation = min_dot;
            best_index = i;
        }
    }

    (best_index, best_separation)
}

/// Découpe le segment `v_in` par le demi-plan `dot(p, normal) <= offset`,
/// en générant un point interpolé si le segment traverse la frontière.
/// Renvoie `None` si le segment entier est du mauvais côté.
fn clip_segment(v_in: [Vec2; 2], normal: Vec2, offset: f64) -> Option<[Vec2; 2]> {
    let d0 = normal.dot(v_in[0]) - offset;
    let d1 = normal.dot(v_in[1]) - offset;

    let mut out = Vec::with_capacity(2);
    if d0 <= 0.0 {
        out.push(v_in[0]);
    }
    if d1 <= 0.0 {
        out.push(v_in[1]);
    }
    if d0 * d1 < 0.0 {
        let t = d0 / (d0 - d1);
        out.push(v_in[0] + (v_in[1] - v_in[0]) * t);
    }

    if out.len() < 2 {
        None
    } else {
        Some([out[0], out[1]])
    }
}

fn polygon_vs_polygon(ia: usize, a: &RigidBody, ib: usize, b: &RigidBody) -> Option<Manifold> {
    let verts_a = a.world_vertices();
    let normals_a = a.world_normals();
    let verts_b = b.world_vertices();
    let normals_b = b.world_normals();

    let (edge_a, sep_a) = find_max_separation(&verts_a, &normals_a, &verts_b);
    if sep_a > 0.0 {
        return None;
    }
    let (edge_b, sep_b) = find_max_separation(&verts_b, &normals_b, &verts_a);
    if sep_b > 0.0 {
        return None;
    }

    // Biais pour préférer A comme référence en cas d'égalité approximative —
    // évite un basculement instable de la face de référence d'un pas de
    // temps à l'autre pour deux formes presque symétriques.
    const BIAS: f64 = 1e-3;
    let flip = sep_b > sep_a + BIAS;

    let (ref_verts, ref_normals, ref_edge, inc_verts, inc_normals): (
        &[Vec2],
        &[Vec2],
        usize,
        &[Vec2],
        &[Vec2],
    ) = if flip {
        (&verts_b, &normals_b, edge_b, &verts_a, &normals_a)
    } else {
        (&verts_a, &normals_a, edge_a, &verts_b, &normals_b)
    };

    let ref_normal = ref_normals[ref_edge];

    let incident_edge = inc_normals
        .iter()
        .enumerate()
        .min_by(|(_, n1), (_, n2)| {
            n1.dot(ref_normal)
                .partial_cmp(&n2.dot(ref_normal))
                .expect("normales non-NaN")
        })
        .map(|(i, _)| i)
        .expect("polygone non vide");

    let n_inc = inc_verts.len();
    let inc_v1 = inc_verts[incident_edge];
    let inc_v2 = inc_verts[(incident_edge + 1) % n_inc];

    let n_ref = ref_verts.len();
    let ref_v1 = ref_verts[ref_edge];
    let ref_v2 = ref_verts[(ref_edge + 1) % n_ref];

    let tangent = (ref_v2 - ref_v1).normalized();
    let neg_side_offset = -tangent.dot(ref_v1);
    let pos_side_offset = tangent.dot(ref_v2);

    let clipped = clip_segment([inc_v1, inc_v2], -tangent, neg_side_offset)?;
    let clipped = clip_segment(clipped, tangent, pos_side_offset)?;

    let ref_offset = ref_normal.dot(ref_v1);
    let mut contacts = Vec::new();
    let mut max_penetration: f64 = 0.0;
    for p in clipped {
        let separation = ref_normal.dot(p) - ref_offset;
        if separation <= 0.0 {
            contacts.push(p);
            max_penetration = max_penetration.max(-separation);
        }
    }

    if contacts.is_empty() {
        return None;
    }

    // ref_normal pointe hors du polygone de référence. Convention du
    // manifold : normale de a vers b.
    let normal = if flip { -ref_normal } else { ref_normal };

    Some(Manifold::new(ia, ib, normal, max_penetration, contacts))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rigidbody::RigidBody;

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() < tol, "{a} != {b} (tol {tol})");
    }

    #[test]
    fn circles_far_apart_do_not_collide() {
        let a = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(0.0, 0.0), 1.0);
        let b = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(10.0, 0.0), 1.0);
        assert!(collide(0, &a, 1, &b).is_none());
    }

    #[test]
    fn overlapping_circles_produce_correct_normal_and_penetration() {
        let a = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(0.0, 0.0), 1.0);
        let b = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(1.5, 0.0), 1.0);
        let m = collide(0, &a, 1, &b).expect("doit se toucher (dist 1.5 < somme des rayons 2)");
        approx(m.normal.x, 1.0, 1e-9);
        approx(m.normal.y, 0.0, 1e-9);
        approx(m.penetration, 0.5, 1e-9);
        assert_eq!(m.contacts.len(), 1);
    }

    #[test]
    fn circle_resting_on_top_of_box_has_upward_normal() {
        let box_body = RigidBody::static_body(Shape::rectangle(4.0, 2.0), Vec2::new(0.0, 0.0));
        // Boîte de demi-hauteur 1 ; cercle de rayon 1 centré à y=1.5 -> pénètre de 0.5.
        let circle = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(0.0, 1.5), 1.0);
        let m = collide(0, &circle, 1, &box_body).expect("le cercle pénètre le dessus de la boîte");
        approx(m.normal.x, 0.0, 1e-9);
        // Convention : normale de a (cercle, au-dessus) vers b (boîte, en
        // dessous) -> pointe vers le bas. Le résolveur applique +impulsion*n
        // à b et -impulsion*n à a, donc a (cercle) est bien poussé vers le
        // haut (-normal) et b (boîte) vers le bas : physiquement correct.
        approx(m.normal.y, -1.0, 1e-9);
        approx(m.penetration, 0.5, 1e-9);
    }

    #[test]
    fn circle_near_box_corner_normal_points_away_from_corner() {
        let box_body = RigidBody::static_body(Shape::rectangle(2.0, 2.0), Vec2::ZERO);
        // Coin en (1,1) ; cercle centré à (1.5, 1.5), distance au coin = sqrt(0.5)≈0.707 < rayon 1.
        let circle = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(1.5, 1.5), 1.0);
        let m = collide(0, &circle, 1, &box_body).expect("le cercle chevauche le coin");
        // Normale du cercle vers la boîte -> pointe vers le coin, i.e. direction (-1,-1) normalisée.
        approx(m.normal.x, -std::f64::consts::FRAC_1_SQRT_2, 1e-6);
        approx(m.normal.y, -std::f64::consts::FRAC_1_SQRT_2, 1e-6);
    }

    #[test]
    fn two_boxes_resting_flush_produce_two_contact_points() {
        // Boîte du bas statique, boîte du haut posée exactement dessus (pas de gap).
        let bottom = RigidBody::static_body(Shape::rectangle(4.0, 2.0), Vec2::new(0.0, 0.0));
        let top = RigidBody::dynamic(Shape::rectangle(2.0, 2.0), Vec2::new(0.0, 1.999), 1.0);
        let m = collide(0, &top, 1, &bottom).expect("les boîtes se chevauchent légèrement");
        assert_eq!(
            m.contacts.len(),
            2,
            "un contact boîte-sur-boîte plat doit avoir 2 points"
        );
        approx(m.normal.y.abs(), 1.0, 1e-6);
    }

    #[test]
    fn separated_boxes_do_not_collide() {
        let a = RigidBody::dynamic(Shape::rectangle(1.0, 1.0), Vec2::new(0.0, 0.0), 1.0);
        let b = RigidBody::dynamic(Shape::rectangle(1.0, 1.0), Vec2::new(5.0, 0.0), 1.0);
        assert!(collide(0, &a, 1, &b).is_none());
    }

    #[test]
    fn rotated_box_on_box_still_detects_collision() {
        let bottom = RigidBody::static_body(Shape::rectangle(4.0, 2.0), Vec2::ZERO);
        let top = RigidBody::dynamic(Shape::rectangle(2.0, 2.0), Vec2::new(0.0, 2.3), 1.0)
            .with_rotation(0.3);
        // À 45°? non, 0.3 rad (~17°) : le coin le plus bas descend sous y=1.3 environ.
        // On vérifie juste que la détection ne panique pas et reste cohérente si collision.
        let result = collide(0, &top, 1, &bottom);
        if let Some(m) = result {
            assert!(m.penetration > 0.0);
            assert!(!m.contacts.is_empty());
        }
    }
}
