//! Formes convexes supportées : cercle et polygone (un rectangle n'est qu'un
//! polygone à 4 sommets — ça permet de traiter boîte-boîte, boîte-cercle et
//! polygone-polygone avec les deux seules routines de collision
//! cercle-cercle / cercle-polygone / polygone-polygone, plutôt que de dupliquer
//! un cas spécial "AABB" qui de toute façon cesserait d'être axis-aligned dès
//! qu'un corps rigide tourne.

use crate::vec2::Vec2;

#[derive(Debug, Clone)]
pub enum Shape {
    Circle {
        radius: f64,
    },
    /// Sommets en espace local, ordre anti-horaire (CCW), polygone convexe.
    Polygon {
        vertices: Vec<Vec2>,
    },
}

impl Shape {
    pub fn circle(radius: f64) -> Self {
        assert!(radius > 0.0, "le rayon doit être positif");
        Shape::Circle { radius }
    }

    /// Rectangle centré à l'origine locale, largeur/hauteur pleines (pas des
    /// demi-dimensions) — plus lisible côté appelant : `Shape::rectangle(2.0, 1.0)`
    /// donne une boîte de 2×1.
    pub fn rectangle(width: f64, height: f64) -> Self {
        assert!(width > 0.0 && height > 0.0, "dimensions positives requises");
        let hw = width / 2.0;
        let hh = height / 2.0;
        Shape::Polygon {
            vertices: vec![
                Vec2::new(-hw, -hh),
                Vec2::new(hw, -hh),
                Vec2::new(hw, hh),
                Vec2::new(-hw, hh),
            ],
        }
    }

    /// Polygone convexe arbitraire ; les sommets doivent être en ordre CCW.
    /// Non vérifié ici (coûteux à valider en toute généralité) — voir
    /// `is_convex_ccw` dans les tests pour un exemple de garde-fou possible.
    pub fn polygon(vertices: Vec<Vec2>) -> Self {
        assert!(vertices.len() >= 3, "un polygone a au moins 3 sommets");
        Shape::Polygon { vertices }
    }

    /// Normales sortantes de chaque arête (espace local), une par arête,
    /// dans le même ordre que les sommets. `None` pour un cercle (pas
    /// d'arêtes).
    pub fn local_normals(&self) -> Option<Vec<Vec2>> {
        match self {
            Shape::Circle { .. } => None,
            Shape::Polygon { vertices } => {
                let n = vertices.len();
                Some(
                    (0..n)
                        .map(|i| {
                            let a = vertices[i];
                            let b = vertices[(i + 1) % n];
                            // Arête CCW -> normale sortante = perp() tournée de -90°,
                            // i.e. (edge.y, -edge.x), normalisée.
                            let edge = b - a;
                            Vec2::new(edge.y, -edge.x).normalized()
                        })
                        .collect(),
                )
            }
        }
    }

    /// Aire et moment d'inertie (par unité de masse, i.e. pour une masse
    /// unitaire) autour du centre de masse local. Utilisé par `RigidBody`
    /// pour dériver la vraie masse et le vrai moment d'inertie une fois la
    /// densité (ou la masse totale voulue) connue.
    pub fn unit_mass_area_and_inertia(&self) -> (f64, f64) {
        match self {
            // Disque plein : aire = πr², I/m = r²/2 (moment d'inertie
            // standard d'un disque homogène autour de son centre).
            Shape::Circle { radius } => (
                std::f64::consts::PI * radius * radius,
                0.5 * radius * radius,
            ),
            Shape::Polygon { vertices } => polygon_area_and_unit_inertia(vertices),
        }
    }
}

/// Aire signée (shoelace formula) et I/m d'un polygone convexe homogène
/// autour de son propre centre de masse (le polygone est supposé déjà
/// centré, ce qui est le cas de `Shape::rectangle` ; pour un polygone
/// arbitraire construit à la main, centrer les sommets sur le centroïde
/// avant de les donner à `Shape::polygon` si on veut une rotation physique
/// correcte autour du centre de masse).
///
/// Formule standard (triangulation en éventail depuis l'origine locale) :
/// voir par ex. la dérivation de Box2D `b2PolygonShape::ComputeMass`.
fn polygon_area_and_unit_inertia(vertices: &[Vec2]) -> (f64, f64) {
    let n = vertices.len();
    let mut area = 0.0;
    let mut inertia_numerator = 0.0;

    for i in 0..n {
        let p1 = vertices[i];
        let p2 = vertices[(i + 1) % n];
        let cross = p1.cross(p2);
        let triangle_area = 0.5 * cross;
        area += triangle_area;

        // Contribution au moment d'inertie du triangle (origine, p1, p2)
        // autour de l'origine, pour une densité surfacique unitaire.
        let intx2 = p1.x * p1.x + p1.x * p2.x + p2.x * p2.x;
        let inty2 = p1.y * p1.y + p1.y * p2.y + p2.y * p2.y;
        inertia_numerator += cross * (intx2 + inty2);
    }

    let area = area.abs();
    // Formule de triangulation en éventail (cf. Box2D `b2PolygonShape::ComputeMass`) :
    // `inertia_numerator` accumulé ci-dessus vaut 12× le moment d'inertie à densité
    // unitaire (I = inertia_numerator / 12). Pour obtenir I/m (indépendant de la
    // densité), on divise ensuite par la masse à densité unitaire, qui vaut `area`.
    let unit_inertia = inertia_numerator / (12.0 * area);
    // La formule ci-dessus donne I autour de l'origine locale des sommets ;
    // c'est le centre de masse tant que les sommets sont déjà centrés
    // (vrai pour `rectangle`).
    (area, unit_inertia.abs())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() < tol, "{a} != {b} (tol {tol})");
    }

    #[test]
    fn circle_area_and_inertia_match_formula() {
        let (area, unit_i) = Shape::circle(2.0).unit_mass_area_and_inertia();
        approx(area, std::f64::consts::PI * 4.0, 1e-9);
        approx(unit_i, 2.0, 1e-9); // r²/2 = 4/2 = 2
    }

    #[test]
    fn rectangle_area_matches_width_times_height() {
        let (area, _) = Shape::rectangle(4.0, 3.0).unit_mass_area_and_inertia();
        approx(area, 12.0, 1e-9);
    }

    #[test]
    fn rectangle_unit_inertia_matches_known_formula() {
        // I/m d'un rectangle homogène (w×h) autour de son centre :
        // (w² + h²) / 12 — formule standard de mécanique.
        let (w, h) = (4.0, 3.0);
        let (_, unit_i) = Shape::rectangle(w, h).unit_mass_area_and_inertia();
        approx(unit_i, (w * w + h * h) / 12.0, 1e-6);
    }

    #[test]
    fn rectangle_normals_point_outward_and_are_unit_length() {
        let normals = Shape::rectangle(2.0, 2.0).local_normals().unwrap();
        assert_eq!(normals.len(), 4);
        for n in &normals {
            approx(n.length(), 1.0, 1e-9);
        }
        // La normale de l'arête du bas (de (-1,-1) à (1,-1)) doit pointer vers -y.
        approx(normals[0].x, 0.0, 1e-9);
        approx(normals[0].y, -1.0, 1e-9);
    }

    #[test]
    fn circle_has_no_local_normals() {
        assert!(Shape::circle(1.0).local_normals().is_none());
    }
}
