//! Vecteur 2D minimal — aucune dépendance externe.
//!
//! Volontairement réécrit à la main (plutôt que d'utiliser `glam`/`nalgebra`) :
//! ce projet est un moteur physique pédagogique, l'algèbre vectorielle en fait
//! partie intégrante (cf. `neural-net-from-scratch` pour la même philosophie
//! côté calcul différentiel).

use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };

    pub fn new(x: f64, y: f64) -> Self {
        Vec2 { x, y }
    }

    /// Produit scalaire.
    pub fn dot(self, rhs: Vec2) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }

    /// Produit vectoriel 2D (scalaire — composante z du produit vectoriel 3D
    /// si on plonge les deux vecteurs dans le plan z=0). Utilisé pour le
    /// couple (torque) et pour déterminer l'orientation de trois points.
    pub fn cross(self, rhs: Vec2) -> f64 {
        self.x * rhs.y - self.y * rhs.x
    }

    /// Produit vectoriel d'un scalaire (vitesse angulaire) par un vecteur —
    /// donne la composante de vitesse linéaire induite par une rotation.
    /// Convention : `s.cross_scalar(v)` = rotation de `v` de 90° puis
    /// mise à l'échelle par `s`.
    pub fn scalar_cross(s: f64, v: Vec2) -> Vec2 {
        Vec2::new(-s * v.y, s * v.x)
    }

    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }

    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }

    /// Normalise le vecteur ; renvoie `Vec2::ZERO` si la longueur est nulle
    /// (évite une division par zéro silencieuse qui produirait NaN).
    pub fn normalized(self) -> Vec2 {
        let len = self.length();
        if len < f64::EPSILON {
            Vec2::ZERO
        } else {
            self / len
        }
    }

    /// Perpendiculaire (rotation de +90°).
    pub fn perp(self) -> Vec2 {
        Vec2::new(-self.y, self.x)
    }

    pub fn rotated(self, angle: f64) -> Vec2 {
        let (s, c) = angle.sin_cos();
        Vec2::new(self.x * c - self.y * s, self.x * s + self.y * c)
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl AddAssign for Vec2 {
    fn add_assign(&mut self, rhs: Vec2) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, rhs: Vec2) -> Vec2 {
        Vec2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl SubAssign for Vec2 {
    fn sub_assign(&mut self, rhs: Vec2) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2::new(-self.x, -self.y)
    }
}

impl Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, rhs: f64) -> Vec2 {
        Vec2::new(self.x * rhs, self.y * rhs)
    }
}

impl Div<f64> for Vec2 {
    type Output = Vec2;
    fn div(self, rhs: f64) -> Vec2 {
        Vec2::new(self.x / rhs, self.y / rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-9, "{a} != {b}");
    }

    #[test]
    fn dot_product_orthogonal_is_zero() {
        approx(Vec2::new(1.0, 0.0).dot(Vec2::new(0.0, 1.0)), 0.0);
    }

    #[test]
    fn cross_product_matches_determinant() {
        approx(Vec2::new(1.0, 0.0).cross(Vec2::new(0.0, 1.0)), 1.0);
        approx(Vec2::new(0.0, 1.0).cross(Vec2::new(1.0, 0.0)), -1.0);
    }

    #[test]
    fn normalized_has_unit_length() {
        let v = Vec2::new(3.0, 4.0).normalized();
        approx(v.length(), 1.0);
        approx(v.x, 0.6);
        approx(v.y, 0.8);
    }

    #[test]
    fn normalized_zero_vector_is_zero_not_nan() {
        let v = Vec2::ZERO.normalized();
        assert_eq!(v, Vec2::ZERO);
    }

    #[test]
    fn perp_is_90_degrees_counterclockwise() {
        let v = Vec2::new(1.0, 0.0).perp();
        approx(v.x, 0.0);
        approx(v.y, 1.0);
    }

    #[test]
    fn rotated_by_pi_over_2_matches_perp() {
        let v = Vec2::new(1.0, 0.0).rotated(std::f64::consts::FRAC_PI_2);
        approx(v.x, 0.0);
        approx(v.y, 1.0);
    }

    #[test]
    fn scalar_cross_matches_angular_velocity_convention() {
        // Un point à distance r du centre, avec vitesse angulaire w=1,
        // doit avoir une vitesse linéaire perpendiculaire à r, de même norme.
        let r = Vec2::new(2.0, 0.0);
        let v = Vec2::scalar_cross(1.0, r);
        approx(v.x, 0.0);
        approx(v.y, 2.0);
    }
}
