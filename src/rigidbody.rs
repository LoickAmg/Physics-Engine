use crate::shape::Shape;
use crate::vec2::Vec2;

/// Corps rigide 2D. La masse et l'inertie sont dérivées de la forme et de la
/// densité au moment de la construction plutôt que saisies à la main, pour
/// garantir leur cohérence physique (un cercle deux fois plus gros a
/// forcément une masse et une inertie plus grandes, pas des valeurs
/// arbitraires découplées de sa géométrie).
#[derive(Debug, Clone)]
pub struct RigidBody {
    pub shape: Shape,
    pub position: Vec2,
    pub rotation: f64,
    pub velocity: Vec2,
    pub angular_velocity: f64,

    mass: f64,
    inv_mass: f64,
    inertia: f64,
    inv_inertia: f64,

    pub restitution: f64,
    pub friction: f64,

    /// Force/couple accumulés pour le pas de temps courant, remis à zéro
    /// après intégration (voir `World::step`).
    force: Vec2,
    torque: f64,
}

impl RigidBody {
    /// Corps dynamique : masse et inertie dérivées de `shape` et `density`.
    pub fn dynamic(shape: Shape, position: Vec2, density: f64) -> Self {
        assert!(density > 0.0, "densité positive requise");
        let (area, unit_inertia) = shape.unit_mass_area_and_inertia();
        let mass = area * density;
        let inertia = unit_inertia * mass;
        RigidBody {
            shape,
            position,
            rotation: 0.0,
            velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            mass,
            inv_mass: 1.0 / mass,
            inertia,
            inv_inertia: if inertia > 0.0 { 1.0 / inertia } else { 0.0 },
            restitution: 0.3,
            friction: 0.3,
            force: Vec2::ZERO,
            torque: 0.0,
        }
    }

    /// Corps statique : masse et inertie infinies (inv_mass = inv_inertia =
    /// 0), ne bouge jamais quelle que soit la force/impulsion reçue. Sert de
    /// sol, mur, ou obstacle fixe.
    pub fn static_body(shape: Shape, position: Vec2) -> Self {
        RigidBody {
            shape,
            position,
            rotation: 0.0,
            velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            mass: f64::INFINITY,
            inv_mass: 0.0,
            inertia: f64::INFINITY,
            inv_inertia: 0.0,
            restitution: 0.3,
            friction: 0.5,
            force: Vec2::ZERO,
            torque: 0.0,
        }
    }

    pub fn with_restitution(mut self, restitution: f64) -> Self {
        self.restitution = restitution.clamp(0.0, 1.0);
        self
    }

    pub fn with_friction(mut self, friction: f64) -> Self {
        assert!(friction >= 0.0);
        self.friction = friction;
        self
    }

    pub fn with_velocity(mut self, velocity: Vec2) -> Self {
        self.velocity = velocity;
        self
    }

    pub fn with_rotation(mut self, rotation: f64) -> Self {
        self.rotation = rotation;
        self
    }

    pub fn with_angular_velocity(mut self, angular_velocity: f64) -> Self {
        self.angular_velocity = angular_velocity;
        self
    }

    pub fn is_static(&self) -> bool {
        self.inv_mass == 0.0
    }

    pub fn mass(&self) -> f64 {
        self.mass
    }

    pub fn inv_mass(&self) -> f64 {
        self.inv_mass
    }

    pub fn inertia(&self) -> f64 {
        self.inertia
    }

    pub fn inv_inertia(&self) -> f64 {
        self.inv_inertia
    }

    /// Applique une force au centre de masse (pas de couple induit).
    pub fn apply_force(&mut self, force: Vec2) {
        self.force += force;
    }

    /// Applique une force en un point donné en espace monde — induit un
    /// couple si le point n'est pas le centre de masse (bras de levier `r`).
    pub fn apply_force_at_point(&mut self, force: Vec2, world_point: Vec2) {
        self.force += force;
        let r = world_point - self.position;
        self.torque += r.cross(force);
    }

    /// Impulsion instantanée (change la vitesse directement, pas
    /// l'accélération) — utilisée par le résolveur de collisions, mais
    /// aussi utilisable pour une explosion, un tir, etc.
    pub fn apply_impulse_at_point(&mut self, impulse: Vec2, world_point: Vec2) {
        if self.is_static() {
            return;
        }
        self.velocity += impulse * self.inv_mass;
        let r = world_point - self.position;
        self.angular_velocity += self.inv_inertia * r.cross(impulse);
    }

    pub(crate) fn take_accumulated_force_and_torque(&mut self) -> (Vec2, f64) {
        let ft = (self.force, self.torque);
        self.force = Vec2::ZERO;
        self.torque = 0.0;
        ft
    }

    /// Vitesse du point matériel situé en `world_point` (translation + terme
    /// de rotation `w × r`) — c'est la vitesse qui doit s'annuler entre deux
    /// corps en contact, pas la vitesse du centre de masse.
    pub fn velocity_at_point(&self, world_point: Vec2) -> Vec2 {
        let r = world_point - self.position;
        self.velocity + Vec2::scalar_cross(self.angular_velocity, r)
    }

    /// Sommets du polygone en espace monde (position + rotation appliquées).
    /// Panique si appelé sur un cercle — utiliser `matches!` ou le
    /// pattern-matching sur `self.shape` avant d'appeler cette méthode.
    pub fn world_vertices(&self) -> Vec<Vec2> {
        match &self.shape {
            Shape::Circle { .. } => panic!("world_vertices() appelé sur un cercle"),
            Shape::Polygon { vertices } => vertices
                .iter()
                .map(|v| self.position + v.rotated(self.rotation))
                .collect(),
        }
    }

    pub fn world_normals(&self) -> Vec<Vec2> {
        self.shape
            .local_normals()
            .expect("world_normals() appelé sur un cercle")
            .into_iter()
            .map(|n| n.rotated(self.rotation))
            .collect()
    }

    /// AABB englobante en espace monde — utilisée par la broad-phase.
    pub fn world_aabb(&self) -> Aabb {
        match &self.shape {
            Shape::Circle { radius } => Aabb {
                min: self.position - Vec2::new(*radius, *radius),
                max: self.position + Vec2::new(*radius, *radius),
            },
            Shape::Polygon { .. } => {
                let verts = self.world_vertices();
                let mut min = verts[0];
                let mut max = verts[0];
                for v in &verts[1..] {
                    min.x = min.x.min(v.x);
                    min.y = min.y.min(v.y);
                    max.x = max.x.max(v.x);
                    max.y = max.y.max(v.y);
                }
                Aabb { min, max }
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}

impl Aabb {
    pub fn overlaps(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    /// Étend l'AABB d'une marge fixe dans toutes les directions — utile pour
    /// que la broad-phase reste valide même si un corps rapide traverse
    /// presque tout son AABB en un seul pas de temps (marge de sécurité, pas
    /// une vraie détection de tunneling continue).
    pub fn expanded(&self, margin: f64) -> Aabb {
        Aabb {
            min: self.min - Vec2::new(margin, margin),
            max: self.max + Vec2::new(margin, margin),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn static_body_has_zero_inverse_mass_and_inertia() {
        let b = RigidBody::static_body(Shape::rectangle(1.0, 1.0), Vec2::ZERO);
        assert!(b.is_static());
        assert_eq!(b.inv_mass(), 0.0);
        assert_eq!(b.inv_inertia(), 0.0);
    }

    #[test]
    fn dynamic_body_mass_scales_with_density() {
        let a = RigidBody::dynamic(Shape::circle(1.0), Vec2::ZERO, 1.0);
        let b = RigidBody::dynamic(Shape::circle(1.0), Vec2::ZERO, 2.0);
        assert!((b.mass() - 2.0 * a.mass()).abs() < 1e-9);
    }

    #[test]
    fn apply_impulse_at_center_of_mass_changes_only_linear_velocity() {
        let mut b = RigidBody::dynamic(Shape::circle(1.0), Vec2::new(5.0, 5.0), 1.0);
        b.apply_impulse_at_point(Vec2::new(1.0, 0.0), b.position);
        assert!(b.velocity.x > 0.0);
        assert_eq!(b.angular_velocity, 0.0);
    }

    #[test]
    fn apply_impulse_off_center_induces_rotation() {
        let mut b = RigidBody::dynamic(Shape::rectangle(2.0, 2.0), Vec2::ZERO, 1.0);
        let point = b.position + Vec2::new(0.0, 1.0); // bord supérieur
        b.apply_impulse_at_point(Vec2::new(1.0, 0.0), point);
        assert_ne!(b.angular_velocity, 0.0);
    }

    #[test]
    fn static_body_ignores_impulses() {
        let mut b = RigidBody::static_body(Shape::circle(1.0), Vec2::ZERO);
        b.apply_impulse_at_point(Vec2::new(100.0, 100.0), Vec2::new(1.0, 0.0));
        assert_eq!(b.velocity, Vec2::ZERO);
        assert_eq!(b.angular_velocity, 0.0);
    }

    #[test]
    fn velocity_at_point_includes_rotational_term() {
        let mut b = RigidBody::dynamic(Shape::circle(1.0), Vec2::ZERO, 1.0);
        b.angular_velocity = 1.0;
        let v = b.velocity_at_point(Vec2::new(1.0, 0.0));
        // w × r avec r=(1,0), w=1 -> (0,1)
        assert!((v.x - 0.0).abs() < 1e-9);
        assert!((v.y - 1.0).abs() < 1e-9);
    }

    #[test]
    fn aabb_overlap_detects_separation() {
        let a = Aabb {
            min: Vec2::new(0.0, 0.0),
            max: Vec2::new(1.0, 1.0),
        };
        let b = Aabb {
            min: Vec2::new(2.0, 2.0),
            max: Vec2::new(3.0, 3.0),
        };
        assert!(!a.overlaps(&b));
        let c = Aabb {
            min: Vec2::new(0.5, 0.5),
            max: Vec2::new(1.5, 1.5),
        };
        assert!(a.overlaps(&c));
    }
}
