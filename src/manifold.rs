use crate::vec2::Vec2;

/// Résultat d'une détection de collision entre deux corps (indexés `a` et
/// `b` dans `World::bodies`). La normale pointe toujours de `a` vers `b`.
/// Jusqu'à 2 points de contact (nécessaire pour qu'une boîte posée sur un
/// sol ne bascule pas — un seul point de contact ne contraint qu'une
/// translation, pas la rotation induite par un léger déséquilibre).
#[derive(Debug, Clone)]
pub struct Manifold {
    pub body_a: usize,
    pub body_b: usize,
    pub normal: Vec2,
    pub penetration: f64,
    pub contacts: Vec<Vec2>,
    /// Impulsions accumulées (normale et tangentielle) par point de
    /// contact, pendant la résolution du pas courant — et, via
    /// `World::step`, reportées d'un pas à l'autre pour les contacts
    /// persistants (*warm starting* : réutiliser la solution précédente
    /// comme point de départ accélère la convergence et surtout stabilise
    /// fortement les piles d'objets au repos — sans ça, chaque pas
    /// recalcule la solution "à froid" et le bruit numérique résultant finit
    /// par faire basculer une pile pourtant à l'équilibre).
    pub normal_impulse: Vec<f64>,
    pub tangent_impulse: Vec<f64>,
}

impl Manifold {
    pub fn new(
        body_a: usize,
        body_b: usize,
        normal: Vec2,
        penetration: f64,
        contacts: Vec<Vec2>,
    ) -> Self {
        let n = contacts.len();
        Manifold {
            body_a,
            body_b,
            normal,
            penetration,
            contacts,
            normal_impulse: vec![0.0; n],
            tangent_impulse: vec![0.0; n],
        }
    }
}
