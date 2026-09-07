# Physics Engine

Un moteur physique 2D de corps rigides écrit **from scratch** en Rust : pas
de `nalgebra`/`glam` pour l'algèbre vectorielle, pas de `rapier`/`nphysics`
pour la simulation. L'objectif n'est pas de rivaliser avec ces moteurs de
production, mais de comprendre — et prouver par les tests — comment un
moteur physique fonctionne réellement : détection de collision, résolution
par impulsions, friction, stabilité d'une pile d'objets au repos.

## Ce que fait (et ne fait pas) ce moteur

Corps rigides 2D (cercles et polygones convexes — un rectangle est un
polygone à 4 sommets), intégration semi-implicite d'Euler, détection de
collision par arête de séparation (SAT) avec découpage de manifold à deux
points de contact, résolution par impulsions séquentielles accumulées avec
*warm starting*, friction de Coulomb, correction positionnelle.

Ce que ce moteur ne fait **pas** : détection de collision continue (CCD —
un objet très rapide peut transpercer un obstacle fin en un seul pas de
temps), contraintes articulées (ressorts, charnières, moteurs), formes
concaves, multi-threading. Ce sont des extensions réelles mais hors du
périmètre pédagogique de ce projet.

## Pourquoi un rectangle est un polygone, pas un cas spécial

La tentation naturelle est de coder trois routines de collision séparées :
cercle-cercle, cercle-AABB, AABB-AABB. Le problème : dès qu'un corps rigide
tourne, une "AABB" (*axis-aligned bounding box*) cesse d'être axis-aligned —
ce n'est plus la bonne abstraction. Ce moteur ne connaît que deux formes,
`Circle` et `Polygon` (un rectangle est un polygone à 4 sommets construit
par `Shape::rectangle`), et donc seulement trois routines de collision :
cercle-cercle, cercle-polygone, polygone-polygone — cette dernière couvrant
boîte-boîte, boîte-polygone quelconque, et tout ça avec rotation, sans code
dupliqué.

## Détection de collision : SAT + découpage de manifold

Pour deux polygones, l'algorithme ([Separating Axis
Theorem](https://en.wikipedia.org/wiki/Hyperplane_separation_theorem))
teste chaque normale d'arête des deux polygones comme axe de séparation
candidat : s'il existe un axe le long duquel les deux formes ne se
chevauchent pas, elles ne sont pas en collision. Si aucun axe ne sépare, la
pénétration minimale identifie l'arête de référence.

Le point clé pour la **stabilité** : une boîte posée sur un sol a besoin de
**deux points de contact**, pas un seul — sinon rien n'empêche la boîte de
basculer autour de l'unique point retenu au moindre déséquilibre. Ce moteur
implémente le découpage classique (arête de référence / arête incidente,
puis découpage de l'arête incidente contre les deux plans latéraux de
l'arête de référence — popularisé par Box2D et le tutoriel *How to Create a
Custom Physics Engine* de Randy Gaul, ici entièrement réécrit et testé
indépendamment) qui produit ces deux points quand la géométrie le permet.

Le cas cercle-polygone traite séparément trois régions (centre à
l'intérieur du polygone, projection sur une arête, proximité d'un sommet) —
un piège classique est d'oublier le cas "sommet" et de ne détecter que les
collisions face-à-face, ratant les coins.

## Résolution : impulsions séquentielles accumulées, avec warm starting

Plutôt que résoudre un système d'équations exact pour tous les contacts
simultanément (coûteux, rarement nécessaire pour un pas de simulation), le
résolveur relâche chaque contact l'un après l'autre et répète plusieurs
fois (*sequential impulse solver*, la technique de Box2D/Chipmunk).

Deux pièges rencontrés en écrivant ce solveur, et pourquoi ils comptent :

- **Le biais de restitution doit être figé sur la vitesse initiale du pas,
  pas recalculé à chaque itération.** Une première version de ce moteur
  recalculait `-(1+e)·v_normal` à partir de la vitesse *courante* à chaque
  itération — et un contact déjà résolu (vitesse de séparation correcte)
  se faisait immédiatement défaire par l'itération suivante, qui le
  percevait comme "s'éloignant trop vite" et retirait l'impulsion. Résultat
  : le solveur oscillait entre impulsion pleine et impulsion nulle sans
  jamais converger, annulant purement et simplement la restitution après un
  nombre pair d'itérations. Le correctif standard : calculer le biais
  (`-restitution × vitesse_approche_initiale`) **une fois**, avant la
  boucle d'itérations, et faire converger la vitesse courante vers cette
  cible fixe plutôt que vers zéro.
- **Sans *warm starting*, une pile de boîtes identiques parfaitement
  alignées finit par basculer.** Physiquement, cette configuration est un
  équilibre instable : n'importe quel bruit numérique de l'ordre de 1e-12
  suffit à amorcer une légère rotation, qui s'auto-amplifie sous la
  gravité. Reporter d'un pas à l'autre les impulsions déjà résolues pour
  les contacts qui persistent (plutôt que de repartir de zéro à chaque pas)
  réduit ce bruit de façon décisive — c'est la pratique universelle des
  moteurs de production, pas une optimisation cosmétique. Vérifié
  concrètement par `stack_of_five_boxes_does_not_collapse` (5 boîtes
  identiques, aucun décalage initial — le cas le plus défavorable).

## Architecture

```
src/
├── vec2.rs        # Vecteur 2D, algèbre vectorielle (dot/cross/perp/rotate)
├── shape.rs        # Cercle et polygone convexe, aire + moment d'inertie
├── rigidbody.rs      # Corps rigide (masse/inertie dérivées de la forme), AABB
├── broadphase.rs       # Sweep & prune (tri + balayage sur l'axe X)
├── collision.rs          # Narrow-phase : cercle-cercle / cercle-polygone / SAT
├── manifold.rs             # Résultat de collision (normale, pénétration, contacts)
├── resolver.rs               # Impulsions séquentielles accumulées + warm starting
├── world.rs                    # Orchestre le pas de simulation complet
├── scenes.rs                     # 4 scènes de démo partagées CLI/visuel
└── bin/
    ├── cli.rs                      # Simulation headless (vérification rapide)
    └── pixels.rs                     # Rendu 2D interactif (winit + softbuffer)
```

## Utilisation

```bash
cargo test                                    # 48 tests (40 unitaires + 8 intégration)
cargo run --bin physics-cli -- stack 300      # scène headless : bounce/stack/billiard/ramp
cargo run --features pixels --bin physics-pixels   # rendu interactif
```

Contrôles du rendu interactif : `1`/`2`/`3`/`4` changent de scène, `R`
réinitialise, `P` met en pause, `+`/`-` accélèrent/ralentissent, `C`
affiche/masque les points de contact (utile pour *voir* ce que le
résolveur résout réellement), molette pour zoomer, clic-glisser pour
déplacer la vue.

### Les 4 scènes de démonstration

- **bounce** — une balle tombe et rebondit (restitution 0.7) sur un sol
  statique.
- **stack** — 5 boîtes identiques empilées sans aucun décalage initial
  (cas le plus défavorable pour la stabilité), doivent rester debout grâce
  au warm starting.
- **billiard** — collisions circulaires sans gravité ; la quantité de
  mouvement totale doit rester exactement constante (aucune force externe).
- **ramp** — une balle glisse et roule sur un plan incliné avec friction.

## Tests

```bash
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
cargo fmt --check
```

48 tests, tous verts. Les plus significatifs :

- `stack_of_five_boxes_does_not_collapse` — le test qui a motivé
  l'implémentation du warm starting (voir plus haut).
- `elastic_head_on_collision_conserves_momentum_and_energy_over_time` —
  collision élastique simulée sur 5 secondes, quantité de mouvement
  conservée exactement, énergie conservée à moins de 5% près (la
  correction positionnelle dissipe volontairement un peu d'énergie ; ce
  n'est pas un intégrateur symplectique exact comme dans `double-pendulum`).
- `projectile_without_collision_follows_semi_implicit_euler_trajectory` —
  compare la trajectoire à la solution **discrète exacte** de l'intégrateur
  (pas à la parabole continue) : Euler semi-implicite a un biais
  systématique connu de `0.5·a·dt·t`, ce n'est pas une imprécision du
  moteur mais une propriété de cet intégrateur (cf. `double-pendulum` pour
  la même discussion appliquée à Verlet symplectique).
- `fast_impact_never_tunnels_fully_through_the_floor` — documente
  explicitement la limite connue de ce moteur sans détection de collision
  continue : un enfoncement transitoire d'une fraction de pas est
  acceptable sur un impact très rapide, un tunneling complet ne l'est
  jamais.
- `rectangle_unit_inertia_matches_known_formula` — vérifie le moment
  d'inertie d'un rectangle homogène contre la formule connue `(w²+h²)/12`.

## Limites connues

- **Pas de détection de collision continue (CCD)** : un corps assez rapide
  peut s'enfoncer transitoirement dans un obstacle fin sur un seul pas de
  temps avant que la correction positionnelle ne le ressorte au pas
  suivant (voir `fast_impact_never_tunnels_fully_through_the_floor`).
- **Correspondance de contacts entre pas simplifiée** : le warm starting
  reporte les impulsions d'un pas à l'autre en faisant correspondre les
  manifolds par paire de corps + nombre de points de contact, plutôt que
  par un vrai identifiant de *feature* SAT (arête/sommet). Suffisant pour
  les scènes de ce projet, mais moins robuste qu'une correspondance par
  feature dans des scènes très dynamiques avec beaucoup de contacts
  simultanés changeants.
- **Formes convexes uniquement**, pas de formes concaves (qu'il faudrait
  décomposer en polygones convexes).
- **Aucune contrainte articulée** (ressort, charnière, moteur) — seul le
  contact non-pénétrant est modélisé.

## Licence

MIT — voir `LICENSE`.
