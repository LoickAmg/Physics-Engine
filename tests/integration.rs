//! Tests d'intégration en boîte noire : uniquement l'API publique de la
//! crate `physics`, comme un utilisateur externe l'utiliserait. Complète
//! les tests unitaires internes (`#[cfg(test)]` dans chaque module) qui
//! vérifient des détails d'implémentation.

use physics::{RigidBody, Shape, Vec2, World};

fn approx(a: f64, b: f64, tol: f64, msg: &str) {
    assert!((a - b).abs() < tol, "{msg}: {a} != {b} (tol {tol})");
}

#[test]
fn projectile_without_collision_follows_semi_implicit_euler_trajectory() {
    // Sans aucune collision, on compare à la solution DISCRÈTE exacte de
    // l'intégrateur d'Euler semi-implicite (vitesse mise à jour avant la
    // position), pas à la parabole continue : semi-implicite Euler est
    // seulement précis à l'ordre 1 en position pour une accélération
    // constante, avec un biais systématique connu de 0.5*a*dt*t (dérivable
    // en sommant explicitement la récurrence v_n=v0+a*n*dt,
    // x_n=x0+dt*sum(v_k)) — ce n'est pas une imprécision du moteur mais une
    // propriété connue de cet intégrateur (cf. double-pendulum pour la même
    // discussion appliquée à Verlet symplectique).
    let mut world = World::new(Vec2::new(0.0, -10.0));
    let idx = world.add_body(RigidBody::dynamic(
        Shape::circle(0.1),
        Vec2::new(0.0, 100.0),
        1.0,
    ));
    world.bodies[idx].velocity = Vec2::new(3.0, 5.0);

    let dt = 1.0 / 1000.0;
    let n = 2000;
    for _ in 0..n {
        world.step(dt);
    }
    let t = n as f64 * dt;

    let expected_x = 3.0 * t; // vitesse x constante (pas de force en x) : exact.
                              // Récurrence exacte de l'intégrateur : x_n = x0 + n*dt*v0y + a*dt²*n(n+1)/2.
    let expected_y = 100.0 + (n as f64) * dt * 5.0 + (-10.0) * dt * dt * (n * (n + 1)) as f64 / 2.0;

    approx(world.bodies[idx].position.x, expected_x, 1e-6, "position x");
    approx(world.bodies[idx].position.y, expected_y, 1e-6, "position y");
}

#[test]
fn ball_never_falls_through_the_floor() {
    // Hauteur de chute modérée : sans détection de collision continue (CCD),
    // un impact à très haute vitesse peut légitimement enfoncer le corps
    // d'une fraction de pas avant que la correction positionnelle ne le
    // ressorte au pas suivant (limitation connue et documentée de tout
    // moteur à impulsions discrètes sans CCD, Box2D inclus) — ce test vérifie
    // le cas d'usage normal, pas le cas limite extrême.
    let mut world = World::new(Vec2::new(0.0, -9.81));
    world.add_body(RigidBody::static_body(
        Shape::rectangle(20.0, 1.0),
        Vec2::new(0.0, -0.5),
    ));
    let ball = world.add_body(
        RigidBody::dynamic(Shape::circle(0.5), Vec2::new(0.0, 5.0), 1.0).with_restitution(0.4),
    );

    for _ in 0..600 {
        world.step(1.0 / 60.0);
        assert!(
            world.bodies[ball].position.y > 0.3,
            "la balle a traversé le sol à y={}",
            world.bodies[ball].position.y
        );
    }
}

#[test]
fn fast_impact_never_tunnels_fully_through_the_floor() {
    // Cas limite : chute de très haut (impact ~20 m/s dans un seul pas de
    // 1/60s). Sans CCD, un enfoncement transitoire d'une fraction de pas est
    // attendu et acceptable ; ce qui ne doit JAMAIS arriver, c'est un
    // tunneling complet à travers le sol (épaisseur 1.0, base à y=-1).
    let mut world = World::new(Vec2::new(0.0, -9.81));
    world.add_body(RigidBody::static_body(
        Shape::rectangle(20.0, 1.0),
        Vec2::new(0.0, -0.5),
    ));
    let ball = world.add_body(
        RigidBody::dynamic(Shape::circle(0.5), Vec2::new(0.0, 20.0), 1.0).with_restitution(0.4),
    );

    for _ in 0..300 {
        world.step(1.0 / 60.0);
        assert!(
            world.bodies[ball].position.y > -0.9,
            "tunneling complet à travers le sol : y={}",
            world.bodies[ball].position.y
        );
    }
    // Et après stabilisation, la balle doit bien être revenue se poser dessus.
    assert!(world.bodies[ball].position.y > 0.3);
}

#[test]
fn two_boxes_falling_side_by_side_do_not_interpenetrate() {
    let mut world = World::new(Vec2::new(0.0, -9.81));
    world.add_body(RigidBody::static_body(
        Shape::rectangle(20.0, 1.0),
        Vec2::new(0.0, -0.5),
    ));
    let a = world.add_body(RigidBody::dynamic(
        Shape::rectangle(1.0, 1.0),
        Vec2::new(-0.55, 5.0),
        1.0,
    ));
    let b = world.add_body(RigidBody::dynamic(
        Shape::rectangle(1.0, 1.0),
        Vec2::new(0.55, 5.0),
        1.0,
    ));

    for _ in 0..300 {
        world.step(1.0 / 60.0);
    }

    let gap = (world.bodies[b].position.x - world.bodies[a].position.x).abs();
    assert!(
        gap > 0.95,
        "les deux boîtes se chevauchent après repos : écart={gap}"
    );
}

#[test]
fn frictionless_ball_on_flat_ground_keeps_rolling_forever_in_x() {
    // Sans friction, une bille qui touche le sol avec une vitesse
    // horizontale ne doit subir aucune décélération le long de x (seule la
    // composante normale/verticale de sa vitesse est affectée par le contact).
    let mut world = World::new(Vec2::new(0.0, -9.81));
    world.add_body(
        RigidBody::static_body(Shape::rectangle(50.0, 1.0), Vec2::new(0.0, -0.5))
            .with_friction(0.0),
    );
    let ball = world.add_body(
        RigidBody::dynamic(Shape::circle(0.5), Vec2::new(-10.0, 0.5), 1.0)
            .with_friction(0.0)
            .with_restitution(0.0),
    );
    world.bodies[ball].velocity = Vec2::new(2.0, 0.0);

    for _ in 0..300 {
        world.step(1.0 / 60.0);
    }

    approx(
        world.bodies[ball].velocity.x,
        2.0,
        1e-6,
        "vitesse horizontale sans friction",
    );
}

#[test]
fn friction_decelerates_a_sliding_box_on_the_ground() {
    let mut world = World::new(Vec2::new(0.0, -9.81));
    world.add_body(
        RigidBody::static_body(Shape::rectangle(50.0, 1.0), Vec2::new(0.0, -0.5))
            .with_friction(0.6),
    );
    let box_idx = world.add_body(
        RigidBody::dynamic(Shape::rectangle(1.0, 1.0), Vec2::new(-10.0, 0.5), 1.0)
            .with_friction(0.6)
            .with_restitution(0.0),
    );
    world.bodies[box_idx].velocity = Vec2::new(5.0, 0.0);

    let initial_speed = world.bodies[box_idx].velocity.x;
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    let speed_after_2s = world.bodies[box_idx].velocity.x;

    assert!(
        speed_after_2s < initial_speed,
        "la friction doit ralentir la boîte : avant={initial_speed}, après={speed_after_2s}"
    );
    assert!(
        speed_after_2s >= 0.0,
        "la friction ne doit jamais inverser le sens du mouvement"
    );
}

#[test]
fn stack_of_five_boxes_does_not_collapse() {
    let mut world = World::new(Vec2::new(0.0, -9.81));
    world.add_body(RigidBody::static_body(
        Shape::rectangle(10.0, 1.0),
        Vec2::new(0.0, -0.5),
    ));

    let mut indices = Vec::new();
    for i in 0..5 {
        indices.push(
            world.add_body(
                RigidBody::dynamic(
                    Shape::rectangle(1.0, 1.0),
                    Vec2::new(0.0, i as f64 + 0.5),
                    1.0,
                )
                .with_friction(0.8)
                .with_restitution(0.0),
            ),
        );
    }

    for _ in 0..600 {
        world.step(1.0 / 60.0);
    }

    for (rank, &idx) in indices.iter().enumerate() {
        assert!(
            world.bodies[idx].position.x.abs() < 0.5,
            "boîte {rank} tombée sur le côté : x={}",
            world.bodies[idx].position.x
        );
        assert!(
            world.bodies[idx].rotation.abs() < 0.3,
            "boîte {rank} a trop basculé : rotation={}",
            world.bodies[idx].rotation
        );
    }
}

#[test]
fn elastic_collision_between_unequal_masses_conserves_momentum() {
    let mut world = World::without_gravity();
    let heavy = world.add_body(
        RigidBody::dynamic(Shape::circle(1.0), Vec2::new(-5.0, 0.0), 3.0).with_restitution(1.0),
    );
    let light = world.add_body(
        RigidBody::dynamic(Shape::circle(1.0), Vec2::new(5.0, 0.0), 1.0).with_restitution(1.0),
    );
    world.bodies[heavy].velocity = Vec2::new(1.0, 0.0);
    world.bodies[light].velocity = Vec2::ZERO;

    let momentum_before = world.total_momentum();
    for _ in 0..600 {
        world.step(1.0 / 120.0);
    }
    let momentum_after = world.total_momentum();

    approx(
        momentum_before.x,
        momentum_after.x,
        1e-6,
        "quantité de mouvement totale",
    );
}
