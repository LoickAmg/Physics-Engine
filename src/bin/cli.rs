//! CLI headless : lance une scène prédéfinie et imprime son évolution —
//! sert de vérification manuelle rapide (sans rendu) et de démonstration
//! des scènes disponibles. Les mêmes scènes sont utilisées par le frontend
//! visuel (`physics-pixels`, feature `pixels`) via `physics::scenes`.

use physics::scenes::Scene;
use std::env;

fn main() {
    let scene_name = env::args().nth(1).unwrap_or_else(|| "bounce".to_string());
    let steps: usize = env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(180);

    let scene = match scene_name.as_str() {
        "bounce" => Scene::Bounce,
        "stack" => Scene::Stack,
        "billiard" => Scene::Billiard,
        "ramp" => Scene::Ramp,
        other => {
            eprintln!("Scène inconnue : {other}");
            eprintln!("Scènes disponibles : bounce, stack, billiard, ramp");
            std::process::exit(1);
        }
    };

    let mut world = scene.build();

    println!(
        "Scène « {} » — {} corps, {steps} pas à 1/60s",
        scene.label(),
        world.bodies.len()
    );
    println!(
        "{:>6} | {:>10} | {:>10} | {:>12} | {:>12}",
        "pas", "énergie", "|momentum|", "corps[dernier].y", "corps[dernier].rot"
    );

    let dt = 1.0 / 60.0;
    for step in 0..steps {
        world.step(dt);
        if step % 10 == 0 || step == steps - 1 {
            let last = world.bodies.len() - 1;
            println!(
                "{:>6} | {:>10.4} | {:>10.4} | {:>12.4} | {:>12.4}",
                step,
                world.kinetic_energy(),
                world.total_momentum().length(),
                world.bodies[last].position.y,
                world.bodies[last].rotation,
            );
        }
    }
}
