//! Headless perf check: a packed party-game-sized scene.
//!   cargo run --example bench --release

use squish2d::{BlobParams, Vec2, World, WorldParams};
use std::time::Instant;

fn main() {
    let mut w = World::new(WorldParams {
        gravity: Vec2::new(0.0, 900.0),
        ..Default::default()
    });
    w.set_container(Vec2::new(0.0, 0.0), Vec2::new(1200.0, 700.0));
    let mut n = 0;
    for gy in 0..5 {
        for gx in 0..8 {
            let mut p = BlobParams::new(
                Vec2::new(80.0 + gx as f32 * 140.0, 80.0 + gy as f32 * 130.0),
                55.0,
            );
            p.area_compliance = 2.0;
            w.spawn_blob(p);
            n += 1;
        }
    }
    // heavy compression: everyone wants to be bigger than the box allows
    for id in w.blob_ids().collect::<Vec<_>>() {
        w.set_area_scale(id, 1.5);
    }
    let steps = 600;
    let t0 = Instant::now();
    for _ in 0..steps {
        w.step(1.0 / 60.0);
    }
    let el = t0.elapsed();
    println!(
        "{n} blobs ({} particles), {steps} steps in {:.1?} => {:.3} ms/step",
        n * 32,
        el,
        el.as_secs_f64() * 1000.0 / steps as f64
    );
}
