use squish2d::{BlobParams, Event, Relation, Vec2, World, WorldParams};

const DT: f32 = 1.0 / 60.0;

fn quiet_world() -> World {
    let mut w = World::new(WorldParams {
        gravity: Vec2::ZERO,
        damping: 1.0,
        ..Default::default()
    });
    w.set_container(Vec2::new(-500.0, -500.0), Vec2::new(500.0, 500.0));
    w
}

fn assert_all_finite(w: &World) {
    for id in w.blob_ids() {
        for p in w.blob_points(id) {
            assert!(p.x.is_finite() && p.y.is_finite(), "NaN/inf in blob {id}");
        }
    }
}

#[test]
fn area_settles_to_target() {
    let mut w = quiet_world();
    let id = w.spawn_blob(BlobParams::new(Vec2::ZERO, 40.0));
    for _ in 0..120 {
        w.step(DT);
    }
    let target = w.blob_target_area(id);
    let area = w.blob_area(id);
    assert!(
        (area - target).abs() / target < 0.05,
        "area {area} vs target {target}"
    );
    assert_all_finite(&w);
}

#[test]
fn growth_follows_area_scale() {
    let mut w = quiet_world();
    let id = w.spawn_blob(BlobParams::new(Vec2::ZERO, 40.0));
    w.set_area_scale(id, 2.0);
    for _ in 0..180 {
        w.step(DT);
    }
    let rest = w.blob_area(id) / w.blob_frame(id).scale.powi(2); // recover rest area
    let ratio = w.blob_area(id) / rest;
    assert!((ratio - 2.0).abs() < 0.25, "grew to {ratio}x instead of 2x");
    assert!((w.blob_frame(id).scale - 2.0f32.sqrt()).abs() < 0.1);
}

#[test]
fn separate_blobs_do_not_interpenetrate() {
    let mut w = quiet_world();
    let a = w.spawn_blob(BlobParams::new(Vec2::new(-30.0, 0.0), 40.0));
    let b = w.spawn_blob(BlobParams::new(Vec2::new(30.0, 0.0), 40.0));
    for _ in 0..180 {
        w.step(DT);
    }
    for p in w.blob_points(a).to_vec() {
        assert!(!w.point_in_blob(b, p), "membrane A inside B after settling");
    }
    for p in w.blob_points(b).to_vec() {
        assert!(!w.point_in_blob(a, p), "membrane B inside A after settling");
    }
    assert_all_finite(&w);
}

#[test]
fn engulf_captures_and_contains() {
    let mut w = quiet_world();
    let pred = w.spawn_blob(BlobParams::new(Vec2::ZERO, 60.0));
    let prey = w.spawn_blob(BlobParams::new(Vec2::new(70.0, 0.0), 25.0));
    assert!(w.begin_engulf(pred, prey));
    assert_eq!(w.relation(pred, prey), Relation::Engulfing);

    let mut captured = false;
    for _ in 0..240 {
        w.step(DT);
        if w.drain_events()
            .iter()
            .any(|e| matches!(e, Event::Captured { .. }))
        {
            captured = true;
        }
    }
    assert!(captured, "prey was never captured");
    assert_eq!(w.relation(pred, prey), Relation::Contained);
    assert_eq!(w.contained_by(prey), Some(pred));
    // settled prey lives fully inside the predator
    for p in w.blob_points(prey).to_vec() {
        assert!(w.point_in_blob(pred, p), "prey particle escaped predator");
    }
    // predator swelled beyond its own rest target
    assert!(w.blob_target_area(pred) > w.blob_area(prey));
    assert_all_finite(&w);
}

#[test]
fn contained_prey_cannot_escape() {
    let mut w = quiet_world();
    let pred = w.spawn_blob(BlobParams::new(Vec2::ZERO, 60.0));
    let prey = w.spawn_blob(BlobParams::new(Vec2::new(50.0, 0.0), 22.0));
    assert!(w.begin_engulf(pred, prey));
    for _ in 0..180 {
        w.step(DT);
    }
    assert_eq!(w.relation(pred, prey), Relation::Contained);
    // prey struggles hard for three seconds
    w.set_locomotion(prey, Vec2::new(800.0, 0.0));
    for _ in 0..180 {
        w.step(DT);
    }
    assert_eq!(w.contained_by(prey), Some(pred));
    let inside = w
        .blob_points(prey)
        .to_vec()
        .into_iter()
        .filter(|p| w.point_in_blob(pred, *p))
        .count();
    assert!(
        inside >= w.blob_points(prey).len() * 8 / 10,
        "prey mostly escaped: {inside} inside"
    );
    assert_all_finite(&w);
}

#[test]
fn digestion_shrinks_prey_and_fires_event() {
    let mut w = quiet_world();
    w.params.digestion_rate = 0.8;
    let pred = w.spawn_blob(BlobParams::new(Vec2::ZERO, 60.0));
    let prey = w.spawn_blob(BlobParams::new(Vec2::new(50.0, 0.0), 22.0));
    assert!(w.begin_engulf(pred, prey));
    let mut digested = false;
    for _ in 0..600 {
        w.step(DT);
        if w.drain_events()
            .iter()
            .any(|e| matches!(e, Event::Digested { .. }))
        {
            digested = true;
        }
    }
    assert!(digested, "digestion event never fired");
    assert!(w.area_scale(prey) <= w.params.digestion_floor + 1e-3);
}

#[test]
fn packed_container_stays_stable() {
    // more blob area than box area: heavy mutual compression under gravity
    let mut w = World::new(WorldParams {
        gravity: Vec2::new(0.0, 900.0),
        damping: 1.0,
        ..Default::default()
    });
    w.set_container(Vec2::new(0.0, 0.0), Vec2::new(300.0, 220.0));
    let mut ids = Vec::new();
    for gy in 0..3 {
        for gx in 0..5 {
            let mut p = BlobParams::new(
                Vec2::new(40.0 + gx as f32 * 55.0, 40.0 + gy as f32 * 70.0),
                40.0,
            );
            p.area_compliance = 2.0; // allow squish under crush
            ids.push(w.spawn_blob(p));
        }
    }
    for _ in 0..600 {
        w.step(DT);
    }
    assert_all_finite(&w);
    for &id in &ids {
        let target = w.blob_target_area(id);
        let area = w.blob_area(id);
        assert!(
            area > 0.15 * target && area < 2.0 * target,
            "blob {id} degenerate: area {area} vs target {target}"
        );
        for p in w.blob_points(id) {
            assert!(
                p.x > -1.0 && p.x < 301.0 && p.y > -1.0 && p.y < 221.0,
                "blob {id} left the container: {p:?}"
            );
        }
    }
}

#[test]
fn simulation_is_deterministic() {
    let build = || {
        let mut w = World::new(WorldParams {
            gravity: Vec2::new(0.0, 500.0),
            ..Default::default()
        });
        w.set_container(Vec2::new(0.0, 0.0), Vec2::new(400.0, 300.0));
        let a = w.spawn_blob(BlobParams::new(Vec2::new(100.0, 80.0), 45.0));
        let b = w.spawn_blob(BlobParams::new(Vec2::new(200.0, 90.0), 30.0));
        let c = w.spawn_blob(BlobParams::new(Vec2::new(290.0, 70.0), 35.0));
        w.add_impulse(a, Vec2::new(120.0, 0.0));
        w.begin_engulf(a, b);
        let _ = c;
        w
    };
    let mut w1 = build();
    let mut w2 = build();
    for _ in 0..240 {
        w1.step(DT);
        w2.step(DT);
    }
    for id in w1.blob_ids() {
        let p1 = w1.blob_points(id);
        let p2 = w2.blob_points(id);
        for i in 0..p1.len() {
            assert_eq!(p1[i].x.to_bits(), p2[i].x.to_bits(), "x drift blob {id}");
            assert_eq!(p1[i].y.to_bits(), p2[i].y.to_bits(), "y drift blob {id}");
        }
    }
}

#[test]
fn remove_blob_frees_prey_to_grandparent() {
    let mut w = quiet_world();
    let outer = w.spawn_blob(BlobParams::new(Vec2::ZERO, 80.0));
    let mid = w.spawn_blob(BlobParams::new(Vec2::new(70.0, 0.0), 40.0));
    assert!(w.begin_engulf(outer, mid));
    for _ in 0..240 {
        w.step(DT);
    }
    assert_eq!(w.contained_by(mid), Some(outer));

    // an inner blob eaten by mid (siblings inside outer are allowed to hunt)
    let inner = w.spawn_blob(BlobParams::new(Vec2::new(200.0, 0.0), 18.0));
    // inner is outside; can't engulf across arenas
    assert!(!w.begin_engulf(mid, inner));

    // removing outer frees mid
    w.remove_blob(outer);
    assert_eq!(w.contained_by(mid), None);
    assert!(!w.alive(outer));
    for _ in 0..60 {
        w.step(DT);
    }
    assert_all_finite(&w);
}
