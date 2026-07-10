//! Interactive playground: amoeba predation + crush-them-together stress test.
//!
//!   cargo run --example demo --release
//!
//! drag blobs with the mouse; scroll over a blob to grow/shrink it;
//! SPACE spawn, E auto-eat, D digestion, G gravity, R reset.

use macroquad::prelude::*;
use macroquad::rand::gen_range;
use squish2d as sq;

fn conf() -> Conf {
    Conf {
        window_title: "squish2d demo".to_owned(),
        window_width: 1280,
        window_height: 720,
        high_dpi: false,
        ..Default::default()
    }
}

#[derive(Clone, Copy)]
struct Visual {
    hue: f32,
    base_radius: f32,
}

struct App {
    world: sq::World,
    visuals: Vec<Visual>,
}

fn to_mq(p: sq::Vec2) -> Vec2 {
    vec2(p.x, p.y)
}

fn rot2(v: (f32, f32), a: f32) -> (f32, f32) {
    let (s, c) = a.sin_cos();
    (v.0 * c - v.1 * s, v.0 * s + v.1 * c)
}

fn hsl(h: f32, s: f32, l: f32, a: f32) -> Color {
    let h = h.rem_euclid(1.0) * 6.0;
    let c = (1.0 - (2.0 * l - 1.0).abs()) * s;
    let x = c * (1.0 - (h.rem_euclid(2.0) - 1.0).abs());
    let (r, g, b) = match h as i32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    let m = l - c / 2.0;
    Color::new(r + m, g + m, b + m, a)
}

fn spawn(app: &mut App, center: sq::Vec2, radius: f32) {
    let mut p = sq::BlobParams::new(center, radius);
    p.area_compliance = 12.0; // squishy volume
    p.edge_compliance = 1e-3; // stretchy membrane
    p.bend_compliance = 2e-3; // floppy
    p.mass = (radius / 50.0).powi(2);
    let id = app.world.spawn_blob(p);
    let v = Visual {
        hue: gen_range(0.0, 1.0),
        base_radius: radius,
    };
    if id < app.visuals.len() {
        app.visuals[id] = v;
    } else {
        app.visuals.push(v);
    }
}

fn setup() -> App {
    let world = sq::World::new(sq::WorldParams {
        gravity: sq::Vec2::ZERO,
        substeps: 8,
        damping: 0.7,
        friction: 0.3,
        engulf_pull: 140.0,
        ..Default::default()
    });
    let mut app = App {
        world,
        visuals: Vec::new(),
    };
    app.world
        .set_container(sq::Vec2::new(16.0, 16.0), sq::Vec2::new(1264.0, 704.0));
    for gy in 0..3 {
        for gx in 0..3 {
            let c = sq::Vec2::new(
                240.0 + gx as f32 * 400.0 + gen_range(-60.0, 60.0),
                160.0 + gy as f32 * 200.0 + gen_range(-40.0, 40.0),
            );
            spawn(&mut app, c, gen_range(28.0, 64.0));
        }
    }
    app
}

fn depth(w: &sq::World, id: sq::BlobId) -> usize {
    let mut d = 0;
    let mut cur = w.contained_by(id);
    while let Some(c) = cur {
        d += 1;
        cur = w.contained_by(c);
        if d > 16 {
            break;
        }
    }
    d
}

fn draw_blob(app: &App, id: sq::BlobId) {
    let w = &app.world;
    let v = app.visuals[id];
    let pts = w.blob_points(id);
    let n = pts.len();
    let frame = w.blob_frame(id);
    let c = to_mq(frame.pos);
    let contained = w.contained_by(id).is_some();

    let fill = hsl(v.hue, 0.55, 0.55, 0.45);
    let line = hsl(v.hue, 0.65, 0.70, 1.0);
    for i in 0..n {
        draw_triangle(c, to_mq(pts[i]), to_mq(pts[(i + 1) % n]), fill);
    }
    for i in 0..n {
        let a = to_mq(pts[i]);
        let b = to_mq(pts[(i + 1) % n]);
        draw_line(a.x, a.y, b.x, b.y, 3.0, line);
    }

    // face, pinned by the shape-matched frame
    let r = v.base_radius * frame.scale;
    let face = |local: (f32, f32)| -> Vec2 {
        let p = rot2((local.0 * r, local.1 * r), frame.rot);
        vec2(c.x + p.0, c.y + p.1)
    };
    let white = Color::new(0.97, 0.97, 0.94, 1.0);
    let dark = Color::new(0.10, 0.10, 0.14, 1.0);
    for side in [-1.0f32, 1.0] {
        let e = face((0.32 * side, -0.18));
        draw_circle(e.x, e.y, (0.13 * r).max(2.0), white);
        draw_circle(e.x, e.y, (0.055 * r).max(1.0), dark);
    }
    // mouth: smile normally, worry when swallowed
    let mut prev: Option<Vec2> = None;
    for k in 0..=8 {
        let t = k as f32 / 8.0 - 0.5;
        let bulge = 1.0 - (2.0 * t) * (2.0 * t);
        let y = if contained {
            0.30 - 0.14 * bulge
        } else {
            0.18 + 0.12 * bulge
        };
        let p = face((t * 0.66, y));
        if let Some(q) = prev {
            draw_line(q.x, q.y, p.x, p.y, (0.05 * r).max(1.5), dark);
        }
        prev = Some(p);
    }
}

#[macroquad::main(conf)]
async fn main() {
    let mut app = setup();
    let mut grab: Option<sq::GrabId> = None;
    let mut auto_eat = true;
    let mut digestion = false;
    let mut gravity = false;
    let mut frame_no: u64 = 0;
    let mut last_event = String::new();
    let mut event_age = 0.0f32;

    loop {
        let (mx, my) = mouse_position();
        let m = sq::Vec2::new(mx, my);

        // --- input ---------------------------------------------------------
        if is_mouse_button_pressed(MouseButton::Left) {
            grab = app.world.grab_nearest(m, 60.0, 4e-4);
        }
        if let Some(g) = grab {
            if is_mouse_button_down(MouseButton::Left) {
                app.world.set_grab_target(g, m);
            } else {
                app.world.release_grab(g);
                grab = None;
            }
        }
        let wheel = mouse_wheel().1;
        if wheel.abs() > 0.01 {
            if let Some(id) = app.world.blob_at_point(m) {
                let s = (app.world.area_scale(id) * 1.15f32.powf(wheel.signum())).max(0.25);
                app.world.set_area_scale(id, s);
            }
        }
        if is_key_pressed(KeyCode::Space) {
            spawn(&mut app, m, gen_range(24.0, 60.0));
        }
        if is_key_pressed(KeyCode::G) {
            gravity = !gravity;
            app.world.params.gravity = if gravity {
                sq::Vec2::new(0.0, 1100.0)
            } else {
                sq::Vec2::ZERO
            };
        }
        if is_key_pressed(KeyCode::D) {
            digestion = !digestion;
            app.world.params.digestion_rate = if digestion { 0.18 } else { 0.0 };
        }
        if is_key_pressed(KeyCode::E) {
            auto_eat = !auto_eat;
        }
        if is_key_pressed(KeyCode::R) {
            app = setup();
            grab = None;
        }

        // --- auto predation ------------------------------------------------
        if auto_eat && frame_no.is_multiple_of(24) {
            let ids: Vec<_> = app.world.blob_ids().collect();
            for &a in &ids {
                for &b in &ids {
                    if a == b {
                        continue;
                    }
                    let (ta, tb) = (app.world.blob_target_area(a), app.world.blob_target_area(b));
                    if ta < tb * 2.2 {
                        continue;
                    }
                    let ra = (ta / std::f32::consts::PI).sqrt();
                    let rb = (tb / std::f32::consts::PI).sqrt();
                    let d = app.world.blob_center(a).distance(app.world.blob_center(b));
                    if d < 0.75 * (ra + rb) {
                        app.world.begin_engulf(a, b);
                    }
                }
            }
        }

        // --- step ----------------------------------------------------------
        app.world.step(1.0 / 60.0);
        for ev in app.world.drain_events() {
            match ev {
                sq::Event::Captured { pred, prey } => {
                    last_event = format!("blob {pred} swallowed blob {prey}");
                    event_age = 0.0;
                }
                sq::Event::Digested { pred, prey } => {
                    last_event = format!("blob {pred} digested blob {prey}");
                    event_age = 0.0;
                    app.world.remove_blob(prey);
                }
                sq::Event::EngulfAborted { .. } => {}
            }
        }
        event_age += 1.0 / 60.0;
        frame_no += 1;

        // --- draw ----------------------------------------------------------
        clear_background(Color::new(0.078, 0.082, 0.11, 1.0));
        draw_rectangle_lines(
            16.0,
            16.0,
            1248.0,
            688.0,
            2.0,
            Color::new(0.3, 0.32, 0.4, 1.0),
        );

        let mut ids: Vec<_> = app.world.blob_ids().collect();
        ids.sort_by_key(|&id| depth(&app.world, id)); // containers first, prey on top
        let particle_total: usize = ids.iter().map(|&id| app.world.blob_points(id).len()).sum();
        for id in ids {
            draw_blob(&app, id);
        }

        let hud = Color::new(0.75, 0.78, 0.85, 1.0);
        draw_text(
            "drag: move   scroll: grow/shrink   SPACE: spawn   E: auto-eat   D: digestion   G: gravity   R: reset",
            24.0, 40.0, 20.0, hud,
        );
        draw_text(
            format!(
                "auto-eat {}   digestion {}   gravity {}   blobs {}   particles {}   fps {}",
                if auto_eat { "ON" } else { "off" },
                if digestion { "ON" } else { "off" },
                if gravity { "ON" } else { "off" },
                app.world.blob_ids().count(),
                particle_total,
                get_fps(),
            ),
            24.0,
            66.0,
            20.0,
            hud,
        );
        if !last_event.is_empty() && event_age < 3.0 {
            draw_text(&last_event, 24.0, 694.0, 22.0, hsl(0.12, 0.8, 0.7, 1.0));
        }

        next_frame().await;
    }
}
