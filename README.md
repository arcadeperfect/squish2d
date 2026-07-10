# squish2d

Robust 2D soft-body blobs. No springs, no rigid-body hacks: an **XPBD** particle-ring
solver using the **small steps** scheme (many substeps × one constraint iteration),
so it cannot explode — constraint error shows up as momentary softness, never NaNs.

Built for two apps: an amoeba party game (blobs subsume each other and stay distinct
entities inside their predator) and a to-do app (blobs grow with urgency and crush
against each other in a container).

## Run the demo

```sh
cargo run --example demo --release
```

- **drag** blobs with the mouse
- **scroll** over a blob to grow/shrink it (the "urgency" knob)
- **SPACE** spawn at cursor, **E** auto-eat, **D** digestion, **G** gravity, **R** reset

## Model

Each blob is a ring of particles with:

- **distance constraints** along the membrane (+ soft second-neighbour bending)
- an **area constraint** = 2D pressure. `set_area_scale(id, x)` is the gameplay knob;
  membrane rest lengths follow `sqrt(area)` so growth stays relaxed
- **shape matching** for a stable rotation frame (`blob_frame`) to pin face decals to
- particle-vs-membrane-edge **contacts** with position-level friction

**Subsume, not merge:** `begin_engulf(pred, prey)` disables the pair's contacts and
reels prey in; on capture the contact flips from keep-out to *keep-in*. Prey remains
a fully simulated blob inside the predator (bulging its wall, draggable, escapable
via `release_prey` / growth). Constraint topology never changes, which keeps the
whole thing in XPBD's most robust regime. Digestion, swell, nesting and sibling
collisions inside a host all fall out of the same pair-state machine.

## API sketch

```rust
let mut w = World::new(WorldParams::default());
w.set_container(Vec2::ZERO, Vec2::new(1280.0, 720.0));
let id = w.spawn_blob(BlobParams::new(Vec2::new(200.0, 200.0), 50.0));
w.set_area_scale(id, 1.8);          // urgency / growth
w.step(1.0 / 60.0);                 // fixed dt => deterministic
let outline = w.blob_points(id);    // render however you like (metaballs, mesh, ...)
let frame = w.blob_frame(id);       // pos + rot + scale for the face decal
```

Engine-agnostic core (only dep: `glam`). Hosts planned: Bevy plugin (party game),
wasm-bindgen (web/Tauri), godot-rust (blob_todo).

## References

- Ten Minute Physics 09 + 10 (Müller) — XPBD walkthrough + unbreakable soft bodies
- [Small Steps in Physics Simulation](https://mmacklin.com/smallsteps.pdf) (Macklin et al. 2019)
- [XPBD](https://mmacklin.com/xpbd.pdf) (Macklin, Müller, Chentanez 2016)
- [PBD survey course notes](https://matthias-research.github.io/pages/publications/PBDTutorial2017-CourseNotes.pdf) (Bender, Müller, Macklin 2017)
- [Meshless Deformations Based on Shape Matching](https://matthias-research.github.io/pages/publications/MeshlessDeformations_SIG05.pdf) (Müller et al. 2005)
- [avbd-demo2d](https://github.com/savant117/avbd-demo2d) — modern 2D unified-solver reference
