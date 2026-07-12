# squish2d

**full disclosure**

This was made with ai. Do with that as you will. 


2D soft-body blobs. **XPBD** particle-ring solver using the **small steps** 
scheme (many substeps × one constraint iteration), so it cannot explode — constraint 
error shows up as momentary softness, never NaNs.

```sh
cargo run --example demo --release
```

## Techniques

Each blob is a ring of particles solved with:

- **distance constraints** along the membrane (+ soft second-neighbour bending)
- an **area constraint** = 2D pressure; `set_area_scale` is the size knob
- **shape matching** for a stable rotation frame (face decals, shape memory)
- particle-vs-edge **contacts** with position-level friction


Engine-agnostic core; only dependency is `glam`. Fixed `dt` → deterministic.

## References

- Ten Minute Physics 09 + 10 (Müller) — XPBD + unbreakable soft bodies
- [Small Steps in Physics Simulation](https://mmacklin.com/smallsteps.pdf) (Macklin et al. 2019)
- [XPBD](https://mmacklin.com/xpbd.pdf) (Macklin, Müller, Chentanez 2016)
- [PBD survey course notes](https://matthias-research.github.io/pages/publications/PBDTutorial2017-CourseNotes.pdf) (Bender, Müller, Macklin 2017)
- [Meshless Deformations Based on Shape Matching](https://matthias-research.github.io/pages/publications/MeshlessDeformations_SIG05.pdf) (Müller et al. 2005)
