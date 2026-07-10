//! # squish2d
//!
//! Robust 2D soft-body blobs. XPBD ring solver using the "small steps" scheme
//! (many substeps, one constraint iteration each), so it cannot explode —
//! constraint error shows up as temporary softness, never as NaNs.
//!
//! Each blob is a closed ring of particles with:
//! - distance constraints along the membrane (+ soft bending),
//! - one area constraint acting as 2D pressure (`area_scale` is the gameplay
//!   knob: urgency, mass eaten, ...),
//! - shape matching for a stable decal frame (faces) and optional shape memory.
//!
//! Blobs never merge. A predator *subsumes* prey: during `Engulfing` the pair's
//! contacts are disabled and prey is reeled in; once captured (`Contained`) the
//! contact constraint flips from keep-out to keep-in, so prey stays a distinct
//! blob living inside the predator. Constraint topology never changes.

mod blob;
mod contacts;
mod geom;
mod relations;
mod solver;

pub use blob::{BlobFrame, BlobId, BlobParams};
pub use glam::Vec2;

use blob::Blob;
use geom::{fit_frame, poly_query, polygon_area};

#[derive(Clone, Copy, Debug)]
pub struct WorldParams {
    pub gravity: Vec2,
    /// XPBD substeps per `step()` call. More substeps = stiffer and more
    /// accurate at the same cost profile; 6-10 is a good range.
    pub substeps: u32,
    /// Linear velocity damping, 1/s.
    pub damping: f32,
    /// Coulomb-ish friction coefficient for all contacts.
    pub friction: f32,
    /// Speed (units/s) at which an engulfing predator reels prey toward its centroid.
    pub engulf_pull: f32,
    /// Engulf attempts abort after this many seconds.
    pub engulf_timeout: f32,
    /// Fraction of prey membrane that must be inside the predator to capture.
    pub capture_fraction: f32,
    /// Contained prey lose this fraction of their `area_scale` per second. 0 = off.
    pub digestion_rate: f32,
    /// Digestion never shrinks `area_scale` below this.
    pub digestion_floor: f32,
    /// Smoothing rate (1/s) for area-target changes (growth, capture swell).
    pub growth_rate: f32,
    /// Area headroom a predator gains per unit of contained prey area.
    pub swell_factor: f32,
}

impl Default for WorldParams {
    fn default() -> Self {
        Self {
            gravity: Vec2::ZERO,
            substeps: 8,
            damping: 0.5,
            friction: 0.4,
            engulf_pull: 100.0,
            engulf_timeout: 4.0,
            capture_fraction: 0.85,
            digestion_rate: 0.0,
            digestion_floor: 0.15,
            growth_rate: 3.0,
            swell_factor: 1.15,
        }
    }
}

/// Directional relation from a predator to a prey blob.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Relation {
    Separate,
    Engulfing,
    Contained,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// Predator finished engulfing prey; prey is now contained.
    Captured { pred: BlobId, prey: BlobId },
    /// An engulf attempt timed out and was abandoned.
    EngulfAborted { pred: BlobId, prey: BlobId },
    /// Digestion shrank contained prey down to the digestion floor.
    Digested { pred: BlobId, prey: BlobId },
}

pub type GrabId = usize;

#[derive(Clone, Copy, Debug)]
pub(crate) enum RelState {
    Engulfing { t: f32 },
    Contained,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct PairRel {
    pub pred: BlobId,
    pub prey: BlobId,
    pub state: RelState,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Grab {
    pub particle: usize,
    pub target: Vec2,
    pub compliance: f32,
    pub active: bool,
}

pub struct World {
    pub params: WorldParams,
    pub(crate) pos: Vec<Vec2>,
    pub(crate) prev: Vec<Vec2>,
    pub(crate) vel: Vec<Vec2>,
    pub(crate) inv_mass: Vec<f32>,
    pub(crate) blobs: Vec<Blob>,
    pub(crate) rels: Vec<PairRel>,
    pub(crate) grabs: Vec<Grab>,
    pub(crate) container: Option<(Vec2, Vec2)>,
    pub(crate) events: Vec<Event>,
    pub(crate) grad_scratch: Vec<Vec2>,
    pub(crate) aabb_scratch: Vec<(Vec2, Vec2)>,
}

impl World {
    pub fn new(params: WorldParams) -> Self {
        Self {
            params,
            pos: Vec::new(),
            prev: Vec::new(),
            vel: Vec::new(),
            inv_mass: Vec::new(),
            blobs: Vec::new(),
            rels: Vec::new(),
            grabs: Vec::new(),
            container: None,
            events: Vec::new(),
            grad_scratch: Vec::new(),
            aabb_scratch: Vec::new(),
        }
    }

    /// Axis-aligned box the blobs must stay inside.
    pub fn set_container(&mut self, min: Vec2, max: Vec2) {
        self.container = Some((min, max));
    }

    pub fn spawn_blob(&mut self, params: BlobParams) -> BlobId {
        let count = params.particle_count.max(8);
        let radius = params.radius.max(1e-3);
        // Reuse a dead slot with the same particle count to keep arrays compact.
        let reuse = self.blobs.iter().position(|b| !b.alive && b.count == count);
        let (id, start) = match reuse {
            Some(id) => (id, self.blobs[id].start),
            None => {
                let start = self.pos.len();
                self.pos.resize(start + count, Vec2::ZERO);
                self.prev.resize(start + count, Vec2::ZERO);
                self.vel.resize(start + count, Vec2::ZERO);
                self.inv_mass.resize(start + count, 0.0);
                (self.blobs.len(), start)
            }
        };

        let mut rest_local = Vec::with_capacity(count);
        for i in 0..count {
            let a = std::f32::consts::TAU * i as f32 / count as f32;
            rest_local.push(Vec2::new(a.cos(), a.sin()) * radius);
        }
        let inv_mass = count as f32 / params.mass.max(1e-6);
        for (i, r) in rest_local.iter().enumerate() {
            self.pos[start + i] = params.center + *r;
            self.prev[start + i] = params.center + *r;
            self.vel[start + i] = Vec2::ZERO;
            self.inv_mass[start + i] = inv_mass;
        }
        let rest_area = polygon_area(&rest_local);
        let base_edge = rest_local[0].distance(rest_local[1]);
        let base_chord = rest_local[0].distance(rest_local[2]);
        let particle_radius = params.particle_radius.unwrap_or(base_edge * 0.5);

        let blob = Blob {
            start,
            count,
            alive: true,
            params,
            rest_area,
            base_edge,
            base_chord,
            particle_radius,
            rest_local,
            area_scale: 1.0,
            swell: 0.0,
            area_target: rest_area,
            current_area: rest_area,
            contained_by: None,
            locomotion: Vec2::ZERO,
            frame_pos: params.center,
            frame_rot: 0.0,
        };
        match reuse {
            Some(_) => self.blobs[id] = blob,
            None => self.blobs.push(blob),
        }
        id
    }

    /// Remove a blob. Anything it contained is handed to its own container
    /// (or freed). Its slot is reused by later spawns of the same particle count.
    pub fn remove_blob(&mut self, id: BlobId) {
        if !self.alive(id) {
            return;
        }
        let parent = self.blobs[id].contained_by;
        let mut reparent: Vec<BlobId> = Vec::new();
        self.rels.retain(|r| {
            if r.prey == id {
                return false;
            }
            if r.pred == id {
                if matches!(r.state, RelState::Contained) {
                    reparent.push(r.prey);
                }
                return false;
            }
            true
        });
        for prey in reparent {
            self.blobs[prey].contained_by = parent;
            if let Some(gp) = parent {
                self.rels.push(PairRel {
                    pred: gp,
                    prey,
                    state: RelState::Contained,
                });
            }
        }
        self.blobs[id].alive = false;
        let (s, c) = (self.blobs[id].start, self.blobs[id].count);
        for g in self.grabs.iter_mut() {
            if g.active && g.particle >= s && g.particle < s + c {
                g.active = false;
            }
        }
        for i in s..s + c {
            self.vel[i] = Vec2::ZERO;
        }
    }

    /// Advance the simulation. Call with a fixed `dt` for determinism.
    pub fn step(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        self.update_targets(dt);
        let n = self.params.substeps.max(1);
        let h = dt / n as f32;
        for _ in 0..n {
            self.integrate(h);
            self.solve_blobs(h);
            self.solve_grabs(h);
            self.solve_pairs(h);
            self.solve_container();
            self.update_velocities(h);
        }
        self.update_relations(dt);
        self.update_frames();
    }

    // ---- queries ----------------------------------------------------------

    pub fn alive(&self, id: BlobId) -> bool {
        id < self.blobs.len() && self.blobs[id].alive
    }

    /// Ids of all live blobs.
    pub fn blob_ids(&self) -> impl Iterator<Item = BlobId> + '_ {
        self.blobs
            .iter()
            .enumerate()
            .filter(|(_, b)| b.alive)
            .map(|(i, _)| i)
    }

    /// Membrane particle positions, in ring order. Render this as the outline.
    pub fn blob_points(&self, id: BlobId) -> &[Vec2] {
        let b = &self.blobs[id];
        &self.pos[b.start..b.start + b.count]
    }

    /// Pose for rendering a decal (face) on the blob.
    pub fn blob_frame(&self, id: BlobId) -> BlobFrame {
        let b = &self.blobs[id];
        BlobFrame {
            pos: b.frame_pos,
            rot: b.frame_rot,
            scale: (b.current_area.max(0.0) / b.rest_area).sqrt(),
        }
    }

    pub fn blob_center(&self, id: BlobId) -> Vec2 {
        self.blobs[id].frame_pos
    }

    /// Polygon area measured after the last step.
    pub fn blob_area(&self, id: BlobId) -> f32 {
        self.blobs[id].current_area
    }

    /// The area the solver is currently trying to enforce.
    pub fn blob_target_area(&self, id: BlobId) -> f32 {
        self.blobs[id].area_target
    }

    pub fn area_scale(&self, id: BlobId) -> f32 {
        self.blobs[id].area_scale
    }

    /// The gameplay knob: desired area = rest area * scale (urgency, growth...).
    pub fn set_area_scale(&mut self, id: BlobId, scale: f32) {
        self.blobs[id].area_scale = scale.max(0.01);
    }

    /// Uniform acceleration applied to the blob every substep (locomotion/AI).
    pub fn set_locomotion(&mut self, id: BlobId, accel: Vec2) {
        self.blobs[id].locomotion = accel;
    }

    /// Instant velocity change applied to every particle of the blob.
    pub fn add_impulse(&mut self, id: BlobId, dv: Vec2) {
        let b = &self.blobs[id];
        for i in b.start..b.start + b.count {
            self.vel[i] += dv;
        }
    }

    pub fn contained_by(&self, id: BlobId) -> Option<BlobId> {
        self.blobs[id].contained_by
    }

    pub fn relation(&self, pred: BlobId, prey: BlobId) -> Relation {
        for r in &self.rels {
            if r.pred == pred && r.prey == prey {
                return match r.state {
                    RelState::Engulfing { .. } => Relation::Engulfing,
                    RelState::Contained => Relation::Contained,
                };
            }
        }
        Relation::Separate
    }

    /// Innermost live blob whose polygon contains `p` (contained prey win over
    /// their predator, so this is what you want for picking).
    pub fn blob_at_point(&self, p: Vec2) -> Option<BlobId> {
        let mut best: Option<(BlobId, f32)> = None;
        for (id, b) in self.blobs.iter().enumerate() {
            if !b.alive {
                continue;
            }
            let hit = poly_query(&self.pos[b.start..b.start + b.count], p);
            if hit.inside && best.is_none_or(|(_, a)| b.current_area < a) {
                best = Some((id, b.current_area));
            }
        }
        best.map(|(id, _)| id)
    }

    /// True if `p` is inside the blob's polygon.
    pub fn point_in_blob(&self, id: BlobId, p: Vec2) -> bool {
        let b = &self.blobs[id];
        poly_query(&self.pos[b.start..b.start + b.count], p).inside
    }

    pub fn drain_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }

    // ---- subsume ----------------------------------------------------------

    /// Start swallowing `prey`. Contacts between the pair switch off and the
    /// prey is reeled toward the predator's centroid; capture happens when
    /// `capture_fraction` of the prey membrane is inside the predator.
    /// Both blobs must be in the same "arena" (both free, or siblings inside
    /// the same host). Returns false if the relationship is not allowed.
    pub fn begin_engulf(&mut self, pred: BlobId, prey: BlobId) -> bool {
        if pred == prey || !self.alive(pred) || !self.alive(prey) {
            return false;
        }
        // prey must not already be someone's prey
        if self.rels.iter().any(|r| r.prey == prey) {
            return false;
        }
        if self.blobs[pred].contained_by != self.blobs[prey].contained_by {
            return false;
        }
        if self.is_ancestor(prey, pred) {
            return false;
        }
        self.rels.push(PairRel {
            pred,
            prey,
            state: RelState::Engulfing { t: 0.0 },
        });
        true
    }

    /// End a Contained/Engulfing relation. A contained prey is expelled by the
    /// regular keep-out contacts (it squeezes back out through the membrane).
    pub fn release_prey(&mut self, pred: BlobId, prey: BlobId) -> bool {
        let Some(i) = self
            .rels
            .iter()
            .position(|r| r.pred == pred && r.prey == prey)
        else {
            return false;
        };
        self.rels.swap_remove(i);
        if self.blobs[prey].contained_by == Some(pred) {
            let gp = self.blobs[pred].contained_by;
            self.blobs[prey].contained_by = gp;
            if let Some(gp) = gp {
                self.rels.push(PairRel {
                    pred: gp,
                    prey,
                    state: RelState::Contained,
                });
            }
        }
        true
    }

    pub(crate) fn is_ancestor(&self, anc: BlobId, of: BlobId) -> bool {
        let mut cur = self.blobs[of].contained_by;
        let mut hops = 0;
        while let Some(c) = cur {
            if c == anc {
                return true;
            }
            cur = self.blobs[c].contained_by;
            hops += 1;
            if hops > 64 {
                break;
            }
        }
        false
    }

    // ---- grabs (mouse / touch springs) ------------------------------------

    /// Attach a compliant spring to the particle nearest `p` (within `max_dist`).
    pub fn grab_nearest(&mut self, p: Vec2, max_dist: f32, compliance: f32) -> Option<GrabId> {
        let mut best: Option<(usize, f32)> = None;
        for b in &self.blobs {
            if !b.alive {
                continue;
            }
            for i in b.start..b.start + b.count {
                let d = self.pos[i].distance(p);
                if d <= max_dist && best.is_none_or(|(_, bd)| d < bd) {
                    best = Some((i, d));
                }
            }
        }
        let (particle, _) = best?;
        let g = Grab {
            particle,
            target: p,
            compliance,
            active: true,
        };
        if let Some(slot) = self.grabs.iter().position(|g| !g.active) {
            self.grabs[slot] = g;
            Some(slot)
        } else {
            self.grabs.push(g);
            Some(self.grabs.len() - 1)
        }
    }

    pub fn set_grab_target(&mut self, id: GrabId, p: Vec2) {
        if let Some(g) = self.grabs.get_mut(id) {
            g.target = p;
        }
    }

    pub fn release_grab(&mut self, id: GrabId) {
        if let Some(g) = self.grabs.get_mut(id) {
            g.active = false;
        }
    }

    pub(crate) fn update_frames(&mut self) {
        for b in 0..self.blobs.len() {
            if !self.blobs[b].alive {
                continue;
            }
            let (s, c) = (self.blobs[b].start, self.blobs[b].count);
            let (cen, rot) = fit_frame(&self.pos[s..s + c], &self.blobs[b].rest_local);
            let area = polygon_area(&self.pos[s..s + c]);
            let bl = &mut self.blobs[b];
            bl.frame_pos = cen;
            bl.frame_rot = rot;
            bl.current_area = area.max(0.0);
        }
    }
}
