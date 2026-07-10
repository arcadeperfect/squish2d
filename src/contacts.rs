//! Blob-blob contacts. The subsume mechanic lives here: the same
//! particle-vs-membrane-edge contact runs in keep-out mode between separate
//! blobs and keep-in mode for contained prey — eating a blob just flips the
//! side of an inequality constraint.

use crate::geom::poly_query;
use crate::{RelState, World};
use glam::Vec2;

enum PairKind {
    /// Contacts handled by an ancestor relation elsewhere; do nothing.
    Skip,
    Free,
    Engulf {
        pred: usize,
        prey: usize,
    },
    Contained {
        pred: usize,
        prey: usize,
    },
}

impl World {
    pub(crate) fn solve_pairs(&mut self, h: f32) {
        let nb = self.blobs.len();
        self.build_aabbs();
        for a in 0..nb {
            if !self.blobs[a].alive {
                continue;
            }
            for b in (a + 1)..nb {
                if !self.blobs[b].alive {
                    continue;
                }
                match self.pair_kind(a, b) {
                    PairKind::Skip => {}
                    PairKind::Engulf { pred, prey } => self.engulf_pull(pred, prey, h),
                    PairKind::Contained { pred, prey } => {
                        self.keep_in(pred, prey);
                        // the predator's own membrane must not fold into the prey
                        self.keep_out(pred, prey);
                    }
                    PairKind::Free => {
                        if self.aabbs_overlap(a, b) {
                            self.keep_out(a, b);
                            self.keep_out(b, a);
                        }
                    }
                }
            }
        }
    }

    fn pair_kind(&self, a: usize, b: usize) -> PairKind {
        for r in &self.rels {
            if (r.pred == a && r.prey == b) || (r.pred == b && r.prey == a) {
                return match r.state {
                    RelState::Engulfing { .. } => PairKind::Engulf {
                        pred: r.pred,
                        prey: r.prey,
                    },
                    RelState::Contained => PairKind::Contained {
                        pred: r.pred,
                        prey: r.prey,
                    },
                };
            }
        }
        // Blobs nested more than one level apart never touch directly;
        // the chain of direct Contained relations keeps everyone sorted.
        if self.is_ancestor(a, b) || self.is_ancestor(b, a) {
            return PairKind::Skip;
        }
        PairKind::Free
    }

    fn build_aabbs(&mut self) {
        let nb = self.blobs.len();
        self.aabb_scratch.clear();
        self.aabb_scratch
            .resize(nb, (Vec2::splat(f32::MAX), Vec2::splat(f32::MIN)));
        for b in 0..nb {
            if !self.blobs[b].alive {
                continue;
            }
            let (s, c, r) = (
                self.blobs[b].start,
                self.blobs[b].count,
                self.blobs[b].particle_radius,
            );
            let mut mn = Vec2::splat(f32::MAX);
            let mut mx = Vec2::splat(f32::MIN);
            for i in s..s + c {
                mn = mn.min(self.pos[i]);
                mx = mx.max(self.pos[i]);
            }
            self.aabb_scratch[b] = (mn - Vec2::splat(r), mx + Vec2::splat(r));
        }
    }

    fn aabbs_overlap(&self, a: usize, b: usize) -> bool {
        let (amn, amx) = self.aabb_scratch[a];
        let (bmn, bmx) = self.aabb_scratch[b];
        amn.x <= bmx.x && bmn.x <= amx.x && amn.y <= bmx.y && bmn.y <= amx.y
    }

    /// During engulfing the pair ignores contacts and the prey gets reeled
    /// rigidly toward the predator's centroid.
    fn engulf_pull(&mut self, pred: usize, prey: usize, h: f32) {
        let centroid = |w: &World, b: usize| -> Vec2 {
            let (s, c) = (w.blobs[b].start, w.blobs[b].count);
            let mut cen = Vec2::ZERO;
            for i in s..s + c {
                cen += w.pos[i];
            }
            cen / c as f32
        };
        let cp = centroid(self, pred);
        let cq = centroid(self, prey);
        let d = cp - cq;
        let len = d.length();
        if len < 1e-6 {
            return;
        }
        let shift = d / len * (self.params.engulf_pull * h).min(len);
        let (s, c) = (self.blobs[prey].start, self.blobs[prey].count);
        for i in s..s + c {
            self.pos[i] += shift;
        }
    }

    /// Particles of `pa` may not enter the polygon of `pb` (and stay a
    /// membrane-thickness away from it).
    fn keep_out(&mut self, pa: usize, pb: usize) {
        let (as_, ac, ar) = (
            self.blobs[pa].start,
            self.blobs[pa].count,
            self.blobs[pa].particle_radius,
        );
        let (bs, bc, br) = (
            self.blobs[pb].start,
            self.blobs[pb].count,
            self.blobs[pb].particle_radius,
        );
        let margin = ar + br;
        let (bmn, bmx) = self.aabb_scratch[pb];
        for k in 0..ac {
            let pi = as_ + k;
            let p = self.pos[pi];
            if p.x < bmn.x - ar || p.x > bmx.x + ar || p.y < bmn.y - ar || p.y > bmx.y + ar {
                continue;
            }
            let hit = poly_query(&self.pos[bs..bs + bc], p);
            if hit.dist < 1e-6 {
                continue; // degenerate this substep; the next one resolves it
            }
            let (cst, n) = if hit.inside {
                // tunnelled through the membrane: exit via the nearest wall
                (hit.dist + margin, (hit.point - p) / hit.dist)
            } else if hit.dist < margin {
                (margin - hit.dist, (p - hit.point) / hit.dist)
            } else {
                continue;
            };
            self.apply_contact(pi, bs + hit.edge, bs + (hit.edge + 1) % bc, hit.t, n, cst);
        }
    }

    /// Contained prey: particles of `prey` must stay inside the polygon of
    /// `pred`. Pushing the prey inward pushes the predator wall outward, which
    /// is what makes a fed predator visibly bulge.
    fn keep_in(&mut self, pred: usize, prey: usize) {
        let (qs, qc, qr) = (
            self.blobs[prey].start,
            self.blobs[prey].count,
            self.blobs[prey].particle_radius,
        );
        let (ps, pc, pr) = (
            self.blobs[pred].start,
            self.blobs[pred].count,
            self.blobs[pred].particle_radius,
        );
        let margin = qr + pr;
        for k in 0..qc {
            let pi = qs + k;
            let p = self.pos[pi];
            let hit = poly_query(&self.pos[ps..ps + pc], p);
            if hit.dist < 1e-6 {
                continue;
            }
            let (cst, n) = if hit.inside {
                if hit.dist >= margin {
                    continue;
                }
                (margin - hit.dist, (p - hit.point) / hit.dist)
            } else {
                // escaped through the wall: pull back inside
                (hit.dist + margin, (hit.point - p) / hit.dist)
            };
            self.apply_contact(pi, ps + hit.edge, ps + (hit.edge + 1) % pc, hit.t, n, cst);
        }
    }

    /// Resolve one particle-vs-edge contact (rigid, with friction), pushing
    /// the particle along `n` by `cst` and the edge endpoints the other way.
    fn apply_contact(&mut self, pi: usize, e0: usize, e1: usize, t: f32, n: Vec2, cst: f32) {
        let wp = self.inv_mass[pi];
        let w0 = self.inv_mass[e0];
        let w1 = self.inv_mass[e1];
        let denom = wp + w0 * (1.0 - t) * (1.0 - t) + w1 * t * t;
        if denom <= 1e-12 {
            return;
        }
        let s = cst / denom;
        self.pos[pi] += n * (s * wp);
        self.pos[e0] -= n * (s * w0 * (1.0 - t));
        self.pos[e1] -= n * (s * w1 * t);

        // position-level Coulomb friction against the contact point
        let mu = self.params.friction;
        if mu <= 0.0 {
            return;
        }
        let disp_p = self.pos[pi] - self.prev[pi];
        let disp_c =
            (self.pos[e0] - self.prev[e0]) * (1.0 - t) + (self.pos[e1] - self.prev[e1]) * t;
        let rel = disp_p - disp_c;
        let tan = rel - n * rel.dot(n);
        let tl = tan.length();
        if tl < 1e-9 {
            return;
        }
        let f = tan * (tl.min(mu * cst) / tl);
        self.pos[pi] -= f * (wp / denom);
        self.pos[e0] += f * (w0 * (1.0 - t) / denom);
        self.pos[e1] += f * (w1 * t / denom);
    }
}
