//! XPBD substep core: integration, per-blob constraints, container, velocities.
//!
//! One Gauss-Seidel pass per constraint per substep ("small steps" scheme,
//! Macklin et al. 2019). Lambda accumulators are unnecessary at 1 iteration.

use crate::geom::fit_frame;
use crate::World;
use glam::Vec2;

impl World {
    pub(crate) fn integrate(&mut self, h: f32) {
        let g = self.params.gravity;
        let damp = (1.0 - self.params.damping * h).clamp(0.0, 1.0);
        for b in 0..self.blobs.len() {
            if !self.blobs[b].alive {
                continue;
            }
            let (s, c, loco) = (
                self.blobs[b].start,
                self.blobs[b].count,
                self.blobs[b].locomotion,
            );
            for i in s..s + c {
                self.vel[i] += (g + loco) * h;
                self.vel[i] *= damp;
                self.prev[i] = self.pos[i];
                self.pos[i] += self.vel[i] * h;
            }
        }
    }

    pub(crate) fn update_velocities(&mut self, h: f32) {
        let inv_h = 1.0 / h;
        for b in 0..self.blobs.len() {
            if !self.blobs[b].alive {
                continue;
            }
            let (s, c) = (self.blobs[b].start, self.blobs[b].count);
            for i in s..s + c {
                self.vel[i] = (self.pos[i] - self.prev[i]) * inv_h;
            }
        }
    }

    pub(crate) fn solve_blobs(&mut self, h: f32) {
        for b in 0..self.blobs.len() {
            if !self.blobs[b].alive {
                continue;
            }
            self.solve_edges(b, h);
            self.solve_bend(b, h);
            self.solve_area(b, h);
            self.solve_shape_memory(b, h);
        }
    }

    /// Membrane rest lengths scale with sqrt(area target), so a growing blob
    /// relaxes its membrane instead of fighting its own pressure.
    fn ring_scale(&self, b: usize) -> f32 {
        let bl = &self.blobs[b];
        (bl.area_target / bl.rest_area).max(1e-6).sqrt()
    }

    fn solve_edges(&mut self, b: usize, h: f32) {
        let (s, c) = (self.blobs[b].start, self.blobs[b].count);
        let rest = self.blobs[b].base_edge * self.ring_scale(b);
        let alpha = self.blobs[b].params.edge_compliance / (h * h);
        self.solve_ring_distance(s, c, 1, rest, alpha);
    }

    fn solve_bend(&mut self, b: usize, h: f32) {
        let (s, c) = (self.blobs[b].start, self.blobs[b].count);
        let rest = self.blobs[b].base_chord * self.ring_scale(b);
        let alpha = self.blobs[b].params.bend_compliance / (h * h);
        self.solve_ring_distance(s, c, 2, rest, alpha);
    }

    /// Distance constraints between ring particles `i` and `i + stride`.
    fn solve_ring_distance(&mut self, s: usize, c: usize, stride: usize, rest: f32, alpha: f32) {
        for i in 0..c {
            let a = s + i;
            let b = s + (i + stride) % c;
            let d = self.pos[b] - self.pos[a];
            let len = d.length();
            if len < 1e-9 {
                continue;
            }
            let n = d / len;
            let cst = len - rest;
            let wsum = self.inv_mass[a] + self.inv_mass[b];
            if wsum <= 0.0 {
                continue;
            }
            let corr = -cst / (wsum + alpha);
            self.pos[a] -= n * (corr * self.inv_mass[a]);
            self.pos[b] += n * (corr * self.inv_mass[b]);
        }
    }

    /// Polygon-area constraint: the 2D pressure holding the blob open.
    fn solve_area(&mut self, b: usize, h: f32) {
        let (s, c) = (self.blobs[b].start, self.blobs[b].count);
        let target = self.blobs[b].area_target;
        let alpha = self.blobs[b].params.area_compliance / (h * h);

        let mut area = 0.0f32;
        for i in 0..c {
            area += self.pos[s + i].perp_dot(self.pos[s + (i + 1) % c]);
        }
        area *= 0.5;
        let cst = area - target;

        // dA/dp_i = 0.5 * (y_next - y_prev, x_prev - x_next)
        self.grad_scratch.clear();
        let mut denom = alpha;
        for i in 0..c {
            let prv = self.pos[s + (i + c - 1) % c];
            let nxt = self.pos[s + (i + 1) % c];
            let d = nxt - prv;
            let g = Vec2::new(d.y, -d.x) * 0.5;
            denom += self.inv_mass[s + i] * g.length_squared();
            self.grad_scratch.push(g);
        }
        if denom <= 1e-12 {
            return;
        }
        let dl = -cst / denom;
        for i in 0..c {
            let w = self.inv_mass[s + i];
            self.pos[s + i] += self.grad_scratch[i] * (dl * w);
        }
    }

    /// Optional gentle pull toward the shape-matched rest shape.
    fn solve_shape_memory(&mut self, b: usize, h: f32) {
        let rate = self.blobs[b].params.shape_memory;
        if rate <= 0.0 {
            return;
        }
        let (s, c) = (self.blobs[b].start, self.blobs[b].count);
        let (cen, rot) = fit_frame(&self.pos[s..s + c], &self.blobs[b].rest_local);
        let scale = self.ring_scale(b);
        let r = Vec2::from_angle(rot);
        let k = (rate * h).min(1.0);
        for i in 0..c {
            let goal = cen + r.rotate(self.blobs[b].rest_local[i] * scale);
            let p = self.pos[s + i];
            self.pos[s + i] = p + (goal - p) * k;
        }
    }

    pub(crate) fn solve_grabs(&mut self, h: f32) {
        for gi in 0..self.grabs.len() {
            let g = self.grabs[gi];
            if !g.active {
                continue;
            }
            let w = self.inv_mass[g.particle];
            if w <= 0.0 {
                continue;
            }
            let alpha = g.compliance / (h * h);
            let d = self.pos[g.particle] - g.target;
            let len = d.length();
            if len < 1e-9 {
                continue;
            }
            let corr = -len / (w + alpha);
            self.pos[g.particle] += (d / len) * (corr * w);
        }
    }

    /// Rigid axis-aligned container with position-level friction.
    pub(crate) fn solve_container(&mut self) {
        let Some((mn, mx)) = self.container else {
            return;
        };
        let mu = self.params.friction;
        for b in 0..self.blobs.len() {
            if !self.blobs[b].alive {
                continue;
            }
            let (s, c, r) = (
                self.blobs[b].start,
                self.blobs[b].count,
                self.blobs[b].particle_radius,
            );
            for i in s..s + c {
                let mut p = self.pos[i];
                let pv = self.prev[i];
                if p.x < mn.x + r {
                    let pen = (mn.x + r) - p.x;
                    p.x = mn.x + r;
                    p.y -= (p.y - pv.y).clamp(-mu * pen, mu * pen);
                }
                if p.x > mx.x - r {
                    let pen = p.x - (mx.x - r);
                    p.x = mx.x - r;
                    p.y -= (p.y - pv.y).clamp(-mu * pen, mu * pen);
                }
                if p.y < mn.y + r {
                    let pen = (mn.y + r) - p.y;
                    p.y = mn.y + r;
                    p.x -= (p.x - pv.x).clamp(-mu * pen, mu * pen);
                }
                if p.y > mx.y - r {
                    let pen = p.y - (mx.y - r);
                    p.y = mx.y - r;
                    p.x -= (p.x - pv.x).clamp(-mu * pen, mu * pen);
                }
                self.pos[i] = p;
            }
        }
    }
}
