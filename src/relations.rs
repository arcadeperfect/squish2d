//! Per-step (not per-substep) bookkeeping: area-target smoothing, digestion,
//! predator swell, and the engulf state machine.

use crate::geom::poly_query;
use crate::{Event, RelState, World};

impl World {
    /// Runs once per `step`, before the substeps.
    pub(crate) fn update_targets(&mut self, dt: f32) {
        // digestion: contained prey slowly shrink
        if self.params.digestion_rate > 0.0 {
            let floor = self.params.digestion_floor;
            let decay = (1.0 - self.params.digestion_rate * dt).max(0.0);
            for i in 0..self.rels.len() {
                let (pred, prey, contained) = {
                    let r = &self.rels[i];
                    (r.pred, r.prey, matches!(r.state, RelState::Contained))
                };
                if !contained || !self.blobs[prey].alive {
                    continue;
                }
                let before = self.blobs[prey].area_scale;
                let after = (before * decay).max(floor);
                self.blobs[prey].area_scale = after;
                if before > floor + 1e-4 && after <= floor + 1e-4 {
                    self.events.push(Event::Digested { pred, prey });
                }
            }
        }

        // predator swell: room for what it swallowed (one-frame lag on nesting
        // is fine — targets are smoothed anyway)
        for b in &mut self.blobs {
            b.swell = 0.0;
        }
        for i in 0..self.rels.len() {
            let (pred, prey, contained) = {
                let r = &self.rels[i];
                (r.pred, r.prey, matches!(r.state, RelState::Contained))
            };
            if !contained {
                continue;
            }
            let add = self.blobs[prey].area_target;
            self.blobs[pred].swell += add;
        }

        // smooth every blob's working target toward its desired area
        let k = (self.params.growth_rate * dt).min(1.0);
        let swell_factor = self.params.swell_factor;
        for b in &mut self.blobs {
            if !b.alive {
                continue;
            }
            let desired = b.rest_area * b.area_scale + b.swell * swell_factor;
            b.area_target += (desired - b.area_target) * k;
        }
    }

    /// Runs once per `step`, after the substeps: advances engulf timers and
    /// promotes Engulfing -> Contained when the prey is swallowed.
    pub(crate) fn update_relations(&mut self, dt: f32) {
        let mut captures: Vec<usize> = Vec::new();
        let mut drops: Vec<usize> = Vec::new();

        for i in 0..self.rels.len() {
            let (pred, prey) = (self.rels[i].pred, self.rels[i].prey);
            if !self.blobs[pred].alive || !self.blobs[prey].alive {
                drops.push(i); // dead participant; drop silently
                continue;
            }
            let RelState::Engulfing { t } = self.rels[i].state else {
                continue;
            };
            let t = t + dt;

            // fraction of the prey membrane inside the predator polygon
            let (ps, pc) = (self.blobs[pred].start, self.blobs[pred].count);
            let (qs, qc) = (self.blobs[prey].start, self.blobs[prey].count);
            let mut inside = 0usize;
            for k in 0..qc {
                if poly_query(&self.pos[ps..ps + pc], self.pos[qs + k]).inside {
                    inside += 1;
                }
            }
            let frac = inside as f32 / qc as f32;

            if frac >= self.params.capture_fraction {
                captures.push(i);
            } else if t > self.params.engulf_timeout {
                drops.push(i);
                self.events.push(Event::EngulfAborted { pred, prey });
            } else {
                self.rels[i].state = RelState::Engulfing { t };
            }
        }

        for &i in &captures {
            let (pred, prey) = (self.rels[i].pred, self.rels[i].prey);
            self.rels[i].state = RelState::Contained;
            self.blobs[prey].contained_by = Some(pred);
            self.events.push(Event::Captured { pred, prey });
        }
        drops.sort_unstable_by(|a, b| b.cmp(a));
        for i in drops {
            self.rels.swap_remove(i);
        }
    }
}
