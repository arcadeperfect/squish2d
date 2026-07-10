use glam::Vec2;

/// Result of querying a point against a ring polygon.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PolyHit {
    pub inside: bool,
    /// Closest point on the polygon boundary.
    pub point: Vec2,
    /// Distance from the query point to `point`.
    pub dist: f32,
    /// Index of the edge's first vertex (relative to the ring slice).
    pub edge: usize,
    /// Position along that edge in [0, 1].
    pub t: f32,
}

/// Closest boundary point + even-odd inside test, in one pass over the ring.
pub(crate) fn poly_query(ring: &[Vec2], p: Vec2) -> PolyHit {
    let n = ring.len();
    let mut inside = false;
    let mut best = PolyHit {
        inside: false,
        point: ring[0],
        dist: f32::MAX,
        edge: 0,
        t: 0.0,
    };
    for i in 0..n {
        let a = ring[i];
        let b = ring[(i + 1) % n];
        // even-odd ray cast along +x
        if (a.y > p.y) != (b.y > p.y) {
            let x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if p.x < x {
                inside = !inside;
            }
        }
        let ab = b - a;
        let len2 = ab.length_squared();
        let t = if len2 > 1e-12 {
            ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = a + ab * t;
        let d = p.distance(q);
        if d < best.dist {
            best.point = q;
            best.dist = d;
            best.edge = i;
            best.t = t;
        }
    }
    best.inside = inside;
    best
}

/// Signed polygon area (positive for the winding used by `spawn_blob`).
pub(crate) fn polygon_area(ring: &[Vec2]) -> f32 {
    let n = ring.len();
    let mut a = 0.0;
    for i in 0..n {
        a += ring[i].perp_dot(ring[(i + 1) % n]);
    }
    0.5 * a
}

/// Least-squares rigid fit of `rest` onto `ring`: returns (centroid, rotation).
/// Closed form for 2D shape matching (Müller et al. 2005).
pub(crate) fn fit_frame(ring: &[Vec2], rest: &[Vec2]) -> (Vec2, f32) {
    let n = ring.len();
    let mut cen = Vec2::ZERO;
    for p in ring {
        cen += *p;
    }
    cen /= n as f32;
    let mut sn = 0.0f32;
    let mut cs = 0.0f32;
    for i in 0..n {
        let q = rest[i];
        let r = ring[i] - cen;
        sn += q.perp_dot(r);
        cs += q.dot(r);
    }
    (cen, sn.atan2(cs))
}
