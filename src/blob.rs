use glam::Vec2;

pub type BlobId = usize;

/// Parameters for spawning a blob.
///
/// Compliance is XPBD compliance (inverse stiffness): 0 = hard constraint.
/// Distance-type compliances (edge, bend) are independent of world scale;
/// `area_compliance` scales with (blob radius)^2, so tune it per project.
#[derive(Clone, Copy, Debug)]
pub struct BlobParams {
    pub center: Vec2,
    pub radius: f32,
    /// Membrane particle count (clamped to >= 8).
    pub particle_count: usize,
    /// Total blob mass, distributed uniformly over its particles.
    pub mass: f32,
    /// Membrane edge compliance. 0 = inextensible membrane.
    pub edge_compliance: f32,
    /// Second-neighbour (bending) compliance. Higher = floppier membrane.
    pub bend_compliance: f32,
    /// Area (pressure) compliance. 0 = incompressible; raise it to let blobs
    /// lose volume when crushed against each other.
    pub area_compliance: f32,
    /// 0 = off. Otherwise a per-second rate pulling particles toward the
    /// shape-matched rest shape (gentle identity / shape memory).
    pub shape_memory: f32,
    /// Collision half-thickness of the membrane. Default: half the rest edge length.
    pub particle_radius: Option<f32>,
}

impl BlobParams {
    pub fn new(center: Vec2, radius: f32) -> Self {
        Self {
            center,
            radius,
            particle_count: 32,
            mass: 1.0,
            edge_compliance: 0.0,
            bend_compliance: 5e-4,
            area_compliance: 0.0,
            shape_memory: 0.0,
            particle_radius: None,
        }
    }
}

/// Pose of a blob for rendering decals (faces) on it.
#[derive(Clone, Copy, Debug)]
pub struct BlobFrame {
    /// Centroid of the membrane.
    pub pos: Vec2,
    /// Shape-matched rotation (radians) relative to the spawn orientation.
    pub rot: f32,
    /// sqrt(current area / rest area) — how much the blob has grown/shrunk.
    pub scale: f32,
}

pub(crate) struct Blob {
    pub start: usize,
    pub count: usize,
    pub alive: bool,
    pub params: BlobParams,
    /// Polygon area of the spawn ring (slightly under pi*r^2 for small counts).
    pub rest_area: f32,
    /// Edge / second-neighbour chord lengths of the spawn ring.
    pub base_edge: f32,
    pub base_chord: f32,
    pub particle_radius: f32,
    /// Spawn-ring offsets from the centroid; shape-matching reference.
    pub rest_local: Vec<Vec2>,
    /// User knob: desired area = rest_area * area_scale (plus swell from prey).
    pub area_scale: f32,
    /// Extra area contributed by contained prey (recomputed every step).
    pub swell: f32,
    /// Smoothed working target the solver actually enforces.
    pub area_target: f32,
    /// Polygon area measured after the last step.
    pub current_area: f32,
    pub contained_by: Option<BlobId>,
    /// Uniform acceleration applied to the blob's particles (locomotion/AI).
    pub locomotion: Vec2,
    pub frame_pos: Vec2,
    pub frame_rot: f32,
}
