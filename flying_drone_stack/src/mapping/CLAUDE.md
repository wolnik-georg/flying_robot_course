# mapping/ — SLAM Stack (Occupancy Map, VO, Loop Closure, Pose Graph)

## Components

| File | What it does |
|------|-------------|
| `occupancy.rs` | Sparse log-odds 3D voxel map (5 cm), ray-cast from multi-ranger |
| `keyframe.rs` | Visual keyframes, FAST-9+BRIEF features, 8-pt essential matrix, spatial grid index |
| `vo_trajectory.rs` | Chains relative VO poses into world-frame XY position + sigma |
| `loop_closure.rs` | `LoopConstraint` type + `PoseGraph` Gauss-Seidel optimizer |

## OccupancyMap

```rust
pub struct OccupancyMap { /* sparse HashMap<(i16,i16,i16), f32> */ }

impl OccupancyMap {
    pub fn new() -> Self
    pub fn update(
        &mut self,
        drone_pos: Vec3,
        roll_deg: f32, pitch_deg: f32, yaw_deg: f32,
        // multi-ranger readings (None = out of range / missing)
        front_m: Option<f32>, back_m: Option<f32>,
        left_m: Option<f32>, right_m: Option<f32>,
        up_m: Option<f32>,   down_m: Option<f32>,
    )
    pub fn probe_direction(&self, pos: Vec3, dir: Vec3, radius_m: f32) -> bool  // occupied?
    pub fn stats(&self) -> MapStats
    pub fn save_ply(&self, path: &str) -> Result<(), String>
}
```

Voxel size: 5 cm. Log-odds update: +0.7 (hit), -0.4 (free). Threshold: >0 = occupied.

## KeyframeStore

```rust
pub struct KeyframeStore { /* rolling buffer KF_MAX_STORED=50, SpatialGrid 1.5m cells */ }

impl KeyframeStore {
    pub fn new() -> Self
    // Register new camera frame. Returns VO result if two frames matched.
    pub fn push(
        &mut self,
        frame: ImageFrame,
        pos: Vec3,
        yaw_deg: f32,
        range_z: f32,
    ) -> Option<KeyframeResult>
    pub fn num_keyframes(&self) -> usize
    pub fn get_last_keyframe(&self) -> Option<&Keyframe>
}

pub struct KeyframeResult {
    pub translation_m: Vec3,     // relative translation (body frame)
    pub rotation: Quat,          // relative rotation
    pub scale: f32,              // metric scale from range_z ratio
    pub is_loop_closure: bool,
    pub lc_constraint: Option<LoopConstraint>,
    pub match_count: usize,
    pub inlier_count: usize,
}
```

**Keyframe trigger** (constants in keyframe.rs):
- `KF_MIN_DIST_M = 0.15` — new KF if moved ≥ 0.15 m
- `KF_MIN_YAW_DEG = 30.0` — or rotated ≥ 30°
- `KF_MAX_STORED = 50` — rolling buffer

**Loop closure gates** (constants in keyframe.rs):
- `LOOP_SEARCH_RADIUS_M = 1.5` — spatial gate
- `LOOP_MIN_AGE = 5` — min keyframe index separation
- `LOOP_MIN_MATCHES = 12` — ratio-test filtered feature matches
- `LOOP_MIN_INLIERS = 8` — essential matrix inliers

**SpatialGrid**: O(1) candidate lookup — 3×3 neighbourhood of 1.5m cells.
Replaced O(N) linear scan in Phase 8. Same loop closure results, much faster.

## VoTrajectory

```rust
pub struct VoTrajectory { /* relative pose chain, accumulated world pose, sigma_xy */ }

impl VoTrajectory {
    pub fn new() -> Self
    pub fn push(&mut self, translation_m: Vec3, rotation: Quat)
    pub fn current_position(&self) -> Vec3       // world frame XY position
    pub fn current_orientation(&self) -> Quat
    pub fn sigma_xy(&self) -> f32               // XY position uncertainty [m]
    // Re-seed from MEKF or pose-graph correction.
    // NOTE: does NOT reset sigma_xy (fix: reset_to preserves uncertainty)
    pub fn reset_to(&mut self, pos: Vec3, orient: Quat)
}
```

**sigma_xy** grows as VO drifts. After `reset_to`, it is preserved (NOT zeroed) — this was
a bug that was fixed: sigma should accumulate, not restart from zero on each correction.

**Camera-to-body convention**:
- `body_x = cam_z` (forward)
- `body_y = cam_x` (right)
- `body_z = -cam_y` (down)

## PoseGraph + LoopConstraint

```rust
pub struct LoopConstraint {
    pub kf_idx_a: usize,
    pub kf_idx_b: usize,
    pub translation_m: Vec3,
    pub covariance: [[f32; 3]; 3],
}

pub struct PoseGraph { /* nodes: Vec<(Vec3, Quat)>, sequential + loop edges */ }

impl PoseGraph {
    pub fn new() -> Self
    pub fn add_node(&mut self, pos: Vec3, orient: Quat)
    pub fn add_sequential_edge(&mut self, from: usize, to: usize, trans: Vec3, rot: Quat)
    pub fn add_loop_edge(&mut self, constraint: LoopConstraint)
    pub fn optimize(&mut self, num_sweeps: usize)  // Gauss-Seidel; 100 sweeps typical
    pub fn get_node(&self, idx: usize) -> Option<(Vec3, Quat)>
    pub fn num_nodes(&self) -> usize
}
```

Pose graph constants:
- `W_SEQ = 100` — sequential edge weight
- `LOOP_WEIGHT_FACTOR = 10` — loop constraint weight relative to sequential

## How main.rs uses the SLAM stack

```rust
// Initialisation (before flight loop)
let omap = Arc::new(Mutex::new(OccupancyMap::new()));
let mut kf_store = KeyframeStore::new();
let mut vo_traj = VoTrajectory::new();
let mut pose_graph = PoseGraph::new();

// In the control loop (step_perception! macro):
// 1. Update occupancy map
omap.lock().unwrap().update(pos, roll, pitch, yaw, front, back, left, right, up, None);

// 2. Drain AI Deck frames (async background thread)
if let Some(frame) = ai_frame_rx.try_recv() {
    if let Some(kf_result) = kf_store.push(frame, pos, yaw_deg, range_z) {
        vo_traj.push(kf_result.translation_m, kf_result.rotation);
        let vo_pos = vo_traj.current_position();
        mekf_update_vo(&mut mekf_state, [vo_pos.x, vo_pos.y], R_VO_XY);
        if kf_result.is_loop_closure {
            if let Some(lc) = kf_result.lc_constraint {
                pose_graph.add_loop_edge(lc);
                pose_graph.optimize(100);
                // correct vo_traj from optimized graph node
                if let Some((pg_pos, pg_rot)) = pose_graph.get_node(kf_store.num_keyframes() - 1) {
                    vo_traj.reset_to(pg_pos, pg_rot);
                }
            }
        }
    }
}
```

## Validated flight results

| CSV | KFs | LCs | Max correction | vo_sigma |
|-----|-----|-----|---------------|---------|
| `circle_2026-03-30_18-51-03.csv` | 9 | 12 | 0.24 m | 0.033→0.273 |
| `circle_2026-03-30_18-45-11.csv` | 10 | 0 | — | low texture |

SLAM tiers validated: 0–6. Tier 7 (explore end-to-end) awaiting re-flight after April 7 bug fixes.
