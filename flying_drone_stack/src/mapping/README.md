# Mapping — 3D Occupancy Grid + Camera Keyframes + VO + Loop Closure SLAM

## 1. Overview

The `mapping` module builds a 3D model of the environment from onboard sensors and
camera images, and integrates camera pose estimates into the MEKF state.  It provides
four components:

| Component | File | Purpose |
|-----------|------|---------|
| **OccupancyMap** | `occupancy.rs` | Sparse probabilistic 3D voxel grid updated from the Multi-ranger Deck 6-axis range sensors |
| **KeyframeStore** | `keyframe.rs` | Rolling camera keyframe buffer with BRIEF feature matching, essential-matrix visual odometry, and loop closure detection |
| **VoTrajectory** | `vo_trajectory.rs` | Integrates consecutive `KeyframeResult` relative poses into a global VO position estimate; feeds `mekf_update_vo` |
| **PoseGraph** | `loop_closure.rs` | Sparse XY pose graph with Gauss-Seidel optimisation; corrects long-term VO drift via loop closure constraints |

The four components work together in the main loop: `OccupancyMap` provides
collision-safe setpoints; `KeyframeStore` produces relative pose estimates from
consecutive camera frames and detects loop closures; `VoTrajectory` accumulates those
estimates and fuses them with the MEKF; `PoseGraph` applies loop closure corrections
to eliminate accumulated drift.  Each can also be used independently.

---

## 2. Mathematical Foundation

### 2.1 Log-Odds Occupancy Grid

Each voxel in the map stores a **log-odds belief** `l = log p/(1−p)`:

```
l > 0   →  probably occupied
l < 0   →  probably free
l = 0 or absent  →  unknown
```

**Update rule** per ray observation:

```
If voxel is on the ray path (free evidence):
    l ← clamp(l + LOG_ODDS_FREE, LOG_ODDS_MIN, LOG_ODDS_MAX)

If voxel is at the ray endpoint (occupied evidence):
    l ← clamp(l + LOG_ODDS_OCC,  LOG_ODDS_MIN, LOG_ODDS_MAX)
```

Clamping at `±LOG_ODDS_MAX = ±3.0` (corresponding to 95% / 5% probability) prevents
over-confidence — after 10 consistent free-space observations a voxel cannot be locked
at 0% occupied, allowing the map to recover from outlier readings.

**Frontier detection**: A voxel is a *frontier* if it is free (`l < FRONTIER_FREE_THRESH`)
and has at least one 6-connected neighbour that is *unknown* (absent from the map).
Frontiers are the targets for autonomous exploration.

### 2.2 Ray Casting

For each valid range reading from sensor direction `d_body` (body frame):

```
1. Rotate body direction to world frame:  d_world = R(roll, pitch, yaw) · d_body
2. For step = 0, VOXEL_SIZE_M, 2·VOXEL_SIZE_M, ..., range − VOXEL_SIZE_M:
       free-evidence update at  pos + step · d_world
3. Occupied-evidence update at  pos + range · d_world  (endpoint)
```

The attitude rotation `R(roll, pitch, yaw)` uses the ZYX Euler → quaternion → rotation
matrix path, matching the convention used in the MEKF and firmware.

**Six sensor directions** (body frame):

| Sensor | Body axis | CRTP variable |
|--------|-----------|---------------|
| Front  | +X | `range.front` |
| Back   | −X | `range.back`  |
| Left   | +Y | `range.left`  |
| Right  | −Y | `range.right` |
| Up     | +Z | `range.up`    |
| Down   | −Z | `range.zrange` (Flow Deck) |

### 2.3 Normalized 8-Point Essential Matrix

When a new camera keyframe is accepted, the `KeyframeStore` estimates the relative
camera pose using the **normalized 8-point algorithm** (Hartley & Zisserman, 2004):

```
1. Match features between keyframes A and B
       For each feature in A, find the nearest neighbour in B by Hamming distance.
       Reject matches where best_dist ≥ 0.75 × second_best_dist (Lowe ratio test).

2. Hartley normalisation: translate by centroid, scale so mean distance = √2.
       This dramatically improves numerical conditioning.

3. Build AᵀA (9×9) from the epipolar constraint:
       Row of A = [u'u, u'v, u', v'u, v'v, v', u, v, 1]
       for each matched pair (u,v) ↔ (u',v').

4. Find the null space of A = smallest eigenvector of AᵀA
       → Jacobi eigendecomposition of the 9×9 symmetric matrix.

5. Reshape to 3×3 matrix F (fundamental matrix).
       Denormalize: F = Tb^T · F_norm · Ta

6. Enforce rank-2 constraint: SVD(F), set σ₃ = 0, recompose.
       → Jacobi eigendecomposition of FᵀF (3×3), zero smallest singular value.

7. E = Kᵀ · F · K  (essential matrix from fundamental matrix + intrinsics K)

8. Decompose E → {R, t}:
       SVD(E) → U, S, Vᵀ
       R₁ = U·W·Vᵀ,  R₂ = U·Wᵀ·Vᵀ   (W = [[0,-1,0],[1,0,0],[0,0,1]])
       t  = ±U[:,2]
       Four candidate solutions; cheirality test selects the unique valid one.

9. Scale t by (range_zA + range_zB) / 2   (metric scale from ToF altitude)
```

**References**:
- Hartley, R. & Zisserman, A. (2004). *Multiple View Geometry in Computer Vision*, 2nd ed.
- Lowe, D. (2004). Distinctive image features from scale-invariant keypoints. *IJCV* 60(2).

### 2.4 VO Pose Chain

`VoTrajectory` integrates the relative poses from `KeyframeResult` into a running
world-frame position estimate using a **passive (coordinate-transform)** rotation
convention.

**Camera-to-body remapping**: the HiMax camera is mounted looking forward-down.
The axis convention is:

```
camera +Z (optical axis) → body +X (forward)
camera +X (image right)  → body +Y (left)
camera +Y (image down)   → body -Z (down)
```

Applied to each relative translation before accumulation:
```
t_body.x = t_cam.z
t_body.y = t_cam.x
t_body.z = −t_cam.y
```

**Orientation chain**: let `R_{world←k}` be the rotation that maps camera-k vectors
to world-frame vectors.  When a new keyframe k+1 arrives with relative rotation
`R_{k+1←k}` (i.e. `v_k = R · v_{k+1}` — passive convention):

```
R_{world←k+1} = R_{world←k} · R_{k+1←k}^T
```

Implemented as `mat3_mul_mat3_bt(old_R, result.rotation)` where `_bt` is
`A·Bᵀ`.

**World-frame translation**:
```
t_world = R_{world←k} · t_body
position += t_world
```

**Drift uncertainty**: sigma grows proportionally to total translation:
```
sigma_xy ← min(sigma_xy + VO_DRIFT_FRAC × |t_body|, VO_MAX_SIGMA)
```

When `sigma_xy < VO_MAX_SIGMA × 0.9`, the estimate is fused with the MEKF via
two scalar position updates (X then Y) with noise `r_vo = sigma_xy²`.  After fusion
`reset_to(mekf_pos)` re-anchors the VO trajectory to prevent accumulated drift from
compounding on the next keyframe.  `sigma_xy` is **not** reset on `reset_to` — the
position is corrected but the uncertainty about future VO drift is unchanged.  Only
`seed()` (full re-initialisation at start of flight) resets sigma to zero.

### 2.5 Loop Closure Detection (`KeyframeStore::detect_loop`)

After each new keyframe is accepted, `detect_loop(new_kf_index)` searches the rolling
buffer for candidate frames that are:

1. **Spatially close** — within `LOOP_SEARCH_RADIUS_M = 1.5 m` (XY only).
2. **Temporally distant** — at least `LOOP_MIN_AGE = 5` positions older in the buffer,
   so the immediate predecessor is not matched.
3. **Visually matched** — at least `LOOP_MIN_MATCHES = 12` feature matches after the
   ratio test.
4. **Geometrically consistent** — at least `LOOP_MIN_INLIERS = 8` essential-matrix inliers.

The returned `LoopConstraint` contains the world-frame displacement from the old frame
to the new one, derived by the same camera-to-body and body-to-world transforms used in
`VoTrajectory::integrate`.

#### Spatial hash grid — O(1) candidate lookup

Candidate retrieval uses a `SpatialGrid` (sparse `HashMap<(i32,i32), Vec<usize>>`).
Cell size equals `LOOP_SEARCH_RADIUS_M = 1.5 m` so the **3×3 neighbourhood** of the
query cell contains every candidate within the search radius:

```
If |C.x − Q.x| < R and |C.y − Q.y| < R
   ⟹ |floor(C.x/R) − floor(Q.x/R)| ≤ 1  (same for y)
   ⟹ C's cell is in the 3×3 neighbourhood of Q's cell  ✓
```

At most 9 cells × ~25 keyframes/cell = 225 distance checks regardless of flight
length, versus O(N) for a linear scan.  Diagonal-corner false positives (up to
R√2 ≈ 2.1 m) are eliminated by an exact squared-distance check before feature
matching.

### 2.6 Pose-Graph Optimisation (`PoseGraph`)

`PoseGraph` maintains a sparse 2-D graph of XY nodes (one per accepted keyframe):

- **Sequential edges** (weight `W_SEQ = 100`) are added automatically when a node is
  registered; the delta is the MEKF XY position difference at registration time.
- **Loop edges** (weight `10 × inlier_count`, typically 80–200) are added when a loop
  closure is detected.

The optimiser runs `PG_ITERATIONS = 100` sweeps of Gauss-Seidel:

```
For each non-anchor slot s (slot 0 is fixed):
    num = Σ_{from_s→j} w·(x_j − Δ)  +  Σ_{i→to_s} w·(x_i + Δ)
    x_s ← num / Σ w
```

Convergence is guaranteed for any connected graph with a fixed anchor.
When the corrected latest-node position moves by more than 1 mm, `optimize()` returns
the new position which is applied to the MEKF via `mekf_update_vo` with tight noise
`R_LOOP = 0.01 m²` (10 cm σ).

---

## 3. Architecture Diagram

```
            Multi-ranger Deck (6 directions)
            + Flight Deck down-range (range.zrange)
            + EKF attitude (roll, pitch, yaw)
                          │
                          ▼
              ┌────────────────────────┐
              │   OccupancyMap         │
              │   .update()            │  ← called every 50 ms in main loop
              │   ray-cast 6 sensors   │
              │   log-odds voxel grid  │
              │   .frontiers() →       │──────────► ExplorationPlanner
              │   .to_ply() →          │──────────► PLY file (MeshLab)
              └────────────────────────┘

            AI Deck CPX camera
            (JPEG frames ~20-30 Hz)
            + EKF position + yaw + range_z
                          │
                          ▼
              ┌────────────────────────┐
              │   KeyframeStore        │
              │   .push()              │  ← called per received frame
              │   pose change check    │
              │   FAST-9 + BRIEF       │
              │   feature matching     │
              │   normalized 8-point   │
              │   E → R, t (metric)    │
              │   → KeyframeResult     │
              │   .detect_loop()       │──────────► LoopConstraint
              └────────────────────────┘                    │
                          │                                 ▼
                          ▼                   ┌────────────────────────┐
              ┌────────────────────────┐      │   PoseGraph            │
              │   VoTrajectory         │      │   .add_node()          │
              │   .integrate()         │      │   .add_loop()          │
              │   position, sigma_xy   │      │   .optimize()          │
              │   chi² gate            │      │   Gauss-Seidel 100×    │
              │   → vo_pos: Vec3       │──┐   │   → corrected XY       │
              └────────────────────────┘  │   └────────────────────────┘
                                          │                 │
                                          └────────────────►▼
                                                mekf_update_vo(x, y)
                                                (estimation/mekf.rs)

OccupancyMap and KeyframeStore/VoTrajectory/PoseGraph are independent — either can run
without the other.
```

---

## 4. Key Data Types

### `OccupancyMap` — `src/mapping/occupancy.rs`

```rust
pub struct OccupancyMap { /* sparse HashMap<(i16,i16,i16), f32> */ }
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new()` | `→ Self` | Create empty map |
| `update(pos, roll, pitch, yaw, front, back, left, right, up, down)` | `→ ()` | Ray-cast one timestep |
| `frontiers()` | `→ Vec<Vec3>` | Free cells with unknown neighbours |
| `is_occupied(key)` | `→ bool` | True if log-odds > 0 |
| `stats()` | `→ MapStats` | Total / occupied / free voxel counts |
| `to_ply()` | `→ Vec<u8>` | ASCII PLY of occupied voxels only |
| `to_ply_full()` | `→ Vec<u8>` | ASCII PLY of all voxels with log-odds colour |

### `MapStats` — `src/mapping/occupancy.rs`

| Field | Type | Description |
|-------|------|-------------|
| `n_total` | `usize` | All voxels in the map (occupied + free) |
| `n_occupied` | `usize` | Voxels with log-odds > `OCCUPIED_THRESH` |
| `n_free` | `usize` | Voxels with log-odds < `FREE_THRESH` |

### `KeyframeStore` — `src/mapping/keyframe.rs`

```rust
pub struct KeyframeStore { /* rolling Vec<Keyframe>, intrinsics: CameraIntrinsics */ }
```

| Method | Description |
|--------|-------------|
| `new()` | HiMax HM01B0 default intrinsics |
| `with_intrinsics(k)` | Custom camera matrix |
| `push(frame, pos, yaw_deg, range_z)` | Accept frame if pose changed; returns `Option<KeyframeResult>` |
| `len()`, `is_empty()`, `get(idx)` | Buffer inspection |

### `KeyframeResult` — `src/mapping/keyframe.rs`

| Field | Type | Description |
|-------|------|-------------|
| `kf_index` | `usize` | Index of the newly accepted keyframe |
| `match_count` | `usize` | Feature pairs that passed ratio test |
| `rotation` | `[[f32;3];3]` | Relative rotation R (current ← previous) |
| `translation_dir` | `Vec3` | Unit-length translation direction |
| `translation_m` | `Vec3` | Metric translation (scaled by average range_z) |
| `inlier_count` | `usize` | Matches with positive cheirality |

### `VoTrajectory` — `src/mapping/vo_trajectory.rs`

```rust
pub struct VoTrajectory {
    pub position:    Vec3,           // accumulated world-frame position
    pub kf_count:    usize,          // successful integrations so far
    pub sigma_xy:    f32,            // current XY uncertainty (m)
    orientation:     [[f32;3];3],    // R_{world ← current_camera}
    initialized:     bool,
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new()` | `→ Self` | Identity orientation, zero position, uninitialised |
| `seed(pos)` | `→ ()` | Set initial world position (called once MEKF is trusted) |
| `integrate(result)` | `→ Option<Vec3>` | Accept `KeyframeResult`; returns new world position or `None` if rejected |
| `reset_to(pos)` | `→ ()` | Re-anchor to `pos` after MEKF fusion; **preserves `sigma_xy`** (position-only correction — drift uncertainty is unaffected by reseeding) |

**Rejection criteria** (inside `integrate`):
- `result.inlier_count < MIN_INLIERS` (6) → noisy essential matrix
- `|translation_m| < MIN_TRANSLATION_M` (0.05 m) → nearly pure rotation, scale unreliable

### `LoopConstraint` — `src/mapping/loop_closure.rs`

| Field | Type | Description |
|-------|------|-------------|
| `from_idx` | `usize` | `kf_index` of the older keyframe |
| `to_idx` | `usize` | `kf_index` of the newer keyframe |
| `translation_world` | `[f32; 2]` | World-frame displacement from→to [m] |
| `inlier_count` | `usize` | Essential-matrix inliers used to compute the displacement |

### `PoseGraph` — `src/mapping/loop_closure.rs`

```rust
pub struct PoseGraph {
    kf_indices:  Vec<usize>,                   // kf_index for each slot
    positions:   [[f32;2]; PG_MAX_NODES],       // XY per slot
    n_nodes:     usize,
    seq_edges:   Vec<(usize,usize,[f32;2],f32)>,// (from,to,delta,w)
    loop_edges:  Vec<(usize,usize,[f32;2],f32)>,
    pub lc_count: usize,
}
```

| Method | Signature | Description |
|--------|-----------|-------------|
| `new()` | `→ Self` | Empty graph |
| `add_node(kf_idx, xy)` | `→ ()` | Register node; auto-adds sequential edge from previous |
| `add_loop(lc)` | `→ ()` | Add loop edge; silently ignored if from/to not registered |
| `optimize()` | `→ Option<[f32;2]>` | Run Gauss-Seidel; returns corrected latest-node XY if moved > 1 mm |
| `latest_pos()` | `→ Option<[f32;2]>` | XY of most recently registered node |

---

## 5. Algorithm Walkthrough

### OccupancyMap::update

**Entry point:** `src/mapping/occupancy.rs` → `update()`.

```
update(pos, roll, pitch, yaw, front, back, left, right, up, down)
  │
  ├─ rpy_to_quat(roll, pitch, yaw) → q
  │
  ├─ For each of the 6 sensor directions d_body:
  │     d_world = q.rotate(d_body)            ← rotate body axis to world
  │     if range is Some(r) and MIN_RANGE < r < MAX_RANGE:
  │         cast_ray(pos, d_world, r)
  │
  └─ cast_ray(origin, direction, length):
        step = VOXEL_SIZE_M (= 0.05 m)
        for t in [0, step, 2·step, ..., length - step]:
            mark voxel at origin + t·direction as FREE
        mark voxel at origin + length·direction as OCCUPIED
```

### KeyframeStore::push

**Entry point:** `src/mapping/keyframe.rs` → `push()`.

```
push(frame, pos, yaw_deg, range_z)
  │
  ├─ Pose change check:
  │     dist = |pos - last_kf.pos|
  │     dyaw = |yaw_deg - last_kf.yaw_deg| (shortest angle)
  │     if dist < KF_MIN_DIST_M AND dyaw < KF_MIN_YAW_DEG → return None
  │
  ├─ detect_features(frame, FAST_THRESHOLD) → Vec<Feature>
  │
  ├─ match_features(prev_kf.features, new_features)
  │     brute-force Hamming NN with Lowe ratio test
  │     → Vec<FeatureMatch>
  │
  ├─ if match_count >= MIN_MATCHES_FOR_ESSENTIAL (8):
  │     estimate_essential(prev_feats, curr_feats, matches, intrinsics)
  │       → (R: [[f32;3];3], t: Vec3, inlier_count: usize)
  │
  ├─ scale = (range_zA + range_zB) / 2
  │   translation_m = t · scale
  │
  └─ return Some(KeyframeResult { ... })
```

### VoTrajectory::integrate

**Entry point:** `src/mapping/vo_trajectory.rs` → `integrate()`.

```
integrate(result: &KeyframeResult) → Option<Vec3>
  │
  ├─ Rejection gates:
  │     if result.inlier_count < MIN_INLIERS → return None
  │     if |result.translation_m| < MIN_TRANSLATION_M → return None
  │
  ├─ Camera-to-body remapping:
  │     t_body = (t_cam.z, t_cam.x, -t_cam.y)     [cam +Z → body +X, etc.]
  │
  ├─ Rotate to world frame:
  │     t_world = self.orientation · t_body
  │
  ├─ Accumulate position:
  │     self.position += t_world
  │
  ├─ Update orientation (passive convention):
  │     self.orientation = self.orientation · result.rotation^T
  │                      = mat3_mul_mat3_bt(old_R, result.rotation)
  │
  ├─ Grow drift sigma:
  │     self.sigma_xy = min(sigma_xy + VO_DRIFT_FRAC × |t_body|, VO_MAX_SIGMA)
  │
  ├─ self.kf_count += 1
  │
  └─ return Some(self.position)
```

**Caller pattern** (in `step_perception!` macro, `src/bin/main.rs`):
```
if let Some(vo_pos) = vo_traj.integrate(&kf) {
    if vo_traj.sigma_xy < VO_MAX_SIGMA * 0.9 {
        r_vo = vo_traj.sigma_xy² (min R_VO_MIN = 0.04)
        inno_x = vo_pos.x − mekf.x[0]
        inno_y = vo_pos.y − mekf.x[1]
        gate   = (r_vo + sigma[0][0]) × 9.0          ← chi² gate (3σ)
        if inno_x² + inno_y² < gate:
            mekf_update_vo(&mut mekf, [vo_pos.x, vo_pos.y], r_vo)
            vo_traj.reset_to(mekf_pos)               ← re-anchor after fusion
    }
}
```

---

## 6. Constants & Tuning

### OccupancyMap constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `VOXEL_SIZE_M` | 0.05 m | Spatial resolution (5 cm) |
| `LOG_ODDS_OCC` | 0.7 | Positive increment per occupied observation |
| `LOG_ODDS_FREE` | −0.4 | Negative increment per free observation |
| `LOG_ODDS_MIN` | −3.0 | Saturation (95% free confidence) |
| `LOG_ODDS_MAX` | 3.0 | Saturation (95% occupied confidence) |
| `OCCUPIED_THRESH` | 0.0 | Decision boundary for `is_occupied` |
| `FRONTIER_FREE_THRESH` | −0.1 | Log-odds below which a voxel is "free" for frontier purposes |
| `MAX_RANGE_M` | 4.0 m | VL53L1x sensor saturation limit |
| `MIN_RANGE_M` | 0.02 m | VL53L1x minimum reliable distance |

**Tuning notes**:
- `LOG_ODDS_OCC > |LOG_ODDS_FREE|` means occupied evidence outweighs free.
  This is deliberate: a single hit should outweigh 1.75 free passes, making the map
  slightly conservative (useful for collision avoidance).
- Increase `VOXEL_SIZE_M` for faster frontier detection on large spaces.
  Decrease for finer obstacle resolution (memory grows as O(1/r³)).

### KeyframeStore constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `KF_MIN_DIST_M` | 0.15 m | Minimum translation between keyframes |
| `KF_MIN_YAW_DEG` | 30° | Minimum yaw change between keyframes |
| `KF_MAX_STORED` | 50 | Rolling buffer size |
| `FAST_THRESHOLD` | 10 | FAST-9 corner intensity threshold |
| `MATCH_MAX_HAMMING` | 64 | Maximum descriptor distance for a valid match |
| `MATCH_RATIO` | 0.75 | Lowe ratio test threshold |
| `MIN_MATCHES_FOR_ESSENTIAL` | 8 | Minimum matches to attempt E matrix estimation |

**Tuning notes**:
- `KF_MIN_DIST_M = 0.15` gives one keyframe per 15 cm of travel — roughly one every
  2 s at typical circle speed (0.075 m/s). This was halved from 0.30 to double the
  keyframe rate given the AI Deck's limited streaming window (~13–70 s before Nina reboots).
- `MATCH_MAX_HAMMING = 64` means ≤25% of BRIEF bits differ. For low-texture scenes
  this may need to be raised to 80–96.
- `KF_MAX_STORED = 50` caps memory at ≈50 × (5 KB image + feature list) ≈ 250 KB.

### VoTrajectory constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `VO_MAX_SIGMA` | 0.50 m | Maximum allowed XY uncertainty before VO fusion is suppressed |
| `VO_DRIFT_FRAC` | 0.10 | Sigma grows by 10% of each step's translation magnitude |
| `MIN_INLIERS` | 6 | Minimum cheirality-valid matches for a `KeyframeResult` to be accepted |
| `MIN_TRANSLATION_M` | 0.05 m | Minimum metric translation to accept (avoids degenerate pure-rotation steps) |
| `R_VO_MIN` | 0.04 m² | Floor on `r_vo = sigma_xy²` in MEKF update (prevents over-confident fusion) |

**Tuning notes**:
- `VO_DRIFT_FRAC = 0.10` means after 5 m of travel (without re-seeding) sigma reaches
  0.50 m and VO fusion stops. On typical short flights (< 3 m) sigma stays below 0.30 m
  and VO fuses continuously.
- Increase `MIN_INLIERS` to 8–10 for noisier scenes; decrease to 4 for very sparse
  indoor environments (few features). Values below 5 risk unreliable R,t estimates.
- `MIN_TRANSLATION_M = 0.05` skips steps where the drone is nearly stationary — pure
  rotations produce a degenerate essential matrix with large translation uncertainty.

### Loop closure + pose-graph constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `LOOP_SEARCH_RADIUS_M` | 1.5 m | XY radius for loop candidate search |
| `LOOP_MIN_AGE` | 5 | Buffer positions between new KF and oldest candidate |
| `LOOP_MIN_MATCHES` | 12 | Minimum ratio-test matches to attempt essential matrix |
| `LOOP_MIN_INLIERS` | 8 | Minimum cheirality inliers to accept loop |
| `LOOP_WEIGHT_FACTOR` | 10.0 | Loop edge weight = `10 × inlier_count` |
| `W_SEQ` | 100.0 | Sequential edge weight (≈10 cm σ per step) |
| `PG_ITERATIONS` | 100 | Gauss-Seidel sweeps per `optimize()` call |
| `PG_MAX_NODES` | 60 | Maximum nodes (headroom above KF_MAX_STORED = 50) |
| `R_LOOP` | 0.01 m² | MEKF noise for pose-graph correction (10 cm σ) |

**Tuning notes**:
- At 12 inliers, loop weight = 120. On a 3-node graph with two sequential edges
  (each w=100), a single loop edge corrects ≈50% of the drift. More nodes distribute
  the correction along the chain — the desired behaviour for long trajectories.
- Increase `LOOP_SEARCH_RADIUS_M` if the drone covers a large area; decrease it to
  reduce false-positive loop closures in feature-poor environments.
- `LOOP_MIN_AGE = 5` means the drone must travel ≈0.75 m (5 × 0.15 m keyframe spacing)
  before a location can be re-detected — prevents the previous frame from matching itself.

---

## 7. Connections to Other Modules

| Direction | Module | What is exchanged |
|-----------|--------|------------------|
| Consumes | `perception/sensors/crtp.rs` | `MultiRangeMeasurement` → `update()` arguments |
| Consumes | `perception/sensors/cpx.rs` | `ImageFrame` → `push()` argument |
| Consumes | `perception/processing/features.rs` | `detect_features()` called inside `push()` |
| Consumes | `estimation/mekf.rs` | EKF pos/attitude used as map update pose |
| Produces for | `planning/exploration.rs` | `OccupancyMap::frontiers()` drives frontier selection |
| Produces for | `bin/build_map.rs` | `to_ply()` / `to_ply_full()` → offline visualisation |
| Produces for | `bin/main.rs` | `KeyframeResult` → `VoTrajectory::integrate` → `mekf_update_vo` |
| Produces for | `estimation/mekf.rs` | `VoTrajectory` calls `mekf_update_vo(x, y, r_vo)` for XY correction |

---

## 8. Common Pitfalls

**Do not update the map from ground readings**: Range sensors at < MIN_RANGE_M
(< 2 cm) are excluded automatically, but rows where `range_z < 0.10 m` (drone still
on ground) should be filtered by the caller (as `build_map.rs` does with `--min-height`).
Feeding ground rows marks large spherical blobs of occupied voxels at the origin.

**Multi-ranger gaps are normal**: The VL53L1x saturates at 4.0 m and returns
`0xFFFF` (65535 mm) when out of range. `CrtpMultiRangeAdapter` maps this to `None`.
The `update()` call skips `None` sensors — so gaps in the CSV are expected and correct.

**Essential matrix scale ambiguity**: The 8-point algorithm recovers translation
**direction only** (unit vector). Metric scale is recovered by multiplying by the average
`range_z` of the two keyframes. This assumes the camera optical axis is close to
vertical (reasonable for a downward-facing or horizontally-mounted camera on a level
drone). For highly tilted flight the scale estimate will be off.

**Frontier height band**: `ExplorationPlanner::nearest_frontier` filters by a ±0.25 m
height band around `hover_z`. The `OccupancyMap` stores 3D frontiers but the planner
only uses a horizontal slice. This is intentional (the drone flies at constant altitude).

**Jacobi SVD convergence**: The 9×9 Jacobi in `keyframe.rs` uses 500 iterations with
a 1×10⁻⁸ off-diagonal convergence threshold. For degenerate scenes (planar structures,
all features co-planar) convergence may be slow and the essential matrix unreliable.
The `inlier_count` field of `KeyframeResult` is the best indicator of reliability —
prefer keyframe estimates where `inlier_count ≥ 6`.

**First keyframe returns None**: The first call to `push()` always returns `None`
(no predecessor to compare against). The second call, if far enough, returns the
first `KeyframeResult`.

**VO seed timing**: `VoTrajectory::seed()` must be called before the first
`integrate()`, otherwise all calls return `None`. In `main.rs` the seed is deferred
until `mekf_seeded` is true (first valid range_z reading). Do not seed from a
pre-flight MEKF state — the position estimate at takeoff is unreliable.

**Camera mount calibration**: the axis remapping (`body_x=cam_z, body_y=cam_x,
body_z=-cam_y`) assumes the HiMax camera is mounted pointing **forward-down** with
`+Z` (optical axis) facing drone `+X` (forward). If the camera is rotated (e.g. tilted
or in a different port), the rotation table in `vo_trajectory.rs::integrate` must be
updated. Wrong mounting = odometry in the wrong direction, visible as VO diverging
away from the MEKF trajectory.

**Pure-rotation rejection**: `MIN_TRANSLATION_M = 0.05 m` prevents accepting
keyframes during pure yaw or hover. These steps produce a valid `rotation` but the
`translation_m` magnitude is near zero, making metric scale recovery via `range_z`
unreliable. When the drone is hovering in place, expect `integrate()` to return `None`
every cycle — this is correct behaviour.

**One-cycle CSV lag**: the `pending_vo` tuple in `main.rs` carries VO values into the
**next** CSV row. The row written by `fw_logging_step` records the VO state from the
*previous* control cycle. This is a deliberate design choice (logging fires before VO
is computed in the same cycle) and the lag is one 50 ms step — negligible for analysis.
The `pending_pg` tuple carries the same one-cycle lag for pose-graph columns.

**Frame eviction and kf_index stability**: `KeyframeStore` evicts the oldest frame once
more than `KF_MAX_STORED = 50` keyframes are stored.  After eviction, slot indices into
`self.frames` are no longer stable.  The `kf_index` field on `Keyframe` is a separate
monotonic counter that survives eviction — `PoseGraph::slot_of()` uses it to look up
nodes by stable identity.

**First-keyframe detect_loop returns None**: `detect_loop` checks
`self.frames.len() <= LOOP_MIN_AGE` before searching. For the first 5 accepted
keyframes it always returns `None`. This is correct and expected.

**Yaw-only world-frame remap in detect_loop**: the world-frame translation is computed
using only the new keyframe's `yaw_deg` (not roll/pitch). For near-level hover flights
roll and pitch are small (< 5°), so the error introduced is < 1% of the translation
magnitude. If the drone flies significantly tilted, a full 3D rotation should be used.

---

## 9. Usage Examples

### Analyse loop closure and VO drift offline

```bash
# Analyse a Phase 6 flight log (49 columns)
cargo run --release --bin slam_eval -- runs/circle_2026-03-21_10-00-00.csv

# Output includes:
#   Total loop closures detected
#   VO drift stats (mean/max sigma_xy, gate-out rows)
#   Per-loop-event: correction magnitude (mekf vs pg position)
#   RMSE between mekf_x/y and pg_x/y trajectories
```

### Build a 3D map offline from flight CSV logs

```bash
# Single flight
cargo run --release --bin build_map -- runs/hover_2026-03-17_19-33-08.csv

# Multiple flights, custom output path
cargo run --release --bin build_map -- runs/*.csv --out results/data/combined_map.ply

# Include free voxels (for confidence visualization)
cargo run --release --bin build_map -- runs/circle.csv --full

# Open in MeshLab or CloudCompare
meshlab results/data/map.ply
cloudcompare results/data/map.ply
```

### Use OccupancyMap in code

```rust,no_run
use multirotor_simulator::mapping::OccupancyMap;
use multirotor_simulator::math::Vec3;

let mut map = OccupancyMap::new();

// Update from one log row:
map.update(
    Vec3::new(0.5, 0.0, 0.3),   // drone position
    0.0, 2.0, 15.0,             // roll, pitch, yaw (degrees)
    Some(1.2),                   // front range (m)
    None,                        // back (out of range)
    Some(0.8),                   // left range (m)
    None,                        // right (out of range)
    Some(1.5),                   // up range (m)
    Some(0.30),                  // down range (m)
);

let stats = map.stats();
println!("{} occupied voxels, {} free", stats.n_occupied, stats.n_free);

// Get exploration targets:
let frontiers = map.frontiers();
println!("{} frontiers available", frontiers.len());

// Export point cloud:
std::fs::write("map.ply", map.to_ply()).unwrap();
```

### Use KeyframeStore in code

```rust,no_run
use multirotor_simulator::mapping::KeyframeStore;
use multirotor_simulator::math::Vec3;
// frame comes from CpxCamera::recv_frame() or SimCamera::poll()

let mut store = KeyframeStore::new();

// On each received camera frame (20-30 Hz):
// if let Some(result) = store.push(frame, pos, yaw_deg, range_z) {
//     println!("kf#{}: {} matches, t = ({:.3},{:.3},{:.3}) m",
//         result.kf_index, result.match_count,
//         result.translation_m.x, result.translation_m.y, result.translation_m.z);
//     if result.inlier_count >= 6 {
//         // Use result.rotation and result.translation_m for odometry
//     }
// }
```

### Use VoTrajectory for VO fusion

```rust,no_run
use multirotor_simulator::mapping::VoTrajectory;
use multirotor_simulator::estimation::mekf::{MekfState, mekf_update_vo};
use multirotor_simulator::mapping::vo_trajectory::VO_MAX_SIGMA;
use multirotor_simulator::math::Vec3;

let mut vo = VoTrajectory::new();
let mut mekf: MekfState = /* ... */;

// Once MEKF is trusted (first valid range reading):
vo.seed(Vec3::new(mekf.x[0], mekf.x[1], mekf.x[2]));

// On each KeyframeResult from KeyframeStore::push():
// if let Some(kf_result) = store.push(frame, pos, yaw_deg, range_z) {
//     if let Some(vo_pos) = vo.integrate(&kf_result) {
//         if vo.sigma_xy < VO_MAX_SIGMA * 0.9 {
//             let r_vo = vo.sigma_xy.powi(2).max(0.04);
//             let inno_x = vo_pos.x - mekf.x[0];
//             let inno_y = vo_pos.y - mekf.x[1];
//             let gate = (r_vo + mekf.sigma.data[0][0]).max(0.01) * 9.0;
//             if inno_x * inno_x + inno_y * inno_y < gate {
//                 let (px, py) = (vo_pos.x, vo_pos.y);
//                 mekf_update_vo(&mut mekf, [px, py], r_vo);
//                 vo.reset_to(Vec3::new(mekf.x[0], mekf.x[1], mekf.x[2]));
//             }
//         }
//         println!("VO pos: ({:.2}, {:.2}), sigma: {:.3}", vo_pos.x, vo_pos.y, vo.sigma_xy);
//     }
// }
```

---

## 10. Related Tests

| Test location | What it covers |
|---------------|---------------|
| `src/mapping/occupancy.rs` (inline, 9 tests) | Ray casting, log-odds saturation, frontier detection, PLY export, stats, voxel key round-trip |
| `src/mapping/keyframe.rs` (inline, 12 tests) | Hamming distance, keyframe acceptance/rejection, yaw trigger, SVD correctness, angle wrapping, rank-2 enforcement, detect_loop age gate, detect_loop spatial gate |
| `src/mapping/vo_trajectory.rs` (inline, 12 tests) | Seed/init, inlier rejection, small-translation rejection, cam→body axis mapping (+Z→+X, +X→+Y), two-step accumulation, orientation rotation, sigma growth, sigma cap, reset, kf_count tracking |
| `src/mapping/loop_closure.rs` (inline, 8 tests) | Empty/single-node graph, anchor fixed, sequential-only no-change, loop corrects drift, weight-pull relationship, lc_count, latest_pos |
| `src/bin/build_map.rs` | Integration: replay 3 real flight CSVs → 770 occupied voxels (visual inspection) |

### Validation Results (March 2026 flights)

Running `build_map` on three combined hover/circle/figure-8 flights:

```
Loaded: runs/hover_2026-03-17_19-33-08.csv  (484 rows, 484 airborne)
Loaded: runs/circle_2026-03-17_19-34-22.csv (812 rows, 812 airborne)
Loaded: runs/figure8_2026-03-17_19-36-11.csv (810 rows, 810 airborne)

Map stats: 10394 total voxels, 770 occupied, 9624 free
Saved: results/data/map.ply (770 points)
```

The 770 occupied voxels form recognisable room surfaces (walls, ceiling, floor) when
opened in MeshLab with `Render → Color → Per Vertex`.
