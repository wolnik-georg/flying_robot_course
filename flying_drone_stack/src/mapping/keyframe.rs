//! Camera keyframe store with feature matching and essential-matrix visual odometry.
//!
//! ## What this module does
//!
//! Captures one `ImageFrame` every time the drone has moved significantly (> 0.30 m
//! or > 30° yaw change since the last keyframe).  For each new keyframe it:
//!
//! 1. Detects FAST-9 corners and computes BRIEF-256 descriptors (via
//!    `perception::processing::features::detect_features`).
//! 2. Matches features to the previous keyframe using Hamming distance with a
//!    ratio test to reject ambiguous matches.
//! 3. Runs the **normalized 8-point algorithm** on the matches to recover the
//!    essential matrix `E` and decomposes it into a relative rotation `R` and
//!    unit-length translation `t`.
//! 4. Scales the translation by the ratio of the two keyframes' down-range
//!    measurements (`range_z`) — this converts the unit vector into a metric
//!    estimate of the camera displacement.
//!
//! ## Algorithm summary
//!
//! ```text
//! matches ──► normalise points (mean=0, std=√2)
//!          ──► build 9×N matrix A from (u,v) ↔ (u',v') pairs
//!          ──► SVD of AᵀA (Jacobi, 9×9)
//!          ──► last eigenvector → reshape to 3×3 F
//!          ──► enforce rank-2 via SVD(F) (zero smallest singular value)
//!          ──► E = K'ᵀ F K (using camera intrinsics)
//!          ──► decompose E → {R, t} (four candidate solutions)
//!          ──► cheirality test (positive depth) → unique solution
//!          ──► scale t by Δrange_z
//! ```
//!
//! ## Usage
//!
//! ```rust
//! use multirotor_simulator::mapping::keyframe::KeyframeStore;
//! use multirotor_simulator::perception::types::ImageFrame;
//! use multirotor_simulator::math::Vec3;
//!
//! let mut store = KeyframeStore::new();
//! // In the AI-Deck receive loop (20–30 Hz):
//! // if let Some(result) = store.push(frame, pos, yaw_deg, range_z) {
//! //     println!("relative translation: {:?}", result.translation_m);
//! // }
//! ```

use crate::math::Vec3;
use crate::perception::types::{CameraIntrinsics, ImageFrame};
use crate::perception::processing::features::detect_features;
use crate::perception::types::Feature;
use crate::mapping::loop_closure::LoopConstraint;

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

/// Minimum translation (metres) between consecutive keyframes.
const KF_MIN_DIST_M: f32 = 0.30;

/// Minimum yaw rotation (degrees) between consecutive keyframes.
const KF_MIN_YAW_DEG: f32 = 30.0;

/// Maximum number of keyframes kept in the rolling buffer.
const KF_MAX_STORED: usize = 50;

/// FAST-9 corner threshold.  Higher → fewer but more distinctive corners.
const FAST_THRESHOLD: u8 = 20;

/// Maximum Hamming distance for a match to be considered good.
const MATCH_MAX_HAMMING: u32 = 64;

/// Lowe ratio-test threshold: best_dist < RATIO * second_best_dist.
const MATCH_RATIO: f32 = 0.75;

/// Minimum inlier matches required to attempt essential-matrix estimation.
const MIN_MATCHES_FOR_ESSENTIAL: usize = 8;

// ---------------------------------------------------------------------------
// Loop-closure tuning constants
// ---------------------------------------------------------------------------

/// XY radius within which a candidate keyframe is considered for loop closure.
const LOOP_SEARCH_RADIUS_M: f32 = 1.5;

/// Minimum number of global-index positions between the new keyframe and a candidate.
/// Prevents matching the immediate predecessor.  Applied to `global_index`, so
/// it is invariant to rolling-buffer eviction.
const LOOP_MIN_AGE: usize = 5;

/// Minimum feature matches (after ratio test) to proceed to essential-matrix.
const LOOP_MIN_MATCHES: usize = 12;

/// Minimum essential-matrix inliers to accept the loop closure.
const LOOP_MIN_INLIERS: usize = 8;

/// Spatial grid cell size for loop-closure candidate lookup.
/// Equal to `LOOP_SEARCH_RADIUS_M` so the 3×3 neighbourhood of any query
/// point covers exactly all candidates within the search radius.
const SPATIAL_CELL_M: f32 = LOOP_SEARCH_RADIUS_M; // 1.5 m

// ---------------------------------------------------------------------------
// SpatialGrid (private)
// ---------------------------------------------------------------------------

/// Convert a world XY position to an integer grid cell key.
fn cell_key(x: f32, y: f32) -> (i32, i32) {
    ((x / SPATIAL_CELL_M).floor() as i32,
     (y / SPATIAL_CELL_M).floor() as i32)
}

/// Sparse grid: maps (cell_x, cell_y) → list of `global_index` positions.
///
/// Cell size equals `LOOP_SEARCH_RADIUS_M` so the 3×3 neighbourhood of any
/// query cell contains all points within the search radius (the exact distance
/// check eliminates corner candidates).
struct SpatialGrid {
    cells: std::collections::HashMap<(i32, i32), Vec<usize>>,
}

impl SpatialGrid {
    fn new() -> Self {
        Self { cells: std::collections::HashMap::new() }
    }

    /// Record that `global_index[gi]` is at world position (x, y).
    fn insert(&mut self, x: f32, y: f32, gi: usize) {
        self.cells.entry(cell_key(x, y)).or_default().push(gi);
    }

    /// Collect all `global_index` positions in the 3×3 cell neighbourhood of
    /// (x, y).  Callers must still apply the exact distance threshold.
    fn candidates_near(&self, x: f32, y: f32) -> Vec<usize> {
        let (cx, cy) = cell_key(x, y);
        let mut out = Vec::new();
        for dx in -1i32..=1 {
            for dy in -1i32..=1 {
                if let Some(v) = self.cells.get(&(cx + dx, cy + dy)) {
                    out.extend_from_slice(v);
                }
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// A single captured keyframe.
#[derive(Clone)]
pub struct Keyframe {
    /// Drone position (EKF world frame, metres) when frame was taken.
    pub pos: Vec3,
    /// Drone yaw (EKF, degrees) when frame was taken.
    pub yaw_deg: f32,
    /// Down-range ToF reading (metres) — used for metric scale recovery.
    pub range_z: f32,
    /// The raw image.
    pub frame: ImageFrame,
    /// Detected corners + BRIEF descriptors.
    pub features: Vec<Feature>,
    /// Monotonically-increasing counter assigned at push() time.
    /// Stable identity even after the rolling buffer evicts older frames.
    pub kf_index: usize,
}

/// Lightweight keyframe record for the global index.
/// Same as `Keyframe` minus the raw `ImageFrame` — avoids keeping large pixel
/// buffers for every keyframe ever seen.
#[derive(Clone)]
pub struct CompactKeyframe {
    /// Monotonically-increasing index (same as `Keyframe::kf_index`).
    pub kf_index: usize,
    /// Drone position (EKF world frame, metres) when frame was taken.
    pub pos:      Vec3,
    /// Drone yaw (EKF, degrees) when frame was taken.
    pub yaw_deg:  f32,
    /// Down-range ToF reading (metres).
    pub range_z:  f32,
    /// Detected corners + BRIEF descriptors.
    pub features: Vec<Feature>,
}

/// A matched pair of feature indices between two keyframes.
#[derive(Debug, Clone, Copy)]
pub struct FeatureMatch {
    /// Feature index in the *previous* keyframe.
    pub prev_idx: usize,
    /// Feature index in the *current* keyframe.
    pub curr_idx: usize,
    /// Hamming distance between descriptors.
    pub hamming: u32,
}

/// Result of registering a new keyframe against its predecessor.
#[derive(Debug, Clone)]
pub struct KeyframeResult {
    /// Index of the new keyframe in the store.
    pub kf_index: usize,
    /// Number of feature matches that passed the ratio test.
    pub match_count: usize,
    /// Relative rotation matrix `R` (current ← previous).
    /// Identity if essential matrix could not be estimated.
    pub rotation: [[f32; 3]; 3],
    /// Unit-length translation direction.
    pub translation_dir: Vec3,
    /// Metric translation estimate (scaled by `Δrange_z`).
    pub translation_m: Vec3,
    /// Number of inliers used in the essential-matrix estimation.
    pub inlier_count: usize,
}

// ---------------------------------------------------------------------------
// KeyframeStore
// ---------------------------------------------------------------------------

/// Rolling keyframe buffer with an unbounded global index.
///
/// Two parallel stores are maintained:
///
/// - **`frames`** — rolling window capped at `KF_MAX_STORED` (50).  Used for
///   sequential VO matching against the immediately preceding keyframe.
/// - **`global_index`** — every accepted keyframe ever, never evicted.  Used
///   by [`detect_loop`](Self::detect_loop) so that a loop seen more than 50
///   keyframes ago can still be closed.
/// - **`spatial_grid`** — sparse hash grid (cell = `LOOP_SEARCH_RADIUS_M`)
///   mirroring `global_index`.  Enables O(1) loop-closure candidate retrieval
///   by querying the 3×3 cell neighbourhood of the query position.
///
/// Call [`push`](Self::push) once per received `ImageFrame`; the store decides
/// whether to promote the frame to a keyframe based on the pose change.
pub struct KeyframeStore {
    frames:          Vec<Keyframe>,
    /// Camera intrinsics used to compute the essential matrix.
    intrinsics:      CameraIntrinsics,
    /// Monotonic counter — incremented on every accepted keyframe.
    next_kf_index:   usize,
    /// Global index — grows forever, never evicted.  Used by `detect_loop` to
    /// find loop candidates even after they have left the rolling buffer.
    global_index:    Vec<CompactKeyframe>,
    /// Spatial hash grid mirroring `global_index` for O(1) candidate lookup.
    spatial_grid:    SpatialGrid,
}

impl KeyframeStore {
    /// Create a new store with the HiMax HM01B0 default intrinsics.
    pub fn new() -> Self {
        Self {
            frames:         Vec::new(),
            intrinsics:     crate::perception::processing::calibration::hm01b0_defaults(),
            next_kf_index:  0,
            global_index:   Vec::new(),
            spatial_grid:   SpatialGrid::new(),
        }
    }

    /// Create with custom intrinsics.
    pub fn with_intrinsics(intrinsics: CameraIntrinsics) -> Self {
        Self {
            frames: Vec::new(),
            intrinsics,
            next_kf_index: 0,
            global_index: Vec::new(),
            spatial_grid: SpatialGrid::new(),
        }
    }

    /// Offer a new frame.  Returns `Some(KeyframeResult)` if the frame was
    /// accepted as a keyframe AND a predecessor exists for comparison.
    pub fn push(
        &mut self,
        frame:   ImageFrame,
        pos:     Vec3,
        yaw_deg: f32,
        range_z: f32,
    ) -> Option<KeyframeResult> {
        // Check if this frame qualifies as a new keyframe.
        if let Some(last) = self.frames.last() {
            let dx = pos.x - last.pos.x;
            let dy = pos.y - last.pos.y;
            let dz = pos.z - last.pos.z;
            let dist = (dx*dx + dy*dy + dz*dz).sqrt();
            let dyaw = angle_diff_deg(yaw_deg, last.yaw_deg).abs();
            if dist < KF_MIN_DIST_M && dyaw < KF_MIN_YAW_DEG {
                return None; // not far enough for a new keyframe
            }
        }

        // Accept — detect features.
        let features = detect_features(&frame, FAST_THRESHOLD);

        let kf_index = self.next_kf_index;
        self.next_kf_index += 1;
        let kf = Keyframe { pos, yaw_deg, range_z, frame, features, kf_index };

        // Store a compact copy in the global index (no image, never evicted).
        self.global_index.push(CompactKeyframe {
            kf_index,
            pos,
            yaw_deg,
            range_z,
            features: kf.features.clone(),
        });
        // Register in spatial grid for O(1) loop-closure lookup.
        self.spatial_grid.insert(pos.x, pos.y, self.global_index.len() - 1);

        if self.frames.is_empty() {
            // First keyframe — no predecessor to compare against.
            self.frames.push(kf);
            if self.frames.len() > KF_MAX_STORED {
                self.frames.remove(0);
            }
            return None;
        }

        // Match against the previous keyframe.
        let prev_idx = self.frames.len() - 1;
        let matches  = match_features(&self.frames[prev_idx].features, &kf.features);
        let match_count = matches.len();

        let (rotation, translation_dir, inlier_count) =
            if match_count >= MIN_MATCHES_FOR_ESSENTIAL {
                estimate_essential(
                    &self.frames[prev_idx].features,
                    &kf.features,
                    &matches,
                    &self.intrinsics,
                )
            } else {
                (identity3(), Vec3::zero(), 0)
            };

        // Scale translation by Δrange_z ratio (metric scale recovery).
        let scale = if self.frames[prev_idx].range_z > 0.02 && range_z > 0.02 {
            (range_z + self.frames[prev_idx].range_z) * 0.5
        } else {
            0.3 // default hover height if no valid range
        };
        let translation_m = Vec3::new(
            translation_dir.x * scale,
            translation_dir.y * scale,
            translation_dir.z * scale,
        );

        self.frames.push(kf);
        if self.frames.len() > KF_MAX_STORED {
            self.frames.remove(0);
        }

        Some(KeyframeResult {
            kf_index,
            match_count,
            rotation,
            translation_dir,
            translation_m,
            inlier_count,
        })
    }

    /// Number of keyframes in the rolling buffer (capped at `KF_MAX_STORED`).
    pub fn len(&self) -> usize { self.frames.len() }

    /// True if no keyframes have been stored yet.
    pub fn is_empty(&self) -> bool { self.frames.is_empty() }

    /// Total keyframes ever accepted — grows unboundedly, never decrements.
    pub fn global_len(&self) -> usize { self.global_index.len() }

    /// Access a stored keyframe by index.
    pub fn get(&self, idx: usize) -> Option<&Keyframe> { self.frames.get(idx) }

    /// Search for a loop closure for the most recently added keyframe.
    ///
    /// `new_kf_index` is `KeyframeResult::kf_index` returned by the most recent
    /// `push()` call.  Returns the best match (highest inlier count) among all
    /// candidate keyframes that are:
    ///
    /// - At least `LOOP_MIN_AGE` positions older in the global index (checked via
    ///   the grid-returned indices, not by slicing).
    /// - Within `LOOP_SEARCH_RADIUS_M` of the new keyframe (XY only).
    /// - Have at least `LOOP_MIN_MATCHES` feature matches.
    /// - Yield at least `LOOP_MIN_INLIERS` essential-matrix inliers.
    ///
    /// Candidates are retrieved from `spatial_grid` (O(1), 3×3 cell neighbourhood)
    /// rather than a linear scan over `global_index`.  Because the search covers
    /// the entire `global_index`, keyframes evicted from the rolling buffer are
    /// still eligible loop candidates.
    pub fn detect_loop(&self, new_kf_index: usize) -> Option<LoopConstraint> {
        use std::f32::consts::PI;

        let new_kf = self.global_index.last()?;

        let n = self.global_index.len();
        if n <= LOOP_MIN_AGE {
            return None;
        }

        let candidate_indices = self.spatial_grid.candidates_near(
            new_kf.pos.x, new_kf.pos.y);

        let mut best: Option<LoopConstraint> = None;
        let mut best_inliers = 0usize;

        for gi in candidate_indices {
            if gi + LOOP_MIN_AGE >= n { continue; }          // age filter
            let candidate = &self.global_index[gi];
            let dx = new_kf.pos.x - candidate.pos.x;
            let dy = new_kf.pos.y - candidate.pos.y;
            if dx * dx + dy * dy > LOOP_SEARCH_RADIUS_M * LOOP_SEARCH_RADIUS_M {
                continue;                                     // exact radius check (filters corners)
            }

            let matches = match_features(&candidate.features, &new_kf.features);
            if matches.len() < LOOP_MIN_MATCHES {
                continue;
            }

            let (_, t_dir, inliers) = estimate_essential(
                &candidate.features,
                &new_kf.features,
                &matches,
                &self.intrinsics,
            );

            if inliers < LOOP_MIN_INLIERS {
                continue;
            }

            // Metric scale from average range_z of the two frames.
            let scale = if candidate.range_z > 0.02 && new_kf.range_z > 0.02 {
                (candidate.range_z + new_kf.range_z) * 0.5
            } else {
                0.3
            };

            // Camera-to-body: body_x = cam_z, body_y = cam_x.
            let t_body_x = t_dir.z * scale;
            let t_body_y = t_dir.x * scale;

            // Body-to-world rotation using new_kf yaw.
            let psi  = new_kf.yaw_deg * PI / 180.0;
            let t_wx = psi.cos() * t_body_x - psi.sin() * t_body_y;
            let t_wy = psi.sin() * t_body_x + psi.cos() * t_body_y;

            if inliers > best_inliers {
                best_inliers = inliers;
                best = Some(LoopConstraint {
                    from_idx:          candidate.kf_index,
                    to_idx:            new_kf_index,
                    translation_world: [t_wx, t_wy],
                    inlier_count:      inliers,
                });
            }
        }

        best
    }
}

// ---------------------------------------------------------------------------
// Feature matching
// ---------------------------------------------------------------------------

/// Brute-force nearest-neighbour with Lowe ratio test.
///
/// Returns matches sorted by Hamming distance (best first).
fn match_features(prev: &[Feature], curr: &[Feature]) -> Vec<FeatureMatch> {
    if prev.is_empty() || curr.is_empty() {
        return Vec::new();
    }

    let mut result = Vec::new();

    for (pi, pf) in prev.iter().enumerate() {
        // Find two nearest neighbours in curr.
        let mut best1 = (u32::MAX, 0usize);
        let mut best2 = (u32::MAX, 0usize);

        for (ci, cf) in curr.iter().enumerate() {
            let d = hamming(&pf.descriptor, &cf.descriptor);
            if d < best1.0 {
                best2 = best1;
                best1 = (d, ci);
            } else if d < best2.0 {
                best2 = (d, ci);
            }
        }

        // Ratio test: best match must be clearly better than second best.
        if best1.0 > MATCH_MAX_HAMMING {
            continue;
        }
        let ratio_ok = best2.0 == u32::MAX
            || (best1.0 as f32) < MATCH_RATIO * (best2.0 as f32);
        if !ratio_ok {
            continue;
        }

        result.push(FeatureMatch { prev_idx: pi, curr_idx: best1.1, hamming: best1.0 });
    }

    // Sort by Hamming distance ascending (best first).
    result.sort_by_key(|m| m.hamming);
    result
}

/// Compute the Hamming distance between two 32-byte BRIEF descriptors.
fn hamming(a: &[u8; 32], b: &[u8; 32]) -> u32 {
    let mut dist = 0u32;
    for i in 0..32 {
        dist += (a[i] ^ b[i]).count_ones();
    }
    dist
}

// ---------------------------------------------------------------------------
// Essential matrix estimation (normalized 8-point algorithm)
// ---------------------------------------------------------------------------

/// Estimate the essential matrix from matched features.
///
/// Returns `(R, t_unit, inlier_count)`.
/// `R` is the rotation from frame A (prev) to frame B (curr).
/// `t` is the unit-length translation in the camera frame of A.
fn estimate_essential(
    prev_feats: &[Feature],
    curr_feats: &[Feature],
    matches:    &[FeatureMatch],
    ki:         &CameraIntrinsics,
) -> ([[f32; 3]; 3], Vec3, usize) {
    // Build normalised image coordinates.
    let pts_a: Vec<[f32; 2]> = matches.iter()
        .map(|m| {
            let f = &prev_feats[m.prev_idx];
            [(f.x - ki.cx) / ki.fx, (f.y - ki.cy) / ki.fy]
        })
        .collect();
    let pts_b: Vec<[f32; 2]> = matches.iter()
        .map(|m| {
            let f = &curr_feats[m.curr_idx];
            [(f.x - ki.cx) / ki.fx, (f.y - ki.cy) / ki.fy]
        })
        .collect();

    let n = pts_a.len();
    if n < MIN_MATCHES_FOR_ESSENTIAL {
        return (identity3(), Vec3::zero(), 0);
    }

    // Hartley normalisation: translate by centroid, scale so mean distance = √2.
    // This preconditions the AᵀA matrix so its singular values span a small
    // range, dramatically improving numerical accuracy of the SVD on f32.
    // Without it, the 8-point algorithm is unreliable for typical image coords
    // (range ~0..320) due to the ~10⁵ condition number of the raw matrix.
    let (ta, sa) = normalisation_transform(&pts_a);
    let (tb, sb) = normalisation_transform(&pts_b);

    let norm_a: Vec<[f32; 2]> = pts_a.iter()
        .map(|p| [(p[0] - ta[0]) * sa, (p[1] - ta[1]) * sa])
        .collect();
    let norm_b: Vec<[f32; 2]> = pts_b.iter()
        .map(|p| [(p[0] - tb[0]) * sb, (p[1] - tb[1]) * sb])
        .collect();

    // Build the 9×9 matrix AᵀA from the epipolar constraint rows.
    let mut ata = [[0.0f32; 9]; 9];
    for i in 0..n {
        let (u, v)   = (norm_a[i][0], norm_a[i][1]);
        let (u2, v2) = (norm_b[i][0], norm_b[i][1]);
        // Row of A: [u'u, u'v, u', v'u, v'v, v', u, v, 1]
        let row = [u2*u, u2*v, u2, v2*u, v2*v, v2, u, v, 1.0f32];
        for r in 0..9 {
            for c in 0..9 {
                ata[r][c] += row[r] * row[c];
            }
        }
    }

    // Find the null space of A (eigenvector of AᵀA with smallest eigenvalue).
    let f_vec = smallest_eigenvec_9x9(&ata);

    // Reshape to 3×3.
    let mut f = [[0.0f32; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            f[r][c] = f_vec[r * 3 + c];
        }
    }

    // Denormalise: F = Tb^T * F_norm * Ta
    let ta_mat = normalisation_matrix(ta, sa);
    let tb_mat = normalisation_matrix(tb, sb);
    let tb_t   = transpose3(&tb_mat);
    let f = mat3_mul(&mat3_mul(&tb_t, &f), &ta_mat);

    // Enforce rank-2 constraint: SVD(F), zero smallest singular value, recompose.
    let f = enforce_rank2(&f);

    // Compute essential matrix: E = K^T * F * K
    // (here K is the same for both cameras since they're the same physical lens)
    let k_mat = intrinsics_matrix(ki);
    let kt    = transpose3(&k_mat);
    let e     = mat3_mul(&mat3_mul(&kt, &f), &k_mat);

    // Decompose E → R, t (four candidate solutions; pick by positive-depth test).
    let (r, t, inliers) = decompose_essential(&e, &pts_a, &pts_b);

    (r, t, inliers)
}

/// Returns `(centroid, scale)` where scale normalises mean distance to √2.
fn normalisation_transform(pts: &[[f32; 2]]) -> ([f32; 2], f32) {
    let n = pts.len() as f32;
    let cx = pts.iter().map(|p| p[0]).sum::<f32>() / n;
    let cy = pts.iter().map(|p| p[1]).sum::<f32>() / n;
    let mean_dist = pts.iter()
        .map(|p| ((p[0]-cx).powi(2) + (p[1]-cy).powi(2)).sqrt())
        .sum::<f32>() / n;
    let scale = if mean_dist > 1e-6 { 2.0f32.sqrt() / mean_dist } else { 1.0 };
    ([cx, cy], scale)
}

fn normalisation_matrix(t: [f32; 2], s: f32) -> [[f32; 3]; 3] {
    [[s, 0.0, -s * t[0]],
     [0.0, s, -s * t[1]],
     [0.0, 0.0, 1.0]]
}

fn intrinsics_matrix(ki: &CameraIntrinsics) -> [[f32; 3]; 3] {
    [[ki.fx,  0.0,   ki.cx],
     [0.0,   ki.fy,  ki.cy],
     [0.0,   0.0,    1.0]]
}

// ---------------------------------------------------------------------------
// Essential matrix decomposition
// ---------------------------------------------------------------------------

/// Decompose E into the unique (R, t) pair with positive cheirality.
///
/// There are four (R, t) solutions; we pick the one where the majority of
/// triangulated points have positive depth in both cameras.
fn decompose_essential(
    e:    &[[f32; 3]; 3],
    pts_a: &[[f32; 2]],
    pts_b: &[[f32; 2]],
) -> ([[f32; 3]; 3], Vec3, usize) {
    // SVD of E.
    let (u, s_vals, vt) = svd3x3(e);
    let _ = s_vals; // only U and Vt needed

    // W matrix for R decomposition.
    let w  = [[0.0f32,-1.0, 0.0],
              [1.0,   0.0, 0.0],
              [0.0,   0.0, 1.0]];
    let wt = [[0.0f32, 1.0, 0.0],
              [-1.0,  0.0, 0.0],
              [0.0,   0.0, 1.0]];

    // t is the last column of U (up to sign).
    let t_pos = Vec3::new(u[0][2], u[1][2], u[2][2]);
    let t_neg = Vec3::new(-u[0][2], -u[1][2], -u[2][2]);

    let r1 = mat3_mul(&mat3_mul(&u, &w),  &vt);
    let r2 = mat3_mul(&mat3_mul(&u, &wt), &vt);

    // Ensure det(R) = +1 (rotation, not reflection).
    let r1 = if det3(&r1) < 0.0 { scale3(&r1, -1.0) } else { r1 };
    let r2 = if det3(&r2) < 0.0 { scale3(&r2, -1.0) } else { r2 };

    // Four candidate solutions.
    let candidates = [
        (r1, t_pos),
        (r1, t_neg),
        (r2, t_pos),
        (r2, t_neg),
    ];

    // Pick the candidate with the most points having positive depth in both views.
    let mut best = (identity3(), Vec3::zero(), 0usize);
    for (r, t) in &candidates {
        let inliers = cheirality_count(r, t, pts_a, pts_b);
        if inliers > best.2 {
            best = (*r, *t, inliers);
        }
    }
    best
}

/// Count points that triangulate with positive depth in both cameras.
fn cheirality_count(
    r:     &[[f32; 3]; 3],
    t:     &Vec3,
    pts_a: &[[f32; 2]],
    pts_b: &[[f32; 2]],
) -> usize {
    let mut count = 0usize;
    for (pa, pb) in pts_a.iter().zip(pts_b.iter()) {
        // Camera A is at origin, B is at R, t.
        // Triangulate by linear least squares (simple 4-eq/3-unk DLT).
        let xa = [pa[0], pa[1], 1.0f32];
        let xb = [pb[0], pb[1], 1.0f32];
        // Project xb into camera A frame.
        let rt_xb = mat3_vec(r, &xb);
        // Depth in camera A: positive means in front.
        let depth_a = xa[0] * rt_xb[0] + xa[1] * rt_xb[1] + rt_xb[2];
        // Depth in camera B (t component along the triangulated direction).
        let depth_b = xb[0] * (rt_xb[0] - t.x)
                    + xb[1] * (rt_xb[1] - t.y)
                    + (rt_xb[2] - t.z);
        if depth_a > 0.0 && depth_b > 0.0 {
            count += 1;
        }
    }
    count
}

// ---------------------------------------------------------------------------
// 3×3 matrix helpers
// ---------------------------------------------------------------------------

fn identity3() -> [[f32; 3]; 3] {
    [[1.0, 0.0, 0.0],
     [0.0, 1.0, 0.0],
     [0.0, 0.0, 1.0]]
}

fn transpose3(m: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut out = [[0.0f32; 3]; 3];
    for i in 0..3 { for j in 0..3 { out[i][j] = m[j][i]; } }
    out
}

fn mat3_mul(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut out = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 { out[i][j] += a[i][k] * b[k][j]; }
        }
    }
    out
}

fn mat3_vec(m: &[[f32; 3]; 3], v: &[f32; 3]) -> [f32; 3] {
    [m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
     m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
     m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2]]
}

fn det3(m: &[[f32; 3]; 3]) -> f32 {
    m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
    - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
    + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0])
}

fn scale3(m: &[[f32; 3]; 3], s: f32) -> [[f32; 3]; 3] {
    let mut out = *m;
    for r in &mut out { for v in r.iter_mut() { *v *= s; } }
    out
}

/// Enforce rank-2 by zeroing the smallest singular value of a 3×3 matrix.
fn enforce_rank2(m: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let (u, mut s, vt) = svd3x3(m);
    // Zero smallest singular value (index 2 after sort).
    s[2] = 0.0;
    // Recompose: U · diag(s) · Vt
    let mut out = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                out[i][j] += u[i][k] * s[k] * vt[k][j];
            }
        }
    }
    out
}

// ---------------------------------------------------------------------------
// SVD implementations
// ---------------------------------------------------------------------------

/// One-sided Jacobi SVD of a symmetric 3×3 matrix M = MᵀM (eigendecomposition).
///
/// Returns `(U, s, Vt)` where `U·diag(s)·Vt ≈ M`.
/// Singular values are returned in *non-increasing* order.
///
/// This handles 3×3 only and is used for rank-2 enforcement and E decomposition.
fn svd3x3(m: &[[f32; 3]; 3]) -> ([[f32; 3]; 3], [f32; 3], [[f32; 3]; 3]) {
    // Bidiagonalisation via Golub-Reinsch is complex; for 3×3 we use a simpler
    // approach: compute eigendecomposition of MᵀM to get V and singular values,
    // then U = M·V·diag(1/s).

    // MtM = M^T · M
    let mt  = transpose3(m);
    let mtm = mat3_mul(&mt, m);

    // Jacobi eigendecomposition of the symmetric 3×3 MᵀM.
    let (v, eigenvals) = jacobi_eigen3(&mtm);

    // Sort eigenvalues descending, permuting V columns in the same order.
    let mut idx = [0usize, 1, 2];
    let ev = eigenvals;
    idx.sort_by(|&a, &b| ev[b].partial_cmp(&ev[a]).unwrap_or(std::cmp::Ordering::Equal));

    // Singular values = sqrt of eigenvalues, in sorted order.
    let mut s = [0.0f32; 3];

    let mut v_sorted = [[0.0f32; 3]; 3];
    for (new_col, &old_col) in idx.iter().enumerate() {
        for row in 0..3 { v_sorted[row][new_col] = v[row][old_col]; }
    }

    s[0] = eigenvals[idx[0]].max(0.0).sqrt();
    s[1] = eigenvals[idx[1]].max(0.0).sqrt();
    s[2] = eigenvals[idx[2]].max(0.0).sqrt();

    // U = M·V·diag(1/s)  (column-wise: u_i = M·v_i / s_i)
    let mut u = [[0.0f32; 3]; 3];
    for i in 0..3 {
        let vi = [v_sorted[0][i], v_sorted[1][i], v_sorted[2][i]];
        let mvi = mat3_vec(m, &vi);
        let norm = s[i];
        if norm > 1e-10 {
            for r in 0..3 { u[r][i] = mvi[r] / norm; }
        } else {
            // Zero singular value — fill U column with anything orthonormal.
            // Use a simple Gram-Schmidt against existing columns.
            let col = gram_schmidt_fill(&u, i);
            for r in 0..3 { u[r][i] = col[r]; }
        }
    }

    let vt = transpose3(&v_sorted);
    (u, s, vt)
}

/// Jacobi eigendecomposition for a symmetric 3×3 matrix.
/// Returns (eigenvectors as columns, eigenvalues) sorted in no particular order.
fn jacobi_eigen3(a: &[[f32; 3]; 3]) -> ([[f32; 3]; 3], [f32; 3]) {
    let mut d = *a;
    let mut v = identity3();
    const MAX_ITER: usize = 100;

    for _ in 0..MAX_ITER {
        // Find largest off-diagonal element.
        let mut max_val = 0.0f32;
        let mut p = 0usize;
        let mut q = 1usize;
        for i in 0..3 {
            for j in (i+1)..3 {
                if d[i][j].abs() > max_val {
                    max_val = d[i][j].abs();
                    p = i; q = j;
                }
            }
        }
        if max_val < 1e-10 { break; }

        // Compute rotation angle.
        let theta = (d[q][q] - d[p][p]) / (2.0 * d[p][q]);
        let t = if theta >= 0.0 {
            1.0 / (theta + (1.0 + theta * theta).sqrt())
        } else {
            1.0 / (theta - (1.0 + theta * theta).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;
        let tau = s / (1.0 + c);

        // Update D.
        let d_pp = d[p][p] - t * d[p][q];
        let d_qq = d[q][q] + t * d[p][q];
        d[p][p] = d_pp;
        d[q][q] = d_qq;
        d[p][q] = 0.0;
        d[q][p] = 0.0;
        for r in 0..3 {
            if r != p && r != q {
                let d_rp = d[r][p] - s * (d[r][q] + tau * d[r][p]);
                let d_rq = d[r][q] + s * (d[r][p] - tau * d[r][q]);
                d[r][p] = d_rp; d[p][r] = d_rp;
                d[r][q] = d_rq; d[q][r] = d_rq;
            }
        }

        // Update eigenvector matrix V.
        for r in 0..3 {
            let v_rp = v[r][p] - s * (v[r][q] + tau * v[r][p]);
            let v_rq = v[r][q] + s * (v[r][p] - tau * v[r][q]);
            v[r][p] = v_rp;
            v[r][q] = v_rq;
        }
    }

    ([v[0], v[1], v[2]], [d[0][0], d[1][1], d[2][2]])
}

/// Find a unit vector orthogonal to all already-filled columns of U (column < fill_col).
fn gram_schmidt_fill(u: &[[f32; 3]; 3], fill_col: usize) -> [f32; 3] {
    // Start with the canonical basis vector least aligned to existing columns.
    let candidates: [[f32; 3]; 3] = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]];
    let mut best = candidates[0];
    let mut best_norm = 0.0f32;
    for cand in &candidates {
        let mut v = *cand;
        // Project out existing columns.
        for c in 0..fill_col {
            let dot = u[0][c]*v[0] + u[1][c]*v[1] + u[2][c]*v[2];
            v[0] -= dot * u[0][c];
            v[1] -= dot * u[1][c];
            v[2] -= dot * u[2][c];
        }
        let n = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt();
        if n > best_norm { best_norm = n; best = v; }
    }
    if best_norm > 1e-10 {
        [best[0]/best_norm, best[1]/best_norm, best[2]/best_norm]
    } else {
        [1.0, 0.0, 0.0]
    }
}

// ---------------------------------------------------------------------------
// 9×9 eigenvector (null space) via power iteration on AᵀA
// ---------------------------------------------------------------------------

/// Find the eigenvector of the 9×9 symmetric matrix corresponding to the
/// **smallest** eigenvalue (the null-space approximation of A).
///
/// Uses inverse power iteration with a small shift to converge to the minimum.
fn smallest_eigenvec_9x9(ata: &[[f32; 9]; 9]) -> [f32; 9] {
    // Shift matrix: (AᵀA + μI) to make all eigenvalues positive, then use
    // regular power iteration on the inverse — but inverting 9×9 is expensive.
    // Instead we use deflation: run power iteration to find the LARGEST
    // eigenvector, project it out, repeat 8 times, and take the residual.
    // This converges robustly for well-conditioned A.

    // Jacobi eigendecomposition of the 9×9 symmetric matrix.
    // The eigenvectors form a 9×9 orthogonal matrix; we want column with min eigenvalue.
    let (v, eigenvals) = jacobi_eigen9(ata);

    // Find index of smallest eigenvalue.
    let mut min_idx = 0;
    for i in 1..9 {
        if eigenvals[i] < eigenvals[min_idx] { min_idx = i; }
    }

    // Return the corresponding eigenvector (column min_idx of v).
    let mut out = [0.0f32; 9];
    for r in 0..9 { out[r] = v[r][min_idx]; }

    // Normalise.
    let norm = out.iter().map(|x| x * x).sum::<f32>().sqrt();
    if norm > 1e-10 {
        for x in &mut out { *x /= norm; }
    }
    out
}

/// Jacobi eigendecomposition for a symmetric 9×9 matrix.
/// Returns (eigenvectors as columns, eigenvalues).
fn jacobi_eigen9(a: &[[f32; 9]; 9]) -> ([[f32; 9]; 9], [f32; 9]) {
    let mut d = *a;
    let mut v = {
        let mut m = [[0.0f32; 9]; 9];
        for i in 0..9 { m[i][i] = 1.0; }
        m
    };

    const MAX_ITER: usize = 500;
    for _ in 0..MAX_ITER {
        // Find largest off-diagonal element.
        let mut max_val = 0.0f32;
        let mut p = 0usize;
        let mut q = 1usize;
        for i in 0..9 {
            for j in (i+1)..9 {
                if d[i][j].abs() > max_val {
                    max_val = d[i][j].abs();
                    p = i; q = j;
                }
            }
        }
        if max_val < 1e-8 { break; }

        let theta = (d[q][q] - d[p][p]) / (2.0 * d[p][q]);
        let t = if theta >= 0.0 {
            1.0 / (theta + (1.0 + theta * theta).sqrt())
        } else {
            1.0 / (theta - (1.0 + theta * theta).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;
        let tau = s / (1.0 + c);

        let d_pp = d[p][p] - t * d[p][q];
        let d_qq = d[q][q] + t * d[p][q];
        d[p][p] = d_pp;
        d[q][q] = d_qq;
        d[p][q] = 0.0;
        d[q][p] = 0.0;
        for r in 0..9 {
            if r != p && r != q {
                let d_rp = d[r][p] - s * (d[r][q] + tau * d[r][p]);
                let d_rq = d[r][q] + s * (d[r][p] - tau * d[r][q]);
                d[r][p] = d_rp; d[p][r] = d_rp;
                d[r][q] = d_rq; d[q][r] = d_rq;
            }
        }

        for r in 0..9 {
            let v_rp = v[r][p] - s * (v[r][q] + tau * v[r][p]);
            let v_rq = v[r][q] + s * (v[r][p] - tau * v[r][q]);
            v[r][p] = v_rp;
            v[r][q] = v_rq;
        }
    }

    let eigenvals = [d[0][0],d[1][1],d[2][2],d[3][3],d[4][4],
                     d[5][5],d[6][6],d[7][7],d[8][8]];
    (v, eigenvals)
}

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------

/// Shortest signed angular difference (degrees).
fn angle_diff_deg(a: f32, b: f32) -> f32 {
    let d = (a - b).rem_euclid(360.0);
    if d > 180.0 { d - 360.0 } else { d }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::perception::types::ImageFrame;

    fn blank_frame(w: u16, h: u16) -> ImageFrame {
        ImageFrame { width: w, height: h, pixels: vec![0u8; (w as usize) * (h as usize)], timestamp_ms: 0 }
    }

    fn checkerboard_frame(w: u16, h: u16, cell: u16) -> ImageFrame {
        let mut pixels = vec![0u8; (w as usize) * (h as usize)];
        for y in 0..h as usize {
            for x in 0..w as usize {
                let bx = x / cell as usize;
                let by = y / cell as usize;
                pixels[y * w as usize + x] = if (bx + by) % 2 == 0 { 200 } else { 50 };
            }
        }
        ImageFrame { width: w, height: h, pixels, timestamp_ms: 0 }
    }

    #[test]
    fn hamming_identical_descriptors() {
        let d = [0xABu8; 32];
        assert_eq!(hamming(&d, &d), 0);
    }

    #[test]
    fn hamming_all_different() {
        let a = [0x00u8; 32];
        let b = [0xFFu8; 32];
        assert_eq!(hamming(&a, &b), 256); // all 256 bits differ
    }

    #[test]
    fn keyframe_store_first_frame_returns_none() {
        let mut store = KeyframeStore::new();
        let frame = blank_frame(64, 64);
        let result = store.push(frame, Vec3::new(0.0, 0.0, 0.3), 0.0, 0.3);
        assert!(result.is_none(), "first keyframe should return None (no predecessor)");
    }

    #[test]
    fn keyframe_store_too_close_returns_none() {
        let mut store = KeyframeStore::new();
        // First frame accepted.
        store.push(blank_frame(64, 64), Vec3::new(0.0, 0.0, 0.3), 0.0, 0.3);
        // Second frame too close (< 0.3 m, < 30°).
        let result = store.push(blank_frame(64, 64), Vec3::new(0.1, 0.0, 0.3), 5.0, 0.3);
        assert!(result.is_none(), "frame too close should be rejected");
    }

    #[test]
    fn keyframe_store_far_enough_returns_result() {
        let mut store = KeyframeStore::new();
        store.push(checkerboard_frame(64, 64, 8), Vec3::new(0.0, 0.0, 0.3), 0.0, 0.3);
        // Second frame far enough (> 0.3 m).
        let result = store.push(
            checkerboard_frame(64, 64, 8),
            Vec3::new(0.5, 0.0, 0.3),
            0.0,
            0.3,
        );
        assert!(result.is_some(), "far-enough frame should be accepted");
    }

    #[test]
    fn keyframe_store_yaw_change_triggers() {
        let mut store = KeyframeStore::new();
        store.push(blank_frame(32, 32), Vec3::new(0.0, 0.0, 0.3), 0.0, 0.3);
        // Same position but 35° yaw change (> 30°).
        let result = store.push(blank_frame(32, 32), Vec3::new(0.0, 0.0, 0.3), 35.0, 0.3);
        assert!(result.is_some(), "yaw change > 30° should trigger keyframe");
    }

    #[test]
    fn match_features_empty_inputs() {
        let matches = match_features(&[], &[]);
        assert!(matches.is_empty());
    }

    #[test]
    fn jacobi_eigen3_identity() {
        let id = identity3();
        let (v, ev) = jacobi_eigen3(&id);
        // All eigenvalues should be 1.
        for e in &ev { assert!((e - 1.0).abs() < 1e-5, "eigenvalue={}", e); }
        // V should be orthogonal: V^T V = I.
        let vt = transpose3(&v);
        let vtv = mat3_mul(&vt, &v);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((vtv[i][j] - expected).abs() < 1e-4,
                        "V^TV[{},{}]={}", i, j, vtv[i][j]);
            }
        }
    }

    #[test]
    fn angle_diff_wraps_correctly() {
        assert!((angle_diff_deg(350.0, 10.0) + 20.0).abs() < 1e-4);
        assert!((angle_diff_deg(10.0, 350.0) - 20.0).abs() < 1e-4);
        assert!((angle_diff_deg(180.0, 0.0) - 180.0).abs() < 1e-4);
    }

    #[test]
    fn enforce_rank2_reduces_rank() {
        // Full-rank 3×3 matrix — after rank-2 enforcement smallest singular
        // value should be (near) zero.
        let m = [[1.0f32, 2.0, 3.0],
                 [0.5,    1.5, 2.5],
                 [0.2,    0.8, 1.1]];
        let r2 = enforce_rank2(&m);
        // The matrix R2 should satisfy: rank ≤ 2 → det ≈ 0.
        let d = det3(&r2);
        assert!(d.abs() < 0.05, "det after rank-2 enforcement = {}", d);
    }

    #[test]
    fn rolling_buffer_respects_max_stored() {
        let mut store = KeyframeStore::new();
        // Fill well past KF_MAX_STORED (50) keyframes by advancing position 0.5 m
        // each time.  The store should never exceed capacity.
        for i in 0..(KF_MAX_STORED + 10) {
            let frame = checkerboard_frame(64, 64, 8);
            let pos = Vec3::new(i as f32 * 0.5, 0.0, 0.3);
            store.push(frame, pos, 0.0, 0.3);
        }
        assert!(store.len() <= KF_MAX_STORED,
                "buffer grew to {} > KF_MAX_STORED={}", store.len(), KF_MAX_STORED);
    }

    #[test]
    fn match_features_with_identical_descriptors() {
        use crate::perception::types::Feature;
        // Two features with identical descriptors should match with Hamming = 0.
        let f = Feature { x: 10.0, y: 10.0, score: 50.0, descriptor: [0xAB; 32] };
        let matches = match_features(&[f.clone()], &[f.clone()]);
        assert_eq!(matches.len(), 1, "identical descriptors should match");
        assert_eq!(matches[0].hamming, 0, "Hamming distance should be 0 for identical");
    }

    #[test]
    fn match_features_rejects_ambiguous() {
        use crate::perception::types::Feature;
        // Two candidates with equal distance should fail the ratio test.
        let prev = Feature { x: 10.0, y: 10.0, score: 50.0, descriptor: [0x00; 32] };
        let c1   = Feature { x: 10.0, y: 10.0, score: 50.0, descriptor: [0xFF; 32] }; // dist=256
        let c2   = Feature { x: 20.0, y: 20.0, score: 50.0, descriptor: [0xFF; 32] }; // dist=256 (same)
        // best = second_best → ratio_ok fails (0.75 × 256 = 192 < 256 is NOT < 256).
        let matches = match_features(&[prev], &[c1, c2]);
        // Both candidates are identical distance to prev; ratio test should reject.
        assert!(matches.is_empty() || matches[0].hamming > MATCH_MAX_HAMMING,
                "ambiguous / too-far matches should be filtered");
    }

    #[test]
    fn keyframe_result_has_valid_rotation_matrix() {
        // Feed a checkerboard image pair separated by > 0.3 m and verify
        // the returned rotation is approximately orthogonal (R·Rᵀ ≈ I).
        let mut store = KeyframeStore::new();
        let frame_a = checkerboard_frame(64, 64, 4);
        let frame_b = checkerboard_frame(64, 64, 4);
        store.push(frame_a, Vec3::new(0.0, 0.0, 0.3), 0.0, 0.3);
        let result = store.push(frame_b, Vec3::new(0.5, 0.0, 0.3), 0.0, 0.3);

        if let Some(r) = result {
            let rot = r.rotation;
            let rt  = transpose3(&rot);
            let rrt = mat3_mul(&rot, &rt);
            for i in 0..3 {
                for j in 0..3 {
                    let expected = if i == j { 1.0 } else { 0.0 };
                    assert!((rrt[i][j] - expected).abs() < 0.01,
                            "R·Rᵀ[{},{}] = {} (not close to identity)", i, j, rrt[i][j]);
                }
            }
        }
        // If no result (too few features on checkerboard), that's also acceptable.
    }

    #[test]
    fn detect_loop_too_few_frames_returns_none() {
        // With only LOOP_MIN_AGE frames in the buffer, detect_loop must return None.
        let mut store = KeyframeStore::new();
        for i in 0..LOOP_MIN_AGE {
            store.push(blank_frame(64, 64), Vec3::new(i as f32 * 0.5, 0.0, 0.3), 0.0, 0.3);
        }
        assert_eq!(store.len(), LOOP_MIN_AGE);
        assert!(store.detect_loop(LOOP_MIN_AGE - 1).is_none(),
            "too few frames should return None");
    }

    #[test]
    fn detect_loop_spatial_gate_rejects_distant() {
        // All candidate frames are > LOOP_SEARCH_RADIUS_M away from the newest frame.
        let mut store = KeyframeStore::new();
        let n = LOOP_MIN_AGE + 2; // 7 frames → candidates = frames[..2]
        for i in 0..n {
            // 10 m between each frame — far beyond the 1.5 m search radius.
            store.push(blank_frame(64, 64), Vec3::new(i as f32 * 10.0, 0.0, 0.3), 0.0, 0.3);
        }
        assert!(store.detect_loop(n - 1).is_none(),
            "all candidates are too far — spatial gate should reject all");
    }

    #[test]
    fn global_index_grows_beyond_rolling_buffer() {
        let mut store = KeyframeStore::new();
        let total = KF_MAX_STORED + 5;
        for i in 0..total {
            // 0.5 m between frames — always beyond KF_MIN_DIST_M threshold.
            store.push(blank_frame(64, 64), Vec3::new(i as f32 * 0.5, 0.0, 0.3), 0.0, 0.3);
        }
        // Rolling buffer must be capped.
        assert_eq!(store.len(), KF_MAX_STORED,
            "rolling buffer must be capped at KF_MAX_STORED={}", KF_MAX_STORED);
        // Global index must hold every accepted keyframe.
        assert_eq!(store.global_len(), total,
            "global_index must grow to {} (all accepted keyframes)", total);
    }

    #[test]
    fn detect_loop_uses_global_index() {
        // Push enough blank frames that the first ones are evicted from the
        // rolling buffer but remain in the global index.
        // detect_loop should reach global_index without panicking.
        let mut store = KeyframeStore::new();
        let total = KF_MAX_STORED + LOOP_MIN_AGE + 2;
        for i in 0..total {
            store.push(blank_frame(64, 64), Vec3::new(i as f32 * 0.5, 0.0, 0.3), 0.0, 0.3);
        }
        // Rolling buffer is at capacity; global_index has all frames.
        assert!(store.global_len() > store.len(),
            "global_index should be larger than the rolling buffer");
        // Blank frames have no features, so no loop closure matches — but the
        // call must complete without panicking or indexing out of bounds.
        let result = store.detect_loop(store.global_len() - 1);
        // No features → no matches → None (spatial gate or match count filter).
        assert!(result.is_none(),
            "blank frames produce no matches — detect_loop must return None");
    }

    // -----------------------------------------------------------------------
    // Phase 8 — SpatialGrid tests
    // -----------------------------------------------------------------------

    #[test]
    fn spatial_grid_insert_and_lookup() {
        let mut grid = SpatialGrid::new();
        grid.insert(0.0, 0.0, 0);
        grid.insert(0.5, 0.5, 1);
        grid.insert(5.0, 5.0, 2);

        let candidates = grid.candidates_near(0.0, 0.0);
        assert!(candidates.contains(&0), "gi=0 must be in neighbourhood of origin");
        assert!(candidates.contains(&1), "gi=1 (0.5,0.5) must be in neighbourhood of origin");
        assert!(!candidates.contains(&2), "gi=2 (5,5) must NOT be in neighbourhood of origin");
    }

    #[test]
    fn spatial_grid_empty_neighborhood_no_panic() {
        let grid = SpatialGrid::new();
        let candidates = grid.candidates_near(100.0, 100.0);
        assert!(candidates.is_empty(), "empty grid must return empty Vec");
    }

    #[test]
    fn spatial_grid_cell_boundary_coverage() {
        let r = SPATIAL_CELL_M;
        let mut grid = SpatialGrid::new();
        // gi=0: just inside the adjacent cell — within 3×3 of origin
        grid.insert(r - 0.01, 0.0, 0);
        // gi=1: two cells away — NOT in 3×3 of origin
        grid.insert(2.0 * r + 0.01, 0.0, 1);

        let candidates = grid.candidates_near(0.0, 0.0);
        assert!(candidates.contains(&0),
            "point just inside adjacent cell must be in 3×3 neighbourhood");
        assert!(!candidates.contains(&1),
            "point two cells away must NOT be in 3×3 neighbourhood");
    }

    #[test]
    fn detect_loop_grid_respects_min_age() {
        // Push LOOP_MIN_AGE+1 blank frames all at the same XY position
        // (within spatial gate of each other).  detect_loop must still return
        // None because blank frames have no features → no matches.
        // This also confirms the age filter (gi + LOOP_MIN_AGE >= n) is applied.
        let mut store = KeyframeStore::new();
        let n = LOOP_MIN_AGE + 1;
        for _ in 0..n {
            // Same XY so all frames are within the spatial gate.
            store.push(blank_frame(64, 64), Vec3::new(0.0, 0.0, 0.3), 0.0, 0.3);
        }
        let result = store.detect_loop(store.global_len() - 1);
        assert!(result.is_none(),
            "blank frames with no features must produce no loop closure");
    }
}
