//! Visual odometry trajectory accumulator.
//!
//! Chains consecutive [`KeyframeResult`] relative poses into a running global
//! VO position estimate.  The accumulated position drifts slowly over time;
//! the drift uncertainty is tracked in [`VoTrajectory::sigma_xy`] so that the
//! MEKF VO update (`mekf_update_vo`) can weight the measurement appropriately.
//!
//! ## Coordinate convention
//!
//! `KeyframeResult.translation_m` is expressed in the **camera frame of the
//! previous keyframe**.  The AI Deck HiMax HM01B0 camera uses the convention
//! established in `perception/sensors/sim.rs`:
//!
//! ```text
//! cam_x  =  +body_y   (image left → right)
//! cam_y  =  −body_z   (image top  → down)
//! cam_z  =  +body_x   (optical axis, forward)
//! ```
//!
//! Inverse (camera → body):
//! ```text
//! body_x =  cam_z
//! body_y =  cam_x
//! body_z = −cam_y
//! ```
//!
//! [`VoTrajectory`] applies this remapping before rotating into the world frame.
//! The world frame follows the MEKF / EKF convention: +X = initial forward,
//! +Y = left, +Z = up.
//!
//! ## Orientation accumulation
//!
//! `KeyframeResult.rotation` is `R_{curr ← prev}` in the passive (coordinate-
//! transform) convention: `v_curr = R * v_prev`.
//!
//! The accumulated orientation `self.orientation = R_{world ← cam_k}` is updated
//! after each integration step as:
//!
//! ```text
//! R_{world ← cam_{k+1}} = R_{world ← cam_k} · R_{curr ← prev}^T
//! ```
//!
//! This allows the translation `t_prev` (in the previous camera frame) to be
//! rotated into the world frame as `t_world = self.orientation * remap(t_prev)`.
//!
//! ## Usage
//!
//! ```rust,no_run
//! use multirotor_simulator::mapping::VoTrajectory;
//! use multirotor_simulator::math::Vec3;
//!
//! let mut vo = VoTrajectory::new();
//! // Seed with the MEKF position at the first keyframe.
//! vo.seed(Vec3::new(0.0, 0.0, 0.3));
//! // In the AI-deck keyframe callback:
//! // if let Some(pos) = vo.integrate(&kf_result) { ... }
//! ```

use crate::math::Vec3;
use crate::mapping::keyframe::KeyframeResult;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Fractional position uncertainty per metre of VO translation.
/// 10 % drift is conservative for BRIEF+FAST visual odometry on indoor floors.
const VO_DRIFT_FRAC: f32 = 0.10;

/// Maximum accumulated VO position uncertainty [m].
/// When `sigma_xy` reaches this value the MEKF update is gated out — the VO
/// estimate has drifted too far to be trusted without a position fix.
pub const VO_MAX_SIGMA: f32 = 0.50;

/// Minimum number of essential-matrix inliers required for integration.
/// Below 6 the cheirality test may select a degenerate R/t decomposition.
const MIN_INLIERS: usize = 6;

/// Minimum translation magnitude [m] to trigger integration.
/// Skips rotation-only keyframe pairs (e.g. during a 360° scan) where the
/// essential matrix is ill-conditioned due to the pure-rotation degeneracy.
const MIN_TRANSLATION_M: f32 = 0.05;

// ---------------------------------------------------------------------------
// VoTrajectory
// ---------------------------------------------------------------------------

/// Integrates consecutive [`KeyframeResult`] relative poses into a running
/// global VO trajectory.
///
/// Call [`seed`](Self::seed) once with a trusted MEKF position before the
/// first frame arrives, then call [`integrate`](Self::integrate) for every
/// new [`KeyframeResult`].  After a successful MEKF VO fusion call
/// [`reset_to`](Self::reset_to) to reseed from the corrected filter estimate.
pub struct VoTrajectory {
    /// Current accumulated VO position in world frame [m].
    pub position: Vec3,
    /// Accumulated orientation: R_{world ← current_camera}.
    /// Starts as identity (camera roughly aligned with world at seed time).
    orientation: [[f32; 3]; 3],
    /// Number of keyframes successfully integrated.
    pub kf_count: usize,
    /// Accumulated position uncertainty [m].  Grows proportionally to the
    /// total translated distance; fed as `r_vo = sigma_xy²` into the MEKF.
    pub sigma_xy: f32,
    /// Whether `seed()` has been called.
    initialized: bool,
}

impl VoTrajectory {
    /// Create an uninitialised accumulator.  Must call `seed()` before `integrate()`.
    pub fn new() -> Self {
        Self {
            position:    Vec3::new(0.0, 0.0, 0.0),
            orientation: identity3(),
            kf_count:    0,
            sigma_xy:    0.0,
            initialized: false,
        }
    }

    /// Seed the trajectory at a known world-frame position.
    ///
    /// `pos` should be the MEKF position estimate at the time the first
    /// keyframe is captured.  The orientation is reset to identity (camera
    /// approximately aligned with the world frame at hover).
    pub fn seed(&mut self, pos: Vec3) {
        self.position    = pos;
        self.orientation = identity3();
        self.sigma_xy    = 0.0;
        self.kf_count    = 0;
        self.initialized = true;
    }

    /// Integrate one [`KeyframeResult`] and return the updated world-frame
    /// position, or `None` when:
    ///
    /// - [`seed`](Self::seed) has not been called yet.
    /// - `result.inlier_count < MIN_INLIERS` (potentially degenerate solution).
    /// - `|translation_m| < MIN_TRANSLATION_M` (pure-rotation step, skipped).
    pub fn integrate(&mut self, result: &KeyframeResult) -> Option<Vec3> {
        if !self.initialized {
            return None;
        }
        if result.inlier_count < MIN_INLIERS {
            return None;
        }
        let t_cam = result.translation_m;
        let t_mag = (t_cam.x * t_cam.x + t_cam.y * t_cam.y + t_cam.z * t_cam.z).sqrt();
        if t_mag < MIN_TRANSLATION_M {
            return None;
        }

        // Remap translation from camera frame to body frame:
        //   body_x = cam_z,  body_y = cam_x,  body_z = -cam_y
        let t_body = Vec3::new(t_cam.z, t_cam.x, -t_cam.y);

        // Rotate body-frame translation into world frame using the CURRENT
        // orientation (R_{world ← prev_camera}), i.e. before the rotation update.
        let t_world = mat3_mul_vec3(&self.orientation, t_body);
        self.position.x += t_world.x;
        self.position.y += t_world.y;
        self.position.z += t_world.z;

        // Update accumulated orientation: chain relative rotation.
        // R_{world ← curr} = R_{world ← prev} · R_{curr ← prev}^T
        self.orientation = mat3_mul_mat3_bt(&self.orientation, &result.rotation);

        // Grow uncertainty proportionally to translation distance.
        self.sigma_xy = (self.sigma_xy + VO_DRIFT_FRAC * t_mag).min(VO_MAX_SIGMA);
        self.kf_count += 1;

        Some(self.position)
    }

    /// Reset the accumulated position to `pos` and clear sigma.
    ///
    /// Call this after a successful MEKF VO fusion to reseed the trajectory
    /// from the now-corrected filter estimate, preventing drift accumulation.
    /// The accumulated orientation is preserved (position-only correction).
    pub fn reset_to(&mut self, pos: Vec3) {
        self.position = pos;
        self.sigma_xy = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Linear algebra helpers (3×3)
// ---------------------------------------------------------------------------

fn identity3() -> [[f32; 3]; 3] {
    [[1.0, 0.0, 0.0],
     [0.0, 1.0, 0.0],
     [0.0, 0.0, 1.0]]
}

/// Multiply a 3×3 matrix by a 3-vector: m * v.
fn mat3_mul_vec3(m: &[[f32; 3]; 3], v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

/// Compute a * b^T (multiply a by the transpose of b).
fn mat3_mul_mat3_bt(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut c = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            // c[i][j] = row i of a  ·  row j of b  (= col j of b^T)
            for k in 0..3 {
                c[i][j] += a[i][k] * b[j][k];
            }
        }
    }
    c
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_result(
        tx: f32, ty: f32, tz: f32,
        rotation: [[f32; 3]; 3],
        inlier_count: usize,
    ) -> KeyframeResult {
        KeyframeResult {
            kf_index:       1,
            match_count:    inlier_count,
            rotation,
            translation_dir: Vec3::new(0.0, 0.0, 1.0),
            translation_m:   Vec3::new(tx, ty, tz),
            inlier_count,
        }
    }

    fn identity_result(tx: f32, ty: f32, tz: f32) -> KeyframeResult {
        make_result(tx, ty, tz, identity3(), 8)
    }

    // ── Rejection tests ───────────────────────────────────────────────────

    #[test]
    fn not_initialized_returns_none() {
        let mut vo = VoTrajectory::new();
        assert!(vo.integrate(&identity_result(0.3, 0.0, 0.0)).is_none());
    }

    #[test]
    fn low_inlier_count_rejected() {
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(0.0, 0.0, 0.0));
        let r = make_result(0.3, 0.0, 0.0, identity3(), 3); // MIN_INLIERS = 6
        assert!(vo.integrate(&r).is_none());
        assert_eq!(vo.kf_count, 0, "no keyframe should be counted");
    }

    #[test]
    fn small_translation_rejected() {
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(0.0, 0.0, 0.3));
        let r = identity_result(0.01, 0.0, 0.0); // < MIN_TRANSLATION_M = 0.05
        assert!(vo.integrate(&r).is_none());
    }

    // ── Translation mapping ───────────────────────────────────────────────

    #[test]
    fn cam_z_maps_to_world_x() {
        // cam_z = body_x → with identity orientation → world_x.
        // t_cam = (0, 0, 0.3) → t_body = (0.3, 0, 0) → t_world = (0.3, 0, 0).
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(0.0, 0.0, 0.0));
        let r = identity_result(0.0, 0.0, 0.3);
        let p = vo.integrate(&r).unwrap();
        assert!((p.x - 0.3).abs() < 1e-5, "expected world x=0.3, got {}", p.x);
        assert!(p.y.abs() < 1e-5,          "expected world y=0,   got {}", p.y);
    }

    #[test]
    fn cam_x_maps_to_world_y() {
        // cam_x = body_y → with identity orientation → world_y.
        // t_cam = (0.2, 0, 0) → t_body = (0, 0.2, 0) → t_world = (0, 0.2, 0).
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(0.0, 0.0, 0.0));
        let r = identity_result(0.2, 0.0, 0.0);
        let p = vo.integrate(&r).unwrap();
        assert!(p.x.abs() < 1e-5,          "expected world x=0, got {}", p.x);
        assert!((p.y - 0.2).abs() < 1e-5,  "expected world y=0.2, got {}", p.y);
    }

    // ── Accumulation ──────────────────────────────────────────────────────

    #[test]
    fn two_steps_double_displacement() {
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(0.0, 0.0, 0.0));
        let r = identity_result(0.0, 0.0, 0.2);
        vo.integrate(&r).unwrap();
        vo.integrate(&r).unwrap();
        assert!((vo.position.x - 0.4).abs() < 1e-4,
            "expected 0.4 m, got {}", vo.position.x);
        assert_eq!(vo.kf_count, 2);
    }

    #[test]
    fn seed_offset_preserved() {
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(1.0, 2.0, 0.3));
        let r = identity_result(0.0, 0.0, 0.1);
        let p = vo.integrate(&r).unwrap();
        // world_x should be seed_x + cam_z
        assert!((p.x - 1.1).abs() < 1e-5, "expected x=1.1, got {}", p.x);
        assert!((p.y - 2.0).abs() < 1e-5, "expected y=2.0, got {}", p.y);
    }

    // ── Orientation ───────────────────────────────────────────────────────

    #[test]
    fn orientation_rotates_subsequent_translation() {
        // Step 1: rotation R_rel such that the accumulated orientation becomes
        // [[0,-1,0],[1,0,0],[0,0,1]] (maps body_x → world_y).
        //
        // R_{world←curr} = R_{world←prev} · R_{curr←prev}^T
        // With R_{world←prev} = I we need:
        //   [[0,-1,0],[1,0,0],[0,0,1]] = I · R_rel^T
        //   ⇒  R_rel = [[0,-1,0],[1,0,0],[0,0,1]]^T = [[0,1,0],[-1,0,0],[0,0,1]]
        let r_rot: [[f32; 3]; 3] = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(0.0, 0.0, 0.0));

        // Step 1: tiny forward step + rotation (barely above MIN_TRANSLATION_M).
        let step1 = make_result(0.0, 0.0, 0.06, r_rot, 8);
        vo.integrate(&step1).unwrap(); // position ≈ (0.06, 0, 0)

        // Step 2: cam_z=0.3 → body_x=0.3 → rotated by [[0,-1,0],[1,0,0],[0,0,1]] → world_y=0.3.
        let step2 = identity_result(0.0, 0.0, 0.3);
        let p = vo.integrate(&step2).unwrap();
        assert!((p.y - 0.3).abs() < 1e-4,
            "expected world_y ≈ 0.3 after 90° orientation, got {}", p.y);
    }

    // ── Sigma / uncertainty ───────────────────────────────────────────────

    #[test]
    fn sigma_grows_with_distance() {
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(0.0, 0.0, 0.0));
        vo.integrate(&identity_result(0.0, 0.0, 0.3)).unwrap();
        assert!(vo.sigma_xy > 0.0, "sigma should grow after a step");
    }

    #[test]
    fn sigma_capped_at_max() {
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(0.0, 0.0, 0.0));
        // 200 × 0.3 m = 60 m; uncapped sigma would be 6.0, but max is 0.5.
        for _ in 0..200 {
            let _ = vo.integrate(&identity_result(0.0, 0.0, 0.3));
        }
        assert!((vo.sigma_xy - VO_MAX_SIGMA).abs() < 1e-5,
            "sigma should saturate at {}, got {}", VO_MAX_SIGMA, vo.sigma_xy);
    }

    #[test]
    fn reset_clears_sigma_and_updates_position() {
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(0.0, 0.0, 0.0));
        for _ in 0..10 {
            let _ = vo.integrate(&identity_result(0.0, 0.0, 0.3));
        }
        assert!(vo.sigma_xy > 0.0);
        vo.reset_to(Vec3::new(1.0, 2.0, 0.3));
        assert_eq!(vo.sigma_xy, 0.0, "sigma should clear after reset");
        assert!((vo.position.x - 1.0).abs() < 1e-5);
        assert!((vo.position.y - 2.0).abs() < 1e-5);
    }

    #[test]
    fn kf_count_tracks_successful_integrations() {
        let mut vo = VoTrajectory::new();
        vo.seed(Vec3::new(0.0, 0.0, 0.0));
        for _ in 0..5 {
            vo.integrate(&identity_result(0.0, 0.0, 0.3)).unwrap();
        }
        assert_eq!(vo.kf_count, 5);
    }
}
