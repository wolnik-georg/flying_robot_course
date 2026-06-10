//! Richter et al. (2016) trajectory planner — Mode 1.
//!
//! Extends the Mellinger/`SplineTrajectory` (Mode 0) minimum-snap QP with:
//!   1. **Automatic time allocation** via a single aggressiveness scalar `k_t`.
//!      No manual segment durations needed.  Higher `k_t` → faster, snappier flight.
//!   2. **Post-solve feasibility checking** against thrust and angular-rate limits.
//!
//! The polynomial basis, Gram matrix, and Clarabel QP solver are reused from
//! `spline.rs`; only the time-allocation outer loop is new.
//!
//! ## Usage
//!
//! ```ignore
//! use crate::planning::richter::{RichterTrajectory, RichterWaypoint};
//! use crate::math::Vec3;
//!
//! let waypoints = vec![
//!     RichterWaypoint { pos: Vec3::new(0.0, 0.0, 1.0), yaw: 0.0 },
//!     RichterWaypoint { pos: Vec3::new(1.0, 0.0, 1.0), yaw: 0.0 },
//!     RichterWaypoint { pos: Vec3::new(1.0, 1.0, 1.0), yaw: 0.0 },
//! ];
//!
//! let traj = RichterTrajectory::plan(&waypoints, 1.0, false).unwrap();
//! let flat = traj.eval(traj.total_time / 2.0);
//! let report = traj.check_feasibility(0.027, 0.55, 10.0);
//! assert!(report.feasible, "trajectory violates limits");
//! ```
//!
//! ## Time allocation derivation (Richter §3.4)
//!
//! The combined cost is:
//!   J_total = Σ_i J_snap_i(T_i) + k_t · Σ_i T_i
//!
//! where J_snap_i = (1/T_i^7) · ∫₀¹ (p''''(t))² dt for segment i.
//!
//! Gradient:  g_i = −7·J_i/T_i + k_t
//! Fixed-point (g_i = 0):  T_i* = 7·J_i / k_t
//!
//! Each iteration solves the QP with current T_i, computes J_i from the
//! resulting coefficients, then moves T_i toward T_i* with damping α = 0.3.
//!
//! ## Reference
//!   Richter, C., Bry, A., Roy, N. (2016).
//!   *Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense
//!   Indoor Environments.* In: Robotics Research, Springer, pp. 649–666.

use crate::math::Vec3;
use super::flatness::{FlatOutput, compute_flatness};
use super::spline::{SplineTrajectory, SplineSegment, Waypoint};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Minimum allowed segment duration [s].  Prevents degenerate QP scaling.
/// With 1/T^7 in the objective, T=0.05s gives a factor of ~1.3e8 which causes
/// Clarabel numerical issues.  0.15s keeps the factor below ~1.4e4.
const T_MIN: f32 = 0.15;

/// Feasibility check sample interval [s].
const SAMPLE_DT: f32 = 0.02; // 50 Hz

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// A waypoint for the Richter planner — identical to `spline::Waypoint`.
/// Re-exported for ergonomic use without needing to import `spline` directly.
pub type RichterWaypoint = Waypoint;

/// Report from a feasibility check against physical actuator limits.
#[derive(Debug, Clone)]
pub struct FeasibilityReport {
    /// Peak thrust [N] along the trajectory.
    pub max_thrust_n: f32,
    /// Peak angular-rate magnitude [rad/s] along the trajectory.
    pub max_omega_rad_s: f32,
    /// `true` if both limits are respected everywhere.
    pub feasible: bool,
    /// Multiply `total_time` by this factor to (approximately) make the
    /// trajectory feasible.  1.0 when already feasible.
    pub suggested_time_scale: f32,
}

/// Richter-style trajectory: minimum-snap with automatic time allocation.
///
/// Internally stores a `SplineTrajectory` and delegates all evaluation to it.
/// The only difference from Mode 0 (`SplineTrajectory`) is that segment times
/// are chosen automatically via `k_t` gradient descent rather than manually.
#[derive(Debug, Clone)]
pub struct RichterTrajectory {
    inner: SplineTrajectory,
    /// Per-segment durations after time allocation [s].
    pub segment_times: Vec<f32>,
    /// Total trajectory duration [s].
    pub total_time: f32,
    /// Aggressiveness parameter used during planning.
    pub k_t: f32,
}

// ---------------------------------------------------------------------------
// Core implementation
// ---------------------------------------------------------------------------

impl RichterTrajectory {
    /// Plan a minimum-snap trajectory through `waypoints`.
    ///
    /// Uses distance-proportional time allocation (`T_i = dist_i / v_avg(k_t)`)
    /// without iterative refinement.  This is numerically robust and sufficient
    /// for segments of comparable length.  For segments with very different
    /// complexities, call `plan_with_iters` with a small `n_iter` (1–3) as an
    /// opt-in optimisation step.
    ///
    /// # Parameters
    /// - `waypoints`: sequence of (position, yaw) waypoints.
    /// - `k_t`: aggressiveness.
    ///   - `k_t ≈ 0.1` → slow, ~1 m/s average (similar to current spline defaults)
    ///   - `k_t ≈ 1.0` → moderate, ~2 m/s average
    ///   - `k_t ≈ 10.0` → fast, ~5 m/s average (Richter's 8 m/s experiments)
    /// - `periodic`: if `true`, enforce smooth looping (first == last waypoint pos).
    pub fn plan(
        waypoints: &[RichterWaypoint],
        k_t: f32,
        periodic: bool,
    ) -> Result<Self, String> {
        Self::plan_with_iters(waypoints, k_t, periodic, 0)
    }

    /// Paper-aligned variant: same Richter formulation but with nonzero timing iterations.
    ///
    /// This keeps the same `k_t` semantics while running the time-redistribution loop,
    /// giving stronger position-time coupling than the default single-pass allocation.
    pub fn plan_paper(
        waypoints: &[RichterWaypoint],
        k_t: f32,
        periodic: bool,
    ) -> Result<Self, String> {
        // A small fixed number is stable in practice and keeps runtime predictable.
        Self::plan_with_iters(waypoints, k_t, periodic, 8)
    }

    /// Same as `plan` but with explicit iteration count for the time-redistribution loop.
    ///
    /// **Note**: The Clarabel QP solver can exhibit numerical instability when called
    /// many times in sequence within a single process (a known limitation of the
    /// `simple_qp` / Clarabel backend in debug builds).  Keep `n_iter` ≤ 3 for
    /// reliability.  `plan()` uses `n_iter = 0` (init-times only) which is robust.
    pub fn plan_with_iters(
        waypoints: &[RichterWaypoint],
        k_t: f32,
        periodic: bool,
        n_iter: usize,
    ) -> Result<Self, String> {
        if waypoints.len() < 2 {
            return Err("need at least 2 waypoints".to_string());
        }
        let k_t = k_t.max(1e-6_f32);

        // Step 1: distance-based initial time guess
        let times = init_times(waypoints, k_t, periodic);

        // Step 2: gradient-descent time allocation
        let times = optimize_times(waypoints, times, k_t, n_iter, periodic)?;

        // Step 3: final QP solve with optimised times
        let inner = SplineTrajectory::plan(waypoints, &times, periodic)?;
        let total_time = times.iter().sum();

        Ok(RichterTrajectory { inner, segment_times: times, total_time, k_t })
    }

    /// Build a Richter trajectory with **manually-specified** segment durations.
    ///
    /// Skips the automatic `k_t`-based time allocation (which requires segment
    /// arc-lengths above a threshold to stay numerically stable).  Useful when
    /// the waypoints are so closely spaced that `init_times` would clamp all
    /// segments to `T_MIN = 0.15 s`, making the subsequent min-snap QP degenerate.
    ///
    /// With `n_iter = 0` the result is identical to `SplineTrajectory::plan` with
    /// the same times; set `n_iter > 0` to run Richter's gradient-descent
    /// redistribution starting from the provided times.
    pub fn plan_from_times(
        waypoints: &[RichterWaypoint],
        times: &[f32],
        k_t: f32,
        periodic: bool,
        n_iter: usize,
    ) -> Result<Self, String> {
        if waypoints.len() < 2 {
            return Err("need at least 2 waypoints".to_string());
        }
        if times.len() != waypoints.len() - 1 {
            return Err(format!(
                "plan_from_times: need {} times for {} waypoints, got {}",
                waypoints.len() - 1, waypoints.len(), times.len()
            ));
        }
        let k_t = k_t.max(1e-6_f32);
        let times = times.iter().map(|&t| t.max(T_MIN)).collect::<Vec<_>>();
        let times = optimize_times(waypoints, times, k_t, n_iter, periodic)?;
        let inner = SplineTrajectory::plan(waypoints, &times, periodic)?;
        let total_time = times.iter().sum();
        Ok(RichterTrajectory { inner, segment_times: times, total_time, k_t })
    }

    /// Same as `plan` (Mode 1, 0 redistribution iters) but pins acceleration at
    /// selected waypoints.  Richter's time allocation runs unchanged; only the
    /// final QP solve uses `plan_with_accel_pins`.
    ///
    /// `accel_pins`: `(waypoint_index, acceleration_m_s2)` pairs — same semantics
    /// as `SplineTrajectory::plan_with_accel_pins`.
    pub fn plan_with_accel_pins(
        waypoints: &[RichterWaypoint],
        k_t: f32,
        accel_pins: &[(usize, Vec3)],
        periodic: bool,
    ) -> Result<Self, String> {
        Self::plan_accel_pins_with_iters(waypoints, k_t, accel_pins, periodic, 0)
    }

    /// Same as `plan_paper` (Mode 3, 8 redistribution iters) but pins acceleration
    /// at selected waypoints.
    pub fn plan_paper_with_accel_pins(
        waypoints: &[RichterWaypoint],
        k_t: f32,
        accel_pins: &[(usize, Vec3)],
        periodic: bool,
    ) -> Result<Self, String> {
        Self::plan_accel_pins_with_iters(waypoints, k_t, accel_pins, periodic, 8)
    }

    fn plan_accel_pins_with_iters(
        waypoints: &[RichterWaypoint],
        k_t: f32,
        accel_pins: &[(usize, Vec3)],
        periodic: bool,
        n_iter: usize,
    ) -> Result<Self, String> {
        if waypoints.len() < 2 {
            return Err("need at least 2 waypoints".to_string());
        }
        let k_t = k_t.max(1e-6_f32);
        let times = init_times(waypoints, k_t, periodic);
        // Round to nearest 10 ms: init_times produces irrational f32 values (dist/v_avg)
        // whose 1/T^7 scale factors cause Clarabel KKT-matrix instability in the
        // more-constrained accel-pin QP.  Clean 10 ms increments stay well-conditioned
        // while preserving k_t aggressiveness discrimination across the full range.
        let times: Vec<f32> = times.iter().map(|&t| (t * 100.0).round() / 100.0).collect();
        let times = optimize_times(waypoints, times, k_t, n_iter, periodic)?;
        let inner = SplineTrajectory::plan_with_accel_pins(waypoints, &times, accel_pins, periodic)?;
        let total_time = times.iter().sum();
        Ok(RichterTrajectory { inner, segment_times: times, total_time, k_t })
    }

    /// Same as `plan_from_times` but pins acceleration at selected waypoints.
    ///
    /// Use this for Mode 3 (paper) or dense-waypoint cases where timing is
    /// specified manually, combined with accel-pinned inversion at key waypoints.
    pub fn plan_from_times_with_accel_pins(
        waypoints: &[RichterWaypoint],
        times: &[f32],
        k_t: f32,
        accel_pins: &[(usize, Vec3)],
        periodic: bool,
        n_iter: usize,
    ) -> Result<Self, String> {
        if waypoints.len() < 2 {
            return Err("need at least 2 waypoints".to_string());
        }
        if times.len() != waypoints.len() - 1 {
            return Err(format!(
                "plan_from_times_with_accel_pins: need {} times for {} waypoints, got {}",
                waypoints.len() - 1, waypoints.len(), times.len()
            ));
        }
        let k_t = k_t.max(1e-6_f32);
        let times = times.iter().map(|&t| t.max(T_MIN)).collect::<Vec<_>>();
        let times = optimize_times(waypoints, times, k_t, n_iter, periodic)?;
        let inner = SplineTrajectory::plan_with_accel_pins(waypoints, &times, accel_pins, periodic)?;
        let total_time = times.iter().sum();
        Ok(RichterTrajectory { inner, segment_times: times, total_time, k_t })
    }

    /// Evaluate flat outputs at physical time `t` (clamped to [0, total_time]).
    ///
    /// Drop-in replacement for `SplineTrajectory::eval`.
    #[inline]
    pub fn eval(&self, t: f32) -> FlatOutput {
        self.inner.eval(t)
    }

    /// Check whether the trajectory respects thrust and angular-rate limits.
    ///
    /// Samples the trajectory at 50 Hz, computes flatness at each point, and
    /// records the peak thrust and angular-rate magnitude.
    ///
    /// # Parameters
    /// - `mass`: drone mass [kg]
    /// - `max_thrust_n`: maximum total thrust [N]
    /// - `max_omega_rad_s`: maximum angular-rate magnitude [rad/s]
    pub fn check_feasibility(
        &self,
        mass: f32,
        max_thrust_n: f32,
        max_omega_rad_s: f32,
    ) -> FeasibilityReport {
        let n_steps = (self.total_time / SAMPLE_DT).ceil() as usize + 1;
        let mut peak_thrust: f32 = 0.0;
        let mut peak_omega: f32 = 0.0;

        for k in 0..n_steps {
            let t = (k as f32 * SAMPLE_DT).min(self.total_time);
            let flat = self.eval(t);
            let fr = compute_flatness(&flat, mass);
            let omega_mag = (fr.omega.x * fr.omega.x
                + fr.omega.y * fr.omega.y
                + fr.omega.z * fr.omega.z)
                .sqrt();
            peak_thrust = peak_thrust.max(fr.thrust);
            peak_omega = peak_omega.max(omega_mag);
        }

        let feasible = peak_thrust <= max_thrust_n && peak_omega <= max_omega_rad_s;

        // Estimate how much to scale total_time to bring within limits.
        // Thrust  ∝ 1/T² → need T_scale ≥ sqrt(peak_thrust / max_thrust)
        // Omega   ∝ 1/T  → need T_scale ≥ peak_omega / max_omega
        let scale_thrust = if peak_thrust > max_thrust_n {
            (peak_thrust / max_thrust_n).sqrt()
        } else {
            1.0
        };
        let scale_omega = if peak_omega > max_omega_rad_s {
            peak_omega / max_omega_rad_s
        } else {
            1.0
        };
        let suggested_time_scale = scale_thrust.max(scale_omega);

        FeasibilityReport {
            max_thrust_n: peak_thrust,
            max_omega_rad_s: peak_omega,
            feasible,
            suggested_time_scale,
        }
    }

    /// Return a reference to the underlying `SplineTrajectory` segments.
    /// Useful for inspecting polynomial coefficients or plotting.
    pub fn segments(&self) -> &[SplineSegment] {
        &self.inner.segments
    }

    /// Return a reference to the underlying `SplineTrajectory`.
    /// Used by flight binaries to serialise position coefficients for firmware upload.
    pub fn as_spline(&self) -> &SplineTrajectory {
        &self.inner
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute the physical-time snap integral for one polynomial axis.
///
///   J_phys = (1/T^7) · ∫₀¹ (p''''(t))² dt
///          = (1/T^7) · Σᵢⱼ Q[i][j] · c[i] · c[j]   (i,j ∈ 4..=8)
///
/// where Q is the 5×5 Gram matrix for the 4th-derivative basis on [0,1].
fn snap_cost_physical(c: &[f32; 9], duration: f32) -> f32 {
    // Derivative scaling factors: d⁴/dt⁴ (aᵢ tⁱ) = i!/(i-4)! · aᵢ · t^{i-4}
    // for i = 4..8: factors = [4!, 5!/1!, 6!/2!, 7!/3!, 8!/4!] = [24,120,360,840,1680]
    const FACTORS: [f32; 5] = [24.0, 120.0, 360.0, 840.0, 1680.0];

    let mut cost = 0.0_f32;
    for i in 0..5 {
        for j in 0..5 {
            // Gram entry: ∫₀¹ t^i · t^j dt = 1/(i+j+1)
            let gram = 1.0 / (i + j + 1) as f32;
            cost += FACTORS[i] * FACTORS[j] * gram * c[i + 4] * c[j + 4];
        }
    }
    cost / duration.powi(7)
}

/// Sum snap cost over x, y, z axes for one segment.
fn segment_snap_cost(seg: &SplineSegment) -> f32 {
    snap_cost_physical(&seg.cx, seg.duration)
        + snap_cost_physical(&seg.cy, seg.duration)
        + snap_cost_physical(&seg.cz, seg.duration)
}

/// Distance between consecutive waypoints.
fn waypoint_dist(a: &RichterWaypoint, b: &RichterWaypoint) -> f32 {
    let dx = b.pos.x - a.pos.x;
    let dy = b.pos.y - a.pos.y;
    let dz = b.pos.z - a.pos.z;
    (dx * dx + dy * dy + dz * dz).sqrt().max(0.01) // avoid zero-length segments
}

/// Initial time allocation based on distance and aggressiveness.
///
/// Average speed heuristic:  v_avg = 0.5 + 1.5 · sqrt(k_t)  [m/s]
///   k_t = 0.1 → v_avg ≈ 1.0 m/s  (slow, like current spline defaults)
///   k_t = 1.0 → v_avg = 2.0 m/s  (moderate)
///   k_t = 10  → v_avg ≈ 5.2 m/s  (fast)
fn init_times(waypoints: &[RichterWaypoint], k_t: f32, _periodic: bool) -> Vec<f32> {
    // Always n_wp - 1 segments, matching SplineTrajectory::plan expectation.
    let n_seg = waypoints.len() - 1;
    let v_avg = 0.5_f32 + 1.5 * k_t.sqrt();

    (0..n_seg)
        .map(|i| {
            // waypoints[i+1] always valid since i+1 <= n_seg = n_wp-1 < n_wp
            let dist = waypoint_dist(&waypoints[i], &waypoints[i + 1]);
            (dist / v_avg).max(T_MIN)
        })
        .collect()
}

/// Time allocation via proportional redistribution (Richter §3.4, scale-invariant form).
///
/// The Lagrange optimality condition for fixed total time is:
///   J_i / T_i = constant   ⟹   T_i ∝ J_i
///
/// Update (damping α = 0.5):
///   T_i ← T_i + α · (T_total · J_i/J_total − T_i)
///
/// This is purely ratio-based: only J_i/J_total matters, so the huge absolute
/// values of physical snap cost cancel out.  Total time is held approximately
/// constant; k_t controls speed via init_times.
///
/// Each iteration re-solves the QP with current T_i to get updated J_i.
fn optimize_times(
    waypoints: &[RichterWaypoint],
    mut times: Vec<f32>,
    _k_t: f32,
    n_iter: usize,
    periodic: bool,
) -> Result<Vec<f32>, String> {
    const ALPHA: f32 = 0.5; // damping: must be in (0, 1)

    let t_total: f32 = times.iter().sum();

    for _ in 0..n_iter {
        // Solve QP with current times; stop redistribution if solver fails (rare
        // numerical issue with Clarabel on certain time configurations)
        let traj = match SplineTrajectory::plan(waypoints, &times, periodic) {
            Ok(t) => t,
            Err(_) => break, // use current times for the final solve
        };

        // Collect per-segment snap costs (scale-invariant ratio is all that matters)
        let j: Vec<f32> = traj
            .segments
            .iter()
            .map(|s| segment_snap_cost(s).max(1e-30_f32))
            .collect();
        let j_total: f32 = j.iter().sum();

        // Redistribute: move each T_i toward T_total * J_i/J_total (Lagrange target)
        let n_seg = times.len();
        let t_avg = t_total / n_seg as f32;
        for (i, t_i) in times.iter_mut().enumerate() {
            let target = (t_total * j[i] / j_total).max(T_MIN);
            *t_i = *t_i + ALPHA * (target - *t_i);
            // Also enforce: no segment shorter than 10% of average (prevents extreme ratios
            // that produce 1/T^7 objective factors large enough to confuse Clarabel)
            *t_i = t_i.max(T_MIN).max(t_avg * 0.1);
        }
    }
    Ok(times)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;

    fn three_wp() -> Vec<RichterWaypoint> {
        vec![
            RichterWaypoint { pos: Vec3::new(0.0, 0.0, 1.0), yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new(1.0, 0.0, 1.0), yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new(1.0, 1.0, 1.0), yaw: 0.0 },
        ]
    }

    // ── T1: passes through waypoints ────────────────────────────────────────
    #[test]
    fn richter_passes_through_waypoints() {
        let wps = three_wp();
        let traj = RichterTrajectory::plan(&wps, 1.0, false).expect("plan failed");

        let start = traj.eval(0.0);
        let mid   = traj.eval(traj.segment_times[0]);
        let end   = traj.eval(traj.total_time);

        assert!((start.pos.x - 0.0).abs() < 1e-3, "start x: {}", start.pos.x);
        assert!((start.pos.z - 1.0).abs() < 1e-3, "start z: {}", start.pos.z);
        assert!((mid.pos.x   - 1.0).abs() < 1e-2, "mid x:   {}", mid.pos.x);
        assert!((end.pos.y   - 1.0).abs() < 1e-3, "end y:   {}", end.pos.y);
    }

    // ── T2: rest-to-rest boundary conditions ────────────────────────────────
    #[test]
    fn richter_zero_velocity_at_endpoints() {
        let wps = three_wp();
        let traj = RichterTrajectory::plan(&wps, 1.0, false).expect("plan failed");

        let start = traj.eval(0.0);
        let end   = traj.eval(traj.total_time);

        assert!(start.vel.x.abs() < 1e-3, "start vx: {}", start.vel.x);
        assert!(start.vel.y.abs() < 1e-3, "start vy: {}", start.vel.y);
        assert!(end.vel.x.abs()   < 1e-3, "end vx:   {}", end.vel.x);
        assert!(end.vel.y.abs()   < 1e-3, "end vy:   {}", end.vel.y);
    }

    // ── T3: higher k_t → shorter total time ─────────────────────────────────
    #[test]
    fn richter_higher_kt_gives_shorter_time() {
        let wps = three_wp();
        let slow = RichterTrajectory::plan(&wps, 0.1, false).expect("slow");
        // k_t=2 → v_avg≈2.6 m/s → T≈0.38s per segment (well above Clarabel's ~0.25s threshold)
        let fast = RichterTrajectory::plan(&wps, 2.0, false).expect("fast");

        println!("slow total_time={:.3}s  fast total_time={:.3}s", slow.total_time, fast.total_time);
        assert!(
            fast.total_time < slow.total_time,
            "k_t=2 ({:.3}s) should be faster than k_t=0.1 ({:.3}s)",
            fast.total_time, slow.total_time
        );
    }

    // ── T4: feasibility check on CF2.1 params ───────────────────────────────
    #[test]
    fn richter_feasibility_slow_trajectory() {
        let wps = three_wp();
        // Very slow (k_t=0.1, ~1 m/s): should easily be feasible for CF2.1
        let traj = RichterTrajectory::plan(&wps, 0.1, false).expect("plan");
        let report = traj.check_feasibility(
            0.027,   // mass [kg]
            0.55,    // max total thrust [N] (≈ 4 × motor_max)
            15.0,    // max omega [rad/s]
        );
        println!(
            "Feasibility: thrust={:.3}N omega={:.2}rad/s feasible={} scale={:.2}",
            report.max_thrust_n, report.max_omega_rad_s, report.feasible,
            report.suggested_time_scale
        );
        assert!(
            report.feasible,
            "slow trajectory should be feasible: thrust={:.3}N omega={:.2}rad/s",
            report.max_thrust_n, report.max_omega_rad_s
        );
    }

    // ── T5: periodic trajectory closes smoothly ──────────────────────────────
    #[test]
    fn richter_periodic_closes() {
        // First == last position for periodic
        let wps = vec![
            RichterWaypoint { pos: Vec3::new(0.5, 0.0, 1.0), yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new(0.0, 0.5, 1.0), yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new(-0.5, 0.0, 1.0), yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new(0.0, -0.5, 1.0), yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new(0.5, 0.0, 1.0), yaw: 0.0 },  // closes
        ];
        let traj = RichterTrajectory::plan(&wps, 1.0, true).expect("periodic");
        // Start and end should be the same position
        let start = traj.eval(0.0);
        let end   = traj.eval(traj.total_time);
        assert!((start.pos.x - end.pos.x).abs() < 1e-2, "periodic x mismatch");
        assert!((start.pos.y - end.pos.y).abs() < 1e-2, "periodic y mismatch");
    }

    // ── T6: snap_cost_physical is positive and scales correctly ─────────────
    #[test]
    fn snap_cost_scales_with_duration() {
        // For fixed polynomial shape, physical snap cost should decrease as T increases
        let c: [f32; 9] = [0.0, 0.0, 0.0, 0.0, 1.0, -4.0, 6.0, -4.0, 1.0];
        let j1 = snap_cost_physical(&c, 1.0);
        let j2 = snap_cost_physical(&c, 2.0);
        assert!(j1 > 0.0, "snap cost should be positive");
        assert!(
            j1 > j2,
            "longer duration should reduce snap cost: j(T=1)={:.4} j(T=2)={:.4}",
            j1, j2
        );
        // Expected ratio: j1/j2 ≈ 2^7 = 128 (from 1/T^7 scaling)
        let ratio = j1 / j2;
        assert!(
            (ratio - 128.0).abs() < 1.0,
            "snap cost ratio should be ~128 (2^7), got {:.2}",
            ratio
        );
    }

    // ── T7: accel-pinned teardrop achieves inversion at apex ─────────────────
    #[test]
    fn richter_accel_pin_teardrop_inverts_at_apex() {
        use crate::planning::compute_flatness;

        const G: f32 = 9.81;
        const Z: f32 = 0.5;
        let r = 0.20_f32;
        let h = 1.30_f32;

        let wps = vec![
            RichterWaypoint { pos: Vec3::new( 0.0, 0.0, Z),            yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new( r,   0.0, Z + h * 0.20), yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new( 0.0, 0.0, Z + h * 0.60), yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new( 0.0, 0.0, Z + h),        yaw: 0.0 }, // apex
            RichterWaypoint { pos: Vec3::new( 0.0, 0.0, Z + h * 0.60), yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new(-r,   0.0, Z + h * 0.20), yaw: 0.0 },
            RichterWaypoint { pos: Vec3::new( 0.0, 0.0, Z),            yaw: 0.0 },
        ];
        // pin a_z = -1.5g at WP3 (apex) → flatness yields body_z = (0,0,-1) (fully inverted)
        let pins = vec![(3usize, Vec3::new(0.0, 0.0, -1.5 * G))];

        // Proportional durations (same formula as teardrop_waypoints): avoids Richter
        // auto-allocation clamping t01 to T_MIN=0.15s which makes the QP infeasible.
        let d01 = (r * r + (h * 0.20) * (h * 0.20)).sqrt();
        let d12 = (r * r + (h * 0.40) * (h * 0.40)).sqrt();
        let d23 = h * 0.40;
        let seg_t = 0.20_f32;
        let t01 = seg_t.max(0.15);
        let t12 = ((d12 / d01) * seg_t).max(0.15);
        let t23 = ((d23 / d01) * seg_t).max(0.15);
        let durs = vec![t01, t12, t23, t23, t12, t01];

        let traj = RichterTrajectory::plan_from_times_with_accel_pins(&wps, &durs, 1.4, &pins, true, 0)
            .expect("teardrop plan failed");

        // Apex is at WP3 = end of segment 2
        let t_apex = traj.segment_times[0] + traj.segment_times[1] + traj.segment_times[2];
        let flat = traj.eval(t_apex);
        let fr = compute_flatness(&flat, 0.027);

        // body_z world-frame z-component: rot is row-major, column [:][2] is body_z
        let body_z_z = fr.rot[2][2];
        assert!(
            body_z_z < -0.9,
            "apex should be fully inverted (body_z.z < -0.9), got {:.4}",
            body_z_z
        );
    }
}
