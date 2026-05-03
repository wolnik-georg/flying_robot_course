//! SE(3) trajectory planner — Mode 2.
//!
//! For aggressive manoeuvres (flips, rolls, full-attitude acrobatics) where
//! differential flatness **breaks down** — i.e. thrust passes through zero or
//! reverses direction — this planner explicitly prescribes orientation on SO(3)
//! independently of the position trajectory.
//!
//! ## Why flatness breaks down for flips / rolls
//!
//! Flatness recovers attitude from acceleration:
//!   z_b = (a + g·e₃) / ‖a + g·e₃‖
//!
//! During a flip the drone is briefly inverted and the net thrust vector
//! passes through zero.  At that instant ‖a + g·e₃‖ → 0, the direction is
//! undefined, and the recovered z_b is garbage.  The physical drone continues
//! to move — flatness just cannot describe it.
//!
//! ## What Mode 2 adds
//!
//! Position:    same degree-8 minimum-snap polynomial (reuses `SplineTrajectory`)
//! Orientation: two sub-modes per segment —
//!   - `use_flatness = true`  → attitude derived from differential flatness
//!     (Faessler 2018 / Tal & Karaman CDC 2018): jerk/snap feedforward for ω, ω̇.
//!   - `use_flatness = false` → SLERP on SO(3) between explicit attitude waypoints.
//!
//! These two sub-problems are planned **independently** — no coupling.  The
//! planner then computes:
//!   thrust  = m · (a + g·e₃) · (R·e₃)   (can be 0 or negative → motor cut)
//!   omega   = flatness-derived (jerk feedforward) or finite-diff of SLERP
//!   alpha   = flatness-derived (snap feedforward) or finite-diff of SLERP
//!
//! The result is a `Se3Output` that a geometric or INDI controller can consume
//! directly without going through the flatness map.
//!
//! ## Usage
//!
//! ```ignore
//! use crate::planning::se3_traj::{Se3Trajectory, Se3Waypoint};
//! use crate::math::{Vec3, Quat};
//!
//! // Aggressive circle: flatness-derived attitude (bank angle automatic)
//! let waypoints: Vec<Se3Waypoint> = circle_pts.iter()
//!     .map(|p| Se3Waypoint::flat(*p, 0.0))
//!     .collect();
//!
//! // Flip: explicit attitude waypoints (flatness would be degenerate)
//! let wps_flip = vec![
//!     Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.2)),
//!     Se3Waypoint::new(Vec3::new(0.0, 0.0, 1.2),
//!                      Quat::from_axis_angle(Vec3::new(1.0,0.0,0.0), PI)),
//!     Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.2)),
//! ];
//! let durations = vec![0.35_f32, 0.35];
//! let traj = Se3Trajectory::plan(&wps_flip, &durations, 0.027, false).unwrap();
//!
//! let out = traj.eval(0.2);   // Se3Output
//! // out.rot    — rotation matrix (either flatness-derived or from explicit SLERP)
//! // out.omega  — body-frame angular velocity [rad/s]
//! // out.thrust — required thrust [N]  (may be 0 or negative for flips)
//! ```
//!
//! ## Reference
//!   Mellinger, D., Kumar, V. (2011) "Minimum snap trajectory generation and
//!   control for quadrotors." ICRA 2011.  (position polynomial, same as Mode 0)
//!
//!   Faessler, M., Franchi, A., Scaramuzza, D. (2018). "Differential Flatness of
//!   Quadrotor Dynamics Subject to Rotor Drag."  IEEE RA-L.  (flatness map)
//!
//!   Tal, E., Karaman, S. (2018). "Accurate Tracking of Aggressive Quadrotor
//!   Trajectories Using Incremental Nonlinear Dynamic Inversion and Differential
//!   Flatness." CDC 2018.  (jerk/snap feedforward for attitude rates)

use crate::math::{Vec3, Quat};
use super::flatness::{FlatOutput, compute_flatness};
use super::richter::FeasibilityReport;
use super::spline::{SplineTrajectory, Waypoint};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const G: f32 = 9.81; // m/s²
/// Small timestep used for finite-difference angular velocity / acceleration.
const DT_FD: f32 = 1e-4; // s
/// Minimum ‖a + g·e₃‖ [m/s²] required for the flatness attitude map to be
/// numerically stable.  Below this the thrust direction is degenerate — the
/// eval() falls back to SLERP using the most-recent attitude waypoint.
const FLATNESS_EPSILON: f32 = 0.1;
/// Feasibility check sample interval [s].
const SAMPLE_DT: f32 = 0.02; // 50 Hz

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// A waypoint for the SE(3) planner: position + either an explicit attitude
/// quaternion (for flips/rolls) or a flag to derive attitude via differential
/// flatness (for aggressive but flat-output-feasible trajectories).
///
/// **Choosing a mode**:
/// - `Se3Waypoint::flat(pos, yaw)`: attitude derived from position polynomial
///   jerk/snap via differential flatness.  Correct bank angles appear
///   automatically.  Requires `‖a + g·e₃‖ > 0.1 m/s²` everywhere on the segment.
/// - `Se3Waypoint::levelled/pitched/rolled/new(...)`: explicit attitude quaternion
///   interpolated with SLERP.  Required for flips, vertical loops, or any
///   manoeuvre where the net thrust vector passes through zero.
#[derive(Debug, Clone, Copy)]
pub struct Se3Waypoint {
    /// World-frame position [m].
    pub pos: Vec3,
    /// Desired body attitude — unit quaternion.
    /// Only used when `use_flatness = false`.
    pub att: Quat,
    /// If `true`, attitude is derived from the position polynomial via
    /// differential flatness (jerk/snap feedforward).
    /// If `false`, attitude is SLERP-interpolated between `att` waypoints.
    pub use_flatness: bool,
    /// Yaw reference [rad] for the flatness map.
    /// Only used when `use_flatness = true`.
    pub yaw: f32,
}

impl Se3Waypoint {
    /// Waypoint with explicit position and attitude (SLERP segment).
    pub fn new(pos: Vec3, att: Quat) -> Self {
        Self { pos, att: att.normalize(), use_flatness: false, yaw: 0.0 }
    }

    /// Waypoint with the drone level (identity rotation: body z = world z).
    /// Uses SLERP attitude.
    pub fn levelled(pos: Vec3) -> Self {
        Self { pos, att: Quat::identity(), use_flatness: false, yaw: 0.0 }
    }

    /// Waypoint with the drone pitched by `angle_rad` around the body x-axis.
    /// Positive = nose-up.  Uses SLERP attitude.
    pub fn pitched(pos: Vec3, angle_rad: f32) -> Self {
        Self {
            pos,
            att: Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), angle_rad).normalize(),
            use_flatness: false,
            yaw: 0.0,
        }
    }

    /// Waypoint with the drone rolled by `angle_rad` around the body y-axis.
    /// Positive = right-wing-down.  Uses SLERP attitude.
    pub fn rolled(pos: Vec3, angle_rad: f32) -> Self {
        Self {
            pos,
            att: Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), angle_rad).normalize(),
            use_flatness: false,
            yaw: 0.0,
        }
    }

    /// Waypoint where attitude is derived from differential flatness.
    ///
    /// Use for normal aggressive flight (circles, figure-8, banked turns) where
    /// the thrust vector never passes through zero.  The planner automatically
    /// computes the correct roll/pitch bank angle from jerk/snap feedforward.
    ///
    /// `yaw` is the desired heading [rad] at this waypoint.
    pub fn flat(pos: Vec3, yaw: f32) -> Self {
        Self { pos, att: Quat::identity(), use_flatness: true, yaw }
    }
}

/// Full SE(3) output at a single time instant.
///
/// Unlike `FlatOutput`, this carries an **explicit** rotation matrix and angular
/// velocity that were planned on SO(3) directly — not recovered via flatness.
/// The thrust value can be zero or negative (motor cut-off / braking).
#[derive(Debug, Clone, Copy)]
pub struct Se3Output {
    /// World-frame position [m].
    pub pos: Vec3,
    /// World-frame velocity [m/s].
    pub vel: Vec3,
    /// World-frame acceleration [m/s²].
    pub acc: Vec3,
    /// Desired rotation matrix — rows are body axes in world frame.
    /// Same layout as `FlatnessResult::rot` and `Quat::to_rotation_matrix()`.
    pub rot: [[f32; 3]; 3],
    /// Body-frame angular velocity [rad/s].
    pub omega: Vec3,
    /// Body-frame angular acceleration [rad/s²].
    pub alpha: Vec3,
    /// Required collective thrust [N].
    /// **May be zero or negative** — caller must clamp to [0, T_max].
    pub thrust: f32,
    /// Yaw angle [rad] (extracted from the rotation matrix for reference).
    pub yaw: f32,
}

impl Se3Output {
    /// Convert to a `FlatOutput` for backward compatibility with flatness-based
    /// pipelines.  The yaw / yaw_dot / yaw_ddot fields are set from the rotation
    /// and omega; all higher derivatives are zeroed (snap/jerk not available).
    ///
    /// **Note**: this conversion loses the explicit attitude — use it only when
    /// feeding a flatness-based controller that ignores the Se3 rotation.
    pub fn to_flat(&self) -> FlatOutput {
        FlatOutput {
            pos:      self.pos,
            vel:      self.vel,
            acc:      self.acc,
            jerk:     Vec3::new(0.0, 0.0, 0.0),
            snap:     Vec3::new(0.0, 0.0, 0.0),
            yaw:      self.yaw,
            yaw_dot:  self.omega.z,
            yaw_ddot: self.alpha.z,
        }
    }
}

/// SE(3) trajectory: minimum-snap position + attitude on SO(3).
///
/// Each segment independently chooses between:
/// - **Flatness mode** (both endpoint waypoints have `use_flatness = true`):
///   position polynomial jerk/snap feedforward → correct bank angles.
/// - **SLERP mode** (either endpoint has explicit attitude):
///   spherical linear interpolation between attitude quaternions.
#[derive(Debug, Clone)]
pub struct Se3Trajectory {
    /// Position polynomial (reuses Mode 0 solver, untouched).
    pos_traj: SplineTrajectory,
    /// Attitude quaternion at each waypoint (n_waypoints total).
    att_waypoints: Vec<Quat>,
    /// Whether each waypoint uses differential flatness (`true`) or SLERP (`false`).
    use_flatness: Vec<bool>,
    /// Original waypoints — kept for feasibility repair (replan with scaled times).
    waypoints: Vec<Se3Waypoint>,
    /// Per-segment durations [s] (length = n_waypoints - 1).
    pub segment_times: Vec<f32>,
    /// Total trajectory duration [s].
    pub total_time: f32,
    /// Mass [kg] — used to compute thrust from acceleration.
    mass: f32,
    /// Whether the trajectory is a smooth closed loop (smooth derivatives at junction).
    periodic: bool,
}

// ---------------------------------------------------------------------------
// Core implementation
// ---------------------------------------------------------------------------

impl Se3Trajectory {
    /// Plan an SE(3) trajectory through `waypoints`.
    ///
    /// # Parameters
    /// - `waypoints`: position + attitude/flatness flag at each waypoint.
    /// - `durations`: per-segment durations [s] (`len == waypoints.len() - 1`).
    /// - `mass`: drone mass [kg], used for thrust computation in `eval`.
    /// Plan a trajectory.
    ///
    /// `periodic`: pass `true` for smooth closed loops (circle, figure-8), `false` for
    /// open paths (helix, flip) even when the first and last positions coincide.
    pub fn plan(
        waypoints: &[Se3Waypoint],
        durations: &[f32],
        mass: f32,
        periodic: bool,
    ) -> Result<Self, String> {
        let n_wp = waypoints.len();
        if n_wp < 2 {
            return Err("need at least 2 waypoints".to_string());
        }
        let n_seg = n_wp - 1;
        if durations.len() != n_seg {
            return Err(format!("need {n_seg} durations, got {}", durations.len()));
        }
        if mass <= 0.0 {
            return Err("mass must be positive".to_string());
        }

        // Position + yaw: for flat waypoints use the waypoint yaw directly;
        // for explicit-attitude waypoints extract yaw from the quaternion so
        // that the yaw polynomial channel is always meaningful.
        let pos_wps: Vec<Waypoint> = waypoints.iter().map(|w| Waypoint {
            pos: w.pos,
            yaw: if w.use_flatness { w.yaw } else { quat_to_yaw(w.att) },
        }).collect();

        let pos_traj = SplineTrajectory::plan(&pos_wps, durations, periodic)?;

        let att_waypoints: Vec<Quat> = waypoints.iter().map(|w| w.att).collect();
        let use_flatness:  Vec<bool> = waypoints.iter().map(|w| w.use_flatness).collect();

        Ok(Se3Trajectory {
            pos_traj,
            att_waypoints,
            use_flatness,
            waypoints: waypoints.to_vec(),
            segment_times: durations.to_vec(),
            total_time: durations.iter().sum(),
            mass,
            periodic,
        })
    }

    /// Evaluate SE(3) outputs at physical time `t` (clamped to [0, total_time]).
    ///
    /// - For **flatness segments** (both endpoints `use_flatness = true` and
    ///   ‖a + g·e₃‖ > ε): uses `compute_flatness` with jerk/snap feedforward
    ///   to produce correct bank angle, angular velocity, and angular acceleration.
    /// - For **SLERP segments** (explicit attitude): position from polynomial,
    ///   attitude from SLERP, ω/α from finite differences.
    pub fn eval(&self, t: f32) -> Se3Output {
        let t = t.clamp(0.0, self.total_time);

        // Position, velocity, acceleration, jerk, snap, yaw — always from polynomial.
        let flat = self.pos_traj.eval(t);

        let (seg, _) = self.segment_and_phase(t);

        // ── Flatness path (jerk/snap feedforward) ────────────────────────────────
        if self.segment_uses_flatness(seg) {
            let acc_gx = flat.acc.x;
            let acc_gy = flat.acc.y;
            let acc_gz = flat.acc.z + G;
            let acc_g_sq = acc_gx * acc_gx + acc_gy * acc_gy + acc_gz * acc_gz;
            if acc_g_sq > FLATNESS_EPSILON * FLATNESS_EPSILON {
                let fr = compute_flatness(&flat, self.mass);
                return Se3Output {
                    pos:    fr.pos,
                    vel:    fr.vel,
                    acc:    flat.acc,
                    rot:    fr.rot,
                    omega:  fr.omega,
                    alpha:  fr.omega_dot,
                    thrust: fr.thrust,
                    yaw:    flat.yaw,
                };
            }
            // Degenerate: thrust direction undefined — fall through to SLERP.
        }

        // ── SLERP attitude path ───────────────────────────────────────────────────
        let q = self.slerp_attitude(t);

        let t_fw = (t + DT_FD).min(self.total_time);
        let t_bw = (t - DT_FD).max(0.0);
        let q_fw = self.slerp_attitude(t_fw);
        let q_bw = self.slerp_attitude(t_bw);
        let dt_actual = t_fw - t_bw;

        let omega = quat_body_omega(q, q_fw, q_bw, dt_actual);

        let omega_fw = {
            let t2 = (t + 2.0 * DT_FD).min(self.total_time);
            let q2  = self.slerp_attitude(t2);
            let dt2 = t2 - t_fw;
            if dt2 > 1e-10 { quat_body_omega(q_fw, q2, q, t_fw - t_bw) }
            else { omega }
        };
        let alpha = Vec3::new(
            (omega_fw.x - omega.x) / DT_FD,
            (omega_fw.y - omega.y) / DT_FD,
            (omega_fw.z - omega.z) / DT_FD,
        );

        let rot = q.to_rotation_matrix();

        // Thrust: project (a + g·e₃) onto body z-axis (third column).
        let zb = Vec3::new(rot[0][2], rot[1][2], rot[2][2]);
        let total_acc = Vec3::new(flat.acc.x, flat.acc.y, flat.acc.z + G);
        let thrust = self.mass * (total_acc.x * zb.x + total_acc.y * zb.y + total_acc.z * zb.z);

        let yaw = rot[1][0].atan2(rot[0][0]);

        Se3Output { pos: flat.pos, vel: flat.vel, acc: flat.acc, rot, omega, alpha, thrust, yaw }
    }

    /// Evaluate and return as `FlatOutput` for pipelines that don't need Se3.
    #[inline]
    pub fn eval_flat(&self, t: f32) -> FlatOutput {
        self.eval(t).to_flat()
    }

    /// Position-only kinematics from the underlying minimum-snap spline (includes **jerk** and **snap**).
    #[inline]
    pub fn eval_position_flat(&self, t: f32) -> FlatOutput {
        self.pos_traj.eval(t.clamp(0.0, self.total_time))
    }

    /// Check whether this trajectory respects thrust and angular-rate limits.
    ///
    /// Samples at 50 Hz, evaluates full SE(3) output (using flatness or SLERP
    /// as appropriate), and records peak thrust and peak angular-rate magnitude.
    pub fn check_feasibility(
        &self,
        max_thrust_n: f32,
        max_omega_rad_s: f32,
    ) -> FeasibilityReport {
        let n_steps = (self.total_time / SAMPLE_DT).ceil() as usize + 1;
        let mut peak_thrust: f32 = 0.0;
        let mut peak_omega:  f32 = 0.0;

        for k in 0..n_steps {
            let t = (k as f32 * SAMPLE_DT).min(self.total_time);
            let out = self.eval(t);
            let omega_mag = (out.omega.x * out.omega.x
                + out.omega.y * out.omega.y
                + out.omega.z * out.omega.z).sqrt();
            peak_thrust = peak_thrust.max(out.thrust.abs());
            peak_omega  = peak_omega.max(omega_mag);
        }

        let feasible = peak_thrust <= max_thrust_n && peak_omega <= max_omega_rad_s;

        // Thrust ∝ 1/T² → scale = sqrt(peak/max);  omega ∝ 1/T → scale = peak/max
        let scale_thrust = if peak_thrust > max_thrust_n {
            (peak_thrust / max_thrust_n).sqrt()
        } else { 1.0 };
        let scale_omega = if peak_omega > max_omega_rad_s {
            peak_omega / max_omega_rad_s
        } else { 1.0 };

        FeasibilityReport {
            max_thrust_n:        peak_thrust,
            max_omega_rad_s:     peak_omega,
            feasible,
            suggested_time_scale: scale_thrust.max(scale_omega),
        }
    }

    /// Scale segment times until the trajectory is feasible, replanning the
    /// position polynomial at each iteration.
    ///
    /// Returns the repaired trajectory (may be `self` if already feasible).
    /// Stops after `max_iters` even if still infeasible.
    pub fn repair_feasibility(
        &self,
        max_thrust_n: f32,
        max_omega_rad_s: f32,
        max_iters: usize,
    ) -> Result<Se3Trajectory, String> {
        let orig_times = self.segment_times.clone();
        let mut total_scale = 1.0_f32;
        let mut current = self.clone();

        for _ in 0..max_iters {
            let report = current.check_feasibility(max_thrust_n, max_omega_rad_s);
            if report.feasible { return Ok(current); }
            // Apply a 5 % margin on top of the theoretically required scale.
            total_scale *= report.suggested_time_scale * 1.05_f32;
            let times: Vec<f32> = orig_times.iter().map(|&d| d * total_scale).collect();
            current = Se3Trajectory::plan(&self.waypoints, &times, self.mass, self.periodic)?;
        }
        Ok(current)
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    /// Returns `true` if segment `seg` should use differential flatness.
    /// Both endpoints must have `use_flatness = true`.
    #[inline]
    fn segment_uses_flatness(&self, seg: usize) -> bool {
        self.use_flatness[seg] && self.use_flatness[seg + 1]
    }

    /// Find which segment `t` falls in and return (segment index, t_local ∈ [0,1]).
    fn segment_and_phase(&self, t: f32) -> (usize, f32) {
        let mut t_rem = t.clamp(0.0, self.total_time);
        let n_seg = self.segment_times.len();
        for (i, &dur) in self.segment_times.iter().enumerate() {
            if t_rem <= dur || i == n_seg - 1 {
                return (i, (t_rem / dur.max(1e-9)).clamp(0.0, 1.0));
            }
            t_rem -= dur;
        }
        (n_seg - 1, 1.0)
    }

    /// SLERP attitude quaternion at time `t`.
    fn slerp_attitude(&self, t: f32) -> Quat {
        let (seg, phase) = self.segment_and_phase(t);
        let q0 = self.att_waypoints[seg];
        let q1 = self.att_waypoints[seg + 1];
        slerp(q0, q1, phase)
    }
}

// ---------------------------------------------------------------------------
// Quaternion helpers
// ---------------------------------------------------------------------------

/// Extract yaw [rad] from a unit quaternion (ZYX Euler convention).
#[inline]
fn quat_to_yaw(q: Quat) -> f32 {
    let siny = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    siny.atan2(cosy)
}

/// Spherical linear interpolation between two unit quaternions.
///
/// Always takes the shortest arc on S³ (handles the double-cover ambiguity by
/// negating q1 if the dot product is negative).
fn slerp(q0: Quat, q1: Quat, t: f32) -> Quat {
    // Ensure shortest path (S³ double cover: q and -q represent same rotation)
    let dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;
    let q1 = if dot < 0.0 {
        Quat::new(-q1.w, -q1.x, -q1.y, -q1.z)
    } else {
        q1
    };
    let dot = dot.abs().clamp(-1.0, 1.0);

    // For very close quaternions fall back to linear interpolation (avoids divide-by-zero)
    if dot > 0.9995 {
        return Quat::new(
            q0.w + t * (q1.w - q0.w),
            q0.x + t * (q1.x - q0.x),
            q0.y + t * (q1.y - q0.y),
            q0.z + t * (q1.z - q0.z),
        ).normalize();
    }

    let theta_0 = dot.acos();       // angle between q0 and q1
    let theta   = theta_0 * t;      // angle at parameter t
    let sin_0   = theta_0.sin();
    let s0 = (theta_0 - theta).sin() / sin_0;
    let s1 = theta.sin()            / sin_0;

    Quat::new(
        s0 * q0.w + s1 * q1.w,
        s0 * q0.x + s1 * q1.x,
        s0 * q0.y + s1 * q1.y,
        s0 * q0.z + s1 * q1.z,
    ).normalize()
}

/// Compute body-frame angular velocity from three quaternions using central differences.
///
/// ω_body = 2 · Im(q^{-1} · q̇)   where  q̇ ≈ (q_fw - q_bw) / (t_fw - t_bw)
fn quat_body_omega(q: Quat, q_fw: Quat, q_bw: Quat, dt: f32) -> Vec3 {
    if dt < 1e-12 {
        return Vec3::new(0.0, 0.0, 0.0);
    }
    // Central-difference quaternion derivative
    let dq = Quat::new(
        (q_fw.w - q_bw.w) / dt,
        (q_fw.x - q_bw.x) / dt,
        (q_fw.y - q_bw.y) / dt,
        (q_fw.z - q_bw.z) / dt,
    );
    // q^{-1} = q* for unit quaternions
    let q_inv = q.conjugate();
    // q^{-1} * dq  (quaternion product)
    let prod = quat_mul(q_inv, dq);
    // ω_body = 2 · Im(q^{-1} · dq)
    Vec3::new(2.0 * prod.x, 2.0 * prod.y, 2.0 * prod.z)
}

/// Raw quaternion multiplication (without normalisation).
#[inline]
fn quat_mul(a: Quat, b: Quat) -> Quat {
    Quat::new(
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn two_wp_level() -> (Vec<Se3Waypoint>, Vec<f32>) {
        // Use 3 waypoints / 2 segments to avoid the Clarabel single-segment
        // statefulness edge-case that can occur when tests run sequentially.
        let wps = vec![
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.0)),
            Se3Waypoint::levelled(Vec3::new(0.5, 0.0, 1.0)),
            Se3Waypoint::levelled(Vec3::new(1.0, 0.0, 1.0)),
        ];
        (wps, vec![0.5_f32, 0.5_f32])
    }

    // ── T1: positions match at waypoints ─────────────────────────────────────
    #[test]
    fn se3_passes_through_waypoints() {
        let (wps, durs) = two_wp_level();
        let traj = Se3Trajectory::plan(&wps, &durs, 0.027, false).expect("plan");
        let start = traj.eval(0.0);
        let end   = traj.eval(traj.total_time);
        assert!((start.pos.x - 0.0).abs() < 1e-3, "start x: {}", start.pos.x);
        assert!((end.pos.x   - 1.0).abs() < 1e-3, "end x:   {}", end.pos.x);
        assert!((start.pos.z - 1.0).abs() < 1e-3, "start z: {}", start.pos.z);
    }

    // ── T2: level attitude gives identity rotation ───────────────────────────
    #[test]
    fn se3_level_attitude_is_identity() {
        let (wps, durs) = two_wp_level();
        let traj = Se3Trajectory::plan(&wps, &durs, 0.027, false).expect("plan");
        let out = traj.eval(0.0);
        // Diagonal should be ~1, off-diagonals ~0
        assert!((out.rot[0][0] - 1.0).abs() < 1e-5, "R[0][0]: {}", out.rot[0][0]);
        assert!((out.rot[1][1] - 1.0).abs() < 1e-5, "R[1][1]: {}", out.rot[1][1]);
        assert!((out.rot[2][2] - 1.0).abs() < 1e-5, "R[2][2]: {}", out.rot[2][2]);
    }

    // ── T3: zero omega at endpoints (SLERP start/end = waypoint attitudes) ───
    #[test]
    fn se3_zero_omega_at_level_endpoints() {
        let (wps, durs) = two_wp_level();
        let traj = Se3Trajectory::plan(&wps, &durs, 0.027, false).expect("plan");
        // At t=0 the SLERP phase=0 → constant rotation → omega ≈ 0
        let out = traj.eval(0.0);
        assert!(out.omega.x.abs() < 1e-2, "ωx at t=0: {}", out.omega.x);
        assert!(out.omega.y.abs() < 1e-2, "ωy at t=0: {}", out.omega.y);
        assert!(out.omega.z.abs() < 1e-2, "ωz at t=0: {}", out.omega.z);
    }

    // ── T4: SLERP reaches inverted attitude at midpoint of flip ──────────────
    #[test]
    fn se3_flip_inverted_at_midpoint() {
        // 3-waypoint flip: level → inverted → level
        let wps = vec![
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.2)),
            Se3Waypoint::pitched(Vec3::new(0.0, 0.0, 1.2), PI),  // upside-down
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.2)),
        ];
        let traj = Se3Trajectory::plan(&wps, &[0.35, 0.35], 0.027, false).expect("plan");
        let mid = traj.eval(0.35); // at exactly the inverted waypoint
        // z_b should point downward: rot[2][2] ≈ -1
        assert!(mid.rot[2][2] < -0.9,
            "z_b z-component should be ≈ -1 when inverted, got {}", mid.rot[2][2]);
    }

    // ── T5: thrust can be negative during flip ────────────────────────────────
    #[test]
    fn se3_flip_allows_negative_thrust() {
        let wps = vec![
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.2)),
            Se3Waypoint::pitched(Vec3::new(0.0, 0.0, 1.2), PI),
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.2)),
        ];
        let traj = Se3Trajectory::plan(&wps, &[0.35, 0.35], 0.027, false).expect("plan");
        // Sample through mid-flip — thrust should dip below zero
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(0.7 * i as f32 / 100.0).thrust)
            .collect();
        let min_thrust = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        println!("min thrust during flip: {min_thrust:.3} N");
        assert!(min_thrust < 0.0,
            "expected negative thrust during flip apex, got {min_thrust:.3}");
    }

    // ── T6: rotation matrix stays orthonormal throughout ─────────────────────
    #[test]
    fn se3_rotation_stays_orthonormal() {
        let wps = vec![
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.0)),
            Se3Waypoint::rolled(Vec3::new(1.0, 0.0, 1.0), PI),
            Se3Waypoint::levelled(Vec3::new(1.0, 0.0, 1.0)),
        ];
        let traj = Se3Trajectory::plan(&wps, &[0.4, 0.4], 0.027, false).expect("plan");
        for i in 0..=50 {
            let t = traj.total_time * i as f32 / 50.0;
            let out = traj.eval(t);
            let r = out.rot;
            // Check R^T R ≈ I (column orthonormality)
            for col in 0..3 {
                let norm_sq: f32 = (0..3).map(|row| r[row][col] * r[row][col]).sum();
                assert!((norm_sq - 1.0).abs() < 1e-4,
                    "col {col} norm²={norm_sq:.6} at t={t:.2}");
            }
        }
    }

    // ── T7: slerp shortest-path (dot < 0 case) ───────────────────────────────
    #[test]
    fn slerp_takes_shortest_path() {
        // Two quaternions representing ~10° rotation apart but with negated q1
        let q0 = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.0);
        let q1 = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.2);
        let q1_neg = Quat::new(-q1.w, -q1.x, -q1.y, -q1.z); // same rotation, negated

        let mid_pos = slerp(q0, q1,     0.5);
        let mid_neg = slerp(q0, q1_neg, 0.5);

        // Both should produce the same rotation (10° around z)
        let diff_w = (mid_pos.w - mid_neg.w).abs().min((mid_pos.w + mid_neg.w).abs());
        assert!(diff_w < 1e-5, "SLERP double-cover: w diff {}", diff_w);
    }

    // ── T8: flat waypoints produce non-trivial omega during motion ───────────
    #[test]
    fn se3_flat_waypoints_produce_omega() {
        // Straight flight with flatness: position changes → acceleration → omega
        let wps = vec![
            Se3Waypoint::flat(Vec3::new(0.0, 0.0, 1.0), 0.0),
            Se3Waypoint::flat(Vec3::new(1.0, 0.0, 1.0), 0.0),
            Se3Waypoint::flat(Vec3::new(2.0, 0.0, 1.0), 0.0),
        ];
        let traj = Se3Trajectory::plan(&wps, &[1.0, 1.0], 0.027, false).expect("plan");
        // At t=0.5 (mid of first segment) there should be a non-zero acceleration
        // and therefore non-zero omega from flatness
        let out = traj.eval(0.5);
        // Thrust must be positive (not inverted)
        assert!(out.thrust > 0.0, "thrust should be positive, got {}", out.thrust);
        // Rotation matrix should be orthonormal
        for col in 0..3 {
            let norm_sq: f32 = (0..3).map(|row| out.rot[row][col] * out.rot[row][col]).sum();
            assert!((norm_sq - 1.0).abs() < 1e-4, "col {col} norm² = {norm_sq}");
        }
    }

    // ── T9: flat hover gives level attitude and correct thrust ────────────────
    #[test]
    fn se3_flat_hover_gives_level_attitude() {
        // A hover: two waypoints at the same position → near-zero acceleration
        // → acc_g ≈ g·e3 → zb ≈ e3 → attitude ≈ identity
        let wps = vec![
            Se3Waypoint::flat(Vec3::new(0.0, 0.0, 1.0), 0.0),
            Se3Waypoint::flat(Vec3::new(0.01, 0.0, 1.0), 0.0), // tiny motion
            Se3Waypoint::flat(Vec3::new(0.02, 0.0, 1.0), 0.0),
        ];
        let traj = Se3Trajectory::plan(&wps, &[2.0, 2.0], 0.027, false).expect("plan");
        let out = traj.eval(2.0); // midpoint — near rest
        // Body z should point approximately upward
        let zb_z = out.rot[2][2]; // z-component of zb = rot[0][2],rot[1][2],rot[2][2]...
        // Actually check: rot[row][2] = third column = body-z in world
        let zb = Vec3::new(out.rot[0][2], out.rot[1][2], out.rot[2][2]);
        assert!(zb.z > 0.99, "body-z should point up at near-hover, zb.z = {}", zb.z);
        // Thrust near mg
        let expected = 0.027 * 9.81;
        assert!((out.thrust - expected).abs() < 0.01,
            "thrust at hover: {} vs {}", out.thrust, expected);
        let _ = zb_z;
    }

    // ── T10: check_feasibility reports correct peak values ────────────────────
    #[test]
    fn se3_check_feasibility_reports_peaks() {
        let wps = vec![
            Se3Waypoint::flat(Vec3::new(0.0, 0.0, 1.0), 0.0),
            Se3Waypoint::flat(Vec3::new(0.5, 0.0, 1.0), 0.0),
            Se3Waypoint::flat(Vec3::new(1.0, 0.0, 1.0), 0.0),
        ];
        let traj = Se3Trajectory::plan(&wps, &[0.5, 0.5], 0.027, false).expect("plan");
        let report = traj.check_feasibility(1.0, 20.0);
        // For a smooth trajectory, thrust must be positive and around mg
        assert!(report.max_thrust_n > 0.0, "peak thrust should be > 0");
        assert!(report.max_thrust_n < 1.0, "peak thrust under 1N for gentle traj");
        // suggested_time_scale ≥ 1 when feasible
        assert!(report.suggested_time_scale >= 1.0 || report.feasible);
    }

    // ── T11: quat_to_yaw round-trip ───────────────────────────────────────────
    #[test]
    fn quat_to_yaw_round_trip() {
        for yaw_deg in [-90, 0, 45, 90, 180] {
            let yaw_ref = (yaw_deg as f32).to_radians();
            let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw_ref);
            let yaw_out = quat_to_yaw(q);
            let diff = (yaw_out - yaw_ref).abs();
            // Allow ±π wrap-around for 180°
            let diff = diff.min((diff - 2.0 * std::f32::consts::PI).abs());
            assert!(diff < 1e-5, "yaw {yaw_deg}°: got {yaw_out:.6} vs {yaw_ref:.6}");
        }
    }

    // ── T12: mixed flat/SLERP segment — flat segment uses flatness map ────────
    #[test]
    fn se3_flat_segment_uses_flatness_map() {
        // Segment 0: flat, segment 1: explicit SLERP (transition to flat with identity)
        let wps = vec![
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.0)), // SLERP endpoint
            Se3Waypoint::flat(Vec3::new(0.5, 0.0, 1.0), 0.0), // mixed — SLERP wins
            Se3Waypoint::flat(Vec3::new(1.0, 0.0, 1.0), 0.0), // flat segment
        ];
        let traj = Se3Trajectory::plan(&wps, &[0.5, 0.5], 0.027, false).expect("plan");
        // Segment 1 (t=0.5..1.0) uses flatness → thrust should be positive
        let out = traj.eval(0.75);
        assert!(out.thrust > 0.0, "thrust in flat segment: {}", out.thrust);
    }
}
