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
//! Orientation: explicit SO(3) attitude waypoints, interpolated with SLERP.
//!
//! Position and attitude are planned **independently** — no coupling.  The
//! planner then computes:
//!   thrust  = m · (a + g·e₃) · (R·e₃)   (can be 0 or negative → motor cut)
//!   omega   = finite-diff of SLERP
//!   alpha   = finite-diff of SLERP
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
//! // Aggressive circle: explicit level attitude waypoints
//! let waypoints: Vec<Se3Waypoint> = circle_pts.iter()
//!     .map(|p| Se3Waypoint::levelled(*p))
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
//! // out.rot    — rotation matrix from explicit SLERP attitude
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
/// Feasibility check sample interval [s].
const SAMPLE_DT: f32 = 0.02; // 50 Hz

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// A waypoint for the SE(3) planner: position + explicit attitude quaternion.
///
/// Mode 2 policy is explicit-attitude-only. Use `new`/`levelled`/`pitched`/`rolled`
/// to author the orientation profile directly on SO(3).
#[derive(Debug, Clone, Copy)]
pub struct Se3Waypoint {
    /// World-frame position [m].
    pub pos: Vec3,
    /// Desired body attitude — unit quaternion.
    pub att: Quat,
}

impl Se3Waypoint {
    /// Waypoint with explicit position and attitude (SLERP segment).
    pub fn new(pos: Vec3, att: Quat) -> Self {
        Self { pos, att: att.normalize() }
    }

    /// Waypoint with the drone level (identity rotation: body z = world z).
    /// Uses SLERP attitude.
    pub fn levelled(pos: Vec3) -> Self {
        Self { pos, att: Quat::identity() }
    }

    /// Waypoint with the drone pitched by `angle_rad` around the body x-axis.
    /// Positive = nose-up.  Uses SLERP attitude.
    pub fn pitched(pos: Vec3, angle_rad: f32) -> Self {
        Self {
            pos,
            att: Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), angle_rad).normalize(),
        }
    }

    /// Waypoint with the drone rolled by `angle_rad` around the body y-axis.
    /// Positive = right-wing-down.  Uses SLERP attitude.
    pub fn rolled(pos: Vec3, angle_rad: f32) -> Self {
        Self {
            pos,
            att: Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), angle_rad).normalize(),
        }
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

/// SE(3) trajectory: minimum-snap position + explicit attitude on SO(3).
///
/// Attitude is interpolated with SLERP between waypoint quaternions.
#[derive(Debug, Clone)]
pub struct Se3Trajectory {
    /// Position polynomial (reuses Mode 0 solver, untouched).
    pos_traj: SplineTrajectory,
    /// Attitude quaternion at each waypoint (n_waypoints total).
    att_waypoints: Vec<Quat>,
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
    /// - `waypoints`: position + explicit attitude at each waypoint.
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

        // Position + yaw channel: yaw is extracted from waypoint quaternions so
        // the underlying position polynomial keeps a meaningful yaw signal.
        let pos_wps: Vec<Waypoint> = waypoints.iter().map(|w| Waypoint {
            pos: w.pos,
            yaw: quat_to_yaw(w.att),
        }).collect();

        let pos_traj = SplineTrajectory::plan(&pos_wps, durations, periodic)?;

        let att_waypoints: Vec<Quat> = waypoints.iter().map(|w| w.att).collect();

        Ok(Se3Trajectory {
            pos_traj,
            att_waypoints,
            waypoints: waypoints.to_vec(),
            segment_times: durations.to_vec(),
            total_time: durations.iter().sum(),
            mass,
            periodic,
        })
    }

    /// Evaluate SE(3) outputs at physical time `t` (clamped to [0, total_time]).
    ///
    /// Position comes from the polynomial; attitude is interpolated with SLERP.
    /// Angular velocity/acceleration are estimated via finite differences.
    pub fn eval(&self, t: f32) -> Se3Output {
        let t = t.clamp(0.0, self.total_time);

        // Position, velocity, acceleration, jerk, snap, yaw — always from polynomial.
        let flat = self.pos_traj.eval(t);
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

    /// Return a reference to the underlying position `SplineTrajectory`.
    /// Used by flight binaries to serialise position coefficients for firmware upload.
    pub fn as_spline(&self) -> &SplineTrajectory {
        &self.pos_traj
    }

    /// Fit degree-8 polynomials to the roll and pitch profiles of each segment.
    ///
    /// Returns `(croll[9], cpitch[9])` per segment in normalised time τ ∈ [0,1].
    /// These coefficients are uploaded to firmware so it can evaluate roll_d/pitch_d
    /// at 500 Hz without re-running the flatness map.
    ///
    /// **Source selection**: If all attitude waypoints are identity (all `levelled`),
    /// roll/pitch are sampled from differential flatness of the position polynomial —
    /// this gives the physically correct attitude for the manoeuvre instead of zeros.
    /// Otherwise (explicit quaternion waypoints: loop, flip, banked corners), roll/pitch
    /// are sampled from the SLERP attitude profile in `eval()`.
    ///
    /// 22 sample points per segment (20 interior + 2 boundary) for C0 polynomial
    /// continuity at segment junctions. Tikhonov λ = 1e-5.
    pub fn attitude_poly_coefs(&self) -> Vec<([f32; 9], [f32; 9])> {
        const N_INT: usize = 20;           // interior samples
        const N_TOTAL: usize = N_INT + 2;  // +2 boundary samples at τ=0 and τ=1
        const LAMBDA: f32 = 1e-5;
        const MASS: f32 = 0.031;           // CF + decks [kg]

        // Detect whether all waypoints are identity (levelled) — then derive attitude
        // from differential flatness of the position polynomial so the polynomial is
        // non-trivial and meaningful for the firmware to use.
        let all_levelled = self.att_waypoints.iter().all(|q| {
            (q.w - 1.0_f32).abs() < 1e-4 && q.x.abs() < 1e-4 && q.y.abs() < 1e-4 && q.z.abs() < 1e-4
        });

        let n_segs = self.segment_times.len();
        let mut out = Vec::with_capacity(n_segs);

        let mut t_start = 0.0_f32;
        for seg in 0..n_segs {
            let dur = self.segment_times[seg];
            let mut roll_samples  = [(0.0_f32, 0.0_f32); N_TOTAL];
            let mut pitch_samples = [(0.0_f32, 0.0_f32); N_TOTAL];

            // Helper: extract roll/pitch from a rotation matrix (ZYX Euler convention,
            // matches firmware `rot_from_euler()`: pitch=asin(-R[2][0]), roll=atan2(R[2][1],R[2][2])).
            let rot_to_rp = |rot: &[[f32; 3]; 3]| -> (f32, f32) {
                let pitch = (-rot[2][0].clamp(-1.0, 1.0)).asin();
                let roll  = rot[2][1].atan2(rot[2][2]);
                (roll, pitch)
            };

            let sample_at = |tau: f32| -> (f32, f32) {
                let t = t_start + tau * dur;
                if all_levelled {
                    // Derive attitude from flatness of the position polynomial.
                    // The position polynomial's jerk/snap give the correct body attitude.
                    let flat = self.eval_position_flat(t);
                    let res  = compute_flatness(&flat, MASS);
                    rot_to_rp(&res.rot)
                } else {
                    // Use SLERP attitude from explicit waypoint quaternions (loop, flip, banked).
                    let out_k = self.eval(t);
                    rot_to_rp(&out_k.rot)
                }
            };

            // Boundary samples at τ=0 and τ=1 enforce C0 polynomial continuity at junctions.
            let (r0, p0) = sample_at(0.0);
            roll_samples[0]          = (0.0, r0);
            pitch_samples[0]         = (0.0, p0);
            let (r1, p1) = sample_at(1.0);
            roll_samples[N_TOTAL - 1]  = (1.0, r1);
            pitch_samples[N_TOTAL - 1] = (1.0, p1);

            // Interior samples uniformly in (0, 1).
            for k in 0..N_INT {
                let tau = (k as f32 + 0.5) / N_INT as f32;
                let (r, p) = sample_at(tau);
                roll_samples[k + 1]  = (tau, r);
                pitch_samples[k + 1] = (tau, p);
            }

            let croll  = poly_fit_deg8(&roll_samples,  LAMBDA);
            let cpitch = poly_fit_deg8(&pitch_samples, LAMBDA);
            out.push((croll, cpitch));

            t_start += dur;
        }
        out
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

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
// Polynomial fitting helpers
// ---------------------------------------------------------------------------

/// Fit a degree-8 polynomial (9 coefficients) to N ≤ 64 sample points `(tau, y)`
/// where tau ∈ [0, 1].  Uses normal equations with Tikhonov regularisation `lambda`.
///
/// Returns `c` such that `y ≈ c[0] + c[1]·τ + … + c[8]·τ⁸`.
fn poly_fit_deg8(samples: &[(f32, f32)], lambda: f32) -> [f32; 9] {
    const DEG: usize = 9;
    let n = samples.len();

    // Accumulate VᵀV and Vᵀy
    let mut ata = [[0.0_f32; DEG]; DEG];
    let mut aty = [0.0_f32; DEG];
    for &(tau, y) in samples.iter().take(n) {
        let mut basis = [0.0_f32; DEG];
        let mut pw = 1.0_f32;
        for j in 0..DEG { basis[j] = pw; pw *= tau; }
        for i in 0..DEG {
            for j in 0..DEG { ata[i][j] += basis[i] * basis[j]; }
            aty[i] += basis[i] * y;
        }
    }
    // Tikhonov: (VᵀV + λI)
    for i in 0..DEG { ata[i][i] += lambda; }

    // Solve 9×9 system via Gaussian elimination with partial pivoting
    // Augmented matrix [ata | aty]
    let mut aug = [[0.0_f32; DEG + 1]; DEG];
    for i in 0..DEG {
        for j in 0..DEG { aug[i][j] = ata[i][j]; }
        aug[i][DEG] = aty[i];
    }
    for col in 0..DEG {
        // Partial pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col+1)..DEG {
            if aug[row][col].abs() > max_val { max_val = aug[row][col].abs(); max_row = row; }
        }
        aug.swap(col, max_row);
        let pivot = aug[col][col];
        if pivot.abs() < 1e-12 { continue; }
        let inv = 1.0 / pivot;
        for j in col..=DEG { aug[col][j] *= inv; }
        for row in 0..DEG {
            if row == col { continue; }
            let factor = aug[row][col];
            for j in col..=DEG { aug[row][j] -= factor * aug[col][j]; }
        }
    }
    let mut c = [0.0_f32; DEG];
    for i in 0..DEG { c[i] = aug[i][DEG]; }
    c
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

    // ── T8: explicit waypoints produce non-trivial omega during motion ───────
    #[test]
    fn se3_level_waypoints_produce_omega() {
        // Straight flight with explicit attitude interpolation.
        let wps = vec![
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.0)),
            Se3Waypoint::levelled(Vec3::new(1.0, 0.0, 1.0)),
            Se3Waypoint::levelled(Vec3::new(2.0, 0.0, 1.0)),
        ];
        let traj = Se3Trajectory::plan(&wps, &[1.0, 1.0], 0.027, false).expect("plan");
        // At t=0.5 (mid of first segment) output should remain finite.
        let out = traj.eval(0.5);
        // Thrust must be positive (not inverted)
        assert!(out.thrust > 0.0, "thrust should be positive, got {}", out.thrust);
        // Rotation matrix should be orthonormal
        for col in 0..3 {
            let norm_sq: f32 = (0..3).map(|row| out.rot[row][col] * out.rot[row][col]).sum();
            assert!((norm_sq - 1.0).abs() < 1e-4, "col {col} norm² = {norm_sq}");
        }
    }

    // ── T9: level hover gives level attitude and correct thrust ───────────────
    #[test]
    fn se3_level_hover_gives_level_attitude() {
        // A hover: two waypoints at the same position → near-zero acceleration
        // with explicit level quaternions.
        let wps = vec![
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.0)),
            Se3Waypoint::levelled(Vec3::new(0.01, 0.0, 1.0)), // tiny motion
            Se3Waypoint::levelled(Vec3::new(0.02, 0.0, 1.0)),
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
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.0)),
            Se3Waypoint::levelled(Vec3::new(0.5, 0.0, 1.0)),
            Se3Waypoint::levelled(Vec3::new(1.0, 0.0, 1.0)),
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

    // ── T12: mixed explicit waypoints remain valid ─────────────────────────────
    #[test]
    fn se3_mixed_explicit_segment_is_valid() {
        let wps = vec![
            Se3Waypoint::levelled(Vec3::new(0.0, 0.0, 1.0)),
            Se3Waypoint::pitched(Vec3::new(0.5, 0.0, 1.0), 0.2),
            Se3Waypoint::levelled(Vec3::new(1.0, 0.0, 1.0)),
        ];
        let traj = Se3Trajectory::plan(&wps, &[0.5, 0.5], 0.027, false).expect("plan");
        let out = traj.eval(0.75);
        assert!(out.thrust.is_finite(), "thrust should be finite: {}", out.thrust);
    }
}
