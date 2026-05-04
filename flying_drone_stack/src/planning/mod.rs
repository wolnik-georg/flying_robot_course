//! Motion planning module.
//!
//! Provides:
//! - Mode 0: Minimum-snap 8th-order polynomial spline planner (`spline`) — Mellinger (2012)
//! - Mode 1: Richter planner with automatic time allocation (`richter`) — Richter et al. (2016)
//! - Mode 2: SE(3) collocation planner (`se3_traj`) — orientation explicitly prescribed on SO(3)
//! - Differential flatness for multirotors (`flatness`)
//! - Frontier-based autonomous exploration (`exploration`)
//!
//! ## Switching between planning modes
//!
//! Use `TrajectoryPlanner` as a single unified interface — parallel to how
//! `CONTROLLER_MODE` switches between geometric / INDI attitude / INDI full:
//!
//! ```ignore
//! use multirotor_simulator::planning::{TrajectoryPlanner, Waypoint};
//! use multirotor_simulator::math::Vec3;
//!
//! let wps = vec![
//!     Waypoint { pos: Vec3::new(0.0, 0.0, 1.0), yaw: 0.0 },
//!     Waypoint { pos: Vec3::new(1.0, 0.0, 1.0), yaw: 0.0 },
//! ];
//!
//! // Mode 0 — supply durations manually (Mellinger / SplineTrajectory):
//! let planner = TrajectoryPlanner::spline(&wps, &[1.5], false).unwrap();
//!
//! // Mode 1 — supply k_t only (Richter / RichterTrajectory):
//! let planner = TrajectoryPlanner::richter(&wps, 0.5, false).unwrap();
//!
//! // Both modes: identical interface from here on
//! let flat = planner.eval(0.5);
//! println!("total_time={:.2}s", planner.total_time());
//! ```
//!
//! In a flight binary, changing one constant switches the planner for the
//! whole binary — nothing else needs to change:
//!
//! ```ignore
//! const PLANNER_MODE: u8 = 1; // 0 = spline, 1 = richter, 2 = se3 (Se3Waypoint + segment durs)
//! let planner = match PLANNER_MODE {
//!     1 => TrajectoryPlanner::richter(&wps, K_T, false).unwrap(),
//!     2 => TrajectoryPlanner::se3(&se3_wps, &durs, mass_kg, false).unwrap(),
//!     _ => TrajectoryPlanner::spline(&wps, &DURATIONS, false).unwrap(),
//! };
//! ```

pub mod flatness;
pub mod spline;
pub mod richter;
pub mod se3_traj;
pub mod exploration;

pub use flatness::{
    FlatOutput, FlatOutput as FlatState, FlatnessResult, body_torque_diagonal, compute_flatness,
    flatness_to_reference, rot_to_quat,
};
pub use spline::{SplineSegment, SplineTrajectory, Waypoint};
pub use richter::{RichterTrajectory, RichterWaypoint, FeasibilityReport};
pub use se3_traj::{Se3Trajectory, Se3Waypoint, Se3Output};

// ---------------------------------------------------------------------------
// Unified planner interface
// ---------------------------------------------------------------------------

/// A unified trajectory planner that can be Mode 0 (spline) or Mode 1 (Richter).
///
/// Both modes expose the same `eval(t) → FlatOutput` interface, making it
/// trivial to switch between them in any flight binary with a single constant.
/// This is the motion-planning parallel to `CONTROLLER_MODE` in `firmware_app`.
///
/// ## Mode summary
/// | Mode | Variant | Key input | Best for |
/// |------|---------|-----------|----------|
/// | 0 | `Spline` | `durations: &[f32]` | Fixed, hand-tuned timing |
/// | 1 | `Richter` | `k_t: f32` | Automatic timing, aggressiveness knob |
/// | 2 | `Se3` | `waypoints: &[Se3Waypoint]` | Flips/rolls, explicit attitude on SO(3) |
#[derive(Debug, Clone)]
pub enum TrajectoryPlanner {
    /// Mode 0 — Mellinger (2012) minimum-snap, manually specified segment durations.
    Spline(SplineTrajectory),
    /// Mode 1 — Richter et al. (2016), automatic time allocation from `k_t`.
    Richter(RichterTrajectory),
    /// Mode 2 — SE(3) planner: minimum-snap position + SLERP attitude on SO(3).
    /// For flips / rolls where flatness breaks down (thrust through zero).
    Se3(Se3Trajectory),
}

impl TrajectoryPlanner {
    /// Build a **Mode 0** planner (SplineTrajectory — manual durations).
    pub fn spline(
        waypoints: &[Waypoint],
        durations: &[f32],
        periodic: bool,
    ) -> Result<Self, String> {
        SplineTrajectory::plan(waypoints, durations, periodic).map(TrajectoryPlanner::Spline)
    }

    /// Build a **Mode 1** planner (RichterTrajectory — automatic time allocation).
    ///
    /// - `k_t ≈ 0.1` → ~1.0 m/s average speed (gentle, comparable to Mode 0 defaults)
    /// - `k_t ≈ 0.5` → ~1.6 m/s (moderate)
    /// - `k_t ≈ 2.0` → ~2.6 m/s (fast)
    pub fn richter(
        waypoints: &[Waypoint],
        k_t: f32,
        periodic: bool,
    ) -> Result<Self, String> {
        RichterTrajectory::plan(waypoints, k_t, periodic).map(TrajectoryPlanner::Richter)
    }

    /// Build a **Mode 1** planner with manually-specified initial segment durations.
    ///
    /// Useful when waypoints are so densely spaced that `richter()` would clamp all
    /// segment times to `T_MIN = 0.15 s`, making the Clarabel QP degenerate.
    /// With `n_iter = 0` (the default for `richter()`) this is equivalent to
    /// `spline()` with the same durations; set `n_iter > 0` to let Richter
    /// redistribute times from this starting point.
    pub fn richter_from_times(
        waypoints: &[Waypoint],
        times: &[f32],
        k_t: f32,
        periodic: bool,
        n_iter: usize,
    ) -> Result<Self, String> {
        RichterTrajectory::plan_from_times(waypoints, times, k_t, periodic, n_iter)
            .map(TrajectoryPlanner::Richter)
    }

    /// Build a **Mode 2** planner (Se3Trajectory — explicit SO(3) attitude waypoints).
    ///
    /// Use for aggressive manoeuvres where the thrust vector passes through zero
    /// (full flips, rolls) and differential flatness produces undefined attitude.
    ///
    /// - `waypoints`: position + attitude quaternion at each waypoint.
    /// - `durations`: per-segment durations [s] (len == waypoints.len() − 1).
    /// - `mass`: drone mass [kg], used to compute required thrust at each instant.
    /// - `periodic`: `true` for smooth closed loops (circle, figure-8), `false` for open paths.
    pub fn se3(
        waypoints: &[Se3Waypoint],
        durations: &[f32],
        mass: f32,
        periodic: bool,
    ) -> Result<Self, String> {
        Se3Trajectory::plan(waypoints, durations, mass, periodic).map(TrajectoryPlanner::Se3)
    }

    /// Evaluate flat outputs at time `t` — **identical interface for all modes**.
    ///
    /// `t` is clamped to `[0, total_time()]`.  Call this at your control rate
    /// (20–100 Hz) to feed the flatness map → full-state setpoint.
    ///
    /// For Mode 2 (`Se3`) this returns the position / velocity / acceleration and
    /// yaw from the SE(3) trajectory, but **discards** the explicit rotation.
    /// Use `eval_se3(t)` to get the full `Se3Output` with `rot`, `omega`, `thrust`.
    #[inline]
    pub fn eval(&self, t: f32) -> FlatOutput {
        match self {
            TrajectoryPlanner::Spline(s)  => s.eval(t),
            TrajectoryPlanner::Richter(r) => r.eval(t),
            TrajectoryPlanner::Se3(s)     => s.eval_flat(t),
        }
    }

    /// Evaluate full SE(3) outputs at time `t`.
    ///
    /// - Mode 2 (`Se3`): returns native `Se3Output` with explicit `rot`, `omega`,
    ///   `alpha`, `thrust`.
    /// - Mode 0/1: synthesises an `Se3Output` by running the flatness map on the
    ///   polynomial output.  `thrust` will always be non-negative (flatness
    ///   assumption), `alpha` is zeroed.
    pub fn eval_se3(&self, t: f32) -> Se3Output {
        match self {
            TrajectoryPlanner::Se3(s) => s.eval(t),
            _ => {
                // Fall back: run flatness on the flat output and convert
                let flat = self.eval(t);
                Se3Output {
                    pos:    flat.pos,
                    vel:    flat.vel,
                    acc:    flat.acc,
                    rot:    [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]], // identity
                    omega:  crate::math::Vec3::new(0.0, 0.0, flat.yaw_dot),
                    alpha:  crate::math::Vec3::new(0.0, 0.0, flat.yaw_ddot),
                    thrust: 0.0, // caller must compute via flatness
                    yaw:    flat.yaw,
                }
            }
        }
    }

    /// Total trajectory duration [s].
    #[inline]
    pub fn total_time(&self) -> f32 {
        match self {
            TrajectoryPlanner::Spline(s)  => s.total_time,
            TrajectoryPlanner::Richter(r) => r.total_time,
            TrajectoryPlanner::Se3(s)     => s.total_time,
        }
    }

    /// Return a reference to the underlying `SplineTrajectory` position polynomial.
    ///
    /// Used by onboard flight binaries to serialise coefficients for firmware upload.
    /// All three modes store a `SplineTrajectory` internally for the position axes;
    /// the attitude differences (Mode 2) are only relevant for Rust-side simulation.
    pub fn as_spline(&self) -> &SplineTrajectory {
        match self {
            TrajectoryPlanner::Spline(s)  => s,
            TrajectoryPlanner::Richter(r) => r.as_spline(),
            TrajectoryPlanner::Se3(s)     => s.as_spline(),
        }
    }

    /// Fit degree-8 polynomials to roll and pitch for each segment.
    ///
    /// Returns `(croll[9], cpitch[9])` per segment in normalised time τ ∈ [0,1].
    /// Mode 0/1 return an empty Vec (firmware derives attitude from flatness).
    /// Mode 2 returns one pair per segment for firmware upload.
    pub fn attitude_poly_coefs(&self) -> Vec<([f32; 9], [f32; 9])> {
        match self {
            TrajectoryPlanner::Se3(s) => s.attitude_poly_coefs(),
            _ => Vec::new(),
        }
    }

    /// Which mode is active.  0 = Spline, 1 = Richter, 2 = Se3.
    #[inline]
    pub fn mode(&self) -> u8 {
        match self {
            TrajectoryPlanner::Spline(_)  => 0,
            TrajectoryPlanner::Richter(_) => 1,
            TrajectoryPlanner::Se3(_)     => 2,
        }
    }

    /// Per-segment durations [s] (length = number of polynomial segments).
    pub fn segment_durations(&self) -> Vec<f32> {
        match self {
            TrajectoryPlanner::Spline(s)  => s.segments.iter().map(|seg| seg.duration).collect(),
            TrajectoryPlanner::Richter(r) => r.segment_times.clone(),
            TrajectoryPlanner::Se3(s)     => s.segment_times.clone(),
        }
    }

    /// Times **between** segment polynomials at interior junctions only
    /// (excludes `t=0` and `t=total_time`). Empty when there is only one segment.
    pub fn segment_junction_times(&self) -> Vec<f32> {
        let durs = self.segment_durations();
        if durs.len() <= 1 {
            return vec![];
        }
        let mut acc = 0.0_f32;
        let mut out = Vec::with_capacity(durs.len() - 1);
        for &d in &durs[..durs.len() - 1] {
            acc += d;
            out.push(acc);
        }
        out
    }

    /// Run a feasibility check against thrust and angular-rate limits.
    ///
    /// Returns `None` for Mode 0 (`Spline`) — no built-in check.
    /// Mode 1 (`Richter`) and Mode 2 (`Se3`) both return `Some(FeasibilityReport)`.
    pub fn check_feasibility(
        &self,
        mass_kg: f32,
        max_thrust_n: f32,
        max_omega_rad_s: f32,
    ) -> Option<FeasibilityReport> {
        match self {
            TrajectoryPlanner::Richter(r) =>
                Some(r.check_feasibility(mass_kg, max_thrust_n, max_omega_rad_s)),
            TrajectoryPlanner::Se3(s) =>
                Some(s.check_feasibility(max_thrust_n, max_omega_rad_s)),
            _ => None,
        }
    }

    /// Scale segment times until this trajectory respects the given limits,
    /// replanning the position polynomial at each iteration.
    ///
    /// Only implemented for Mode 2 (`Se3`).  Returns `None` for Mode 0/1.
    pub fn repair_feasibility(
        &self,
        max_thrust_n: f32,
        max_omega_rad_s: f32,
        max_iters: usize,
    ) -> Option<Result<Self, String>> {
        match self {
            TrajectoryPlanner::Se3(s) =>
                Some(s.repair_feasibility(max_thrust_n, max_omega_rad_s, max_iters)
                    .map(TrajectoryPlanner::Se3)),
            _ => None,
        }
    }
}
