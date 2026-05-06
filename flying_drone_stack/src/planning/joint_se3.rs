//! Joint SE(3) planner — Mode 3.
//!
//! This mode co-designs position timing and attitude in an iterative loop:
//! 1) plan position with Richter timing
//! 2) derive attitude from the resulting translational dynamics (flatness)
//! 3) build an Se3 trajectory and check feasibility
//! 4) rescale time and repeat when limits are violated
//!
//! Unlike Mode 2 (explicit attitude waypoints), Mode 3 does not require
//! user-authored attitude waypoints and keeps timing/attitude coupled.

use crate::math::{Quat, Vec3};
use super::flatness::{compute_flatness, rot_to_quat};
use super::richter::{FeasibilityReport, RichterTrajectory};
use super::se3_traj::{Se3Output, Se3Trajectory, Se3Waypoint};
use super::spline::{SplineTrajectory, Waypoint};

/// Configuration for Mode 3 joint planning.
#[derive(Debug, Clone, Copy)]
pub struct JointSe3Config {
    /// Richter aggressiveness used for initial timing.
    pub k_t: f32,
    /// Closed-loop topology flag (same semantics as Mode 0/1/2).
    pub periodic: bool,
    /// Vehicle mass [kg].
    pub mass: f32,
    /// Feasibility limit: max thrust magnitude [N].
    pub max_thrust_n: f32,
    /// Feasibility limit: max body-rate magnitude [rad/s].
    pub max_omega_rad_s: f32,
    /// Max number of timing repair iterations.
    pub max_iters: usize,
    /// Extra safety margin applied to suggested time scale.
    pub time_scale_margin: f32,
    /// Optional attitude constraints for advanced joint planning (Mode 4).
    pub attitude_constraint: JointAttitudeConstraint,
}

/// Optional attitude constraints for joint SE(3) planning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointAttitudeConstraint {
    /// Default Mode 3 behaviour: attitude emerges from flatness + yaw policy.
    None,
    /// Enforce an inverted attitude at the highest-altitude waypoint.
    ///
    /// Useful for loop-like manoeuvres where inversion at apex is a hard
    /// choreography requirement.
    InvertAtApex,
}

impl Default for JointSe3Config {
    fn default() -> Self {
        Self {
            k_t: 0.02,
            periodic: false,
            mass: 0.027,
            max_thrust_n: 0.55,
            max_omega_rad_s: 12.0,
            max_iters: 6,
            time_scale_margin: 1.05,
            attitude_constraint: JointAttitudeConstraint::None,
        }
    }
}

/// Mode 3 trajectory: co-designed timing + attitude, emitted as native Se3.
#[derive(Debug, Clone)]
pub struct JointSe3Trajectory {
    /// Final timing-optimized position trajectory.
    richter: RichterTrajectory,
    /// Final SE(3) trajectory with attitude generated from flatness-consistent knots.
    se3: Se3Trajectory,
    /// Attitude waypoints used to build `se3` (for inspection/exports).
    se3_waypoints: Vec<Se3Waypoint>,
    /// Planning config used to build this solution.
    cfg: JointSe3Config,
}

impl JointSe3Trajectory {
    /// Plan Mode 3 from position waypoints only.
    pub fn plan(waypoints: &[Waypoint], cfg: JointSe3Config) -> Result<Self, String> {
        Self::plan_internal(waypoints, None, cfg)
    }

    /// Plan Mode 3 from position waypoints with caller-provided initial segment times.
    ///
    /// Useful for dense paths (e.g. helix with many short segments) where pure
    /// distance-based Richter initialisation can become numerically fragile.
    pub fn plan_from_times(
        waypoints: &[Waypoint],
        times: &[f32],
        cfg: JointSe3Config,
    ) -> Result<Self, String> {
        Self::plan_internal(waypoints, Some(times), cfg)
    }

    fn plan_internal(
        waypoints: &[Waypoint],
        init_times: Option<&[f32]>,
        cfg: JointSe3Config,
    ) -> Result<Self, String> {
        if waypoints.len() < 2 {
            return Err("need at least 2 waypoints".to_string());
        }
        if cfg.mass <= 0.0 {
            return Err("mass must be positive".to_string());
        }
        if cfg.max_thrust_n <= 0.0 || cfg.max_omega_rad_s <= 0.0 {
            return Err("feasibility limits must be positive".to_string());
        }

        let mut richter = if let Some(times) = init_times {
            RichterTrajectory::plan_from_times(waypoints, times, cfg.k_t, cfg.periodic, 0)?
        } else {
            RichterTrajectory::plan(waypoints, cfg.k_t, cfg.periodic)?
        };
        let mut se3_waypoints =
            build_attitude_waypoints_from_richter(&richter, waypoints, cfg.mass, cfg.attitude_constraint);
        let mut se3 = Se3Trajectory::plan(&se3_waypoints, &richter.segment_times, cfg.mass, cfg.periodic)?;

        for _ in 0..cfg.max_iters {
            let rep = se3.check_feasibility(cfg.max_thrust_n, cfg.max_omega_rad_s);
            if rep.feasible {
                return Ok(Self { richter, se3, se3_waypoints, cfg });
            }

            let scale = (rep.suggested_time_scale * cfg.time_scale_margin).max(1.0);
            let scaled_times: Vec<f32> = richter.segment_times.iter().map(|&d| d * scale).collect();
            richter = RichterTrajectory::plan_from_times(
                waypoints,
                &scaled_times,
                cfg.k_t,
                cfg.periodic,
                0,
            )?;
            se3_waypoints =
                build_attitude_waypoints_from_richter(&richter, waypoints, cfg.mass, cfg.attitude_constraint);
            se3 = Se3Trajectory::plan(&se3_waypoints, &richter.segment_times, cfg.mass, cfg.periodic)?;
        }

        Ok(Self { richter, se3, se3_waypoints, cfg })
    }

    #[inline]
    pub fn eval(&self, t: f32) -> Se3Output {
        self.se3.eval(t)
    }

    #[inline]
    pub fn eval_flat(&self, t: f32) -> super::flatness::FlatOutput {
        self.se3.eval_flat(t)
    }

    #[inline]
    pub fn as_spline(&self) -> &SplineTrajectory {
        self.se3.as_spline()
    }

    #[inline]
    pub fn total_time(&self) -> f32 {
        self.se3.total_time
    }

    #[inline]
    pub fn segment_durations(&self) -> Vec<f32> {
        self.se3.segment_times.clone()
    }

    #[inline]
    pub fn attitude_poly_coefs(&self) -> Vec<([f32; 9], [f32; 9])> {
        self.se3.attitude_poly_coefs()
    }

    #[inline]
    pub fn check_feasibility(&self) -> FeasibilityReport {
        self.se3.check_feasibility(self.cfg.max_thrust_n, self.cfg.max_omega_rad_s)
    }

    #[inline]
    pub fn as_se3(&self) -> &Se3Trajectory {
        &self.se3
    }

    #[inline]
    pub fn se3_waypoints(&self) -> &[Se3Waypoint] {
        &self.se3_waypoints
    }

    #[inline]
    pub fn as_richter(&self) -> &RichterTrajectory {
        &self.richter
    }
}

fn build_attitude_waypoints_from_richter(
    richter: &RichterTrajectory,
    pos_waypoints: &[Waypoint],
    mass: f32,
    constraint: JointAttitudeConstraint,
) -> Vec<Se3Waypoint> {
    let mut t_knots = Vec::with_capacity(richter.segment_times.len() + 1);
    t_knots.push(0.0_f32);
    let mut acc = 0.0_f32;
    for &d in &richter.segment_times {
        acc += d;
        t_knots.push(acc);
    }

    let mut se3_wps: Vec<Se3Waypoint> = pos_waypoints
        .iter()
        .zip(t_knots.iter())
        .map(|(wp, &t)| {
            let flat = richter.eval(t);
            let fr = compute_flatness(&flat, mass);
            // Build attitude from flatness body-z + desired waypoint yaw.
            // This avoids Euler branch ambiguities and keeps yaw policy explicit.
            let zb = Vec3::new(fr.rot[0][2], fr.rot[1][2], fr.rot[2][2]).normalize();
            let xc = Vec3::new(wp.yaw.cos(), wp.yaw.sin(), 0.0);
            let mut yb = zb.cross(&xc);
            if yb.norm() < 1e-6 {
                // Fallback when zb and xc are nearly colinear.
                yb = Vec3::new(0.0, 1.0, 0.0);
            } else {
                yb = yb.normalize();
            }
            let xb = yb.cross(&zb).normalize();
            // `rot_to_quat` expects the flatness layout [col0, col1, col2],
            // i.e. each entry is a body axis expressed in world coordinates.
            let rot = [
                [xb.x, xb.y, xb.z],
                [yb.x, yb.y, yb.z],
                [zb.x, zb.y, zb.z],
            ];
            let q_fix = rot_to_quat(&rot);
            let q = Quat::new(q_fix[0], q_fix[1], q_fix[2], q_fix[3]).normalize();
            Se3Waypoint::new(wp.pos, q)
        })
        .collect();

    // Keep quaternion signs continuous across knots so SLERP always follows
    // the short arc. This avoids large artificial body-rate spikes from
    // q and -q representation flips between neighbouring waypoints.
    for i in 1..se3_wps.len() {
        let qa = se3_wps[i - 1].att;
        let qb = se3_wps[i].att;
        let dot = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
        if dot < 0.0 {
            se3_wps[i].att = Quat::new(-qb.w, -qb.x, -qb.y, -qb.z);
        }
    }

    apply_attitude_constraint(&mut se3_wps, pos_waypoints, constraint);
    enforce_quat_sign_continuity(&mut se3_wps);

    se3_wps
}

fn apply_attitude_constraint(
    se3_wps: &mut [Se3Waypoint],
    pos_waypoints: &[Waypoint],
    constraint: JointAttitudeConstraint,
) {
    if constraint != JointAttitudeConstraint::InvertAtApex || se3_wps.is_empty() {
        return;
    }
    let mut apex_idx = 0usize;
    let mut apex_z = pos_waypoints[0].pos.z;
    for (i, wp) in pos_waypoints.iter().enumerate().skip(1) {
        if wp.pos.z > apex_z {
            apex_z = wp.pos.z;
            apex_idx = i;
        }
    }
    let yaw = pos_waypoints[apex_idx].yaw;
    // Inverted at apex: keep heading from waypoint yaw, flip body-z downward.
    let q = quat_from_world_z(yaw, Vec3::new(0.0, 0.0, -1.0));
    se3_wps[apex_idx].att = q;
}

fn quat_from_world_z(yaw: f32, zb: Vec3) -> Quat {
    let xc = Vec3::new(yaw.cos(), yaw.sin(), 0.0);
    let mut yb = zb.cross(&xc);
    if yb.norm() < 1e-6 {
        yb = Vec3::new(0.0, 1.0, 0.0);
    } else {
        yb = yb.normalize();
    }
    let xb = yb.cross(&zb).normalize();
    let rot = [
        [xb.x, xb.y, xb.z],
        [yb.x, yb.y, yb.z],
        [zb.x, zb.y, zb.z],
    ];
    let q_fix = rot_to_quat(&rot);
    Quat::new(q_fix[0], q_fix[1], q_fix[2], q_fix[3]).normalize()
}

fn enforce_quat_sign_continuity(se3_wps: &mut [Se3Waypoint]) {
    for i in 1..se3_wps.len() {
        let qa = se3_wps[i - 1].att;
        let qb = se3_wps[i].att;
        let dot = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
        if dot < 0.0 {
            se3_wps[i].att = Quat::new(-qb.w, -qb.x, -qb.y, -qb.z);
        }
    }
}

