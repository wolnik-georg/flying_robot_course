//! Planning Mode Simulation
//!
//! Completely separate from Assignment 4. This binary validates all planning
//! planning modes across six trajectory shapes in closed-loop simulation
//! before any real hardware flight.
//!
//! ## Trajectories
//!   circle   — horizontal circle, R=0.5 m, 8 waypoints
//!   figure8  — Poly4D-matched 11 waypoints / 10 segments (same as onboard_figure8 /
//!              spline_figure8_test), periodic min-snap closure
//!   helix    — ascending spiral, 8 waypoints per lap
//!   corner   — racing-style banked corner (open path, non-periodic)
//!   flip     — forward 360° pitch flip (Mode 2 only; Mode 0/1 motor cut expected)
//!   loop     — vertical loop in X-Z plane, R=0.5 m (requires enough speed to invert)
//!   corkscrew— ascending helix + rolling 360° per lap (Mode 0/1 helix position; Mode 2 explicit roll)
//!   roll     — stationary 360° pitch maneuver / backflip (Mode 2 only, y-axis rotation)
//!   screw    — stationary XY, ascending Z + 360° yaw spin (Mode 2 only; body-z always up, no inversion)
//!
//! ## Planning modes applied per trajectory
//!   Mode 0 (SplineTrajectory — manual durations)  : circle, figure8, helix, corner, loop, corkscrew
//!   Mode 1 (RichterTrajectory — auto k_t timing)  : circle, figure8, helix, corner, loop, corkscrew
//!   Mode 2 (Se3Trajectory — explicit attitude on all trajectories):
//!     — circle, figure8, helix use explicit levelled quaternion waypoints
//!     — corner uses explicit roll-bias profile (pre-bank -> apex bank -> unload)
//!     — corkscrew uses explicit x-axis roll schedule (0→2π per lap, passes through zero thrust)
//!     — flip, roll, loop use explicit quaternion waypoints (zero-thrust inversion)
//!     — screw uses Rz yaw-spin waypoints (body-z always up, no inversion)
//!     — **position** segment times match **Mode 1 Richter** allocation (same `k_t` / `periodic`
//!       per shape); flip and roll keep explicit manual segment times.
//!
//! ## Controllers simulated
//!   Geo  — GeometricController::default()          (Lee SE(3) — model-based torque)
//!   INDI — indi_for_sim()   (incremental inner loop, same outer)
//!   Each trajectory is run with both; CSVs named <mode>_<traj>_geo.csv / _indi.csv.
//!
//! ## Outputs
//!   **Closed-loop** (controller + simulator), one folder per mode and trajectory shape:
//!   `results/planning_sim/closed_loop/mode{N}/{trajectory}/{geo,indi}.csv`
//!   columns: t, ref_x/y/z, sim_x/y/z, ref_roll/pitch/yaw, sim_roll/pitch/yaw,
//!            ref_thrust, cmd_thrust, cmd_tx/ty/tz, error_3d
//!
//!   **Planner-only** (open-loop reference, no simulation) for validating motion planning:
//!   `results/planning_sim/reference/mode{N}/{trajectory}/`
//!   — `reference.csv` (dense samples + quaternion), `waypoints.csv`, `segment_junctions.csv`,
//!   `planning_meta.txt`, and optional `feasibility.txt` for Mode 1 (Richter).
//!   See `--export-reference-only`.
//!
//! ## Run
//!   cargo run --release --bin planning_sim
//!   scripts/plot_planning_all.sh
//!
//!   cargo run --release --bin planning_sim -- --export-reference-only
//!   scripts/plot_planning_all.sh

use multirotor_simulator::prelude::*;
use multirotor_simulator::planning::se3_traj::{Se3Trajectory, Se3Waypoint};
use std::f32::consts::PI;
use std::fs::File;
use std::io::Write;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const DT:   f32 = 0.001;  // 1 kHz physics
const MASS: f32 = 0.027;  // Crazyflie 2.1 [kg]
const Z:    f32 = 1.0;    // default hover height [m]
const FIG8_PERIODIC: bool = false;

// ---------------------------------------------------------------------------
// INDI-with-feedforward for simulation
//
// IndiController::crazyflie_default() diverges on trajectory tracking because
// it has no angular-acceleration feedforward (alpha_d = 0).  Without feedforward,
// du = (alpha_ref - alpha_meas) / g1 is non-zero at every steady-state point
// where the reference attitude is changing, so tau_prev integrates without bound.
//
// Fix: include alpha_d from compute_flatness (FlatnessResult.omega_dot) in alpha_ref:
//   alpha_ref = alpha_d  -  KR·eR  -  KW·eω
// This gives du≈0 at perfect tracking and matches the geometric controller in gains.
//
// Gains:  KR = kr_geo / J  [rad/s²/rad],  KW = 2·KR  (Routh: KW > KR)
// Outer loop: identical to geometric (position error → thrust + Rd).
// ---------------------------------------------------------------------------

struct IndiSimState {
    tau_prev:    Vec3,
    omega_prev:  Vec3, // previous omega for finite-difference alpha_meas (no filter — sim has no noise)
    alpha_filt:  Vec3, // low-pass filtered angular acceleration estimate
    initialized: bool,
}

impl IndiSimState {
    fn new() -> Self {
        Self {
            tau_prev:   Vec3::zero(),
            omega_prev: Vec3::zero(),
            alpha_filt: Vec3::zero(),
            initialized: false,
        }
    }

    /// Compute INDI torque with angular-acceleration feedforward.
    ///
    /// # Arguments
    /// - `state`    : current multirotor state
    /// - `rd`       : desired rotation matrix from outer loop (used for attitude error eR)
    /// - `rot_ff`   : desired rotation matrix from flatness/Se3 (frame in which omega_d/alpha_d live)
    ///                Pass the same value as `rd` when they are the same (e.g. Se3 mode).
    /// - `omega_d`  : reference body angular velocity in `rot_ff` body frame
    /// - `alpha_d`  : reference body angular acceleration in `rot_ff` body frame
    /// - `params`   : model parameters
    fn compute(
        &mut self,
        state:   &MultirotorState,
        rd:      &[[f32; 3]; 3],
        rot_ff:  &[[f32; 3]; 3],   // frame in which omega_d / alpha_d are expressed
        omega_d: Vec3,
        alpha_d: Vec3,
        thrust:  f32,
        params:  &MultirotorParams,
    ) -> Vec3 {
        let jxx = params.inertia[0][0];
        let jyy = params.inertia[1][1];
        let jzz = params.inertia[2][2];

        // Gains equivalent to GeometricController::default() in [rad/s²] space:
        //   kr_indi = kr_geo / J  →  J * kr_indi = kr_geo  (same torque per angle)
        //   kw_indi = kw_geo / J  →  J * kw_indi = kw_geo  (same damping per rate)
        // ζ = kw_indi / (2√kr_indi) = kw_geo / (2*√(kr_geo*J)) ≈ 1.69 (overdamped, stable)
        let kr = Vec3::new(0.007  / jxx, 0.007  / jyy, 0.008  / jzz); // ≈ 422 rad/s²/rad
        let kw = Vec3::new(0.00115/ jxx, 0.00115/ jyy, 0.002  / jzz); // ≈ 69.4 rad/s²/(rad/s)

        let omega  = state.angular_velocity;
        let r = state.orientation.to_rotation_matrix();

        // eR = ½ vee(Rd^T R − R^T Rd)
        let mut rd_t_r = [[0.0_f32; 3]; 3];
        let mut r_t_rd = [[0.0_f32; 3]; 3];
        for i in 0..3 { for j in 0..3 { for k in 0..3 {
            rd_t_r[i][j] += rd[k][i] * r[k][j];
            r_t_rd[i][j] += r[k][i] * rd[k][j];
        }}}
        let er = Vec3::new(
            0.5 * (rd_t_r[2][1] - r_t_rd[2][1]),
            0.5 * (rd_t_r[0][2] - r_t_rd[0][2]),
            0.5 * (rd_t_r[1][0] - r_t_rd[1][0]),
        );

        // omega_d and alpha_d are expressed in the rot_ff body frame (flatness/Se3 desired frame).
        // Transform to actual body frame R:  v_R = R^T * rot_ff * v_ff
        // Note: rot_ff ≠ rd when outer-loop position errors cause rd to diverge from flatness rot.
        let mut omega_d_r = Vec3::zero();
        let mut alpha_d_r = Vec3::zero();
        for i in 0..3 {
            let mut wd_world_i = 0.0_f32;
            let mut ad_world_i = 0.0_f32;
            for j in 0..3 {
                wd_world_i += rot_ff[i][j] * [omega_d.x, omega_d.y, omega_d.z][j];
                ad_world_i += rot_ff[i][j] * [alpha_d.x, alpha_d.y, alpha_d.z][j];
            }
            // project into actual body frame R: v_R = R^T v_W, i.e. v_R[j] = Σ_i R[i][j] v_W[i]
            omega_d_r.x += r[i][0] * wd_world_i;
            omega_d_r.y += r[i][1] * wd_world_i;
            omega_d_r.z += r[i][2] * wd_world_i;
            alpha_d_r.x += r[i][0] * ad_world_i;
            alpha_d_r.y += r[i][1] * ad_world_i;
            alpha_d_r.z += r[i][2] * ad_world_i;
        }
        // eomega in actual body frame R (matches geometric controller)
        let eomega = omega - omega_d_r;

        // Seed on first call — initialize tau_prev to geometric baseline
        if !self.initialized {
            self.omega_prev = omega;
            self.alpha_filt = Vec3::zero();
            self.initialized = true;
            let tau0 = Vec3::new(
                -0.007 * er.x - 0.00115 * eomega.x,
                -0.007 * er.y - 0.00115 * eomega.y,
                -0.008 * er.z - 0.002   * eomega.z,
            );
            self.tau_prev = tau0;
            return tau0;
        }

        // Finite-difference for alpha_meas + light LPF to reduce numerical chatter
        // from integration/discretization (helps incremental loop robustness).
        let alpha_raw = (omega - self.omega_prev) * (1.0 / DT);
        const ALPHA_LPF_BETA: f32 = 0.2;
        self.alpha_filt = self.alpha_filt + (alpha_raw - self.alpha_filt) * ALPHA_LPF_BETA;
        let alpha_meas = self.alpha_filt;
        self.omega_prev = omega;

        // alpha_ref = feedforward (in R frame) + feedback
        let alpha_ref = Vec3::new(
            alpha_d_r.x - kr.x * er.x - kw.x * eomega.x,
            alpha_d_r.y - kr.y * er.y - kw.y * eomega.y,
            alpha_d_r.z - kr.z * er.z - kw.z * eomega.z,
        );

        // INDI increment: δτ = (alpha_ref - alpha_meas) · J
        let du = Vec3::new(
            (alpha_ref.x - alpha_meas.x) * jxx,
            (alpha_ref.y - alpha_meas.y) * jyy,
            (alpha_ref.z - alpha_meas.z) * jzz,
        );

        // Incremental torque (clamped per-axis to motor capability)
        const TAU_CLAMP_NOMINAL: f32 = 0.0040;
        const TAU_CLAMP_LOW_T:  f32 = 0.0010;
        let tau_clamp = if thrust < 0.05 { TAU_CLAMP_LOW_T } else { TAU_CLAMP_NOMINAL };
        let tau = Vec3::new(
            (self.tau_prev.x + du.x).clamp(-tau_clamp, tau_clamp),
            (self.tau_prev.y + du.y).clamp(-tau_clamp, tau_clamp),
            (self.tau_prev.z + du.z).clamp(-tau_clamp, tau_clamp),
        );
        self.omega_prev = omega;

        // Match INDI anti-windup behavior: freeze/reset incremental state below
        // a small thrust threshold (e.g. during motor cut in inversion).
        if thrust < 0.01 {
            self.tau_prev = Vec3::zero();
            return Vec3::zero();
        }

        self.tau_prev = tau;
        // Gyroscopic compensation ω × Jω
        let j_omega   = Vec3::new(jxx*omega.x, jyy*omega.y, jzz*omega.z);
        let gyro_comp = omega.cross(&j_omega);
        tau + gyro_comp
    }
}

fn outer_loop_with_gains(
    state: &MultirotorState,
    reference: &TrajectoryReference,
    params: &MultirotorParams,
    kp: Vec3,
    kv: Vec3,
) -> (f32, [[f32; 3]; 3]) {
    let ep = reference.position - state.position;
    let ev = reference.velocity - state.velocity;
    let f_d = reference.acceleration
        + Vec3::new(kp.x*ep.x, kp.y*ep.y, kp.z*ep.z)
        + Vec3::new(kv.x*ev.x, kv.y*ev.y, kv.z*ev.z)
        + Vec3::new(0.0, 0.0, params.gravity);
    let thrust_force = f_d * params.mass;
    let r = state.orientation.to_rotation_matrix();
    let body_z = Vec3::new(r[0][2], r[1][2], r[2][2]);
    let thrust = thrust_force.dot(&body_z).max(0.0);

    // Desired rotation from thrust direction + yaw
    let mut zdes = Vec3::new(0.0, 0.0, 1.0);
    if thrust_force.norm() > 0.0 { zdes = thrust_force.normalize(); }
    let xcdes = Vec3::new(reference.yaw.cos(), reference.yaw.sin(), 0.0);
    let zcrossx = zdes.cross(&xcdes);
    let ydes = if zcrossx.norm() > 0.0 { zcrossx.normalize() } else { Vec3::new(0.0, 1.0, 0.0) };
    let xdes = ydes.cross(&zdes);
    let rd = [[xdes.x, ydes.x, zdes.x],
              [xdes.y, ydes.y, zdes.y],
              [xdes.z, ydes.z, zdes.z]];
    (thrust, rd)
}

// ---------------------------------------------------------------------------
// CSV record — identical schema for every run
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct Rec {
    t:          f32,
    ref_x:      f32, ref_y:      f32, ref_z:      f32,
    ref_roll:   f32, ref_pitch:  f32, ref_yaw:    f32,
    ref_thrust: f32,
    sim_x:      f32, sim_y:      f32, sim_z:      f32,
    sim_roll:   f32, sim_pitch:  f32, sim_yaw:    f32,
    cmd_thrust: f32,
    cmd_tx:     f32, cmd_ty:     f32, cmd_tz:     f32,
    error_3d:   f32,
}

fn write_csv(path: &str, recs: &[Rec]) {
    let mut f = File::create(path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(f,
        "t,ref_x,ref_y,ref_z,ref_roll,ref_pitch,ref_yaw,ref_thrust,\
         sim_x,sim_y,sim_z,sim_roll,sim_pitch,sim_yaw,\
         cmd_thrust,cmd_tx,cmd_ty,cmd_tz,error_3d"
    ).unwrap();
    for r in recs {
        writeln!(f,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6}",
            r.t,
            r.ref_x, r.ref_y, r.ref_z,
            r.ref_roll, r.ref_pitch, r.ref_yaw, r.ref_thrust,
            r.sim_x, r.sim_y, r.sim_z,
            r.sim_roll, r.sim_pitch, r.sim_yaw,
            r.cmd_thrust, r.cmd_tx, r.cmd_ty, r.cmd_tz,
            r.error_3d,
        ).unwrap();
    }
}

fn rms_3d(recs: &[Rec]) -> f32 {
    let s: f32 = recs.iter().map(|r| r.error_3d * r.error_3d).sum();
    (s / recs.len() as f32).sqrt()
}

// ---------------------------------------------------------------------------
// Euler angle helpers
// ---------------------------------------------------------------------------

fn rot_to_euler(rot: &[[f32; 3]; 3]) -> (f32, f32, f32) {
    // Row-major: rot[row][col]
    let pitch = -(rot[2][0]).asin();
    let roll  = rot[2][1].atan2(rot[2][2]);
    let yaw   = rot[1][0].atan2(rot[0][0]);
    (roll, pitch, yaw)
}

// ---------------------------------------------------------------------------
// Planner-only reference export (no controller / no simulator)
// ---------------------------------------------------------------------------

/// Sample period for `reference.csv` (Hz = 1/REF_DT). Slightly coarser than physics DT
/// to keep file sizes reasonable for plotting.
const REF_DT: f32 = 0.002;

#[derive(Clone, Copy)]
struct FeasibilityLimits {
    max_thrust_n: f32,
    max_omega_rad_s: f32,
    max_torque_axis_nm: f32,
}

const FEAS_CF2: FeasibilityLimits = FeasibilityLimits {
    max_thrust_n: 0.55,
    max_omega_rad_s: 12.0,
    max_torque_axis_nm: 0.004,
};

fn reference_dir(mode: u8, traj_folder: &str) -> String {
    format!("results/planning_sim/reference/mode{}/{}", mode, traj_folder)
}

fn closed_loop_csv(mode: u8, traj_folder: &str, kind: &str) -> String {
    format!(
        "results/planning_sim/closed_loop/mode{}/{}/{}.csv",
        mode, traj_folder, kind
    )
}

/// Motor ω² before non-negativity clamp — any negative entry means the (T,τ) pair is not
/// realisable with **positive** thrust only on this X-quad mixing (same algebra as
/// [`MotorAction::from_thrust_torque`]).
fn motor_omega_sq_unclamped(thrust: f32, torque: Vec3, p: &MultirotorParams) -> [f32; 4] {
    let kf = p.kf;
    let kt = p.kt;
    let l = p.arm_length;
    let sqrt2 = 2.0_f32.sqrt();
    let l_sqrt2 = l / sqrt2;
    let thrust_term = thrust / (4.0 * kf);
    let roll_term = torque.x / (4.0 * kf * l_sqrt2);
    let pitch_term = torque.y / (4.0 * kf * l_sqrt2);
    let yaw_term = torque.z / (4.0 * kt);
    [
        thrust_term - roll_term - pitch_term + yaw_term,
        thrust_term + roll_term - pitch_term - yaw_term,
        thrust_term + roll_term + pitch_term + yaw_term,
        thrust_term - roll_term + pitch_term - yaw_term,
    ]
}

fn ensure_parent(path: &str) {
    if let Some(p) = std::path::Path::new(path).parent() {
        std::fs::create_dir_all(p).unwrap_or_else(|e| panic!("mkdir {:?}: {}", p, e));
    }
}

fn write_closed_loop_csv(mode: u8, traj_folder: &str, kind: &str, recs: &[Rec]) {
    let path = closed_loop_csv(mode, traj_folder, kind);
    ensure_parent(&path);
    write_csv(&path, recs);
}

/// Flat trajectory (Mode 0 / 1): full `FlatOutput` + `compute_flatness` (Euler, thrust, ω, τ, quaternion).
fn write_flat_reference_csv(path: &str, planner: &TrajectoryPlanner) {
    ensure_parent(path);
    let total = planner.total_time();
    let steps = ((total / REF_DT).ceil() as usize).saturating_add(1).max(2);
    let mut f = File::create(path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(
        f,
        "t,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,sx,sy,sz,\
         yaw,yaw_dot,yaw_ddot,\
         euler_roll,euler_pitch,euler_yaw,\
         thrust_N,\
         omega_x,omega_y,omega_z,\
         angacc_x,angacc_y,angacc_z,\
         torque_x,torque_y,torque_z,\
         qw,qx,qy,qz,\
         angjerk_x,angjerk_y,angjerk_z"
    )
    .unwrap();

    let mut prev_omegadot: Option<Vec3> = None;
    for k in 0..steps {
        let t = (k as f32 * REF_DT).min(total);
        let flat = planner.eval(t);
        let fr = compute_flatness(&flat, MASS);
        let (er, ep, ey) = rot_to_euler(&fr.rot);
        let q = rot_to_quat(&fr.rot);
        let aj = if let Some(po) = prev_omegadot {
            Vec3::new(
                (fr.omega_dot.x - po.x) / REF_DT,
                (fr.omega_dot.y - po.y) / REF_DT,
                (fr.omega_dot.z - po.z) / REF_DT,
            )
        } else {
            Vec3::zero()
        };
        prev_omegadot = Some(fr.omega_dot);
        writeln!(
            f,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            t,
            flat.pos.x, flat.pos.y, flat.pos.z,
            flat.vel.x, flat.vel.y, flat.vel.z,
            flat.acc.x, flat.acc.y, flat.acc.z,
            flat.jerk.x, flat.jerk.y, flat.jerk.z,
            flat.snap.x, flat.snap.y, flat.snap.z,
            flat.yaw, flat.yaw_dot, flat.yaw_ddot,
            er, ep, ey,
            fr.thrust,
            fr.omega.x, fr.omega.y, fr.omega.z,
            fr.omega_dot.x, fr.omega_dot.y, fr.omega_dot.z,
            fr.torque.x, fr.torque.y, fr.torque.z,
            q[0], q[1], q[2], q[3],
            aj.x, aj.y, aj.z,
        )
        .unwrap();
    }
}

/// Mode 2: native `Se3Trajectory::eval` + position spline jerk/snap + angular jerk from Δα/Δt.
fn write_se3_reference_csv(path: &str, traj: &Se3Trajectory) {
    ensure_parent(path);
    let total = traj.total_time;
    let steps = ((total / REF_DT).ceil() as usize).saturating_add(1).max(2);
    let mut f = File::create(path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(
        f,
        "t,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,sx,sy,sz,\
         yaw,yaw_dot,yaw_ddot,\
         euler_roll,euler_pitch,euler_yaw,\
         thrust_N,\
         omega_x,omega_y,omega_z,\
         angacc_x,angacc_y,angacc_z,\
         torque_x,torque_y,torque_z,\
         qw,qx,qy,qz,\
         angjerk_x,angjerk_y,angjerk_z"
    )
    .unwrap();

    let mut prev_alpha: Option<Vec3> = None;
    for k in 0..steps {
        let t = (k as f32 * REF_DT).min(total);
        let out = traj.eval(t);
        // Jerk/snap from the **position** min-snap spline (independent of SLERP attitude).
        let pflat = traj.eval_position_flat(t);
        let (er, ep, ey) = rot_to_euler(&out.rot);
        let flat = out.to_flat();
        let q = rot_to_quat(&out.rot);
        let tau = body_torque_diagonal(out.omega, out.alpha);
        let aj = if let Some(pa) = prev_alpha {
            Vec3::new(
                (out.alpha.x - pa.x) / REF_DT,
                (out.alpha.y - pa.y) / REF_DT,
                (out.alpha.z - pa.z) / REF_DT,
            )
        } else {
            Vec3::zero()
        };
        prev_alpha = Some(out.alpha);
        writeln!(
            f,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            t,
            out.pos.x, out.pos.y, out.pos.z,
            out.vel.x, out.vel.y, out.vel.z,
            out.acc.x, out.acc.y, out.acc.z,
            pflat.jerk.x, pflat.jerk.y, pflat.jerk.z,
            pflat.snap.x, pflat.snap.y, pflat.snap.z,
            flat.yaw, flat.yaw_dot, flat.yaw_ddot,
            er, ep, ey,
            out.thrust,
            out.omega.x, out.omega.y, out.omega.z,
            out.alpha.x, out.alpha.y, out.alpha.z,
            tau.x, tau.y, tau.z,
            q[0], q[1], q[2], q[3],
            aj.x, aj.y, aj.z,
        )
        .unwrap();
    }
}

/// Interior junction times from segment durations (same semantics as [`TrajectoryPlanner::segment_junction_times`]).
fn junction_times_from_durations(durs: &[f32]) -> Vec<f32> {
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

fn write_flat_waypoints_csv(path: &str, wps: &[Waypoint]) {
    ensure_parent(path);
    let mut f = File::create(path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(f, "px,py,pz,yaw_rad").unwrap();
    for w in wps {
        writeln!(
            f,
            "{:.6},{:.6},{:.6},{:.6}",
            w.pos.x, w.pos.y, w.pos.z, w.yaw
        )
        .unwrap();
    }
}

fn write_se3_waypoints_csv(path: &str, wps: &[Se3Waypoint]) {
    ensure_parent(path);
    let mut f = File::create(path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(f, "px,py,pz,qw,qx,qy,qz").unwrap();
    for w in wps {
        let q = w.att;
        writeln!(
            f,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            w.pos.x, w.pos.y, w.pos.z, q.w, q.x, q.y, q.z
        )
        .unwrap();
    }
}

fn write_segment_junctions_csv(path: &str, junctions: &[f32]) {
    ensure_parent(path);
    let mut f = File::create(path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(f, "t_junction_s").unwrap();
    for t in junctions {
        writeln!(f, "{:.6}", t).unwrap();
    }
}

fn write_planning_meta_txt(
    dir: &str,
    folder_mode: u8,
    traj: &str,
    planner_kind: &str,
    se3_jerk_snap_zero: bool,
    lim: FeasibilityLimits,
) {
    let path = format!("{}/planning_meta.txt", dir);
    ensure_parent(&path);
    let mut f = File::create(&path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(f, "folder_mode={}", folder_mode).unwrap();
    writeln!(f, "trajectory_name={}", traj).unwrap();
    writeln!(f, "planner_kind={}", planner_kind).unwrap();
    writeln!(
        f,
        "feasibility_max_thrust_N={:.6}",
        lim.max_thrust_n
    )
    .unwrap();
    writeln!(
        f,
        "feasibility_max_omega_rad_s={:.6}",
        lim.max_omega_rad_s
    )
    .unwrap();
    writeln!(
        f,
        "feasibility_max_torque_axis_Nm={:.6}",
        lim.max_torque_axis_nm
    )
    .unwrap();
    writeln!(
        f,
        "se3_export_jerk_snap_zero={}",
        if se3_jerk_snap_zero { 1 } else { 0 }
    )
    .unwrap();
    if !se3_jerk_snap_zero {
        writeln!(
            f,
            "se3_jerk_snap_source=position_min_snap_spline_independent_of_attitude"
        )
        .unwrap();
        writeln!(
            f,
            "se3_torque_model=diagonal_inertia_J_alpha_minus_Jomega_cross_omega"
        )
        .unwrap();
    }
    writeln!(
        f,
        "euler_note=ZYX_from_rotation_matrix_gimbal_possible_near_pitch_pm_pi2_prefer_quaternion_columns"
    )
    .unwrap();
}

fn append_planning_meta_kv(dir: &str, key: &str, value: &str) {
    let path = format!("{}/planning_meta.txt", dir);
    let mut f = std::fs::OpenOptions::new()
        .append(true)
        .create(true)
        .open(&path)
        .unwrap_or_else(|e| panic!("cannot append {path}: {e}"));
    writeln!(f, "{}={}", key, value).unwrap();
}

fn export_flat_reference_bundle(
    dir: &str,
    planner: &TrajectoryPlanner,
    wps: &[Waypoint],
    traj: &str,
    folder_mode: u8,
    planner_kind: &str,
    richter_feas_report: bool,
) {
    std::fs::create_dir_all(dir).expect("mkdir");
    write_flat_reference_csv(&format!("{}/reference.csv", dir), planner);
    write_flat_waypoints_csv(&format!("{}/waypoints.csv", dir), wps);
    write_segment_junctions_csv(
        &format!("{}/segment_junctions.csv", dir),
        &planner.segment_junction_times(),
    );
    write_planning_meta_txt(dir, folder_mode, traj, planner_kind, false, FEAS_CF2);
    if richter_feas_report {
        write_richter_feasibility_sidecar(dir, planner);
    }
    let params_cf = MultirotorParams::crazyflie();
    write_sampled_feasibility_flat(dir, planner);
    write_motor_mixing_summary_flat(dir, planner, &params_cf);
    println!("  wrote {} (reference + sidecars)", dir);
}

fn write_se3_feasibility_sidecar(dir: &str, traj: &Se3Trajectory, lim: FeasibilityLimits) {
    let total = traj.total_time;
    let steps = ((total / REF_DT).ceil() as usize).saturating_add(1).max(2);
    let mut max_th = 0.0_f32;
    let mut max_om = 0.0_f32;
    let mut max_tau_ax = 0.0_f32;
    let mut t_at_th = 0.0_f32;
    let mut t_at_om = 0.0_f32;
    let mut t_at_tau = 0.0_f32;
    for k in 0..steps {
        let t = (k as f32 * REF_DT).min(total);
        let out = traj.eval(t);
        let tau = body_torque_diagonal(out.omega, out.alpha);
        let ta = out.thrust.abs();
        let om = (out.omega.x * out.omega.x + out.omega.y * out.omega.y + out.omega.z * out.omega.z).sqrt();
        let tax = tau.x.abs().max(tau.y.abs()).max(tau.z.abs());
        if ta > max_th {
            max_th = ta;
            t_at_th = t;
        }
        if om > max_om {
            max_om = om;
            t_at_om = t;
        }
        if tax > max_tau_ax {
            max_tau_ax = tax;
            t_at_tau = t;
        }
    }
    let feasible = max_th <= lim.max_thrust_n + 1e-5
        && max_om <= lim.max_omega_rad_s + 1e-3
        && max_tau_ax <= lim.max_torque_axis_nm + 1e-6;
    let path = format!("{}/feasibility_se3.txt", dir);
    ensure_parent(&path);
    let mut f = File::create(&path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(f, "feasible={}", feasible).unwrap();
    writeln!(f, "peak_abs_thrust_N={:.6}", max_th).unwrap();
    writeln!(f, "peak_thrust_time_s={:.6}", t_at_th).unwrap();
    writeln!(f, "peak_omega_rad_s={:.6}", max_om).unwrap();
    writeln!(f, "peak_omega_time_s={:.6}", t_at_om).unwrap();
    writeln!(f, "peak_torque_axis_abs_Nm={:.6}", max_tau_ax).unwrap();
    writeln!(f, "peak_torque_time_s={:.6}", t_at_tau).unwrap();
    writeln!(f, "limits_thrust_N={:.6}", lim.max_thrust_n).unwrap();
    writeln!(f, "limits_omega_rad_s={:.6}", lim.max_omega_rad_s).unwrap();
    writeln!(f, "limits_torque_axis_Nm={:.6}", lim.max_torque_axis_nm).unwrap();
}

fn write_sampled_feasibility_flat(dir: &str, planner: &TrajectoryPlanner) {
    let total = planner.total_time();
    let steps = ((total / REF_DT).ceil() as usize).saturating_add(1).max(2);
    let mut max_th = 0.0_f32;
    let mut max_om = 0.0_f32;
    let mut max_tau_ax = 0.0_f32;
    let mut t_th = 0.0_f32;
    let mut t_om = 0.0_f32;
    let mut t_tau = 0.0_f32;
    for k in 0..steps {
        let t = (k as f32 * REF_DT).min(total);
        let flat = planner.eval(t);
        let fr = compute_flatness(&flat, MASS);
        let ta = fr.thrust.abs();
        let om = (fr.omega.x * fr.omega.x + fr.omega.y * fr.omega.y + fr.omega.z * fr.omega.z).sqrt();
        let tax = fr.torque.x.abs().max(fr.torque.y.abs()).max(fr.torque.z.abs());
        if ta > max_th {
            max_th = ta;
            t_th = t;
        }
        if om > max_om {
            max_om = om;
            t_om = t;
        }
        if tax > max_tau_ax {
            max_tau_ax = tax;
            t_tau = t;
        }
    }
    let feasible = max_th <= FEAS_CF2.max_thrust_n + 1e-5
        && max_om <= FEAS_CF2.max_omega_rad_s + 1e-3
        && max_tau_ax <= FEAS_CF2.max_torque_axis_nm + 1e-6;
    let path = format!("{}/feasibility_sampled.txt", dir);
    ensure_parent(&path);
    let mut f = File::create(&path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(f, "feasible_thrust_omega_torque={}", feasible).unwrap();
    writeln!(f, "peak_thrust_N={:.6}", max_th).unwrap();
    writeln!(f, "peak_thrust_time_s={:.6}", t_th).unwrap();
    writeln!(f, "peak_omega_rad_s={:.6}", max_om).unwrap();
    writeln!(f, "peak_omega_time_s={:.6}", t_om).unwrap();
    writeln!(f, "peak_torque_axis_abs_Nm={:.6}", max_tau_ax).unwrap();
    writeln!(f, "peak_torque_time_s={:.6}", t_tau).unwrap();
    writeln!(f, "limits_thrust_N={:.6}", FEAS_CF2.max_thrust_n).unwrap();
    writeln!(f, "limits_omega_rad_s={:.6}", FEAS_CF2.max_omega_rad_s).unwrap();
    writeln!(f, "limits_torque_axis_Nm={:.6}", FEAS_CF2.max_torque_axis_nm).unwrap();
    writeln!(
        f,
        "note=grid_sampled_same_dt_as_reference_csv_differential_flatness"
    )
    .unwrap();
}

fn write_motor_mixing_summary_flat(dir: &str, planner: &TrajectoryPlanner, params: &MultirotorParams) {
    let total = planner.total_time();
    let steps = ((total / REF_DT).ceil() as usize).saturating_add(1).max(2);
    let mut n_neg = 0usize;
    for k in 0..steps {
        let t = (k as f32 * REF_DT).min(total);
        let flat = planner.eval(t);
        let fr = compute_flatness(&flat, MASS);
        let raw = motor_omega_sq_unclamped(fr.thrust, fr.torque, params);
        if raw.iter().any(|&x| x < -1e-8) {
            n_neg += 1;
        }
    }
    let frac_ok = 1.0 - (n_neg as f32 / steps as f32);
    let path = format!("{}/motor_mixing_summary.txt", dir);
    ensure_parent(&path);
    let mut f = File::create(&path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(
        f,
        "samples_with_negative_motor_thrust_squared_before_clamp={}",
        n_neg
    )
    .unwrap();
    writeln!(f, "total_samples={}", steps).unwrap();
    writeln!(f, "fraction_positive_motor_mix_feasible={:.6}", frac_ok).unwrap();
    writeln!(
        f,
        "note=inverse_X_mix_any_negative_omega_sq_means_T_tau_not_realisable_with_positive_rotors_only"
    )
    .unwrap();
}

fn write_motor_mixing_summary_se3(dir: &str, traj: &Se3Trajectory, params: &MultirotorParams) {
    let total = traj.total_time;
    let steps = ((total / REF_DT).ceil() as usize).saturating_add(1).max(2);
    let mut n_neg = 0usize;
    for k in 0..steps {
        let t = (k as f32 * REF_DT).min(total);
        let out = traj.eval(t);
        let tau = body_torque_diagonal(out.omega, out.alpha);
        let raw = motor_omega_sq_unclamped(out.thrust, tau, params);
        if raw.iter().any(|&x| x < -1e-8) {
            n_neg += 1;
        }
    }
    let frac_ok = 1.0 - (n_neg as f32 / steps as f32);
    let path = format!("{}/motor_mixing_summary.txt", dir);
    ensure_parent(&path);
    let mut f = File::create(&path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
    writeln!(
        f,
        "samples_with_negative_motor_thrust_squared_before_clamp={}",
        n_neg
    )
    .unwrap();
    writeln!(f, "total_samples={}", steps).unwrap();
    writeln!(f, "fraction_positive_motor_mix_feasible={:.6}", frac_ok).unwrap();
    writeln!(
        f,
        "note=uses_body_torque_from_diagonal_inertia_model_negative_total_thrust_can_invalidate_mixing"
    )
    .unwrap();
}

fn export_se3_reference_bundle(
    dir: &str,
    traj: &Se3Trajectory,
    wps: &[Se3Waypoint],
    traj_name: &str,
    position_segment_times_basis: &str,
    folder_mode: u8,
    planner_kind: &str,
    lim: FeasibilityLimits,
) {
    std::fs::create_dir_all(dir).expect("mkdir");
    write_se3_reference_csv(&format!("{}/reference.csv", dir), traj);
    write_se3_waypoints_csv(&format!("{}/waypoints.csv", dir), wps);
    write_segment_junctions_csv(
        &format!("{}/segment_junctions.csv", dir),
        &junction_times_from_durations(&traj.segment_times),
    );
    write_planning_meta_txt(dir, folder_mode, traj_name, planner_kind, false, lim);
    append_planning_meta_kv(dir, "se3_position_segment_times_basis", position_segment_times_basis);
    write_se3_feasibility_sidecar(dir, traj, lim);
    let params_cf = MultirotorParams::crazyflie();
    write_motor_mixing_summary_se3(dir, traj, &params_cf);
    println!("  wrote {} (reference + sidecars)", dir);
}

fn write_richter_feasibility_sidecar(dir: &str, planner: &TrajectoryPlanner) {
    if let Some(rep) = planner.check_feasibility(MASS, FEAS_CF2.max_thrust_n, FEAS_CF2.max_omega_rad_s) {
        let path = format!("{}/feasibility.txt", dir);
        let mut f = File::create(&path).unwrap_or_else(|e| panic!("cannot create {path}: {e}"));
        writeln!(f, "feasible={}", rep.feasible).unwrap();
        writeln!(f, "max_thrust_N={:.6}", rep.max_thrust_n).unwrap();
        writeln!(f, "max_omega_rad_s={:.6}", rep.max_omega_rad_s).unwrap();
        writeln!(f, "suggested_time_scale={:.6}", rep.suggested_time_scale).unwrap();
        writeln!(
            f,
            "\nNote: reference.csv is the unscaled Richter planner in trajectory time.\n\
             Closed-loop helix may apply suggested_time_scale when running the simulator; \
             compare feasibility here with actual tracking CSVs under closed_loop/."
        )
        .unwrap();
    }
}

fn export_planning_reference_only() {
    println!("Planner-only reference export (no simulation)");
    println!("===========================================\n");
    std::fs::create_dir_all("results/planning_sim/reference").expect("mkdir reference");

    let (circle_wps, circle_durs, _) = circle_waypoints(0.5, 8);
    let (fig8_wps, fig8_durs) = figure8_waypoints();
    // Mode 1 (Richter) uses coarse helix waypoints: Richter auto-allocates timing from k_t,
    // and very short arc-segments (n=24) yield sub-ms durations → degenerate Clarabel QP.
    // n=12 (30°/segment) keeps segment arc-length long enough for stable time allocation.
    // All modes use n=24 helix waypoints (15 deg/segment) for smooth circle top-view.
    // Richter's auto time-allocation clamps all 48 segments to T_MIN=0.15s (arc ~0.080m
    // gives T_i=0.127s < T_MIN). Fix: use plan_from_times with uniform 0.375s.
    let (helix_wps, helix_durs) = helix_waypoints(0.3, 0.4, 24, 0.375);
    let (corner_wps, corner_durs) = corner_waypoints();
    let (loop_wps, loop_durs) = loop_waypoints(0.5, 8, 0.5);
    let circle_se3_wps = to_se3_hover_waypoints(&circle_wps);
    let fig8_se3_wps = to_se3_level_waypoints(&fig8_wps);
    let helix_se3_wps = to_se3_level_waypoints(&helix_wps);
    let (corner_se3_wps, _corner_se3_durs_manual) = corner_waypoints_se3();
    let (flip_se3_wps, flip_se3_durs) = flip_waypoints_se3();
    let (loop_se3_wps, _loop_se3_durs_manual) = loop_waypoints_se3(0.5, 8, 0.5);
    let (corkscrew_wps, corkscrew_durs) = corkscrew_waypoints(0.3, 0.4, 8);
    let (corkscrew_se3_wps, _) = corkscrew_waypoints_se3(0.3, 0.4, 8);
    let (roll_se3_wps, roll_se3_durs) = roll_waypoints_se3();

    // Build Mode 1 planners first — their segment durations are reused for Mode 2 position
    // timing, avoiding 4 redundant Clarabel calls that would otherwise push the total
    // sequential QP count past Clarabel's stability limit (~10-15 calls per process).
    let p1_circle = TrajectoryPlanner::richter(&circle_wps, 0.01,  true ).expect("m1 circle");
    let p1_fig8   = TrajectoryPlanner::richter(&fig8_wps,   0.008, FIG8_PERIODIC).expect("m1 fig8");
    let p1_helix  = TrajectoryPlanner::richter_from_times(
        &helix_wps, &helix_durs, 0.008, false, 0,
    ).expect("m1 helix");
    let p1_corner = TrajectoryPlanner::richter(&corner_wps, 0.02, false).expect("m1 corner");
    let p1_loop   = TrajectoryPlanner::richter(&loop_wps,   0.01,  true ).expect("m1 loop");
    let p1_corkscrew = TrajectoryPlanner::richter(&corkscrew_wps, 0.01, false).expect("m1 corkscrew");
    let corkscrew_m2_durs = p1_corkscrew.segment_durations();

    // Mode 2 position timing: reuse Mode 1 durations for all trajectories (same waypoint count).
    // Helix: all modes share helix_durs (uniform 0.375s, 48 segments).
    let circle_m2_durs = p1_circle.segment_durations();
    let fig8_m2_durs   = p1_fig8.segment_durations();
    let helix_m2_durs  = helix_durs.clone();
    let corner_m2_durs = p1_corner.segment_durations();
    let loop_m2_durs   = p1_loop.segment_durations();

    // Mode 2 (SE(3)) — plan immediately while QP call count is still low (~5+n_m2)
    {
        let traj =
            Se3Trajectory::plan(&circle_se3_wps, &circle_m2_durs, MASS, true).expect("m2 circle");
        export_se3_reference_bundle(
            &reference_dir(2, "circle"),
            &traj,
            &circle_se3_wps,
            "circle",
            "Richter_same_as_mode1_circle",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }
    {
        let traj = Se3Trajectory::plan(&fig8_se3_wps, &fig8_m2_durs, MASS, FIG8_PERIODIC).expect("m2 fig8");
        export_se3_reference_bundle(
            &reference_dir(2, "figure8"),
            &traj,
            &fig8_se3_wps,
            "figure8",
            "Richter_same_as_mode1_figure8",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }
    {
        let traj =
            Se3Trajectory::plan(&helix_se3_wps, &helix_m2_durs, MASS, false).expect("m2 helix");
        export_se3_reference_bundle(
            &reference_dir(2, "helix"),
            &traj,
            &helix_se3_wps,
            "helix",
            "Richter_same_as_mode1_helix",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }
    {
        let traj = Se3Trajectory::plan(&corner_se3_wps, &corner_m2_durs, MASS, false)
            .expect("m2 corner");
        export_se3_reference_bundle(
            &reference_dir(2, "corner"),
            &traj,
            &corner_se3_wps,
            "corner",
            "Richter_same_as_mode1_corner_banked_se3_waypoints",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }
    {
        let traj = Se3Trajectory::plan(&flip_se3_wps, &flip_se3_durs, MASS, false).expect("m2 flip");
        export_se3_reference_bundle(
            &reference_dir(2, "flip"),
            &traj,
            &flip_se3_wps,
            "flip",
            "manual_flip_segments",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }
    {
        let traj = Se3Trajectory::plan(&loop_se3_wps, &loop_m2_durs, MASS, true).expect("m2 loop");
        export_se3_reference_bundle(
            &reference_dir(2, "loop"),
            &traj,
            &loop_se3_wps,
            "loop",
            "Richter_same_as_mode1_loop_se3_waypoints",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }
    {
        let traj = Se3Trajectory::plan(&corkscrew_se3_wps, &corkscrew_m2_durs, MASS, false)
            .expect("m2 corkscrew");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    corkscrew thrust range [{t_min:.3}, {t_max:.3}] N");
        export_se3_reference_bundle(
            &reference_dir(2, "corkscrew"),
            &traj,
            &corkscrew_se3_wps,
            "corkscrew",
            "Richter_same_as_mode1_corkscrew_x_axis_roll",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }
    {
        let traj = Se3Trajectory::plan(&roll_se3_wps, &roll_se3_durs, MASS, false)
            .expect("m2 roll");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    roll thrust range [{t_min:.3}, {t_max:.3}] N  (negative = motor cut at apex)");
        export_se3_reference_bundle(
            &reference_dir(2, "roll"),
            &traj,
            &roll_se3_wps,
            "roll",
            "manual_roll_segments_y_axis",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }
    {
        let (immelmann_se3_wps, immelmann_se3_durs) = immelmann_waypoints_se3(0.5);
        let traj = Se3Trajectory::plan(&immelmann_se3_wps, &immelmann_se3_durs, MASS, false)
            .expect("m2 immelmann");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    immelmann thrust range [{t_min:.3}, {t_max:.3}] N");
        export_se3_reference_bundle(
            &reference_dir(2, "immelmann"),
            &traj,
            &immelmann_se3_wps,
            "immelmann",
            "manual_half_loop_segments_Ry_neg_theta",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }
    {
        let (splits_se3_wps, splits_se3_durs) = splits_waypoints_se3(0.5);
        let traj = Se3Trajectory::plan(&splits_se3_wps, &splits_se3_durs, MASS, false)
            .expect("m2 splits");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    splits thrust range [{t_min:.3}, {t_max:.3}] N");
        export_se3_reference_bundle(
            &reference_dir(2, "splits"),
            &traj,
            &splits_se3_wps,
            "splits",
            "manual_roll_then_half_loop_Ry_pi_plus_psi",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }
    {
        let (screw_se3_wps, screw_se3_durs) = screw_waypoints_se3(0.5, 2.0);
        let traj = Se3Trajectory::plan(&screw_se3_wps, &screw_se3_durs, MASS, false)
            .expect("m2 screw");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    screw   thrust range [{t_min:.3}, {t_max:.3}] N");
        export_se3_reference_bundle(
            &reference_dir(2, "screw"),
            &traj,
            &screw_se3_wps,
            "screw",
            "vertical_ascent_body_x_roll_Rx_theta",
            2,
            "Se3Trajectory",
            FEAS_CF2,
        );
    }

    // Mode 1 — export using already-built planners (no extra QP calls)
    export_flat_reference_bundle(
        &reference_dir(1, "circle"),  &p1_circle, &circle_wps, "circle",  1, "RichterTrajectory", true,
    );
    export_flat_reference_bundle(
        &reference_dir(1, "figure8"), &p1_fig8,   &fig8_wps,   "figure8", 1, "RichterTrajectory", true,
    );
    export_flat_reference_bundle(
        &reference_dir(1, "helix"),   &p1_helix,  &helix_wps, "helix", 1, "RichterTrajectory", true,
    );
    export_flat_reference_bundle(
        &reference_dir(1, "corner"),  &p1_corner, &corner_wps, "corner", 1, "RichterTrajectory", true,
    );
    export_flat_reference_bundle(
        &reference_dir(1, "loop"),    &p1_loop,   &loop_wps,   "loop",    1, "RichterTrajectory", true,
    );
    export_flat_reference_bundle(
        &reference_dir(1, "corkscrew"), &p1_corkscrew, &corkscrew_wps, "corkscrew", 1, "RichterTrajectory", false,
    );

    // Mode 0 — SplineTrajectory plans (QP calls come last, after Mode 2 is safely done)
    {
        let p = TrajectoryPlanner::spline(&circle_wps, &circle_durs, true).expect("m0 circle");
        export_flat_reference_bundle(
            &reference_dir(0, "circle"), &p, &circle_wps, "circle", 0, "SplineTrajectory", false,
        );
    }
    {
        let p = TrajectoryPlanner::spline(&fig8_wps, &fig8_durs, false).expect("m0 fig8");
        export_flat_reference_bundle(
            &reference_dir(0, "figure8"), &p, &fig8_wps, "figure8", 0, "SplineTrajectory", false,
        );
    }
    {
        let p = TrajectoryPlanner::spline(&helix_wps, &helix_durs, false).expect("m0 helix");
        export_flat_reference_bundle(
            &reference_dir(0, "helix"), &p, &helix_wps, "helix", 0, "SplineTrajectory", false,
        );
    }
    {
        let p = TrajectoryPlanner::spline(&corner_wps, &corner_durs, false).expect("m0 corner");
        export_flat_reference_bundle(
            &reference_dir(0, "corner"), &p, &corner_wps, "corner", 0, "SplineTrajectory", false,
        );
    }
    {
        let p = TrajectoryPlanner::spline(&loop_wps, &loop_durs, true).expect("m0 loop");
        export_flat_reference_bundle(
            &reference_dir(0, "loop"), &p, &loop_wps, "loop", 0, "SplineTrajectory", false,
        );
    }
    {
        let p = TrajectoryPlanner::spline(&corkscrew_wps, &corkscrew_durs, false).expect("m0 corkscrew");
        export_flat_reference_bundle(
            &reference_dir(0, "corkscrew"), &p, &corkscrew_wps, "corkscrew", 0, "SplineTrajectory", false,
        );
    }

    println!("\nNext: scripts/plot_planning_all.sh");
}

fn quat_to_euler(q: &Quat) -> (f32, f32, f32) {
    let (w, x, y, z) = (q.w, q.x, q.y, q.z);
    let roll  = (2.0*(w*x + y*z)).atan2(1.0 - 2.0*(x*x + y*y));
    let pitch = (2.0*(w*y - z*x)).clamp(-1.0, 1.0).asin();
    let yaw   = (2.0*(w*z + x*y)).atan2(1.0 - 2.0*(y*y + z*z));
    (roll, pitch, yaw)
}

/// Relative rotation angle between two rotation matrices [rad].
fn rot_angle_between(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> f32 {
    let mut at_b = [[0.0_f32; 3]; 3];
    for i in 0..3 { for j in 0..3 { for k in 0..3 {
        at_b[i][j] += a[k][i] * b[k][j];
    }}}
    let tr = at_b[0][0] + at_b[1][1] + at_b[2][2];
    let c = ((tr - 1.0) * 0.5).clamp(-1.0, 1.0);
    c.acos()
}

// ---------------------------------------------------------------------------
// Closed-loop runner — Mode 0 / Mode 1  (FlatOutput → flatness → geometric)
// ---------------------------------------------------------------------------

fn run_flat(
    planner: &TrajectoryPlanner,
    params:  &MultirotorParams,
    ctrl:    &mut dyn Controller,
    yaw_ff_scale: f32,
    label:   &str,
) -> Vec<Rec> {
    let total = planner.total_time();
    let n     = (total / DT) as usize;

    let flat0 = planner.eval(0.0);
    let fr0   = compute_flatness(&flat0, params.mass);
    let q0    = rot_to_quat(&fr0.rot);

    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position         = flat0.pos;
    sim.state_mut().velocity         = flat0.vel;
    sim.state_mut().orientation      = Quat::new(q0[0], q0[1], q0[2], q0[3]);
    sim.state_mut().angular_velocity = Vec3::zero();

    let mut recs = Vec::with_capacity(n);

    for k in 0..n {
        let t    = k as f32 * DT;
        let flat = planner.eval(t);
        let fr   = compute_flatness(&flat, params.mass);
        let state = sim.state().clone();

        let reference = TrajectoryReference {
            position:         flat.pos,
            velocity:         flat.vel,
            acceleration:     flat.acc,
            jerk:             flat.jerk,
            yaw:              flat.yaw,
            yaw_rate:         flat.yaw_dot * yaw_ff_scale,
            yaw_acceleration: flat.yaw_ddot * yaw_ff_scale,
        };
        let ctrl_out = ctrl.compute_control(&state, &reference, params, DT);

        let dx = flat.pos.x - state.position.x;
        let dy = flat.pos.y - state.position.y;
        let dz = flat.pos.z - state.position.z;

        let (sim_roll, sim_pitch, sim_yaw) = quat_to_euler(&state.orientation);
        let (ref_roll, ref_pitch, ref_yaw) = rot_to_euler(&fr.rot);

        recs.push(Rec {
            t,
            ref_x: flat.pos.x, ref_y: flat.pos.y, ref_z: flat.pos.z,
            ref_roll, ref_pitch, ref_yaw,
            ref_thrust: fr.thrust,
            sim_x: state.position.x, sim_y: state.position.y, sim_z: state.position.z,
            sim_roll, sim_pitch, sim_yaw,
            cmd_thrust: ctrl_out.thrust,
            cmd_tx: ctrl_out.torque.x, cmd_ty: ctrl_out.torque.y, cmd_tz: ctrl_out.torque.z,
            error_3d: (dx*dx + dy*dy + dz*dz).sqrt(),
        });

        let action = MotorAction::from_thrust_torque(ctrl_out.thrust, ctrl_out.torque, params);
        sim.step(&action);
    }

    println!("    {label:<28} RMS 3D = {:6.1} mm", rms_3d(&recs) * 1000.0);
    recs
}

/// Time-scaled flat runner: evaluates the same planner trajectory at t/time_scale.
/// Useful when a feasibility report suggests slowing down without replanning.
fn run_flat_scaled(
    planner: &TrajectoryPlanner,
    params:  &MultirotorParams,
    ctrl:    &mut dyn Controller,
    yaw_ff_scale: f32,
    time_scale: f32,
    label:   &str,
) -> Vec<Rec> {
    let ts = time_scale.max(1.0);
    let total = planner.total_time() * ts;
    let n     = (total / DT) as usize;

    let flat0 = planner.eval(0.0);
    let fr0   = compute_flatness(&flat0, params.mass);
    let q0    = rot_to_quat(&fr0.rot);

    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position         = flat0.pos;
    sim.state_mut().velocity         = flat0.vel;
    sim.state_mut().orientation      = Quat::new(q0[0], q0[1], q0[2], q0[3]);
    sim.state_mut().angular_velocity = Vec3::zero();

    let mut recs = Vec::with_capacity(n);

    for k in 0..n {
        let t = k as f32 * DT;
        let tref = (t / ts).min(planner.total_time());
        let flat = planner.eval(tref);
        let fr   = compute_flatness(&flat, params.mass);
        let state = sim.state().clone();

        let reference = TrajectoryReference {
            position:         flat.pos,
            velocity:         flat.vel * (1.0 / ts),
            acceleration:     flat.acc * (1.0 / (ts * ts)),
            jerk:             flat.jerk * (1.0 / (ts * ts * ts)),
            yaw:              flat.yaw,
            yaw_rate:         flat.yaw_dot * (yaw_ff_scale / ts),
            yaw_acceleration: flat.yaw_ddot * (yaw_ff_scale / (ts * ts)),
        };
        let ctrl_out = ctrl.compute_control(&state, &reference, params, DT);

        let dx = flat.pos.x - state.position.x;
        let dy = flat.pos.y - state.position.y;
        let dz = flat.pos.z - state.position.z;
        let (sim_roll, sim_pitch, sim_yaw) = quat_to_euler(&state.orientation);
        let (ref_roll, ref_pitch, ref_yaw) = rot_to_euler(&fr.rot);

        recs.push(Rec {
            t,
            ref_x: flat.pos.x, ref_y: flat.pos.y, ref_z: flat.pos.z,
            ref_roll, ref_pitch, ref_yaw,
            ref_thrust: fr.thrust,
            sim_x: state.position.x, sim_y: state.position.y, sim_z: state.position.z,
            sim_roll, sim_pitch, sim_yaw,
            cmd_thrust: ctrl_out.thrust,
            cmd_tx: ctrl_out.torque.x, cmd_ty: ctrl_out.torque.y, cmd_tz: ctrl_out.torque.z,
            error_3d: (dx*dx + dy*dy + dz*dz).sqrt(),
        });

        let action = MotorAction::from_thrust_torque(ctrl_out.thrust, ctrl_out.torque, params);
        sim.step(&action);
    }

    println!("    {label:<28} RMS 3D = {:6.1} mm", rms_3d(&recs) * 1000.0);
    recs
}

// ---------------------------------------------------------------------------
// Closed-loop runner — Mode 2  (Se3Output → geometric via flat conversion)
// ---------------------------------------------------------------------------

fn run_se3(
    traj:   &Se3Trajectory,
    params: &MultirotorParams,
    ctrl:   &mut dyn Controller,
    label:  &str,
) -> Vec<Rec> {
    let total = traj.total_time;
    let n     = (total / DT) as usize;

    let out0 = traj.eval(0.0);
    let q0   = rot_to_quat(&out0.rot);

    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position         = out0.pos;
    sim.state_mut().velocity         = out0.vel;
    sim.state_mut().orientation      = Quat::new(q0[0], q0[1], q0[2], q0[3]);
    // Start with zero body rates for consistency with flat-mode runners and to
    // avoid injecting an initial angular-rate transient from numerical derivatives.
    sim.state_mut().angular_velocity = Vec3::zero();

    let mut recs = Vec::with_capacity(n);
    let mut yaw_ref_unwrapped = rot_to_euler(&out0.rot).2;

    for k in 0..n {
        let t   = k as f32 * DT;
        let out = traj.eval(t);
        let state = sim.state().clone();

        // Keep yaw reference continuous (avoid [-pi, pi] wrap jumps from rotation->yaw conversion).
        let yaw_raw = rot_to_euler(&out.rot).2;
        let mut dyaw = yaw_raw - yaw_ref_unwrapped;
        while dyaw > PI { dyaw -= 2.0 * PI; }
        while dyaw < -PI { dyaw += 2.0 * PI; }
        yaw_ref_unwrapped += dyaw;

        // Feed position/velocity/acceleration through geometric controller.
        let flat = out.to_flat();
        let reference = TrajectoryReference {
            position:         flat.pos,
            velocity:         flat.vel,
            acceleration:     flat.acc,
            jerk:             flat.jerk,
            yaw:              yaw_ref_unwrapped,
            yaw_rate:         flat.yaw_dot,
            yaw_acceleration: flat.yaw_ddot,
        };
        let ctrl_out = ctrl.compute_control(&state, &reference, params, DT);

        let dx = out.pos.x - state.position.x;
        let dy = out.pos.y - state.position.y;
        let dz = out.pos.z - state.position.z;

        let (sim_roll, sim_pitch, sim_yaw) = quat_to_euler(&state.orientation);
        let (ref_roll, ref_pitch, ref_yaw) = rot_to_euler(&out.rot);

        recs.push(Rec {
            t,
            ref_x: out.pos.x, ref_y: out.pos.y, ref_z: out.pos.z,
            ref_roll, ref_pitch, ref_yaw,
            ref_thrust: out.thrust,
            sim_x: state.position.x, sim_y: state.position.y, sim_z: state.position.z,
            sim_roll, sim_pitch, sim_yaw,
            cmd_thrust: ctrl_out.thrust,
            cmd_tx: ctrl_out.torque.x, cmd_ty: ctrl_out.torque.y, cmd_tz: ctrl_out.torque.z,
            error_3d: (dx*dx + dy*dy + dz*dz).sqrt(),
        });

        // Clamp to [0, T_max] — motor cut during inversion is physically correct
        let t_clamped = ctrl_out.thrust.max(0.0);
        let action = MotorAction::from_thrust_torque(t_clamped, ctrl_out.torque, params);
        sim.step(&action);
    }

    println!("    {label:<28} RMS 3D = {:6.1} mm", rms_3d(&recs) * 1000.0);
    recs
}

/// Like [`run_flat_scaled`]: evaluate the same SE(3) reference at `t/time_scale` (slower playback).
fn run_se3_scaled(
    traj:   &Se3Trajectory,
    params: &MultirotorParams,
    ctrl:   &mut dyn Controller,
    time_scale: f32,
    label:  &str,
) -> Vec<Rec> {
    let ts = time_scale.max(1.0);
    let total = traj.total_time * ts;
    let n     = (total / DT) as usize;

    let out0 = traj.eval(0.0);
    let q0   = rot_to_quat(&out0.rot);

    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position         = out0.pos;
    sim.state_mut().velocity         = out0.vel;
    sim.state_mut().orientation      = Quat::new(q0[0], q0[1], q0[2], q0[3]);
    sim.state_mut().angular_velocity = Vec3::zero();

    let mut recs = Vec::with_capacity(n);
    let mut yaw_ref_unwrapped = rot_to_euler(&out0.rot).2;

    for k in 0..n {
        let t    = k as f32 * DT;
        let tref = (t / ts).min(traj.total_time);
        let out  = traj.eval(tref);
        let state = sim.state().clone();

        let yaw_raw = rot_to_euler(&out.rot).2;
        let mut dyaw = yaw_raw - yaw_ref_unwrapped;
        while dyaw > PI { dyaw -= 2.0 * PI; }
        while dyaw < -PI { dyaw += 2.0 * PI; }
        yaw_ref_unwrapped += dyaw;

        let flat = out.to_flat();
        let inv = 1.0 / ts;
        let inv2 = inv * inv;
        let reference = TrajectoryReference {
            position:         flat.pos,
            velocity:         flat.vel * inv,
            acceleration:     flat.acc * inv2,
            jerk:             Vec3::zero(),
            yaw:              yaw_ref_unwrapped,
            yaw_rate:         flat.yaw_dot * inv,
            yaw_acceleration: flat.yaw_ddot * inv2,
        };
        let ctrl_out = ctrl.compute_control(&state, &reference, params, DT);

        let dx = out.pos.x - state.position.x;
        let dy = out.pos.y - state.position.y;
        let dz = out.pos.z - state.position.z;

        let (sim_roll, sim_pitch, sim_yaw) = quat_to_euler(&state.orientation);
        let (ref_roll, ref_pitch, ref_yaw) = rot_to_euler(&out.rot);

        recs.push(Rec {
            t,
            ref_x: out.pos.x, ref_y: out.pos.y, ref_z: out.pos.z,
            ref_roll, ref_pitch, ref_yaw,
            ref_thrust: out.thrust,
            sim_x: state.position.x, sim_y: state.position.y, sim_z: state.position.z,
            sim_roll, sim_pitch, sim_yaw,
            cmd_thrust: ctrl_out.thrust,
            cmd_tx: ctrl_out.torque.x, cmd_ty: ctrl_out.torque.y, cmd_tz: ctrl_out.torque.z,
            error_3d: (dx*dx + dy*dy + dz*dz).sqrt(),
        });

        let t_clamped = ctrl_out.thrust.max(0.0);
        let action = MotorAction::from_thrust_torque(t_clamped, ctrl_out.torque, params);
        sim.step(&action);
    }

    println!("    {label:<28} RMS 3D = {:6.1} mm", rms_3d(&recs) * 1000.0);
    recs
}

// ---------------------------------------------------------------------------
// INDI runners (with angular-acceleration feedforward from flatness)
// ---------------------------------------------------------------------------

fn run_flat_indi(
    planner: &TrajectoryPlanner,
    params:  &MultirotorParams,
    yaw_ff_scale: f32,
    label:   &str,
) -> Vec<Rec> {
    let total = planner.total_time();
    let n     = (total / DT) as usize;

    let flat0 = planner.eval(0.0);
    let fr0   = compute_flatness(&flat0, params.mass);
    let q0    = rot_to_quat(&fr0.rot);

    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position         = flat0.pos;
    sim.state_mut().velocity         = flat0.vel;
    sim.state_mut().orientation      = Quat::new(q0[0], q0[1], q0[2], q0[3]);
    sim.state_mut().angular_velocity = fr0.omega; // start with reference angular velocity

    let mut indi = IndiSimState::new();
    let mut recs = Vec::with_capacity(n);

    for k in 0..n {
        let t    = k as f32 * DT;
        let flat = planner.eval(t);
        let fr   = compute_flatness(&flat, params.mass);
        let state = sim.state().clone();

        let reference = TrajectoryReference {
            position:         flat.pos,
            velocity:         flat.vel,
            acceleration:     flat.acc,
            jerk:             flat.jerk,
            yaw:              flat.yaw,
            yaw_rate:         flat.yaw_dot * yaw_ff_scale,
            yaw_acceleration: flat.yaw_ddot * yaw_ff_scale,
        };

    // Slightly stronger vertical outer-loop for INDI to compensate incremental inner-loop lag
    // that otherwise appears as persistent z bias on curvy trajectories (fig8/helix).
    let (thrust, rd) = outer_loop_with_gains(
        &state, &reference, params,
        Vec3::new(12.0, 12.0, 8.5),
        Vec3::new(8.0, 8.0, 5.0),
    );
        // Adaptive feedforward: only trust flatness angular feedforward when rd and fr.rot
        // are close; otherwise fade to zero to avoid frame/consistency mismatch transients.
        let ff_mismatch = rot_angle_between(&rd, &fr.rot);
        let ff_gain = 0.8 * ((0.45 - ff_mismatch) / (0.45 - 0.15)).clamp(0.0, 1.0);
        let omega_ff = fr.omega * ff_gain;
        let alpha_ff = fr.omega_dot * ff_gain;
        let rot_ff = if ff_gain > 0.0 { &fr.rot } else { &rd };
        let torque = indi.compute(&state, &rd, rot_ff, omega_ff, alpha_ff, thrust, params);

        let dx = flat.pos.x - state.position.x;
        let dy = flat.pos.y - state.position.y;
        let dz = flat.pos.z - state.position.z;
        let (sim_roll, sim_pitch, sim_yaw) = quat_to_euler(&state.orientation);
        let (ref_roll, ref_pitch, ref_yaw) = rot_to_euler(&fr.rot);

        recs.push(Rec {
            t,
            ref_x: flat.pos.x, ref_y: flat.pos.y, ref_z: flat.pos.z,
            ref_roll, ref_pitch, ref_yaw,
            ref_thrust: fr.thrust,
            sim_x: state.position.x, sim_y: state.position.y, sim_z: state.position.z,
            sim_roll, sim_pitch, sim_yaw,
            cmd_thrust: thrust,
            cmd_tx: torque.x, cmd_ty: torque.y, cmd_tz: torque.z,
            error_3d: (dx*dx + dy*dy + dz*dz).sqrt(),
        });

        let action = MotorAction::from_thrust_torque(thrust, torque, params);
        sim.step(&action);
    }

    println!("    {label:<28} RMS 3D = {:6.1} mm", rms_3d(&recs) * 1000.0);
    recs
}

fn run_flat_indi_scaled(
    planner: &TrajectoryPlanner,
    params:  &MultirotorParams,
    yaw_ff_scale: f32,
    time_scale: f32,
    label:   &str,
) -> Vec<Rec> {
    let ts = time_scale.max(1.0);
    let total = planner.total_time() * ts;
    let n     = (total / DT) as usize;

    let flat0 = planner.eval(0.0);
    let fr0   = compute_flatness(&flat0, params.mass);
    let q0    = rot_to_quat(&fr0.rot);

    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position         = flat0.pos;
    sim.state_mut().velocity         = flat0.vel;
    sim.state_mut().orientation      = Quat::new(q0[0], q0[1], q0[2], q0[3]);
    sim.state_mut().angular_velocity = fr0.omega;

    let mut indi = IndiSimState::new();
    let mut recs = Vec::with_capacity(n);

    for k in 0..n {
        let t = k as f32 * DT;
        let tref = (t / ts).min(planner.total_time());
        let flat = planner.eval(tref);
        let fr   = compute_flatness(&flat, params.mass);
        let state = sim.state().clone();

        let reference = TrajectoryReference {
            position:         flat.pos,
            velocity:         flat.vel * (1.0 / ts),
            acceleration:     flat.acc * (1.0 / (ts * ts)),
            jerk:             flat.jerk * (1.0 / (ts * ts * ts)),
            yaw:              flat.yaw,
            yaw_rate:         flat.yaw_dot * (yaw_ff_scale / ts),
            yaw_acceleration: flat.yaw_ddot * (yaw_ff_scale / (ts * ts)),
        };

        let (thrust, rd) = outer_loop_with_gains(
            &state, &reference, params,
            Vec3::new(12.0, 12.0, 8.5),
            Vec3::new(8.0, 8.0, 5.0),
        );
        let ff_mismatch = rot_angle_between(&rd, &fr.rot);
        let ff_gain = 0.8 * ((0.45 - ff_mismatch) / (0.45 - 0.15)).clamp(0.0, 1.0);
        let omega_ff = fr.omega * ff_gain;
        let alpha_ff = fr.omega_dot * ff_gain;
        let rot_ff = if ff_gain > 0.0 { &fr.rot } else { &rd };
        let torque = indi.compute(&state, &rd, rot_ff, omega_ff, alpha_ff, thrust, params);

        let dx = flat.pos.x - state.position.x;
        let dy = flat.pos.y - state.position.y;
        let dz = flat.pos.z - state.position.z;
        let (sim_roll, sim_pitch, sim_yaw) = quat_to_euler(&state.orientation);
        let (ref_roll, ref_pitch, ref_yaw) = rot_to_euler(&fr.rot);

        recs.push(Rec {
            t,
            ref_x: flat.pos.x, ref_y: flat.pos.y, ref_z: flat.pos.z,
            ref_roll, ref_pitch, ref_yaw,
            ref_thrust: fr.thrust,
            sim_x: state.position.x, sim_y: state.position.y, sim_z: state.position.z,
            sim_roll, sim_pitch, sim_yaw,
            cmd_thrust: thrust,
            cmd_tx: torque.x, cmd_ty: torque.y, cmd_tz: torque.z,
            error_3d: (dx*dx + dy*dy + dz*dz).sqrt(),
        });

        let action = MotorAction::from_thrust_torque(thrust, torque, params);
        sim.step(&action);
    }

    println!("    {label:<28} RMS 3D = {:6.1} mm", rms_3d(&recs) * 1000.0);
    recs
}

fn run_se3_indi(
    traj:   &Se3Trajectory,
    params: &MultirotorParams,
    label:  &str,
) -> Vec<Rec> {
    let total = traj.total_time;
    let n     = (total / DT) as usize;

    let out0 = traj.eval(0.0);
    let q0   = rot_to_quat(&out0.rot);

    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position         = out0.pos;
    sim.state_mut().velocity         = out0.vel;
    sim.state_mut().orientation      = Quat::new(q0[0], q0[1], q0[2], q0[3]);
    sim.state_mut().angular_velocity = out0.omega; // seed from flatness, same as run_flat_indi

    let mut indi = IndiSimState::new();
    let mut recs = Vec::with_capacity(n);
    let mut yaw_ref_unwrapped = rot_to_euler(&out0.rot).2;

    for k in 0..n {
        let t   = k as f32 * DT;
        let out = traj.eval(t);
        let state = sim.state().clone();

        // Keep yaw reference continuous (avoid [-pi, pi] wrap jumps).
        let yaw_raw = rot_to_euler(&out.rot).2;
        let mut dyaw = yaw_raw - yaw_ref_unwrapped;
        while dyaw > PI { dyaw -= 2.0 * PI; }
        while dyaw < -PI { dyaw += 2.0 * PI; }
        yaw_ref_unwrapped += dyaw;

        let flat = out.to_flat();
        let reference = TrajectoryReference {
            position:         flat.pos,
            velocity:         flat.vel,
            acceleration:     flat.acc,
            jerk:             Vec3::zero(),
            yaw:              yaw_ref_unwrapped,
            yaw_rate:         0.0,
            yaw_acceleration: 0.0,
        };
        let (thrust, rd) = outer_loop_with_gains(
            &state, &reference, params,
            Vec3::new(12.0, 12.0, 8.5),
            Vec3::new(8.0, 8.0, 5.0),
        );
        // Adaptive feedforward: fade omega/alpha when rd and out.rot diverge.
        let ff_mismatch = rot_angle_between(&rd, &out.rot);
        let ff_gain = 0.8 * ((0.45 - ff_mismatch) / (0.45 - 0.15)).clamp(0.0, 1.0);
        let omega_ff = out.omega * ff_gain;
        let alpha_ff = out.alpha * ff_gain;
        let rot_ff = if ff_gain > 0.0 { &out.rot } else { &rd };
        let torque = indi.compute(&state, &rd, rot_ff, omega_ff, alpha_ff, thrust, params);

        let dx = out.pos.x - state.position.x;
        let dy = out.pos.y - state.position.y;
        let dz = out.pos.z - state.position.z;
        let (sim_roll, sim_pitch, sim_yaw) = quat_to_euler(&state.orientation);
        let (ref_roll, ref_pitch, ref_yaw) = rot_to_euler(&out.rot);

        recs.push(Rec {
            t,
            ref_x: out.pos.x, ref_y: out.pos.y, ref_z: out.pos.z,
            ref_roll, ref_pitch, ref_yaw,
            ref_thrust: out.thrust,
            sim_x: state.position.x, sim_y: state.position.y, sim_z: state.position.z,
            sim_roll, sim_pitch, sim_yaw,
            cmd_thrust: thrust,
            cmd_tx: torque.x, cmd_ty: torque.y, cmd_tz: torque.z,
            error_3d: (dx*dx + dy*dy + dz*dz).sqrt(),
        });

        let t_clamped = thrust.max(0.0);
        let action = MotorAction::from_thrust_torque(t_clamped, torque, params);
        sim.step(&action);
    }

    println!("    {label:<28} RMS 3D = {:6.1} mm", rms_3d(&recs) * 1000.0);
    recs
}

/// [`run_se3_scaled`] counterpart for INDI (matches Mode 1 helix time-stretch).
fn run_se3_indi_scaled(
    traj:   &Se3Trajectory,
    params: &MultirotorParams,
    time_scale: f32,
    label:  &str,
) -> Vec<Rec> {
    let ts = time_scale.max(1.0);
    let total = traj.total_time * ts;
    let n     = (total / DT) as usize;

    let out0 = traj.eval(0.0);
    let q0   = rot_to_quat(&out0.rot);

    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position         = out0.pos;
    sim.state_mut().velocity         = out0.vel;
    sim.state_mut().orientation      = Quat::new(q0[0], q0[1], q0[2], q0[3]);
    sim.state_mut().angular_velocity = out0.omega; // seed from flatness

    let mut indi = IndiSimState::new();
    let mut recs = Vec::with_capacity(n);
    let mut yaw_ref_unwrapped = rot_to_euler(&out0.rot).2;

    for k in 0..n {
        let t    = k as f32 * DT;
        let tref = (t / ts).min(traj.total_time);
        let out  = traj.eval(tref);
        let state = sim.state().clone();

        let yaw_raw = rot_to_euler(&out.rot).2;
        let mut dyaw = yaw_raw - yaw_ref_unwrapped;
        while dyaw > PI { dyaw -= 2.0 * PI; }
        while dyaw < -PI { dyaw += 2.0 * PI; }
        yaw_ref_unwrapped += dyaw;

        let flat = out.to_flat();
        let inv = 1.0 / ts;
        let inv2 = inv * inv;
        let reference = TrajectoryReference {
            position:         flat.pos,
            velocity:         flat.vel * inv,
            acceleration:     flat.acc * inv2,
            jerk:             Vec3::zero(),
            yaw:                yaw_ref_unwrapped,
            yaw_rate:           0.0,
            yaw_acceleration:   0.0,
        };
        let (thrust, rd) = outer_loop_with_gains(
            &state, &reference, params,
            Vec3::new(12.0, 12.0, 8.5),
            Vec3::new(8.0, 8.0, 5.0),
        );
        let ff_mismatch = rot_angle_between(&rd, &out.rot);
        let ff_gain = 0.8 * ((0.45 - ff_mismatch) / (0.45 - 0.15)).clamp(0.0, 1.0);
        let omega_ff = out.omega * (ff_gain * inv);
        let alpha_ff = out.alpha * (ff_gain * inv2);
        let rot_ff = if ff_gain > 0.0 { &out.rot } else { &rd };
        let torque = indi.compute(&state, &rd, rot_ff, omega_ff, alpha_ff, thrust, params);

        let dx = out.pos.x - state.position.x;
        let dy = out.pos.y - state.position.y;
        let dz = out.pos.z - state.position.z;
        let (sim_roll, sim_pitch, sim_yaw) = quat_to_euler(&state.orientation);
        let (ref_roll, ref_pitch, ref_yaw) = rot_to_euler(&out.rot);

        recs.push(Rec {
            t,
            ref_x: out.pos.x, ref_y: out.pos.y, ref_z: out.pos.z,
            ref_roll, ref_pitch, ref_yaw,
            ref_thrust: out.thrust,
            sim_x: state.position.x, sim_y: state.position.y, sim_z: state.position.z,
            sim_roll, sim_pitch, sim_yaw,
            cmd_thrust: thrust,
            cmd_tx: torque.x, cmd_ty: torque.y, cmd_tz: torque.z,
            error_3d: (dx*dx + dy*dy + dz*dz).sqrt(),
        });

        let t_clamped = thrust.max(0.0);
        let action = MotorAction::from_thrust_torque(t_clamped, torque, params);
        sim.step(&action);
    }

    println!("    {label:<28} RMS 3D = {:6.1} mm", rms_3d(&recs) * 1000.0);
    recs
}

// ---------------------------------------------------------------------------
// Trajectory waypoint builders
// ---------------------------------------------------------------------------

/// Circle: N evenly-spaced waypoints on a horizontal circle (closed loop)
fn circle_waypoints(r: f32, n: usize) -> (Vec<Waypoint>, Vec<f32>, f32) {
    let seg_t = 1.0_f32;   // 1 s per segment
    let wps: Vec<Waypoint> = (0..=n).map(|i| {
        let theta = 2.0 * PI * i as f32 / n as f32;
        Waypoint { pos: Vec3::new(r * theta.cos(), r * theta.sin(), Z), yaw: theta + PI/2.0 }
    }).collect();
    let durs = vec![seg_t; n];
    (wps, durs, seg_t * n as f32)
}

/// Figure-8: **same geometry and segment times** as `onboard_figure8.rs` /
/// `spline_figure8_test.rs` (Poly4D / professor layout, ~±0.92 m × ±0.45 m in X–Y).
///
/// Z is set to hover height `Z` (onboard uses relative z=0 and adds `HOVER_HEIGHT` in firmware).
///
/// All callers use **`periodic = false`** (rest-to-rest) to match the professor's waypoint design
/// and the actual onboard flight behaviour. The drone decelerates to zero at the origin crossing
/// between reps; this avoids the rep-boundary jerk that `periodic = true` causes when tracking
/// error prevents a perfectly smooth velocity wrap-around.
fn figure8_waypoints() -> (Vec<Waypoint>, Vec<f32>) {
    let wps = vec![
        Waypoint { pos: Vec3::new(0.000000, 0.000000, Z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.396058, -0.445604, Z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.922409, -0.291165, Z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.923174, 0.289869, Z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.405364, 0.450742, Z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.000000, 0.000000, Z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.402804, -0.449354, Z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.921641, -0.292459, Z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.923935, 0.288570, Z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.398611, 0.447039, Z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.000000, 0.000000, Z), yaw: 0.0 },
    ];
    let durs = vec![
        1.050000_f32, 0.710000, 0.620000, 0.700000, 0.560000, 0.560000, 0.700000, 0.620000,
        0.710000, 1.050000,
    ];
    (wps, durs)
}

/// Helix: ascending then descending spiral.
///
/// Uses `n` segments per lap (30° each for n=12) for a smooth circular cross-section.
/// The path is open (first ≠ last position due to z-change), so `periodic=false` is correct.
fn helix_waypoints(r: f32, dz: f32, n: usize, seg_t: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let mut wps = Vec::new();
    let mut durs = Vec::new();
    // Ascending lap (0 → 2π)
    for i in 0..=n {
        let theta = 2.0 * PI * i as f32 / n as f32;
        let z = Z + dz * i as f32 / n as f32;
        wps.push(Waypoint { pos: Vec3::new(r * theta.cos(), r * theta.sin(), z), yaw: 0.0 });
        if i < n { durs.push(seg_t); }
    }
    // Descending lap (2π → 4π) — same rotation direction, decreasing z
    for i in 1..=n {
        let theta = 2.0 * PI * (1.0 + i as f32 / n as f32);
        let z = Z + dz - dz * i as f32 / n as f32;
        wps.push(Waypoint { pos: Vec3::new(r * theta.cos(), r * theta.sin(), z), yaw: 0.0 });
        durs.push(seg_t);
    }
    (wps, durs)
}

/// Racing-style corner (open path, XY plane) with manually-shaped durations.
///
/// Geometry: straight entry -> curved apex -> straight exit.
/// This is intentionally non-periodic and keeps z constant at hover height.
fn corner_waypoints() -> (Vec<Waypoint>, Vec<f32>) {
    let z = Z;
    let wps = vec![
        Waypoint { pos: Vec3::new(-0.9, -0.2, z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.5, -0.2, z), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.15, -0.10, z), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.05,  0.10, z), yaw: 0.0 }, // apex neighborhood
        Waypoint { pos: Vec3::new( 0.18,  0.42, z), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.22,  0.78, z), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.22,  1.10, z), yaw: 0.0 },
    ];
    // Slightly slower at entry/exit, faster around apex.
    let durs = vec![0.55, 0.40, 0.34, 0.34, 0.42, 0.55];
    (wps, durs)
}

/// Vertical loop in the X-Z plane: N waypoints on a circle in X-Z
fn loop_waypoints(r: f32, n: usize, seg_t: f32) -> (Vec<Waypoint>, Vec<f32>) {
    // Circle parametrised in X-Z: x = r·sin(θ), z = Z + r·(1 - cos(θ))
    // At θ=0: bottom (hover height). At θ=π: top (inverted).
    let wps: Vec<Waypoint> = (0..=n).map(|i| {
        let theta = 2.0 * PI * i as f32 / n as f32;
        Waypoint {
            pos: Vec3::new(r * theta.sin(), 0.0, Z + r * (1.0 - theta.cos())),
            yaw: 0.0,
        }
    }).collect();
    let durs = vec![seg_t; n];
    (wps, durs)
}

/// Flip waypoints for Mode 2: pure hover flip (position fixed, attitude rotates 360°).
///
/// The drone stays at (0, 0, Z) throughout; only pitch changes: 0 → π → 0.
/// This is "Option A": the reference is a single point in 3-D space, and the
/// entire maneuver is an attitude maneuver. The simulation will show position
/// excursion due to motor cut at the inverted apex (thrust clamped to 0).
fn flip_waypoints_se3() -> (Vec<Se3Waypoint>, Vec<f32>) {
    let pos = Vec3::new(0.0, 0.0, Z);
    let wps = vec![
        Se3Waypoint::levelled(pos),          // start: level at hover height
        Se3Waypoint::pitched(pos, PI),       // apex: inverted, same position
        Se3Waypoint::levelled(pos),          // end: level, same position
    ];
    (wps, vec![0.4, 0.4])  // 0.8 s total
}

/// Convert flat waypoints to explicit SE(3) level attitude waypoints.
///
/// Mode 2 policy: attitude is authored explicitly (quaternion path),
/// not derived from differential flatness.
fn to_se3_level_waypoints(wps: &[Waypoint]) -> Vec<Se3Waypoint> {
    wps.iter().map(|w| Se3Waypoint::levelled(w.pos)).collect()
}

/// Convert waypoints to explicit SE(3) level attitude waypoints.
/// Identical to `to_se3_level_waypoints`; kept as a separate name for call-site clarity.
fn to_se3_hover_waypoints(wps: &[Waypoint]) -> Vec<Se3Waypoint> {
    wps.iter().map(|w| Se3Waypoint::levelled(w.pos)).collect()
}

/// Vertical loop for Mode 2: explicit attitude through full inversion.
///
/// Uses n=8 waypoints (45° each), matching onboard_loop.rs.
/// Attitude = Ry(-θ): body-z tracks the centripetal direction throughout.
/// q(θ) = [cos(θ/2), 0, −sin(θ/2), 0] — at bottom upright, at top inverted.
fn loop_waypoints_se3(r: f32, n: usize, seg_t: f32) -> (Vec<Se3Waypoint>, Vec<f32>) {
    let wps: Vec<Se3Waypoint> = (0..=n).map(|i| {
        let theta = 2.0 * PI * i as f32 / n as f32;
        let pos = Vec3::new(r * theta.sin(), 0.0, Z + r * (1.0 - theta.cos()));
        let att = Quat::new((theta * 0.5).cos(), 0.0, -(theta * 0.5).sin(), 0.0);
        Se3Waypoint::new(pos, att)
    }).collect();
    (wps, vec![seg_t; n])
}

/// Banked-corner Se3 waypoints with explicit roll profile.
///
/// Same position as `corner_waypoints()`, but attitude includes pre-bank, apex bank,
/// and unload phases to emulate racing-style cornering.
fn corner_waypoints_se3() -> (Vec<Se3Waypoint>, Vec<f32>) {
    let (flat_wps, durs) = corner_waypoints();
    // Roll schedule [rad], aligned with corner progression.
    let bank = [0.0_f32, 0.18, 0.38, 0.52, 0.36, 0.16, 0.0];
    let wps: Vec<Se3Waypoint> = flat_wps
        .iter()
        .zip(bank.iter())
        .map(|(w, &phi)| {
            let q = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), phi);
            Se3Waypoint::new(w.pos, q)
        })
        .collect();
    (wps, durs)
}

/// Corkscrew helix: one ascending lap with n=8 waypoints for Modes 0 and 1 (flatness attitude).
fn corkscrew_waypoints(r: f32, dz: f32, n: usize) -> (Vec<Waypoint>, Vec<f32>) {
    // seg_t: uniform segment duration at omega=0.6 rad/s
    let seg_t = 2.0 * PI / (n as f32 * 0.6_f32);
    let wps: Vec<Waypoint> = (0..=n).map(|i| {
        let frac = i as f32 / n as f32;
        let theta = 2.0 * PI * frac;
        Waypoint {
            pos: Vec3::new(r * theta.cos(), r * theta.sin(), Z + dz * frac),
            yaw: 0.0,
        }
    }).collect();
    (wps, vec![seg_t; n])
}

/// Corkscrew helix Mode 2: same geometry + explicit roll schedule (one 360° roll per lap).
///
/// Roll quaternion at waypoint k: `Rx(2π · k/n)` — body rolls around x-axis continuously.
/// At k=n/2 (halfway) the drone is inverted → thrust crosses zero → motor cut in simulation.
fn corkscrew_waypoints_se3(r: f32, dz: f32, n: usize) -> (Vec<Se3Waypoint>, Vec<f32>) {
    let seg_t = 2.0 * PI / (n as f32 * 0.6_f32);
    let wps: Vec<Se3Waypoint> = (0..=n).map(|i| {
        let frac = i as f32 / n as f32;
        let theta = 2.0 * PI * frac;
        let pos = Vec3::new(r * theta.cos(), r * theta.sin(), Z + dz * frac);
        let roll_angle = 2.0 * PI * frac;  // 0 at start, 2π at end (identity again)
        let q = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll_angle);
        Se3Waypoint::new(pos, q)
    }).collect();
    (wps, vec![seg_t; n])
}

/// Stationary roll (y-axis inversion, pitch maneuver, distinct from flip's x-axis).
///
/// Position held constant; attitude rotates around y-axis: level → inverted → level.
/// This is aerobatically a "backflip" (nose-over-tail), distinct from `flip` (aileron roll around x).
/// Zero thrust at 180° apex → motor cut in simulation.
fn roll_waypoints_se3() -> (Vec<Se3Waypoint>, Vec<f32>) {
    let pos = Vec3::new(0.0, 0.0, Z);
    let wps = vec![
        Se3Waypoint::levelled(pos),
        Se3Waypoint::rolled(pos, PI),    // y-axis: pitch nose down → inverted
        Se3Waypoint::levelled(pos),
    ];
    (wps, vec![0.4, 0.4])
}

/// Immelmann turn: ascending half-loop (θ: 0→π) + stationary roll recovery at apex.
///
/// Phase 1 (4 segs): half vertical loop in XZ.  q = Ry(-θ) — body-z centripetal toward center.
///   θ=0: upright at (0,0,Z).  θ=π: inverted at (0,0,Z+2R).
/// Phase 2 (1 seg): stationary at apex — Ry(-π) → Rz(π) = upright, heading reversed.
/// Zero thrust at inversion → motor cut, same as loop Mode 2.
fn immelmann_waypoints_se3(r: f32) -> (Vec<Se3Waypoint>, Vec<f32>) {
    let n     = 4usize;
    let t_seg = (PI * r / 1.05_f32) / n as f32; // half circumference at ~1.05 m/s per segment
    let t_roll = 0.35_f32;

    let mut wps  = Vec::new();
    let mut durs = Vec::new();

    // Phase 1: ascending half-loop
    for i in 0..=n {
        let theta = PI * i as f32 / n as f32;
        let pos = Vec3::new(r * theta.sin(), 0.0, Z + r * (1.0 - theta.cos()));
        // Ry(-θ): body-z = (-sin θ, 0, cos θ) = centripetal toward loop center
        let q = Quat::new((theta * 0.5).cos(), 0.0, -(theta * 0.5).sin(), 0.0);
        wps.push(Se3Waypoint::new(pos, q));
        if i < n { durs.push(t_seg); }
    }

    // Phase 2: roll recovery at apex — Ry(-π) → Rz(π) [0,0,0,1]
    let apex = Vec3::new(0.0, 0.0, Z + 2.0 * r);
    wps.push(Se3Waypoint::new(apex, Quat::new(0.0, 0.0, 0.0, 1.0)));
    durs.push(t_roll);

    (wps, durs)
}

/// Split-S: stationary half-roll to inverted + descending half-loop.
///
/// Starts at elevated position (Z + 2R) — sim initialises drone there automatically.
/// Phase 1 (1 seg): stationary at (0,0,Z+2R) — identity → Ry(π) = inverted.
/// Phase 2 (4 segs): descending half-loop. q = Ry(π+ψ).
///   ψ=0: inverted at top (0,0,Z+2R).  ψ=π: upright at (0,0,Z).
/// Zero thrust at inversion → motor cut.
fn splits_waypoints_se3(r: f32) -> (Vec<Se3Waypoint>, Vec<f32>) {
    let n      = 4usize;
    let t_seg  = (PI * r / 1.05_f32) / n as f32;
    let t_roll = 0.35_f32;

    let start_z = Z + 2.0 * r;
    let start   = Vec3::new(0.0, 0.0, start_z);

    // Phase 1: roll to inverted (identity → Ry(π) = [0,0,1,0])
    let mut wps = vec![
        Se3Waypoint::levelled(start),
        Se3Waypoint::new(start, Quat::new(0.0, 0.0, 1.0, 0.0)), // Ry(π)
    ];
    let mut durs = vec![t_roll];

    // Phase 2: descending half-loop (ψ: 0→π)
    // Loop center at (0,0,Z+R). Position: (R·sin ψ, 0, Z+R + R·cos ψ) = (R·sin ψ, 0, start_z − R + R·cos ψ).
    // Quaternion Ry(π+ψ): w=cos((π+ψ)/2), y=sin((π+ψ)/2) — w: 0→-1, continuous through inversion.
    // ψ=0 position already at start (Phase 1 end), so add ψ = π·i/n for i=1..=n.
    for i in 1..=n {
        let psi = PI * i as f32 / n as f32;
        let pos = Vec3::new(r * psi.sin(), 0.0, start_z - r + r * psi.cos());
        let half = (PI + psi) * 0.5;
        let q = Quat::new(half.cos(), 0.0, half.sin(), 0.0);
        wps.push(Se3Waypoint::new(pos, q));
        durs.push(t_seg);
    }

    (wps, durs)
}

/// Screw ascent: stationary XY, linear Z ascent, continuous yaw rotation (0 → 2π).
///
/// n=4 segments × 90° yaw per segment. Z rises from Z to Z+dz. XY stays at (0,0).
/// Quaternion Rz(θ) = [cos(θ/2), 0, 0, sin(θ/2)] — body-z ALWAYS points up.
/// Thrust is constant ≈ mg throughout — no inversion, no motor cut.
/// Distinct from corkscrew (circular XY helix + body roll): path is a vertical line.
fn screw_waypoints_se3(dz: f32, t_screw: f32) -> (Vec<Se3Waypoint>, Vec<f32>) {
    let n = 4usize;
    let seg_t = t_screw / n as f32;
    let wps: Vec<Se3Waypoint> = (0..=n).map(|i| {
        let frac = i as f32 / n as f32;
        let theta = 2.0 * PI * frac;
        let q = Quat::new((theta * 0.5).cos(), 0.0, 0.0, (theta * 0.5).sin()); // Rz(θ)
        Se3Waypoint::new(Vec3::new(0.0, 0.0, Z + dz * frac), q)
    }).collect();
    (wps, vec![seg_t; n])
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|a| a == "--export-reference-only") {
        export_planning_reference_only();
        return;
    }

    println!("Planning Mode Simulation");
    println!("========================\n");
    println!("Modes:       0=Spline(manual durations)  1=Richter(auto k_t)  2=SE3(explicit attitude)");
    println!("Trajectories: circle  figure8  helix  corner  loop  flip  corkscrew  roll  immelmann  splits  screw\n");

    std::fs::create_dir_all("results/planning_sim").expect("cannot create output dir");

    let mut params = MultirotorParams::crazyflie();
    params.dt = DT;

    // ── Build waypoints ──────────────────────────────────────────────────────
    let (circle_wps,   circle_durs,  _) = circle_waypoints(0.5, 8);
    let (fig8_wps,     fig8_durs)       = figure8_waypoints();
    // All modes use n=24 helix (15 deg/segment). plan_from_times bypasses Richter's
    // auto-allocation (which would clamp 48 short-arc segments to T_MIN=0.15s).
    let (helix_wps,    helix_durs)      = helix_waypoints(0.3, 0.4, 24, 0.375); // n=24 = 15 deg/seg
    let (corner_wps,   corner_durs)     = corner_waypoints();
    let (loop_wps,     loop_durs)       = loop_waypoints(0.5, 8, 0.5);
    let circle_se3_wps = to_se3_hover_waypoints(&circle_wps);
    let fig8_se3_wps   = to_se3_level_waypoints(&fig8_wps);
    let helix_se3_wps  = to_se3_level_waypoints(&helix_wps);
    let (corner_se3_wps, _corner_se3_durs_manual) = corner_waypoints_se3();
    let (flip_se3_wps, flip_se3_durs) = flip_waypoints_se3();
    let (loop_se3_wps, _loop_se3_durs_manual) = loop_waypoints_se3(0.5, 8, 0.5); // n=8 → 45°/seg
    let (corkscrew_wps,    corkscrew_durs)    = corkscrew_waypoints(0.3, 0.4, 8);
    let (corkscrew_se3_wps, _)               = corkscrew_waypoints_se3(0.3, 0.4, 8);
    let (roll_se3_wps,    roll_se3_durs)     = roll_waypoints_se3();
    let (immelmann_se3_wps, immelmann_se3_durs) = immelmann_waypoints_se3(0.5);
    let (splits_se3_wps,    splits_se3_durs)    = splits_waypoints_se3(0.5);
    let (screw_se3_wps,     screw_se3_durs)     = screw_waypoints_se3(0.5, 2.0);

    // Build Mode 1 planners first — reuse for Mode 2 duration extraction (no redundant QP calls).
    // k_t = 0.01: v_avg = 0.5 + 1.5·√0.01 = 0.65 m/s
    //   → circle 0.59 s/seg, helix 0.36 s/seg, loop 0.59 s/seg.
    // 1/T^7 conditioning: T=0.36 s → 3.5e3 (ok); T=0.15 s (T_MIN) → 2.2e7 (bad).
    // k_t=0.1 → T_helix≈0.24 s → 1/T^7≈2.2e4 → Clarabel diverges.
    //
    // CF 2.1 limits used for feasibility check:
    const MAX_THRUST: f32 = 0.55;  // N (total, including decks)
    const MAX_OMEGA:  f32 = 12.0;  // rad/s
    let p1_circle = TrajectoryPlanner::richter(&circle_wps, 0.01,  true ).expect("m1 circle");
    let p1_fig8   = TrajectoryPlanner::richter(&fig8_wps,   0.008, FIG8_PERIODIC).expect("m1 fig8");
    let p1_helix  = TrajectoryPlanner::richter_from_times(
        &helix_wps, &helix_durs, 0.008, false, 0,
    ).expect("m1 helix");
    let p1_corner = TrajectoryPlanner::richter(&corner_wps, 0.02, false).expect("m1 corner");
    let p1_loop   = TrajectoryPlanner::richter(&loop_wps,   0.01,  true ).expect("m1 loop");
    let p1_corkscrew = TrajectoryPlanner::richter(&corkscrew_wps, 0.01, false).expect("m1 corkscrew");
    let corkscrew_m2_durs = p1_corkscrew.segment_durations();

    // Mode 2 position segment times: reuse Mode 1 durations for all trajectories (same waypoints).
    // Helix: all modes share helix_durs (uniform 0.375s, 48 segments).
    let circle_m2_durs = p1_circle.segment_durations();
    let fig8_m2_durs   = p1_fig8.segment_durations();
    let helix_m2_durs  = helix_durs.clone();
    let corner_m2_durs = p1_corner.segment_durations();
    let loop_m2_durs   = p1_loop.segment_durations();

    // Build Mode 0 planners (SplineTrajectory QP calls happen AFTER Mode 1 builds).
    let p0_circle = TrajectoryPlanner::spline(&circle_wps, &circle_durs, true ).expect("m0 circle");
    let p0_fig8   = TrajectoryPlanner::spline(&fig8_wps,   &fig8_durs,   false).expect("m0 fig8");
    let p0_helix  = TrajectoryPlanner::spline(&helix_wps,  &helix_durs,  false).expect("m0 helix");
    let p0_corner = TrajectoryPlanner::spline(&corner_wps, &corner_durs, false).expect("m0 corner");
    let p0_loop   = TrajectoryPlanner::spline(&loop_wps,   &loop_durs,   true ).expect("m0 loop");
    let p0_corkscrew = TrajectoryPlanner::spline(&corkscrew_wps, &corkscrew_durs, false).expect("m0 corkscrew");

    // Set in Mode 1 helix (same stretch factor as `run_flat_scaled` for Mode 2 helix sim).
    let helix_sim_time_scale: f32;

    // ────────────────────────────────────────────────────────────────────────
    println!("── Mode 0 (SplineTrajectory — manual durations) ────────────────");
    {
        println!("  [Geo]");
        let rg = run_flat(&p0_circle, &params, &mut GeometricController::default(), 1.0, "mode0 circle geo");
        write_closed_loop_csv(0, "circle", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p0_circle, &params, 1.0, "mode0 circle indi");
        write_closed_loop_csv(0, "circle", "indi", &ri);
    }
    {
        println!("  [Geo]");
        let rg = run_flat(&p0_fig8, &params, &mut GeometricController::default(), 0.5, "mode0 figure8 geo");
        write_closed_loop_csv(0, "figure8", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p0_fig8, &params, 0.5, "mode0 figure8 indi");
        write_closed_loop_csv(0, "figure8", "indi", &ri);
    }
    {
        println!("  [Geo]");
        let rg = run_flat(&p0_helix, &params, &mut GeometricController::default(), 1.0, "mode0 helix geo");
        write_closed_loop_csv(0, "helix", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p0_helix, &params, 1.0, "mode0 helix indi");
        write_closed_loop_csv(0, "helix", "indi", &ri);
    }
    {
        println!("  [Geo]");
        let rg = run_flat(&p0_corner, &params, &mut GeometricController::default(), 1.0, "mode0 corner geo");
        write_closed_loop_csv(0, "corner", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p0_corner, &params, 1.0, "mode0 corner indi");
        write_closed_loop_csv(0, "corner", "indi", &ri);
    }
    {
        println!("  [Geo]");
        let rg = run_flat(&p0_loop, &params, &mut GeometricController::default(), 1.0, "mode0 loop geo");
        write_closed_loop_csv(0, "loop", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p0_loop, &params, 1.0, "mode0 loop indi");
        write_closed_loop_csv(0, "loop", "indi", &ri);
    }
    {
        println!("  [Geo]");
        let rg = run_flat(&p0_corkscrew, &params, &mut GeometricController::default(), 1.0, "mode0 corkscrew geo");
        write_closed_loop_csv(0, "corkscrew", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p0_corkscrew, &params, 1.0, "mode0 corkscrew indi");
        write_closed_loop_csv(0, "corkscrew", "indi", &ri);
    }

    // ────────────────────────────────────────────────────────────────────────
    println!("\n── Mode 1 (RichterTrajectory — auto k_t, per-trajectory) ────────");
    {
        println!("    circle  total_time = {:.2} s", p1_circle.total_time());
        if let Some(rep) = p1_circle.check_feasibility(MASS, MAX_THRUST, MAX_OMEGA) {
            println!("    circle  feasible={} peak_T={:.3}N peak_ω={:.2}rad/s scale={:.2}",
                rep.feasible, rep.max_thrust_n, rep.max_omega_rad_s, rep.suggested_time_scale);
        }
        println!("  [Geo]");
        let rg = run_flat(&p1_circle, &params, &mut GeometricController::default(), 1.0, "mode1 circle geo");
        write_closed_loop_csv(1, "circle", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p1_circle, &params, 1.0, "mode1 circle indi");
        write_closed_loop_csv(1, "circle", "indi", &ri);
    }
    {
        println!("    figure8 total_time = {:.2} s", p1_fig8.total_time());
        if let Some(rep) = p1_fig8.check_feasibility(MASS, MAX_THRUST, MAX_OMEGA) {
            println!("    figure8 feasible={} peak_T={:.3}N peak_ω={:.2}rad/s scale={:.2}",
                rep.feasible, rep.max_thrust_n, rep.max_omega_rad_s, rep.suggested_time_scale);
        }
        println!("  [Geo]");
        let rg = run_flat(&p1_fig8, &params, &mut GeometricController::default(), 0.5, "mode1 figure8 geo");
        write_closed_loop_csv(1, "figure8", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p1_fig8, &params, 0.5, "mode1 figure8 indi");
        write_closed_loop_csv(1, "figure8", "indi", &ri);
    }
    {
        println!("    helix   total_time = {:.2} s", p1_helix.total_time());
        let mut helix_time_scale = 1.0_f32;
        if let Some(rep) = p1_helix.check_feasibility(MASS, MAX_THRUST, MAX_OMEGA) {
            println!("    helix   feasible={} peak_T={:.3}N peak_ω={:.2}rad/s scale={:.2}",
                rep.feasible, rep.max_thrust_n, rep.max_omega_rad_s, rep.suggested_time_scale);
            helix_time_scale = rep.suggested_time_scale.max(1.0);
        }
        helix_sim_time_scale = helix_time_scale;
        println!("  [Geo]");
        let rg = run_flat_scaled(&p1_helix, &params, &mut GeometricController::default(), 1.0, helix_time_scale, "mode1 helix geo");
        write_closed_loop_csv(1, "helix", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi_scaled(&p1_helix, &params, 1.0, helix_time_scale, "mode1 helix indi");
        write_closed_loop_csv(1, "helix", "indi", &ri);
    }
    {
        println!("    corner  total_time = {:.2} s", p1_corner.total_time());
        if let Some(rep) = p1_corner.check_feasibility(MASS, MAX_THRUST, MAX_OMEGA) {
            println!("    corner  feasible={} peak_T={:.3}N peak_ω={:.2}rad/s scale={:.2}",
                rep.feasible, rep.max_thrust_n, rep.max_omega_rad_s, rep.suggested_time_scale);
        }
        println!("  [Geo]");
        let rg = run_flat(&p1_corner, &params, &mut GeometricController::default(), 1.0, "mode1 corner geo");
        write_closed_loop_csv(1, "corner", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p1_corner, &params, 1.0, "mode1 corner indi");
        write_closed_loop_csv(1, "corner", "indi", &ri);
    }
    {
        println!("    loop    total_time = {:.2} s", p1_loop.total_time());
        if let Some(rep) = p1_loop.check_feasibility(MASS, MAX_THRUST, MAX_OMEGA) {
            println!("    loop    feasible={} peak_T={:.3}N peak_ω={:.2}rad/s scale={:.2}",
                rep.feasible, rep.max_thrust_n, rep.max_omega_rad_s, rep.suggested_time_scale);
        }
        println!("  [Geo]");
        let rg = run_flat(&p1_loop, &params, &mut GeometricController::default(), 1.0, "mode1 loop geo");
        write_closed_loop_csv(1, "loop", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p1_loop, &params, 1.0, "mode1 loop indi");
        write_closed_loop_csv(1, "loop", "indi", &ri);
    }
    {
        println!("    corkscrew total_time = {:.2} s", p1_corkscrew.total_time());
        if let Some(rep) = p1_corkscrew.check_feasibility(MASS, MAX_THRUST, MAX_OMEGA) {
            println!("    corkscrew feasible={} peak_T={:.3}N peak_ω={:.2}rad/s scale={:.2}",
                rep.feasible, rep.max_thrust_n, rep.max_omega_rad_s, rep.suggested_time_scale);
        }
        println!("  [Geo]");
        let rg = run_flat(&p1_corkscrew, &params, &mut GeometricController::default(), 1.0, "mode1 corkscrew geo");
        write_closed_loop_csv(1, "corkscrew", "geo", &rg);
        println!("  [INDI]");
        let ri = run_flat_indi(&p1_corkscrew, &params, 1.0, "mode1 corkscrew indi");
        write_closed_loop_csv(1, "corkscrew", "indi", &ri);
    }

    // ────────────────────────────────────────────────────────────────────────
    println!("\n── Mode 2 (Se3Trajectory — explicit attitude, incl. banked corner) ────");
    {
        let traj =
            Se3Trajectory::plan(&circle_se3_wps, &circle_m2_durs, MASS, true).expect("m2 circle");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3(&traj, &params, &mut geo_m2, "mode2 circle geo");
        write_closed_loop_csv(2, "circle", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi(&traj, &params, "mode2 circle indi");
        write_closed_loop_csv(2, "circle", "indi", &ri);
    }
    {
        let traj = Se3Trajectory::plan(&fig8_se3_wps, &fig8_m2_durs, MASS, FIG8_PERIODIC).expect("m2 fig8");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3(&traj, &params, &mut geo_m2, "mode2 figure8 geo");
        write_closed_loop_csv(2, "figure8", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi(&traj, &params, "mode2 figure8 indi");
        write_closed_loop_csv(2, "figure8", "indi", &ri);
    }
    {
        let traj =
            Se3Trajectory::plan(&helix_se3_wps, &helix_m2_durs, MASS, false).expect("m2 helix");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3_scaled(
            &traj,
            &params,
            &mut geo_m2,
            helix_sim_time_scale,
            "mode2 helix geo",
        );
        write_closed_loop_csv(2, "helix", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi_scaled(&traj, &params, helix_sim_time_scale, "mode2 helix indi");
        write_closed_loop_csv(2, "helix", "indi", &ri);
    }
    {
        let traj = Se3Trajectory::plan(&corner_se3_wps, &corner_m2_durs, MASS, false)
            .expect("m2 corner");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3(&traj, &params, &mut geo_m2, "mode2 corner geo");
        write_closed_loop_csv(2, "corner", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi(&traj, &params, "mode2 corner indi");
        write_closed_loop_csv(2, "corner", "indi", &ri);
    }
    {
        // Flip — position held, attitude does a full 360° pitch (Option A hover flip)
        let traj = Se3Trajectory::plan(&flip_se3_wps, &flip_se3_durs, MASS, false)
            .expect("m2 flip");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    flip  thrust range [{t_min:.3}, {t_max:.3}] N  (negative = motor cut at apex)");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3(&traj, &params, &mut geo_m2, "mode2 flip geo");
        write_closed_loop_csv(2, "flip", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi(&traj, &params, "mode2 flip indi");
        write_closed_loop_csv(2, "flip", "indi", &ri);
    }
    {
        // Loop — vertical circle with explicit attitude through inversion.
        // NOTE: geometric controller clamps thrust to 0 during inversion → motor cut.
        // High RMS is expected (drone free-falls through the apex).
        // Mode 2 exists to enable flips/loops on a more powerful platform (Bolt 1.1);
        // a full-INDI controller with negative-thrust motors is needed for sub-100 mm RMS.
        let traj = Se3Trajectory::plan(&loop_se3_wps, &loop_m2_durs, MASS, true)
            .expect("m2 loop");
        let thrusts: Vec<f32> = (0..=200)
            .map(|i| traj.eval(traj.total_time * i as f32 / 200.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    loop  thrust range [{t_min:.3}, {t_max:.3}] N  (negative = motor cut during inversion)");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3(&traj, &params, &mut geo_m2, "mode2 loop geo");
        write_closed_loop_csv(2, "loop", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi(&traj, &params, "mode2 loop indi");
        write_closed_loop_csv(2, "loop", "indi", &ri);
    }
    {
        // Corkscrew: helix + explicit 360° roll per lap (x-axis rotation; zero thrust at 180°)
        let traj = Se3Trajectory::plan(&corkscrew_se3_wps, &corkscrew_m2_durs, MASS, false)
            .expect("m2 corkscrew");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    corkscrew thrust range [{t_min:.3}, {t_max:.3}] N  (negative = motor cut at roll apex)");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3(&traj, &params, &mut geo_m2, "mode2 corkscrew geo");
        write_closed_loop_csv(2, "corkscrew", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi(&traj, &params, "mode2 corkscrew indi");
        write_closed_loop_csv(2, "corkscrew", "indi", &ri);
    }
    {
        // Roll: stationary y-axis inversion (pitch maneuver / backflip, distinct from flip's x-axis)
        let traj = Se3Trajectory::plan(&roll_se3_wps, &roll_se3_durs, MASS, false)
            .expect("m2 roll");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    roll  thrust range [{t_min:.3}, {t_max:.3}] N  (negative = motor cut at apex)");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3(&traj, &params, &mut geo_m2, "mode2 roll geo");
        write_closed_loop_csv(2, "roll", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi(&traj, &params, "mode2 roll indi");
        write_closed_loop_csv(2, "roll", "indi", &ri);
    }

    {
        // Immelmann — ascending half-loop + apex roll recovery (Mode 2 only)
        let traj = Se3Trajectory::plan(&immelmann_se3_wps, &immelmann_se3_durs, MASS, false)
            .expect("m2 immelmann");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    immelmann thrust range [{t_min:.3}, {t_max:.3}] N  (negative = motor cut at inversion)");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3(&traj, &params, &mut geo_m2, "mode2 immelmann geo");
        write_closed_loop_csv(2, "immelmann", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi(&traj, &params, "mode2 immelmann indi");
        write_closed_loop_csv(2, "immelmann", "indi", &ri);
    }
    {
        // Split-S — half-roll to inverted + descending half-loop (Mode 2 only)
        // Sim starts at Z + 2R = 2.0m; ends at Z = 1.0m.
        let traj = Se3Trajectory::plan(&splits_se3_wps, &splits_se3_durs, MASS, false)
            .expect("m2 splits");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    splits  thrust range [{t_min:.3}, {t_max:.3}] N  (negative = motor cut at inversion)");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3(&traj, &params, &mut geo_m2, "mode2 splits geo");
        write_closed_loop_csv(2, "splits", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi(&traj, &params, "mode2 splits indi");
        write_closed_loop_csv(2, "splits", "indi", &ri);
    }
    {
        // Screw — stationary XY, ascending Z, body x-axis roll (Mode 2 only)
        let traj = Se3Trajectory::plan(&screw_se3_wps, &screw_se3_durs, MASS, false)
            .expect("m2 screw");
        let thrusts: Vec<f32> = (0..=100)
            .map(|i| traj.eval(traj.total_time * i as f32 / 100.0).thrust)
            .collect();
        let t_min = thrusts.iter().cloned().fold(f32::MAX, f32::min);
        let t_max = thrusts.iter().cloned().fold(f32::MIN, f32::max);
        println!("    screw   thrust range [{t_min:.3}, {t_max:.3}] N  (negative = motor cut at inversion)");
        let mut geo_m2 = GeometricController::default();
        geo_m2.ki_pos = Vec3::zero();
        geo_m2.ki = Vec3::zero();
        println!("  [Geo]");
        let rg = run_se3(&traj, &params, &mut geo_m2, "mode2 screw geo");
        write_closed_loop_csv(2, "screw", "geo", &rg);
        println!("  [INDI]");
        let ri = run_se3_indi(&traj, &params, "mode2 screw indi");
        write_closed_loop_csv(2, "screw", "indi", &ri);
    }

    // ────────────────────────────────────────────────────────────────────────
    println!("\n── Output files ────────────────────────────────────────────────");
    println!("  results/planning_sim/closed_loop/mode<N>/<trajectory>/{{geo,indi}}.csv");
    println!("  Trajectories: circle, figure8, helix, corner, loop, corkscrew  (+ flip, roll, immelmann, splits, screw for mode 2 only)");
    println!("\nNext: scripts/plot_planning_all.sh");
    println!("Planner-only reference export: cargo run --release --bin planning_sim -- --export-reference-only");
}
