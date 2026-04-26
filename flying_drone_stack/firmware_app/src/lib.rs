//! SE(3) Geometric Controller — Crazyflie Firmware App (no_std Rust)
//!
//! Implements Lee et al. (2010) geometric controller on SE(3).
//! Tuned for stable hover on Crazyflie 2.1 / Brushless.

#![no_std]
#![allow(non_upper_case_globals, non_camel_case_types, non_snake_case)]

use panic_halt as _;

// ── Bindgen bindings ───────────────────────────────────────────────────────
mod bindings {
    #![allow(dead_code, non_upper_case_globals, non_camel_case_types, non_snake_case, clippy::all)]
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}
use bindings::{control_s, setpoint_s, sensorData_s, state_s};

// ── Vector3 helpers ────────────────────────────────────────────────────────
#[derive(Copy, Clone)]
struct Vec3 {
    x: f32, y: f32, z: f32,
}

impl Vec3 {
    #[inline(always)]
    const fn new(x: f32, y: f32, z: f32) -> Self { Self { x, y, z } }
    #[inline(always)]
    const fn zero() -> Self { Self::new(0.0, 0.0, 0.0) }

    #[inline(always)] fn add(self, b: Self) -> Self { Self::new(self.x + b.x, self.y + b.y, self.z + b.z) }
    #[inline(always)] fn sub(self, b: Self) -> Self { Self::new(self.x - b.x, self.y - b.y, self.z - b.z) }
    #[inline(always)] fn scale(self, s: f32) -> Self { Self::new(self.x * s, self.y * s, self.z * s) }
    #[inline(always)] fn dot(self, b: Self) -> f32 { self.x*b.x + self.y*b.y + self.z*b.z }
    #[inline(always)] fn cross(self, b: Self) -> Self {
        Self::new(self.y*b.z - self.z*b.y, self.z*b.x - self.x*b.z, self.x*b.y - self.y*b.x)
    }
    #[inline(always)] fn norm(self) -> f32 { libm::sqrtf(self.x*self.x + self.y*self.y + self.z*self.z) }
    #[inline(always)] fn normalize(self) -> Self {
        let n = self.norm();
        if n > 1e-9 { self.scale(1.0 / n) } else { Self::new(0.0, 0.0, 1.0) }
    }
}

// ── Matrix helpers ─────────────────────────────────────────────────────────
type Mat3 = [[f32; 3]; 3];

#[inline]
fn quat_to_rot(qw: f32, qx: f32, qy: f32, qz: f32) -> Mat3 {
    [
        [qw*qw + qx*qx - qy*qy - qz*qz, 2.0*(qx*qy - qw*qz), 2.0*(qx*qz + qw*qy)],
        [2.0*(qx*qy + qw*qz), qw*qw - qx*qx + qy*qy - qz*qz, 2.0*(qy*qz - qw*qx)],
        [2.0*(qx*qz - qw*qy), 2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz],
    ]
}

#[inline]
fn mat_at_b(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut o = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                o[i][j] += a[k][i] * b[k][j];
            }
        }
    }
    o
}

#[inline]
fn matsub(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut o = [[0.0f32; 3]; 3];
    for i in 0..3 { for j in 0..3 { o[i][j] = a[i][j] - b[i][j]; } }
    o
}

#[inline]
fn vee_half(m: &Mat3) -> Vec3 {
    Vec3::new(m[2][1] * 0.5, m[0][2] * 0.5, m[1][0] * 0.5)
}

// ── Constants ──────────────────────────────────────────────────────────────
const MASS: f32 = 0.027;           // kg — Crazyflie 2.1 + Flow Deck v2 (~3.2 g)
const GRAVITY: f32 = 9.81;
const HOVER_THRUST: f32 = MASS * GRAVITY; // ≈ 0.304 N

// Inertia (from official firmware / System ID)
const JXX: f32 = 16.571710e-6;
const JYY: f32 = 16.655602e-6;
const JZZ: f32 = 29.261652e-6;

// ── GAINS BLOCK A — our tuned gains ────────────────────────────────────────
// Increased KP_Z/KV_Z for stiff altitude hold; light integral for XY drift.
// Comment out block A and uncomment block B to compare against stock firmware.
// const KP_X: f32 = 7.5;    const KP_Y: f32 = 7.5;    const KP_Z: f32 = 26.0;
// const KV_X: f32 = 9.0;    const KV_Y: f32 = 9.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;                                                         // light integral — corrects steady-state offset
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.007;  const KR_Y: f32 = 0.007;  const KR_Z: f32 = 0.008;
// const KW_X: f32 = 0.00115;const KW_Y: f32 = 0.00115;const KW_Z: f32 = 0.002;

// ── GAINS BLOCK B — official Crazyflie Lee controller defaults ─────────────
// Source: crazyflie-firmware controller_lee.c (Kpos_P/D/I, KR, Komega defaults).
// Comment out block A above and uncomment these lines to use stock gains.
// const KP_X: f32 = 6.0;    const KP_Y: f32 = 6.0;    const KP_Z: f32 = 6.0;
// const KV_X: f32 = 4.0;    const KV_Y: f32 = 4.0;    const KV_Z: f32 = 4.0;
// const KI_P: f32 = 0.0;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.007;  const KR_Y: f32 = 0.007;  const KR_Z: f32 = 0.01;
// const KW_X: f32 = 0.0023; const KW_Y: f32 = 0.0023; const KW_Z: f32 = 0.003;

// ── GAINS BLOCK C  ───────────────────────────────────────
// Confirmed better than Block A: XY RMSE 16.2 cm (was 21.2 cm, −24%).
// Lag correlation −0.73 (was −0.92). Roll error halved. Pitch still ~10°.
// Active — do NOT comment out unless switching to another block.
// const KP_X: f32 = 9.0;    const KP_Y: f32 = 9.0;    const KP_Z: f32 = 26.0;
// const KV_X: f32 = 11.0;   const KV_Y: f32 = 11.0;   const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.007;  const KR_Y: f32 = 0.007;  const KR_Z: f32 = 0.008;
// const KW_X: f32 = 0.00115;const KW_Y: f32 = 0.00115;const KW_Z: f32 = 0.002;

// ── GAINS BLOCK D — (slight improvement over C) ────────────
// XY RMSE 15.9 cm (C: 16.4 cm, −3%). Diminishing returns — position gains near limit.
// Pitch error stuck at ~10° across A/C/D → attitude loop is now the bottleneck.
// Lag corr −0.77 (slightly worse than C −0.73) — sign of incipient overshoot.
// Active — do NOT comment out unless switching to another block.
// const KP_X: f32 = 11.0;   const KP_Y: f32 = 11.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 13.0;   const KV_Y: f32 = 13.0;   const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.007;  const KR_Y: f32 = 0.007;  const KR_Z: f32 = 0.008;
// const KW_X: f32 = 0.00115;const KW_Y: f32 = 0.00115;const KW_Z: f32 = 0.002;

// ── GAINS BLOCK E — current baseline ───────────────────────────────────────
// Best so far: XY RMSE 13.9 cm (−35% vs Block A, −12% vs Block D).
// Attitude gains (KR 0.007→0.010, KW 0.00115→0.0018) gave the extra jump.
// Pitch error stuck at ~10° — structural: ~8.6° is position-correction demand,
// not attitude lag. Will only fall if position tracking improves further.
// Analysis: zeta=1.96 (overdamped), dominant tau=1.10s, ctrl BW=0.14Hz = fig8 freq.
// Comment out and activate Block G to test critically-damped position loop.
const KP_X: f32 = 11.0;   const KP_Y: f32 = 11.0;   const KP_Z: f32 = 26.0;
const KV_X: f32 = 13.0;   const KV_Y: f32 = 13.0;   const KV_Z: f32 = 14.0;
const KI_P: f32 = 0.05;
const KI_LIMIT: f32 = 2.0;

const KR_X: f32 = 0.009;  const KR_Y: f32 = 0.009;  const KR_Z: f32 = 0.009;
const KW_X: f32 = 0.0016; const KW_Y: f32 = 0.0016; const KW_Z: f32 = 0.002;

// ── GAINS BLOCK G — conservative step toward critical damping ───────────────
// zeta=1.21, dominant tau=0.57s, BW=0.28Hz (2x fig8 freq). KV=8 only.
// If stable with no oscillation, proceed to Block H.
// const KP_X: f32 = 11.0;   const KP_Y: f32 = 11.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 8.0;    const KV_Y: f32 = 8.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.009;  const KR_Y: f32 = 0.009;  const KR_Z: f32 = 0.009;
// const KW_X: f32 = 0.0016; const KW_Y: f32 = 0.0016; const KW_Z: f32 = 0.002;

// ── GAINS BLOCK H — near-critical damping ───────────────────────────────────
// zeta=1.06 (just above critical), dominant tau=0.42s, BW=0.38Hz (2.8x fig8).
// Theoretical optimum for KP=11 is KV=6.6 (zeta=1.0); KV=7 is a safe margin.
// Dominant pole 2.4x faster than Block E. Try only if Block G shows no oscillation.
// const KP_X: f32 = 11.0;   const KP_Y: f32 = 11.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 7.0;    const KV_Y: f32 = 7.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.009;  const KR_Y: f32 = 0.009;  const KR_Z: f32 = 0.009;
// const KW_X: f32 = 0.0016; const KW_Y: f32 = 0.0016; const KW_Z: f32 = 0.002;

// ── GAINS BLOCK I — higher bandwidth at exact critical damping ───────────────
// KP=16 raises omega_n to 4.0 rad/s. KV=8 gives zeta = 8/(2*4) = 1.00 exactly.
// Dominant tau=0.25s, BW=0.64Hz (4.7x fig8 freq). Theoretically best tracking.
// Note: KV=8 same as Block G — the difference is higher KP which raises the
// natural frequency, making the whole loop 2.3x faster than Block G.
// Risk: KP=16 is a large jump (+45%). Start with hover before running figure-8.
// const KP_X: f32 = 16.0;   const KP_Y: f32 = 16.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 8.0;    const KV_Y: f32 = 8.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.009;  const KR_Y: f32 = 0.009;  const KR_Z: f32 = 0.009;
// const KW_X: f32 = 0.0016; const KW_Y: f32 = 0.0016; const KW_Z: f32 = 0.002;

// ── Controller mode selector — change this one constant to switch controllers ─
//
//   0 = Geometric SE(3)  — current baseline, fully validated
//   1 = Attitude INDI    — gyro-only incremental, no RPM deck needed
//                          When ready: flash, tune FC_GYRO_HZ, re-run maneuver suite
//   2 = Full INDI        — RPM-based incremental, requires RPM deck
//                          DO NOT activate until: RPM deck fitted, KT identified on bench
//
// Unused controller paths are fully eliminated by the compiler (const branch).
const CONTROLLER_MODE: u8 = 0;

const FC_GYRO_HZ: f32 = 60.0;  // gyro LP filter cutoff [Hz] — modes 1 & 2, range 30–80 Hz

// Motor model constants — needed for CONTROLLER_MODE = 2 (full INDI) only.
// KT: identify on thrust stand (thrust vs RPM²) before first full-INDI flight.
// Motor spin directions: verify against powerDistributionForceTorque.c in crazyflie-firmware.
const KT:      f32 = 3.16e-10;    // thrust coefficient [N / RPM²]  — PLACEHOLDER, bench-identify
const KQ_KT:   f32 = 0.005964552; // drag-to-thrust moment ratio (confirmed from CAD)
const ARM_LEN: f32 = 0.046;       // motor arm length [m]



// ── Controller State ───────────────────────────────────────────────────────
struct State {
    i_ep: Vec3,             // position integral
    last_tick: u32,
    omega_prev: Vec3,       // previous angular velocity (INDI: gyro differentiation)
    omega_dot_filt: Vec3,   // low-pass filtered angular acceleration (INDI)
    tau_prev: Vec3,         // previous torque command (INDI memory)
}

impl State {
    const fn zero() -> Self {
        Self {
            i_ep: Vec3::zero(), last_tick: 0,
            omega_prev: Vec3::zero(),
            omega_dot_filt: Vec3::zero(),
            tau_prev: Vec3::zero(),
        }
    }
    fn reset(&mut self) {
        self.i_ep = Vec3::zero();
        self.last_tick = 0;
        self.omega_prev = Vec3::zero();
        self.omega_dot_filt = Vec3::zero();
        self.tau_prev = Vec3::zero();
    }
}

static mut CTRL: State = State::zero();

// ── Attitude INDI torque ────────────────────────────────────────────────────
//
// Incremental Nonlinear Dynamic Inversion (Smeur et al. 2016 / Faessler 2018).
// Replaces the KR/KW model-based torque with an incremental correction:
//
//   α_des  = geometric virtual control / J  (same KR/KW as before, just rescaled)
//   α_meas = (ω − ω_prev) / dt             (differentiated gyro, low-pass filtered)
//   Δτ     = J · (α_des − α_meas)
//   τ_new  = τ_prev + Δτ + gyro_comp
//
// Motor lag, blade flapping, and model errors are captured in α_meas and corrected
// one timestep at a time — no model of those effects needed.
//
// Tuning: FC_GYRO_HZ is the only new parameter.
//   Too high → noise in α_meas corrupts every correction.
//   Too low  → phase lag destabilises the attitude loop.
//   Start at 60 Hz; lower if oscillating at hover, raise only if response is sluggish.
#[allow(dead_code)]
fn indi_torque(
    er: Vec3, e_omega: Vec3, omega: Vec3, gyro_comp: Vec3,
    s: &mut State, dt: f32,
) -> Vec3 {
    // Desired angular acceleration × J  (virtual control from geometric law)
    let v_des = Vec3::new(
        -KR_X * er.x - KW_X * e_omega.x,
        -KR_Y * er.y - KW_Y * e_omega.y,
        -KR_Z * er.z - KW_Z * e_omega.z,
    );

    // Measured angular acceleration (raw, then filtered)
    let alpha_raw = Vec3::new(
        (omega.x - s.omega_prev.x) / dt,
        (omega.y - s.omega_prev.y) / dt,
        (omega.z - s.omega_prev.z) / dt,
    );
    // 1st-order IIR: k = dt / (dt + RC),  RC = 1 / (2π · fc)
    let rc = 1.0 / (2.0 * core::f32::consts::PI * FC_GYRO_HZ);
    let k  = dt / (dt + rc);
    s.omega_dot_filt = Vec3::new(
        k * alpha_raw.x + (1.0 - k) * s.omega_dot_filt.x,
        k * alpha_raw.y + (1.0 - k) * s.omega_dot_filt.y,
        k * alpha_raw.z + (1.0 - k) * s.omega_dot_filt.z,
    );

    // Δτ = J · (α_des − α_meas)
    let delta_tau = Vec3::new(
        v_des.x - JXX * s.omega_dot_filt.x,
        v_des.y - JYY * s.omega_dot_filt.y,
        v_des.z - JZZ * s.omega_dot_filt.z,
    );

    let tau = Vec3::new(
        s.tau_prev.x + delta_tau.x + gyro_comp.x,
        s.tau_prev.y + delta_tau.y + gyro_comp.y,
        s.tau_prev.z + delta_tau.z + gyro_comp.z,
    );
    s.tau_prev = tau;
    tau
}

// ── Full INDI torque ────────────────────────────────────────────────────────
//
// Same incremental law as attitude INDI but τ_current comes from actual per-motor
// RPM² via the motor model G(Ω), instead of being carried from the previous step:
//
//   τ_current = G(Ω) · [Ω₁², Ω₂², Ω₃², Ω₄²]   ← from RPM measurements
//   Δτ        = J · (α_des − α_meas)
//   τ_new     = τ_current + Δτ
//
// More accurate than attitude INDI at high angular rates where commanded and actual
// torque diverge due to motor lag and saturation.
//
// PREREQUISITES before setting CONTROLLER_MODE = 2:
//   1. RPM deck physically fitted to the drone
//   2. KT identified on bench (thrust vs RPM² sweep on thrust stand)
//   3. Motor spin directions verified against powerDistributionForceTorque.c
//   4. rpm_sq filled from actual sensor readings in the call site below
#[allow(dead_code)]
fn compute_tau_from_rpm(rpm_sq: [f32; 4]) -> Vec3 {
    // F_i = KT * RPM_i²
    let f = [KT*rpm_sq[0], KT*rpm_sq[1], KT*rpm_sq[2], KT*rpm_sq[3]];
    // X-frame mixer (standard CF2.x, viewed from above):
    //   M1 front-right CW,  M2 back-right CCW,  M3 back-left CW,  M4 front-left CCW
    // Verify signs against powerDistributionForceTorque.c before first flight.
    let lh = ARM_LEN * 0.707106781; // L / √2  (arm projected onto body x/y axis)
    Vec3::new(
        (-f[0] + f[1] + f[2] - f[3]) * lh,    // τ_x (roll)
        (-f[0] - f[1] + f[2] + f[3]) * lh,    // τ_y (pitch)
        (-f[0] + f[1] - f[2] + f[3]) * KQ_KT, // τ_z (yaw)
    )
}

#[allow(dead_code)]
fn full_indi_torque(
    er: Vec3, e_omega: Vec3, omega: Vec3, gyro_comp: Vec3,
    s: &mut State, dt: f32,
    rpm_sq: [f32; 4],
) -> Vec3 {
    // Current torque state measured from RPM (replaces tau_prev from attitude INDI)
    let tau_current = compute_tau_from_rpm(rpm_sq);

    // Desired angular acceleration × J  (identical virtual control to attitude INDI)
    let v_des = Vec3::new(
        -KR_X * er.x - KW_X * e_omega.x,
        -KR_Y * er.y - KW_Y * e_omega.y,
        -KR_Z * er.z - KW_Z * e_omega.z,
    );

    // Measured angular acceleration — same IIR filter as attitude INDI
    let alpha_raw = Vec3::new(
        (omega.x - s.omega_prev.x) / dt,
        (omega.y - s.omega_prev.y) / dt,
        (omega.z - s.omega_prev.z) / dt,
    );
    let rc = 1.0 / (2.0 * core::f32::consts::PI * FC_GYRO_HZ);
    let k  = dt / (dt + rc);
    s.omega_dot_filt = Vec3::new(
        k * alpha_raw.x + (1.0 - k) * s.omega_dot_filt.x,
        k * alpha_raw.y + (1.0 - k) * s.omega_dot_filt.y,
        k * alpha_raw.z + (1.0 - k) * s.omega_dot_filt.z,
    );

    // Δτ = J · (α_des − α_meas)
    let delta_tau = Vec3::new(
        v_des.x - JXX * s.omega_dot_filt.x,
        v_des.y - JYY * s.omega_dot_filt.y,
        v_des.z - JZZ * s.omega_dot_filt.z,
    );

    // τ_new = τ_current (from RPM, not tau_prev) + Δτ + gyro_comp
    let tau = Vec3::new(
        tau_current.x + delta_tau.x + gyro_comp.x,
        tau_current.y + delta_tau.y + gyro_comp.y,
        tau_current.z + delta_tau.z + gyro_comp.z,
    );
    s.tau_prev = tau; // keep in sync in case mode is switched mid-flight
    tau
}

// ── Desired rotation matrix ────────────────────────────────────────────────
fn desired_rot(f_d: Vec3, yaw_d: f32) -> Mat3 {
    let zdes = f_d.normalize();
    let xcdes = Vec3::new(libm::cosf(yaw_d), libm::sinf(yaw_d), 0.0);
    let zcx = zdes.cross(xcdes);
    let ydes = if zcx.norm() > 1e-9 { zcx.normalize() } else { Vec3::new(0.0, 1.0, 0.0) };
    let xdes = ydes.cross(zdes);

    [[xdes.x, ydes.x, zdes.x],
     [xdes.y, ydes.y, zdes.y],
     [xdes.z, ydes.z, zdes.z]]
}

// ── Core geometric controller ──────────────────────────────────────────────
fn geometric_step(
    pos: Vec3, vel: Vec3, r: &Mat3, omega: Vec3,
    pd: Vec3, vd: Vec3, ad: Vec3, yaw_d: f32,
    dt: f32, s: &mut State,
) -> (f32, Vec3) {
    let ep = pd.sub(pos);
    let ev = vd.sub(vel);

    // Integral with anti-windup
    s.i_ep = s.i_ep.add(ep.scale(dt));
    s.i_ep = Vec3::new(
        s.i_ep.x.clamp(-KI_LIMIT, KI_LIMIT),
        s.i_ep.y.clamp(-KI_LIMIT, KI_LIMIT),
        s.i_ep.z.clamp(-KI_LIMIT, KI_LIMIT),
    );

    // Desired force in world frame
    let f_d = ad
        .add(Vec3::new(KP_X*ep.x, KP_Y*ep.y, KP_Z*ep.z))
        .add(Vec3::new(KV_X*ev.x, KV_Y*ev.y, KV_Z*ev.z))
        .add(Vec3::new(KI_P*s.i_ep.x, KI_P*s.i_ep.y, KI_P*s.i_ep.z))
        .add(Vec3::new(0.0, 0.0, GRAVITY));

    let thrust_vec = f_d.scale(MASS);

    // Thrust = projection onto current body z-axis
    let body_z = Vec3::new(r[0][2], r[1][2], r[2][2]);
    let thrust = thrust_vec.dot(body_z).max(0.0);

    // Reset integral when on the ground (stronger condition than before)
    if thrust < 0.05 {
        s.i_ep = Vec3::zero();
    }

    // Desired rotation matrix
    let rd = desired_rot(thrust_vec, yaw_d);

    // Rotation error eR = ½ (Rd^T R − R^T Rd)^∨
    let er = vee_half(&matsub(&mat_at_b(&rd, r), &mat_at_b(r, &rd)));

    let e_omega = omega;

    // Gyroscopic term
    let j_omega = Vec3::new(JXX*omega.x, JYY*omega.y, JZZ*omega.z);
    let gyro_comp = omega.cross(j_omega);

    // Torque command — selected by CONTROLLER_MODE (const → unused branches compiled out).
    // Mode 2: replace [0.0; 4] with actual RPM² from deck when available, e.g.:
    //   let rpm_sq = [sens.motor.m1 as f32 * sens.motor.m1 as f32, ...];
    let torque = if CONTROLLER_MODE == 1 {
        indi_torque(er, e_omega, omega, gyro_comp, s, dt)
    } else if CONTROLLER_MODE == 2 {
        full_indi_torque(er, e_omega, omega, gyro_comp, s, dt, [0.0; 4]) // TODO: real RPM²
    } else {
        Vec3::new(
            -KR_X*er.x - KW_X*e_omega.x + gyro_comp.x,
            -KR_Y*er.y - KW_Y*e_omega.y + gyro_comp.y,
            -KR_Z*er.z - KW_Z*e_omega.z + gyro_comp.z,
        )
    };

    (thrust, torque)
}

// ── Firmware entry points ──────────────────────────────────────────────────
#[no_mangle]
pub extern "C" fn controllerOutOfTreeInit() {
    unsafe { (*core::ptr::addr_of_mut!(CTRL)).reset(); }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTreeTest() -> bool {
    true
}

#[no_mangle]
pub unsafe extern "C" fn controllerOutOfTree(
    control: *mut control_s,
    setpoint: *const setpoint_s,
    sensors: *const sensorData_s,
    state: *const state_s,
    tick: u32,
) {
    let s = &mut *core::ptr::addr_of_mut!(CTRL);

    let dt = if s.last_tick == 0 { 0.002_f32 }
             else { (tick.wrapping_sub(s.last_tick)) as f32 * 0.001_f32 };
    s.last_tick = tick;

    // Current state
    let st = &*state;
    let pos = Vec3::new(st.position.x, st.position.y, st.position.z);
    let vel = Vec3::new(st.velocity.x, st.velocity.y, st.velocity.z);

    let qw = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q3;
    let qx = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q0;
    let qy = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q1;
    let qz = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q2;
    let r = quat_to_rot(qw, qx, qy, qz);

    let deg2rad = core::f32::consts::PI / 180.0_f32;
    let g = &(*sensors).gyro;
    let omega = Vec3::new(g.axis[0]*deg2rad, g.axis[1]*deg2rad, g.axis[2]*deg2rad);

    let sp = &*setpoint;

    // SAFE ARMING: only enable controller when test harness sends real position (z > 0.05 m)
    let armed = sp.position.z > 0.05_f32;

    let pd = Vec3::new(sp.position.x, sp.position.y, sp.position.z);
    let vd = Vec3::new(sp.velocity.x, sp.velocity.y, sp.velocity.z);
    let ad = Vec3::new(sp.acceleration.x, sp.acceleration.y, sp.acceleration.z);
    let yaw_d = sp.attitude.yaw * deg2rad;

    let (thrust_si, torque) = geometric_step(pos, vel, &r, omega, pd, vd, ad, yaw_d, dt, s);

    s.omega_prev = omega;

    // Output
    let out = &mut *control;
    let union_ptr = (&mut out.__bindgen_anon_1) as *mut _ as *mut f32;

    if armed {
        *union_ptr.add(0) = thrust_si;
        *union_ptr.add(1) = torque.x;
        *union_ptr.add(2) = torque.y;
        *union_ptr.add(3) = torque.z;
    } else {
        *union_ptr.add(0) = 0.0;
        *union_ptr.add(1) = 0.0;
        *union_ptr.add(2) = 0.0;
        *union_ptr.add(3) = 0.0;
    }

    out.controlMode = bindings::control_mode_e_controlModeForceTorque;
}