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

// ── Stage-1 filter: 2nd-order Butterworth ──────────────────────────────────
// Bilinear-transform coefficients (τ = 1/(2π·fc)):
//   denom = τ² + √2·τ·dt + dt²
//   b = dt²/denom,  a1 = 2(dt²−τ²)/denom,  a2 = (τ²−√2·τ·dt+dt²)/denom
// y[n] = b·(x[n] + 2·x[n−1] + x[n−2]) − a1·y[n−1] − a2·y[n−2]
#[derive(Copy, Clone)]
struct Butterworth2 {
    b: f32, a1: f32, a2: f32,
    x1: f32, x2: f32, y1: f32, y2: f32,
}
impl Butterworth2 {
    const fn zero() -> Self {
        Self { b: 0.0, a1: 0.0, a2: 0.0, x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0 }
    }
    fn init(&mut self, fc: f32, dt: f32) {
        const SQRT2: f32 = 1.414_213_6_f32;
        let tau   = 1.0 / (2.0 * core::f32::consts::PI * fc);
        let denom = tau*tau + SQRT2*tau*dt + dt*dt;
        self.b  = dt*dt / denom;
        self.a1 = 2.0 * (dt*dt - tau*tau) / denom;
        self.a2 = (tau*tau - SQRT2*tau*dt + dt*dt) / denom;
    }
    fn update(&mut self, x: f32) -> f32 {
        let y = self.b*x + 2.0*self.b*self.x1 + self.b*self.x2
              - self.a1*self.y1 - self.a2*self.y2;
        self.x2 = self.x1; self.x1 = x;
        self.y2 = self.y1; self.y1 = y;
        y
    }
    fn seed(&mut self, v: f32) { self.x1 = v; self.x2 = v; self.y1 = v; self.y2 = v; }
    fn reset_state(&mut self) { self.seed(0.0); }
}

// ── Stage-3 filter: 1st-order IIR ──────────────────────────────────────────
// k = dt/(dt+RC), RC = 1/(2π·fc) — precomputed at init for fixed-rate 500 Hz
// y[n] = k·x[n] + (1−k)·y[n−1]
#[derive(Copy, Clone)]
struct IirFilter {
    k: f32,
    state: f32,
}
impl IirFilter {
    const fn zero() -> Self { Self { k: 0.0, state: 0.0 } }
    fn init(&mut self, fc: f32, dt: f32) {
        let rc  = 1.0 / (2.0 * core::f32::consts::PI * fc);
        self.k  = dt / (dt + rc);
    }
    fn update(&mut self, x: f32) -> f32 {
        self.state = self.k * x + (1.0 - self.k) * self.state;
        self.state
    }
    fn reset_state(&mut self) { self.state = 0.0; }
}

// ── Matrix helpers ─────────────────────────────────────────────────────────
type Mat3 = [[f32; 3]; 3];

#[inline]
// ToDo: double check quat to rot transformation to find sign flip in y-axis pitch
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

#[inline]
fn mat_mul_vec(m: &Mat3, v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
        m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
        m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z,
    )
}

// ── Constants ──────────────────────────────────────────────────────────────
const GRAVITY: f32 = 9.81;

// Inertia (from official firmware / System ID)
const JXX: f32 = 16.571710e-6;
const JYY: f32 = 16.655602e-6;
const JZZ: f32 = 29.261652e-6;

// ── Controller mode ────────────────────────────────────────────────────────
// Runtime-configurable via CRTP param indi_gains.controller_mode (no reflash needed):
//   0 = Geometric SE(3)               (bit 1 = 0, bit 0 = 0)
//   1 = Position INDI outer loop only (bit 1 = 0, bit 0 = 1)
//   2 = Attitude INDI inner loop only (bit 1 = 1, bit 0 = 0)
//   3 = Full INDI — pos + att         (bit 1 = 1, bit 0 = 1); requires RPM deck + KT identified

// ── INDI tuning params — runtime-configurable via CRTP param group "indi_gains" ──
// Defaults below match the C globals in traj_iface.c (same values).
// Change without reflashing: set indi_gains.kr / .kw / .kt1..kt4 etc. via CS2 yaml,
// cfclient Parameters tab, or: cf.param.set_value('indi_gains.kr', '200.0')
// KR [1/s²]: ωₙ = √KR.  KW [1/s]: ζ = KW/(2·ωₙ).  Keep KW > KR for Routh stability.
// Conservative start: ωₙ=10 rad/s (KR=100), ζ=1.5 (KW=30).  Target: KR=603, KW=66.
// Filter reinit happens automatically in the controller loop when fc_bw/fc_iir change.
const ARM_M: f32      = 0.032_526_9_f32; // √2/2 × 0.046 m (diagonal arm projection)
const TORQUE_RATIO: f32 = 0.005_964_552_f32; // k_Q/k_T = THRUST2TORQUE [m]

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
// const KP_X: f32 = 11.0;   const KP_Y: f32 = 11.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 13.0;   const KV_Y: f32 = 13.0;   const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.009;  const KR_Y: f32 = 0.009;  const KR_Z: f32 = 0.009;
// const KW_X: f32 = 0.0016; const KW_Y: f32 = 0.0016; const KW_Z: f32 = 0.002;

// ── GAINS BLOCK G (seems better then E)— conservative step toward critical damping ───────────────
// zeta=1.21, dominant tau=0.57s, BW=0.28Hz (2x fig8 freq). KV=8 only.
// If stable with no oscillation, proceed to Block H.
// const KP_X: f32 = 11.0;   const KP_Y: f32 = 11.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 8.0;    const KV_Y: f32 = 8.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.009;  const KR_Y: f32 = 0.009;  const KR_Z: f32 = 0.009;
// const KW_X: f32 = 0.0016; const KW_Y: f32 = 0.0016; const KW_Z: f32 = 0.002;

// ── GAINS BLOCK H (seems worse then g and maybe bit worse then E) — near-critical damping ───────────────────────────────────
// zeta=1.06 (just above critical), dominant tau=0.42s, BW=0.38Hz (2.8x fig8).
// Theoretical optimum for KP=11 is KV=6.6 (zeta=1.0); KV=7 is a safe margin.
// Dominant pole 2.4x faster than Block E. Try only if Block G shows no oscillation.
// const KP_X: f32 = 11.0;   const KP_Y: f32 = 11.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 7.0;    const KV_Y: f32 = 7.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.009;  const KR_Y: f32 = 0.009;  const KR_Z: f32 = 0.009;
// const KW_X: f32 = 0.0016; const KW_Y: f32 = 0.0016; const KW_Z: f32 = 0.002;

// ── GAINS BLOCK I (best position loop; attitude still overdamped, current new baseline) ─────────────
// Apr 27 2026 flights: XY RMSE 12.7–13.3 cm. Along-track lag −7.4 cm mean.
// Worst errors at center crossing (segs 5-6): 20-21 cm XY RMSE.
// Bottleneck analysis: attitude loop zeta=2.07, tau_dom=167ms — too overdamped.
// KW_X=0.0016 is 2.07x the critical value for KR=0.009/JXX. Position BW adequate.
// KP=16 raises omega_n to 4.0 rad/s. KV=8 gives zeta=1.00 exactly.
// Dominant pos tau=0.25s, BW=0.64Hz (4.7x fig8 freq).
// const KP_X: f32 = 16.0;   const KP_Y: f32 = 16.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 8.0;    const KV_Y: f32 = 8.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.009;  const KR_Y: f32 = 0.009;  const KR_Z: f32 = 0.009;
// const KW_X: f32 = 0.0016; const KW_Y: f32 = 0.0016; const KW_Z: f32 = 0.002;

// ── GAINS BLOCK J (seems little worse then I) — first attitude fix: zeta 2.07→1.23 (2x faster) ────
// Root cause: attitude loop zeta=2.07, tau_dom=167ms. Center crossing needs 560ms
// to reverse — attitude loop never settles. Single change: KW 0.0016→0.0010.
//   KR=0.010: omega_n=24.6 rad/s
//   KW=0.0010: zeta=1.23, tau_dom=79ms  (was 167ms)
// Position gains unchanged from Block I — not the bottleneck.
// Start with hover, then figure-8. If stable with no oscillation: proceed to Block K.
// const KP_X: f32 = 16.0;   const KP_Y: f32 = 16.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 8.0;    const KV_Y: f32 = 8.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
// const KW_X: f32 = 0.0010; const KW_Y: f32 = 0.0010; const KW_Z: f32 = 0.00125;

// ── GAINS BLOCK K — flown Apr 27. XY RMSE 12.0-12.3cm. Better roll (15° vs 25°).
// BUT: gyro jitter higher than I (dGx std 22-28 vs 17-21 °/s/step) — KW too low,
// less damping of gyro noise. Marginally better tracking, visibly noisier gyro.
// KR=0.010, KW=0.00082: zeta=1.01, tau_dom=46ms.
// const KP_X: f32 = 16.0;   const KP_Y: f32 = 16.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 8.0;    const KV_Y: f32 = 8.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
// const KW_X: f32 = 0.00082;const KW_Y: f32 = 0.00082;const KW_Z: f32 = 0.00103;

// ── GAINS BLOCK L — flown Apr 27. OSCILLATED. Do not use. ───────────────────────
// KR=0.012 hit the stability margin: motor lag (~20-50ms) adds phase that the
// simplified 2nd-order model ignores. KR ≤ 0.010 is the practical ceiling for the
// geometric controller on this hardware. KR=0.012 causes heavy oscillations.
// const KP_X: f32 = 16.0;   const KP_Y: f32 = 16.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 8.0;    const KV_Y: f32 = 8.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.012;  const KR_Y: f32 = 0.012;  const KR_Z: f32 = 0.012;
// const KW_X: f32 = 0.00089;const KW_Y: f32 = 0.00089;const KW_Z: f32 = 0.00111;

// ── GAINS BLOCK M — flown Apr 27. OSCILLATED HEAVILY. Do not use. ───────────────
// Same root cause as L: KR=0.012 with lower KW = even less stability margin.
// const KP_X: f32 = 16.0;   const KP_Y: f32 = 16.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 8.0;    const KV_Y: f32 = 8.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.012;  const KR_Y: f32 = 0.012;  const KR_Z: f32 = 0.012;
// const KW_X: f32 = 0.00075;const KW_Y: f32 = 0.00075;const KW_Z: f32 = 0.00094;

// ── GAINS BLOCK N (active) — sweet spot between K and J ──────────────────────────
// K (KW=0.00082) has better tracking but noisy gyro. J (KW=0.0010) is smoother.
// N splits the difference: KW=0.0011, zeta=1.35, tau_dom=92ms.
//   KR ceiling confirmed at 0.010 (KR=0.012 oscillates on real hardware).
//   More damping than K → smoother angular velocity in plots.
//   Still 1.8x faster response than Block I (tau 92ms vs 167ms).
//   XY RMSE expected ~12-12.5cm, roll max ~16-18°, cleaner gyro than K.
// current baseline
// const KP_X: f32 = 16.0;   const KP_Y: f32 = 16.0;   const KP_Z: f32 = 26.0;
// const KV_X: f32 = 8.0;    const KV_Y: f32 = 8.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
// const KW_X: f32 = 0.00110;const KW_Y: f32 = 0.00110;const KW_Z: f32 = 0.00138;

// best one 
// const KP_X: f32 = 24.0;   const KP_Y: f32 = 24.0;   const KP_Z: f32 = 30.0;
// const KV_X: f32 = 6.0;    const KV_Y: f32 = 6.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
// const KW_X: f32 = 0.00110;const KW_Y: f32 = 0.00110;const KW_Z: f32 = 0.00138;

// // third best
// const KP_X: f32 = 26.0;   const KP_Y: f32 = 26.0;   const KP_Z: f32 = 30.0;
// const KV_X: f32 = 6.0;    const KV_Y: f32 = 6.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;
// const KI_LIMIT: f32 = 2.0;

// const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
// const KW_X: f32 = 0.00110;const KW_Y: f32 = 0.00110;const KW_Z: f32 = 0.00138;

// ── GAINS BLOCK OFFICIAL — exact defaults from crazyflie-firmware controller_lee.c ─────────────
// Source: g_self defaults, Khaled Wahba (MIT licence), 2024 crazyflie-firmware.
//
// Official values:  KP=7, KV=4, KI_pos=0,  KR=0.007, KW=0.00115, KR_Z=0.008, KW_Z=0.002
//                   KI_att=0.03 (attitude integral — now implemented, set 0.0 in all other blocks)
//
// Mathematical comparison vs our active (KP=28, KV=6, KR=0.010, KW=0.00110):
//
//                        att wn   att ζ  att τ_dom   pos wn   pos ζ  pos τ_dom  BW_pos  cascade
//   Official             20.6     1.69    148 ms      16.1     4.60    565 ms    0.28Hz  3.81×
//   Ours (KP=28)         24.6     1.35     92 ms      32.2     3.45    210 ms    0.76Hz  2.28×
//   Our Block N (KP=16)  24.6     1.35     92 ms      24.3     6.09    497 ms    0.32Hz  5.40×
//
//   Figure-8 ω = 0.86 rad/s (7.28 s period):
//     Official:    pos dominant pole = 1.8 rad/s →  2.1× trajectory  (barely adequate)
//     Ours KP=28:  pos dominant pole = 4.8 rad/s →  5.5× trajectory  (3× better)
//
// VERDICT: Official position loop bandwidth is only 2.1× the figure-8 frequency (rule of
// thumb: need ≥5×).  Our KP=28 gives 5.5×.  Prediction: official RMSE ≈ 18–22 cm.
// KI_P set to 0.0 to match official exactly; change to 0.05 to add our position integral.
// OFFICIAL block — INACTIVE
// const KP_X: f32 = 7.0;    const KP_Y: f32 = 7.0;    const KP_Z: f32 = 7.0;
// const KV_X: f32 = 4.0;    const KV_Y: f32 = 4.0;    const KV_Z: f32 = 4.0;
// const KI_P: f32 = 0.0;    const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.007;  const KR_Y: f32 = 0.007;  const KR_Z: f32 = 0.008;
// const KW_X: f32 = 0.00115;const KW_Y: f32 = 0.00115;const KW_Z: f32 = 0.002;
// const KI_ATT: f32 = 0.03;

// ★ BEST OPTICAL FLOW BASELINE (Block N equivalent) — DO NOT MODIFY ★ INACTIVE ★
// Best result with optical flow + traj.mode=1 (Rust upload): smooth figure-8,
// XY RMSE ~12–12.5 cm, gyro cleaner than K, roll max ~16–18°.
// KP=28/KV=6 gave zeta≈1.35, tau_dom=92ms. Reactivate for optical flow sessions.
// const KP_X: f32 = 28.0;   const KP_Y: f32 = 28.0;   const KP_Z: f32 = 30.0;
// const KV_X: f32 = 6.0;    const KV_Y: f32 = 6.0;    const KV_Z: f32 = 14.0;
// const KI_P: f32 = 0.05;   const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
// const KW_X: f32 = 0.00110;const KW_Y: f32 = 0.00110;const KW_Z: f32 = 0.00138;
// const KI_ATT: f32 = 0.0;

// ═══════════════════════════════════════════════════════════════════════════════
// MOCAP GAIN BLOCKS — tuned with OptiTrack / CS2 / traj.mode=0
// Key difference vs optical flow: EKF derives velocity by differentiating accurate
// mocap positions → KV must be significantly lower to avoid amplifying that noise.
// Strategy: keep KP high (positions are accurate), drop KV, attitude gains from Block N.
// ═══════════════════════════════════════════════════════════════════════════════

// ── MOCAP BLOCK O — starting point: high KP, low KV ─────────────────────────
// Hypothesis: KP=28 is fine (accurate mocap positions), KV=3 avoids noise amplification.
// KP/KV are now runtime params (pos_gains.kp_xy/kp_z/kv_xy/kv_z in crazyflies.yaml).
// Defaults match the values below: kp_xy=28, kp_z=30, kv_xy=3, kv_z=7.
const KI_P: f32 = 0.05;   const KI_LIMIT: f32 = 2.0;
const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
const KW_X: f32 = 0.00110;const KW_Y: f32 = 0.00110;const KW_Z: f32 = 0.00138;
const KI_ATT: f32 = 0.0;

// ── v2 improvement flags ───────────────────────────────────────────────────
// All false = identical behaviour to active-development. Enable one at a time.
//
// 1. Position integral: accumulates XY/Z position error to correct steady-state offset.
//    KI_P and KI_LIMIT are already defined above (same values used when enabled or disabled;
//    when disabled the integral term is zeroed out below).
const ENABLE_POSITION_INTEGRAL: bool = false;
//
// 2. Attitude integral: accumulates SO(3) attitude error to correct steady-state tilt
//    from motor asymmetry or COM offset.  KI_ATT=0.03 (official Lee firmware default).
//    When disabled KI_ATT=0.0 above makes this a no-op anyway; flag kept for clarity.
const ENABLE_ATTITUDE_INTEGRAL: bool = false;
//
// 3. Butterworth pre-filter on the IMU accelerometer (same fc_bw as alpha chain).
//    Smooths a_meas in the position INDI outer loop, reducing noise in a_indi.
//    Only active when position INDI outer loop is enabled (ctrl_mode bit 0 = 1).
const ENABLE_ACC_PREFILTER: bool = false;
//
// 4. Tilt-compensated gravity: gz_comp = g / cos(θ) instead of g.
//    When the drone tilts, vertical thrust = T·cos(θ) — commanding g/cos(θ) pre-compensates
//    so altitude hold is not disturbed by tilt.  Clamped at cos(60°)=0.5 to prevent blow-up.
const ENABLE_TILT_GRAVITY_COMP: bool = true;

// ── MOCAP BLOCK P — if O wobbles: reduce KP to match lower KV ───────────────
// Reduce KP to keep zeta reasonable. Target: zeta ≈ 1.0 for KP=16, KV=2.83.
// Uncomment and comment out O if Block O still oscillates.
// const KP_X: f32 = 16.0;   const KP_Y: f32 = 16.0;   const KP_Z: f32 = 20.0;
// const KV_X: f32 = 3.0;    const KV_Y: f32 = 3.0;    const KV_Z: f32 = 5.0;
// const KI_P: f32 = 0.05;   const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
// const KW_X: f32 = 0.00110;const KW_Y: f32 = 0.00110;const KW_Z: f32 = 0.00138;
// const KI_ATT: f32 = 0.0;

// ── MOCAP BLOCK Q — if O is stable but tracks poorly: raise KP ──────────────
// If O is smooth but too sluggish (large XY error on figure-8), try KP=36, KV=4.
// const KP_X: f32 = 36.0;   const KP_Y: f32 = 36.0;   const KP_Z: f32 = 30.0;
// const KV_X: f32 = 4.0;    const KV_Y: f32 = 4.0;    const KV_Z: f32 = 7.0;
// const KI_P: f32 = 0.05;   const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
// const KW_X: f32 = 0.00110;const KW_Y: f32 = 0.00110;const KW_Z: f32 = 0.00138;
// const KI_ATT: f32 = 0.0;

// ── MOCAP BLOCK R — cut KV further: KV=2 ────────────────────────────────────
// O (KV=3) still wobbly → push KV even lower. KP stays high, attitude unchanged.
// const KP_X: f32 = 28.0;   const KP_Y: f32 = 28.0;   const KP_Z: f32 = 30.0;
// const KV_X: f32 = 2.0;    const KV_Y: f32 = 2.0;    const KV_Z: f32 = 5.0;
// const KI_P: f32 = 0.05;   const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
// const KW_X: f32 = 0.00110;const KW_Y: f32 = 0.00110;const KW_Z: f32 = 0.00138;
// const KI_ATT: f32 = 0.0;

// ── MOCAP BLOCK S — ★ BEST MOCAP RESULT SO FAR (2026-05-16) ★ ───────────────
// KV=2 + mellinger attitude gains → slightly better than R and O.
// Key finding: attitude gains KR=0.010 were contributing to wobble with mocap.
// KR=0.007, KW=0.00115 (mellinger defaults) proven stable on this exact setup.
// Position tracking likely still sluggish (KP=28, KV=2) — improve with Block T.
// const KP_X: f32 = 28.0;   const KP_Y: f32 = 28.0;   const KP_Z: f32 = 30.0;
// const KV_X: f32 = 2.0;    const KV_Y: f32 = 2.0;    const KV_Z: f32 = 5.0;
// const KI_P: f32 = 0.05;   const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.007;  const KR_Y: f32 = 0.007;  const KR_Z: f32 = 0.008;
// const KW_X: f32 = 0.00115;const KW_Y: f32 = 0.00115;const KW_Z: f32 = 0.002;
// const KI_ATT: f32 = 0.0;

// ── MOCAP BLOCK T — next session: push KP up, keep S attitude ───────────────
// S attitude gains are the mocap baseline. Now improve position tracking.
// Raise KP to 36, bring KV up slightly to 3 (ratio KP/KV²=4 → near-critical).
// If stable and tighter tracking: this becomes the new mocap baseline.
// const KP_X: f32 = 36.0;   const KP_Y: f32 = 36.0;   const KP_Z: f32 = 30.0;
// const KV_X: f32 = 3.0;    const KV_Y: f32 = 3.0;    const KV_Z: f32 = 6.0;
// const KI_P: f32 = 0.05;   const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.007;  const KR_Y: f32 = 0.007;  const KR_Z: f32 = 0.008;
// const KW_X: f32 = 0.00115;const KW_Y: f32 = 0.00115;const KW_Z: f32 = 0.002;
// const KI_ATT: f32 = 0.0;



// ── Controller State ───────────────────────────────────────────────────────
struct State {
    // Shared (both paths)
    i_ep: Vec3,
    i_error_att: Vec3,
    last_tick: u32,
    // INDI filter chain: raw omega → diff → BW → α_meas
    bw_x: Butterworth2, bw_y: Butterworth2, bw_z: Butterworth2,
    // INDI reference filter: α_ref → BW → α_ref_filt (phase alignment with α_meas)
    bw_ref_x: Butterworth2, bw_ref_y: Butterworth2, bw_ref_z: Butterworth2,
    // v2 improvement #3: accelerometer pre-filter for position INDI outer loop
    bw_acc_x: Butterworth2, bw_acc_y: Butterworth2, bw_acc_z: Butterworth2,
    omega_prev: Vec3,   // raw gyro from previous cycle (for finite-difference)
    tau_prev: Vec3,
    indi_init: bool,
    fc_bw_last: f32,
}

impl State {
    const fn zero() -> Self {
        Self {
            i_ep: Vec3::zero(), i_error_att: Vec3::zero(), last_tick: 0,
            bw_x: Butterworth2::zero(), bw_y: Butterworth2::zero(), bw_z: Butterworth2::zero(),
            bw_ref_x: Butterworth2::zero(), bw_ref_y: Butterworth2::zero(), bw_ref_z: Butterworth2::zero(),
            bw_acc_x: Butterworth2::zero(), bw_acc_y: Butterworth2::zero(), bw_acc_z: Butterworth2::zero(),
            omega_prev: Vec3::zero(),
            tau_prev: Vec3::zero(),
            indi_init: false,
            fc_bw_last: 0.0,
        }
    }
    fn reset(&mut self) {
        self.i_ep = Vec3::zero();
        self.i_error_att = Vec3::zero();
        self.last_tick = 0;
        self.bw_x.reset_state(); self.bw_y.reset_state(); self.bw_z.reset_state();
        self.bw_ref_x.reset_state(); self.bw_ref_y.reset_state(); self.bw_ref_z.reset_state();
        self.bw_acc_x.reset_state(); self.bw_acc_y.reset_state(); self.bw_acc_z.reset_state();
        self.omega_prev = Vec3::zero();
        self.tau_prev = Vec3::zero();
        self.indi_init = false;
    }
}

static mut CTRL: State = State::zero();

// ── INDI log variables (Mode 1) ────────────────────────────────────────────
// Variables owned by C (traj_iface.c); Rust writes via C function calls which
// are opaque to Rust LTO — values are always computed and stored correctly.
extern "C" {
    fn indi_log_write(arx: f32, ary: f32, arz: f32, ax: f32, ay: f32, az: f32);
    fn indi_tau_write(tx: f32, ty: f32, tz: f32);
}

// ── Onboard trajectory state ───────────────────────────────────────────────
// Set to current tick when traj.start=1 is received; cleared when mode→0.
// Declared separately from CTRL so the trajectory engine is independent of the
// geometric controller state machine.
static mut TRAJ_T0: u32 = 0;

// ── C trajectory interface (declared in firmware_app/traj_iface.c) ─────────
// Position layout: [duration, cx0..cx8, cy0..cy8] × n_segs (19 f32 per seg).
// Z layout:        [cz0..cz8]                      × n_segs ( 9 f32 per seg, no duration).
// Attitude layout: [croll0..croll8, cpitch0..cpitch8] × n_segs (18 f32 per seg).
const TRAJ_MAX_SEGS: usize = 12;
const TRAJ_FLOATS_PER_SEG:     usize = 19;
const TRAJ_Z_FLOATS_PER_SEG:   usize = 9;
const TRAJ_ATT_FLOATS_PER_SEG: usize = 18;

extern "C" {
    // ── Runtime-tunable INDI params (traj_iface.c PARAM_GROUP indi_gains) ────
    static mut g_controller_mode: u8;   // 0=geometric, 1=pos INDI, 2=att INDI, 3=full INDI
    static mut g_indi_kr:     f32;
    static mut g_indi_kw:     f32;
    static mut g_indi_kr_z:   f32;
    static mut g_indi_kw_z:   f32;
    // Per-motor thrust coefficients [N/RPM²]
    static mut g_indi_kt1:    f32;
    static mut g_indi_kt2:    f32;
    static mut g_indi_kt3:    f32;
    static mut g_indi_kt4:    f32;
    static mut g_indi_fc_bw:  f32;
    static mut g_indi_mass:   f32;
    // Runtime-tunable position gains (traj_iface.c PARAM_GROUP pos_gains)
    static mut g_kp_xy: f32;
    static mut g_kp_z:  f32;
    static mut g_kv_xy: f32;
    static mut g_kv_z:  f32;
}

extern "C" {
    static mut g_traj_coefs:     [f32; 228]; // TRAJ_MAX_SEGS * TRAJ_FLOATS_PER_SEG
    static mut g_traj_z_coefs:   [f32; 108]; // TRAJ_MAX_SEGS * TRAJ_Z_FLOATS_PER_SEG
    static mut g_traj_z_mode:    u8;  // 0=ramp (default), 1=polynomial (3D trajectories)
    static mut g_traj_z_ci:      u8;  // z upload index
    static mut g_traj_z_cv:      f32; // z upload value
    static mut g_traj_z_cw:      u8;  // z commit flag
    static mut g_traj_att_coefs:     [f32; 216]; // TRAJ_MAX_SEGS * TRAJ_ATT_FLOATS_PER_SEG
    static mut g_traj_att_mode:      u8;  // 0=flatness, 1=polynomial
    static mut g_traj_att_ctrl_mode: u8;  // 0=hard override, 1=hybrid (flatness rd + poly omega_d)
    static mut g_traj_att_ci:        u8;  // attitude upload index
    static mut g_traj_att_cv:        f32; // attitude upload value
    static mut g_traj_att_cw:        u8;  // attitude commit flag
    static mut g_traj_n_segs:    u8;
    static mut g_traj_mode:      u8;  // 0=passthrough (Mode B), 1=onboard eval (Mode D)
    static mut g_traj_start:     u8;  // laptop writes 1; firmware latches T0
    static mut g_traj_origin_x:  f32;
    static mut g_traj_origin_y:  f32;
    static mut g_traj_hover_z:   f32;
    static mut g_traj_dz:        f32; // Z gain per lap: 0=flat, +0.40=ascending helix
    static mut g_traj_coef_ci:   u8;  // position upload index
    static mut g_traj_coef_cv:   f32; // position upload value
    static mut g_traj_coef_cw:   u8;  // position commit flag
    // RPM bridge (traj_iface.c) — reads per-motor RPM via the log system for INDI Mode 1
    fn rpm_get_all(m1: *mut u16, m2: *mut u16, m3: *mut u16, m4: *mut u16);
}

// ── Spline evaluation helpers ───────────────────────────────────────────────
//
// Each segment stores a degree-8 polynomial for x and y separately, in normalised
// time t ∈ [0, 1] (τ/T where τ is local time, T is segment duration).
// Physical derivatives are recovered by dividing by Tⁿ:
//   pos  = p(t)
//   vel  = p'(t) / T
//   acc  = p''(t) / T²
//   jerk = p'''(t) / T³

/// Evaluate position, velocity, acceleration, jerk, and snap of a single axis
/// via Horner's method.
///
/// `c`     — 9 polynomial coefficients [c0 .. c8] in normalised time
/// `t`     — normalised time ∈ [0, 1]
/// `t_dur` — segment duration T [s]
///
/// Returns `(pos, vel/T, acc/T², jerk/T³, snap/T⁴)` in physical units.
#[inline]
fn poly_eval_axis(c: &[f32; 9], t: f32, t_dur: f32) -> (f32, f32, f32, f32, f32) {
    // Position  p(t) — Horner from highest degree
    let p = ((((((((c[8]) * t + c[7]) * t + c[6]) * t + c[5]) * t
             + c[4]) * t + c[3]) * t + c[2]) * t + c[1]) * t + c[0];

    // Velocity  p'(t) = c1 + 2c2·t + … + 8c8·t⁷
    let v = (((((((8.0*c[8]) * t + 7.0*c[7]) * t + 6.0*c[6]) * t
             + 5.0*c[5]) * t + 4.0*c[4]) * t + 3.0*c[3]) * t + 2.0*c[2]) * t + c[1];

    // Acceleration  p''(t) = 2c2 + 6c3·t + … + 56c8·t⁶
    let a = ((((((56.0*c[8]) * t + 42.0*c[7]) * t + 30.0*c[6]) * t
             + 20.0*c[5]) * t + 12.0*c[4]) * t + 6.0*c[3]) * t + 2.0*c[2];

    // Jerk  p'''(t) = 6c3 + 24c4·t + … + 336c8·t⁵
    let j = (((((336.0*c[8]) * t + 210.0*c[7]) * t + 120.0*c[6]) * t
             + 60.0*c[5]) * t + 24.0*c[4]) * t + 6.0*c[3];

    // Snap  p''''(t) = 24c4 + 120c5·t + 360c6·t² + 840c7·t³ + 1680c8·t⁴
    let sn = ((((1680.0*c[8]) * t + 840.0*c[7]) * t + 360.0*c[6]) * t
              + 120.0*c[5]) * t + 24.0*c[4];

    let inv_t  = 1.0 / t_dur;
    let inv_t2 = inv_t * inv_t;
    let inv_t3 = inv_t2 * inv_t;
    let inv_t4 = inv_t3 * inv_t;
    (p, v * inv_t, a * inv_t2, j * inv_t3, sn * inv_t4)
}

/// Evaluate the onboard spline at global trajectory time `t_global` [s].
///
/// # Safety
/// Caller must hold the `unsafe` context and ensure `n_segs ≤ TRAJ_MAX_SEGS`
/// and that the coefficients have been fully uploaded before T0 is latched.
///
/// Returns `(pos_rel, vel, acc, jerk, snap)` — pos_rel is relative to trajectory
/// origin (caller adds `g_traj_origin_{x,y}` and `g_traj_hover_z`).
unsafe fn eval_traj_onboard(t_global: f32, n_segs: usize) -> (Vec3, Vec3, Vec3, Vec3, Vec3) {
    // Compute total trajectory duration so we can wrap t for periodic reps.
    let mut total_dur = 0.0_f32;
    for i in 0..n_segs {
        total_dur += g_traj_coefs[i * TRAJ_FLOATS_PER_SEG];
    }
    // Wrap t into [0, total_dur) — handles reps=2+ without polynomial extrapolation.
    // The figure-8 is a closed loop (start == end), so wrapping is C0-continuous.
    let t = if total_dur > 0.0 {
        t_global - libm::floorf(t_global / total_dur) * total_dur
    } else {
        t_global
    };

    // Find active segment
    let mut t_start = 0.0_f32;
    let mut seg = n_segs.saturating_sub(1);
    for i in 0..n_segs {
        let dur = g_traj_coefs[i * TRAJ_FLOATS_PER_SEG];
        if t < t_start + dur {
            seg = i;
            break;
        }
        t_start += dur;
    }

    let base  = seg * TRAJ_FLOATS_PER_SEG;
    let dur   = g_traj_coefs[base];
    let tau   = (t - t_start).max(0.0).min(dur);
    let t_n   = tau / dur; // normalised ∈ [0, 1]

    // Extract coefficients into fixed-size arrays
    let mut cx = [0.0_f32; 9];
    let mut cy = [0.0_f32; 9];
    for k in 0..9 {
        cx[k] = g_traj_coefs[base + 1 + k];
        cy[k] = g_traj_coefs[base + 10 + k];
    }

    let (px, vx, ax, jx, sx) = poly_eval_axis(&cx, t_n, dur);
    let (py, vy, ay, jy, sy) = poly_eval_axis(&cy, t_n, dur);

    // Z: linear ramp using t_global (not wrapped t) so multi-rep helices stack
    // continuously upward instead of resetting each lap.
    // For n_reps=1, t_global ∈ [0, total_dur) → lap_frac ∈ [0,1) — identical to before.
    // When g_traj_dz=0 (circle, figure-8, loop), all z components are 0 → backward compatible.
    let lap_frac = if total_dur > 0.0 { t_global / total_dur } else { 0.0 };
    let vz = g_traj_dz / total_dur.max(0.001);

    (
        Vec3::new(px, py, lap_frac * g_traj_dz),
        Vec3::new(vx, vy, vz),
        Vec3::new(ax, ay, 0.0),
        Vec3::new(jx, jy, 0.0),
        Vec3::new(sx, sy, 0.0),
    )
}

/// Build rotation matrix from ZYX Euler angles (roll, pitch, yaw) in radians.
///
/// Returns R such that R · e₃ (body z in world) = desired thrust direction.
#[inline]
fn rot_from_euler(roll: f32, pitch: f32, yaw: f32) -> Mat3 {
    let (cr, sr) = (libm::cosf(roll),  libm::sinf(roll));
    let (cp, sp) = (libm::cosf(pitch), libm::sinf(pitch));
    let (cy, sy) = (libm::cosf(yaw),   libm::sinf(yaw));
    // ZYX: R = Rz(yaw) · Ry(pitch) · Rx(roll)  (row-major, body→world)
    [
        [ cp*cy,  sr*sp*cy - cr*sy,  cr*sp*cy + sr*sy],
        [ cp*sy,  sr*sp*sy + cr*cy,  cr*sp*sy - sr*cy],
        [-sp,     sr*cp,             cr*cp            ],
    ]
}

/// Evaluate roll and pitch polynomials for the active segment at normalised time τ.
///
/// Returns `(roll_d [rad], pitch_d [rad], omega_d [rad/s body-frame])`.
///
/// # Safety
/// Caller must hold the `unsafe` context.
#[inline]
unsafe fn eval_att_poly(t_global: f32, yaw_d: f32, n_segs: usize) -> (f32, f32, Vec3) {
    // Find the same active segment as eval_traj_onboard.
    let mut total_dur = 0.0_f32;
    for i in 0..n_segs {
        total_dur += g_traj_coefs[i * TRAJ_FLOATS_PER_SEG];
    }
    let t = if total_dur > 0.0 {
        t_global - libm::floorf(t_global / total_dur) * total_dur
    } else {
        t_global
    };
    let mut t_start = 0.0_f32;
    let mut seg = n_segs.saturating_sub(1);
    for i in 0..n_segs {
        let dur = g_traj_coefs[i * TRAJ_FLOATS_PER_SEG];
        if t < t_start + dur { seg = i; break; }
        t_start += dur;
    }
    let dur   = g_traj_coefs[seg * TRAJ_FLOATS_PER_SEG];
    let tau   = ((t - t_start).max(0.0).min(dur)) / dur; // ∈ [0,1]

    let base = seg * TRAJ_ATT_FLOATS_PER_SEG;
    let mut cr = [0.0_f32; 9];
    let mut cp = [0.0_f32; 9];
    for k in 0..9 {
        cr[k] = g_traj_att_coefs[base + k];
        cp[k] = g_traj_att_coefs[base + 9 + k];
    }

    let (roll_d,  roll_dot,  _, _, _) = poly_eval_axis(&cr, tau, dur);
    let (pitch_d, pitch_dot, _, _, _) = poly_eval_axis(&cp, tau, dur);

    // Body-frame ω from ZYX Euler rates (yaw_dot = 0 for all flat trajectories)
    let cr_ = libm::cosf(roll_d);
    let sr_ = libm::sinf(roll_d);
    let omega_x =  roll_dot;
    let omega_y =  pitch_dot * cr_;
    let omega_z = -pitch_dot * sr_;
    let _ = yaw_d; // yaw_dot = 0, no yaw contribution
    (roll_d, pitch_d, Vec3::new(omega_x, omega_y, omega_z))
}

/// Evaluate the Z-axis polynomial at `t_global` [s] for 3D trajectories (z_mode=1).
///
/// Uses the same segment-finding logic as `eval_traj_onboard` (reads durations from
/// the position coefficient buffer) but evaluates `g_traj_z_coefs` for the Z axis.
///
/// Returns `(pz [m], vz [m/s], az [m/s²], jz [m/s³], sz [m/s⁴])` — all relative to `hover_z`.
///
/// # Safety
/// Caller must hold the `unsafe` context.
#[inline]
unsafe fn eval_z_poly(t_global: f32, n_segs: usize) -> (f32, f32, f32, f32, f32) {
    let mut total_dur = 0.0_f32;
    for i in 0..n_segs {
        total_dur += g_traj_coefs[i * TRAJ_FLOATS_PER_SEG];
    }
    let t = if total_dur > 0.0 {
        t_global - libm::floorf(t_global / total_dur) * total_dur
    } else {
        t_global
    };
    let mut t_start = 0.0_f32;
    let mut seg = n_segs.saturating_sub(1);
    for i in 0..n_segs {
        let dur = g_traj_coefs[i * TRAJ_FLOATS_PER_SEG];
        if t < t_start + dur { seg = i; break; }
        t_start += dur;
    }
    let dur = g_traj_coefs[seg * TRAJ_FLOATS_PER_SEG];
    let tau = ((t - t_start).max(0.0).min(dur)) / dur.max(1e-9);

    let base = seg * TRAJ_Z_FLOATS_PER_SEG;
    let mut cz = [0.0_f32; 9];
    for k in 0..9 {
        cz[k] = g_traj_z_coefs[base + k];
    }
    let (pz, vz, az, jz, sz) = poly_eval_axis(&cz, tau, dur);
    (pz, vz, az, jz, sz)
}

/// Compute desired body angular velocity ω_d from flatness (yaw=const → ψ̇=0).
///
/// From Faessler et al. 2018, Appendix A, simplified for ψ̇ = 0:
///   ωx = −(yb · jerk) / c
///   ωy =  (xb · jerk) / c
///   ωz = 0
/// where `c = |acc + g·ez|` and `xb`, `yb` are the desired body x/y axes.
#[inline]
fn omega_desired(acc: Vec3, jerk: Vec3, yaw: f32) -> Vec3 {
    let cos_psi = libm::cosf(yaw);
    let sin_psi = libm::sinf(yaw);
    let yc = Vec3::new(-sin_psi, cos_psi, 0.0);

    let acc_g = Vec3::new(acc.x, acc.y, acc.z + GRAVITY);
    let c = acc_g.norm();
    if c < 0.1 { return Vec3::zero(); }

    // Body axes from flatness (same construction as desired_rot)
    let xb = yc.cross(acc_g).normalize();
    let yb = acc_g.cross(xb).normalize();

    let omega_x = -yb.dot(jerk) / c;
    let omega_y =  xb.dot(jerk) / c;
    // omega_z = 0 because yaw_dot = 0 for this trajectory
    Vec3::new(omega_x, omega_y, 0.0)
}

/// Compute desired body angular acceleration α_des from snap via differential flatness.
///
/// Time derivative of omega_desired with ψ̇ = 0 (constant yaw). Derivation:
///   ẏb = ωx·b3, so ẏb·j = ωx·ċ;  ẋb = -ωy·b3, so ẋb·j = -ωy·ċ
///   ċ = b3·j (rate of change of thrust magnitude)
/// Substituting ωx = -(yb·j)/c and ωy = (xb·j)/c yields Eq. (15) of Tal & Karaman 2020:
///   αx = 2(yb·j)(b3·j)/c² − (yb·s)/c
///   αy = (xb·s)/c − 2(xb·j)(b3·j)/c²
///   αz = 0
#[inline]
fn alpha_desired(acc: Vec3, jerk: Vec3, snap: Vec3, yaw: f32) -> Vec3 {
    let cos_psi = libm::cosf(yaw);
    let sin_psi = libm::sinf(yaw);
    let yc = Vec3::new(-sin_psi, cos_psi, 0.0);

    let acc_g = Vec3::new(acc.x, acc.y, acc.z + GRAVITY);
    let c = acc_g.norm();
    if c < 0.1 { return Vec3::zero(); }

    let xb    = yc.cross(acc_g).normalize();
    let yb    = acc_g.cross(xb).normalize();
    let b3    = acc_g.scale(1.0 / c);
    let c_dot = b3.dot(jerk);   // ċ = b3·j

    let inv_c  = 1.0 / c;
    let inv_c2 = inv_c * inv_c;

    Vec3::new(
        2.0 * yb.dot(jerk) * c_dot * inv_c2 - yb.dot(snap) * inv_c,
        xb.dot(snap) * inv_c - 2.0 * xb.dot(jerk) * c_dot * inv_c2,
        0.0,
    )
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

// ── INDI helpers ───────────────────────────────────────────────────────────

// Inverse of powerDistributionForceTorque (power_distribution_quadrotor.c):
//   τ_x = arm × (F[2]+F[3]−F[0]−F[1]),  arm = √2/2 × ARM_LENGTH
//   τ_y = arm × (F[1]+F[2]−F[0]−F[3])
//   τ_z = (k_Q/k_T) × (F[1]+F[3]−F[0]−F[2])
fn rpms_to_torque(rpms: [u16; 4], kts: [f32; 4]) -> Vec3 {
    let f = [
        kts[0] * (rpms[0] as f32) * (rpms[0] as f32),
        kts[1] * (rpms[1] as f32) * (rpms[1] as f32),
        kts[2] * (rpms[2] as f32) * (rpms[2] as f32),
        kts[3] * (rpms[3] as f32) * (rpms[3] as f32),
    ];
    Vec3::new(
        ARM_M        * (f[2] + f[3] - f[0] - f[1]),
        ARM_M        * (f[1] + f[2] - f[0] - f[3]),
        TORQUE_RATIO * (f[1] + f[3] - f[0] - f[2]),
    )
}

// ── Reference geometric controller (preserved — identical to the geometric path
//    inside controller_step when mode=0; kept as dead code for fallback reference) ──
#[allow(dead_code)]
fn geometric_step_ref(
    pos: Vec3, vel: Vec3, r: &Mat3, omega: Vec3,
    pd: Vec3, vd: Vec3, ad: Vec3, yaw_d: f32,
    omega_d: Vec3,
    rd_override: Option<&Mat3>,
    dt: f32, s: &mut State,
) -> (f32, Vec3) {
    let ep = pd.sub(pos);
    let ev = vd.sub(vel);
    s.i_ep = s.i_ep.add(ep.scale(dt));
    s.i_ep = Vec3::new(
        s.i_ep.x.clamp(-KI_LIMIT, KI_LIMIT),
        s.i_ep.y.clamp(-KI_LIMIT, KI_LIMIT),
        s.i_ep.z.clamp(-KI_LIMIT, KI_LIMIT),
    );
    let (kp_xy, kp_z, kv_xy, kv_z) = unsafe { (g_kp_xy, g_kp_z, g_kv_xy, g_kv_z) };
    let f_d = ad
        .add(Vec3::new(kp_xy*ep.x, kp_xy*ep.y, kp_z*ep.z))
        .add(Vec3::new(kv_xy*ev.x, kv_xy*ev.y, kv_z*ev.z))
        .add(Vec3::new(KI_P*s.i_ep.x, KI_P*s.i_ep.y, KI_P*s.i_ep.z))
        .add(Vec3::new(0.0, 0.0, GRAVITY));
    let thrust_vec = f_d.scale(unsafe { g_indi_mass });
    let body_z = Vec3::new(r[0][2], r[1][2], r[2][2]);
    let thrust = thrust_vec.dot(body_z).max(0.0);
    if thrust < 0.05 { s.i_ep = Vec3::zero(); s.i_error_att = Vec3::zero(); }
    let rd_flatness;
    let rd: &Mat3 = match rd_override {
        Some(ro) => ro,
        None => { rd_flatness = desired_rot(thrust_vec, yaw_d); &rd_flatness }
    };
    let er = vee_half(&matsub(&mat_at_b(rd, r), &mat_at_b(r, rd)));
    s.i_error_att = s.i_error_att.add(er.scale(dt));
    let e_omega   = omega.sub(omega_d);
    let j_omega   = Vec3::new(JXX*omega.x, JYY*omega.y, JZZ*omega.z);
    let gyro_comp = omega.cross(j_omega);
    // v2 improvement #2: attitude integral (0.03 = official Lee default; 0.0 = disabled)
    let ki_att = if ENABLE_ATTITUDE_INTEGRAL { 0.03_f32 } else { 0.0_f32 };
    let torque = Vec3::new(
        -KR_X*er.x - KW_X*e_omega.x + gyro_comp.x - ki_att*s.i_error_att.x,
        -KR_Y*er.y - KW_Y*e_omega.y + gyro_comp.y - ki_att*s.i_error_att.y,
        -KR_Z*er.z - KW_Z*e_omega.z + gyro_comp.z - ki_att*s.i_error_att.z,
    );
    (thrust, torque)
}

/// Unified controller step (Tal & Karaman 2020 / Lee et al. 2010).
///
/// mode bits:
///   bit 0 = position INDI outer loop: replaces model acceleration with measured
///   bit 1 = attitude INDI inner loop: incremental torque from RPM^2 + alpha feedback
///
/// Gyroscopic compensation applied only when bit 1 is clear (geometric attitude).
/// RPMs are read once and shared between position INDI (a_model) and att INDI (tau_current).
fn controller_step(
    mode: u8,
    pos: Vec3, vel: Vec3, r: &Mat3, omega: Vec3,
    acc_body: Vec3,   // IMU accelerometer in g units, body frame
    pd: Vec3, vd: Vec3, ad: Vec3, yaw_d: f32,
    omega_d: Vec3, alpha_des: Vec3,
    rd_override: Option<&Mat3>,
    dt: f32, s: &mut State,
) -> (f32, Vec3) {
    // -- Runtime params -------------------------------------------------------
    let kr_xy = unsafe { g_indi_kr };
    let kw_xy = unsafe { g_indi_kw };
    let kr_z  = unsafe { g_indi_kr_z };
    let kw_z  = unsafe { g_indi_kw_z };
    let mass  = unsafe { g_indi_mass };
    let kt1   = unsafe { g_indi_kt1 };
    let kt2   = unsafe { g_indi_kt2 };
    let kt3   = unsafe { g_indi_kt3 };
    let kt4   = unsafe { g_indi_kt4 };

    // -- Read RPMs once (shared by pos INDI a_model and att INDI tau_current) -
    let mut m1 = 0u16; let mut m2 = 0u16; let mut m3 = 0u16; let mut m4 = 0u16;
    if mode != 0 {
        unsafe { rpm_get_all(&mut m1, &mut m2, &mut m3, &mut m4); }
    }
    let rpms_active = (m1 | m2 | m3 | m4) > 0;

    // -- Position loop --------------------------------------------------------
    let ep = pd.sub(pos);
    let ev = vd.sub(vel);
    s.i_ep = s.i_ep.add(ep.scale(dt));
    s.i_ep = Vec3::new(
        s.i_ep.x.clamp(-KI_LIMIT, KI_LIMIT),
        s.i_ep.y.clamp(-KI_LIMIT, KI_LIMIT),
        s.i_ep.z.clamp(-KI_LIMIT, KI_LIMIT),
    );

    // Position INDI: a_indi = a_meas - a_model (world frame)
    let a_indi = if mode & 1 != 0 && rpms_active {
        let f_total = kt1*(m1 as f32)*(m1 as f32)
                    + kt2*(m2 as f32)*(m2 as f32)
                    + kt3*(m3 as f32)*(m3 as f32)
                    + kt4*(m4 as f32)*(m4 as f32);
        let body_z_w = Vec3::new(r[0][2], r[1][2], r[2][2]);
        // a_model: net thrust acceleration in world frame
        let a_model = body_z_w.scale(f_total / mass).add(Vec3::new(0.0, 0.0, -GRAVITY));
        // a_meas: rotate body specific force to world, add gravity
        let a_meas  = mat_mul_vec(r, acc_body.scale(GRAVITY)).add(Vec3::new(0.0, 0.0, -GRAVITY));
        a_meas.sub(a_model)
    } else {
        Vec3::zero()
    };

    let (kp_xy, kp_z, kv_xy, kv_z) = unsafe { (g_kp_xy, g_kp_z, g_kv_xy, g_kv_z) };
    // v2 improvement #1: position integral (zero term when disabled)
    let ki_term = if ENABLE_POSITION_INTEGRAL {
        Vec3::new(KI_P*s.i_ep.x, KI_P*s.i_ep.y, KI_P*s.i_ep.z)
    } else {
        Vec3::zero()
    };
    // v2 improvement #4: tilt-compensated gravity (g/cos θ instead of g)
    let gz_comp = if ENABLE_TILT_GRAVITY_COMP {
        GRAVITY / r[2][2].max(0.5_f32)  // r[2][2] = cos(θ); clamped at cos(60°)=0.5
    } else {
        GRAVITY
    };
    let f_d = ad
        .add(Vec3::new(kp_xy*ep.x, kp_xy*ep.y, kp_z*ep.z))
        .add(Vec3::new(kv_xy*ev.x, kv_xy*ev.y, kv_z*ev.z))
        .add(ki_term)
        .add(Vec3::new(0.0, 0.0, gz_comp))
        .add(a_indi);
    let thrust_vec = f_d.scale(mass);
    let body_z = Vec3::new(r[0][2], r[1][2], r[2][2]);
    let thrust = thrust_vec.dot(body_z).max(0.0);
    if thrust < 0.05 {
        s.i_ep = Vec3::zero();
        s.i_error_att = Vec3::zero();
    }

    // -- Desired rotation -----------------------------------------------------
    let rd_flatness;
    let rd: &Mat3 = match rd_override {
        Some(ro) => ro,
        None => { rd_flatness = desired_rot(thrust_vec, yaw_d); &rd_flatness }
    };

    // -- Attitude error eR = 0.5*(Rd^T R - R^T Rd)^vee ------------------------
    let er = vee_half(&matsub(&mat_at_b(rd, r), &mat_at_b(r, rd)));
    s.i_error_att = s.i_error_att.add(er.scale(dt));

    // -- Passive filter: runs in ALL modes for filter characterisation logging --
    // alpha_raw and alpha_meas are always computed and logged; in geometric mode
    // (mode & 2 == 0) they are not used for torque but are visible in indi_state log.
    if !s.indi_init {
        s.omega_prev = omega;
        s.indi_init = true;
    }
    let inv_dt = if dt > 1e-9 { 1.0 / dt } else { 500.0 };
    let alpha_raw  = omega.sub(s.omega_prev).scale(inv_dt);
    let alpha_meas = Vec3::new(
        s.bw_x.update(alpha_raw.x),
        s.bw_y.update(alpha_raw.y),
        s.bw_z.update(alpha_raw.z),
    );
    s.omega_prev = omega;
    unsafe { indi_log_write(alpha_raw.x, alpha_raw.y, alpha_raw.z,
                             alpha_meas.x, alpha_meas.y, alpha_meas.z); }

    // -- Attitude law ---------------------------------------------------------
    let torque = if mode & 2 != 0 {
        // INDI attitude path — reuses alpha_raw/alpha_meas computed above

        // alpha_ref = alpha_des - KR*eR - KW*e_omega  (Tal & Karaman Eq. 28)
        let e_omega = omega.sub(omega_d);
        let alpha_ref = Vec3::new(
            alpha_des.x - kr_xy*er.x - kw_xy*e_omega.x,
            alpha_des.y - kr_xy*er.y - kw_xy*e_omega.y,
            alpha_des.z - kr_z *er.z - kw_z *e_omega.z,
        );

        // Same BW on alpha_ref -> alpha_ref_filt (phase alignment with alpha_meas)
        let alpha_ref_filt = Vec3::new(
            s.bw_ref_x.update(alpha_ref.x),
            s.bw_ref_y.update(alpha_ref.y),
            s.bw_ref_z.update(alpha_ref.z),
        );

        // tau_current from per-motor RPM^2 (falls back to tau_prev when deck absent)
        let tau_current = if rpms_active {
            rpms_to_torque([m1, m2, m3, m4], [kt1, kt2, kt3, kt4])
        } else {
            s.tau_prev
        };

        // delta_tau = J*(alpha_ref_filt - alpha_meas),  tau = tau_current + delta_tau
        let alpha_err = alpha_ref_filt.sub(alpha_meas);
        let delta_tau = Vec3::new(JXX*alpha_err.x, JYY*alpha_err.y, JZZ*alpha_err.z);
        let tau       = tau_current.add(delta_tau);
        s.tau_prev    = tau;

        unsafe { indi_tau_write(tau.x, tau.y, tau.z); }

        tau
    } else {
        // Geometric attitude path
        let e_omega   = omega.sub(omega_d);
        let j_omega   = Vec3::new(JXX*omega.x, JYY*omega.y, JZZ*omega.z);
        let gyro_comp = omega.cross(j_omega);
        // v2 improvement #2: attitude integral (same gate as geometric_step path)
        let ki_att = if ENABLE_ATTITUDE_INTEGRAL { 0.03_f32 } else { 0.0_f32 };
        Vec3::new(
            -KR_X*er.x - KW_X*e_omega.x + gyro_comp.x - ki_att*s.i_error_att.x,
            -KR_Y*er.y - KW_Y*e_omega.y + gyro_comp.y - ki_att*s.i_error_att.y,
            -KR_Z*er.z - KW_Z*e_omega.z + gyro_comp.z - ki_att*s.i_error_att.z,
        )
    };

    (thrust, torque)
}

// ── Firmware entry points ──────────────────────────────────────────────────
#[no_mangle]
pub extern "C" fn controllerOutOfTreeInit() {
    unsafe {
        let s = &mut *core::ptr::addr_of_mut!(CTRL);
        s.reset();
        // Pre-compute INDI filter coefficients for 500 Hz nominal rate using
        // current C globals (may already be set via param before init runs).
        const DT: f32 = 0.002_f32;
        let fc_bw = g_indi_fc_bw;
        s.bw_x.init(fc_bw, DT);     s.bw_y.init(fc_bw, DT);     s.bw_z.init(fc_bw, DT);
        s.bw_ref_x.init(fc_bw, DT); s.bw_ref_y.init(fc_bw, DT); s.bw_ref_z.init(fc_bw, DT);
        s.bw_acc_x.init(fc_bw, DT); s.bw_acc_y.init(fc_bw, DT); s.bw_acc_z.init(fc_bw, DT);
        s.fc_bw_last = fc_bw;
    }
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

    // Reinit BW filters when fc_bw changes at runtime (at most once per param write)
    const DT_NOM: f32 = 0.002_f32;
    let fc_bw = g_indi_fc_bw;
    if (fc_bw - s.fc_bw_last).abs() > 0.1 {
        s.bw_x.init(fc_bw, DT_NOM);     s.bw_y.init(fc_bw, DT_NOM);     s.bw_z.init(fc_bw, DT_NOM);
        s.bw_ref_x.init(fc_bw, DT_NOM); s.bw_ref_y.init(fc_bw, DT_NOM); s.bw_ref_z.init(fc_bw, DT_NOM);
        s.bw_acc_x.init(fc_bw, DT_NOM); s.bw_acc_y.init(fc_bw, DT_NOM); s.bw_acc_z.init(fc_bw, DT_NOM);
        s.fc_bw_last = fc_bw;
    }

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

    // Accelerometer in g units, body frame — used by position INDI outer loop.
    // v2 improvement #3: optionally pre-filter with same Butterworth as alpha chain.
    let acc = &(*sensors).acc;
    let acc_body = if ENABLE_ACC_PREFILTER {
        Vec3::new(
            s.bw_acc_x.update(acc.axis[0]),
            s.bw_acc_y.update(acc.axis[1]),
            s.bw_acc_z.update(acc.axis[2]),
        )
    } else {
        Vec3::new(acc.axis[0], acc.axis[1], acc.axis[2])
    };

    let sp = &*setpoint;

    // ── Coefficient upload handlers ────────────────────────────────────────
    // Position / Z / Attitude: laptop writes idx, value, commit=1; we store and clear.
    // All run at 500 Hz → max latency 2 ms, well within the radio round-trip.
    if g_traj_coef_cw != 0 {
        let ci = g_traj_coef_ci as usize;
        if ci < TRAJ_MAX_SEGS * TRAJ_FLOATS_PER_SEG {
            g_traj_coefs[ci] = g_traj_coef_cv;
        }
        g_traj_coef_cw = 0;
    }
    if g_traj_z_cw != 0 {
        let zci = g_traj_z_ci as usize;
        if zci < TRAJ_MAX_SEGS * TRAJ_Z_FLOATS_PER_SEG {
            g_traj_z_coefs[zci] = g_traj_z_cv;
        }
        g_traj_z_cw = 0;
    }
    if g_traj_att_cw != 0 {
        let aci = g_traj_att_ci as usize;
        if aci < TRAJ_MAX_SEGS * TRAJ_ATT_FLOATS_PER_SEG {
            g_traj_att_coefs[aci] = g_traj_att_cv;
        }
        g_traj_att_cw = 0;
    }

    // ── Mode D: onboard trajectory eval ───────────────────────────────────
    // When g_traj_mode == 1 and traj.start == 1, latch T0 on the first tick,
    // then derive the position/velocity/acceleration/omega reference locally from
    // the uploaded spline — no radio setpoints needed during the manoeuvre.
    // When g_traj_mode == 0, reset T0 so the next Mode D activation gets a fresh start.
    if g_traj_mode == 0 {
        TRAJ_T0 = 0;
    } else if g_traj_mode == 1 && g_traj_start == 1 && TRAJ_T0 == 0 {
        TRAJ_T0 = tick; // latch start time
    }

    let (pd, vd, ad, omega_d, alpha_des) = if g_traj_mode == 1 && TRAJ_T0 > 0 {
        // ── Mode D: onboard trajectory eval ───────────────────────────────
        // All trajectory types (flat, helix, loop) — Z source selected by z_mode:
        //   z_mode=0: linear ramp via traj.dz (circle, figure-8, helix)
        //   z_mode=1: degree-8 polynomial from g_traj_z_coefs (loop, any 3D path)
        let t = tick.wrapping_sub(TRAJ_T0) as f32 * 0.001_f32;
        let n_segs = (g_traj_n_segs as usize).min(TRAJ_MAX_SEGS);
        let (pos_rel, vel, acc, jerk, snap) = eval_traj_onboard(t, n_segs);

        let ox = g_traj_origin_x;
        let oy = g_traj_origin_y;
        let hz = g_traj_hover_z;

        // Z: polynomial override (3D) or linear ramp (flat/helix)
        let (pz, vz, az, jz, sz) = if g_traj_z_mode == 1 {
            eval_z_poly(t, n_segs)
        } else {
            (pos_rel.z, vel.z, acc.z, jerk.z, snap.z)
        };

        let pd = Vec3::new(ox + pos_rel.x, oy + pos_rel.y, hz + pz);
        let vd = Vec3::new(vel.x, vel.y, vz);
        let ad = Vec3::new(acc.x, acc.y, az);
        let jerk_3d = Vec3::new(jerk.x, jerk.y, jz);
        let snap_3d = Vec3::new(snap.x, snap.y, sz);
        // ω_d feedforward: include Z jerk when z_mode=1 (non-zero for 3D paths).
        let omega_d  = omega_desired(ad, jerk_3d, 0.0_f32);
        // α_des feedforward: snap → Ω̇_ref via flatness (Tal & Karaman Eq. 15).
        let alpha_des = alpha_desired(ad, jerk_3d, snap_3d, 0.0_f32);
        (pd, vd, ad, omega_d, alpha_des)
    } else {
        // Mode B passthrough: read full feedforward from CRTP setpoint.
        // Crazyflie HLC populates pos/vel/acc/jerk/snap/attitudeRate from poly4d_eval —
        // we use them all here to get the complete Tal & Karaman feedforward through CS2.
        let pd = Vec3::new(sp.position.x, sp.position.y, sp.position.z);
        let vd = Vec3::new(sp.velocity.x, sp.velocity.y, sp.velocity.z);
        let ad = Vec3::new(sp.acceleration.x, sp.acceleration.y, sp.acceleration.z);
        let jerk = Vec3::new(sp.jerk.x, sp.jerk.y, sp.jerk.z);
        let snap = Vec3::new(sp.snap.x, sp.snap.y, sp.snap.z);
        // ω_d from HLC's flatness-derived body rates (deg/s → rad/s)
        let omega_d = Vec3::new(
            sp.attitudeRate.roll  * deg2rad,
            sp.attitudeRate.pitch * deg2rad,
            sp.attitudeRate.yaw   * deg2rad,
        );
        // α_des via differential flatness (Tal & Karaman Eq. 15) using HLC's jerk + snap
        let yaw_d_local = sp.attitude.yaw * deg2rad;
        let alpha_des = alpha_desired(ad, jerk, snap, yaw_d_local);
        (pd, vd, ad, omega_d, alpha_des)
    };

    // SAFE ARMING: only enable controller when test harness sends real position
    // (z > 0.05 m). In Mode D the laptop sends a hover keepalive, so z is always
    // set and this check passes naturally.
    let armed = sp.position.z > 0.05_f32;

    let yaw_d = sp.attitude.yaw * deg2rad;

    // Attitude polynomial dispatch (att_mode=1 only, trajectory must be active).
    //
    // att_ctrl_mode=0 (hard override):
    //   rd = polynomial(t) — ignores position-loop feedback in rotation command.
    //   omega_d = polynomial angular velocity derivative.
    //   Required for loop/flip where flatness is undefined at zero-thrust events.
    //
    // att_ctrl_mode=1 (hybrid, default):
    //   rd = desired_rot(f_d, yaw) — derived from total feedback force (position tracking
    //   errors always reflected in the rotation command → globally stable position loop).
    //   omega_d = polynomial angular velocity derivative — pre-planned, smooth feedforward.
    //   Recommended for all flat trajectories (circle, figure-8, helix, corner).
    let is_traj_active = (g_traj_mode == 1 || g_traj_mode == 2) && TRAJ_T0 > 0;
    let (rd_poly_storage, omega_d_poly, use_poly) = if g_traj_att_mode == 1 && is_traj_active {
        let t = tick.wrapping_sub(TRAJ_T0) as f32 * 0.001_f32;
        let n_segs = (g_traj_n_segs as usize).min(TRAJ_MAX_SEGS);
        let (roll_d, pitch_d, od) = eval_att_poly(t, yaw_d, n_segs);
        (rot_from_euler(roll_d, pitch_d, yaw_d), od, true)
    } else {
        ([[0.0f32; 3]; 3], Vec3::zero(), false)
    };
    let (omega_d_final, rd_override) = if use_poly {
        if g_traj_att_ctrl_mode == 0 {
            // Hard override: polynomial sets both rd and omega_d.
            (omega_d_poly, Some(&rd_poly_storage))
        } else {
            // Hybrid: polynomial provides omega_d feedforward only; rd comes from flatness.
            (omega_d_poly, None)
        }
    } else {
        (omega_d, None)
    };

    let mode = g_controller_mode;
    let (thrust_si, torque) = controller_step(
        mode, pos, vel, &r, omega, acc_body,
        pd, vd, ad, yaw_d, omega_d_final, alpha_des, rd_override, dt, s,
    );

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