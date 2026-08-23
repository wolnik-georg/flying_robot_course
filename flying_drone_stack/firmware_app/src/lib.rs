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

mod residual_nn;
use residual_nn::ResidualNet;

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

// ── Optional stage-2 filter: 2nd-order notch (band-reject), runtime-tunable ──
// Added 2026-07-28: the shake investigation's own final conclusion (500 Hz SD-log
// analysis, docs/results_2026-07-15_brushless.md §16.6) found the 5-8 Hz shake is a
// genuine NARROW spectral peak (7.22 Hz measured), not broadband, and that the
// existing Butterworth is set way above it (fc_bw=60-70 Hz, |filt/raw|=1.000 at
// 7.22 Hz measured directly) -- lowering fc_bw down to that band was tried and made
// things 3-4x worse (adds phase lag everywhere, not just at the shake frequency).
// This filter targets ONLY the diagnosed band, leaving the rest of the spectrum --
// and its phase -- untouched by construction (that's what "notch" means). RBJ
// Audio-EQ-Cookbook band-reject biquad, Direct Form I. f0 (center) and bandwidth
// are runtime params (indi_gains.notch_f0/notch_bw) so this can be re-tuned without
// reflashing if the shake frequency drifts session-to-session (6.3-7.9 Hz measured
// across different sessions in the prior investigation).
#[derive(Copy, Clone)]
struct NotchFilter {
    b0: f32, b1: f32, b2: f32, a1: f32, a2: f32,
    x1: f32, x2: f32, y1: f32, y2: f32,
}
impl NotchFilter {
    const fn zero() -> Self {
        Self { b0: 1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0, x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0 }
    }
    /// f0: notch center [Hz]. bw: notch bandwidth [Hz] (Q = f0/bw -- e.g. f0=7.2,
    /// bw=5.0 -> Q=1.44, -3dB points near 5 and 10 Hz, matching the diagnosed band).
    fn init(&mut self, f0: f32, bw: f32, dt: f32) {
        let w0 = 2.0 * core::f32::consts::PI * f0 * dt;
        let q = (f0 / bw.max(0.1)).max(0.1);
        let alpha = libm::sinf(w0) / (2.0 * q);
        let cos_w0 = libm::cosf(w0);
        let a0 = 1.0 + alpha;
        self.b0 =  1.0        / a0;
        self.b1 = -2.0*cos_w0 / a0;
        self.b2 =  1.0        / a0;
        self.a1 = -2.0*cos_w0 / a0;
        self.a2 = (1.0 - alpha) / a0;
    }
    fn update(&mut self, x: f32) -> f32 {
        let y = self.b0*x + self.b1*self.x1 + self.b2*self.x2
              - self.a1*self.y1 - self.a2*self.y2;
        self.x2 = self.x1; self.x1 = x;
        self.y2 = self.y1; self.y1 = y;
        y
    }
    fn reset_state(&mut self) { self.x1 = 0.0; self.x2 = 0.0; self.y1 = 0.0; self.y2 = 0.0; }
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

// Inertia (from official firmware / System ID).
// CF2.1 standard / upgraded (default — unchanged).
#[cfg(not(drone_bl))]
const JXX: f32 = 16.571710e-6;
#[cfg(not(drone_bl))]
const JYY: f32 = 16.655602e-6;
#[cfg(not(drone_bl))]
const JZZ: f32 = 29.261652e-6;
// CF2.1 Brushless (CF21BL) — from Busetto et al. 2025 (arXiv:2512.14450), Table in Sec 4.1.
// J = diag(Jx,Jy,Jz), measured on same CF21BL platform (Flow-deck v2 + AI-deck, 45 g total).
#[cfg(drone_bl)]
const JXX: f32 = 23.951e-6;  // 2.3951e-5 kg·m²
#[cfg(drone_bl)]
const JYY: f32 = 23.951e-6;  // 2.3951e-5 kg·m²
#[cfg(drone_bl)]
const JZZ: f32 = 32.347e-6;  // 3.2347e-5 kg·m²

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
// Diagonal arm projection √2/2 × arm_length, and k_Q/k_T = THRUST2TORQUE [m].
// CF2.1 standard / upgraded (default — unchanged): arm 0.046 m.
#[cfg(not(drone_bl))]
const ARM_M: f32      = 0.032_526_9_f32;
#[cfg(not(drone_bl))]
const TORQUE_RATIO: f32 = 0.005_964_552_f32;
// CF2.1 Brushless (CF21BL): arm from Figure 2 (paper).
#[cfg(drone_bl)]
const ARM_M: f32      = 0.035_355_3_f32; // √2/2 × 0.050 m — confirmed from paper Fig 2
// TORQUE_RATIO (2026-07-18, fixed): matches the ACTUAL compiled stock-mixer THRUST2TORQUE for
// CF21BL (platform_defaults_cf21bl.h, CONFIG_ENABLE_THRUST_BAT_COMPENSATED=y — confirmed against
// the generated .config), not the Busetto et al. 2025 paper's independently-measured kM/kF
// (0.0020779). rpms_to_torque() estimates the CURRENT yaw torque from measured RPM² to be used
// as tau_current in the INDI increment; that estimate must be self-consistent with whatever
// conversion the stock power_distribution_quadrotor.c mixer actually used to command it, not
// with an independently-derived aerodynamic ratio that ignores ESC/prop-efficiency losses baked
// into Bitcraze's own calibration for this exact hardware.
#[cfg(drone_bl)]
const TORQUE_RATIO: f32 = 0.005_692_788_4_f32; // THRUST2TORQUE, platform_defaults_cf21bl.h

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

// ── MOCAP BLOCK O — CF2.1 STANDARD MOTORS — locked Jul 2 2026 ────────────────
// Final result: Geom RMSE 2.0 cm (kt=0.05), speed limit kt=0.15 (5.32 s/lap).
// KP/KV live in crazyflies.yaml: kp_xy=40, kv_xy=8, kp_z=30, kv_z=7.
// Att: ωₙ=24.6 rad/s, ζ=1.35, τ_dom=92 ms.
// const KI_P: f32 = 0.05;   const KI_LIMIT: f32 = 2.0;
// const KR_X: f32 = 0.010;  const KR_Y: f32 = 0.010;  const KR_Z: f32 = 0.010;
// const KW_X: f32 = 0.00110;const KW_Y: f32 = 0.00110;const KW_Z: f32 = 0.00138;
// const KI_ATT: f32 = 0.0;

// ── MOCAP BLOCK P — CF2.1 UPGRADED MOTORS (thrust upgrade kit) ────────────────
// The 5 Hz oscillation / 31 cm hover offset were NOT a gain problem: the firmware
// was compiled with the 2.1+ thrust model (THRUST_MAX 0.12 N) while the drone has
// the thrust upgrade kit (0.18 N). The mixer under-estimated thrust by ~1.5× → 1.5×
// effective attitude gain and over-thrust. Real fix: CONFIG_CRAZYFLIE_THRUST_UPGRADE_KIT=y
// in app-config (mixer now matches hardware). With that, gains are nominal → use the
// same values as Block O (standard). No ÷1.48 hack needed.
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
const ENABLE_ATTITUDE_INTEGRAL: bool = false;  // crashed on ground — windup before takeoff
//
// 3. Butterworth pre-filter on the IMU accelerometer (same fc_bw as alpha chain).
//    Smooths a_meas in the position INDI outer loop, reducing noise in a_indi.
//    Only active when position INDI outer loop is enabled (ctrl_mode bit 0 = 1).
const ENABLE_ACC_PREFILTER: bool = true;
//
// 4. Tilt-compensated gravity: gz_comp = g / cos(θ) instead of g.
//    When the drone tilts, vertical thrust = T·cos(θ) — commanding g/cos(θ) pre-compensates
//    so altitude hold is not disturbed by tilt.  Clamped at cos(60°)=0.5 to prevent blow-up.
const ENABLE_TILT_GRAVITY_COMP: bool = false;
//
// ── Round 2: robustness / margin-widening fixes (see docs/results_2026-06-20.md) ──
// Both share the same outer position loop (force = ff + KP·ep + KV·ev + KI·i_ep).
// The July 1 crashes were a conditional divergence: an intermittent mocap latency/quality
// dip eats the phase margin of the underdamped KP=40 loop (ζ≈0.26). These two fixes let the
// loop *shrug off* that dip so KP can stay at 40 (better INDI tracking) without diverging.
// Applied to the XY velocity-error feedback in BOTH geometric and INDI paths; Z untouched.
//
// 5. Velocity-feedback low-pass: 2nd-order Butterworth (VEL_FB_FC_HZ) on the XY velocity
//    error ev before the KV·ev term. Removes the differentiated-mocap HF noise that KV
//    amplifies, so KV can be raised (kv_xy in crazyflies.yaml, target ζ≈0.7 at KP=40)
//    for damping *without* injecting noise. Cutoff >> loop crossover (~1 Hz) so it adds
//    negligible phase lag at the crossover while cutting the noise.
const ENABLE_VEL_FB_FILTER: bool = true;
//
// 6. Pose-glitch guard: saturation clamp (±EV_XY_MAX) on the XY velocity error. A single
//    glitched mocap frame produces a huge ev spike → violent motor command; the clamp caps
//    the response so one bad frame can't flip the drone. Applied to the raw ev *before* the
//    filter so an outlier never enters the filter memory.
const ENABLE_POSE_GLITCH_GUARD: bool = true;
//
// Round-2 tuning constants
const VEL_FB_FC_HZ: f32 = 10.0;   // velocity-feedback low-pass cutoff [Hz]
const EV_XY_MAX:    f32 = 0.6;    // max |XY velocity error| fed to KV term [m/s]
//
// A. INDI RPM sanity/hold guard: a motor reading implausibly low (< RPM_GUARD_LO) while a
//    sibling is clearly airborne (> RPM_GUARD_HI) is a deck glitch — substitute the last valid
//    reading for that motor so the INDI torque estimate (a_indi + tau_current) is not corrupted.
//    Thresholds match the data-driven "genuine mid-flight dropout" definition (only 19:16 & 19:34
//    on July 1). INDI-only; no effect on geometric (which never reads RPM).
const ENABLE_RPM_GUARD: bool = false;
const RPM_GUARD_LO: u16 = 8000;   // below this while a sibling is high ⇒ treat as glitch
const RPM_GUARD_HI: u16 = 18000;  // a motor above this ⇒ clearly airborne (arms the guard)
//
// C. Gyro-feedback low-pass: 2nd-order Butterworth (GYRO_FB_FC_HZ) on the gyro used in the
//    attitude-rate error e_omega = ω − ω_d (the KW damping term), in both geometric and INDI
//    attitude laws. Removes high-frequency MEMS noise that KW would otherwise inject into torque.
//    HIGH cutoff matched to the fast attitude loop (≫ velocity's 10 Hz) so it adds negligible phase
//    lag at the attitude crossover. Only the damping feedback is filtered; the gyroscopic
//    feedforward (ω×Jω) and the INDI α-chain keep raw ω.
const ENABLE_GYRO_FB_FILTER: bool = false;
const GYRO_FB_FC_HZ: f32 = 50.0;  // gyro attitude-rate-feedback low-pass cutoff [Hz]

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
    // INDI filter chain (legacy order, default): raw omega → diff → BW → α_meas
    bw_x: Butterworth2, bw_y: Butterworth2, bw_z: Butterworth2,
    // INDI filter chain (paper order, filt_order=1): raw omega → BW → diff → α_meas
    // Tal & Karaman 2021 / stock crazyflie-firmware controller_indi.c both filter the gyro
    // BEFORE differentiating, not after. Separate filter instances from bw_x/y/z (legacy) and
    // from bw_gyro_x/y/z (v2 fix C, e_omega feedback, different cutoff) to keep both paths intact.
    bw_pre_x: Butterworth2, bw_pre_y: Butterworth2, bw_pre_z: Butterworth2,
    omega_filt_prev: Vec3,   // previous FILTERED gyro (for paper-order finite-difference)
    // INDI increment base filter (filt_tau=1): low-pass tau_current (μ_f) with the SAME
    // Butterworth as α_meas so the two terms of τ = μ_f + J·(α_ref − α_meas) are phase-matched.
    // Tal & Karaman Eq. 29/31 (μ_f is FILTERED) and stock controller_indi.c (indi.u[0].o[0] =
    // filter_pqr'd command). Missing filter here = phase mismatch that drives the limit cycle.
    bw_tau_x: Butterworth2, bw_tau_y: Butterworth2, bw_tau_z: Butterworth2,
    bw_tau_init: bool,
    // INDI reference filter: α_ref → BW → α_ref_filt (phase alignment with α_meas)
    bw_ref_x: Butterworth2, bw_ref_y: Butterworth2, bw_ref_z: Butterworth2,
    // Optional stage-2 notch (indi_gains.notch_en, 2026-07-28): α_meas → notch →
    // α_meas_notch (always computed, for filter-characterisation logging in ALL
    // modes, same philosophy as alpha_raw/alpha_meas above). α_ref_filt gets the
    // SAME notch applied (notch_ref_*) so measurement and reference stay
    // phase-matched when the notch is enabled -- mirrors why bw_ref_* exists.
    notch_x: NotchFilter, notch_y: NotchFilter, notch_z: NotchFilter,
    notch_ref_x: NotchFilter, notch_ref_y: NotchFilter, notch_ref_z: NotchFilter,
    // Notch on the increment-base term tau_current (mu_f), mirroring bw_tau_x/y/z's own
    // reasoning: tau_current sums DIRECTLY into the final command (tau = tau_current +
    // delta_tau), so if only alpha_meas/alpha_ref got the notch and tau_current didn't,
    // the two terms of that sum would carry mismatched phase lag at the notch's own
    // frequency -- same class of problem filt_tau exists to prevent for the stage-1
    // Butterworth. Physically also matters: tau_current is RPM-derived (or tau_prev
    // feedback) and can carry the same 5-10 Hz resonance directly.
    notch_tau_x: NotchFilter, notch_tau_y: NotchFilter, notch_tau_z: NotchFilter,
    notch_f0_last: f32, notch_bw_last: f32,
    // v2 improvement #3: accelerometer pre-filter for position INDI outer loop
    bw_acc_x: Butterworth2, bw_acc_y: Butterworth2, bw_acc_z: Butterworth2,
    // v2 fix #5: XY velocity-error feedback low-pass (robustness to mocap noise)
    bw_vel_x: Butterworth2, bw_vel_y: Butterworth2,
    bw_vel_init: bool,
    // v2 fix C: gyro low-pass for attitude-rate feedback (e_omega)
    bw_gyro_x: Butterworth2, bw_gyro_y: Butterworth2, bw_gyro_z: Butterworth2,
    bw_gyro_init: bool,
    // v2 fix A: last valid per-motor RPM (INDI dropout hold)
    rpm_prev: [u16; 4],
    omega_prev: Vec3,   // raw gyro from previous cycle (for finite-difference)
    tau_prev: Vec3,
    tau_act: Vec3,      // act_dyn base state (first-order model of clamped commands)
    indi_init: bool,
    fc_bw_last: f32,
    /// Previous peer sample (x, y, z, timestamp_ms), for differencing a relative velocity the
    /// peer API does not provide. Per-vehicle, hence in State rather than a global: the host
    /// simulator swaps this block per drone, and shared peer history would corrupt both.
    peer_prev: [(f32, f32, f32, u32); residual_nn::MAX_NEIGHBOURS],
}

impl State {
    const fn zero() -> Self {
        Self {
            i_ep: Vec3::zero(), i_error_att: Vec3::zero(), last_tick: 0,
            bw_x: Butterworth2::zero(), bw_y: Butterworth2::zero(), bw_z: Butterworth2::zero(),
            bw_pre_x: Butterworth2::zero(), bw_pre_y: Butterworth2::zero(), bw_pre_z: Butterworth2::zero(),
            omega_filt_prev: Vec3::zero(),
            bw_tau_x: Butterworth2::zero(), bw_tau_y: Butterworth2::zero(), bw_tau_z: Butterworth2::zero(),
            bw_tau_init: false,
            bw_ref_x: Butterworth2::zero(), bw_ref_y: Butterworth2::zero(), bw_ref_z: Butterworth2::zero(),
            notch_x: NotchFilter::zero(), notch_y: NotchFilter::zero(), notch_z: NotchFilter::zero(),
            notch_ref_x: NotchFilter::zero(), notch_ref_y: NotchFilter::zero(), notch_ref_z: NotchFilter::zero(),
            notch_tau_x: NotchFilter::zero(), notch_tau_y: NotchFilter::zero(), notch_tau_z: NotchFilter::zero(),
            notch_f0_last: 0.0, notch_bw_last: 0.0,
            bw_acc_x: Butterworth2::zero(), bw_acc_y: Butterworth2::zero(), bw_acc_z: Butterworth2::zero(),
            bw_vel_x: Butterworth2::zero(), bw_vel_y: Butterworth2::zero(),
            bw_vel_init: false,
            bw_gyro_x: Butterworth2::zero(), bw_gyro_y: Butterworth2::zero(), bw_gyro_z: Butterworth2::zero(),
            bw_gyro_init: false,
            rpm_prev: [0u16; 4],
            omega_prev: Vec3::zero(),
            tau_prev: Vec3::zero(),
            tau_act: Vec3::zero(),
            indi_init: false,
            fc_bw_last: 0.0,
            peer_prev: [(0.0, 0.0, 0.0, 0); residual_nn::MAX_NEIGHBOURS],
        }
    }
    fn reset(&mut self) {
        self.i_ep = Vec3::zero();
        self.i_error_att = Vec3::zero();
        self.last_tick = 0;
        self.bw_x.reset_state(); self.bw_y.reset_state(); self.bw_z.reset_state();
        self.bw_pre_x.reset_state(); self.bw_pre_y.reset_state(); self.bw_pre_z.reset_state();
        self.omega_filt_prev = Vec3::zero();
        self.bw_tau_x.reset_state(); self.bw_tau_y.reset_state(); self.bw_tau_z.reset_state();
        self.bw_tau_init = false;
        self.bw_ref_x.reset_state(); self.bw_ref_y.reset_state(); self.bw_ref_z.reset_state();
        self.notch_x.reset_state(); self.notch_y.reset_state(); self.notch_z.reset_state();
        self.notch_ref_x.reset_state(); self.notch_ref_y.reset_state(); self.notch_ref_z.reset_state();
        self.notch_tau_x.reset_state(); self.notch_tau_y.reset_state(); self.notch_tau_z.reset_state();
        self.bw_acc_x.reset_state(); self.bw_acc_y.reset_state(); self.bw_acc_z.reset_state();
        self.bw_vel_x.reset_state(); self.bw_vel_y.reset_state();
        self.bw_vel_init = false;
        self.bw_gyro_x.reset_state(); self.bw_gyro_y.reset_state(); self.bw_gyro_z.reset_state();
        self.bw_gyro_init = false;
        self.rpm_prev = [0u16; 4];
        self.omega_prev = Vec3::zero();
        self.tau_prev = Vec3::zero();
        self.tau_act = Vec3::zero();
        self.indi_init = false;
        self.peer_prev = [(0.0, 0.0, 0.0, 0); residual_nn::MAX_NEIGHBOURS];
    }
}

static mut CTRL: State = State::zero();

/// Learned residual model. Inert until a complete weight set is uploaded, so a drone that has
/// never been given weights behaves exactly as it did before this existed.
static mut RNN: ResidualNet = ResidualNet::new();

/// Evaluate the learned residual from the peers the firmware already knows about.
///
/// Crazyswarm2 broadcasts every vehicle's pose and the firmware stores the ones that are not
/// its own, so this needs no new communication. The peer API carries position only, so the
/// relative velocity is differenced against the previous sample using the timestamp that comes
/// with it; a peer seen for the first time, or one whose timestamp has not advanced, is given
/// zero relative velocity rather than a divided-by-nothing spike.
///
/// The prediction is always computed and logged, whatever `rnn.en` says. Comparing predicted
/// against measured residual IS the evaluation of every learned method here, and that
/// comparison is only possible if the prediction is recorded on flights where it is not being
/// used -- including flights under the geometric controller.
unsafe fn rnn_predict(s: &mut State, own_pos: Vec3, own_vel: Vec3) -> Vec3 {
    const M: usize = residual_nn::MAX_NEIGHBOURS;
    let (mut xs, mut ys, mut zs) = ([0.0f32; M], [0.0f32; M], [0.0f32; M]);
    let mut ts = [0u32; M];
    let n = peer_get_all(xs.as_mut_ptr(), ys.as_mut_ptr(), zs.as_mut_ptr(),
                         ts.as_mut_ptr(), M as u8) as usize;

    let mut rel = [(Vec3::zero(), Vec3::zero()); M];
    for k in 0..n.min(M) {
        let p = Vec3::new(xs[k], ys[k], zs[k]);
        let (px, py, pz, pt) = s.peer_prev[k];
        let dt_ms = ts[k].wrapping_sub(pt);
        let v = if pt != 0 && dt_ms > 0 && dt_ms < 500 {
            let inv = 1000.0 / dt_ms as f32;
            Vec3::new((p.x - px) * inv, (p.y - py) * inv, (p.z - pz) * inv)
        } else {
            Vec3::zero()
        };
        s.peer_prev[k] = (p.x, p.y, p.z, ts[k]);
        rel[k] = (p.sub(own_pos), v.sub(own_vel));
    }

    let pred = RNN.eval(&rel, n);
    rnn_pred_write(pred.x, pred.y, pred.z, RNN.clamped as u8);
    pred
}

/// Service the weight-upload protocol. Called once per control tick, before the arming check so
/// the network can be loaded while the drone is still on the ground; each call handles at most
/// one parameter write, which is all the host can deliver per CRTP packet anyway.
///
/// Kept entirely separate from the control path: uploading weights mid-flight changes nothing
/// until `rnn.en` is set, and an incomplete upload leaves `ready` at 0.
unsafe fn rnn_service() {
    if g_rnn_begin != 0 {
        RNN.begin_upload(g_rnn_n);
        g_rnn_begin = 0;
        g_rnn_ready = 0;
    }
    if g_rnn_wc != 0 {
        RNN.set_weight(g_rnn_wi as usize, g_rnn_wv);
        g_rnn_wc = 0;
    }
    if g_rnn_end != 0 {
        g_rnn_ready = if RNN.finish_upload() { 1 } else { 0 };
        g_rnn_end = 0;
    }
}

/// Run one pass of the weight-upload protocol, for the host simulator ONLY.
///
/// On the drone `rnn_service` is called from `controllerOutOfTree` on every tick, before the
/// arming check, so weights load while the vehicle is on the ground. The simulator's SIL layer
/// returns early when a vehicle is idle and never calls the controller at all, so piggybacking
/// there would stall the upload until takeoff and then load weights mid-flight. This exists so
/// the simulator can drive the SAME protocol on the SAME state at the same point in the run.
/// It adds no second code path: it calls `rnn_service` and nothing else.
#[no_mangle]
pub extern "C" fn oot_rnn_service() {
    unsafe { rnn_service() }
}

// Address and size of the controller state, for the host simulator ONLY.
//
// On the drone there is one vehicle per MCU, so a single static is correct. The
// software-in-the-loop simulator, however, runs every simulated drone through this one
// compiled controller, and the calls interleave: drone A, drone B, drone A... They would
// share `last_tick`, the INDI filter chain and the integrators, which silently destroys
// both. (Symptom: geometric flies fine because it is near-memoryless, while INDI never
// leaves the ground.) The simulator swaps this block in and out per vehicle, giving each
// one its own controller state.
//
// Nothing here reads or writes the state, and no control logic depends on it. It exists
// so the host does not have to duplicate the layout of `State`, which would rot.
#[no_mangle]
pub extern "C" fn oot_state_ptr() -> *mut u8 {
    core::ptr::addr_of_mut!(CTRL) as *mut u8
}

#[no_mangle]
pub extern "C" fn oot_state_size() -> usize {
    core::mem::size_of::<State>()
}

/// Diagonal inertia the controller was compiled with, for the host simulator ONLY.
///
/// J is a compile-time constant chosen by the `drone_bl` cfg, so the controller and the
/// simulated plant can disagree about which airframe they are without anything saying so.
/// The plant reads it from here instead of carrying a second copy. Same reasoning as
/// THRUST_MAX / ARM_LENGTH in oot_host.c; no control logic reads this.
#[no_mangle]
pub extern "C" fn oot_inertia(axis: i32) -> f32 {
    match axis {
        0 => JXX,
        1 => JYY,
        _ => JZZ,
    }
}

// ── INDI log variables (Mode 1) ────────────────────────────────────────────
// Variables owned by C (traj_iface.c); Rust writes via C function calls which
// are opaque to Rust LTO — values are always computed and stored correctly.
extern "C" {
    fn peer_get_all(xs: *mut f32, ys: *mut f32, zs: *mut f32, ts: *mut u32, max: u8) -> u8;
    fn rnn_pred_write(x: f32, y: f32, z: f32, clamped: u8);
    fn indi_log_write(arx: f32, ary: f32, arz: f32, ax: f32, ay: f32, az: f32);
    fn indi_tau_write(tx: f32, ty: f32, tz: f32);
    fn indi_notch_log_write(anx: f32, any: f32, anz: f32);
    fn indi_a_res_write(ax: f32, ay: f32, az: f32);
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
// 14 = 12-segment periodic cores (oval/tilted_oval) + 1 entry + 1 exit segment
// (Issue B rest-to-rest fix). Position upload index is u16 (14*19=266 > u8 range).
const TRAJ_MAX_SEGS: usize = 14;
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
    // Optional stage-2 notch filter (2026-07-28, shake investigation): 0=off (default,
    // byte-identical to today), 1=on. f0/bw are runtime-tunable [Hz] (Q = f0/bw).
    static mut g_indi_omega_src: u8;
    static mut g_indi_frame_conv: u8;
    static mut g_indi_notch_en: u8;
    static mut g_indi_notch_f0: f32;
    static mut g_indi_notch_bw: f32;
    // H1a diagnostic: force tau_current = tau_prev even when RPM deck is active (0=off, 1=on)
    static mut g_indi_ff_free: u8;
    // 0 = legacy diff-then-filter order (default), 1 = paper order (filter-then-diff)
    static mut g_indi_filt_order: u8;
    // 0 = legacy unfiltered tau_current (default), 1 = filter μ_f to phase-match α_meas (paper Eq.29)
    static mut g_indi_filt_tau: u8;
    // INDI effectiveness scale on the J·Δα increment (default 1.0). <1.0 reduces over-correction.
    static mut g_indi_j_scale: f32;
    // Increment-base source: 0.0 = RPM base (default), >0 = act_dyn model time constant [s]
    static mut g_indi_act_tau:      f32;
    // Output clamps (bitmask: bit0=tau_xy, bit1=tau_z, bit2=tilt, bit3=thrust; 0 = all off)
    static mut g_indi_clamp_en:     u8;
    static mut g_indi_tau_xy_max:   f32; // [Nm]
    static mut g_indi_tau_z_max:    f32; // [Nm]
    static mut g_indi_tilt_max_deg: f32; // [deg]
    static mut g_indi_thrust_max:   f32; // [N]
    // Runtime-tunable position gains (traj_iface.c PARAM_GROUP pos_gains)
    // Residual-network weight upload and control, mirroring the trajectory upload protocol.
    static mut g_rnn_wi:    u16;
    static mut g_rnn_wv:    f32;
    static mut g_rnn_wc:    u8;
    static mut g_rnn_n:     u16;
    static mut g_rnn_begin: u8;
    static mut g_rnn_end:   u8;
    static mut g_rnn_en:    u8;
    static mut g_rnn_ready: u8;

    static mut g_kp_xy: f32;
    static mut g_kp_z:  f32;
    static mut g_kv_xy: f32;
    static mut g_kv_z:  f32;
}

extern "C" {
    static mut g_traj_coefs:     [f32; 266]; // TRAJ_MAX_SEGS * TRAJ_FLOATS_PER_SEG
    static mut g_traj_z_coefs:   [f32; 126]; // TRAJ_MAX_SEGS * TRAJ_Z_FLOATS_PER_SEG
    static mut g_traj_z_mode:    u8;  // 0=ramp (default), 1=polynomial (3D trajectories)
    static mut g_traj_z_ci:      u8;  // z upload index
    static mut g_traj_z_cv:      f32; // z upload value
    static mut g_traj_z_cw:      u8;  // z commit flag
    static mut g_traj_att_coefs:     [f32; 252]; // TRAJ_MAX_SEGS * TRAJ_ATT_FLOATS_PER_SEG
    static mut g_traj_att_mode:      u8;  // 0=flatness, 1=polynomial
    static mut g_traj_att_ctrl_mode: u8;  // 0=hard override, 1=hybrid (flatness rd + poly omega_d)
    static mut g_traj_att_ci:        u8;  // attitude upload index
    static mut g_traj_att_cv:        f32; // attitude upload value
    static mut g_traj_att_cw:        u8;  // attitude commit flag
    static mut g_traj_n_segs:    u8;
    static mut g_traj_mode:      u8;  // 0=passthrough (Mode B), 1=onboard eval (Mode D)
    static mut g_traj_start:     u8;  // laptop writes 1; firmware latches T0
    static mut g_traj_reps:      u8;  // 0=loop forever; N=self-stop after N laps
    static mut g_traj_n_entry:   u8;  // leading segments played ONCE before the core laps
    static mut g_traj_n_exit:    u8;  // trailing segments played ONCE after the core laps
    static mut g_traj_origin_x:  f32;
    static mut g_traj_origin_y:  f32;
    static mut g_traj_hover_z:   f32;
    static mut g_traj_dz:        f32; // Z gain per lap: 0=flat, +0.40=ascending helix
    static mut g_traj_coef_ci:   u16; // position upload index (u16: 14 segs = 266 coefs)
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
/// Map global trajectory time onto the entry/core/exit timeline and locate the
/// active segment (Issue B rest-to-rest fix). Segments [0..n_entry) play ONCE,
/// the core [n_entry..n_segs-n_exit) loops (g_traj_reps laps; reps=0 = forever,
/// exit never reached), and the last n_exit segments play ONCE after the laps.
/// n_entry = n_exit = 0 reproduces the legacy wrap-everything behaviour exactly.
///
/// Returns `(seg, t_in_seg, dur, core_frac, t_core)` — core_frac is the
/// un-wrapped core lap progress (0 during entry, held at reps after the laps),
/// used only for the legacy dz z-ramp.
unsafe fn traj_locate(t_global: f32, n_segs: usize) -> (usize, f32, f32, f32, f32) {
    let n_entry = (g_traj_n_entry as usize).min(n_segs);
    let n_exit  = (g_traj_n_exit  as usize).min(n_segs - n_entry);
    let core_lo = n_entry;
    let core_hi = n_segs - n_exit;

    let mut t_entry = 0.0_f32;
    for i in 0..core_lo { t_entry += g_traj_coefs[i * TRAJ_FLOATS_PER_SEG]; }
    let mut t_core = 0.0_f32;
    for i in core_lo..core_hi { t_core += g_traj_coefs[i * TRAJ_FLOATS_PER_SEG]; }

    let reps = g_traj_reps as f32;
    let (lo, hi, t_loc, core_frac);
    if t_global < t_entry {
        lo = 0; hi = core_lo; t_loc = t_global; core_frac = 0.0;
    } else {
        let tc = t_global - t_entry;
        if g_traj_reps == 0 || tc < reps * t_core || core_hi == n_segs {
            // Core laps: wrap tc into [0, t_core). C0-continuous for closed loops.
            let wrapped = if t_core > 0.0 {
                tc - libm::floorf(tc / t_core) * t_core
            } else {
                tc
            };
            lo = core_lo; hi = core_hi; t_loc = wrapped;
            core_frac = if t_core > 0.0 { tc / t_core } else { 0.0 };
        } else {
            // Exit region (past t_end it clamps to the last point; self-stop fires there).
            lo = core_hi; hi = n_segs; t_loc = tc - reps * t_core; core_frac = reps;
        }
    }

    let mut t_start = 0.0_f32;
    let mut seg = hi.saturating_sub(1).max(lo);
    for i in lo..hi {
        let dur = g_traj_coefs[i * TRAJ_FLOATS_PER_SEG];
        if t_loc < t_start + dur { seg = i; break; }
        t_start += dur;
    }
    let dur  = g_traj_coefs[seg * TRAJ_FLOATS_PER_SEG].max(1e-9);
    let t_in = (t_loc - t_start).max(0.0).min(dur);
    (seg, t_in, dur, core_frac, t_core)
}

/// Returns `(pos_rel, vel, acc, jerk, snap)` — pos_rel is relative to trajectory
/// origin (caller adds `g_traj_origin_{x,y}` and `g_traj_hover_z`).
unsafe fn eval_traj_onboard(t_global: f32, n_segs: usize) -> (Vec3, Vec3, Vec3, Vec3, Vec3) {
    let (seg, tau, dur, core_frac, t_core) = traj_locate(t_global, n_segs);
    let base = seg * TRAJ_FLOATS_PER_SEG;
    let t_n  = tau / dur; // normalised ∈ [0, 1]

    // Extract coefficients into fixed-size arrays
    let mut cx = [0.0_f32; 9];
    let mut cy = [0.0_f32; 9];
    for k in 0..9 {
        cx[k] = g_traj_coefs[base + 1 + k];
        cy[k] = g_traj_coefs[base + 10 + k];
    }

    let (px, vx, ax, jx, sx) = poly_eval_axis(&cx, t_n, dur);
    let (py, vy, ay, jy, sy) = poly_eval_axis(&cy, t_n, dur);

    // Z: linear ramp using the un-wrapped core progress so multi-rep helices stack
    // continuously upward instead of resetting each lap.
    // With n_entry=n_exit=0 and one rep, core_frac == t_global/total — identical to before.
    // When g_traj_dz=0 (circle, figure-8, loop), all z components are 0 → backward compatible.
    let vz = g_traj_dz / t_core.max(0.001);

    (
        Vec3::new(px, py, core_frac * g_traj_dz),
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
    // Same entry/core/exit segment mapping as eval_traj_onboard.
    let (seg, t_in, dur, _, _) = traj_locate(t_global, n_segs);
    let tau = t_in / dur; // ∈ [0,1]

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
    // Same entry/core/exit segment mapping as eval_traj_onboard.
    let (seg, t_in, dur, _, _) = traj_locate(t_global, n_segs);
    let tau = t_in / dur;

    let base = seg * TRAJ_Z_FLOATS_PER_SEG;
    let mut cz = [0.0_f32; 9];
    for k in 0..9 {
        cz[k] = g_traj_z_coefs[base + k];
    }
    let (pz, vz, az, jz, sz) = poly_eval_axis(&cz, tau, dur);
    (pz, vz, az, jz, sz)
}

/// Frame convention pinned for Mode D (onboard trajectory eval). Mode D is the flight
/// artifact of the completed INDI project — every result, gain block and kt ceiling was
/// obtained with the Faessler ω_d/α_des construction, so it stays on it permanently and
/// is NOT affected by `g_indi_frame_conv`. Change this constant only if you intend to
/// invalidate that baseline.
const FRAME_CONV_MODE_D: u8 = 0;

/// Desired body x/y axes for a thrust vector `acc_g` and yaw ψ, in the convention
/// selected by `conv` (see `g_indi_frame_conv` in traj_iface.c).
///
/// z_B is unambiguous — it is the thrust direction. The only choice is where yaw sits
/// around it, and the two standard constructions agree ONLY when the tilt is aligned
/// with a principal axis; off-axis they diverge with tilt.
///
/// `conv == 1` — Mellinger (default): `y_B = normalize(z_B × x_C)`, `x_B = y_B × z_B`.
///   Identical to `desired_rot()` above, to the official `controller_lee.c` /
///   `controller_mellinger.c`, and to the HLC's `pptraj.c`.
/// `conv == 0` — Faessler et al. 2018 App. A: `x_B = normalize(y_C × z_B)`,
///   `y_B = normalize(z_B × x_B)`. Legacy: what ω_d/α_des used before 2026-08-22.
#[inline]
fn body_axes(acc_g: Vec3, yaw: f32, conv: u8) -> (Vec3, Vec3) {
    let cos_psi = libm::cosf(yaw);
    let sin_psi = libm::sinf(yaw);
    if conv == 1 {
        let zb = acc_g.normalize();
        let xc = Vec3::new(cos_psi, sin_psi, 0.0);
        let zcx = zb.cross(xc);
        // Degenerate when the thrust axis is parallel to x_C (tilt ≈ 90° into the yaw
        // direction) — mirrors the same guard in desired_rot().
        let yb = if zcx.norm() > 1e-9 { zcx.normalize() } else { Vec3::new(0.0, 1.0, 0.0) };
        (yb.cross(zb), yb)
    } else {
        let yc = Vec3::new(-sin_psi, cos_psi, 0.0);
        let xb = yc.cross(acc_g).normalize();
        (xb, acc_g.cross(xb).normalize())
    }
}

/// Compute desired body angular velocity ω_d from flatness (yaw=const → ψ̇=0).
///
/// Simplified for ψ̇ = 0 (structure is identical in both frame conventions; only the
/// basis differs — cf. `controller_lee.c`'s `omega_des`):
///   ωx = −(yb · jerk) / c
///   ωy =  (xb · jerk) / c
///   ωz = 0
/// where `c = |acc + g·ez|` and `xb`, `yb` come from `body_axes(.., conv)`.
///
/// Feeds the KW attitude-rate damping term of BOTH the geometric and INDI laws.
#[inline]
fn omega_desired(acc: Vec3, jerk: Vec3, yaw: f32, conv: u8) -> Vec3 {
    let acc_g = Vec3::new(acc.x, acc.y, acc.z + GRAVITY);
    let c = acc_g.norm();
    if c < 0.1 { return Vec3::zero(); }

    let (xb, yb) = body_axes(acc_g, yaw, conv);

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
/// Frame convention follows `body_axes(.., conv)` — must match `omega_desired`, since
/// α_des is analytically the time-derivative of ω_d. Feeds the INDI snap feedforward
/// only (the geometric law does not use it).
#[inline]
fn alpha_desired(acc: Vec3, jerk: Vec3, snap: Vec3, yaw: f32, conv: u8) -> Vec3 {
    let acc_g = Vec3::new(acc.x, acc.y, acc.z + GRAVITY);
    let c = acc_g.norm();
    if c < 0.1 { return Vec3::zero(); }

    let (xb, yb) = body_axes(acc_g, yaw, conv);
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
/// v2 fixes 5 & 6: condition the XY velocity-error feedback for robustness to intermittent
/// mocap latency/quality. Fix 6 clamps ev first (a single glitched pose frame can't command a
/// violent motor spike, and the outlier never enters the filter memory); fix 5 then low-passes
/// ev (removes differentiated-mocap HF noise so KV damping can be raised). Z is left untouched.
/// No-op when both flags are false ⇒ identical behaviour to stable.
#[inline]
fn condition_vel_error(ev: Vec3, dt: f32, s: &mut State) -> Vec3 {
    let mut ex = ev.x;
    let mut ey = ev.y;
    if ENABLE_POSE_GLITCH_GUARD {
        ex = ex.clamp(-EV_XY_MAX, EV_XY_MAX);
        ey = ey.clamp(-EV_XY_MAX, EV_XY_MAX);
    }
    if ENABLE_VEL_FB_FILTER {
        if !s.bw_vel_init {
            s.bw_vel_x.init(VEL_FB_FC_HZ, dt); s.bw_vel_x.seed(ex);
            s.bw_vel_y.init(VEL_FB_FC_HZ, dt); s.bw_vel_y.seed(ey);
            s.bw_vel_init = true;
        }
        ex = s.bw_vel_x.update(ex);
        ey = s.bw_vel_y.update(ey);
    }
    Vec3::new(ex, ey, ev.z)
}

/// v2 fix C: low-pass the gyro used in the attitude-rate feedback term (e_omega = ω − ω_d).
/// Removes high-frequency MEMS noise that the KW damping gain would otherwise inject into torque.
/// High cutoff (GYRO_FB_FC_HZ) matched to the fast attitude loop ⇒ negligible phase lag at the
/// attitude crossover. Only the damping feedback is filtered; the gyroscopic feedforward (ω×Jω)
/// and the INDI angular-accel chain keep raw ω. Must be called every cycle for filter continuity.
/// No-op (returns raw ω) when the flag is false ⇒ identical behaviour to stable.
#[inline]
fn gyro_feedback(omega: Vec3, dt: f32, s: &mut State) -> Vec3 {
    if !ENABLE_GYRO_FB_FILTER { return omega; }
    if !s.bw_gyro_init {
        s.bw_gyro_x.init(GYRO_FB_FC_HZ, dt); s.bw_gyro_x.seed(omega.x);
        s.bw_gyro_y.init(GYRO_FB_FC_HZ, dt); s.bw_gyro_y.seed(omega.y);
        s.bw_gyro_z.init(GYRO_FB_FC_HZ, dt); s.bw_gyro_z.seed(omega.z);
        s.bw_gyro_init = true;
    }
    Vec3::new(
        s.bw_gyro_x.update(omega.x),
        s.bw_gyro_y.update(omega.y),
        s.bw_gyro_z.update(omega.z),
    )
}

fn geometric_step_ref(
    pos: Vec3, vel: Vec3, r: &Mat3, omega: Vec3,
    pd: Vec3, vd: Vec3, ad: Vec3, yaw_d: f32,
    omega_d: Vec3,
    rd_override: Option<&Mat3>,
    dt: f32, s: &mut State,
) -> (f32, Vec3) {
    let ep = pd.sub(pos);
    let ev = condition_vel_error(vd.sub(vel), dt, s);
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
    let e_omega   = gyro_feedback(omega, dt, s).sub(omega_d);   // v2 fix C (no-op when off)
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
// Torque clamp (clamp_en bit0 = roll/pitch, bit1 = yaw), mirroring stock controller_indi.c's
// bound_control_input. Values sized from per-motor thrust headroom (see traj_iface.c). Applied to
// BOTH controller branches; in INDI it must run BEFORE tau_prev is stored so the increment memory
// only ever contains torques that were actually sent (stock propagates post-clamp too).
fn clamp_torque(t: Vec3) -> Vec3 {
    let en = unsafe { g_indi_clamp_en };
    let mut out = t;
    if en & 0b0001 != 0 {
        let m = unsafe { g_indi_tau_xy_max };
        out.x = out.x.clamp(-m, m);
        out.y = out.y.clamp(-m, m);
    }
    if en & 0b0010 != 0 {
        let m = unsafe { g_indi_tau_z_max };
        out.z = out.z.clamp(-m, m);
    }
    out
}

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
    // Read RPM in EVERY mode. This used to be gated on `mode != 0`, which silently
    // defeated the intent stated at a_res below: the residual is supposed to be
    // measurable while flying plain geometric, because the training data for
    // Geometric+NN has to come from non-INDI flights. With the gate in place a_res read
    // exactly zero whenever the geometric controller was selected. Reading RPM does not
    // change geometric's control law -- a_indi stays zero there -- only what is logged.
    unsafe { rpm_get_all(&mut m1, &mut m2, &mut m3, &mut m4); }
    let rpms_active = (m1 | m2 | m3 | m4) > 0;

    // v2 fix A: INDI RPM sanity/hold guard (protects both a_indi and tau_current below).
    // A motor reading implausibly low while a sibling is clearly airborne is a deck glitch;
    // substitute the last valid reading for that motor. Armed only when clearly airborne
    // (max > RPM_GUARD_HI) so it never fires pre-takeoff or during the spin-up ramp.
    if ENABLE_RPM_GUARD && rpms_active {
        let mx = m1.max(m2).max(m3).max(m4);
        if mx > RPM_GUARD_HI {
            if m1 < RPM_GUARD_LO { m1 = s.rpm_prev[0]; }
            if m2 < RPM_GUARD_LO { m2 = s.rpm_prev[1]; }
            if m3 < RPM_GUARD_LO { m3 = s.rpm_prev[2]; }
            if m4 < RPM_GUARD_LO { m4 = s.rpm_prev[3]; }
        }
        s.rpm_prev = [m1, m2, m3, m4];
    }

    // Evaluate the learned residual. Always computed and logged, whatever rnn.en says --
    // comparing predicted against measured a_res IS the evaluation of every learned method
    // here, and that needs the prediction recorded on flights where it is not used.
    let rnn_pred = unsafe { rnn_predict(s, pos, vel) };

    // -- Position loop --------------------------------------------------------
    let ep = pd.sub(pos);
    let ev = condition_vel_error(vd.sub(vel), dt, s);   // v2 fixes 5 & 6 (no-op when off)
    s.i_ep = s.i_ep.add(ep.scale(dt));
    s.i_ep = Vec3::new(
        s.i_ep.x.clamp(-KI_LIMIT, KI_LIMIT),
        s.i_ep.y.clamp(-KI_LIMIT, KI_LIMIT),
        s.i_ep.z.clamp(-KI_LIMIT, KI_LIMIT),
    );

    // Residual acceleration a_res = a_meas - a_model (world frame).
    //
    // This is f_res / m — the unmodelled force acting on the vehicle, which in close-proximity
    // formation flight is dominated by the downwash of the other drones. It is the quantity the
    // residual-learning literature (Neural-Swarm2, Aggregate Downwash, Flatness-Preserving
    // Residual) trains on, so it is logged as `indi.a_res_{x,y,z}`.
    //
    // Computed whenever RPMs are available and INDEPENDENT of controller mode, so the residual
    // can be measured while flying the plain geometric controller too — training data for
    // Geometric+NN has to come from non-INDI flights. Only the *control* use below is gated on
    // the position-INDI bit, so behaviour is unchanged: when the loop is off, a_indi is zero
    // exactly as before, and a_res is merely observed.
    let a_res = if rpms_active {
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
    unsafe { indi_a_res_write(a_res.x, a_res.y, a_res.z); }

    // Position INDI feedforward — unchanged: only fed into f_d when the outer loop is enabled.
    let a_indi = if mode & 1 != 0 { a_res } else { Vec3::zero() };

    // Learned residual feedforward — strategy 2 (Geometric + NN), and the predictive half of
    // strategy 4 (hybrid) when the INDI term above is also on.
    //
    // Gated on rnn.en, which is 0 unless a host sets it, so with the network idle this is
    // exactly zero and the control law is byte-identical to what flew before. It carries the
    // same sign as a_res below, for the same reason: it is an estimate of the SAME quantity.
    // The difference is only in where the number comes from -- INDI measures the disturbance
    // after the fact, the network predicts it before it arrives.
    //
    // Note the two are deliberately allowed to be on together. Double-counting is a real risk
    // and is exactly what strategy 4 exists to measure; it is not prevented here, because
    // preventing it would remove the comparison.
    let a_nn = if unsafe { g_rnn_en } != 0 && unsafe { g_rnn_ready } != 0 {
        rnn_pred
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
        // MINUS the residual, not plus. From m*a = f_thrust + f_res + m*g, the thrust
        // needed to hold a_des is m*a_des - m*g_vec - f_res, so the desired-acceleration
        // vector carries -a_res. Adding it instead made the controller reinforce every
        // unmodelled force rather than reject it: measured at exactly 2.00x the
        // geometric sag under a known 20 mN disturbance, and 1.89-2.10x across every
        // formation scenario in simulation. Invisible in single-drone flight, where
        // a_res ~ 0 -- it only appears once another vehicle's downwash is present.
        .sub(a_indi)
        .sub(a_nn);
    let mut thrust_vec = f_d.scale(mass);

    // Tilt clamp (clamp_en bit2): limit the desired-thrust-vector angle from vertical, mirroring
    // stock position_controller_indi.c pq_clamp. Skipped when an attitude override is active
    // (rd_override: loops/flips command tilt >90° on purpose) or when the vector points down.
    let clamp_en = unsafe { g_indi_clamp_en };
    if clamp_en & 0b0100 != 0 && rd_override.is_none() && thrust_vec.z > 0.0 {
        let tan_max = libm::tanf(unsafe { g_indi_tilt_max_deg } * core::f32::consts::PI / 180.0);
        let h = libm::sqrtf(thrust_vec.x * thrust_vec.x + thrust_vec.y * thrust_vec.y);
        let h_max = thrust_vec.z * tan_max;
        if h > h_max {
            let k = h_max / h;
            thrust_vec.x *= k;
            thrust_vec.y *= k;
        }
    }

    let body_z = Vec3::new(r[0][2], r[1][2], r[2][2]);
    let mut thrust = thrust_vec.dot(body_z).max(0.0);
    // Thrust clamp (clamp_en bit3): total thrust ceiling, mirroring stock MAX_THRUST.
    if clamp_en & 0b1000 != 0 {
        thrust = thrust.min(unsafe { g_indi_thrust_max });
    }
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
        s.omega_filt_prev = omega;
        s.indi_init = true;
    }
    let inv_dt = if dt > 1e-9 { 1.0 / dt } else { 500.0 };
    // alpha_raw: always the unfiltered diff of raw omega, kept for logging/comparison
    // regardless of filt_order — semantics of the logged "raw" signal stay stable.
    let alpha_raw = omega.sub(s.omega_prev).scale(inv_dt);
    s.omega_prev = omega;

    // filt_order (indi_gains.filt_order, default 0): which α_meas the CONTROL LAW actually
    // uses. 0 = legacy (diff raw omega, then BW-filter the result) — today's exact behaviour,
    // byte-identical, standard/upgraded gains stay valid. 1 = paper order (Tal & Karaman 2021
    // Sec. III-D + stock crazyflie-firmware controller_indi.c: BW-filter omega FIRST, then
    // differentiate) — verified against both papers and the stock C implementation before
    // implementing; not just a local guess.
    let filt_order = unsafe { g_indi_filt_order } != 0;
    let alpha_meas = if filt_order {
        let omega_filt = Vec3::new(
            s.bw_pre_x.update(omega.x),
            s.bw_pre_y.update(omega.y),
            s.bw_pre_z.update(omega.z),
        );
        let a = omega_filt.sub(s.omega_filt_prev).scale(inv_dt);
        s.omega_filt_prev = omega_filt;
        a
    } else {
        Vec3::new(
            s.bw_x.update(alpha_raw.x),
            s.bw_y.update(alpha_raw.y),
            s.bw_z.update(alpha_raw.z),
        )
    };
    unsafe { indi_log_write(alpha_raw.x, alpha_raw.y, alpha_raw.z,
                             alpha_meas.x, alpha_meas.y, alpha_meas.z); }

    // Stage-2 notch on alpha_meas -- ALWAYS computed and logged (all modes), same
    // "passive filter for characterisation logging" philosophy as alpha_raw/alpha_meas
    // above, regardless of whether notch_en actually gates it into the control law
    // below. Lets alpha_raw / alpha_meas (BW) / alpha_meas_notch be compared directly
    // from one flight without needing to fly twice.
    let alpha_meas_notch = Vec3::new(
        s.notch_x.update(alpha_meas.x),
        s.notch_y.update(alpha_meas.y),
        s.notch_z.update(alpha_meas.z),
    );
    unsafe { indi_notch_log_write(alpha_meas_notch.x, alpha_meas_notch.y, alpha_meas_notch.z); }

    // v2 fix C: gyro used in the attitude-rate error (KW damping). Runs every cycle for filter
    // continuity; returns raw ω when disabled. Feedforward (ω×Jω) and α-chain keep raw ω.
    let omega_fb = gyro_feedback(omega, dt, s);

    // -- Reference-side alpha chain + tau_current chain: computed and filtered every cycle,
    // same "always run, gate only the selection" pattern as alpha_meas/alpha_meas_notch above.
    // BUG FIX 2026-07-29: this entire block used to live inside the `mode & 2 != 0` branch
    // below, so alpha_ref_notch/tau_current_notch never ran during the geometric ramp phase
    // (mode==0) and started completely cold (zero filter state) at the exact tick INDI took
    // over -- injecting a large spurious alpha_err from the filters' own startup transient
    // against the already-warm alpha_meas_notch. That mismatch, not notch bandwidth, was the
    // actual cause of every notch_en=1 flight crashing at the geometric->INDI handover
    // regardless of which bw was tried (2026-07-29 lab session, 100% crash rate at bw in
    // {5, 6.5, 8, 10, 20}). Hoisting this out unconditionally keeps all three chains
    // (alpha_meas, alpha_ref, tau_current) warmed up in lockstep the whole flight.

    // alpha_ref = alpha_des - KR*eR - KW*e_omega  (Tal & Karaman Eq. 28)
    let e_omega = omega_fb.sub(omega_d);
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

    // Stage-2 notch on alpha_ref_filt -- SAME filter (same f0/bw, same phase
    // response) applied to the reference chain as was just applied to the
    // measured chain above (alpha_meas_notch), mirroring exactly why bw_ref_x/y/z
    // exists for the stage-1 Butterworth: measurement and reference must go
    // through matched filtering or the error term picks up a spurious phase
    // mismatch. Always computed (cheap, no side effects) so notch_en can toggle
    // at runtime without a reinit-timing hazard.
    let alpha_ref_notch = Vec3::new(
        s.notch_ref_x.update(alpha_ref_filt.x),
        s.notch_ref_y.update(alpha_ref_filt.y),
        s.notch_ref_z.update(alpha_ref_filt.z),
    );

    // Increment base source (indi_gains.act_tau, 2026-07-22):
    //   act_tau = 0 (default): tau_current from per-motor RPM^2 (falls back to tau_prev
    //     when deck absent, or when H1a diagnostic ff_free forces it) — today's behaviour.
    //   act_tau > 0: act_dyn base — first-order model of the previous (clamped) commands
    //     with that time constant, replacing the RPM reading (stock controller_indi.c
    //     structure). Sim (tests/test_indi_actuator_lag.rs) predicts a matched model is
    //     EQUIVALENT to the RPM base; this is the hardware A/B knob to verify that.
    let ff_free = unsafe { g_indi_ff_free } != 0;
    let act_tau = unsafe { g_indi_act_tau };
    let tau_current_raw = if act_tau > 0.0 {
        let k = dt / (dt + act_tau);
        s.tau_act = s.tau_act.add(s.tau_prev.sub(s.tau_act).scale(k));
        s.tau_act
    } else if rpms_active && !ff_free {
        rpms_to_torque([m1, m2, m3, m4], [kt1, kt2, kt3, kt4])
    } else {
        s.tau_prev
    };

    // filt_tau (indi_gains.filt_tau, default 0): low-pass the increment base term (μ_f) with
    // the SAME Butterworth as α_meas, so τ = μ_f + J·(α_ref − α_meas) is phase-matched — Tal &
    // Karaman Eq. 29/31 (μ_f filtered) + stock controller_indi.c (filtered command base). 0 =
    // legacy (unfiltered tau_current), byte-identical to prior behaviour. Seed on first use to
    // avoid a startup transient; passes DC so hover steady-state torque is unchanged.
    // ALSO forced on whenever notch_en=1, regardless of filt_tau's own value -- otherwise
    // tau_current could reach the stage-2 notch below at a shallower filter depth (raw,
    // no BW) than alpha_meas/alpha_ref (always BW'd first), reintroducing exactly the
    // phase-mismatch class filt_tau exists to prevent. This makes notch_en self-contained:
    // enabling it alone always gives tau_current the same BW-then-notch depth as the other
    // two chains, independent of what filt_tau happens to be set to.
    let tau_bw_needed = unsafe { g_indi_filt_tau != 0 || g_indi_notch_en != 0 };
    let tau_current = if tau_bw_needed {
        if !s.bw_tau_init {
            s.bw_tau_x.seed(tau_current_raw.x);
            s.bw_tau_y.seed(tau_current_raw.y);
            s.bw_tau_z.seed(tau_current_raw.z);
            s.bw_tau_init = true;
        }
        Vec3::new(
            s.bw_tau_x.update(tau_current_raw.x),
            s.bw_tau_y.update(tau_current_raw.y),
            s.bw_tau_z.update(tau_current_raw.z),
        )
    } else {
        tau_current_raw
    };

    // Stage-2 notch on tau_current (indi_gains.notch_en) -- tau_current is now
    // guaranteed BW-filtered first whenever notch_en=1 (tau_bw_needed above), so this
    // always sees the same BW-then-notch depth as alpha_meas/alpha_ref, regardless of
    // filt_tau's own setting. Default off = byte-identical to prior behaviour.
    // ALWAYS updated (not just when notch_en=1), same "always run, gate only the
    // selection" pattern as alpha_meas_notch/alpha_ref_notch above -- keeps the filter
    // state continuously warmed up so toggling notch_en at runtime has no cold-start
    // transient on this chain specifically (matching the other two chains, whose notch
    // filters never stop running).
    let tau_current_notch = Vec3::new(
        s.notch_tau_x.update(tau_current.x),
        s.notch_tau_y.update(tau_current.y),
        s.notch_tau_z.update(tau_current.z),
    );

    // -- Attitude law ---------------------------------------------------------
    let torque = if mode & 2 != 0 {
        // INDI attitude path — reuses alpha_raw/alpha_meas/alpha_ref/tau_current computed above

        let tau_current_sel = if unsafe { g_indi_notch_en } != 0 {
            tau_current_notch
        } else {
            tau_current
        };

        // delta_tau = j_scale*J*(alpha_ref_filt - alpha_meas),  tau = tau_current + delta_tau
        // j_scale (indi_gains.j_scale, default 1.0) scales the INDI effectiveness estimate. The
        // increment applies torque j_scale*J per unit alpha error; the motor produces a real
        // accel of (that)/J_real, so the per-cycle correction ratio is j_scale*J/J_real. J here
        // is the Busetto 2025 value, which is a Crazyflow SIMULATOR DEFAULT (not system-ID'd) for
        // a 45 g drone flown under a GEOMETRIC controller — none of which validates it for this
        // 41 g drone's INDI loop. If J is too high (ratio>1) INDI over-corrects → limit cycle;
        // sweep j_scale down (1.0→0.8→0.65→0.5) to find the ratio that stops the oscillation.
        // Default 1.0 = byte-identical to prior behaviour; standard/upgraded unaffected.
        let j_scale = unsafe { g_indi_j_scale };

        // notch_en (indi_gains.notch_en, default 0): 0 = today's behaviour exactly
        // (alpha_ref_filt, alpha_meas -- stage-1 BW only). 1 = both chains additionally
        // go through the matched stage-2 notch targeting the diagnosed 5-10 Hz shake
        // band, leaving the rest of the spectrum -- and its phase -- untouched.
        let (alpha_ref_for_err, alpha_meas_for_err) = if unsafe { g_indi_notch_en } != 0 {
            (alpha_ref_notch, alpha_meas_notch)
        } else {
            (alpha_ref_filt, alpha_meas)
        };
        let alpha_err = alpha_ref_for_err.sub(alpha_meas_for_err);
        let delta_tau = Vec3::new(
            j_scale*JXX*alpha_err.x,
            j_scale*JYY*alpha_err.y,
            j_scale*JZZ*alpha_err.z,
        );
        // Clamp BEFORE storing tau_prev and logging: the increment memory and the log must
        // reflect the torque actually sent, never an unrealizable command (oval 2026-07-21
        // blow-up reached 52 mNm vs ~14 mNm physical headroom).
        let tau       = clamp_torque(tau_current_sel.add(delta_tau));
        s.tau_prev    = tau;

        unsafe { indi_tau_write(tau.x, tau.y, tau.z); }

        tau
    } else {
        // Geometric attitude path
        let e_omega   = omega_fb.sub(omega_d);
        let j_omega   = Vec3::new(JXX*omega.x, JYY*omega.y, JZZ*omega.z);
        let gyro_comp = omega.cross(j_omega);
        // v2 improvement #2: attitude integral (same gate as geometric_step path)
        let ki_att = if ENABLE_ATTITUDE_INTEGRAL { 0.03_f32 } else { 0.0_f32 };
        clamp_torque(Vec3::new(
            -KR_X*er.x - KW_X*e_omega.x + gyro_comp.x - ki_att*s.i_error_att.x,
            -KR_Y*er.y - KW_Y*e_omega.y + gyro_comp.y - ki_att*s.i_error_att.y,
            -KR_Z*er.z - KW_Z*e_omega.z + gyro_comp.z - ki_att*s.i_error_att.z,
        ))
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
        s.bw_pre_x.init(fc_bw, DT); s.bw_pre_y.init(fc_bw, DT); s.bw_pre_z.init(fc_bw, DT);
        s.bw_tau_x.init(fc_bw, DT); s.bw_tau_y.init(fc_bw, DT); s.bw_tau_z.init(fc_bw, DT);
        s.bw_ref_x.init(fc_bw, DT); s.bw_ref_y.init(fc_bw, DT); s.bw_ref_z.init(fc_bw, DT);
        s.bw_acc_x.init(fc_bw, DT); s.bw_acc_y.init(fc_bw, DT); s.bw_acc_z.init(fc_bw, DT);
        s.fc_bw_last = fc_bw;
        let notch_f0 = g_indi_notch_f0;
        let notch_bw = g_indi_notch_bw;
        s.notch_x.init(notch_f0, notch_bw, DT);     s.notch_y.init(notch_f0, notch_bw, DT);     s.notch_z.init(notch_f0, notch_bw, DT);
        s.notch_ref_x.init(notch_f0, notch_bw, DT); s.notch_ref_y.init(notch_f0, notch_bw, DT); s.notch_ref_z.init(notch_f0, notch_bw, DT);
        s.notch_tau_x.init(notch_f0, notch_bw, DT); s.notch_tau_y.init(notch_f0, notch_bw, DT); s.notch_tau_z.init(notch_f0, notch_bw, DT);
        s.notch_f0_last = notch_f0;
        s.notch_bw_last = notch_bw;
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
        s.bw_pre_x.init(fc_bw, DT_NOM); s.bw_pre_y.init(fc_bw, DT_NOM); s.bw_pre_z.init(fc_bw, DT_NOM);
        s.bw_tau_x.init(fc_bw, DT_NOM); s.bw_tau_y.init(fc_bw, DT_NOM); s.bw_tau_z.init(fc_bw, DT_NOM);
        s.bw_ref_x.init(fc_bw, DT_NOM); s.bw_ref_y.init(fc_bw, DT_NOM); s.bw_ref_z.init(fc_bw, DT_NOM);
        s.bw_acc_x.init(fc_bw, DT_NOM); s.bw_acc_y.init(fc_bw, DT_NOM); s.bw_acc_z.init(fc_bw, DT_NOM);
        s.fc_bw_last = fc_bw;
    }

    // Reinit the optional notch filter when its center/bandwidth change at runtime
    // (independent of fc_bw above -- separate params, separate change-detection).
    let notch_f0 = g_indi_notch_f0;
    let notch_bw = g_indi_notch_bw;
    if (notch_f0 - s.notch_f0_last).abs() > 0.05 || (notch_bw - s.notch_bw_last).abs() > 0.05 {
        s.notch_x.init(notch_f0, notch_bw, DT_NOM);     s.notch_y.init(notch_f0, notch_bw, DT_NOM);     s.notch_z.init(notch_f0, notch_bw, DT_NOM);
        s.notch_ref_x.init(notch_f0, notch_bw, DT_NOM); s.notch_ref_y.init(notch_f0, notch_bw, DT_NOM); s.notch_ref_z.init(notch_f0, notch_bw, DT_NOM);
        s.notch_tau_x.init(notch_f0, notch_bw, DT_NOM); s.notch_tau_y.init(notch_f0, notch_bw, DT_NOM); s.notch_tau_z.init(notch_f0, notch_bw, DT_NOM);
        s.notch_f0_last = notch_f0;
        s.notch_bw_last = notch_bw;
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
    // Residual-network weights: same idx/value/commit protocol, same 500 Hz service rate.
    rnn_service();

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

    // One-shot self-stop: when traj.reps > 0 and all laps are done, switch back to
    // Mode B passthrough so the laptop keepalive cmdFullState takes over immediately
    // at the rest-to-rest endpoint — no early-stop margin needed on the laptop side.
    if g_traj_mode == 1 && TRAJ_T0 > 0 && g_traj_reps > 0 {
        let t_elapsed = tick.wrapping_sub(TRAJ_T0) as f32 * 0.001_f32;
        let n_segs_chk = (g_traj_n_segs as usize).min(TRAJ_MAX_SEGS);
        // Timeline end = entry (once) + reps × core + exit (once). With
        // n_entry=n_exit=0 this is total_dur × reps — identical to before.
        let n_entry = (g_traj_n_entry as usize).min(n_segs_chk);
        let n_exit  = (g_traj_n_exit  as usize).min(n_segs_chk - n_entry);
        let mut t_entry = 0.0_f32;
        let mut t_core  = 0.0_f32;
        let mut t_exit  = 0.0_f32;
        for i in 0..n_segs_chk {
            let d = g_traj_coefs[i * TRAJ_FLOATS_PER_SEG];
            if i < n_entry { t_entry += d; }
            else if i < n_segs_chk - n_exit { t_core += d; }
            else { t_exit += d; }
        }
        let t_total = t_entry + t_core * g_traj_reps as f32 + t_exit;
        if t_core > 0.0 && t_elapsed >= t_total {
            g_traj_mode = 0;
            g_traj_start = 0;
            TRAJ_T0 = 0;
        }
    }

    // Body-frame convention for omega_d / alpha_des (see g_indi_frame_conv).
    // Applies to the passthrough (Mode E / HLC) branch only — Mode D is pinned below.
    let frame_conv = g_indi_frame_conv;

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
        // Frame convention is PINNED to Faessler here and deliberately ignores
        // g_indi_frame_conv: Mode D is the completed INDI project's flight artifact and
        // must keep reproducing the gains/kt ceilings it was commissioned with. The
        // Mellinger correction applies to the Mode E path only.
        let omega_d  = omega_desired(ad, jerk_3d, 0.0_f32, FRAME_CONV_MODE_D);
        // α_des feedforward: snap → Ω̇_ref via flatness (Tal & Karaman Eq. 15).
        let alpha_des = alpha_desired(ad, jerk_3d, snap_3d, 0.0_f32, FRAME_CONV_MODE_D);
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
        let yaw_d_local = sp.attitude.yaw * deg2rad;
        // ω_d source — see g_indi_omega_src in traj_iface.c.
        //   0 (default): HLC's flatness-derived body rates from the setpoint (deg/s → rad/s).
        //   1: recompute locally from the HLC's jerk with omega_desired(), so ω_d uses the
        //      SAME body-frame construction as desired_rot/R_d — i.e. Mode D's convention.
        // Mode 1 needs jerk to be meaningful; cmdFullState memsets it to zero (Mode B, hover
        // keepalive, takeoff/land streaming), so fall back to the setpoint rate when jerk is
        // negligible. That keeps every non-HLC caller byte-identical under either setting.
        let sp_omega = Vec3::new(
            sp.attitudeRate.roll  * deg2rad,
            sp.attitudeRate.pitch * deg2rad,
            sp.attitudeRate.yaw   * deg2rad,
        );
        let omega_d = if g_indi_omega_src == 1 && jerk.norm() > 1e-4 {
            omega_desired(ad, jerk, yaw_d_local, frame_conv)
        } else {
            sp_omega
        };
        // α_des via differential flatness (Tal & Karaman Eq. 15) using HLC's jerk + snap
        let alpha_des = alpha_desired(ad, jerk, snap, yaw_d_local, frame_conv);
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

    // INDI windup guard: tau_prev integrates while unarmed (rpms=0 → falls back to tau_prev).
    // Reset it every tick while unarmed so INDI starts from zero on first armed tick.
    if !armed && mode & 2 != 0 {
        s.tau_prev = Vec3::zero();
    }

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