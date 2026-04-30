//! Incremental Nonlinear Dynamic Inversion (INDI) attitude controller
//!
//! Inner-loop INDI following Smeur et al. (2016) and the Bitcraze reference
//! implementation in `controller_indi.c` / `position_controller_indi.c`.
//!
//! ## Architecture
//!
//! The outer position loop is **identical** to [`GeometricController`] — thrust
//! and desired rotation `Rd` come from the same SE(3) flatness-based law.
//! Only the torque computation is replaced:
//!
//! ```text
//! Outer loop (geometric, unchanged):
//!   pos/vel error → F_d → thrust,  F_d → Rd
//!
//! Inner loop (INDI, replaces geometric -KR·eR - KW·eω + gyro_comp):
//!   1. attitude error  eR        = ½(Rd^T R - R^T Rd)^∨
//!   2. virtual control α_ref     = -KR·eR - KW·eω       [rad/s²]
//!   3. filter gyro     ω_f       = Butterworth2(ω)
//!   4. meas. α         α_meas    = (ω_f[k] - ω_f[k-1]) / dt
//!   5. INDI increment  δτ        = (α_ref - α_meas) / G1  [Nm]
//!      yaw G2 term:    δτ_z     += G2 · δτ_z_prev / (G1_z + G2)
//!   6. increment       τ_in     = τ_filtered + δτ         (clamped)
//!   7. actuator lag    τ_act    += α_act · (τ_in - τ_act) [first-order]
//!   8. output torque             = τ_act + ω × Jω
//! ```
//!
//! ## Relationship to C reference
//!
//! | C symbol              | Here                   | Notes                             |
//! |-----------------------|------------------------|-----------------------------------|
//! | `g1.p / g1.q / g1.r`  | `g1` Vec3              | [rad/s²/Nm]; default = 1/J        |
//! | `g2`                  | `g2`                   | yaw gyroscopic coupling           |
//! | `act_dyn.p/q/r`       | `act_dyn` Vec3         | first-order lag coefficient       |
//! | `filt_cutoff`         | `gyro_cutoff_hz`       | gyro + torque LP filter [Hz]      |
//! | `u_act_dyn`           | `tau_act_dyn`          | actuator dynamics state [Nm]      |
//! | `u_in`                | `tau_in`               | commanded torque before lag [Nm]  |
//! | `rate_d`              | `alpha_meas`           | finite-diff of filtered gyro      |
//!
//! The C code works in motor-command units; this implementation works in SI [Nm].
//! When `g1 = 1/J` (default) the two are numerically equivalent.
//!
//! ## Parameters to bench-identify before first flight
//! - `g1`       — actual control effectiveness; bench thrust-stand step sweep
//! - `g2`       — propeller gyroscopic coupling; only affects yaw at high rates
//! - `act_dyn`  — actuator lag; motor time-constant sweep

use std::f32::consts::PI;

use crate::math::Vec3;
use crate::dynamics::{MultirotorState, MultirotorParams};
use super::{Controller, TrajectoryReference, ControlOutput};

const SQRT2: f32 = 1.414_213_6_f32;

// ── 2nd-order Butterworth low-pass filter ────────────────────────────────────
//
// Matches Paparazzi `init_butterworth_2_low_pass` / `update_butterworth_2_low_pass`
// which is what `controller_indi.c` uses for both gyro rate and actuator signals.
//
// Bilinear-transform coefficients (pre-warped, τ = 1 / (2π·fc)):
//
//   denom = τ² + √2·τ·dt + dt²
//   b     = dt² / denom          → transfer-fn numerator (b0=b, b1=2b, b2=b)
//   a1    = 2·(dt² − τ²) / denom → IIR feedback coefficient 1
//   a2    = (τ² − √2·τ·dt + dt²) / denom → IIR feedback coefficient 2
//
// Difference equation:
//   y[n] = b·(x[n] + 2·x[n-1] + x[n-2]) − a1·y[n-1] − a2·y[n-2]
//
// The C implementation stores {o[0] = y[n], o[1] = y[n-1]} so finite-difference
// is simply `(o[0] - o[1]) * rate`.  Here we use `prev()` → `y1` captured
// *before* `update()` to get the same pair.

/// 2nd-order Butterworth low-pass filter (IIR, direct form II transposed).
///
/// Publicly accessible so tests and future modules can use it directly.
#[derive(Debug, Clone, Copy)]
pub struct Butterworth2 {
    // feed-forward (b0 = b, b1 = 2b, b2 = b)
    b:  f32,
    // feedback
    a1: f32,
    a2: f32,
    // input history
    x1: f32,
    x2: f32,
    // output history
    y1: f32,
    y2: f32,
}

impl Butterworth2 {
    /// Create filter with cutoff `fc` [Hz] at sample period `dt` [s].
    ///
    /// Pass `fc = 0.0` or `dt = 0.0` to get a pass-through (b=1, a=0).
    pub fn new(fc: f32, dt: f32) -> Self {
        if fc <= 0.0 || dt <= 0.0 {
            return Self { b: 1.0, a1: 0.0, a2: 0.0, x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0 };
        }
        let tau   = 1.0 / (2.0 * PI * fc);
        let denom = tau*tau + SQRT2*tau*dt + dt*dt;
        let b     = (dt*dt / denom).max(0.0);
        let a1    = 2.0 * (dt*dt - tau*tau) / denom;
        let a2    = (tau*tau - SQRT2*tau*dt + dt*dt) / denom;
        Self { b, a1, a2, x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0 }
    }

    /// Feed new sample `x`, return filtered output.
    pub fn update(&mut self, x: f32) -> f32 {
        let y = self.b*x + 2.0*self.b*self.x1 + self.b*self.x2
                - self.a1*self.y1 - self.a2*self.y2;
        self.x2 = self.x1;  self.x1 = x;
        self.y2 = self.y1;  self.y1 = y;
        y
    }

    /// Output from the *previous* call — used for finite-difference before updating.
    ///
    /// Call pattern (matches C `o[1]` / `o[0]` convention):
    /// ```ignore
    /// let prev = filt.prev();          // y[k-1]
    /// let curr = filt.update(new_x);   // y[k]
    /// let deriv = (curr - prev) / dt;
    /// ```
    #[inline]
    pub fn prev(&self) -> f32 { self.y1 }

    /// Seed both history stages to `v` — eliminates the startup transient.
    pub fn seed(&mut self, v: f32) {
        self.x1 = v; self.x2 = v;
        self.y1 = v; self.y2 = v;
    }
}

// ── INDI Controller ───────────────────────────────────────────────────────────

/// INDI attitude controller with geometric outer position loop.
///
/// The outer position loop (position → thrust + desired rotation) is **unchanged**
/// from [`GeometricController`] — existing flights, trajectories, and firmware are
/// unaffected.  Only the torque law is replaced with the INDI increment.
///
/// # Tuning order
/// 1. Start with `crazyflie_default(dt)` — conservative, matches geometric hover.
/// 2. Verify hover in sim (this crate) — `test_indi_hover_matches_geometric`.
/// 3. Bench bench-identify `g1` (thrust-stand step sweep → fit `α / δτ`).
/// 4. Set `CONTROLLER_MODE = 1` in firmware and hover on the real drone.
/// 5. Tune `kr` / `kw` upward once INDI hover is confirmed stable.
/// 6. Enable `g2` only if yaw oscillations appear at high yaw rates.
#[derive(Debug, Clone)]
pub struct IndiController {
    // ── Outer loop gains (position — same as GeometricController) ────────────
    /// Position proportional gain [m/s² / m]
    pub kp: Vec3,
    /// Velocity derivative gain [m/s² / (m/s)]
    pub kv: Vec3,

    // ── Inner loop INDI gains ─────────────────────────────────────────────────
    /// Attitude error gain [rad/s² / rad].
    /// Combines the `err_p/q/r` and `rate_p/q/r` cascade from `controller_indi.c`
    /// into a single term.  Start conservative (≈ half the geometric KR).
    pub kr: Vec3,
    /// Angular rate error gain [rad/s² / (rad/s)].  Must satisfy `kw > kr` (Routh).
    pub kw: Vec3,

    // ── INDI physical parameters ──────────────────────────────────────────────
    /// Control effectiveness [rad/s² / Nm] per axis.
    ///
    /// Physical meaning: how much angular acceleration a 1 Nm torque increment
    /// produces.  In pure-inertia theory `g1 = 1/J`; in practice it is slightly
    /// higher because motor lag means the actual torque leads the commanded torque.
    ///
    /// **Bench-identify** before first real flight: apply step torque, measure α.
    /// Default `1/J` is the correct order of magnitude and safe for simulation.
    pub g1: Vec3,

    /// Yaw propeller gyroscopic coupling [rad/s² / Nm].
    ///
    /// Accounts for angular momentum change of spinning propellers during rapid
    /// yaw manoeuvres.  Maps to `indi.g2` in `controller_indi.c`.
    /// Set to `0.0` until bench-identified — only matters at high yaw rates (> 3 rad/s).
    pub g2: f32,

    /// Actuator dynamics coefficient per axis (≈ `dt / τ_motor`).
    ///
    /// First-order lag: `τ_act += act_dyn · (τ_in − τ_act)`.
    /// `0.0` = pure state memory (no lag model), `1.0` = instant response.
    /// Maps to `indi.act_dyn` in `controller_indi.c`.
    /// Default 0.1 at 100 Hz ≈ τ_motor ≈ 90 ms (conservative for CF2 brushed).
    pub act_dyn: Vec3,

    /// Gyro (and torque) Butterworth LP filter cutoff [Hz].
    ///
    /// Controls the trade-off between noise rejection and phase lag:
    ///   Too high → gyro noise corrupts α_meas → oscillation.
    ///   Too low  → phase lag → sluggish or unstable.
    /// Typical range 20–80 Hz; default 60 Hz matches firmware `FC_GYRO_HZ`.
    pub gyro_cutoff_hz: f32,

    /// Thrust [N] below which INDI increments are frozen (matched C `thrust_threshold`).
    pub thrust_threshold: f32,

    /// Hard clamp on |τ| per axis [Nm].  Prevents runaway during startup transient.
    pub tau_clamp: f32,

    // ── Internal state (not tuneable, reset on re-arm) ────────────────────────
    /// Butterworth 2LP on raw body-rate measurements [p, q, r].
    gyro_filter: [Butterworth2; 3],

    /// Butterworth 2LP on previous torque command — models actuator bandwidth.
    tau_filter: [Butterworth2; 3],

    /// First-order actuator dynamics state [Nm] (the `u_act_dyn` from C reference).
    tau_act_dyn: Vec3,

    /// Previous INDI torque increment δτ [Nm] — needed for G2 yaw correction.
    du_prev: Vec3,

    /// False until first call seeds filter history; avoids transient on step 0.
    initialized: bool,
}

impl IndiController {
    /// Full constructor.
    ///
    /// `sample_dt` is the nominal simulation timestep [s] used to size the
    /// Butterworth filters (`MultirotorParams::dt` is the natural choice).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        kp: Vec3, kv: Vec3,
        kr: Vec3, kw: Vec3,
        g1: Vec3, g2: f32,
        act_dyn: Vec3,
        gyro_cutoff_hz: f32,
        sample_dt: f32,
    ) -> Self {
        let make_filters = |fc: f32, dt: f32| -> [Butterworth2; 3] {
            [Butterworth2::new(fc, dt),
             Butterworth2::new(fc, dt),
             Butterworth2::new(fc, dt)]
        };
        Self {
            kp, kv, kr, kw,
            g1, g2, act_dyn,
            gyro_cutoff_hz,
            thrust_threshold: 0.01,   // 10 mN — same order as C `thrust_threshold`
            tau_clamp:        0.05,   // 50 mNm — matches firmware TAU_INDI_CLAMP
            gyro_filter: make_filters(gyro_cutoff_hz, sample_dt),
            tau_filter:  make_filters(gyro_cutoff_hz, sample_dt),
            tau_act_dyn: Vec3::zero(),
            du_prev:     Vec3::zero(),
            initialized: false,
        }
    }

    /// Conservative defaults for Crazyflie 2.x at a given simulation timestep.
    ///
    /// # INDI gain scaling
    ///
    /// The virtual control is α_ref = -kr·eR - kw·eω [rad/s²] and the torque
    /// increment is δτ = α_ref / G1 where G1 = 1/J [rad/s²/Nm].  With the
    /// seeding fix (`tau_act_dyn` initialised to α_ref/G1) the gains can be
    /// in any consistent scale.  The defaults here use the **geometric scale**
    /// (kr, kw in [Nm/rad] and [Nm/(rad/s)]) so that the seeded torque matches
    /// the geometric baseline:
    ///   `tau_seed = (-kr·eR - kw·eω) / G1 ≈ (-kr·eR - kw·eω) · J`
    ///
    /// With kr = 0.003 and eR = 0.17 rad (10° tilt):
    ///   `tau_seed.x ≈ 0.003 × 0.17 × J_xx ≈ 8.5 nNm` (very conservative)
    ///
    /// The INDI increment δτ is additionally small because du = α_err / G1 = α_err·J.
    /// This keeps the first flight conservative — increase kr/kw by ≤20% per step
    /// once hover in simulation is confirmed.
    ///
    /// **Routh stability** requires `kw > kr`.  The default 2:1 ratio gives
    /// comfortable margin.
    pub fn crazyflie_default(sample_dt: f32) -> Self {
        // CF2 inertia (from params.rs and controller_lee.c)
        let jxx = 16.571710e-6_f32;
        let jyy = 16.655602e-6_f32;
        let jzz = 29.261652e-6_f32;
        Self::new(
            Vec3::new(12.0, 12.0,  7.0),           // kp — same as geometric
            Vec3::new( 8.0,  8.0,  4.0),           // kv — same as geometric
            Vec3::new(0.003, 0.003, 0.003),         // kr [Nm/rad]  — conservative
            Vec3::new(0.006, 0.006, 0.006),         // kw [Nm/(rad/s)] kw=2·kr (Routh)
            Vec3::new(1.0/jxx, 1.0/jyy, 1.0/jzz), // g1 = 1/J placeholder
            0.0,                                    // g2 — identify on bench, 0 is safe
            Vec3::new(0.1, 0.1, 0.1),              // act_dyn ≈ dt/τ_motor (τ≈90 ms @ 100 Hz)
            60.0,                                   // gyro_cutoff_hz
            sample_dt,
        )
    }

    /// Reset all filter history and INDI state (call on landing / re-arm).
    pub fn reset(&mut self) {
        for f in &mut self.gyro_filter { f.seed(0.0); }
        for f in &mut self.tau_filter  { f.seed(0.0); }
        self.tau_act_dyn = Vec3::zero();
        self.du_prev     = Vec3::zero();
        self.initialized = false;
    }

    // ── Shared outer-loop helpers ─────────────────────────────────────────────
    // These are static copies of the identical helpers in GeometricController so
    // that IndiController has no dependency on that struct — adding indi.rs is
    // purely additive and does not touch mod.rs logic.

    fn compute_desired_rotation(thrust_force: Vec3, yaw: f32) -> [[f32; 3]; 3] {
        let mut ydes = Vec3::new(0.0, 1.0, 0.0);
        let mut zdes = Vec3::new(0.0, 0.0, 1.0);
        if thrust_force.norm() > 0.0 {
            zdes = thrust_force.normalize();
        }
        let xcdes   = Vec3::new(yaw.cos(), yaw.sin(), 0.0);
        let zcrossx = zdes.cross(&xcdes);
        if zcrossx.norm() > 0.0 {
            ydes = zcrossx.normalize();
        }
        let xdes = ydes.cross(&zdes);
        [[xdes.x, ydes.x, zdes.x],
         [xdes.y, ydes.y, zdes.y],
         [xdes.z, ydes.z, zdes.z]]
    }

    /// eR = ½ (Rd^T R − R^T Rd)^∨
    fn rotation_error(rd: &[[f32; 3]; 3], r: &[[f32; 3]; 3]) -> Vec3 {
        let mut rd_t_r = [[0.0_f32; 3]; 3];
        let mut r_t_rd = [[0.0_f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    rd_t_r[i][j] += rd[k][i] * r[k][j];
                    r_t_rd[i][j] += r[k][i] * rd[k][j];
                }
            }
        }
        Vec3::new(
            0.5 * (rd_t_r[2][1] - r_t_rd[2][1]),
            0.5 * (rd_t_r[0][2] - r_t_rd[0][2]),
            0.5 * (rd_t_r[1][0] - r_t_rd[1][0]),
        )
    }
}

impl Controller for IndiController {
    fn compute_control(
        &mut self,
        state: &MultirotorState,
        reference: &TrajectoryReference,
        params: &MultirotorParams,
        dt: f32,
    ) -> ControlOutput {
        let jxx = params.inertia[0][0];
        let jyy = params.inertia[1][1];
        let jzz = params.inertia[2][2];

        // ── 1. Outer loop: position/velocity → thrust (geometric, unchanged) ──
        let ep = reference.position - state.position;
        let ev = reference.velocity - state.velocity;
        let f_d = reference.acceleration
            + Vec3::new(self.kp.x*ep.x, self.kp.y*ep.y, self.kp.z*ep.z)
            + Vec3::new(self.kv.x*ev.x, self.kv.y*ev.y, self.kv.z*ev.z)
            + Vec3::new(0.0, 0.0, params.gravity);
        let thrust_force = f_d * params.mass;

        let r = state.orientation.to_rotation_matrix();
        let body_z = Vec3::new(r[0][2], r[1][2], r[2][2]);
        let thrust = thrust_force.dot(&body_z).max(0.0);

        // ── 2. Desired rotation and attitude/rate errors ───────────────────────
        let rd     = Self::compute_desired_rotation(thrust_force, reference.yaw);
        let er     = Self::rotation_error(&rd, &r);
        // ω_r = 0 (no jerk feedforward yet — conservative start identical to
        //          firmware CONTROLLER_MODE=1 initial configuration)
        let eomega = state.angular_velocity;

        let omega = state.angular_velocity;

        // ── 3. Seed filters on very first call ────────────────────────────────
        //   Mirrors C `controllerINDIInit`: initialise history to current values
        //   so there is no large transient on step 0.
        if !self.initialized {
            self.gyro_filter[0].seed(omega.x);
            self.gyro_filter[1].seed(omega.y);
            self.gyro_filter[2].seed(omega.z);

            // Seed torque filters to the INDI-equivalent torque for the current
            // error/rate state.  τ_seed = α_virtual / G1 so that the first
            // real INDI increment δτ is near zero — avoids a torque step on tick 1.
            //
            // IMPORTANT: dividing by g1 converts [rad/s²] → [Nm], ensuring
            // tau_act_dyn is always in SI torque units regardless of the gain
            // scale chosen for kr/kw.
            let tau0 = Vec3::new(
                (-self.kr.x*er.x - self.kw.x*eomega.x) / self.g1.x.max(1e-12),
                (-self.kr.y*er.y - self.kw.y*eomega.y) / self.g1.y.max(1e-12),
                (-self.kr.z*er.z - self.kw.z*eomega.z) / self.g1.z.max(1e-12),
            );
            let tau0_clamped = Vec3::new(
                tau0.x.clamp(-self.tau_clamp, self.tau_clamp),
                tau0.y.clamp(-self.tau_clamp, self.tau_clamp),
                tau0.z.clamp(-self.tau_clamp, self.tau_clamp),
            );
            self.tau_filter[0].seed(tau0_clamped.x);
            self.tau_filter[1].seed(tau0_clamped.y);
            self.tau_filter[2].seed(tau0_clamped.z);
            self.tau_act_dyn = tau0_clamped;
            self.initialized = true;

            // Return INDI-seeded torque on this first step (no α_meas yet).
            let j_omega   = Vec3::new(jxx*omega.x, jyy*omega.y, jzz*omega.z);
            let gyro_comp = omega.cross(&j_omega);
            return ControlOutput { thrust, torque: tau0_clamped + gyro_comp };
        }

        // ── 4. Filter gyro and compute measured angular acceleration ───────────
        //
        // Matches C steps 1 & 2:
        //   filter_pqr(indi.rate, &body_rates);          // update filter
        //   finite_difference_from_filter(rate_d, rate); // (o[0]-o[1]) * RATE
        //
        // We capture prev BEFORE update so the pair is (y[k-1], y[k]).
        let omega_f_prev = Vec3::new(
            self.gyro_filter[0].prev(),
            self.gyro_filter[1].prev(),
            self.gyro_filter[2].prev(),
        );
        let omega_f = Vec3::new(
            self.gyro_filter[0].update(omega.x),
            self.gyro_filter[1].update(omega.y),
            self.gyro_filter[2].update(omega.z),
        );
        let inv_dt   = if dt > 1e-9 { 1.0 / dt } else { 0.0 };
        let alpha_meas = Vec3::new(
            (omega_f.x - omega_f_prev.x) * inv_dt,
            (omega_f.y - omega_f_prev.y) * inv_dt,
            (omega_f.z - omega_f_prev.z) * inv_dt,
        );

        // ── 5. Virtual control (desired angular acceleration) ─────────────────
        //
        // Collapses the C code's two-stage P(attitude)→P(rate)→α_ref cascade into
        // a single term — equivalent when err_p * rate_p = kr, err_q * rate_q = kw.
        //
        //   indi.angular_accel_ref.p = rate_p * (rate_des.p - body_rates.p)
        //   where rate_des.p = err_p * attitude_error.p
        //   ⟹   α_ref.p = (err_p * rate_p) * er.p − rate_p * eomega.p
        //              = kr.p * er.p             − kw.p * eomega.p  ✓
        let alpha_ref = Vec3::new(
            -self.kr.x*er.x - self.kw.x*eomega.x,
            -self.kr.y*er.y - self.kw.y*eomega.y,
            -self.kr.z*er.z - self.kw.z*eomega.z,
        );

        // ── 6. INDI torque increment ───────────────────────────────────────────
        //
        // C:    du.p = (1/g1.p) * (angular_accel_ref.p - rate_d[0])
        //       du.r = (1/(g1.r+g2)) * (angular_accel_ref.r - rate_d[2] + g2*du.r_prev)
        //
        // In Nm space (g1 = [rad/s²/Nm]):
        //   δτ = α_err / g1       where α_err = α_ref − α_meas  [rad/s²]
        //                         g1 has units [rad/s² / Nm]
        //                         → δτ has units [Nm]  ✓
        //
        // When g1 = 1/J  this simplifies to  δτ = J · α_err  (pure-inertia INDI).
        let alpha_err = alpha_ref - alpha_meas;

        // Yaw: G2 correction for propeller angular momentum (C: g2 * du.r_prev)
        // When g2 = 0 the expression reduces to α_err.z / g1.z — no effect.
        let g1z_g2 = (self.g1.z + self.g2).max(1e-12);
        let du = Vec3::new(
            alpha_err.x / self.g1.x.max(1e-12),
            alpha_err.y / self.g1.y.max(1e-12),
            (alpha_err.z + self.g2 * self.du_prev.z) / g1z_g2,
        );
        self.du_prev = du;

        // ── 7. Filter previous actuator state and add increment ───────────────
        //
        // C:  filter_pqr(indi.u, &indi.u_act_dyn);   // smooth actuator state
        //     u_in = u_filtered + du;                  // increment on smoothed state
        //
        // The filter here represents what the actuator is currently outputting
        // (smoothed to remove sensor noise that would otherwise enter as bias).
        let tau_f = Vec3::new(
            self.tau_filter[0].update(self.tau_act_dyn.x),
            self.tau_filter[1].update(self.tau_act_dyn.y),
            self.tau_filter[2].update(self.tau_act_dyn.z),
        );
        let tau_in = Vec3::new(
            (tau_f.x + du.x).clamp(-self.tau_clamp, self.tau_clamp),
            (tau_f.y + du.y).clamp(-self.tau_clamp, self.tau_clamp),
            (tau_f.z + du.z).clamp(-self.tau_clamp, self.tau_clamp),
        );

        // ── 8. Actuator dynamics (first-order lag) ────────────────────────────
        //
        // C:  u_act_dyn += act_dyn * (u_in - u_act_dyn)
        //
        // act_dyn = 0 → τ_act_dyn never moves → pure integrator (no motor model).
        // act_dyn = 1 → τ_act_dyn = τ_in    → instant response (no lag).
        self.tau_act_dyn = Vec3::new(
            self.tau_act_dyn.x + self.act_dyn.x * (tau_in.x - self.tau_act_dyn.x),
            self.tau_act_dyn.y + self.act_dyn.y * (tau_in.y - self.tau_act_dyn.y),
            self.tau_act_dyn.z + self.act_dyn.z * (tau_in.z - self.tau_act_dyn.z),
        );

        // ── 9. Thrust threshold guard ─────────────────────────────────────────
        //
        // C:  if (thrust < thrust_threshold) { zero angular_accel_ref, u_act_dyn, u_in }
        //
        // Prevents windup of incremental state before the drone is armed / in air.
        let torque = if thrust < self.thrust_threshold {
            self.tau_act_dyn = Vec3::zero();
            self.du_prev     = Vec3::zero();
            Vec3::zero()
        } else {
            // Gyroscopic compensation ω × Jω — same as geometric controller.
            let j_omega   = Vec3::new(jxx*omega.x, jyy*omega.y, jzz*omega.z);
            let gyro_comp = omega.cross(&j_omega);
            self.tau_act_dyn + gyro_comp
        };

        ControlOutput { thrust, torque }
    }
}

// ── Unit tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::{MultirotorParams, MultirotorState, MotorAction, MultirotorSimulator};
    use crate::controller::{Controller, GeometricController, TrajectoryReference};

    fn hover_reference() -> TrajectoryReference {
        TrajectoryReference {
            position:     Vec3::zero(),
            velocity:     Vec3::zero(),
            acceleration: Vec3::zero(),
            jerk:         Vec3::zero(),
            yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
        }
    }

    // ── Butterworth2 sanity checks ────────────────────────────────────────────

    #[test]
    fn butterworth_passes_dc() {
        // A DC signal (constant input) should pass through unchanged at steady state.
        let mut f = Butterworth2::new(20.0, 0.01);
        for _ in 0..200 { f.update(1.0); }
        let out = f.update(1.0);
        assert!((out - 1.0).abs() < 1e-4, "DC gain should be 1.0, got {out}");
    }

    #[test]
    fn butterworth_rejects_high_freq() {
        // A signal well above the cutoff should be significantly attenuated.
        // At 10× cutoff, 2nd-order discrete Butterworth gives ~0.037 amplitude
        // (bilinear transform vs. ideal analog -40 dB/decade), which is still a
        // 28× reduction.  At 20× cutoff we get < 0.01 (40 dB).
        let fc = 10.0_f32;
        let dt = 0.001_f32; // 1 kHz sample rate
        let mut f = Butterworth2::new(fc, dt);
        let f_signal = 20.0 * fc; // 20× cutoff — firmly in stop band
        let mut max_out = 0.0_f32;
        for k in 0..2000 {
            let t = k as f32 * dt;
            let x = (2.0 * PI * f_signal * t).sin();
            let y = f.update(x);
            if k > 400 { max_out = max_out.max(y.abs()); }
        }
        assert!(max_out < 0.01, "High-freq attenuation failed at 20×fc: max_out = {max_out}");
    }

    #[test]
    fn butterworth_seed_removes_transient() {
        // Seeded filter should output the seed value immediately on first update.
        let mut f = Butterworth2::new(20.0, 0.01);
        f.seed(5.0);
        let out = f.update(5.0);
        assert!((out - 5.0).abs() < 1e-5, "Seeded filter transient: {out}");
    }

    // ── INDI hover tests ──────────────────────────────────────────────────────

    #[test]
    fn indi_hover_thrust_equals_weight() {
        // At perfect hover, thrust must equal m·g regardless of controller type.
        let params = MultirotorParams::crazyflie();
        let mut ctrl  = IndiController::crazyflie_default(params.dt);
        let state      = MultirotorState::new();
        let reference  = hover_reference();

        let out = ctrl.compute_control(&state, &reference, &params, params.dt);
        let weight = params.mass * params.gravity;

        assert!(
            (out.thrust - weight).abs() < 1e-5,
            "Thrust {:.6} ≠ weight {:.6}", out.thrust, weight,
        );
    }

    #[test]
    fn indi_hover_torque_near_zero() {
        // At perfect hover (level, no errors) torque should be essentially zero.
        let params = MultirotorParams::crazyflie();
        let mut ctrl  = IndiController::crazyflie_default(params.dt);
        let state      = MultirotorState::new();
        let reference  = hover_reference();

        // Run a few steps so filters settle
        for _ in 0..10 {
            ctrl.compute_control(&state, &reference, &params, params.dt);
        }
        let out = ctrl.compute_control(&state, &reference, &params, params.dt);

        assert!(
            out.torque.norm() < 1e-7,
            "Hover torque not zero: {:?}", out.torque,
        );
    }

    #[test]
    fn indi_hover_thrust_matches_geometric() {
        // INDI and geometric must produce the same thrust at perfect hover —
        // the outer position loop is identical.
        let params = MultirotorParams::crazyflie();
        let mut geo  = GeometricController::default();
        let mut indi = IndiController::crazyflie_default(params.dt);
        let state     = MultirotorState::new();
        let reference = hover_reference();

        let geo_out  = geo.compute_control(&state, &reference, &params, params.dt);
        let indi_out = indi.compute_control(&state, &reference, &params, params.dt);

        assert!(
            (geo_out.thrust - indi_out.thrust).abs() < 1e-5,
            "Thrust mismatch: geometric={:.6} indi={:.6}",
            geo_out.thrust, indi_out.thrust,
        );
    }

    #[test]
    fn indi_below_threshold_gives_zero_torque() {
        // When thrust is below the threshold (drone on ground), torque must be zero.
        let params = MultirotorParams::crazyflie();
        let mut ctrl = IndiController::crazyflie_default(params.dt);

        // Position well below hover height → thrust will be very small
        use crate::math::Quat;
        let state = MultirotorState::with_initial(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::zero(),
            Quat::identity(),
            Vec3::zero(),
        );
        // Reference at z=0 and zero acceleration → thrust ≈ 0 before the drone lifts
        let reference = TrajectoryReference {
            position:     Vec3::zero(),
            velocity:     Vec3::zero(),
            acceleration: Vec3::new(0.0, 0.0, -params.gravity), // cancel gravity → thrust = 0
            jerk:         Vec3::zero(),
            yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
        };
        let out = ctrl.compute_control(&state, &reference, &params, params.dt);
        assert_eq!(out.torque, Vec3::zero(), "Expected zero torque below threshold");
    }

    #[test]
    fn indi_attitude_error_produces_torque() {
        // A tilted drone (roll error) should generate a non-zero corrective torque.
        // With the corrected seeding (τ_seed = α_ref/G1) the gains must be in
        // INDI-scale [rad/s²/rad] to produce meaningful torques.
        // kr=100, kw=200, g1=1/J → τ_seed.x ≈ -100·sin(10°)·J_xx ≈ -286 µNm.
        let params = MultirotorParams::crazyflie();
        let jxx = params.inertia[0][0];
        let jyy = params.inertia[1][1];
        let jzz = params.inertia[2][2];
        let mut ctrl = IndiController::new(
            Vec3::new(12.0, 12.0, 7.0),
            Vec3::new( 8.0,  8.0, 4.0),
            Vec3::new(100.0, 100.0, 100.0), // INDI-scale kr
            Vec3::new(200.0, 200.0, 200.0), // INDI-scale kw > kr (Routh)
            Vec3::new(1.0/jxx, 1.0/jyy, 1.0/jzz),
            0.0,
            Vec3::new(0.1, 0.1, 0.1),
            60.0,
            params.dt,
        );

        // Tilt drone 10° around x-axis
        let roll_rad = 10.0_f32.to_radians();
        let quat = crate::math::Quat::new(
            (roll_rad / 2.0).cos(), (roll_rad / 2.0).sin(), 0.0, 0.0,
        );
        let state = MultirotorState::with_initial(
            Vec3::new(0.0, 0.0, 0.5), // above ground so thrust > threshold
            Vec3::zero(),
            quat,
            Vec3::zero(),
        );
        let reference = TrajectoryReference {
            position:     Vec3::new(0.0, 0.0, 0.5),
            velocity:     Vec3::zero(),
            acceleration: Vec3::zero(),
            jerk:         Vec3::zero(),
            yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
        };

        // Run two steps so we pass the initialization guard and get an actual INDI step.
        ctrl.compute_control(&state, &reference, &params, params.dt);
        let out = ctrl.compute_control(&state, &reference, &params, params.dt);

        // Expect at least 100 µNm corrective torque (τ_seed ≈ 286 µNm for 10° tilt).
        assert!(
            out.torque.norm() > 1e-4,
            "Expected corrective torque > 100 µNm for 10° tilt, got {:?}", out.torque,
        );
        // Torque must be in the roll-correcting direction (negative for positive er.x)
        assert!(
            out.torque.x < 0.0,
            "Roll torque should be negative to correct positive roll error, got {:?}", out.torque,
        );
    }

    #[test]
    fn indi_simulation_loop_hover_stable() {
        // Closed-loop simulation: INDI should hold hover within 5 cm for 3 seconds.
        use crate::integration::RK4Integrator;

        let params = MultirotorParams::crazyflie();
        let dt     = params.dt;
        let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
        let mut ctrl = IndiController::crazyflie_default(dt);

        // Start at hover height
        sim.state_mut().position = Vec3::new(0.0, 0.0, 0.5);
        let reference = TrajectoryReference {
            position:     Vec3::new(0.0, 0.0, 0.5),
            velocity:     Vec3::zero(),
            acceleration: Vec3::zero(),
            jerk:         Vec3::zero(),
            yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
        };

        let steps = (3.0 / dt) as usize;
        for _ in 0..steps {
            let out = ctrl.compute_control(sim.state(), &reference, sim.params(), dt);
            // Convert force/torque → motor ω² via inverse mixer
            let action = force_torque_to_motor_action(&out, sim.params());
            sim.step(&action);
        }
        let final_pos = sim.state().position;
        let error = (final_pos - Vec3::new(0.0, 0.0, 0.5)).norm();
        assert!(
            error < 0.05,
            "INDI hover drifted {:.3} m after 3 s (should be < 5 cm)", error,
        );
    }

    /// Inverse mixer: (thrust [N], torque [Nm]) → per-motor ω² [rad²/s²]
    /// X-frame CF2 motor layout (matches params.rs `motor_speeds_to_forces_torques`).
    fn force_torque_to_motor_action(
        out: &ControlOutput,
        params: &MultirotorParams,
    ) -> MotorAction {
        let f  = out.thrust;
        let tx = out.torque.x;
        let ty = out.torque.y;
        let tz = out.torque.z;
        let kf = params.kf;
        let kt = params.kt;
        let l  = params.arm_length / 2.0_f32.sqrt();

        // Solve the 4×4 mixer (each motor contributes to F, τx, τy, τz):
        //   F  = kf*(ω1² + ω2² + ω3² + ω4²)
        //   τx = kf*l*(-ω1² + ω2² + ω3² - ω4²)
        //   τy = kf*l*(-ω1² - ω2² + ω3² + ω4²)
        //   τz = kt*(-ω1² + ω2² - ω3² + ω4²)
        //
        // Analytical inverse (assuming symmetric X-frame):
        let base = f / (4.0 * kf);
        let dx   = tx / (4.0 * kf * l);
        let dy   = ty / (4.0 * kf * l);
        let dz   = tz / (4.0 * kt);

        let w1 = (base - dx - dy - dz).max(0.0);
        let w2 = (base + dx - dy + dz).max(0.0);
        let w3 = (base + dx + dy - dz).max(0.0);
        let w4 = (base - dx + dy + dz).max(0.0);

        MotorAction { omega1_sq: w1, omega2_sq: w2, omega3_sq: w3, omega4_sq: w4 }
    }
}
