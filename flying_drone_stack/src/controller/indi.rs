use std::f32::consts::PI;

use crate::math::Vec3;
use crate::dynamics::{MultirotorState, MultirotorParams};
use super::{Controller, TrajectoryReference, ControlOutput};

const SQRT2: f32 = std::f32::consts::SQRT_2;

// ── Stage-1 filter: second-order Butterworth ──────────────────────────────────
//
// Applied to the raw gyro signal ω before differentiation.
// Removes motor-vibration and broadband sensor noise from the source signal.
//
// Bilinear-transform coefficients (pre-warped, τ = 1/(2π·fc)):
//   denom = τ² + √2·τ·Δt + Δt²
//   b     = Δt² / denom                          (b0 = b, b1 = 2b, b2 = b)
//   a1    = 2·(Δt² − τ²) / denom
//   a2    = (τ² − √2·τ·Δt + Δt²) / denom
//
// Difference equation:
//   y[n] = b·(x[n] + 2·x[n−1] + x[n−2]) − a1·y[n−1] − a2·y[n−2]

/// Second-order Butterworth low-pass filter (single axis, fixed Δt).
///
/// Coefficients are computed once from `fc` and `dt` at construction.
/// Both must be positive and `dt` must match the actual control timestep.
#[derive(Debug, Clone)]
pub struct Butterworth2 {
    b:  f32,
    a1: f32, a2: f32,
    x1: f32, x2: f32,
    y1: f32, y2: f32,
}

impl Butterworth2 {
    /// Create a Butterworth filter with cutoff `fc` [Hz] at sample period `dt` [s].
    /// Pass `fc = 0` or `dt = 0` to obtain a transparent pass-through (b=1, a=0).
    pub fn new(fc: f32, dt: f32) -> Self {
        if fc <= 0.0 || dt <= 0.0 {
            return Self { b: 1.0, a1: 0.0, a2: 0.0,
                          x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0 };
        }
        let tau   = 1.0 / (2.0 * PI * fc);
        let denom = tau*tau + SQRT2*tau*dt + dt*dt;
        let b     = dt*dt / denom;
        let a1    = 2.0 * (dt*dt - tau*tau) / denom;
        let a2    = (tau*tau - SQRT2*tau*dt + dt*dt) / denom;
        Self { b, a1, a2, x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0 }
    }

    /// Feed a new sample; returns the filtered output.
    pub fn update(&mut self, x: f32) -> f32 {
        let y = self.b*x + 2.0*self.b*self.x1 + self.b*self.x2
              - self.a1*self.y1 - self.a2*self.y2;
        self.x2 = self.x1; self.x1 = x;
        self.y2 = self.y1; self.y1 = y;
        y
    }

    /// Seed all internal history stages to `v`.
    /// Eliminates the startup transient when the filter resumes from a known value.
    pub fn seed(&mut self, v: f32) {
        self.x1 = v; self.x2 = v;
        self.y1 = v; self.y2 = v;
    }

    pub fn reset(&mut self) { self.seed(0.0); }

    /// Last output — `y[k−1]` — useful for inspecting filter state.
    pub fn output(&self) -> f32 { self.y1 }
}

// ── Stage-3 filter: first-order IIR ───────────────────────────────────────────
//
// Applied to α_raw after the finite difference.
// Provides an additional, independently-tunable stage of smoothing.
//
// Coefficient recomputed each step so dt variations are handled correctly:
//   k = Δt / (Δt + RC),   RC = 1 / (2π·fc)
//   y[k] = k·x[k] + (1−k)·y[k−1]

/// First-order IIR low-pass filter (single axis, variable Δt).
///
/// The filter gain `k` is recomputed from `fc` and the actual `dt` on every call,
/// so this filter handles variable timesteps correctly.
#[derive(Debug, Clone)]
pub struct IirFilter {
    /// Cutoff frequency [Hz].
    pub fc: f32,
    state: f32,
}

impl IirFilter {
    pub fn new(fc: f32) -> Self { Self { fc, state: 0.0 } }

    /// Feed a new sample `x` at timestep `dt`; returns the filtered output.
    pub fn update(&mut self, x: f32, dt: f32) -> f32 {
        let rc = 1.0 / (2.0 * PI * self.fc);
        let k  = dt / (dt + rc);
        self.state = k * x + (1.0 - k) * self.state;
        self.state
    }

    pub fn reset(&mut self) { self.state = 0.0; }
    pub fn output(&self) -> f32 { self.state }
}

// ── INDI Controller ────────────────────────────────────────────────────────────

/// INDI attitude controller following Tal & Karaman (2020).
///
/// **Outer position loop** (position → thrust + desired rotation Rd) is identical
/// to [`GeometricController`] and is kept untouched.
///
/// **Inner INDI loop** (replaces the geometric −KR·eR − KW·eω torque law):
///
/// Three-stage angular acceleration measurement chain:
/// ```text
/// Stage 1  ω_raw     →  Butterworth2(fc_butterworth)  →  ω_filtered
/// Stage 2  ω_filtered →  (ω_f[k] − ω_f[k−1]) / Δt    →  α_raw
/// Stage 3  α_raw     →  IirFilter(fc_iir)             →  α_meas
/// ```
///
/// Control law:
/// ```text
/// α_ref = α_des − K_R·e_R − K_ω·e_ω          (K_R in 1/s², K_ω in 1/s)
/// δτ    = J · (α_ref − α_meas)
/// τ     = τ_prev + δτ                          (τ_prev_command in simulation)
/// ```
///
/// **Gains** `kr` / `kw` are in angular-acceleration units (1/s² and 1/s), not Nm.
/// Closed-loop attitude: ωₙ = √KR,  ζ = KW / (2ωₙ).
/// Equivalent geometric: KR_geo = J·KR_indi,  KW_geo = J·KW_indi.
///
/// **Simulation note**: τ_prev_command is used as the torque base because the RPM deck
/// is unavailable in simulation.  Firmware uses τ_current from RPM² instead.
#[derive(Debug, Clone)]
pub struct IndiController {
    // ── Outer loop gains (same as GeometricController) ────────────────────────
    pub kp:     Vec3,
    pub kv:     Vec3,
    pub ki_pos: Vec3,

    // ── INDI inner loop gains ──────────────────────────────────────────────────
    /// Attitude error gain K_R [1/s²].  ωₙ = √KR.
    pub kr: Vec3,
    /// Angular rate error gain K_ω [1/s].  ζ = KW/(2·√KR).  Must satisfy ζ > 0.
    pub kw: Vec3,

    // ── Filter parameters ──────────────────────────────────────────────────────
    /// Butterworth stage-1 cutoff [Hz].
    pub fc_butterworth: f32,
    /// IIR stage-3 cutoff [Hz].
    pub fc_iir: f32,

    /// Snap feedforward α_des [rad/s²].
    /// Caller sets this each step when the planner provides snap; otherwise leave
    /// at zero for pure-feedback INDI (still robust via α_meas cancellation).
    pub alpha_des: Vec3,

    // ── Private filter state ───────────────────────────────────────────────────
    gyro_bw_x: Butterworth2,
    gyro_bw_y: Butterworth2,
    gyro_bw_z: Butterworth2,
    alpha_iir_x: IirFilter,
    alpha_iir_y: IirFilter,
    alpha_iir_z: IirFilter,

    // ── Private INDI state ─────────────────────────────────────────────────────
    /// Previous Butterworth-filtered ω — used for the stage-2 finite difference.
    omega_filt_prev: Vec3,
    /// Previous torque command — the τ base for the increment in simulation.
    tau_prev: Vec3,
    i_error_pos: Vec3,
    initialized: bool,
    nominal_dt: f32,
}

impl IndiController {
    /// Create a new INDI controller.
    ///
    /// `nominal_dt` — expected control timestep [s]; used to compute the
    /// Butterworth coefficients.  Must match the `dt` passed to `compute_control`.
    /// Typical: `0.002` (500 Hz).
    pub fn new(
        kp: Vec3, kv: Vec3,
        kr: Vec3, kw: Vec3,
        fc_butterworth: f32, fc_iir: f32,
        nominal_dt: f32,
    ) -> Self {
        Self {
            kp, kv,
            ki_pos: Vec3::zero(),
            kr, kw,
            fc_butterworth, fc_iir,
            alpha_des: Vec3::zero(),
            gyro_bw_x: Butterworth2::new(fc_butterworth, nominal_dt),
            gyro_bw_y: Butterworth2::new(fc_butterworth, nominal_dt),
            gyro_bw_z: Butterworth2::new(fc_butterworth, nominal_dt),
            alpha_iir_x: IirFilter::new(fc_iir),
            alpha_iir_y: IirFilter::new(fc_iir),
            alpha_iir_z: IirFilter::new(fc_iir),
            omega_filt_prev: Vec3::zero(),
            tau_prev: Vec3::zero(),
            i_error_pos: Vec3::zero(),
            initialized: false,
            nominal_dt,
        }
    }

    /// Crazyflie 2.x defaults at 500 Hz.
    ///
    /// KR and KW derived from the tested geometric Block N gains
    /// (KR_geo = 0.010 Nm/rad, KW_geo = 0.00110 Nm/(rad/s)) via KR_indi = KR_geo/J:
    ///   J_xx = 16.57e-6 → KR_xy ≈ 603, KW_xy ≈ 66  (ωₙ = 24.6 rad/s, ζ = 1.34)
    ///   J_zz = 29.26e-6 → KR_z  ≈ 342, KW_z  ≈ 47
    pub fn default_crazyflie() -> Self {
        let dt = 0.002_f32;
        let mut ctrl = Self::new(
            Vec3::new(12.0, 12.0,  7.0),
            Vec3::new( 8.0,  8.0,  4.0),
            Vec3::new(603.0, 603.0, 342.0),
            Vec3::new( 66.0,  66.0,  47.0),
            60.0, 60.0,
            dt,
        );
        ctrl.ki_pos = Vec3::new(0.05, 0.05, 0.05);
        ctrl
    }

    /// Reset all filter state and INDI integrator.
    /// Call when landing or re-arming to avoid stale τ_prev and filter history.
    pub fn reset(&mut self) {
        let dt = self.nominal_dt;
        self.gyro_bw_x = Butterworth2::new(self.fc_butterworth, dt);
        self.gyro_bw_y = Butterworth2::new(self.fc_butterworth, dt);
        self.gyro_bw_z = Butterworth2::new(self.fc_butterworth, dt);
        self.alpha_iir_x.reset();
        self.alpha_iir_y.reset();
        self.alpha_iir_z.reset();
        self.omega_filt_prev = Vec3::zero();
        self.tau_prev        = Vec3::zero();
        self.i_error_pos     = Vec3::zero();
        self.initialized     = false;
    }

    // ── Read-only accessors for tests / logging ────────────────────────────────
    pub fn tau_prev(&self)   -> Vec3 { self.tau_prev }
    pub fn i_error_pos(&self) -> Vec3 { self.i_error_pos }

    /// Seed τ_prev from externally measured torque (e.g., from motor RPMs or simulator state).
    ///
    /// In hardware this is done automatically via `rpm_get_all()`.  In simulation,
    /// call this after each `sim.step()` with the torque computed from the actual
    /// (motor-lag-filtered) motor ω² to match the hardware INDI loop.
    pub fn seed_tau_prev(&mut self, tau: Vec3) { self.tau_prev = tau; }
    pub fn alpha_meas(&self) -> Vec3 {
        Vec3::new(
            self.alpha_iir_x.output(),
            self.alpha_iir_y.output(),
            self.alpha_iir_z.output(),
        )
    }
    pub fn omega_filt_prev(&self) -> Vec3 { self.omega_filt_prev }
}

impl Controller for IndiController {
    fn compute_control(
        &mut self,
        state:     &MultirotorState,
        reference: &TrajectoryReference,
        params:    &MultirotorParams,
        dt:        f32,
    ) -> ControlOutput {
        let omega = state.angular_velocity;

        // ── Stage 1: Butterworth pre-filter on raw gyro ────────────────────────
        let omega_filt = Vec3::new(
            self.gyro_bw_x.update(omega.x),
            self.gyro_bw_y.update(omega.y),
            self.gyro_bw_z.update(omega.z),
        );

        // ── Initialization on first call ───────────────────────────────────────
        // Seed omega_filt_prev = current filtered value so that stage-2 finite
        // difference returns zero on this first step (α_raw = α_meas = 0).
        // τ_prev starts at zero; the first torque will be J·α_ref (geometric-
        // equivalent), giving a smooth start without a step from zero.
        if !self.initialized {
            self.omega_filt_prev = omega_filt;
            self.initialized     = true;
        }

        // ── Stage 2: finite difference → raw angular acceleration ──────────────
        let inv_dt    = 1.0 / dt.max(1e-9);
        let alpha_raw = (omega_filt - self.omega_filt_prev) * inv_dt;

        // ── Stage 3: IIR post-filter on α_raw ─────────────────────────────────
        let alpha_meas = Vec3::new(
            self.alpha_iir_x.update(alpha_raw.x, dt),
            self.alpha_iir_y.update(alpha_raw.y, dt),
            self.alpha_iir_z.update(alpha_raw.z, dt),
        );

        // Update omega_filt_prev for the next step.
        self.omega_filt_prev = omega_filt;

        // ── Outer loop: position → force → thrust (identical to geometric) ──────
        let ep = reference.position - state.position;
        let ev = reference.velocity - state.velocity;

        self.i_error_pos = self.i_error_pos + ep * dt;
        let ki_lim = 0.5_f32;
        self.i_error_pos = Vec3::new(
            self.i_error_pos.x.clamp(-ki_lim, ki_lim),
            self.i_error_pos.y.clamp(-ki_lim, ki_lim),
            self.i_error_pos.z.clamp(-ki_lim, ki_lim),
        );

        let f_d = reference.acceleration
            + Vec3::new(self.kp.x * ep.x, self.kp.y * ep.y, self.kp.z * ep.z)
            + Vec3::new(self.kv.x * ev.x, self.kv.y * ev.y, self.kv.z * ev.z)
            + Vec3::new(
                self.ki_pos.x * self.i_error_pos.x,
                self.ki_pos.y * self.i_error_pos.y,
                self.ki_pos.z * self.i_error_pos.z,
            )
            + Vec3::new(0.0, 0.0, params.gravity as f32);

        let thrust_force = f_d * params.mass as f32;
        let r            = state.orientation.to_rotation_matrix();
        let body_z       = Vec3::new(r[0][2], r[1][2], r[2][2]);
        let thrust       = thrust_force.dot(&body_z);

        // Reset integrals and INDI state when on the ground (mirrors geometric).
        if thrust < 0.01 {
            self.i_error_pos     = Vec3::zero();
            self.tau_prev        = Vec3::zero();
            self.omega_filt_prev = omega_filt;
            self.alpha_iir_x.reset();
            self.alpha_iir_y.reset();
            self.alpha_iir_z.reset();
        }

        // ── Desired rotation Rd and ω_d feedforward from jerk ─────────────────
        let rd = desired_rotation(thrust_force, reference.yaw);

        let xdes = Vec3::new(rd[0][0], rd[1][0], rd[2][0]);
        let ydes = Vec3::new(rd[0][1], rd[1][1], rd[2][1]);
        let zdes = Vec3::new(rd[0][2], rd[1][2], rd[2][2]);

        // ω_d: desired body angular velocity from jerk feedforward.
        // Projects jerk onto the plane perpendicular to the thrust direction.
        let mut hw = Vec3::zero();
        if thrust.abs() > 1e-6 {
            let jerk      = reference.jerk;
            let jerk_perp = jerk - zdes * zdes.dot(&jerk);
            hw = jerk_perp * (params.mass as f32 / thrust);
        }
        let z_world  = Vec3::new(0.0, 0.0, 1.0);
        let omega_des = Vec3::new(
            -hw.dot(&ydes),
             hw.dot(&xdes),
            reference.yaw_rate * zdes.dot(&z_world),
        );
        // Transform desired ω from world frame to body frame: ω_r = R^T Rd ω_des
        let omega_r = matvec(&transpose(&r), matvec(&rd, omega_des));

        // ── Attitude errors ────────────────────────────────────────────────────
        // eR = ½ (Rd^T R − R^T Rd)^∨   (SO(3) rotation error)
        let er = rotation_error(&rd, &r);
        // e_ω uses the Butterworth-filtered body rate to match the filter chain
        // applied to α_meas: both use ω_filtered (not the raw gyro).
        let e_omega = omega_filt - omega_r;

        // ── Eq. 2: α_ref = α_des − K_R·e_R − K_ω·e_ω ────────────────────────
        let alpha_ref = Vec3::new(
            self.alpha_des.x - self.kr.x * er.x - self.kw.x * e_omega.x,
            self.alpha_des.y - self.kr.y * er.y - self.kw.y * e_omega.y,
            self.alpha_des.z - self.kr.z * er.z - self.kw.z * e_omega.z,
        );

        // ── Eq. 4: δτ = J·(α_ref − α_meas),   τ = τ_prev + δτ ───────────────
        let alpha_err = alpha_ref - alpha_meas;
        let delta_tau = Vec3::new(
            params.inertia[0][0] * alpha_err.x,
            params.inertia[1][1] * alpha_err.y,
            params.inertia[2][2] * alpha_err.z,
        );

        let torque    = self.tau_prev + delta_tau;
        self.tau_prev = torque;

        ControlOutput { thrust, torque }
    }
}

// ── Private geometry helpers ───────────────────────────────────────────────────

fn desired_rotation(thrust_force: Vec3, yaw: f32) -> [[f32; 3]; 3] {
    let mut ydes = Vec3::new(0.0, 1.0, 0.0);
    let mut zdes = Vec3::new(0.0, 0.0, 1.0);
    if thrust_force.norm() > 0.0 {
        zdes = thrust_force.normalize();
    }
    let xcdes  = Vec3::new(yaw.cos(), yaw.sin(), 0.0);
    let zcrossx = zdes.cross(&xcdes);
    if zcrossx.norm() > 0.0 {
        ydes = zcrossx.normalize();
    }
    let xdes = ydes.cross(&zdes);
    [
        [xdes.x, ydes.x, zdes.x],
        [xdes.y, ydes.y, zdes.y],
        [xdes.z, ydes.z, zdes.z],
    ]
}

fn rotation_error(rd: &[[f32; 3]; 3], r: &[[f32; 3]; 3]) -> Vec3 {
    let rd_t_r = mat_at_b(rd, r);
    let r_t_rd = mat_at_b(r,  rd);
    let diff   = matsub(&rd_t_r, &r_t_rd);
    Vec3::new(diff[2][1] * 0.5, diff[0][2] * 0.5, diff[1][0] * 0.5)
}

fn mat_at_b(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut o = [[0.0_f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 { o[i][j] += a[k][i] * b[k][j]; }
        }
    }
    o
}

fn matsub(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut o = [[0.0_f32; 3]; 3];
    for i in 0..3 { for j in 0..3 { o[i][j] = a[i][j] - b[i][j]; } }
    o
}

fn matvec(m: &[[f32; 3]; 3], v: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
        m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
        m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z,
    )
}

fn transpose(m: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    [[m[0][0],m[1][0],m[2][0]],
     [m[0][1],m[1][1],m[2][1]],
     [m[0][2],m[1][2],m[2][2]]]
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::{MultirotorParams, MultirotorState};

    fn hover_ref() -> TrajectoryReference {
        TrajectoryReference {
            position: Vec3::zero(), velocity: Vec3::zero(),
            acceleration: Vec3::zero(), jerk: Vec3::zero(),
            yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
        }
    }

    // ── Butterworth2 ──────────────────────────────────────────────────────────

    #[test]
    fn butterworth_dc_gain_is_one() {
        let mut f = Butterworth2::new(20.0, 0.002);
        for _ in 0..500 { f.update(1.0); }
        let out = f.update(1.0);
        assert!((out - 1.0).abs() < 1e-4,
            "Butterworth DC gain should be 1.0, got {out}");
    }

    #[test]
    fn butterworth_attenuates_above_cutoff() {
        // At 20× the cutoff frequency a 2nd-order filter gives at least 26 dB attenuation.
        let fc = 10.0_f32;
        let dt = 0.001_f32;
        let mut f = Butterworth2::new(fc, dt);
        let f_sig = 20.0 * fc;
        let mut peak = 0.0_f32;
        for k in 0..3000 {
            let x   = (2.0 * PI * f_sig * k as f32 * dt).sin();
            let out = f.update(x);
            if k > 500 { peak = peak.max(out.abs()); }
        }
        assert!(peak < 0.05,
            "Expected strong attenuation at 20× cutoff, peak amplitude = {peak}");
    }

    #[test]
    fn butterworth_seed_eliminates_transient() {
        let mut f = Butterworth2::new(20.0, 0.002);
        f.seed(3.0);
        let out = f.update(3.0);
        assert!((out - 3.0).abs() < 1e-5,
            "Seeded filter should output the seed value immediately: {out}");
    }

    // ── IirFilter ─────────────────────────────────────────────────────────────

    #[test]
    fn iir_zero_input_stays_zero() {
        let mut f = IirFilter::new(60.0);
        for _ in 0..100 {
            let out = f.update(0.0, 0.002);
            assert!(out.abs() < 1e-9);
        }
    }

    #[test]
    fn iir_step_converges_to_input() {
        let mut f = IirFilter::new(60.0);
        let mut out = 0.0_f32;
        for _ in 0..300 { out = f.update(1.0, 0.002); }
        assert!((out - 1.0).abs() < 0.01,
            "IIR should converge to step input: {out}");
    }

    // ── IndiController ────────────────────────────────────────────────────────

    #[test]
    fn hover_thrust_equals_weight() {
        let params = MultirotorParams::crazyflie();
        let mut ctrl  = IndiController::default_crazyflie();
        let state      = MultirotorState::new();
        let out        = ctrl.compute_control(&state, &hover_ref(), &params, 0.002);
        let weight     = params.mass * params.gravity;
        assert!((out.thrust - weight).abs() < 1e-5,
            "Hover thrust {:.6} ≠ weight {:.6}", out.thrust, weight);
    }

    #[test]
    fn hover_torque_converges_to_zero() {
        let params = MultirotorParams::crazyflie();
        let mut ctrl   = IndiController::default_crazyflie();
        let state       = MultirotorState::new();
        let reference   = hover_ref();
        // Run enough steps for the increment to settle: τ_prev accumulates until
        // α_ref ≈ α_meas, at which point δτ → 0 and τ_prev stabilises.
        let mut last_torque = Vec3::zero();
        for _ in 0..2000 {
            let out    = ctrl.compute_control(&state, &reference, &params, 0.002);
            last_torque = out.torque;
        }
        assert!(last_torque.norm() < 1e-4,
            "Hover torque should converge to zero, norm = {:.6}", last_torque.norm());
    }

    #[test]
    fn filter_chain_produces_zero_alpha_at_steady_hover() {
        // At constant hover (omega = 0), after settling the filter chain should
        // output alpha_meas ≈ 0.
        let params = MultirotorParams::crazyflie();
        let mut ctrl   = IndiController::default_crazyflie();
        let state       = MultirotorState::new();
        for _ in 0..500 {
            ctrl.compute_control(&state, &hover_ref(), &params, 0.002);
        }
        let alpha = ctrl.alpha_meas();
        assert!(alpha.norm() < 1e-6,
            "alpha_meas should be zero at steady hover: {:?}", alpha);
    }

    #[test]
    fn torque_increments_correct_direction_for_roll_error() {
        // A drone with positive roll error should accumulate negative τ_x
        // (correcting the roll back to zero).
        let params   = MultirotorParams::crazyflie();
        let mut ctrl = IndiController::default_crazyflie();

        // Tilt drone 10° around the x axis.
        let roll = 10.0_f32.to_radians();
        let state = MultirotorState::with_initial(
            Vec3::new(0.0, 0.0, 0.5),
            Vec3::zero(),
            crate::math::Quat::new(
                (roll / 2.0).cos(), (roll / 2.0).sin(), 0.0, 0.0
            ),
            Vec3::zero(),
        );
        let reference = TrajectoryReference {
            position:     Vec3::new(0.0, 0.0, 0.5),
            velocity:     Vec3::zero(),
            acceleration: Vec3::zero(),
            jerk:         Vec3::zero(),
            yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
        };

        // Run several steps so τ_prev accumulates the corrective torque.
        let mut out = ControlOutput { thrust: 0.0, torque: Vec3::zero() };
        for _ in 0..50 {
            out = ctrl.compute_control(&state, &reference, &params, 0.002);
        }
        assert!(out.torque.x < 0.0,
            "Positive roll error → negative τ_x correction, got τ_x = {}", out.torque.x);
    }

    #[test]
    fn alpha_des_feedforward_shifts_torque() {
        // Setting α_des to a positive x value should increase τ_x relative to
        // the zero-feedforward case, because δτ = J·(α_ref − α_meas) and
        // α_ref includes α_des.
        let params = MultirotorParams::crazyflie();

        let mut ctrl_no_ff = IndiController::default_crazyflie();
        let mut ctrl_ff    = IndiController::default_crazyflie();
        ctrl_ff.alpha_des  = Vec3::new(500.0, 0.0, 0.0);

        let state = MultirotorState::new();
        let reference = hover_ref();

        // Run enough steps for the increment to build.
        let mut out_no_ff = ControlOutput { thrust: 0.0, torque: Vec3::zero() };
        let mut out_ff    = ControlOutput { thrust: 0.0, torque: Vec3::zero() };
        for _ in 0..100 {
            out_no_ff = ctrl_no_ff.compute_control(&state, &reference, &params, 0.002);
            out_ff    = ctrl_ff   .compute_control(&state, &reference, &params, 0.002);
        }
        assert!(out_ff.torque.x > out_no_ff.torque.x,
            "Positive α_des.x should raise τ_x: no_ff={:.4e} ff={:.4e}",
            out_no_ff.torque.x, out_ff.torque.x);
    }

    #[test]
    fn reset_clears_all_state() {
        let params = MultirotorParams::crazyflie();
        let mut ctrl   = IndiController::default_crazyflie();
        let state       = MultirotorState::new();
        for _ in 0..50 {
            ctrl.compute_control(&state, &hover_ref(), &params, 0.002);
        }
        ctrl.reset();
        assert!(!ctrl.initialized);
        assert_eq!(ctrl.tau_prev(),    Vec3::zero());
        assert_eq!(ctrl.i_error_pos(), Vec3::zero());
        assert_eq!(ctrl.alpha_meas(),  Vec3::zero());
    }
}
