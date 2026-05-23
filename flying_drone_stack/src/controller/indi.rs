use crate::math::Vec3;
use crate::dynamics::{MultirotorState, MultirotorParams};
use super::{Controller, TrajectoryReference, ControlOutput};

/// First-order IIR low-pass filter applied per-axis to a Vec3.
///
/// Difference equation:  y[k] = k·x[k] + (1−k)·y[k−1]
/// Filter gain:          k = Δt / (Δt + RC),  RC = 1 / (2π·fc)
#[derive(Debug, Clone)]
pub struct IirFilter {
    pub fc: f32,
    state: Vec3,
}

impl IirFilter {
    pub fn new(fc: f32) -> Self {
        Self { fc, state: Vec3::zero() }
    }

    /// Update the filter with a new sample; returns the filtered output.
    pub fn update(&mut self, raw: Vec3, dt: f32) -> Vec3 {
        let rc = 1.0 / (2.0 * std::f32::consts::PI * self.fc);
        let k = dt / (dt + rc);
        self.state = self.state + (raw - self.state) * k;
        self.state
    }

    pub fn reset(&mut self) { self.state = Vec3::zero(); }

    pub fn state(&self) -> Vec3 { self.state }
}

/// INDI controller (Tal & Karaman 2020).
///
/// The four core equations for the attitude inner loop:
/// 1. α_des  ← snap feedforward via differential flatness (set via `self.alpha_des`)
/// 2. α_ref  = α_des − K_R · e_R − K_ω · e_ω
/// 3. α_meas = LPF( (ω[k] − ω[k−1]) / Δt )          ← first-order IIR
/// 4. δτ = J · (α_ref − α_meas),   τ = τ_prev + δτ
///
/// Gains `kr` / `kw` are in rad/s²/rad and rad/s²/(rad/s) respectively
/// (angular-acceleration units, NOT torque units). J converts them to Nm internally.
///
/// The outer position loop is identical to `GeometricController`.
#[derive(Debug, Clone)]
pub struct IndiController {
    // Outer loop gains (same units as GeometricController)
    pub kp: Vec3,
    pub kv: Vec3,
    pub ki_pos: Vec3,
    // INDI attitude gains [rad/s² per rad], [rad/s² per rad/s]
    pub kr: Vec3,
    pub kw: Vec3,
    /// Angular-acceleration LP filter cutoff [Hz].
    pub fc: f32,
    /// α_des feedforward (rad/s²) from trajectory snap. Caller sets this each step;
    /// leave at zero for pure-feedback INDI (still robust due to α_meas cancellation).
    pub alpha_des: Vec3,

    // Private INDI state
    alpha_filter: IirFilter,
    omega_prev: Vec3,
    tau_prev: Vec3,
    i_error_pos: Vec3,
    initialized: bool,
}

impl IndiController {
    pub fn new(kp: Vec3, kv: Vec3, kr: Vec3, kw: Vec3, fc: f32) -> Self {
        Self {
            kp,
            kv,
            ki_pos: Vec3::zero(),
            kr,
            kw,
            fc,
            alpha_des: Vec3::zero(),
            alpha_filter: IirFilter::new(fc),
            omega_prev: Vec3::zero(),
            tau_prev: Vec3::zero(),
            i_error_pos: Vec3::zero(),
            initialized: false,
        }
    }

    /// Crazyflie 2.x defaults — position loop from GeometricController, INDI attitude loop.
    ///
    /// Conservative first-flight gains:
    ///   KR = 200  →  ωₙ ≈ 14.1 rad/s   (att closed-loop bandwidth)
    ///   KW = 40   →  ζ  ≈ 1.42          (overdamped — safe start)
    ///   Equivalent torque: J·KR ≈ 0.0033 Nm/rad,  J·KW ≈ 0.00066 Nm/(rad/s)
    pub fn default() -> Self {
        let mut ctrl = Self::new(
            Vec3::new(12.0, 12.0, 7.0),
            Vec3::new(8.0,  8.0,  4.0),
            Vec3::new(200.0, 200.0, 150.0),
            Vec3::new(40.0,  40.0,  30.0),
            60.0,
        );
        ctrl.ki_pos = Vec3::new(0.05, 0.05, 0.05);
        ctrl
    }

    pub fn reset(&mut self) {
        self.i_error_pos = Vec3::zero();
        self.tau_prev    = Vec3::zero();
        self.omega_prev  = Vec3::zero();
        self.alpha_filter.reset();
        self.initialized = false;
    }

    pub fn i_error_pos(&self) -> Vec3 { self.i_error_pos }
    pub fn tau_prev(&self)    -> Vec3 { self.tau_prev }
    pub fn alpha_meas(&self)  -> Vec3 { self.alpha_filter.state() }
}

impl Controller for IndiController {
    fn compute_control(
        &mut self,
        state: &MultirotorState,
        reference: &TrajectoryReference,
        params: &MultirotorParams,
        dt: f32,
    ) -> ControlOutput {
        // ── Outer loop: position → desired force → thrust ──────────────────
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
        let r = state.orientation.to_rotation_matrix();
        let body_z = Vec3::new(r[0][2], r[1][2], r[2][2]);
        let thrust = thrust_force.dot(&body_z);

        if thrust < 0.01 {
            self.i_error_pos = Vec3::zero();
        }

        // ── Desired rotation + ω_d feedforward (jerk-based, same as geometric) ──
        let rd = desired_rotation(thrust_force, reference.yaw);
        let er = rotation_error(&rd, &r);

        let xdes = Vec3::new(rd[0][0], rd[1][0], rd[2][0]);
        let ydes = Vec3::new(rd[0][1], rd[1][1], rd[2][1]);
        let zdes = Vec3::new(rd[0][2], rd[1][2], rd[2][2]);

        let mut hw = Vec3::zero();
        if thrust != 0.0 {
            let jerk = reference.jerk;
            let jerk_perp = jerk - zdes * zdes.dot(&jerk);
            hw = jerk_perp * (params.mass as f32 / thrust);
        }

        let z_world = Vec3::new(0.0, 0.0, 1.0);
        let omega_des = Vec3::new(
            -hw.dot(&ydes),
            hw.dot(&xdes),
            reference.yaw_rate * zdes.dot(&z_world),
        );
        let omega_r = matvec(&transpose(&r), matvec(&rd, omega_des));

        let e_omega = state.angular_velocity - omega_r;

        // ── INDI inner loop ─────────────────────────────────────────────────
        let torque = if !self.initialized {
            // First call: seed tau_prev with approximate geometric torque so that
            // τ = τ_prev + δτ does not step from zero on the second call.
            // Gyroscopic term: ω × Jω
            let j_omega = Vec3::new(
                params.inertia[0][0] * state.angular_velocity.x,
                params.inertia[1][1] * state.angular_velocity.y,
                params.inertia[2][2] * state.angular_velocity.z,
            );
            let gyro_comp = state.angular_velocity.cross(&j_omega);
            let tau_seed = Vec3::new(
                -self.kr.x * params.inertia[0][0] * er.x
                    - self.kw.x * params.inertia[0][0] * e_omega.x
                    + gyro_comp.x,
                -self.kr.y * params.inertia[1][1] * er.y
                    - self.kw.y * params.inertia[1][1] * e_omega.y
                    + gyro_comp.y,
                -self.kr.z * params.inertia[2][2] * er.z
                    - self.kw.z * params.inertia[2][2] * e_omega.z
                    + gyro_comp.z,
            );
            self.tau_prev   = tau_seed;
            self.omega_prev = state.angular_velocity;
            self.initialized = true;
            tau_seed
        } else {
            // Eq. 3: α_meas = LPF( (ω[k] − ω[k−1]) / Δt )
            let alpha_raw = (state.angular_velocity - self.omega_prev) * (1.0 / dt);
            let alpha_meas = self.alpha_filter.update(alpha_raw, dt);

            // Eq. 2: α_ref = α_des − K_R·e_R − K_ω·e_ω
            let alpha_ref = Vec3::new(
                self.alpha_des.x - self.kr.x * er.x - self.kw.x * e_omega.x,
                self.alpha_des.y - self.kr.y * er.y - self.kw.y * e_omega.y,
                self.alpha_des.z - self.kr.z * er.z - self.kw.z * e_omega.z,
            );

            // Eq. 4: δτ = J · (α_ref − α_meas),   τ = τ_prev + δτ
            let alpha_err = alpha_ref - alpha_meas;
            let delta_tau = Vec3::new(
                params.inertia[0][0] * alpha_err.x,
                params.inertia[1][1] * alpha_err.y,
                params.inertia[2][2] * alpha_err.z,
            );

            let tau = self.tau_prev + delta_tau;
            self.tau_prev   = tau;
            self.omega_prev = state.angular_velocity;
            tau
        };

        ControlOutput { thrust, torque }
    }
}

// ── Private geometry helpers (same math as GeometricController, private scope) ────

fn desired_rotation(thrust_force: Vec3, yaw: f32) -> [[f32; 3]; 3] {
    let norm = thrust_force.norm();
    let mut ydes = Vec3::new(0.0, 1.0, 0.0);
    let mut zdes = Vec3::new(0.0, 0.0, 1.0);
    if norm > 0.0 {
        zdes = thrust_force.normalize();
    }
    let xcdes = Vec3::new(yaw.cos(), yaw.sin(), 0.0);
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
    let r_t_rd = mat_at_b(r, rd);
    let diff = matsub(&rd_t_r, &r_t_rd);
    Vec3::new(diff[2][1] * 0.5, diff[0][2] * 0.5, diff[1][0] * 0.5)
}

fn mat_at_b(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut o = [[0.0_f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                o[i][j] += a[k][i] * b[k][j];
            }
        }
    }
    o
}

fn matsub(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut o = [[0.0_f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            o[i][j] = a[i][j] - b[i][j];
        }
    }
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
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::{MultirotorParams, MultirotorState};

    fn hover_reference() -> TrajectoryReference {
        TrajectoryReference {
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            acceleration: Vec3::zero(),
            jerk: Vec3::zero(),
            yaw: 0.0,
            yaw_rate: 0.0,
            yaw_acceleration: 0.0,
        }
    }

    #[test]
    fn test_iir_filter_zero_input() {
        let mut f = IirFilter::new(60.0);
        let zero = Vec3::zero();
        for _ in 0..100 {
            let out = f.update(zero, 0.002);
            assert!(out.norm() < 1e-6);
        }
    }

    #[test]
    fn test_iir_filter_step_converges() {
        let mut f = IirFilter::new(60.0);
        let target = Vec3::new(100.0, 0.0, 0.0);
        let mut last = Vec3::zero();
        for _ in 0..200 {
            last = f.update(target, 0.002);
        }
        assert!((last.x - 100.0).abs() < 0.1, "filter should converge to input: {}", last.x);
    }

    #[test]
    fn test_hover_thrust() {
        let params = MultirotorParams::crazyflie();
        let mut ctrl = IndiController::default();
        let state = MultirotorState::new();
        let reference = hover_reference();
        let output = ctrl.compute_control(&state, &reference, &params, 0.002);
        let expected = params.mass * params.gravity;
        assert!(
            (output.thrust - expected).abs() < 1e-5,
            "expected hover thrust {expected:.4}, got {:.4}", output.thrust
        );
    }

    #[test]
    fn test_hover_torque_converges_to_zero() {
        let params = MultirotorParams::crazyflie();
        let mut ctrl = IndiController::default();
        let state = MultirotorState::new();
        let reference = hover_reference();
        // Run multiple steps at hover — torque should decay toward zero
        let mut last_torque = Vec3::zero();
        for _ in 0..500 {
            let out = ctrl.compute_control(&state, &reference, &params, 0.002);
            last_torque = out.torque;
        }
        assert!(
            last_torque.norm() < 1e-4,
            "torque should converge to zero at hover, got norm={:.6}", last_torque.norm()
        );
    }

    #[test]
    fn test_torque_increment_direction() {
        // When alpha_meas = 0 and alpha_ref > 0, delta_tau should be positive (J·alpha_ref)
        let params = MultirotorParams::crazyflie();
        let mut ctrl = IndiController::default();
        ctrl.alpha_des = Vec3::new(100.0, 0.0, 0.0); // positive x acceleration demand
        let state = MultirotorState::new();
        let reference = hover_reference();
        // First call initializes
        ctrl.compute_control(&state, &reference, &params, 0.002);
        // Second call: alpha_meas ≈ 0 (no actual motion), alpha_ref > 0 → tau_x should increase
        let tau_before = ctrl.tau_prev();
        ctrl.compute_control(&state, &reference, &params, 0.002);
        let tau_after = ctrl.tau_prev();
        assert!(
            tau_after.x > tau_before.x,
            "positive alpha_des.x should increase tau_x: before={}, after={}",
            tau_before.x, tau_after.x
        );
    }

    #[test]
    fn test_reset_clears_state() {
        let params = MultirotorParams::crazyflie();
        let mut ctrl = IndiController::default();
        let state = MultirotorState::new();
        let reference = hover_reference();
        for _ in 0..10 {
            ctrl.compute_control(&state, &reference, &params, 0.002);
        }
        ctrl.reset();
        assert!(!ctrl.initialized);
        assert_eq!(ctrl.tau_prev(), Vec3::zero());
        assert_eq!(ctrl.i_error_pos(), Vec3::zero());
    }
}
