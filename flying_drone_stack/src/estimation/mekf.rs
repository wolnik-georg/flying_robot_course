//! Multiplicative Extended Kalman Filter (MEKF) for attitude and position estimation.
//!
//! Direct Rust port of `mekf_offline.py`.
//!
//! State:  x = [p(3)  b(3)  δ(3)]  (9-dimensional, f32)
//!   p     : position in world frame [m]
//!   b     : velocity in body frame  [m/s]
//!   δ     : attitude error angles   [rad]
//!   q_ref : reference quaternion maintained *outside* the 9-state (f32 [w,x,y,z])
//!
//! IMU is treated as the control input (not a measurement).
//! Measurements: height (range sensor, mm→m) and optical flow (pixels).
//!
//! Reference: Markley (2003), Mueller et al. (2015), course slides.

use crate::math::Mat9;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------
const G_MS2: f32 = 9.81;
const NP: f32 = 350.0;       // flow sensor: nominal pixel count
const THETA_P: f32 = 0.71674; // flow sensor: angle per pixel [rad]
const DEG2RAD: f32 = std::f32::consts::PI / 180.0;
const G_TO_MS2: f32 = G_MS2;

/// Maximum allowed diagonal variance in Σ.
/// Mirrors Crazyflie firmware kalman_core.c MAX_COVARIANCE (100).
/// Prevents the position–velocity cross-term from growing unboundedly in f32,
/// which would drive K[position] → 1 and corrupt x/y estimates.
const MAX_COVARIANCE: f32 = 100.0;

// ---------------------------------------------------------------------------
// Quaternion helpers (f32, [w, x, y, z] convention)
// ---------------------------------------------------------------------------

/// Hamilton product of two quaternions [w,x,y,z].
fn quat_mult(q: [f32; 4], p: [f32; 4]) -> [f32; 4] {
    let [qw, qx, qy, qz] = q;
    let [pw, px, py, pz] = p;
    [
        qw * pw - qx * px - qy * py - qz * pz,
        qx * pw + qw * px - qz * py + qy * pz,
        qy * pw + qz * px + qw * py - qx * pz,
        qz * pw - qy * px + qx * py + qw * pz,
    ]
}

/// Conjugate (= inverse for unit quaternion).
fn quat_conj(q: [f32; 4]) -> [f32; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Normalise.
fn quat_norm(q: [f32; 4]) -> [f32; 4] {
    let n = (q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]).sqrt();
    if n > 1e-7 { [q[0]/n, q[1]/n, q[2]/n, q[3]/n] } else { [1.0, 0.0, 0.0, 0.0] }
}

/// Rotate vector v by quaternion q:  q * [0,v] * q^{-1}.
fn quat_rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let qv = [0.0, v[0], v[1], v[2]];
    let r = quat_mult(quat_mult(q, qv), quat_conj(q));
    [r[1], r[2], r[3]]
}

/// Small-angle error quaternion from δ (Markley eq 19).
fn q_from_delta(d: [f32; 3]) -> [f32; 4] {
    let d2 = d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
    quat_norm([1.0 - d2/8.0, d[0]/2.0, d[1]/2.0, d[2]/2.0])
}

/// Convert unit quaternion [w,x,y,z] to ZYX Euler angles [roll, pitch, yaw] in radians.
pub fn quat_to_euler(q: [f32; 4]) -> [f32; 3] {
    let [w, x, y, z] = q;
    let sinr_cosp = 2.0 * (w*x + y*z);
    let cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    let sinp = 2.0 * (w*y - z*x);
    let sinp = sinp.clamp(-1.0, 1.0);
    let pitch = sinp.asin();

    let siny_cosp = 2.0 * (w*z + x*y);
    let cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    [roll, pitch, yaw]
}

// ---------------------------------------------------------------------------
// Noise parameters
// ---------------------------------------------------------------------------

/// Tunable noise parameters.  Defaults are from the Python grid-search best.
#[derive(Debug, Clone, Copy)]
pub struct MekfParams {
    /// Process noise: position [m²/step]
    pub q_pos: f32,
    /// Process noise: body velocity [m²/s² per step]
    pub q_vel: f32,
    /// Process noise: attitude error [rad²/step]
    pub q_att: f32,
    /// Measurement noise: height [m²]
    pub r_height: f32,
    /// Measurement noise: optical flow [pixels²]
    pub r_flow: f32,
}

impl Default for MekfParams {
    fn default() -> Self {
        Self {
            q_pos:    1e-7,
            q_vel:    1e-3,
            q_att:    1e-6,
            r_height: 1e-3,
            r_flow:   8.0,
        }
    }
}

impl MekfParams {
    fn make_process_noise(&self) -> Mat9 {
        Mat9::diag([
            self.q_pos, self.q_pos, self.q_pos,
            self.q_vel, self.q_vel, self.q_vel,
            self.q_att, self.q_att, self.q_att,
        ])
    }

    fn make_initial_covariance() -> Mat9 {
        Mat9::diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01])
    }
}

// ---------------------------------------------------------------------------
// Filter state
// ---------------------------------------------------------------------------

/// Complete MEKF state (carries q_ref outside the 9-vector).
#[derive(Debug, Clone)]
pub struct MekfState {
    /// Reference quaternion [w, x, y, z]
    pub q_ref: [f32; 4],
    /// 9-vector state: [p(3), b(3), δ(3)]
    pub x: [f32; 9],
    /// 9×9 covariance matrix
    pub sigma: Mat9,
}

impl MekfState {
    /// Initialise with a known starting quaternion (seed from first EKF sample).
    pub fn new(q_ref: [f32; 4], _params: &MekfParams) -> Self {
        let mut x = [0.0f32; 9];
        x[2] = 1.0; // initial height guess ~1 m
        Self {
            q_ref: quat_norm(q_ref),
            x,
            sigma: MekfParams::make_initial_covariance(),
        }
    }
}

// ---------------------------------------------------------------------------
// MEKF algorithm steps
// ---------------------------------------------------------------------------

/// Reset: fold δ back into q_ref, zero δ.
pub fn mekf_reset(state: &mut MekfState) {
    let delta = [state.x[6], state.x[7], state.x[8]];
    state.q_ref = quat_norm(quat_mult(state.q_ref, q_from_delta(delta)));
    state.x[6] = 0.0;
    state.x[7] = 0.0;
    state.x[8] = 0.0;
}

/// Predict: propagate mean and covariance using gyro + accel as inputs.
pub fn mekf_predict(
    state: &mut MekfState,
    gyro_rads: [f32; 3],
    accel_ms2: [f32; 3],
    dt: f32,
    r_proc: &Mat9,
) {
    // Numerical Jacobian G = ∂g/∂x
    let g = compute_jacobian(&state.x, state.q_ref, gyro_rads, accel_ms2, dt);

    // Propagate mean
    state.x = process_model(&state.x, state.q_ref, gyro_rads, accel_ms2, dt);

    // Propagate covariance: Σ = G Σ Gᵀ + Q
    let gsg = g.mat_mul(&state.sigma).mat_mul(&g.transpose());
    state.sigma = gsg.add(r_proc);
    // Force symmetry to prevent f32 rounding error accumulating over thousands
    // of steps — identical to what embedded EKF implementations do on hardware.
    state.sigma.symmetrise();
    // Cap diagonal variance to prevent unbounded growth in f32.
    // The position-velocity cross-covariance Σ[p,v] grows without bound when
    // velocity is poorly observed, driving K[position] → 1 and corrupting x/y.
    // The Crazyflie EKF uses the same approach (kalman_core.c: MAX_COVARIANCE).
    state.sigma.clamp_diagonal(MAX_COVARIANCE);
}

fn process_model(
    x: &[f32; 9],
    q_ref: [f32; 4],
    gyro: [f32; 3],
    accel: [f32; 3],
    dt: f32,
) -> [f32; 9] {
    let p = [x[0], x[1], x[2]];
    let b = [x[3], x[4], x[5]];
    let delta = [x[6], x[7], x[8]];

    // Full quaternion including error
    let q_full = quat_norm(quat_mult(q_ref, q_from_delta(delta)));

    let v_world = quat_rotate(q_full, b);
    let g_body  = quat_rotate(quat_conj(q_full), [0.0, 0.0, -G_MS2]);

    let p_new = [p[0] + v_world[0]*dt, p[1] + v_world[1]*dt, p[2] + v_world[2]*dt];
    let b_new = [
        b[0] + (g_body[0] + accel[0]) * dt,
        b[1] + (g_body[1] + accel[1]) * dt,
        b[2] + (g_body[2] + accel[2]) * dt,
    ];
    let d_new = [delta[0] + gyro[0]*dt, delta[1] + gyro[1]*dt, delta[2] + gyro[2]*dt];

    [
        p_new[0], p_new[1], p_new[2],
        b_new[0], b_new[1], b_new[2],
        d_new[0], d_new[1], d_new[2],
    ]
}

fn compute_jacobian(
    x: &[f32; 9],
    q_ref: [f32; 4],
    gyro: [f32; 3],
    accel: [f32; 3],
    dt: f32,
) -> Mat9 {
    const EPS: f32 = 1e-5;
    let f0 = process_model(x, q_ref, gyro, accel, dt);
    let mut g = Mat9::zeros();
    for i in 0..9 {
        let mut xp = *x;
        xp[i] += EPS;
        let fp = process_model(&xp, q_ref, gyro, accel, dt);
        for row in 0..9 {
            g.data[row][i] = (fp[row] - f0[row]) / EPS;
        }
    }
    g
}

/// Scalar EKF measurement update.
fn scalar_update(
    x: &mut [f32; 9],
    sigma: &mut Mat9,
    h_pred: f32,
    z_meas: f32,
    h_row: &[f32; 9],
    r_scalar: f32,
) {
    let s = Mat9::h_sigma_ht(h_row, sigma) + r_scalar;
    let sigma_ht = Mat9::sigma_ht(sigma, h_row);
    let k: [f32; 9] = std::array::from_fn(|i| sigma_ht[i] / s);
    let innov = z_meas - h_pred;
    for i in 0..9 {
        x[i] += k[i] * innov;
    }
    *sigma = Mat9::joseph_update(sigma, &k, h_row, r_scalar);
    sigma.symmetrise();
}

/// Height measurement update.  h(x) = p_z,  H = [0,0,1, 0,…,0].
pub fn mekf_update_height(state: &mut MekfState, z_m: f32, r_height: f32) {
    let h_row = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let h_pred = state.x[2];
    scalar_update(&mut state.x, &mut state.sigma, h_pred, z_m, &h_row, r_height);
}

/// Optical flow measurement update (two scalar updates).
/// Model: h = (dt * NP / (pz * θp)) * [bx, by]
pub fn mekf_update_flow(
    state: &mut MekfState,
    dnx: f32,
    dny: f32,
    dt_flow: f32,
    r_flow: f32,
) {
    let pz = state.x[2];
    if pz.abs() < 0.05 {
        return;
    }
    let scale = dt_flow * NP / (pz * THETA_P);

    // x-component
    let bx = state.x[3];
    let h_pred_x = scale * bx;
    let mut h_x = [0.0f32; 9];
    h_x[2] = -scale / pz * bx; // ∂/∂pz
    h_x[3] = scale;             // ∂/∂bx
    scalar_update(&mut state.x, &mut state.sigma, h_pred_x, dnx, &h_x, r_flow);

    // recompute scale after state update
    let pz2 = state.x[2];
    if pz2.abs() < 0.05 {
        return;
    }
    let scale2 = dt_flow * NP / (pz2 * THETA_P);
    let by = state.x[4];
    let h_pred_y = scale2 * by;
    let mut h_y = [0.0f32; 9];
    h_y[2] = -scale2 / pz2 * by; // ∂/∂pz
    h_y[4] = scale2;              // ∂/∂by
    scalar_update(&mut state.x, &mut state.sigma, h_pred_y, dny, &h_y, r_flow);
}

// ---------------------------------------------------------------------------
// High-level convenience wrapper
// ---------------------------------------------------------------------------

/// All-in-one MEKF that holds its own state.
pub struct Mekf {
    pub state: MekfState,
    pub params: MekfParams,
    r_proc: Mat9,
    prev_t: Option<f32>,
    last_gyro: Option<(f32, [f32; 3])>,
    last_accel: Option<(f32, [f32; 3])>,
    last_flow_t: Option<f32>,
}

impl Mekf {
    /// Create filter with tuned default params.  Call `seed_from_quat` before first `feed_row`.
    pub fn new(params: MekfParams) -> Self {
        let r_proc = params.make_process_noise();
        Self {
            state: MekfState::new([1.0, 0.0, 0.0, 0.0], &params),
            params,
            r_proc,
            prev_t: None,
            last_gyro: None,
            last_accel: None,
            last_flow_t: None,
        }
    }

    /// Overwrite the initial reference quaternion (call before feeding any data).
    pub fn seed_qref(&mut self, q: [f32; 4]) {
        self.state.q_ref = quat_norm(q);
    }

    /// Feed one CSV row (all fields optional / NaN when absent).
    ///
    /// Returns `Some([roll, pitch, yaw, px, py, pz])` every time an IMU predict
    /// step is completed, otherwise `None`.
    pub fn feed_row(
        &mut self,
        t: f32,
        gyro_xyz_degs: Option<[f32; 3]>,
        accel_xyz_g: Option<[f32; 3]>,
        range_mm: Option<f32>,
        flow_dnx: Option<f32>,
        flow_dny: Option<f32>,
    ) -> Option<[f32; 6]> {
        // --- accumulate gyro/accel (they arrive on separate rows) ---
        if let Some(g) = gyro_xyz_degs {
            self.last_gyro = Some((t, [g[0]*DEG2RAD, g[1]*DEG2RAD, g[2]*DEG2RAD]));
        }
        if let Some(a) = accel_xyz_g {
            self.last_accel = Some((t, [a[0]*G_TO_MS2, a[1]*G_TO_MS2, a[2]*G_TO_MS2]));
        }

        let mut new_estimate: Option<[f32; 6]> = None;

        // --- predict when we have a fresh gyro+accel pair ---
        if let (Some((tg, og)), Some((ta, oa))) = (self.last_gyro, self.last_accel) {
            let t_imu = tg.max(ta);
            if let Some(prev) = self.prev_t {
                if t_imu > prev {
                    let dt = t_imu - prev;
                    mekf_reset(&mut self.state);
                    mekf_predict(&mut self.state, og, oa, dt, &self.r_proc);

                    let euler = quat_to_euler(self.state.q_ref);
                    let pos = [self.state.x[0], self.state.x[1], self.state.x[2]];
                    new_estimate = Some([euler[0], euler[1], euler[2], pos[0], pos[1], pos[2]]);
                }
            }
            self.prev_t = Some(t_imu);
            self.last_gyro  = None;
            self.last_accel = None;
        }

        // --- height update ---
        if let Some(mm) = range_mm {
            mekf_update_height(&mut self.state, mm / 1000.0, self.params.r_height);
        }

        // --- flow update ---
        if let (Some(dnx), Some(dny)) = (flow_dnx, flow_dny) {
            if let Some(ft) = self.last_flow_t {
                if t > ft {
                    let dt_flow = t - ft;
                    mekf_update_flow(&mut self.state, dnx, dny, dt_flow, self.params.r_flow);
                }
            }
            self.last_flow_t = Some(t);
        }

        new_estimate
    }
}
