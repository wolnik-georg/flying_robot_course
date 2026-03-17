//! Multiplicative Extended Kalman Filter (MEKF) for attitude and position estimation.
//!
//! Direct Rust port of `mekf_offline.py`, updated to match the Crazyflie firmware
//! Kalman filter (`kalman_core.c`, Mueller et al. 2015).
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
//!
//! ## Differences from original MEKF (now matching firmware)
//!
//! 1. **Coriolis / transport-theorem correction**: body-frame velocity update now includes
//!    `−ω×v` (as in `kalman_core.c` lines 480–482):
//!    ```text
//!    dv_body/dt = acc + R^T·[0,0,−g] − ω×v
//!    ```
//!    Previously only `acc + R^T·[0,0,−g]` was used, missing the gyroscopic coupling.
//!
//! 2. **Analytical Jacobian**: replaces the finite-difference numerical Jacobian
//!    with the closed-form expression derived from the linearised process model,
//!    matching the explicit `A`-matrix construction in `kalman_core.c`.

use crate::math::Mat9;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------
const G_MS2: f32 = 9.81;
// Flow sensor scale: predicted_flow_pixels = dt * NP / (pz * THETA_P) * body_vel
//
// Calibrated empirically from March 15 flights (circle + figure-8):
//   MEKF XY span was 4.9× too small with the old constants (NP=350, THETA_P=0.717 rad).
//   Amplitude ratio pos_ekf / mekf_x across all dynamic logs → mean 4.88.
//   Correct NP/THETA_P = 488 / 4.88 ≈ 100.
//
// Keeping NP=350 to preserve the formula structure; THETA_P absorbs the correction:
//   THETA_P = 350 / 100 = 3.50 rad  — this is NOT a physical per-pixel angle but
//   an effective sensor calibration constant for the PMW3901 as read by the CF firmware.
//   (The old 0.717 rad came from the Python mekf_offline prototype and was never validated
//    against real flight data.)
const NP: f32 = 350.0;       // flow sensor: nominal pixel count (CF convention)
                              // theta_p is now in MekfParams (default 3.50, empirical March 2026)
                              // NP/theta_p ≈ 100  (old value 488 was 4.9× too large)
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

/// Convert unit quaternion [w,x,y,z] to a 3×3 rotation matrix R (body→world).
///
/// Convention: `v_world = R * v_body`.  Equivalently `v_body = R^T * v_world`.
///
/// Row-major storage: `R[i][j]` is row `i`, column `j`.
pub fn quat_to_rot(q: [f32; 4]) -> [[f32; 3]; 3] {
    let [w, x, y, z] = q;
    [
        [1.0 - 2.0*(y*y + z*z),  2.0*(x*y - w*z),         2.0*(x*z + w*y)       ],
        [2.0*(x*y + w*z),         1.0 - 2.0*(x*x + z*z),  2.0*(y*z - w*x)       ],
        [2.0*(x*z - w*y),         2.0*(y*z + w*x),          1.0 - 2.0*(x*x + y*y)],
    ]
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
    /// Zero-motion gate: skip flow update when |dnx| < threshold AND |dny| < threshold.
    /// Filters PMW3901 zero-padding artefacts (sensor fires at <20 Hz; missing samples
    /// are logged as 0.0, which appear as 46% of airborne readings on March 15 flights).
    /// Default 0.3 px — slightly above sensor quantisation noise (~0.1 px) but well
    /// below any real motion signal at hover (typical |flow| ≈ 1–5 px at 0.3 m height).
    pub zero_flow_threshold: f32,
    /// Flow calibration constant [rad].  NP/theta_p is the effective pixels-per-rad scale.
    /// Default 3.50 — calibrated against March 2026 flights with CF 2.1 + Flow Deck v2.
    /// Use 0.717 for the original course dataset (fr00.csv / State Estimation lab).
    pub theta_p: f32,
}

impl Default for MekfParams {
    fn default() -> Self {
        Self {
            q_pos:    1e-7,
            q_vel:    1e-3,
            q_att:    1e-6,
            r_height: 1e-3,
            r_flow:   8.0,
            zero_flow_threshold: 0.3,
            theta_p:  3.50,
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
    // Analytical Jacobian (δ≈0 after reset — see compute_jacobian docs)
    let g = compute_jacobian(&state.x, state.q_ref, gyro_rads, dt);

    // Propagate mean
    state.x = process_model(&state.x, state.q_ref, gyro_rads, accel_ms2, dt);

    // Propagate covariance: Σ = G Σ Gᵀ + Q
    let gsg = g.mat_mul(&state.sigma).mat_mul(&g.transpose());
    state.sigma = gsg.add(r_proc);
    // Force symmetry to prevent f32 rounding error accumulating over thousands
    // of steps — identical to what embedded EKF implementations do on hardware.
    state.sigma.symmetrise();
    // Cap diagonal variance to prevent unbounded growth in f32.
    state.sigma.clamp_diagonal(MAX_COVARIANCE);
}

// ---------------------------------------------------------------------------
// Process model
// ---------------------------------------------------------------------------

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

    // World-frame velocity: v_world = R * b
    let v_world = quat_rotate(q_full, b);

    // Gravity in body frame: g_body = R^T * [0, 0, −g]
    let g_body = quat_rotate(quat_conj(q_full), [0.0, 0.0, -G_MS2]);

    // Coriolis / transport-theorem term: ω × b
    // Matches firmware kalman_core.c predictDt() lines 480–482:
    //   dPX += dt*(acc_x + gyro_z*PY − gyro_y*PZ − g*R[2][0])
    // i.e. dv = acc + g_body − ω×b  (the minus sign is from dv/dt in rotating frame)
    let omega_cross_b = [
        gyro[1] * b[2] - gyro[2] * b[1],
        gyro[2] * b[0] - gyro[0] * b[2],
        gyro[0] * b[1] - gyro[1] * b[0],
    ];

    let p_new = [
        p[0] + v_world[0] * dt,
        p[1] + v_world[1] * dt,
        p[2] + v_world[2] * dt,
    ];
    let b_new = [
        b[0] + (accel[0] + g_body[0] - omega_cross_b[0]) * dt,
        b[1] + (accel[1] + g_body[1] - omega_cross_b[1]) * dt,
        b[2] + (accel[2] + g_body[2] - omega_cross_b[2]) * dt,
    ];
    let d_new = [
        delta[0] + gyro[0] * dt,
        delta[1] + gyro[1] * dt,
        delta[2] + gyro[2] * dt,
    ];

    [
        p_new[0], p_new[1], p_new[2],
        b_new[0], b_new[1], b_new[2],
        d_new[0], d_new[1], d_new[2],
    ]
}

// ---------------------------------------------------------------------------
// Analytical Jacobian  G = ∂g/∂x
// ---------------------------------------------------------------------------

/// Analytical Jacobian of the process model.
///
/// Assumes `mekf_reset` has been called before this predict step, so δ ≈ 0
/// and `q_full ≈ q_ref`.  This is the same assumption made in the Crazyflie
/// firmware (`kalman_core.c`: attitude error `D0/D1/D2` are zeroed in
/// `kalmanCoreFinalize` before each `kalmanCorePredict` call).
///
/// Block structure (state order: [p, b, δ]):
/// ```text
///   G[p, p] = I                  (position depends linearly on itself)
///   G[p, b] = R · dt             (position advances via R·v_body)
///   G[p, δ] = −R·[b]₍ₓ₎ · dt   (tilt changes which way body velocity maps to world)
///   G[b, b] = I − [ω]₍ₓ₎ · dt  (Coriolis coupling between body-velocity components)
///   G[b, δ] = [g_body]₍ₓ₎ · dt (tilt changes gravity projection in body frame)
///   G[δ, δ] = I                  (attitude error advances via gyro integration)
///   all other blocks = 0
/// ```
/// where `[v]₍ₓ₎` denotes the 3×3 skew-symmetric matrix of vector `v`.
fn compute_jacobian(
    x: &[f32; 9],
    q_ref: [f32; 4],
    gyro: [f32; 3],
    dt: f32,
) -> Mat9 {
    let b  = [x[3], x[4], x[5]];
    let r  = quat_to_rot(q_ref); // δ≈0 after reset → q_full ≈ q_ref

    // g_body = R^T · [0, 0, −g]  =  −g · third_row_of_R
    let gb = [-G_MS2 * r[2][0], -G_MS2 * r[2][1], -G_MS2 * r[2][2]];

    let mut g = Mat9::zeros();

    // ── G[p, p] = I ─────────────────────────────────────────────────────────
    g.data[0][0] = 1.0;
    g.data[1][1] = 1.0;
    g.data[2][2] = 1.0;

    // ── G[p, b] = R · dt ────────────────────────────────────────────────────
    for row in 0..3 {
        for col in 0..3 {
            g.data[row][3 + col] = r[row][col] * dt;
        }
    }

    // ── G[p, δ] = −R · [b]₍ₓ₎ · dt ─────────────────────────────────────────
    // [b]₍ₓ₎ columns:  col0 = [0, b2, −b1]ᵀ,  col1 = [−b2, 0, b0]ᵀ,  col2 = [b1, −b0, 0]ᵀ
    // −R·[b]₍ₓ₎ col k  =  −R · [b]₍ₓ₎[:,k]
    //   col0: −R·[0, b2, −b1]ᵀ = b[1]·R[:,2] − b[2]·R[:,1]
    //   col1: −R·[−b2, 0, b0]ᵀ = b[2]·R[:,0] − b[0]·R[:,2]
    //   col2: −R·[b1, −b0, 0]ᵀ = b[0]·R[:,1] − b[1]·R[:,0]
    for row in 0..3 {
        g.data[row][6] = (b[1] * r[row][2] - b[2] * r[row][1]) * dt;
        g.data[row][7] = (b[2] * r[row][0] - b[0] * r[row][2]) * dt;
        g.data[row][8] = (b[0] * r[row][1] - b[1] * r[row][0]) * dt;
    }

    // ── G[b, b] = I − [ω]₍ₓ₎ · dt ──────────────────────────────────────────
    // [ω]₍ₓ₎ = [[0, −wz, wy], [wz, 0, −wx], [−wy, wx, 0]]
    g.data[3][3] = 1.0;              g.data[3][4] =  gyro[2] * dt;  g.data[3][5] = -gyro[1] * dt;
    g.data[4][3] = -gyro[2] * dt;   g.data[4][4] = 1.0;             g.data[4][5] =  gyro[0] * dt;
    g.data[5][3] =  gyro[1] * dt;   g.data[5][4] = -gyro[0] * dt;  g.data[5][5] = 1.0;

    // ── G[b, δ] = [g_body]₍ₓ₎ · dt ─────────────────────────────────────────
    // [gb]₍ₓ₎ = [[0, −gbz, gby], [gbz, 0, −gbx], [−gby, gbx, 0]]
    let (gbx, gby, gbz) = (gb[0], gb[1], gb[2]);
    g.data[3][6] = 0.0;         g.data[3][7] = -gbz * dt;  g.data[3][8] =  gby * dt;
    g.data[4][6] =  gbz * dt;   g.data[4][7] = 0.0;         g.data[4][8] = -gbx * dt;
    g.data[5][6] = -gby * dt;   g.data[5][7] =  gbx * dt;  g.data[5][8] = 0.0;

    // ── G[δ, δ] = I ─────────────────────────────────────────────────────────
    g.data[6][6] = 1.0;
    g.data[7][7] = 1.0;
    g.data[8][8] = 1.0;

    g
}

// ---------------------------------------------------------------------------
// Numerical Jacobian (kept for cross-checking in tests)
// ---------------------------------------------------------------------------

/// Finite-difference numerical Jacobian.  Kept to validate the analytical version.
///
/// EPS = 1e-4 balances f32 cancellation error (≈ ε_machine / EPS ≈ 1.2e-3)
/// against truncation error (O(EPS) for mostly-linear dynamics).
/// The resulting per-entry accuracy is ~1e-3, sufficient to detect structural
/// errors (wrong signs, missing blocks) while passing f32 precision tests.
#[cfg(test)]
pub fn compute_jacobian_numerical(
    x: &[f32; 9],
    q_ref: [f32; 4],
    gyro: [f32; 3],
    accel: [f32; 3],
    dt: f32,
) -> Mat9 {
    const EPS: f32 = 1e-4;
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

// ---------------------------------------------------------------------------
// Measurement updates
// ---------------------------------------------------------------------------

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
///
/// `pz_meas`: actual ToF height [m] if available.  When provided it is used
/// for the scale factor instead of the estimated state height (state.x[2]).
/// This avoids the feedback loop where a slightly wrong height estimate causes
/// a slightly wrong velocity update, which then corrupts height further.
pub fn mekf_update_flow(
    state: &mut MekfState,
    dnx: f32,
    dny: f32,
    dt_flow: f32,
    r_flow: f32,
    pz_meas: Option<f32>,
    theta_p: f32,
) {
    // Prefer the measured ToF height; fall back to the state estimate.
    let pz = pz_meas.unwrap_or(state.x[2]);
    if pz.abs() < 0.05 {
        return;
    }
    let scale = dt_flow * NP / (pz * theta_p);

    // x-component
    let bx = state.x[3];
    let h_pred_x = scale * bx;
    let mut h_x = [0.0f32; 9];
    h_x[2] = -scale / pz * bx; // ∂/∂pz
    h_x[3] = scale;             // ∂/∂bx
    scalar_update(&mut state.x, &mut state.sigma, h_pred_x, dnx, &h_x, r_flow);

    // y-component — reuse same pz (ToF reading doesn't change between x and y)
    let pz2 = pz_meas.unwrap_or(state.x[2]);
    if pz2.abs() < 0.05 {
        return;
    }
    let scale2 = dt_flow * NP / (pz2 * theta_p);
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
        // Pass the ToF height so the scale factor uses the actual measured height
        // rather than the estimated state height.  This removes the main source of
        // XY drift on dynamic manoeuvres (height wobble → wrong velocity scale).
        let pz_meas = range_mm.map(|mm| mm / 1000.0);
        if let (Some(dnx), Some(dny)) = (flow_dnx, flow_dny) {
            // Zero-motion gate: skip update when both components are below threshold.
            // The PMW3901 fires at <20 Hz and the firmware pads missing samples with 0.0,
            // producing ~46% spurious zero readings in the 100 Hz log stream.
            // Applying these zeros as measurements drives the velocity estimate toward zero
            // even during real motion (underestimation bias).
            let thr = self.params.zero_flow_threshold;
            let is_zero_padded = dnx.abs() < thr && dny.abs() < thr;
            if !is_zero_padded {
                if let Some(ft) = self.last_flow_t {
                    if t > ft {
                        let dt_flow = t - ft;
                        mekf_update_flow(&mut self.state, dnx, dny, dt_flow, self.params.r_flow, pz_meas, self.params.theta_p);
                    }
                }
            }
            self.last_flow_t = Some(t);
        }

        new_estimate
    }
}

// ---------------------------------------------------------------------------
// Internal unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ── quat_to_rot ──────────────────────────────────────────────────────────

    #[test]
    fn test_quat_to_rot_identity() {
        let r = quat_to_rot([1.0, 0.0, 0.0, 0.0]);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((r[i][j] - expected).abs() < 1e-6,
                    "R[{i}][{j}] = {} expected {expected}", r[i][j]);
            }
        }
    }

    #[test]
    fn test_quat_to_rot_90deg_yaw() {
        // 90° CCW yaw around world Z: q = [cos45°, 0, 0, sin45°]
        let s = (std::f32::consts::FRAC_PI_4).sin();
        let c = (std::f32::consts::FRAC_PI_4).cos();
        let r = quat_to_rot([c, 0.0, 0.0, s]);
        // Body X should map to world Y: R * [1,0,0] = [0,1,0]
        assert!((r[0][0]).abs() < 1e-6, "R[0][0]={}", r[0][0]);
        assert!((r[1][0] - 1.0).abs() < 1e-6, "R[1][0]={}", r[1][0]);
        assert!((r[2][0]).abs() < 1e-6, "R[2][0]={}", r[2][0]);
    }

    #[test]
    fn test_quat_to_rot_orthogonal() {
        // Any quaternion → R must be orthogonal: R^T R = I
        let q = quat_norm([0.6, 0.2, -0.3, 0.7]);
        let r = quat_to_rot(q);
        for i in 0..3 {
            for j in 0..3 {
                let dot: f32 = (0..3).map(|k| r[k][i] * r[k][j]).sum();
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((dot - expected).abs() < 1e-5,
                    "RtR[{i}][{j}] = {dot}, expected {expected}");
            }
        }
    }

    // ── Coriolis term in process model ───────────────────────────────────────

    #[test]
    fn test_coriolis_rotates_velocity_correctly() {
        // Drone spinning at ωz = 1 rad/s with body velocity bx = 1 m/s.
        // Transport theorem: dv_body/dt gets −ω×v contribution.
        // ω×v = [0,0,1]×[1,0,0] = [0·0−1·0, 1·1−0·0, 0·0−0·1] = [0, 1, 0]
        // So dv.y should be −1 m/s² from Coriolis alone (minus gravity/accel terms).
        let mut x = [0.0f32; 9];
        x[3] = 1.0; // bx = 1 m/s
        x[2] = 0.0; // p_z (height) — irrelevant here

        let q_ref = [1.0f32, 0.0, 0.0, 0.0]; // level orientation
        let gyro  = [0.0f32, 0.0, 1.0];       // ωz = 1 rad/s
        let accel = [0.0f32, 0.0, 9.81];       // exactly cancels gravity at level

        let dt = 0.01;
        let x_new = process_model(&x, q_ref, gyro, accel, dt);

        // bx should decrease (−ω×v contribution in x is +gyro_z*by−gyro_y*bz = 0, no change x)
        // by should decrease: −(ω×v).y = −(gyro_z*bx − gyro_x*bz) = −(1·1 − 0) = −1 m/s²
        let d_by = x_new[4] - x[4];
        assert!(d_by < -0.005, "Coriolis should pull by negative; got d_by={d_by:.6}");
    }

    #[test]
    fn test_hover_velocity_stable() {
        // Level hover: acc = [0,0,g], gyro = 0, vel = 0 → velocity should not drift.
        let x = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0_f32];
        let q_ref = [1.0f32, 0.0, 0.0, 0.0];
        let gyro  = [0.0f32; 3];
        let accel = [0.0f32, 0.0, 9.81]; // accelerometer reads +g during hover
        let x_new = process_model(&x, q_ref, gyro, accel, 0.01);
        // Body velocity should remain near zero
        assert!(x_new[3].abs() < 1e-6, "bx drifted: {}", x_new[3]);
        assert!(x_new[4].abs() < 1e-6, "by drifted: {}", x_new[4]);
        assert!(x_new[5].abs() < 1e-6, "bz drifted: {}", x_new[5]);
    }

    // ── Analytical Jacobian matches numerical ─────────────────────────────────

    #[test]
    fn test_analytical_jacobian_matches_numerical_identity() {
        // Level orientation, strong gyro input so Coriolis terms (~gyro*dt) are
        // large enough to catch sign errors despite the 5e-3 tolerance.
        // Smallest Coriolis term: G[5][3] = gyro[1]*dt = 0.5*0.01 = 0.005 → wrong sign gives 0.01 >> 5e-3.
        let mut x = [0.0f32; 9];
        x[3] = 0.5; x[4] = -0.3; x[5] = 0.2;
        let q_ref = [1.0f32, 0.0, 0.0, 0.0];
        let gyro  = [0.5f32, 0.5, 1.0];  // strong, so gyro*dt terms are ≥ 0.005
        let accel = [0.1f32, -0.1, 9.81];
        let dt = 0.01;

        let analytical = compute_jacobian(&x, q_ref, gyro, dt);
        let numerical  = compute_jacobian_numerical(&x, q_ref, gyro, accel, dt);

        for i in 0..9 {
            for j in 0..9 {
                let a = analytical.data[i][j];
                let n = numerical.data[i][j];
                // Tolerance 5e-3: EPS=1e-4 gives f32 numerical error ~1e-3; structural
                // errors (wrong sign) produce diffs ≥ 2×|term| ≥ 0.01 >> 5e-3.
                assert!((a - n).abs() < 5e-3,
                    "G[{i}][{j}]: analytical={a:.6}, numerical={n:.6}, diff={:.6}", (a-n).abs());
            }
        }
    }

    #[test]
    fn test_analytical_jacobian_matches_numerical_tilted() {
        // 30° roll — exercises position-attitude cross-terms G[p,δ] = −R·[b]₍ₓ₎·dt
        // and gravity-attitude coupling G[b,δ] = [g_body]₍ₓ₎·dt.
        let angle = std::f32::consts::PI / 6.0;
        let q_ref = quat_norm([(angle/2.0).cos(), (angle/2.0).sin(), 0.0, 0.0]);
        let mut x = [0.0f32; 9];
        x[3] = 0.5; x[4] = -0.3; x[5] = 0.2;
        let gyro  = [0.5f32, 0.5, 1.0];
        let accel = [0.0f32, -1.0, 8.5];
        let dt = 0.01;

        let analytical = compute_jacobian(&x, q_ref, gyro, dt);
        let numerical  = compute_jacobian_numerical(&x, q_ref, gyro, accel, dt);

        for i in 0..9 {
            for j in 0..9 {
                let a = analytical.data[i][j];
                let n = numerical.data[i][j];
                assert!((a - n).abs() < 5e-3,
                    "G[{i}][{j}]: analytical={a:.6}, numerical={n:.6}, diff={:.6}", (a-n).abs());
            }
        }
    }
}
