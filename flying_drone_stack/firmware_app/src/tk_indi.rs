//! controller=11 (`ControllerTypeOot6`) -- the INDI controller of Tal & Karaman (2020),
//! "Accurate Tracking of Aggressive Quadrotor Trajectories using Incremental Nonlinear Dynamic
//! Inversion and Differential Flatness" (arXiv 1809.04048v2), items 1-9 of docs/74 §4d.
//!
//! The law is written in the paper's own frames (world NED, body FRD, xi = body->world, specific
//! thrust tau < 0). Only the inputs and the outputs are converted from / to the firmware frames
//! (world ENU, body FLU). Equation numbers in the comments are the paper's. Literal NumPy
//! reference: `experiments/analysis/tk_indi_reference.py` (checked against this file by
//! `host/test_tk_indi_rust_vs_ref.py`).
//!
//! NOT in the law: clamps, notch, velocity low-pass, integral action. Additions, all marked
//! GUARD: division-by-zero guards, and priming of the filters with the first sample.
//! Motor loop (32)-(36) of the paper (encoders) does not exist: thrust and torque go to the stock
//! mixer, motor speeds come from DShot telemetry (`rpm_get_all`).

use crate::bindings::{control_s, sensorData_s, setpoint_s, state_s};
use crate::quat_to_rot;

extern "C" {
    fn rpm_get_all(m1: *mut u16, m2: *mut u16, m3: *mut u16, m4: *mut u16);
    fn oot_rpm_logs_available() -> bool;
    fn indi_tau_write(tx: f32, ty: f32, tz: f32);
    fn indi_a_res_write(ax: f32, ay: f32, az: f32);
    fn indi_e_r_write(ex: f32, ey: f32, ez: f32, norm: f32);
    fn tk_diag_write(thrust: f32, rpm_ok: f32, ns: f32, tau_f: f32);
    // shared with lib.rs: indi_gains.mass / kt1..4 [N/RPM^2]
    static g_indi_mass: f32;
    static g_indi_kt1: f32;
    static g_indi_kt2: f32;
    static g_indi_kt3: f32;
    static g_indi_kt4: f32;
    // runtime params, PARAM_GROUP(ctrlTk) in traj_iface.c
    static g_tk_kx_xy: f32;
    static g_tk_kx_z: f32;
    static g_tk_kv_xy: f32;
    static g_tk_kv_z: f32;
    static g_tk_ka_xy: f32;
    static g_tk_ka_z: f32;
    static g_tk_kxi_xy: f32;
    static g_tk_kxi_z: f32;
    static g_tk_kom_xy: f32;
    static g_tk_kom_z: f32;
    static g_tk_fc: f32;
    static g_tk_jrz: f32;
    static g_tk_yaw_lit: u8;
}

type V = [f32; 3];
type M = [[f32; 3]; 3];

const GRAV: f32 = 9.81;
const DT: f32 = 1.0 / 1000.0; // the stock stabilizer calls the controller every 1 kHz tick
const PI: f32 = core::f32::consts::PI;
const DEG2RAD: f32 = PI / 180.0;
const ARM_M: f32 = 0.0353553;
const T2T: f32 = 0.0056928;
#[cfg(drone_bl)]
const J: V = [23.951e-6, 23.951e-6, 32.347e-6];
#[cfg(not(drone_bl))]
const J: V = [16.571710e-6, 16.655602e-6, 29.261652e-6];

// firmware mixer sign pattern (FLU)
const MIX_X: [f32; 4] = [-1.0, -1.0, 1.0, 1.0];
const MIX_Y: [f32; 4] = [-1.0, 1.0, 1.0, -1.0];
const MIX_Z: [f32; 4] = [-1.0, 1.0, -1.0, 1.0];
const CB: V = [1.0, -1.0, -1.0]; // FLU <-> FRD (diagonal)

#[inline] fn dot(a: V, b: V) -> f32 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
#[inline] fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline] fn add(a: V, b: V) -> V { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
#[inline] fn sub(a: V, b: V) -> V { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
#[inline] fn scl(a: V, k: f32) -> V { [a[0] * k, a[1] * k, a[2] * k] }
#[inline] fn mv(m: &M, v: V) -> V { [dot(m[0], v), dot(m[1], v), dot(m[2], v)] }
#[inline] fn mtv(m: &M, v: V) -> V {
    [
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    ]
}
#[inline] fn col(m: &M, c: usize) -> V { [m[0][c], m[1][c], m[2][c]] }
fn mm(a: &M, b: &M) -> M {
    let mut o = [[0.0f32; 3]; 3];
    for i in 0..3 { for j in 0..3 { for k in 0..3 { o[i][j] += a[i][k] * b[k][j]; } } }
    o
}
#[inline] fn norm(a: V) -> f32 { libm::sqrtf(dot(a, a)) }
fn skew(v: V) -> M { [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]] }
fn tr(m: &M) -> M {
    [[m[0][0], m[1][0], m[2][0]], [m[0][1], m[1][1], m[2][1]], [m[0][2], m[1][2], m[2][2]]]
}
// [iz]x^T
const SKIZ_T: M = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 0.0]];

fn qmul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

#[derive(Copy, Clone)]
struct Bq {
    a0: f32,
    a1: f32,
    b0: f32,
    b1: f32,
    i: [f32; 2],
    o: [f32; 2],
    primed: bool,
}
impl Bq {
    const fn zero() -> Self {
        Self { a0: 0.0, a1: 0.0, b0: 0.0, b1: 0.0, i: [0.0; 2], o: [0.0; 2], primed: false }
    }
    /// 2nd-order Butterworth, same design as the Bitcraze lpf2p / Omar's filter (q = 0.7071).
    fn design(&mut self, fc: f32, dt: f32) {
        let tau = 1.0 / (2.0 * PI * fc);
        let q = 0.7071_f32;
        let k = libm::tanf(dt / (2.0 * tau));
        let poly = k * k + k / q + 1.0;
        self.a0 = 2.0 * (k * k - 1.0) / poly;
        self.a1 = (k * k - k / q + 1.0) / poly;
        self.b0 = k * k / poly;
        self.b1 = 2.0 * self.b0;
    }
    fn update(&mut self, x: f32) -> f32 {
        if !self.primed {
            // GUARD (not in the paper): start the filter at the first sample, not at zero
            self.i = [x, x];
            self.o = [x, x];
            self.primed = true;
        }
        let y = self.b0 * x + self.b1 * self.i[0] + self.b0 * self.i[1]
            - self.a0 * self.o[0] - self.a1 * self.o[1];
        self.i[1] = self.i[0];
        self.i[0] = x;
        self.o[1] = self.o[0];
        self.o[0] = y;
        y
    }
}

struct State {
    f_a: [Bq; 3],
    f_s: [Bq; 3],
    f_w2: [Bq; 4],
    f_w: [Bq; 4],
    f_om: [Bq; 3],
    prev_om: V,
    prev_w: [f32; 4],
    prev_tau: f32,
    have_prev: bool,
    fc_applied: f32,
    initialized: bool,
}

static mut ST: State = State {
    f_a: [Bq::zero(); 3],
    f_s: [Bq::zero(); 3],
    f_w2: [Bq::zero(); 4],
    f_w: [Bq::zero(); 4],
    f_om: [Bq::zero(); 3],
    prev_om: [0.0; 3],
    prev_w: [0.0; 4],
    prev_tau: 0.0,
    have_prev: false,
    fc_applied: 0.0,
    initialized: false,
};

fn design_all(s: &mut State, fc: f32) {
    for f in s.f_a.iter_mut().chain(s.f_s.iter_mut()).chain(s.f_om.iter_mut()) { f.design(fc, DT); }
    for f in s.f_w2.iter_mut().chain(s.f_w.iter_mut()) { f.design(fc, DT); }
    s.fc_applied = fc;
}
fn reprime(s: &mut State) {
    for f in s.f_a.iter_mut().chain(s.f_s.iter_mut()).chain(s.f_om.iter_mut()) { f.primed = false; }
    for f in s.f_w2.iter_mut().chain(s.f_w.iter_mut()) { f.primed = false; }
    s.have_prev = false;
}
fn filt3(f: &mut [Bq; 3], v: V) -> V { [f[0].update(v[0]), f[1].update(v[1]), f[2].update(v[2])] }

/// Solve the 4x4 system A x = b (Gaussian elimination, partial pivoting). None if singular.
fn solve4(mut a: [[f32; 4]; 4], mut b: [f32; 4]) -> Option<[f32; 4]> {
    for c in 0..4 {
        let mut p = c;
        for r in (c + 1)..4 { if libm::fabsf(a[r][c]) > libm::fabsf(a[p][c]) { p = r; } }
        if libm::fabsf(a[p][c]) < 1e-12 { return None; }
        a.swap(c, p);
        b.swap(c, p);
        for r in (c + 1)..4 {
            let f = a[r][c] / a[c][c];
            for k in c..4 { a[r][k] -= f * a[c][k]; }
            b[r] -= f * b[c];
        }
    }
    let mut x = [0.0f32; 4];
    for r in (0..4).rev() {
        let mut acc = b[r];
        for k in (r + 1)..4 { acc -= a[r][k] * x[k]; }
        x[r] = acc / a[r][r];
    }
    Some(x)
}

/// Eqs. (14), (15): (Omega_ref, Omegadot_ref) in the body frame. GUARD: |tau| < 0.5 or singular -> 0.
fn feedforward(r: &M, tau: f32, taud: f32, om: V, j: V, sn: V, psid: f32, psidd: f32) -> (V, V) {
    let (bx, by, bz) = (col(r, 0), col(r, 1), col(r, 2));
    // (12): S = [-bx2 bx1]/(r^T r) * [[0,-bz1,by1],[0,-bz2,by2]]
    let d = bx[0] * bx[0] + bx[1] * bx[1];
    let n = [-bx[1], bx[0]];
    let p = [[0.0, -bz[0], by[0]], [0.0, -bz[1], by[1]]];
    let np = [
        n[0] * p[0][0] + n[1] * p[1][0],
        n[0] * p[0][1] + n[1] * p[1][1],
        n[0] * p[0][2] + n[1] * p[1][2],
    ];
    if libm::fabsf(tau) < 0.5 || d < 1e-9 {
        return ([0.0; 3], [0.0; 3]);
    }
    let sv = [np[0] / d, np[1] / d, np[2] / d];
    let rs = mm(r, &SKIZ_T); // R [iz]x^T
    let mut a = [[0.0f32; 4]; 4];
    for i in 0..3 {
        for k in 0..3 { a[i][k] = tau * rs[i][k]; }
        a[i][3] = bz[i];
    }
    a[3][0] = sv[0];
    a[3][1] = sv[1];
    a[3][2] = sv[2];
    let sol = match solve4(a, [j[0], j[1], j[2], psid]) {
        Some(x) => x,
        None => return ([0.0; 3], [0.0; 3]),
    };
    let om_ref = [sol[0], sol[1], sol[2]];
    // (15): rhs = [s_ref; psidd] - [R (2 taud + tau [Om]x) [iz]x^T Om ; Sdot Om]
    let w = mv(&SKIZ_T, om);
    let t1 = add(scl(w, 2.0 * taud), scl(cross(om, w), tau));
    let aa = mv(r, t1);
    // Sdot Om: columns of R rotate with R [Om]x
    let rd = mm(r, &skew(om));
    let (dbx, dby, dbz) = (col(&rd, 0), col(&rd, 1), col(&rd, 2));
    let nd = [-dbx[1], dbx[0]];
    let pd = [[0.0, -dbz[0], dby[0]], [0.0, -dbz[1], dby[1]]];
    let dd = 2.0 * (bx[0] * dbx[0] + bx[1] * dbx[1]);
    let mut sdot = [0.0f32; 3];
    for k in 0..3 {
        let t = nd[0] * p[0][k] + nd[1] * p[1][k] + n[0] * pd[0][k] + n[1] * pd[1][k];
        sdot[k] = t / d - np[k] * dd / (d * d);
    }
    let sdom = dot(sdot, om);
    let rhs = [sn[0] - aa[0], sn[1] - aa[1], sn[2] - aa[2], psidd - sdom];
    let sol2 = match solve4(a, rhs) {
        Some(x) => x,
        None => return (om_ref, [0.0; 3]),
    };
    (om_ref, [sol2[0], sol2[1], sol2[2]])
}

unsafe fn step_inner(s: &mut State, control: &mut control_s, sp: &setpoint_s, sensors: &sensorData_s, st: &state_s) {
    let mode_abs = crate::bindings::mode_e_modeAbs;
    let mode_dis = crate::bindings::mode_e_modeDisable;
    let cm_ft = crate::bindings::control_mode_e_controlModeForceTorque;
    let uptr = (&mut control.__bindgen_anon_1) as *mut _ as *mut f32;

    let in_pos_mode = sp.mode.x == mode_abs || sp.mode.y == mode_abs || sp.mode.z == mode_abs;
    if !in_pos_mode {
        // Only the position-mode setpoints of the study are supported; everything else = motors off.
        let _ = mode_dis;
        control.controlMode = cm_ft;
        for k in 0..4 { *uptr.add(k) = 0.0; }
        reprime(s);
        return;
    }

    // ---- gains (Table III defaults, ctrlTk params) --------------------------------------
    let kx = [g_tk_kx_xy, g_tk_kx_xy, g_tk_kx_z];
    let kv = [g_tk_kv_xy, g_tk_kv_xy, g_tk_kv_z];
    let ka = [g_tk_ka_xy, g_tk_ka_xy, g_tk_ka_z];
    let kxi = [g_tk_kxi_xy, g_tk_kxi_xy, g_tk_kxi_z];
    let kom = [g_tk_kom_xy, g_tk_kom_xy, g_tk_kom_z];
    let fc = g_tk_fc;
    if fc != s.fc_applied && fc > 0.0 { design_all(s, fc); }
    let m = g_indi_mass;
    let kt = [g_indi_kt1, g_indi_kt2, g_indi_kt3, g_indi_kt4];

    // ---- inputs: ENU/FLU -> NED/FRD ----------------------------------------------------------
    let q = &st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1;
    let r_enu = quat_to_rot(q.q3, q.q0, q.q1, q.q2);
    // R_ned = C_W R_enu C_B,  C_W: (e,n,u) -> (n,e,-u)
    let mut r: M = [[0.0; 3]; 3];
    for j in 0..3 {
        r[0][j] = r_enu[1][j] * CB[j];
        r[1][j] = r_enu[0][j] * CB[j];
        r[2][j] = -r_enu[2][j] * CB[j];
    }
    let w_ned = |v: V| -> V { [v[1], v[0], -v[2]] };
    let x = w_ned([st.position.x, st.position.y, st.position.z]);
    let v = w_ned([st.velocity.x, st.velocity.y, st.velocity.z]);
    let g = &sensors.gyroNoLpf;
    let om = [g.axis[0] * DEG2RAD * CB[0], g.axis[1] * DEG2RAD * CB[1], g.axis[2] * DEG2RAD * CB[2]];
    let ac = &sensors.acc;
    let ab = [ac.axis[0] * GRAV * CB[0], ac.axis[1] * GRAV * CB[1], ac.axis[2] * GRAV * CB[2]];
    let iz: V = [0.0, 0.0, 1.0];

    // ---- filtered measurements (Fig. 4) --------------------------------------------------------
    let a_f = filt3(&mut s.f_a, add(mv(&r, ab), scl(iz, GRAV))); // (16)
    let om_f = filt3(&mut s.f_om, om);
    let rpm_ok = oot_rpm_logs_available();
    let mut m1 = 0u16;
    let mut m2 = 0u16;
    let mut m3 = 0u16;
    let mut m4 = 0u16;
    if rpm_ok { rpm_get_all(&mut m1, &mut m2, &mut m3, &mut m4); }
    let rpm = [m1 as f32, m2 as f32, m3 as f32, m4 as f32];
    let bz = col(&r, 2);
    let s_f: V;
    let tau_f: f32;
    let mut w_f = [0.0f32; 4];
    let mu_f_flu: Option<V>;
    if rpm_ok {
        let mut t_i = [0.0f32; 4];
        let mut tsum = 0.0f32;
        for i in 0..4 { t_i[i] = kt[i] * rpm[i] * rpm[i]; tsum += t_i[i]; }
        let tau_raw = -tsum / m;
        s_f = filt3(&mut s.f_s, scl(bz, tau_raw));
        let mut w2f = [0.0f32; 4];
        let mut w2sum = 0.0f32;
        for i in 0..4 {
            w2f[i] = s.f_w2[i].update(t_i[i]);
            w_f[i] = s.f_w[i].update(rpm[i] * 2.0 * PI / 60.0);
            w2sum += w2f[i];
        }
        tau_f = -w2sum / m;
        let mut tx = 0.0f32;
        let mut ty = 0.0f32;
        let mut tz = 0.0f32;
        for i in 0..4 { tx += MIX_X[i] * w2f[i]; ty += MIX_Y[i] * w2f[i]; tz += MIX_Z[i] * w2f[i]; }
        mu_f_flu = Some([ARM_M * tx, ARM_M * ty, T2T * tz]);
    } else {
        // GUARD: no motor-speed source -> IMU-only increment (s_f from the measured acceleration)
        s_f = sub(a_f, scl(iz, GRAV));
        tau_f = -norm(s_f);
        for f in s.f_w2.iter_mut().chain(s.f_w.iter_mut()) { f.update(0.0); }
        mu_f_flu = None;
    }
    let (omd_f, wd_f, taud_f) = if s.have_prev {
        (
            scl(sub(om_f, s.prev_om), 1.0 / DT),
            [
                (w_f[0] - s.prev_w[0]) / DT,
                (w_f[1] - s.prev_w[1]) / DT,
                (w_f[2] - s.prev_w[2]) / DT,
                (w_f[3] - s.prev_w[3]) / DT,
            ],
            (tau_f - s.prev_tau) / DT,
        )
    } else {
        ([0.0; 3], [0.0; 4], 0.0)
    };
    s.prev_om = om_f;
    s.prev_w = w_f;
    s.prev_tau = tau_f;
    s.have_prev = true;
    // mu_f = G1 (w^2)_f + G2 wdot_f (5); the rotor-inertia term G2 acts on the yaw row only
    let mu_f: V = match mu_f_flu {
        Some(mf) => {
            let mut jz = 0.0f32;
            for i in 0..4 { jz += MIX_Z[i] * wd_f[i]; }
            let mfz = mf[2] + g_tk_jrz * jz;
            [mf[0] * CB[0], mf[1] * CB[1], mfz * CB[2]]
        }
        None => [J[0] * omd_f[0], J[1] * omd_f[1], J[2] * omd_f[2]],
    };

    // ---- position / acceleration (17), (20), (21) -----------------------------------------------
    let xr = w_ned([sp.position.x, sp.position.y, sp.position.z]);
    let vr = w_ned([sp.velocity.x, sp.velocity.y, sp.velocity.z]);
    let ar = w_ned([sp.acceleration.x, sp.acceleration.y, sp.acceleration.z]);
    let jr = w_ned([sp.jerk.x, sp.jerk.y, sp.jerk.z]);
    let sr = w_ned([sp.snap.x, sp.snap.y, sp.snap.z]);
    let mut a_c = [0.0f32; 3];
    for i in 0..3 {
        a_c[i] = kx[i] * (xr[i] - x[i]) + kv[i] * (vr[i] - v[i]) + ka[i] * (ar[i] - a_f[i]) + ar[i];
    }
    let s_c = add(s_f, sub(a_c, a_f)); // (20)
    let ns = norm(s_c);
    let t_c = m * ns; // (21)

    // ---- incremental attitude (22)-(27) -------------------------------------------------------
    let v_b = if ns > 1e-6 { mtv(&r, scl(s_c, 1.0 / ns)) } else { [0.0, 0.0, -1.0] }; // (22) GUARD
    let cz = cross(iz, v_b);
    let mut qbar = [1.0 - dot(iz, v_b), -cz[0], -cz[1], -cz[2]]; // (23)
    let nq = libm::sqrtf(qbar[0] * qbar[0] + qbar[1] * qbar[1] + qbar[2] * qbar[2] + qbar[3] * qbar[3]);
    if nq > 1e-9 {
        for k in 0..4 { qbar[k] /= nq; }
    } else {
        qbar = [0.0, 1.0, 0.0, 0.0]; // GUARD: opposite direction, any rotation axis
    }
    let rbar = quat_to_rot(qbar[0], qbar[1], qbar[2], qbar[3]);
    let psi = PI / 2.0 - sp.attitude.yaw * DEG2RAD; // ENU yaw -> NED heading
    let nbar = mtv(&mm(&r, &rbar), [libm::sinf(psi), -libm::cosf(psi), 0.0]); // (24)
    let mut qpsi = [1.0f32, 0.0, 0.0, 0.0];
    if libm::fabsf(nbar[1]) > 1e-9 {
        if g_tk_yaw_lit != 0 {
            qpsi[3] = -nbar[0] / nbar[1]; // (25) as printed
        } else {
            let phi = libm::atanf(-nbar[0] / nbar[1]);
            qpsi = [libm::cosf(phi / 2.0), 0.0, 0.0, libm::sinf(phi / 2.0)];
        }
    }
    let nqp = libm::sqrtf(qpsi[0] * qpsi[0] + qpsi[3] * qpsi[3]);
    qpsi[0] /= nqp;
    qpsi[3] /= nqp;
    let qc = qmul(qbar, qpsi); // (26)
    let w = libm::fmaxf(-1.0, libm::fminf(1.0, qc[0]));
    let sq = libm::sqrtf(libm::fmaxf(0.0, 1.0 - w * w));
    let kk = if sq > 1e-6 { 2.0 * libm::acosf(w) / sq } else { 2.0 };
    let xi_e = [kk * qc[1], kk * qc[2], kk * qc[3]]; // (27)

    // ---- feed-forward (14), (15) with the measured tau_f, xi, Omega_f ------------------------------
    let (om_ref, omd_ref) = feedforward(
        &r, tau_f, taud_f, om_f, jr, sr, -sp.attitudeRate.yaw * DEG2RAD, -sp.attitudeAcc.yaw * DEG2RAD,
    );
    let mut omd_c = [0.0f32; 3];
    for i in 0..3 {
        omd_c[i] = kxi[i] * xi_e[i] + kom[i] * (om_ref[i] - om_f[i]) + omd_ref[i]; // (28)
    }
    let mut mu_c = [0.0f32; 3];
    for i in 0..3 {
        mu_c[i] = mu_f[i] + J[i] * (omd_c[i] - omd_f[i]); // (31)
    }
    let mu = [mu_c[0] * CB[0], mu_c[1] * CB[1], mu_c[2] * CB[2]]; // FRD -> FLU

    // a_res = a_meas - a_model [ENU], same definition as controller 6 / 9 / 10
    let a_model = add(s_f, scl(iz, GRAV));
    let ares = sub(a_f, a_model);
    indi_a_res_write(ares[1], ares[0], -ares[2]);
    indi_tau_write(mu[0], mu[1], mu[2]);
    indi_e_r_write(xi_e[0], -xi_e[1], -xi_e[2], norm(xi_e));
    tk_diag_write(t_c, if rpm_ok { 1.0 } else { 0.0 }, ns, tau_f);

    control.controlMode = cm_ft;
    *uptr.add(0) = t_c;
    *uptr.add(1) = mu[0];
    *uptr.add(2) = mu[1];
    *uptr.add(3) = mu[2];
}

#[no_mangle]
pub extern "C" fn oot6_state_ptr() -> *mut u8 {
    unsafe { core::ptr::addr_of_mut!(ST) as *mut u8 }
}

#[no_mangle]
pub extern "C" fn oot6_state_size() -> usize {
    core::mem::size_of::<State>()
}

#[no_mangle]
pub extern "C" fn controllerOutOfTree6Init() {
    unsafe {
        let s = &mut *core::ptr::addr_of_mut!(ST);
        let fc = g_tk_fc;
        design_all(s, if fc > 0.0 { fc } else { 30.0 });
        reprime(s);
        s.initialized = true;
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTree6Test() -> bool {
    true
}

#[no_mangle]
pub unsafe extern "C" fn controllerOutOfTree6(
    control: *mut control_s,
    setpoint: *const setpoint_s,
    sensors: *const sensorData_s,
    state: *const state_s,
    _tick: u32,
) {
    let already_init = core::ptr::addr_of!((*core::ptr::addr_of!(ST)).initialized).read();
    if !already_init {
        controllerOutOfTree6Init();
    }
    let s = &mut *core::ptr::addr_of_mut!(ST);
    step_inner(s, &mut *control, &*setpoint, &*sensors, &*state);
}
