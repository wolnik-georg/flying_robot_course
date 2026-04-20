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

// // ── Safe adjustment: only attitude gains (KR/KW) increased slightly ───────
// // ── Only attitude gains increased to fight pitch/roll/yaw wobble during circle ─────
// const KP_X: f32 = 7.5;    const KP_Y: f32 = 7.5;    const KP_Z: f32 = 26.0;   // unchanged - good for hover & cross
// const KV_X: f32 = 9.0;    const KV_Y: f32 = 9.0;    const KV_Z: f32 = 14.0;   // unchanged
// const KI_P: f32 = 0.025;                                                        // unchanged
// const KI_LIMIT: f32 = 0.4;

// const KR_X: f32 = 0.0125; const KR_Y: f32 = 0.0125; const KR_Z: f32 = 0.0145; // rollback
// const KW_X: f32 = 0.0028; const KW_Y: f32 = 0.0028; const KW_Z: f32 = 0.0032; // rollback

// ── Exact gains from the official C Lee controller (CDC 2010) ─────────────
const KP_X: f32 = 7.0;    const KP_Y: f32 = 7.0;    const KP_Z: f32 = 7.0;
const KV_X: f32 = 4.0;    const KV_Y: f32 = 4.0;    const KV_Z: f32 = 4.0;
const KI_P: f32 = 0.0;    // official firmware uses 0.0 integral on position
const KI_LIMIT: f32 = 2.0;

const KR_X: f32 = 0.007;  const KR_Y: f32 = 0.007;  const KR_Z: f32 = 0.008;
const KW_X: f32 = 0.00115;const KW_Y: f32 = 0.00115;const KW_Z: f32 = 0.002;

// ── Controller State ───────────────────────────────────────────────────────
struct State {
    i_ep: Vec3,          // position integral
    last_tick: u32,
    omega_prev: Vec3,    // for future INDI
}

impl State {
    const fn zero() -> Self {
        Self { i_ep: Vec3::zero(), last_tick: 0, omega_prev: Vec3::zero() }
    }
    fn reset(&mut self) {
        self.i_ep = Vec3::zero();
        self.last_tick = 0;
        self.omega_prev = Vec3::zero();
    }
}

static mut CTRL: State = State::zero();

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

    // Angular velocity error (ω_d = 0 for hover)
    let e_omega = omega;

    // Gyroscopic term
    let j_omega = Vec3::new(JXX*omega.x, JYY*omega.y, JZZ*omega.z);
    let gyro_comp = omega.cross(j_omega);

    // Torque command
    let torque = Vec3::new(
        -KR_X*er.x - KW_X*e_omega.x + gyro_comp.x,
        -KR_Y*er.y - KW_Y*e_omega.y + gyro_comp.y,
        -KR_Z*er.z - KW_Z*e_omega.z + gyro_comp.z,
    );

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