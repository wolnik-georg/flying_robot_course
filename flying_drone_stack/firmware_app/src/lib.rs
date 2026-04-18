//! SE(3) Geometric Controller — Crazyflie Firmware App (no_std Rust)
//!
//! Implements the Lee et al. (2010) geometric controller on SE(3) as a
//! Crazyflie out-of-tree controller.  The firmware calls the three
//! `controllerOutOfTree*` entry points at 500 Hz (ATTITUDE_RATE).
//!
//! Architecture:
//! ```text
//! Laptop Rust stack (20 Hz, radio)          Crazyflie STM32 (500 Hz)
//! ┌────────────────────────────┐            ┌──────────────────────────────┐
//! │ Trajectory generator       │            │ Sensor drivers (IMU, flow)   │
//! │ SLAM / MEKF / safety layer │──setpoint─▶│ Kalman EKF state estimator   │
//! │ CSV logging                │            │ ← controllerOutOfTree() ←    │
//! └────────────────────────────┘            │   (this file, 500 Hz, no_std)│
//!                                           │ Motor mixing + PWM drivers   │
//!                                           └──────────────────────────────┘
//! ```
//!
//! # Lecture slides requirements addressed
//! - `#![no_std]` — std → core (lecture: "switch from std to core")
//! - `f32` arithmetic throughout (lecture: "switch to f32")
//! - `bindgen` generates FFI types from `stabilizer_types.h` (lecture: "bindgen")
//! - Controller logic in Rust, linked into the C firmware binary
//!
//! # Future extension: INDI controller
//! Replace `geometric_control()` with an INDI rate loop.  The `sensors.gyro`
//! at 500 Hz is already available here — exactly what INDI needs for the
//! incremental angular-rate term `Δω = ω - ω_prev`.

#![no_std]
#![allow(non_upper_case_globals, non_camel_case_types, non_snake_case)]

use panic_halt as _;

// ── Bindgen-generated bindings ────────────────────────────────────────────────
// Generated at build time from wrapper.h → stabilizer_types.h + controller.h.
// Placed in $OUT_DIR/bindings.rs by build.rs.
mod bindings {
    #![allow(dead_code, non_upper_case_globals, non_camel_case_types, non_snake_case,
             clippy::all)]
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

use bindings::{control_s, setpoint_s, sensorData_s, state_s};

// ── 3-D vector arithmetic (no_std, f32) ───────────────────────────────────────

#[derive(Copy, Clone)]
struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}

impl Vec3 {
    #[inline(always)]
    const fn new(x: f32, y: f32, z: f32) -> Self { Self { x, y, z } }

    #[inline(always)]
    const fn zero() -> Self { Self::new(0.0, 0.0, 0.0) }

    #[inline(always)]
    fn add(self, b: Self) -> Self { Self::new(self.x+b.x, self.y+b.y, self.z+b.z) }

    #[inline(always)]
    fn sub(self, b: Self) -> Self { Self::new(self.x-b.x, self.y-b.y, self.z-b.z) }

    #[inline(always)]
    fn scale(self, s: f32) -> Self { Self::new(self.x*s, self.y*s, self.z*s) }

    #[inline(always)]
    fn dot(self, b: Self) -> f32 { self.x*b.x + self.y*b.y + self.z*b.z }

    #[inline(always)]
    fn cross(self, b: Self) -> Self {
        Self::new(
            self.y*b.z - self.z*b.y,
            self.z*b.x - self.x*b.z,
            self.x*b.y - self.y*b.x,
        )
    }

    #[inline(always)]
    fn norm_sq(self) -> f32 { self.x*self.x + self.y*self.y + self.z*self.z }

    #[inline(always)]
    fn norm(self) -> f32 { libm::sqrtf(self.norm_sq()) }

    #[inline(always)]
    fn normalize(self) -> Self {
        let n = self.norm();
        if n > 1e-9 { self.scale(1.0 / n) } else { Self::new(0.0, 0.0, 1.0) }
    }
}

// ── 3×3 rotation-matrix helpers ───────────────────────────────────────────────

type Mat3 = [[f32; 3]; 3];

/// Build a rotation matrix R from unit quaternion (qw, qx, qy, qz).
///
/// Quaternion convention confirmed from Crazyflie `controller_brescianini.c`:
///   `state->attitudeQuaternion.{x, y, z, w}` → qx, qy, qz, qw (scalar last).
/// The union in `stabilizer_types.h` maps: q0↔x, q1↔y, q2↔z, q3↔w.
///
/// R maps body frame to world frame: v_world = R · v_body.
#[inline]
fn quat_to_rot(qw: f32, qx: f32, qy: f32, qz: f32) -> Mat3 {
    [
        [qw*qw+qx*qx-qy*qy-qz*qz, 2.0*(qx*qy-qw*qz),          2.0*(qx*qz+qw*qy)         ],
        [2.0*(qx*qy+qw*qz),        qw*qw-qx*qx+qy*qy-qz*qz,   2.0*(qy*qz-qw*qx)         ],
        [2.0*(qx*qz-qw*qy),        2.0*(qy*qz+qw*qx),          qw*qw-qx*qx-qy*qy+qz*qz  ],
    ]
}

/// A^T · B  (used for rotation error: Rd^T R and R^T Rd)
#[inline]
fn mat_at_b(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut o = [[0.0f32; 3]; 3];
    for i in 0..3 { for j in 0..3 { for k in 0..3 { o[i][j] += a[k][i] * b[k][j]; } } }
    o
}

/// A - B
#[inline]
fn matsub(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut o = [[0.0f32; 3]; 3];
    for i in 0..3 { for j in 0..3 { o[i][j] = a[i][j] - b[i][j]; } }
    o
}

/// Vee operator ½(M - Mᵀ)∨ for a skew-symmetric matrix M = Rd^T R - R^T Rd.
/// Extracts the rotation-error vector eR.
#[inline]
fn vee_half(m: &Mat3) -> Vec3 {
    Vec3::new(m[2][1] * 0.5, m[0][2] * 0.5, m[1][0] * 0.5)
}

// ── Physical constants (Crazyflie 2.1) ────────────────────────────────────────
// Source: firmware controller_lee.c and System ID paper (Foerster, ETHZ)

const MASS: f32 = 0.027;           // kg
const GRAVITY: f32 = 9.81;         // m/s²
const JXX: f32 = 16.571710e-6;     // kg·m²
const JYY: f32 = 16.655602e-6;
const JZZ: f32 = 29.261652e-6;

// ── Controller gains ──────────────────────────────────────────────────────────
// Identical to the offboard GeometricController::default() in controller/mod.rs.
// Validated in hardware (Mar 2026, circle and hover flights).

const KP_X: f32 = 12.0;    // position proportional (XY)
const KP_Y: f32 = 12.0;
const KP_Z: f32 = 7.0;     // position proportional (Z)
const KV_X: f32 = 8.0;     // position derivative (XY)
const KV_Y: f32 = 8.0;
const KV_Z: f32 = 4.0;     // position derivative (Z)
const KI_P: f32 = 0.05;    // position integral (all axes)
const KI_LIMIT: f32 = 0.5; // anti-windup clamp [m/s²·s]
const KR_X: f32 = 0.007;   // attitude proportional
const KR_Y: f32 = 0.007;
const KR_Z: f32 = 0.008;
const KW_X: f32 = 0.00115; // attitude derivative
const KW_Y: f32 = 0.00115;
const KW_Z: f32 = 0.002;

// ── INDI preparation — inverse inertia + motor geometry ───────────────────────
//
// These constants are NOT used by the geometric controller.  They are pre-computed
// here so the INDI rate loop can reference them without changing the build.
//
// Inverse inertia (diagonal): maps torque → angular acceleration.
//   J_INV_XX = 1 / JXX,  etc.  Units: (N·m)⁻¹·(rad/s²) = rad·s⁻²·N⁻¹·m⁻¹
#[allow(dead_code)]
const J_INV_XX: f32 = 60_343.4_f32;   // 1 / 16.5717e-6
#[allow(dead_code)]
const J_INV_YY: f32 = 60_039.8_f32;   // 1 / 16.6556e-6
#[allow(dead_code)]
const J_INV_ZZ: f32 = 34_176.5_f32;   // 1 / 29.2617e-6
//
// Crazyflie 2.1 motor geometry (for reference — used by power_distribution_quadrotor.c):
//   ARM_LENGTH    = 0.046 m  (centre to motor, diagonal)
//   arm_eff       = sqrt(2)/2 × ARM_LENGTH = 0.032527 m  (projected along roll/pitch axis)
//   THRUST2TORQUE = 0.005964552  (reaction-torque / thrust, default props)
//
// Motor torque allocation B (3×4): [τx, τy, τz]^T = B · [f1, f2, f3, f4]^T
//   τx = arm_eff × (−f1 − f2 + f3 + f4)   (roll)
//   τy = arm_eff × (−f1 + f2 + f3 − f4)   (pitch)
//   τz = THRUST2TORQUE × (−f1 + f2 − f3 + f4)  (yaw)
//
// B⁻¹ (pseudo-inverse, 4×3) is used in motor-level INDI.
// For torque-level INDI (replacing geometric attitude loop), only J_INV is needed:
//   Δτ = J · (ω̇_des − ω̇_filt)   →   multiply result by J, not J_INV.

// ── Controller state ──────────────────────────────────────────────────────────

struct State {
    i_ep:       Vec3,  // position integral accumulator
    last_tick:  u32,   // previous stabilizer tick for dt computation
    omega_prev: Vec3,  // previous body angular rate — stored for INDI Δω = ω − ω_prev
}

impl State {
    const fn zero() -> Self {
        Self { i_ep: Vec3::zero(), last_tick: 0, omega_prev: Vec3::zero() }
    }

    fn reset(&mut self) {
        self.i_ep       = Vec3::zero();
        self.last_tick  = 0;
        self.omega_prev = Vec3::zero();
    }
}

// Single static instance — only accessed from the stabilizer FreeRTOS task
// (no concurrent access, so `static mut` is safe here).
static mut CTRL: State = State::zero();

// ── Desired rotation matrix ───────────────────────────────────────────────────

/// Compute Rd from thrust direction and desired yaw angle.
/// Matches `controllerLeeLeeComputeDesiredRot()` in firmware (Lee et al. 2010, Eq. 7).
fn desired_rot(f_d: Vec3, yaw_d: f32) -> Mat3 {
    let zdes = f_d.normalize();                                   // ẑ_d = F_d / |F_d|
    let xcdes = Vec3::new(libm::cosf(yaw_d), libm::sinf(yaw_d), 0.0); // desired heading
    let zcx   = zdes.cross(xcdes);
    let ydes  = if zcx.norm() > 1e-9 { zcx.normalize() } else { Vec3::new(0.0, 1.0, 0.0) };
    let xdes  = ydes.cross(zdes);

    // Column-major: Rd = [xdes | ydes | zdes]
    [
        [xdes.x, ydes.x, zdes.x],
        [xdes.y, ydes.y, zdes.y],
        [xdes.z, ydes.z, zdes.z],
    ]
}

// ── Core SE(3) control step ───────────────────────────────────────────────────

/// Compute thrust [N] and body torques [Nm] for the given state and setpoint.
///
/// Implements the Lee geometric controller (Lee et al. CDC 2010):
///   F_d = m (ÿ_d + Kp·ep + Kv·ev + Ki·∫ep + g·ẑ)
///   τ   = −KR·eR − Kω·eΩ + ω × Jω
fn geometric_step(
    pos:   Vec3,  // current position [m]
    vel:   Vec3,  // current velocity [m/s]
    r:    &Mat3,  // current rotation (body→world)
    omega: Vec3,  // current body angular rate [rad/s]
    pd:    Vec3,  // desired position [m]
    vd:    Vec3,  // desired velocity [m/s]
    ad:    Vec3,  // desired acceleration (feed-forward) [m/s²]
    yaw_d: f32,   // desired yaw [rad]
    dt:    f32,   // time step [s]
    s:    &mut State,
) -> (f32, Vec3) {
    // Position and velocity errors
    let ep = pd.sub(pos);
    let ev = vd.sub(vel);

    // Position integral with anti-windup clamp
    s.i_ep = s.i_ep.add(ep.scale(dt));
    s.i_ep = Vec3::new(
        s.i_ep.x.clamp(-KI_LIMIT, KI_LIMIT),
        s.i_ep.y.clamp(-KI_LIMIT, KI_LIMIT),
        s.i_ep.z.clamp(-KI_LIMIT, KI_LIMIT),
    );

    // Desired world-frame force per unit mass, then scale by mass
    let f_d = ad
        .add(Vec3::new(KP_X*ep.x, KP_Y*ep.y, KP_Z*ep.z))
        .add(Vec3::new(KV_X*ev.x, KV_Y*ev.y, KV_Z*ev.z))
        .add(Vec3::new(KI_P*s.i_ep.x, KI_P*s.i_ep.y, KI_P*s.i_ep.z))
        .add(Vec3::new(0.0, 0.0, GRAVITY));
    let thrust_vec = f_d.scale(MASS);

    // Thrust: project desired force onto current body z-axis (Lee Eq. 14)
    let body_z = Vec3::new(r[0][2], r[1][2], r[2][2]);
    let thrust = thrust_vec.dot(body_z);

    // Reset integrals on very low thrust (mirrors firmware behaviour)
    if thrust < 0.01 {
        s.i_ep = Vec3::zero();
    }

    // Desired rotation matrix from thrust direction and yaw
    let rd = desired_rot(thrust_vec, yaw_d);

    // Rotation error: eR = ½ (Rd^T R − R^T Rd)^∨  (Lee Eq. 10)
    let er = vee_half(&matsub(&mat_at_b(&rd, r), &mat_at_b(r, &rd)));

    // Angular velocity error: eΩ = ω − R^T Rd ω_d
    // For position setpoints ω_d = 0 → eΩ = ω (simplified, adequate for hover + circle)
    let e_omega = omega;

    // Gyroscopic compensation: ω × Jω
    let j_omega   = Vec3::new(JXX*omega.x, JYY*omega.y, JZZ*omega.z);
    let gyro_comp = omega.cross(j_omega);

    // Torque: τ = −KR·eR − Kω·eΩ + ω×Jω  (Lee Eq. 16)
    let torque = Vec3::new(
        -KR_X*er.x - KW_X*e_omega.x + gyro_comp.x,
        -KR_Y*er.y - KW_Y*e_omega.y + gyro_comp.y,
        -KR_Z*er.z - KW_Z*e_omega.z + gyro_comp.z,
    );

    (thrust.max(0.0), torque)
}

// ── Firmware entry points ─────────────────────────────────────────────────────

/// Called once when the firmware selects ControllerTypeOot.
#[no_mangle]
pub extern "C" fn controllerOutOfTreeInit() {
    // SAFETY: called from firmware init, single-threaded at this point.
    unsafe { (*core::ptr::addr_of_mut!(CTRL)).reset(); }
}

/// Firmware self-test hook — always passes.
#[no_mangle]
pub extern "C" fn controllerOutOfTreeTest() -> bool {
    true
}

/// Main control callback — called at 500 Hz by the stabilizer task.
///
/// Reads `state` (Kalman EKF estimate) and `sensors` (raw IMU), reads the
/// position `setpoint` sent from the laptop, runs the SE(3) controller, and
/// writes thrust + torques into `control` using `controlModeForceTorque`.
///
/// # INDI extension point
/// To implement INDI, replace `geometric_step()` with an incremental law:
///   Δu = G⁻¹ (ν_des − ω̇_meas + G·u₀)
/// `omega` and `omega_prev` (add a field to `State`) are already available here.
#[no_mangle]
pub unsafe extern "C" fn controllerOutOfTree(
    control:  *mut control_s,
    setpoint: *const setpoint_s,
    sensors:  *const sensorData_s,
    state:    *const state_s,
    tick:     u32,          // stabilizerStep_t = uint32_t; stabilizer runs at 1000 Hz
) {
    let s = &mut *core::ptr::addr_of_mut!(CTRL);

    // dt from tick counter (tick increments by 1 per ms; controller called every 2 ms)
    let dt = if s.last_tick == 0 {
        0.002_f32
    } else {
        (tick.wrapping_sub(s.last_tick)) as f32 * 0.001_f32
    };
    s.last_tick = tick;

    // ── Current state from Kalman EKF ─────────────────────────────────────────
    let st = &*state;

    let pos = Vec3::new(st.position.x, st.position.y, st.position.z);
    let vel = Vec3::new(st.velocity.x, st.velocity.y, st.velocity.z);

    // Quaternion convention (stabilizer_types.h union, verified in controller_brescianini.c):
    //   {x, y, z, w} member of the union — w is the scalar (Hamilton, scalar-last storage).
    //   In the q0..q3 naming: q0=x(qx), q1=y(qy), q2=z(qz), q3=w(qw=scalar).
    // Extra nesting level: quaternion_s outer union → inner anonymous struct → q0..q3
    let qw = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q3; // scalar
    let qx = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q0;
    let qy = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q1;
    let qz = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q2;
    let r = quat_to_rot(qw, qx, qy, qz);

    // Gyro from IMU: firmware stores in deg/s → convert to rad/s for the controller.
    // sensors.gyro is Axis3f; access via .axis[0..2] (avoids nested union names).
    let deg2rad = core::f32::consts::PI / 180.0_f32;
    let g = &(*sensors).gyro;
    let omega = Vec3::new(
        g.axis[0] * deg2rad,
        g.axis[1] * deg2rad,
        g.axis[2] * deg2rad,
    );

    // ── Setpoint from laptop (position + yaw via CRTP) ────────────────────────
    //
    // The laptop calls cf.commander.setpoint_position(x, y, z, yaw) which sends a
    // CRTP `positionType` (type 7) packet.  That packet carries ONLY position and yaw;
    // the firmware zeros the setpoint struct before decoding, so:
    //   setpoint.velocity     = [0, 0, 0]   — no velocity feedforward
    //   setpoint.acceleration = [0, 0, 0]   — no acceleration feedforward
    //
    // vd and ad below are therefore zero for all current flight modes.  The controller
    // still works correctly (position P+I compensates), but trajectory tracking on
    // circle/figure-8 maneuvers loses the velocity/acceleration feedforward terms.
    //
    // TODO (INDI project): switch main.rs to send `fullStateType` (type 6) packets so
    // that vd and ad carry the trajectory velocity and acceleration.  This also requires
    // extracting yaw from setpoint.attitudeQuaternion instead of setpoint.attitude.yaw.
    let sp = &*setpoint;
    let pd = Vec3::new(sp.position.x, sp.position.y, sp.position.z);
    let vd = Vec3::new(sp.velocity.x, sp.velocity.y, sp.velocity.z);
    let ad = Vec3::new(sp.acceleration.x, sp.acceleration.y, sp.acceleration.z);
    // Desired yaw: positionType stores it in degrees in setpoint.attitude.yaw
    let yaw_d = sp.attitude.yaw * deg2rad;

    // ── SE(3) geometric controller ────────────────────────────────────────────
    let (thrust_si, torque) = geometric_step(
        pos, vel, &r, omega,
        pd, vd, ad, yaw_d,
        dt, s,
    );

    // Store angular rate for next cycle (INDI needs Δω = ω − ω_prev).
    s.omega_prev = omega;

    // ── Write output: controlModeForceTorque (mode = 1) ───────────────────────
    //
    // Layout of control_s union (stabilizer_types.h, forceTorque member):
    //   offset  0 (f32): thrustSi [N]
    //   offset  4 (f32): torque[0] = torqueX [Nm]
    //   offset  8 (f32): torque[1] = torqueY [Nm]
    //   offset 12 (f32): torque[2] = torqueZ [Nm]
    //   offset 16 (u32): controlMode = 1 (controlModeForceTorque)
    //
    // We write via a *mut f32 pointer into the union to avoid deep bindgen
    // anonymous-struct names while keeping the layout explicit and correct.
    let out = &mut *control;
    let union_ptr = (&mut out.__bindgen_anon_1) as *mut _ as *mut f32;
    *union_ptr.add(0) = thrust_si;
    *union_ptr.add(1) = torque.x;
    *union_ptr.add(2) = torque.y;
    *union_ptr.add(3) = torque.z;
    // controlMode field follows the union in the struct
    out.controlMode = bindings::control_mode_e_controlModeForceTorque;
}
