//! controller=7 (`ControllerTypeOot2`) — a faithful Rust port of Cobo & Briesewitz's
//! NA-INDI controller, `~/Desktop/NA-INDI-firmware/src/modules/src/controller/controller_lee.c`
//! (MIT, Copyright (c) 2024 Khaled Wahba). Kept in its own module / own controller slot so it
//! never touches `lib.rs`'s geometric+INDI (`ctrl_mode` 0-3, `ControllerTypeOot`).
//!
//! Operator instruction (2026-09-14): match their code and implementation exactly, including
//! the mixed-dt-source design that reads like a bug (see `attitude_indi` below). Not "fixed"
//! on purpose.
//!
//! Deviations from the reference source, and why each is unavoidable rather than a choice:
//!   - `mass`/`J`: reference hardcodes 0.034 kg / their own J for a specific vehicle. We use
//!     this project's own `g_indi_mass` / `JXX,JYY,JZZ` (per-platform, `lib.rs`) because flying
//!     with the reference's fixed mass on a different, heavier airframe is a physical error, not
//!     an implementation choice. The control LAW, gains, filters and dt handling are unchanged.
//!   - `sensors->gyroNoLpf`: the reference's own firmware fork added an unfiltered-gyro field to
//!     `stabilizer_types.h` that upstream bitcraze (and therefore our firmware) does not have.
//!     We substitute the regular filtered `sensors.gyro` — confirmed absent via bindgen output,
//!     not assumed.
//!   - `use_nn` / the NN feedforward block: omitted. Every gain block we found upstream ships
//!     with `use_nn=0`; the block is dead code in every flown NA-INDI config. Left out rather
//!     than ported unused.
//!   - RPM→thrust: reference reads `kappa_f` per motor from a deck-specific PWM/RPM log path
//!     (`indi & 4` branch) we don't have wired the same way. We reuse this project's own
//!     `g_indi_kt1..4` (t_i = kt_i * rpm_i^2) and `rpm_get_all()`, already the correct per-motor
//!     thrust constants for this hardware.
//!
//! Everything else — Kpos_P/D/I + limits, KR/Komega/KI, the 80/40/10 Hz Butterworth cutoffs,
//! the position-side fixed dt vs. the attitude-INDI's wall-clock-measured dt, the `vclampnorm`
//! clamps, the rotation-error sign convention, the `RATE_DO_EXECUTE(ATTITUDE_RATE, tick)` gate —
//! is copied as literally as the port allows.

use crate::bindings::{control_s, setpoint_s, sensorData_s, state_s};
use crate::{quat_to_rot, mat_at_b, matsub, vee_half, mat_mul_vec, clamp_norm, Vec3, Mat3, GRAVITY};
use crate::{JXX, JYY, JZZ, g_indi_mass, g_indi_kt1, g_indi_kt2, g_indi_kt3, g_indi_kt4, g_indi_frame_conv};

extern "C" {
    fn rpm_get_all(m1: *mut u16, m2: *mut u16, m3: *mut u16, m4: *mut u16);
    fn usecTimestamp() -> u64;
}

// ── Reference `controller_lee.c` default gains (g_self), unchanged ─────────────────────────
const KPOS_P: Vec3 = Vec3 { x: 12.0, y: 12.0, z: 12.0 };
const KPOS_P_LIMIT: f32 = 100.0;
const KPOS_D: Vec3 = Vec3 { x: 10.5, y: 10.5, z: 10.5 };
const KPOS_D_LIMIT: f32 = 100.0;
const KPOS_I: Vec3 = Vec3 { x: 2.0, y: 2.0, z: 2.0 };
const KPOS_I_LIMIT: f32 = 100.0;
const KR: Vec3 = Vec3 { x: 0.007, y: 0.007, z: 0.01 };
const KOMEGA: Vec3 = Vec3 { x: 0.002, y: 0.002, z: 0.002 };
const KI_ATT: Vec3 = Vec3 { x: 0.01, y: 0.01, z: 0.01 };

// Reference indi=3 (position+attitude INDI both on) — the mode this port exists to fly.
const INDI_MODE: u8 = 3;

// ATTITUDE_RATE = 500 Hz, RATE_MAIN_LOOP = 1000 Hz (NA-INDI-firmware/stabilizer_types.h).
// The reference gates its OWN execution with RATE_DO_EXECUTE(ATTITUDE_RATE, tick) = tick % 2 == 0
// at the top of controllerLee() — reproduced literally below, not "fixed" to run every call.
const ATTITUDE_RATE: f32 = 500.0;
const DT_FIXED: f32 = 1.0 / ATTITUDE_RATE;

const CUTOFF_ACC: f32 = 80.0;  // Hz, position-INDI accel filters
const CUTOFF_TAU: f32 = 40.0;  // Hz, attitude-INDI roll/pitch filters
const CUTOFF_Z: f32 = 10.0;    // Hz, attitude-INDI yaw filter

// arm = sqrt(2)/2 * 0.046, t2t = 0.006 — reference literals for a stock CF2.1, kept verbatim
// (this is the RPM->torque mixer model, a property of the reference's own thrust/torque
// identification, not this project's airframe constant `ARM_M`/`TORQUE_RATIO`).
const ARM_REF: f32 = 0.707_106_78_f32 * 0.046;
const T2T_REF: f32 = 0.006;

/// Second-order Butterworth low-pass, exact `filter.h`/`filter.c` discretisation:
/// pre-warped bilinear, fixed Q = 0.7071, direct-form-II-ish two-tap history.
/// `init(tau, sample_time, value)` seeds state to `value` (reference always inits with 0.0,
/// i.e. a genuine cold start at boot — not seeded from the first live sample).
#[derive(Copy, Clone)]
struct Butterworth2LowPass {
    a: [f32; 2], b: [f32; 2], i: [f32; 2], o: [f32; 2],
}
impl Butterworth2LowPass {
    const fn zero() -> Self { Self { a: [0.0; 2], b: [0.0; 2], i: [0.0; 2], o: [0.0; 2] } }
    fn init(&mut self, cutoff_hz: f32, sample_time: f32, value: f32) {
        let tau = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
        let q = 0.7071_f32;
        let k = libm::tanf(sample_time / (2.0 * tau));
        let poly = k * k + k / q + 1.0;
        self.a[0] = 2.0 * (k * k - 1.0) / poly;
        self.a[1] = (k * k - k / q + 1.0) / poly;
        self.b[0] = k * k / poly;
        self.b[1] = 2.0 * self.b[0];
        self.i = [value, value];
        self.o = [value, value];
    }
    fn update(&mut self, value: f32) -> f32 {
        let out = self.b[0] * value + self.b[1] * self.i[0] + self.b[0] * self.i[1]
                - self.a[0] * self.o[0] - self.a[1] * self.o[1];
        self.i[1] = self.i[0]; self.i[0] = value;
        self.o[1] = self.o[0]; self.o[0] = out;
        out
    }
}

#[derive(Copy, Clone)]
struct Vec3Filt { x: Butterworth2LowPass, y: Butterworth2LowPass, z: Butterworth2LowPass }
impl Vec3Filt {
    const fn zero() -> Self { Self { x: Butterworth2LowPass::zero(), y: Butterworth2LowPass::zero(), z: Butterworth2LowPass::zero() } }
    fn update(&mut self, v: Vec3) -> Vec3 { Vec3::new(self.x.update(v.x), self.y.update(v.y), self.z.update(v.z)) }
}

struct State {
    i_error_pos: Vec3,
    i_error_att: Vec3,
    omega_prev: Vec3,       // previous (filtered, since we lack gyroNoLpf) body rate, rad/s
    timestamp_prev: u64,    // usecTimestamp() at the previous ATTITUDE_RATE-gated execution
    filter_acc_rpm: Vec3Filt,
    filter_acc_imu: Vec3Filt,
    filter_tau_rpm: Vec3Filt,
    filter_tau_imu: Vec3Filt,
    initialized: bool,
}
impl State {
    const fn zero() -> Self {
        Self {
            i_error_pos: Vec3::zero(), i_error_att: Vec3::zero(),
            omega_prev: Vec3::zero(), timestamp_prev: 0,
            filter_acc_rpm: Vec3Filt::zero(), filter_acc_imu: Vec3Filt::zero(),
            filter_tau_rpm: Vec3Filt::zero(), filter_tau_imu: Vec3Filt::zero(),
            initialized: false,
        }
    }
}

static mut ST: State = State::zero();

fn vclampscl(v: Vec3, limit: f32) -> Vec3 {
    Vec3::new(v.x.clamp(-limit, limit), v.y.clamp(-limit, limit), v.z.clamp(-limit, limit))
}

#[no_mangle]
pub extern "C" fn controllerOutOfTree2Init() {
    unsafe {
        let s = &mut *core::ptr::addr_of_mut!(ST);
        *s = State::zero();
        // Reference inits every filter once, in controllerLeeInit(), with sample_time =
        // 1/ATTITUDE_RATE and value = 0.0 — a fixed-rate design assumption baked in at boot,
        // reproduced verbatim (not re-derived from a measured dt the way lib.rs's own
        // Butterworth2::init does for ctrl_mode 0-3).
        s.filter_acc_rpm.x.init(CUTOFF_ACC, DT_FIXED, 0.0);
        s.filter_acc_rpm.y.init(CUTOFF_ACC, DT_FIXED, 0.0);
        s.filter_acc_rpm.z.init(CUTOFF_ACC, DT_FIXED, 0.0);
        s.filter_acc_imu.x.init(CUTOFF_ACC, DT_FIXED, 0.0);
        s.filter_acc_imu.y.init(CUTOFF_ACC, DT_FIXED, 0.0);
        s.filter_acc_imu.z.init(CUTOFF_ACC, DT_FIXED, 0.0);
        s.filter_tau_rpm.x.init(CUTOFF_TAU, DT_FIXED, 0.0);
        s.filter_tau_rpm.y.init(CUTOFF_TAU, DT_FIXED, 0.0);
        s.filter_tau_rpm.z.init(CUTOFF_Z,   DT_FIXED, 0.0);
        s.filter_tau_imu.x.init(CUTOFF_TAU, DT_FIXED, 0.0);
        s.filter_tau_imu.y.init(CUTOFF_TAU, DT_FIXED, 0.0);
        s.filter_tau_imu.z.init(CUTOFF_Z,   DT_FIXED, 0.0);
        s.initialized = true;
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTree2Test() -> bool {
    true
}

#[no_mangle]
pub unsafe extern "C" fn controllerOutOfTree2(
    control: *mut control_s,
    setpoint: *const setpoint_s,
    sensors: *const sensorData_s,
    state: *const state_s,
    tick: u32,
) {
    // RATE_DO_EXECUTE(ATTITUDE_RATE, tick) == (tick % (RATE_MAIN_LOOP / ATTITUDE_RATE) == 0)
    // == (tick % 2 == 0), verified against NA-INDI-firmware/stabilizer_types.h. Our own
    // stabilizer calls controller() every tick at 1000 Hz with no gate of its own (see
    // firmware_app/CLAUDE.md), so this internal gate is what makes the reference's fixed
    // 1/ATTITUDE_RATE dt design actually correct here too. On a skipped tick we leave
    // `*control` untouched, exactly as the reference's early `return;` does.
    if tick % 2 != 0 {
        return;
    }

    let s = &mut *core::ptr::addr_of_mut!(ST);
    if !s.initialized {
        controllerOutOfTree2Init();
    }
    let dt = DT_FIXED;

    let st = &*state;
    let pos = Vec3::new(st.position.x, st.position.y, st.position.z);
    let vel = Vec3::new(st.velocity.x, st.velocity.y, st.velocity.z);
    let qw = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q3;
    let qx = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q0;
    let qy = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q1;
    let qz = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q2;
    let r: Mat3 = quat_to_rot(qw, qx, qy, qz);
    let z_body = Vec3::new(r[0][2], r[1][2], r[2][2]);

    let deg2rad = core::f32::consts::PI / 180.0_f32;
    let g = &(*sensors).gyro;
    // Substitutes for gyroNoLpf -- see module doc.
    let omega = Vec3::new(g.axis[0] * deg2rad, g.axis[1] * deg2rad, g.axis[2] * deg2rad);
    let acc = &(*sensors).acc;

    let sp = &*setpoint;
    let pd = Vec3::new(sp.position.x, sp.position.y, sp.position.z);
    let vd = Vec3::new(sp.velocity.x, sp.velocity.y, sp.velocity.z);
    let acc_d = Vec3::new(sp.acceleration.x, sp.acceleration.y, sp.acceleration.z);
    let jerk_d = Vec3::new(sp.jerk.x, sp.jerk.y, sp.jerk.z);
    let snap_d = Vec3::new(sp.snap.x, sp.snap.y, sp.snap.z);
    let yaw_d = sp.attitude.yaw * deg2rad;
    let yaw_dot_d = sp.attitudeRate.yaw * deg2rad;

    let mass = g_indi_mass;

    // -- Position controller (reference: only when setpoint mode is "modeAbs"; our HLC/Mode E
    //    setpoints are always absolute position, so this is unconditional here) --------------
    let pos_e = vclampscl(pd.sub(pos), KPOS_P_LIMIT);
    let vel_e = vclampscl(vd.sub(vel), KPOS_D_LIMIT);
    s.i_error_pos = s.i_error_pos.add(pos_e.scale(dt));
    s.i_error_pos = vclampscl(s.i_error_pos, KPOS_I_LIMIT);
    let a_d = acc_d
        .add(Vec3::new(KPOS_D.x * vel_e.x, KPOS_D.y * vel_e.y, KPOS_D.z * vel_e.z))
        .add(Vec3::new(KPOS_P.x * pos_e.x, KPOS_P.y * pos_e.y, KPOS_P.z * pos_e.z))
        .add(Vec3::new(KPOS_I.x * s.i_error_pos.x, KPOS_I.y * s.i_error_pos.y, KPOS_I.z * s.i_error_pos.z));

    // -- RPM -> per-motor thrust (see module doc: our kt_i / rpm_get_all(), not kappa_f) -----
    let mut m1 = 0u16; let mut m2 = 0u16; let mut m3 = 0u16; let mut m4 = 0u16;
    rpm_get_all(&mut m1, &mut m2, &mut m3, &mut m4);
    let t1 = g_indi_kt1 * (m1 as f32) * (m1 as f32);
    let t2 = g_indi_kt2 * (m2 as f32) * (m2 as f32);
    let t3 = g_indi_kt3 * (m3 as f32) * (m3 as f32);
    let t4 = g_indi_kt4 * (m4 as f32) * (m4 as f32);
    let f_rpm = t1 + t2 + t3 + t4;

    // -- Position INDI (indi & 1) -------------------------------------------------------------
    let a_indi = if INDI_MODE & 1 != 0 {
        let a_rpm = clamp_norm(
            z_body.scale(f_rpm / mass).sub(Vec3::new(0.0, 0.0, GRAVITY)),
            10.0,
        );
        let a_rpm_f = s.filter_acc_rpm.update(a_rpm);
        let a_imu = clamp_norm(Vec3::new(acc.axis[0], acc.axis[1], acc.axis[2]).scale(GRAVITY), 10.0);
        let a_imu_f = s.filter_acc_imu.update(a_imu);
        a_imu_f.sub(a_rpm_f)
    } else {
        Vec3::zero()
    };

    let f_d = a_d.sub(a_indi);
    let thrust_si = mass * f_d.dot(z_body);
    if thrust_si < 0.01 {
        s.i_error_pos = Vec3::zero();
    }

    // -- Desired rotation from F_d (reference's xb/yb/zb construction) -----------------------
    let yc = Vec3::new(-libm::sinf(yaw_d), libm::cosf(yaw_d), 0.0);
    let xb = yc.cross(f_d).normalize();
    let yb = f_d.cross(xb).normalize();
    let zb = xb.cross(yb);
    let r_des: Mat3 = [
        [xb.x, yb.x, zb.x],
        [xb.y, yb.y, zb.y],
        [xb.z, yb.z, zb.z],
    ];

    // eR = 0.5 * vee(Rdes^T R - R^T Rdes); mat_at_b(a,b) = a^T b
    let e_rm = matsub(&mat_at_b(&r_des, &r), &mat_at_b(&r, &r_des));
    let e_r = vee_half(&e_rm);

    // Differential-flatness omega_des / omega_des_dot: this port's setpoints (HLC / Mode E)
    // carry only up to snap over CRTP, same as lib.rs's own omega_desired/alpha_desired path.
    // We reuse those, which implement the identical Tal & Karaman flatness relations the
    // reference derives inline via its B1/B3/C3/D1-D3/E1-E3 intermediates.
    let frame_conv = g_indi_frame_conv;
    let omega_des = crate::omega_desired(acc_d, jerk_d, yaw_d, frame_conv);
    let alpha_des = crate::alpha_desired(acc_d, jerk_d, snap_d, yaw_dot_d, frame_conv);

    let r_t_rdes = mat_at_b(&r, &r_des);
    let omega_r = mat_mul_vec(&r_t_rdes, omega_des);
    let omega_error = omega.sub(omega_r);
    s.i_error_att = s.i_error_att.add(e_r.scale(dt));

    let j = Vec3::new(JXX, JYY, JZZ);
    let j_omega = Vec3::new(j.x * omega.x, j.y * omega.y, j.z * omega.z);
    let gyro_term = omega.cross(j_omega);

    // u = -KR*eR - Komega*omega_error - KI*i_error_att + omega x (J*omega)
    //     - J * ( omega x omega_r - R^T Rdes * omega_des_dot )
    let cross_term = omega.cross(omega_r);
    let flat_term = mat_mul_vec(&r_t_rdes, alpha_des);
    let diff = cross_term.sub(flat_term);
    let last_term = Vec3::new(j.x * diff.x, j.y * diff.y, j.z * diff.z);
    let mut u = Vec3::new(-KR.x * e_r.x, -KR.y * e_r.y, -KR.z * e_r.z)
        .sub(Vec3::new(KOMEGA.x * omega_error.x, KOMEGA.y * omega_error.y, KOMEGA.z * omega_error.z))
        .sub(Vec3::new(KI_ATT.x * s.i_error_att.x, KI_ATT.y * s.i_error_att.y, KI_ATT.z * s.i_error_att.z))
        .add(gyro_term)
        .sub(last_term);

    // -- Attitude/torque INDI (indi & 2) -------------------------------------------------------
    if INDI_MODE & 2 != 0 {
        let tau_rpm = clamp_norm(
            Vec3::new(
                -ARM_REF * t1 - ARM_REF * t2 + ARM_REF * t3 + ARM_REF * t4,
                -ARM_REF * t1 + ARM_REF * t2 + ARM_REF * t3 - ARM_REF * t4,
                -T2T_REF * t1 + T2T_REF * t2 - T2T_REF * t3 + T2T_REF * t4,
            ),
            0.006,
        );
        let tau_rpm_f = s.filter_tau_rpm.update(tau_rpm);

        // Wall-clock dt for the angular-acceleration finite difference -- deliberately NOT
        // the fixed `dt` above. This mixed-dt-source design is the "rate issue" preserved
        // verbatim per the 2026-09-14 operator instruction, not corrected to share `dt`.
        let timestamp = usecTimestamp();
        let dt_meas = if s.timestamp_prev == 0 {
            dt
        } else {
            ((timestamp - s.timestamp_prev) as f32) / 1.0e6
        };
        let angular_acc = omega.sub(s.omega_prev).scale(1.0 / dt_meas.max(1e-6));
        let j_alpha = Vec3::new(j.x * angular_acc.x, j.y * angular_acc.y, j.z * angular_acc.z);
        let j_omega2 = Vec3::new(j.x * omega.x, j.y * omega.y, j.z * omega.z);
        let tau_imu = clamp_norm(j_alpha.sub(j_omega2.cross(omega)), 0.006);
        let tau_imu_f = s.filter_tau_imu.update(tau_imu);
        s.omega_prev = omega;
        s.timestamp_prev = timestamp;

        let indi_moments = tau_imu_f.sub(tau_rpm_f);
        u = u.sub(indi_moments);
    }

    // Reference resets the position integral above and arming is handled by the caller
    // (traj_iface.c zeroes setpoints pre-arm); write the raw union offsets the same way
    // lib.rs's own controllerOutOfTree does (control_s.thrust/torque share a bindgen union).
    let out = &mut *control;
    let union_ptr = (&mut out.__bindgen_anon_1) as *mut _ as *mut f32;
    *union_ptr.add(0) = thrust_si;
    *union_ptr.add(1) = u.x;
    *union_ptr.add(2) = u.y;
    *union_ptr.add(3) = u.z;
    out.controlMode = crate::bindings::control_mode_e_controlModeForceTorque;
}
