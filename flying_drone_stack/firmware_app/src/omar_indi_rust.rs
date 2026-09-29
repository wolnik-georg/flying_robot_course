//! controller=10 (`ControllerTypeOot5`) — Rust line-port of `controller_omar_indi.c`
//! (stabilizer.controller=9, literal C reference). Same gains, PD+I position loop, Lee
//! `R_des`, attitude law with unconditional ω×Jω, `.indi` bitmask for RPM/IMU residuals.
//!
//! **Intentional deviation:** RPM availability uses `oot_rpm_logs_available()` (log-var-id
//! validation, same family as `rpm_get_all()` in traj_iface.c) — NOT Omar's
//! `paramGetVarId("deck","bcRpm")` probe that crashed controller=9 before deck.bcRpm
//! backport. RPM values always come from `rpm_get_all()`.

use crate::bindings::{control_s, setpoint_s, sensorData_s, state_s};
use crate::{quat_to_rot, mat_at_b, matsub, vee_half, mat_mul_vec, Vec3, Mat3, GRAVITY};

extern "C" {
    fn rpm_get_all(m1: *mut u16, m2: *mut u16, m3: *mut u16, m4: *mut u16);
    fn usecTimestamp() -> u64;
    fn oot_rpm_logs_available() -> bool;
    fn powerDistributionGetMaxThrust() -> f32;
    // Telemetry only -- shared with lib.rs/naindi.rs, additive, no control-law effect.
    // Added 2026-09-29: controller=10's first flight (A1) showed a single-motor-dominant
    // command pattern (one motor ramps while the other three idle, drone never lifts off)
    // but tau_x/y/z read dead-zero in the uSD log because nothing called these before --
    // the SAME gap exists in the C reference (controller_omar_indi.c also never calls
    // these), so it's not a Rust-port regression, but it means neither c=9 nor c=10 had
    // ever produced real torque telemetry on this project's uSD config until now.
    fn indi_tau_write(tx: f32, ty: f32, tz: f32);
    fn indi_a_res_write(ax: f32, ay: f32, az: f32);
    fn indi_e_r_write(ex: f32, ey: f32, ez: f32, norm: f32);
    // yaml/cfclient-settable: PARAM_GROUP(ctrlOot5).indi in traj_iface.c. Read live every
    // tick (not cached into State at Init) so it behaves like a real runtime param, same as
    // ctrlOmarIndi.indi for controller=9 -- see docs/41 §13 for why c=9 shipping with this
    // defaulting to 0 silently flew plain geometric instead of INDI for two days.
    static g_oot5_indi: u8;
}

const ATTITUDE_RATE: f32 = 500.0;
const DT: f32 = 1.0 / ATTITUDE_RATE;
const PI: f32 = core::f32::consts::PI;
const DEG2RAD: f32 = PI / 180.0;

// Omar g_self defaults (controller_omar_indi.c) — CF_MASS from platform 42700 µg.
const MASS: f32 = 0.0427;
const J: Vec3 = Vec3 {
    x: 16.571710e-6,
    y: 16.655602e-6,
    z: 29.261652e-6,
};
const KPOS_P: Vec3 = Vec3 { x: 7.0, y: 7.0, z: 7.0 };
const KPOS_P_LIMIT: f32 = 100.0;
const KPOS_D: Vec3 = Vec3 { x: 4.0, y: 4.0, z: 4.0 };
const KPOS_D_LIMIT: f32 = 100.0;
const KPOS_I: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };
const KPOS_I_LIMIT: f32 = 2.0;
const KR: Vec3 = Vec3 { x: 0.007, y: 0.007, z: 0.008 };
const KOMEGA: Vec3 = Vec3 { x: 0.00115, y: 0.00115, z: 0.002 };
const KI_ATT: Vec3 = Vec3 { x: 0.03, y: 0.03, z: 0.03 };

const MOTORRPM2FORCE: f32 = 3.911_940_273_307_74e-8;
const ARM: f32 = 0.707_106_781 * 0.050;
const T2T: f32 = 0.005_692_788_4;
const FILTER_CUTOFF_HZ: f32 = 30.0;

#[derive(Copy, Clone)]
struct Butterworth2 {
    a: [f32; 2],
    b: [f32; 2],
    i: [f32; 2],
    o: [f32; 2],
}
impl Butterworth2 {
    const fn zero() -> Self {
        Self { a: [0.0; 2], b: [0.0; 2], i: [0.0; 2], o: [0.0; 2] }
    }
    fn init(&mut self, cutoff_hz: f32, sample_time: f32, value: f32) {
        let tau = 1.0 / (2.0 * PI * cutoff_hz);
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
        self.i[1] = self.i[0];
        self.i[0] = value;
        self.o[1] = self.o[0];
        self.o[0] = out;
        out
    }
    fn get(&self) -> f32 {
        self.o[0]
    }
}

struct Vec3Filt {
    x: Butterworth2,
    y: Butterworth2,
    z: Butterworth2,
}
impl Vec3Filt {
    const fn zero() -> Self {
        Self {
            x: Butterworth2::zero(),
            y: Butterworth2::zero(),
            z: Butterworth2::zero(),
        }
    }
    fn init_all(&mut self, cutoff_hz: f32, sample_time: f32) {
        self.x.init(cutoff_hz, sample_time, 0.0);
        self.y.init(cutoff_hz, sample_time, 0.0);
        self.z.init(cutoff_hz, sample_time, 0.0);
    }
    fn update(&mut self, v: Vec3) -> Vec3 {
        Vec3::new(self.x.update(v.x), self.y.update(v.y), self.z.update(v.z))
    }
    fn get(&self) -> Vec3 {
        Vec3::new(self.x.get(), self.y.get(), self.z.get())
    }
}

struct State {
    indi: u8,
    i_error_pos: Vec3,
    i_error_att: Vec3,
    omega_prev: Vec3,
    timestamp_prev: u64,
    r_des: Mat3,
    filter_acc_rpm: Vec3Filt,
    filter_acc_imu: Vec3Filt,
    filter_tau_rpm: Vec3Filt,
    filter_angular_acc: Vec3Filt,
    initialized: bool,
}

static mut ST: State = State {
    indi: 0,
    i_error_pos: Vec3::zero(),
    i_error_att: Vec3::zero(),
    omega_prev: Vec3::zero(),
    timestamp_prev: 0,
    r_des: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    filter_acc_rpm: Vec3Filt::zero(),
    filter_acc_imu: Vec3Filt::zero(),
    filter_tau_rpm: Vec3Filt::zero(),
    filter_angular_acc: Vec3Filt::zero(),
    initialized: false,
};

#[inline]
fn vclampscl(v: Vec3, min: f32, max: f32) -> Vec3 {
    Vec3::new(
        libm::fmaxf(min, libm::fminf(max, v.x)),
        libm::fmaxf(min, libm::fminf(max, v.y)),
        libm::fmaxf(min, libm::fminf(max, v.z)),
    )
}

#[inline]
fn veltmul(k: Vec3, v: Vec3) -> Vec3 {
    Vec3::new(k.x * v.x, k.y * v.y, k.z * v.z)
}

#[inline]
fn mcolumn(m: &Mat3, col: usize) -> Vec3 {
    Vec3::new(m[0][col], m[1][col], m[2][col])
}

#[inline]
fn mcolumns(a: Vec3, b: Vec3, c: Vec3) -> Mat3 {
    [
        [a.x, b.x, c.x],
        [a.y, b.y, c.y],
        [a.z, b.z, c.z],
    ]
}

#[inline]
fn quat2rpy(qw: f32, qx: f32, qy: f32, qz: f32) -> Vec3 {
    Vec3::new(
        libm::atan2f(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)),
        libm::asinf(2.0 * (qw * qy - qx * qz)),
        libm::atan2f(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)),
    )
}

#[inline]
fn rpy2quat(rpy: Vec3) -> (f32, f32, f32, f32) {
    let (r, p, y) = (rpy.x, rpy.y, rpy.z);
    let (cr, sr) = (libm::cosf(r / 2.0), libm::sinf(r / 2.0));
    let (cp, sp) = (libm::cosf(p / 2.0), libm::sinf(p / 2.0));
    let (cy, sy) = (libm::cosf(y / 2.0), libm::sinf(y / 2.0));
    let qx = sr * cp * cy - cr * sp * sy;
    let qy = cr * sp * cy + sr * cp * sy;
    let qz = cr * cp * sy - sr * sp * cy;
    let qw = cr * cp * cy + sr * sp * sy;
    (qw, qx, qy, qz)
}

#[inline]
fn mat2quat(m: &Mat3) -> (f32, f32, f32, f32) {
    let w = libm::sqrtf(libm::fmaxf(0.0, 1.0 + m[0][0] + m[1][1] + m[2][2])) / 2.0;
    let mut x = libm::sqrtf(libm::fmaxf(0.0, 1.0 + m[0][0] - m[1][1] - m[2][2])) / 2.0;
    let mut y = libm::sqrtf(libm::fmaxf(0.0, 1.0 - m[0][0] + m[1][1] - m[2][2])) / 2.0;
    let mut z = libm::sqrtf(libm::fmaxf(0.0, 1.0 - m[0][0] - m[1][1] + m[2][2])) / 2.0;
    if m[2][1] - m[1][2] < 0.0 {
        x = -x;
    }
    if m[0][2] - m[2][0] < 0.0 {
        y = -y;
    }
    if m[1][0] - m[0][1] < 0.0 {
        z = -z;
    }
    (w, x, y, z)
}

#[inline]
fn mcrossmat(v: Vec3) -> Mat3 {
    [
        [0.0, -v.z, v.y],
        [v.z, 0.0, -v.x],
        [-v.y, v.x, 0.0],
    ]
}

fn reset_integrators(s: &mut State) {
    s.i_error_pos = Vec3::zero();
    s.i_error_att = Vec3::zero();
}

fn motor_thrust_from_rpm(rpm: u16) -> f32 {
    let w = rpm as f32 * 2.0 * PI / 60.0;
    MOTORRPM2FORCE * w * w
}

unsafe fn step_inner(s: &mut State, control: &mut control_s, sp: &setpoint_s, sensors: &sensorData_s, st: &state_s) {
    let dt = DT;
    let mode_abs = crate::bindings::mode_e_modeAbs;
    let mode_vel = crate::bindings::mode_e_modeVelocity;
    let mode_dis = crate::bindings::mode_e_modeDisable;
    let cm_ft = crate::bindings::control_mode_e_controlModeForceTorque;

    let mut desired_yaw = 0.0_f32;
    if sp.mode.yaw == mode_vel {
        desired_yaw = (st.attitude.yaw + sp.attitudeRate.yaw * dt) * DEG2RAD;
    } else if sp.mode.yaw == mode_abs {
        desired_yaw = sp.attitude.yaw * DEG2RAD;
    } else if sp.mode.quat == mode_abs {
        let rpy = quat2rpy(
            st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q3,
            st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q0,
            st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q1,
            st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q2,
        );
        desired_yaw = rpy.z;
    }

    s.indi = unsafe { g_oot5_indi };
    let rpm_ok = unsafe { oot_rpm_logs_available() };
    let mut t1 = 0.0_f32;
    let mut t2 = 0.0;
    let mut t3 = 0.0;
    let mut t4 = 0.0;
    if s.indi != 0 && rpm_ok {
        let mut m1 = 0u16;
        let mut m2 = 0u16;
        let mut m3 = 0u16;
        let mut m4 = 0u16;
        unsafe { rpm_get_all(&mut m1, &mut m2, &mut m3, &mut m4) };
        t1 = motor_thrust_from_rpm(m1);
        t2 = motor_thrust_from_rpm(m2);
        t3 = motor_thrust_from_rpm(m3);
        t4 = motor_thrust_from_rpm(m4);
    }

    let xc = Vec3::new(libm::cosf(desired_yaw), libm::sinf(desired_yaw), 0.0);
    let yc = Vec3::new(-libm::sinf(desired_yaw), libm::cosf(desired_yaw), 0.0);

    let mut thrust_si = 0.0_f32;

    if sp.mode.x == mode_abs || sp.mode.y == mode_abs || sp.mode.z == mode_abs {
        let pos_d = Vec3::new(sp.position.x, sp.position.y, sp.position.z);
        let vel_d = Vec3::new(sp.velocity.x, sp.velocity.y, sp.velocity.z);
        let acc_d = Vec3::new(
            sp.acceleration.x,
            sp.acceleration.y,
            sp.acceleration.z + GRAVITY,
        );
        let state_pos = Vec3::new(st.position.x, st.position.y, st.position.z);
        let state_vel = Vec3::new(st.velocity.x, st.velocity.y, st.velocity.z);

        let pos_e = vclampscl(pos_d.sub(state_pos), -KPOS_P_LIMIT, KPOS_P_LIMIT);
        let vel_e = vclampscl(vel_d.sub(state_vel), -KPOS_D_LIMIT, KPOS_D_LIMIT);
        s.i_error_pos = s.i_error_pos.add(pos_e.scale(dt));

        let a_d = acc_d
            .add(veltmul(KPOS_D, vel_e))
            .add(veltmul(KPOS_P, pos_e))
            .add(veltmul(KPOS_I, s.i_error_pos));

        let qw = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q3;
        let qx = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q0;
        let qy = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q1;
        let qz = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q2;
        let r = quat_to_rot(qw, qx, qy, qz);
        let z = Vec3::new(0.0, 0.0, 1.0);

        let mut a_indi = Vec3::zero();
        if (s.indi & 1) != 0 && rpm_ok {
            let f_rpm = t1 + t2 + t3 + t4;
            let a_rpm = mat_mul_vec(&r, z).scale(f_rpm / MASS).sub(Vec3::new(0.0, 0.0, GRAVITY));
            let _ = s.filter_acc_rpm.update(a_rpm);
            let a_imu = Vec3::new(st.acc.x, st.acc.y, st.acc.z).scale(GRAVITY);
            let _ = s.filter_acc_imu.update(a_imu);
            let a_rpm_f = s.filter_acc_rpm.get();
            let a_imu_f = s.filter_acc_imu.get();
            a_indi = a_rpm_f.sub(a_imu_f);
        }
        unsafe { indi_a_res_write(a_indi.x, a_indi.y, a_indi.z) };

        thrust_si = MASS * a_d.add(a_indi).dot(mat_mul_vec(&r, z));
        if thrust_si < 0.01 {
            reset_integrators(s);
        }

        let f_d = a_d.add(a_indi);
        let xb = yc.cross(f_d).normalize();
        let yb = f_d.cross(xb).normalize();
        let zb = xb.cross(yb);
        s.r_des = mcolumns(xb, yb, zb);
    } else {
        if sp.mode.z == mode_dis && sp.thrust < 1000.0 {
            control.controlMode = cm_ft;
            let union_ptr = (&mut control.__bindgen_anon_1) as *mut _ as *mut f32;
            unsafe {
                *union_ptr.add(0) = 0.0;
                *union_ptr.add(1) = 0.0;
                *union_ptr.add(2) = 0.0;
                *union_ptr.add(3) = 0.0;
            }
            reset_integrators(s);
            return;
        }
        let max_thrust = unsafe { powerDistributionGetMaxThrust() };
        thrust_si = sp.thrust as f32 / 65535.0 * max_thrust;
        let rpy = Vec3::new(
            sp.attitude.roll * DEG2RAD,
            -sp.attitude.pitch * DEG2RAD,
            desired_yaw,
        );
        let (qw, qx, qy, qz) = rpy2quat(rpy);
        s.r_des = quat_to_rot(qw, qx, qy, qz);
    }

    let qw = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q3;
    let qx = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q0;
    let qy = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q1;
    let qz = st.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q2;
    let r = quat_to_rot(qw, qx, qy, qz);

    let e_rm = matsub(&mat_at_b(&s.r_des, &r), &mat_at_b(&r, &s.r_des));
    let e_r = vee_half(&e_rm);
    unsafe { indi_e_r_write(e_r.x, e_r.y, e_r.z, e_r.norm()) };

    let g = &sensors.gyro;
    let omega = Vec3::new(g.axis[0] * DEG2RAD, g.axis[1] * DEG2RAD, g.axis[2] * DEG2RAD);

    let xb = mcolumn(&s.r_des, 0);
    let yb = mcolumn(&s.r_des, 1);
    let zb = mcolumn(&s.r_des, 2);
    let des_jerk = Vec3::new(sp.jerk.x, sp.jerk.y, sp.jerk.z);
    let c = thrust_si / MASS;
    let b1 = c;
    let b3 = -yc.dot(zb);
    let c3 = yc.cross(zb).norm();
    let d1 = xb.dot(des_jerk);
    let d2 = -yb.dot(des_jerk);
    let d3 = sp.attitudeRate.yaw * DEG2RAD * xc.dot(xb);

    let mut omega_des = Vec3::zero();
    if thrust_si != 0.0 {
        omega_des.x = d2 / b1;
        omega_des.y = d1 / b1;
        omega_des.z = (b1 * d3 - b3 * d1) / (b1 * c3);
    }

    let yaw_ddot = sp.attitudeAcc.yaw * DEG2RAD;
    let yaw_dot = sp.attitudeRate.yaw * DEG2RAD;
    let des_snap = Vec3::new(sp.snap.x, sp.snap.y, sp.snap.z);
    let c_dot = zb.dot(des_jerk);
    let e1 = xb.dot(des_snap) - 2.0 * c_dot * omega_des.y - c * omega_des.x * omega_des.z;
    let e2 = -yb.dot(des_snap) - 2.0 * c_dot * omega_des.x + c * omega_des.y * omega_des.z;
    let e3 = yaw_ddot * xc.dot(xb)
        + 2.0 * yaw_dot * omega_des.z * xc.dot(yb)
        - 2.0 * yaw_dot * omega_des.y * xc.dot(zb)
        - omega_des.x * omega_des.y * yc.dot(yb)
        - omega_des.x * omega_des.z * yc.dot(zb);

    let mut omega_des_dot = Vec3::zero();
    if thrust_si != 0.0 {
        omega_des_dot.x = e2 / b1;
        omega_des_dot.y = e1 / b1;
        omega_des_dot.z = (b1 * e3 - b3 * e1) / (b1 * c3);
    }

    let r_des_t = mat_at_b(&r, &s.r_des);
    let omega_r = mat_mul_vec(&r_des_t, omega_des);
    let omega_error = omega.sub(omega_r);
    s.i_error_att = s.i_error_att.add(e_r.scale(dt));

    let j_omega = veltmul(J, omega);
    let w_x = mcrossmat(omega);
    let cross_term = mat_mul_vec(&w_x, omega_r);
    let j_term = mat_mul_vec(&r_des_t, omega_des_dot);
    let gyro_torque = omega.cross(j_omega);

    let term_j = cross_term.sub(j_term);
    let mut u = veltmul(KR, e_r).scale(-1.0)
        .add(veltmul(KOMEGA, omega_error).scale(-1.0))
        .add(veltmul(KI_ATT, s.i_error_att).scale(-1.0))
        .add(gyro_torque)
        .sub(veltmul(J, term_j));

    if (s.indi & 2) != 0 && rpm_ok {
        let tau_rpm = Vec3::new(
            -ARM * t1 - ARM * t2 + ARM * t3 + ARM * t4,
            -ARM * t1 + ARM * t2 + ARM * t3 - ARM * t4,
            -T2T * t1 + T2T * t2 - T2T * t3 + T2T * t4,
        );
        let _ = s.filter_tau_rpm.update(tau_rpm);
        let tau_rpm_f = s.filter_tau_rpm.get();

        let timestamp = unsafe { usecTimestamp() };
        let dt_meas = if s.timestamp_prev == 0 {
            dt
        } else {
            (timestamp - s.timestamp_prev) as f32 / 1.0e6
        };
        let angular_acc = omega.sub(s.omega_prev).scale(1.0 / dt_meas);
        let _ = s.filter_angular_acc.update(angular_acc);
        let angular_acc_f = s.filter_angular_acc.get();
        let tau_gyro_f = veltmul(J, angular_acc_f);
        s.omega_prev = omega;
        s.timestamp_prev = timestamp;

        let indi_moments = tau_rpm_f.sub(tau_gyro_f);
        u = u.add(indi_moments);
    }
    unsafe { indi_tau_write(u.x, u.y, u.z) };

    control.controlMode = cm_ft;
    let union_ptr = (&mut control.__bindgen_anon_1) as *mut _ as *mut f32;
    unsafe {
        *union_ptr.add(0) = thrust_si;
        *union_ptr.add(1) = u.x;
        *union_ptr.add(2) = u.y;
        *union_ptr.add(3) = u.z;
    }
}

#[no_mangle]
pub extern "C" fn oot5_state_ptr() -> *mut u8 {
    unsafe { core::ptr::addr_of_mut!(ST) as *mut u8 }
}

#[no_mangle]
pub extern "C" fn oot5_state_size() -> usize {
    core::mem::size_of::<State>()
}

#[no_mangle]
pub extern "C" fn omar_indi_rust_set_indi(mode: u8) {
    unsafe {
        ST.indi = mode;
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTree5Init() {
    unsafe {
        // ST.indi is not reset here -- it is synced from g_oot5_indi (the yaml/cfclient
        // param) every tick in step_inner, before first use, same tick as this Init call.
        ST.i_error_pos = Vec3::zero();
        ST.i_error_att = Vec3::zero();
        ST.omega_prev = Vec3::zero();
        ST.timestamp_prev = 0;
        ST.filter_acc_rpm.init_all(FILTER_CUTOFF_HZ, DT);
        ST.filter_acc_imu.init_all(FILTER_CUTOFF_HZ, DT);
        ST.filter_tau_rpm.init_all(FILTER_CUTOFF_HZ, DT);
        ST.filter_angular_acc.init_all(FILTER_CUTOFF_HZ, DT);
        ST.initialized = true;
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTree5Test() -> bool {
    true
}

#[no_mangle]
pub unsafe extern "C" fn controllerOutOfTree5(
    control: *mut control_s,
    setpoint: *const setpoint_s,
    sensors: *const sensorData_s,
    state: *const state_s,
    tick: u32,
) {
    if tick % 2 != 0 {
        return;
    }
    // Check + Init BEFORE taking any &mut into ST -- calling Init() while a live &mut
    // reference into the same static is already held (the previous version of this
    // function did exactly that) is undefined behaviour under Rust's aliasing model.
    // The host numerical test never caught this because it always calls Init() once,
    // manually, before its first dispatcher call -- so this lazy-init branch, which is
    // the ONLY path real hardware ever takes (every flight starts uninitialized), had
    // never actually been exercised by any test. Found 2026-09-29 after a real flight
    // produced zero thrust/torque the entire hover -- see docs/41 §13.
    let already_init = core::ptr::addr_of!((*core::ptr::addr_of!(ST)).initialized).read();
    if !already_init {
        controllerOutOfTree5Init();
    }
    let s = &mut *core::ptr::addr_of_mut!(ST);
    step_inner(s, &mut *control, &*setpoint, &*sensors, &*state);
}
