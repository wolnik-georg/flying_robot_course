//! controller=8 (`ControllerTypeOot3`, planned) — a faithful Rust port of Cobo & Briesewitz's
//! NA-INDI controller **with `use_nn` enabled**, `~/Desktop/NA-INDI-firmware/src/modules/src/
//! controller/controller_lee.c` (MIT, Copyright (c) 2024 Khaled Wahba).
//!
//! **Status, 2026-09-16: wired into a firmware controller slot (`ControllerTypeOot3`) AND
//! numerically verified against the reference's own compiled C — 6/6 hand-picked test vectors
//! match to ~1e-9 on thrust and torque, see `host/test_naindi_hybrid_reference.py` and
//! `host/naindi_reference_build_notes.md`'s "NA-INDI hybrid" addendum. STILL NEVER FLOWN** —
//! numerical verification is a precondition for flying, not a substitute for it; do not fly
//! without clearing the same hardware-validation gate every other controller change here does.
//! One real bug was caught along the way, in the test harness rather than the port: the
//! reference's `use_nn` path calls `usecTimestamp()` three times per tick (two of them
//! unused profiling calls bracketing the NN forward pass), which inflated the scratch build's
//! fake clock 3x relative to this port's single call and showed up as torque-only mismatches
//! on the first attempt — confirmed NOT a port bug by comparing `nn_forward()` directly
//! against the reference's own compiled version bit-for-bit on identical input first.
//!
//! ## Why this is a SEPARATE module from `naindi.rs` (controller=7), not an extension of it
//!
//! Per operator instruction (2026-09-16): kept fully isolated -- own file, own state, own
//! controller slot, no shared statics with `naindi.rs`. This is the SAME control law as
//! `controller=7` (`naindi.rs`) with one addition -- `use_nn` is enabled and its NN evaluation
//! wired in -- but the two must stay independently buildable, testable and flyable, so a change
//! to one can never silently affect the other. Small, faithful duplication (this file's INDI
//! logic mirrors `naindi.rs` line-for-line where they overlap) rather than a shared abstraction,
//! matching how `controller=7` itself was kept isolated from `lib.rs`'s own geometric+INDI.
//!
//! ## What "use_nn" actually is, and why it is a DIFFERENT problem from Strategy 2
//!
//! `naindi.rs`'s own module doc (controller=7) says "every gain block we found upstream ships
//! with `use_nn=0`; the block is dead code in every flown NA-INDI config" and leaves it out
//! entirely. That characterisation undersold what exists: `~/Desktop/NA-INDI-firmware/.../nn.c`
//! ships a REAL, TRAINED network (19 -> 24 -> 24 -> 24 -> 6, real weights, real min-max
//! normalisation ranges -- `X_maxs[15..19]` run to ~65500, exactly `motorsGetRatio()`'s 16-bit
//! range, confirming these are genuinely calibrated, not placeholders). See
//! `naindi_hybrid_weights.rs` for the full extraction record.
//!
//! **But its input is 19-dim OWN-STATE ONLY** -- rotation matrix columns, EKF acceleration,
//! velocity, gyro, motor PWM ratios -- **no relative/neighbour state at all**. It was trained on
//! their own single-vehicle payload-swing experiment (their paper's case), not on inter-vehicle
//! downwash. Porting it faithfully with their real weights (this file) makes `controller=8` a
//! genuine, flyable "NA-INDI exactly as published" -- a legitimate reference/citation-check --
//! but it does **NOT by itself compensate the interaction force this thesis compares** unless
//! separately retrained on OUR C.1 data using this same architecture and input convention. Our
//! uSD logging already covers what that retraining would need (`motor.m1-4` + `gyro.*` already
//! logged, same as `a_res` for the label) -- a real, separate scoping decision, not attempted
//! here. See `docs/strategy_controller_map.html` for where this sits relative to Strategy 4.
//!
//! ## The exact algorithm, traced from `controller_lee.c` and `nn_utils.c` line-for-line
//!
//! Input vector (19), built once per tick, exactly `controller_lee.c` lines ~274-293:
//!   `[0..6)`  first two columns of R: `R[0][0], R[0][1], R[1][0], R[1][1], R[2][0], R[2][1]`
//!   `[6..9)`  `state->acc * 9.81`, EXCEPT z which is `(state->acc.z + 1.0) * 9.81` -- their
//!             own `state->acc.z` convention already has gravity removed (reads ~0 at hover),
//!             so +1 restores the ~1G baseline before scaling. x/y get no such offset.
//!   `[9..12)` `state->velocity.{x,y,z}` directly, m/s.
//!   `[12..15)` `radians(sensors->gyro.{x,y,z})` -- the REGULAR, LOW-PASS-FILTERED gyro, **not**
//!             `gyroNoLpf`. This is a real, easy-to-miss distinction: the surrounding INDI code
//!             in `controller_lee.c` (and this project's `naindi.rs`) uses `gyroNoLpf` for
//!             `self->omega`, but the NN's own input vector explicitly reads plain `sensors->
//!             gyro` (confirmed at `controller_lee.c` line 288 vs. line 395). Ported as read.
//!   `[15..19)` `motorsGetRatio(0..3)` -- commanded PWM ratio, u16, NOT measured RPM. New FFI
//!             binding here (`motorsGetRatio`); our own `crazyflie-firmware` already has this
//!             exact function (`src/drivers/src/motors.c`), no patch needed.
//!
//! `nn_forward` (`nn_utils.c`'s `layer()` + `nn.c`'s min-max normalisation, exactly):
//!   1. Per-input min-max scale to [-1, 1]: `((x - X_MINS[i]) / (X_MAXS[i] - X_MINS[i]) - 0.5) * 2`
//!   2. Four linear layers, 19->24->24->24->6, **LeakyReLU (slope 0.01)** on the first three,
//!      linear on the last. Weight layout is `[n_in][n_out]` (`nn_utils.c`'s `layer_weight[jj][ii]`,
//!      `jj`=input index) -- the TRANSPOSE of PyTorch's `nn.Linear`, kept as their C stores it.
//!   3. Per-output min-max UNSCALE from [-1, 1]: `(y+1)*0.5*(Y_MAXS[i]-Y_MINS[i]) + Y_MINS[i]`.
//!   Output: `[f_x, f_y, f_z, tau_x, tau_y, tau_z]` in the reference's own units (their `a_nn`/
//!   `u_nn` are consumed as accelerations/angular-accelerations directly, not forces -- ported
//!   as their code uses them, no unit conversion inserted).
//!
//! Integration into the control law (`controller_lee.c` lines ~296-343 and ~445-497), literally:
//!   - `a_nn.x, a_nn.y = nn_output[0,1]` iff `use_nn & 1`; `a_nn.z = nn_output[2]` iff `use_nn & 2`.
//!   - `a_rpm = clamp(f_rpm/mass * R*z_body - g + a_nn, 10)` -- a_nn is folded in BEFORE the
//!     position-INDI filter pair, so `a_indi = a_imu_filtered - a_rpm_filtered` ends up seeing
//!     only what the NN did not already explain.
//!   - `F_d = a_d - a_indi - a_nn` -- a_nn is ALSO subtracted directly here. (Not a
//!     double-subtraction bug: the filtering above means `a_indi` carries a lagged, partial
//!     image of `-a_nn`, not the full raw value -- ported exactly as written, not "simplified".)
//!   - `u_nn.xyz = nn_output[3,4,5]` iff `use_nn & 4`; folded into `tau_rpm` and `u` the same way,
//!     symmetric with the force side (`controller_lee.c` lines 451-497).
//!
//! `USE_NN` here is a compile-time constant (`0b111` = all three: lateral force, vertical force,
//! torque -- "full NA-INDI"), matching this project's own existing pattern for `INDI_MODE` in
//! `naindi.rs` (also a compile-time constant there, not yet a runtime CRTP param). This is an
//! ASSUMPTION, not a recovered fact: neither `~/Desktop/NA-INDI` nor `~/Desktop/NA-INDI-firmware`
//! contains a specific flown `use_nn` value in this snapshot (`docs/22`'s own open question #5 --
//! the results notebook reads a `data/` directory that isn't present either). Wiring `use_nn` as
//! a runtime-settable param (like `naindi.rs`'s `INDI_MODE` would also need to become, separately)
//! is future work, not blocking a first numerical-verification pass with this default.

use crate::bindings::{control_s, setpoint_s, sensorData_s, state_s};
use crate::{quat_to_rot, mat_at_b, matsub, vee_half, mat_mul_vec, clamp_norm, Vec3, Mat3, GRAVITY};
use crate::{JXX, JYY, JZZ, g_indi_mass, g_indi_kt1, g_indi_kt2, g_indi_kt3, g_indi_kt4, g_indi_frame_conv};

extern "C" {
    fn rpm_get_all(m1: *mut u16, m2: *mut u16, m3: *mut u16, m4: *mut u16);
    fn usecTimestamp() -> u64;
    // New binding: our own crazyflie-firmware already exports this (src/drivers/src/motors.c);
    // no patch needed, unlike gyroNoLpf which naindi.rs's controller=7 DID need to add.
    fn motorsGetRatio(id: u32) -> u16;
}

// ── Reference `controller_lee.c` default gains (g_self), unchanged -- identical to naindi.rs ──
const KPOS_P: Vec3 = Vec3 { x: 12.0, y: 12.0, z: 12.0 };
const KPOS_P_LIMIT: f32 = 100.0;
const KPOS_D: Vec3 = Vec3 { x: 10.5, y: 10.5, z: 10.5 };
const KPOS_D_LIMIT: f32 = 100.0;
const KPOS_I: Vec3 = Vec3 { x: 2.0, y: 2.0, z: 2.0 };
const KPOS_I_LIMIT: f32 = 100.0;
const KR: Vec3 = Vec3 { x: 0.007, y: 0.007, z: 0.01 };
const KOMEGA: Vec3 = Vec3 { x: 0.002, y: 0.002, z: 0.002 };
const KI_ATT: Vec3 = Vec3 { x: 0.01, y: 0.01, z: 0.01 };

const INDI_MODE: u8 = 3;   // force + moment INDI both on, same as naindi.rs
// See module doc: an assumption, not a recovered fact. Bit 1 = lateral force, bit 2 = vertical
// force, bit 4 = torque -- `self->use_nn & N` in controller_lee.c.
const USE_NN: u8 = 0b111;

const ATTITUDE_RATE: f32 = 500.0;
const DT_FIXED: f32 = 1.0 / ATTITUDE_RATE;

const CUTOFF_ACC: f32 = 80.0;
const CUTOFF_TAU: f32 = 40.0;
const CUTOFF_Z: f32 = 10.0;

// 2026-09-16 CORRECTED: was hardcoded to the reference's own stock-CF2.1 literals
// unconditionally -- see naindi.rs's own (much longer) comment on the identical fix there
// for the full story. Short version: unlike mass/J/kt (already airframe-aware via
// g_indi_mass/JXX,JYY,JZZ/g_indi_kt1-4), a hardcoded arm/t2t made `tau_rpm` model a
// DIFFERENT vehicle's motors than the one actually simulated/flown. Confirmed empirically
// (2026-09-16 CS2 sim, single-drone hover, controller=8): a growing attitude oscillation to
// a full tumble by ~t=14s, absent under controller=6 on the identical scenario, root-caused
// to this same mismatch shared with naindi.rs (this file mirrors naindi.rs's INDI logic
// line-for-line where they overlap, including this bug). Now airframe-aware with the same
// #[cfg(drone_bl)] switch lib.rs uses, plus its own test-only override (kept separate from
// naindi.rs's, per this module's isolation requirement) so test_naindi_hybrid_reference.py
// can still pin the reference's own literals for the numerical-verification comparison.
#[cfg(not(drone_bl))]
const ARM_REF_DEFAULT: f32 = 0.032_526_9_f32;   // sqrt(2)/2 * 0.046 -- standard/upgraded CF2.1
#[cfg(not(drone_bl))]
const T2T_REF_DEFAULT: f32 = 0.005_964_552_f32;
#[cfg(drone_bl)]
const ARM_REF_DEFAULT: f32 = 0.035_355_3_f32;   // sqrt(2)/2 * 0.050 -- CF21BL brushless
#[cfg(drone_bl)]
const T2T_REF_DEFAULT: f32 = 0.005_692_788_4_f32;

static mut ARM_T2T_TEST_OVERRIDE: Option<(f32, f32)> = None;

/// Test-only override, mirroring naindi_hybrid_test_set_j exactly (see there for rationale).
#[no_mangle]
pub extern "C" fn naindi_hybrid_test_set_arm(arm: f32, t2t: f32) {
    unsafe { ARM_T2T_TEST_OVERRIDE = Some((arm, t2t)); }
}

/// Second-order Butterworth low-pass -- identical to `naindi.rs`'s, duplicated rather than
/// shared per this module's isolation requirement (see module doc).
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
    omega_prev: Vec3,
    timestamp_prev: u64,
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

// Test-only inertia override for host-level numerical-vector validation against the
// reference's own compiled C (see host/test_naindi_hybrid_reference.py), mirroring
// naindi.rs's own naindi_test_set_j() exactly (same rationale: J is a compile-time
// per-platform constant in real flight; this hook only exists so a test can pin both
// sides to the SAME inertia). Never called outside that test; defaults to the real
// per-platform JXX/JYY/JZZ untouched. Kept as this file's own static, not shared with
// naindi.rs's J_TEST_OVERRIDE, per this module's isolation requirement.
static mut J_TEST_OVERRIDE: Option<Vec3> = None;

#[no_mangle]
pub extern "C" fn naindi_hybrid_test_set_j(x: f32, y: f32, z: f32) {
    unsafe { J_TEST_OVERRIDE = Some(Vec3::new(x, y, z)); }
}

// 2026-09-17: sim-only KR/KOMEGA override, mirroring naindi.rs's own naindi_test_set_gains()
// exactly -- same shared root-cause hypothesis (docs/07 History (39)/(40)): reference torque
// gains tuned for their J, applied unchanged to this project's real, heavier CF21BL J, lower
// both omega_n and zeta at once. Kept as this file's own static, not shared with naindi.rs's,
// per this module's isolation requirement. Never used unless a sim harness calls the setter.
static mut GAIN_TEST_OVERRIDE: Option<(Vec3, Vec3)> = None;

#[no_mangle]
pub extern "C" fn naindi_hybrid_test_set_gains(kr_x: f32, kr_y: f32, kr_z: f32,
                                                komega_x: f32, komega_y: f32, komega_z: f32) {
    unsafe {
        GAIN_TEST_OVERRIDE = Some((Vec3::new(kr_x, kr_y, kr_z), Vec3::new(komega_x, komega_y, komega_z)));
    }
}

fn vclampscl(v: Vec3, limit: f32) -> Vec3 {
    Vec3::new(v.x.clamp(-limit, limit), v.y.clamp(-limit, limit), v.z.clamp(-limit, limit))
}

// ── The NN itself: nn_utils.c's layer() + nn.c's min-max normalisation, exactly ──────────────

/// `nn_utils.c`'s `layer()`: `out[o] = activation(sum_i in[i] * w[i*cols + o] + b[o])`.
/// `w` is `[n_in][n_out]` row-major, i.e. the transpose of `nn.Linear` -- see module doc.
fn nn_layer(input: &[f32], w: &[f32], b: &[f32], out: &mut [f32], leaky: bool) {
    let cols = out.len();
    for (o, out_o) in out.iter_mut().enumerate() {
        let mut acc = 0.0_f32;
        for (i, &x) in input.iter().enumerate() {
            acc += x * w[i * cols + o];
        }
        acc += b[o];
        *out_o = if leaky { if acc > 0.0 { acc } else { acc * 0.01 } } else { acc };
    }
}

/// `nn_forward()` in `nn.c`, ported exactly: min-max in -> 4 layers -> min-max out.
/// Returns `[f_x, f_y, f_z, tau_x, tau_y, tau_z]` in the reference's own units.
fn nn_forward(input: &[f32; 19]) -> [f32; 6] {
    use crate::naindi_hybrid_weights::{W0, B0, W1, B1, W2, B2, W3, B3, X_MINS, X_MAXS, Y_MINS, Y_MAXS};

    let mut scaled = [0.0_f32; 19];
    for i in 0..19 {
        scaled[i] = ((input[i] - X_MINS[i]) / (X_MAXS[i] - X_MINS[i]) - 0.5) * 2.0;
    }

    let mut h0 = [0.0_f32; 24];
    nn_layer(&scaled, &W0, &B0, &mut h0, true);
    let mut h1 = [0.0_f32; 24];
    nn_layer(&h0, &W1, &B1, &mut h1, true);
    let mut h2 = [0.0_f32; 24];
    nn_layer(&h1, &W2, &B2, &mut h2, true);
    let mut raw = [0.0_f32; 6];
    nn_layer(&h2, &W3, &B3, &mut raw, false);

    let mut y = [0.0_f32; 6];
    for i in 0..6 {
        y[i] = (raw[i] + 1.0) * 0.5 * (Y_MAXS[i] - Y_MINS[i]) + Y_MINS[i];
    }
    y
}

/// Test-only hook exposing `nn_forward` directly, for host-level numerical verification without
/// going through the whole control law. NOT a substitute for comparing against the reference's
/// own compiled `nn_forward()` -- see module doc's status note -- but lets this port's own
/// arithmetic (weight indexing, activation, normalisation direction) be checked against an
/// independent re-implementation of the same extracted numbers before that harness exists.
#[no_mangle]
pub extern "C" fn naindi_hybrid_test_nn_forward(input: *const f32, out: *mut f32) {
    unsafe {
        let inp: &[f32; 19] = &*(input as *const [f32; 19]);
        let y = nn_forward(inp);
        for i in 0..6 {
            *out.add(i) = y[i];
        }
    }
}

#[no_mangle]
pub extern "C" fn controllerOutOfTree3Init() {
    unsafe {
        let s = &mut *core::ptr::addr_of_mut!(ST);
        *s = State::zero();
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
pub extern "C" fn controllerOutOfTree3Test() -> bool {
    true
}

#[no_mangle]
pub unsafe extern "C" fn controllerOutOfTree3(
    control: *mut control_s,
    setpoint: *const setpoint_s,
    sensors: *const sensorData_s,
    state: *const state_s,
    tick: u32,
) {
    if tick % 2 != 0 {
        return;
    }

    let s = &mut *core::ptr::addr_of_mut!(ST);
    if !s.initialized {
        controllerOutOfTree3Init();
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
    let g = &(*sensors).gyroNoLpf;
    let omega = Vec3::new(g.axis[0] * deg2rad, g.axis[1] * deg2rad, g.axis[2] * deg2rad);
    let acc = &st.acc;

    let sp = &*setpoint;
    let pd = Vec3::new(sp.position.x, sp.position.y, sp.position.z);
    let vd = Vec3::new(sp.velocity.x, sp.velocity.y, sp.velocity.z);
    let acc_d = Vec3::new(sp.acceleration.x, sp.acceleration.y, sp.acceleration.z);
    let acc_d_g = Vec3::new(acc_d.x, acc_d.y, acc_d.z + GRAVITY);
    let jerk_d = Vec3::new(sp.jerk.x, sp.jerk.y, sp.jerk.z);
    let snap_d = Vec3::new(sp.snap.x, sp.snap.y, sp.snap.z);
    let yaw_d = sp.attitude.yaw * deg2rad;
    let yaw_dot_d = sp.attitudeRate.yaw * deg2rad;

    let mass = g_indi_mass;

    // -- Position controller ------------------------------------------------------------------
    let pos_e = vclampscl(pd.sub(pos), KPOS_P_LIMIT);
    let vel_e = vclampscl(vd.sub(vel), KPOS_D_LIMIT);
    s.i_error_pos = s.i_error_pos.add(pos_e.scale(dt));
    s.i_error_pos = vclampscl(s.i_error_pos, KPOS_I_LIMIT);
    let a_d = acc_d_g
        .add(Vec3::new(KPOS_D.x * vel_e.x, KPOS_D.y * vel_e.y, KPOS_D.z * vel_e.z))
        .add(Vec3::new(KPOS_P.x * pos_e.x, KPOS_P.y * pos_e.y, KPOS_P.z * pos_e.z))
        .add(Vec3::new(KPOS_I.x * s.i_error_pos.x, KPOS_I.y * s.i_error_pos.y, KPOS_I.z * s.i_error_pos.z));

    // -- RPM -> per-motor thrust (this project's own kt_i / rpm_get_all(), as naindi.rs) -------
    let mut m1 = 0u16; let mut m2 = 0u16; let mut m3 = 0u16; let mut m4 = 0u16;
    rpm_get_all(&mut m1, &mut m2, &mut m3, &mut m4);
    let t1 = g_indi_kt1 * (m1 as f32) * (m1 as f32);
    let t2 = g_indi_kt2 * (m2 as f32) * (m2 as f32);
    let t3 = g_indi_kt3 * (m3 as f32) * (m3 as f32);
    let t4 = g_indi_kt4 * (m4 as f32) * (m4 as f32);
    let f_rpm = t1 + t2 + t3 + t4;

    // -- The NN: input vector exactly `controller_lee.c` lines ~274-293 (see module doc) ------
    let nn_out = {
        let mut input = [0.0_f32; 19];
        input[0] = r[0][0]; input[1] = r[0][1];
        input[2] = r[1][0]; input[3] = r[1][1];
        input[4] = r[2][0]; input[5] = r[2][1];
        input[6] = acc.x * 9.81;
        input[7] = acc.y * 9.81;
        input[8] = (acc.z + 1.0) * 9.81;
        input[9] = vel.x; input[10] = vel.y; input[11] = vel.z;
        // The REGULAR (LPF'd) gyro, not gyroNoLpf -- see module doc, this is deliberate and
        // matches the reference exactly, even though the rest of this file uses gyroNoLpf.
        let gy = &(*sensors).gyro;
        input[12] = gy.axis[0] * deg2rad;
        input[13] = gy.axis[1] * deg2rad;
        input[14] = gy.axis[2] * deg2rad;
        input[15] = motorsGetRatio(0) as f32;
        input[16] = motorsGetRatio(1) as f32;
        input[17] = motorsGetRatio(2) as f32;
        input[18] = motorsGetRatio(3) as f32;
        nn_forward(&input)
    };
    let mut a_nn = Vec3::zero();
    if USE_NN & 1 != 0 { a_nn.x = nn_out[0]; a_nn.y = nn_out[1]; }
    if USE_NN & 2 != 0 { a_nn.z = nn_out[2]; }
    let mut u_nn = Vec3::zero();
    if USE_NN & 4 != 0 { u_nn = Vec3::new(nn_out[3], nn_out[4], nn_out[5]); }

    // -- Position INDI, with a_nn folded in before the filter pair (controller_lee.c ~296-334) -
    let a_indi = if INDI_MODE & 1 != 0 {
        let a_rpm = clamp_norm(
            z_body.scale(f_rpm / mass).sub(Vec3::new(0.0, 0.0, GRAVITY)).add(a_nn),
            10.0,
        );
        let a_rpm_f = s.filter_acc_rpm.update(a_rpm);
        let a_imu = clamp_norm(Vec3::new(acc.x, acc.y, acc.z).scale(GRAVITY), 10.0);
        let a_imu_f = s.filter_acc_imu.update(a_imu);
        a_imu_f.sub(a_rpm_f)
    } else {
        Vec3::zero()
    };

    // F_d = a_d - a_indi - a_nn (controller_lee.c line 342: vsub2(a_d, a_indi, a_nn)) --
    // a_nn subtracted BOTH here and folded into a_rpm above; not a double-subtraction bug, see
    // module doc.
    let f_d = a_d.sub(a_indi).sub(a_nn);
    let thrust_si = mass * f_d.dot(z_body);
    if thrust_si < 0.01 {
        s.i_error_pos = Vec3::zero();
    }

    // -- Desired rotation from F_d --------------------------------------------------------------
    let yc = Vec3::new(-libm::sinf(yaw_d), libm::cosf(yaw_d), 0.0);
    let xb = yc.cross(f_d).normalize();
    let yb = f_d.cross(xb).normalize();
    let zb = xb.cross(yb);
    let r_des: Mat3 = [
        [xb.x, yb.x, zb.x],
        [xb.y, yb.y, zb.y],
        [xb.z, yb.z, zb.z],
    ];

    let e_rm = matsub(&mat_at_b(&r_des, &r), &mat_at_b(&r, &r_des));
    let e_r = vee_half(&e_rm);

    let frame_conv = g_indi_frame_conv;
    let omega_des = crate::omega_desired(acc_d, jerk_d, yaw_d, frame_conv);
    let alpha_des = crate::alpha_desired(acc_d, jerk_d, snap_d, yaw_dot_d, frame_conv);

    let r_t_rdes = mat_at_b(&r, &r_des);
    let omega_r = mat_mul_vec(&r_t_rdes, omega_des);
    let omega_error = omega.sub(omega_r);
    s.i_error_att = s.i_error_att.add(e_r.scale(dt));

    let j = unsafe { J_TEST_OVERRIDE }.unwrap_or(Vec3::new(JXX, JYY, JZZ));
    let (kr, komega) = unsafe { GAIN_TEST_OVERRIDE }.unwrap_or((KR, KOMEGA));
    let j_omega = Vec3::new(j.x * omega.x, j.y * omega.y, j.z * omega.z);
    let gyro_term = omega.cross(j_omega);

    let cross_term = omega.cross(omega_r);
    let flat_term = mat_mul_vec(&r_t_rdes, alpha_des);
    let diff = cross_term.sub(flat_term);
    let last_term = Vec3::new(j.x * diff.x, j.y * diff.y, j.z * diff.z);
    let mut u = Vec3::new(-kr.x * e_r.x, -kr.y * e_r.y, -kr.z * e_r.z)
        .sub(Vec3::new(komega.x * omega_error.x, komega.y * omega_error.y, komega.z * omega_error.z))
        .sub(Vec3::new(KI_ATT.x * s.i_error_att.x, KI_ATT.y * s.i_error_att.y, KI_ATT.z * s.i_error_att.z))
        .add(gyro_term)
        .sub(last_term);

    // -- Attitude/torque INDI, with u_nn folded in symmetrically with the force side -----------
    // (controller_lee.c lines 451-497: tau_rpm += u_nn before filtering; u -= indi_moments; u -= u_nn)
    if INDI_MODE & 2 != 0 {
        let (arm_ref, t2t_ref) = unsafe { ARM_T2T_TEST_OVERRIDE }
            .unwrap_or((ARM_REF_DEFAULT, T2T_REF_DEFAULT));
        let tau_rpm = clamp_norm(
            Vec3::new(
                -arm_ref * t1 - arm_ref * t2 + arm_ref * t3 + arm_ref * t4,
                -arm_ref * t1 + arm_ref * t2 + arm_ref * t3 - arm_ref * t4,
                -t2t_ref * t1 + t2t_ref * t2 - t2t_ref * t3 + t2t_ref * t4,
            ).add(u_nn),
            0.006,
        );
        let tau_rpm_f = s.filter_tau_rpm.update(tau_rpm);

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
        u = u.sub(indi_moments).sub(u_nn);
    }

    let out = &mut *control;
    let union_ptr = (&mut out.__bindgen_anon_1) as *mut _ as *mut f32;
    *union_ptr.add(0) = thrust_si;
    *union_ptr.add(1) = u.x;
    *union_ptr.add(2) = u.y;
    *union_ptr.add(3) = u.z;
    out.controlMode = crate::bindings::control_mode_e_controlModeForceTorque;
}
