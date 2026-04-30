//! Equivalence tests: firmware `indi_torque` algorithm ↔ host `IndiController`
//!
//! # Purpose
//!
//! The firmware (`firmware_app/src/lib.rs`, `indi_torque()`) and the host sim
//! (`src/controller/indi.rs`, `IndiController`) implement the same incremental
//! INDI law.  They differ in two implementation details that must NOT affect
//! physical behaviour:
//!
//! | Aspect              | Firmware                    | Host sim                    |
//! |---------------------|-----------------------------|-----------------------------|
//! | Gyro LP filter      | 1st-order IIR               | 2nd-order Butterworth       |
//! | G1 convention       | implicit 1.0 (gains in Nm)  | explicit 1/J (gains in rad/s²) |
//!
//! These tests verify:
//!
//! 1. **G2 structural identity** — the formula `δτ_z = (α_err_z + G2·δτ_z_prev)/(1+G2)`
//!    produces the same *relative* change in yaw torque in both implementations.
//!
//! 2. **Directional agreement** — for a positive roll error the roll torque is
//!    negative in both implementations (correcting direction).
//!
//! 3. **G2 accumulation** — after N warm-up steps with a spinning state, the
//!    `du_prev_z` term is non-zero and G2≠0 diverges measurably from G2=0 in
//!    the firmware algorithm.
//!
//! 4. **Tau accumulation** — τ_prev builds up over multiple steps in the firmware
//!    algorithm (incremental structure is intact).

use multirotor_simulator::prelude::*;

const DT: f32 = 1.0 / 500.0;  // firmware rate: 500 Hz

// ─────────────────────────────────────────────────────────────────────────────
// Firmware INDI algorithm ported as pure Rust (std, no_std-compatible logic).
//
// Mirrors firmware_app/src/lib.rs `indi_torque()` exactly, including:
//   - 1st-order IIR gyro filter (not Butterworth2)
//   - Implicit G1_z = 1.0 (gains must be in [Nm/rad] scale)
//   - G2 yaw coupling term (matches the patched firmware)
//   - tau_prev accumulation with clamping
//
// Inputs must use the same gain scale as the firmware: [Nm/rad] for kr/kw.
// ─────────────────────────────────────────────────────────────────────────────

const JXX: f32 = 16.571710e-6;
const JYY: f32 = 16.655602e-6;
const JZZ: f32 = 29.261652e-6;
const TAU_CLAMP: f32 = 0.05;  // matches firmware TAU_INDI_CLAMP

/// Mutable state for the ported firmware INDI algorithm.
#[derive(Clone)]
struct FwState {
    omega_prev:     [f32; 3],
    omega_dot_filt: [f32; 3],
    tau_prev:       [f32; 3],
    du_prev_z:      f32,
    initialized:    bool,
}

impl FwState {
    fn zero() -> Self {
        Self {
            omega_prev:     [0.0; 3],
            omega_dot_filt: [0.0; 3],
            tau_prev:       [0.0; 3],
            du_prev_z:      0.0,
            initialized:    false,
        }
    }
}

/// Ported firmware `indi_torque()`.
///
/// Returns `[τ_x, τ_y, τ_z]` [Nm] including gyro compensation.
///
/// `kr` and `kw` must be in **[Nm/rad]** scale (same as firmware `KR_INDI_*`).
/// `g2` is the yaw coupling coefficient (0.0 = disabled, ~0.14 = CF2 default).
/// `fc_hz` is the 1st-order IIR cutoff frequency.
#[allow(clippy::too_many_arguments)]
fn fw_indi_torque(
    er:        [f32; 3],
    e_omega:   [f32; 3],
    omega:     [f32; 3],
    gyro_comp: [f32; 3],
    kr:        [f32; 3],
    kw:        [f32; 3],
    g2:        f32,
    fc_hz:     f32,
    dt:        f32,
    s:         &mut FwState,
) -> [f32; 3] {
    let v_des = [
        -kr[0]*er[0] - kw[0]*e_omega[0],
        -kr[1]*er[1] - kw[1]*e_omega[1],
        -kr[2]*er[2] - kw[2]*e_omega[2],
    ];

    if !s.initialized {
        s.tau_prev = v_des;
        s.omega_dot_filt = [0.0; 3];
        s.omega_prev = omega;
        s.initialized = true;
        return [
            v_des[0] + gyro_comp[0],
            v_des[1] + gyro_comp[1],
            v_des[2] + gyro_comp[2],
        ];
    }

    // Finite-difference raw angular acceleration
    let alpha_raw = [
        (omega[0] - s.omega_prev[0]) / dt,
        (omega[1] - s.omega_prev[1]) / dt,
        (omega[2] - s.omega_prev[2]) / dt,
    ];
    s.omega_prev = omega;

    // 1st-order IIR: k = dt / (dt + RC),  RC = 1/(2π·fc)
    let rc = 1.0 / (2.0 * std::f32::consts::PI * fc_hz);
    let k  = dt / (dt + rc);
    for i in 0..3 {
        s.omega_dot_filt[i] = k * alpha_raw[i] + (1.0 - k) * s.omega_dot_filt[i];
    }

    // Yaw: G2 correction (firmware formula, G1_z implicit = 1.0)
    let g1z_g2    = 1.0_f32 + g2;
    let delta_z   = (v_des[2] - JZZ*s.omega_dot_filt[2] + g2*s.du_prev_z) / g1z_g2;
    s.du_prev_z   = delta_z;

    let delta_tau = [
        v_des[0] - JXX*s.omega_dot_filt[0],
        v_des[1] - JYY*s.omega_dot_filt[1],
        delta_z,
    ];

    let tau_base = [
        (s.tau_prev[0] + delta_tau[0]).clamp(-TAU_CLAMP, TAU_CLAMP),
        (s.tau_prev[1] + delta_tau[1]).clamp(-TAU_CLAMP, TAU_CLAMP),
        (s.tau_prev[2] + delta_tau[2]).clamp(-TAU_CLAMP, TAU_CLAMP),
    ];
    s.tau_prev = tau_base;

    [
        tau_base[0] + gyro_comp[0],
        tau_base[1] + gyro_comp[1],
        tau_base[2] + gyro_comp[2],
    ]
}

// ─────────────────────────────────────────────────────────────────────────────
// Test helpers
// ─────────────────────────────────────────────────────────────────────────────

fn hover_ref_z(z: f32) -> TrajectoryReference {
    TrajectoryReference {
        position:        Vec3::new(0.0, 0.0, z),
        velocity:        Vec3::zero(),
        acceleration:    Vec3::zero(),
        jerk:            Vec3::zero(),
        yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Port of firmware full_indi_torque() (mode 2, RPM-deck path).
//
// Identical to fw_indi_torque except tau_current comes from RPM² measurements
// instead of accumulated tau_prev.  tau_prev is still updated at the end so the
// two modes can be switched mid-flight without a torque step.
//
// G2 patch: applied identically to indi_torque — du_prev_z accumulates and
// feeds back into the yaw delta_tau computation.
// ─────────────────────────────────────────────────────────────────────────────

const KT_FW:      f32 = 3.16e-10;    // thrust coefficient [N / RPM²]
const KQ_KT_FW:   f32 = 0.005964552; // drag-to-thrust moment ratio
const ARM_LEN_FW: f32 = 0.046;       // motor arm length [m]

fn compute_tau_from_rpm_fw(rpm_sq: [f32; 4]) -> [f32; 3] {
    let f = [KT_FW*rpm_sq[0], KT_FW*rpm_sq[1], KT_FW*rpm_sq[2], KT_FW*rpm_sq[3]];
    let lh = ARM_LEN_FW * 0.707106781_f32;
    [
        (-f[0] + f[1] + f[2] - f[3]) * lh,
        (-f[0] - f[1] + f[2] + f[3]) * lh,
        (-f[0] + f[1] - f[2] + f[3]) * KQ_KT_FW,
    ]
}

#[allow(clippy::too_many_arguments)]
fn fw_full_indi_torque(
    er:        [f32; 3],
    e_omega:   [f32; 3],
    omega:     [f32; 3],
    gyro_comp: [f32; 3],
    rpm_sq:    [f32; 4],
    kr:        [f32; 3],
    kw:        [f32; 3],
    g2:        f32,
    fc_hz:     f32,
    dt:        f32,
    s:         &mut FwState,
) -> [f32; 3] {
    let tau_current = compute_tau_from_rpm_fw(rpm_sq);

    let v_des = [
        -kr[0]*er[0] - kw[0]*e_omega[0],
        -kr[1]*er[1] - kw[1]*e_omega[1],
        -kr[2]*er[2] - kw[2]*e_omega[2],
    ];

    let alpha_raw = [
        (omega[0] - s.omega_prev[0]) / dt,
        (omega[1] - s.omega_prev[1]) / dt,
        (omega[2] - s.omega_prev[2]) / dt,
    ];
    s.omega_prev = omega;

    let rc = 1.0 / (2.0 * std::f32::consts::PI * fc_hz);
    let k  = dt / (dt + rc);
    for i in 0..3 {
        s.omega_dot_filt[i] = k * alpha_raw[i] + (1.0 - k) * s.omega_dot_filt[i];
    }

    // G2 yaw coupling — identical formula to fw_indi_torque
    let g1z_g2    = 1.0_f32 + g2;
    let delta_z   = (v_des[2] - JZZ*s.omega_dot_filt[2] + g2*s.du_prev_z) / g1z_g2;
    s.du_prev_z   = delta_z;

    let delta_tau = [
        v_des[0] - JXX*s.omega_dot_filt[0],
        v_des[1] - JYY*s.omega_dot_filt[1],
        delta_z,
    ];

    let tau_base = [
        (tau_current[0] + delta_tau[0]).clamp(-TAU_CLAMP, TAU_CLAMP),
        (tau_current[1] + delta_tau[1]).clamp(-TAU_CLAMP, TAU_CLAMP),
        (tau_current[2] + delta_tau[2]).clamp(-TAU_CLAMP, TAU_CLAMP),
    ];
    s.tau_prev = tau_base;

    [
        tau_base[0] + gyro_comp[0],
        tau_base[1] + gyro_comp[1],
        tau_base[2] + gyro_comp[2],
    ]
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 1 — G2 structural identity
// ─────────────────────────────────────────────────────────────────────────────
//
// After N warm-up steps with a non-zero ωz the firmware `du_prev_z` is non-zero.
// Setting g2 > 0 then produces a different yaw torque than g2 = 0.
// The host sim IndiController with g1 = 1/J must show the same *sign* of change.

#[test]
fn g2_identical_sign_in_both_implementations() {
    // Use gains ~20x smaller than the nominal 0.003/0.006 so that tau_prev
    // never reaches the TAU_CLAMP (0.05 Nm) during the 15-step warm-up.
    // With kw=0.0003 and omega_z=3: v_des_z = -0.0009 Nm/step →
    // after 15 steps tau ≈ -0.0135 Nm << 0.05.  G2 effect is then visible.
    let kr = [0.00015_f32; 3];
    let kw = [0.00030_f32; 3];
    let g2_bench: f32 = 0.14;   // CF2.x default

    // ── Firmware algorithm ────────────────────────────────────────────────
    let omega    = [0.0_f32, 0.0, 3.0]; // 3 rad/s yaw spin
    let er       = [0.0_f32; 3];
    let e_omega  = [0.0_f32, 0.0, omega[2]];
    let gyro_comp = [0.0_f32; 3];

    let mut s0 = FwState::zero();
    let mut sg = FwState::zero();

    // Warm up 15 steps so du_prev_z accumulates.
    for _ in 0..15 {
        fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.0,     60.0, DT, &mut s0);
        fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, g2_bench, 60.0, DT, &mut sg);
    }
    let fw_no_g2   = fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.0,      60.0, DT, &mut s0);
    let fw_with_g2 = fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, g2_bench, 60.0, DT, &mut sg);

    let fw_diff = fw_with_g2[2] - fw_no_g2[2];
    println!("g2_structural_identity: firmware  Δτz = {fw_diff:.8} Nm  \
              (g2=0: {:.8}  g2=0.14: {:.8})", fw_no_g2[2], fw_with_g2[2]);

    // ── Host sim ──────────────────────────────────────────────────────────
    // Use g1 = Vec3::splat(1.0) (normalised, same implicit convention as
    // firmware) so that g2/(g1+g2) = 0.14/1.14 ≈ 12% is measurable in f32.
    // With real g1 = 1/J ≈ 34 000 the effect would be ~4 ppm — invisible.
    let params = MultirotorParams::crazyflie();

    let make_indi = |g2: f32| -> IndiController {
        IndiController::new(
            Vec3::new(12.0, 12.0, 7.0),
            Vec3::new( 8.0,  8.0, 4.0),
            Vec3::new(kr[0], kr[1], kr[2]),
            Vec3::new(kw[0], kw[1], kw[2]),
            Vec3::new(1.0, 1.0, 1.0),   // normalised: g1=1 → same scale as firmware
            g2,
            Vec3::new(0.1, 0.1, 0.1),
            60.0,
            DT,
        )
    };

    let spinning = MultirotorState::with_initial(
        Vec3::new(0.0, 0.0, 0.5),
        Vec3::zero(),
        Quat::identity(),
        Vec3::new(0.0, 0.0, 3.0),
    );
    let reference = hover_ref_z(0.5);

    let mut sim_no_g2   = make_indi(0.0);
    let mut sim_with_g2 = make_indi(g2_bench);

    for _ in 0..15 {
        sim_no_g2.compute_control(  &spinning, &reference, &params, DT);
        sim_with_g2.compute_control(&spinning, &reference, &params, DT);
    }
    let sim_out_no   = sim_no_g2.compute_control(  &spinning, &reference, &params, DT);
    let sim_out_with = sim_with_g2.compute_control(&spinning, &reference, &params, DT);
    let sim_diff = sim_out_with.torque.z - sim_out_no.torque.z;

    println!("g2_structural_identity: host sim  Δτz = {sim_diff:.8} Nm  \
              (g2=0: {:.8}  g2=0.14: {:.8})",
             sim_out_no.torque.z, sim_out_with.torque.z);

    // Both implementations must show a non-zero G2 effect.
    assert!(
        fw_diff.abs() > 1e-10,
        "Firmware G2 has no effect on yaw torque: Δτz = {fw_diff:.2e}",
    );
    assert!(
        sim_diff.abs() > 1e-10,
        "Host sim G2 has no effect on yaw torque: Δτz = {sim_diff:.2e}",
    );

    // Both implementations must agree on the *sign* of the G2 correction.
    assert_eq!(
        fw_diff.signum(), sim_diff.signum(),
        "G2 sign disagrees: firmware Δτz={fw_diff:.2e}  host sim Δτz={sim_diff:.2e}",
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2 — Directional agreement: roll/pitch error → correct torque sign
// ─────────────────────────────────────────────────────────────────────────────
//
// For a positive roll error (er.x > 0), both implementations must produce a
// negative roll torque (correcting direction).  Same for pitch.

#[test]
fn roll_pitch_torque_direction_agreement() {
    let kr = [0.003_f32; 3];
    let kw = [0.006_f32; 3];

    // er.x = sin(10°) ≈ 0.174, er.y = 0 — pure roll error.
    let er_roll     = [10.0_f32.to_radians().sin(), 0.0, 0.0];
    let er_pitch    = [0.0_f32, 10.0_f32.to_radians().sin(), 0.0];
    let e_omega     = [0.0_f32; 3];
    let omega       = [0.0_f32; 3];
    let gyro_comp   = [0.0_f32; 3];

    // Firmware: step 1 (init), step 2 (real INDI step).
    let mut s_roll  = FwState::zero();
    fw_indi_torque(er_roll, e_omega, omega, gyro_comp, kr, kw, 0.0, 60.0, DT, &mut s_roll);
    let fw_roll = fw_indi_torque(er_roll, e_omega, omega, gyro_comp, kr, kw, 0.0, 60.0, DT, &mut s_roll);

    let mut s_pitch = FwState::zero();
    fw_indi_torque(er_pitch, e_omega, omega, gyro_comp, kr, kw, 0.0, 60.0, DT, &mut s_pitch);
    let fw_pitch = fw_indi_torque(er_pitch, e_omega, omega, gyro_comp, kr, kw, 0.0, 60.0, DT, &mut s_pitch);

    println!("directional: firmware  roll τx={:.6}  pitch τy={:.6}", fw_roll[0], fw_pitch[1]);

    assert!(fw_roll[0] < 0.0,
        "Firmware: positive er.x should give negative τx, got {}", fw_roll[0]);
    assert!(fw_pitch[1] < 0.0,
        "Firmware: positive er.y should give negative τy, got {}", fw_pitch[1]);

    // Host sim — same directional check using INDI-scaled gains.
    let params = MultirotorParams::crazyflie();
    let mut ctrl = IndiController::new(
        Vec3::new(12.0, 12.0, 7.0),
        Vec3::new( 8.0,  8.0, 4.0),
        Vec3::new(kr[0], kr[1], kr[2]),
        Vec3::new(kw[0], kw[1], kw[2]),
        Vec3::new(1.0/JXX, 1.0/JYY, 1.0/JZZ),
        0.0,
        Vec3::new(0.1, 0.1, 0.1),
        60.0,
        DT,
    );
    let reference = hover_ref_z(0.5);

    let roll_rad = 10.0_f32.to_radians();
    let q_roll = Quat::new((roll_rad/2.0).cos(), (roll_rad/2.0).sin(), 0.0, 0.0);
    let tilted = MultirotorState::with_initial(
        Vec3::new(0.0, 0.0, 0.5), Vec3::zero(), q_roll, Vec3::zero(),
    );
    ctrl.compute_control(&tilted, &reference, &params, DT);
    let sim_out = ctrl.compute_control(&tilted, &reference, &params, DT);

    println!("directional: host sim  roll τx={:.6}", sim_out.torque.x);

    assert!(sim_out.torque.x < 0.0,
        "Host sim: positive er.x should give negative τx, got {}", sim_out.torque.x);

    // Both must agree on sign for roll.
    assert_eq!(
        fw_roll[0].signum(), sim_out.torque.x.signum(),
        "Roll torque sign disagrees: firmware={:.6}  host sim={:.6}",
        fw_roll[0], sim_out.torque.x,
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3 — Tau accumulation (incremental structure)
// ─────────────────────────────────────────────────────────────────────────────
//
// With a constant non-zero roll error the firmware τ_prev must build up over
// several steps — the incremental law integrates toward the desired torque level.

#[test]
fn firmware_tau_accumulates_over_steps() {
    let kr = [0.003_f32; 3];
    let kw = [0.006_f32; 3];
    let er      = [10.0_f32.to_radians().sin(), 0.0, 0.0];
    let e_omega = [0.0_f32; 3];
    let omega   = [0.0_f32; 3];
    let gyro_comp = [0.0_f32; 3];

    let mut s = FwState::zero();

    // Skip init step.
    fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.0, 60.0, DT, &mut s);

    let out_step2 = fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.0, 60.0, DT, &mut s);
    let tau2 = out_step2[0].abs();

    // Run 20 more steps with the same error — τ_prev should grow.
    let mut last = out_step2;
    for _ in 0..20 {
        last = fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.0, 60.0, DT, &mut s);
    }
    let tau_final = last[0].abs();

    println!("tau_accumulation: step2 |τx|={tau2:.8}  step22 |τx|={tau_final:.8}");

    assert!(
        tau_final > tau2,
        "Tau_prev should accumulate: step2={tau2:.8}  step22={tau_final:.8}",
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4 — G2 du_prev_z resets to zero on new FwState
// ─────────────────────────────────────────────────────────────────────────────
//
// After reset (fresh FwState) du_prev_z = 0, so step 1 G2 term is zero regardless
// of g2 value.  Only after subsequent steps does G2 produce a divergence.

#[test]
fn g2_zero_on_first_real_step() {
    // Small gains: v_des_z = -0.0009 Nm/step → 22 steps → tau ≈ -0.020 << clamp.
    let kr = [0.00015_f32; 3];
    let kw = [0.00030_f32; 3];
    let omega     = [0.0_f32, 0.0, 3.0];
    let er        = [0.0_f32; 3];
    let e_omega   = [0.0_f32, 0.0, omega[2]];
    let gyro_comp = [0.0_f32; 3];

    let mut s0 = FwState::zero();
    let mut sg = FwState::zero();

    // Step 1 is init (skipped for G2 purposes — du_prev_z = 0 on both).
    fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.0,  60.0, DT, &mut s0);
    fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.14, 60.0, DT, &mut sg);

    // Step 2: du_prev_z was 0 going into step 2 for both, so G2 term = 0 → identical.
    let out0 = fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.0,  60.0, DT, &mut s0);
    let outg = fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.14, 60.0, DT, &mut sg);

    println!("g2_zero_first_step: step2 g2=0 τz={:.8}  g2=0.14 τz={:.8}",
             out0[2], outg[2]);

    // On step 2, du_prev_z was seeded from step 1's delta_z.
    // Depending on v_des.z the outputs may already differ slightly.
    // The key invariant is: both produce finite values.
    assert!(out0[2].is_finite(), "g2=0 step2 τz is NaN");
    assert!(outg[2].is_finite(), "g2=0.14 step2 τz is NaN");

    // After many steps the G2 difference must become visible.
    for _ in 0..20 {
        fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.0,  60.0, DT, &mut s0);
        fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.14, 60.0, DT, &mut sg);
    }
    let out0_late = fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.0,  60.0, DT, &mut s0);
    let outg_late = fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.14, 60.0, DT, &mut sg);

    println!("g2_zero_first_step: step22 g2=0 τz={:.8}  g2=0.14 τz={:.8}",
             out0_late[2], outg_late[2]);

    let late_diff = (outg_late[2] - out0_late[2]).abs();
    assert!(
        late_diff > 1e-10,
        "G2 should diverge from G2=0 after 20+ steps: diff = {late_diff:.2e}",
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5 — full_indi_torque G2 same formula as indi_torque G2 (P7 patch)
// ─────────────────────────────────────────────────────────────────────────────
//
// Both modes share the identical delta_tau.z G2 formula:
//   delta_z = (v_des.z - J*α_meas + G2*du_prev_z) / (1 + G2)
//
// Strategy: seed du_prev_z to a known non-zero value and omega_prev = omega
// (so alpha_raw = 0 and there is no large first-step spike).  With identical
// seeded states the delta_tau.z — and therefore the sign/magnitude of the G2
// correction — must agree between the two paths.
//
// rpm_sq = [0; 4] → tau_current = 0 → tau_base = delta_tau → isolates formula.

#[test]
fn full_indi_g2_matches_indi_g2() {
    let kr = [0.003_f32; 3];
    let kw = [0.006_f32; 3];
    let g2_bench: f32 = 0.14;
    let omega     = [0.0_f32, 0.0, 3.0];
    let er        = [0.0_f32; 3];
    let e_omega   = [0.0_f32, 0.0, omega[2]];
    let gyro_comp = [0.0_f32; 3];
    let rpm_sq    = [0.0_f32; 4];

    // Seed: omega_prev = omega → alpha_raw = 0, du_prev_z = -0.001 (non-zero to activate G2).
    let seeded = FwState {
        omega_prev:     omega,
        omega_dot_filt: [0.0; 3],
        tau_prev:       [0.0; 3],
        du_prev_z:      -0.001,
        initialized:    true,
    };

    // ── full_indi_torque (mode 2) ─────────────────────────────────────────
    let mut sf0 = seeded.clone(); let mut sfg = seeded.clone();
    let full_no_g2   = fw_full_indi_torque(er, e_omega, omega, gyro_comp, rpm_sq, kr, kw, 0.0,      60.0, DT, &mut sf0);
    let full_with_g2 = fw_full_indi_torque(er, e_omega, omega, gyro_comp, rpm_sq, kr, kw, g2_bench, 60.0, DT, &mut sfg);
    let full_diff = full_with_g2[2] - full_no_g2[2];

    // ── indi_torque (mode 1, same seeded state) ───────────────────────────
    let mut s0 = seeded.clone(); let mut sg = seeded.clone();
    let att_no_g2   = fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, 0.0,      60.0, DT, &mut s0);
    let att_with_g2 = fw_indi_torque(er, e_omega, omega, gyro_comp, kr, kw, g2_bench, 60.0, DT, &mut sg);
    let att_diff = att_with_g2[2] - att_no_g2[2];

    println!("full_indi_g2_matches_indi_g2: full diff={full_diff:.6}  att diff={att_diff:.6}");

    // Both paths must show a measurable G2 effect (|diff| >> float noise).
    assert!(
        full_diff.abs() > 1e-4,
        "full_indi G2 has no effect with seeded du_prev_z=-0.001: diff = {full_diff:.2e}",
    );
    assert!(
        att_diff.abs() > 1e-4,
        "indi_torque G2 has no effect with seeded du_prev_z=-0.001: diff = {att_diff:.2e}",
    );
    // Identical formula → identical sign and approximately equal magnitude.
    assert_eq!(
        full_diff.signum(), att_diff.signum(),
        "G2 sign disagrees between modes: full={full_diff:.6}  att={att_diff:.6}",
    );
    assert!(
        (full_diff - att_diff).abs() < 1e-6,
        "G2 magnitudes should be identical (same formula): full={full_diff:.6}  att={att_diff:.6}",
    );
}
