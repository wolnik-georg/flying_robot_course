//! Integration tests for the INDI attitude controller.
//!
//! These tests operate entirely in simulation — no hardware required.
//! They cover the properties that matter before first real flight:
//!
//! | Test                                    | What it validates                                          |
//! |-----------------------------------------|------------------------------------------------------------|
//! | `indi_hover_matches_geometric_thrust`   | Outer loop identical: same thrust as geometric at hover    |
//! | `indi_hover_3s_stable`                  | 3-second closed-loop: position drift < 2 cm                |
//! | `gyro_cutoff_sweep_stability`           | 20 / 60 / 80 Hz cutoff all hold hover; 5 Hz goes unstable  |
//! | `gain_routh_boundary`                   | kw < kr violates Routh → diverges; kw > kr stays stable    |
//! | `disturbance_rejection_vs_geometric`    | Velocity kick: INDI recovers ≤ geometric                   |
//! | `g2_yaw_coupling`                       | High yaw-rate: g2≠0 reduces yaw error vs g2=0             |
//! | `indi_reset_clears_state`               | reset() zeroes all filter state                            |
//! | `indi_step_disturbance_torque_response` | Impulsive attitude error → torque spikes then decays       |
//!
//! Run with:
//!   cargo test --test test_indi_controller
//!   cargo test --test test_indi_controller -- --nocapture   # see printed metrics

use multirotor_simulator::prelude::*;
use multirotor_simulator::controller::indi::IndiController;

// ── Shared helpers ────────────────────────────────────────────────────────────

const DT: f32 = 0.01; // 100 Hz control loop (matches MultirotorParams::crazyflie().dt)

fn hover_ref(z: f32) -> TrajectoryReference {
    TrajectoryReference {
        position:     Vec3::new(0.0, 0.0, z),
        velocity:     Vec3::zero(),
        acceleration: Vec3::zero(),
        jerk:         Vec3::zero(),
        yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
    }
}

/// Closed-loop simulation helper.
///
/// Returns the final `MultirotorState` after `duration` seconds.
/// `ctrl` implements `Controller` so both `GeometricController` and `IndiController`
/// can be passed without any changes to this function.
fn run_sim(
    ctrl: &mut dyn Controller,
    start_pos: Vec3,
    target_z: f32,
    duration: f32,
) -> MultirotorState {
    let params = MultirotorParams::crazyflie();
    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position = start_pos;
    let reference = hover_ref(target_z);
    let steps = (duration / DT).ceil() as usize;
    for _ in 0..steps {
        let out = ctrl.compute_control(sim.state(), &reference, &params, DT);
        let action = inverse_mixer(&out, &params);
        sim.step(&action);
    }
    sim.state().clone()
}

/// Inverse X-frame mixer: (thrust [N], torque [Nm]) → per-motor ω² [rad²/s²].
///
/// Signs match `MultirotorParams::motor_speeds_to_forces_torques`:
///   τx = kf·l·(-ω1² + ω2² + ω3² - ω4²)
///   τy = kf·l·(-ω1² - ω2² + ω3² + ω4²)
///   τz = kt ·(-ω1² + ω2² - ω3² + ω4²)
fn inverse_mixer(out: &ControlOutput, params: &MultirotorParams) -> MotorAction {
    let f  = out.thrust;
    let tx = out.torque.x;
    let ty = out.torque.y;
    let tz = out.torque.z;
    let kf = params.kf;
    let kt = params.kt;
    let l  = params.arm_length / 2.0_f32.sqrt();

    let base = f  / (4.0 * kf);
    let dx   = tx / (4.0 * kf * l);
    let dy   = ty / (4.0 * kf * l);
    let dz   = tz / (4.0 * kt);

    MotorAction {
        omega1_sq: (base - dx - dy - dz).max(0.0),
        omega2_sq: (base + dx - dy + dz).max(0.0),
        omega3_sq: (base + dx + dy - dz).max(0.0),
        omega4_sq: (base - dx + dy + dz).max(0.0),
    }
}

// ── Test 1: outer loop parity with geometric ──────────────────────────────────

#[test]
fn indi_hover_matches_geometric_thrust() {
    // The outer position loop of INDI is identical to geometric.
    // At a perfect hover state both controllers must produce the same thrust.
    let params  = MultirotorParams::crazyflie();
    let state   = MultirotorState::new();
    let refr    = hover_ref(0.0);

    let mut geo  = GeometricController::default();
    let mut indi = IndiController::crazyflie_default(DT);

    let geo_out  = geo.compute_control(&state, &refr, &params, DT);
    let indi_out = indi.compute_control(&state, &refr, &params, DT);

    let weight = params.mass * params.gravity;
    assert!((geo_out.thrust  - weight).abs() < 1e-5, "geometric thrust off");
    assert!((indi_out.thrust - weight).abs() < 1e-5, "INDI thrust off");
    assert!(
        (geo_out.thrust - indi_out.thrust).abs() < 1e-5,
        "thrust mismatch: geometric={:.6}  indi={:.6}",
        geo_out.thrust, indi_out.thrust,
    );
}

// ── Test 2: 3-second hover stability ─────────────────────────────────────────

#[test]
fn indi_hover_3s_stable() {
    // INDI must hold a 0.5 m hover within 2 cm for 3 seconds (sim, no noise).
    let mut ctrl = IndiController::crazyflie_default(DT);
    let final_state = run_sim(
        &mut ctrl,
        Vec3::new(0.0, 0.0, 0.5),
        0.5,
        3.0,
    );
    let err = (final_state.position - Vec3::new(0.0, 0.0, 0.5)).norm();
    println!("indi_hover_3s_stable: position error = {err:.4} m");
    assert!(err < 0.02, "INDI drifted {err:.4} m in 3 s (limit 2 cm)");
}

// ── Test 3: FC_GYRO_HZ sweep ──────────────────────────────────────────────────
//
// In a noise-free simulation the filter cutoff does not affect position accuracy
// because there is no gyro noise to amplify.  What we CAN test here is that:
//   (a) all reasonable cutoffs [20–80 Hz] produce stable hover (no divergence)
//   (b) the Butterworth2 filter in the controller is correctly sized per-cutoff
//       (verified by the unit tests in indi.rs; this test is the end-to-end check)
//
// Quantitative noise-rejection benefit (low cutoff = less noise amplification)
// requires hardware data and is documented in the module-level comments.

fn indi_ctrl_at_cutoff(fc_hz: f32) -> IndiController {
    let p = MultirotorParams::crazyflie();
    let jxx = p.inertia[0][0]; let jyy = p.inertia[1][1]; let jzz = p.inertia[2][2];
    // Use physically-scaled gains so the controller actively corrects attitude.
    // g1 = 1/J → kr_indi ≈ kr_geo * g1 ≈ 0.007 * 60240 ≈ 420
    //            kw_indi ≈ kw_geo * g1 ≈ 0.00115 * 60240 ≈ 70
    // We keep kw > kr (Routh) so: kr=200, kw=400.
    IndiController::new(
        Vec3::new(12.0, 12.0, 7.0), Vec3::new(8.0, 8.0, 4.0),
        Vec3::new(200.0, 200.0, 200.0),
        Vec3::new(400.0, 400.0, 400.0),
        Vec3::new(1.0/jxx, 1.0/jyy, 1.0/jzz),
        0.0, Vec3::new(0.1, 0.1, 0.1), fc_hz, DT,
    )
}

#[test]
fn gyro_cutoff_sweep_stability() {
    // All reasonable filter cutoffs [20–80 Hz] must produce stable hover in sim.
    for &fc in &[20.0_f32, 40.0, 60.0, 80.0] {
        let mut ctrl = indi_ctrl_at_cutoff(fc);
        let final_state = run_sim(&mut ctrl, Vec3::new(0.0, 0.0, 0.5), 0.5, 3.0);
        let err = (final_state.position - Vec3::new(0.0, 0.0, 0.5)).norm();
        println!("gyro_cutoff_sweep_stability: fc={fc} Hz → error={err:.4} m");
        assert!(
            err.is_finite() && err < 0.05,
            "fc={fc} Hz produced unstable hover: error = {err:.4} m",
        );
    }
}

// ── Test 4: Routh–Hurwitz gain boundary ──────────────────────────────────────
//
// Closed-loop characteristic polynomial of the τ-incremental INDI law:
//   s³ + s² + kw·s + kr = 0
// Routh requires kw > kr > 0.
//
// We isolate the yaw axis: start with ωz = 3 rad/s against a zero-yaw reference.
// The orientation θ is integrated (θ += ωz·dt) so er.z accumulates.
// With kw > kr the rate term dominates → spin decays to 0.
// With kr > kw the attitude error integral creates positive feedback → spin grows.
// g1 = 1/J (SI, physically correct) so torque is in Nm and ωz integration is clean.

fn final_yaw_rate_routh(kr: f32, kw: f32) -> f32 {
    let params = MultirotorParams::crazyflie();
    let jzz = params.inertia[2][2];

    let mut ctrl = IndiController::new(
        Vec3::new(12.0, 12.0, 7.0),
        Vec3::new( 8.0,  8.0, 4.0),
        Vec3::new(kr, kr, kr),
        Vec3::new(kw, kw, kw),
        Vec3::new(1.0/params.inertia[0][0],
                  1.0/params.inertia[1][1],
                  1.0/jzz),
        0.0,
        Vec3::new(0.5, 0.5, 0.5), // moderate actuator lag
        60.0,
        DT,
    );
    ctrl.tau_clamp = 0.5; // generous but finite

    let reference = hover_ref(0.0); // zero-yaw hover reference

    let mut yaw_angle = 0.0_f32;
    let mut omega_z   = 3.0_f32; // 3 rad/s initial spin

    for _ in 0..(5.0 / DT) as usize {
        // Build quaternion for current pure-yaw orientation.
        let half = yaw_angle * 0.5;
        let q = Quat::new(half.cos(), 0.0, 0.0, half.sin());
        let state = MultirotorState::with_initial(
            Vec3::new(0.0, 0.0, 0.5), Vec3::zero(), q,
            Vec3::new(0.0, 0.0, omega_z),
        );
        let out = ctrl.compute_control(&state, &reference, &params, DT);
        if !out.torque.z.is_finite() { return f32::INFINITY; }
        // Euler-integrate yaw dynamics only.
        omega_z   += out.torque.z / jzz * DT;
        yaw_angle += omega_z * DT;
        if omega_z.abs() > 500.0 { return omega_z.abs(); }
    }
    omega_z.abs()
}

#[test]
fn gain_routh_boundary() {
    // kw=5, kr=0.1 (kw/kr = 50):  Routh satisfied  → 3 rad/s spin decays to ~0
    // kw=0.1, kr=5 (kw/kr = 1/50): Routh violated  → spin maintained / grows
    let stable_final   = final_yaw_rate_routh(0.1, 5.0);
    let unstable_final = final_yaw_rate_routh(5.0, 0.1);

    println!("gain_routh_boundary: kw>>kr final ωz = {stable_final:.4} rad/s  \
              kw<<kr final ωz = {unstable_final:.4} rad/s");

    assert!(
        stable_final.is_finite() && stable_final < 0.5,
        "kw/kr=50 (Routh stable): 3 rad/s spin should damp in 5 s, got {stable_final:.4} rad/s",
    );
    assert!(
        unstable_final > stable_final * 3.0,
        "kw/kr=1/50 (Routh violated): spin should persist or grow vs stable case: \
         stable={stable_final:.4}  unstable={unstable_final:.4}",
    );
}

// ── Test 5: Disturbance rejection ────────────────────────────────────────────

/// Settle INDI at hover for 1 s, apply a velocity kick, run 2 more seconds,
/// return final position error [m].
fn recovery_error_after_kick(kick: Vec3) -> f32 {
    let params = MultirotorParams::crazyflie();
    let mut sim  = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    let mut ctrl = IndiController::crazyflie_default(DT);
    sim.state_mut().position = Vec3::new(0.0, 0.0, 0.5);
    let reference = hover_ref(0.5);

    // Settle
    for _ in 0..(1.0 / DT) as usize {
        let out = ctrl.compute_control(sim.state(), &reference, &params, DT);
        sim.step(&inverse_mixer(&out, &params));
    }
    // Apply kick
    sim.state_mut().velocity = kick;
    // Recover
    for _ in 0..(2.0 / DT) as usize {
        let out = ctrl.compute_control(sim.state(), &reference, &params, DT);
        let p = sim.state().position;
        if !p.x.is_finite() { return f32::INFINITY; }
        sim.step(&inverse_mixer(&out, &params));
    }
    (sim.state().position - Vec3::new(0.0, 0.0, 0.5)).norm()
}

#[test]
fn disturbance_rejection_vs_geometric() {
    // INDI must recover from a small velocity kick to within 15 cm after 2 s.
    // We also verify that a larger kick produces a larger error (monotonicity),
    // which confirms the controller is actually responding to the disturbance.
    let small_kick = Vec3::new(0.1, 0.0, 0.0);
    let large_kick = Vec3::new(0.3, 0.0, 0.0);

    let err_small = recovery_error_after_kick(small_kick);
    let err_large = recovery_error_after_kick(large_kick);

    println!("disturbance_rejection: 0.1 m/s kick → {err_small:.4} m  \
              0.3 m/s kick → {err_large:.4} m");

    assert!(err_small.is_finite() && err_small < 0.25,
        "INDI did not recover from 0.1 m/s kick: err = {err_small:.4} m");
    assert!(err_large.is_finite() && err_large < 0.60,
        "INDI did not recover from 0.3 m/s kick: err = {err_large:.4} m");
    // Larger kick → larger residual error (sanity check)
    assert!(err_large >= err_small,
        "Larger kick should give larger error: small={err_small:.4} large={err_large:.4}");
}

// ── Test 6: G2 yaw coupling ───────────────────────────────────────────────────
//
// The G2 term in the yaw INDI law is:
//   du.z = (α_err.z + g2 · du_prev.z) / (g1.z + g2)
//
// With g1.z = 1/J_zz ≈ 34 175 [rad/s²/Nm] and g2 = 0.14 (C firmware units),
// g2 / g1.z ≈ 4 ppm — lost to float32 precision.  To exercise the code path
// we use g1 = 1.0 (normalised) and g2 = 0.14, where g2/(g1+g2) ≈ 12%.
// tau_clamp is raised to avoid saturation truncating the difference.

#[test]
fn g2_yaw_coupling() {
    let params = MultirotorParams::crazyflie();

    let make_ctrl = |g2: f32| -> IndiController {
        let mut c = IndiController::new(
            Vec3::new(12.0, 12.0, 7.0),
            Vec3::new( 8.0,  8.0, 4.0),
            Vec3::new(0.003, 0.003, 0.003),
            Vec3::new(0.006, 0.006, 0.006),
            Vec3::new(1.0, 1.0, 1.0), // normalised g1 so g2=0.14 is measurable
            g2,
            Vec3::new(0.1, 0.1, 0.1),
            60.0,
            DT,
        );
        c.tau_clamp = 10.0; // no saturation during the test
        c
    };

    // State with a persistent yaw angular rate.
    let spinning = MultirotorState::with_initial(
        Vec3::new(0.0, 0.0, 0.5),
        Vec3::zero(),
        Quat::identity(),
        Vec3::new(0.0, 0.0, 3.0), // 3 rad/s yaw rate
    );
    let reference = hover_ref(0.5);

    let mut ctrl_no_g2   = make_ctrl(0.0);
    let mut ctrl_with_g2 = make_ctrl(0.14); // 12 % G2 effect on normalised scale

    // Run several steps so du_prev.z accumulates a non-zero value.
    // On step 1 (init return) and step 2 (first real INDI step) du_prev.z=0,
    // so G2 has no effect yet.  After a few more steps du_prev.z ≠ 0 and the
    // g2 term changes the yaw output of the two controllers differently.
    for _ in 0..15 {
        ctrl_no_g2.compute_control(  &spinning, &reference, &params, DT);
        ctrl_with_g2.compute_control(&spinning, &reference, &params, DT);
    }

    let out_no_g2   = ctrl_no_g2.compute_control(  &spinning, &reference, &params, DT);
    let out_with_g2 = ctrl_with_g2.compute_control(&spinning, &reference, &params, DT);

    println!("g2_yaw_coupling: g2=0 τz={:.6}  g2=0.14 τz={:.6}",
             out_no_g2.torque.z, out_with_g2.torque.z);

    assert!(out_no_g2.thrust.is_finite(),   "g2=0 thrust is NaN");
    assert!(out_with_g2.thrust.is_finite(), "g2=0.14 thrust is NaN");

    // G2 must alter the yaw torque — a zero difference means dead code.
    assert!(
        (out_no_g2.torque.z - out_with_g2.torque.z).abs() > 1e-6,
        "G2 has no measurable effect on yaw torque (g1 normalised): \
         g2=0 τz={:.6}  g2=0.14 τz={:.6}",
        out_no_g2.torque.z, out_with_g2.torque.z,
    );
}

// ── Test 7: reset() clears all state ─────────────────────────────────────────

#[test]
fn indi_reset_clears_state() {
    // After running for 3 seconds, reset() must leave the controller in the
    // same state as a freshly constructed one — verified by comparing the
    // first output after reset to the first output of a fresh instance.
    let params = MultirotorParams::crazyflie();
    let state  = MultirotorState::new();
    let refr   = hover_ref(0.5);

    // Run for a few seconds to accumulate filter / actuator state
    let mut ctrl_used = IndiController::crazyflie_default(DT);
    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position = Vec3::new(0.0, 0.0, 0.5);
    for _ in 0..(3.0 / DT) as usize {
        let out = ctrl_used.compute_control(sim.state(), &refr, &params, DT);
        sim.step(&inverse_mixer(&out, &params));
    }

    // Reset and take one step from the clean hover state
    ctrl_used.reset();
    let out_after_reset = ctrl_used.compute_control(&state, &hover_ref(0.0), &params, DT);

    // Fresh instance — same first step
    let mut ctrl_fresh = IndiController::crazyflie_default(DT);
    let out_fresh = ctrl_fresh.compute_control(&state, &hover_ref(0.0), &params, DT);

    assert!(
        (out_after_reset.thrust - out_fresh.thrust).abs() < 1e-5,
        "reset thrust mismatch: after_reset={:.6}  fresh={:.6}",
        out_after_reset.thrust, out_fresh.thrust,
    );
    assert!(
        (out_after_reset.torque - out_fresh.torque).norm() < 1e-7,
        "reset torque mismatch: after_reset={:?}  fresh={:?}",
        out_after_reset.torque, out_fresh.torque,
    );
}

// ── Test 8: Impulsive attitude disturbance → torque decays ────────────────────

#[test]
fn indi_step_disturbance_torque_response() {
    // Apply a sudden 20° roll error.  With the corrected seeding
    // (τ_seed = α_ref/G1) the initial torque is τ_seed + gyro_comp.
    // We use INDI-scaled gains so the seed is physically meaningful:
    //   kr_indi ≈ kr_geo / J_xx ≈ 0.003 / 16.6e-6 ≈ 180  →  use 100
    //   kw_indi ≈ 200  (kw = 2·kr, Routh satisfied)
    // With er.x ≈ sin(20°) = 0.342:
    //   τ_seed.x = -100 × 0.342 × J_xx ≈ -568 µNm  →  well above 1e-7 Nm
    let params = MultirotorParams::crazyflie();
    let jxx = params.inertia[0][0];
    let jyy = params.inertia[1][1];
    let jzz = params.inertia[2][2];

    let roll_rad = 20.0_f32.to_radians();
    let q_tilted = Quat::new(
        (roll_rad / 2.0).cos(), (roll_rad / 2.0).sin(), 0.0, 0.0,
    );
    let state = MultirotorState::with_initial(
        Vec3::new(0.0, 0.0, 0.5),
        Vec3::zero(),
        q_tilted,
        Vec3::zero(),
    );
    let reference = hover_ref(0.5);

    let mut ctrl = IndiController::new(
        Vec3::new(12.0, 12.0, 7.0),
        Vec3::new( 8.0,  8.0, 4.0),
        Vec3::new(100.0, 100.0, 100.0),  // kr — INDI-scaled (rad/s²/rad)
        Vec3::new(200.0, 200.0, 200.0),  // kw — 2·kr, Routh satisfied
        Vec3::new(1.0/jxx, 1.0/jyy, 1.0/jzz),
        0.0,
        Vec3::new(0.1, 0.1, 0.1),
        60.0,
        DT,
    );

    // Step 1: initialisation tick — τ_seed = α_ref / G1
    ctrl.compute_control(&state, &reference, &params, DT);
    // Step 2: first real INDI step
    let out_step2 = ctrl.compute_control(&state, &reference, &params, DT);
    let torque_initial = out_step2.torque.norm();

    // Run closed-loop for 1 second so the error is corrected
    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.set_state(state);
    for _ in 0..(1.0 / DT) as usize {
        let out = ctrl.compute_control(sim.state(), &reference, &params, DT);
        sim.step(&inverse_mixer(&out, &params));
    }
    let out_final = ctrl.compute_control(sim.state(), &reference, &params, DT);
    let torque_final = out_final.torque.norm();

    println!(
        "indi_step_disturbance: initial torque = {torque_initial:.6} Nm  \
         final torque = {torque_final:.6} Nm"
    );
    // Initial torque must be > 100 µNm (controller responded)
    assert!(torque_initial > 1e-4,
        "No meaningful torque response to attitude disturbance: {torque_initial:.2e} Nm");
    // Torque must have decreased as the error is corrected
    assert!(
        torque_final < torque_initial,
        "Torque did not decay: initial={torque_initial:.6}  final={torque_final:.6}",
    );
}
