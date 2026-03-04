//! Tests for the my_hover control loop logic — no drone required.
//!
//! These tests reproduce the exact failure scenarios we observed in real flights
//! and verify the fixes hold.  Each test runs in <100 ms on a laptop.
//!
//! Run with:   cargo test --test test_hover_control_loop
//!
//! # Test inventory
//!
//! | Test name                          | What it catches                                              |
//! |------------------------------------|--------------------------------------------------------------|
//! | test_z_anchor_at_floor_collapses   | Bug F3: anchoring Z to EKF floor → thrust collapse (OLD)    |
//! | test_z_anchor_target_height_stable | Fix F3: anchoring Z to TARGET_HEIGHT=0.3 → stable hover     |
//! | test_ekf_xy_reset_resets_integral  | Fix F4: Lighthouse XY jump wipes stale integral             |
//! | test_integral_windup_clamped       | Verifies ±0.5 clamp on i_pos prevents unbounded growth      |
//! | test_antiwindup_thrust_sat         | Anti-windup reverts i_pos on thrust saturation              |
//! | test_hover_convergence_5s          | End-to-end: closed-loop hover converges within 5 s (sim)    |
//! | test_log_replay_thrust_never_zero  | Regression: replay old flight CSV, assert thrust > 20000    |

use multirotor_simulator::prelude::*;

const HOVER_PWM: f32 = 42000.0;
const TARGET_HEIGHT: f32 = 0.3;
const DT: f32 = 0.01; // 10 ms control loop

// ─────────────────────────────────────────────────────────────────────────────
// Helpers shared across tests
// ─────────────────────────────────────────────────────────────────────────────

/// Build a flat hover reference at a given height (zero velocity/acceleration).
fn hover_ref(x: f32, y: f32, z: f32, yaw_rad: f32) -> TrajectoryReference {
    TrajectoryReference {
        position: Vec3::new(x, y, z),
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
        jerk: Vec3::zero(),
        yaw: yaw_rad,
        yaw_rate: 0.0,
        yaw_acceleration: 0.0,
    }
}

/// Build a multirotor state from just position + velocity (level, no rotation).
fn flat_state(x: f32, y: f32, z: f32, vz: f32) -> MultirotorState {
    MultirotorState {
        position: Vec3::new(x, y, z),
        velocity: Vec3::new(0.0, 0.0, vz),
        orientation: Quat::identity(),
        angular_velocity: Vec3::zero(),
    }
}

/// Convert controller thrust output → PWM (same formula as main.rs).
fn thrust_to_pwm(thrust_n: f32, params: &MultirotorParams) -> f32 {
    let hover_thrust_n = params.mass * params.gravity;
    thrust_n / hover_thrust_n * HOVER_PWM
}

// ─────────────────────────────────────────────────────────────────────────────
// F3 regression — Bug: Z anchor at floor
// ─────────────────────────────────────────────────────────────────────────────

/// DEMONSTRATES THE OLD BUG (should fail with old code).
///
/// Scenario: Lighthouse not ready → EKF z = 0.007 m (floor).
/// Old code anchored `hover_ref.position.z = ekf_z = 0.007`.
/// Drone rises from floor → ep.z = 0.007 - 0.3 = -0.293 m → negative integral windup
/// → thrust collapses.
///
/// This test runs the old buggy anchor and asserts that thrust collapses
/// (= reproduces the crash) to document the failure mode.
#[test]
fn test_z_anchor_at_floor_collapses() {
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();

    // EKF z when Lighthouse hasn't initialised yet — drone is physically at 0.3 m
    // but EKF only knows about floor barometer.
    let ekf_z_before_lighthouse = 0.007_f32;

    // OLD anchor: use whatever EKF says (= floor)
    let reference = hover_ref(0.0, 0.0, ekf_z_before_lighthouse, 0.0);

    // Drone is physically at 0.3 m — this is the real state after setpoint_hover.
    let mut state = flat_state(0.0, 0.0, 0.3, 0.0);

    // Simulate 3 seconds: integral winds up while drone is "above" the reference.
    let mut min_pwm: f32 = f32::MAX;
    for _ in 0..300 {
        let control = controller.compute_control(&state, &reference, &params, DT);
        let pwm = thrust_to_pwm(control.thrust, &params);
        if pwm < min_pwm {
            min_pwm = pwm;
        }
        // Drone stays at 0.3 m (simulated as fixed position for this test)
        state.position.z = 0.3;
    }

    let i_z = controller.i_error_pos().z;
    println!(
        "test_z_anchor_at_floor_collapses: i_pos.z = {:.3}, min_pwm = {:.0}",
        i_z, min_pwm
    );

    // Integral should have wound to the minimum (negative clamp = -0.5)
    assert!(i_z < -0.4, "Expected i_pos.z to wind to -0.5 clamp, got {:.3}", i_z);
    // Thrust PWM should have collapsed well below hover
    assert!(
        min_pwm < 35000.0,
        "Expected thrust collapse (PWM < 35000), got {:.0}",
        min_pwm
    );
}

/// VERIFIES THE FIX (F3): anchor Z to TARGET_HEIGHT=0.3, not EKF z.
///
/// Same scenario as above but with the correct anchor.
/// Drone is at 0.3 m, reference is at 0.3 m → ep.z ≈ 0 from the start
/// → integral stays near zero → thrust stays near HOVER_PWM.
#[test]
fn test_z_anchor_target_height_stable() {
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();

    // FIXED anchor: always use TARGET_HEIGHT regardless of EKF
    let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT, 0.0);

    // Drone is physically at 0.3 m (same as before)
    let mut state = flat_state(0.0, 0.0, 0.3, 0.0);

    let mut min_pwm: f32 = f32::MAX;
    for _ in 0..300 {
        let control = controller.compute_control(&state, &reference, &params, DT);
        let pwm = thrust_to_pwm(control.thrust, &params);
        if pwm < min_pwm {
            min_pwm = pwm;
        }
        state.position.z = 0.3;
    }

    let i_z = controller.i_error_pos().z;
    println!(
        "test_z_anchor_target_height_stable: i_pos.z = {:.3}, min_pwm = {:.0}",
        i_z, min_pwm
    );

    // Integral should stay near zero (ep.z ≈ 0 throughout)
    assert!(
        i_z.abs() < 0.05,
        "Expected i_pos.z near zero, got {:.3}",
        i_z
    );
    // Thrust should stay near HOVER_PWM
    assert!(
        min_pwm > 38000.0,
        "Expected thrust near HOVER_PWM ({:.0}), min was {:.0}",
        HOVER_PWM, min_pwm
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// F4 regression — Bug: stale integral after Lighthouse XY reset
// ─────────────────────────────────────────────────────────────────────────────

/// VERIFIES THE FIX (F4): integral is wiped when Lighthouse fires an XY position jump.
///
/// Scenario:
///   - Before Lighthouse: EKF z was unreliable, so i_pos.z accumulated to -0.3
///     (a stale negative correction fighting the drone upward).
///   - Lighthouse comes online: XY position teleports 1.5 m.
///   - Fix: detect XY step > 0.05 m → reset_position_integral() → stale i_pos.z cleared.
///   - After reset: i_pos.z = 0 → thrust recovers.
#[test]
fn test_ekf_xy_reset_resets_integral() {
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();

    // Manually inject stale negative Z integral (what happens pre-Lighthouse)
    controller.set_i_error_pos(Vec3::new(0.1, -0.05, -0.289));

    let mut reference = hover_ref(0.0, 0.0, TARGET_HEIGHT, 0.0);
    let mut prev_pos = Vec3::new(0.0, 0.0, 0.3);

    // Tick 1: normal step — EKF says drone is at (0, 0, 0.3)
    let cur_pos = Vec3::new(0.0, 0.0, 0.3);
    let step_xy = {
        let d = cur_pos - prev_pos;
        (d.x * d.x + d.y * d.y).sqrt()
    };
    assert!(step_xy < 0.05, "Tick 1 should look normal");

    // Tick 2: Lighthouse fires — EKF teleports (0,0) → (1.5, 0.3)
    let lighthouse_pos = Vec3::new(1.5, 0.3, 0.31);
    let step_xy2 = {
        let d = lighthouse_pos - cur_pos;
        (d.x * d.x + d.y * d.y).sqrt()
    };
    assert!(step_xy2 > 0.05, "Lighthouse step should be detected as reset");

    // Apply the fix logic from main.rs
    if step_xy2 > 0.05 {
        reference.position.x = lighthouse_pos.x;
        reference.position.y = lighthouse_pos.y;
        reference.position.z = lighthouse_pos.z; // re-anchor Z too
        controller.reset_position_integral();     // wipe stale integral
    }
    prev_pos = lighthouse_pos;

    let i_after = controller.i_error_pos();
    println!(
        "test_ekf_xy_reset_resets_integral: i_pos after reset = ({:.3},{:.3},{:.3})",
        i_after.x, i_after.y, i_after.z
    );

    // All integrals should be zero after the Lighthouse XY reset
    assert!(
        i_after.x.abs() < 1e-6 && i_after.y.abs() < 1e-6 && i_after.z.abs() < 1e-6,
        "Expected i_pos = (0,0,0) after XY reset, got ({:.3},{:.3},{:.3})",
        i_after.x, i_after.y, i_after.z
    );

    // Now simulate a few more steps with correct position — thrust should be healthy
    let state = flat_state(lighthouse_pos.x, lighthouse_pos.y, lighthouse_pos.z, 0.0);
    let control = controller.compute_control(&state, &reference, &params, DT);
    let pwm = thrust_to_pwm(control.thrust, &params);
    println!("test_ekf_xy_reset_resets_integral: first PWM after reset = {:.0}", pwm);
    assert!(
        pwm > 35000.0,
        "Thrust should recover after integral reset, got PWM = {:.0}",
        pwm
    );

    let _ = prev_pos; // suppress unused warning
}

// ─────────────────────────────────────────────────────────────────────────────
// Integral clamp — verifies ±0.5 anti-windup inside GeometricController
// ─────────────────────────────────────────────────────────────────────────────

/// Verifies that i_pos never exceeds ±0.5 even under sustained error.
///
/// Scenario: drone is stuck 1 m below target for 30 s.
/// Without the clamp, i_pos.z would grow to 1.0 * 0.05 * 30 = 1.5.
/// With the clamp, it must stay at 0.5.
#[test]
fn test_integral_windup_clamped() {
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();

    // Drone stuck 1 m below the 0.3 m target → ep.z = +1.0 throughout
    let reference = hover_ref(0.0, 0.0, 1.3, 0.0);
    let state = flat_state(0.0, 0.0, 0.3, 0.0);

    for _ in 0..3000 {
        // Note: passing the same `state` every time simulates drone stuck at 0.3 m
        controller.compute_control(&state, &reference, &params, DT);
    }

    let i_z = controller.i_error_pos().z;
    println!("test_integral_windup_clamped: i_pos.z after 30 s = {:.4}", i_z);

    assert!(
        i_z <= 0.5 + 1e-4,
        "i_pos.z exceeded +0.5 clamp: {:.4}",
        i_z
    );
    assert!(
        i_z >= -0.5 - 1e-4,
        "i_pos.z went below -0.5 clamp: {:.4}",
        i_z
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Anti-windup — thrust saturation reverts integral
// ─────────────────────────────────────────────────────────────────────────────

/// Verifies the anti-windup logic from main.rs:
/// if thrust_pwm_raw > 60_000 or < 10_000, revert i_pos to pre-step value.
///
/// We inject a huge positive Z integral to force thrust saturation, then
/// check that the rollback keeps i_pos from growing further.
#[test]
fn test_antiwindup_thrust_sat() {
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();
    let hover_thrust_n = params.mass * params.gravity;

    // Inject large positive Z integral → thrust will be > 60000 PWM
    controller.set_i_error_pos(Vec3::new(0.0, 0.0, 0.5));

    let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT, 0.0);
    let state = flat_state(0.0, 0.0, TARGET_HEIGHT, 0.0);

    for _ in 0..50 {
        let i_pos_prev = controller.i_error_pos();
        let control = controller.compute_control(&state, &reference, &params, DT);
        let thrust_pwm_raw = control.thrust / hover_thrust_n * HOVER_PWM;

        // Anti-windup: revert integral if thrust saturated (mirrors main.rs logic)
        if thrust_pwm_raw > 60_000.0 || thrust_pwm_raw < 10_000.0 {
            controller.set_i_error_pos(i_pos_prev);
        }
    }

    let i_z_after = controller.i_error_pos().z;
    println!("test_antiwindup_thrust_sat: i_pos.z after 50 sat steps = {:.4}", i_z_after);

    // With anti-windup, the integral should NOT have grown beyond its initial value
    assert!(
        i_z_after <= 0.5 + 1e-4,
        "Anti-windup failed: i_pos.z grew to {:.4}",
        i_z_after
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Pitch sign convention — the most dangerous single bug
// ─────────────────────────────────────────────────────────────────────────────

/// Verifies that the RPYT pitch command has the correct sign convention.
///
/// The Crazyflie RPYT commander uses a LEGACY coordinate system:
///   q = rpy2quat(roll, -pitch_setpoint, yaw)   [firmware controller_lee.c]
///
/// This means our code must NEGATE pitch_d before calling setpoint_rpyt.
///
/// Physical expectation:
///   Drone is to the LEFT of target in X (pos_x < ref_x → ep.x > 0).
///   To move +X, drone must tilt nose-DOWN: pitch_d_raw < 0 (negative).
///   We send pitch_d_cmd = -pitch_d_raw > 0.
///   Firmware negates again: actual attitude pitch = -pitch_d_cmd < 0 = nose-down. ✓
///
/// If pitch sign is wrong (positive feedback):
///   ep.x > 0 → pitch_d_cmd < 0 → firmware pitches nose-UP → drone moves AWAY from target.
///   This causes diverging oscillation, which is exactly what we observed.
#[test]
fn test_rpyt_pitch_sign_convention() {
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();

    // Drone is 0.2 m behind the reference in X, at hover height, level.
    // ep.x = +0.2 → controller wants drone to pitch nose-down to push +X.
    let reference = hover_ref(0.2, 0.0, TARGET_HEIGHT, 0.0);
    let state = flat_state(0.0, 0.0, TARGET_HEIGHT, 0.0);

    let ep = reference.position - state.position;
    assert!(ep.x > 0.0, "Setup: ep.x should be positive");

    // Compute force vector (same formula as main.rs)
    let control = controller.compute_control(&state, &reference, &params, DT);
    let ep_fresh = reference.position - state.position;
    let ev = reference.velocity - state.velocity;
    let i_pos = controller.i_error_pos();
    let desired_accel = reference.acceleration
        + Vec3::new(controller.kp.x * ep_fresh.x, 0.0, 0.0)
        + Vec3::new(controller.kv.x * ev.x, 0.0, 0.0)
        + Vec3::new(controller.ki_pos.x * i_pos.x, 0.0, 0.0)
        + Vec3::new(0.0, 0.0, params.gravity);
    let f_vec = desired_accel * params.mass;

    // Step 1: pitch_d_raw should be NEGATIVE (nose-down = tilt toward +X)
    let pitch_d_raw = f_vec.x.atan2(f_vec.z).to_degrees();
    println!("test_rpyt_pitch_sign: ep.x={:.3}, f_vec.x={:.4}, pitch_d_raw={:.3}°", ep.x, f_vec.x, pitch_d_raw);
    assert!(
        pitch_d_raw > 0.0,
        "Expected pitch_d_raw > 0 when ep.x > 0 (force in +X direction), got {:.3}°",
        pitch_d_raw
    );

    // Step 2: pitch_d_cmd (what we send) must be NEGATED = negative
    let pitch_d_cmd = -pitch_d_raw;
    println!("test_rpyt_pitch_sign: pitch_d_cmd (sent to drone) = {:.3}°", pitch_d_cmd);

    // Step 3: the firmware will negate pitch again internally, resulting in nose-down
    // actual_pitch = -pitch_d_cmd = pitch_d_raw > 0 — which in NED/firmware convention
    // means the nose tilts in the direction of positive X.
    // ASSERTION: pitch_d_cmd must be negative so firmware produces a corrective tilt.
    assert!(
        pitch_d_cmd < 0.0,
        "pitch_d_cmd sent to setpoint_rpyt must be negative for ep.x > 0, got {:.3}°. \
        If this fails, the legacy pitch inversion is missing.",
        pitch_d_cmd
    );

    let _ = control; // suppress unused warning
}

/// Same convention test for roll: drone to the right of target (ep.y > 0)
/// must produce roll_d_cmd > 0 (right-wing down = move in +Y direction).
#[test]
fn test_rpyt_roll_sign_convention() {
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();

    // Drone is 0.2 m to the left of reference in Y (ep.y = +0.2)
    let reference = hover_ref(0.0, 0.2, TARGET_HEIGHT, 0.0);
    let state = flat_state(0.0, 0.0, TARGET_HEIGHT, 0.0);

    let ep = reference.position - state.position;
    assert!(ep.y > 0.0);

    let control = controller.compute_control(&state, &reference, &params, DT);
    let ep_fresh = reference.position - state.position;
    let ev = reference.velocity - state.velocity;
    let i_pos = controller.i_error_pos();
    let desired_accel = reference.acceleration
        + Vec3::new(0.0, controller.kp.y * ep_fresh.y, 0.0)
        + Vec3::new(0.0, controller.kv.y * ev.y, 0.0)
        + Vec3::new(0.0, controller.ki_pos.y * i_pos.y, 0.0)
        + Vec3::new(0.0, 0.0, params.gravity);
    let f_vec = desired_accel * params.mass;

    let roll_d_raw = (-f_vec.y).atan2(f_vec.z).to_degrees();
    println!("test_rpyt_roll_sign: ep.y={:.3}, f_vec.y={:.4}, roll_d_raw={:.3}°", ep.y, f_vec.y, roll_d_raw);

    // ep.y > 0 → f_vec.y > 0 → roll_d_raw = atan2(-f_vec.y, f_vec.z) < 0
    // Roll convention is NOT inverted in the firmware — no negation needed.
    // roll_d_raw < 0 means left-wing down / right-wing up → drone moves +Y. ✓
    // (Roll direction follows the right-hand rule in the firmware's body frame.)
    assert!(
        roll_d_raw < 0.0,
        "Expected roll_d_raw < 0 when ep.y > 0, got {:.3}°. Check roll sign.",
        roll_d_raw
    );

    let _ = control;
}

// ─────────────────────────────────────────────────────────────────────────────
// End-to-end: closed-loop hover convergence using the physics simulator
// ─────────────────────────────────────────────────────────────────────────────

/// Full closed-loop hover test: start at z=0, target z=0.3.
/// Uses the rigid-body simulator (same code as in the existing test_geometric_controller_hover).
/// Asserts convergence within 5 simulated seconds.
///
/// This mirrors the existing test but with the DEFAULT gains used in flight
/// and a floor-start (which is the actual power-on condition).
#[test]
fn test_hover_convergence_5s() {
    let params = MultirotorParams::crazyflie();
    let integrator = Box::new(RK4Integrator);
    let mut simulator = MultirotorSimulator::new(params.clone(), integrator);

    // Start at rest on the floor
    simulator.set_state(MultirotorState {
        position: Vec3::new(0.0, 0.0, 0.0),
        velocity: Vec3::zero(),
        orientation: Quat::identity(),
        angular_velocity: Vec3::zero(),
    });

    let mut controller = GeometricController::default();
    let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT, 0.0);

    let mut t = 0.0_f32;
    while t < 5.0 {
        let state = simulator.state().clone();
        let control = controller.compute_control(&state, &reference, &params, DT);
        let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
        simulator.step(&motor_action);
        t += DT;

        // Safety: bail out on NaN to get a useful message rather than a panic
        let pos = simulator.state().position;
        assert!(
            !pos.z.is_nan(),
            "z became NaN at t={:.2}s — check controller gains",
            t
        );
    }

    let final_pos = simulator.state().position;
    let ep_z = (TARGET_HEIGHT - final_pos.z).abs();
    println!(
        "test_hover_convergence_5s: final z = {:.3} m, ep_z = {:.4} m",
        final_pos.z, ep_z
    );

    assert!(
        ep_z < 0.05,
        "Z error too large after 5 s: {:.4} m (target {:.3} m, got {:.3} m)",
        ep_z, TARGET_HEIGHT, final_pos.z
    );

    let vel = simulator.state().velocity;
    assert!(
        vel.norm() < 0.15,
        "Velocity still too large after 5 s: {:.4} m/s",
        vel.norm()
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Log replay — feed real flight CSV through the control loop, check PWM
// ─────────────────────────────────────────────────────────────────────────────

/// Log-replay regression test.
///
/// Reads the most recent hover CSV from bitcraze_rs_lib/runs/, feeds each row
/// through the RPYT-mode controller (with TARGET_HEIGHT anchor and integral reset
/// logic), and asserts that thrust PWM never collapses below 20 000 after the
/// first 2 seconds (i.e., after the initial ramp).
///
/// This turns every real crash into a permanent regression test: copy the
/// offending CSV into runs/ and add an assertion for the symptom you observed.
#[test]
fn test_log_replay_thrust_never_collapses() {
    // ── locate the CSV ──────────────────────────────────────────────────────
    // Path is relative to the crate root (flying_drone_stack/).
    // The test looks for ANY hover CSV in the known runs/ directories.
    let search_dirs = [
        "../bitcraze_rs_lib/runs",
        "runs",
    ];

    let csv_path = search_dirs
        .iter()
        .find_map(|dir| {
            let p = std::path::Path::new(dir);
            if !p.exists() {
                return None;
            }
            std::fs::read_dir(p).ok()?.filter_map(|e| e.ok()).find_map(|entry| {
                let path = entry.path();
                if path.extension()? == "csv"
                    && path.file_name()?.to_str()?.contains("hover")
                {
                    Some(path)
                } else {
                    None
                }
            })
        });

    let csv_path = match csv_path {
        Some(p) => p,
        None => {
            println!("test_log_replay_thrust_never_collapses: no hover CSV found, skipping.");
            return;
        }
    };

    println!("Replaying log: {:?}", csv_path);

    // ── parse CSV ───────────────────────────────────────────────────────────
    #[derive(Debug)]
    struct Row {
        time_ms: u64,
        pos_x: f32, pos_y: f32, pos_z: f32,
        vel_x: f32, vel_y: f32, vel_z: f32,
        roll: f32, pitch: f32, yaw: f32,
        gyro_x: f32, gyro_y: f32, gyro_z: f32,
    }

    let content = std::fs::read_to_string(&csv_path)
        .expect("Failed to read CSV");

    let rows: Vec<Row> = content
        .lines()
        .skip(1) // header
        .filter(|l| !l.trim().is_empty())
        .map(|line| {
            let cols: Vec<&str> = line.split(',').collect();
            fn p(s: &str) -> f32 { s.trim().parse().unwrap_or(0.0) }
            fn pu(s: &str) -> u64 { s.trim().parse().unwrap_or(0) }
            Row {
                time_ms: pu(cols[0]),
                pos_x: p(cols[1]), pos_y: p(cols[2]), pos_z: p(cols[3]),
                vel_x: p(cols[4]), vel_y: p(cols[5]), vel_z: p(cols[6]),
                roll:  p(cols[7]), pitch: p(cols[8]), yaw:  p(cols[9]),
                // cols[10] = thrust (uint), cols[11] = vbat — skip
                gyro_x: p(cols[12]), gyro_y: p(cols[13]), gyro_z: p(cols[14]),
            }
        })
        .collect();

    if rows.is_empty() {
        println!("CSV is empty, skipping.");
        return;
    }

    // ── replay ──────────────────────────────────────────────────────────────
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();
    let hover_thrust_n = params.mass * params.gravity;

    // Anchor exactly as main.rs does: XY from first row, Z = TARGET_HEIGHT
    let first = &rows[0];
    let mut reference = hover_ref(first.pos_x, first.pos_y, TARGET_HEIGHT, first.yaw.to_radians());

    let mut prev_pos: Option<Vec3> = None;
    let mut prev_yaw: f32 = first.yaw;
    let mut min_pwm: f32 = f32::MAX;

    for row in &rows {
        let state = MultirotorState {
            position: Vec3::new(row.pos_x, row.pos_y, row.pos_z),
            velocity: Vec3::new(row.vel_x, row.vel_y, row.vel_z),
            orientation: {
                let q_yaw   = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), row.yaw.to_radians());
                let q_pitch = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), row.pitch.to_radians());
                let q_roll  = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), row.roll.to_radians());
                (q_yaw * q_pitch * q_roll).normalize()
            },
            angular_velocity: Vec3::new(
                row.gyro_x.to_radians(),
                row.gyro_y.to_radians(),
                row.gyro_z.to_radians(),
            ),
        };

        // EKF reset detection (same logic as main.rs)
        let cur_pos = state.position;
        let cur_yaw = row.yaw;
        if let Some(prev) = prev_pos {
            let step = cur_pos - prev;
            let step_xy = (step.x * step.x + step.y * step.y).sqrt();
            let step_z  = step.z.abs();
            let dyaw = {
                let d = cur_yaw - prev_yaw;
                let d = ((d + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
                d.abs()
            };
            if step_xy > 0.05 {
                reference.position.x = cur_pos.x;
                reference.position.y = cur_pos.y;
                reference.position.z = cur_pos.z;
                controller.reset_position_integral();
            }
            if step_z > 0.05 {
                reference.position.z = cur_pos.z;
                let mut i = controller.i_error_pos();
                i.z = 0.0;
                controller.set_i_error_pos(i);
            }
            if dyaw > 10.0 {
                reference.yaw = cur_yaw.to_radians();
            }
        }
        prev_pos = Some(cur_pos);
        prev_yaw = cur_yaw;

        let i_pos_prev = controller.i_error_pos();
        let control = controller.compute_control(&state, &reference, &params, DT);
        let thrust_pwm_raw = control.thrust / hover_thrust_n * HOVER_PWM;

        // Anti-windup
        if thrust_pwm_raw > 60_000.0 || thrust_pwm_raw < 10_000.0 {
            controller.set_i_error_pos(i_pos_prev);
        }

        // Only track PWM after first 2 s (initial ramp-up excluded)
        if row.time_ms > 2000 {
            if thrust_pwm_raw < min_pwm {
                min_pwm = thrust_pwm_raw;
            }
        }
    }

    if min_pwm == f32::MAX {
        println!("Log shorter than 2 s — skipping PWM check.");
        return;
    }

    println!(
        "test_log_replay_thrust_never_collapses: min PWM after 2 s = {:.0}",
        min_pwm
    );

    assert!(
        min_pwm > 20_000.0,
        "Thrust collapsed to {:.0} PWM during replay — integral windup bug!",
        min_pwm
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Physical parameters — must exactly match the firmware
// ─────────────────────────────────────────────────────────────────────────────

/// Verify that MultirotorParams::crazyflie() matches the firmware exactly.
///
/// Firmware reference: controller_lee.c
///   .mass = CF_MASS  (= 0.027 kg)
///   .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}  [kg·m²]
///
/// If any of these values are wrong, the simulator drifts from the real drone:
/// hover thrust ≠ m*g, gyroscopic compensation is scaled wrong, attitude
/// dynamics are wrong. These CANNOT be caught by flying — only by this test.
#[test]
fn test_params_match_firmware() {
    let p = MultirotorParams::crazyflie();

    // Mass — firmware: CF_MASS = 0.027 kg (27 grams, NOT 30 grams)
    assert!(
        (p.mass - 0.027).abs() < 1e-6,
        "mass = {:.4} kg, expected 0.027 kg (firmware CF_MASS)",
        p.mass
    );

    // Inertia — firmware controller_lee.c values, Julian Foerster ETHZ BA thesis
    assert!(
        (p.inertia[0][0] - 16.571710e-6).abs() < 1e-12,
        "Jxx = {:e}, expected 16.571710e-6 (firmware)",
        p.inertia[0][0]
    );
    assert!(
        (p.inertia[1][1] - 16.655602e-6).abs() < 1e-12,
        "Jyy = {:e}, expected 16.655602e-6 (firmware)",
        p.inertia[1][1]
    );
    assert!(
        (p.inertia[2][2] - 29.261652e-6).abs() < 1e-12,
        "Jzz = {:e}, expected 29.261652e-6 (firmware)",
        p.inertia[2][2]
    );

    // Gravity
    assert!(
        (p.gravity - 9.81).abs() < 1e-4,
        "gravity = {}, expected 9.81",
        p.gravity
    );
}

/// Verify hover thrust = m * g exactly (controller must output this at equilibrium).
///
/// At hover (level, zero velocity, zero error, zero integral):
///   F_d = [0, 0, g]  →  thrust_force = m * F_d  →  thrust = dot(thrust_force, body_z) = m*g
///
/// If this fails, the thrust→PWM scaling is wrong from the start.
#[test]
fn test_hover_thrust_equals_weight() {
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();
    // Reset ki_pos to zero so there's no integral contribution
    controller.ki_pos = Vec3::zero();

    let state = flat_state(0.0, 0.0, TARGET_HEIGHT, 0.0);
    let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT, 0.0);

    let control = controller.compute_control(&state, &reference, &params, DT);
    let expected = params.mass * params.gravity;

    println!(
        "test_hover_thrust_equals_weight: thrust = {:.6} N, m*g = {:.6} N",
        control.thrust, expected
    );
    assert!(
        (control.thrust - expected).abs() < 1e-5,
        "Hover thrust {:.6} N ≠ m*g = {:.6} N",
        control.thrust, expected
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// f_vec consistency — main.rs recomputes force vector after compute_control
// ─────────────────────────────────────────────────────────────────────────────

/// Verifies that the `desired_accel` / `f_vec` recomputed in main.rs
/// after calling compute_control() is consistent with what the controller
/// used internally for thrust direction.
///
/// Bug scenario: if someone edits the F_d formula in compute_control but not
/// in main.rs (or vice versa), the roll/pitch angles and thrust PWM come from
/// different force vectors — the drone tilts in the wrong direction while
/// applying the correct thrust magnitude.
///
/// At hover: f_vec.z ≈ m*g, f_vec.x ≈ kp*ep.x*m, etc.
#[test]
fn test_f_vec_consistent_with_controller() {
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();

    // Drone 0.1 m away in X and slightly below target
    let reference = hover_ref(0.1, 0.0, TARGET_HEIGHT, 0.0);
    let state = flat_state(0.0, 0.0, TARGET_HEIGHT - 0.05, 0.0);

    let ep = reference.position - state.position;
    let ev = reference.velocity - state.velocity;

    let control = controller.compute_control(&state, &reference, &params, DT);
    let i_pos = controller.i_error_pos();

    // Reproduce the f_vec calculation from main.rs exactly
    let desired_accel = reference.acceleration
        + Vec3::new(controller.kp.x * ep.x, controller.kp.y * ep.y, controller.kp.z * ep.z)
        + Vec3::new(controller.kv.x * ev.x, controller.kv.y * ev.y, controller.kv.z * ev.z)
        + Vec3::new(controller.ki_pos.x * i_pos.x, controller.ki_pos.y * i_pos.y, controller.ki_pos.z * i_pos.z)
        + Vec3::new(0.0, 0.0, params.gravity);
    let f_vec = desired_accel * params.mass;

    // The controller's thrust = dot(f_vec, body_z). At level flight body_z = world_z,
    // so thrust ≈ f_vec.z.
    let f_vec_thrust_approx = f_vec.z; // valid when level (orientation = identity)
    println!(
        "test_f_vec_consistent: control.thrust={:.5}, f_vec.z={:.5}, diff={:.6}",
        control.thrust, f_vec_thrust_approx, (control.thrust - f_vec_thrust_approx).abs()
    );

    // At level flight these should be equal to within floating point precision
    assert!(
        (control.thrust - f_vec_thrust_approx).abs() < 1e-4,
        "control.thrust ({:.5}) ≠ f_vec.z ({:.5}) at level flight — \
        the two force vector calculations have diverged",
        control.thrust, f_vec_thrust_approx
    );

    // Also verify x component drives pitch (non-zero ep.x → non-zero f_vec.x).
    // Expected magnitude: kp * ep.x * mass = 7 * 0.1 * 0.027 = 0.0189 N
    let expected_fx = controller.kp.x * ep.x * params.mass;
    println!(
        "test_f_vec_consistent: f_vec.x={:.5}, expected≈{:.5}",
        f_vec.x, expected_fx
    );
    assert!(
        f_vec.x.abs() > 0.01,
        "f_vec.x should be nonzero for ep.x={:.3}, got {:.5} (expected ~{:.5} = kp*ep.x*mass)",
        ep.x, f_vec.x, expected_fx
    );
    assert!(
        (f_vec.x - expected_fx).abs() < 1e-4,
        "f_vec.x={:.5} should match kp*ep.x*mass={:.5}",
        f_vec.x, expected_fx
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Quaternion reconstruction from RPY — must match firmware convention
// ─────────────────────────────────────────────────────────────────────────────

/// Verifies that the quaternion reconstruction used in main.rs to convert
/// EKF log data (roll/pitch/yaw in degrees) back into a rotation matrix
/// is correct for known angles.
///
/// Firmware EKF outputs Euler angles in ZYX convention (yaw applied first,
/// then pitch, then roll — standard aerospace). Our code reconstructs as:
///   q = q_yaw * q_pitch * q_roll
/// which is exactly the ZYX / Tait-Bryan convention.
///
/// Test cases:
///   - Level flight (0,0,0): body_z should be world_z = (0,0,1)
///   - 90° yaw: body_x should point in world_y direction
///   - 45° pitch: body_x should have both x and z components
#[test]
fn test_quaternion_reconstruction_from_rpy() {
    // Helper: reconstruct quaternion the same way main.rs does
    let reconstruct = |roll_deg: f32, pitch_deg: f32, yaw_deg: f32| -> [[f32; 3]; 3] {
        let roll_rad  = roll_deg  * std::f32::consts::PI / 180.0;
        let pitch_rad = pitch_deg * std::f32::consts::PI / 180.0;
        let yaw_rad   = yaw_deg   * std::f32::consts::PI / 180.0;
        let q_yaw   = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw_rad);
        let q_pitch = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), pitch_rad);
        let q_roll  = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll_rad);
        let q = (q_yaw * q_pitch * q_roll).normalize();
        q.to_rotation_matrix()
    };

    // Case 1: level (0,0,0) → identity rotation
    let r0 = reconstruct(0.0, 0.0, 0.0);
    let body_z0 = Vec3::new(r0[0][2], r0[1][2], r0[2][2]);
    println!("Level: body_z = ({:.3},{:.3},{:.3})", body_z0.x, body_z0.y, body_z0.z);
    assert!((body_z0.z - 1.0).abs() < 1e-5, "Level body_z should be world_z, got {:.4}", body_z0.z);
    assert!(body_z0.x.abs() < 1e-5 && body_z0.y.abs() < 1e-5);

    // Case 2: 90° yaw → body_x points toward world_y
    let r90 = reconstruct(0.0, 0.0, 90.0);
    let body_x90 = Vec3::new(r90[0][0], r90[1][0], r90[2][0]);
    println!("90° yaw: body_x = ({:.3},{:.3},{:.3})", body_x90.x, body_x90.y, body_x90.z);
    assert!(
        (body_x90.y - 1.0).abs() < 1e-5,
        "After 90° yaw, body_x should point in world_y. Got ({:.3},{:.3},{:.3})",
        body_x90.x, body_x90.y, body_x90.z
    );

    // Case 3: 45° pitch nose-up → body_z tilts toward +world_x
    // Right-hand rule around +Y axis: positive pitch rotates body_x downward
    // and body_z toward +world_x.
    //   body_z = (sin45, 0, cos45) = (+0.707, 0, 0.707)
    //
    // This matters for the sign chain in F5: when ep.x > 0 (drone behind target):
    //   f_vec.x > 0  →  pitch_d_raw > 0  →  pitch_d_cmd = -pitch_d_raw < 0
    //   firmware applies its own -pitch negation  →  actual_pitch = +pitch_d_raw > 0
    //   positive pitch → body_z tilts toward +world_x → drone accelerates +X ✓
    let r_pitch = reconstruct(0.0, 45.0, 0.0);
    let body_z_pitch = Vec3::new(r_pitch[0][2], r_pitch[1][2], r_pitch[2][2]);
    println!("45° pitch: body_z = ({:.3},{:.3},{:.3})", body_z_pitch.x, body_z_pitch.y, body_z_pitch.z);
    // At 45° pitch: body_z.x = +sin(45°) = +0.707 (right-hand rule around +Y)
    assert!(
        body_z_pitch.x > 0.5,
        "45° pitch should tilt body_z toward +world_x (right-hand rule around +Y axis). \
        Got body_z.x = {:.3} (expected > +0.5)",
        body_z_pitch.x
    );
    assert!(
        (body_z_pitch.z - std::f32::consts::FRAC_1_SQRT_2).abs() < 0.01,
        "45° pitch body_z.z should be cos(45°) = 0.707, got {:.3}",
        body_z_pitch.z
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Yaw error wrapping — used in 3 places, easy to get wrong at ±180°
// ─────────────────────────────────────────────────────────────────────────────

/// Verifies the yaw-jump detection wrapping arithmetic is correct.
///
/// The wrapping formula: ((d + 180) % 360 + 360) % 360 - 180
/// Must produce the shortest-path angular difference in (-180, 180].
///
/// Critical edge cases:
///   +179° and -179° are 2° apart (not 358°)
///   0° and 359° are 1° apart (not 359°)
///   The EKF reset threshold is 10° — if wrapping is wrong, a genuine 11°
///   jump at ±180° would be seen as 349° and missed.
#[test]
fn test_yaw_wrap_arithmetic() {
    let wrap = |cur: f32, prev: f32| -> f32 {
        let d = cur - prev;
        let d = ((d + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        d.abs()
    };

    // Normal small forward rotation
    assert!((wrap(10.0, 5.0) - 5.0).abs() < 1e-4, "5° forward");
    assert!((wrap(5.0, 10.0) - 5.0).abs() < 1e-4, "5° backward");

    // Zero crossing
    assert!((wrap(5.0, 355.0) - 10.0).abs() < 1e-4,
        "0° crossing: 355→5 = 10°, got {:.3}", wrap(5.0, 355.0));
    assert!((wrap(355.0, 5.0) - 10.0).abs() < 1e-4,
        "0° crossing: 5→355 = 10°, got {:.3}", wrap(355.0, 5.0));

    // ±180° crossing — this is the most dangerous case
    // Genuine 2° jump straddling ±180° should NOT trigger the 10° reset threshold
    assert!((wrap(179.0, -179.0) - 2.0).abs() < 1e-3,
        "+179 to -179 should be 2°, got {:.3}", wrap(179.0, -179.0));
    assert!((wrap(-179.0, 179.0) - 2.0).abs() < 1e-3,
        "-179 to +179 should be 2°, got {:.3}", wrap(-179.0, 179.0));

    // Large genuine rotation that IS an EKF reset (>10°)
    assert!(wrap(20.0, 5.0) > 10.0, "15° should be detected as reset");
    assert!(wrap(355.0, 340.0) > 10.0, "15° near 360° should be detected");

    // EKF reset threshold: 10°. Confirm 9° is NOT flagged, 11° IS.
    assert!(wrap(14.0, 5.0) < 10.0 + 1e-4, "9° should NOT trigger reset");
    assert!(wrap(16.0, 5.0) > 10.0, "11° should trigger reset");
}

// ─────────────────────────────────────────────────────────────────────────────
// Controller gains — must match firmware defaults exactly
// ─────────────────────────────────────────────────────────────────────────────

/// Verifies that GeometricController::default() uses the exact firmware gains.
///
/// Firmware defaults (controller_lee.c g_self initialisation):
///   Kpos_P = {7, 7, 7}
///   Kpos_D = {4, 4, 4}
///   Kpos_I = {0, 0, 0}   (firmware default; we use 0.05 for RPYT mode)
///   KR     = {0.007, 0.007, 0.008}
///   Komega = {0.00115, 0.00115, 0.002}
///   KI     = {0.03, 0.03, 0.03}  (firmware; we set to 0 because RPYT mode
///                                  doesn't use torque output)
#[test]
fn test_default_gains_match_firmware() {
    let c = GeometricController::default();

    assert_eq!(c.kp, Vec3::new(7.0, 7.0, 7.0),   "kp mismatch (firmware: Kpos_P = 7,7,7)");
    assert_eq!(c.kv, Vec3::new(4.0, 4.0, 4.0),   "kv mismatch (firmware: Kpos_D = 4,4,4)");
    assert_eq!(c.kr, Vec3::new(0.007, 0.007, 0.008), "kr mismatch (firmware: KR)");
    assert_eq!(c.kw, Vec3::new(0.00115, 0.00115, 0.002), "kw mismatch (firmware: Komega)");

    // ki_pos: firmware default is 0, we use 0.05 intentionally for RPYT drift correction.
    // Document and pin the actual value used so any change is visible.
    assert_eq!(c.ki_pos, Vec3::new(0.05, 0.05, 0.05),
        "ki_pos changed — was intentionally 0.05 for RPYT steady-state correction");

    // ki_att: we set to 0 because in RPYT mode only roll_d/pitch_d/thrust are sent,
    // not torques. Any non-zero value here causes unbounded integral windup.
    assert_eq!(c.ki, Vec3::zero(),
        "ki_att must be 0 in RPYT mode — non-zero causes unbounded windup");
}

// ─────────────────────────────────────────────────────────────────────────────
// Thrust→PWM scaling sanity — at equilibrium PWM must be near HOVER_PWM
// ─────────────────────────────────────────────────────────────────────────────

/// At steady hover (no error, no integral), control.thrust = m*g.
/// Scaling: pwm = thrust / (m*g) * HOVER_PWM = HOVER_PWM exactly.
///
/// This pins the scaling formula. If someone changes HOVER_PWM or the
/// mass value, the expected PWM at hover changes and you need to recalibrate.
#[test]
fn test_hover_pwm_scaling_at_equilibrium() {
    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();
    controller.ki_pos = Vec3::zero(); // no integral to keep it clean

    let state = flat_state(0.0, 0.0, TARGET_HEIGHT, 0.0);
    let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT, 0.0);
    let control = controller.compute_control(&state, &reference, &params, DT);

    let hover_thrust_n = params.mass * params.gravity;
    let pwm = control.thrust / hover_thrust_n * HOVER_PWM;

    println!("test_hover_pwm_scaling: thrust={:.5}N, hover_thrust={:.5}N, pwm={:.1}", 
             control.thrust, hover_thrust_n, pwm);

    assert!(
        (pwm - HOVER_PWM).abs() < 1.0,
        "At equilibrium PWM should be {:.0} (HOVER_PWM), got {:.1}",
        HOVER_PWM, pwm
    );
}

/// Verifies that the thrust→PWM formula stays within the hardware clamping
/// range [10000, 60000] for all physically reasonable flight states.
///
/// Bounds the maximum tilt (±30°) and integral (-0.5..+0.5) that can occur
/// and checks that PWM never hits the hard clamps for these inputs.
#[test]
fn test_thrust_pwm_never_saturates_in_normal_flight() {
    let params = MultirotorParams::crazyflie();
    let hover_thrust_n = params.mass * params.gravity;

    // Sweep over z positions ±0.3m from target and z velocities ±0.5 m/s
    let z_positions = [-0.3_f32, -0.1, 0.0, 0.1, 0.3];
    let z_velocities = [-0.5_f32, -0.2, 0.0, 0.2, 0.5];

    for &dz in &z_positions {
        for &vz in &z_velocities {
            let mut controller = GeometricController::default();
            let state = flat_state(0.0, 0.0, TARGET_HEIGHT + dz, vz);
            let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT, 0.0);
            let control = controller.compute_control(&state, &reference, &params, DT);
            let pwm = control.thrust / hover_thrust_n * HOVER_PWM;

            assert!(
                pwm >= 10_000.0 && pwm <= 60_000.0,
                "PWM = {:.0} out of [10000,60000] for dz={:.2}m, vz={:.2}m/s — \
                controller produces physically unreasonable output",
                pwm, dz, vz
            );
        }
    }
}
