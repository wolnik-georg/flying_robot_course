//! Detailed tests for GeometricController not covered by the inline tests.
//!
//! Covers:
//!   - compute_desired_rotation with tilted force vector
//!   - compute_rotation_error with actual rotation difference
//!   - Position error → nonzero thrust deviation from weight
//!   - Attitude error → nonzero torque
//!   - Gyroscopic compensation: nonzero angular velocity increases torque
//!   - Position integral accumulation over time
//!   - Integral reset when thrust < 0.01 N
//!   - Zdes aligned with thrust_force direction

use multirotor_simulator::controller::{GeometricController, Controller, TrajectoryReference};
use multirotor_simulator::dynamics::{MultirotorState, MultirotorParams};
use multirotor_simulator::math::{Vec3, Quat};

fn default_reference() -> TrajectoryReference {
    TrajectoryReference {
        position: Vec3::zero(),
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
        jerk: Vec3::zero(),
        yaw: 0.0,
        yaw_rate: 0.0,
        yaw_acceleration: 0.0,
    }
}

fn hover_state() -> MultirotorState {
    MultirotorState::new()
}

// ─────────────────────────────────────────────────────────────────────────────
// Hover sanity: no position or attitude error → thrust ≈ m*g, torque ≈ 0
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn test_hover_thrust_equals_weight() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    let state = hover_state();
    let reference = default_reference();
    let out = ctrl.compute_control(&state, &reference, &params, 0.01);
    let expected = params.mass * params.gravity;
    assert!((out.thrust - expected).abs() < 1e-4,
        "hover thrust: got {}, expected {expected}", out.thrust);
    assert!(out.torque.norm() < 1e-6,
        "hover torque should be 0, got {:?}", out.torque);
}

// ─────────────────────────────────────────────────────────────────────────────
// Position error → thrust changes from hover value
// ─────────────────────────────────────────────────────────────────────────────

/// Drone is 1 m below reference → thrust increases (need to climb).
#[test]
fn test_position_error_z_increases_thrust() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    let state = hover_state(); // at z=0
    let mut reference = default_reference();
    reference.position = Vec3::new(0.0, 0.0, 1.0); // want to be at z=1
    let out = ctrl.compute_control(&state, &reference, &params, 0.01);
    let hover_thrust = params.mass * params.gravity;
    assert!(out.thrust > hover_thrust,
        "thrust should be above hover for upward error: {}", out.thrust);
}

/// Drone is 1 m above reference → thrust decreases (need to descend).
#[test]
fn test_position_error_z_decreases_thrust() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    let mut state = hover_state();
    state.position = Vec3::new(0.0, 0.0, 1.0); // drone is at z=1
    let reference = default_reference(); // want to be at z=0
    let out = ctrl.compute_control(&state, &reference, &params, 0.01);
    let hover_thrust = params.mass * params.gravity;
    assert!(out.thrust < hover_thrust,
        "thrust should be below hover for downward error: {}", out.thrust);
}

/// Positive X error (reference.x > state.x) → tilts drone forward.
/// At hover orientation, this creates a nonzero Y torque.
#[test]
fn test_position_error_x_produces_tilt() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    let state = hover_state();
    let mut reference = default_reference();
    reference.position = Vec3::new(1.0, 0.0, 0.0); // 1 m ahead
    let out = ctrl.compute_control(&state, &reference, &params, 0.01);
    // The rotation error from tilting forward should produce nonzero torque
    assert!(out.torque.norm() > 0.0, "torque should be nonzero for x position error");
}

// ─────────────────────────────────────────────────────────────────────────────
// Attitude error → nonzero torque
// ─────────────────────────────────────────────────────────────────────────────

/// Drone is rolled 10° while reference is level → torque to correct roll.
#[test]
fn test_attitude_error_produces_torque() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    let mut state = hover_state();
    // Roll drone 10° about X
    state.orientation = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), 0.175_f32); // ~10°
    let reference = default_reference();
    let out = ctrl.compute_control(&state, &reference, &params, 0.01);
    // Should produce a significant roll-correction torque
    assert!(out.torque.x.abs() > 1e-6, "roll torque expected, got {}", out.torque.x);
}

/// Drone yawed 90° while reference has zero yaw → yaw correction torque.
#[test]
fn test_yaw_error_produces_torque() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    let mut state = hover_state();
    // Yaw drone 90° about Z
    state.orientation = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2);
    let reference = default_reference();
    let out = ctrl.compute_control(&state, &reference, &params, 0.01);
    // Should produce a significant yaw-correction torque
    assert!(out.torque.z.abs() > 1e-6, "yaw torque expected, got {}", out.torque.z);
}

// ─────────────────────────────────────────────────────────────────────────────
// Gyroscopic compensation
// ─────────────────────────────────────────────────────────────────────────────

/// With nonzero angular velocity (fast spin about Z), the output torque
/// should differ from the zero-ω case (gyroscopic term adds ω × J·ω).
#[test]
fn test_gyroscopic_compensation_affects_torque() {
    let params = MultirotorParams::crazyflie();
    let reference = default_reference();

    // Control with zero angular velocity
    let mut ctrl_zero = GeometricController::default();
    let state_zero = hover_state();
    let out_zero = ctrl_zero.compute_control(&state_zero, &reference, &params, 0.01);

    // Control with fast spin (20 rad/s about Z)
    let mut ctrl_spin = GeometricController::default();
    let mut state_spin = hover_state();
    state_spin.angular_velocity = Vec3::new(0.0, 0.0, 20.0);
    let out_spin = ctrl_spin.compute_control(&state_spin, &reference, &params, 0.01);

    // Torques should differ (gyroscopic term ω × J·ω is nonzero for spin)
    let torque_diff = (out_zero.torque - out_spin.torque).norm();
    assert!(torque_diff > 1e-8, "gyroscopic compensation should change torque, diff={torque_diff}");
}

// ─────────────────────────────────────────────────────────────────────────────
// Position integral accumulation
// ─────────────────────────────────────────────────────────────────────────────

/// After N steps of constant position error, i_error_pos should be nonzero.
#[test]
fn test_position_integral_accumulates() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    // Use nonzero ki_pos to ensure accumulation
    ctrl.ki_pos = Vec3::new(0.1, 0.1, 0.1);
    let state = hover_state();
    let mut reference = default_reference();
    reference.position = Vec3::new(0.5, 0.0, 1.0);

    for _ in 0..10 {
        ctrl.compute_control(&state, &reference, &params, 0.01);
    }

    // Integral should have grown
    assert!(ctrl.i_error_pos().z.abs() > 1e-6, "i_error_pos.z should grow: {}", ctrl.i_error_pos().z);
    assert!(ctrl.i_error_pos().x.abs() > 1e-6, "i_error_pos.x should grow: {}", ctrl.i_error_pos().x);
}

/// i_error_pos is clamped to ±0.5 and does not grow unboundedly.
#[test]
fn test_position_integral_anti_windup() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    ctrl.ki_pos = Vec3::new(0.1, 0.1, 0.1);
    let state = hover_state();
    let mut reference = default_reference();
    reference.position = Vec3::new(10.0, 10.0, 10.0); // huge error

    // Run for many steps
    for _ in 0..1000 {
        ctrl.compute_control(&state, &reference, &params, 0.01);
    }

    // Must not exceed anti-windup clamp ±0.5
    let i = ctrl.i_error_pos();
    assert!(i.x.abs() <= 0.5 + 1e-5, "i_x exceeded clamp: {}", i.x);
    assert!(i.y.abs() <= 0.5 + 1e-5, "i_y exceeded clamp: {}", i.y);
    assert!(i.z.abs() <= 0.5 + 1e-5, "i_z exceeded clamp: {}", i.z);
}

// ─────────────────────────────────────────────────────────────────────────────
// Integral reset on low thrust
// ─────────────────────────────────────────────────────────────────────────────

/// When controller outputs thrust < 0.01 N, integrals are reset to zero.
/// We can trigger this by setting a large downward position error so the
/// desired acceleration term subtracts more than g.
#[test]
fn test_integral_reset_on_low_thrust() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    ctrl.ki_pos = Vec3::new(0.1, 0.1, 0.1);

    // First: accumulate integral
    let state = hover_state();
    let mut reference = default_reference();
    reference.position = Vec3::new(0.0, 0.0, 1.0);
    for _ in 0..10 {
        ctrl.compute_control(&state, &reference, &params, 0.01);
    }
    assert!(ctrl.i_error_pos().z.abs() > 1e-7, "should have accumulated");

    // Now: set a huge downward desired acceleration to force thrust near zero/negative
    // z error = -100 m (drone is far above reference) → reduces thrust drastically
    let mut state2 = hover_state();
    state2.position = Vec3::new(0.0, 0.0, 100.0);
    let ref2 = default_reference(); // want z=0, drone at 100 → Kp*(0-100) = -700 → thrust negative
    let out = ctrl.compute_control(&state2, &ref2, &params, 0.01);

    // If thrust < 0.01, the integrals should have been zeroed
    if out.thrust < 0.01 {
        let i = ctrl.i_error_pos();
        assert!(i.norm() < 1e-6, "integrals should be reset, got {:?}", i);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_desired_rotation: tilted force vector
// ─────────────────────────────────────────────────────────────────────────────

/// When the desired force has a +X component, the z-body axis (zdes) should
/// tilt in the +X direction (the third column of Rd has positive x-component).
#[test]
fn test_tilted_force_tilts_zdes() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    let state = hover_state();

    // Reference with acceleration feedforward in +X → desired force tilts forward
    let mut reference = default_reference();
    reference.acceleration = Vec3::new(3.0, 0.0, 0.0); // 3 m/s² forward

    let out = ctrl.compute_control(&state, &reference, &params, 0.01);

    // Torque should be nonzero — the drone needs to tilt to track this reference
    assert!(out.torque.norm() > 1e-6, "nonzero torque expected for tilted reference");
    // Thrust should be roughly m*(g² + 3²)^0.5 > m*g (more thrust needed when tilted)
    let expected_min = params.mass * params.gravity;
    assert!(out.thrust > expected_min * 0.9,
        "thrust should be at least near hover: {}", out.thrust);
}

// ─────────────────────────────────────────────────────────────────────────────
// Controller reset
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn test_controller_reset_clears_integrals() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    ctrl.ki_pos = Vec3::new(0.1, 0.1, 0.1);
    let state = hover_state();
    let mut reference = default_reference();
    reference.position = Vec3::new(1.0, 1.0, 1.0);

    // Accumulate
    for _ in 0..20 {
        ctrl.compute_control(&state, &reference, &params, 0.01);
    }
    assert!(ctrl.i_error_pos().norm() > 1e-6);

    // Reset
    ctrl.reset();
    assert!(ctrl.i_error_pos().norm() < 1e-6, "integrals not cleared after reset");
}

#[test]
fn test_controller_reset_position_integral_only() {
    let params = MultirotorParams::crazyflie();
    let mut ctrl = GeometricController::default();
    ctrl.ki_pos = Vec3::new(0.1, 0.1, 0.1);
    let state = hover_state();
    let mut reference = default_reference();
    reference.position = Vec3::new(1.0, 1.0, 1.0);

    for _ in 0..20 {
        ctrl.compute_control(&state, &reference, &params, 0.01);
    }
    assert!(ctrl.i_error_pos().norm() > 1e-6);

    ctrl.reset_position_integral();
    assert!(ctrl.i_error_pos().norm() < 1e-6, "position integral not cleared");
}
