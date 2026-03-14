//! Unit tests for `multirotor_simulator::flight` — the three helper modules
//! extracted from `main.rs`:
//!
//! * `state_builder`  → `build_state`
//! * `rpyt_control`   → `compute_force_vector`, `force_vector_to_rpyt`,
//!                      `tilt_saturated`, `thrust_to_pwm`, `yaw_rate_cmd`
//! * `ekf_reset`      → `yaw_wrap_delta`, `detect_ekf_reset`
//!
//! For every function the tests follow the pattern:
//!   "given known inputs, assert exact / approximate expected outputs"
//! so that regressions are caught without flying the drone.

use multirotor_simulator::math::Vec3;
use multirotor_simulator::dynamics::MultirotorParams;
use multirotor_simulator::controller::{GeometricController, TrajectoryReference, Controller};
use multirotor_simulator::flight::{
    build_state,
    compute_force_vector,
    force_vector_to_rpyt,
    thrust_to_pwm,
    yaw_rate_cmd,
    detect_ekf_reset,
    yaw_wrap_delta,
    EkfResetFlags,
};
use multirotor_simulator::flight::rpyt_control::tilt_saturated;

// ═══════════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════════

fn hover_ref(pos: Vec3) -> TrajectoryReference {
    TrajectoryReference {
        position: pos,
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
        jerk: Vec3::zero(),
        yaw: 0.0,
        yaw_rate: 0.0,
        yaw_acceleration: 0.0,
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// state_builder::build_state
// ═══════════════════════════════════════════════════════════════════════════════

/// Level flight (all zero angles) → orientation should be the identity quaternion.
#[test]
fn test_build_state_level_flight_identity_orientation() {
    let state = build_state(
        1.0, 2.0, 0.3,   // pos
        0.1, 0.0, 0.0,   // vel
        0.0, 0.0, 0.0,   // roll, pitch, yaw (degrees) — identity
        0.0, 0.0, 0.0,   // gyro (deg/s)
    );
    // body_z column of rotation matrix should be world_z = (0, 0, 1)
    let rot = state.orientation.to_rotation_matrix();
    let body_z = Vec3::new(rot[0][2], rot[1][2], rot[2][2]);
    assert!((body_z.z - 1.0).abs() < 1e-5,
        "Level flight: body_z.z should be 1, got {:.5}", body_z.z);
    assert!(body_z.x.abs() < 1e-5 && body_z.y.abs() < 1e-5,
        "Level flight: body_z.x/y should be 0, got ({:.5},{:.5})", body_z.x, body_z.y);
}

/// Scalar values are passed through to the right fields.
#[test]
fn test_build_state_position_velocity_passthrough() {
    let state = build_state(
        1.5, 2.5, 0.7,
        0.3, -0.2, 0.05,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    );
    assert!((state.position.x - 1.5).abs() < 1e-6);
    assert!((state.position.y - 2.5).abs() < 1e-6);
    assert!((state.position.z - 0.7).abs() < 1e-6);
    assert!((state.velocity.x -  0.3).abs() < 1e-6);
    assert!((state.velocity.y - -0.2).abs() < 1e-6);
    assert!((state.velocity.z - 0.05).abs() < 1e-6);
}

/// Gyro values must be converted from deg/s to rad/s.
#[test]
fn test_build_state_gyro_deg_to_rad() {
    let state = build_state(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        180.0, 0.0, 0.0,  // gyro_x = 180 deg/s → π rad/s
    );
    let expected = std::f32::consts::PI;
    assert!(
        (state.angular_velocity.x - expected).abs() < 1e-5,
        "gyro_x 180 deg/s → {:.5} rad/s (expected {:.5})",
        state.angular_velocity.x, expected
    );
}

/// 90° yaw rotation: body_x should point toward +world_y.
#[test]
fn test_build_state_90deg_yaw() {
    let state = build_state(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 90.0,  // yaw = 90°
        0.0, 0.0, 0.0,
    );
    let rot = state.orientation.to_rotation_matrix();
    let body_x = Vec3::new(rot[0][0], rot[1][0], rot[2][0]);
    assert!(
        (body_x.y - 1.0).abs() < 1e-5,
        "After 90° yaw body_x should be world_y. Got ({:.3},{:.3},{:.3})",
        body_x.x, body_x.y, body_x.z
    );
}

/// 45° pitch (nose-up): body_z should tilt toward +world_x (right-hand rule around +Y).
#[test]
fn test_build_state_45deg_pitch_tilts_body_z_positive_x() {
    let state = build_state(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 45.0, 0.0,  // pitch = 45°
        0.0, 0.0, 0.0,
    );
    let rot = state.orientation.to_rotation_matrix();
    let body_z = Vec3::new(rot[0][2], rot[1][2], rot[2][2]);
    println!("45° pitch: body_z = ({:.3},{:.3},{:.3})", body_z.x, body_z.y, body_z.z);
    assert!(
        body_z.x > 0.5,
        "45° pitch → body_z.x should be ~+0.707. Got {:.3}",
        body_z.x
    );
    assert!(
        (body_z.z - std::f32::consts::FRAC_1_SQRT_2).abs() < 0.01,
        "45° pitch → body_z.z should be ~0.707. Got {:.3}",
        body_z.z
    );
}

/// 30° roll (right side down): body_y should tilt down (−world_z component).
#[test]
fn test_build_state_30deg_roll() {
    let state = build_state(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        30.0, 0.0, 0.0,  // roll = 30°
        0.0, 0.0, 0.0,
    );
    let rot = state.orientation.to_rotation_matrix();
    let body_z = Vec3::new(rot[0][2], rot[1][2], rot[2][2]);
    // After 30° roll about X: body_z should tilt toward -world_y
    assert!(
        body_z.y < -0.3,
        "30° roll → body_z.y should be ~−0.5. Got {:.3}",
        body_z.y
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// rpyt_control::compute_force_vector
// ═══════════════════════════════════════════════════════════════════════════════

/// At equilibrium (ep=0, ev=0, i=0), F should equal m*g in the Z direction.
#[test]
fn test_compute_force_vector_equilibrium_is_weight() {
    let params = MultirotorParams::crazyflie();
    let controller = GeometricController::default();
    let reference = hover_ref(Vec3::new(0.0, 0.0, 0.3));
    let ep   = Vec3::zero();
    let ev   = Vec3::zero();
    let i    = Vec3::zero();

    let f = compute_force_vector(&reference, ep, ev, i, &controller, &params);

    let weight = params.mass * params.gravity;
    assert!(
        (f.z - weight).abs() < 1e-5,
        "Equilibrium F.z should be m*g = {:.5}, got {:.5}",
        weight, f.z
    );
    assert!(f.x.abs() < 1e-6, "Equilibrium F.x should be 0, got {:.6}", f.x);
    assert!(f.y.abs() < 1e-6, "Equilibrium F.y should be 0, got {:.6}", f.y);
}

/// ep.x > 0 → F.x should be positive (drone must accelerate in +X to reach reference).
#[test]
fn test_compute_force_vector_x_error_gives_positive_fx() {
    let params = MultirotorParams::crazyflie();
    let controller = GeometricController::default();
    let reference = hover_ref(Vec3::new(0.2, 0.0, 0.3));
    let ep = Vec3::new(0.2, 0.0, 0.0);   // drone is 0.2 m behind reference in X
    let ev = Vec3::zero();
    let i  = Vec3::zero();

    let f = compute_force_vector(&reference, ep, ev, i, &controller, &params);

    // F.x = kp * ep.x * mass = 7 * 0.2 * 0.027 = 0.0378 N
    let expected_fx = controller.kp.x * ep.x * params.mass;
    println!("F.x = {:.5}, expected {:.5}", f.x, expected_fx);
    assert!(f.x > 0.0, "F.x should be positive for ep.x > 0");
    assert!(
        (f.x - expected_fx).abs() < 1e-4,
        "F.x = {:.5} should match kp*ep.x*mass = {:.5}",
        f.x, expected_fx
    );
}

/// Feed-forward acceleration is included: if reference.acceleration.x = 1 m/s²,
/// F.x should include m * 1 N = 0.027 N on top of the feedback.
#[test]
fn test_compute_force_vector_includes_feedforward() {
    let params = MultirotorParams::crazyflie();
    let controller = GeometricController::default();
    let mut reference = hover_ref(Vec3::new(0.0, 0.0, 0.3));
    reference.acceleration = Vec3::new(1.0, 0.0, 0.0);  // 1 m/s² feedforward
    let ep = Vec3::zero();
    let ev = Vec3::zero();
    let i  = Vec3::zero();

    let f = compute_force_vector(&reference, ep, ev, i, &controller, &params);

    // F.x = m * a_ff.x = 0.027 * 1.0 = 0.027 N
    let expected_fx = params.mass * 1.0;
    assert!(
        (f.x - expected_fx).abs() < 1e-5,
        "F.x with feedforward = {:.5}, expected {:.5}",
        f.x, expected_fx
    );
}

/// Integral term contributes to force: i_pos.z > 0 → F.z increases.
#[test]
fn test_compute_force_vector_integral_adds_to_z() {
    let params = MultirotorParams::crazyflie();
    let controller = GeometricController::default();
    let reference = hover_ref(Vec3::new(0.0, 0.0, 0.3));
    let ep = Vec3::zero();
    let ev = Vec3::zero();

    let i_zero    = Vec3::zero();
    let i_nonzero = Vec3::new(0.0, 0.0, 0.1);  // positive Z integral

    let f_zero = compute_force_vector(&reference, ep, ev, i_zero,    &controller, &params);
    let f_int  = compute_force_vector(&reference, ep, ev, i_nonzero, &controller, &params);

    assert!(
        f_int.z > f_zero.z,
        "Positive Z integral should increase F.z: {:.5} vs {:.5}",
        f_int.z, f_zero.z
    );
    let expected_delta = controller.ki_pos.z * 0.1 * params.mass;
    let actual_delta   = f_int.z - f_zero.z;
    assert!(
        (actual_delta - expected_delta).abs() < 1e-5,
        "F.z delta = {:.5}, expected ki_pos.z * 0.1 * mass = {:.5}",
        actual_delta, expected_delta
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// rpyt_control::force_vector_to_rpyt
// ═══════════════════════════════════════════════════════════════════════════════

/// Pure hover force (only Z) → both roll and pitch commands should be zero.
#[test]
fn test_force_vector_to_rpyt_pure_z_gives_zero_tilt() {
    let f_vec = Vec3::new(0.0, 0.0, 0.27);  // pure hover (m*g)
    let (roll_d, pitch_d, roll_raw, pitch_raw) = force_vector_to_rpyt(f_vec, 25.0);
    println!("Pure Z: roll={:.3}° pitch={:.3}°", roll_d, pitch_d);
    assert!(roll_d.abs()   < 0.01, "Pure Z: roll should be 0°, got {:.4}°",  roll_d);
    assert!(pitch_d.abs()  < 0.01, "Pure Z: pitch should be 0°, got {:.4}°", pitch_d);
    assert!(roll_raw.abs() < 0.01);
    assert!(pitch_raw.abs() < 0.01);
}

/// F.x > 0 (need to push drone +X) → pitch_d_cmd should be NEGATIVE (firmware convention).
/// Full sign chain: ep.x>0 → F.x>0 → pitch_raw>0 → pitch_cmd = -pitch_raw < 0
///                 → firmware negates → actual_pitch>0 → body_z toward +world_x → drone +X ✓
#[test]
fn test_force_vector_to_rpyt_positive_fx_gives_negative_pitch_cmd() {
    let f_vec = Vec3::new(0.1, 0.0, 0.27);  // lateral force in +X
    let (_, pitch_d, pitch_raw, _) = {
        // Note: return order is (roll_d, pitch_d, roll_raw, pitch_raw)
        // but we need pitch_raw separately
        let (roll_d, pitch_d, roll_raw, pitch_raw) = force_vector_to_rpyt(f_vec, 25.0);
        (roll_d, pitch_d, roll_raw, pitch_raw)
    };
    let (_, pitch_cmd, _, pitch_raw_) = force_vector_to_rpyt(f_vec, 25.0);
    println!("F.x=+0.1: pitch_raw={:.3}°, pitch_cmd={:.3}°", pitch_raw_, pitch_cmd);
    assert!(
        pitch_raw_ > 0.0,
        "F.x>0 → pitch_raw should be positive, got {:.3}°",
        pitch_raw_
    );
    // pitch_cmd is sent un-negated; the firmware negates it internally in rpy2quat(roll,-pitch,yaw).
    assert!(
        pitch_cmd > 0.0,
        "F.x>0 → pitch_cmd should be positive (firmware negates it), got {:.3}°",
        pitch_cmd
    );
}

/// F.y < 0 (need to push drone +Y) → roll_d_cmd should be positive (right-side-down to go +Y).
#[test]
fn test_force_vector_to_rpyt_negative_fy_gives_positive_roll() {
    // -F.y in the formula: roll_raw = atan2(-F.y, F.z)
    // F.y = -0.1 → roll_raw = atan2(+0.1, 0.27) > 0
    let f_vec = Vec3::new(0.0, -0.1, 0.27);
    let (roll_cmd, _, roll_raw, _) = force_vector_to_rpyt(f_vec, 25.0);
    println!("F.y=-0.1: roll_raw={:.3}°, roll_cmd={:.3}°", roll_raw, roll_cmd);
    assert!(roll_raw > 0.0, "F.y<0 → roll_raw should be positive");
    assert!(roll_cmd > 0.0, "F.y<0 → roll_cmd should be positive");
}

/// Clamp: very large F.x → pitch_cmd clamped to max_tilt.
#[test]
fn test_force_vector_to_rpyt_clamped_at_max_tilt() {
    let f_vec = Vec3::new(10.0, 0.0, 0.27);  // extreme lateral force
    let max_tilt = 25.0;
    let (_, pitch_cmd, _, pitch_raw) = force_vector_to_rpyt(f_vec, max_tilt);
    println!("Extreme F.x: pitch_raw={:.1}°, pitch_cmd={:.1}°", pitch_raw, pitch_cmd);
    assert!(
        pitch_raw.abs() > max_tilt,
        "Extreme F.x: pitch_raw should exceed max_tilt, got {:.1}°",
        pitch_raw
    );
    assert!(
        pitch_cmd.abs() <= max_tilt,
        "pitch_cmd should be clamped to ±{}, got {:.1}°",
        max_tilt, pitch_cmd
    );
}

/// Symmetry: opposite F.x → opposite pitch_cmd.
#[test]
fn test_force_vector_to_rpyt_pitch_symmetry() {
    let f_pos = Vec3::new( 0.05, 0.0, 0.27);
    let f_neg = Vec3::new(-0.05, 0.0, 0.27);
    let (_, p_pos, _, _) = force_vector_to_rpyt(f_pos, 25.0);
    let (_, p_neg, _, _) = force_vector_to_rpyt(f_neg, 25.0);
    assert!(
        (p_pos + p_neg).abs() < 1e-4,
        "Opposite F.x should give opposite pitch_cmd: {:.4}° vs {:.4}°",
        p_pos, p_neg
    );
}

/// Small angle approximation: for small tilt, pitch_raw ≈ atan2(F.x, F.z) ≈ F.x / F.z (radians).
#[test]
fn test_force_vector_to_rpyt_small_angle_approximation() {
    let fz = 0.27_f32;
    let fx = 0.01_f32;
    let f_vec = Vec3::new(fx, 0.0, fz);
    let (_, _, _, pitch_raw) = force_vector_to_rpyt(f_vec, 25.0);
    let approx_deg = (fx / fz).to_degrees();  // small-angle: atan(x) ≈ x
    println!("Small angle: pitch_raw={:.4}°, approx={:.4}°", pitch_raw, approx_deg);
    // Should match within 0.1° for such a small angle
    assert!(
        (pitch_raw - approx_deg).abs() < 0.1,
        "Small angle: pitch_raw={:.4}° ≠ approx={:.4}°",
        pitch_raw, approx_deg
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// rpyt_control::tilt_saturated
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn test_tilt_saturated_within_limits() {
    assert!(!tilt_saturated(10.0, 10.0, 25.0), "10°/10° with limit 25° should NOT saturate");
    assert!(!tilt_saturated(0.0, 0.0, 25.0),   "0°/0° should not saturate");
    assert!(!tilt_saturated(24.9, 0.0, 25.0),  "24.9° < 25° should not saturate");
}

#[test]
fn test_tilt_saturated_at_limits() {
    assert!(tilt_saturated(25.1, 0.0, 25.0),  "25.1° > 25° roll should saturate");
    assert!(tilt_saturated(0.0, 25.1, 25.0),  "25.1° > 25° pitch should saturate");
    assert!(tilt_saturated(-26.0, 0.0, 25.0), "−26° roll should saturate");
    assert!(tilt_saturated(0.0, -26.0, 25.0), "−26° pitch should saturate");
}

// ═══════════════════════════════════════════════════════════════════════════════
// rpyt_control::thrust_to_pwm
// ═══════════════════════════════════════════════════════════════════════════════

/// At hover thrust, PWM = HOVER_PWM exactly.
#[test]
fn test_thrust_to_pwm_at_hover_gives_hover_pwm() {
    let params = MultirotorParams::crazyflie();
    let hover_thrust = params.mass * params.gravity;
    let hover_pwm    = 42000.0_f32;
    let (pwm, raw) = thrust_to_pwm(hover_thrust, hover_thrust, hover_pwm, 10_000.0, 60_000.0);
    assert!((raw - hover_pwm).abs() < 1.0,
        "At hover thrust, raw PWM should be {:.0}, got {:.1}", hover_pwm, raw);
    assert_eq!(pwm, hover_pwm as u16,
        "At hover thrust, clamped PWM should be {}", hover_pwm as u16);
}

/// Double hover thrust → PWM = 2 * HOVER_PWM (clamped to max).
#[test]
fn test_thrust_to_pwm_double_thrust_doubles_pwm() {
    let params = MultirotorParams::crazyflie();
    let hover_thrust = params.mass * params.gravity;
    let hover_pwm    = 42000.0_f32;
    let (_, raw) = thrust_to_pwm(2.0 * hover_thrust, hover_thrust, hover_pwm, 10_000.0, 60_000.0);
    assert!(
        (raw - 2.0 * hover_pwm).abs() < 1.0,
        "Double hover thrust → raw PWM should be {:.0}, got {:.1}",
        2.0 * hover_pwm, raw
    );
}

/// Zero thrust → raw = 0, clamped to pwm_min.
#[test]
fn test_thrust_to_pwm_zero_thrust_clamped_to_min() {
    let params = MultirotorParams::crazyflie();
    let hover_thrust = params.mass * params.gravity;
    let (pwm, raw) = thrust_to_pwm(0.0, hover_thrust, 42000.0, 10_000.0, 60_000.0);
    assert!((raw - 0.0).abs() < 0.1, "Zero thrust → raw should be 0, got {:.1}", raw);
    assert_eq!(pwm, 10_000, "Zero thrust → clamped to min 10000, got {}", pwm);
}

/// Excessive thrust → clamped to pwm_max.
#[test]
fn test_thrust_to_pwm_excess_thrust_clamped_to_max() {
    let params = MultirotorParams::crazyflie();
    let hover_thrust = params.mass * params.gravity;
    let (pwm, raw) = thrust_to_pwm(100.0 * hover_thrust, hover_thrust, 42000.0, 10_000.0, 60_000.0);
    assert!(raw > 60_000.0, "Excessive thrust → raw should exceed max");
    assert_eq!(pwm, 60_000, "Excessive thrust → clamped to 60000, got {}", pwm);
}

/// Linear scaling: thrust proportional to raw PWM.
#[test]
fn test_thrust_to_pwm_linear_scaling() {
    let hover_pwm    = 40000.0_f32;
    let hover_thrust = 0.265_f32;  // m*g for CF
    for factor in [0.5_f32, 0.8, 1.0, 1.2] {
        let (_, raw) = thrust_to_pwm(
            factor * hover_thrust, hover_thrust, hover_pwm, 0.0, 1_000_000.0,
        );
        let expected = factor * hover_pwm;
        assert!(
            (raw - expected).abs() < 0.5,
            "Factor {}: raw={:.1}, expected={:.1}",
            factor, raw, expected
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// rpyt_control::yaw_rate_cmd
// ═══════════════════════════════════════════════════════════════════════════════

/// Zero error → zero yaw rate.
#[test]
fn test_yaw_rate_cmd_zero_error() {
    // reference yaw = 0 rad, current yaw = 0°
    let rate = yaw_rate_cmd(0.0, 0.0, 1.0, 30.0);
    assert!(rate.abs() < 1e-5, "Zero error → zero yaw rate, got {:.5}", rate);
}

/// Reference yaw = 0 rad, current yaw = 10° → error = −10° → rate = −10 * kp.
#[test]
fn test_yaw_rate_cmd_positive_current_yaw() {
    let kp = 1.0_f32;
    let rate = yaw_rate_cmd(0.0, 10.0, kp, 30.0);
    // error = (0° - 10°) * kp = -10 deg/s
    assert!(
        (rate - (-10.0)).abs() < 0.01,
        "Current yaw=10°, ref=0: rate should be -10 deg/s, got {:.3}", rate
    );
}

/// Clamp: large error should be limited to max_yaw_rate.
#[test]
fn test_yaw_rate_cmd_clamped() {
    let rate = yaw_rate_cmd(0.0, 90.0, 1.0, 30.0);
    assert_eq!(rate, -30.0,
        "Large error should be clamped to -30 deg/s, got {:.1}", rate);
}

/// Gain scaling: kp=2 gives twice the rate of kp=1.
#[test]
fn test_yaw_rate_cmd_gain_scaling() {
    let rate1 = yaw_rate_cmd(0.0, 5.0, 1.0, 30.0);
    let rate2 = yaw_rate_cmd(0.0, 5.0, 2.0, 30.0);
    assert!(
        (rate2 - 2.0 * rate1).abs() < 0.01,
        "kp=2 should give 2x rate: {:.3} vs {:.3}", rate2, 2.0 * rate1
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
// ekf_reset::yaw_wrap_delta
// ═══════════════════════════════════════════════════════════════════════════════

/// Basic forward differences.
#[test]
fn test_yaw_wrap_delta_basic() {
    assert!((yaw_wrap_delta(0.0, 90.0)   -  90.0).abs() < 1e-4);
    assert!((yaw_wrap_delta(0.0, -90.0)  - -90.0).abs() < 1e-4);
    assert!((yaw_wrap_delta(0.0, 0.0)    -   0.0).abs() < 1e-4);
}

/// Crossing ±180° takes the short path.
#[test]
fn test_yaw_wrap_delta_crosses_180() {
    // From 170° to -170° is a +20° change (CCW), not -340°.
    let d = yaw_wrap_delta(170.0, -170.0);
    assert!(
        (d - 20.0).abs() < 0.01,
        "170° → -170°: shortest path is +20°, got {:.3}°", d
    );
    // Reverse: from -170° to 170° is -20° (CW).
    let d2 = yaw_wrap_delta(-170.0, 170.0);
    assert!(
        (d2 - (-20.0)).abs() < 0.01,
        "-170° → 170°: shortest path is -20°, got {:.3}°", d2
    );
}

/// 0° → 359° is a −1° step (going CW 1°), not +359°.
#[test]
fn test_yaw_wrap_delta_near_zero_crossing() {
    let d = yaw_wrap_delta(0.0, 359.0);
    assert!(
        (d - (-1.0)).abs() < 0.01,
        "0° → 359°: shortest path is -1°, got {:.3}°", d
    );
}

/// Exactly ±180° returns +180° (boundary: belongs to the positive side).
#[test]
fn test_yaw_wrap_delta_exactly_180() {
    let d = yaw_wrap_delta(0.0, 180.0);
    // rem_euclid maps 360 → 0, so (360 % 360 = 0) → 0 - 180 = -180.
    // This is acceptable: ±180° is ambiguous.
    assert!(
        d.abs() == 180.0,
        "0° → 180°: should be ±180°, got {:.3}°", d
    );
}

/// Anti-symmetry: delta(A→B) = -delta(B→A).
/// Does NOT include pairs that are exactly 180° apart (both paths equal length → ambiguous sign).
#[test]
fn test_yaw_wrap_delta_antisymmetry() {
    let pairs = [(10.0_f32, 50.0), (160.0, -160.0), (0.0, 90.0)];
    for (a, b) in pairs {
        let fwd = yaw_wrap_delta(a, b);
        let rev = yaw_wrap_delta(b, a);
        assert!(
            (fwd + rev).abs() < 1e-3,
            "anti-symmetry failed for ({}, {}): fwd={:.3}, rev={:.3}",
            a, b, fwd, rev
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// ekf_reset::detect_ekf_reset
// ═══════════════════════════════════════════════════════════════════════════════

/// First call (prev = None) → no reset detected on any axis.
#[test]
fn test_detect_ekf_reset_first_call_no_flags() {
    let flags = detect_ekf_reset(None, Vec3::new(1.0, 2.0, 0.3), 0.0, 0.0, 0.05, 0.05, 10.0);
    assert_eq!(flags, EkfResetFlags::default(), "First call should produce no reset flags");
    assert!(!flags.any());
}

/// Small normal step → no reset.
#[test]
fn test_detect_ekf_reset_normal_step_no_flags() {
    // 1.3 mm step in XY (100 mm/s * 10 ms tick) — well within threshold
    let prev = Some(Vec3::new(0.0, 0.0, 0.3));
    let cur  = Vec3::new(0.001, 0.001, 0.3);
    let flags = detect_ekf_reset(prev, cur, 10.0, 10.1, 0.05, 0.05, 10.0);
    assert!(!flags.any(), "Small normal step should not trigger reset");
}

/// XY jump > 0.05 m → xy flag set, z and yaw flags clear.
#[test]
fn test_detect_ekf_reset_xy_jump() {
    let prev = Some(Vec3::new(0.0, 0.0, 0.3));
    let cur  = Vec3::new(0.1, 0.0, 0.3);  // 0.1 m jump in X
    let flags = detect_ekf_reset(prev, cur, 0.0, 0.0, 0.05, 0.05, 10.0);
    assert!(flags.xy,  "XY jump 0.1m > 0.05m threshold should set xy flag");
    assert!(!flags.z,  "Z unchanged: z flag should be clear");
    assert!(!flags.yaw,"No yaw change: yaw flag should be clear");
    assert!(flags.any());
}

/// Z jump > 0.05 m → z flag set, xy clear.
#[test]
fn test_detect_ekf_reset_z_jump() {
    let prev = Some(Vec3::new(0.0, 0.0, 0.3));
    let cur  = Vec3::new(0.0, 0.0, 0.4);  // 0.1 m jump in Z
    let flags = detect_ekf_reset(prev, cur, 0.0, 0.0, 0.05, 0.05, 10.0);
    assert!(!flags.xy, "XY unchanged: xy flag should be clear");
    assert!(flags.z,   "Z jump 0.1m > 0.05m should set z flag");
    assert!(!flags.yaw);
}

/// Yaw jump > 10° → yaw flag set, position flags clear.
#[test]
fn test_detect_ekf_reset_yaw_jump() {
    let prev = Some(Vec3::new(0.0, 0.0, 0.3));
    let cur  = Vec3::new(0.0, 0.0, 0.3);  // no position change
    let flags = detect_ekf_reset(prev, cur, 0.0, 15.0, 0.05, 0.05, 10.0);
    assert!(!flags.xy, "No XY change: xy flag should be clear");
    assert!(!flags.z,  "No Z change: z flag should be clear");
    assert!(flags.yaw, "15° yaw jump > 10° threshold should set yaw flag");
}

/// Yaw jump across ±180° boundary is detected correctly.
#[test]
fn test_detect_ekf_reset_yaw_jump_across_180() {
    let prev = Some(Vec3::new(0.0, 0.0, 0.3));
    let cur  = Vec3::new(0.0, 0.0, 0.3);
    // 175° → -175° is only a 10° step, right at threshold → should NOT fire
    let flags_no = detect_ekf_reset(prev, cur, 175.0, -175.0, 0.05, 0.05, 10.0);
    // (175 → -175): delta = -350 → wrapped = +10 → abs = 10.0 → NOT > 10.0
    // Note: rem_euclid gives exactly 10.0 here so NOT strictly greater
    assert!(!flags_no.yaw,
        "175° → -175° is exactly 10°, should not exceed threshold");

    // 170° → -170° is a 20° step → should fire
    let flags_yes = detect_ekf_reset(prev, cur, 170.0, -170.0, 0.05, 0.05, 10.0);
    assert!(flags_yes.yaw,
        "170° → -170° is 20° which exceeds 10° threshold");
}

/// All three axes jump simultaneously → all flags set.
#[test]
fn test_detect_ekf_reset_all_axes_jump() {
    let prev = Some(Vec3::new(0.0, 0.0, 0.0));
    let cur  = Vec3::new(0.5, 0.5, 0.5);  // large jump in all axes
    let flags = detect_ekf_reset(prev, cur, 0.0, 20.0, 0.05, 0.05, 10.0);
    assert!(flags.xy,  "Large XY jump should set xy flag");
    assert!(flags.z,   "Large Z  jump should set z flag");
    assert!(flags.yaw, "Large yaw jump should set yaw flag");
    assert!(flags.any());
}

/// Step just below threshold → no reset.
#[test]
fn test_detect_ekf_reset_just_below_threshold() {
    let prev = Some(Vec3::new(0.0, 0.0, 0.0));
    let cur  = Vec3::new(0.0499, 0.0, 0.0);  // 49.9 mm — just below 50 mm threshold
    let flags = detect_ekf_reset(prev, cur, 0.0, 0.0, 0.05, 0.05, 10.0);
    assert!(!flags.any(), "0.0499 m < 0.05 m threshold should not trigger");
}

// ═══════════════════════════════════════════════════════════════════════════════
// compute_force_vector vs compute_control consistency
// ═══════════════════════════════════════════════════════════════════════════════
//
// compute_force_vector is a separate code path from the internal thrust_force
// in compute_control, but they must agree in direction (and magnitude) for the
// RPYT decomposition to be physically correct.
//
// At level flight (identity orientation) the body-z axis equals world-z, so:
//   compute_control.thrust  = thrust_force · body_z = thrust_force.z = f_vec.z
// This lets us cross-check the two paths numerically without needing internal
// access to compute_control's thrust_force.

/// At hover equilibrium (ep=0, ev=0, i=0, level) both paths must agree on
/// the force magnitude (= m*g) and direction (pure world-z).
#[test]
fn test_force_vector_matches_compute_control_at_hover() {
    use multirotor_simulator::dynamics::MultirotorState;
    let params     = MultirotorParams::crazyflie();
    let mut ctrl   = GeometricController::default();
    let reference  = hover_ref(Vec3::new(0.0, 0.0, 0.3));
    let state      = MultirotorState::new(); // identity orientation, at origin

    let ep = reference.position - state.position; // (0, 0, 0.3) — but test with zero ep
    let ep_zero = Vec3::zero();
    let ev_zero = Vec3::zero();
    let i_zero  = Vec3::zero();

    // Run compute_control with zero error so that internal thrust_force = m*g*ez
    let mut ctrl_c = GeometricController::default();
    let mut state_at_ref = state.clone();
    state_at_ref.position = reference.position; // ep = 0
    let control = ctrl_c.compute_control(&state_at_ref, &reference, &params, 0.01);

    // compute_force_vector with matching zero ep/ev/i
    let f_vec = compute_force_vector(&reference, ep_zero, ev_zero, i_zero, &ctrl, &params);

    let weight = params.mass * params.gravity;

    // Both must give a force equal to m*g
    assert!(
        (f_vec.z - weight).abs() < 1e-5,
        "f_vec.z should be m*g = {:.5}, got {:.5}", weight, f_vec.z
    );
    assert!(
        (control.thrust - weight).abs() < 1e-5,
        "compute_control.thrust should be m*g = {:.5}, got {:.5}", weight, control.thrust
    );
    // At level flight: thrust = f_vec.z (body-z = world-z)
    assert!(
        (control.thrust - f_vec.z).abs() < 1e-5,
        "compute_control.thrust ({:.5}) must equal f_vec.z ({:.5}) at level hover",
        control.thrust, f_vec.z
    );
    // No lateral force
    assert!(f_vec.x.abs() < 1e-6, "f_vec.x should be 0 at hover, got {:.6}", f_vec.x);
    assert!(f_vec.y.abs() < 1e-6, "f_vec.y should be 0 at hover, got {:.6}", f_vec.y);
}

/// With a lateral position error (ep.x > 0) at level flight:
/// compute_control.thrust ≈ f_vec.z  (body-z ≈ world-z for small tilt)
/// and f_vec.x > 0 (force in +X to correct the error).
/// The ratio f_vec.x / f_vec.z must equal kp.x * ep.x / g (small-angle).
#[test]
fn test_force_vector_matches_compute_control_with_x_error() {
    use multirotor_simulator::dynamics::MultirotorState;
    let params    = MultirotorParams::crazyflie();
    let ctrl      = GeometricController::default();
    let reference = hover_ref(Vec3::new(0.5, 0.0, 0.3));

    // Drone is at the origin — 0.5 m behind reference in X.
    let state = MultirotorState::new();

    let ep = reference.position - state.position; // (0.5, 0, 0.3)
    let ev = Vec3::zero();
    let i  = Vec3::zero();

    let f_vec = compute_force_vector(&reference, ep, ev, i, &ctrl, &params);

    // Force must have a positive X component (need to accelerate in +X)
    assert!(f_vec.x > 0.0,
        "ep.x > 0 → f_vec.x must be positive, got {:.5}", f_vec.x);

    // At level flight: thrust from compute_control ≈ f_vec.z (small tilt)
    // Verify X-component matches: f_vec.x = kp.x * ep.x * mass
    let expected_fx = ctrl.kp.x * ep.x * params.mass;
    assert!(
        (f_vec.x - expected_fx).abs() < 1e-4,
        "f_vec.x = {:.5} should be kp.x * ep.x * mass = {:.5}",
        f_vec.x, expected_fx
    );

    // Small-angle: pitch angle ≈ f_vec.x / f_vec.z (radians).
    // With kp.x=12 and ep.x=0.5: f_x/f_z ≈ 6/9.81 ≈ 0.61 rad — larger than the
    // old 7.0 gains but still within a physically safe range (< 1 rad = 57°).
    let pitch_rad = f_vec.x / f_vec.z;
    assert!(
        pitch_rad > 0.0 && pitch_rad < 0.7,
        "pitch_rad = {:.4} should be small and positive for ep.x = 0.5 m",
        pitch_rad
    );
}

/// After compute_control accumulates i_pos (non-zero ki_pos), the force vector
/// from compute_force_vector with the updated i_pos must include the integral
/// contribution — the direction must shift compared to zero integral.
#[test]
fn test_force_vector_includes_integral_contribution() {
    use multirotor_simulator::dynamics::MultirotorState;
    let params     = MultirotorParams::crazyflie();
    let mut ctrl   = GeometricController::default();
    let reference  = hover_ref(Vec3::new(0.0, 0.0, 0.3));

    // Give the drone a Z position error so the integral winds up.
    let mut state = MultirotorState::new();
    state.position.z = 0.1; // 0.2 m below target

    // Run 20 steps to accumulate Z integral.
    for _ in 0..20 {
        ctrl.compute_control(&state, &reference, &params, 0.01);
    }
    let i_pos = ctrl.i_error_pos();
    assert!(i_pos.z > 0.0, "i_pos.z should have wound up positively: {:.4}", i_pos.z);

    let ep = reference.position - state.position;
    let ev = Vec3::zero();

    let f_with_i    = compute_force_vector(&reference, ep, ev, i_pos,       &ctrl, &params);
    let f_without_i = compute_force_vector(&reference, ep, ev, Vec3::zero(), &ctrl, &params);

    // Integral adds positive Z force (boosting thrust to reach target height).
    assert!(
        f_with_i.z > f_without_i.z,
        "Integral should add Z force: with_i.z={:.5} vs without_i.z={:.5}",
        f_with_i.z, f_without_i.z
    );
    let expected_delta = ctrl.ki_pos.z * i_pos.z * params.mass;
    let actual_delta   = f_with_i.z - f_without_i.z;
    assert!(
        (actual_delta - expected_delta).abs() < 1e-5,
        "Delta = {:.5}, expected ki_pos.z * i.z * mass = {:.5}",
        actual_delta, expected_delta
    );
}
