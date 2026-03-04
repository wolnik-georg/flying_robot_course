//! Tests for `safety.rs`: clamp_position, clamp_velocity, check_safety.

use multirotor_simulator::safety::{SafetyLimits, check_safety};
use multirotor_simulator::math::Vec3;

fn default_limits() -> SafetyLimits {
    SafetyLimits {
        min_altitude: 0.1,
        max_altitude: 2.0,
        max_speed: 1.5,
        x_min: -2.0,
        x_max: 2.0,
        y_min: -2.0,
        y_max: 2.0,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// clamp_position
// ─────────────────────────────────────────────────────────────────────────────

/// Position inside all limits → unchanged.
#[test]
fn test_clamp_position_inside() {
    let limits = default_limits();
    let pos = Vec3::new(0.5, -0.5, 1.0);
    let clamped = limits.clamp_position(pos);
    assert!((clamped.x - 0.5).abs() < 1e-6);
    assert!((clamped.y - -0.5).abs() < 1e-6);
    assert!((clamped.z - 1.0).abs() < 1e-6);
}

/// Position below min_altitude → clamped to min_altitude.
#[test]
fn test_clamp_position_below_floor() {
    let limits = default_limits();
    let pos = Vec3::new(0.0, 0.0, 0.0); // below 0.1
    let clamped = limits.clamp_position(pos);
    assert!((clamped.z - 0.1).abs() < 1e-6, "z={}", clamped.z);
}

/// Position above max_altitude → clamped to max_altitude.
#[test]
fn test_clamp_position_above_ceiling() {
    let limits = default_limits();
    let pos = Vec3::new(0.0, 0.0, 5.0); // above 2.0
    let clamped = limits.clamp_position(pos);
    assert!((clamped.z - 2.0).abs() < 1e-6, "z={}", clamped.z);
}

/// Position beyond x_max → clamped.
#[test]
fn test_clamp_position_x_geofence() {
    let limits = default_limits();
    let pos = Vec3::new(10.0, 0.0, 1.0);
    let clamped = limits.clamp_position(pos);
    assert!((clamped.x - 2.0).abs() < 1e-6, "x={}", clamped.x);
    assert!((clamped.z - 1.0).abs() < 1e-6); // z unchanged
}

/// Position below x_min → clamped.
#[test]
fn test_clamp_position_x_geofence_negative() {
    let limits = default_limits();
    let pos = Vec3::new(-10.0, 0.0, 1.0);
    let clamped = limits.clamp_position(pos);
    assert!((clamped.x - -2.0).abs() < 1e-6, "x={}", clamped.x);
}

/// Position beyond y_max → clamped.
#[test]
fn test_clamp_position_y_geofence() {
    let limits = default_limits();
    let pos = Vec3::new(0.0, 5.0, 1.0);
    let clamped = limits.clamp_position(pos);
    assert!((clamped.y - 2.0).abs() < 1e-6, "y={}", clamped.y);
}

/// All limits violated simultaneously → all components clamped.
#[test]
fn test_clamp_position_all_violated() {
    let limits = default_limits();
    let pos = Vec3::new(-100.0, 100.0, -50.0);
    let clamped = limits.clamp_position(pos);
    assert!((clamped.x - -2.0).abs() < 1e-6);
    assert!((clamped.y - 2.0).abs() < 1e-6);
    assert!((clamped.z - 0.1).abs() < 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
// clamp_velocity
// ─────────────────────────────────────────────────────────────────────────────

/// Velocity below max_speed → unchanged.
#[test]
fn test_clamp_velocity_under_limit() {
    let limits = default_limits();
    let vel = Vec3::new(0.5, 0.5, 0.0); // |v| ≈ 0.707 < 1.5
    let clamped = limits.clamp_velocity(vel);
    assert!((clamped.x - 0.5).abs() < 1e-6);
    assert!((clamped.y - 0.5).abs() < 1e-6);
}

/// Velocity exactly at max_speed → unchanged.
#[test]
fn test_clamp_velocity_at_limit() {
    let limits = default_limits();
    let vel = Vec3::new(1.5, 0.0, 0.0); // |v| = 1.5
    let clamped = limits.clamp_velocity(vel);
    assert!((clamped.x - 1.5).abs() < 1e-5);
    assert!(clamped.y.abs() < 1e-6);
}

/// Velocity above max_speed → direction preserved, magnitude clamped.
#[test]
fn test_clamp_velocity_over_limit_direction_preserved() {
    let limits = default_limits();
    let vel = Vec3::new(3.0, 0.0, 0.0); // |v| = 3.0, 2× max
    let clamped = limits.clamp_velocity(vel);
    let speed = clamped.norm();
    assert!((speed - 1.5).abs() < 1e-5, "speed={speed}");
    // Direction should still be +x
    assert!(clamped.x > 0.0);
    assert!(clamped.y.abs() < 1e-6);
    assert!(clamped.z.abs() < 1e-6);
}

/// Diagonal velocity above limit → direction and magnitude correct.
#[test]
fn test_clamp_velocity_diagonal_over_limit() {
    let limits = default_limits();
    let vel = Vec3::new(2.0, 2.0, 0.0); // |v| = 2√2 ≈ 2.83
    let clamped = limits.clamp_velocity(vel);
    let speed = clamped.norm();
    assert!((speed - 1.5).abs() < 1e-4, "speed={speed}");
    // x/y components should be equal (diagonal direction preserved)
    assert!((clamped.x - clamped.y).abs() < 1e-5);
}

/// Zero velocity → stays zero (avoid divide-by-zero).
#[test]
fn test_clamp_velocity_zero() {
    let limits = default_limits();
    let vel = Vec3::zero();
    let clamped = limits.clamp_velocity(vel);
    assert!(clamped.norm() < 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
// check_safety
// ─────────────────────────────────────────────────────────────────────────────

/// All values within limits → all flags true.
#[test]
fn test_check_safety_all_ok() {
    let limits = default_limits();
    let pos = Vec3::new(0.0, 0.0, 1.0);
    let vel = Vec3::new(0.5, 0.0, 0.0);
    let status = check_safety(&limits, pos, vel);
    assert!(status.altitude_ok, "altitude should be ok");
    assert!(status.speed_ok, "speed should be ok");
    assert!(status.geofence_ok, "geofence should be ok");
}

/// Altitude below min → altitude_ok = false.
#[test]
fn test_check_safety_altitude_low() {
    let limits = default_limits();
    let pos = Vec3::new(0.0, 0.0, 0.0); // below 0.1
    let vel = Vec3::zero();
    let status = check_safety(&limits, pos, vel);
    assert!(!status.altitude_ok, "altitude should be violated");
    assert!(status.geofence_ok);
    assert!(status.speed_ok);
}

/// Altitude above max → altitude_ok = false.
#[test]
fn test_check_safety_altitude_high() {
    let limits = default_limits();
    let pos = Vec3::new(0.0, 0.0, 3.0); // above 2.0
    let vel = Vec3::zero();
    let status = check_safety(&limits, pos, vel);
    assert!(!status.altitude_ok, "altitude should be violated");
}

/// Speed above max → speed_ok = false.
#[test]
fn test_check_safety_speed_exceeded() {
    let limits = default_limits();
    let pos = Vec3::new(0.0, 0.0, 1.0);
    let vel = Vec3::new(5.0, 0.0, 0.0); // speed = 5 > 1.5
    let status = check_safety(&limits, pos, vel);
    assert!(!status.speed_ok, "speed should be violated");
    assert!(status.altitude_ok);
    assert!(status.geofence_ok);
}

/// X out of geofence → geofence_ok = false.
#[test]
fn test_check_safety_geofence_x() {
    let limits = default_limits();
    let pos = Vec3::new(5.0, 0.0, 1.0); // x > 2.0
    let vel = Vec3::zero();
    let status = check_safety(&limits, pos, vel);
    assert!(!status.geofence_ok, "geofence should be violated");
    assert!(status.altitude_ok);
    assert!(status.speed_ok);
}

/// Y out of geofence → geofence_ok = false.
#[test]
fn test_check_safety_geofence_y() {
    let limits = default_limits();
    let pos = Vec3::new(0.0, -5.0, 1.0); // y < -2.0
    let vel = Vec3::zero();
    let status = check_safety(&limits, pos, vel);
    assert!(!status.geofence_ok, "geofence should be violated");
}

/// clamped_pos in returned status is properly clamped even when flags are false.
#[test]
fn test_check_safety_clamped_pos_is_valid() {
    let limits = default_limits();
    let pos = Vec3::new(10.0, 10.0, 10.0);
    let vel = Vec3::new(10.0, 0.0, 0.0);
    let status = check_safety(&limits, pos, vel);
    assert!(status.clamped_pos.x <= limits.x_max);
    assert!(status.clamped_pos.y <= limits.y_max);
    assert!(status.clamped_pos.z <= limits.max_altitude);
    let clamped_speed = status.clamped_vel.norm();
    assert!(clamped_speed <= limits.max_speed + 1e-4);
}

/// All limits violated simultaneously → all flags false.
#[test]
fn test_check_safety_all_violated() {
    let limits = default_limits();
    let pos = Vec3::new(100.0, -100.0, -1.0);
    let vel = Vec3::new(20.0, 0.0, 0.0);
    let status = check_safety(&limits, pos, vel);
    assert!(!status.altitude_ok);
    assert!(!status.speed_ok);
    assert!(!status.geofence_ok);
}
