//! Tests for differential flatness (Faessler et al. 2018).
//!
//! Validates:
//!   1. Thrust equals mass * ‖p̈ + g·ez‖ for arbitrary accelerations
//!   2. Rotation matrix is orthogonal: R^T R = I
//!   3. Banked turn produces correct roll angle from lateral acceleration
//!   4. Constant acceleration with zero jerk/snap gives zero angular velocity
//!   5. rot_to_quat round-trip: R → q → R recovers original rotation

use multirotor_simulator::planning::{compute_flatness, FlatOutput, rot_to_quat};
use multirotor_simulator::math::Vec3;

const MASS: f32 = 0.027;  // Crazyflie mass [kg]
const G: f32   = 9.81;    // m/s²

fn flat_hover() -> FlatOutput {
    FlatOutput {
        pos: Vec3::new(0.0, 0.0, 0.3),
        vel: Vec3::zero(),
        acc: Vec3::zero(),
        jerk: Vec3::zero(),
        snap: Vec3::zero(),
        yaw: 0.0, yaw_dot: 0.0, yaw_ddot: 0.0,
    }
}

fn flat_with_acc(ax: f32, ay: f32, az: f32) -> FlatOutput {
    FlatOutput { acc: Vec3::new(ax, ay, az), ..flat_hover() }
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 1 — Thrust = m * ‖p̈ + g·ez‖
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_flatness_thrust_equals_norm_acc_plus_g() {
    // Test several acceleration cases
    let cases: &[(f32, f32, f32)] = &[
        (0.0, 0.0, 0.0),      // hover
        (1.0, 0.0, 0.0),      // forward acceleration
        (0.0, 1.0, 0.0),      // lateral acceleration
        (-0.5, 0.3, 0.2),     // arbitrary
        (0.0, 0.0, -2.0),     // descending (still positive thrust)
    ];

    for &(ax, ay, az) in cases {
        let flat = flat_with_acc(ax, ay, az);
        let res = compute_flatness(&flat, MASS);

        let expected = MASS * ((ax * ax + ay * ay + (az + G) * (az + G)).sqrt());
        assert!(
            (res.thrust - expected).abs() < 1e-5,
            "acc=({},{},{}) thrust={:.6} expected={:.6}", ax, ay, az, res.thrust, expected
        );
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 2 — Rotation matrix is orthogonal: R^T R = I (det R = 1)
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_flatness_rotation_orthogonal() {
    let cases: &[(f32, f32, f32, f32)] = &[
        (0.0, 0.0, 0.0, 0.0),       // hover, yaw=0
        (1.0, 0.0, 0.0, 0.0),       // forward acc
        (0.0, 1.0, 0.0, 0.5),       // lateral acc, yaw=0.5 rad
        (-0.3, 0.4, 0.1, -1.2),     // arbitrary
    ];

    for &(ax, ay, az, yaw) in cases {
        let flat = FlatOutput { yaw, ..flat_with_acc(ax, ay, az) };
        let res = compute_flatness(&flat, MASS);

        // Build full 3x3 R from column representation [xb, yb, zb]
        let r = &res.rot;
        // r[0]=xb, r[1]=yb, r[2]=zb (each is a [f32;3] column)
        // R^T R should be identity
        for i in 0..3 {
            for j in 0..3 {
                let dot: f32 = (0..3).map(|k| r[i][k] * r[j][k]).sum();
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (dot - expected).abs() < 1e-5,
                    "R^T R [{i}][{j}] = {dot:.6}, expected {expected:.1} for acc=({ax},{ay},{az})"
                );
            }
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 3 — Banked turn: lateral acc → correct body tilt and increased thrust
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_flatness_banked_turn_thrust_and_tilt() {
    // Pure lateral acceleration in x: acc = [ax, 0, 0], yaw = 0
    let ax = 2.0f32;  // m/s²
    let flat = flat_with_acc(ax, 0.0, 0.0);
    let res = compute_flatness(&flat, MASS);

    // Thrust must be greater than hover thrust (tilted drone needs more thrust)
    let hover_thrust = MASS * G;
    assert!(res.thrust > hover_thrust,
        "banked thrust {:.4} should exceed hover {:.4}", res.thrust, hover_thrust);

    // Expected thrust: m * sqrt(ax² + g²)
    let expected_thrust = MASS * (ax * ax + G * G).sqrt();
    assert!((res.thrust - expected_thrust).abs() < 1e-5,
        "banked thrust {:.6} != {:.6}", res.thrust, expected_thrust);

    // zb should tilt toward +x (acc in +x → zb has positive x component)
    let zb = res.rot[2];
    assert!(zb[0] > 0.0,
        "banked turn: zb[0] should be positive (tilt toward +x), got {:.4}", zb[0]);

    // Expected tilt angle: atan(ax / g)
    let expected_tilt = (ax / G).atan();
    let actual_tilt = zb[0].atan2(zb[2]);
    assert!((actual_tilt - expected_tilt).abs() < 1e-4,
        "tilt angle {:.4}° != {:.4}°", actual_tilt.to_degrees(), expected_tilt.to_degrees());
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 4 — Constant acceleration + zero jerk/snap → zero angular velocity
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_flatness_zero_omega_for_constant_acceleration() {
    // With constant acc (zero jerk, zero snap, zero yaw rates), omega must be zero
    // because the desired orientation is constant → no rotation needed
    let cases: &[(f32, f32, f32)] = &[
        (0.0, 0.0, 0.0),
        (1.0, 0.5, 0.0),
        (-0.3, 0.0, 0.1),
    ];

    for &(ax, ay, az) in cases {
        let flat = flat_with_acc(ax, ay, az);
        // jerk = 0, snap = 0 (already zero in flat_with_acc)
        let res = compute_flatness(&flat, MASS);

        assert!(res.omega.x.abs() < 1e-5,
            "omega.x should be 0 for const acc, got {:.6}", res.omega.x);
        assert!(res.omega.y.abs() < 1e-5,
            "omega.y should be 0 for const acc, got {:.6}", res.omega.y);
        assert!(res.omega.z.abs() < 1e-5,
            "omega.z should be 0 for const acc, got {:.6}", res.omega.z);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 5 — rot_to_quat: R → q → R round-trip
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_flatness_rot_to_quat_round_trip() {
    use multirotor_simulator::estimation::quat_to_rot;

    let cases: &[(f32, f32, f32, f32)] = &[
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.3),
        (0.0, 0.5, 0.0, 1.0),
        (-0.3, 0.2, 0.0, -0.5),
    ];

    for &(ax, ay, az, yaw) in cases {
        let flat = FlatOutput { yaw, ..flat_with_acc(ax, ay, az) };
        let res = compute_flatness(&flat, MASS);

        // flatness rot → quaternion
        let q = rot_to_quat(&res.rot);

        // quaternion → rotation matrix
        let r2 = quat_to_rot(q);

        // Compare both rotation matrices (column-major vs row-major)
        // res.rot[col][row], r2[row][col]
        for col in 0..3 {
            for row in 0..3 {
                let from_flatness = res.rot[col][row];
                let from_quat = r2[row][col];
                assert!(
                    (from_flatness - from_quat).abs() < 1e-4,
                    "R[{row}][{col}] mismatch: flatness={from_flatness:.5} quat={from_quat:.5} \
                     for acc=({ax},{ay},{az}) yaw={yaw}"
                );
            }
        }
    }
}
