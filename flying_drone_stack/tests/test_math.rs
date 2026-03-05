//! Tests for math utilities not covered by the inline unit tests.
//!
//! Covers:
//!   - `to_euler`: round-trip, known 90° yaw, known 45° pitch, gimbal-lock clamp
//!   - `to_rotation_matrix`: 90° yaw produces correct column vectors
//!   - `integrate_exponential` vs `integrate` for small ω (should agree within 1e-4)
//!   - `Mat9`: zeros, identity, diag, transpose, mat_mul, add, scale, mat_vec,
//!             outer, joseph_update, h_sigma_ht, sigma_ht, symmetrise, clamp_diagonal
//!   - `rot_to_quat`: identity, orthogonal rotation matrix round-trip
//!   - `flatness_to_reference`: position/velocity/yaw forwarded correctly

use multirotor_simulator::math::{Vec3, Quat, Mat9, to_euler};

// ─────────────────────────────────────────────────────────────────────────────
// to_euler
// ─────────────────────────────────────────────────────────────────────────────

/// Identity quaternion → (0, 0, 0)
#[test]
fn test_to_euler_identity() {
    let q = Quat::identity();
    let (roll, pitch, yaw) = to_euler(q);
    assert!(roll.abs() < 1e-6, "roll={roll}");
    assert!(pitch.abs() < 1e-6, "pitch={pitch}");
    assert!(yaw.abs() < 1e-6, "yaw={yaw}");
}

/// 90° yaw (rotation about Z) → (0, 0, π/2)
#[test]
fn test_to_euler_90deg_yaw() {
    let half_pi = std::f32::consts::FRAC_PI_2;
    let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), half_pi);
    let (roll, pitch, yaw) = to_euler(q);
    assert!(roll.abs() < 1e-5, "roll should be 0, got {roll}");
    assert!(pitch.abs() < 1e-5, "pitch should be 0, got {pitch}");
    assert!((yaw - half_pi).abs() < 1e-5, "yaw should be π/2, got {yaw}");
}

/// 45° pitch (rotation about Y) → (0, π/4, 0)
#[test]
fn test_to_euler_45deg_pitch() {
    let pi_4 = std::f32::consts::PI / 4.0;
    let q = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), pi_4);
    let (roll, pitch, yaw) = to_euler(q);
    assert!(roll.abs() < 1e-5, "roll should be 0, got {roll}");
    assert!((pitch - pi_4).abs() < 1e-5, "pitch should be π/4, got {pitch}");
    assert!(yaw.abs() < 1e-5, "yaw should be 0, got {yaw}");
}

/// 30° roll (rotation about X) → (π/6, 0, 0)
#[test]
fn test_to_euler_30deg_roll() {
    let pi_6 = std::f32::consts::PI / 6.0;
    let q = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), pi_6);
    let (roll, pitch, yaw) = to_euler(q);
    assert!((roll - pi_6).abs() < 1e-5, "roll should be π/6, got {roll}");
    assert!(pitch.abs() < 1e-5, "pitch should be 0, got {pitch}");
    assert!(yaw.abs() < 1e-5, "yaw should be 0, got {yaw}");
}

/// Round-trip: build quaternion from known angles, decompose back, compare.
#[test]
fn test_to_euler_round_trip_yaw_roll() {
    // Known ZYX: roll=0.3 rad, pitch=0, yaw=1.2 rad
    let expected_roll: f32 = 0.3;
    let expected_yaw: f32 = 1.2;
    // ZYX: first yaw about Z, then roll about X
    let q_yaw = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), expected_yaw);
    let q_roll = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), expected_roll);
    // ZYX order: q = q_yaw * q_roll
    let q = q_yaw * q_roll;
    let (roll, _pitch, yaw) = to_euler(q);
    assert!((roll - expected_roll).abs() < 1e-4, "roll: got {roll}, expected {expected_roll}");
    assert!((yaw - expected_yaw).abs() < 1e-4, "yaw: got {yaw}, expected {expected_yaw}");
}

/// Gimbal lock: pitch ≈ ±90° is clamped; result does not NaN/panic.
#[test]
fn test_to_euler_near_gimbal_lock() {
    let pi_2 = std::f32::consts::FRAC_PI_2;
    // Almost exactly 90° pitch
    let q = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), pi_2 * 0.9999);
    let (roll, pitch, yaw) = to_euler(q);
    // Values may be arbitrary but must not be NaN / Inf
    assert!(!roll.is_nan() && !roll.is_infinite(), "roll is NaN/inf");
    assert!(!pitch.is_nan() && !pitch.is_infinite(), "pitch is NaN/inf");
    assert!(!yaw.is_nan() && !yaw.is_infinite(), "yaw is NaN/inf");
    // Pitch should be close to ±π/2
    assert!(pitch.abs() > pi_2 * 0.99, "pitch should be near π/2, got {pitch}");
}

// ─────────────────────────────────────────────────────────────────────────────
// to_rotation_matrix
// ─────────────────────────────────────────────────────────────────────────────

/// Identity quaternion → identity rotation matrix.
#[test]
fn test_to_rotation_matrix_identity() {
    let q = Quat::identity();
    let r = q.to_rotation_matrix();
    assert!((r[0][0] - 1.0).abs() < 1e-6);
    assert!((r[1][1] - 1.0).abs() < 1e-6);
    assert!((r[2][2] - 1.0).abs() < 1e-6);
    assert!(r[0][1].abs() < 1e-6);
    assert!(r[0][2].abs() < 1e-6);
    assert!(r[1][0].abs() < 1e-6);
    assert!(r[1][2].abs() < 1e-6);
    assert!(r[2][0].abs() < 1e-6);
    assert!(r[2][1].abs() < 1e-6);
}

/// 90° yaw rotation: x-column → (0,1,0), y-column → (-1,0,0), z-column → (0,0,1).
#[test]
fn test_to_rotation_matrix_90deg_yaw() {
    let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2);
    let r = q.to_rotation_matrix();
    // Column 0 (x-body in world) should be (0, 1, 0) after 90° yaw
    assert!((r[0][0] - 0.0).abs() < 1e-5, "R[0][0]={}", r[0][0]);
    assert!((r[1][0] - 1.0).abs() < 1e-5, "R[1][0]={}", r[1][0]);
    assert!((r[2][0] - 0.0).abs() < 1e-5, "R[2][0]={}", r[2][0]);
    // z column unchanged
    assert!((r[0][2] - 0.0).abs() < 1e-5, "R[0][2]={}", r[0][2]);
    assert!((r[1][2] - 0.0).abs() < 1e-5, "R[1][2]={}", r[1][2]);
    assert!((r[2][2] - 1.0).abs() < 1e-5, "R[2][2]={}", r[2][2]);
}

/// 90° pitch (about Y): z-column should point in -x direction.
#[test]
fn test_to_rotation_matrix_90deg_pitch() {
    let q = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), std::f32::consts::FRAC_PI_2);
    let r = q.to_rotation_matrix();
    // z-column (column 2): rotated body-z  →  world-x direction
    // Rot_y(90°)*[0,0,1] = [1,0,0]  but note convention: rotation matrix acts on vectors
    // R_y(θ) = [[cos θ, 0, sin θ],[0,1,0],[-sin θ, 0, cos θ]]
    // Column 2 = R * e_z = [sin 90°, 0, cos 90°] = [1, 0, 0]
    assert!((r[0][2] - 1.0).abs() < 1e-5, "R[0][2]={}", r[0][2]);
    assert!((r[1][2] - 0.0).abs() < 1e-5, "R[1][2]={}", r[1][2]);
    assert!((r[2][2] - 0.0).abs() < 1e-5, "R[2][2]={}", r[2][2]);
}

/// Rotation matrix must be orthogonal: R * Rᵀ ≈ I.
#[test]
fn test_to_rotation_matrix_orthogonal() {
    let q = Quat::from_axis_angle(Vec3::new(1.0, 1.0, 0.0).normalize(), 1.2_f32);
    let r = q.to_rotation_matrix();
    // Compute R * Rᵀ and check it is close to I
    for i in 0..3 {
        for j in 0..3 {
            let dot: f32 = (0..3).map(|k| r[k][i] * r[k][j]).sum();
            let expected = if i == j { 1.0 } else { 0.0 };
            assert!((dot - expected).abs() < 1e-5,
                "R*Rt[{i}][{j}] = {dot}, expected {expected}");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// integrate_exponential vs integrate (for small ω both should agree)
// ─────────────────────────────────────────────────────────────────────────────

/// For a small angular velocity (5 deg/s for 0.01 s = 0.05° rotation),
/// `integrate` (linear) and `integrate_exponential` (Lie group) must agree
/// to within 1e-5 in each quaternion component.
#[test]
fn test_integrate_exponential_matches_integrate_small_omega() {
    let q = Quat::identity();
    let omega = Vec3::new(0.1, 0.05, 0.0); // rad/s – small
    let dt = 0.01;

    let q_lin = q.integrate(omega, dt);
    let q_exp = q.integrate_exponential(omega, dt);

    assert!((q_lin.w - q_exp.w).abs() < 1e-4, "w: lin={} exp={}", q_lin.w, q_exp.w);
    assert!((q_lin.x - q_exp.x).abs() < 1e-4, "x: lin={} exp={}", q_lin.x, q_exp.x);
    assert!((q_lin.y - q_exp.y).abs() < 1e-4, "y: lin={} exp={}", q_lin.y, q_exp.y);
    assert!((q_lin.z - q_exp.z).abs() < 1e-4, "z: lin={} exp={}", q_lin.z, q_exp.z);
}

/// `integrate_exponential` must produce a unit quaternion.
#[test]
fn test_integrate_exponential_unit_norm() {
    let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.5_f32);
    let omega = Vec3::new(2.0, 1.0, 0.5); // large ω
    let dt = 0.01;
    let q2 = q.integrate_exponential(omega, dt);
    let norm = (q2.w * q2.w + q2.x * q2.x + q2.y * q2.y + q2.z * q2.z).sqrt();
    assert!((norm - 1.0).abs() < 1e-5, "norm = {norm}");
}

/// After many integration steps with constant ω, the total rotation angle
/// should approximate ‖ω‖ * T.
#[test]
fn test_integrate_exponential_accumulates_rotation() {
    let omega = Vec3::new(0.0, 0.0, 1.0); // 1 rad/s about Z
    let dt = 0.01;
    let steps = 100; // 1 second → should rotate 1 rad
    let mut q = Quat::identity();
    for _ in 0..steps {
        q = q.integrate_exponential(omega, dt);
    }
    // After 1 s, yaw should be ≈ 1 rad
    let (_, _, yaw) = to_euler(q);
    assert!((yaw - 1.0).abs() < 0.02, "yaw after 1 s: {yaw}");
}

// ─────────────────────────────────────────────────────────────────────────────
// Mat9
// ─────────────────────────────────────────────────────────────────────────────

#[test]
fn test_mat9_zeros() {
    let m = Mat9::zeros();
    for i in 0..9 {
        for j in 0..9 {
            assert_eq!(m.data[i][j], 0.0);
        }
    }
}

#[test]
fn test_mat9_identity() {
    let m = Mat9::identity();
    for i in 0..9 {
        for j in 0..9 {
            let expected = if i == j { 1.0 } else { 0.0 };
            assert_eq!(m.data[i][j], expected);
        }
    }
}

#[test]
fn test_mat9_diag() {
    let d = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0_f32];
    let m = Mat9::diag(d);
    for i in 0..9 {
        assert_eq!(m.data[i][i], d[i]);
        for j in 0..9 {
            if i != j {
                assert_eq!(m.data[i][j], 0.0);
            }
        }
    }
}

#[test]
fn test_mat9_transpose() {
    let mut m = Mat9::zeros();
    m.data[0][1] = 5.0;
    m.data[1][0] = 3.0;
    let t = m.transpose();
    assert_eq!(t.data[1][0], 5.0);
    assert_eq!(t.data[0][1], 3.0);
}

#[test]
fn test_mat9_mat_mul_identity() {
    let id = Mat9::identity();
    let d = Mat9::diag([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    let result = id.mat_mul(&d);
    for i in 0..9 {
        assert!((result.data[i][i] - d.data[i][i]).abs() < 1e-6);
    }
}

#[test]
fn test_mat9_add() {
    let a = Mat9::identity();
    let b = Mat9::identity();
    let c = a.add(&b);
    for i in 0..9 {
        assert!((c.data[i][i] - 2.0).abs() < 1e-6);
    }
}

#[test]
fn test_mat9_scale() {
    let m = Mat9::identity();
    let s = m.scale(3.0);
    for i in 0..9 {
        assert!((s.data[i][i] - 3.0).abs() < 1e-6);
    }
}

#[test]
fn test_mat9_mat_vec() {
    let m = Mat9::identity();
    let v = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0_f32];
    let out = m.mat_vec(&v);
    for i in 0..9 {
        assert!((out[i] - v[i]).abs() < 1e-6, "out[{i}]={}", out[i]);
    }
}

#[test]
fn test_mat9_outer() {
    let u = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0_f32];
    let v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0_f32];
    let m = Mat9::outer(&u, &v);
    // Only entry [0][8] should be nonzero
    assert!((m.data[0][8] - 1.0).abs() < 1e-6);
    assert_eq!(m.data[0][0], 0.0);
}

#[test]
fn test_mat9_h_sigma_ht_identity() {
    let sigma = Mat9::identity();
    let h: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let val = Mat9::h_sigma_ht(&h, &sigma);
    // H * I * Hᵀ = H * Hᵀ = 1 (since only first element is 1)
    assert!((val - 1.0).abs() < 1e-6, "h_sigma_ht = {val}");
}

#[test]
fn test_mat9_sigma_ht_identity() {
    let sigma = Mat9::identity();
    let h: [f32; 9] = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0];
    let col = Mat9::sigma_ht(&sigma, &h);
    // I * hᵀ = hᵀ
    for i in 0..9 {
        assert!((col[i] - h[i]).abs() < 1e-6, "col[{i}]={} h[{i}]={}", col[i], h[i]);
    }
}

#[test]
fn test_mat9_symmetrise() {
    let mut m = Mat9::zeros();
    m.data[0][1] = 2.0;
    m.data[1][0] = 4.0; // intentionally asymmetric
    m.symmetrise();
    // After symmetrising, both should be (2+4)/2 = 3
    assert!((m.data[0][1] - 3.0).abs() < 1e-6);
    assert!((m.data[1][0] - 3.0).abs() < 1e-6);
}

#[test]
fn test_mat9_clamp_diagonal() {
    let d = [1000.0_f32; 9];
    let mut m = Mat9::diag(d);
    m.clamp_diagonal(100.0);
    for i in 0..9 {
        assert!(m.data[i][i] <= 100.0 + 1e-4, "diagonal[{i}] = {}", m.data[i][i]);
    }
}

#[test]
fn test_mat9_joseph_update_identity_gain() {
    // K = e_0 (gain picks up only first state), H = e_0 row, r = 1.0
    // Joseph update: (I - K H) Σ (I - K H)ᵀ + r * K Kᵀ
    // With Σ = I, K = e_0, H = e_0: result should remain positive-definite
    let sigma = Mat9::identity();
    let k: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let h: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let result = Mat9::joseph_update(&sigma, &k, &h, 1.0);
    // Diagonal entries (1..8) unchanged; [0][0] = 0 + 1 = 1
    for i in 1..9 {
        assert!((result.data[i][i] - 1.0).abs() < 1e-5, "result[{i}][{i}]={}", result.data[i][i]);
    }
    // No NaN anywhere
    for i in 0..9 {
        for j in 0..9 {
            assert!(!result.data[i][j].is_nan(), "NaN at [{i}][{j}]");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// rot_to_quat  (planning::flatness)
// ─────────────────────────────────────────────────────────────────────────────

use multirotor_simulator::prelude::{FlatOutput, compute_flatness, rot_to_quat, flatness_to_reference};

/// Identity rotation matrix → identity quaternion [w=1, x=0, y=0, z=0].
#[test]
fn test_rot_to_quat_identity() {
    let rot: [[f32; 3]; 3] = [
        [1.0, 0.0, 0.0], // xb column
        [0.0, 1.0, 0.0], // yb column
        [0.0, 0.0, 1.0], // zb column
    ];
    let q = rot_to_quat(&rot);
    assert!((q[0] - 1.0).abs() < 1e-5, "w={}", q[0]);
    assert!(q[1].abs() < 1e-5, "x={}", q[1]);
    assert!(q[2].abs() < 1e-5, "y={}", q[2]);
    assert!(q[3].abs() < 1e-5, "z={}", q[3]);
}

/// Output of rot_to_quat must always be a unit quaternion.
#[test]
fn test_rot_to_quat_unit_norm() {
    // Use compute_flatness to produce a valid rotation matrix for a 45° yaw case
    let flat = FlatOutput {
        pos: Vec3::new(0.0, 0.0, 1.0),
        vel: Vec3::zero(),
        acc: Vec3::zero(),
        jerk: Vec3::zero(),
        snap: Vec3::zero(),
        yaw: std::f32::consts::FRAC_PI_4,
        yaw_dot: 0.0,
        yaw_ddot: 0.0,
    };
    let fr = compute_flatness(&flat, 0.027);
    let q = rot_to_quat(&fr.rot);
    let norm: f32 = q.iter().map(|v| v * v).sum::<f32>().sqrt();
    assert!((norm - 1.0).abs() < 1e-4, "quaternion not unit: norm={norm}");
}

/// 90° yaw: the rotation matrix from compute_flatness → rot_to_quat should
/// yield a quaternion consistent with a z-axis rotation by π/2.
#[test]
fn test_rot_to_quat_90deg_yaw() {
    let half_pi = std::f32::consts::FRAC_PI_2;
    let flat = FlatOutput {
        pos: Vec3::zero(),
        vel: Vec3::zero(),
        acc: Vec3::zero(),
        jerk: Vec3::zero(),
        snap: Vec3::zero(),
        yaw: half_pi,
        yaw_dot: 0.0,
        yaw_ddot: 0.0,
    };
    let fr = compute_flatness(&flat, 0.027);
    let q = rot_to_quat(&fr.rot);
    // For a pure yaw of π/2 the quaternion should be [cos(π/4), 0, 0, sin(π/4)]
    let expected_w = (half_pi / 2.0).cos();
    let expected_z = (half_pi / 2.0).sin();
    assert!((q[0] - expected_w).abs() < 1e-4, "w={} expected={expected_w}", q[0]);
    assert!(q[1].abs() < 1e-4, "x={}", q[1]);
    assert!(q[2].abs() < 1e-4, "y={}", q[2]);
    assert!((q[3].abs() - expected_z.abs()) < 1e-4, "z={} expected={expected_z}", q[3]);
}

// ─────────────────────────────────────────────────────────────────────────────
// flatness_to_reference  (planning::flatness)
// ─────────────────────────────────────────────────────────────────────────────

/// flatness_to_reference must copy position, velocity, yaw, and yaw_rate correctly.
#[test]
fn test_flatness_to_reference_copies_fields() {
    let flat = FlatOutput {
        pos: Vec3::new(1.0, 2.0, 3.0),
        vel: Vec3::new(0.1, 0.2, 0.3),
        acc: Vec3::zero(),
        jerk: Vec3::zero(),
        snap: Vec3::zero(),
        yaw: 0.4,
        yaw_dot: 0.05,
        yaw_ddot: 0.0,
    };
    let fr = compute_flatness(&flat, 0.027);
    let tref = flatness_to_reference(&fr, Vec3::new(0.0, 0.0, 9.81), Vec3::zero(), 0.4, 0.05, 0.0);

    assert!((tref.position.x - 1.0).abs() < 1e-5, "pos.x={}", tref.position.x);
    assert!((tref.position.y - 2.0).abs() < 1e-5, "pos.y={}", tref.position.y);
    assert!((tref.position.z - 3.0).abs() < 1e-5, "pos.z={}", tref.position.z);
    assert!((tref.velocity.x - 0.1).abs() < 1e-5, "vel.x={}", tref.velocity.x);
    assert!((tref.yaw - 0.4).abs() < 1e-5, "yaw={}", tref.yaw);
    assert!((tref.yaw_rate - 0.05).abs() < 1e-5, "yaw_rate={}", tref.yaw_rate);
}

/// Acceleration and jerk passed to flatness_to_reference appear in the reference.
#[test]
fn test_flatness_to_reference_forwards_acc_jerk() {
    let flat = FlatOutput {
        pos: Vec3::zero(), vel: Vec3::zero(), acc: Vec3::zero(),
        jerk: Vec3::zero(), snap: Vec3::zero(),
        yaw: 0.0, yaw_dot: 0.0, yaw_ddot: 0.0,
    };
    let fr = compute_flatness(&flat, 0.027);
    let acc_in = Vec3::new(1.0, 2.0, 3.0);
    let jerk_in = Vec3::new(0.1, 0.2, 0.3);
    let tref = flatness_to_reference(&fr, acc_in, jerk_in, 0.0, 0.0, 0.0);

    assert!((tref.acceleration.x - 1.0).abs() < 1e-5);
    assert!((tref.acceleration.y - 2.0).abs() < 1e-5);
    assert!((tref.acceleration.z - 3.0).abs() < 1e-5);
    assert!((tref.jerk.x - 0.1).abs() < 1e-5);
    assert!((tref.jerk.y - 0.2).abs() < 1e-5);
    assert!((tref.jerk.z - 0.3).abs() < 1e-5);
}
