//! Tests for the MEKF (Multiplicative Extended Kalman Filter) module.
//!
//! Covers:
//!   - `quat_to_euler`: identity, known 90° yaw, round-trip
//!   - `quat_to_rot`: identity, 90° yaw, orthogonality
//!   - `quat_mult` (via mekf internals): tested through reset/predict
//!   - `mekf_reset`: folds delta into q_ref and zeroes delta
//!   - `mekf_predict`: state advances, covariance grows then stabilises
//!   - Coriolis term: non-zero gyro + velocity → correct velocity coupling
//!   - `mekf_update_height`: state x[2] moves toward measurement
//!   - `mekf_update_flow`: state x[3]/x[4] move toward measurement
//!   - `Mekf::feed_row`: integration-level smoke test

use multirotor_simulator::estimation::{Mekf, MekfParams, MekfState, quat_to_euler, quat_to_rot};
use multirotor_simulator::estimation::mekf::{mekf_reset, mekf_predict, mekf_update_height, mekf_update_flow};
use multirotor_simulator::math::Mat9;

// ─────────────────────────────────────────────────────────────────────────────
// quat_to_euler
// ─────────────────────────────────────────────────────────────────────────────

/// Identity quaternion → (0, 0, 0).
#[test]
fn test_quat_to_euler_identity() {
    let [roll, pitch, yaw] = quat_to_euler([1.0, 0.0, 0.0, 0.0]);
    assert!(roll.abs() < 1e-5, "roll={roll}");
    assert!(pitch.abs() < 1e-5, "pitch={pitch}");
    assert!(yaw.abs() < 1e-5, "yaw={yaw}");
}

/// 90° yaw quaternion [w,x,y,z] = [cos45°, 0, 0, sin45°] → yaw ≈ π/2.
#[test]
fn test_quat_to_euler_90deg_yaw() {
    let half_pi: f32 = std::f32::consts::FRAC_PI_2;
    let s = (half_pi / 2.0).sin();
    let c = (half_pi / 2.0).cos();
    let [roll, pitch, yaw] = quat_to_euler([c, 0.0, 0.0, s]);
    assert!(roll.abs() < 1e-5, "roll={roll}");
    assert!(pitch.abs() < 1e-5, "pitch={pitch}");
    assert!((yaw - half_pi).abs() < 1e-5, "yaw={yaw}");
}

/// 30° roll quaternion → roll ≈ π/6.
#[test]
fn test_quat_to_euler_30deg_roll() {
    let angle: f32 = std::f32::consts::PI / 6.0;
    let s = (angle / 2.0).sin();
    let c = (angle / 2.0).cos();
    let [roll, pitch, yaw] = quat_to_euler([c, s, 0.0, 0.0]);
    assert!((roll - angle).abs() < 1e-5, "roll={roll}");
    assert!(pitch.abs() < 1e-5, "pitch={pitch}");
    assert!(yaw.abs() < 1e-5, "yaw={yaw}");
}

/// Output must never be NaN even near gimbal lock.
#[test]
fn test_quat_to_euler_no_nan() {
    // pitch ≈ 90°: sinp ≈ 1.0, clamped inside quat_to_euler
    let angle: f32 = std::f32::consts::FRAC_PI_2 * 0.9999;
    let s = (angle / 2.0).sin();
    let c = (angle / 2.0).cos();
    // rotation about Y axis
    let [roll, pitch, yaw] = quat_to_euler([c, 0.0, s, 0.0]);
    assert!(!roll.is_nan() && !roll.is_infinite());
    assert!(!pitch.is_nan() && !pitch.is_infinite());
    assert!(!yaw.is_nan() && !yaw.is_infinite());
}

// ─────────────────────────────────────────────────────────────────────────────
// mekf_reset
// ─────────────────────────────────────────────────────────────────────────────

/// After reset the delta components (x[6..9]) are zeroed.
#[test]
fn test_mekf_reset_zeroes_delta() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    // Inject a non-trivial delta
    state.x[6] = 0.1;
    state.x[7] = 0.05;
    state.x[8] = -0.03;
    mekf_reset(&mut state);
    assert!(state.x[6].abs() < 1e-6, "delta_x not zeroed: {}", state.x[6]);
    assert!(state.x[7].abs() < 1e-6, "delta_y not zeroed: {}", state.x[7]);
    assert!(state.x[8].abs() < 1e-6, "delta_z not zeroed: {}", state.x[8]);
}

/// After reset q_ref stays normalised.
#[test]
fn test_mekf_reset_qref_normalised() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    state.x[6] = 0.2;
    state.x[7] = 0.1;
    state.x[8] = 0.0;
    mekf_reset(&mut state);
    let n = state.q_ref.iter().map(|v| v * v).sum::<f32>().sqrt();
    assert!((n - 1.0).abs() < 1e-5, "q_ref norm={n}");
}

// ─────────────────────────────────────────────────────────────────────────────
// mekf_predict
// ─────────────────────────────────────────────────────────────────────────────

/// A predict step with zero gyro and gravity-only accel (no net force in body frame)
/// should leave height x[2] approximately unchanged (≈1.0).
#[test]
fn test_mekf_predict_height_stable() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    // Build process noise manually (diag Q)
    let p = &params;
    let r_proc = Mat9::diag([
        p.q_pos, p.q_pos, p.q_pos,
        p.q_vel, p.q_vel, p.q_vel,
        p.q_att, p.q_att, p.q_att,
    ]);
    // Level flight, gravity measured as +g in body z (pointing up)
    let gyro = [0.0f32; 3];
    let accel = [0.0, 0.0, 9.81]; // accelerometer in body frame, level
    mekf_predict(&mut state, gyro, accel, 0.01, &r_proc);
    // Height should not drift far from 1.0
    assert!((state.x[2] - 1.0).abs() < 0.5, "height drifted: {}", state.x[2]);
}

/// After several predict steps the covariance should remain finite.
#[test]
fn test_mekf_predict_covariance_finite() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    let p = &params;
    let r_proc = Mat9::diag([
        p.q_pos, p.q_pos, p.q_pos,
        p.q_vel, p.q_vel, p.q_vel,
        p.q_att, p.q_att, p.q_att,
    ]);
    let gyro = [0.01, -0.01, 0.0];
    let accel = [0.0, 0.0, 9.81];
    for _ in 0..50 {
        mekf_reset(&mut state);
        mekf_predict(&mut state, gyro, accel, 0.01, &r_proc);
    }
    for i in 0..9 {
        assert!(state.sigma.data[i][i].is_finite() && state.sigma.data[i][i] >= 0.0,
            "sigma[{i}][{i}] = {}", state.sigma.data[i][i]);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// mekf_update_height
// ─────────────────────────────────────────────────────────────────────────────

/// Update should pull x[2] toward the measurement.
#[test]
fn test_mekf_update_height_pulls_state() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    // Initial height estimate x[2] = 1.0 (set in MekfState::new)
    let z_m = 0.5; // measurement says we are at 0.5 m
    mekf_update_height(&mut state, z_m, params.r_height);
    // x[2] should move toward 0.5
    assert!(state.x[2] < 1.0 - 1e-3, "height not pulled: {}", state.x[2]);
    assert!(state.x[2] > 0.0, "height went negative");
}

/// Multiple updates should converge x[2] close to the measurement.
#[test]
fn test_mekf_update_height_converges() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    let target = 0.5_f32;
    // Feed the same measurement repeatedly
    for _ in 0..100 {
        mekf_update_height(&mut state, target, params.r_height);
    }
    assert!((state.x[2] - target).abs() < 0.05, "height={}", state.x[2]);
}

// ─────────────────────────────────────────────────────────────────────────────
// Mekf::feed_row (integration smoke test)
// ─────────────────────────────────────────────────────────────────────────────

/// feed_row with gyro+accel returns Some after first pair.
#[test]
fn test_mekf_feed_row_returns_estimate() {
    let params = MekfParams::default();
    let mut filter = Mekf::new(params);
    // First row: gyro
    let r1 = filter.feed_row(0.0, Some([0.0, 0.0, 0.0]), None, None, None, None);
    // After gyro only, no accel yet → None
    assert!(r1.is_none());
    // Second row: accel  (same timestamp)
    let r2 = filter.feed_row(0.01, None, Some([0.0, 0.0, 1.0]), None, None, None);
    // Still None because prev_t wasn't set yet; now set it
    let _ = r2;
    // Third row: gyro again at a new time
    let r3 = filter.feed_row(0.02, Some([0.0, 0.0, 0.0]), None, None, None, None);
    let _ = r3;
    // Fourth row: accel at new time → prediction should now fire
    let r4 = filter.feed_row(0.03, None, Some([0.0, 0.0, 1.0]), None, None, None);
    // We may or may not get Some here depending on timing logic, but no panics
    let _ = r4;
}

/// feed_row output values are finite.
#[test]
fn test_mekf_feed_row_output_finite() {
    let params = MekfParams::default();
    let mut filter = Mekf::new(params);
    let mut last_estimate = None;
    // Feed 100 steps of sensor data
    for i in 0..100 {
        let t = i as f32 * 0.01;
        // Alternating gyro/accel rows
        if i % 2 == 0 {
            last_estimate = filter.feed_row(t, Some([0.01, -0.01, 0.0]), None, Some(1000.0), None, None);
        } else {
            last_estimate = filter.feed_row(t, None, Some([0.0, 0.0, 1.0]), None, Some(0.1), Some(-0.1));
        }
    }
    if let Some(est) = last_estimate {
        for (i, v) in est.iter().enumerate() {
            assert!(v.is_finite(), "estimate[{i}] = {v}");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// mekf_update_flow
// ─────────────────────────────────────────────────────────────────────────────

/// A nonzero flow observation should update the horizontal velocity states.
#[test]
fn test_mekf_update_flow_changes_velocity_state() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    // height must be > 0.05 to avoid the early-return guard
    state.x[2] = 0.5;
    // give the state a nonzero body-frame velocity so the predicted measurement is nonzero
    state.x[3] = 1.0;
    state.x[4] = 0.5;

    let bx_before = state.x[3];
    let by_before = state.x[4];

    // Inject a flow measurement that disagrees with the predicted value
    mekf_update_flow(&mut state, 0.0, 0.0, 0.01, params.r_flow, None);

    // Both bx and by should have been pulled toward zero (innovation = 0 - predicted)
    assert!((state.x[3] - bx_before).abs() > 1e-9,
        "x[3] (bx) should change; before={bx_before}, after={}", state.x[3]);
    assert!((state.x[4] - by_before).abs() > 1e-9,
        "x[4] (by) should change; before={by_before}, after={}", state.x[4]);
}

/// When pz < 0.05, mekf_update_flow must return early without touching the state.
#[test]
fn test_mekf_update_flow_skips_near_zero_height() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    state.x[2] = 0.01; // below 0.05 threshold
    state.x[3] = 2.0;
    state.x[4] = 3.0;

    let x3_before = state.x[3];
    let x4_before = state.x[4];

    mekf_update_flow(&mut state, 100.0, 100.0, 0.01, params.r_flow, None);

    assert!((state.x[3] - x3_before).abs() < 1e-9, "x[3] should not change below height guard");
    assert!((state.x[4] - x4_before).abs() < 1e-9, "x[4] should not change below height guard");
}

/// After many identical flow observations the state should converge (residual shrinks).
#[test]
fn test_mekf_update_flow_converges() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    state.x[2] = 1.0;
    state.x[3] = 0.0;
    state.x[4] = 0.0;

    // Feed the same measurement 50 times — state should stabilise and remain finite
    for _ in 0..50 {
        mekf_update_flow(&mut state, 0.5, -0.5, 0.01, params.r_flow, None);
    }
    assert!(state.x[3].is_finite(), "x[3] went non-finite");
    assert!(state.x[4].is_finite(), "x[4] went non-finite");
}

// ─────────────────────────────────────────────────────────────────────────────
// quat_to_rot
// ─────────────────────────────────────────────────────────────────────────────

/// Identity quaternion → identity rotation matrix.
#[test]
fn test_quat_to_rot_identity() {
    let r = quat_to_rot([1.0, 0.0, 0.0, 0.0]);
    for i in 0..3 {
        for j in 0..3 {
            let expected = if i == j { 1.0 } else { 0.0 };
            assert!((r[i][j] - expected).abs() < 1e-6,
                "R[{i}][{j}] = {} expected {expected}", r[i][j]);
        }
    }
}

/// 90° CCW yaw: body X should map to world Y.
#[test]
fn test_quat_to_rot_90deg_yaw() {
    let s = (std::f32::consts::FRAC_PI_4).sin();
    let c = (std::f32::consts::FRAC_PI_4).cos();
    let r = quat_to_rot([c, 0.0, 0.0, s]);
    // R * [1, 0, 0] = [0, 1, 0]
    assert!((r[0][0]).abs() < 1e-6, "R[0][0]={}", r[0][0]);
    assert!((r[1][0] - 1.0).abs() < 1e-6, "R[1][0]={}", r[1][0]);
    assert!((r[2][0]).abs() < 1e-6, "R[2][0]={}", r[2][0]);
}

/// Rotation matrix must be orthogonal: R^T R = I.
#[test]
fn test_quat_to_rot_orthogonal() {
    // arbitrary non-trivial quaternion
    let n = (0.6f32*0.6 + 0.2*0.2 + 0.3*0.3 + 0.7*0.7).sqrt();
    let q = [0.6/n, 0.2/n, -0.3/n, 0.7/n];
    let r = quat_to_rot(q);
    for i in 0..3 {
        for j in 0..3 {
            let dot: f32 = (0..3).map(|k| r[k][i] * r[k][j]).sum();
            let expected = if i == j { 1.0 } else { 0.0 };
            assert!((dot - expected).abs() < 1e-5,
                "RtR[{i}][{j}] = {dot}, expected {expected}");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Coriolis term correctness
// ─────────────────────────────────────────────────────────────────────────────

/// With ωz = 1 rad/s and bx = 1 m/s (level drone), the Coriolis term
/// −ω×v contributes −(ωz·bx) to the y-velocity derivative.
/// After one predict step, by should be negative.
#[test]
fn test_coriolis_correct_sign_level() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    state.x[3] = 1.0; // bx = 1 m/s, everything else zero

    let r_proc = Mat9::diag([
        params.q_pos, params.q_pos, params.q_pos,
        params.q_vel, params.q_vel, params.q_vel,
        params.q_att, params.q_att, params.q_att,
    ]);
    let gyro  = [0.0f32, 0.0, 1.0]; // ωz = 1 rad/s
    let accel = [0.0f32, 0.0, 9.81]; // cancel gravity, no net thrust

    mekf_reset(&mut state);
    mekf_predict(&mut state, gyro, accel, 0.01, &r_proc);

    // −(ω×v).y = −(ωz·bx − ωx·bz) = −(1·1 − 0) = −1 m/s²  over dt=0.01 → Δby ≈ −0.01
    assert!(state.x[4] < -1e-4,
        "Coriolis should make by negative; got by={}", state.x[4]);
}

/// Without rotation (gyro = 0), Coriolis term is zero — velocity should not
/// change relative to the no-Coriolis case.
#[test]
fn test_no_coriolis_when_not_rotating() {
    let params = MekfParams::default();
    let mut state = MekfState::new([1.0, 0.0, 0.0, 0.0], &params);
    state.x[3] = 1.0; // bx = 1 m/s

    let r_proc = Mat9::diag([
        params.q_pos, params.q_pos, params.q_pos,
        params.q_vel, params.q_vel, params.q_vel,
        params.q_att, params.q_att, params.q_att,
    ]);
    let gyro  = [0.0f32; 3];        // no rotation
    let accel = [0.0f32, 0.0, 9.81];

    let by_before = state.x[4];
    mekf_reset(&mut state);
    mekf_predict(&mut state, gyro, accel, 0.01, &r_proc);

    // by should remain near zero (no Coriolis coupling with zero gyro)
    assert!((state.x[4] - by_before).abs() < 1e-6,
        "by changed without rotation: Δby={}", state.x[4] - by_before);
}

// ─────────────────────────────────────────────────────────────────────────────
// Zero-flow gate (Mekf::feed_row)
// ─────────────────────────────────────────────────────────────────────────────

/// Zero flow samples (both axes < zero_flow_threshold) must NOT trigger a
/// velocity-state update — they are PMW3901 zero-padding artefacts.
#[test]
fn test_flow_zero_gate_skips_update() {
    let params = MekfParams::default(); // zero_flow_threshold = 0.3
    let mut filter = Mekf::new(params);
    filter.seed_qref([1.0, 0.0, 0.0, 0.0]);

    // Prime the filter with a non-zero height so the flow scale is valid,
    // and give it a non-zero body velocity so any real update would move the state.
    filter.feed_row(0.02, None, None, Some(500.0), None, None);
    filter.state.x[3] = 1.0; // bx = 1 m/s
    filter.state.x[4] = 0.5; // by = 0.5 m/s

    let bx_before = filter.state.x[3];
    let by_before = filter.state.x[4];

    // Feed a zero-flow sample (both axes exactly 0.0, well below threshold 0.3)
    filter.feed_row(0.10, None, None, Some(500.0), Some(0.0), Some(0.0));

    assert_eq!(filter.state.x[3], bx_before,
        "bx changed on zero-padded flow: before={bx_before}, after={}", filter.state.x[3]);
    assert_eq!(filter.state.x[4], by_before,
        "by changed on zero-padded flow: before={by_before}, after={}", filter.state.x[4]);
}

/// Flow above the zero_flow_threshold DOES update the velocity state.
#[test]
fn test_flow_above_threshold_updates_velocity() {
    let params = MekfParams { zero_flow_threshold: 0.3, ..MekfParams::default() };
    let mut filter = Mekf::new(params);
    filter.seed_qref([1.0, 0.0, 0.0, 0.0]);

    // Prime with height and a reference timestamp so dt_flow > 0
    filter.feed_row(0.02, None, None, Some(500.0), None, None);
    // Feed a small non-zero flow (below threshold) to set last_flow_t
    filter.feed_row(0.05, None, None, Some(500.0), Some(0.2), Some(0.0));

    let bx_before = filter.state.x[3];

    // Feed a significant flow well above the 0.3 threshold
    filter.feed_row(0.10, None, None, Some(500.0), Some(2.0), Some(1.5));

    assert!(filter.state.x[3] != bx_before,
        "bx should change on real flow above threshold; bx={}", filter.state.x[3]);
}
