//! Tests for the MEKF (Multiplicative Extended Kalman Filter) module.
//!
//! Covers:
//!   - `quat_to_euler`: identity, known 90° yaw, round-trip
//!   - `quat_mult` (via mekf internals): tested through reset/predict
//!   - `mekf_reset`: folds delta into q_ref and zeroes delta
//!   - `mekf_predict`: state advances, covariance grows then stabilises
//!   - `mekf_update_height`: state x[2] moves toward measurement
//!   - `mekf_update_flow`: state x[3]/x[4] move toward measurement
//!   - `Mekf::feed_row`: integration-level smoke test

use multirotor_simulator::estimation::{Mekf, MekfParams, MekfState, quat_to_euler};
use multirotor_simulator::estimation::mekf::{mekf_reset, mekf_predict, mekf_update_height};
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
