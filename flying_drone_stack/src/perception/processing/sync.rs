//! IMU–camera temporal synchronisation.
//!
//! Camera frames arrive at ~20–30 Hz; the IMU runs at 100 Hz.  SLAM algorithms
//! need a time-aligned IMU measurement for each camera frame to propagate the
//! pose prediction.
//!
//! `ImuCameraSync` maintains a rolling buffer of IMU samples and provides
//! linear interpolation at any query timestamp.
//!
//! ## Usage
//!
//! ```rust,no_run
//! use multirotor_simulator::perception::processing::sync::ImuCameraSync;
//! use multirotor_simulator::math::Vec3;
//!
//! let mut sync = ImuCameraSync::new(500); // keep 500 ms of IMU history
//!
//! // In the IMU callback (100 Hz):
//! // sync.push_imu(t_ms, gyro_rads, accel_ms2);
//!
//! // When a camera frame arrives at timestamp t_cam:
//! // if let Some((gyro, accel)) = sync.query_at(t_cam) { ... }
//! ```

use std::collections::VecDeque;
use crate::math::Vec3;

// ---------------------------------------------------------------------------
// ImuCameraSync
// ---------------------------------------------------------------------------

/// Buffered IMU samples for interpolating measurements at camera timestamps.
///
/// Samples older than `max_age_ms` are automatically pruned on each push.
pub struct ImuCameraSync {
    /// Ring buffer of `(timestamp_ms, gyro_rads, accel_ms2)`.
    imu_buffer: VecDeque<(u64, Vec3, Vec3)>,
    /// Maximum age of IMU samples to retain [ms].
    max_age_ms: u64,
}

impl ImuCameraSync {
    /// Create a new sync buffer.
    ///
    /// `max_age_ms`: oldest IMU sample to retain.
    /// 200–500 ms is typical; larger values use more memory but allow
    /// late-arriving camera frames to still find a bracket.
    pub fn new(max_age_ms: u64) -> Self {
        Self {
            imu_buffer: VecDeque::with_capacity(64),
            max_age_ms,
        }
    }

    /// Push a new IMU sample.
    ///
    /// - `t_ms`:      timestamp of this measurement [ms]
    /// - `gyro`:      angular velocity [rad/s, body frame]
    /// - `accel`:     specific force [m/s², body frame]
    pub fn push_imu(&mut self, t_ms: u64, gyro: Vec3, accel: Vec3) {
        self.imu_buffer.push_back((t_ms, gyro, accel));

        // Prune samples older than max_age_ms.
        let newest = t_ms;
        while let Some(&(t_old, _, _)) = self.imu_buffer.front() {
            if newest.saturating_sub(t_old) > self.max_age_ms {
                self.imu_buffer.pop_front();
            } else {
                break;
            }
        }
    }

    /// Query the interpolated IMU measurement at timestamp `t_ms`.
    ///
    /// Returns `None` if:
    /// - The buffer is empty.
    /// - `t_ms` is before the oldest sample.
    /// - `t_ms` is after the newest sample.
    ///
    /// Otherwise linearly interpolates between the two samples bracketing
    /// `t_ms`.
    pub fn query_at(&self, t_ms: u64) -> Option<(Vec3, Vec3)> {
        if self.imu_buffer.len() < 2 { return None; }

        // Find the two samples bracketing t_ms.
        let mut before_idx = None;
        let mut after_idx  = None;

        for (i, &(t, _, _)) in self.imu_buffer.iter().enumerate() {
            if t <= t_ms {
                before_idx = Some(i);
            } else if after_idx.is_none() {
                after_idx = Some(i);
                break;
            }
        }

        let bi = before_idx?;
        let ai = after_idx?;

        let (t0, g0, a0) = self.imu_buffer[bi];
        let (t1, g1, a1) = self.imu_buffer[ai];

        if t1 == t0 { return Some((g0, a0)); }

        let alpha = (t_ms - t0) as f32 / (t1 - t0) as f32;

        let gyro  = lerp_vec3(g0, g1, alpha);
        let accel = lerp_vec3(a0, a1, alpha);

        Some((gyro, accel))
    }

    /// Number of IMU samples currently buffered.
    pub fn len(&self) -> usize { self.imu_buffer.len() }

    /// True if no IMU samples are buffered.
    pub fn is_empty(&self) -> bool { self.imu_buffer.is_empty() }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

#[inline]
fn lerp_vec3(a: Vec3, b: Vec3, t: f32) -> Vec3 {
    Vec3::new(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
    )
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sync_interpolation() {
        let mut sync = ImuCameraSync::new(1000);

        let g0 = Vec3::new(0.0, 0.0, 0.0);
        let g1 = Vec3::new(1.0, 2.0, 3.0);
        let a0 = Vec3::new(0.0, 0.0, 9.81);
        let a1 = Vec3::new(0.1, 0.2, 9.81);

        sync.push_imu(100, g0, a0);
        sync.push_imu(200, g1, a1);

        // Query at midpoint (t=150 ms) → should be exactly half.
        let (g, a) = sync.query_at(150).expect("should interpolate");
        assert!((g.x - 0.5).abs() < 1e-5, "gyro.x at midpoint: {}", g.x);
        assert!((g.y - 1.0).abs() < 1e-5, "gyro.y at midpoint: {}", g.y);
        assert!((g.z - 1.5).abs() < 1e-5, "gyro.z at midpoint: {}", g.z);
        assert!((a.x - 0.05).abs() < 1e-5, "accel.x at midpoint: {}", a.x);
    }

    #[test]
    fn sync_exact_sample_returned() {
        let mut sync = ImuCameraSync::new(1000);
        let g = Vec3::new(0.1, 0.2, 0.3);
        let a = Vec3::new(0.0, 0.0, 9.81);
        sync.push_imu(100, g, a);
        sync.push_imu(200, Vec3::zero(), Vec3::zero());

        let (gq, _) = sync.query_at(100).unwrap();
        assert!((gq.x - 0.1).abs() < 1e-5);
    }

    #[test]
    fn sync_before_oldest_is_none() {
        let mut sync = ImuCameraSync::new(1000);
        sync.push_imu(100, Vec3::zero(), Vec3::zero());
        sync.push_imu(200, Vec3::zero(), Vec3::zero());
        assert!(sync.query_at(50).is_none(),
            "query before oldest sample should return None");
    }

    #[test]
    fn sync_after_newest_is_none() {
        let mut sync = ImuCameraSync::new(1000);
        sync.push_imu(100, Vec3::zero(), Vec3::zero());
        sync.push_imu(200, Vec3::zero(), Vec3::zero());
        assert!(sync.query_at(300).is_none(),
            "query after newest sample should return None");
    }

    #[test]
    fn sync_old_samples_pruned() {
        let mut sync = ImuCameraSync::new(100); // only keep 100 ms
        for t in [0u64, 50, 100, 150, 200, 250].iter() {
            sync.push_imu(*t, Vec3::zero(), Vec3::zero());
        }
        // After pushing t=250, samples at t=0,50,100 (age > 100 ms) should be gone.
        assert!(sync.query_at(10).is_none(), "t=10 should have been pruned");
        assert!(sync.query_at(150).is_some(), "t=150 should still be in buffer");
    }

    #[test]
    fn sync_empty_buffer_returns_none() {
        let sync = ImuCameraSync::new(1000);
        assert!(sync.query_at(100).is_none());
    }
}
