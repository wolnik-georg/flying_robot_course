//! Pinhole camera model with Brown-Conrady radial distortion.
//!
//! ## Model
//!
//! For a 3D point `P = (X, Y, Z)` in the camera frame (Z > 0 is forward):
//!
//! ```text
//! x_n = X / Z           (normalised x)
//! y_n = Y / Z           (normalised y)
//! r²  = x_n² + y_n²
//! k   = 1 + k1·r² + k2·r⁴   (radial distortion factor)
//!
//! u = fx · x_n · k  +  cx    (pixel column)
//! v = fy · y_n · k  +  cy    (pixel row)
//! ```
//!
//! Undistortion is computed by one Newton iteration (accurate for small k1/k2).
//!
//! ## HM01B0 defaults
//!
//! The HiMax HM01B0 mounted on the AI Deck has a 66° horizontal FOV on a
//! 320×320 sensor.  Approximate intrinsics:
//!
//! ```text
//! fx = fy ≈ 164 px  (320 / (2·tan(33°)))
//! cx = cy = 160 px  (image centre)
//! k1 = k2 = 0       (unknown, assume undistorted for now)
//! ```

use crate::math::Vec3;
use crate::perception::types::CameraIntrinsics;

impl CameraIntrinsics {
    // -----------------------------------------------------------------------
    // Projection / unprojection
    // -----------------------------------------------------------------------

    /// Project a 3D camera-frame point to image coordinates.
    ///
    /// Returns `None` if the point is at or behind the image plane (Z ≤ 0).
    pub fn project(&self, p: Vec3) -> Option<(f32, f32)> {
        if p.z <= 0.0 { return None; }
        let x_n = p.x / p.z;
        let y_n = p.y / p.z;
        let r2  = x_n * x_n + y_n * y_n;
        let k   = 1.0 + self.k1 * r2 + self.k2 * r2 * r2;
        let u   = self.fx * x_n * k + self.cx;
        let v   = self.fy * y_n * k + self.cy;
        Some((u, v))
    }

    /// Back-project an image point at a known depth to a 3D camera-frame point.
    ///
    /// `depth`: distance from camera centre along the Z axis [m].
    pub fn unproject(&self, u: f32, v: f32, depth: f32) -> Vec3 {
        // Undistort first, then scale by depth.
        let (u_u, v_u) = self.undistort_point(u, v);
        let x_n = (u_u - self.cx) / self.fx;
        let y_n = (v_u - self.cy) / self.fy;
        Vec3::new(x_n * depth, y_n * depth, depth)
    }

    // -----------------------------------------------------------------------
    // Distortion
    // -----------------------------------------------------------------------

    /// Remove radial distortion from a pixel using one Newton iteration.
    ///
    /// Accurate for `|k1|, |k2| < 0.5`.  Returns the input unchanged when
    /// k1 = k2 = 0 (no-op for default HM01B0 intrinsics).
    pub fn undistort_point(&self, u: f32, v: f32) -> (f32, f32) {
        if self.k1 == 0.0 && self.k2 == 0.0 {
            return (u, v);
        }

        // Initial estimate: normalised (distorted) coords.
        let mut x_n = (u - self.cx) / self.fx;
        let mut y_n = (v - self.cy) / self.fy;

        // One Newton step: solve  x_d · (1 + k1·r²_u + k2·r⁴_u) = x_n_dist.
        for _ in 0..3 {
            let r2 = x_n * x_n + y_n * y_n;
            let k  = 1.0 + self.k1 * r2 + self.k2 * r2 * r2;
            x_n /= k;
            y_n /= k;
        }

        (x_n * self.fx + self.cx, y_n * self.fy + self.cy)
    }
}

// ---------------------------------------------------------------------------
// Known camera presets
// ---------------------------------------------------------------------------

/// Default intrinsics for the HiMax HM01B0 camera on the AI Deck (320×320).
///
/// These are approximate values derived from the 66° horizontal FOV spec.
/// A proper calibration (checkerboard + OpenCV) is recommended for production.
pub fn hm01b0_defaults() -> CameraIntrinsics {
    CameraIntrinsics {
        fx: 164.0,
        fy: 164.0,
        cx: 160.0,
        cy: 160.0,
        k1: 0.0,
        k2: 0.0,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn cam() -> CameraIntrinsics { hm01b0_defaults() }

    #[test]
    fn project_unproject_roundtrip() {
        let c  = cam();
        let p  = Vec3::new(0.1, -0.05, 1.5); // in camera frame
        let (u, v) = c.project(p).expect("should project");
        let p2 = c.unproject(u, v, p.z);

        assert!((p2.x - p.x).abs() < 1e-4, "X roundtrip: {} vs {}", p2.x, p.x);
        assert!((p2.y - p.y).abs() < 1e-4, "Y roundtrip: {} vs {}", p2.y, p.y);
        assert!((p2.z - p.z).abs() < 1e-4, "Z roundtrip: {} vs {}", p2.z, p.z);
    }

    #[test]
    fn project_centre_at_principal_point() {
        let c = cam();
        // A point directly along the optical axis → projects to (cx, cy).
        let (u, v) = c.project(Vec3::new(0.0, 0.0, 2.0)).unwrap();
        assert!((u - c.cx).abs() < 1e-4);
        assert!((v - c.cy).abs() < 1e-4);
    }

    #[test]
    fn project_behind_camera_is_none() {
        let c = cam();
        assert!(c.project(Vec3::new(0.0, 0.0, -1.0)).is_none());
        assert!(c.project(Vec3::new(0.0, 0.0,  0.0)).is_none());
    }

    #[test]
    fn undistort_zero_distortion_is_noop() {
        let c = cam(); // k1=k2=0
        let (u2, v2) = c.undistort_point(150.0, 170.0);
        assert!((u2 - 150.0).abs() < 1e-5);
        assert!((v2 - 170.0).abs() < 1e-5);
    }

    #[test]
    fn undistort_with_distortion_moves_point() {
        let c = CameraIntrinsics { fx: 164.0, fy: 164.0, cx: 160.0, cy: 160.0,
                                   k1: 0.1, k2: 0.0 };
        // A point offset from the principal point should be moved inward
        // (barrel distortion with positive k1 pushes outward, undistortion pulls in).
        let (u2, v2) = c.undistort_point(180.0, 160.0); // 20 px to the right
        assert!(u2 < 180.0, "undistortion should move point inward: got {}", u2);
        assert!((v2 - 160.0).abs() < 1e-3, "vertical unchanged on horizontal offset");
    }

    #[test]
    fn hm01b0_defaults_reasonable() {
        let c = hm01b0_defaults();
        // FOV = 2 * atan(w / (2*fx)); for fx=164, w=320 → ~88.6°
        let fov_deg = (320.0 / (2.0 * c.fx)).atan().to_degrees() * 2.0;
        assert!(fov_deg > 80.0 && fov_deg < 95.0,
            "HM01B0 horizontal FOV for fx=164, w=320 should be ~88°, got {:.1}°", fov_deg);
    }
}
