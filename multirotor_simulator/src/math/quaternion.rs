//! Quaternion implementation for 3D rotations

//! provided robust, numerically stable representation of 3D rotations
// using unit quarternions. Avoids gimbal lock and provides smooth interpolation

use super::vec3::Vec3;
use std::ops::{Add, Mul};

/// Unit quaternion representing a 3D rotation
/// quaternion q = w + xi + yj + zk where i² = j² = k² = ijk = -1
/// for unit quaternions (||q|| = 1), represents rotation by angle theta around axis n:
/// q = (cos(theta/2), n*sin(theta/2))

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Quat {
    pub w: f32, // real component (scalar part)
    pub x: f32, // imaginary i component
    pub y: f32, // imaginary j component
    pub z: f32, // imaginary k component
}

impl Quat {
    /// Create a quaternion with specified components
    /// Rust note: Explicit constructor function - good practice for clarity
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// create identity quaternion (no rotation)
    /// identity quaternion: q = (1, 0, 0, 0) represents zero rotation
    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0)
    }

    /// create from axis-angle representation
    /// convert rotation axis n and angle theta to quaternion q = (cos(theta/2) n*sin(theta/2))
    /// rust note: takes vec3 by value (copy trait allows this without move)
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Self {
        let half_angle = angle * 0.5;
        let sin_half = half_angle.sin();
        let cos_half = half_angle.cos();
        let axis_norm = axis.normalize(); // ensure axis is unit length
    

        Self {
            w: cos_half,
            x: axis_norm.x * sin_half,
            y: axis_norm.y * sin_half,
            z: axis_norm.z * sin_half,
        }
    }


    /// calcualate quaternion conjugate q* = (w, -x, -y, -z)
    /// for unit quaternions, conjugate equals invserse: q* = q⁻¹
    /// used to reverse rotations and rotate vectors
    pub fn conjugate(&self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Calculate quaternion magnitude ||q|| = sqrt(w² + x² + y² + z²)
    /// For unit quaternions used in rotations, this should be 1.0
    pub fn norm(&self) -> f32 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Normalize to unit quaternion (||q|| = 1)
    /// Essential after numerical integration to prevent drift from unit constraint
    pub fn normalize(&self) -> Self {
        let n = self.norm();
        if n > 0.0 {
            Self::new(self.w / n, self.x / n, self.y / n, self.z / n)
        } else {
            Self::identity()  // Fallback to no rotation if magnitude is zero
        }
    }

    /// Rotate a vector using this quaternion
    /// this is more efficient than converting to rotation matrix
    pub fn rotate_vector(&self, v: Vec3) -> Vec3 {
        let q_vec = Quat::new(0.0, v.x, v.y, v.z); // convert vector to pure quaternion
        let result = *self * q_vec * self.conjugate(); // apply rotation
        Vec3::new(result.x, result.y, result.z) // extract vector part
    }

    /// calculate time derivative given angular velocity
    /// this is the fundamental differential equation for quaternion integration
    /// rust note: takes reference to self, return owned Quat
    pub fn derivative(&self, omega: Vec3) -> Quat {
        let omega_quat = Quat::new(0.0, omega.x, omega.y, omega.z);
        (*self * omega_quat) * 0.5
    }

    /// compute quaternion exponential exp(q)
    /// maps quaternion to 3D rotation, used in exponential map integration
    pub fn exp(&self) -> Quat {
        let imag_norm = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();

        if imag_norm < 1e-10 { // small angle approximation
            Quat::new(self.w.exp(), 0.0, 0.0, 0.0)
        } else {
            let exp_w = self.w.exp();
            let cos_theta = imag_norm.cos();
            let sin_theta = imag_norm.sin();
            let scale = sin_theta / imag_norm;

            Quat::new(
                exp_w * cos_theta,
                exp_w * scale * self.x,
                exp_w * scale * self.y,
                exp_w * scale * self.z,
            )
        }
    }

    // integrate using axis-angle approximation
    // used exponential map
    // more accurate then linear integration for large angular velocities
    pub fn integrate(&self, omega: Vec3, dt: f32) -> Quat {
        let omega_norm = omega.norm();
        if omega_norm > 1e-10 {
            let axis = omega.normalize();
            let angle = omega_norm * dt;
            let delta_q = Quat::from_axis_angle(axis, angle);
            (*self * delta_q).normalize() // compose rotations and renormalize
        } else {
            self.normalize() // no rotation if angular velocity is zero
        }
    }


    /// Convert quaternion to rotation matrix
    /// Returns 3x3 rotation matrix R such that v' = R * v
    /// For quaternion q = (w, x, y, z), the rotation matrix is:
    /// R = [[1-2y²-2z², 2xy-2wz, 2xz+2wy],
    ///      [2xy+2wz, 1-2x²-2z², 2yz-2wx],
    ///      [2xz-2wy, 2yz+2wx, 1-2x²-2y²]]
    pub fn to_rotation_matrix(&self) -> [[f32; 3]; 3] {
        let w = self.w;
        let x = self.x;
        let y = self.y;
        let z = self.z;

        let ww = w * w;
        let xx = x * x;
        let yy = y * y;
        let zz = z * z;
        let wx = w * x;
        let wy = w * y;
        let wz = w * z;
        let xy = x * y;
        let xz = x * z;
        let yz = y * z;

        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]
    }

    // integrate using exponential map (more stable)
    // alternative integration using quaternion exponential
    // generally more numerically stable than axis-angle for large rotations
    pub fn integrate_exponential(&self, omega: Vec3, dt: f32) -> Quat {
        let omega_quat = Quat::new(0.0, omega.x * dt * 0.5, omega.y * dt * 0.5, omega.z * dt * 0.5);
        let delta_q = omega_quat.exp();
        (*self * delta_q).normalize()
    }
}

// quaternion addition (for integration)
// rust note: component-wise addition, used in euler integration
// not a geometric quaternion operation, but needed for numerical methods
impl Add for Quat {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self::new(
            self.w + other.w,
            self.x + other.x,
            self.y + other.y,
            self.z + other.z,
        )
    }
}

/// Scalar multiplication
/// Used for scaling quaternion derivatives in integration
impl Mul<f32> for Quat {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Self::new(
            self.w * scalar,
            self.x * scalar,
            self.y * scalar,
            self.z * scalar,
        )
    }
}

/// Quaternion multiplication (Hamilton product)
/// Fundamental operation for composing rotations: q = q1 ⊗ q2
/// Order matters! Quaternion multiplication is non-commutative
/// Rust note: `impl Mul for Quat` allows using * operator between quaternions
impl Mul for Quat {
    type Output = Self;

    // hamilton product
    fn mul(self, other: Self) -> Self {
        Self::new(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        )
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_quat_new() {
        let q = Quat::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 2.0);
        assert_eq!(q.y, 3.0);
        assert_eq!(q.z, 4.0);
    }

    #[test]
    fn test_quat_identity() {
        let q = Quat::identity();
        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 0.0);
        assert_eq!(q.y, 0.0);
        assert_eq!(q.z, 0.0);
    }

    #[test]
    fn test_quat_from_axis_angle() {
        // 90 degree rotation around z-axis
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let angle = PI / 2.0;
        let q = Quat::from_axis_angle(axis, angle);

        // Should be (cos(45°), 0, 0, sin(45°))
        let expected_cos = (PI / 4.0).cos();
        let expected_sin = (PI / 4.0).sin();

        assert!((q.w - expected_cos).abs() < 1e-6);
        assert!((q.x - 0.0).abs() < 1e-6);
        assert!((q.y - 0.0).abs() < 1e-6);
        assert!((q.z - expected_sin).abs() < 1e-6);
    }

    #[test]
    fn test_quat_conjugate() {
        let q = Quat::new(1.0, 2.0, 3.0, 4.0);
        let conj = q.conjugate();
        assert_eq!(conj.w, 1.0);
        assert_eq!(conj.x, -2.0);
        assert_eq!(conj.y, -3.0);
        assert_eq!(conj.z, -4.0);
    }

    #[test]
    fn test_quat_norm() {
        let q = Quat::new(1.0, 2.0, 3.0, 4.0);
        let norm = q.norm();
        assert!((norm - (30.0_f32).sqrt()).abs() < 1e-6);

        let identity = Quat::identity();
        assert!((identity.norm() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_quat_normalize() {
        let q = Quat::new(1.0, 2.0, 3.0, 4.0);
        let normalized = q.normalize();
        assert!((normalized.norm() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_quat_rotate_vector() {
        // 90 degree rotation around z-axis
        let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), PI / 2.0);
        let v = Vec3::new(1.0, 0.0, 0.0);
        let rotated = q.rotate_vector(v);

        // Should rotate (1,0,0) to (0,1,0)
        assert!((rotated.x - 0.0).abs() < 1e-6);
        assert!((rotated.y - 1.0).abs() < 1e-6);
        assert!((rotated.z - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_quat_derivative() {
        let q = Quat::identity();
        let omega = Vec3::new(0.0, 0.0, 1.0); // Rotation around z-axis
        let dq = q.derivative(omega);

        // For identity quaternion, derivative should be (0, 0, 0, 0.5)
        assert!((dq.w - 0.0).abs() < 1e-6);
        assert!((dq.x - 0.0).abs() < 1e-6);
        assert!((dq.y - 0.0).abs() < 1e-6);
        assert!((dq.z - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_quat_exp() {
        let q = Quat::new(0.0, 0.0, 0.0, PI / 4.0); // Pure quaternion for small angle
        let exp_q = q.exp();

        // Should be approximately (cos(θ/2), sin(θ/2) * axis)
        let half_angle = PI / 4.0; // θ = π/4, so θ/2 = π/8? No, for pure quaternion q = (0,0,0,θ), |q| = θ
        let angle = PI / 4.0; // |imag| = π/4
        assert!((exp_q.w - angle.cos()).abs() < 1e-6);
        assert!((exp_q.z - angle.sin()).abs() < 1e-6);
    }

    #[test]
    fn test_quat_integrate() {
        let q = Quat::identity();
        let omega = Vec3::new(0.0, 0.0, 1.0); // Constant angular velocity
        let dt = 0.1;
        let q_new = q.integrate(omega, dt);

        // Should be a small rotation around z-axis
        assert!(q_new.w > 0.0); // Still mostly identity
        assert!((q_new.z - (0.1f32 * 0.5f32).sin()).abs() < 1e-6); // sin(angle/2) where angle = omega_norm * dt = 0.1
    }

    #[test]
    fn test_quat_to_rotation_matrix() {
        // Identity quaternion should give identity matrix
        let q = Quat::identity();
        let matrix = q.to_rotation_matrix();

        assert!((matrix[0][0] - 1.0).abs() < 1e-6);
        assert!((matrix[0][1] - 0.0).abs() < 1e-6);
        assert!((matrix[0][2] - 0.0).abs() < 1e-6);
        assert!((matrix[1][0] - 0.0).abs() < 1e-6);
        assert!((matrix[1][1] - 1.0).abs() < 1e-6);
        assert!((matrix[1][2] - 0.0).abs() < 1e-6);
        assert!((matrix[2][0] - 0.0).abs() < 1e-6);
        assert!((matrix[2][1] - 0.0).abs() < 1e-6);
        assert!((matrix[2][2] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_quat_multiplication() {
        let q1 = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), PI / 4.0);
        let q2 = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), PI / 4.0);
        let q3 = q1 * q2; // Should be 90 degree rotation

        // 45° + 45° = 90° rotation around z-axis
        let expected = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), PI / 2.0);

        assert!((q3.w - expected.w).abs() < 1e-6);
        assert!((q3.x - expected.x).abs() < 1e-6);
        assert!((q3.y - expected.y).abs() < 1e-6);
        assert!((q3.z - expected.z).abs() < 1e-6);
    }

    #[test]
    fn test_quat_identity_operations() {
        let q = Quat::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), PI / 2.0);
        let identity = Quat::identity();

        // Identity should not change rotation when multiplied
        let result1 = q * identity;
        let result2 = identity * q;

        assert!((result1.w - q.w).abs() < 1e-6);
        assert!((result1.x - q.x).abs() < 1e-6);
        assert!((result1.y - q.y).abs() < 1e-6);
        assert!((result1.z - q.z).abs() < 1e-6);

        assert!((result2.w - q.w).abs() < 1e-6);
        assert!((result2.x - q.x).abs() < 1e-6);
        assert!((result2.y - q.y).abs() < 1e-6);
        assert!((result2.z - q.z).abs() < 1e-6);
    }
}
