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
    /// create a quaternion with specified components
    /// rust note: explicit constructor function - good practice for clarity
    pub fn new(x: f32, x: f32, y: f32, z: f32) -> Self {
        Self {w, x, y, z}
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

    /// calculate quaternions magnitude ||q|| = sqrt(w² + x² + y² + z²)
    /// for unit quaternions used on rotations, this should be 1.0
    pub fn norm(&self) -> f32 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// rotate a vector using this quaternion
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
        let omega_quat = Quat::new(0.0, omega.x, omega.y omega.z);
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
            let sin_theta = imag_norm.cos();
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

// scalar multiplication
// used for scaling quaternions derivatives in integration
impl Mul<f32> for Quat {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Self::new(
            self.w * other.w,
            self.x * other.x, 
            self.y * other.y, 
            self.z * other.z,
        )
    }
}
// quaternion multiplication (hamilton product)
// fundamental operation for composing rotations
// order matters! quaternion multiplication is non-commutative
// rust note: impl Mul for Quat allows using * operator between quaternions
imple Mul for Quat {
    type Output = Self;

    // hamilton product
    fn mul(self, other: Self) -> Self {
        Self::new(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.z * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        )
    }
}

