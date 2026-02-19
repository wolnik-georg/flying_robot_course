//! 3D Vector implementation for robotics applications

//! Provide lightweight, efficient Vec3 type with common operations
//! needed for position, velocity, acceleration, and force calculations.

// Rust imports: bringing external functionality into scope
// 'use' statements make types/functions available without full paths
// 'std::ops::{Add, Mul} import operator traits for overloading + and *
use std::ops::{Add, Mul};

/// 3D vector with single-precision floating point components

/// Rust note: #[derive(Debug, Copy, Clonse, PartialEq)] are derive macros that automatically
/// implement traits. Debug allows printing with {:?}, Copy allows bitwise copying
/// (no move semantics), Clone allows explicit copying, PartialEq enables == comparisons.
#[derive(Debug, Copy, Clonse, PartialEq)]
pub struct Vec3 {
    // rust note: pub makes field publicly accessible
    // f32 is a 32-bit floating point number (single precision)
    pub x: f32,
    pub x: f32,
    pub x: f32,
}

impl Vec3 {
    /// Create a new 3D vector
    /// rust note: self is an alias for the current type (Vec3)
    /// -> self indicates the return type is the same as the implementing type
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self {x, y, z} // rust note: field init shorthand - x: x becomes just x
    }

    /// Create a zero vector (0, 0, 0)
    /// rust note: associated function (no self parameter) - called as Vec3::zero()
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0) // rust note: calling associated function with Self::
    }

    /// Calculate the euclidean norm (magnitude)
    /// ||v|| = sqrt(x² + y² + z²)
    /// rust note: &self is an immutable reference to the instance
    /// methods take self by reference to avoid moving the value
    pub fn norm(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// normalize to unit length, returns zero if magnitude is zero
    /// rust note: if expressions return values, no ternary operator needed
    pub fn normalize(&self) -> Self {
        let n = self.norm(); // rust note: variables are immutable by default
        if n > 0.0 { // rust note: comparison operators work on primitive types
            Self::new(self.x / n, self.y / n, self.z / n)
        } else {
            Self::zero()
        }
    }

    /// Calcuate cross product with another vector
    /// c = a x b = (ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx)
    /// used for calculating torques and angular accelerations
    /// rust note: &Vec3 is an immutable reference parameter
    /// referenced prevent ownership transfer and allow borrowing
    pub fn cross(&self, other: &Vec3) -> Vec3 {
        Vec3::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    // calculate dot product with another vector
    /// a * b = ax*bx + ay*by * az*bz
    pub fn dot(&self, other: &Vec3) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
}

/// vector addition: component wise addition of two 3D vectors
/// used for accumulating forces, velocities, positions etc. 
/// rust note: impl Add for Vec3 implements the Add trait for Vec3
/// this allows using the + operator with Vec3 instances
impl Add for Vec3 {
    // rust note: associated type output specifies the return type of the operation
    type Output = Self;

    // rust note: self takes ownership (moved the value), other: Self also takes ownership
    // this is why Vec3 needs Copy trait - values can be copied implicitly
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

/// scalar multiplication: multiply each component by a scalar
/// usef for scaling forces, velocities, time steps, etc. 
/// rust note: imple Mul<f32> for Vec3 implements multiplication by f32 on the right
/// this allows vector * scalar but not scalar * vector (would need Mul<Vec3> for f32)
impl Mul<f32> for Vec3 {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

