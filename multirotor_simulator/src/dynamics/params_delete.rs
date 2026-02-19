//! Physical parameters for multirotor systems
//!
//! Defines aircraft-specific constants and dynamics calculations

use crate::math::Vec3;
use super::state::MotorAction;

/// Physical parameters defining multirotor dynamics
/// These constants define the aircraft's physical properties
/// Rust note: #[derive(Debug, Clone)] auto-implements debugging and cloning
#[derive(Debug, Clone)]
pub struct MultirotorParams {
    /// Total mass [kg]
    pub mass: f32,
    /// Arm length (center to motor) [m]
    /// Distance from center of mass to motor centerline
    pub arm_length: f32,
    /// Moment of inertia matrix [kg⋅m²] (diagonal)
    /// For symmetric quadrotors: Jxx ≈ Jyy, Jzz ≈ 2*Jxx
    /// Rust note: [[f32; 3]; 3] is a 3x3 array (array of arrays)
    pub inertia: [[f32; 3]; 3],
    /// Gravitational acceleration [m/s²]
    pub gravity: f32,
    /// Force constant: thrust = kf * ω² [N/(rad/s)²]
    /// Depends on propeller geometry and air density
    pub kf: f32,
    /// Torque constant: torque = kt * ω² [Nm/(rad/s)²]
    /// Reaction torque from propeller drag
    pub kt: f32,
    /// Integration time step [s]
    /// Smaller dt = more accurate but slower simulation
    pub dt: f32,
}

impl MultirotorParams {
    /// Create parameters for Bitcraze Crazyflie 2.1
    /// Reference platform for course - 27g nano quadcopter
    /// Values from: https://www.bitcraze.io/documentation/system/platform/cf2-specifications/
    pub fn crazyflie() -> Self {
        Self {
            mass: 0.027,           // 27 grams
            arm_length: 0.046,     // 46mm motor-to-motor / 2
            inertia: [
                [1.7e-5, 0.0, 0.0],    // Jxx (roll inertia)
                [0.0, 1.7e-5, 0.0],    // Jyy (pitch inertia)
                [0.0, 0.0, 2.9e-5],    // Jzz (yaw inertia)
            ],
            gravity: 9.81,         // Standard Earth gravity
            kf: 2.5e-6,           // Thrust coefficient (empirically identified)
            kt: 1.0e-7,           // Torque coefficient
            dt: 0.01,             // 10ms time step (100 Hz control rate)
        }
    }

    /// Convert motor speeds to thrust force and body torques
    /// Returns (total_thrust, torque_vector)
    /// 
    /// Physics:
    /// - Thrust from each motor: Fi = kf * ωi²
    /// - Total thrust: F = Σ Fi (all motors)
    /// - Roll/pitch torques: lever arm × motor thrust difference
    /// - Yaw torque: Σ (±kt * ωi²) from motor reaction torques
    pub fn motor_speeds_to_forces_torques(&self, action: &MotorAction) -> (f32, Vec3) {
        let omega1_sq = action.omega1_sq;
        let omega2_sq = action.omega2_sq;
        let omega3_sq = action.omega3_sq;
        let omega4_sq = action.omega4_sq;

        // Total thrust force (sum of all motor thrusts)
        let f = self.kf * (omega1_sq + omega2_sq + omega3_sq + omega4_sq);

        // Torques for X configuration
        // Motor positions at 45° angles, so lever arm = L/√2
        let sqrt2 = 2.0_f32.sqrt();
        let l_sqrt2 = self.arm_length / sqrt2;

        // Roll torque (left vs right motors)
        let tau_x = self.kf * l_sqrt2 * (omega4_sq - omega2_sq);
        // Pitch torque (front vs back motors)
        let tau_y = self.kf * l_sqrt2 * (omega3_sq - omega1_sq);
        // Yaw torque (CW vs CCW motors, reaction torque)
        let tau_z = self.kt * (omega1_sq - omega2_sq + omega3_sq - omega4_sq);

        (f, Vec3::new(tau_x, tau_y, tau_z))
    }

    /// Compute angular acceleration from torque and angular velocity
    /// Euler's equation: α = J⁻¹(τ - ω × Jω)
    /// The ω × Jω term represents gyroscopic effects
    /// Rust note: Takes references (&) to avoid moving/copying data
    pub fn angular_acceleration(&self, omega: Vec3, tau: Vec3) -> Vec3 {
        // Gyroscopic term: ω × Jω
        let omega_cross_Jomega = omega.cross(&Self::matrix_vec_mul(&self.inertia, &omega));

        // For diagonal inertia, inverse is just 1/Jii for each component
        let j11 = self.inertia[0][0];
        let j22 = self.inertia[1][1];
        let j33 = self.inertia[2][2];

        Vec3::new(
            (tau.x - omega_cross_Jomega.x) / j11,
            (tau.y - omega_cross_Jomega.y) / j22,
            (tau.z - omega_cross_Jomega.z) / j33,
        )
    }

    /// Matrix-vector multiplication helper
    /// Computes J * ω for 3x3 matrix times 3D vector
    /// Rust note: &[[f32; 3]; 3] is a reference to a 3x3 array
    fn matrix_vec_mul(matrix: &[[f32; 3]; 3], vec: &Vec3) -> Vec3 {
        Vec3::new(
            matrix[0][0] * vec.x + matrix[0][1] * vec.y + matrix[0][2] * vec.z,
            matrix[1][0] * vec.x + matrix[1][1] * vec.y + matrix[1][2] * vec.z,
            matrix[2][0] * vec.x + matrix[2][1] * vec.y + matrix[2][2] * vec.z,
        )
    }
}
