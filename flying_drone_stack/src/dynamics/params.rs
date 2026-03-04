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
    /// Motor time constant [s] for first-order motor dynamics (controls bandwidth)
    pub motor_time_constant: f32,
}

impl MultirotorParams {
    /// Create parameters for Bitcraze Crazyflie 2.1
    /// Reference platform for course - 27g nano quadcopter
    /// Values from firmware: https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/controller/controller_lee.c
    /// and: System Identification of the Crazyflie 2.0 Nano Quadrocopter
    ///      BA theses, Julian Foerster, ETHZ
    pub fn crazyflie() -> Self {
        Self {
            mass: 0.027,           // 27 grams (firmware: CF_MASS = 0.027 kg)
            arm_length: 0.046,     // 46mm motor-to-motor / 2
            inertia: [
                // Firmware controller_lee.c: .J = {16.571710e-6, 16.655602e-6, 29.261652e-6}
                [16.571710e-6, 0.0, 0.0],  // Jxx (roll inertia)  [kg·m²]
                [0.0, 16.655602e-6, 0.0],  // Jyy (pitch inertia) [kg·m²]
                [0.0, 0.0, 29.261652e-6],  // Jzz (yaw inertia)   [kg·m²]
            ],
            gravity: 9.81,         // Standard Earth gravity
            kf: 2.5e-6,           // Thrust coefficient (empirically identified)
            kt: 1.0e-7,           // Torque coefficient
            dt: 0.01,             // 10ms time step (100 Hz control rate)
            motor_time_constant: 0.03, // 30 ms motor time constant (reasonable for small brushless)
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

        // Roll torque (right motors faster = positive roll)
        let tau_x = self.kf * l_sqrt2 * (omega2_sq + omega3_sq - omega1_sq - omega4_sq);
        // Pitch torque (back motors faster = positive pitch)
        let tau_y = self.kf * l_sqrt2 * (omega3_sq + omega4_sq - omega1_sq - omega2_sq);
        // Yaw torque (CW motors 1,3 positive, CCW motors 2,4 negative)
        // CW motors produce clockwise reaction torque (positive yaw)
        let tau_z = self.kt * (omega1_sq - omega2_sq + omega3_sq - omega4_sq);

        (f, Vec3::new(tau_x, tau_y, tau_z))
    }

    /// Compute angular acceleration from torque and angular velocity
    /// Euler's equation: α = J⁻¹(τ - ω × Jω)
    /// The ω × Jω term represents gyroscopic effects
    /// Rust note: Takes references (&) to avoid moving/copying data
    pub fn angular_acceleration(&self, omega: Vec3, tau: Vec3) -> Vec3 {
        // Gyroscopic term: ω × Jω
        let omega_cross_jomega = omega.cross(&Self::matrix_vec_mul(&self.inertia, &omega));

        // For diagonal inertia, inverse is just 1/Jii for each component
        let j11 = self.inertia[0][0];
        let j22 = self.inertia[1][1];
        let j33 = self.inertia[2][2];

        Vec3::new(
            (tau.x - omega_cross_jomega.x) / j11,
            (tau.y - omega_cross_jomega.y) / j22,
            (tau.z - omega_cross_jomega.z) / j33,
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


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crazyflie_params() {
        let params = MultirotorParams::crazyflie();

        assert_eq!(params.mass, 0.027);
        assert_eq!(params.arm_length, 0.046);
        assert_eq!(params.gravity, 9.81);
        assert_eq!(params.kf, 2.5e-6);
        assert_eq!(params.kt, 1.0e-7);
        assert_eq!(params.dt, 0.01);

        // Check inertia matrix — must match firmware controller_lee.c exactly
        assert!((params.inertia[0][0] - 16.571710e-6).abs() < 1e-10);
        assert!((params.inertia[1][1] - 16.655602e-6).abs() < 1e-10);
        assert!((params.inertia[2][2] - 29.261652e-6).abs() < 1e-10);
        assert_eq!(params.inertia[0][1], 0.0); // Off-diagonal should be zero
    }

    #[test]
    fn test_motor_speeds_to_forces_torques() {
        let params = MultirotorParams::crazyflie();

        // Test hover condition (all motors equal)
        let hover_action = MotorAction::hover();
        let (thrust, torque) = params.motor_speeds_to_forces_torques(&hover_action);

        // Hover thrust should equal weight
        let expected_thrust = params.mass * params.gravity;
        assert!((thrust - expected_thrust).abs() < 1e-3);

        // Hover torque should be zero (balanced motors)
        assert!(torque.norm() < 1e-6);
    }

    #[test]
    fn test_motor_speeds_to_forces_torques_roll() {
        let params = MultirotorParams::crazyflie();

        // Test roll torque (left motors faster than right)
        let action = MotorAction::new(
            25000.0, // Front-left (slow)
            30000.0, // Front-right (fast)
            30000.0, // Back-right (fast)
            25000.0, // Back-left (slow)
        );
        let (_thrust, torque) = params.motor_speeds_to_forces_torques(&action);

        // Should have some roll torque (sign depends on motor configuration)
        assert!(torque.x.abs() > 0.0);
        assert!(torque.norm() > 0.0); // Should have some torque
    }

    #[test]
    fn test_motor_speeds_to_forces_torques_pitch() {
        let params = MultirotorParams::crazyflie();

        // Test pitch torque (back motors faster than front)
        let action = MotorAction::new(
            25000.0, // Front-left (slow)
            25000.0, // Front-right (slow)
            30000.0, // Back-right (fast)
            30000.0, // Back-left (fast)
        );
        let (_thrust, torque) = params.motor_speeds_to_forces_torques(&action);

        // Should have some pitch torque (sign depends on motor configuration)
        assert!(torque.y.abs() > 0.0);
        assert!(torque.norm() > 0.0); // Should have some torque
    }

    #[test]
    fn test_motor_speeds_to_forces_torques_yaw() {
        let params = MultirotorParams::crazyflie();

        // Test yaw torque (CW motors faster than CCW)
        let action = MotorAction::new(
            30000.0, // Front-left (CW, fast)
            25000.0, // Front-right (CCW, slow)
            30000.0, // Back-right (CW, fast)
            25000.0, // Back-left (CCW, slow)
        );
        let (_thrust, torque) = params.motor_speeds_to_forces_torques(&action);

        // Should have positive yaw torque
        assert!(torque.x.abs() < 1e-6); // No roll
        assert!(torque.y.abs() < 1e-6); // No pitch
        assert!(torque.z > 0.0);
    }

    #[test]
    fn test_angular_acceleration() {
        let params = MultirotorParams::crazyflie();

        // Test with zero angular velocity
        let omega = Vec3::zero();
        let tau = Vec3::new(1e-5, 2e-5, 3e-5); // Small torques
        let alpha = params.angular_acceleration(omega, tau);

        // Should be tau / J for each component
        assert!((alpha.x - tau.x / params.inertia[0][0]).abs() < 1e-6);
        assert!((alpha.y - tau.y / params.inertia[1][1]).abs() < 1e-6);
        assert!((alpha.z - tau.z / params.inertia[2][2]).abs() < 1e-6);
    }

    #[test]
    fn test_angular_acceleration_with_gyroscopic() {
        let params = MultirotorParams::crazyflie();

        // Test with non-zero angular velocity (gyroscopic effects)
        let omega = Vec3::new(10.0, 20.0, 30.0); // High angular velocity
        let tau = Vec3::zero(); // No external torque
        let alpha = params.angular_acceleration(omega, tau);

        // Should have non-zero acceleration due to gyroscopic term
        // The exact values depend on the inertia matrix and omega
        assert!(alpha.norm() > 0.0);
    }

    #[test]
    fn test_matrix_vec_mul() {
        let matrix = [
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [7.0, 8.0, 9.0],
        ];
        let vec = Vec3::new(1.0, 1.0, 1.0);
        let result = MultirotorParams::matrix_vec_mul(&matrix, &vec);

        assert_eq!(result.x, 6.0);  // 1+2+3
        assert_eq!(result.y, 15.0); // 4+5+6
        assert_eq!(result.z, 24.0); // 7+8+9
    }
}