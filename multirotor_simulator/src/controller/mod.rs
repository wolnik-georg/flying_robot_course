//! Geometric Controller for Multirotor Trajectory Tracking
//!
//! Implementation of the Lee geometric controller on SE(3) for quadrotor trajectory tracking.
//! Based on: Lee, Taeyoung, Melvin Leok, and N Harris McClamroch.
//! "Geometric tracking control of a quadrotor UAV on SE (3)." 49th IEEE Conference
//! on Decision and Control (CDC), 2010.

use crate::math::Vec3;
use crate::dynamics::{MultirotorState, MultirotorParams};

/// Reference trajectory point for controller input
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryReference {
    /// Desired position [m]
    pub position: Vec3,
    /// Desired velocity [m/s]
    pub velocity: Vec3,
    /// Desired acceleration [m/s²]
    pub acceleration: Vec3,
    /// Desired yaw angle [rad]
    pub yaw: f32,
    /// Desired yaw rate [rad/s]
    pub yaw_rate: f32,
    /// Desired yaw acceleration [rad/s²]
    pub yaw_acceleration: f32,
}

/// Control output from geometric controller
#[derive(Debug, Clone, Copy)]
pub struct ControlOutput {
    /// Total thrust force [N]
    pub thrust: f32,
    /// Body-frame torque [Nm]
    pub torque: Vec3,
}

/// Trait for controllers that can compute control outputs
pub trait Controller {
    fn compute_control(
        &self,
        state: &MultirotorState,
        reference: &TrajectoryReference,
        params: &MultirotorParams,
    ) -> ControlOutput;
}

/// Geometric controller for SE(3) trajectory tracking
///
/// Implements the controller from Lee et al. (2010) with position and attitude control.
#[derive(Debug, Clone)]
pub struct GeometricController {
    /// Position proportional gains [m/s²]
    pub kp: Vec3,
    /// Position derivative gains [N/(m/s)]
    pub kv: Vec3,
    /// Attitude proportional gains [Nm/rad]
    pub kr: Vec3,
    /// Attitude derivative gains [Nm/(rad/s)]
    pub kw: Vec3,
}

impl GeometricController {
    /// Create a new geometric controller with specified gains
    pub fn new(kp: Vec3, kv: Vec3, kr: Vec3, kw: Vec3) -> Self {
        Self { kp, kv, kr, kw }
    }

    /// Create controller with default gains (from official Crazyflie Lee controller)
    /// Reference: controller_lee.c in bitcraze/crazyflie-firmware
    /// Lee controller uses same SE(3) geometric control as our implementation
    pub fn default() -> Self {
        Self {
            kp: Vec3::new(7.0, 7.0, 7.0),            // Position P gains (Kpos_P from Lee controller)
            kv: Vec3::new(4.0, 4.0, 4.0),            // Velocity D gains (Kpos_D from Lee controller)
            kr: Vec3::new(0.007, 0.007, 0.008),      // Attitude P gains (KR from Lee controller)
            kw: Vec3::new(0.00115, 0.00115, 0.002),  // Angular velocity D gains (Komega from Lee controller)
        }
    }

    /// Compute desired rotation matrix from thrust vector and yaw
    ///
    /// The desired rotation matrix Rd aligns the body z-axis with the thrust direction
    /// while respecting the desired yaw angle.
    fn compute_desired_rotation(&self, thrust_force: Vec3, yaw: f32) -> [[f32; 3]; 3] {
        // Normalize thrust direction to get desired z-axis
        let zb_d = thrust_force.normalize();

        // Desired x-axis in world frame (forward direction with yaw)
        let xc = Vec3::new(yaw.cos(), yaw.sin(), 0.0);

        // Desired y-axis (perpendicular to both zb_d and xc)
        // Use yb_d = zb_d × xc, which gives the left direction
        let yb_d_unnorm = zb_d.cross(&xc);
        
        // Check for singularity (when thrust is nearly vertical and aligned with xc)
        let yb_d = if yb_d_unnorm.norm() < 0.01 {
            // Near singularity: use a fallback y-axis
            // If zb_d is nearly vertical, use world y-axis
            Vec3::new(-yaw.sin(), yaw.cos(), 0.0)
        } else {
            yb_d_unnorm.normalize()
        };

        // Re-orthogonalize x-axis: xb_d = yb_d × zb_d
        let xb_d = yb_d.cross(&zb_d).normalize();

        // Construct rotation matrix Rd = [xb_d, yb_d, zb_d]
        [
            [xb_d.x, yb_d.x, zb_d.x],
            [xb_d.y, yb_d.y, zb_d.y],
            [xb_d.z, yb_d.z, zb_d.z],
        ]
    }

    /// Compute rotation error vector eR
    ///
    /// eR = 1/2 (Rd^T R - R^T Rd)^∨
    /// where ^∨ is the vee operator (skew-symmetric matrix to vector)
    fn compute_rotation_error(rd: &[[f32; 3]; 3], r: &[[f32; 3]; 3]) -> Vec3 {
        // Compute Rd^T * R
        let rd_t_r = matmul_transpose_a_f32(rd, r);

        // Compute R^T * Rd
        let r_t_rd = matmul_transpose_a_f32(r, rd);

        // Compute (Rd^T R - R^T Rd)
        let diff = matsub_f32(&rd_t_r, &r_t_rd);

        // Extract skew-symmetric part and convert to vector (vee operator)
        // For a skew-symmetric matrix [[0, -c, b], [c, 0, -a], [-b, a, 0]],
        // the vee operator gives [a, b, c]
        let a = (diff[2][1] - diff[1][2]) / 2.0;
        let b = (diff[0][2] - diff[2][0]) / 2.0;
        let c = (diff[1][0] - diff[0][1]) / 2.0;

        Vec3::new(a, b, c)
    }

    /// Compute angular velocity error eω
    ///
    /// eω = ω - R^T Rd ωd
    fn compute_angular_velocity_error(
        &self,
        omega: Vec3,
        rd: &[[f32; 3]; 3],
        r: &[[f32; 3]; 3],
        omega_d: Vec3,
    ) -> Vec3 {
        // Compute R^T * Rd * ωd
        let r_t = transpose_f32(r);
        let rd_omega_d = matvecmul_f32_mat(rd, omega_d);
        let r_t_rd_omega_d = matvecmul_f32_mat(&r_t, rd_omega_d);

        omega - r_t_rd_omega_d
    }
}

impl Controller for GeometricController {
    fn compute_control(
        &self,
        state: &MultirotorState,
        reference: &TrajectoryReference,
        params: &MultirotorParams,
    ) -> ControlOutput {
        // Position control law: Fd = m(p̈d + Kp ep + Kv ev + gez)
        let ep = reference.position - state.position;
        let ev = reference.velocity - state.velocity;

        let feedforward = reference.acceleration
            + Vec3::new(
                self.kp.x * ep.x,
                self.kp.y * ep.y,
                self.kp.z * ep.z,
            )
            + Vec3::new(
                self.kv.x * ev.x,
                self.kv.y * ev.y,
                self.kv.z * ev.z,
            )
            + Vec3::new(0.0, 0.0, params.gravity as f32); // Gravity compensation: +g in z direction

        let thrust_force = feedforward * params.mass as f32;

        // Compute desired rotation from thrust and yaw
        let rd = self.compute_desired_rotation(thrust_force, reference.yaw);

        // Convert current rotation quaternion to matrix
        let r = state.orientation.to_rotation_matrix();

        // Compute rotation error
        let er = Self::compute_rotation_error(&rd, &r);

        // Compute angular velocity error
        let eomega = self.compute_angular_velocity_error(
            state.angular_velocity,
            &rd,
            &r,
            Vec3::new(0.0, 0.0, reference.yaw_rate), // ωd = [0, 0, ψ̇d]
        );

        // Attitude control law: τ = -KR er - Kω eω
        // Simplified - removed Coriolis and feedforward terms for stability
        let torque_proportional = Vec3::new(
            -self.kr.x * er.x,
            -self.kr.y * er.y,
            -self.kr.z * er.z,
        );
        let torque_derivative = Vec3::new(
            -self.kw.x * eomega.x,
            -self.kw.y * eomega.y,
            -self.kw.z * eomega.z,
        );

        let torque = torque_proportional + torque_derivative;

        ControlOutput {
            thrust: thrust_force.norm(),
            torque,
        }
    }
}

/// Helper function: Matrix multiplication A^T * B (f32)
fn matmul_transpose_a_f32(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                result[i][j] += a[k][i] * b[k][j];
            }
        }
    }
    result
}

/// Helper function: Matrix-vector multiplication (f32)
fn matvecmul_f32_mat(mat: &[[f32; 3]; 3], vec: Vec3) -> Vec3 {
    Vec3::new(
        mat[0][0] * vec.x + mat[0][1] * vec.y + mat[0][2] * vec.z,
        mat[1][0] * vec.x + mat[1][1] * vec.y + mat[1][2] * vec.z,
        mat[2][0] * vec.x + mat[2][1] * vec.y + mat[2][2] * vec.z,
    )
}

/// Helper function: Matrix subtraction (f32)
fn matsub_f32(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
    result
}

/// Helper function: Matrix transpose (f32)
fn transpose_f32(mat: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    [
        [mat[0][0], mat[1][0], mat[2][0]],
        [mat[0][1], mat[1][1], mat[2][1]],
        [mat[0][2], mat[1][2], mat[2][2]],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::MultirotorParams;

    #[test]
    fn test_geometric_controller_creation() {
        let controller = GeometricController::default();
        // Check official Crazyflie Lee controller gains
        assert_eq!(controller.kp, Vec3::new(7.0, 7.0, 7.0));
        assert_eq!(controller.kv, Vec3::new(4.0, 4.0, 4.0));
        assert_eq!(controller.kr, Vec3::new(0.007, 0.007, 0.008));
        assert_eq!(controller.kw, Vec3::new(0.00115, 0.00115, 0.002));
    }

    #[test]
    fn test_hover_control() {
        let controller = GeometricController::default();
        let params = MultirotorParams::crazyflie();

        // Hover at origin
        let state = MultirotorState::new();
        let reference = TrajectoryReference {
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            acceleration: Vec3::zero(),
            yaw: 0.0,
            yaw_rate: 0.0,
            yaw_acceleration: 0.0,
        };

        let control = controller.compute_control(&state, &reference, &params);

        // Should produce thrust equal to weight
        assert!((control.thrust - params.mass * params.gravity).abs() < 1e-6);
        // Should have zero torque for hover
        assert!(control.torque.norm() < 1e-6);
    }
}