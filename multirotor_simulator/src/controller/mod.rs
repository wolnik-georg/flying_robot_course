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
    /// Desired jerk [m/s³]
    pub jerk: Vec3,
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
        &mut self,
        state: &MultirotorState,
        reference: &TrajectoryReference,
        params: &MultirotorParams,
        dt: f32,
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
    /// Attitude integral gains [Nm/(rad·s)]
    pub ki: Vec3,
    /// Accumulated attitude error for integral term
    i_error_att: Vec3,
}

impl GeometricController {
    /// Create a new geometric controller with specified gains
    pub fn new(kp: Vec3, kv: Vec3, kr: Vec3, kw: Vec3, ki: Vec3) -> Self {
        Self { 
            kp, 
            kv, 
            kr, 
            kw, 
            ki,
            i_error_att: Vec3::zero(),
        }
    }

    /// Create controller with default gains (from official Crazyflie Lee controller)
    /// Reference: controller_lee.c in bitcraze/crazyflie-firmware
    /// Lee controller uses same SE(3) geometric control as our implementation
    pub fn default() -> Self {
        Self {
            kp: Vec3::new(12.0, 12.0, 15.0),   // stronger position hold (was 7.0)
            kv: Vec3::new(8.0, 8.0, 10.0),     // better damping (was 4.0)
            kr: Vec3::new(0.010, 0.010, 0.012), // slightly higher attitude stiffness
            kw: Vec3::new(0.0015, 0.0015, 0.0020),
            ki: Vec3::new(0.05, 0.05, 0.05),   // small integral to kill steady-state error
            i_error_att: Vec3::zero(),
        }
    }

    /// Reset integral error accumulator (call when touching down or taking off)
    pub fn reset(&mut self) {
        self.i_error_att = Vec3::zero();
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
    /// The matrix (Rd^T R - R^T Rd) is skew-symmetric by construction
    fn compute_rotation_error(rd: &[[f32; 3]; 3], r: &[[f32; 3]; 3]) -> Vec3 {
        // Compute Rd^T * R
        let rd_t_r = matmul_transpose_a_f32(rd, r);

        // Compute R^T * Rd
        let r_t_rd = matmul_transpose_a_f32(r, rd);

        // Compute (Rd^T R - R^T Rd)
        let diff = matsub_f32(&rd_t_r, &r_t_rd);

        // Extract rotation error using vee operator
        // For skew-symmetric matrix [[0, -c, b], [c, 0, -a], [-b, a, 0]],
        // the vee operator gives [a, b, c] = [diff[2][1], diff[0][2], diff[1][0]]
        Vec3::new(
            diff[2][1] * 0.5,
            diff[0][2] * 0.5,
            diff[1][0] * 0.5,
        )
    }
}

impl Controller for GeometricController {
    fn compute_control(
        &mut self,
        state: &MultirotorState,
        reference: &TrajectoryReference,
        params: &MultirotorParams,
        dt: f32,
    ) -> ControlOutput {
        // Position control law: Fd = m ( p̈d - Kp ep - Kv ev + g ez )
        // Negative signs for position and velocity errors (negative feedback)
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
        let thrust = thrust_force.norm();

        // Compute desired rotation from thrust and yaw
        let rd = self.compute_desired_rotation(thrust_force, reference.yaw);

        // Convert current rotation quaternion to matrix
        let r = state.orientation.to_rotation_matrix();

        // Compute rotation error
        let er = Self::compute_rotation_error(&rd, &r);

        // Accumulate integral error (reset when thrust is low, i.e., on ground)
        if thrust > 0.01 {
            self.i_error_att = self.i_error_att + er * dt;
        } else {
            self.i_error_att = Vec3::zero();
        }

        // Compute desired angular velocity using jerk feedforward (from Crazyflie Lee controller)
        let xdes = Vec3::new(rd[0][0], rd[1][0], rd[2][0]);
        let ydes = Vec3::new(rd[0][1], rd[1][1], rd[2][1]);
        let zdes = Vec3::new(rd[0][2], rd[1][2], rd[2][2]);
        
        let mut hw = Vec3::zero();
        if thrust > 0.05 {  // higher threshold to avoid inf
            // Project jerk onto plane perpendicular to thrust direction
            let jerk_component_along_thrust = zdes.dot(&reference.jerk);
            let jerk_perpendicular = reference.jerk - zdes * jerk_component_along_thrust;
            hw = jerk_perpendicular * (params.mass as f32 / thrust);
        }
        
        // Desired angular velocity in world frame
        let z_world = Vec3::new(0.0, 0.0, 1.0);
        let desired_yaw_rate = reference.yaw_rate * zdes.dot(&z_world);
        let omega_des = Vec3::new(
            -hw.dot(&ydes),
            hw.dot(&xdes),
            desired_yaw_rate,
        );
        
        // Transform desired angular velocity to body frame: ω_r = R^T * Rd * ω_des
        let rd_omega_des = matvecmul_f32_mat(&rd, omega_des);
        let r_t = transpose_f32(&r);
        let omega_r = matvecmul_f32_mat(&r_t, rd_omega_des);

        // Compute angular velocity error
        let eomega = state.angular_velocity - omega_r;

        // Attitude control law: τ = -KR·er - Kω·eω - KI·∫er + ω × Jω
        // Full Lee controller with integral term and gyroscopic compensation
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
        let torque_integral = Vec3::new(
            -self.ki.x * self.i_error_att.x,
            -self.ki.y * self.i_error_att.y,
            -self.ki.z * self.i_error_att.z,
        );
        
        // Gyroscopic compensation: ω × Jω
        let j_omega = Vec3::new(
            params.inertia[0][0] * state.angular_velocity.x,
            params.inertia[1][1] * state.angular_velocity.y,
            params.inertia[2][2] * state.angular_velocity.z,
        );
        let gyroscopic_compensation = state.angular_velocity.cross(&j_omega);

        let torque = torque_proportional + torque_derivative + torque_integral + gyroscopic_compensation;

        ControlOutput {
            thrust,
            torque,
        }
    }
}

/// Additional debugging information returned alongside the normal control output.
/// This mirrors most of the intermediate variables used by `compute_control` so that
/// caller code (e.g., a unit test or a debug executable) can inspect what the controller
/// is doing on a step-by-step basis.
#[derive(Debug)]
pub struct DebugInfo {
    pub ep: Vec3,
    pub ev: Vec3,
    pub feedforward: Vec3,
    pub thrust: f32,
    pub er: Vec3,
    pub eomega: Vec3,
    pub hw: Vec3,
    pub omega_des: Vec3,
    pub omega_r: Vec3,
    pub torque_proportional: Vec3,
    pub torque_derivative: Vec3,
    pub torque_integral: Vec3,
    pub gyroscopic_compensation: Vec3,
    pub torque: Vec3,
}

impl GeometricController {
    /// Compute control output and return debug information about the calculation.
    ///
    /// This is intended for instrumentation and testing; it duplicates the logic of
    /// `compute_control` but exposes intermediate results. The returned `DebugInfo`
    /// allows callers to understand which component of the controller might be
    /// driving an unexpected response.
    pub fn compute_control_debug(
        &mut self,
        state: &MultirotorState,
        reference: &TrajectoryReference,
        params: &MultirotorParams,
        dt: f32,
    ) -> (ControlOutput, DebugInfo) {
        // start by copying most of compute_control
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
            + Vec3::new(0.0, 0.0, params.gravity as f32);

        let thrust_force = feedforward * params.mass as f32;
        let thrust = thrust_force.norm();

        let rd = self.compute_desired_rotation(thrust_force, reference.yaw);
        let r = state.orientation.to_rotation_matrix();

        let er = Self::compute_rotation_error(&rd, &r);

        if thrust > 0.01 {
            self.i_error_att = self.i_error_att + er * dt;
        } else {
            self.i_error_att = Vec3::zero();
        }

        let xdes = Vec3::new(rd[0][0], rd[1][0], rd[2][0]);
        let ydes = Vec3::new(rd[0][1], rd[1][1], rd[2][1]);
        let zdes = Vec3::new(rd[0][2], rd[1][2], rd[2][2]);

        let mut hw = Vec3::zero();
        if thrust > 0.05 {
            let jerk_component_along_thrust = zdes.dot(&reference.jerk);
            let jerk_perpendicular = reference.jerk - zdes * jerk_component_along_thrust;
            hw = jerk_perpendicular * (params.mass as f32 / thrust);
        }

        let z_world = Vec3::new(0.0, 0.0, 1.0);
        let desired_yaw_rate = reference.yaw_rate * zdes.dot(&z_world);
        let omega_des = Vec3::new(
            -hw.dot(&ydes),
            hw.dot(&xdes),
            desired_yaw_rate,
        );

        let rd_omega_des = matvecmul_f32_mat(&rd, omega_des);
        let r_t = transpose_f32(&r);
        let omega_r = matvecmul_f32_mat(&r_t, rd_omega_des);

        let eomega = state.angular_velocity - omega_r;

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
        let torque_integral = Vec3::new(
            -self.ki.x * self.i_error_att.x,
            -self.ki.y * self.i_error_att.y,
            -self.ki.z * self.i_error_att.z,
        );

        let j_omega = Vec3::new(
            params.inertia[0][0] * state.angular_velocity.x,
            params.inertia[1][1] * state.angular_velocity.y,
            params.inertia[2][2] * state.angular_velocity.z,
        );
        let gyroscopic_compensation = state.angular_velocity.cross(&j_omega);

        let torque = torque_proportional + torque_derivative + torque_integral + gyroscopic_compensation;

        (
            ControlOutput { thrust, torque },
            DebugInfo {
                ep,
                ev,
                feedforward,
                thrust,
                er,
                eomega,
                hw,
                omega_des,
                omega_r,
                torque_proportional,
                torque_derivative,
                torque_integral,
                gyroscopic_compensation,
                torque,
            },
        )
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
        let mut controller = GeometricController::default();
        let params = MultirotorParams::crazyflie();

        // Hover at origin
        let state = MultirotorState::new();
        let reference = TrajectoryReference {
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            acceleration: Vec3::zero(),
            jerk: Vec3::zero(),
            yaw: 0.0,
            yaw_rate: 0.0,
            yaw_acceleration: 0.0,
        };

        let control = controller.compute_control(&state, &reference, &params, 0.01);

        // Should produce thrust equal to weight
        assert!((control.thrust - params.mass * params.gravity).abs() < 1e-6);
        // Should have zero torque for hover
        assert!(control.torque.norm() < 1e-6);
    }

    #[test]
    fn test_compute_desired_rotation_simple() {
        let controller = GeometricController::default();
        // thrust aligned with world z axis, yaw = 0 -> expect identity rotation
        let thrust = Vec3::new(0.0, 0.0, 1.0);
        let rd = controller.compute_desired_rotation(thrust, 0.0);
        // rd should be nearly identity matrix
        assert!((rd[0][0] - 1.0).abs() < 1e-6);
        assert!((rd[1][1] - 1.0).abs() < 1e-6);
        assert!((rd[2][2] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_rotation_error_identity() {
        let rd = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]];
        let r = rd;
        let err = GeometricController::compute_rotation_error(&rd, &r);
        // identical rotations -> zero error
        assert_eq!(err, Vec3::zero());
    }
}