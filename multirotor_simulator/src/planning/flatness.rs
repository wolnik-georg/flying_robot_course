//! Differential flatness for multirotors.
//!
//! Given flat outputs z(t) = [p(t), ψ(t)] and their derivatives up to 4th order,
//! compute the full state (R, ω) and actions (f, τ) using the closed-form
//! expressions from Faessler, Franchi, and Scaramuzza 2018.
//!
//! Flat output derivatives required:
//!   p   (position)
//!   ṗ   (velocity)
//!   p̈   (acceleration)
//!   p⃛   (jerk)
//!   p⁴  (snap)
//!   ψ   (yaw)
//!   ψ̇   (yaw rate)
//!   ψ̈   (yaw acceleration)
//!
//! Reference: Faessler, M., Franchi, A., Scaramuzza, D. (2018).
//! "Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag
//!  for Accurate Tracking of High-Speed Trajectories."
//!  IEEE RA-L 3(2):620-626. Appendix A.

use crate::math::Vec3;

const G: f32 = 9.81; // m/s²

/// Full flat output at a single time instant.
#[derive(Debug, Clone, Copy)]
pub struct FlatOutput {
    pub pos: Vec3,
    pub vel: Vec3,
    pub acc: Vec3,
    pub jerk: Vec3,
    pub snap: Vec3,
    pub yaw: f32,
    pub yaw_dot: f32,
    pub yaw_ddot: f32,
}

/// Desired full state and actions recovered from flat outputs.
#[derive(Debug, Clone, Copy)]
pub struct FlatnessResult {
    /// Desired position
    pub pos: Vec3,
    /// Desired velocity
    pub vel: Vec3,
    /// Desired rotation matrix columns [xb, yb, zb] stored as [col0, col1, col2]
    pub rot: [[f32; 3]; 3],
    /// Desired thrust (scalar, N)
    pub thrust: f32,
    /// Desired angular velocity [ωx, ωy, ωz] rad/s
    pub omega: Vec3,
    /// Desired angular acceleration [ω̇x, ω̇y, ω̇z] rad/s²
    pub omega_dot: Vec3,
    /// Desired torque τ = Jω̇ − Jω×ω  (N·m)
    pub torque: Vec3,
}

// Crazyflie 2.1 inertia tensor diagonal [kg·m²]
const J_XX: f32 = 1.657_171e-5;
const J_YY: f32 = 1.657_171e-5;
const J_ZZ: f32 = 2.9261_652e-5;

fn norm3(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn normalize3(v: [f32; 3]) -> [f32; 3] {
    let n = norm3(v);
    if n > 1e-9 { [v[0] / n, v[1] / n, v[2] / n] } else { [0.0, 0.0, 1.0] }
}

fn cross3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn vec3_arr(v: Vec3) -> [f32; 3] { [v.x, v.y, v.z] }

#[allow(dead_code)]
fn arr_vec3(a: [f32; 3]) -> Vec3 { Vec3::new(a[0], a[1], a[2]) }

#[allow(dead_code)]
fn scale3(s: f32, v: [f32; 3]) -> [f32; 3] {
    [s * v[0], s * v[1], s * v[2]]
}

#[allow(dead_code)]
fn add3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[allow(dead_code)]
fn sub3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Compute desired rotation, thrust, angular velocity, angular acceleration,
/// and torque from flat outputs using Faessler et al. 2018.
pub fn compute_flatness(flat: &FlatOutput, mass: f32) -> FlatnessResult {
    let psi = flat.yaw;
    let psi_d = flat.yaw_dot;
    let psi_dd = flat.yaw_ddot;

    let acc = vec3_arr(flat.acc);
    let jerk = vec3_arr(flat.jerk);
    let snap = vec3_arr(flat.snap);

    // ── Rotation (slide 8) ──────────────────────────────────────────────────
    // xc = [cos ψ, sin ψ, 0]ᵀ
    // yc = [−sin ψ, cos ψ, 0]ᵀ
    let xc = [psi.cos(), psi.sin(), 0.0_f32];
    let yc = [-psi.sin(), psi.cos(), 0.0_f32];

    // zb = normalize(p̈ + g·ez)
    let acc_g = [acc[0], acc[1], acc[2] + G];
    let zb = normalize3(acc_g);

    // xb = normalize(yc × zb)
    let xb_raw = cross3(yc, zb);
    let xb = normalize3(xb_raw);

    // yb = zb × xb  (already unit if xb, zb are unit and orthogonal)
    let yb = cross3(zb, xb);

    // R = [xb | yb | zb]  column-major
    let rot = [
        [xb[0], xb[1], xb[2]],
        [yb[0], yb[1], yb[2]],
        [zb[0], zb[1], zb[2]],
    ];

    // ── Thrust (slide 9) ────────────────────────────────────────────────────
    // f = m * zb · (p̈ + g·ez)
    let thrust = mass * dot3(zb, acc_g);

    // ── c scalar (norm of yc × zb) ──────────────────────────────────────────
    let c = norm3(cross3(yc, zb));
    // c3 = ‖yc × zb‖  (same as c when yc·zb != ±1)

    // ── Angular velocity (slide 10 / Faessler App.A) ────────────────────────
    // cĊ  = zb · p⃛
    let c_dot = dot3(zb, jerk);

    // d1 = xb · p⃛
    let d1 = dot3(xb, jerk);
    // d2 = −yb · p⃛
    let d2 = -dot3(yb, jerk);
    // b3 = −yc · zb   (note sign convention in Faessler)
    let b3 = -dot3(yc, zb);
    // d3 = ψ̇ * xc · xb
    let d3 = psi_d * dot3(xc, xb);

    // c3 = ‖yc × zb‖ = c  (already computed)
    let omega_x = d2 / c;
    let omega_y = d1 / c;
    let omega_z = (c * d3 - b3 * d1) / (c * c);

    let omega = Vec3::new(omega_x, omega_y, omega_z);

    // ── Angular acceleration (slide 11 / Faessler App.A) ────────────────────
    // ω̇x = (1/c)(xb·p⁴ − 2ċωy − c·ωx·ωz)
    // ω̇y = (1/c)(−yb·p⁴ − 2ċωx + c·ωy·ωz)       [Faessler sign corrected]
    // ω̇z via ψ̈ term
    //
    // Note: these are scalar equations — we project snap onto body axes first.
    let snap_dot_xb = dot3(xb, snap);
    let snap_dot_yb = dot3(yb, snap);

    let e1 = snap_dot_xb - 2.0 * c_dot * omega_y - c * omega_x * omega_z;
    let e2 = -snap_dot_yb - 2.0 * c_dot * omega_x + c * omega_y * omega_z;

    let e3 = psi_dd * dot3(xc, xb)
        + 2.0 * psi_d * omega_z * dot3(xc, yb)
        - 2.0 * psi_d * omega_y * dot3(xc, zb)
        - omega_x * omega_y * dot3(yc, yb)
        - omega_x * omega_z * dot3(yc, zb);

    let omega_dot_x = e2 / c;
    let omega_dot_y = e1 / c;
    let omega_dot_z = (c * e3 - b3 * e1) / (c * c);

    let omega_dot = Vec3::new(omega_dot_x, omega_dot_y, omega_dot_z);

    // ── Torque (slide 12) ───────────────────────────────────────────────────
    // τ = J ω̇ − J ω × ω
    // Diagonal inertia: J ω̇ = [Jxx ω̇x, Jyy ω̇y, Jzz ω̇z]
    // J ω × ω: (Jω) × ω
    let j_omega = [J_XX * omega_x, J_YY * omega_y, J_ZZ * omega_z];
    let j_omega_cross_omega = cross3(j_omega, [omega_x, omega_y, omega_z]);

    let torque = Vec3::new(
        J_XX * omega_dot_x - j_omega_cross_omega[0],
        J_YY * omega_dot_y - j_omega_cross_omega[1],
        J_ZZ * omega_dot_z - j_omega_cross_omega[2],
    );

    FlatnessResult {
        pos: flat.pos,
        vel: flat.vel,
        rot,
        thrust,
        omega,
        omega_dot,
        torque,
    }
}

/// Convert the rotation matrix (column-major [xb,yb,zb]) to a unit quaternion [w,x,y,z].
pub fn rot_to_quat(rot: &[[f32; 3]; 3]) -> [f32; 4] {
    // Extract columns: xb=rot[0], yb=rot[1], zb=rot[2]
    // Build 3x3 matrix R[row][col]
    let r = [
        [rot[0][0], rot[1][0], rot[2][0]],
        [rot[0][1], rot[1][1], rot[2][1]],
        [rot[0][2], rot[1][2], rot[2][2]],
    ];
    let trace = r[0][0] + r[1][1] + r[2][2];
    let (w, x, y, z) = if trace > 0.0 {
        let s = 0.5 / (trace + 1.0).sqrt();
        (0.25 / s, (r[2][1] - r[1][2]) * s, (r[0][2] - r[2][0]) * s, (r[1][0] - r[0][1]) * s)
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = 2.0 * (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt();
        ((r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s)
    } else if r[1][1] > r[2][2] {
        let s = 2.0 * (1.0 - r[0][0] + r[1][1] - r[2][2]).sqrt();
        ((r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s)
    } else {
        let s = 2.0 * (1.0 - r[0][0] - r[1][1] + r[2][2]).sqrt();
        ((r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s)
    };
    let n = (w * w + x * x + y * y + z * z).sqrt();
    [w / n, x / n, y / n, z / n]
}

/// Build a TrajectoryReference from a FlatnessResult for the geometric controller.
pub fn flatness_to_reference(fr: &FlatnessResult, acc: Vec3, jerk: Vec3, psi: f32, psi_dot: f32, psi_ddot: f32) -> crate::controller::TrajectoryReference {
    use crate::controller::TrajectoryReference;
    TrajectoryReference {
        position: fr.pos,
        velocity: fr.vel,
        acceleration: acc,
        jerk,
        yaw: psi,
        yaw_rate: psi_dot,
        yaw_acceleration: psi_ddot,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hover_flatness() {
        // In hover: acc = 0, everything else = 0, yaw = 0
        let flat = FlatOutput {
            pos: Vec3::new(0.0, 0.0, 1.0),
            vel: Vec3::zero(),
            acc: Vec3::zero(),
            jerk: Vec3::zero(),
            snap: Vec3::zero(),
            yaw: 0.0,
            yaw_dot: 0.0,
            yaw_ddot: 0.0,
        };
        let mass = 0.034;
        let res = compute_flatness(&flat, mass);

        // Thrust should be mass * g
        let expected_thrust = mass * G;
        assert!((res.thrust - expected_thrust).abs() < 1e-4,
            "hover thrust: got {}, expected {}", res.thrust, expected_thrust);

        // Angular velocity and torque should be zero
        assert!(res.omega.x.abs() < 1e-6);
        assert!(res.omega.y.abs() < 1e-6);
        assert!(res.omega.z.abs() < 1e-6);
        assert!(res.torque.x.abs() < 1e-6);
        assert!(res.torque.y.abs() < 1e-6);
        assert!(res.torque.z.abs() < 1e-6);

        // zb should point up
        let zb = res.rot[2];
        assert!((zb[2] - 1.0).abs() < 1e-5, "hover zb should be [0,0,1]");
    }
}
