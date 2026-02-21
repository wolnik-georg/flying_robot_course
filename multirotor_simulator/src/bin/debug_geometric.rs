//! Debug the geometric controller internal calculations

use multirotor_simulator::prelude::*;

fn print_matrix(name: &str, mat: &[[f32; 3]; 3]) {
    println!("{}:", name);
    for i in 0..3 {
        println!("  [{:8.5}, {:8.5}, {:8.5}]", mat[i][0], mat[i][1], mat[i][2]);
    }
}

fn main() {
    println!("=== Detailed Controller Internal Debug ===\n");

    let params = MultirotorParams::crazyflie();
    
    // State: drone at x=0.1, identity orientation
    let state = MultirotorState::with_initial(
        Vec3::new(0.1, 0.0, 0.5),
        Vec3::zero(),
        Quat::identity(),
        Vec3::zero(),
    );

    // Reference: hover at origin
    let reference = TrajectoryReference {
        position: Vec3::new(0.0, 0.0, 0.5),
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
            jerk: Vec3::zero(),
        yaw: 0.0,
        yaw_rate: 0.0,
        yaw_acceleration: 0.0,
    };

    println!("Step 1: Position control");
    let ep = reference.position - state.position;
    let ev = reference.velocity - state.velocity;
    println!("  ep = ({:.6}, {:.6}, {:.6})", ep.x, ep.y, ep.z);
    println!("  ev = ({:.6}, {:.6}, {:.6})", ev.x, ev.y, ev.z);
    
    let kp = Vec3::new(0.1, 0.1, 0.1);
    let kv = Vec3::new(0.05, 0.05, 0.05);
    
    let feedforward = reference.acceleration 
        + Vec3::new(kp.x * ep.x, kp.y * ep.y, kp.z * ep.z)
        + Vec3::new(kv.x * ev.x, kv.y * ev.y, kv.z * ev.z)
        + Vec3::new(0.0, 0.0, params.gravity);
    
    println!("  Feedforward acceleration = ({:.6}, {:.6}, {:.6})", feedforward.x, feedforward.y, feedforward.z);
    
    let thrust_force = feedforward * params.mass;
    println!("  Thrust force = ({:.6}, {:.6}, {:.6}) N", thrust_force.x, thrust_force.y, thrust_force.z);
    println!();

    println!("Step 2: Desired rotation from thrust");
    let zb_d = thrust_force.normalize();
    println!("  Desired z-axis (thrust direction): ({:.6}, {:.6}, {:.6})", zb_d.x, zb_d.y, zb_d.z);
    
    let yaw: f32 = 0.0;
    let xc = Vec3::new(yaw.cos(), yaw.sin(), 0.0);
    println!("  Desired x direction (from yaw): ({:.6}, {:.6}, {:.6})", xc.x, xc.y, xc.z);
    
    let yb_d_unnorm = zb_d.cross(&xc);
    println!("  y-axis (unnormalized): ({:.6}, {:.6}, {:.6})", yb_d_unnorm.x, yb_d_unnorm.y, yb_d_unnorm.z);
    println!("  y-axis magnitude: {:.6}", yb_d_unnorm.norm());
    
    let yb_d = if yb_d_unnorm.norm() < 0.01 {
        Vec3::new(-yaw.sin(), yaw.cos(), 0.0)
    } else {
        yb_d_unnorm.normalize()
    };
    println!("  Desired y-axis (normalized): ({:.6}, {:.6}, {:.6})", yb_d.x, yb_d.y, yb_d.z);
    
    let xb_d = yb_d.cross(&zb_d).normalize();
    println!("  Desired x-axis (reorthogonalized): ({:.6}, {:.6}, {:.6})", xb_d.x, xb_d.y, xb_d.z);
    
    let rd = [
        [xb_d.x, yb_d.x, zb_d.x],
        [xb_d.y, yb_d.y, zb_d.y],
        [xb_d.z, yb_d.z, zb_d.z],
    ];
    print_matrix("  Desired rotation matrix Rd", &rd);
    println!();

    println!("Step 3: Current rotation");
    let r = state.orientation.to_rotation_matrix();
    print_matrix("  Current rotation matrix R", &r);
    println!("  (Should be identity since quaternion is identity)");
    println!();

    println!("Step 4: Rotation error");
    // Compute Rd^T * R
    let rd_t = [
        [rd[0][0], rd[1][0], rd[2][0]],
        [rd[0][1], rd[1][1], rd[2][1]],
        [rd[0][2], rd[1][2], rd[2][2]],
    ];
    
    let rd_t_r = matmul(&rd_t, &r);
    print_matrix("  Rd^T * R", &rd_t_r);
    
    let r_t_rd = matmul(&r, &rd);
    print_matrix("  R^T * Rd (same as R*Rd since R=I)", &r_t_rd);
    
    let diff = matsub(&rd_t_r, &r_t_rd);
    print_matrix("  Rd^T*R - R^T*Rd", &diff);
    
    let er_x = (diff[2][1] - diff[1][2]) / 2.0;
    let er_y = (diff[0][2] - diff[2][0]) / 2.0;
    let er_z = (diff[1][0] - diff[0][1]) / 2.0;
    println!("  Rotation error eR = ({:.6}, {:.6}, {:.6})", er_x, er_y, er_z);
    println!();

    println!("Step 5: Attitude control torque");
    let kr = Vec3::new(0.05, 0.05, 0.05);
    let _kw = Vec3::new(0.01, 0.01, 0.01);
    
    let torque_prop = Vec3::new(-kr.x * er_x, -kr.y * er_y, -kr.z * er_z);
    println!("  Proportional torque (-KR * eR) = ({:.6}, {:.6}, {:.6}) Nm", torque_prop.x, torque_prop.y, torque_prop.z);
    
    println!("\nExpected:");
    println!("  - Need positive pitch torque to tilt nose up");
    println!("  - So torque.y should be POSITIVE");
    println!("  - Actual torque.y = {:.6}", torque_prop.y);
    
    if torque_prop.y > 0.0 {
        println!("  ✓ Correct sign!");
    } else {
        println!("  ✗ WRONG SIGN - this is the bug!");
    }
}

fn matmul(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    result
}

fn matsub(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut result = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
    result
}
