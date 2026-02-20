//! Verify coordinate system and rotation conventions

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Coordinate System Verification ===\n");

    let params = MultirotorParams::crazyflie();
    
    println!("Test 1: What does positive pitch rotation do?");
    println!("Starting with identity quaternion (level flight)");
    println!("Rotating +0.1 radians about y-axis");
    
    // Small positive rotation about y-axis
    let angle = 0.1; // radians
    let quat = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), angle);
    
    println!("  Quaternion: ({:.3}, {:.3}, {:.3}, {:.3})", quat.w, quat.x, quat.y, quat.z);
    
    let rot_matrix = quat.to_rotation_matrix();
    println!("  Rotation matrix:");
    for i in 0..3 {
        println!("    [{:.5}, {:.5}, {:.5}]", rot_matrix[i][0], rot_matrix[i][1], rot_matrix[i][2]);
    }
    
    // The body z-axis (thrust direction) in world frame is the 3rd column
    let body_z_in_world = Vec3::new(rot_matrix[0][2], rot_matrix[1][2], rot_matrix[2][2]);
    println!("  Body z-axis in world frame: ({:.5}, {:.5}, {:.5})", 
        body_z_in_world.x, body_z_in_world.y, body_z_in_world.z);
    
    if body_z_in_world.x > 0.0 {
        println!("  → Thrust points FORWARD (+x) - nose is DOWN");
    } else if body_z_in_world.x < 0.0 {
        println!("  → Thrust points BACKWARD (-x) - nose is UP");
    }
    println!();
    
    println!("Test 2: What does negative pitch rotation do?");
    let angle2 = -0.1;
    let quat2 = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), angle2);
    let rot_matrix2 = quat2.to_rotation_matrix();
    let body_z_in_world2 = Vec3::new(rot_matrix2[0][2], rot_matrix2[1][2], rot_matrix2[2][2]);
    println!("  Body z-axis in world frame: ({:.5}, {:.5}, {:.5})",
        body_z_in_world2.x, body_z_in_world2.y, body_z_in_world2.z);
    
    if body_z_in_world2.x > 0.0 {
        println!("  → Thrust points FORWARD (+x) - nose is DOWN");
    } else if body_z_in_world2.x < 0.0 {
        println!("  → Thrust points BACKWARD (-x) - nose is UP");
    }
    println!();
    
    println!("Test 3: Which motor combination creates positive pitch torque?");
    let motor_action_pos = MotorAction::from_thrust_torque(
        params.mass * params.gravity,
        Vec3::new(0.0, 0.001, 0.0),  // Positive pitch torque
        &params
    );
    println!("  Positive pitch torque:");
    println!("    Front motors (1,2): {:.1}, {:.1}", motor_action_pos.omega1_sq, motor_action_pos.omega2_sq);
    println!("    Back motors (3,4): {:.1}, {:.1}", motor_action_pos.omega3_sq, motor_action_pos.omega4_sq);
    
    if motor_action_pos.omega3_sq + motor_action_pos.omega4_sq > motor_action_pos.omega1_sq + motor_action_pos.omega2_sq {
        println!("    → Back motors FASTER");
    } else {
        println!("    → Front motors FASTER");
    }
    println!();
    
    println!("Summary:");
    println!("  If positive pitch torque → back motors faster");
    println!("  If back motors faster → positive angular acceleration about y");
    println!("  If positive rotation about y → body z-axis rotates toward +x");
    println!("  If body z-axis points toward +x → thrust points forward → nose is DOWN");
}
