//! Test motor mixing forward and inverse

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Motor Mixing Verification ===\n");

    let params = MultirotorParams::crazyflie();

    println!("Test 1: Pure positive pitch torque");
    println!("Expected: back motors faster than front");
    let test_torque = Vec3::new(0.0, 0.001, 0.0); // Positive pitch
    let motor_action = MotorAction::from_thrust_torque(params.mass * params.gravity, test_torque, &params);
    
    println!("  Input: thrust={:.6} N, torque=(0, {:.6}, 0) Nm", params.mass * params.gravity, test_torque.y);
    println!("  Motors: FL={:.1}, FR={:.1}, BR={:.1}, BL={:.1}", 
        motor_action.omega1_sq, motor_action.omega2_sq, motor_action.omega3_sq, motor_action.omega4_sq);
    println!("  Front avg: {:.1}, Back avg: {:.1}", 
        (motor_action.omega1_sq + motor_action.omega2_sq)/2.0,
        (motor_action.omega3_sq + motor_action.omega4_sq)/2.0);
    
    // Forward check
    let (thrust_back, torque_back) = params.motor_speeds_to_forces_torques(&motor_action);
    println!("  Forward model: thrust={:.6} N, torque=({:.6}, {:.6}, {:.6}) Nm", 
        thrust_back, torque_back.x, torque_back.y, torque_back.z);
    
    if motor_action.omega3_sq + motor_action.omega4_sq > motor_action.omega1_sq + motor_action.omega2_sq {
        println!("  ✓ Back motors faster (correct for positive pitch torque)");
    } else {
        println!("  ✗ Front motors faster (WRONG for positive pitch torque!)");
    }
    println!();

    println!("Test 2: Pure negative pitch torque");  
    println!("Expected: front motors faster than back");
    let test_torque2 = Vec3::new(0.0, -0.001, 0.0); // Negative pitch
    let motor_action2 = MotorAction::from_thrust_torque(params.mass * params.gravity, test_torque2, &params);
    
    println!("  Input: thrust={:.6} N, torque=(0, {:.6}, 0) Nm", params.mass * params.gravity, test_torque2.y);
    println!("  Motors: FL={:.1}, FR={:.1}, BR={:.1}, BL={:.1}",
        motor_action2.omega1_sq, motor_action2.omega2_sq, motor_action2.omega3_sq, motor_action2.omega4_sq);
    println!("  Front avg: {:.1}, Back avg: {:.1}",
        (motor_action2.omega1_sq + motor_action2.omega2_sq)/2.0,
        (motor_action2.omega3_sq + motor_action2.omega4_sq)/2.0);
    
    // Forward check
    let (thrust_back2, torque_back2) = params.motor_speeds_to_forces_torques(&motor_action2);
    println!("  Forward model: thrust={:.6} N, torque=({:.6}, {:.6}, {:.6}) Nm",
        thrust_back2, torque_back2.x, torque_back2.y, torque_back2.z);
    
    if motor_action2.omega1_sq + motor_action2.omega2_sq > motor_action2.omega3_sq + motor_action2.omega4_sq {
        println!("  ✓ Front motors faster (correct for negative pitch torque)");
    } else {
        println!("  ✗ Back motors faster (WRONG for negative pitch torque!)");
    }
    println!();

    println!("Test 3: What pitch torque do we NEED to tilt thrust backward?");
    println!("Scenario: drone at x=0.1, need to go to x=0");
    println!("  Need thrust with -x component");
    println!("  This means nose should tilt UP (pitch positive)");
    println!("  So we need POSITIVE pitch torque");
    println!();
    
    println!("Test 4: Physics check");
    println!("Question: If back motors spin faster, which way does the drone pitch?");
    println!("  Back motors create more downward force at the back");
    println!("  This creates a torque that pitches the NOSE UP (positive pitch)");
    println!("  When nose pitches up, thrust vector tilts backward (creates -x thrust component)");
    println!("  This is CORRECT for pulling drone back from +x position!");
}
