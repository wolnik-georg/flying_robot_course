//! Debug rotation calculation specifically

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Debug Desired Rotation Calculation ===\n");

    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::new(
        Vec3::new(0.1, 0.1, 0.1),
        Vec3::new(0.05, 0.05, 0.05),
        Vec3::new(0.05, 0.05, 0.05),
        Vec3::new(0.01, 0.01, 0.01),
        Vec3::new(0.03, 0.03, 0.03), // Attitude integral gains
    );

    // Reference: hover at (0, 0, 0.5)
    let reference = TrajectoryReference {
        position: Vec3::new(0.0, 0.0, 0.5),
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
            jerk: Vec3::zero(),
        yaw: 0.0,
        yaw_rate: 0.0,
        yaw_acceleration: 0.0,
    };

    // Test case: offset in +x
    let state = MultirotorState::with_initial(
        Vec3::new(0.1, 0.0, 0.5), // 10cm offset in x
        Vec3::zero(),
        Quat::identity(),
        Vec3::zero(),
    );

    println!("State:");
    println!("  Position: ({:.3}, {:.3}, {:.3})", state.position.x, state.position.y, state.position.z);
    println!("  Velocity: ({:.3}, {:.3}, {:.3})", state.velocity.x, state.velocity.y, state.velocity.z);
    println!("  Orientation: identity (no rotation)");
    println!();

    println!("Reference:");
    println!("  Position: ({:.3}, {:.3}, {:.3})", reference.position.x, reference.position.y, reference.position.z);
    println!("  Velocity: ({:.3}, {:.3}, {:.3})", reference.velocity.x, reference.velocity.y, reference.velocity.z);
    println!("  Acceleration: ({:.3}, {:.3}, {:.3})", reference.acceleration.x, reference.acceleration.y, reference.acceleration.z);
    println!();

    // Manually compute what the controller should do
    println!("Position control calculation:");
    let ep = reference.position - state.position;
    println!("  Position error ep = ref - state = ({:.3}, {:.3}, {:.3})", ep.x, ep.y, ep.z);
    
    let ev = reference.velocity - state.velocity;
    println!("  Velocity error ev = ref - state = ({:.3}, {:.3}, {:.3})", ev.x, ev.y, ev.z);
    
    let kp_ep = Vec3::new(controller.kp.x * ep.x, controller.kp.y * ep.y, controller.kp.z * ep.z);
    println!("  Kp * ep = ({:.3}, {:.3}, {:.3})", kp_ep.x, kp_ep.y, kp_ep.z);
    
    let kv_ev = Vec3::new(controller.kv.x * ev.x, controller.kv.y * ev.y, controller.kv.z * ev.z);
    println!("  Kv * ev = ({:.3}, {:.3}, {:.3})", kv_ev.x, kv_ev.y, kv_ev.z);
    
    let gravity_comp = Vec3::new(0.0, 0.0, params.gravity);
    println!("  Gravity compensation = ({:.3}, {:.3}, {:.3})", gravity_comp.x, gravity_comp.y, gravity_comp.z);
    
    let feedforward = reference.acceleration + kp_ep + kv_ev + gravity_comp;
    println!("  Total feedforward acceleration = ({:.3}, {:.3}, {:.3})", feedforward.x, feedforward.y, feedforward.z);
    
    let thrust_force = feedforward * params.mass;
    println!("  Thrust force vector F = m * a = ({:.6}, {:.6}, {:.6}) N", thrust_force.x, thrust_force.y, thrust_force.z);
    println!("  Thrust magnitude = {:.6} N", thrust_force.norm());
    println!();

    println!("Expected behavior:");
    println!("  - Position error is in -x direction (drone is too far +x)");
    println!("  - So Kp * ep gives acceleration in -x direction");
    println!("  - Thrust force should have -x component to pull drone back");
    println!("  - This requires tilting: nose down (negative pitch) or right side down");
    println!();

    println!("Desired rotation from thrust vector:");
    let thrust_norm = thrust_force.normalize();
    println!("  Thrust direction (normalized) = ({:.6}, {:.6}, {:.6})", thrust_norm.x, thrust_norm.y, thrust_norm.z);
    println!("  Expected: negative x component, large positive z component");
    println!();

    // Now compute the actual control
    let control = controller.compute_control(&state, &reference, &params, 0.01);
    println!("Controller output:");
    println!("  Thrust: {:.6} N", control.thrust);
    println!("  Torque: ({:.6}, {:.6}, {:.6}) Nm", control.torque.x, control.torque.y, control.torque.z);
    println!();

    // Convert to motors
    let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
    println!("Motor commands:");
    println!("  Motor 1 (FL): ω² = {:.2}", motor_action.omega1_sq);
    println!("  Motor 2 (FR): ω² = {:.2}", motor_action.omega2_sq);
    println!("  Motor 3 (BR): ω² = {:.2}", motor_action.omega3_sq);
    println!("  Motor 4 (BL): ω² = {:.2}", motor_action.omega4_sq);
    
    // Check balance
    let avg = (motor_action.omega1_sq + motor_action.omega2_sq + motor_action.omega3_sq + motor_action.omega4_sq) / 4.0;
    println!("  Average: ω² = {:.2}", avg);
    println!("  Front (1,2) vs Back (3,4): {:.2} vs {:.2}", 
        (motor_action.omega1_sq + motor_action.omega2_sq) / 2.0,
        (motor_action.omega3_sq + motor_action.omega4_sq) / 2.0);
    println!("  Left (1,4) vs Right (2,3): {:.2} vs {:.2}",
        (motor_action.omega1_sq + motor_action.omega4_sq) / 2.0,
        (motor_action.omega2_sq + motor_action.omega3_sq) / 2.0);
    
    if motor_action.omega3_sq + motor_action.omega4_sq > motor_action.omega1_sq + motor_action.omega2_sq {
        println!("  → Back motors faster: should pitch forward (negative pitch)");
    } else if motor_action.omega1_sq + motor_action.omega2_sq > motor_action.omega3_sq + motor_action.omega4_sq {
        println!("  → Front motors faster: should pitch backward (positive pitch)");
    }
}
