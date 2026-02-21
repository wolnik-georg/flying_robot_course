//! Debug rotation error calculation in detail

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Rotation Error Debug ===\n");

    let params = MultirotorParams::crazyflie();
    
    // Scenario: drone slightly tilted, controller should produce torque to correct it
    // Start with small pitch rotation (nose tilted down slightly)
    let small_angle = 0.01; // 0.01 radians ≈ 0.57 degrees
    let quat = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), small_angle);
    
    let state = MultirotorState::with_initial(
        Vec3::new(0.0, 0.0, 0.5),
        Vec3::zero(),
        quat,
        Vec3::zero(), // No angular velocity initially
    );

    // Want to hover - thrust straight up, level attitude
    let reference = TrajectoryReference {
        position: Vec3::new(0.0, 0.0, 0.5),
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
            jerk: Vec3::zero(),
        yaw: 0.0,
        yaw_rate: 0.0,
        yaw_acceleration: 0.0,
    };

    let mut controller = GeometricController::new(
        Vec3::new(0.1, 0.1, 0.1),
        Vec3::new(0.05, 0.05, 0.05),
        Vec3::new(0.05, 0.05, 0.05),
        Vec3::new(0.01, 0.01, 0.01),
        Vec3::new(0.03, 0.03, 0.03), // Attitude integral gains
    );

    println!("Initial condition:");
    println!("  Drone pitched by {:.4} rad ({:.2}°)", small_angle, small_angle * 57.3);
    println!("  Quaternion: ({:.6}, {:.6}, {:.6}, {:.6})", 
        state.orientation.w, state.orientation.x, state.orientation.y, state.orientation.z);
    println!("  Angular velocity: ({:.6}, {:.6}, {:.6})",
        state.angular_velocity.x, state.angular_velocity.y, state.angular_velocity.z);
    println!();

    // Compute control
    let control = controller.compute_control(&state, &reference, &params, 0.01);

    println!("Controller output:");
    println!("  Thrust: {:.6} N", control.thrust);
    println!("  Torque: ({:.6}, {:.6}, {:.6}) Nm",
        control.torque.x, control.torque.y, control.torque.z);
    println!();

    // Apply one timestep
    let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
    let (thrust_back, torque_back) = params.motor_speeds_to_forces_torques(&motor_action);

    println!("Motor action:");
    println!("  ω1²={:.1}, ω2²={:.1}, ω3²={:.1}, ω4²={:.1}",
        motor_action.omega1_sq, motor_action.omega2_sq, motor_action.omega3_sq, motor_action.omega4_sq);
    println!("  Thrust (verified): {:.6} N", thrust_back);
    println!("  Torque (verified): ({:.6}, {:.6}, {:.6}) Nm",
        torque_back.x, torque_back.y, torque_back.z);
    println!();

    // Compute resulting angular acceleration
    let angular_accel = params.angular_acceleration(state.angular_velocity, torque_back);
    println!("Angular acceleration:");
    println!("  α = ({:.6}, {:.6}, {:.6}) rad/s²",
        angular_accel.x, angular_accel.y, angular_accel.z);
    println!();

    // After 0.01s, what will angular velocity be?
    let dt = 0.01;
    let new_omega = state.angular_velocity + angular_accel * dt;
    println!("After dt={:.3}s:", dt);
    println!("  New ω = ({:.6}, {:.6}, {:.6}) rad/s",
        new_omega.x, new_omega.y, new_omega.z);
    println!();

    println!("Analysis:");
    println!("  Initial pitch: {:.4} rad (nose down)", small_angle);
    println!("  Torque.y = {:.6} Nm", control.torque.y);
    if control.torque.y > 0.0 {
        println!("  → Positive pitch torque → nose goes further DOWN");
        println!("  ✗ WRONG DIRECTION!");
    } else {
        println!("  → Negative pitch torque → nose goes UP");
        println!("  ✓ CORRECT DIRECTION!");
    }
}
