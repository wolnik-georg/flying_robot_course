//! Simulate 5 steps to see oscillation

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Multi-Step Oscillation Test ===\n");

    let params = MultirotorParams::crazyflie();
    let integrator = Box::new(RK4Integrator);
    let mut simulator = MultirotorSimulator::new(params.clone(), integrator);
    
    let controller = GeometricController::new(
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(0.5, 0.5, 0.5),
        Vec3::new(0.5, 0.5, 0.5),
        Vec3::new(0.1, 0.1, 0.1),  // 10x stronger damping
    );

    // Start with small pitch
    let small_angle = 0.01;
    let quat = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), small_angle);
    let initial_state = MultirotorState::with_initial(
        Vec3::new(0.0, 0.0, 0.5),
        Vec3::zero(),
        quat,
        Vec3::zero(),
    );
    simulator.set_state(initial_state);

    let reference = TrajectoryReference {
        position: Vec3::new(0.0, 0.0, 0.5),
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
        yaw: 0.0,
        yaw_rate: 0.0,
        yaw_acceleration: 0.0,
    };

    println!("Initial: pitch={:.6} rad, ω_y={:.6} rad/s\n", small_angle, 0.0);

    for step in 0..20 {
        let state = simulator.state();
        
        // Extract pitch angle from quaternion (approximate for small angles)
        let pitch = 2.0 * state.orientation.y;
        
        println!("Step {}: pitch={:.6} rad, ω_y={:.6} rad/s",
            step, pitch, state.angular_velocity.y);

        if state.angular_velocity.y.abs() > 100.0 {
            println!("\n⚠️  Angular velocity exploded!");
            break;
        }

        let control = controller.compute_control(state, &reference, &params);
        let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
        simulator.step(&motor_action);
    }
}
