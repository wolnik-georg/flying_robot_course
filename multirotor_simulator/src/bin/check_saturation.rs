//! Check if motor saturation is causing instability

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Motor Saturation Check ===\n");

    let params = MultirotorParams::crazyflie();
    let integrator = Box::new(RK4Integrator);
    let mut simulator = MultirotorSimulator::new(params.clone(), integrator);
    
    let controller = GeometricController::new(
        Vec3::new(0.1, 0.1, 0.1),
        Vec3::new(0.05, 0.05, 0.05),
        Vec3::new(0.05, 0.05, 0.05),
        Vec3::new(0.01, 0.01, 0.01),
    );
    
    let trajectory = Figure8Trajectory::with_params(8.0, 0.5, 0.5);
    
    let initial_ref = trajectory.get_reference(0.0);
    let initial_state = MultirotorState::with_initial(
        initial_ref.position,
        initial_ref.velocity,
        Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), initial_ref.yaw),
        Vec3::zero(),
    );
    simulator.set_state(initial_state);

    let dt = 0.01;
    for step in 0..20 {
        let t = step as f32 * dt;
        let reference = trajectory.get_reference(t);
        let state = simulator.state();

        let control = controller.compute_control(state, &reference, &params);
        let motor_action = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);

        // Check for negative motor speeds BEFORE clamping
        let kf = params.kf;
        let kt = params.kt;
        let l = params.arm_length;
        let sqrt2 = 2.0_f32.sqrt();
        let l_sqrt2 = l / sqrt2;

        let thrust_term = control.thrust / (4.0 * kf);
        let roll_term = control.torque.x / (4.0 * kf * l_sqrt2);
        let pitch_term = control.torque.y / (4.0 * kf * l_sqrt2);
        let yaw_term = control.torque.z / (4.0 * kt);

        let omega1_unclamped = thrust_term - roll_term - pitch_term + yaw_term;
        let omega2_unclamped = thrust_term + roll_term - pitch_term - yaw_term;
        let omega3_unclamped = thrust_term + roll_term + pitch_term + yaw_term;
        let omega4_unclamped = thrust_term - roll_term + pitch_term - yaw_term;

        let saturated = omega1_unclamped < 0.0 || omega2_unclamped < 0.0 || 
                       omega3_unclamped < 0.0 || omega4_unclamped < 0.0;

        if step % 2 == 0 || saturated {
            println!("t={:.3}s:", t);
            println!("  Thrust: {:.4} N, Torque: ({:.4}, {:.4}, {:.4}) Nm",
                control.thrust, control.torque.x, control.torque.y, control.torque.z);
            println!("  Motor speeds² (unclamped): FL={:.1}, FR={:.1}, BR={:.1}, BL={:.1}",
                omega1_unclamped, omega2_unclamped, omega3_unclamped, omega4_unclamped);
            println!("  Motor speeds² (clamped): FL={:.1}, FR={:.1}, BR={:.1}, BL={:.1}",
                motor_action.omega1_sq, motor_action.omega2_sq, motor_action.omega3_sq, motor_action.omega4_sq);
            
            if saturated {
                println!("  ⚠️  MOTOR SATURATION DETECTED!");
            }
            println!();
        }

        simulator.step(&motor_action);
    }
}
