use multirotor_simulator::prelude::*;
// we intentionally do not import `compute_derivatives` because it is pub(crate);
// the simulator internal step already exercises the dynamics.

fn main() {
    println!("=== Pipeline Debug (trajectory -> control -> motors -> dynamics) ===\n");

    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::default();

    // Choose a trajectory to exercise
    let trajectory: Box<dyn Trajectory> = Box::new(CircleTrajectory::new(0.3, 0.3, 0.1));
    // let trajectory: Box<dyn Trajectory> = Box::new(Figure8Trajectory::with_params(8.0, 0.5, 0.5));

    // initial state matches start of trajectory
    let initial_ref = trajectory.get_reference(0.0);
    let mut state = MultirotorState::with_initial(
        initial_ref.position,
        initial_ref.velocity,
        Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), initial_ref.yaw),
        Vec3::zero(),
    );

    let integrator = Box::new(RK4Integrator);
    let mut simulator = MultirotorSimulator::new(params.clone(), integrator);

    // run for a handful of steps
    let dt = 0.01;
    for step in 0..20 {
        let time = step as f32 * dt;
        println!("--- step {} (t={:.3}) ---", step, time);

        // 1) trajectory generator
        let reference = trajectory.get_reference(time);
        println!("reference = {:?}\n", reference);

        // 2) controller
        let (control, dbg) = controller.compute_control_debug(&state, &reference, &params, dt);
        println!("control output = {:?}", control);
        println!("controller debug = {:?}\n", dbg);

        // 3) motor mixing
        let motor = MotorAction::from_thrust_torque(control.thrust, control.torque, &params);
        println!("motor action = {:?}\n", motor);

    // 4) dynamics / integrator (via simulator)
    // the simulator internally computes derivatives and applies the integrator
    simulator.set_state(state.clone());
        simulator.step(&motor);
        state = simulator.state().clone();
        println!("new state = {:?}\n", state);
    }
}