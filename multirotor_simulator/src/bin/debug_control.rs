use multirotor_simulator::prelude::*;
use std::f32::consts::PI;

fn main() {
    println!("=== Debug Control Outputs ===\n");

    let params = MultirotorParams::crazyflie();
    let mut controller = GeometricController::new(
        Vec3::new(7.0, 7.0, 7.0),
        Vec3::new(4.0, 4.0, 4.0),
        Vec3::new(0.007, 0.007, 0.008),
        Vec3::new(0.00115, 0.00115, 0.002),
        Vec3::new(0.03, 0.03, 0.03),
    );

    let traj: Box<dyn Trajectory> = Box::new(CircleTrajectory::new(0.3, 0.3, 0.1));
    let traj2: Box<dyn Trajectory> = Box::new(Figure8Trajectory::with_params(8.0, 0.5, 0.5));

    let mut state = MultirotorState::with_initial(
        Vec3::new(0.3, 0.0, 0.3),
        Vec3::zero(),
        Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), PI/2.0),
        Vec3::zero(),
    );

    let dt = 0.01;
    println!("-- circle trajectory --");
    for i in 0..20 {
        let t = i as f32 * dt;
        let r = traj.get_reference(t);



        // use the new debug helper to compute both output and internals
        let (out, dbg) = controller.compute_control_debug(&state, &r, &params, dt);
        println!("t={:.3}", t);
        println!("  ep={:?} ev={:?}", dbg.ep, dbg.ev);
        println!("  feedf={:?} thrust={:.6}", dbg.feedforward, dbg.thrust);
        println!("  er={:?} eomega={:?}", dbg.er, dbg.eomega);
        println!("  hw={:?} omega_des={:?} omega_r={:?}", dbg.hw, dbg.omega_des, dbg.omega_r);
        println!("  torques P={:?} D={:?} I={:?} G={:?} total={:?}\n",
                 dbg.torque_proportional,
                 dbg.torque_derivative,
                 dbg.torque_integral,
                 dbg.gyroscopic_compensation,
                 dbg.torque);

        // step state forward (simple Euler to update for next iteration)
    state.position = state.position + state.velocity * dt;
    state.velocity = state.velocity + (dbg.feedforward) * dt; // approximate
    state.angular_velocity = state.angular_velocity + out.torque * dt; // nonsense but just to vary
    }

    // Now run figure-8 using fresh initial state
    state = MultirotorState::with_initial(
        Vec3::new(0.0, 0.0, 0.5),
        Vec3::zero(),
        Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.0),
        Vec3::zero(),
    );
    println!("-- figure-8 trajectory --");
    for i in 0..20 {
        let t = i as f32 * dt;
        let r = traj2.get_reference(t);
        let (out, dbg) = controller.compute_control_debug(&state, &r, &params, dt);
        println!("t={:.3}", t);
        println!("  ep={:?} ev={:?}", dbg.ep, dbg.ev);
        println!("  feedf={:?} thrust={:.6}", dbg.feedforward, dbg.thrust);
        println!("  er={:?} eomega={:?}", dbg.er, dbg.eomega);
        println!("  hw={:?} omega_des={:?} omega_r={:?}\n", dbg.hw, dbg.omega_des, dbg.omega_r);
        state.position = state.position + state.velocity * dt;
        state.velocity = state.velocity + (dbg.feedforward) * dt;
        state.angular_velocity = state.angular_velocity + out.torque * dt;
    }
}
