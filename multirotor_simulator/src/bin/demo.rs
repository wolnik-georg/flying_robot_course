//! Simple demonstration of the multirotor simulator

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Multirotor Simulator Demo ===\n");

    // Create Crazyflie parameters
    let params = MultirotorParams::crazyflie();
    println!("Aircraft: Crazyflie 2.1");
    println!("Mass: {} kg", params.mass);
    println!("Arm length: {} m\n", params.arm_length);

    // Test each integration method
    let methods: Vec<(&str, Box<dyn Integrator>)> = vec![
        ("Euler", Box::new(EulerIntegrator)),
        ("RK4", Box::new(RK4Integrator)),
        ("Exp+Euler", Box::new(ExpEulerIntegrator)),
        ("Exp+RK4", Box::new(ExpRK4Integrator)),
    ];

    let hover_action = MotorAction::hover();
    let num_steps = 1000;

    println!("Simulating hover for {} steps (dt = {} s, total = {} s)", 
             num_steps, params.dt, num_steps as f32 * params.dt);
    println!("{:-<60}", "");

    for (name, integrator) in methods {
        let mut sim = MultirotorSimulator::new(params.clone(), integrator);
        
        // Simulate
        let states = sim.simulate(&hover_action, num_steps);
        
        // Report final state
        let final_state = states.last().unwrap();
        println!("{:12} | z = {:8.3} m | vz = {:8.3} m/s", 
            name,
            final_state.position.z,
            final_state.velocity.z
        );
    }

    println!("\n✓ Demo complete!");
}
