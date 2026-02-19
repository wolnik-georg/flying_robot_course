//! Test that new modular code produces identical results to original monolithic implementation
//!
//! This runs the same experiment with all 4 integration methods and exports results
//! for comparison with the original implementation

use multirotor_simulator::prelude::*;
use std::fs::File;
use std::io::Write;

fn main() -> std::io::Result<()> {
    println!("=== Testing Modular vs Monolithic Implementation ===\n");

    // Test parameters - same as original
    let params = MultirotorParams::crazyflie();
    let num_steps = 10;  // Match original: 10 steps = 0.1 seconds
    let action = MotorAction::hover();

    println!("Configuration:");
    println!("  Aircraft: Crazyflie 2.1");
    println!("  Time steps: {}", num_steps);
    println!("  dt: {} s", params.dt);
    println!("  Duration: {} s", num_steps as f32 * params.dt);
    println!("  Motor speeds: hover mode\n");

    // Test all 4 integration methods
    let methods = vec![
        ("Euler", Box::new(EulerIntegrator) as Box<dyn Integrator>),
        ("RK4", Box::new(RK4Integrator) as Box<dyn Integrator>),
        ("Exp+Euler", Box::new(ExpEulerIntegrator) as Box<dyn Integrator>),
        ("Exp+RK4", Box::new(ExpRK4Integrator) as Box<dyn Integrator>),
    ];

    let mut all_results = Vec::new();

    for (name, integrator) in methods {
        println!("Running: {}", name);
        
        let mut sim = MultirotorSimulator::new(params.clone(), integrator);
        let mut trajectory = Vec::new();

        // Save initial state
        trajectory.push((0.0, sim.state().clone()));

        // Simulate
        for step in 1..=num_steps {
            sim.step(&action);
            let time = step as f32 * params.dt;
            trajectory.push((time, sim.state().clone()));
        }

        // Print final state
        let final_state = &trajectory.last().unwrap().1;
        println!("  Final position: ({:.6}, {:.6}, {:.6})", 
                 final_state.position.x, 
                 final_state.position.y, 
                 final_state.position.z);
        println!("  Final velocity: ({:.6}, {:.6}, {:.6})", 
                 final_state.velocity.x, 
                 final_state.velocity.y, 
                 final_state.velocity.z);
        println!("  Final orientation: w={:.6}, x={:.6}, y={:.6}, z={:.6}",
                 final_state.orientation.w,
                 final_state.orientation.x,
                 final_state.orientation.y,
                 final_state.orientation.z);
        println!();

        all_results.push((name, trajectory));
    }

    // Export to CSV for comparison
    println!("Exporting results to CSV...");
    
    for (name, trajectory) in &all_results {
        let filename = format!("trajectory_modular_{}.csv", name.replace("+", "_"));
        let mut file = File::create(&filename)?;
        
        // CSV header
        writeln!(file, "time,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz")?;
        
        // Write trajectory
        for (time, state) in trajectory {
            writeln!(file, "{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                time,
                state.position.x, state.position.y, state.position.z,
                state.velocity.x, state.velocity.y, state.velocity.z,
                state.orientation.w, state.orientation.x, 
                state.orientation.y, state.orientation.z,
                state.angular_velocity.x, state.angular_velocity.y, 
                state.angular_velocity.z)?;
        }
        
        println!("  Wrote: {}", filename);
    }

    println!("\n✓ Modular implementation test complete!");
    println!("\nTo compare with original:");
    println!("  1. Run the original: cd '../Simulation and Dynamics/assignment1' && cargo run");
    println!("  2. Compare CSV files to verify identical results");

    Ok(())
}
