//! Check trajectory accelerations to see if they're reasonable

use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Trajectory Acceleration Analysis ===\n");

    let trajectory = Figure8Trajectory::with_params(8.0, 0.5, 0.5);
    
    println!("Checking trajectory accelerations over first 0.5 seconds:");
    println!();

    let mut max_acc = 0.0_f32;
    let mut max_acc_time = 0.0_f32;
    
    for i in 0..50 {
        let t = i as f32 * 0.01;
        let reference = trajectory.get_reference(t);
        
        let acc_mag = reference.acceleration.norm();
        if acc_mag > max_acc {
            max_acc = acc_mag;
            max_acc_time = t;
        }
        
        if i % 5 == 0 || acc_mag > 1.0 {
            println!("t={:.3}s:", t);
            println!("  Position: ({:.4}, {:.4}, {:.4})", 
                reference.position.x, reference.position.y, reference.position.z);
            println!("  Velocity: ({:.4}, {:.4}, {:.4}) = {:.4} m/s", 
                reference.velocity.x, reference.velocity.y, reference.velocity.z, reference.velocity.norm());
            println!("  Acceleration: ({:.4}, {:.4}, {:.4}) = {:.4} m/s²", 
                reference.acceleration.x, reference.acceleration.y, reference.acceleration.z, acc_mag);
            println!();
        }
    }

    println!("Maximum acceleration: {:.4} m/s² at t={:.3}s", max_acc, max_acc_time);
    println!("Gravity: 9.81 m/s²");
    println!("Ratio to gravity: {:.2}x", max_acc / 9.81);
}
