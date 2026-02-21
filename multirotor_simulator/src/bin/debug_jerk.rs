use multirotor_simulator::prelude::*;

fn main() {
    println!("=== Debug Jerk Computation ===\n");

    // Test circle trajectory (same params used in assignment2)
    // radius, height, angular velocity
    let trajectory = CircleTrajectory::new(0.3, 0.3, 0.1);
    
    for t in [0.0, 0.5, 1.0, 2.0] {
    // use Trajectory trait method
    let tref = trajectory.get_reference(t);
        println!("t = {:.1}s (Circle):", t);
        println!("  Position:     ({:.6}, {:.6}, {:.6})", tref.position.x, tref.position.y, tref.position.z);
        println!("  Velocity:     ({:.6}, {:.6}, {:.6})", tref.velocity.x, tref.velocity.y, tref.velocity.z);
        println!("  Acceleration: ({:.6}, {:.6}, {:.6})", tref.acceleration.x, tref.acceleration.y, tref.acceleration.z);
        println!("  Jerk:         ({:.6}, {:.6}, {:.6})", tref.jerk.x, tref.jerk.y, tref.jerk.z);
        println!();
    }
    
    // Test figure-8 trajectory
    let trajectory = Figure8Trajectory::with_params(8.0, 0.5, 0.5);
    
    for t in [0.0, 0.5, 1.0, 2.0] {
    let tref = trajectory.get_reference(t);
        println!("t = {:.1}s (Figure-8):", t);
        println!("  Position:     ({:.6}, {:.6}, {:.6})", tref.position.x, tref.position.y, tref.position.z);
        println!("  Velocity:     ({:.6}, {:.6}, {:.6})", tref.velocity.x, tref.velocity.y, tref.velocity.z);
        println!("  Acceleration: ({:.6}, {:.6}, {:.6})", tref.acceleration.x, tref.acceleration.y, tref.acceleration.z);
        println!("  Jerk:         ({:.6}, {:.6}, {:.6})", tref.jerk.x, tref.jerk.y, tref.jerk.z);
        println!();
    }
}
