#!/bin/bash
# Quick reference for running the multirotor simulator

cat << 'EOF'

╔════════════════════════════════════════════════════════════════════╗
║           MULTIROTOR SIMULATOR - QUICK REFERENCE                   ║
╚════════════════════════════════════════════════════════════════════╝

📍 PROJECT LOCATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  /home/georg/Desktop/flying_robot_course/multirotor_simulator/

🚀 MOST COMMON COMMANDS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  cd /home/georg/Desktop/flying_robot_course/multirotor_simulator

  cargo run --bin demo              # Quick demo (recommended!)
  cargo run --bin test_equivalence  # Full comparison test
  cargo test                        # Run all tests
  cargo build --release             # Optimized build
  cargo doc --open                  # View documentation

📦 INTEGRATION METHODS AVAILABLE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  1. EulerIntegrator       - Simple, fast, least accurate
  2. RK4Integrator         - Best accuracy/speed tradeoff
  3. ExpEulerIntegrator    - Exponential map for orientation
  4. ExpRK4Integrator      - Most accurate (recommended)

🔧 CREATE YOUR OWN EXPERIMENT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  1. Create: src/bin/my_test.rs
  
  2. Add code:
     use multirotor_simulator::prelude::*;
     
     fn main() {
         let params = MultirotorParams::crazyflie();
         let integrator = Box::new(RK4Integrator);
         let mut sim = MultirotorSimulator::new(params, integrator);
         let action = MotorAction::hover();
         
         for _ in 0..100 {
             sim.step(&action);
         }
         
         println!("Final z: {}", sim.state().position.z);
     }
  
  3. Run:
     cargo run --bin my_test

📚 LEARN MORE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  README.md          - Usage guide
  ARCHITECTURE.md    - Design documentation
  src/               - Source code with educational comments
  cargo doc --open   - Generated API documentation

EOF
