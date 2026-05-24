//! Closed-loop physics simulation tests for IndiController.
//!
//! Each test runs IndiController inside the rigid-body MultirotorSimulator —
//! the same physics used for GeometricController — to catch issues before
//! real flights. Mirrors the structure of test_hover_control_loop.rs.
//!
//! Run with:   cargo test --test test_indi_closed_loop

use multirotor_simulator::prelude::*;

const DT: f32 = 0.002;        // 500 Hz — matches firmware rate
const TARGET_HEIGHT: f32 = 0.5;

fn hover_ref(x: f32, y: f32, z: f32) -> TrajectoryReference {
    TrajectoryReference {
        position: Vec3::new(x, y, z),
        velocity: Vec3::zero(),
        acceleration: Vec3::zero(),
        jerk: Vec3::zero(),
        yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
    }
}

// ── Hover convergence ─────────────────────────────────────────────────────────

/// INDI closed-loop hover: start at rest on floor, verify Z converges within 5 s.
/// Mirrors test_hover_convergence_5s from test_hover_control_loop.rs.
#[test]
fn indi_hover_convergence_from_floor() {
    let params = MultirotorParams::crazyflie();
    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.set_state(MultirotorState {
        position: Vec3::new(0.0, 0.0, 0.0),
        velocity: Vec3::zero(),
        orientation: Quat::identity(),
        angular_velocity: Vec3::zero(),
    });

    let mut ctrl = IndiController::default_crazyflie();
    let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT);

    let mut t = 0.0_f32;
    while t < 5.0 {
        let state = sim.state().clone();
        let out   = ctrl.compute_control(&state, &reference, &params, DT);
        let action = MotorAction::from_thrust_torque(out.thrust, out.torque, &params);
        sim.step(&action);
        t += DT;

        let pos = sim.state().position;
        assert!(!pos.z.is_nan(), "z is NaN at t={:.3}s", t);
    }

    let pos = sim.state().position;
    let ez  = (TARGET_HEIGHT - pos.z).abs();
    assert!(ez < 0.05,
        "INDI hover Z error {:.4} m after 5 s (target {:.2} m, got {:.3} m)",
        ez, TARGET_HEIGHT, pos.z);
    assert!(sim.state().velocity.norm() < 0.15,
        "INDI hover velocity still {:.4} m/s after 5 s",
        sim.state().velocity.norm());
}

/// INDI hover stays level — roll and pitch must remain small at steady state.
#[test]
fn indi_hover_stays_level() {
    let params = MultirotorParams::crazyflie();
    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    let mut ctrl = IndiController::default_crazyflie();
    let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT);

    // Run to steady hover
    let mut t = 0.0_f32;
    while t < 5.0 {
        let state = sim.state().clone();
        let out   = ctrl.compute_control(&state, &reference, &params, DT);
        sim.step(&MotorAction::from_thrust_torque(out.thrust, out.torque, &params));
        t += DT;
    }

    // Extract Euler angles from orientation
    let q = sim.state().orientation;
    let (roll, pitch, _) = crate::quat_to_euler(q);
    assert!(roll.abs()  < 3.0_f32.to_radians(),
        "INDI hover roll  {:.2}° — not level", roll.to_degrees());
    assert!(pitch.abs() < 3.0_f32.to_radians(),
        "INDI hover pitch {:.2}° — not level", pitch.to_degrees());
}

// ── Attitude recovery ─────────────────────────────────────────────────────────

/// Start with a 20° roll offset — INDI must right the drone and reach hover.
#[test]
fn indi_recovers_from_roll_disturbance() {
    let params = MultirotorParams::crazyflie();
    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));

    let roll = 20.0_f32.to_radians();
    sim.set_state(MultirotorState {
        position: Vec3::new(0.0, 0.0, TARGET_HEIGHT),
        velocity: Vec3::zero(),
        orientation: Quat::new(
            (roll / 2.0).cos(), (roll / 2.0).sin(), 0.0, 0.0,
        ),
        angular_velocity: Vec3::zero(),
    });

    let mut ctrl = IndiController::default_crazyflie();
    let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT);

    let mut t = 0.0_f32;
    while t < 3.0 {
        let state = sim.state().clone();
        let out   = ctrl.compute_control(&state, &reference, &params, DT);
        sim.step(&MotorAction::from_thrust_torque(out.thrust, out.torque, &params));
        t += DT;
        assert!(!sim.state().position.z.is_nan(), "z is NaN at t={:.3}s", t);
    }

    let q = sim.state().orientation;
    let (roll_final, _, _) = crate::quat_to_euler(q);
    assert!(roll_final.abs() < 5.0_f32.to_radians(),
        "INDI did not recover from roll: residual {:.1}°", roll_final.to_degrees());
}

/// Start with a 20° pitch offset — INDI must right the drone and reach hover.
#[test]
fn indi_recovers_from_pitch_disturbance() {
    let params = MultirotorParams::crazyflie();
    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));

    let pitch = 20.0_f32.to_radians();
    sim.set_state(MultirotorState {
        position: Vec3::new(0.0, 0.0, TARGET_HEIGHT),
        velocity: Vec3::zero(),
        orientation: Quat::new(
            (pitch / 2.0).cos(), 0.0, (pitch / 2.0).sin(), 0.0,
        ),
        angular_velocity: Vec3::zero(),
    });

    let mut ctrl = IndiController::default_crazyflie();
    let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT);

    let mut t = 0.0_f32;
    while t < 3.0 {
        let state = sim.state().clone();
        let out   = ctrl.compute_control(&state, &reference, &params, DT);
        sim.step(&MotorAction::from_thrust_torque(out.thrust, out.torque, &params));
        t += DT;
        assert!(!sim.state().position.z.is_nan(), "z is NaN at t={:.3}s", t);
    }

    let q = sim.state().orientation;
    let (_, pitch_final, _) = crate::quat_to_euler(q);
    assert!(pitch_final.abs() < 5.0_f32.to_radians(),
        "INDI did not recover from pitch: residual {:.1}°", pitch_final.to_degrees());
}

// ── Trajectory tracking ───────────────────────────────────────────────────────

/// INDI lateral step response: start at hover, move setpoint 0.3 m in X.
///
/// Mirrors the hover convergence test but exercises the XY position loop and
/// the attitude inner loop under a non-zero lateral tilt demand.
///
/// After each simulation step, `τ_prev` is seeded from the actual (motor-lag-filtered)
/// motor ω² — matching the hardware INDI loop which reads τ_current from RPM.
/// Without this feed-back, τ_prev_command can accumulate unboundedly because the
/// filter delay causes the controller to keep adding to it.
#[test]
fn indi_tracks_lateral_step() {
    let params = MultirotorParams::crazyflie();
    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));

    // Drone starts at hover; setpoint is 0.3 m away in X.
    sim.set_state(MultirotorState {
        position: Vec3::new(0.0, 0.0, TARGET_HEIGHT),
        velocity: Vec3::zero(),
        orientation: Quat::identity(),
        angular_velocity: Vec3::zero(),
    });

    let mut ctrl = IndiController::default_crazyflie();
    let reference = hover_ref(0.3, 0.0, TARGET_HEIGHT);

    let mut t = 0.0_f32;
    while t < 5.0 {
        let state = sim.state().clone();
        let out   = ctrl.compute_control(&state, &reference, &params, DT);
        let action = MotorAction::from_thrust_torque(out.thrust, out.torque, &params);
        sim.step(&action);
        t += DT;

        // Feed actual (motor-lag-filtered) torque back as τ_prev — mirrors the RPM
        // measurement in firmware that makes INDI stable against filter-delay windup.
        let ma  = sim.current_motor_action();
        let tau_actual = tau_from_motor_action(ma, &params);
        ctrl.seed_tau_prev(tau_actual);

        let pos = sim.state().position;
        assert!(!pos.x.is_nan() && !pos.y.is_nan() && !pos.z.is_nan(),
            "INDI position is NaN at t={:.3}s", t);
    }

    let pos = sim.state().position;
    let ex  = (0.3 - pos.x).abs();
    let ey  = pos.y.abs();
    assert!(ex < 0.05,
        "INDI lateral step X error {:.3} m after 5 s (target 0.30 m, got {:.3} m)",
        ex, pos.x);
    assert!(ey < 0.05,
        "INDI lateral step Y drift {:.3} m after 5 s", ey);
    assert!(sim.state().velocity.norm() < 0.15,
        "INDI lateral step velocity {:.4} m/s at 5 s — not settled", sim.state().velocity.norm());
}

// ── Comparison: INDI vs Geometric ────────────────────────────────────────────

/// Both controllers must converge to the same hover equilibrium.
/// Verifies the INDI outer loop is equivalent to the geometric outer loop.
#[test]
fn indi_and_geometric_hover_to_same_equilibrium() {
    let params = MultirotorParams::crazyflie();
    let reference = hover_ref(0.0, 0.0, TARGET_HEIGHT);

    let run = |mut ctrl: Box<dyn Controller>| -> (f32, f32) {
        let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
        let mut t = 0.0_f32;
        while t < 5.0 {
            let state = sim.state().clone();
            let out   = ctrl.compute_control(&state, &reference, &params, DT);
            sim.step(&MotorAction::from_thrust_torque(out.thrust, out.torque, &params));
            t += DT;
        }
        let pos = sim.state().position;
        (pos.z, sim.state().velocity.norm())
    };

    let (geo_z, geo_v)  = run(Box::new(GeometricController::default()));
    let (indi_z, indi_v) = run(Box::new(IndiController::default_crazyflie()));

    assert!((geo_z - indi_z).abs() < 0.05,
        "Hover Z differs: geometric={:.3} m, INDI={:.3} m", geo_z, indi_z);
    assert!(indi_v < 0.15,
        "INDI still moving after 5 s: {:.4} m/s", indi_v);
    let _ = geo_v;
}

/// Same circle setup with GeometricController — establishes whether the scenario itself is fair.
#[test]
fn geometric_circle_same_setup_reference() {
    let params = MultirotorParams::crazyflie();
    let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    let mut ctrl = GeometricController::default();
    let (r, omega_c, dt) = (0.3_f32, 1.0_f32, 0.002_f32);
    let mut max_err = 0.0_f32;
    let mut t = 0.0_f32;
    while t < 10.0 {
        let px = r*(omega_c*t).cos(); let py = r*(omega_c*t).sin();
        let vx = -r*omega_c*(omega_c*t).sin(); let vy = r*omega_c*(omega_c*t).cos();
        let ax = -r*omega_c.powi(2)*(omega_c*t).cos(); let ay = -r*omega_c.powi(2)*(omega_c*t).sin();
        let jx = r*omega_c.powi(3)*(omega_c*t).sin(); let jy = -r*omega_c.powi(3)*(omega_c*t).cos();
        let reference = TrajectoryReference { position: Vec3::new(px,py,0.5), velocity: Vec3::new(vx,vy,0.0), acceleration: Vec3::new(ax,ay,0.0), jerk: Vec3::new(jx,jy,0.0), yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0 };
        let state = sim.state().clone();
        let out = ctrl.compute_control(&state, &reference, &params, dt);
        sim.step(&MotorAction::from_thrust_torque(out.thrust, out.torque, &params));
        t += dt;
        if t > 1.0 {
            let pos = sim.state().position;
            let e = ((pos.x-px).powi(2)+(pos.y-py).powi(2)).sqrt();
            if e > max_err { max_err = e; }
        }
    }
    println!("Geometric circle max XY err: {:.4} m", max_err);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn quat_to_euler(q: Quat) -> (f32, f32, f32) {
    use multirotor_simulator::math::to_euler;
    to_euler(q)
}

/// Compute actual body torque from motor ω² (mirrors Crazyflie power distribution).
///
/// Used in simulation to provide τ_current to INDI, matching the hardware path
/// that reads torque from per-motor RPM² via `rpm_get_all()`.
fn tau_from_motor_action(ma: &MotorAction, p: &MultirotorParams) -> Vec3 {
    let kf  = p.kf;
    let kt  = p.kt;
    let arm = p.arm_length / 2.0_f32.sqrt(); // effective moment arm (X-config)
    let (w1, w2, w3, w4) = (ma.omega1_sq, ma.omega2_sq, ma.omega3_sq, ma.omega4_sq);
    Vec3::new(
        kf * arm * (-w1 + w2 + w3 - w4), // roll  (τ_x)
        kf * arm * ( w1 - w2 + w3 - w4), // pitch (τ_y) — sign matches firmware
        kt       * (-w1 + w2 - w3 + w4), // yaw   (τ_z)
    )
}
