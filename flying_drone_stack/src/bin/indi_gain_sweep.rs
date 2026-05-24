//! INDI gain sweep — two sub-sweeps on (KR_xy, ζ) and (fc_butterworth, fc_iir).
//!
//! Outer position loop is fixed to the active firmware Block O values:
//!   KP_X/Y=28, KV_X/Y=3, KP_Z=30, KV_Z=7  (src: firmware_app/src/lib.rs ★ ACTIVE ★)
//!
//! Sub-sweeps:
//!   gain   — KR_xy × ζ grid; filters fixed at 60/60 Hz (firmware default)
//!   filter — fc_butterworth × fc_iir grid; KR/ζ fixed at best gain-sweep result
//!
//! Maneuvers:  hover (0,0,0.5 static)  |  circle (r=0.3m ω=1 rad/s)  |  figure-8 (a=0.92m T=7.28s)
//!
//! Usage:
//!   cargo run --release --bin indi_gain_sweep 2>/dev/null | tee results/indi_gain_sweep.csv
//!   python scripts/plot_indi_gain_sweep.py results/indi_gain_sweep.csv

use multirotor_simulator::prelude::*;
use std::f32::consts::PI;

const DT: f32 = 0.002;
const HEIGHT: f32 = 0.5;

// ── Outer-loop gains ──────────────────────────────────────────────────────────
// Hardware Block O has KP=28, KV=3 (ζ_pos≈0.28 — underdamped by design; mocap
// delay + 20 Hz update rate add effective damping not present in sim).
// Using KV=6 here (ζ_pos≈0.57) is representative of position bandwidth while
// keeping the sim from falsely diverging due to missing delay modelling.
const FW_KP_XY: f32 = 28.0;
const FW_KV_XY: f32 = 6.0;   // hardware=3; use 6 in sim to match BW without false divergence
const FW_KP_Z:  f32 = 30.0;
const FW_KV_Z:  f32 = 7.0;
const FW_KI:    f32 = 0.05;

// ── Fixed gain/filter reference points for the secondary sweep ────────────────
// Best gain-sweep cell: KR=1200, ζ=0.7 (11 cm circle RMSE; stable for both traj.)
const FIXED_KR:   f32 = 1200.0;
const FIXED_ZETA: f32 = 0.7;

// ── Trajectory parameters ─────────────────────────────────────────────────────
const CIRCLE_R: f32     = 0.3;
const CIRCLE_W: f32     = 1.0;   // rad/s

const F8_A: f32     = 0.92;
const F8_B: f32     = 0.45;
const F8_W: f32     = 2.0 * PI / 7.28;  // ≈ 0.863 rad/s

fn circle_ref(t: f32) -> TrajectoryReference {
    let (s, c) = ((CIRCLE_W * t).sin(), (CIRCLE_W * t).cos());
    let w = CIRCLE_W;
    TrajectoryReference {
        position:     Vec3::new( CIRCLE_R * c,       CIRCLE_R * s,       HEIGHT),
        velocity:     Vec3::new(-CIRCLE_R*w * s,      CIRCLE_R*w * c,     0.0),
        acceleration: Vec3::new(-CIRCLE_R*w*w * c,   -CIRCLE_R*w*w * s,  0.0),
        jerk:         Vec3::new( CIRCLE_R*w*w*w * s, -CIRCLE_R*w*w*w * c, 0.0),
        yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
    }
}

fn figure8_ref(t: f32) -> TrajectoryReference {
    let (w, a, b) = (F8_W, F8_A, F8_B / 2.0);
    let (s1, c1) = ((w*t).sin(), (w*t).cos());
    let (s2, c2) = ((2.0*w*t).sin(), (2.0*w*t).cos());
    TrajectoryReference {
        position:     Vec3::new( a*s1,           b*s2,            HEIGHT),
        velocity:     Vec3::new( a*w*c1,          2.0*b*w*c2,      0.0),
        acceleration: Vec3::new(-a*w*w*s1,       -4.0*b*w*w*s2,   0.0),
        jerk:         Vec3::new(-a*w*w*w*c1,     -8.0*b*w*w*w*c2,  0.0),
        yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
    }
}

fn hover_ref() -> TrajectoryReference {
    TrajectoryReference {
        position: Vec3::new(0.0, 0.0, HEIGHT),
        velocity: Vec3::zero(), acceleration: Vec3::zero(), jerk: Vec3::zero(),
        yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
    }
}

// ── helpers ───────────────────────────────────────────────────────────────────

fn tau_from_motor_action(ma: &MotorAction, p: &MultirotorParams) -> Vec3 {
    let arm = p.arm_length / 2.0_f32.sqrt();
    let (w1, w2, w3, w4) = (ma.omega1_sq, ma.omega2_sq, ma.omega3_sq, ma.omega4_sq);
    Vec3::new(
        p.kf * arm * (-w1 + w2 + w3 - w4),
        p.kf * arm * ( w1 - w2 + w3 - w4),
        p.kt       * (-w1 + w2 - w3 + w4),
    )
}

fn make_ctrl(params: &MultirotorParams, kr_xy: f32, kw_xy: f32,
             fc_bw: f32, fc_iir: f32) -> IndiController {
    // Z-axis attitude gains: scale from xy by inertia ratio J_zz/J_xx
    let ratio = params.inertia[2][2] / params.inertia[0][0]; // ≈ 1.765
    let mut ctrl = IndiController::default_crazyflie();
    // Override outer loop to firmware Block O
    ctrl.kp = Vec3::new(FW_KP_XY, FW_KP_XY, FW_KP_Z);
    ctrl.kv = Vec3::new(FW_KV_XY, FW_KV_XY, FW_KV_Z);
    ctrl.ki_pos = Vec3::new(FW_KI, FW_KI, FW_KI);
    // Inner attitude loop (swept)
    ctrl.kr = Vec3::new(kr_xy, kr_xy, kr_xy / ratio);
    ctrl.kw = Vec3::new(kw_xy, kw_xy, kw_xy / ratio.sqrt());
    // Filters (swept in filter sub-sweep, fixed at 60/60 in gain sub-sweep)
    ctrl.fc_butterworth = fc_bw;
    ctrl.fc_iir = fc_iir;
    ctrl
}

/// Simulate one run.  Returns (rmse_xy, rmse_z, peak_xy) or None on divergence.
fn run_sim<F>(
    params: &MultirotorParams,
    kr_xy: f32, kw_xy: f32,
    fc_bw: f32, fc_iir: f32,
    init_pos: Vec3,
    t_total: f32, warmup: f32,
    mut ref_fn: F,
) -> Option<(f32, f32, f32)>
where F: FnMut(f32) -> TrajectoryReference,
{
    let mut sim  = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));
    sim.state_mut().position = init_pos;
    let mut ctrl = make_ctrl(params, kr_xy, kw_xy, fc_bw, fc_iir);

    let mut sum_xy = 0.0_f64;
    let mut sum_z  = 0.0_f64;
    let mut peak   = 0.0_f32;
    let mut count  = 0u32;
    let mut t      = 0.0_f32;

    while t < t_total {
        let rf    = ref_fn(t);
        let state = sim.state().clone();
        let out   = ctrl.compute_control(&state, &rf, params, DT);
        let action = MotorAction::from_thrust_torque(out.thrust, out.torque, params);
        sim.step(&action);
        ctrl.seed_tau_prev(tau_from_motor_action(sim.current_motor_action(), params));
        t += DT;

        let pos = sim.state().position;
        if !pos.x.is_finite() || !pos.y.is_finite() || !pos.z.is_finite() { return None; }
        if pos.x.abs() > 20.0 || pos.y.abs() > 20.0 || pos.z.abs() > 10.0 { return None; }

        if t > warmup {
            let ex = pos.x - rf.position.x;
            let ey = pos.y - rf.position.y;
            let ez = pos.z - rf.position.z;
            let exy = (ex*ex + ey*ey).sqrt();
            sum_xy += (ex*ex + ey*ey) as f64;
            sum_z  += (ez*ez) as f64;
            if exy > peak { peak = exy; }
            count += 1;
        }
    }
    if count == 0 { return None; }
    let n = count as f64;
    Some(((sum_xy/n).sqrt() as f32, (sum_z/n).sqrt() as f32, peak))
}

fn run_hover  (p: &MultirotorParams, kr: f32, kw: f32, fb: f32, fi: f32) -> Option<(f32,f32,f32)> {
    run_sim(p, kr, kw, fb, fi, Vec3::new(0.0, 0.0, 0.0), 7.0,  4.0, |_| hover_ref())
}
fn run_circle (p: &MultirotorParams, kr: f32, kw: f32, fb: f32, fi: f32) -> Option<(f32,f32,f32)> {
    run_sim(p, kr, kw, fb, fi, Vec3::new(CIRCLE_R, 0.0, HEIGHT), 14.0, 2.0, circle_ref)
}
fn run_figure8(p: &MultirotorParams, kr: f32, kw: f32, fb: f32, fi: f32) -> Option<(f32,f32,f32)> {
    run_sim(p, kr, kw, fb, fi, Vec3::new(0.0, 0.0, HEIGHT), 22.0, 7.5, figure8_ref)
}

fn dispatch(maneuver: &str, p: &MultirotorParams, kr: f32, kw: f32,
            fb: f32, fi: f32) -> Option<(f32,f32,f32)> {
    match maneuver {
        "hover"   => run_hover(p, kr, kw, fb, fi),
        "circle"  => run_circle(p, kr, kw, fb, fi),
        "figure8" => run_figure8(p, kr, kw, fb, fi),
        _ => unreachable!(),
    }
}

fn print_row(sweep: &str, maneuver: &str, kr: f32, kw: f32, zeta: f32,
             fb: f32, fi: f32, result: Option<(f32,f32,f32)>) {
    let (rxy, rz, peak, st) = match result {
        Some((a,b,c)) => (format!("{:.5}",a), format!("{:.5}",b), format!("{:.5}",c), "ok"),
        None          => ("NaN".into(), "NaN".into(), "NaN".into(), "diverged"),
    };
    let omega_n = kr.sqrt();
    println!("{},{},{:.0},{:.2},{:.1},{:.2},{:.1},{:.1},{},{},{},{}",
        sweep, maneuver, kr, kw, zeta,
        omega_n, fb, fi, rxy, rz, peak, st);
}

fn main() {
    let params = MultirotorParams::crazyflie();
    let maneuvers = ["hover", "circle", "figure8"];

    println!("sweep,maneuver,kr_xy,kw_xy,zeta,omega_n_rads,fc_bw_hz,fc_iir_hz,rmse_xy_m,rmse_z_m,peak_xy_m,status");

    // ── Sub-sweep 1: KR × ζ (filters fixed at firmware defaults 60/60 Hz) ──────
    eprintln!("Running gain sweep (KR × ζ)...");
    // Bandwidth cascade: att_ωₙ ≥ 5 × pos_ωₙ = 5×√28 ≈ 26.5 rad/s → KR ≥ 700.
    // KR<600 stable for hover (no lateral demand), marginal for trajectories.
    let kr_vals   = [100.0_f32, 300.0, 500.0, 603.0, 800.0, 1000.0, 1200.0];
    let zeta_vals = [0.7_f32, 0.8, 1.0, 1.2, 1.4, 1.7, 2.0];

    for &maneuver in &maneuvers {
        for &kr in &kr_vals {
            for &zeta in &zeta_vals {
                let kw = 2.0 * zeta * kr.sqrt();
                print_row("gain", maneuver, kr, kw, zeta, 60.0, 60.0,
                          dispatch(maneuver, &params, kr, kw, 60.0, 60.0));
            }
        }
    }

    // ── Sub-sweep 2: fc_butterworth × fc_iir (KR/ζ fixed at best gain-sweep) ──
    eprintln!("Running filter sweep (fc_bw × fc_iir) at KR={} ζ={}...", FIXED_KR, FIXED_ZETA);
    let fc_vals = [10.0_f32, 20.0, 40.0, 60.0, 80.0, 100.0];
    let fixed_kw = 2.0 * FIXED_ZETA * FIXED_KR.sqrt();

    for &maneuver in &maneuvers {
        for &fc_bw in &fc_vals {
            for &fc_iir in &fc_vals {
                print_row("filter", maneuver, FIXED_KR, fixed_kw, FIXED_ZETA,
                          fc_bw, fc_iir,
                          dispatch(maneuver, &params, FIXED_KR, fixed_kw, fc_bw, fc_iir));
            }
        }
    }

    eprintln!("Done. Run: python scripts/plot_indi_gain_sweep.py results/indi_gain_sweep.csv");
}
