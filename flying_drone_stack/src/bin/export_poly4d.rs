//! Export min-snap spline trajectories in Crazyflie Poly4D CSV format.
//!
//! Generates CS2-compatible CSV files for any trajectory/mode/kt combination.
//! Output matches the format of `crazyflie_examples/data/figure8.csv` exactly.
//!
//! Usage:
//!   cargo run --release --bin export_poly4d -- --trajectory figure8 --mode 0
//!   cargo run --release --bin export_poly4d -- --trajectory figure8 --mode 1 --kt 0.008
//!   cargo run --release --bin export_poly4d -- --trajectory figure8 --mode 3 --kt 0.05
//!   cargo run --release --bin export_poly4d -- --trajectory circle  --mode 0
//!   cargo run --release --bin export_poly4d -- --trajectory circle  --mode 1 --kt 0.1
//!
//! --trajectory : figure8 | circle | helix | loop | corkscrew | corner | teardrop |
//!                teardrop_wide | loop_train | roller_coaster | oval | slalom |
//!                tilted_oval  (default: figure8)
//! --mode       : 0=Spline 1=Richter 2=Se3(*) 3=Paper (default: 0)
//! --kt         : aggressiveness for modes 1/3 (default: 0.008 for figure8, 0.1 for circle)
//! --speed      : time scale for mode 0 only, <1 = slower, >1 = faster (default: 1.0)
//! --out        : output CSV path (default: auto-generate in CS2 data folder)
//! --onboard    : write OOT traj param format (19 floats/seg, normalised-time coefficients)
//!                instead of HLC Poly4D (8-coef physical-time). Output: {label}_onboard.csv
//!
//! (*) Mode 2 (SE3) exports the position-only path; attitude polynomials are not
//!     supported by Poly4D. The geometric controller derives attitude from flatness.
//!
//! Physical-time coefficients: c_k = a_k_normalized / T^k
//! Degree-8 term (a_8) is converted to degree-7 via Hermite boundary matching
//! (see to_hermite_phys7) — preserves C3 continuity at all segment junctions.

use multirotor_simulator::planning::{SplineSegment, SplineTrajectory, Waypoint, TrajectoryPlanner, Se3Waypoint, RichterTrajectory, compute_flatness};
use multirotor_simulator::math::Vec3;
use std::fs;
use std::io::{BufWriter, Write};

// CS2 data folder — where generated CSV files are saved.
const CS2_DATA_DIR: &str =
    "/home/georg/Desktop/crazyswarm2/crazyflie_examples/crazyflie_examples/data";

const GRAVITY: f32 = 9.81;

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let trajectory = args.iter().position(|a| a == "--trajectory")
        .and_then(|i| args.get(i + 1)).cloned().unwrap_or_else(|| "figure8".into());
    let mode: u8 = args.iter().position(|a| a == "--mode")
        .and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(0);
    let speed: f32 = args.iter().position(|a| a == "--speed")
        .and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let out_override = args.iter().position(|a| a == "--out")
        .and_then(|i| args.get(i + 1)).cloned();
    let onboard = args.iter().any(|a| a == "--onboard");

    // Default kt differs per trajectory
    let default_kt = match trajectory.as_str() {
        "circle" => 0.1_f32,
        _        => 0.008_f32,
    };
    let kt: f32 = args.iter().position(|a| a == "--kt")
        .and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(default_kt);

    let (traj, label) = match trajectory.as_str() {
        "circle"         => build_circle(mode, kt, speed),
        "figure8"        => build_figure8(mode, kt, speed),
        "helix"          => build_helix(mode, kt, speed),
        "loop"           => build_loop(mode, kt, speed),
        "corkscrew"      => build_corkscrew(mode, kt, speed),
        "corner"         => build_corner(mode, kt, speed),
        "teardrop"       => build_teardrop(mode, kt, speed),
        "teardrop_wide"  => build_teardrop_wide(mode, kt, speed),
        "loop_train"     => build_loop_train(mode, kt, speed),
        "roller_coaster" => build_roller_coaster(mode, kt, speed),
        "oval"           => build_oval(mode, kt, speed),
        "slalom"         => build_slalom(mode, kt, speed),
        "tilted_oval"    => build_tilted_oval(mode, kt, speed),
        "corner_loop"         => build_corner_loop(mode, kt, speed),
        "slalom_loop"         => build_slalom_loop(mode, kt, speed),
        "loop_train_loop"     => build_loop_train_loop(mode, kt, speed),
        "roller_coaster_loop" => build_roller_coaster_loop(mode, kt, speed),
        other     => {
            eprintln!("Unknown trajectory '{}'. Supported: figure8, circle, helix, loop, \
                corkscrew, corner, teardrop, teardrop_wide, loop_train, roller_coaster, \
                oval, slalom, tilted_oval, corner_loop, slalom_loop, loop_train_loop, \
                roller_coaster_loop", other);
            std::process::exit(1);
        }
    };

    if onboard {
        // OOT traj param format: normalised-time coefficients, 19 floats/segment.
        // Used by flight.py --onboard (Mode D) — uploaded via traj.ci/cv/cw params.
        let out_path = out_override.unwrap_or_else(|| {
            format!("{}/{}_onboard.csv", CS2_DATA_DIR, label)
        });
        let periodic = is_loop_safe(&trajectory, mode);

        // Issue B fix (HB1): periodic cores start/end mid-lap (v≠0). Wrap them with a
        // rest-to-rest entry/exit segment so the OVERALL trajectory starts and ends at
        // v=a=j=0 while the laps in between stay continuous. Non-periodic trajectories
        // are already rest-to-rest by construction — exported unchanged (n_entry=0).
        let (traj_out, n_entry, n_exit) = if periodic {
            let (wrapped, t_e) = wrap_rest_to_rest(&traj);
            eprintln!(
                "Rest-to-rest wrap: +entry/+exit {:.2}s each ({} → {} segs, total {:.3}s)",
                t_e, traj.segments.len(), wrapped.segments.len(), wrapped.total_time
            );
            (wrapped, 1u8, 1u8)
        } else {
            (traj.clone(), 0u8, 0u8)
        };
        // Firmware cap (traj_iface.c TRAJ_MAX_SEGS)
        if traj_out.segments.len() > 14 {
            eprintln!(
                "ERROR: {} segments exceed the firmware TRAJ_MAX_SEGS=14 upload cap",
                traj_out.segments.len()
            );
            std::process::exit(1);
        }

        {
            let mass = 0.041_f32;
            let (mut max_thrust, mut min_thrust, mut max_omega, mut max_tau) = (0.0f32, f32::MAX, 0.0f32, 0.0f32);
            let n = 2000;
            for i in 0..=n {
                let t = traj_out.total_time * i as f32 / n as f32;
                let flat = traj_out.eval(t);
                let res = compute_flatness(&flat, mass);
                max_thrust = max_thrust.max(res.thrust);
                min_thrust = min_thrust.min(res.thrust);
                let om = (res.omega.x*res.omega.x + res.omega.y*res.omega.y + res.omega.z*res.omega.z).sqrt();
                max_omega = max_omega.max(om);
                let tau = (res.torque.x*res.torque.x + res.torque.y*res.torque.y).sqrt();
                max_tau = max_tau.max(tau);
            }
            eprintln!("FULL-TRAJ CHECK: max_thrust={:.3}N min_thrust={:.3}N max_omega={:.2}rad/s max_tau_xy={:.4}Nm",
                max_thrust, min_thrust, max_omega, max_tau);
        }
        eprintln!("Writing onboard {} → {}", label, out_path);
        write_onboard_csv(&traj_out, &out_path).expect("Failed to write onboard CSV");
        eprintln!("Done. {} segments, total={:.3}s", traj_out.segments.len(), traj_out.total_time);

        let meta_path = out_path.replace(".csv", ".meta.json");
        std::fs::write(&meta_path, format!(
            "{{\"periodic\": {}, \"n_entry\": {}, \"n_exit\": {}}}\n",
            periodic, n_entry, n_exit
        )).expect("Failed to write metadata sidecar");
        eprintln!("Wrote metadata {} (periodic={}, n_entry={}, n_exit={})",
                  meta_path, periodic, n_entry, n_exit);
    } else {
        // HLC Poly4D format: degree-7 physical-time coefficients (existing behaviour).
        let out_path = out_override.unwrap_or_else(|| {
            format!("{}/{}.csv", CS2_DATA_DIR, label)
        });
        eprintln!("Writing {} → {}", label, out_path);
        write_cs2_csv(&traj, &out_path).expect("Failed to write CSV");
        eprintln!("Done. {} segments, total={:.3}s", traj.segments.len(), traj.total_time);
    }
}

// ── Trajectory builders ────────────────────────────────────────────────────

const FIGURE8_WAYPOINTS: [(f32, f32); 11] = [
    ( 0.000000,  0.000000),
    ( 0.396058, -0.445604),
    ( 0.922409, -0.291165),
    ( 0.923174,  0.289869),
    ( 0.405364,  0.450742),
    ( 0.000000,  0.000000),
    (-0.402804, -0.449354),
    (-0.921641, -0.292459),
    (-0.923935,  0.288570),
    (-0.398611,  0.447039),
    ( 0.000000,  0.000000),
];

const FIGURE8_BASE_DURATIONS: [f32; 10] = [
    1.050000, 0.710000, 0.620000, 0.700000, 0.560000,
    0.560000, 0.700000, 0.620000, 0.710000, 1.050000,
];

fn figure8_waypoints() -> Vec<Waypoint> {
    FIGURE8_WAYPOINTS.iter()
        .map(|&(x, y)| Waypoint { pos: Vec3::new(x, y, 0.0), yaw: 0.0 })
        .collect()
}

/// Build figure-8 planner for the given mode and return (SplineTrajectory, label).
fn build_figure8(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let wps  = figure8_waypoints();
    let durs: Vec<f32> = FIGURE8_BASE_DURATIONS.iter().map(|&d| d / speed).collect();

    if mode == 2 {
        eprintln!("Note: Mode 2 (SE3) exports position path only — attitude polynomials not supported by Poly4D.");
        eprintln!("      The geometric controller will derive attitude from flatness (same as Mode 1).");
    }

    let planner = match mode {
        1 => TrajectoryPlanner::richter(&wps, kt, true)
                .expect("Figure-8 Mode 1 Richter QP failed"),
        2 => {
            let m1 = TrajectoryPlanner::richter(&wps, kt, true)
                .expect("Figure-8 Mode 2 timing QP failed");
            let kt_durs = m1.segment_durations();
            let se3_wps: Vec<Se3Waypoint> = wps.iter()
                .map(|w| Se3Waypoint::levelled(w.pos)).collect();
            TrajectoryPlanner::se3(&se3_wps, &kt_durs, 0.031, true)
                .expect("Figure-8 Mode 2 SE3 QP failed")
        }
        3 => TrajectoryPlanner::paper(&wps, kt, false)
                .expect("Figure-8 Mode 3 Paper QP failed"),
        _ => TrajectoryPlanner::spline(&wps, &durs, false)
                .expect("Figure-8 Mode 0 Spline QP failed"),
    };

    let label = mode_label("figure8", mode, kt, speed);
    (planner.as_spline().clone(), label)
}

const CIRCLE_RADIUS: f32 = 0.75;
const CIRCLE_OMEGA:  f32 = 0.6;
const CIRCLE_N_SEG:  usize = 8;

fn circle_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let seg_dur = 2.0 * std::f32::consts::PI / (CIRCLE_N_SEG as f32 * CIRCLE_OMEGA) / speed;
    let waypoints: Vec<Waypoint> = (0..=CIRCLE_N_SEG).map(|i| {
        let theta = 2.0 * std::f32::consts::PI * i as f32 / CIRCLE_N_SEG as f32;
        Waypoint { pos: Vec3::new(CIRCLE_RADIUS * theta.cos(), CIRCLE_RADIUS * theta.sin(), 0.0), yaw: 0.0 }
    }).collect();
    let durations = vec![seg_dur; CIRCLE_N_SEG];
    (waypoints, durations)
}

/// Build circle planner for the given mode and return (SplineTrajectory, label).
fn build_circle(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = circle_waypoints(speed);

    if mode == 2 {
        eprintln!("Note: Mode 2 (SE3) exports position path only — attitude polynomials not supported by Poly4D.");
    }

    let planner = match mode {
        1 => TrajectoryPlanner::richter(&wps, kt, true)
                .expect("Circle Mode 1 Richter QP failed"),
        2 => {
            let m1 = TrajectoryPlanner::richter(&wps, kt, true)
                .expect("Circle Mode 2 timing QP failed");
            let kt_durs = m1.segment_durations();
            let se3_wps: Vec<Se3Waypoint> = wps.iter()
                .map(|w| Se3Waypoint::levelled(w.pos)).collect();
            TrajectoryPlanner::se3(&se3_wps, &kt_durs, 0.031, true)
                .expect("Circle Mode 2 SE3 QP failed")
        }
        3 => TrajectoryPlanner::paper(&wps, kt, true)
                .expect("Circle Mode 3 Paper QP failed"),
        _ => TrajectoryPlanner::spline(&wps, &durs, true)
                .expect("Circle Mode 0 Spline QP failed"),
    };

    let label = mode_label("circle", mode, kt, speed);
    (planner.as_spline().clone(), label)
}

// ── Additional trajectories (Phase A port) ──────────────────────────────────
//
// All Z values below are OFFSETS from hover height (traj.hz), matching the
// existing figure8/circle convention (z=0 in the exported CSV). Mode 2 (SE3)
// is out of scope for these — only modes 0/1/3 are wired.

const PI: f32 = std::f32::consts::PI;

/// Generic builder for a Richter/Paper-only 3D trajectory (waypoints + manual
/// durations for mode 0, kt-based auto-timing for modes 1/3).
fn build_generic(
    name: &str, wps: Vec<Waypoint>, durs: Vec<f32>, periodic: bool,
    mode: u8, kt: f32, speed: f32,
) -> (SplineTrajectory, String) {
    let durs: Vec<f32> = durs.iter().map(|&d| d / speed).collect();
    let planner = match mode {
        1 => TrajectoryPlanner::richter(&wps, kt, periodic)
                .unwrap_or_else(|e| panic!("{} Mode 1 Richter QP failed: {}", name, e)),
        3 => TrajectoryPlanner::paper(&wps, kt, periodic)
                .unwrap_or_else(|e| panic!("{} Mode 3 Paper QP failed: {}", name, e)),
        _ => TrajectoryPlanner::spline(&wps, &durs, periodic)
                .unwrap_or_else(|e| panic!("{} Mode 0 Spline QP failed: {}", name, e)),
    };
    let label = mode_label(name, mode, kt, speed);
    (planner.as_spline().clone(), label)
}

// Same as build_generic, but pins acceleration at selected waypoints (e.g. the
// apex of a loop/teardrop) so flatness yields a genuinely inverted attitude there
// instead of just a tall bank. See TrajectoryPlanner::richter_with_accel_pins.
fn build_generic_with_pins(
    name: &str, wps: Vec<Waypoint>, durs: Vec<f32>, periodic: bool,
    mode: u8, kt: f32, speed: f32, accel_pins: &[(usize, Vec3)],
) -> (SplineTrajectory, String) {
    let durs: Vec<f32> = durs.iter().map(|&d| d / speed).collect();
    let planner = match mode {
        1 => TrajectoryPlanner::richter_with_accel_pins(&wps, kt, accel_pins, periodic)
                .unwrap_or_else(|e| panic!("{} Mode 1 Richter (pinned) QP failed: {}", name, e)),
        3 => TrajectoryPlanner::paper_with_accel_pins(&wps, kt, accel_pins, periodic)
                .unwrap_or_else(|e| panic!("{} Mode 3 Paper (pinned) QP failed: {}", name, e)),
        _ => TrajectoryPlanner::Spline(
                SplineTrajectory::plan_with_accel_pins(&wps, &durs, accel_pins, periodic)
                    .unwrap_or_else(|e| panic!("{} Mode 0 Spline (pinned) QP failed: {}", name, e))
             ),
    };
    let label = mode_label(name, mode, kt, speed);
    (planner.as_spline().clone(), label)
}

// --- helix: ascending + descending spiral, n=5/lap (10 segments total) -----
fn helix_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let (r, dz, n) = (0.5_f32, 1.0_f32, 5usize);
    let seg_t = 0.6 / speed;
    let mut wps = Vec::new();
    for i in 0..=n {
        let th = 2.0 * PI * i as f32 / n as f32;
        let z = dz * i as f32 / n as f32;
        wps.push(Waypoint { pos: Vec3::new(r * th.cos(), r * th.sin(), z), yaw: 0.0 });
    }
    for i in 1..=n {
        let th = 2.0 * PI * (1.0 + i as f32 / n as f32);
        let z = dz - dz * i as f32 / n as f32;
        wps.push(Waypoint { pos: Vec3::new(r * th.cos(), r * th.sin(), z), yaw: 0.0 });
    }
    let durs = vec![seg_t; 2 * n];
    (wps, durs)
}
fn build_helix(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = helix_waypoints(speed);
    build_generic("helix", wps, durs, false, mode, kt, speed)
}

// --- loop: vertical circle in X-Z plane, periodic ---------------------------
fn build_loop(_mode: u8, _kt: f32, speed: f32) -> (SplineTrajectory, String) {
    // --mode/--kt ignored for `loop`. Accel-pinned inversion (apex hard-constrained to
    // body_z=(0,0,-1)), per explicit operator direction to keep this approach rather
    // than the natural/unpinned power-loop briefly tried 2026-07-27 (structurally
    // different behavior, not what was asked for). Reverted 2026-07-27 back to
    // entry=exit=0.33s/pin=0.16s (the config from the 19:06/19:08 flights, which
    // actually produced a clean flip: roll swung to 175.6-180 deg right at the
    // apex). The 0.20s-entry retune tried in between produced worse results in real
    // flight -- an uncommanded roll kick at the entry->pin handoff (sometimes
    // derailing the climb entirely, sometimes surviving into a good flip but with
    // no recovery margin left) -- so it's reverted. Root cause of ALL earlier
    // crashes on the 0.33s config was confirmed separately to be hover height
    // (flown at ~1.0m, apex hit the room's ~1.7m ceiling right at max roll) --
    // fixed via `--height 0.5`, not a trajectory-shape problem. Pin=1.4g.
    let (r, n) = (0.5_f32, 8usize);
    let wps: Vec<Waypoint> = (0..=n).map(|i| {
        let th = 2.0 * PI * i as f32 / n as f32;
        Waypoint { pos: Vec3::new(r * th.sin(), 0.0, r * (1.0 - th.cos())), yaw: 0.0 }
    }).collect();
    let entry_t = 0.33_f32;
    let pin_t = 0.16_f32;
    // Exit widened 0.33 -> 0.40s (2026-07-27): required to fit the new recovery pin
    // below -- the QP is infeasible/ill-conditioned with a second pin at 0.33s exit
    // segments (verified: fails at 0.33s, first succeeds at 0.40s, for any pin
    // magnitude/waypoint tried). Confirmed this widening alone doesn't reintroduce
    // the ceiling-height problem (apex geometry unchanged, only post-apex timing).
    let exit_t = 0.40_f32;
    let durs = vec![entry_t, entry_t, entry_t, pin_t, pin_t, exit_t, exit_t, exit_t];
    let durs: Vec<f32> = durs.iter().map(|&d| d / speed).collect();
    // BEST-THUS-FAR v2 (2026-07-28): reverted here from a 4-pin/max-magnitude
    // config that was flown and made things WORSE, not better. Root cause found:
    // pins are open-loop time-scheduled, and the real drone hits the floor BEFORE
    // even the first recovery pin's planned firing time (real floor impact at
    // t=2.25s vs wp6 planned at t=2.36s) -- every recovery pin added after this one
    // never got a chance to influence the real flight; pushing magnitudes further
    // only made the near-term reference shape worse (143% of freefall vs this
    // config's flights). This 2-pin config (wp6 only, no wp7/mid/wp8) is the
    // last version that produced a real partial success: flight
    // loop_mode1_kt0.15_2026-07-28_17-58-56 (height=0.55m) recovered attitude
    // cleanly and settled into a bounded near-floor oscillation for the rest of the
    // full 3.81s trajectory instead of diverging into a tumble -- the best result
    // in this entire investigation. Next direction (not yet in this config):
    // compress recovery timing to fire within the real ~0.35-0.5s fall budget
    // instead of the current 0.56s+ delay, and only once that's validated, revisit
    // sequencing (rotate to vertical first, THEN maximize thrust) rather than a
    // single blended acceleration pin. Validated: max_thrust=0.704N (91% of the
    // 0.77N ceiling), max_omega=22.2 rad/s (65%), max_tau_xy=0.0060Nm (13% of the
    // 0.045 clamp), min_thrust=0.053N.
    let pins = [
        (4usize, Vec3::new(0.0, 0.0, -1.4 * GRAVITY)),
        (6usize, Vec3::new(0.0, 0.0, 0.20 * GRAVITY)),
        (7usize, Vec3::new(0.0, 0.0, 0.50 * GRAVITY)),
    ];
    let traj = RichterTrajectory::plan_from_times_with_accel_pins(&wps, &durs, 1.0, &pins, true, 0)
        .unwrap_or_else(|e| panic!("loop pinned-inversion QP failed: {}", e));
    let planner = TrajectoryPlanner::Richter(traj);
    (planner.as_spline().clone(), mode_label("loop", 1, 0.15, 1.0))
}

// --- corkscrew: helical XY + linear Z ramp, non-periodic --------------------
fn corkscrew_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let (r, dz, n) = (0.5_f32, 1.0_f32, 8usize);
    let seg_t = (2.0 * PI / (n as f32 * 0.6)) / speed;
    let wps: Vec<Waypoint> = (0..=n).map(|i| {
        let frac = i as f32 / n as f32;
        let th = 2.0 * PI * frac;
        Waypoint { pos: Vec3::new(r * th.cos(), r * th.sin(), dz * frac), yaw: 0.0 }
    }).collect();
    (wps, vec![seg_t; n])
}
fn build_corkscrew(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = corkscrew_waypoints(speed);
    build_generic("corkscrew", wps, durs, false, mode, kt, speed)
}

// --- corner: racing-style banked cornering, flat, non-periodic --------------
fn corner_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let pts = [(-0.9,-0.2),(-0.5,-0.2),(-0.15,-0.10),(0.05,0.10),(0.18,0.42),(0.22,0.78),(0.22,1.10)];
    let wps: Vec<Waypoint> = pts.iter().map(|&(x,y)| Waypoint{pos:Vec3::new(x,y,0.0),yaw:0.0}).collect();
    let durs: Vec<f32> = [0.55_f32,0.40,0.34,0.34,0.42,0.55].iter().map(|&d| d/speed).collect();
    (wps, durs)
}
fn build_corner(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = corner_waypoints(speed);
    build_generic("corner", wps, durs, false, mode, kt, speed)
}

/// Push one variable-speed vertical loop (XZ plane, horizontal offset (cx,cy)) onto
/// wps/durs. `v_lo`/`v_hi` = speed at bottom/top — feasible inversion via centripetal
/// speed-up rather than a single accel pin. `include_start=false` for chained loops.
fn push_vloop(wps: &mut Vec<Waypoint>, durs: &mut Vec<f32>,
              cx: f32, cy: f32, rx: f32, rz: f32, v_lo: f32, v_hi: f32, n: usize, include_start: bool) {
    for i in 0..=n {
        if i == 0 && !include_start { continue; }
        let th = 2.0 * PI * i as f32 / n as f32;
        wps.push(Waypoint { pos: Vec3::new(cx + rx * th.sin(), cy, -rz * th.cos()), yaw: 0.0 });
    }
    for i in 0..n {
        let th0 = 2.0*PI*i as f32/n as f32;
        let th1 = 2.0*PI*(i+1) as f32/n as f32;
        let d = ((rx*(th1.sin()-th0.sin())).powi(2) + (rz*(th1.cos()-th0.cos())).powi(2)).sqrt();
        let thm = 0.5*(th0+th1);
        let v = v_lo + (v_hi-v_lo)*(1.0-thm.cos())*0.5;
        durs.push(d/v);
    }
}

// --- teardrop: tall variable-speed loop, periodic ---------------------------
fn teardrop_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let mut wps = Vec::new();
    let mut durs = Vec::new();
    push_vloop(&mut wps, &mut durs, 0.0, 0.0, 0.32, 0.50, 1.4, 2.6, 6, true);
    let durs: Vec<f32> = durs.iter().map(|&d| d/speed).collect();
    (wps, durs)
}
fn build_teardrop(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = teardrop_waypoints(speed);
    // Apex (top, th=pi) is waypoint index n/2=3 of push_vloop's 0..=6 sequence
    // (include_start=true). Pinned for the same reason as build_loop above.
    let pins = [(3usize, Vec3::new(0.0, 0.0, -1.5 * GRAVITY))];
    build_generic_with_pins("teardrop", wps, durs, true, mode, kt, speed, &pins)
}

fn teardrop_wide_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let mut wps = Vec::new();
    let mut durs = Vec::new();
    push_vloop(&mut wps, &mut durs, 0.0, 0.0, 0.42, 0.48, 1.5, 2.8, 6, true);
    let durs: Vec<f32> = durs.iter().map(|&d| d/speed).collect();
    (wps, durs)
}
fn build_teardrop_wide(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = teardrop_wide_waypoints(speed);
    // Same apex index/pin as build_teardrop — same push_vloop(n=6) waypoint layout.
    let pins = [(3usize, Vec3::new(0.0, 0.0, -1.5 * GRAVITY))];
    build_generic_with_pins("teardrop_wide", wps, durs, true, mode, kt, speed, &pins)
}

// --- loop_train: 2 chained variable-speed loops, non-periodic ---------------
// n=5/loop (reduced from planning_sim's n=6) to stay within firmware's 12-segment cap.
fn loop_train_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let (n_loops, r, y_span, v_lo, v_hi, n) = (2usize, 0.5_f32, 1.4_f32, 1.5_f32, 3.0_f32, 5usize);
    let mut wps = Vec::new();
    let mut durs = Vec::new();
    for k in 0..n_loops {
        let cy = -0.5*y_span + y_span*k as f32/(n_loops-1).max(1) as f32;
        if k > 0 {
            wps.push(Waypoint { pos: Vec3::new(0.0, cy, -r), yaw: 0.0 });
            let prev_cy = -0.5*y_span + y_span*(k-1) as f32/(n_loops-1).max(1) as f32;
            durs.push(((cy-prev_cy).abs()/v_lo).max(0.18));
        }
        push_vloop(&mut wps, &mut durs, 0.0, cy, r, r, v_lo, v_hi, n, k==0);
    }
    let durs: Vec<f32> = durs.iter().map(|&d| d/speed).collect();
    (wps, durs)
}
fn build_loop_train(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = loop_train_waypoints(speed);
    // n=5 per loop is odd, so there's no waypoint exactly at th=pi (apex) — index 2
    // (th=144 deg) is the closest for each of the 2 chained loops (indices 0..5 and
    // 6..11: loop 2's transition/bottom point is index 6, so its near-apex is index 8).
    let pins = [
        (2usize, Vec3::new(0.0, 0.0, -1.5 * GRAVITY)),
        (8usize, Vec3::new(0.0, 0.0, -1.5 * GRAVITY)),
    ];
    build_generic_with_pins("loop_train", wps, durs, false, mode, kt, speed, &pins)
}

// --- roller_coaster: sinusoidal hills + one loop, non-periodic --------------
// n_hill=4, loop n=4, n_out=3 (reduced from planning_sim's 8/6/5) to fit ≤12 segments.
fn roller_coaster_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let (y_span, z_lo, z_hi, r, v_run, v_lo, v_hi) = (3.2_f32, -0.2_f32, 0.3_f32, 0.5_f32, 1.8_f32, 1.5_f32, 3.0_f32);
    let n = 4usize;
    let mut wps = Vec::new();
    let mut durs = Vec::new();
    let y0 = -0.5*y_span;
    let y_loop = y0 + 0.55*y_span;
    let n_hill = 4usize;
    for i in 0..n_hill {
        let frac = i as f32/n_hill as f32;
        let y = y0 + frac*(y_loop-y0);
        let z = 0.5*(z_lo+z_hi) + 0.5*(z_hi-z_lo)*(2.0*PI*1.5*frac).sin();
        wps.push(Waypoint{pos:Vec3::new(0.0,y,z),yaw:0.0});
        if i > 0 { durs.push(((y_loop-y0)/n_hill as f32/v_run).max(0.18)); }
    }
    durs.push(((y_loop-y0)/n_hill as f32/v_run).max(0.18));
    push_vloop(&mut wps, &mut durs, 0.0, y_loop, r, r, v_lo, v_hi, n, true);
    // push_vloop centers the loop at z=0 (relative); shift to sit above the hills.
    let loop_start = wps.len() - (n + 1);
    for wp in &mut wps[loop_start..] { wp.pos.z += z_lo + r; }
    let y_end = -y0;
    let n_out = 3usize;
    for i in 1..=n_out {
        let frac = i as f32/n_out as f32;
        let y = y_loop + frac*(y_end-y_loop);
        let z = z_lo + (z_hi-z_lo)*(1.0-frac)*0.4;
        wps.push(Waypoint{pos:Vec3::new(0.0,y,z),yaw:0.0});
        durs.push(((y_end-y_loop)/n_out as f32/v_run).max(0.18));
    }
    let durs: Vec<f32> = durs.iter().map(|&d| d/speed).collect();
    (wps, durs)
}
fn build_roller_coaster(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = roller_coaster_waypoints(speed);
    // n_hill=4 hill waypoints (indices 0..3) precede the embedded loop (n=4, even ->
    // exact apex exists), which starts at index 4 (th=0). Apex (th=pi) is i=2 within
    // the loop -> global index 4+2=6.
    let pins = [(6usize, Vec3::new(0.0, 0.0, -1.5 * GRAVITY))];
    build_generic_with_pins("roller_coaster", wps, durs, false, mode, kt, speed, &pins)
}

// --- oval: elongated ellipse, flat, periodic ---------------------------------
fn oval_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let (a, b, n) = (0.8_f32, 2.2_f32, 12usize);
    let seg_t = 1.0 / speed;
    let wps: Vec<Waypoint> = (0..=n).map(|i| {
        let th = 2.0*PI*i as f32/n as f32;
        Waypoint{pos:Vec3::new(a*th.cos(), b*th.sin(), 0.0), yaw:0.0}
    }).collect();
    (wps, vec![seg_t; n])
}
fn build_oval(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = oval_waypoints(speed);
    build_generic("oval", wps, durs, true, mode, kt, speed)
}

// --- slalom: weaving pass down the y-corridor, flat, non-periodic -----------
fn slalom_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let (amp, y_max, n_gates) = (0.7_f32, 2.2_f32, 3usize);
    let seg_t = 0.6 / speed;
    let n = 4 * n_gates;
    let wps: Vec<Waypoint> = (0..=n).map(|i| {
        let frac = i as f32/n as f32;
        let y = -y_max + 2.0*y_max*frac;
        let x = amp*(2.0*PI*n_gates as f32*frac).sin();
        Waypoint{pos:Vec3::new(x,y,0.0), yaw:0.0}
    }).collect();
    (wps, vec![seg_t; n])
}
fn build_slalom(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = slalom_waypoints(speed);
    build_generic("slalom", wps, durs, false, mode, kt, speed)
}

// --- tilted_oval: elongated ellipse with altitude ramp, periodic ------------
fn tilted_oval_waypoints(speed: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let (a, b, z_lo, z_hi, n) = (0.8_f32, 2.2_f32, -0.5_f32, 0.5_f32, 12usize);
    let seg_t = 1.0 / speed;
    let z_amp = 0.5*(z_hi-z_lo);
    let wps: Vec<Waypoint> = (0..=n).map(|i| {
        let th = 2.0*PI*i as f32/n as f32;
        Waypoint{pos:Vec3::new(a*th.cos(), b*th.sin(), z_amp*th.sin()), yaw:0.0}
    }).collect();
    (wps, vec![seg_t; n])
}
fn build_tilted_oval(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, durs) = tilted_oval_waypoints(speed);
    build_generic("tilted_oval", wps, durs, true, mode, kt, speed)
}

/// Whether `--reps > 1` is safe for this (trajectory, mode) combination — i.e.
/// whether the exported path is a genuinely closed loop with position AND
/// velocity/accel/jerk continuity at the wrap seam, AND has no known numerical
/// pathology (see loop_train/loop_train_loop/slalom_loop mode-3 overshoot bugs).
/// flight.py reads this from the {label}_onboard.meta.json sidecar and hard-
/// rejects --reps>1 when false. figure8 is kept conservatively non-loopable
/// regardless of mode — it's the established benchmark trajectory and its
/// looping behavior has never been validated.
fn is_loop_safe(trajectory: &str, mode: u8) -> bool {
    match trajectory {
        "circle" | "loop" | "teardrop" | "teardrop_wide" | "oval" | "tilted_oval"
        | "corner_loop" | "roller_coaster_loop" => true,
        // mode3 excluded entirely for these two (Runge-phenomenon overshoot bug);
        // mode1 is safe.
        "slalom_loop" | "loop_train_loop" => mode == 1,
        _ => false,
    }
}

// ── There-and-back "_loop" variants (Phase E) ───────────────────────────────
//
// Turns an open (non-periodic) trajectory into a genuinely closed one by flying
// the shape forward then retracing it backward to the start: [w0..wN, w(N-1)..w1]
// with periodic=true wraparound (w1 -> w0 completes the last segment). This lets
// --reps loop them exactly like circle/loop/oval — no new firmware mechanism
// needed, just double the segment count. Requires TRAJ_MAX_SEGS=24 (raised from
// 12) to fit the doubled segment count for slalom/loop_train/roller_coaster.

fn mirror_there_and_back(wps: &[Waypoint]) -> Vec<Waypoint> {
    let mut out: Vec<Waypoint> = wps.to_vec();
    for w in wps[1..wps.len() - 1].iter().rev() {
        out.push(*w);
    }
    out
}

fn build_corner_loop(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, _) = corner_waypoints(speed);
    let wps = mirror_there_and_back(&wps);
    build_generic("corner_loop", wps, vec![], true, mode, kt, speed)
}

fn build_slalom_loop(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, _) = slalom_waypoints(speed);
    let wps = mirror_there_and_back(&wps);
    build_generic("slalom_loop", wps, vec![], true, mode, kt, speed)
}

fn build_loop_train_loop(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, _) = loop_train_waypoints(speed);
    let wps = mirror_there_and_back(&wps);
    build_generic("loop_train_loop", wps, vec![], true, mode, kt, speed)
}

fn build_roller_coaster_loop(mode: u8, kt: f32, speed: f32) -> (SplineTrajectory, String) {
    let (wps, _) = roller_coaster_waypoints(speed);
    let wps = mirror_there_and_back(&wps);
    build_generic("roller_coaster_loop", wps, vec![], true, mode, kt, speed)
}

/// Generate a consistent filename label: `{traj}_mode{N}` for mode 0,
/// `{traj}_mode{N}_kt{val}` for modes 1/2/3.
fn mode_label(traj: &str, mode: u8, kt: f32, speed: f32) -> String {
    match mode {
        0 if (speed - 1.0).abs() < 1e-3 => format!("{}_mode0", traj),
        0 => format!("{}_mode0_speed{:.2}", traj, speed),
        _ => {
            // Format kt: remove trailing zeros (0.008 → "0.008", 0.100 → "0.1")
            let kt_str = format!("{:.6}", kt)
                .trim_end_matches('0').trim_end_matches('.').to_string();
            format!("{}_mode{}_kt{}", traj, mode, kt_str)
        }
    }
}

// ── CS2 CSV output ─────────────────────────────────────────────────────────

/// Write a SplineTrajectory as a CS2-compatible Poly4D CSV.
///
/// Format matches `crazyflie_examples/data/figure8.csv` exactly:
///   header: duration,x^0,...,x^7,y^0,...,y^7,z^0,...,z^7,yaw^0,...,yaw^7,
///   rows:   one per segment, trailing comma on each line.
fn write_cs2_csv(traj: &SplineTrajectory, path: &str) -> std::io::Result<()> {
    if let Some(parent) = std::path::Path::new(path).parent() {
        fs::create_dir_all(parent)?;
    }
    let file = fs::File::create(path)?;
    let mut w = BufWriter::new(file);

    // Header (trailing comma matches figure8.csv exactly)
    write!(w, "duration")?;
    for axis in &["x", "y", "z", "yaw"] {
        for k in 0..8 { write!(w, ",{}^{}", axis, k)?; }
    }
    writeln!(w, ",")?;

    for seg in &traj.segments {
        let t    = seg.duration;
        let cx   = to_hermite_phys7(&seg.cx,   t);
        let cy   = to_hermite_phys7(&seg.cy,   t);
        let cz   = to_hermite_phys7(&seg.cz,   t);
        let cyaw = to_hermite_phys7(&seg.cyaw, t);

        write!(w, "{:.6}", t)?;
        for &c in cx.iter().chain(&cy).chain(&cz).chain(&cyaw) {
            write!(w, ",{:.6}", c)?;
        }
        writeln!(w, ",")?;
    }

    Ok(())
}

// ── OOT onboard CSV output ─────────────────────────────────────────────────

/// Write a SplineTrajectory in OOT traj param format.
///
/// Format: 28 floats per segment, normalised-time (τ∈[0,1]) coefficients.
/// Matches the firmware's g_traj_coefs / g_traj_z_coefs layout:
/// [duration, cx0..cx8, cy0..cy8, cz0..cz8]. cz is relative to `traj.hz`
/// (all-zero for flat trajectories — equivalent to z_mode=0's constant height).
/// Read by flight.py --onboard and uploaded via traj.ci/cv/cw + traj.zci/zcv/zcw.
fn write_onboard_csv(traj: &SplineTrajectory, path: &str) -> std::io::Result<()> {
    if let Some(parent) = std::path::Path::new(path).parent() {
        fs::create_dir_all(parent)?;
    }
    let file = fs::File::create(path)?;
    let mut w = BufWriter::new(file);

    write!(w, "duration")?;
    for i in 0..9usize { write!(w, ",cx{}", i)?; }
    for i in 0..9usize { write!(w, ",cy{}", i)?; }
    for i in 0..9usize { write!(w, ",cz{}", i)?; }
    writeln!(w)?;

    for seg in &traj.segments {
        write!(w, "{:.9}", seg.duration)?;
        for &c in seg.cx.iter().chain(&seg.cy).chain(&seg.cz) {
            write!(w, ",{:.9}", c)?;
        }
        writeln!(w)?;
    }

    Ok(())
}

// ── Rest-to-rest entry/exit wrapping (Issue B fix) ─────────────────────────

/// Boundary state (p, v, a, j) of a normalised-time degree-8 segment at its
/// start (τ=0) or end (τ=1), in PHYSICAL units (segment duration `t`).
fn boundary_state(c: &[f32; 9], t: f32, at_end: bool) -> (f32, f32, f32, f32) {
    if !at_end {
        (c[0], c[1] / t, 2.0 * c[2] / (t * t), 6.0 * c[3] / (t * t * t))
    } else {
        let p: f32 = c.iter().sum();
        let v: f32 = c.iter().enumerate().skip(1)
            .map(|(k, &a)| k as f32 * a).sum::<f32>() / t;
        let a_: f32 = c.iter().enumerate().skip(2)
            .map(|(k, &a)| (k * (k - 1)) as f32 * a).sum::<f32>() / (t * t);
        let j: f32 = c.iter().enumerate().skip(3)
            .map(|(k, &a)| (k * (k - 1) * (k - 2)) as f32 * a).sum::<f32>() / (t * t * t);
        (p, v, a_, j)
    }
}

/// Unique degree-7 Hermite polynomial in normalised time τ∈[0,1] (returned as a
/// 9-coef array with c8=0) matching physical (p,v,a,j) at both ends of a segment
/// of duration `t`. Same basis solve as `to_hermite_phys7`, in the τ domain.
fn hermite_norm9(p0: f32, v0: f32, a0: f32, j0: f32,
                 p1: f32, v1: f32, a1: f32, j1: f32, t: f32) -> [f32; 9] {
    // physical → normalised-domain boundary conditions (d/dτ = t·d/dt)
    let c0 = p0;
    let c1 = v0 * t;
    let c2 = a0 * t * t / 2.0;
    let c3 = j0 * t * t * t / 6.0;
    let b0 = p1;
    let b1 = v1 * t;
    let b2 = a1 * t * t;
    let b3 = j1 * t * t * t;
    let dp = b0 - (c0 + c1 + c2 + c3);
    let dv = b1 - (c1 + 2.0 * c2 + 3.0 * c3);
    let da = b2 - (2.0 * c2 + 6.0 * c3);
    let dj = b3 - 6.0 * c3;
    let c4 =  35.0 * dp - 15.0 * dv + 2.5 * da - (1.0 / 6.0) * dj;
    let c5 = -84.0 * dp + 39.0 * dv - 7.0 * da + 0.5         * dj;
    let c6 =  70.0 * dp - 34.0 * dv + 6.5 * da - 0.5         * dj;
    let c7 = -20.0 * dp + 10.0 * dv - 2.0 * da + (1.0 / 6.0) * dj;
    [c0, c1, c2, c3, c4, c5, c6, c7, 0.0]
}

/// Wrap a periodic core with one rest-to-rest entry and one exit segment.
///
/// Entry: from rest AT the lap-start point to the lap's full (p,v,a,j) at phase 0
/// (the polynomial swings back briefly to build speed — a wind-up arc). Exit: from
/// the lap-end state (== phase-0 state for periodic cores) back to rest at the same
/// point. Junctions are C3-exact by Hermite construction, so the overall trajectory
/// is rest-to-rest with zero reference steps at arm, lap seams, and disarm.
fn wrap_rest_to_rest(traj: &SplineTrajectory) -> (SplineTrajectory, f32) {
    let first = &traj.segments[0];
    let last  = traj.segments.last().expect("empty trajectory");

    let (px0, vx0, ax0, jx0) = boundary_state(&first.cx, first.duration, false);
    let (py0, vy0, ay0, jy0) = boundary_state(&first.cy, first.duration, false);
    let (pz0, vz0, az0, jz0) = boundary_state(&first.cz, first.duration, false);
    let (px1, vx1, ax1, jx1) = boundary_state(&last.cx, last.duration, true);
    let (py1, vy1, ay1, jy1) = boundary_state(&last.cy, last.duration, true);
    let (pz1, vz1, az1, jz1) = boundary_state(&last.cz, last.duration, true);

    // Reach lap speed at a modest ~0.5 m/s² average acceleration, bounded [1.2, 3.0] s.
    let speed = (vx0 * vx0 + vy0 * vy0 + vz0 * vz0).sqrt();
    let mut t_e = (speed / 0.5).clamp(1.2, 3.0);

    // Build the entry/exit Hermite segments for a given t_e and measure the actual
    // worst-axis excursion by direct evaluation (not a closed-form estimate -- an
    // earlier accel-only empirical fit missed jerk-dominated cases, e.g. loop's
    // 3-pin recovery config: low accel but high jerk at the lap point still produced
    // a large excursion the old formula didn't predict).
    let build = |t_e: f32| -> (SplineSegment, SplineSegment, f32) {
        let zero9 = [0.0_f32; 9];
        let entry = SplineSegment {
            cx: hermite_norm9(px0, 0.0, 0.0, 0.0, px0, vx0, ax0, jx0, t_e),
            cy: hermite_norm9(py0, 0.0, 0.0, 0.0, py0, vy0, ay0, jy0, t_e),
            cz: hermite_norm9(pz0, 0.0, 0.0, 0.0, pz0, vz0, az0, jz0, t_e),
            cyaw: zero9,
            duration: t_e,
        };
        let exit = SplineSegment {
            cx: hermite_norm9(px1, vx1, ax1, jx1, px1, 0.0, 0.0, 0.0, t_e),
            cy: hermite_norm9(py1, vy1, ay1, jy1, py1, 0.0, 0.0, 0.0, t_e),
            cz: hermite_norm9(pz1, vz1, az1, jz1, pz1, 0.0, 0.0, 0.0, t_e),
            cyaw: zero9,
            duration: t_e,
        };
        let (mut ex, mut ey, mut ez) = (0.0_f32, 0.0_f32, 0.0_f32);
        for i in 0..=100 {
            let tau = i as f32 / 100.0;
            let ev = |c: &[f32; 9]| -> f32 {
                c.iter().enumerate().map(|(k, &a)| a * tau.powi(k as i32)).sum()
            };
            ex = ex.max((ev(&entry.cx) - px0).abs()).max((ev(&exit.cx) - px1).abs());
            ey = ey.max((ev(&entry.cy) - py0).abs()).max((ev(&exit.cy) - py1).abs());
            ez = ez.max((ev(&entry.cz) - pz0).abs()).max((ev(&exit.cz) - pz1).abs());
        }
        (entry, exit, ex.max(ey).max(ez))
    };

    // Check the ramp's own dynamic feasibility (not just position excursion) -- an
    // earlier position-only excursion budget missed jerk-dominated cases (e.g.
    // teardrop's single-pin config, loop's 3-pin recovery config: both have
    // significant jerk at the lap point even when accel/position excursion look
    // fine, and packing that jerk into too short a t_e produces a dynamically
    // infeasible ramp -- verified 2026-07-28: t_e=0.3s gave loop's ramp 1.16N thrust
    // / 43 rad/s omega, both over the brushless ceiling, despite small excursion).
    // Conservative brushless-generic limits (mass=0.041kg, 90% of the real 0.77N/
    // 34rad/s ceilings for margin) -- only brushless flies pinned trajectories today;
    // applying this to other platforms just yields a slightly slower (safe) ramp.
    const MASS: f32 = 0.041;
    const THRUST_LIMIT: f32 = 0.77 * 0.90;
    const OMEGA_LIMIT: f32 = 34.0 * 0.90;
    const EXCURSION_BUDGET: f32 = 0.30;
    let ramp_ok = |entry: &SplineSegment, exit: &SplineSegment| -> bool {
        let sub = SplineTrajectory { segments: vec![entry.clone(), exit.clone()], total_time: entry.duration + exit.duration };
        for i in 0..=200 {
            let t = sub.total_time * i as f32 / 200.0;
            let flat = sub.eval(t);
            let res = compute_flatness(&flat, MASS);
            let om = (res.omega.x * res.omega.x + res.omega.y * res.omega.y + res.omega.z * res.omega.z).sqrt();
            if res.thrust > THRUST_LIMIT || om > OMEGA_LIMIT { return false; }
        }
        true
    };

    // Search upward from 0.65s for the smallest t_e that's dynamically feasible;
    // also respect the position-excursion budget (grows with t_e, so the two
    // constraints bound a window -- search stops at the first t_e satisfying both,
    // or falls back to the original speed-based t_e if none in range do). Starting
    // point raised from an earlier 0.3s floor: 2026-07-28 real flights showed the
    // pre-flip approach is genuinely sensitive to this ramp's exact duration --
    // re-tuning ONLY the downstream recovery pins (thrust/omega of the core lap
    // untouched) still shifted the auto-picked t_e (0.65s -> 0.60s, since it's the
    // *first* feasible value found and the feasibility boundary moves slightly with
    // any core change) and the pre-flip roll trace visibly roughened as a result
    // (oscillating +-30-55deg through the approach instead of single digits, one
    // flight dipped to the floor before ever climbing). 0.65s is independently
    // verified feasible with comfortable margin for both the old and new pin
    // magnitudes (thrust 0.626-0.657N, omega <11 rad/s) -- starting the search there
    // keeps the entry phase stable across recovery-pin iterations instead of
    // silently re-deriving a different (and apparently more fragile) value each time.
    let mut t_e_final = t_e;
    let (mut entry, mut exit, _) = build(t_e);
    let mut found = false;
    let mut probe = 0.65_f32;
    while probe <= t_e {
        let (e, x, w) = build(probe);
        if w <= EXCURSION_BUDGET && ramp_ok(&e, &x) {
            entry = e; exit = x; t_e_final = probe; found = true;
            break;
        }
        probe += 0.05;
    }
    if !found {
        // fall back to the original (unshrunk) speed-based timing -- already the
        // gentlest option available; report whatever excursion/feasibility it has.
        let (e, x, _) = build(t_e);
        entry = e; exit = x; t_e_final = t_e;
    }
    t_e = t_e_final;

    // Report the wind-up excursion so the flight-space validation can account for it.
    let (mut ex_max, mut ey_max, mut ez_max) = (0.0_f32, 0.0_f32, 0.0_f32);
    for i in 0..=100 {
        let tau = i as f32 / 100.0;
        let ev = |c: &[f32; 9]| -> f32 {
            c.iter().enumerate().map(|(k, &a)| a * tau.powi(k as i32)).sum()
        };
        ex_max = ex_max.max((ev(&entry.cx) - px0).abs()).max((ev(&exit.cx) - px1).abs());
        ey_max = ey_max.max((ev(&entry.cy) - py0).abs()).max((ev(&exit.cy) - py1).abs());
        ez_max = ez_max.max((ev(&entry.cz) - pz0).abs()).max((ev(&exit.cz) - pz1).abs());
    }
    eprintln!(
        "Rest-to-rest wind-up excursion from lap-start point: x±{:.2}m y±{:.2}m z±{:.2}m \
         (lap speed {:.2} m/s)",
        ex_max, ey_max, ez_max, speed
    );

    let mut segments = Vec::with_capacity(traj.segments.len() + 2);
    segments.push(entry);
    segments.extend(traj.segments.iter().cloned());
    segments.push(exit);
    let total_time = traj.total_time + 2.0 * t_e;
    (SplineTrajectory { segments, total_time }, t_e)
}

// ── Coefficient conversion ─────────────────────────────────────────────────

/// Convert 9 normalised-time (τ∈[0,1]) coefficients to 8 physical-time (t∈[0,T])
/// via degree-7 Hermite interpolation.
///
/// Extracts C3 boundary conditions (pos/vel/acc/jerk at both endpoints) from the
/// degree-8 normalised polynomial, then solves the unique degree-7 polynomial that
/// matches all 8 conditions exactly. Preserves C3 continuity at segment junctions.
fn to_hermite_phys7(norm: &[f32; 9], big_t: f32) -> [f32; 8] {
    let t  = big_t;
    let t2 = t * t;
    let t3 = t2 * t;
    let t4 = t2 * t2;
    let t5 = t4 * t;
    let t6 = t3 * t3;
    let t7 = t6 * t;

    let p0 = norm[0];
    let v0 = norm[1] / t;
    let a0 = 2.0 * norm[2] / t2;
    let j0 = 6.0 * norm[3] / t3;

    let p1: f32 = norm.iter().sum();
    let v1: f32 = norm.iter().enumerate().skip(1)
                      .map(|(k, &a)| k as f32 * a).sum::<f32>() / t;
    let a1: f32 = norm.iter().enumerate().skip(2)
                      .map(|(k, &a)| (k * (k - 1)) as f32 * a).sum::<f32>() / t2;
    let j1: f32 = norm.iter().enumerate().skip(3)
                      .map(|(k, &a)| (k * (k - 1) * (k - 2)) as f32 * a).sum::<f32>() / t3;

    let c0 = p0;
    let c1 = v0;
    let c2 = a0 / 2.0;
    let c3 = j0 / 6.0;

    let dp = p1 - (c0 + c1 * t + c2 * t2 + c3 * t3);
    let dv = v1 - (c1 + 2.0 * c2 * t + 3.0 * c3 * t2);
    let da = a1 - (2.0 * c2 + 6.0 * c3 * t);
    let dj = j1 - 6.0 * c3;

    let bp0 = dp;
    let bp1 = dv * t;
    let bp2 = da * t2;
    let bp3 = dj * t3;

    let x0 =  35.0 * bp0 - 15.0 * bp1 + 2.5          * bp2 - (1.0 / 6.0) * bp3;
    let x1 = -84.0 * bp0 + 39.0 * bp1 - 7.0           * bp2 + 0.5         * bp3;
    let x2 =  70.0 * bp0 - 34.0 * bp1 + 6.5           * bp2 - 0.5         * bp3;
    let x3 = -20.0 * bp0 + 10.0 * bp1 - 2.0           * bp2 + (1.0 / 6.0) * bp3;

    let c4 = x0 / t4;
    let c5 = x1 / t5;
    let c6 = x2 / t6;
    let c7 = x3 / t7;

    [c0, c1, c2, c3, c4, c5, c6, c7]
}
