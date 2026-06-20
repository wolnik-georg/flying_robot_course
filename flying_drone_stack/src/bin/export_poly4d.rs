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
//! --trajectory : figure8 | circle  (default: figure8)
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

use multirotor_simulator::planning::{SplineTrajectory, Waypoint, TrajectoryPlanner, Se3Waypoint};
use multirotor_simulator::math::Vec3;
use std::fs;
use std::io::{BufWriter, Write};

// CS2 data folder — where generated CSV files are saved.
const CS2_DATA_DIR: &str =
    "/home/georg/Desktop/crazyswarm2/crazyflie_examples/crazyflie_examples/data";

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
        "circle"  => build_circle(mode, kt, speed),
        "figure8" => build_figure8(mode, kt, speed),
        other     => {
            eprintln!("Unknown trajectory '{}'. Supported: figure8, circle", other);
            std::process::exit(1);
        }
    };

    if onboard {
        // OOT traj param format: normalised-time coefficients, 19 floats/segment.
        // Used by flight.py --onboard (Mode D) — uploaded via traj.ci/cv/cw params.
        let out_path = out_override.unwrap_or_else(|| {
            format!("{}/{}_onboard.csv", CS2_DATA_DIR, label)
        });
        eprintln!("Writing onboard {} → {}", label, out_path);
        write_onboard_csv(&traj, &out_path).expect("Failed to write onboard CSV");
        eprintln!("Done. {} segments, total={:.3}s", traj.segments.len(), traj.total_time);
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
        1 => TrajectoryPlanner::richter(&wps, kt, false)
                .expect("Figure-8 Mode 1 Richter QP failed"),
        2 => {
            let m1 = TrajectoryPlanner::richter(&wps, kt, false)
                .expect("Figure-8 Mode 2 timing QP failed");
            let kt_durs = m1.segment_durations();
            let se3_wps: Vec<Se3Waypoint> = wps.iter()
                .map(|w| Se3Waypoint::levelled(w.pos)).collect();
            TrajectoryPlanner::se3(&se3_wps, &kt_durs, 0.031, false)
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

const CIRCLE_RADIUS: f32 = 0.25;
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
/// Format: 19 floats per segment, normalised-time (τ∈[0,1]) coefficients.
/// Matches the firmware's g_traj_coefs layout: [duration, cx0..cx8, cy0..cy8].
/// Read by flight.py --onboard and uploaded via traj.ci/cv/cw CRTP params.
fn write_onboard_csv(traj: &SplineTrajectory, path: &str) -> std::io::Result<()> {
    if let Some(parent) = std::path::Path::new(path).parent() {
        fs::create_dir_all(parent)?;
    }
    let file = fs::File::create(path)?;
    let mut w = BufWriter::new(file);

    write!(w, "duration")?;
    for i in 0..9usize { write!(w, ",cx{}", i)?; }
    for i in 0..9usize { write!(w, ",cy{}", i)?; }
    writeln!(w)?;

    for seg in &traj.segments {
        write!(w, "{:.9}", seg.duration)?;
        for &c in seg.cx.iter().chain(&seg.cy) {
            write!(w, ",{:.9}", c)?;
        }
        writeln!(w)?;
    }

    Ok(())
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
