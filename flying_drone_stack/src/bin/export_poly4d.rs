//! Export min-snap spline trajectories in Crazyflie Poly4D format.
//!
//! Produces Python-pasteable coefficient lists in the 33-column format:
//!   [duration, cx0..cx7, cy0..cy7, cz0..cz7, cyaw0..cyaw7]
//!
//! Physical-time coefficients: c_k = a_k_normalized / T^k
//! (our planner uses normalized τ∈[0,1]; Poly4D uses t∈[0,T])
//! Degree-8 term a_8/T^8 is dropped (Poly4D supports max degree 7).
//!
//! Usage:
//!   cargo run --release --bin export_poly4d           # exports circle
//!   cargo run --release --bin export_poly4d figure8   # exports figure-8

use multirotor_simulator::planning::{SplineTrajectory, Waypoint};
use multirotor_simulator::math::Vec3;

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_else(|| "circle".into());
    match mode.as_str() {
        "figure8"        => export_figure8(),
        "figure8_match"  => export_figure8_match(),
        "fast_figure8"   => export_figure8_fast(),
        "flip"           => export_flip(),
        "fast_flip"      => export_flip_fast(),
        _                => export_circle(),
    }
}

// ── Coefficient conversion ─────────────────────────────────────────────────

/// Convert 9 normalized-time (τ∈[0,1]) coefficients to 8 physical-time (t∈[0,T])
/// via degree-7 Hermite interpolation.
///
/// The QP planner works in normalized time and returns degree-8 (9-coefficient)
/// polynomials.  Poly4D only supports degree 7.  Simply dropping a_8 and patching
/// c_7 (the old "absorb" trick) preserved POSITION but corrupted the endpoint
/// velocity by a_8/T — causing 1+ m/s jumps at junctions with unequal durations.
///
/// This function instead:
///   1. Extracts the 8 physical boundary conditions (pos, vel, acc, jerk at
///      both t=0 and t=T) from the degree-8 normalized polynomial.
///   2. Solves for the unique degree-7 Hermite polynomial that matches all 8
///      conditions exactly (analytical 4×4 system).
///
/// Because the QP already enforced physical-time continuity at junctions, the
/// boundary conditions are consistent across adjacent segments → the exported
/// Poly4D is C3-continuous in physical time (no velocity jumps).
fn to_hermite_phys7(norm: &[f32; 9], big_t: f32) -> [f32; 8] {
    let t  = big_t;
    let t2 = t * t;
    let t3 = t2 * t;
    let t4 = t2 * t2;
    let t5 = t4 * t;
    let t6 = t3 * t3;
    let t7 = t6 * t;

    // Physical boundary conditions at τ=0 (trivially from leading coefficients)
    let p0 = norm[0];
    let v0 = norm[1] / t;
    let a0 = 2.0 * norm[2] / t2;
    let j0 = 6.0 * norm[3] / t3;

    // Physical boundary conditions at τ=1: evaluate degree-8 polynomial derivatives.
    // d^k p / dτ^k |_{τ=1} = Σ_{i≥k} [i!/(i-k)!] · a_i
    // Physical: d^k p / dt^k |_{t=T} = (d^k p / dτ^k |_{τ=1}) / T^k
    let p1: f32 = norm.iter().sum();
    let v1: f32 = norm.iter().enumerate().skip(1)
                      .map(|(k, &a)| k as f32 * a).sum::<f32>() / t;
    let a1: f32 = norm.iter().enumerate().skip(2)
                      .map(|(k, &a)| (k * (k - 1)) as f32 * a).sum::<f32>() / t2;
    let j1: f32 = norm.iter().enumerate().skip(3)
                      .map(|(k, &a)| (k * (k - 1) * (k - 2)) as f32 * a).sum::<f32>() / t3;

    // c[0..3] fixed by start conditions
    let c0 = p0;
    let c1 = v0;
    let c2 = a0 / 2.0;
    let c3 = j0 / 6.0;

    // Residuals: what c[4..7] must contribute at t=T
    let dp = p1 - (c0 + c1 * t + c2 * t2 + c3 * t3);
    let dv = v1 - (c1 + 2.0 * c2 * t + 3.0 * c3 * t2);
    let da = a1 - (2.0 * c2 + 6.0 * c3 * t);
    let dj = j1 - 6.0 * c3;

    // Solve the 4×4 Hermite system M·x = b' with M^{-1} precomputed.
    // M = [[1,1,1,1],[4,5,6,7],[12,20,30,42],[24,60,120,210]]
    // M^{-1} = [[ 35, -15,  5/2, -1/6],
    //           [-84,  39,   -7,  1/2],
    //           [ 70, -34, 13/2, -1/2],
    //           [-20,  10,   -2,  1/6]]
    // Scaled RHS: b' = [dp, dv·T, da·T², dj·T³]
    let bp0 = dp;
    let bp1 = dv * t;
    let bp2 = da * t2;
    let bp3 = dj * t3;

    let x0 =  35.0 * bp0 - 15.0 * bp1 + 2.5          * bp2 - (1.0 / 6.0) * bp3;
    let x1 = -84.0 * bp0 + 39.0 * bp1 - 7.0           * bp2 + 0.5         * bp3;
    let x2 =  70.0 * bp0 - 34.0 * bp1 + 6.5           * bp2 - 0.5         * bp3;
    let x3 = -20.0 * bp0 + 10.0 * bp1 - 2.0           * bp2 + (1.0 / 6.0) * bp3;

    // Recover physical coefficients: x_i = c_{i+4} · T^{i+4}
    let c4 = x0 / t4;
    let c5 = x1 / t5;
    let c6 = x2 / t6;
    let c7 = x3 / t7;

    [c0, c1, c2, c3, c4, c5, c6, c7]
}

/// Print one trajectory as a Python list and validate waypoint errors.
fn print_poly4d(name: &str, traj: &SplineTrajectory, waypoints: &[Waypoint]) {
    // Validate: check that trajectory passes through every waypoint
    let mut max_err = 0.0f32;
    let durations: Vec<f32> = traj.segments.iter().map(|s| s.duration).collect();
    for (k, wp) in waypoints.iter().enumerate() {
        let t_k: f32 = if k < durations.len() {
            durations[..k].iter().sum()
        } else {
            traj.total_time
        };
        let out = traj.eval(t_k);
        let err = ((out.pos.x - wp.pos.x).powi(2)
                 + (out.pos.y - wp.pos.y).powi(2)
                 + (out.pos.z - wp.pos.z).powi(2)).sqrt();
        max_err = max_err.max(err);
        eprintln!("  wp[{:2}] target=({:+.3},{:+.3}) got=({:+.3},{:+.3}) err={:.4}m",
            k, wp.pos.x, wp.pos.y, out.pos.x, out.pos.y, err);
    }
    eprintln!("  Max waypoint error: {:.4} m", max_err);
    eprintln!();

    // Print Python variable
    println!("{name} = [");
    for seg in &traj.segments {
        let t = seg.duration;
        let cx   = to_hermite_phys7(&seg.cx,   t);
        let cy   = to_hermite_phys7(&seg.cy,   t);
        let cz   = to_hermite_phys7(&seg.cz,   t);
        let cyaw = to_hermite_phys7(&seg.cyaw, t);

        print!("    [{:.6}", t);
        for &c in &cx   { print!(", {:12.6}", c); }
        for &c in &cy   { print!(", {:12.6}", c); }
        for &c in &cz   { print!(", {:12.6}", c); }
        for &c in &cyaw { print!(", {:12.6}", c); }
        println!("],");
    }
    println!("]");
}

// ── Circle ─────────────────────────────────────────────────────────────────

fn export_circle() {
    let radius   = 0.30_f32;   // m  — matches CIRCLE_RADIUS in Python script
    let n_seg    = 16_usize;   // segments (every 22.5°) — smoother than 8
    let omega    = 0.6_f32;    // planned rad/s  (actual = omega × SPEED_SCALE)
    let seg_dur  = std::f32::consts::TAU / (n_seg as f32 * omega);  // ≈ 0.654 s

    // Circle centered at (radius, 0, 0) relative to the drone's hover position.
    // Starts and ends at (0, 0, 0) so relative_position=True works seamlessly.
    // Counterclockwise when viewed from above.
    // Yaw = tangent direction: atan2(dy/dφ, dx/dφ) = atan2(cos φ, sin φ) = π/2 − φ
    // Unwrapped (not wrapped to ±π) so the polynomial is monotone — firmware
    // trig (sinf/cosf) handles values outside ±π correctly.
    let waypoints: Vec<Waypoint> = (0..=n_seg).map(|i| {
        let phi = std::f32::consts::TAU * i as f32 / n_seg as f32;
        let tangent_yaw = std::f32::consts::FRAC_PI_2 - phi;  // unwrapped, decreases by 2π
        Waypoint {
            pos: Vec3::new(radius * (1.0 - phi.cos()), radius * phi.sin(), 0.0),
            yaw: tangent_yaw,
        }
    }).collect();
    let durations = vec![seg_dur; n_seg];

    eprintln!("=== Circle: r={:.2}m, {} segments, {:.3}s each, total={:.2}s ===",
        radius, n_seg, seg_dur, seg_dur * n_seg as f32);
    eprintln!("Tangent yaw: drone faces direction of travel (yaw = π/2 − φ, unwrapped)");
    eprintln!("At SPEED_SCALE=0.5: {:.1}s actual flight", seg_dur * n_seg as f32 / 0.5);
    eprintln!("Solving QP...");

    // periodic=true: derivatives wrap smoothly at the start/end junction.
    // This gives a proper circle shape (no rest-to-rest bow-out in y).
    // With periodic=false the optimizer bows the path ~17% outward in y
    // to satisfy the zero-velocity boundary conditions — it no longer looks
    // circular.  The drone starts the trajectory with a small tangential
    // velocity (~r·ω·SPEED_SCALE ≈ 0.09 m/s) which the controller absorbs.
    let traj = SplineTrajectory::plan(&waypoints, &durations, true)
        .expect("Circle planning failed");

    eprintln!("Done. Waypoint validation:");
    print_poly4d("circle_poly4d", &traj, &waypoints);
}

// ── Figure-8 ───────────────────────────────────────────────────────────────

fn export_figure8() {
    // Matches the waypoint layout used in spline_figure8_test.rs.
    // Centered at (0,0), ±0.25m in x, ±0.18m in y — similar to prof's scale.
    let ax = 0.25_f32;   // x half-amplitude
    let ay = 0.18_f32;   // y half-amplitude
    let n_seg = 6_usize;
    let seg_dur = 4.0_f32;  // s per segment (gentle speed)

    let waypoints = vec![
        Waypoint { pos: Vec3::new( 0.0,    0.0, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( ax,     0.0, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.0,    ay,  0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-ax,     0.0, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.0,   -ay,  0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( ax,     0.0, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.0,    0.0, 0.0), yaw: 0.0 },
    ];
    let durations = vec![seg_dur; n_seg];

    eprintln!("=== Figure-8: ax={:.2}m ay={:.2}m, {} segments, {:.1}s each, total={:.1}s ===",
        ax, ay, n_seg, seg_dur, seg_dur * n_seg as f32);
    eprintln!("Solving QP...");

    let traj = SplineTrajectory::plan(&waypoints, &durations, false)
        .expect("Figure-8 planning failed");

    eprintln!("Done. Waypoint validation:");
    print_poly4d("figure8_poly4d", &traj, &waypoints);
}

// ── Flip ───────────────────────────────────────────────────────────────────

/// Export a smooth 360° pitch flip angle trajectory (0 → 2π, rest-to-rest).
///
/// Uses SplineTrajectory::plan (the same QP planner as circle / figure-8)
/// with **two segments** split at the midpoint (π rad at T/2).
///
/// Why two segments?
///   The planner uses degree-8 polynomials (9 coefficients per axis per segment).
///   For a single segment the QP imposes 10 equality constraints (position +
///   zero vel/acc/jerk/snap at both endpoints) on those 9 unknowns — the system
///   is over-determined and Clarabel reports PrimalInfeasible whenever the
///   endpoint displacement is non-zero.
///   With two segments the problem has 18 variables and ~15 constraints →
///   feasible, and the QP finds the globally min-snap solution with the
///   interior derivatives free to be optimised.
///
/// The z-dimension of the waypoints is repurposed as the angle axis θ.
/// Python evaluates θ(t) and ω(t) = dθ/dt per segment and sends
/// `send_full_state_setpoint` at 25 Hz.
fn export_flip() {
    use std::f32::consts::TAU;
    let t_flip = 0.7_f32;
    let t_seg  = t_flip / 2.0;   // 0.35 s per segment

    // Three waypoints encode the angle profile in the z-dimension:
    //   wp[0] = 0 rad  (start, rest)
    //   wp[1] = π rad  (midpoint, derivatives free — QP optimises)
    //   wp[2] = 2π rad (end, rest)
    let flip_wpts = vec![
        Waypoint { pos: Vec3::new(0.0, 0.0, 0.0),        yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.0, 0.0, TAU / 2.0),  yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.0, 0.0, TAU),         yaw: 0.0 },
    ];
    let durations = vec![t_seg, t_seg];

    eprintln!("=== Flip: 0 → 2π rad (360°), T={:.2}s, 2 segments via QP ===", t_flip);
    eprintln!("Solving min-snap QP (2 segments, z-axis = angle)...");

    let traj = SplineTrajectory::plan(&flip_wpts, &durations, false)
        .expect("Flip planning failed");

    eprintln!("Done.");

    // Validate waypoints
    let flat_0   = traj.eval(0.0);
    let flat_mid = traj.eval(t_seg);
    let flat_end = traj.eval(t_flip);
    eprintln!("  theta(0)   = {:.6} rad  (expected 0.000000)", flat_0.pos.z);
    eprintln!("  theta(T/2) = {:.6} rad  (expected {:.6} = π)", flat_mid.pos.z, TAU / 2.0);
    eprintln!("  theta(T)   = {:.6} rad  (expected {:.6} = 2π)", flat_end.pos.z, TAU);
    eprintln!("  gap start  = {:.2e} rad", flat_0.pos.z.abs());
    eprintln!("  gap end    = {:.2e} rad", (flat_end.pos.z - TAU).abs());

    // Find peak angular rate by sampling (flat.vel.z = dθ/dt in rad/s)
    let n_samples = 1000;
    let omega_max_rads = (0..=n_samples)
        .map(|i| traj.eval(t_flip * i as f32 / n_samples as f32).vel.z.abs())
        .fold(0.0_f32, f32::max);
    eprintln!("  Peak omega = {:.0} deg/s ({:.2} rad/s)",
        omega_max_rads.to_degrees(), omega_max_rads);
    eprintln!();

    print_flip_segs("flip_angle_segs", &traj);
}

/// Print flip angle segments as a Python-pasteable nested list.
///
/// Format: [[T_seg, c0..c7], [T_seg, c0..c7], ...]
/// Each row is one segment; c_k are physical-time coefficients for the
/// z-dimension (= angle θ).  Local time runs 0..T_seg within each segment.
fn print_flip_segs(name: &str, traj: &SplineTrajectory) {
    let total: f32 = traj.segments.iter().map(|s| s.duration).sum();
    println!("# Flip angle trajectory — generated by:");
    println!("# cargo run --release --bin export_poly4d flip");
    println!("#");
    println!("# Each row: [T_seg, c0, c1, ..., c7]");
    println!("# theta(t_local) = sum(c_k * t_local^k),  t_local in [0, T_seg]  [radians]");
    println!("# omega(t_local) = d(theta)/dt_local  [rad/s]");
    println!("# Evaluate segment-by-segment; t_local resets to 0 at each segment boundary.");
    println!("# Total T_FLIP = {:.6} s", total);
    println!("{name} = [");
    for seg in &traj.segments {
        let t = seg.duration;
        let c = to_hermite_phys7(&seg.cz, t);
        print!("    [{:.6}", t);
        for &ci in &c { print!(", {:+.9}", ci); }
        println!("],");
    }
    println!("]");
    println!();
    println!("T_FLIP = sum(row[0] for row in {name})");
}

// ── Fast Flip ──────────────────────────────────────────────────────────────

/// Same as export_flip but T=0.45 s (2× faster) — peak ω ≈ 2100 °/s.
/// Used to demonstrate regime where geometric controller model-mismatch shows.
fn export_flip_fast() {
    use std::f32::consts::TAU;
    let t_flip = 0.45_f32;
    let t_seg  = t_flip / 2.0;

    let flip_wpts = vec![
        Waypoint { pos: Vec3::new(0.0, 0.0, 0.0),        yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.0, 0.0, TAU / 2.0),  yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.0, 0.0, TAU),         yaw: 0.0 },
    ];
    let durations = vec![t_seg, t_seg];

    eprintln!("=== Fast Flip: 0 → 2π rad (360°), T={:.2}s, 2 segments via QP ===", t_flip);
    eprintln!("Solving min-snap QP (2 segments, z-axis = angle)...");

    let traj = SplineTrajectory::plan(&flip_wpts, &durations, false)
        .expect("Fast flip planning failed");

    eprintln!("Done.");

    let flat_0   = traj.eval(0.0);
    let flat_mid = traj.eval(t_seg);
    let flat_end = traj.eval(t_flip);
    eprintln!("  theta(0)   = {:.6} rad  (expected 0.000000)", flat_0.pos.z);
    eprintln!("  theta(T/2) = {:.6} rad  (expected {:.6} = π)", flat_mid.pos.z, TAU / 2.0);
    eprintln!("  theta(T)   = {:.6} rad  (expected {:.6} = 2π)", flat_end.pos.z, TAU);
    eprintln!("  gap start  = {:.2e} rad", flat_0.pos.z.abs());
    eprintln!("  gap end    = {:.2e} rad", (flat_end.pos.z - TAU).abs());

    let n_samples = 1000;
    let omega_max_rads = (0..=n_samples)
        .map(|i| traj.eval(t_flip * i as f32 / n_samples as f32).vel.z.abs())
        .fold(0.0_f32, f32::max);
    eprintln!("  Peak omega = {:.0} deg/s ({:.2} rad/s)",
        omega_max_rads.to_degrees(), omega_max_rads);
    eprintln!();

    print_flip_segs("fast_flip_angle_segs", &traj);
}

// ── Figure-8 matching professor's waypoints ────────────────────────────────

/// Re-plan using the exact waypoints and segment durations from the professor's
/// figure8 trajectory (extracted from his Poly4D c0 values).  This lets us
/// directly compare our min-snap QP output against his pre-computed coefficients.
fn export_figure8_match() {
    // Waypoints = c0 of each professor segment, plus the final endpoint (0,0).
    // Durations match his segment durations exactly.
    let waypoints = vec![
        Waypoint { pos: Vec3::new( 0.000000,  0.000000, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.396058, -0.445604, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.922409, -0.291165, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.923174,  0.289869, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.405364,  0.450742, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.000000,  0.000000, 0.0), yaw: 0.0 },  // mid-crossing
        Waypoint { pos: Vec3::new(-0.402804, -0.449354, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.921641, -0.292459, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.923935,  0.288570, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.398611,  0.447039, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.000000,  0.000000, 0.0), yaw: 0.0 },  // end
    ];
    let durations = vec![1.050000_f32, 0.710000, 0.620000, 0.700000, 0.560000,
                         0.560000, 0.700000, 0.620000, 0.710000, 1.053185];

    eprintln!("=== Figure-8 matching prof's waypoints: {} segments, total={:.2}s ===",
        durations.len(), durations.iter().sum::<f32>());
    eprintln!("Solving QP...");

    let traj = SplineTrajectory::plan(&waypoints, &durations, false)
        .expect("Figure-8 matching failed");

    eprintln!("Done. Waypoint validation:");
    print_poly4d("our_figure8_poly4d", &traj, &waypoints);
}

/// Same waypoints as export_figure8_match but every segment duration halved.
/// Properly QP-optimised fast figure-8 (3.64 s total, 2× speed).
fn export_figure8_fast() {
    let waypoints = vec![
        Waypoint { pos: Vec3::new( 0.000000,  0.000000, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.396058, -0.445604, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.922409, -0.291165, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.923174,  0.289869, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.405364,  0.450742, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.000000,  0.000000, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.402804, -0.449354, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.921641, -0.292459, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.923935,  0.288570, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(-0.398611,  0.447039, 0.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new( 0.000000,  0.000000, 0.0), yaw: 0.0 },
    ];
    // Halve every segment duration — same spatial path, 2× faster
    let durations: Vec<f32> = vec![1.050000_f32, 0.710000, 0.620000, 0.700000, 0.560000,
                                   0.560000, 0.700000, 0.620000, 0.710000, 1.053185]
        .iter().map(|&d| d / 2.0).collect();

    eprintln!("=== Fast Figure-8 (2×): figure8_match waypoints, durations halved ===");
    eprintln!("  {} segments, total={:.4}s", durations.len(), durations.iter().sum::<f32>());
    eprintln!("Solving QP...");

    let traj = SplineTrajectory::plan(&waypoints, &durations, false)
        .expect("Fast figure-8 planning failed");

    eprintln!("Done. Waypoint validation:");
    print_poly4d("fast_figure8_poly4d", &traj, &waypoints);
}
