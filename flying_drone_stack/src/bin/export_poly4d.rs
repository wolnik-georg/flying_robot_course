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
        "figure8"       => export_figure8(),
        "figure8_match" => export_figure8_match(),
        _               => export_circle(),
    }
}

// ── Coefficient conversion ─────────────────────────────────────────────────

/// Convert 9 normalized-time (τ∈[0,1]) coefficients to 8 physical-time (t∈[0,T]).
/// c_k = a_k / T^k for k=0..7.
///
/// The degree-8 term a_8 cannot be stored in Poly4D (max degree 7).
/// To preserve the endpoint position p(T) = Σ a_k exactly, we absorb the
/// dropped contribution (a_8·T^8 / T^7 = a_8·T → contribution at t=T is a_8)
/// into c_7:  c_7 += a_8 / T^7.
/// This keeps p(0) = a_0 and p(T) = Σ a_k exact; velocity at t=T shifts by
/// 7·a_8/T but remains small since a_8 is the optimizer's residual degree.
fn to_phys8(norm: &[f32; 9], big_t: f32) -> [f32; 8] {
    let mut out = [0.0f32; 8];
    let mut tk = 1.0f32;
    for k in 0..8 {
        out[k] = norm[k] / tk;
        tk *= big_t;
    }
    // Absorb the dropped degree-8 term into c_7 to close the segment endpoint.
    // Without this, p(T) = Σ_{k=0}^{7} a_k instead of the correct Σ_{k=0}^{8} a_k.
    out[7] += norm[8] / big_t.powi(7);
    out
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
        let cx   = to_phys8(&seg.cx,   t);
        let cy   = to_phys8(&seg.cy,   t);
        let cz   = to_phys8(&seg.cz,   t);
        let cyaw = to_phys8(&seg.cyaw, t);

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
    let n_seg    = 8_usize;    // segments (every 45°)
    let omega    = 0.6_f32;    // planned rad/s  (actual = omega × SPEED_SCALE)
    let seg_dur  = std::f32::consts::TAU / (n_seg as f32 * omega);  // ≈ 1.309 s

    // Circle centered at (radius, 0, 0) relative to the drone's hover position.
    // Starts and ends at (0, 0, 0) so relative_position=True works seamlessly.
    // Counterclockwise when viewed from above.
    let waypoints: Vec<Waypoint> = (0..=n_seg).map(|i| {
        let phi = std::f32::consts::TAU * i as f32 / n_seg as f32;
        Waypoint {
            pos: Vec3::new(radius * (1.0 - phi.cos()), radius * phi.sin(), 0.0),
            yaw: 0.0,
        }
    }).collect();
    let durations = vec![seg_dur; n_seg];

    eprintln!("=== Circle: r={:.2}m, {} segments, {:.3}s each, total={:.2}s ===",
        radius, n_seg, seg_dur, seg_dur * n_seg as f32);
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
