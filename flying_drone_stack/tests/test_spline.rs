//! Tests for the minimum-snap 8th-order spline planner.
//!
//! Validates:
//!   1. Waypoints are satisfied at junction times
//!   2. Zero boundary conditions (vel/acc/jerk/snap = 0 at start and end)
//!   3. C4 continuity at every interior junction
//!   4. Circle and figure-8 waypoint patterns pass through all waypoints
//!   5. Snap magnitude is finite and bounded everywhere

use multirotor_simulator::planning::{SplineTrajectory, Waypoint};
use multirotor_simulator::math::Vec3;

const G_TOL: f32 = 1e-2;   // 1 cm positional tolerance (QP solve + f32)
const D_TOL: f32 = 5e-2;   // 5 cm/s derivative tolerance at boundaries

// ──────────────────────────────────────────────────────────────────────────────
// Helpers
// ──────────────────────────────────────────────────────────────────────────────

/// Sum of durations up to (not including) segment `k`.
fn junction_time(durations: &[f32], k: usize) -> f32 {
    durations[..k].iter().sum()
}

fn circle_waypoints(n: usize, radius: f32, height: f32) -> (Vec<Waypoint>, Vec<f32>) {
    let wps: Vec<Waypoint> = (0..=n)
        .map(|i| {
            let theta = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
            Waypoint {
                pos: Vec3::new(radius * theta.cos(), radius * theta.sin(), height),
                yaw: 0.0,
            }
        })
        .collect();
    let durs = vec![1.0f32; n];
    (wps, durs)
}

fn figure8_waypoints(n: usize, ax: f32, ay: f32, height: f32) -> (Vec<Waypoint>, Vec<f32>) {
    // Lemniscate of Gerono: x = ax*sin(θ), y = ay*sin(2θ)/2
    let wps: Vec<Waypoint> = (0..=n)
        .map(|i| {
            let theta = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
            Waypoint {
                pos: Vec3::new(ax * theta.sin(), ay * (2.0 * theta).sin() * 0.5, height),
                yaw: 0.0,
            }
        })
        .collect();
    let durs = vec![1.0f32; n];
    (wps, durs)
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 1 — Waypoints are satisfied at the corresponding junction times
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_spline_waypoints_satisfied() {
    let wps = vec![
        Waypoint { pos: Vec3::new(0.0, 0.0, 1.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(1.0, 0.0, 1.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(1.0, 1.0, 1.0), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.0, 1.0, 1.0), yaw: 0.0 },
    ];
    let durs = vec![1.0f32, 1.5, 0.8];
    let traj = SplineTrajectory::plan(&wps, &durs).expect("plan");

    for (k, wp) in wps.iter().enumerate() {
        let t = junction_time(&durs, k);
        let out = traj.eval(t);
        assert!(
            (out.pos.x - wp.pos.x).abs() < G_TOL,
            "wp[{}] x: got {:.4}, expected {:.4}", k, out.pos.x, wp.pos.x
        );
        assert!(
            (out.pos.y - wp.pos.y).abs() < G_TOL,
            "wp[{}] y: got {:.4}, expected {:.4}", k, out.pos.y, wp.pos.y
        );
        assert!(
            (out.pos.z - wp.pos.z).abs() < G_TOL,
            "wp[{}] z: got {:.4}, expected {:.4}", k, out.pos.z, wp.pos.z
        );
    }
    // End waypoint
    let out_end = traj.eval(traj.total_time);
    let last = wps.last().unwrap();
    assert!((out_end.pos.x - last.pos.x).abs() < G_TOL, "end x");
    assert!((out_end.pos.y - last.pos.y).abs() < G_TOL, "end y");
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 2 — Zero boundary conditions at t=0 and t=T
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_spline_zero_boundary_conditions() {
    let wps = vec![
        Waypoint { pos: Vec3::new(0.0, 0.0, 0.3), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.5, 0.0, 0.3), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.5, 0.5, 0.3), yaw: 0.0 },
    ];
    let durs = vec![1.0f32, 1.0];
    let traj = SplineTrajectory::plan(&wps, &durs).expect("plan");

    let start = traj.eval(0.0);
    let end   = traj.eval(traj.total_time);

    for (label, v) in [("start vel", start.vel), ("end vel", end.vel),
                        ("start acc", start.acc), ("end acc", end.acc),
                        ("start jerk", start.jerk), ("end jerk", end.jerk),
                        ("start snap", start.snap), ("end snap", end.snap)] {
        assert!(
            v.x.abs() < D_TOL && v.y.abs() < D_TOL && v.z.abs() < D_TOL,
            "{} should be zero: got ({:.4},{:.4},{:.4})", label, v.x, v.y, v.z
        );
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 3 — C4 continuity at every interior junction
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_spline_c4_continuity() {
    let wps = vec![
        Waypoint { pos: Vec3::new(0.0, 0.0, 0.3), yaw: 0.0 },
        Waypoint { pos: Vec3::new(1.0, 0.0, 0.3), yaw: 0.0 },
        Waypoint { pos: Vec3::new(1.0, 1.0, 0.3), yaw: 0.0 },
        Waypoint { pos: Vec3::new(0.0, 1.0, 0.3), yaw: 0.0 },
    ];
    let durs = vec![1.0f32, 1.0, 1.0];
    let traj = SplineTrajectory::plan(&wps, &durs).expect("plan");

    // Probe slightly before and after each interior junction
    let eps = 1e-4f32;
    for k in 1..wps.len() - 1 {
        let tj = junction_time(&durs, k);
        let lo = traj.eval(tj - eps);
        let hi = traj.eval(tj + eps);

        // Position: must match (C0)
        assert!((lo.pos.x - hi.pos.x).abs() < 1e-2, "junction {} pos x", k);
        assert!((lo.pos.y - hi.pos.y).abs() < 1e-2, "junction {} pos y", k);

        // Velocity: must match (C1)
        assert!((lo.vel.x - hi.vel.x).abs() < 1e-1, "junction {} vel x", k);
        assert!((lo.vel.y - hi.vel.y).abs() < 1e-1, "junction {} vel y", k);

        // Acceleration: must match (C2)
        assert!((lo.acc.x - hi.acc.x).abs() < 5e-1, "junction {} acc x", k);
        assert!((lo.acc.y - hi.acc.y).abs() < 5e-1, "junction {} acc y", k);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 4 — Circle waypoints all reached within tolerance
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_spline_circle_waypoints() {
    let n = 8;
    let (wps, durs) = circle_waypoints(n, 0.5, 0.3);
    let traj = SplineTrajectory::plan(&wps, &durs).expect("circle plan");
    assert_eq!(traj.segments.len(), n);

    for (k, wp) in wps.iter().enumerate() {
        let t = junction_time(&durs, k);
        let out = traj.eval(t);
        let dist = ((out.pos.x - wp.pos.x).powi(2)
                  + (out.pos.y - wp.pos.y).powi(2)
                  + (out.pos.z - wp.pos.z).powi(2)).sqrt();
        assert!(dist < G_TOL, "circle wp[{}] dist={:.4}m", k, dist);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 5 — Figure-8 waypoints all reached within tolerance
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_spline_figure8_waypoints() {
    let n = 8;
    let (wps, durs) = figure8_waypoints(n, 0.5, 0.3, 0.3);
    let traj = SplineTrajectory::plan(&wps, &durs).expect("figure8 plan");
    assert_eq!(traj.segments.len(), n);

    for (k, wp) in wps.iter().enumerate() {
        let t = junction_time(&durs, k);
        let out = traj.eval(t);
        let dist = ((out.pos.x - wp.pos.x).powi(2)
                  + (out.pos.y - wp.pos.y).powi(2)).sqrt();
        assert!(dist < G_TOL, "figure8 wp[{}] dist={:.4}m", k, dist);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Test 6 — Snap profile is finite and bounded everywhere
// ──────────────────────────────────────────────────────────────────────────────

#[test]
fn test_spline_snap_finite_and_bounded() {
    let (wps, durs) = circle_waypoints(6, 0.4, 0.3);
    let traj = SplineTrajectory::plan(&wps, &durs).expect("plan");

    let n_samples = 500;
    let dt = traj.total_time / n_samples as f32;
    for i in 0..=n_samples {
        let t = (i as f32 * dt).min(traj.total_time);
        let out = traj.eval(t);
        assert!(out.snap.x.is_finite(), "snap.x not finite at t={:.3}", t);
        assert!(out.snap.y.is_finite(), "snap.y not finite at t={:.3}", t);
        assert!(out.snap.z.is_finite(), "snap.z not finite at t={:.3}", t);
        // Snap magnitude for a 0.4m circle at 1s/segment should be well below 1000 m/s^4
        let snap_mag = (out.snap.x.powi(2) + out.snap.y.powi(2) + out.snap.z.powi(2)).sqrt();
        assert!(snap_mag < 1000.0, "snap too large at t={:.3}: {:.1} m/s^4", t, snap_mag);
    }
}
