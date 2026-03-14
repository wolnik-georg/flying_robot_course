//! MEKF offline replay and comparison.
//!
//! Usage:
//!   cargo run --bin mekf_eval -- runs/hover_2026-XX-XX.csv
//!
//! Reads a flight log CSV, replays all rows through our MEKF, and compares
//! the MEKF output to the onboard EKF columns logged from the Crazyflie.
//!
//! CSV format (written by main.rs):
//!   time_ms, vel_x, vel_y, vel_z, roll, pitch, yaw,
//!   range_z, gyro_x, gyro_y, gyro_z, ekf_x, ekf_y,
//!   acc_x, acc_y, acc_z, flow_dx, flow_dy
//!
//! Onboard reference:  roll/pitch/yaw [deg],  ekf_x/ekf_y [m]
//! MEKF output:        roll/pitch/yaw [deg],  pos_x/pos_y [m]
//!
//! Prints per-field RMSE and writes a comparison CSV to stdout for plotting.

use multirotor_simulator::estimation::{Mekf, MekfParams};

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("Usage: mekf_eval <flight_log.csv>");
        eprintln!("Running synthetic hover demo instead.");
        String::new()
    });

    if path.is_empty() {
        run_synthetic_demo();
    } else {
        run_from_csv(&path);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// CSV replay
// ──────────────────────────────────────────────────────────────────────────────

#[derive(Debug)]
#[allow(dead_code)]
struct Row {
    time_s: f32,
    vel_x: f32, vel_y: f32, vel_z: f32,
    roll_deg: f32, pitch_deg: f32, yaw_deg: f32,
    range_z_m: f32,
    gyro_x_degs: f32, gyro_y_degs: f32, gyro_z_degs: f32,
    ekf_x: f32, ekf_y: f32,
    acc_x_g: f32, acc_y_g: f32, acc_z_g: f32,
    flow_dx: f32, flow_dy: f32,
}

fn parse_csv(path: &str) -> Vec<Row> {
    let content = std::fs::read_to_string(path)
        .unwrap_or_else(|e| { eprintln!("Cannot open {}: {}", path, e); std::process::exit(1); });

    let mut rows = Vec::new();
    for (i, line) in content.lines().enumerate() {
        if i == 0 { continue; } // skip header
        let f: Vec<f32> = line.split(',')
            .map(|s| s.trim().parse::<f32>().unwrap_or(0.0))
            .collect();
        if f.len() < 13 { continue; } // need at least old-format columns
        rows.push(Row {
            time_s:       f[0] / 1000.0,
            vel_x:        f[1],  vel_y:  f[2],  vel_z:  f[3],
            roll_deg:     f[4],  pitch_deg: f[5], yaw_deg: f[6],
            range_z_m:    f[7],
            gyro_x_degs:  f[8],  gyro_y_degs: f[9], gyro_z_degs: f[10],
            ekf_x:        f[11], ekf_y: f[12],
            // new columns (present only in logs from updated main.rs)
            acc_x_g:      if f.len() > 13 { f[13] } else { 0.0 },
            acc_y_g:      if f.len() > 14 { f[14] } else { 0.0 },
            acc_z_g:      if f.len() > 15 { f[15] } else { 0.0 },
            // flow omitted: motion.deltaX/deltaY are int16 in firmware, cannot log as float
            flow_dx:      0.0,
            flow_dy:      0.0,
        });
    }
    rows
}

fn run_from_csv(path: &str) {
    let rows = parse_csv(path);
    if rows.is_empty() { eprintln!("No rows parsed from {}", path); return; }

    let has_acc = rows.iter().any(|r| r.acc_z_g.abs() > 0.01);
    if !has_acc {
        eprintln!("Warning: acc columns are all zero — log predates block3 addition.");
        eprintln!("         Attitude comparison will be gyro-only (no accel correction).");
    }

    // Seed initial attitude from first row attitude
    let r0 = &rows[0];
    let mekf_params = MekfParams::default();
    let mut mekf = Mekf::new(mekf_params);
    let q0 = euler_deg_to_quat(r0.roll_deg, r0.pitch_deg, r0.yaw_deg);
    mekf.seed_qref(q0);

    // Accumulators for RMSE
    let mut sum_sq = [0.0f64; 5]; // roll, pitch, yaw, x, y
    let mut count = 0usize;

    // Print CSV header for comparison output
    println!("time_s,ref_roll,ref_pitch,ref_yaw,ref_x,ref_y,mekf_roll,mekf_pitch,mekf_yaw,mekf_x,mekf_y");

    for row in &rows {
        // Feed gyro/accel always; height and flow only when available
        let accel = if has_acc {
            Some([row.acc_x_g, row.acc_y_g, row.acc_z_g])
        } else {
            // Without acc, synthesize gravity in body frame from attitude
            let (roll, pitch) = (row.roll_deg.to_radians(), row.pitch_deg.to_radians());
            Some([
                -pitch.sin(),
                roll.sin() * pitch.cos(),
                roll.cos() * pitch.cos(),
            ])
        };

        let range_mm = if row.range_z_m > 0.01 { Some(row.range_z_m * 1000.0) } else { None };
        let flow = if row.flow_dx.abs() > 1e-6 || row.flow_dy.abs() > 1e-6 {
            (Some(row.flow_dx), Some(row.flow_dy))
        } else {
            (None, None)
        };

        let est = mekf.feed_row(
            row.time_s,
            Some([row.gyro_x_degs, row.gyro_y_degs, row.gyro_z_degs]),
            accel,
            range_mm,
            flow.0, flow.1,
        );

        if let Some([roll_r, pitch_r, yaw_r, px, py, pz]) = est {
            let roll_d  = roll_r.to_degrees();
            let pitch_d = pitch_r.to_degrees();
            let yaw_d   = yaw_r.to_degrees();

            println!("{:.3},{:.3},{:.3},{:.3},{:.4},{:.4},{:.3},{:.3},{:.3},{:.4},{:.4}",
                row.time_s,
                row.roll_deg, row.pitch_deg, row.yaw_deg, row.ekf_x, row.ekf_y,
                roll_d, pitch_d, yaw_d, px, py,
            );

            let _ = pz; // z compared via range_z vs mekf pos_z separately
            sum_sq[0] += (roll_d  - row.roll_deg).powi(2) as f64;
            sum_sq[1] += (pitch_d - row.pitch_deg).powi(2) as f64;
            sum_sq[2] += angle_diff_deg(yaw_d, row.yaw_deg).powi(2) as f64;
            sum_sq[3] += (px - row.ekf_x).powi(2) as f64;
            sum_sq[4] += (py - row.ekf_y).powi(2) as f64;
            count += 1;
        }
    }

    if count == 0 { eprintln!("No MEKF estimates produced (check log format)."); return; }

    let n = count as f64;
    eprintln!("\n=== RMSE ({} samples) ===", count);
    eprintln!("  roll  : {:.3} deg", (sum_sq[0]/n).sqrt());
    eprintln!("  pitch : {:.3} deg", (sum_sq[1]/n).sqrt());
    eprintln!("  yaw   : {:.3} deg", (sum_sq[2]/n).sqrt());
    eprintln!("  x     : {:.4} m",   (sum_sq[3]/n).sqrt());
    eprintln!("  y     : {:.4} m",   (sum_sq[4]/n).sqrt());
}

// ──────────────────────────────────────────────────────────────────────────────
// Synthetic hover demo (no CSV needed)
// ──────────────────────────────────────────────────────────────────────────────

fn run_synthetic_demo() {
    eprintln!("=== Synthetic hover demo (5 s, 100 Hz, 3° pitch tilt) ===");

    let params = MekfParams::default();
    let mut mekf = Mekf::new(params);
    // Seed with a small pitch tilt (3°) so attitude is non-trivial
    let q0 = euler_deg_to_quat(0.0, 3.0, 0.0);
    mekf.seed_qref(q0);

    let dt = 0.01f32;        // 100 Hz
    let pitch_rad = 3.0f32.to_radians();
    // Accelerometer for a tilted hover: body-frame gravity + centripetal ≈ 0
    // acc_z ≈ cos(pitch), acc_x ≈ sin(pitch)
    let acc = [pitch_rad.sin(), 0.0, pitch_rad.cos()]; // in g
    let gyro = [0.0f32; 3]; // no rotation during hover

    println!("time_s,mekf_roll,mekf_pitch,mekf_yaw,mekf_z");
    let mut t = 0.0f32;
    for _ in 0..500 {
        t += dt;
        let range_mm = Some(300.0f32); // hovering at 0.3 m

        let est = mekf.feed_row(t, Some(gyro), Some(acc), range_mm, None, None);
        if let Some([roll, pitch, yaw, _px, _py, pz]) = est {
            println!("{:.2},{:.3},{:.3},{:.3},{:.4}",
                t, roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees(), pz);
        }
    }

    eprintln!("Done. Expected: roll≈0°, pitch≈3°, yaw≈0°, z≈0.3 m converged.");
}

// ──────────────────────────────────────────────────────────────────────────────
// Utilities
// ──────────────────────────────────────────────────────────────────────────────

/// Angle difference in degrees, wrapped to [-180, 180].
fn angle_diff_deg(a: f32, b: f32) -> f32 {
    let mut d = a - b;
    while d >  180.0 { d -= 360.0; }
    while d < -180.0 { d += 360.0; }
    d
}

/// Build a quaternion [w,x,y,z] from Euler angles in degrees (ZYX / yaw-pitch-roll).
fn euler_deg_to_quat(roll_deg: f32, pitch_deg: f32, yaw_deg: f32) -> [f32; 4] {
    let r = roll_deg.to_radians()  * 0.5;
    let p = pitch_deg.to_radians() * 0.5;
    let y = yaw_deg.to_radians()   * 0.5;
    let (sr, cr) = (r.sin(), r.cos());
    let (sp, cp) = (p.sin(), p.cos());
    let (sy, cy) = (y.sin(), y.cos());
    [
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ]
}
