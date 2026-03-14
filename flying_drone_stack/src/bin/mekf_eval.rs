//! MEKF offline replay and comparison.
//!
//! Usage:
//!   cargo run --bin mekf_eval -- runs/hover_2026-XX-XX.csv
//!
//! Reads a flight log CSV, replays all rows through our MEKF, and compares
//! the MEKF output to the onboard EKF columns logged from the Crazyflie.
//!
//! Supports both log formats written by main.rs:
//!
//! **New firmware-mode format** (hover / circle / figure8, Mar 2026+):
//!   time_ms, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
//!   roll, pitch, yaw, thrust, vbat,
//!   gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z,
//!   rate_roll, rate_pitch, rate_yaw, [range_z]
//!   → pos_x/y/z  = Lighthouse EKF ground truth
//!   → range_z    = ToF height [m] (MEKF height update input)
//!
//! **Old my_hover / my_circle format**:
//!   time_ms, vel_x, vel_y, vel_z, roll, pitch, yaw, range_z,
//!   gyro_x, gyro_y, gyro_z, ekf_x, ekf_y, acc_x, acc_y, acc_z
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
    // Attitude ground truth (onboard EKF / stabilizer)
    roll_deg: f32, pitch_deg: f32, yaw_deg: f32,
    // Position ground truth (Lighthouse EKF or onboard EKF)
    gt_x: f32, gt_y: f32, gt_z: f32,
    // MEKF inputs
    gyro_x_degs: f32, gyro_y_degs: f32, gyro_z_degs: f32,
    acc_x_g: f32, acc_y_g: f32, acc_z_g: f32,
    // Height update input: ToF range in metres (0 = unavailable)
    range_z_m: f32,
    // Optical flow: raw pixel counts from motion.deltaX/Y (0 = unavailable)
    flow_dx: f32,
    flow_dy: f32,
}

/// Parse a CSV using column headers rather than fixed indices.
/// Handles both the new firmware-mode format and the old my_hover/my_circle format.
fn parse_csv(path: &str) -> Vec<Row> {
    let content = std::fs::read_to_string(path)
        .unwrap_or_else(|e| { eprintln!("Cannot open {}: {}", path, e); std::process::exit(1); });

    let mut lines = content.lines();

    // --- Parse header ---
    let header_line = lines.next().unwrap_or("");
    let cols: Vec<&str> = header_line.split(',').map(|s| s.trim()).collect();

    // Build a name→index map
    let idx = |name: &str| -> Option<usize> {
        cols.iter().position(|&c| c == name)
    };

    // Detect format by presence of "pos_x" (new) vs "ekf_x" (old)
    let is_new_format = idx("pos_x").is_some();

    // Column indices — new format
    let i_time    = idx("time_ms").unwrap_or(0);
    let i_roll    = idx("roll").or_else(|| idx("stabilizer.roll")).unwrap_or(usize::MAX);
    let i_pitch   = idx("pitch").or_else(|| idx("stabilizer.pitch")).unwrap_or(usize::MAX);
    let i_yaw     = idx("yaw").or_else(|| idx("stabilizer.yaw")).unwrap_or(usize::MAX);
    let i_gyro_x  = idx("gyro_x").unwrap_or(usize::MAX);
    let i_gyro_y  = idx("gyro_y").unwrap_or(usize::MAX);
    let i_gyro_z  = idx("gyro_z").unwrap_or(usize::MAX);
    let i_acc_x   = idx("acc_x").unwrap_or(usize::MAX);
    let i_acc_y   = idx("acc_y").unwrap_or(usize::MAX);
    let i_acc_z   = idx("acc_z").unwrap_or(usize::MAX);

    // Ground-truth position: new format uses pos_x/y/z; old format uses ekf_x/ekf_y
    let (i_gt_x, i_gt_y, i_gt_z) = if is_new_format {
        (idx("pos_x").unwrap_or(usize::MAX),
         idx("pos_y").unwrap_or(usize::MAX),
         idx("pos_z").unwrap_or(usize::MAX))
    } else {
        (idx("ekf_x").unwrap_or(usize::MAX),
         idx("ekf_y").unwrap_or(usize::MAX),
         usize::MAX)
    };

    // ToF range: new format may have "range_z" appended; old format has "range_z" at index 7
    let i_range_z = idx("range_z").unwrap_or(usize::MAX);

    // Optical flow: present only in logs captured after the block4 expansion
    let i_flow_dx = idx("flow_dx").unwrap_or(usize::MAX);
    let i_flow_dy = idx("flow_dy").unwrap_or(usize::MAX);

    if is_new_format {
        eprintln!("Detected new firmware-mode log format.");
        eprintln!("  ground truth: pos_x/y/z (onboard EKF via flow+range)");
        if i_range_z != usize::MAX {
            eprintln!("  range_z column found — will use for MEKF height update.");
        } else {
            eprintln!("  No range_z column — using pos_z as height update proxy.");
        }
        if i_flow_dx != usize::MAX {
            eprintln!("  flow_dx/flow_dy columns found — will use for MEKF XY update.");
        } else {
            eprintln!("  No flow columns — XY will dead-reckon only (expected for these logs).");
        }
    } else {
        eprintln!("Detected old my_hover/my_circle log format.");
    }

    let get = |f: &[f32], i: usize| -> f32 {
        if i == usize::MAX || i >= f.len() { 0.0 } else { f[i] }
    };

    let mut rows = Vec::new();
    for line in lines {
        if line.trim().is_empty() { continue; }
        let f: Vec<f32> = line.split(',')
            .map(|s| s.trim().parse::<f32>().unwrap_or(0.0))
            .collect();
        if f.len() < 5 { continue; }

        let gt_z_raw = get(&f, i_gt_z);
        // If no dedicated range_z column, fall back to pos_z (Lighthouse height)
        let range_z = if i_range_z != usize::MAX {
            get(&f, i_range_z)
        } else if is_new_format && gt_z_raw > 0.01 {
            gt_z_raw   // use Lighthouse Z as height proxy
        } else {
            0.0
        };

        rows.push(Row {
            time_s:       get(&f, i_time) / 1000.0,
            roll_deg:     get(&f, i_roll),
            pitch_deg:    get(&f, i_pitch),
            yaw_deg:      get(&f, i_yaw),
            gt_x:         get(&f, i_gt_x),
            gt_y:         get(&f, i_gt_y),
            gt_z:         gt_z_raw,
            gyro_x_degs:  get(&f, i_gyro_x),
            gyro_y_degs:  get(&f, i_gyro_y),
            gyro_z_degs:  get(&f, i_gyro_z),
            acc_x_g:      get(&f, i_acc_x),
            acc_y_g:      get(&f, i_acc_y),
            acc_z_g:      get(&f, i_acc_z),
            range_z_m:    range_z,
            flow_dx:      get(&f, i_flow_dx),
            flow_dy:      get(&f, i_flow_dy),
        });
    }

    eprintln!("Parsed {} rows from {}", rows.len(), path);
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

    // Find first genuinely airborne row (range_z > 0.1 m) to use as origin seed.
    // Before liftoff the drone is sitting on the ground and the onboard EKF position
    // is wherever it was initialised — we seed the MEKF from that same origin so
    // the RMSE measures actual tracking quality, not a constant frame offset.
    //
    // We also START replay only from this row, not from t=0.  During motor spinup
    // the gyros see huge transient rates that have no accel/magnetometer correction
    // available, so integrating them just corrupts the yaw estimate irreversibly.
    let seed_idx = rows.iter()
        .position(|r| r.range_z_m > 0.1)
        .unwrap_or(0);
    let seed_row = &rows[seed_idx];

    // Seed initial attitude from the liftoff row
    let mekf_params = MekfParams::default();
    let mut mekf = Mekf::new(mekf_params);
    let q0 = euler_deg_to_quat(seed_row.roll_deg, seed_row.pitch_deg, seed_row.yaw_deg);
    mekf.seed_qref(q0);

    // Seed initial XY/Z position from onboard EKF at liftoff.
    // In real-time deployment the first EKF fix would do the same thing.
    mekf.state.x[0] = seed_row.gt_x;
    mekf.state.x[1] = seed_row.gt_y;
    mekf.state.x[2] = seed_row.range_z_m; // use ToF directly as Z seed

    eprintln!("Seeded MEKF from first airborne row (range_z > 0.1 m) at index {}:", seed_idx);
    eprintln!("  t={:.3}s  yaw={:.2}°  x={:.4} m  y={:.4} m  z={:.4} m",
        seed_row.time_s, seed_row.yaw_deg, seed_row.gt_x, seed_row.gt_y, seed_row.range_z_m);

    // Accumulators for RMSE: roll, pitch, yaw, x, y, z
    let mut sum_sq = [0.0f64; 6];
    let mut count = 0usize;
    let mut count_z = 0usize;

    // Print CSV header for comparison output (stdout → redirect to file for plotting)
    println!("time_s,ref_roll,ref_pitch,ref_yaw,ref_x,ref_y,ref_z,mekf_roll,mekf_pitch,mekf_yaw,mekf_x,mekf_y,mekf_z");

    for row in &rows[seed_idx..] {
        // Feed gyro/accel always; height only when available
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

        // Pass flow pixel counts if they are non-zero (zero means sensor didn't fire this tick).
        //
        // AXIS CONVENTION: the PMW3901 sensor on the flow deck reports motion in its own
        // image-plane frame, which is rotated 90° relative to the Crazyflie body frame.
        // Correlation analysis against logged vel_x/vel_y shows:
        //   vel_x  ↔  -flow_dy   (r ≈ +0.74)
        //   vel_y  ↔  -flow_dx   (r ≈ +0.30)
        // So we swap and negate before handing to the MEKF.
        let (dnx, dny) = (-row.flow_dy, -row.flow_dx);
        let flow = if dnx != 0.0 || dny != 0.0 {
            (Some(dnx), Some(dny))
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

            println!("{:.3},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4}",
                row.time_s,
                row.roll_deg, row.pitch_deg, row.yaw_deg,
                row.gt_x, row.gt_y, row.gt_z,
                roll_d, pitch_d, yaw_d, px, py, pz,
            );

            sum_sq[0] += (roll_d  - row.roll_deg).powi(2) as f64;
            sum_sq[1] += (pitch_d - row.pitch_deg).powi(2) as f64;
            sum_sq[2] += angle_diff_deg(yaw_d, row.yaw_deg).powi(2) as f64;
            sum_sq[3] += (px - row.gt_x).powi(2) as f64;
            sum_sq[4] += (py - row.gt_y).powi(2) as f64;
            count += 1;

            if row.gt_z > 0.01 {
                sum_sq[5] += (pz - row.gt_z).powi(2) as f64;
                count_z += 1;
            }
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
    if count_z > 0 {
        eprintln!("  z     : {:.4} m   ({} samples with gt_z > 0.01)",
            (sum_sq[5] / count_z as f64).sqrt(), count_z);
    }
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
