//! Assignment 3: MEKF offline validation
//!
//! Loads the figure-8 flight log (fr00.csv), runs the Rust MEKF, and writes
//! results/data/assignment3_mekf.csv for comparison with the on-board EKF.
//!
//! Usage:
//!   cargo run --release --bin assignment3 -- [--csv PATH]
//!
//! Default CSV path: ../State\ Estimation/logging_ekf/logging/fr00.csv
//! (relative to the multirotor_simulator directory)

use multirotor_simulator::prelude::*;
use std::fs::File;
use std::io::Write;
use std::collections::HashMap;

/// One row of fr00.csv.  All sensor fields are Optional (NaN → None).
struct LogRow {
    timestamp: f32,
    gyro: Option<[f32; 3]>,    // deg/s
    accel: Option<[f32; 3]>,   // G
    flow_dx: Option<f32>,      // pixels
    flow_dy: Option<f32>,
    range_mm: Option<f32>,     // mm
    ekf_q: Option<[f32; 4]>,  // on-board EKF quaternion [w,x,y,z]
    ekf_pos: Option<[f32; 3]>, // on-board EKF position [m]
}

fn load_csv(path: &str) -> Vec<LogRow> {
    // The Crazyflie logger wraps long lines (header and data rows can span
    // multiple physical lines).  The `csv` crate handles this transparently.
    // The header starts with "# " — strip that prefix so column names parse cleanly.
    let raw = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("Cannot open '{}': {}", path, e));

    // Replace the leading "# " on the first non-empty line so the csv crate
    // sees a clean header.
    let cleaned: String = {
        let mut done = false;
        raw.lines()
            .map(|l| {
                if !done {
                    if let Some(stripped) = l.strip_prefix("# ") {
                        done = true;
                        return stripped.to_string();
                    }
                    if let Some(stripped) = l.strip_prefix('#') {
                        done = true;
                        return stripped.to_string();
                    }
                }
                l.to_string()
            })
            .collect::<Vec<_>>()
            .join("\n")
    };

    let mut rdr = csv::ReaderBuilder::new()
        .has_headers(true)
        .flexible(true)
        .from_reader(cleaned.as_bytes());

    let headers = rdr.headers().unwrap().clone();
    let col: HashMap<String, usize> = headers.iter()
        .enumerate()
        .map(|(i, h)| (h.trim().to_string(), i))
        .collect();

    let get_f = |record: &csv::StringRecord, name: &str| -> Option<f32> {
        let i = col.get(name)?;
        let s = record.get(*i)?;
        let v: f32 = s.trim().parse().ok()?;
        if v.is_nan() || v.is_infinite() { None } else { Some(v) }
    };

    let mut rows = Vec::new();
    for result in rdr.records() {
        let record = match result {
            Ok(r) => r,
            Err(_) => continue,
        };

        let ts = match get_f(&record, "timestamp") {
            Some(t) => t,
            None => continue,
        };

        let gyro = match (
            get_f(&record, "gyro.x"),
            get_f(&record, "gyro.y"),
            get_f(&record, "gyro.z"),
        ) {
            (Some(x), Some(y), Some(z)) => Some([x, y, z]),
            _ => None,
        };

        let accel = match (
            get_f(&record, "acc.x"),
            get_f(&record, "acc.y"),
            get_f(&record, "acc.z"),
        ) {
            (Some(x), Some(y), Some(z)) => Some([x, y, z]),
            _ => None,
        };

        let flow_dx = get_f(&record, "motion.deltaX");
        let flow_dy = get_f(&record, "motion.deltaY");
        let range_mm = get_f(&record, "range.zrange");

        let ekf_q = match (
            get_f(&record, "stateEstimate.qw"),
            get_f(&record, "stateEstimate.qx"),
            get_f(&record, "stateEstimate.qy"),
            get_f(&record, "stateEstimate.qz"),
        ) {
            (Some(w), Some(x), Some(y), Some(z)) => Some([w, x, y, z]),
            _ => None,
        };

        let ekf_pos = match (
            get_f(&record, "stateEstimate.x"),
            get_f(&record, "stateEstimate.y"),
            get_f(&record, "stateEstimate.z"),
        ) {
            (Some(x), Some(y), Some(z)) => Some([x, y, z]),
            _ => None,
        };

        rows.push(LogRow { timestamp: ts, gyro, accel, flow_dx, flow_dy, range_mm, ekf_q, ekf_pos });
    }

    rows
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let csv_path = args.iter()
        .position(|a| a == "--csv")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("../State Estimation/logging_ekf/logging/fr00.csv");

    println!("Assignment 3: MEKF offline validation (Rust)");
    println!("Loading: {}", csv_path);

    let rows = load_csv(csv_path);
    println!("Loaded {} rows", rows.len());

    // --- Seed q_ref from the first on-board EKF quaternion ---
    let q_seed = rows.iter()
        .find_map(|r| r.ekf_q)
        .unwrap_or([1.0, 0.0, 0.0, 0.0]);

    let params = MekfParams::default();
    let mut mekf = Mekf::new(params);
    mekf.seed_qref(q_seed);

    // Output storage
    let mut mekf_out: Vec<(f32, [f32; 6])> = Vec::new(); // (time, [roll,pitch,yaw,x,y,z])
    let mut ekf_out:  Vec<(f32, [f32; 7])> = Vec::new(); // (time, [roll,pitch,yaw,x,y,z,_])

    // --- Collect on-board EKF baseline and run MEKF ---
    for row in &rows {
        let t = row.timestamp;

        // On-board EKF baseline
        if let (Some(q), Some(p)) = (row.ekf_q, row.ekf_pos) {
            let euler = multirotor_simulator::estimation::mekf::quat_to_euler(q);
            ekf_out.push((t, [euler[0], euler[1], euler[2], p[0], p[1], p[2], 0.0]));
        }

        // Feed sensors into MEKF
        if let Some(est) = mekf.feed_row(
            t,
            row.gyro,
            row.accel,
            row.range_mm,
            row.flow_dx,
            row.flow_dy,
        ) {
            mekf_out.push((t, est));
        }
    }

    println!(
        "MEKF produced {} estimates over {:.2} s",
        mekf_out.len(),
        mekf_out.last().map(|(t,_)| *t).unwrap_or(0.0)
            - mekf_out.first().map(|(t,_)| *t).unwrap_or(0.0)
    );
    println!("On-board EKF has {} samples", ekf_out.len());

    // --- Write MEKF CSV ---
    std::fs::create_dir_all("results/data").expect("Failed to create results/data");
    let mekf_path = "results/data/assignment3_mekf.csv";
    {
        let mut f = File::create(mekf_path).expect("Cannot create MEKF CSV");
        writeln!(f, "time,roll_rad,pitch_rad,yaw_rad,x,y,z").unwrap();
        for (t, est) in &mekf_out {
            writeln!(f, "{:.9},{:.9},{:.9},{:.9},{:.6},{:.6},{:.6}",
                t, est[0], est[1], est[2], est[3], est[4], est[5]).unwrap();
        }
    }
    println!("MEKF results → {}", mekf_path);

    // --- Write on-board EKF CSV ---
    let ekf_path = "results/data/assignment3_ekf.csv";
    {
        let mut f = File::create(ekf_path).expect("Cannot create EKF CSV");
        writeln!(f, "time,roll_rad,pitch_rad,yaw_rad,x,y,z").unwrap();
        for (t, est) in &ekf_out {
            writeln!(f, "{:.9},{:.9},{:.9},{:.9},{:.6},{:.6},{:.6}",
                t, est[0], est[1], est[2], est[3], est[4], est[5]).unwrap();
        }
    }
    println!("On-board EKF baseline → {}", ekf_path);

    // --- Quick quality report (RMS yaw/roll/pitch) ---
    let rad2deg = 180.0_f32 / std::f32::consts::PI;
    if mekf_out.len() > 10 && ekf_out.len() > 10 {
        // Simple nearest-neighbour interpolation for RMS
        let mut sum_sq = [0.0f32; 3];
        let mut n = 0usize;
        for (et, eest) in &ekf_out {
            // find closest MEKF sample
            if let Some((_, mest)) = mekf_out.iter().min_by(|(ta,_),(tb,_)| {
                (ta - et).abs().partial_cmp(&(tb - et).abs()).unwrap()
            }) {
                for i in 0..3 {
                    let d = (mest[i] - eest[i]) * rad2deg;
                    sum_sq[i] += d * d;
                }
                n += 1;
            }
        }
        if n > 0 {
            let nf = n as f32;
            println!("\nOrientation RMS (MEKF vs on-board EKF):");
            println!("  roll  = {:.3}°", (sum_sq[0]/nf).sqrt());
            println!("  pitch = {:.3}°", (sum_sq[1]/nf).sqrt());
            println!("  yaw   = {:.3}°", (sum_sq[2]/nf).sqrt());
        }
    }
}
