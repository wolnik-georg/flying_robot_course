//! SLAM offline evaluator — loop closure and VO drift analysis.
//!
//! Usage:
//!   cargo run --bin slam_eval -- runs/circle_2026-XX-XX.csv
//!
//! Reads a 49-column flight log CSV produced by `main.rs` (Phase 6+),
//! and prints:
//!   - Total loop closures detected (from lc_count column transitions)
//!   - VO drift stats: mean and max sigma_xy, count of gate-out rows
//!   - Pose-graph correction magnitude at each loop event
//!   - RMSE between mekf_x/y and pg_x/pg_y trajectories

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).cloned().unwrap_or_default();

    if path.is_empty() {
        eprintln!("Usage: slam_eval <flight_log.csv>");
        eprintln!("  Expects a 49-column CSV written by main.rs (Phase 6+).");
        std::process::exit(1);
    }

    let content = match std::fs::read_to_string(&path) {
        Ok(c) => c,
        Err(e) => { eprintln!("Error reading {}: {}", path, e); std::process::exit(1); }
    };

    let mut lines = content.lines();
    let header = match lines.next() {
        Some(h) => h,
        None    => { eprintln!("Empty file"); std::process::exit(1); }
    };

    // Build column index map.
    let cols: Vec<&str> = header.split(',').collect();
    let col = |name: &str| -> Option<usize> {
        cols.iter().position(|&c| c == name)
    };

    let i_mekf_x    = col("mekf_x").unwrap_or(usize::MAX);
    let i_mekf_y    = col("mekf_y").unwrap_or(usize::MAX);
    let i_vo_sigma  = col("vo_sigma").unwrap_or(usize::MAX);
    let i_pg_x      = col("pg_x").unwrap_or(usize::MAX);
    let i_pg_y      = col("pg_y").unwrap_or(usize::MAX);
    let i_lc_count  = col("lc_count").unwrap_or(usize::MAX);

    // Check required columns are present.
    for (name, idx) in &[
        ("mekf_x", i_mekf_x), ("mekf_y", i_mekf_y),
        ("vo_sigma", i_vo_sigma),
        ("pg_x", i_pg_x), ("pg_y", i_pg_y), ("lc_count", i_lc_count),
    ] {
        if *idx == usize::MAX {
            eprintln!("Missing column '{}' — is this a Phase 6 CSV?", name);
            std::process::exit(1);
        }
    }

    let get_f32 = |fields: &[&str], i: usize| -> f32 {
        fields.get(i).and_then(|s| s.parse().ok()).unwrap_or(0.0)
    };
    let get_u32 = |fields: &[&str], i: usize| -> u32 {
        fields.get(i).and_then(|s| s.parse().ok()).unwrap_or(0)
    };

    // ── Per-row stats accumulators ────────────────────────────────────────────
    let mut n_rows         = 0usize;
    let mut sigma_sum      = 0.0f32;
    let mut sigma_max      = 0.0f32;
    let mut gate_out_count = 0usize;  // rows where vo_sigma >= 0.45 (90% of VO_MAX_SIGMA=0.5)
    let mut sq_err_x       = 0.0f32;
    let mut sq_err_y       = 0.0f32;
    let mut n_pg           = 0usize;  // rows where pg position is non-zero

    let mut prev_lc_count  = 0u32;
    let mut lc_events: Vec<(usize, f32, f32, f32, f32)> = Vec::new();
    // (row, mekf_x, mekf_y, pg_x, pg_y) at each loop closure event

    for line in lines {
        if line.trim().is_empty() { continue; }
        let fields: Vec<&str> = line.split(',').collect();
        n_rows += 1;

        let mekf_x   = get_f32(&fields, i_mekf_x);
        let mekf_y   = get_f32(&fields, i_mekf_y);
        let vo_sigma = get_f32(&fields, i_vo_sigma);
        let pg_x     = get_f32(&fields, i_pg_x);
        let pg_y     = get_f32(&fields, i_pg_y);
        let lc_count = get_u32(&fields, i_lc_count);

        // VO sigma stats.
        sigma_sum += vo_sigma;
        if vo_sigma > sigma_max { sigma_max = vo_sigma; }
        if vo_sigma >= 0.45 { gate_out_count += 1; }

        // Detect loop closure events (lc_count increased).
        if lc_count > prev_lc_count {
            lc_events.push((n_rows, mekf_x, mekf_y, pg_x, pg_y));
            prev_lc_count = lc_count;
        }

        // RMSE between MEKF and PG positions (only when PG has data).
        if pg_x != 0.0 || pg_y != 0.0 {
            sq_err_x += (mekf_x - pg_x).powi(2);
            sq_err_y += (mekf_y - pg_y).powi(2);
            n_pg += 1;
        }
    }

    // ── Print report ─────────────────────────────────────────────────────────
    println!("=== SLAM Evaluation: {} ===", path);
    println!("Rows parsed         : {}", n_rows);
    println!("Loop closures       : {}", lc_events.len());
    println!();

    println!("--- VO Drift Stats ---");
    if n_rows > 0 {
        println!("  mean sigma_xy     : {:.4} m", sigma_sum / n_rows as f32);
    }
    println!("  max  sigma_xy     : {:.4} m", sigma_max);
    println!("  gate-out rows     : {} ({:.1}%)",
        gate_out_count,
        100.0 * gate_out_count as f32 / n_rows.max(1) as f32);
    println!();

    println!("--- Pose-Graph vs MEKF ---");
    if n_pg > 0 {
        let rmse_x = (sq_err_x / n_pg as f32).sqrt();
        let rmse_y = (sq_err_y / n_pg as f32).sqrt();
        println!("  RMSE x            : {:.4} m  ({} rows with PG data)", rmse_x, n_pg);
        println!("  RMSE y            : {:.4} m", rmse_y);
    } else {
        println!("  No pose-graph data (no loop closures detected or AI Deck not enabled)");
    }
    println!();

    if !lc_events.is_empty() {
        println!("--- Loop Closure Events ---");
        println!("  {:>5}  {:>8}  {:>8}  {:>8}  {:>8}  {:>10}",
            "row", "mekf_x", "mekf_y", "pg_x", "pg_y", "correction");
        for &(row, mx, my, px, py) in &lc_events {
            let corr = ((mx - px).powi(2) + (my - py).powi(2)).sqrt();
            println!("  {:>5}  {:>8.4}  {:>8.4}  {:>8.4}  {:>8.4}  {:>10.4} m",
                row, mx, my, px, py, corr);
        }
    }
}
