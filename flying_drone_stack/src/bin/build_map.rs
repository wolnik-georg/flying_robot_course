//! Offline 3D occupancy map builder.
//!
//! Replays one or more flight CSV logs (from `run_firmware_mode`) through the
//! OccupancyMap and exports an ASCII PLY point cloud for visualisation.
//!
//! ## Usage
//!
//! ```text
//! cargo run --release --bin build_map -- <flight.csv> [<flight2.csv> ...]
//!                                        [--out map.ply] [--full] [--min-height 0.1]
//! ```
//!
//! ## Options
//!
//! | Flag            | Default                    | Meaning                                 |
//! |-----------------|----------------------------|-----------------------------------------|
//! | `--out PATH`    | `results/data/map.ply`     | Output PLY file path                    |
//! | `--full`        | off                        | Export free+occupied voxels, not just occupied |
//! | `--min-height`  | `0.10` m                   | Skip rows where range_z < this (not airborne) |
//!
//! ## Visualisation
//!
//! Open the output in MeshLab or CloudCompare:
//! ```text
//! meshlab results/data/map.ply
//! ```
//! Colour by the `log_odds` scalar for confidence visualisation.
//!
//! ## Expected output (hover flight, indoor room)
//!
//! ```text
//! Loaded: runs/hover_2026-03-17_19-33-08.csv  (484 rows, 484 airborne)
//! Map stats: 1420 total voxels, 86 occupied, 1334 free
//! Saved: results/data/map.ply (86 points)
//! ```

use multirotor_simulator::mapping::OccupancyMap;
use multirotor_simulator::math::Vec3;
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();

    // Collect positional CSV paths and named flags.
    let mut csv_paths: Vec<String> = Vec::new();
    let mut out_path  = "results/data/map.ply".to_string();
    let mut full      = false;
    let mut min_h: f32 = 0.10;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--out" => {
                i += 1;
                out_path = args.get(i).ok_or("--out needs a path")?.clone();
            }
            "--full" => { full = true; }
            "--min-height" => {
                i += 1;
                min_h = args.get(i).ok_or("--min-height needs a value")?.parse()?;
            }
            a if a.starts_with("--") => {
                eprintln!("Unknown flag: {}", a);
                std::process::exit(1);
            }
            path => { csv_paths.push(path.to_string()); }
        }
        i += 1;
    }

    if csv_paths.is_empty() {
        eprintln!("Usage: build_map <flight.csv> [...] [--out map.ply] [--full] [--min-height 0.1]");
        std::process::exit(1);
    }

    let mut map = OccupancyMap::new();
    let mut total_rows = 0usize;
    let mut airborne_rows = 0usize;

    for path in &csv_paths {
        let (rows, airborne) = process_csv(&mut map, path, min_h)?;
        println!("Loaded: {}  ({} rows, {} airborne)", path, rows, airborne);
        total_rows    += rows;
        airborne_rows += airborne;
    }

    let stats = map.stats();
    println!("\nMap stats: {} total voxels, {} occupied, {} free",
             stats.n_total, stats.n_occupied, stats.n_free);
    println!("Total rows processed: {} ({} airborne)", total_rows, airborne_rows);

    fs::create_dir_all("results/data")?;
    let ply = if full { map.to_ply_full() } else { map.to_ply() };
    let n_pts = if full { stats.n_total } else { stats.n_occupied };
    fs::write(&out_path, &ply)?;
    println!("Saved: {} ({} points)", out_path, n_pts);
    println!("\nView with:  meshlab {}", out_path);
    println!("        or: cloudcompare {}", out_path);

    Ok(())
}

// ---------------------------------------------------------------------------
// CSV replay
// ---------------------------------------------------------------------------

/// Column indices extracted from the CSV header.
struct Cols {
    mekf_x:      usize,
    mekf_y:      usize,
    mekf_z:      usize,
    mekf_roll:   usize,
    mekf_pitch:  usize,
    mekf_yaw:    usize,
    range_z:     usize,
    multi_front: usize,
    multi_back:  usize,
    multi_left:  usize,
    multi_right: usize,
    multi_up:    usize,
}

impl Cols {
    fn from_header(header: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let fields: Vec<&str> = header.trim().split(',').collect();
        let idx = |name: &str| -> Result<usize, Box<dyn std::error::Error>> {
            fields.iter().position(|&f| f.trim() == name)
                  .ok_or_else(|| format!("column '{}' not found in CSV", name).into())
        };
        Ok(Cols {
            mekf_x:      idx("mekf_x")?,
            mekf_y:      idx("mekf_y")?,
            mekf_z:      idx("mekf_z")?,
            mekf_roll:   idx("mekf_roll")?,
            mekf_pitch:  idx("mekf_pitch")?,
            mekf_yaw:    idx("mekf_yaw")?,
            range_z:     idx("range_z")?,
            multi_front: idx("multi_front")?,
            multi_back:  idx("multi_back")?,
            multi_left:  idx("multi_left")?,
            multi_right: idx("multi_right")?,
            multi_up:    idx("multi_up")?,
        })
    }
}

/// Read `path`, feed every airborne row into `map`.
/// Returns (total_rows, airborne_rows).
fn process_csv(
    map:    &mut OccupancyMap,
    path:   &str,
    min_h:  f32,
) -> Result<(usize, usize), Box<dyn std::error::Error>> {
    let content  = fs::read_to_string(path)?;
    let mut lines = content.lines();

    let header = lines.next().ok_or("CSV is empty")?;
    let cols   = Cols::from_header(header)?;

    let mut n_total    = 0usize;
    let mut n_airborne = 0usize;

    for line in lines {
        if line.trim().is_empty() { continue; }
        n_total += 1;

        let fields: Vec<f32> = line.split(',')
            .map(|s| s.trim().parse::<f32>().unwrap_or(0.0))
            .collect();

        if fields.len() <= cols.multi_up { continue; }

        let range_z = fields[cols.range_z];
        if range_z < min_h { continue; } // not airborne
        n_airborne += 1;

        let pos = Vec3::new(fields[cols.mekf_x],
                            fields[cols.mekf_y],
                            fields[cols.mekf_z]);

        let roll_deg  = fields[cols.mekf_roll];
        let pitch_deg = fields[cols.mekf_pitch];
        let yaw_deg   = fields[cols.mekf_yaw];

        // Convert 0.0 sentinel (None from CRTP adapter) back to Option<f32>.
        let to_opt = |v: f32| if v > 0.02 { Some(v) } else { None };

        map.update(
            pos, roll_deg, pitch_deg, yaw_deg,
            to_opt(fields[cols.multi_front]),
            to_opt(fields[cols.multi_back]),
            to_opt(fields[cols.multi_left]),
            to_opt(fields[cols.multi_right]),
            to_opt(fields[cols.multi_up]),
            Some(range_z), // Flow Deck down sensor — always valid when airborne
        );
    }

    Ok((n_total, n_airborne))
}
