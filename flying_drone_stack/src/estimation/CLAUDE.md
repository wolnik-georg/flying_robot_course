# estimation/ — Multiplicative Extended Kalman Filter (MEKF)

## What this module does

Full MEKF following Mueller et al. (2015) for attitude + position estimation on the Crazyflie.
Fuses: gyro predict → ToF height update → optical flow XY update → visual odometry XY update.

The MEKF runs live in `src/bin/main.rs` every loop and its output is logged alongside the
firmware EKF in the 49-column CSV (`mekf_roll/pitch/yaw/x/y/z` vs `roll/pitch/yaw/pos_x/y/z`).

## Key types

```rust
pub struct MekfState {
    pub q_ref: [f32; 4],   // reference quaternion [w, x, y, z]
    pub x: [f32; 9],       // error state: [dp(3), dv(3), dθ(3)] — position, velocity, attitude error
    pub sigma: Mat9,        // 9×9 error covariance
}

pub struct MekfParams {
    pub q_pos: f32,               // process noise: position
    pub q_vel: f32,               // process noise: velocity
    pub q_att: f32,               // process noise: attitude
    pub r_height: f32,            // measurement noise: height [m²]
    pub r_flow: f32,              // measurement noise: optical flow [px²]
    pub zero_flow_threshold: f32, // skip flow update if |flow| < this [px]
    pub theta_p: f32,             // flow calibration constant [rad]
}

impl Default for MekfParams // → optimal grid-search values
```

## Key functions

```rust
// Initialise
let mut state = MekfState::new(q_ref: [1.0, 0.0, 0.0, 0.0], &params);

// --- Called every loop (20 Hz in main.rs) ---

// 1. Prediction step (always)
mekf_predict(
    &mut state,
    gyro_rads: [gx, gy, gz],    // body-frame [rad/s] — ALREADY in rad/s
    accel_ms2: [ax, ay, az],    // body-frame [m/s²] — already converted from g
    dt: f32,
    r_proc: &Mat9,              // process noise matrix (built from MekfParams)
);

// 2. Height update (when range_z available)
mekf_update_height(&mut state, pz_meas: f32, r_height: f32);

// 3. Flow update (when flow non-zero)
mekf_update_flow(
    &mut state,
    flow: Option<[f32; 2]>,    // [flow_x, flow_y] [px] — None to skip
    pz: Option<f32>,           // current height [m] — None to skip
    theta_p: f32,              // calibration: 3.50 (flights) or 0.717 (fr00.csv)
    r_flow: f32,
    zero_flow_threshold: f32,
);

// 4. VO update (when new keyframe available)
mekf_update_vo(&mut state, vo_xy: [f32; 2], r_vo_xy: f32);

// 5. Reset (fold attitude error back into q_ref)
mekf_reset(&mut state);

// --- Utility ---
pub fn quat_to_euler(q: [f32; 4]) -> [f32; 3]   // → [roll, pitch, yaw] [rad]
pub fn quat_to_rot(q: [f32; 4]) -> [[f32; 3]; 3]
```

## Critical calibration values

| Parameter | March 2026 flights | Course dataset (fr00.csv) |
|-----------|-------------------|--------------------------|
| `theta_p` | **3.50** | **0.717** |
| `zero_flow_threshold` | 0.3 px | 0.3 px |

**Wrong theta_p causes huge XY drift.** Always check which dataset you're working with.

## Sensor axis conventions

```
PMW3901 optical flow → MEKF body velocity:
  vel_body_x = -flow_dy   (NOT flow_dx!)
  vel_body_y = -flow_dx

Accelerometer: raw in g, convert before predict:
  accel_ms2 = [acc_x_g, acc_y_g, acc_z_g].map(|a| a * 9.81)

Gyro: raw in deg/s in CSV, convert before predict:
  gyro_rads = [gx_dps, gy_dps, gz_dps].map(|g| g * π/180)
```

## Sign convention

`mekf_pitch` has the **opposite sign** to `stabilizer.pitch` from firmware. This is a
display-only issue — negate MEKF pitch when overlaying on firmware pitch in plots.
See `plot_assignment3_flight.py` line 65 for the fix.

## RMSE validation results (Mar 17 2026 flights)

| Maneuver | Roll | Pitch | Yaw | x | y | z |
|----------|------|-------|-----|---|---|---|
| Hover | 1.12° | 0.95° | 1.02° | 3.4 cm | 3.1 cm | 1.1 cm |
| Circle | 0.66° | 0.58° | 3.27° | 5.1 cm | 7.1 cm | 0.5 cm |
| Figure-8 | 0.94° | 0.61° | 1.53° | 5.6 cm | 3.6 cm | 0.8 cm |

Position RMSE measures drift between *two drifting estimates* (flow dead-reckoning) — not
absolute accuracy. Attitude RMSE < 1° confirms filter matches firmware EKF.

## Offline replay

```bash
cargo run --release --bin mekf_eval -- runs/<file>.csv
# prints per-column RMSE vs firmware EKF
# IMPORTANT: loop must start from seed_idx (first row where range_z > 0.1), NOT row 0
```

## Process noise matrix construction

```rust
// In main.rs, built once:
let mut r_proc = Mat9::zeros();
r_proc.data[0][0] = params.q_pos;
r_proc.data[1][1] = params.q_pos;
r_proc.data[2][2] = params.q_pos;
r_proc.data[3][3] = params.q_vel;
r_proc.data[4][4] = params.q_vel;
r_proc.data[5][5] = params.q_vel;
r_proc.data[6][6] = params.q_att;
r_proc.data[7][7] = params.q_att;
r_proc.data[8][8] = params.q_att;
```
