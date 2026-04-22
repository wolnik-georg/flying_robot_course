# flight/ — Flight Layer Helpers

## What this module does

Bridges the control math and the CRTP radio interface for Mode A (position setpoints, RPYT).
Also provides utilities for building `MultirotorState` from raw sensor readings.

## state_builder — build_state

```rust
pub fn build_state(
    pos_m: [f32; 3],
    vel_ms: [f32; 3],
    roll_deg: f32,      // → converted internally to rad
    pitch_deg: f32,     // → converted internally to rad
    yaw_deg: f32,       // → converted internally to rad
    gyro_dps: [f32; 3], // deg/s → converted internally to rad/s
) -> MultirotorState
```

Constructs a `MultirotorState` from the raw values logged in the CSV (firmware EKF output).
Used in both `main.rs` (live flight) and `mekf_eval.rs` (offline replay).

## rpyt_control — RPYT command generation

The RPYT path is used in Mode A (`main.rs` `my_*` maneuvers) and the shadow controller.

```rust
pub struct RpytCmd {
    pub roll: f32,      // [deg] desired roll angle
    pub pitch: f32,     // [deg] desired pitch angle
    pub yaw_rate: f32,  // [deg/s] desired yaw rate
    pub thrust: u16,    // [0, 65535] PWM
}

// Step 1: compute desired force vector from PD position controller
pub fn compute_force_vector(
    state: &MultirotorState,
    reference: &TrajectoryReference,
    controller: &GeometricController,
) -> Vec3   // desired force direction in world frame

// Step 2: convert force vector → roll/pitch commands
pub fn force_vector_to_rpyt(f_vec: Vec3, yaw: f32, params: &MultirotorParams) -> (f32, f32)
// returns (roll_cmd_rad, pitch_cmd_rad)

// Step 3: thrust [N] → PWM
pub fn thrust_to_pwm(thrust_n: f32, params: &MultirotorParams) -> f32

// Yaw rate command
pub fn yaw_rate_cmd(yaw_error_rad: f32, yaw_rate_ref_rads: f32) -> f32  // → [deg/s]
```

## ekf_reset — EKF jump detection

The firmware Kalman EKF occasionally resets (detects large jumps). `main.rs` detects these
and resets the MEKF + controller integral accordingly.

```rust
pub fn detect_ekf_reset(prev: f32, curr: f32, threshold: f32) -> bool
// Returns true if |curr - prev| > threshold (handles yaw wrap separately)

pub fn yaw_wrap_delta(prev_yaw_deg: f32, curr_yaw_deg: f32) -> f32
// Shortest angular difference in [-180, 180] deg

pub fn deg_to_rad(deg: f32) -> f32   // π/180 * deg
pub fn rad_to_deg(rad: f32) -> f32   // 180/π * rad
```

## Typical usage in main.rs (Mode A shadow track)

```rust
// Build state from firmware EKF readings
let state = build_state(
    [pos_x, pos_y, pos_z],
    [vel_x, vel_y, vel_z],
    roll_deg, pitch_deg, yaw_deg,
    [gyro_x_dps, gyro_y_dps, gyro_z_dps],
);

// Shadow controller (position setpoint mode — controller computes, does NOT fly)
let ref_ = trajectory.get_reference(t);
let shadow_output = shadow_ctrl.compute_control(&state, &ref_, &params, dt);

// RPYT mode (my_hover, my_circle, my_figure8 — actually flies)
let f_vec = compute_force_vector(&state, &ref_, &my_ctrl);
let (roll_cmd, pitch_cmd) = force_vector_to_rpyt(f_vec, state.orientation.yaw(), &params);
let thrust_pwm = thrust_to_pwm(shadow_output.thrust, &params);
let yaw_rate = yaw_rate_cmd(yaw_error, ref_.yaw_rate);
cf.commander.setpoint_rpyt(roll_cmd.to_degrees(), pitch_cmd.to_degrees(), yaw_rate, thrust_pwm as u16).await?;
```

## Notes

- RPYT setpoints command the Crazyflie's **attitude stabilizer** (inner loop). The outer
  position loop is closed on the laptop.
- Position setpoints (`setpoint_position`) go to the Crazyflie's **position controller**
  (full outer loop on drone). Used in `run_firmware_mode()`.
- Full-state setpoints (`setpoint_full_state`) go to the onboard OOT SE(3) controller.
  Implemented in `vendor/crazyflie-lib/src/subsystems/commander.rs`.
