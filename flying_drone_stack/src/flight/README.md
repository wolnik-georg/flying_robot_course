# Flight — Hardware Bridge

## 1. Overview

The `flight/` module is the translation layer between the geometric controller (which thinks in SI units, rotation matrices, and force vectors) and the Crazyflie firmware (which accepts roll/pitch angles in degrees, yaw rate in deg/s, and thrust as a 16-bit integer).

It is pure mathematics — no networking, no async, no hardware calls. Every function is testable in isolation and is unit-tested. The three sub-modules are:

| Sub-module | Purpose |
|------------|---------|
| `state_builder.rs` | Convert raw EKF log scalars → `MultirotorState` |
| `rpyt_control.rs` | Convert geometric controller output → RPYT setpoint for firmware |
| `ekf_reset.rs` | Detect EKF position/yaw jumps; yaw angle unwrapping |

Understanding this layer is essential for anyone modifying the real hardware flight code — the coordinate conventions and sign rules here are the most common source of subtle bugs.

---

## 2. The Frame Discipline Rule

**Critical**: There are two coordinate systems in flight:

| Frame | Who owns it | Position reference |
|-------|-------------|-------------------|
| **Firmware EKF frame** | Crazyflie firmware | EKF position at power-on, drifts with Lighthouse |
| **MEKF frame** | Our Rust estimator | Seeded from first airborne EKF row |

The `setpoint_position` command the drone receives is interpreted in the **firmware EKF frame**. The geometric controller computes errors in whatever frame its state input lives in. If you build state from MEKF output and send setpoints, the origin is ~50 cm off — the drone will saturate trying to chase a phantom position.

**The rule**: `build_state` must always use `pos_x/y/z` from the **firmware EKF** log fields, not `mekf_x/y/z`. MEKF output is logged and evaluated but never fed into the control loop.

---

## 3. `build_state` — EKF Scalars → `MultirotorState`

**`src/flight/state_builder.rs:30`**

Converts the raw numbers in each firmware log row into the `MultirotorState` the controller expects.

### Inputs

| Log field | Units | Destination |
|-----------|-------|-------------|
| `pos_x, pos_y, pos_z` | m | `state.position` |
| `vel_x, vel_y, vel_z` | m/s | `state.velocity` |
| `roll, pitch, yaw` | **degrees** (firmware convention) | → quaternion `state.orientation` |
| `gyro_x, gyro_y, gyro_z` | **deg/s** | → rad/s `state.angular_velocity` |

### Quaternion Construction

The firmware uses ZYX Tait-Bryan Euler angles. The quaternion is constructed as:

```
q = q_yaw ⊗ q_pitch ⊗ q_roll
  = Quat::from_axis_angle(ẑ, yaw) ⊗ Quat::from_axis_angle(ŷ, pitch) ⊗ Quat::from_axis_angle(x̂, roll)
```

**Order matters**: yaw is applied first (in the world frame), then pitch (in the yaw-rotated frame), then roll (in the pitch-rotated frame). This matches ZYX convention and the firmware's own EKF output.

The angular velocity is simply the gyro reading converted from deg/s to rad/s — no frame rotation needed because the firmware already reports body-frame rates.

### Why This Function Exists

All three original flight loops (`my_hover`, `my_circle`, `my_figure8`) contained an identical 12-line quaternion-construction block. Extracting it:
1. Makes it unit-testable against known angles
2. Eliminates six opportunities for a copy-paste sign error
3. Keeps control loops focused on control policy

---

## 4. `force_vector_to_rpyt` — Force Vector → Roll/Pitch Commands

**`src/flight/rpyt_control.rs:105`**

This is the most nuanced function in the flight module. It converts the world-frame desired force vector `F_d` from the position controller into roll and pitch angles the firmware can execute.

### Geometric Derivation

The quadrotor can only produce thrust along its body z-axis. To accelerate in the `+X` world direction, the drone must tilt nose-down (negative pitch) so that the thrust vector has a `+X` component:

```
pitch_d_raw = atan2(F_x, F_z)     [rad]
roll_d_raw  = atan2(−F_y, F_z)    [rad]
```

- `F_x > 0` → `pitch_d_raw > 0` → nose tilts toward `+X` (forward)
- `F_y > 0` → `roll_d_raw < 0` → left side tilts up → thrust toward `+Y`

### The Firmware Sign Convention (Critical)

The Crazyflie RPYT commander (`controller_lee.c`) internally computes:

```c
q_att = rpy2quat(roll, -pitch_setpoint, yaw)
```

It **negates** the pitch setpoint. This means:

```
We send:       pitch_d_cmd = +pitch_d_raw   (un-negated)
Firmware does: actual_pitch = −pitch_d_raw
Result:        body z tilts toward +X      ✓
```

If we negated pitch before sending, the firmware double-negates and the drone accelerates *backward*. The code passes pitch un-negated (`pitch_d_cmd = pitch_d_raw`).

Full sign chain for a drone that needs to move in `+X`:
```
ep.x > 0  → F_d.x > 0  → pitch_d_raw > 0  → send +pitch_d_raw
firmware negates: actual_pitch < 0  → nose-down → thrust tilts toward +world_x  ✓
```

Roll has no sign flip: the firmware uses `roll` directly in `rpy2quat`.

### Tilt Clamping

Both roll and pitch are clamped to `±max_tilt_deg` (typically 20–25°). This prevents the drone from attempting dangerous attitudes in response to large position errors. When clamping occurs, `tilt_saturated()` returns true, which triggers the anti-windup rollback in `main.rs`.

---

## 5. `thrust_to_pwm` — Thrust Newtons → PWM Integer

**`src/flight/rpyt_control.rs:142`**

The Crazyflie `setpoint_rpyt` commander expects thrust as a `u16` in `[0, 65535]`. The mapping is linear through the hover point:

```
pwm = (thrust_N / hover_thrust_N) × hover_pwm
```

where `hover_thrust_N = mass × g = 0.027 × 9.81 ≈ 0.265 N` and `hover_pwm` is calibrated empirically (typically around 36000–38000 for a Crazyflie with fresh props).

The result is clamped to `[pwm_min, pwm_max]` (default: 10 000 to 60 000). The raw (unclamped) value is returned alongside the clamped `u16` so the caller can detect saturation.

**Why not use the full `[0, 65535]` range?** Below ~10 000 the motors stall; above ~60 000 the motors are at near-maximum RPM and there is no headroom for attitude corrections.

---

## 6. `yaw_rate_cmd` — Yaw Reference → Yaw Rate Command

**`src/flight/rpyt_control.rs:166`**

```
yaw_rate_cmd = kp_yaw × (ref_yaw_deg − current_yaw_deg)
               clamped to ±max_yaw_rate_deg_s
```

This is a simple proportional yaw controller. The RPYT commander accepts yaw *rate* (not angle) so we must differentiate the angle error. A `kp_yaw` around 1–2 gives yaw bandwidth of ~1 rad/s, sufficient for slow heading changes.

The yaw reference is typically constant (facing forward) for hover and circle manoeuvres, and slowly changing for figure-8.

---

## 7. `detect_ekf_reset` — EKF Jump Detection

**`src/flight/ekf_reset.rs:61`**

The Crazyflie EKF can "teleport" its position or yaw estimate when the Lighthouse positioning system re-initialises. This looks like an impossibly large velocity in one log tick.

The detector compares consecutive ticks:

```
step_xy  = ‖pos_curr − pos_prev‖_xy
step_z   = |pos_curr.z − pos_prev.z|
step_yaw = |yaw_wrap_delta(prev_yaw, curr_yaw)|

flags.xy  = step_xy  > xy_thresh   (default 0.05 m = 5 m/s equivalent)
flags.z   = step_z   > z_thresh    (default 0.05 m)
flags.yaw = step_yaw > yaw_thresh  (default 10°  = 1000 deg/s equivalent)
```

When a jump is detected, `main.rs` re-anchors the trajectory origin to the new EKF position and resets the position integral to prevent the windup from the phantom error.

### `yaw_wrap_delta`

The yaw angle wraps at ±180°. Naively computing `curr_yaw − prev_yaw` can give ±360° for a small crossing. `yaw_wrap_delta` returns the shortest-path angular difference in `(−180°, +180°]`:

```
yaw_wrap_delta(170°, −170°) = +20°   (20° CCW, not 340° CW)
```

---

## 8. Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────┐
│  main.rs per-tick loop                                               │
│                                                                      │
│  firmware log row → build_state()                                    │
│     ↓ MultirotorState (in firmware EKF frame)                        │
│  trajectory.get_reference(t)                                         │
│     ↓ TrajectoryReference                                             │
│  controller.compute_control(state, ref)                              │
│     ↓ ControlOutput {thrust, torque}                                 │
│  compute_force_vector(ref, ep, ev, i_pos, controller, params)        │
│     ↓ Vec3 (world-frame force)                                        │
│  force_vector_to_rpyt(f_vec, max_tilt_deg)                           │
│     ↓ (roll_deg, pitch_deg, roll_raw, pitch_raw)                     │
│  thrust_to_pwm(thrust, hover_thrust, hover_pwm, ...)                 │
│     ↓ (thrust_pwm: u16, raw: f32)                                    │
│  yaw_rate_cmd(ref_yaw, current_yaw, kp_yaw, max_rate)               │
│     ↓ yaw_rate_deg_s: f32                                            │
│                                                                      │
│  cf.commander.setpoint_rpyt(roll, pitch, yaw_rate, thrust_pwm)      │
│                                                                      │
│  detect_ekf_reset(prev_pos, cur_pos, ...)                            │
│     ↓ EkfResetFlags → re-anchor trajectory if any flag set          │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 9. Key Data Types

### `RpytCmd` — `src/flight/rpyt_control.rs:36`

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `roll_deg` | `f32` | degrees | Roll angle, positive = right side down |
| `pitch_deg` | `f32` | degrees | Pitch after firmware sign correction (un-negated from `pitch_d_raw`) |
| `yaw_rate_deg_s` | `f32` | deg/s | Yaw rate command |
| `thrust_pwm` | `u16` | [0, 65535] | Thrust mapped to PWM, clamped to [10000, 60000] |

### `EkfResetFlags` — `src/flight/ekf_reset.rs:22`

| Field | Type | Description |
|-------|------|-------------|
| `xy` | `bool` | XY position jump detected |
| `z` | `bool` | Z position jump detected |
| `yaw` | `bool` | Yaw angle jump detected |
| `step_xy_m` | `f32` | Measured XY step size [m] |
| `step_z_m` | `f32` | Measured Z step size [m] |
| `step_yaw_deg` | `f32` | Measured yaw step size [deg] |

`flags.any()` returns `true` if any axis triggered.

---

## 10. Connections to Other Modules

| Direction | Module | What is exchanged |
|-----------|--------|------------------|
| Consumes | `dynamics/` | `MultirotorState` (produced by `build_state`) |
| Consumes | `controller/` | `GeometricController`, `TrajectoryReference`, `ControlOutput` |
| Consumes | `math/` | `Vec3`, `Quat` for quaternion construction |
| Consumed by | `bin/main.rs` | All flight loop math lives here; `main.rs` is pure orchestration |

---

## 11. Common Pitfalls

**Frame mixing (most common hardware bug)**: Using MEKF position fields (`mekf_x/y/z`) in `build_state` instead of firmware EKF fields (`pos_x/y/z`) shifts the controller's reference frame by ~50 cm. The drone will constantly try to reach a phantom position and will either saturate tilt or crash. Always use firmware EKF fields for state that is passed to the controller.

**Pitch sign double-negation**: If you negate pitch before sending to the firmware (thinking "negative pitch = nose down"), the firmware negates it again and the drone flies backward. The code sends `pitch_d_raw` un-negated. The firmware's sign convention is documented in the `rpyt_control.rs` module-level comment.

**`hover_pwm` calibration**: The `thrust_to_pwm` linear mapping assumes a specific `hover_pwm` constant. If your Crazyflie is heavier than stock (e.g., with a battery protector or extra sensors), the hover PWM is higher and you must re-calibrate. A mismatch means the drone over- or under-thrusts at the hover setpoint.

**EKF jump reaction**: When `detect_ekf_reset` fires, the position integral must be reset immediately — otherwise the accumulated integral from the pre-jump position drives the drone toward the old location. `main.rs` calls `controller.reset_position_integral()` on any `flags.any()`.

**Gyro units**: The firmware logs `gyro.x/y/z` in **deg/s**. `build_state` converts to rad/s before storing in `MultirotorState.angular_velocity`. If you read gyro from a different source that already gives rad/s, do not apply the conversion again.

---

## 12. Related Tests

| Test file | What it covers |
|-----------|---------------|
| `tests/test_flight_math.rs` | `build_state` quaternion correctness (known angles → known rotation matrix); `force_vector_to_rpyt` sign checks (F.x>0 → positive pitch_d_raw); `yaw_wrap_delta` crossing ±180°; `thrust_to_pwm` hover maps to `hover_pwm` |
| `src/flight/state_builder.rs` (inline) | `test_quaternion_reconstruction_from_rpy`: positive pitch → `body_z` tilts toward `+world_x` (full sign chain verification) |
| `src/flight/rpyt_control.rs` (inline) | Roll/pitch/yaw torque directions, thrust PWM clamping |
| `src/flight/ekf_reset.rs` (inline) | Plausible motion → no flags; jump > threshold → flags set; `yaw_wrap_delta` wrap cases |
