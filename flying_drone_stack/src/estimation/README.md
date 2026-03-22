# Estimation — Multiplicative Extended Kalman Filter (MEKF)

## 1. Overview

The MEKF fuses three sensor streams to continuously estimate the drone's position and orientation:

- **IMU** (accelerometer + gyroscope): high-rate (100 Hz+), noisy, drifting over time
- **ToF range sensor**: absolute height measurement, 100 Hz
- **PMW3901 optical flow**: lateral velocity measurement, <20 Hz effective rate
- **Visual odometry (VO)**: absolute XY position correction from camera keyframes, event-driven (Phase 5+)

The filter is a direct Rust port of the Crazyflie firmware's `kalman_core.c` (Mueller et al. 2015), extended with a Coriolis correction and an analytical Jacobian. The "multiplicative" in MEKF refers to how orientation errors are represented: as a small rotation applied *multiplicatively* to a reference quaternion, rather than additively. This avoids the singularities of Euler angles and keeps the quaternion on the unit sphere.

**References**:
- Mueller, M.W., Hehn, M., D'Andrea, R. (2015). Covariance correction step for Kalman filtering with an attitude. *JGCD* 40(9).
- Markley, F.L. (2003). Attitude error representations for Kalman filtering. *JGCD* 26(2).

---

## 2. Physical / Mathematical Foundation

### 2.1 State Vector

The MEKF maintains a 9-dimensional state plus an external reference quaternion:

```
x = [p₀, p₁, p₂,   b₀, b₁, b₂,   δ₀, δ₁, δ₂]  ∈ ℝ⁹
                          ↑               ↑
               body-frame velocity     attitude error angles

q_ref ∈ ℍ₁   (reference quaternion, maintained separately)
```

| Symbol | Meaning | Units | State index |
|--------|---------|-------|-------------|
| `p` | Position in world frame | m | 0–2 |
| `b` | Velocity in body frame | m/s | 3–5 |
| `δ` | Attitude error angles | rad | 6–8 |

The full attitude is `q = q_ref ⊗ q(δ)` where `q(δ) ≈ [1, δ/2]` for small `δ`.

After each predict-update cycle, `δ` is folded back into `q_ref` (`mekf_reset`), keeping `δ ≈ 0`. This is the "multiplicative" reset step.

### 2.2 Predict Step (IMU-Driven)

The process model advances the state by `dt` using gyroscope `ω` and accelerometer `a`:

```
p ← p + R(q_ref)·b·dt                         [position: integrate body vel to world]
b ← b + (a + R^T·[0,0,−g] − ω×b)·dt          [body vel: accel + gravity + Coriolis]
δ ← δ + ω·dt                                  [attitude: integrate gyro]
```

The `−ω×b` term (Coriolis) is the transport-theorem correction for the rotating body frame. Without it, the velocity estimate drifts during rotation. This matches `kalman_core.c` lines 480–482.

The covariance is propagated using the analytical Jacobian `G = ∂g/∂x`:

```
Σ ← G·Σ·Gᵀ + Q
```

`Q = diag(q_pos, q_pos, q_pos, q_vel, q_vel, q_vel, q_att, q_att, q_att)` (process noise).

### 2.3 Analytical Jacobian

The 9×9 Jacobian has a clean block structure (δ≈0 after reset, so `q_full ≈ q_ref`):

```
G[p,p] = I                   (position is self-propagating)
G[p,b] = R·dt                (position advances via world-frame velocity = R·b)
G[p,δ] = −R·[b]×·dt         (tilt changes how body velocity maps to world)
G[b,b] = I − [ω]×·dt        (Coriolis coupling between body velocity components)
G[b,δ] = [g_body]×·dt       (tilt changes gravity projection in body frame)
G[δ,δ] = I                   (attitude error propagates via gyro integration)
```

where `[v]×` denotes the 3×3 skew-symmetric matrix of `v`.

### 2.4 Height Update

The range sensor measures `p_z` directly:

```
z_meas = range_mm / 1000   [m]
h(x) = p_z = x[2]
H = [0, 0, 1, 0, 0, 0, 0, 0, 0]
innovation = z_meas − p_z
```

### 2.5 Flow Update

The PMW3901 measures integrated optical flow in pixels. The flow model relates body velocity to pixel displacement:

```
flow_x_pred = (dt · NP / (p_z · THETA_P)) · bx
flow_y_pred = (dt · NP / (p_z · THETA_P)) · by
```

The constants `NP = 350 px` and `THETA_P = 3.50 rad` are calibrated (see Parameters section). The Jacobian rows include a `∂/∂p_z` term to account for height affecting the flow scale.

Two separate scalar updates are performed (one for x, one for y), using the ToF height directly rather than the estimated `p_z` — this breaks the feedback loop where height error corrupts velocity estimate.

### 2.6 VO Position Update

When a new visual odometry estimate `[vo_x, vo_y]` is available (from `VoTrajectory` or a pose-graph correction), two scalar EKF position updates are performed:

```
H_x = [1, 0, 0, 0, 0, 0, 0, 0, 0]   (observes p_x)
H_y = [0, 1, 0, 0, 0, 0, 0, 0, 0]   (observes p_y)

innovation_x = vo_x − x[0]
innovation_y = vo_y − x[1]

Chi² gate: if innovation_i² > 9 · S_i, skip that axis update
```

The gate threshold `9·S` corresponds to 3-sigma — it rejects VO estimates that are more than 3 standard deviations away from the current position belief. This prevents a bad VO frame or a large accumulated drift jump from corrupting the filter.

The `r_vo_xy` noise parameter is passed by the caller:
- **VO update**: `R_VO = 0.10 m²` — trusts VO loosely (camera pose estimates have ~0.1 m accuracy)
- **Pose-graph correction**: `R_LOOP = 0.01 m²` — trusts loop closure more (global constraint, sub-10 cm accuracy expected)

### 2.7 Kalman Gain and Joseph Form Update

For each scalar measurement:

```
S = H·Σ·Hᵀ + R_sensor     (innovation covariance)
K = Σ·Hᵀ / S               (Kalman gain, 9×1)
x ← x + K·innovation
Σ ← (I − KH)·Σ·(I − KH)ᵀ + K·R·Kᵀ   (Joseph form)
```

The Joseph form `(I−KH)Σ(I−KH)ᵀ + KRKᵀ` is algebraically equivalent to the simpler `(I−KH)Σ` but remains positive-definite even with f32 rounding errors. This is essential — without it the covariance can go negative after a few hundred steps.

---

## 3. Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│  Sensor inputs                                                               │
│  gyro [deg/s] + accel [g]  |  range [mm]  |  flow_x, flow_y [px]           │
│                                               VO position [x,y m] (event)   │
└───────────────────┬─────────────────────────────────────┬────────────────────┘
                    │                                     │
                    ▼                                     ▼
┌───────────────────────────┐      ┌──────────────────────────────────────────┐
│  mekf_reset               │      │   mekf_update_height                     │
│  δ → q_ref (fold error)   │      │   H = [0,0,1,0,...0]                     │
│  δ ← 0                   │      │   z_meas = range_mm / 1000 m             │
└─────────────┬─────────────┘      └──────────────────────────────────────────┘
              │                                           │
              ▼                                           ▼
┌───────────────────────────┐      ┌──────────────────────────────────────────┐
│  mekf_predict             │      │   mekf_update_flow                       │
│  p ← p + R·b·dt           │      │   scale = dt·NP/(pz·THETA_P)            │
│  b ← b + (a+g_body−ω×b)dt│      │   zero-motion gate: skip if             │
│  δ ← δ + ω·dt             │      │   |dnx|<0.3 AND |dny|<0.3               │
│  Σ ← GΣGᵀ + Q             │      │   two scalar EKF updates                 │
└─────────────┬─────────────┘      └──────────────────────────────────────────┘
              │                                           │
              │                    ┌──────────────────────────────────────────┐
              │                    │   mekf_update_vo (event-driven)          │
              │                    │   H_x = [1,0,0,0,...0]                   │
              │                    │   H_y = [0,1,0,0,...0]                   │
              │                    │   z = [vo_x, vo_y], R = r_vo_xy·I       │
              │                    │   chi² gate: skip if innovation² > 9·S   │
              │                    │   two scalar EKF updates (x then y)      │
              └──────────────────┬─┘──────────────────────────────────────────┘
                                 ▼
                    ┌────────────────────────┐
                    │  MekfState output      │
                    │  q_ref → euler angles  │
                    │  x[0..2] → position   │
                    └────────────────────────┘
```

---

## 4. Key Data Types

### `MekfState` — `src/estimation/mekf.rs:190`

| Field | Type | Description |
|-------|------|-------------|
| `q_ref` | `[f32; 4]` | Reference quaternion `[w,x,y,z]` |
| `x` | `[f32; 9]` | State vector `[p(3), b(3), δ(3)]` |
| `sigma` | `Mat9` | 9×9 covariance matrix |

### `MekfParams` — `src/estimation/mekf.rs:138`

| Field | Default | Units | Meaning |
|-------|---------|-------|---------|
| `q_pos` | 1e-7 | m²/step | Process noise — position (very small: position is nearly a random walk driven by velocity) |
| `q_vel` | 1e-3 | m²/s²/step | Process noise — body velocity (accelerometer bias + vibration) |
| `q_att` | 1e-6 | rad²/step | Process noise — attitude error (gyro integration drift) |
| `r_height` | 1e-3 | m² | Height measurement noise (ToF is accurate) |
| `r_flow` | 8.0 | px² | Flow measurement noise (PMW3901 is noisy) |
| `zero_flow_threshold` | 0.3 | px | Gate: skip flow update if both axes below this |
| *(r_vo_xy)* | caller-supplied | m² | VO position noise; `main.rs` uses 0.10 m² (VO), 0.01 m² (pose-graph corrections) |

### `Mekf` — `src/estimation/mekf.rs:499`

High-level wrapper around `MekfState`. Call `mekf.feed_row(t, gyro, accel, range_mm, flow_x, flow_y)` once per CSV row. Returns `Option<[roll, pitch, yaw, px, py, pz]>` on each predict step.

---

## 5. Algorithm Walkthrough

**Per-step flow** in `Mekf::feed_row` (`src/estimation/mekf.rs:533`):

1. **Accumulate IMU** (lines 543–548): Buffer incoming gyro and accel readings.

2. **Predict** (lines 553–568): When a fresh gyro+accel pair arrives:
   - Convert gyro from deg/s to rad/s, accel from g to m/s²
   - Call `mekf_reset` → fold `δ` into `q_ref`
   - Call `mekf_predict` → advance state and covariance

3. **Height update** (lines 572–574): If range_mm is valid, call `mekf_update_height`.

4. **Flow update** (lines 578–598): If both flow components are valid:
   - Apply zero-motion gate: skip if `|dnx| < 0.3 AND |dny| < 0.3`
   - Call `mekf_update_flow` with ToF height as the scale denominator

**VO update — `mekf_update_vo`** (`src/estimation/mekf.rs`, called from `main.rs`):

This is an **event-driven** update, not called every step. `main.rs` calls it whenever a new VO position estimate is available (from `VoTrajectory`) or when the pose graph applies a loop-closure correction:

```
H_x = [1, 0, 0, 0, 0, 0, 0, 0, 0]   (measures p_x directly)
H_y = [0, 1, 0, 0, 0, 0, 0, 0, 0]   (measures p_y directly)
```

Two scalar EKF updates correct the XY position. A **chi² gate** (innovation² > 9·S) rejects large outliers — this prevents a bad VO estimate from jerking the filter. The noise parameter `r_vo_xy` is caller-supplied:
- `main.rs` passes `R_VO = 0.10` m² for normal VO updates
- `main.rs` passes `R_LOOP = 0.01` m² for pose-graph-corrected positions (higher confidence)

**Inside `mekf_predict`** (`src/estimation/mekf.rs:226`):
```
compute_jacobian → G (9×9 analytical Jacobian)
process_model → x_new (propagate mean)
sigma = G * sigma * G^T + Q (propagate covariance)
sigma.symmetrise()          (force (Σ+Σᵀ)/2 — kills f32 asymmetry)
sigma.clamp_diagonal(100.0) (cap variance — mirrors firmware MAX_COVARIANCE)
```

**Inside `scalar_update`** (`src/estimation/mekf.rs:424`):
```
S = H * sigma * H^T + r_sensor   (innovation variance)
K = sigma * H^T / S               (Kalman gain vector)
x += K * (z_meas - h_pred)       (state update)
sigma = joseph_update(sigma, K, H, r_sensor)  (covariance)
sigma.symmetrise()
```

---

## 6. Parameters & Tuning

### Constants baked into the code

| Constant | Value | Why |
|----------|-------|-----|
| `NP` | 350 px | PMW3901 nominal pixel count (CF firmware convention) |
| `THETA_P` | **3.50 rad** | Effective flow calibration constant (see below) |
| `MAX_COVARIANCE` | 100.0 | Mirrors firmware `kalman_core.c` |

### THETA_P Calibration

The original prototype used `THETA_P = 0.717 rad` (from `mekf_offline.py`). When replayed against real March 15 circle flights, the MEKF XY position span was 4.9× smaller than the firmware EKF — the filter was barely seeing any motion.

An offline amplitude-ratio sweep (measuring `max(pos_ekf_x) / max(mekf_x)` across dynamic logs) found the ratio was consistently ≈ 4.88. Correcting: `THETA_P_new = 0.717 × 4.88 ≈ 3.50`.

The 4.9× discrepancy likely comes from the firmware linearising the flow model differently (body frame vs sensor frame axes, scaling convention). `THETA_P = 3.50` is **not** a physical per-pixel angle; it is an empirical calibration constant for the PMW3901 as driven by the CF firmware.

### Zero-Motion Gate (`zero_flow_threshold = 0.3 px`)

The PMW3901 fires at <20 Hz. The CF firmware logs at 100 Hz and zero-pads missing samples. About **46% of airborne flow readings are exact zeros** in the March 15 logs.

Without the gate, these zeros are treated as "I measured zero velocity", which constantly pulls the velocity estimate toward zero — even during active motion. The gate skips any update where both axes are below 0.3 px. This value is above the sensor quantisation noise (~0.1 px) but well below real motion signals at 0.3m height (~1–5 px).

### Process Noise Intuition

`q_pos = 1e-7` is tiny because position is not directly driven by process noise — it follows velocity. `q_vel = 1e-3` is larger because the accelerometer is noisy and has a variable bias. `q_att = 1e-6` is between the two because gyro drift is slow but real.

Increasing `q_vel` → filter trusts measurements more → faster but noisier velocity tracking.
Decreasing `r_flow` → filter trusts flow measurements more → tighter position estimates but vulnerable to sensor noise.

---

## 7. Connections to Other Modules

| Direction | Module | What is exchanged |
|-----------|--------|------------------|
| Consumes | `flight/` | Raw sensor rows: `gyro [deg/s], accel [g], range [mm], flow [px]` |
| Consumes | `mapping/vo_trajectory.rs` | VO XY position estimate `[vo_x, vo_y]` (m) + noise `r_vo_xy = 0.10 m²` |
| Consumes | `mapping/loop_closure.rs` via `main.rs` | Pose-graph corrected XY `[pg_x, pg_y]` (m) + tighter noise `r_loop = 0.01 m²` |
| Produces | `flight/` | Attitude (roll/pitch/yaw) and position (x,y,z) estimates |
| Shadowed by | `main.rs` | MEKF runs in parallel but its output is **not** sent to the drone; firmware EKF controls |
| Validated by | `mekf_eval` binary | Offline replay against recorded flight CSVs |

---

## 8. Common Pitfalls

**Do not feed ground rows before takeoff**: The MEKF loop in `mekf_eval` must start from `seed_idx` — the first row where `range_z > 0.1 m`. Feeding ground rows (where accelerometer reads static +1g and range reads 0) corrupts the attitude estimate. The filter was designed for airborne dynamics.

**Zero-padding bias**: If you remove the zero-motion gate, you will see velocity estimates rail toward zero during hover. This is not a filter divergence but a bias introduced by treating 0.0 px readings as real measurements.

**Stuck EKF attitude after crash**: After a hard crash, the firmware EKF attitude can freeze at wrong values (e.g., −22°/+16°) because `kalman.resetEstimation=1→0` alone is insufficient if the gyro bias is corrupted. The fix described in `MEMORY.md` is to momentarily switch to the complementary filter (estimator=1→2) to re-initialise attitude from accel, then switch back.

**Height feedback loop**: The flow model depends on `p_z`. If you use `state.x[2]` (estimated height) as the denominator rather than the actual ToF reading, a small height error causes a wrong velocity scale, which corrupts velocity, which corrupts height. The code uses `pz_meas` (ToF directly) when available.

**THETA_P is not transferable**: The `THETA_P = 3.50` value is specific to the CF 2.1 + Flow Deck v2 + the CF firmware's flow logging convention. Different hardware or firmware versions will need a fresh calibration.

---

## 9. Related Tests

| Test file | What it covers |
|-----------|---------------|
| `tests/test_mekf.rs` | 20 tests: predict/update/flow gate behaviour, numerical stability, Coriolis correction, RMSE bounds on real flight data |
| `src/estimation/mekf.rs` (inline) | Jacobian analytical vs numerical, Coriolis term, hover velocity stability, quaternion rotation |
| `src/bin/mekf_eval.rs` | Integration test: offline replay against circle/figure-8 CSVs, prints RMSE vs firmware EKF |

### Validation Results (March 2026 flights)

| Manoeuvre | Roll RMSE | Pitch RMSE | Yaw RMSE | X RMSE | Y RMSE | Z RMSE |
|-----------|-----------|------------|----------|--------|--------|--------|
| Hover | 0.9° | 2.1° | 1.5° | 5.3 cm | 5.6 cm | 0.44 cm |
| Circle | 0.8° | 3.1° | 1.4° | 13.1 cm | 15.3 cm | 0.39 cm |
| Figure-8 | 0.9° | 1.6° | 4.2° | 15.3 cm | 7.9 cm | 0.26 cm |
