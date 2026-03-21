# Perception Module

## 1. Overview

The perception module provides a hardware-abstracted sensor layer that sits between raw sensor drivers and future SLAM / RL algorithms.  It exposes clean Rust traits regardless of whether the data source is a simulation or real hardware, making sim-to-real transfer straightforward.

Hardware abstracted:
- **Flow Deck v2**: PMW3901 optical flow (downward) + VL53L1x ToF range (downward), already fused by the MEKF.
- **Multi-ranger Deck**: five additional VL53L1x sensors facing front, back, left, right, up — gives 6-axis laser ranging when combined with the Flow Deck down sensor.
- **AI Deck**: GAP8 SoC + HiMax HM01B0 grayscale camera (320×320), Nina W102 WiFi via CPX protocol.

The module has three layers:
1. **`traits/`** — `SensorSource<M>` and `ImageSource` abstractions (no hardware knowledge).
2. **`sensors/`** — concrete implementations: `sim` (driven from `MultirotorState`), `crtp` (unit-conversion adapters for CRTP log variables), `cpx` (async AI Deck WiFi receiver).
3. **`processing/`** — pure-math algorithms: FAST-9 feature detection, BRIEF-256 descriptors, pinhole calibration, IMU–camera temporal alignment.

The 6-axis range coverage is the key addition over what the MEKF currently uses.  It enables wall/obstacle proximity in all directions (foundation for collision avoidance) and simple 2D/3D occupancy mapping.

**Downstream consumers**: `mapping/` uses `Vec<Feature>` for keyframe-based visual odometry and `MultiRangeMeasurement` for the occupancy map.  SLAM, dense depth maps, loop closure, and RL observation spaces are out of scope for this module.

---

## 2. Mathematical / Physical Foundation

### 2.1 Optical Flow Model

The PMW3901 measures integrated optical flow in pixels.  Body-frame velocity maps to pixel displacement via:

```
dx_px = (v_body_x / h_m) × (dt × NP / THETA_P)  +  N(0, σ)
dy_px = (v_body_y / h_m) × (dt × NP / THETA_P)  +  N(0, σ)
```

where `NP = 350 px` (nominal pixel count) and `THETA_P = 3.50 rad` (empirical calibration constant, see `src/estimation/README.md §6`).  These constants are shared with the MEKF so simulation and estimation are numerically identical.

To convert world-frame velocity to body frame, the Rodrigues formula is applied with the conjugate quaternion: `v_body = R^T × v_world` (`sensors/sim.rs:60`).

### 2.2 Range Sensor Model — Tilt Correction

A downward-pointing VL53L1x at height `h` above a flat floor reads the slant range:

```
range_m = h / cos(tilt)
```

The tilt cosine is read directly from the rotation matrix element R₃₃:

```
cos(tilt) = R₃₃ = 1 − 2(qx² + qy²)
```

This is the world-z component of the body z-axis expressed in world coordinates.  The same formula applies to each of the six sensors on the Multi-ranger Deck; for a non-downward sensor the numerator is the world-axis-aligned obstacle distance and the denominator is the projection of the sensor's body-axis onto that world axis after rotation.

For tilt > ~70° the sensor saturates; the code clamps `cos_tilt` to a minimum of 0.05 to prevent division by zero.

### 2.3 Pinhole Camera Model

For a 3D point P = (X, Y, Z) in the camera frame (Z > 0 is forward):

```
x_n = X / Z,   y_n = Y / Z          (normalised coordinates)
r²  = x_n² + y_n²
k   = 1 + k1·r² + k2·r⁴             (Brown-Conrady radial distortion factor)

u = fx · x_n · k  +  cx             (pixel column)
v = fy · y_n · k  +  cy             (pixel row)
```

Default HM01B0 intrinsics: `fx = fy = 164 px`, `cx = cy = 160 px`, `k1 = k2 = 0` (assumed undistorted).  This gives a horizontal FOV of `2 · atan(320 / (2 · 164)) ≈ 88.6°`.

Undistortion runs three Newton iterations dividing the normalised coordinate by the distortion factor `k` at the current estimate.  Converges in one step for `|k1| < 0.5`.  When k1 = k2 = 0 the function returns the input unchanged (zero-cost no-op).

**Camera-to-body frame convention** (`sensors/sim.rs:332`):
```
cam_x  =  +body_y   (left  → right in image)
cam_y  =  −body_z   (up    → down  in image)
cam_z  =  +body_x   (forward)
```

### 2.4 FAST-9 Corner Detector

For each candidate pixel `p`, 16 pixels are sampled on a Bresenham circle of radius 3.  A pixel is a corner if ≥ 9 **contiguous** circle pixels are all brighter (`> p + threshold`) OR all darker (`< p − threshold`).

Circle offsets (row, col) starting at (0, +3), clockwise (`processing/features.rs:31`):
```
(0,3) (1,3) (2,2) (3,1) (3,0) (3,-1) (2,-2) (1,-3)
(0,-3) (-1,-3) (-2,-2) (-3,-1) (-3,0) (-3,1) (-2,2) (-1,3)
```

**Quick rejection** (`features.rs:136`): check the 4 cardinal pixels (indices 0, 4, 8, 12) first.  If fewer than 2 agree on bright or dark, skip the full check.  This rejects ~75% of non-corner candidates before the full 16-pixel scan.

**Score** (`features.rs:106`): sum of absolute differences from centre for the winning arc, computed over all 16 circle pixels.  Used to rank corners by strength.

**Wrap-around run detection** (`features.rs:83`): the circular array is scanned twice to catch runs that span the index-15 → index-0 boundary.  The second pass starts from where the first ended, not from index 0.

### 2.5 BRIEF-256 Descriptor

For each keypoint, 256 pixel pairs are compared within a 25×25 patch (offsets ±12 pixels).  Each comparison produces 1 bit:

```
bit_i = 1   if  pixel(p1_i) < pixel(p2_i)
bit_i = 0   otherwise
```

256 bits are packed into `[u8; 32]` (bit `i` → byte `i/8`, bit position `i%8`).

The comparison table `BRIEF_PATTERN: [[i8; 4]; 256]` is a compile-time `const` generated by four independent linear functions mapping the pair index to offsets in [-12, 12] (`features.rs:47`).  The table is the same every compilation — no random seed needed at runtime.

Matching uses Hamming distance: `Σ popcount(d1[i] XOR d2[i])` for i in 0..32.  Random match expectation: 128 bits.  Adjacent pixels on the same surface typically score < 50 bits.

### 2.6 CPX Framing Protocol

The AI Deck streams JPEG fragments over TCP port 5000 (Nina W102 WiFi).  Each fragment is preceded by a 4-byte header:

```
Byte 0   : router  — 0x09 = CPX_ROUTING_CAMERA
Byte 1   : flags   — 0x01 = last fragment of this frame; 0x00 = more follow
Byte 2–3 : payload length, little-endian u16
Byte 4+  : JPEG payload bytes
```

A complete JPEG frame spans 2–20 fragments.  The receiver (`CpxCamera`) appends each payload to `frag_buf` until `flags = 0x01`, then decodes the complete JPEG with the `jpeg-decoder` crate.  If the JPEG is RGB24 it is converted to grayscale via ITU-R BT.601 luma (`sensors/cpx.rs:206`).

---

## 3. Architecture

```
   [ Simulation ]                     [ Real Hardware ]
  MultirotorState                     Crazyflie CRTP log
        |                                    |
  sensors/sim.rs                      sensors/crtp.rs
  SimFlowSensor::measure()           CrtpFlowAdapter::from_log_row()
  SimRangeSensor::measure()          CrtpRangeAdapter::from_log_row()
  SimMultiRangeSensor::measure()     CrtpMultiRangeAdapter::from_log_row()
  SimCamera::render()                         |
        |                              AI Deck (CPX WiFi TCP)
        |                                    |
        |                             sensors/cpx.rs
        |                             CpxCamera::recv_frame()
        |                             → JPEG fragment reassembly
        |                             → jpeg-decoder → grayscale pixels
        \____________  traits.rs  ____________/
                           |
              SensorSource<FlowMeasurement>  poll()
              SensorSource<RangeMeasurement> poll()
              ImageSource::next_frame()
                           |
              ┌────────────┴─────────────┐
              │     processing/          │
              │                          │
              │  features.rs             │
              │  detect_features()       │
              │  ├─ is_fast9_corner()    │
              │  └─ compute_brief()      │
              │                          │
              │  calibration.rs          │
              │  CameraIntrinsics::      │
              │  ├─ project()            │
              │  └─ unproject()          │
              │                          │
              │  sync.rs                 │
              │  ImuCameraSync::         │
              │  ├─ push_imu()           │
              │  └─ query_at()           │
              └──────────────────────────┘
                           |
                    Vec<Feature>
                    → mapping/ (KeyframeStore, OccupancyMap)
```

---

## 4. Key Data Types

All defined in `src/perception/types.rs`.

### `FlowMeasurement` — `types.rs:6`

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `dx_px` | `f32` | px | Integrated lateral flow, X body axis |
| `dy_px` | `f32` | px | Integrated lateral flow, Y body axis |
| `dt_s`  | `f32` | s  | Integration period |

### `RangeMeasurement` — `types.rs:14`

| Field | Type | Units | Description |
|-------|------|-------|-------------|
| `range_m` | `f32` | m | Single-axis ToF range (downward from Flow Deck) |

### `MultiRangeMeasurement` — `types.rs:20`

| Field | Type | Description |
|-------|------|-------------|
| `front_m` | `Option<f32>` | +X body axis; `None` = out-of-range |
| `back_m`  | `Option<f32>` | −X body axis |
| `left_m`  | `Option<f32>` | +Y body axis |
| `right_m` | `Option<f32>` | −Y body axis |
| `up_m`    | `Option<f32>` | +Z body axis |
| `down_m`  | `Option<f32>` | −Z body axis (same hardware as Flow Deck range) |

### `CameraIntrinsics` — `types.rs:35`

| Field | Type | Description |
|-------|------|-------------|
| `fx, fy` | `f32` | Focal lengths [px] |
| `cx, cy` | `f32` | Principal point [px] |
| `k1, k2` | `f32` | Brown-Conrady radial distortion coefficients |

### `ImageFrame` — `types.rs:45`

| Field | Type | Description |
|-------|------|-------------|
| `width, height` | `u16` | Image dimensions [px] |
| `pixels` | `Vec<u8>` | Row-major 8-bit grayscale |
| `timestamp_ms` | `u64` | Arrival time [ms since epoch] |

Helper methods: `pixel(row, col) -> u8`, `pixel_i(idx) -> u8`.

### `Feature` — `types.rs:56`

| Field | Type | Description |
|-------|------|-------------|
| `x, y` | `f32` | Image column / row [px] |
| `score` | `f32` | FAST-9 arc sum (higher = stronger corner) |
| `descriptor` | `[u8; 32]` | BRIEF-256 bitstring |

Helper method: `hamming_distance(&other) -> u32`.

### `SensorSource<M>` / `ImageSource` — `traits.rs:8`

```rust
pub trait SensorSource<M> { fn poll(&mut self) -> Option<M>; }
pub trait ImageSource      { fn next_frame(&mut self) -> Option<ImageFrame>; }
```

Synchronous pull-based interface.  Sim sensors use `measure()` / `render()` directly (passing state in); the `poll()` / `next_frame()` methods on sim types return `None` and are provided to satisfy the trait for code that operates generically.

---

## 5. Algorithm Walkthrough

### SimFlowSensor::measure() — `sensors/sim.rs:129`

```
height = max(state.position.z, 0.05)       // clamp avoids /0 near ground
scale  = dt × NP / (height × THETA_P)
v_body = rotate_world_to_body(q, v_world)  // Rodrigues with q_conj
dx_px  = v_body[0] × scale  +  N(0, σ)
dy_px  = v_body[1] × scale  +  N(0, σ)
```

`rotate_world_to_body` uses the conjugate quaternion (negated vector part, `sim.rs:67`).  The formula exactly matches the MEKF flow forward model (`estimation/mekf.rs`) so sim → MEKF round-trips are numerically consistent.

### SimRangeSensor::measure() — `sensors/sim.rs:175`

```
cos_tilt = max(|1 − 2(qx² + qy²)|, 0.05)   // R₃₃, clamped
raw      = state.position.z / cos_tilt
range_m  = clamp(raw + N(0, σ), 0.02, 4.0)
```

### SimCamera::render() — `sensors/sim.rs:310`

For each world landmark:
1. Translate to drone-relative vector in world frame (line 325).
2. Rotate to body frame via `rotate_world_to_body` (line 328).
3. Remap axes to camera frame: `cam_z = body_x`, `cam_x = body_y`, `cam_y = −body_z` (lines 333–335).
4. Skip if `cam_z ≤ 0.01` (behind camera, line 337).
5. Project: `u = fx · cam_x / cam_z + cx`, `v = fy · cam_y / cam_z + cy` (lines 339–340).
6. Paint a 3×3 bright spot (intensity 220) centred at the projected pixel (lines 346–354).

Background intensity is 30; unlit pixels remain at this value.

### CrtpMultiRangeAdapter::from_log_row() — `sensors/crtp.rs:87`

Thin unit-conversion wrapper.  Each u16 value is passed to `mm_to_metres()` (line 109):
```
if mm == u16::MAX  → None             (out-of-range sentinel)
if mm / 1000.0 > 4.0 → None          (saturated reading)
otherwise          → Some(mm / 1000.0)
```

No runtime state — just explicit, testable unit conventions.

### CpxCamera::recv_frame() — `sensors/cpx.rs:128`

```
frag_buf.clear()
loop:
    read 4-byte CPX header → (router, flags, payload_len)
    if router ≠ 0x09  → return Err(UnexpectedRouter)
    if frag_buf.len() + payload_len > 2 MiB → return Err(FrameTooLarge)
    append payload_len bytes to frag_buf
    if flags & 0x01 ≠ 0 → break   // last fragment
decode_jpeg_frame(&frag_buf)
    jpeg_decoder::Decoder::decode()
    if RGB24 → convert to grayscale via ITU-R BT.601 luma
    return ImageFrame { width, height, pixels, timestamp_ms }
```

The `cached_frame` field holds the last decoded frame for the synchronous `ImageSource::next_frame()` poll used by the main loop.

### is_fast9_corner() — `processing/features.rs:126`

```
centre = frame.pixel(row, col)

// Quick rejection (line 136):
check 4 cardinal pixels (indices 0, 4, 8, 12)
if bright_count < 2 AND dark_count < 2 → return None

// Full test (line 149):
for each of 16 circle pixels → classify as +1 (bright), -1 (dark), 0 (neutral)
bright_run = max_circular_run(classes, +1)   // handles wrap
dark_run   = max_circular_run(classes, -1)

if bright_run ≥ 9 OR dark_run ≥ 9 → return Some(fast_score())
else → return None
```

### compute_brief() — `processing/features.rs:175`

```
for bit_idx in 0..256:
    (r1, c1) = clamp(row + BRIEF_PATTERN[bit_idx][0..1], image bounds)
    (r2, c2) = clamp(row + BRIEF_PATTERN[bit_idx][2..3], image bounds)
    if pixel(r1,c1) < pixel(r2,c2):
        descriptor[bit_idx/8] |= 1 << (bit_idx % 8)
return descriptor  // [u8; 32]
```

### detect_features() — `processing/features.rs:205`

```
for row in 3..(height−3):
    for col in 3..(width−3):
        if let Some(score) = is_fast9_corner(frame, row, col, threshold):
            descriptor = compute_brief(frame, row, col)
            push Feature { x: col, y: row, score, descriptor }

sort by score descending
return Vec<Feature>
```

The 3-pixel border exclusion ensures the FAST circle never accesses out-of-bounds pixels.

### CameraIntrinsics::project() / unproject() — `processing/calibration.rs:41`

**project** (forward, 3D → 2D):
```
if Z ≤ 0 → None
x_n = X/Z,  y_n = Y/Z
r²  = x_n² + y_n²
k   = 1 + k1·r² + k2·r⁴
u   = fx · x_n · k + cx
v   = fy · y_n · k + cy
```

**unproject** (backward, 2D + depth → 3D, line 55):
```
(u_u, v_u) = undistort_point(u, v)    // 3 Newton iters
x_n = (u_u − cx) / fx
y_n = (v_u − cy) / fy
return Vec3(x_n·depth, y_n·depth, depth)
```

### ImuCameraSync — `processing/sync.rs:60`

**push_imu** (line 60): append `(t_ms, gyro, accel)` to `VecDeque`; prune front while `newest − t_old > max_age_ms`.

**query_at** (line 83): linear search for the two samples bracketing `t_ms`; return `None` if query is outside the buffer window:
```
alpha = (t_ms − t0) / (t1 − t0)
gyro  = lerp(g0, g1, alpha)
accel = lerp(a0, a1, alpha)
```

---

## 6. Parameters & Tuning

| Constant | Value | Location | Purpose |
|----------|-------|----------|---------|
| `NP` | 350.0 px | `sensors/sim.rs:20` | PMW3901 nominal pixel count (CF firmware convention) |
| `THETA_P` | 3.50 rad | `sensors/sim.rs:21` | Empirical flow calibration constant — must match MEKF |
| `RANGE_MIN_M` | 0.02 m | `sensors/sim.rs:24` | VL53L1x blind zone |
| `RANGE_MAX_M` | 4.0 m | `sensors/sim.rs:25` | VL53L1x maximum range |
| `FAST_MIN_RUN` | 9 | `processing/features.rs:39` | Minimum contiguous arc for FAST-9 |
| BRIEF patch | 25×25 px (±12) | `processing/features.rs:47` | Standard BRIEF window |
| HM01B0 fx/fy | 164 px | `processing/calibration.rs:102` | Approximate focal length for 320×320 sensor |
| HM01B0 cx/cy | 160 px | `processing/calibration.rs:103` | Image centre |
| `CPX_ROUTER_CAMERA` | 0x09 | `sensors/cpx.rs:50` | Camera channel in AI Deck CPX protocol |
| `MAX_FRAME_BYTES` | 2 MiB | `sensors/cpx.rs:55` | Guard against framing errors |

**FAST threshold tuning**: 20 is a good default.  Lower values (10–15) detect more corners but add noise; higher values (30–40) keep only the strongest.  On the HM01B0 at 0.3 m AGL expect 10–80 features on a textured floor.

**Flow sensor noise (`SimFlowSensor`)**: `noise_stddev = 0.05 px` at 100 Hz reproduces the noise level seen in real PMW3901 logs.  Set to 0.0 for exact formula verification.

---

## 7. Connections to Other Modules

| Direction | Module | What is exchanged |
|-----------|--------|------------------|
| Consumes | `dynamics/` | `MultirotorState` (position, velocity, orientation) for all sim sensors |
| Consumes | `estimation/` | `NP`, `THETA_P` constants must remain in sync with MEKF flow model |
| Feeds | `estimation/` | `FlowMeasurement`, `RangeMeasurement` passed to `Mekf::feed_row()` |
| Feeds | `mapping/` | `Vec<Feature>` fed into `KeyframeStore` for visual odometry; `MultiRangeMeasurement` fed into `OccupancyMap::update()` |
| Used by | `bin/sim_closed_loop.rs` | All three sim sensors wired into control loop; feature count logged |
| Used by | `bin/main.rs` | Multi-ranger block5 decoded via `CrtpMultiRangeAdapter`; AI Deck stream via `CpxCamera` |

---

## 8. Common Pitfalls

**THETA_P is not transferable**: The value 3.50 is specific to CF 2.1 + Flow Deck v2 + the CF firmware logging convention.  A different firmware version or sensor calibration requires a fresh amplitude-ratio sweep (see `estimation/README.md §6`).

**PMW3901 axis convention**: `vel_x ↔ −flow_dy` and `vel_y ↔ −flow_dx` (PMW3901 sensor frame ≠ body frame).  This remapping is applied in `main.rs` before passing to the MEKF; `CrtpFlowAdapter::from_log_row()` passes values through as-is by design — the convention lives in one place.

**CPX fragmentation**: A complete JPEG frame arrives in 2–20 fragments.  The `recv_frame()` loop must not stop on the first `flags = 0x00` packet; only `flags = 0x01` signals the last fragment.  If the TCP connection drops mid-frame the reassembly buffer contains a truncated JPEG — `jpeg-decoder` returns an error; reconnect, do not try to decode partial data.

**AI Deck AP-mode IP**: The AI Deck defaults to `192.168.4.1` when in access-point mode.  If connected through a router the IP will differ — check with `arp -n` or the router's DHCP lease list.

**BRIEF boundary clamping**: Keypoints within 12 pixels of the edge have some patch samples clamped to the border.  This reduces descriptor quality slightly but avoids out-of-bounds access.  For production SLAM, exclude keypoints within 15 pixels of the border (3-pixel FAST guard + 12-pixel BRIEF patch radius).

**HM01B0 intrinsics are approximate**: `fx = fy = 164 px` with k1 = k2 = 0 is a first approximation.  A proper calibration with a checkerboard and OpenCV yields both a more accurate focal length and real distortion coefficients.  The undistort Newton solver is already in place; only the constants need updating.

**SimFlowSensor height clamp**: The clamp to `max(z, 0.05)` prevents division-by-zero when the drone is on the ground.  Below 5 cm the flow model is unphysical anyway (optical flow requires a minimum ground distance).

---

## 9. Related Tests

| Test | File | What it covers |
|------|------|----------------|
| `sim_flow_reproduces_mekf_model` | `sensors/sim.rs` | `SimFlowSensor` formula matches MEKF forward model exactly at height 1 m |
| `sim_flow_scales_with_height` | `sensors/sim.rs` | Doubling height halves pixel displacement |
| `sim_range_level_equals_height` | `sensors/sim.rs` | Level hover: range = height |
| `sim_range_cos_tilt_correction` | `sensors/sim.rs` | 30° roll → range = height / cos(30°) |
| `sim_multi_range_all_axes` | `sensors/sim.rs` | All 6 sensor directions at known obstacle distances |
| `sim_multi_range_no_obstacle_is_none` | `sensors/sim.rs` | `f32::INFINITY` obstacle → `None` |
| `sim_camera_projects_landmark` | `sensors/sim.rs` | Forward landmark projects near image centre |
| `sim_camera_behind_is_blank` | `sensors/sim.rs` | Rear landmark does not project |
| `crtp_flow_passthrough` | `sensors/crtp.rs` | Values pass through unchanged |
| `crtp_range_mm_to_metres` | `sensors/crtp.rs` | 500 mm → 0.5 m |
| `crtp_range_overflow_is_none` | `sensors/crtp.rs` | > 4000 mm → `None` |
| `crtp_multi_range_sentinel` | `sensors/crtp.rs` | `u16::MAX` (65535) → `None` on all axes |
| `crtp_multi_range_over_4m_is_none` | `sensors/crtp.rs` | 4001 mm → `None` |
| `cpx_fragment_reassembly` | `sensors/cpx.rs` | 2-fragment JPEG reassembled byte-for-byte correctly |
| `cpx_jpeg_decode_1x1` | `sensors/cpx.rs` | Minimal 1×1 JPEG decoded to a single grayscale pixel |
| `fast_detects_corners` | `processing/features.rs` | Bright rectangle → ≥ 4 corners |
| `fast_flat_image_has_no_corners` | `processing/features.rs` | Uniform image → 0 features |
| `brief_descriptor_same_point_is_deterministic` | `processing/features.rs` | Same pixel always gives the same descriptor |
| `brief_descriptor_distance` | `processing/features.rs` | Adjacent pixels → Hamming dist < 128 |
| `features_sorted_by_score` | `processing/features.rs` | Output is sorted strongest-first |
| `max_circular_run_wrap` | `processing/features.rs` | Run spanning index 15→0 counted correctly |
| `project_unproject_roundtrip` | `processing/calibration.rs` | 3D → 2D → 3D ≈ original within 0.1 mm |
| `project_centre_at_principal_point` | `processing/calibration.rs` | Optical axis point → (cx, cy) |
| `project_behind_camera_is_none` | `processing/calibration.rs` | Z ≤ 0 → `None` |
| `undistort_zero_distortion_is_noop` | `processing/calibration.rs` | k1=k2=0 → input unchanged |
| `undistort_with_distortion_moves_point` | `processing/calibration.rs` | Positive k1 → point moves inward |
| `hm01b0_defaults_reasonable` | `processing/calibration.rs` | FOV for fx=164, w=320 ≈ 88.6° |
| `sync_interpolation` | `processing/sync.rs` | Midpoint query → exactly half of endpoint values |
| `sync_exact_sample_returned` | `processing/sync.rs` | Query at exact sample timestamp |
| `sync_before_oldest_is_none` | `processing/sync.rs` | Query before buffer window → `None` |
| `sync_after_newest_is_none` | `processing/sync.rs` | Query after buffer window → `None` |
| `sync_old_samples_pruned` | `processing/sync.rs` | Samples older than `max_age_ms` are removed |
| `sync_empty_buffer_returns_none` | `processing/sync.rs` | Empty buffer → `None` |
