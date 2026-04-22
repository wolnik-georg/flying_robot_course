# firmware_app — Onboard SE(3) Controller

Separate crate. No_std Rust staticlib for `thumbv7em-none-eabihf` (Cortex-M4 FPU).
Implements `controllerOutOfTree*` hooks — called at **500 Hz** inside the Crazyflie stabilizer task.

## Build & flash

```bash
cd firmware_app
make                  # cargo build --release + link → build/cf2.bin
make cload            # flash via Crazyradio (preferred)

# Direct flash (use firmware_app/build/cf2.bin, NOT crazyflie-firmware/build/)
cfloader flash firmware_app/build/cf2.bin stm32-fw -w radio://0/80/2M/E7E7E7E7E7

# Verify Rust symbols in the compiled static lib
arm-none-eabi-nm target/thumbv7em-none-eabihf/release/libcf_controller_rs.a \
  | grep controllerOutOfTree
```

## Entry points (src/lib.rs)

```rust
#[no_mangle] pub extern "C" fn controllerOutOfTreeInit()
#[no_mangle] pub extern "C" fn controllerOutOfTreeTest() -> bool
#[no_mangle] pub unsafe extern "C" fn controllerOutOfTree(
    control: *mut control_s,
    setpoint: *const setpoint_s,
    sensors: *const sensorData_s,
    state: *const state_s,
    tick: u32,
)
```

## Data flow

```
setpoint_s (from laptop CRTP, 20–25 Hz)
  .position.{x,y,z}       [m]  — desired position
  .velocity.{x,y,z}       [m/s] — desired velocity (non-zero with full-state type 6)
  .acceleration.{x,y,z}   [m/s²] — desired accel (non-zero with full-state type 6)
  .attitude.yaw            [deg] — desired yaw
  .attitudeRate.{roll,pitch,yaw} [rad/s] — desired omega_d (type 6 only)

state_s (from Kalman EKF, 500 Hz)
  .position.{x,y,z}       [m]
  .velocity.{x,y,z}       [m/s]
  .attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.{q0,q1,q2,q3}
    q0=qx, q1=qy, q2=qz, q3=qw  (SCALAR LAST, different from math module!)

sensorData_s (500 Hz IMU)
  .gyro.axis[0,1,2]       [deg/s] — MUST convert: * π/180

control_s output (→ motor mixer)
  union offset 0: thrustSi  [N]
  union offset 4: torque[0] torqueX [Nm]
  union offset 8: torque[1] torqueY [Nm]
  union offset 12: torque[2] torqueZ [Nm]
  .controlMode = controlModeForceTorque
```

## Critical gotchas

**`f32::sqrt()` not available in no_std** — use `libm::sqrtf(x)`.
Same for trig: `libm::sinf`, `libm::cosf`.

**Quaternion convention is REVERSED** vs the math module:
- Math module: `Quat { w, x, y, z }` with w first
- Firmware bindgen: `q0=qx, q1=qy, q2=qz, q3=qw` (scalar last, q3 is w)
- Access: `state.attitudeQuaternion.__bindgen_anon_1.__bindgen_anon_1.q3` for `qw`

**Two levels of anonymous union** in quaternion: `.__bindgen_anon_1.__bindgen_anon_1.qN`

**Gyro in deg/s** — multiply by `π/180` before using in the controller.

**Setpoint yaw in degrees** — multiply by `π/180` before using.

**Arming condition** — controller outputs zeros until `sp.position.z > 0.05 m`.
This prevents spinning during HLC takeoff ramp before the trajectory setpoint reaches height.

**`static mut CTRL` access** — use `core::ptr::addr_of_mut!(CTRL)` to avoid Rust 2024 warnings.

## Controller state (src/lib.rs)

```rust
struct State {
    i_ep: Vec3,          // position integral accumulator
    last_tick: u32,      // previous tick for dt computation
    omega_prev: Vec3,    // previous angular velocity — INDI extension point
}
```

`omega_prev` is updated every cycle. Use `(omega - s.omega_prev) / dt` to estimate `ω̇` for INDI.

## Gains (actual values in lib.rs — verified April 2026)

| Constant | Value | Notes |
|----------|-------|-------|
| `KP_X/Y` | 7.5 | Position proportional XY |
| `KP_Z` | 26.0 | Stiff altitude hold |
| `KV_X/Y` | 9.0 | Position derivative XY |
| `KV_Z` | 14.0 | |
| `KI_P` | 0.0 | Integral disabled (gain zero) |
| `KI_LIMIT` | 2.0 | Anti-windup clamp [m/s²·s] |
| `KR_X/Y` | 0.007 | Attitude proportional |
| `KR_Z` | 0.008 | |
| `KW_X/Y` | 0.00115 | Attitude derivative |
| `KW_Z` | 0.002 | |
| `MASS` | 0.027 kg | CF 2.1 + Flow Deck |
| `JXX` | 16.571710e-6 | |
| `JYY` | 16.655602e-6 | |
| `JZZ` | 29.261652e-6 | |

## Full-state setpoint (type 6)

When the laptop sends `setpoint_full_state` (CRTP type 6, `spline_*_test` binaries):
- `sp.velocity` and `sp.acceleration` are non-zero → controller uses them as feedforward
- `sp.attitudeRate.{roll,pitch,yaw}` contains desired `omega_d` [rad/s]
- `e_omega = omega - omega_d` (angular velocity tracking error)

When the laptop sends `setpoint_position` (type 7, `main.rs`):
- `sp.velocity`, `sp.acceleration`, `sp.attitudeRate` all zero → pure P+I feedback

## INDI extension

To add INDI inner loop (replace KR/KW attitude terms):
1. Add `omega_dot_filt: Vec3` to `State`
2. In `controllerOutOfTree`: compute `omega_dot = (omega - s.omega_prev) / dt`
3. Replace attitude PD terms with `Δτ = J · (omega_dot_desired - omega_dot_filt)`
4. Keep position P+I outer loop unchanged
5. No Kbuild/bindgen/firmware changes needed — scaffolding is complete

## File structure

```
firmware_app/
├── Cargo.toml         # no_std staticlib; deps: panic-halt, libm; build-dep: bindgen
├── app-config         # Kconfig: APP_ENABLE=y, CONTROLLER_OOT=y, STACKSIZE=1024
├── Kbuild             # calls cargo build, copies .a as app.o
├── Makefile           # wraps crazyflie-firmware oot.mk
├── build.rs           # bindgen: wrapper.h → $OUT_DIR/bindings.rs
├── wrapper.h          # includes stabilizer_types.h + controller.h
├── .cargo/config.toml # default-target = thumbv7em-none-eabihf
└── src/lib.rs         # ALL controller code (single file)
```
