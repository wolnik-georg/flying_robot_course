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
    i_ep: Vec3,             // position integral accumulator
    last_tick: u32,         // previous tick for dt computation
    omega_prev: Vec3,       // previous angular velocity (gyro differentiation for INDI)
    omega_dot_filt: Vec3,   // low-pass filtered angular acceleration (INDI)
    tau_prev: Vec3,         // previous torque command (attitude INDI memory)
}
```

`omega_prev` is updated every cycle after the controller runs.

## Gains (active block in lib.rs — Block J, Apr 27 2026)

Multiple gain blocks (A–J) are in lib.rs as commented-out alternatives. Block J is now active.

| Constant | Block J value | Notes |
|----------|--------------|-------|
| `KP_X/Y` | 16.0 | Same as Block I — position BW adequate |
| `KP_Z` | 26.0 | Stiff altitude hold |
| `KV_X/Y` | 8.0 | Same as Block I |
| `KV_Z` | 14.0 | |
| `KI_P` | 0.05 | |
| `KI_LIMIT` | 2.0 | |
| `KR_X/Y` | 0.010 | Slight bump from 0.009 |
| `KR_Z` | 0.010 | |
| `KW_X/Y` | **0.0010** | **Reduced from 0.0016** — fixes attitude overdamping |
| `KW_Z` | 0.0014 | Proportionally reduced |
| `MASS` | 0.027 kg | CF 2.1 + Flow Deck |

**Why Block J**: Block I analysis (Apr 27 2026) showed attitude loop zeta=2.07, tau_dom=167ms.
That is 2× too high, causing the drone to lag in direction reversals (center crossing errors of 20cm).
Block J lowers KW to get zeta=1.23, tau_dom=79ms — 2× faster attitude response.
If stable, try Block K: KW=0.00082 (true critical, zeta=1.0, tau=46ms).

## Block progression (RMSE measured on figure8 with `run_figure8.py`)

| Block | KP | KV | KR | KW | XY RMSE | Lag corr | Notes |
|-------|----|----|-----|------|---------|----------|-------|
| A | 7.5 | 9.0 | 0.007 | 0.00115 | 21.2 cm | −0.92 | baseline |
| C | 9.0 | 11.0 | 0.007 | 0.00115 | 16.2 cm | −0.73 | −24% |
| D | 11.0 | 13.0 | 0.007 | 0.00115 | 15.9 cm | −0.77 | −3% |
| E | 11.0 | 13.0 | 0.009 | 0.0016 | 13.9 cm | — | −12% |
| I | 16.0 | 8.0 | 0.009 | 0.0016 | 12.7–13.3 cm | −0.28 | −8%; attitude still overdamped |
| **J** | 16.0 | 8.0 | 0.010 | **0.0010** | TBD | TBD | **attitude fix: zeta 2.07→1.23** |

## Full-state setpoint (type 6)

When the laptop sends `setpoint_full_state` (CRTP type 6, `spline_*_test` binaries):
- `sp.velocity` and `sp.acceleration` are non-zero → controller uses them as feedforward
- `sp.attitudeRate.{roll,pitch,yaw}` contains desired `omega_d` [rad/s]
- `e_omega = omega - omega_d` (angular velocity tracking error)

When the laptop sends `setpoint_position` (type 7, `main.rs`):
- `sp.velocity`, `sp.acceleration`, `sp.attitudeRate` all zero → pure P+I feedback

## Controller mode (INDI scaffold)

Three controllers are fully implemented in `src/lib.rs`. Switch by changing one constant and reflashing:

```rust
const CONTROLLER_MODE: u8 = 0;   // 0 = Geometric (current)
                                  // 1 = Attitude INDI  (gyro only, no RPM deck)
                                  // 2 = Full INDI      (RPM deck required — see prereqs below)
const FC_GYRO_HZ: f32 = 60.0;    // gyro LP filter cutoff [Hz] — tune for modes 1 & 2
```

**Mode 0 (Geometric):** current baseline, model-based KR/KW torque + gyroscopic compensation.

**Mode 1 (Attitude INDI):** replaces KR/KW torque with `Δτ = J·(α_des − α_meas)`.
- `α_meas = IIR_filter((ω − ω_prev) / dt)` at `FC_GYRO_HZ`
- No RPM deck needed. Only new tuning parameter: `FC_GYRO_HZ` (start 60 Hz).
- To activate: set `CONTROLLER_MODE = 1`, `make cload`, hover-test, adjust `FC_GYRO_HZ`.

**Mode 2 (Full INDI):** replaces `τ_prev` with `τ_current` computed from per-motor RPM².
- Motor model constants: `KT = 3.16e-10` (placeholder — identify on bench), `KQ_KT = 0.005964552`, `ARM_LEN = 0.046 m`.
- **Prerequisites before activating**: RPM deck fitted, `KT` identified on thrust stand, motor spin directions verified against `powerDistributionForceTorque.c`.

**Note on omega_d**: `sp.attitudeRate` (desired ω from flatness) is available in Mode B but currently not used — `e_omega = omega` in all modes. The geometric controller is intentionally kept as the unmodified baseline for comparison against INDI.

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
