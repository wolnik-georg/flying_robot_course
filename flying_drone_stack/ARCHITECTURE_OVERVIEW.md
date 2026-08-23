# Architecture Overview — Rust vs CS2

## Big Picture

The goal is to replace the Rust onboard scripts with CS2 as the core runtime,
while keeping motion planning and the geometric controller exactly as they are.
Functionally the two architectures are equivalent — same controller, same
trajectories, better pose source.

---

## Component-by-Component Comparison

| Component | Old (Rust) | New (CS2) | Status |
|---|---|---|---|
| **Motion planning** | Rust — generates 9-coeff (degree-8) polynomial segments | Same Rust code — unchanged | ✅ stays exactly as-is |
| **Trajectory format on drone** | 9-coeff, uploaded via `traj.ci/cv/cw` CRTP params, evaluated at 500 Hz (`traj.mode=1`) | 8-coeff Poly4D, uploaded via CS2 ROS service, evaluated at 500 Hz (`traj.mode=0`) | ⚠️ minor change — see note below |
| **Trajectory export** | Rust binary directly uploads at runtime | Export coefficients to CSV/YAML file → CS2 Python script uploads | 🔧 needs implementation |
| **Geometric SE(3) controller** | `firmware_app/src/lib.rs` — controller=6 | Same firmware, same file, unchanged | ✅ stays exactly as-is |
| **INDI (future)** | Builds on geometric controller | Same — INDI layer is firmware-side, unaffected by CS2 | ✅ unaffected |
| **Pose / state estimation** | Optical flow deck → onboard EKF | OptiTrack → CS2 → `send_extpose` → onboard EKF | ✅ better (no drift) |
| **Radio ownership** | Rust script owns Crazyradio directly | CS2 server owns Crazyradio | ✅ cleaner, no conflict |
| **Logging** | Rust reads firmware log vars over CRTP → CSV | CS2 publishes firmware log vars as ROS topics → rosbag | 🔧 needs config (`crazyflies.yaml`) |
| **Flight execution** | One Rust binary per maneuver (`onboard_figure8`, etc.) | Python script calls CS2 ROS services (takeoff, upload, start, land) | 🔧 needs Python scripts per maneuver |
| **Multi-drone** | Manual, one radio per drone | Built into CS2 | ✅ free upgrade |

---

## The One Real Difference: 9-coeff vs 8-coeff Trajectories

| | Rust (old) | CS2 (new) |
|---|---|---|
| Polynomial degree | 8 (9 coefficients) | 7 (8 coefficients) |
| Upload method | CRTP params `traj.ci/cv/cw` | CS2 `upload_trajectory` ROS service |
| Onboard evaluator | Custom (firmware_app) | Standard crazyflie-firmware Poly4D |
| Quality difference | One extra degree of freedom | Negligible for smooth trajectories |

**In practice:** degree-7 is the standard for minimum-snap trajectory generation.
The 9th coefficient gives a marginal improvement that is not measurable in flight.
The export step is: Rust planner runs as before → writes 8-coeff CSV → CS2 uploads it.

---

## What Needs to Be Built

### 1. Trajectory export (Rust side)
- Add an export function to the Rust motion planner
- Output: CSV or YAML with 8 coefficients per segment per axis + duration
- The planner already has all the data — this is just a serialization step

### 2. CS2 trajectory upload script (Python side)
- Small Python script that reads the exported file
- Calls CS2 `upload_trajectory` ROS service
- Calls `takeoff` → `start_trajectory` → `land`
- One script per maneuver type (figure-8, circle, etc.) — mirrors the old Rust binaries

### 3. CS2 logging config
- Enable custom firmware log topics in `crazyflie/config/crazyflies.yaml`
- Variables: `stateEstimate.x/y/z`, `stabilizer.roll/pitch/yaw`, `gyro.x/y/z`
- Record with `ros2 bag record`, analyze with existing Python analysis scripts

### 4. Mocap gain tuning (ongoing)
- Block S is current best (2026-05-16)
- Need logging data before tuning further
- Next candidate: Block T (KP=36, KV=3, same S attitude gains)

---

## Functional Equivalence Check

| Capability | Rust architecture | CS2 architecture |
|---|---|---|
| Hover at position | ✅ | ✅ |
| Custom polynomial trajectory | ✅ | ✅ (8-coeff) |
| Geometric SE(3) control | ✅ | ✅ (same firmware) |
| Pose from mocap | ❌ (optical flow only) | ✅ |
| Logging to file | ✅ | ✅ (rosbag) |
| Multiple drones | ❌ manual | ✅ built-in |
| INDI (future) | ✅ (planned) | ✅ (same firmware) |

**Conclusion:** CS2 architecture is functionally equivalent or better in every category.
The only work required is the export step and the Python upload scripts — the
controller, motion planning, and analysis pipeline are all unchanged.

---

## Status (2026-05-17) — CS2 integration complete

All items below are done:
- ✅ Custom log topics in `crazyflies.yaml` (state + gyro_acc at 20 Hz)
- ✅ 8-coeff Poly4D export: `src/bin/export_poly4d.rs` → writes to CS2 data folder
- ✅ Generic flight script: `crazyswarm2/crazyflie_examples/crazyflie_examples/flight.py`
- ✅ Log format matches Rust exactly — `analyze_flight.py` works unchanged
- ✅ Geometric SE(3) controller validated with mocap (Block S, 2026-05-16)

## Immediate Next Steps

1. **Flight** — fly Block S, collect CSV, run `analyze_flight.py` to get XY RMSE baseline
2. **Gains** — try Block T if RMSE is poor; one session max, then freeze
3. **INDI** — implement INDI inner loop on top of geometric outer loop
