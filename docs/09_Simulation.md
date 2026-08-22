# Simulation

**Last updated:** 22 August 2026

How the thesis controllers run in simulation, and what the simulator is and is not good for.

---

## What this is

The Crazyswarm2 software-in-the-loop (SIL) simulator runs the **real firmware** through SWIG
bindings. As of 22 August 2026 it also runs **our** controller: the same Rust source that flies on
the drone (`flying_drone_stack/firmware_app`), cross-compiled for the host and linked into the
bindings. It is not a reimplementation, so a control-law change is tested in sim and flown without
being written twice.

The `neuralswarm` backend adds the Neural-Swarm2 deep-sets downwash model, which means the
interaction force this thesis is about is present in simulation.

| | |
|---|---|
| **Good for** | catching integration bugs before the lab; comparing control laws under an identical, repeatable disturbance; developing the multi-drone scripts |
| **Not good for** | absolute accuracy. The downwash model is someone else's fit to someone else's drone, and the actuators are ideal (no motor lag) |
| **Never** | a substitute for a flight. Sim results are a hypothesis; the lab decides |

---

## Running it

```bash
# once per firmware change
cd flying_drone_stack/firmware_app
RUSTFLAGS="-C panic=abort" cargo build --release --target x86_64-unknown-linux-gnu
cd ~/Desktop/crazyflie-firmware && make bindings_python
export PYTHONPATH=~/Desktop/crazyflie-firmware/build:$PYTHONPATH   # required

# terminal 1 -- server
cd ~/Desktop/crazyswarm2 && source install/setup.bash
ros2 launch crazyflie launch.py backend:=sim \
    crazyflies_yaml_file:=$PWD/crazyflie/config/crazyflies_sim.yaml \
    server_yaml_file:=$PWD/crazyflie/config/server_sim_geo.yaml

# terminal 2 -- any normal flight script, on sim time
ros2 run crazyflie_examples sim_downwash_test --ros-args -p use_sim_time:=true
```

⚠️ The `crazyflie-firmware/bindings/` changes this depends on live in a tree that tracks **bitcraze
upstream**, so they are deliberately **not committed** anywhere. They are ~110 lines and touch no
in-tree source, but the simulator does not build without them, and a `git checkout` or upstream pull
in that tree would remove them silently. A copy is kept as
`flying_drone_stack/firmware_app/host/cffirmware_bindings.patch` — see
[`host/README.md`](../flying_drone_stack/firmware_app/host/README.md) for how to re-apply. **If the
simulator stops building, check this first.**

**Both yaml arguments are required.** Give only `server_yaml_file` and you get the hardware roster,
which is one drone — the two-drone case silently does not happen. **`use_sim_time:=true` is
required** on every client: `launch.py` sets it for rviz and the gui only, so without it commands
are issued on the wall clock while the sim runs ~4x slower, and the flight lands mid-manoeuvre.

### Choosing the controller

`server_sim_geo.yaml` and `server_sim_indi.yaml` differ in two lines:

```yaml
sim:
  controller: oot        # our controller; also: mellinger, pid, brescianini, none
  oot_ctrl_mode: 0       # 0 geometric SE(3) | 1 position INDI | 2 attitude INDI | 3 full INDI
  backend: neuralswarm   # neuralswarm = downwash model | np = no interaction
```

`oot_ctrl_mode` is the same selector as `indi_gains.ctrl_mode` on hardware. Gains come from
`traj_iface.c` defaults, so sim and drone start from the same numbers.

---

## The simulated airframe comes from the firmware

INDI inverts the vehicle model. If the simulated plant has a different mass, thrust constant or arm
length than the firmware was compiled with, that mismatch appears as a **residual force** — and
residual force is the quantity this thesis measures. A plant/controller mismatch would therefore be
indistinguishable from the downwash we are trying to study.

So the plant is built from the firmware's own constants (`g_indi_mass`, `g_indi_kt1..4`,
`THRUST_MAX`, `ARM_LENGTH`, `THRUST2TORQUE`) by `crazyflie_server._setup_oot()` rather than from a
second set of numbers in a yaml. Changing the platform define changes both together. The server
logs what it chose on startup:

```
out-of-tree controller: ctrl_mode=0 (geometric SE(3))
simulated airframe taken from firmware: mass=0.0364 kg, THRUST_MAX=0.200 N/motor, arm=0.050 m
```

An explicit `sim.physics` block overrides this if a deliberate mismatch is ever wanted (for
example, to measure how much model error INDI tolerates).

The airframe follows `-DCONFIG_PLATFORM_CF21BL` in `crazyflie-firmware/bindings/setup.py`; build
with `OOT_PLATFORM=CONFIG_PLATFORM_CF2` for the brushed drone.

---

## Fidelity: what was fixed, and what is still idealised

Four things had to be corrected before the simulator represented our drone at all. They are
recorded because each one first looked like a controller or tuning problem:

| Was | Effect | Now |
|---|---|---|
| The accelerometer was never fed (upstream `TODO`) | INDI inverted a permanent free-fall | `State.acc` = body-frame specific force, computed from thrust **and the downwash force**, so `a_res` is measurable in sim |
| Bindings compiled with no platform define | `THRUST_MAX` 0.1125 N/motor instead of 0.2 → mixer saturated → attitude INDI never left the ground | `-DCONFIG_PLATFORM_CF21BL` |
| Three motor calibrations chained (`powerDistribution` → CF2.0 PWM→RPM fit → `kt·RPM²`) | ~1/3 of commanded thrust delivered; identical ~260 mm droop in every mode, reads as a tuning problem | `pwm_to_force` inverts `powerDistribution` exactly; one force model end to end |
| Controller called at the backend's 2 kHz while `tick` is in ms | `dt = 0` on every second call; anything that differentiates is destroyed. Full INDI never took off; geometric looked fine | control held to 1 kHz, command held across physics substeps, as on hardware |
| One controller instance shared by N simulated drones | Interleaved calls would share `last_tick`, the INDI filters and the integrators | `oot_select_drone(idx)` swaps a per-vehicle state blob. **Any new stateful controller needs the same** |
| **`firmware_params` were never applied** | The controller ran on `traj_iface.c` compile-time defaults — mass 0.0364 not 0.041, kt 1.48e-10 not 4.1e-10, `kr` 100/`kw` 30 instead of 2400/170, and `filt_order`/`filt_tau` off. Attitude INDI diverged to a 45° tilt over ~12 s while flying fine on the drone | `_apply_oot_firmware_params()` pushes `crazyflies.yaml` into the controller, as CRTP does at connect |

The last one is worth dwelling on, because it is a property of SIL rather than a bug in anyone's
code: on a drone there is one vehicle per MCU, so a single static holding the filter state is
correct. Upstream has the same constraint — that is why the Mellinger branch threads a per-drone
`mellinger_control` struct through the call. Our controller keeps its state in one Rust static, so
the simulator swaps that block in and out per vehicle (`oot_state_ptr`/`oot_state_size` in
`lib.rs`, `oot_select_drone` in `host/oot_host.c`). Those two exports are the only additions to
flight code, and they touch no control logic.

Still idealised, and worth remembering before trusting a number:

- **No motor lag.** The actuator time constant (44 ms brushless / 71 ms upgraded) is the leading
  suspect in the 6.3 Hz shake seen in flight — see `docs/investigation_indi_oscillation_2026-07-21.md`.
  The simulator cannot reproduce that, and will therefore make INDI look better than it is.
- **No sensor noise, no EKF.** State is fed in perfectly. The current leading hypothesis for the
  `kr≈600` shake is EKF attitude phase lag, which by construction cannot appear here.
- **No radio, no latency, no dropouts.**
- The downwash model is Neural-Swarm2's pretrained network, fitted to their vehicle, not ours.

Consequence: **the simulator can prove a controller is broken; it cannot prove one is good.**

---

## ⛔ Known blocker: trajectories do not fly in simulation

Everything below was verified on hover, takeoff and `goTo`. **The out-of-tree controller
does not track high-level-commander trajectories in SIL** — it diverges in attitude and
crashes, on the pre-existing exported trajectories (`circle` 139°, `figure8` 79°, `oval`
168° of tilt) as much as on generated ones. The in-tree `mellinger` and `pid` controllers
fly the identical `circle` file at a correct 27° bank, so the defect is on our side of the
simulator, not in the trajectories or the plant.

Ruled out: CSV coefficient precision, piece count, the downwash model, and the missing
jerk/snap in the SIL setpoint (which was a genuine omission against
`crtp_commander_high_level.c`, now fixed — but not the cause; `g_indi_omega_src` defaults
to taking the body rate from the setpoint instead).

Until this is resolved the simulator can validate **static** formation geometry only. See
[`10_Formation_Library.md`](10_Formation_Library.md) for the measurements.

---

## Verified results (22 August 2026)

Single drone, takeoff to 1.0 m, 15 s, brushless gains from `crazyflies.yaml`: all four control
modes hold **1.000 m at 0.0° tilt**.

Two drones stacked 0.5 m apart, `neuralswarm` backend, ROS end to end:

| Control | lower drone (in the wash), cmd 1.0 m | upper drone, cmd 1.5 m |
|---|---|---|
| Geometric SE(3) | **−30.5 mm** | **−2.9 mm** |
| Full INDI | **−58.8 mm** | **−6.2 mm** |

The ~10x asymmetry between lower and upper is the signature of downwash rather than a trim error,
and it is the effect the thesis sets out to compensate.

**Do not read the geometric-vs-INDI difference as a result yet.** It is one run per controller,
steady hover only, with no motor lag in the plant and gains that were tuned on hardware for
trajectory tracking rather than for hover disturbance rejection. It says the pipeline works and
produces plausible physics; it does not say which controller rejects downwash better. That
comparison belongs to Section C.3 and needs the protocol in
[`05_Experimental_Protocol_2Robot.md`](05_Experimental_Protocol_2Robot.md).

An earlier version of these numbers reported INDI failing outright. That was a bad steady-window
filter in the analysis, not the controller.

---

## Why not Isaac Sim

Evaluated and rejected: it needs a discrete NVIDIA GPU, and this laptop has integrated graphics.
The rendering it would add is not what this thesis measures — the physics and the downwash model
are, and those are already here. Blender was also rejected: its Crazyswarm2 integration renders
per-drone FPV images for synthetic vision datasets, not the third-person visualisation that would
have been the reason to add it.

RViz gives the visualisation actually needed. `record_states` writes CSV/npy per drone for analysis.

See [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md) for status.
