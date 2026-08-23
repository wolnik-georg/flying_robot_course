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

## Trajectory tracking: three fixes, and one gap that remains

Trajectories were never exercised in simulation until the formation library needed them —
only hover, takeoff and `goTo`. When they were, the controller diverged on any curved path.
Chasing that produced three real fixes and one unresolved discrepancy.

### Fixed

| Was | Effect |
|---|---|
| The SIL dropped `jerk` and `snap` from the setpoint, where `crtp_commander_high_level.c:395` sets both | A flatness-based controller got a permanently zero jerk in sim. Position looked right; only the attitude feedforward was wrong |
| The host controller compiled **non-brushless** (`DRONE_PLATFORM` unset) while the bindings compiled as CF21BL | Hybrid airframe: CF2.1 inertia with CF21BL arm and thrust. Build the host lib with `DRONE_PLATFORM=bl` |
| The plant used the np backend's default CF2.0 inertia | Now read from the controller via `oot_inertia()`, so the two cannot disagree |
| `pwm_to_rpm` applied a `pwm < 10000` idle deadband on the `kt` path | A motor legitimately given a small force was zeroed, losing thrust **and delivering 21% more torque than commanded** at a 10 mNm request. Verified exact after the fix |

The deadband one is worth remembering: an attitude loop whose gain is a fifth higher than
the controller believes, by an amount that grows with the command, is exactly the shape of
error that turns an adequately damped cascade into an oscillating one.

### ⚠️ Still open: the simulator needs about twice the damping of the real vehicle

None of the above fixed it. The measured position:

| | Hardware | Simulator |
|---|---|---|
| Stability wall | between `kv_xy` 4 and 5 (ζ 0.25–0.31) | between `kv_xy` 8 and 10 (ζ 0.50–0.62) |
| At `kv_xy=5` (ζ 0.31) | **flies, 2.4 cm RMSE — the best measured result** | 1.33 Hz oscillation, diverges |
| At `kv_xy=10` (ζ 0.62) | flies, 2.8–3.0 cm RMSE | flies, 18–28 mm tracking error |

1.33 Hz is the position loop's own frequency (`√kp_xy = √64 = 8 rad/s`), so this is the
outer loop ringing against an inner loop only ~2.5× faster — a cascade ratio the design
notes in `firmware_app/CLAUDE.md` acknowledge as tight (2.28×).

Both agree there **is** a wall near there; the simulator puts it about twice as far out.

#### Rotor drag — investigated and refuted

Rotor drag was the leading suspect: it is genuinely absent from the plant (the model's own
docstring says "no drag"), and it damps the position loop directly. It is now implemented
(`sim.physics.drag`, body-frame diagonal `a = -R D Rᵀ v`, folded into the body force so the
accelerometer feels it — a drag force the IMU did not see would appear as a spurious
residual). It is **not** the explanation, on two independent grounds.

*Required*: closing a ~4.5 s⁻¹ damping deficit means 0.18 N per m/s, i.e. **129 mN at
0.7 m/s — 32% of the vehicle's weight**.

*Measured*, sweeping `d` on figure8 at `kv_xy=5`:

| d [1/s] | drag at 0.7 m/s | Result |
|---|---|---|
| 0.15 (plausible) | 4.3 mN | unstable |
| 0.50 (generous) | 14.3 mN | unstable |
| 1.00 | 28.7 mN | unstable |
| **2.00** | **57.4 mN** | first stable |

A bluff-body estimate for a 10 cm frame gives ~3 mN at 0.7 m/s, and rotor drag typically
exceeds body drag by 2–3×, so the physical value is ~0.1–0.5 s⁻¹. The threshold is
**5–15× higher than anything physical**. Rotor drag is real and worth having in the model,
but it does not account for the gap.

#### Airframe — also refuted

The position gains were tuned in July on hardware; the sim is configured as CF21BL. Running
the identical case under all three airframes at `kv_xy=5`: brushless (J=2.395e-5, cascade
ratio 2.55×) unstable, upgraded CF2.1 (J=1.657e-5, 3.07×) unstable, standard CF2.1
unstable. The airframe is not the difference either.

#### Motor lag — partially explains it, and is now enabled

The plant had no actuator dynamics at all: commanded thrust appeared instantly. That is both
unfaithful (measured hardware rotor time constant is 44 ms brushless, 71 ms upgraded — and
the project's own Rust simulator carries 30 ms) and optimistic.

First-order motor lag is now in the plant (`sim.physics.motor_tau`, enabled at 0.044 s by
default). Measured effect on the stability wall:

| Motor τ | First stable `kv_xy` |
|---|---|
| none (was) | 10.0 |
| 0.030 s | **8.0** |
| 0.044 s | **8.0** (saturates) |

That closes roughly **40%** of the gap to hardware's 5.0, and the plant is more faithful for
it. `crazyflies_sim.yaml` was lowered from `kv_xy: 10.0` to `8.0` accordingly.

Note this is a plant property and cannot be derived from the controller: `indi_gains.act_tau`
is the controller's *model* of actuator lag and is deliberately 0 (the act_dyn experiment was
refuted).

#### `KI_P` — refuted

The position integral (`KI_P = 0.05`, `KI_LIMIT = 2.0`) was the leading suspect after drag.
Rebuilding with `KI_P = 0.0` gives results **bit-identical** to `0.05` (4337.8 mm tracking
error in both, same wall at `kv_xy=10`). With good tracking the error integral stays tiny and
the clamp never engages, so the term contributes nothing here.

#### Still to examine

Ruled out: trajectory generation, CSV coefficient precision, polynomial piece count, the
downwash model, missing jerk/snap, inertia mismatch, the mixer deadband, rotor drag, and
the airframe. Remaining leads, noting that the in-tree `mellinger` and `pid` fly the same
trajectory in the same plant — so the plant is flyable and the difference is specific to
our outer loop:

A useful constraint on what is left: **the remaining explanation has to be something that
makes the REAL vehicle more stable than the current model**, worth roughly 3 more units of
`kv_xy`. That rules out the two obvious remaining fidelity items — adding control/estimator
delay and adding sensor filtering both *reduce* phase margin, so they would make the sim more
faithful while widening the discrepancy, not closing it.

Candidates that would go the right way:

1. **Thrust saturation / battery sag** reducing effective loop gain at large excursions.
2. **EKF velocity smoothing** — the controller flies on an estimate, not truth. Whether that
   helps or hurts here is not obvious and is worth a direct test.
3. Whether the July campaign that produced `kv=5` used comparable trajectories and speeds to
   these test cases. If it was tuned on gentler flights, `kv=5` may simply have less margin
   than the notes imply — which the hardware crash at `kv=4` is consistent with.

**Workaround:** `crazyflies_sim.yaml` sets `kv_xy: 10.0` for simulation only, with the
rationale in-file. `crazyflies.yaml` keeps the flight-proven 5.0 and must not be changed
to match. ζ=0.625 is not invented — the tuning notes record it as the ratio held for the
whole campaign before the 2026-07-19 test lowered it.

**One thing this is worth flagging for the thesis:** hardware crashed 2 of 2 at `kv=4` and
flies at `kv=5`, so the flight configuration is sitting on a very thin margin. Formation
flight adds downwash and a second vehicle to that same loop. Whether ζ=0.31 survives close
formation is an open question the simulator is, at minimum, warning about.

---

## Verified results (22 August 2026)

Single drone, takeoff to 1.0 m, 15 s, brushless gains from `crazyflies.yaml`: all four control
modes hold **1.000 m at 0.0° tilt**.

Two drones stacked 0.5 m apart, `neuralswarm` backend, ROS end to end:

| Control | lower drone (in the wash), cmd 1.0 m | upper drone, cmd 1.5 m |
|---|---|---|
| Geometric SE(3) | **−30.5 mm** | **−2.9 mm** |
| Full INDI *(pre-fix — obsolete)* | ~~−58.8 mm~~ | ~~−6.2 mm~~ |

> ⚠️ The INDI row above was measured **before the `a_res` sign fix** and is kept only as a
> record of what the bug looked like. Position INDI was adding the residual instead of
> subtracting it, so it doubled the sag it was supposed to remove. After the fix INDI holds
> commanded geometry to under a millimetre — see
> [`12_Sim_Formation_Validation_Report.md`](12_Sim_Formation_Validation_Report.md).

The ~10x asymmetry between lower and upper is the signature of downwash rather than a trim error,
and it is the effect the thesis sets out to compensate.

**Do not read the geometric-vs-INDI difference as a result yet.** It is one run per controller,
steady hover only, with no motor lag in the plant and gains that were tuned on hardware for
trajectory tracking rather than for hover disturbance rejection. It says the pipeline works and
produces plausible physics; it does not say which controller rejects downwash better. That
comparison belongs to Section C.4 and needs the protocol in
[`05_Experimental_Protocol_2Robot.md`](05_Experimental_Protocol_2Robot.md).

An earlier version of these numbers reported INDI failing outright. That was a bad steady-window
filter in the analysis, not the controller.

---


## Residual-learning support (added 2026-08-23)

Three additions, all off unless a config asks for them. See
[`13_Residual_Learning.md`](13_Residual_Learning.md) §7 for the reasoning and the dry-run result.

| Config key | Effect |
|---|---|
| `sim.residual_log` | CSV of position, velocity, `a_res`, the network prediction **and the commanded position**, in the same schema `merge_usd_logs.py` produces from real uSD logs — so one loader serves simulation and flight with no special cases |
| `sim.residual_log_hz` | Sample rate, default 100 Hz |
| `sim.rnn_weights` | `.npz` from `tools/residual/train.py`, uploaded through the real `rnn.*` protocol |
| `sim.rnn_enable` | Whether `rnn.en` is set. Separate from loading on purpose: the useful first run records prediction against measurement *without* the model touching the vehicle |

**Peer positions are injected by the server**, since the simulator has no `peer_localization`.
Positions only, matching the real API — handing over a velocity the firmware cannot have would
make simulation easier than reality in exactly the place the residual model depends on.

**Log the commanded position, not just the realised one.** Without it, realised separation cannot
be separated from commanded motion, and any "did the formation hold its geometry" number is a
mixture of tracking error and the trajectory itself. This was got wrong once and produced a
plausible table that meant nothing.

## Why not Isaac Sim

Evaluated and rejected: it needs a discrete NVIDIA GPU, and this laptop has integrated graphics.
The rendering it would add is not what this thesis measures — the physics and the downwash model
are, and those are already here. Blender was also rejected: its Crazyswarm2 integration renders
per-drone FPV images for synthetic vision datasets, not the third-person visualisation that would
have been the reason to add it.

RViz gives the visualisation actually needed. `record_states` writes CSV/npy per drone for analysis.

See [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md) for status.
