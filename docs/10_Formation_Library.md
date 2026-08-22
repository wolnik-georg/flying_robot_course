# Formation Scenario Library

**Last updated:** 22 August 2026

The set of multi-robot geometries used to excite inter-vehicle aerodynamic interaction,
and how to run them. This is the experimental input side of the thesis: every scenario
exists to drive the residual force `indi.a_res_*` through a known, repeatable range so
that the compensation methods in
[`01_Thesis_Project_Snapshot.md`](01_Thesis_Project_Snapshot.md) can be compared on
identical excitation.

Code: `crazyswarm2/crazyflie_examples/crazyflie_examples/formations/` with the runner
`run_formation.py`. Implementation detail lives in that folder's `README.md`; this
document is the thesis-side view — what the scenarios are, why each one is in the set,
and what is still missing.

> Numbering note: `08` and `09` were already taken by the trajectory-upload and simulation
> documents, so this is `10` rather than the `08` originally sketched.

---

## Why these scenarios

Downwash is a function of relative geometry: mostly vertical separation, secondarily
lateral overlap, and — for anything that moves — the *rate* at which those change. A
useful scenario set has to cover all three, and has to include cases where the interaction
is known to be near zero, or there is nothing to compare a positive result against.

| What it isolates | Scenarios |
|---|---|
| Steady vertical separation, no motion | A1 |
| Steady separation while both vehicles move | A2, B1, B2 |
| Wash entry and exit as a transient | A3, A6 |
| Partial / edge overlap | A4 |
| Full sweep of lateral overlap at fixed height | A5 |
| Separation changing continuously | A7, C4 |
| Fast transient crossing | A8, B3 |
| Multi-body superposition | B1, B2 |
| **Near-zero interaction (control cases)** | C1, C3 |
| Wake rather than wash | C2 |
| Ground effect, so it can be told apart from downwash | C5 |

The geometries follow the interaction-force literature — Neural-Swarm and Neural-Swarm2
(vertical stacks, swaps, small separations), ProxFly (docking approach), and the
tight-formation MPC work — but only the geometries. The controllers under test are ours.

---

## The set

`n` is the robot count the scenario requires; the runner refuses a mismatched roster.
Separations are always parameters, never constants; the defaults are the literature values.

### Priority A — two robots

| ID | Scenario | Swept parameter | What it is for |
|----|----------|-----------------|----------------|
| A1 | Vertical stack hover | dz ∈ {0.25, 0.50, 0.75} | Baseline. Nothing moves, so any relative displacement is interaction, not tracking error |
| A2 | Vertical stack tracking | dz ∈ {0.20…0.50}, path line/circle | Constant separation while both move — separates interaction from the trivial hover case |
| A3 | Static-top | dz ∈ {0.20, 0.30, 0.40} | Cleanest excitation: fixed wash source, lower vehicle sweeps through. Captures the transition, not just the level |
| A4 | Offset stack | dz ∈ {0.50, 0.60, 0.80}, offset ∈ {0.10, 0.20} | Partial overlap — asymmetric and hardest to model |
| A5 | Reverse-circle stack | dz = 0.50 | Counter-rotation sweeps lateral offset 0 → 2r → 0 twice per lap at constant height and speed |
| A6 | **Extreme stack** ⚠ | dz ∈ {0.08, 0.10} | Strongest interaction, smallest error budget |
| A7 | Dynamic merge ⚠ | dz 1.10 → 0.10 | One continuous sweep of the whole separation range — gives a model the gradient, not just levels |
| A8 | Vertical swap | dz ∈ {0.20, 0.25} | Wash arrives as a transient with a known arrival time; hardest case for a feed-forward model |

### Priority B — three robots

| ID | Scenario | Swept parameter | What it is for |
|----|----------|-----------------|----------------|
| B1 | I-stack | dz2 ∈ {0.20, 0.30, 0.40} | Bottom vehicle in the combined wash of two — tests whether pairwise superposition holds |
| B2 | V-stack | dz2 ∈ {0.30, 0.40}, r = 0.10 | Asymmetric superposition, which B1 cannot produce |
| B3 | Vertical swap (3) | dz ∈ 0.20–0.25 | Three crossings at three height differences in one flight |

### Priority C — coverage

| ID | Scenario | What it is for |
|----|----------|----------------|
| C1 | Side-by-side coplanar | **Control case.** A residual here and not in A1 is not downwash |
| C2 | Leader-follower line | Wake rather than wash |
| C3 | Equilateral triangle | Coplanar three-body control case |
| C4 | Docking approach ⚠ | Simultaneous lateral and vertical closure (ProxFly-style) |
| C5 | Near-ground pass, 1 robot | Ground effect alone, so it can be separated from downwash at low altitude |

⚠ = refused unless `--allow-extreme`.

---

## Running

Same command in simulation and on hardware; only the launch differs.

```bash
# verify without flying: builds, checks the spec, runs the safety gate, writes the CSVs
ros2 run crazyflie_examples run_formation --scenario A3 --dz 0.30 --check

# --- simulation: TWO yaml files, and each one selects something different --------
#   crazyflies_yaml_file  -> HOW MANY ROBOTS      (sim = 2, sim3 = 3)
#   server_yaml_file      -> WHICH CONTROLLER     (geo = geometric, indi = full INDI)
ros2 launch crazyflie launch.py backend:=sim \
    crazyflies_yaml_file:=$PWD/crazyflie/config/crazyflies_sim.yaml \
    server_yaml_file:=$PWD/crazyflie/config/server_sim_geo.yaml

ros2 run crazyflie_examples run_formation --scenario A3 --dz 0.30 --yes \
    --ros-args -p use_sim_time:=true

# three robots (B1/B2/B3, C1-C3): swap the roster
#   crazyflies_yaml_file:=$PWD/crazyflie/config/crazyflies_sim3.yaml
# full INDI: swap the server
#   server_yaml_file:=$PWD/crazyflie/config/server_sim_indi.yaml
```

**The controller is chosen by the server yaml, not by the runner.** `run_formation` is a
client; the controller lives in the server process. Its `setParam` calls are what would
select the mode on hardware, and they fail harmlessly in simulation (the `keyError`
warnings in the client log are expected). The server prints which law it is running at
startup — check that line rather than assuming.

### Verifying a simulated run

The runner's own post-flight report does not work in simulation: the radio log topics it
subscribes to do not exist there, and `/pose` is published by the hardware C++ server, not
the simulated one. So the runner writes a sidecar `<SID>_<stamp>.meta.json` recording the
exact simulation-clock instant the trajectory started — which matters, because for a
pure-hover scenario like A1 no heuristic could recover that window from the data — and a
separate tool checks the recorded states against it:

```bash
python experiments/analysis/verify_formation_sim.py \
    experiments/logs/A3_<stamp>.meta.json  state_geo/<timestamp>/csv  --controller geo
```

It reports, per robot pair, commanded vs realised dx/dy/dz with mean, standard deviation,
max error and RMSE, and a PASS/FAIL against a stated tolerance.

Swapping only the server yaml is what makes the library useful — identical commanded
excitation, different control law, and the difference is attributable. See
[`09_Simulation.md`](09_Simulation.md), and note its warning: the simulator has no motor
lag, no EKF and no sensor noise, so it can show a scenario is broken but not that a
controller is good.

Every scenario writes per-drone CSVs to `experiments/logs/` with the scenario id and all
parameters in the metadata header, and prints realised-versus-commanded relative geometry
on landing.

---

## Verification

### What IS verified

`python -m crazyflie_examples.formations.scenarios --self-test` builds 38 parameter
combinations, checks each against the geometry its own docstring claims, compiles every
trajectory through the written CSV, and reports piece count and fit error. All 38 pass.
Every scenario also passes the runner's `--check`, which adds the safety gate: **31
parameter combinations, 0 failures**, with the extreme scenarios correctly refused when
`--allow-extreme` is absent. Hover scenarios (A1) fly correctly in the simulator.

### ROS-sim verification status

| Layer | Status |
|---|---|
| Geometry vs each scenario's own spec (offline, through the written CSV) | ✅ 38/38 parameter cases |
| Safety gate (speed, accel, separation, geofence, extreme gating) | ✅ 20/20 combinations |
| **ROS-sim end-to-end, geometric** | 🔄 matrix running |
| **ROS-sim end-to-end, full INDI** | 🔄 matrix running |
| 3-robot scenarios (B1/B2/B3, C1–C3) | 🔄 unblocked by `crazyflies_sim3.yaml`, in the matrix |
| Hardware | ⬜ nothing has flown |

Live results: `experiments/sim_validation/matrix_results.md`. Full write-up when the matrix
finishes: `docs/12_Sim_Formation_Validation_Report.md`.

**Pass criteria:** mean |Δz error| < 50 mm, mean horizontal error < 80 mm, no divergence
(automatic fail above 60° tilt), and the fraction of the trajectory actually recorded is
reported so a run cut short by the server timeout cannot masquerade as a pass.

One calibration point worth keeping in mind when reading the numbers: A1 at Δz 0.50 m
reports 27.6 mm of error under geometric control, and that is almost entirely the lower
vehicle **sagging in the wash** — it independently matches the −30.5 mm measured in the
standalone downwash test. It is physics being measured correctly, not a tracking failure,
and it consumes half the tolerance budget before the scenario does anything.

### Moving scenarios in simulation

Moving scenarios exercise the trajectory path, which had never been run in simulation
before this work — only hover, takeoff and `goTo`. Doing so exposed a controller/simulator
problem that is **not** caused by the formation library: the pre-existing exported
trajectories failed *worse* than the generated ones (stock `oval` 168° of tilt, stock
`circle` 139°, a library shuttle 79°), while the in-tree `mellinger` flew the identical
`circle` file at a correct 27° bank.

Chasing it produced four real fixes — missing jerk/snap in the SIL setpoint, a hybrid
CF2.1/CF21BL airframe in the host build, the plant using the wrong inertia, and a mixer
deadband delivering 21% excess torque — and one unresolved gap: the simulator needs about
twice the position damping of the real vehicle. Full account in
[`09_Simulation.md`](09_Simulation.md).

With the sim-only `kv_xy: 10.0` in `crazyflies_sim.yaml`, the library trajectories track
essentially perfectly — the A3 shuttle to **0.4 mm** and the A5 circle to **0.7 mm**, at
1.2° and 3.2° of tilt respectively (3.1° is the correct bank for A5's centripetal
acceleration). The stock trajectories track to 18–28 mm, their larger tilt coming from the
wind-up ramps they carry.

---

## Open items

**The geofence is a placeholder.** `FLIGHT_SPACE` in `safety.py` is set to a box smaller
than any plausible lab, deliberately, because the real flight volume is recorded nowhere in
the repo and guessing a number that decides where a vehicle may fly is not a guess worth
making silently. The failure mode is a refused scenario rather than a vehicle in the
netting. **Measure the volume and set it**, or pass `--geofence`.

**Coverage gaps, in rough order of how much they would add:**

1. **Yaw is fixed at zero everywhere.** A yawing vehicle changes its wake orientation;
   whether that matters at these separations is untested. Cheap to add — the pipeline
   already carries a yaw channel end to end.
2. **No vertical (z-axis) relative motion except A7/C4.** A vehicle climbing through
   another's wash is a distinct case from translating through it, and arguably more
   demanding since the wash acts along the direction of travel.
3. **Nothing above three robots.** B1/B2 test superposition for two sources; whether it
   still holds for three or four is the natural next question, and the thesis plan already
   contemplates ≥3 robots.
4. **No sustained close-proximity flight.** Every scenario is short. A long formation
   flight would show drift and thermal effects that a 20-second run cannot.
5. **No deliberate disturbance rejection** (a step in commanded separation while close).

**My assessment: the set is complete for the two- and three-robot comparison the thesis
protocol specifies, and I would not add more before flying.** A1–A5 plus A8 cover the
separation/overlap/transient space for two vehicles, B1–B3 add superposition, and C1/C3/C5
supply the control cases without which a positive result is not interpretable. Items 1–2
are the ones I would add first if the first flights suggest the residual depends on
something the current set holds fixed; items 3–5 belong after the two-robot comparison is
done, not before.

---

## Relationship to `formation_flight.py`

`formation_flight.py` flies **one** trajectory shared by every drone in a fixed formation
— still the right tool for "hold a stack and fly a figure-8". `run_formation.py` gives each
drone its **own** trajectory, which is what a hovering vehicle above a translating one, a
counter-rotating pair, or a swap requires. Both use the same Crazyswarm2 upload path and
neither changes it: `uploadTrajectory` is per-drone, `startTrajectory` is a broadcast, so
distinct trajectories under one id all start on a single packet.

See [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md) for status.
