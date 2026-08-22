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

# simulation, geometric control
ros2 launch crazyflie launch.py backend:=sim \
    crazyflies_yaml_file:=$PWD/crazyflie/config/crazyflies_sim.yaml \
    server_yaml_file:=$PWD/crazyflie/config/server_sim_geo.yaml
ros2 run crazyflie_examples run_formation --scenario A3 --dz 0.30 --yes \
    --ros-args -p use_sim_time:=true

# same scenario under full INDI: swap the server yaml for server_sim_indi.yaml
```

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

### ⛔ What is NOT verified: moving scenarios in simulation

**The simulator cannot yet validate any scenario that moves.** Our out-of-tree controller
does not track high-level-commander trajectories in SIL — it diverges in attitude and
crashes. This is **not** caused by the formation library. Measured on the same harness,
geometric control, no downwash:

| Trajectory | Max tilt |
|---|---|
| Stock `circle_mode1_kt0.05` (pre-existing, unmodified) | **139°** |
| Stock `figure8_mode1_kt0.05` | **79°** |
| Stock `oval_mode1_kt0.05` | **168°** |
| A formation-library shuttle | 79° |
| Hover | 0° |
| **Same stock circle under in-tree `mellinger`** | **27°** ✅ |
| **Same stock circle under in-tree `pid`** | **28°** ✅ |

The pre-existing exported trajectories fail *worse* than the generated ones, and the
in-tree controllers fly the identical file correctly at a physically sensible 27° bank. So
the defect is in our controller's trajectory path under simulation, and it predates this
work — the simulator had only ever been exercised on hover, takeoff and `goTo`, never on a
trajectory.

Ruled out so far: CSV coefficient precision (fixed, but the instability is identical when
the CSV is bypassed entirely); piece count (79° at 2, 4, 8 and 16 pieces); the downwash
model (fails with interaction disabled); and missing jerk/snap in the setpoint — the SIL
was indeed dropping both where `crtp_commander_high_level.c` sets them, and that is now
fixed, but the numbers did not move because `g_indi_omega_src` defaults to 0 and takes the
body rate from the setpoint instead.

Remaining leads, in order: the `attitudeRate` the SIL derives from `ev.omega` versus what
the real HLC supplies; the position-loop gains (`kp_xy` 64 is high) under sustained lateral
acceleration; and the integral term winding up over long trajectories.

**Consequence for the plan:** the formation library is correct and safe *by construction and
offline verification*, and A1 is confirmed in sim, but the moving scenarios have not been
flown anywhere yet. Either fix the controller's sim trajectory path first, or treat the
first hardware flights as the initial validation — with the offline geometry check as the
guarantee, which is exactly what it was built for.

Trajectory fitting is exact at segment boundaries and within 2 mm between them — and the
self-test measures that **through the written CSV**, not on the in-memory fit, for the
reason below.

### One bug worth carrying forward

The first sim flight of A3 crashed. Not the interaction, not the controller: **CSV
coefficient precision.** A degree-7 coefficient scales as one over the piece duration to
the seventh power, so on a 6.5 s piece `c7 ≈ 5e-5` while `c4 ≈ 2e-2` — four orders of
magnitude apart in one row. Written with the Rust exporter's `%.6f`, `c7` keeps two
significant figures, and since it multiplies `t⁷ ≈ 5e5` the damage is enormous: a clean
**0.21 m/s²** trajectory came back out of the file at **241 m/s²**.

Fixed with `%.12g`, and `verify_csv()` now re-reads every written file and checks it
against the curve before anything is uploaded.

> ⚠️ **`export_poly4d.rs` has the same latent bug.** Its `write_cs2_csv` writes `{:.6}`
> (the onboard writer uses `{:.9}`). The induced error grows as the seventh power of the
> piece duration, so it is entirely a question of how long the pieces are:
>
> | Longest piece | Position error | Verdict |
> |---|---|---|
> | 1 s | 0.004 mm | fine |
> | 2 s | 0.13 mm | fine |
> | 3 s | 1.6 mm | marginal |
> | 4 s | 11 mm | unusable |
> | 6.5 s | 290 mm | what crashed A3 |
>
> The longest piece across all 255 exported Mode E trajectories today is **3.00 s**
> (`tilted_oval_mode1_kt*`), so the current library sits right at the marginal boundary at
> roughly 1.6 mm — small, but not nothing, and it degrades fast if segments ever get
> longer. Changing that writer to `{:.12e}` would cost nothing and remove the cliff.

The general lesson, which cost a full debugging cycle: an in-memory fit error is not
evidence the trajectory is correct. The file is what flies, so the file is what must be
verified.

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
