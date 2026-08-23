# Sim Formation Validation Report

**Last updated:** 23 August 2026
**Status: complete — 34 of 34 cases, 33 pass, 1 expected deviation, 0 defects.**

Every formation scenario in the library, run end to end in the ROS simulator under both the
geometric SE(3) and the full INDI controller, with the realised inter-robot geometry checked
automatically against what each scenario commanded.

Along the way this found **two genuine bugs in flight code**, neither of which could have
been caught flying a single drone.

```bash
python experiments/analysis/summarise_matrix.py        # audit: completeness + defects
python experiments/analysis/summarise_matrix.py --md   # the results table
```

---

## 1. The formation library

Sixteen scenarios. Each exists to drive the residual force `indi.a_res_*` through a known,
repeatable range; the geometries follow the interaction-force literature (Neural-Swarm and
Neural-Swarm2, ProxFly, tight-formation MPC).

### Two robots — Priority A

| ID | Scenario | What it isolates |
|----|----------|------------------|
| A1 | Vertical stack hover | Baseline. Nothing moves, so any displacement is interaction, not tracking error |
| A2 | Vertical stack tracking | Constant separation while both vehicles move |
| A3 | Static-top | Fixed wash source, lower vehicle sweeps through — captures entry and exit, not just the level |
| A4 | Offset stack | Partial overlap: the lower vehicle crosses the wash edge repeatedly |
| A5 | Reverse-circle stack | Counter-rotation sweeps lateral offset 0 → 2r → 0 twice per lap at constant height |
| A6 | **Extreme stack** ⚠ | Strongest interaction, smallest error budget (Δz 0.08–0.10 m) |
| A7 | Dynamic merge ⚠ | One continuous sweep of the whole separation range, 1.10 → 0.10 m |
| A8 | Vertical swap | Wash arrives as a transient with a known arrival time |

### Three robots — Priority B

| ID | Scenario | What it isolates |
|----|----------|------------------|
| B1 | I-stack | Bottom vehicle in the combined wash of **two** — tests whether pairwise superposition holds |
| B2 | V-stack | Asymmetric superposition, which B1 cannot produce |
| B3 | Vertical swap (3) | Two staggered crossings at two different height differences in one flight |

### Coverage and control cases — Priority C

| ID | Scenario | Robots | What it isolates |
|----|----------|--------|------------------|
| C1 | Side-by-side coplanar | 3 | **Control case.** A residual here and not in A1 is not downwash |
| C2 | Leader-follower line | 3 | Wake rather than wash |
| C3 | Equilateral triangle | 3 | Coplanar three-body control case |
| C4 | Docking approach ⚠ | 2 | Simultaneous lateral and vertical closure (ProxFly-style) |
| C5 | Near-ground pass | 1 | Ground effect alone, so it can be separated from downwash |

⚠ = refused unless `--allow-extreme`.

---

## 2. Results — all 34 cases

**Pass criteria:** mean |Δz error| < 50 mm, mean horizontal error < 80 mm, no divergence
(automatic failure above 60° tilt), and ≥ 90% of the trajectory recorded. Every run below
was 100% recorded and the largest tilt anywhere was 6.4° — the commanded bank for A5's
circle.

### Two robots

| Scenario | geometric | INDI |
|---|---|---|
| A1 Δz = 0.50 m | PASS — 27.6 mm | **PASS — 0.0 mm** |
| A1 Δz = 0.25 m | PASS — 41.5 mm | **PASS — 0.0 mm** |
| A2 tracking | PASS — 38.4 mm | **PASS — 0.1 mm** |
| A3 static-top | PASS — 3.5 mm | **PASS — 0.6 mm** |
| A4 offset stack | PASS — 2.0 mm | **PASS — 0.2 mm** |
| A5 reverse-circle | PASS — 4.2 mm | **PASS — 0.5 mm** |
| A6 extreme Δz = 0.10 m | PASS — 3.2 mm | **PASS — 0.4 mm** |
| A7 merge 1.10 → 0.10 m | PASS — 25.5 mm | **PASS — 0.3 mm** |
| A8 vertical swap | PASS — 2.1 mm | **PASS — 0.3 mm** |
| C4 docking | PASS — 7.2 mm | **PASS — 0.2 mm** |

### Three robots

| Scenario | geometric | INDI |
|---|---|---|
| B1 I-stack | **EXPECTED — 58.7 mm** (see §4) | **PASS — 0.1 mm** |
| B2 V-stack | PASS — 42.6 mm | **PASS — 0.1 mm** |
| B3 vertical swap | PASS — 3.9 mm | **PASS — 0.7 mm** |
| C1 side-by-side | PASS — 0.0 mm | **PASS — 0.0 mm** |
| C2 leader-follower | PASS — 0.0 mm | **PASS — 0.0 mm** |
| C3 triangle | PASS — 0.0 mm | **PASS — 0.0 mm** |

### One robot

| Scenario | geometric | INDI |
|---|---|---|
| C5 near-ground pass | PASS (no divergence) | PASS (no divergence) |

C5 reports no separation error because a single vehicle has no pair to measure. It verifies
only that the scenario flies; the ground-effect measurement itself is a hardware result.

### The control cases are the ones to look at first

**C1, C2 and C3 come out at 0.0 mm under both controllers.** Those are coplanar — same
height, no vehicle above another, so there should be no downwash. If they had shown sag,
the interaction model would be producing force where none belongs and every positive result
in the table would be suspect. They don't, so the positives mean something.

---

## 3. What the numbers say about the two controllers

Geometric sags in proportion to wash exposure; INDI holds the commanded geometry to under a
millimetre in every scenario:

| Wash exposure | geometric |
|---|---|
| none (coplanar C1/C2/C3) | 0.0 mm |
| one source, 0.50 m above | 27.6 mm |
| one source, 0.25 m above | 41.5 mm |
| two sources, laterally offset (B2) | 42.6 mm |
| two sources, vertically aligned (B1) | 58.7 mm |

Monotonic in every direction it should be: more sources, closer, better aligned → more sag.
That progression is itself evidence the interaction model is behaving sensibly.

> ### ⚠️ This is not yet a thesis result
>
> INDI improving on geometric by two to three orders of magnitude **must not be quoted as a
> controller comparison.** In simulation the interaction model is Neural-Swarm2's pretrained
> network fitted to someone else's vehicle; there is no EKF, no sensor noise and no radio
> latency; the simulator runs `kv_xy` 8.0 where hardware runs 5.0; and INDI is cancelling a
> disturbance the simulator itself generates, which is a friendlier problem than reality.
>
> What this **does** establish: the scenarios are correct, the multi-robot plumbing works,
> both controllers fly all sixteen, and the compensation path is now wired the right way
> round. A controller comparison is a hardware result.

---

## 4. The one expected deviation — B1 under geometric

B1 under geometric misses the 50 mm tolerance at 58.7 mm. **This is not a defect and is not
reported as a failure.** Geometric control has no mechanism to reject downwash, so in a
scenario dominated by it the tracking error *is* the disturbance. Calling that a failure
would say something false about the software; calling it a pass would discard the most
interesting measurement in the table.

`EXPECTED` is a verdict that has to be earned. A case only qualifies when **all** of these
hold, and the audit checks them:

1. it is listed with a written reason;
2. the run did not diverge and was fully recorded;
3. **the same scenario under INDI is within tolerance.**

Condition 3 is what makes it safe rather than a rubber stamp — if the scenario or the
geometry were broken, INDI would fail too and the case reverts to a real failure. B1 under
INDI comes in at 0.1 mm, so the scenario is sound and only geometric lacks the mechanism.

---

## 5. Two bugs found in flight code

### The residual was added instead of subtracted

From `m·a = f_thrust + f_res + m·g`, holding a desired acceleration needs thrust
`m·a_des − m·g_vec − f_res`, so the desired-acceleration vector carries **minus** the
residual. It was being added — so position INDI *reinforced* every unmodelled force instead
of rejecting it.

Measured against a known 20 mN downward force:

| controller | before | after |
|---|---|---|
| geometric (no residual feedback) | 10.2 mm sag | 10.2 mm |
| position INDI | **20.3 mm — exactly 2.00×** | **0.0 mm — cancels** |

The same factor appeared in every formation scenario, 1.89× to 2.10×. The measurement
itself was always correct: −0.4871 m/s² against an exact −0.4878.

> **Why no flight caught it.** With no other vehicle nearby `a_res ≈ 0`, and the sign of a
> zero term is invisible. It only manifests once another drone's downwash exists — exactly
> the regime this thesis is about, and exactly what has never been flown.

### `a_res` was dead whenever geometric was selected

RPM was read only when `mode != 0`, so the residual was forced to zero under geometric
control. That silently defeated the intent stated in the comment thirty lines below it: the
residual must be measurable while flying geometric, because **the training data for
Geometric + NN has to come from non-INDI flights.** As written, that path produced nothing.

Reading RPM in every mode does not change geometric's control law — `a_indi` stays zero
there — only what is logged.

**Both fixes are unflown.** Correct by derivation and by simulation, but the first changes a
control term on real hardware. Treat the first INDI flight after it as a new configuration.

---

## 6. Infrastructure fixed to make the results trustworthy

| Problem | Effect if left |
|---|---|
| SIL fed the controller **commanded** RPM while the plant used **lagged actual** RPM | Attitude INDI reconstructed torque that was never applied; neither drone left the ground |
| Scenarios did not record all their parameters | A4 omitted `laps`, so verification rebuilt a two-lap reference and scored a one-lap flight against it — a confident 208 mm comparison against the wrong trajectory |
| Verification paired a run with the **newest** sidecar, not **its own** | A run that died before writing one was scored against the previous scenario's geometry |
| A truncated recording could pass on the fraction captured | `--min-coverage 0.9`; coverage reported on every row |
| Poly4D CSV written with `%.6f` | On a 6.5 s piece, a 0.21 m/s² trajectory came back out of the file at 241 m/s² and flew the vehicle into the ground |

The recurring lesson: **a plausible-looking number is the dangerous failure mode.** Every
one of these produced a confident, physically-shaped value that was simply comparing the
wrong two things.

**The simulator is deterministic** — all nine geometric cases reproduced to 0.1 mm across a
rebuild. A re-run is a genuine regression test, so only failures need repeating unless the
simulator itself changes.

---

## 7. Flight volume — all sixteen scenarios fit

**x −1.0 … 1.0 m, y −2.0 … 2.0 m, z 0.30 … 1.70 m.**
⚠️ Operator estimate (23 August 2026), **not a tape-measure result** — though corroborated: it
matches a figure stated independently a month earlier (z and x identical, y then ±2.1 against
±2.0 now; the conservative value is applied). Confirm before flying: several scenarios sit
within ~10 cm of these walls, and no margin is subtracted — the check runs on *commanded*
positions, so real tracking error sits on top.

The room is **long in y (4 m) and narrow in x (2 m)**, but every translating scenario was
defined along x. Two options fix that:

| placement | scenarios that fit |
|---|---|
| anchored on drone 0 | 11 / 19 |
| `--auto-center` | 18 / 19 |
| `--auto-center --rotate 90` | **19 / 19 at full default parameters** |

`--rotate` turns the whole formation about the vertical axis. The rotation is rigid, so
every inter-robot distance is preserved exactly — verified numerically, and `check_spec`
un-rotates before testing since its assertions are written in the scenario's own frame.
Recommended invocation:

```bash
ros2 run crazyflie_examples run_formation --scenario A3 --dz 0.30 --auto-center --rotate 90
```

---

## 8. Where the thesis stands

### Prepared

| Area | State |
|---|---|
| Planning, literature, protocol | Complete — 7 control strategies defined, 2-robot protocol written |
| Mode D → Mode E migration | Complete, Mode D frozen byte-identical |
| Residual measurement `a_res` | Implemented, logged in every controller mode **(bug fixed today)** |
| Multi-drone flight script + uSD logging | Complete, offline-verified |
| Simulation with the real controllers | Complete — geometric and all three INDI modes, same Rust source that flies |
| **Formation library** | **16 scenarios, all validated in sim under both controllers** |
| Verification tooling | Automated commanded-vs-realised check, completeness audit |

### Left before the core thesis work

| # | Item | Why it blocks |
|---|---|---|
| 1 | **Confirm the flight volume** with a tape measure | Most load-bearing unverified number in the project |
| 2 | Resolve the `crazyflie-firmware` bindings | ~110 uncommitted lines; the simulator does not build without them |
| 3 | Hardware inventory | Working brushless drones, decks, batteries, SD decks |
| 4 | **Fly the single-robot ladder** | Mode E has *never flown*. Flash → figure8 Mode D → figure8 Mode E → circle |
| 5 | Re-validate INDI after the sign fix | It changes a control term; hover before anything else |
| 6 | First 2-robot flights at large separation | A1 at Δz 0.75 → 0.50, then A3 |
| 7 | Confirm `a_res` is non-zero in flight | Needs an RPM source; zero means no thesis data |
| 8 | Freeze the gains | Re-tuning per controller would measure tuning effort, not the methods |

Only then does the core experimental work start: collect residual data → train the NN models
→ integrate the seven methods → run the systematic comparison.

### Known open items, not blocking

- The simulator needs ~2× hardware's position damping (`kv_xy` 8 vs 5). Motor lag closed
  ~40% of it; `KI_P`, rotor drag, inertia and airframe are all refuted. Time-boxed, paused.
- Library coverage gaps: yaw is fixed at zero everywhere, and there is no vertical relative
  motion except A7/C4.
- Nothing above three robots.

---

## 9. Ready for hardware? **Not yet — but nothing in simulation is blocking**

The simulation work is done and the answer is positive: all sixteen formations fly under
both controllers with no defects. Items 1–4 above stand between here and a first flight, and
none of them is a simulation problem.

**Two cautions to carry into the lab.** The sign fix is unflown and changes a control term.
And the hardware margin is thin — `kv_xy = 4` crashed 2 of 2 flights and 5 is what flies,
while formation flight adds downwash to that same loop.

---

## Appendix — reproducing

```bash
experiments/analysis/run_sim_matrix.sh all              # whole matrix
experiments/analysis/rerun_failures.sh --dry-run        # only what is not passing
experiments/analysis/run_sim_matrix.sh one indi 3 --scenario B1 --dz2 0.30 --path line
python experiments/analysis/probe_residual_sign.py      # the check that found the sign bug
```

Two yaml files select two different things: the **roster** picks the robot count
(`crazyflies_sim1/sim/sim3.yaml` = 1/2/3) and the **server** picks the controller
(`server_sim_geo.yaml` / `server_sim_indi.yaml`). The controller cannot be chosen from the
runner — it is a client, and the controller lives in the server process.
