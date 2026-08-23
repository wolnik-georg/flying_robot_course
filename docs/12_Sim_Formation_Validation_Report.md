# Sim Formation Validation Report

**Last updated:** 23 August 2026

Every Priority A and B formation scenario, run end to end in the ROS simulator under both
the geometric SE(3) and the full INDI controller, with the realised inter-robot geometry
checked automatically against what each scenario commanded.

**Result: 20 of 20 cases have a result, 19 pass, 1 fails — and the one failure is correct
physics, not a defect.** Two genuine bugs in flight code were found on the way, neither of
which could have been caught in single-drone flight.

Reproduce the audit at any time:

```bash
python experiments/analysis/summarise_matrix.py        # completeness + failures
python experiments/analysis/summarise_matrix.py --md   # the table below
```

---

## 1. Infrastructure fixes

Six problems had to be fixed before the matrix could run at all or be trusted. Four are in
the simulator or harness; **two are in flight code and matter beyond simulation**.

### In flight code

**The residual was added instead of subtracted** (`firmware_app/src/lib.rs`). From
`m·a = f_thrust + f_res + m·g`, holding a desired acceleration needs thrust
`m·a_des − m·g_vec − f_res`, so the desired-acceleration vector carries **minus** the
residual. It was being added, so position INDI *reinforced* every unmodelled force instead
of rejecting it.

Measured against a known 20 mN downward force: geometric sagged 10.2 mm with no residual
feedback; position INDI sagged **20.3 mm — exactly 2.00×**, when it should have sagged
less. The same factor appeared in every formation scenario (1.89× to 2.10× across hover,
tracking, swap and lemniscate). After the fix both INDI modes hold the commanded height
exactly, 0.0 mm, with the measured residual unchanged at −0.4871 m/s² against an expected
−0.4878.

> **Why no flight caught this.** With no other vehicle nearby `a_res ≈ 0`, and the sign of
> a zero term is invisible. It only manifests once another drone's downwash is present —
> exactly the regime this thesis is about, and exactly what has never been flown.

**`a_res` was dead whenever the geometric controller was selected.** RPM was only read when
`mode != 0`, so `rpms_active` was false and the residual was forced to zero. That silently
defeated the intent stated in the comment thirty lines below it: the residual is meant to be
measurable while flying geometric, because **the training data for Geometric + NN has to
come from non-INDI flights**. As written, that path produced nothing. Reading RPM in every
mode does not change geometric's control law — `a_indi` stays zero there — only what is
logged.

### In the simulator and harness

| Problem | Effect |
|---|---|
| The SIL fed the controller **commanded** RPM while the plant used **lagged actual** RPM | Attitude INDI reconstructs applied torque from rotor speed, so it computed increments against a torque never produced. Neither drone left the ground. The RPM deck measures actual speed, so the plant now publishes the post-lag value |
| Scenarios did not record all their parameters | A4 omitted `laps`, so verification rebuilt a two-lap reference and scored a one-lap flight against it — a confident 208 mm comparison against the wrong trajectory. `build()` now derives the parameter set from the function signature |
| Verification paired a run with the **newest** sidecar, not **its own** | A run that died before writing one was scored against the previous scenario's geometry. Now each run must produce a sidecar newer than a pre-run marker |
| A truncated recording could pass on the fraction captured | `--min-coverage 0.9`; coverage is reported on every row |

---

## 2. Two-robot matrix (Priority A)

| Scenario | Ctrl | Verdict | mean \|e_z\| | RMSE | max tilt | covered |
|---|---|---|---|---|---|---|
| A1 Δz=0.50 | geo | **PASS** | 27.6 mm | 27.6 mm | 0.0° | 100% |
| A1 Δz=0.25 | geo | **PASS** | 41.5 mm | 41.5 mm | 0.0° | 100% |
| A2 | geo | **PASS** | 38.4 mm | 38.4 mm | 1.2° | 100% |
| A3 | geo | **PASS** | 3.5 mm | 8.0 mm | 1.8° | 100% |
| A4 | geo | **PASS** | 2.0 mm | 3.3 mm | 4.0° | 100% |
| A5 | geo | **PASS** | 4.2 mm | 9.5 mm | 6.4° | 100% |
| A8 | geo | **PASS** | 2.1 mm | 4.0 mm | 1.7° | 100% |
| A1 Δz=0.50 | INDI | **PASS** | 0.0 mm | 0.0 mm | 0.0° | 100% |
| A1 Δz=0.25 | INDI | **PASS** | 0.0 mm | 0.0 mm | 0.0° | 100% |
| A2 | INDI | **PASS** | 0.1 mm | 0.1 mm | 1.2° | 100% |
| A3 | INDI | **PASS** | 0.6 mm | 1.6 mm | 1.2° | 100% |
| A4 | INDI | **PASS** | 0.2 mm | 0.7 mm | 4.1° | 100% |
| A5 | INDI | **PASS** | 0.5 mm | 1.4 mm | 6.0° | 100% |
| A8 | INDI | **PASS** | 0.3 mm | 1.1 mm | 1.2° | 100% |

## 3. Three-robot matrix (Priority B)

| Scenario | Ctrl | Verdict | mean \|e_z\| | RMSE | max tilt | covered |
|---|---|---|---|---|---|---|
| B1 I-stack | geo | **FAIL** | 58.7 mm | 58.7 mm | 1.2° | 100% |
| B2 V-stack | geo | **PASS** | 42.6 mm | 42.7 mm | 1.2° | 100% |
| B3 swap | geo | **PASS** | 3.9 mm | 7.7 mm | 0.8° | 100% |
| B1 I-stack | INDI | **PASS** | 0.1 mm | 0.1 mm | 1.2° | 100% |
| B2 V-stack | INDI | **PASS** | 0.1 mm | 0.1 mm | 1.2° | 100% |
| B3 swap | INDI | **PASS** | 0.7 mm | 1.6 mm | 0.7° | 100% |

**Pass criteria:** mean |Δz error| < 50 mm, mean horizontal error < 80 mm, automatic
failure above 60° tilt, and ≥ 90% of the trajectory recorded. No run diverged; the largest
tilt anywhere was 6.4°, which is the commanded bank for A5's circle.

---

## 4. Failures and root causes

### B1 geometric, 58.7 mm — correct physics, not a defect

The only outstanding failure. It is geometric control failing to reject a disturbance it
has **no mechanism** to reject, and the magnitude follows the geometry exactly:

| Case | Wash sources above | Sag |
|---|---|---|
| A1 Δz=0.50 | one | 27.6 mm |
| A1 Δz=0.25 | one, closer | 41.5 mm |
| B2 | two, laterally offset | 42.6 mm |
| **B1** | **two, vertically aligned** | **58.7 mm** |

More sources, closer, better aligned → more sag, monotonically. The same scenario under
INDI comes out at **0.1 mm**, so the geometry and the trajectory are fine; what the number
measures is the downwash itself.

**The tolerance has deliberately not been raised to make this pass.** For a controller with
no disturbance rejection, the tracking error in a downwash scenario *is* the disturbance,
and stamping it PASS would discard the most interesting measurement in the table. Recorded
as a known, understood, expected failure.

### Everything else that failed during the campaign was a measurement bug

Every other failure seen while building this turned out to be in the verification path, not
the flight: A4's 208 mm (wrong reference trajectory), A1's 208.5 mm (wrong sidecar), the
`nan` row (recording ended before the trajectory began), and the INDI runs that never left
the ground (commanded rather than measured RPM). All are fixed and all now pass.

The general lesson, which cost most of the debugging time: **a plausible-looking number is
the dangerous failure mode.** Every one of those produced a confident, physically-shaped
value that was simply comparing the wrong two things.

---

## 5. What this shows about the controllers

INDI reduces the residual displacement by **two to three orders of magnitude** relative to
geometric on every downwash-dominated scenario — 27.6 → 0.0 mm on a stacked hover,
58.7 → 0.1 mm on the three-robot stack that geometric cannot hold.

**This is not yet a thesis result, and should not be quoted as one.** In simulation:

- the interaction model is Neural-Swarm2's pretrained network, fitted to someone else's
  vehicle;
- the plant has first-order motor lag but no EKF, no sensor noise, no radio latency;
- the simulator runs `kv_xy` 8.0 where hardware runs 5.0, because the sim needs about twice
  the position damping for reasons not yet understood ([`09_Simulation.md`](09_Simulation.md));
- INDI is compensating a disturbance the simulator itself generates, which is a friendlier
  problem than reality.

What it *does* establish: the scenarios are correct, the multi-robot plumbing works, both
controllers fly all of them, and the compensation path is now wired the right way round.
A controller comparison is a hardware result.

---

## 6. Remaining gaps

| Gap | Status |
|---|---|
| **Nothing has flown** | Every number here is simulated |
| Sim needs ~2× hardware's position damping | Open, time-boxed, paused — motor lag closed ~40%, `KI_P` / rotor drag / inertia / airframe all refuted |
| Geofence is a placeholder | **Measure the flight volume** before any real flight |
| `crazyflie-firmware` bindings uncommitted | ~110 lines, patch preserved at `firmware_app/host/cffirmware_bindings.patch` |
| A6, A7, C1–C5 not in the matrix | Deliberate — extreme and coverage scenarios come after the required set |
| Yaw fixed at zero; no vertical relative motion except A7/C4 | Coverage gaps in the library itself ([`10_Formation_Library.md`](10_Formation_Library.md)) |

---

## 7. Ready for hardware? **Not yet — three items first**

The simulation work is done and the answer it gives is positive. Three things stand between
here and a first flight, none of them simulation:

1. **Measure the flight volume** and set `FLIGHT_SPACE` in `formations/safety.py`. It is
   deliberately smaller than any real lab, so scenarios are refused rather than flown into
   the netting — but that means it is wrong on purpose and must be replaced.
2. **Resolve the firmware bindings.** The simulator does not build without ~110 uncommitted
   lines in a tree that tracks bitcraze upstream.
3. **Fly the single-robot ladder** in
   [`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md): flash →
   figure8 Mode D → figure8 Mode E → circle. Mode E has never flown; that gate comes before
   any second vehicle.

Then two robots at **large separation only** — A1 at Δz 0.75 → 0.50, A3 at 0.40 → 0.30.
Nothing below 0.5 m on the first day.

### Two things to carry into the lab

**The sign fix is unflown.** It is correct by derivation and by measurement in simulation,
but it changes a control term on real hardware. Treat the first INDI flight after it as a
new configuration, not a continuation — hover first.

**The hardware margin is thin.** `kv_xy = 4` crashed 2 of 2 flights and 5 is what flies.
Formation flight adds downwash to that same loop, and the simulator — even discounting its
2× discrepancy — oscillates at the position loop's own frequency at these gains.

---

## Appendix — reproducing

```bash
# whole matrix, ~1 h
experiments/analysis/run_sim_matrix.sh all

# only what is not passing
experiments/analysis/rerun_failures.sh --dry-run
experiments/analysis/rerun_failures.sh

# a single case
experiments/analysis/run_sim_matrix.sh one indi 3 --scenario B1 --dz2 0.30 --path line

# the residual-sign check that found the flight-code bug
python experiments/analysis/probe_residual_sign.py
```

Two yaml files select two different things: the **roster** picks the robot count
(`crazyflies_sim.yaml` = 2, `crazyflies_sim3.yaml` = 3) and the **server** picks the
controller (`server_sim_geo.yaml` / `server_sim_indi.yaml`). The controller cannot be
chosen from the runner — it is a client, and the controller lives in the server process.

**The simulator is deterministic.** All nine geometric cases reproduced to 0.1 mm across a
rebuild, so a re-run is a genuine regression test rather than a resample.
