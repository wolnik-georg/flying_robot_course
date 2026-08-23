# Hardware Readiness Checklist

**Last updated:** 22 August 2026

Everything that must be true before the first real flights, and the order to fly them in.
This is a preparation document — it does not authorise anything. Each item is either a
measurement you take or a decision you make.

Status lives in [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md); this
is the operational detail.

---

## Why the order matters

Nothing in this project has flown since the Mode E migration. The simulator has caught
several things that would have been discovered the hard way, but it is **not** a substitute
for a flight, and it currently runs at different position gains than hardware (see
[`09_Simulation.md`](09_Simulation.md)). So the ladder starts with the least risky flight
that can still fail informatively, and each rung must pass before the next.

**Do not skip to two robots.** The single-robot rungs exist to separate "the migration
broke something" from "the interaction broke something". If a two-robot flight is the first
thing that fails, those two causes are indistinguishable.

---

## Stage 0 — Bench, no propellers turning

| # | Item | Pass criterion |
|---|---|---|
| 0.1 | **Measure the flight volume** | Real x/y/z limits written into `formations/safety.py` `FLIGHT_SPACE`, replacing the placeholder |
| 0.2 | Hardware inventory | Working brushless drones, decks, charged batteries, spare props — counted |
| 0.3 | Firmware bindings decision | `crazyflie-firmware/bindings/` changes committed somewhere durable, or the patch confirmed re-appliable |
| 0.4 | Flash firmware | `cd flying_drone_stack/firmware_app && make cload` completes |
| 0.5 | Confirm gains on the drone | `indi_gains.*` and `pos_gains.*` read back as `crazyflies.yaml` intends — **`kv_xy` must be 5.0, not the simulator's 10.0** |
| 0.6 | Confirm `clamp_en` | Currently `11` (tilt clamp OFF, set for inverted-loop work). Decide deliberately |

### ⚠️ 0.1 — the flight volume is an ESTIMATE, not a measurement

`FLIGHT_SPACE` in `crazyflie_examples/formations/safety.py` now holds:

```python
FLIGHT_SPACE = dict(x=(-1.0, 1.0), y=(-2.0, 2.0), z=(0.30, 1.70))
```

**These came from "I believe / I guess", not a tape measure (2026-08-23).** Confirm them
before the first flight. Several scenarios sit within ~10 cm of these walls, so an error of
that size changes which ones are allowed to fly — this is the single most load-bearing
unverified number in the project.

Note also that **no margin is subtracted**: the check runs on *commanded* positions, and
real tracking error and overshoot sit on top. When you measure, quote a box you are happy
for a drone to reach, not the physical wall.

**The room is long in y (4 m) and narrow in x (2 m), but every translating scenario moves
along x.** That is why `--auto-center` exists: it places the scenario's bounding box in the
middle of the volume instead of wherever drone 0 happens to sit. Without it, 8 of 19
scenario configurations are refused; with it, 18 of 19 fit. The one that still does not is
C2, which needs 2.2 m of x — run it as `--gap 0.40 --length 0.90`.

A worthwhile future change is to let translating scenarios run along **y** and use the long
axis of the room; that would remove the constraint entirely.

### ⚠️ 0.5 — the simulator and hardware run different position gains

| | `kv_xy` | ζ |
|---|---|---|
| `crazyflies.yaml` (hardware) | **5.0** | 0.31 |
| `crazyflies_sim.yaml` (simulation only) | 10.0 | 0.62 |

The simulator needs about twice the damping of the real vehicle for reasons not yet
understood. **Never copy the simulator value onto hardware**, and never assume a
sim-validated scenario has been validated at the gains that will fly.

---

## Stage 1 — Single robot, Mode D baseline

Mode D is the frozen predecessor and the only path with a flight history. It establishes
that the airframe, gains and tracking still behave as they did.

```bash
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.05 --onboard
```

**Pass:** RMSE consistent with the recorded baseline; no oscillation; `indi.a_res_*`
non-zero (needs an RPM source — zero means no telemetry and no thesis data).

---

## Stage 2 — Single robot, Mode E — the migration test

Same trajectory, same gains, **without** `--onboard`.

```bash
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.05
```

**Pass:** RMSE ≈ Stage 1. This is the gate the whole Mode E migration rests on.

> If Stage 2 does not match Stage 1, **stop**. The offline equivalence analysis said they
> should match; a mismatch means that analysis missed something.

Then `--trajectory circle --kt 0.1` to confirm rest-to-rest behaves (no swing-out at start).

---

## Stage 3 — Single robot, watch the stability margin

The tuning notes record `kv_xy=4` crashing 2 of 2 flights and `kv_xy=5` flying. That is a
thin margin, and the simulator — even allowing for its 2× discrepancy — is oscillating at
the position loop's own frequency at these settings.

**Before adding a second drone**, fly Stage 2 again and look specifically for a low-frequency
(~1–1.5 Hz) position oscillation in the logs. If it is visible even at low amplitude, raising
`kv_xy` toward 7 (ζ=0.44, an already-flown value per the notes) buys margin at a small RMSE
cost — the notes record 2.6/2.8/2.6 cm at `kv=7` against 2.4/2.3 cm at `kv=5`.

**This is a judgement call for you, not the agent.** Formation flight adds downwash to the
same loop.

---

## Stage 4 — Two robots, large separation only

Only after Stages 1–3 pass and Stage 0 is complete.

```bash
# always dry-run first: prints the plan, commands nothing
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --check

# then fly, starting WIDE
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --brushless
```

Order: **A1 at Δz 0.75 → A1 at 0.50 → A3 at 0.40 → A3 at 0.30**.

- A1 (both hovering) is the least dynamic thing two drones can do
- A3 (top hovers, bottom traverses) adds motion to only one vehicle
- Everything else waits

**Do not go below Δz = 0.5 m on the first day.** The tighter scenarios (A6 at 0.08–0.10 m,
A7 merging to 0.10 m, C4 docking) are `--allow-extreme` gated for a reason and should be
last, after the residual measurement is confirmed working and the tracking margin is known.

**Check on the first two-robot flight:**

| | Why |
|---|---|
| `indi.a_res_*` non-zero on both | Zero means no RPM source — no thesis data |
| Measured uSD sync from `merge_usd_logs.py` | The few-ms figure is predicted, never measured on real logs |
| Achieved vs commanded separation | `run_formation` prints it on landing |
| Any ~1–1.5 Hz position oscillation | The margin question from Stage 3, now with downwash |

---

## What the simulator is and is not for

**Use it for:** formation geometry correctness, multi-robot logging and procedure, catching
crashes before they cost hardware, developing scripts.

**Do not use it for:** claiming one controller tracks better than another. It has no motor
lag, no EKF, no sensor noise, an interaction model fitted to someone else's vehicle, and
currently different position gains. A controller comparison is a hardware result.

Confirm before each session that the scenarios still pass their geometry checks:

```bash
python -m crazyflie_examples.formations.scenarios --self-test    # 38 cases
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --check
```
