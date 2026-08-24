# C.0 flight card — one page, print it

**Read nothing else on flight day.** Detail lives in
[`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md); this is the
sequence and the abort criteria.

**Rules for the day.** Rungs are ordered so a failure has one explanation. Do not skip forward:
if a two-robot flight is the first thing that fails, "the migration broke something" and "the
interaction broke something" are indistinguishable. **No data collected before C.0 passes counts.**

---

## 0 · Bench, before any propeller turns

```bash
# 1. what WILL be pushed -- the roster you are actually launching with
grep -A8 'pos_gains' $CS2/crazyflie/config/crazyflies.yaml
```

`firmware_params` go to the drone over CRTP, so the yaml states *intent* and only the drone
states *fact*. **Confirm on the vehicle** — cfclient → Parameters tab → `pos_gains.kv_xy`.

> Worth the extra step: `firmware_params never applied` was one of the five simulator-fidelity
> bugs, and it looked exactly like a controller fault for days. A gain that did not land is
> indistinguishable from a gain that is wrong.

| Check | Must be | If not |
|---|---|---|
| `pos_gains.kv_xy` **on the drone** | **5.0** | **STOP.** 8.0 is the simulator's value (`crazyflies_sim*.yaml`). Hardware crashed 2/2 at 4 and flies at 5 — do not take off on a sim gain |
| `pos_gains.kp_xy` | 64.0 | stop, gains did not apply |
| Launched with `crazyflies.yaml` | not `crazyflies_sim*.yaml` | the sim roster carries the sim gain |
| uSD card in each drone | present, empty | no card = no dataset |

☐ Tape-measure the flight volume before trusting the geofence — scenarios sit within ~10 cm of
the walls. Current numbers (x ±1, y ±2, z 0.30–1.70) are corroborated, **not measured**.

---

## 1 · Single robot, hover

Prove the vehicle is sane and the residual channel is alive.

**Pass:** stable hover · `|a_res|` **small but not identically zero** · no ~1–1.5 Hz position
oscillation.

**Abort if:** `indi.a_res_*` reads **exactly 0.000** → no RPM source → **no thesis data at all.**
Not a small error; the entire signal is absent. Fix before anything else.

> A large `|a_res|` in single-drone hover means the *thrust model* is wrong — there is no other
> vehicle to disturb it. Small-but-nonzero is the pass.

---

## 2 · Residual sign — THE flight of the day

The sign bug made position INDI **add** the residual, reinforcing every unmodelled force. It
measured **exactly 2.00×** on a known disturbance. It is **invisible in single-drone flight**,
so this is the first flight that can see it, and it gates everything downstream.

### The three-line proof

1. **Force a known disturbance** — fly **A1 at Δz = 0.75 m**, two robots. The upper vehicle's
   downwash on the lower one *is* the disturbance; no rig needed. Repeat with INDI on and off,
   nothing else changed.
2. **The field that proves sign** — `indi.a_res_z` on the **lower** vehicle, compared against
   its **separation error**. Compensation must *shrink* the error: |e_z| with INDI < |e_z| with
   geometric, at the same Δz. Pre-fix, the ratio ran 1.89–2.10× the wrong way.
3. **Pass, in one sentence** — *with the residual term enabled, mean |e_z| at Δz 0.75 is no
   larger than with it disabled, and does not grow when Δz tightens to 0.50.*

```bash
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --check      # dry run first
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --brushless  # geometric
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --brushless  # + INDI
```

**Abort if** enabling compensation makes separation error **larger**. That is the sign fix not
having taken, and every downstream number would be built on it.

Probe: `experiments/analysis/probe_residual_sign.py`

---

## 3 · Single robot, Mode D → Mode E → circle

Mode E has **180 hover flights but only one trajectory flight.** These three rungs are what
buys the right to trust it for the campaign.

```bash
# 3a  figure-8, Mode D (the frozen path, has flight history)
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.05 --onboard

# 3b  figure-8, Mode E -- same trajectory, same gains, no --onboard
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.05

# 3c  circle, Mode E -- confirms rest-to-rest start (no swing-out)
ros2 run crazyflie_examples flight -- --trajectory circle --kt 0.1
```

**Pass:** 3b RMSE ≈ 3a RMSE. **This is the gate the whole Mode E migration rests on.**

**Abort if** 3b ≠ 3a. Do not proceed to two robots — the offline analysis said they agree, so a
disagreement means the analysis is wrong about something that also affects formations.

Do **not** use `--rest-to-rest` (measured worse).

---

## 4 · Two robots, wide first

```bash
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --check
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --brushless
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.50 --brushless
```

Order: **A1 @ 0.75 → A1 @ 0.50 → A3 @ 0.40 → A3 @ 0.30.** Nothing below Δz 0.50 on day one.
A6/A7/C4 are `--allow-extreme` gated for a reason and come last.

| Check on the first two-robot flight | Why |
|---|---|
| `indi.a_res_*` non-zero on **both** vehicles | zero = no RPM source = no data |
| `a_res` grows as Δz tightens | confirms it is measuring the interaction, not noise |
| Achieved vs commanded separation | `run_formation` prints it on landing |
| Measured uSD sync — `merge_usd_logs.py` prints it | the few-ms figure is predicted, never measured |
| Any ~1–1.5 Hz position oscillation | thin damping margin, now with downwash on the same loop |

---

## 5 · Freeze the gains

Once 1–4 pass: **write down every gain and stop changing them.** Re-tuning later measures tuning
effort, not the methods. Stage 2 of C.0 is the only place a retune is allowed.

☐ `pos_gains.*` and `indi_gains.*` recorded, dated, committed.

Then → **C.1 collection: A1 + A3 + A4.** A4 is **required** — A3 never excites relative *y*, and
without it the model trains on zero lateral variance and RQ3 becomes unanswerable.

---

## Abort summary

| Symptom | Meaning |
|---|---|
| `a_res` exactly 0.000 | no RPM source — no thesis data |
| Compensation *increases* separation error | sign fix did not take |
| `kv_xy` reads 8.0 | simulator gain on hardware — do not fly |
| Mode E RMSE ≠ Mode D | migration is not equivalent |
| INDI hover worse than geometric hover | sign fix has not done what the derivation says |
| ~1–1.5 Hz position oscillation | damping margin gone; formations will be worse |
