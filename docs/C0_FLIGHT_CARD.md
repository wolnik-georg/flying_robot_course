# C.0 flight card — one page, print it

> **Citation audit closed 27 Aug 2026. Do not block C.0 on more papers.**

**Read nothing else on flight day.** Detail lives in
[`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md); this is the
sequence and the abort criteria.

**Rules for the day.** Rungs are ordered so a failure has one explanation. Do not skip forward:
if a two-robot flight is the first thing that fails, "the migration broke something" and "the
interaction broke something" are indistinguishable. **No data collected before C.0 passes counts.**

---

## ⛔ RUNG −1 · CONTROLLER VALIDATION — added 2026-09-09, do this FIRST

**On 2026-09-09 six hover flights crashed under both controllers.** Three root causes were
found and fixed; **none has been re-flown**. Everything below this block is blocked until
both controllers fly clean again. Full account:
[`lab_sessions/2026-09-09.md`](lab_sessions/2026-09-09.md) · audit:
[`REVIEW_FINDINGS_2026-09-09.md`](REVIEW_FINDINGS_2026-09-09.md).

**Before flying: pull the uSD cards from the 09-07/09-09 flights.** If that data exists,
`a_res` at the divergence may resolve which of three simultaneous unflown changes actually
crashed INDI — for free, at the desk. It is lost once overwritten.

Then reflash (`make cload` — two of the fixes are firmware), rebuild `crazyflie` +
`crazyflie_examples`, and confirm on the vehicle:

| Param | Must read |
|---|---|
| `indi_gains.res_sign` | **1** (frozen `.add`; `-1` is the never-validated fix) |
| `indi_gains.frame_conv` | **0** (Faessler, as frozen) |
| `indi_gains.kr_geo` | **0.010** |
| `indi_gains.clamp_en` | **11** |
| `stabilizer.controller` | **6** |

Six flights. `python3 experiments/analysis/check_flight.py` after **every one**. Stop on the
first FAIL.

| # | `ctrl_mode` | Command |
|---|---|---|
| 1 | 0 | `simple_flight -- --trajectory hover --duration 15` |
| 2 | 0 | `simple_flight -- --trajectory circle --kt 0.1` |
| 3 | 0 | `simple_flight -- --trajectory figure8 --kt 0.008` |
| 4 | 3 | `simple_flight -- --trajectory hover --duration 15` |
| 5 | 3 | `simple_flight -- --trajectory circle --kt 0.1` |
| 6 | 3 | `simple_flight -- --trajectory figure8 --kt 0.008` |

Geometric should print `<- GEOMETRIC_POS_GAINS` at takeoff (40/8); INDI should show 64/5.
**If INDI fails but geometric passes**, the cause pre-dates every 2026-09-09 fix → run the
H0 partition (`ctrl_mode=2`, then `1`). `ctrl_mode=2` is a clean instrument for this: the
residual is provably unreachable there (0/100 samples on a `res_sign` toggle).

**Only after all six PASS** do gain retuning, the rungs below, and multi-drone resume.

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
| `indi_gains.clamp_en` | **11** | 2026-09-07: confirmed the ONE value that differs from the frozen/parked, flight-proven state. 15 (tilt clamp on) produced a real, growing attitude oscillation on both controllers that 11 does not — do not fly at 15 for level formation flight |
| `indi_gains.kr_geo` / `kw_geo` | reads a real number, not `0` | geometric's own attitude gain pair (added 2026-09-07) — a missing param silently behaves like `kr_geo=0`, not a safe fallback. Untuned starting point is `0.010`/`0.00110` |
| Launched with `crazyflies.yaml` | not `crazyflies_sim*.yaml` | the sim roster carries the sim gain |
| uSD card in each drone | present, empty | no card = no dataset |
| `indi.e_r_x/y/z/norm` present in the uSD file | confirm on the first hover | added 2026-09-08; `experiments/analysis/README.md` has the column dictionary |

### ☐ The three unflown residual changes are on the bird

Flash, then confirm the build actually carries all three. None is detectable in single-drone
flight, and §2 is meaningless if any of them is missing from the firmware being flown.

| # | Change | Confirm |
|---|---|---|
| 1 | **Residual sign fix** | present in the flashed build — confirmed in flight at §2, not at the bench |
| 2 | **`a_res` gating fix** | `indi.a_res_*` is live under **geometric**, not only under INDI |
| 3 | **`rnn.en`** | parameter exists and reads **0** — leave it off until the rest of C.0 passes |

### ☑ Tape-measure the flight volume — DONE 2026-09-02

Measured x ±1, y ±2, z 0–1.70 — matches the prior corroborated (not-measured) numbers exactly.
`formations/safety.py` (`FLIGHT_SPACE`) updated to say tape-measured, not placeholder. Practical
z-floor for actual flights (ground effect) confirmed ~0.3-0.4 m; `Z_FLOOR_DEFAULT = 0.30` kept
as the low end of that, unchanged.

Scenarios sit within ~10 cm of the walls. A1 at Δz 0.75 still needs `--height 0.85` to fit.

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

**Analyse with:** `experiments/analysis/analyze_formation.py <timestamp>` — it takes a
**timestamp, not a file path**, and reports commanded-vs-achieved separation together with
`f_res = m·a_res`. Those are exactly the two numbers this rung turns on.

```bash
python3 experiments/analysis/analyze_formation.py 2026-09-02_14-15-10
```

> **Not `probe_residual_sign.py`** — that is a *simulation* probe. It injects a force into the SIL
> controller and needs `cffirmware` on `PYTHONPATH`; it cannot read a hardware log.
>
> **For the full picture (RMSE, sag, e_R, plots), not just this rung's quick check:** once uSD logs
> are off the card, `merge_usd_logs.py` → `experiments/analysis/run_analysis.py` (numbers) and
> `plot_flight.py` (the dashboard PNG) are the ready pipeline — see
> `experiments/analysis/README.md`. `analyze_formation.py` above stays the fast live check;
> `run_analysis.py`/`plot_flight.py` are the ones that actually produce the dataset-quality numbers
> this thesis reports. Use system `python3` for `run_analysis.py`; use
> `~/.pyenv/versions/flying_robots/bin/python` for `plot_flight.py` (system `matplotlib` is broken
> on this machine — numpy ABI mismatch).
>
> **Radio saturates with two drones** (~600 pkt/s per drone against a ~1000 pkt/s dongle limit), so
> the live stream will look thin here. That is expected, not a fault: `run_formation` turns uSD
> logging on for the flight, and **the uSD logs are the numbers you judge this rung by** — merge
> them with `flying_drone_stack/tools/merge_usd_logs.py`, which also prints the measured sync.

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

> **Known trajectory characteristic — not a fault.** `figure8 @ kt=0.05` commands a genuine
> ~7 m/s² acceleration peak ~0.45 s after launch and ~0.45 s before landing (its rest-to-rest
> ramp boundaries) — that's a ~35° tilt (`atan(7.0/9.81)`), confirmed in sim 2026-09-03. If you
> see a sharp tilt right at trajectory start/end, this is why — don't mistake it for a bug or a
> handoff fault. `run_formation`'s own scenario curves don't demand anything this aggressive at
> their boundaries, so this is specific to this trajectory export, not the flight scripts.

---

## 4 · Two robots, wide first

```bash
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --check
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --brushless
ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.50 --brushless
```

Order: **A1 @ 0.75 → A1 @ 0.50 → A3 @ 0.40 → A3 @ 0.30.** Nothing below Δz 0.50 on day one.
A6/A7/C4 are `--allow-extreme` gated for a reason and come last.

**Stay inside the measured geofence** — the one written into `formations/safety.py` at §0, not
the placeholder.

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

**Optional, here and only here — before C.1, after the freeze:** if there's a spare session, this
is the slot for the DShot-vs-deck RPM re-test (`docs/23_DShot_RPM_Investigation.md`) — gains are
frozen, so it's the one point where trying an alternate RPM source doesn't confound the
comparison campaign that follows. Not required to proceed to C.1.

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
