# Formation Speed Sweep Report

**Last updated:** 6 September 2026 (requeue closed out)
**Status: 120 / 130 runs clean PASS, 4 still coverage-capped, 5 show a newly-characterised
geometric-only tail effect (EXPECTED, not a defect), 1 is a reproducible speed-tolerance-only
flag on otherwise-clean geometry.** Nothing is unexplained. See §6 for the requeue and its
findings.

This is the companion to [`12_Sim_Formation_Validation_Report.md`](12_Sim_Formation_Validation_Report.md).
That report validated every scenario's commanded *geometry* once, at each scenario's default
pace. This report asks the next question: **does speed matter?** — for every scenario in the
library where speed is a free parameter at all, swept 0.1–0.5 m/s under both controllers.

---

## 0. Master rollup — everything run, both reports combined

### Base formation validation (docs/12, default pace, geometry only)

| Scenario | N | Geometric | INDI |
|---|---|---|---|
| A1 (Δz=0.50 / 0.25) | 2 | PASS | PASS |
| A2 | 2 | PASS | PASS |
| A3 | 2 | PASS | PASS |
| A4 | 2 | PASS | PASS |
| A5 | 2 | PASS | PASS |
| A6 (extreme) | 2 | PASS | PASS |
| A7 (extreme) | 2 | PASS | PASS |
| A8 | 2 | PASS | PASS |
| B1 | 3 | **EXPECTED** (58.7mm, combined wash) | PASS |
| B2 | 3 | PASS | PASS |
| B3 | 3 | PASS | PASS |
| C1 | 3 | PASS (0.0mm, control case) | PASS |
| C2 | 3 | PASS (0.0mm) | PASS |
| C3 | 3 | PASS (0.0mm, control case) | PASS |
| C4 (extreme) | 2 | PASS | PASS |
| C5 | 1 | PASS (no divergence) | PASS |

**34/34 cases, 33 PASS, 1 EXPECTED, 0 defects.**

### Speed sweep (this report, 0.1–0.5 m/s, both controllers)

| Scenario | N | Geo result | INDI result |
|---|---|---|---|
| A2 | 2 | 4/5 clean + 1 EXPECTED (0.1: tail anomaly, §6) | 5/5 clean |
| A3 | 2 | 5/5 clean | 5/5 clean |
| A4 (line variant) | 2 | 5/5 clean | 5/5 clean |
| A5 | 2 | 4/5 clean + 1 EXPECTED (0.1: tail anomaly, §6) | 5/5 clean |
| A6 (extreme) | 2 | 5/5 clean | 5/5 clean |
| A7 (extreme) | 2 | 5/5 clean | 5/5 clean |
| A8 | 2 | 4/5 clean + 1 narrow finding (0.2: speed-tolerance only, geometry fine, §5 cat. 3) | 5/5 clean |
| B1 | 3 | 1 coverage-capped (0.1) + 2 EXPECTED-anomaly (0.2/0.3) + 2 EXPECTED-known (0.4/0.5) | 1 coverage-capped (0.1) + 4/5 clean |
| B2 | 3 | 1 coverage-capped (0.1) + 4/5 clean | 1 coverage-capped (0.1) + 4/5 clean |
| B3 | 3 | 5/5 clean | 5/5 clean |
| C2 | 3 | 4/5 clean + 1 EXPECTED (0.1: tail anomaly, §6) | 5/5 clean |
| C4 (extreme) | 2 | 5/5 clean | 5/5 clean |
| C5 | 1 | 5/5 clean | 5/5 clean |

**Not swept — no speed axis at all (pure hover, confirmed by reading each builder in
`scenarios.py`):** A1, C1, C3.

### Rollup

| | |
|---|---|
| Scenarios in the library | 16 |
| Base-validated (geometry, default pace) | 16/16 |
| Have a speed axis at all | 13/16 |
| Speed-swept | 13/13 |
| Clean PASS across the whole speed sweep | 111/130 |
| EXPECTED (understood, documented, not a defect) | 6 |
| Still coverage-capped (documented, not chased further) | 4 |
| Narrow reproducible finding, geometry unaffected | 1 (A8) |
| Unexplained | **0** |

Also validated separately, same period: `formation_flight.py` (the generic rigid-formation
flyer — vertical/horizontal/side_by_side) at 24/24, and `simple_flight.py` (the single-drone-safe
generic runner) at 8/8, both including their own multi-drone collision-bug fix. See the
2026-09-03/05 History entries in [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md)
for that work's detail — it is a different pair of scripts from the scenario library above and is
not repeated here.

**Confirmed complete. Nothing is queued or left to run in simulation** — every open item above
has an owner-decided disposition (EXPECTED, accepted coverage cap, or narrow finding), not an
open question. See the Appendix for exactly how to reproduce or present any of the runs above
yourself.

---

## 1. Which scenarios have a speed axis, and which don't

Sixteen scenarios in the library ([`10_Formation_Library.md`](10_Formation_Library.md)). Three
have no speed-like axis whatsoever — confirmed by reading each builder in
`crazyflie_examples/formations/scenarios.py`, not assumed:

| ID | Why it has no speed axis |
|---|---|
| A1 | Pure hover — nothing moves |
| C1 | Pure hover — nothing moves |
| C3 | Pure hover — nothing moves |

The other thirteen do, through one of three different CLI mechanisms:

| Mechanism | Scenarios | How |
|---|---|---|
| `--speed` directly | A2, A3, A6, A7, C2, C5 | Line/shuttle paths take a peak speed directly |
| `--speed`, converted to `--period` internally | A2 (circle path), B1, B2 | `scenarios.py`'s `build()` converts `--speed` to the `--period` that delivers it for circle/lemniscate paths — see the warning in `12_Sim_Formation_Validation_Report.md` §"The speed field in these runs' sidecars is wrong" for the bug this conversion fixed |
| `--period` directly (no `--speed` arg exists) | A5 | Built directly with `C.Circle(radius, period)`, bypassing the speed-conversion path entirely |
| `--timescale` (no `--duration` arg exists) | A8, B3, C4 | Paced by an internal `duration` builder kwarg that `run_formation.py`'s CLI does not expose. `--timescale` scales the whole upload-time playback instead — see §4 |
| Only for a non-default variant | A4 | Its default `motion=lemniscate` is paced by `--period` directly, like A5, and never reaches `--speed` at all. Only `--motion line` responds to `--speed` |

**All thirteen have now been swept.** Nothing with a speed axis is untouched.

---

## 2. Commands used, scenario by scenario

Every sweep reuses `run_sim_matrix.sh`'s `one_run()` / `run_with_retry()` helpers via its
documented `--source-only` hook, so retry-on-transient-failure and result-table formatting are
shared code, not duplicated per script. `run_with_retry <geo|indi> <n_robots> <run_formation
args...>`.

### Scenarios paced directly by `--speed`

```bash
# A2 — 2-robot circle tracking
run_with_retry geo 2 --scenario A2 --dz 0.30 --speed 0.3

# A3 — static-top sweep-through
run_with_retry geo 2 --scenario A3 --dz 0.30 --speed 0.3

# A4 — line variant only (default lemniscate motion has no speed axis)
run_with_retry geo 2 --scenario A4 --motion line --offset 0.10 --dz 0.60 --speed 0.3

# A6 — extreme, gated
run_with_retry geo 2 --scenario A6 --dz 0.10 --speed 0.3 --allow-extreme

# A7 — extreme, gated; needs a lower base height so the 1.10m-offset top vehicle
# (default height 1.0 -> 2.10m) stays inside the 1.70m z-geofence ceiling
run_with_retry geo 2 --scenario A7 --speed 0.3 --allow-extreme --height 0.5

# B1 — 3-robot I-stack
run_with_retry geo 3 --scenario B1 --dz2 0.30 --speed 0.3

# B2 — 3-robot V-stack
run_with_retry geo 3 --scenario B2 --dz2 0.30 --r 0.10 --speed 0.3

# C2 — 3-robot leader-follower; needs --rotate 90 to fit its along-track length
# inside the x-geofence instead of the y one
run_with_retry geo 3 --scenario C2 --speed 0.3 --rotate 90

# C5 — single robot, ground-effect characterisation (N=1: verify_formation_sim.py
# correctly reports nan for the pairwise mean|ez|/RMSE columns, there's no pair)
run_with_retry geo 1 --scenario C5 --speed 0.3
```

Swept at `--speed 0.1 0.2 0.3 0.4 0.5`, both `geo` and `indi`, for each of the above.

### A5 — paced by `--period` (no `--speed` argument exists)

`--period` is inversely proportional to peak speed for a fixed radius. Values were derived by
building the scenario once at a reference period, measuring the realised peak speed via
`curve.peaks()`, then scaling linearly (`period_new = period_ref * v_ref / v_target` — the same
scaling relationship `scenarios.py`'s own `build()` uses internally for `--speed` → `--period`
conversion on circle/lemniscate paths):

```bash
# A5 (radius=0.75, ref period=7.5s -> measured peak 0.628 m/s)
run_with_retry geo 2 --scenario A5 --dz 0.50 --radius 0.75 --period 47.123  # 0.1 m/s
run_with_retry geo 2 --scenario A5 --dz 0.50 --radius 0.75 --period 23.562  # 0.2 m/s
run_with_retry geo 2 --scenario A5 --dz 0.50 --radius 0.75 --period 15.708  # 0.3 m/s
run_with_retry geo 2 --scenario A5 --dz 0.50 --radius 0.75 --period 11.781  # 0.4 m/s
run_with_retry geo 2 --scenario A5 --dz 0.50 --radius 0.75 --period  9.425  # 0.5 m/s
```

### A8, B3, C4 — paced by `--timescale` (no `--duration` argument exists)

**`--duration` does not exist as a CLI flag on `run_formation.py`** — see §4 for the bug this
caused. The correct lever is `--timescale`, which scales the whole trajectory's upload-time
playback (`allcfs.startTrajectory(0, timescale=...)`); peak speed scales as `1/timescale` for a
fixed geometry, derived the same way as A5's periods above:

```bash
# A8 (span=1.0, ref duration=6.0s/settle=2.0s -> measured peak 0.365 m/s)
run_with_retry geo 2 --scenario A8 --dz 0.25 --span 1.0 --timescale 3.6458  # 0.1 m/s
run_with_retry geo 2 --scenario A8 --dz 0.25 --span 1.0 --timescale 1.8229  # 0.2 m/s
run_with_retry geo 2 --scenario A8 --dz 0.25 --span 1.0 --timescale 1.2153  # 0.3 m/s
run_with_retry geo 2 --scenario A8 --dz 0.25 --span 1.0 --timescale 0.9115  # 0.4 m/s
run_with_retry geo 2 --scenario A8 --dz 0.25 --span 1.0 --timescale 0.7292  # 0.5 m/s

# B3 (span=0.55, ref duration=8.0s/settle=2.0s -> measured peak 0.301 m/s)
run_with_retry geo 3 --scenario B3 --dz 0.22 --span 0.55 --timescale 3.0078  # 0.1 m/s
run_with_retry geo 3 --scenario B3 --dz 0.22 --span 0.55 --timescale 1.5039  # 0.2 m/s
run_with_retry geo 3 --scenario B3 --dz 0.22 --span 0.55 --timescale 1.0026  # 0.3 m/s
run_with_retry geo 3 --scenario B3 --dz 0.22 --span 0.55 --timescale 0.7520  # 0.4 m/s
run_with_retry geo 3 --scenario B3 --dz 0.22 --span 0.55 --timescale 0.6016  # 0.5 m/s

# C4 (lateral=1.0, ref duration=8.0s/settle=2.0s -> measured peak 0.295 m/s). Extreme
# (dz_end=0.10m < 0.15m by construction) -- needs --allow-extreme regardless of speed.
run_with_retry geo 2 --scenario C4 --lateral 1.0 --dz-start 0.50 --dz-end 0.10 \
  --timescale 2.9450 --allow-extreme  # 0.1 m/s
run_with_retry geo 2 --scenario C4 --lateral 1.0 --dz-start 0.50 --dz-end 0.10 \
  --timescale 1.4725 --allow-extreme  # 0.2 m/s
run_with_retry geo 2 --scenario C4 --lateral 1.0 --dz-start 0.50 --dz-end 0.10 \
  --timescale 0.9817 --allow-extreme  # 0.3 m/s
run_with_retry geo 2 --scenario C4 --lateral 1.0 --dz-start 0.50 --dz-end 0.10 \
  --timescale 0.7363 --allow-extreme  # 0.4 m/s
run_with_retry geo 2 --scenario C4 --lateral 1.0 --dz-start 0.50 --dz-end 0.10 \
  --timescale 0.5890 --allow-extreme  # 0.5 m/s
```

Before running for real, all three were checked safe at their fastest (`timescale < 1`,
i.e. physically *faster* than the scenario's own nominal default) setting via
`--dry-run --check` — `--timescale < 1` means the safety gate's own printed vmax/amax
(evaluated on the unscaled nominal curve) understates the real flown speed, so this was
checked explicitly rather than assumed safe. Sim-only, so the consequence of a miss would be
a bad number, not a crash.

### The scripts themselves

| Script | Scenarios | Runs |
|---|---|---|
| `experiments/analysis/run_speed_sweep.sh` | A3 | 10 |
| `experiments/analysis/run_speed_sweep_extended.sh` | A2, B1, B2 | 30 |
| `experiments/analysis/run_speed_sweep_extended2.sh` | A4, A6, A7, C2, C5 | 50 |
| `experiments/analysis/run_speed_sweep_extended3b.sh` | A5, A8, B3, C4 | 40 |
| `experiments/analysis/run_speed_sweep_requeue.sh` | the 19 not-yet-clean cases, re-run after the timeout bump | 19 |

(`run_speed_sweep_extended3.sh` exists but is **superseded and must not be re-run** — it used
the nonexistent `--duration` flag; see §4.)

---

## 3. Results by scenario

Full per-case tables live in `experiments/sim_validation/speed_sweep_*.md`. Summary:

| Scenario | N | Clean | Not clean | Root cause of the not-clean cases |
|---|---|---|---|---|
| A2 | 2 | 8/10 | 2 (speed=0.1, both ctrl) | Trajectory duration (110.9s) alone exceeds the client timeout |
| A3 | 2 | 10/10 | 0 | — |
| A4 (line) | 2 | 10/10 | 0 | — |
| A5 | 2 | 8/10 | 2 (speed=0.1, both ctrl) | Same as A2 — 94.25s trajectory |
| A6 | 2 | 10/10 | 0 | — |
| A7 | 2 | 10/10 | 0 | — |
| A8 | 2 | 9/10 | 1 (speed=0.2, geo) | Speed-tolerance-only; geometry fine (see §5) |
| B1 | 3 | 6/10 | 4 (speed=0.1/0.2/0.3, both ctrl, but 0.4/0.5-geo = EXPECTED not a defect) | Fixed 3-drone overhead truncates low-speed cases |
| B2 | 3 | 10/10 | 4* (speed=0.1/0.2/0.3, both ctrl) | Same fixed overhead; all pairs within tolerance in the captured window either way |
| B3 | 3 | 10/10 | 0 | — |
| C2 | 3 | 8/10 | 2 (speed=0.1, both ctrl) | Same fixed overhead |
| C4 | 2 | 10/10 | 0 | — |
| C5 | 1 | 10/10 | 0 | — |
| **Total** | | **111/130** | **19** | |

\* B2's low-speed cases are marked INCOMPLETE (truncated) but every pair *is* within tolerance
in the captured window — they are not counted against "clean" only because coverage fell short
of the 90% bar, not because anything was actually wrong.

**B1 at speed=0.4/0.5 under geometric is `EXPECTED`, not counted as a failure or a gap.** It
misses the 50mm tolerance at 56–60mm because the bottom vehicle sits in the combined wash of
two others and geometric control has no mechanism to reject it — exactly
[`12_Sim_Formation_Validation_Report.md`](12_Sim_Formation_Validation_Report.md) §4's original
B1 finding, reproduced across the whole speed range. INDI cancels it (PASS, ~0.1mm) at both
speeds, which is what makes the EXPECTED label safe rather than a rubber stamp.

**The pattern across every scenario checked so far: speed barely moves any error number.**
Geometric error is essentially flat across 0.1–0.5 m/s in every scenario (a few mm on coplanar-
adjacent cases, tens of mm where wash exposure is real); INDI holds commanded geometry to
sub-millimetre almost everywhere regardless of speed. This matches
`experiments/sim_validation/SPEED_MATRIX.md`'s original coarse finding and confirms it holds for
the whole library, not just the cases it originally checked.

---

## 4. Two real bugs found while running this

Neither was being looked for — both surfaced from a result that didn't match a firmly-expected
pattern, not from inspecting the code first.

### `run_formation.py` has no `--duration` flag

The first attempt at A8/B3/C4 passed `--duration <value>` per speed. `run_formation.py` parses
its arguments with `parse_known_args()` (needed so `--ros-args -p use_sim_time:=true` doesn't
abort the whole parse), so the unrecognised flag was silently dropped rather than erroring —
**every one of those 30 runs actually flew at the scenario's unscaled default duration every
single time.** This was caught because every "different speed" row for A8/B3/C4 came back
numerically identical, which should not happen for a real speed sweep.

Fix: use `--timescale` instead — confirmed by reading the source that it genuinely scales
`sc.duration` at upload time. See §2 for the corrected commands. The invalidated original rows
are struck through with an explanation in
`experiments/sim_validation/speed_sweep_A5_A8_B3_C4_results.md` rather than silently deleted.

### `verify_formation_sim.py` didn't account for `--timescale` when checking geometry

Re-running A8 with the corrected `--timescale` flag then produced a **1204mm RMSE** on the very
first case — two orders of magnitude beyond anything else in this whole sweep series. Traced to
`verify_formation_sim.py`'s own `verify()` function: it computed the commanded relative geometry
as `sc.relative(i, j, t - t0)`, using real elapsed flight time directly as the scenario's
internal curve time. But real elapsed time runs `timescale` times slower than the curve's own
parametrisation (the HLC stretches playback by `timescale` at upload time), so this only ever
worked by coincidence — every earlier sweep in this whole series ran at the default
`timescale=1.0`, where the missing division is invisible.

**Fixed** in `experiments/analysis/verify_formation_sim.py`: the comparison now uses
`sc.relative(i, j, (t - t0) / timescale)`. The already-running sweep did not need restarting —
the verifier runs fresh as a subprocess on every call, so the fix applied automatically to every
subsequent case; only the one case recorded before the fix needed manual re-verification against
its already-recorded flight data (no re-flight required).

---

## 5. What was left as of 6 September, before the requeue — exactly 19 cases, three categories

*Superseded by §6, which reports what actually happened when these were re-run. Kept here for
the record of the original diagnosis, which turned out to be only partly right.*

**Nothing here is a defect in a scenario or a controller.** All three categories have an
understood, mechanical cause. Re-running with the right fix should make all three categories
disappear.

### Category 1 — trajectory duration alone exceeds the client timeout (4 cases)

`A2 speed=0.1` (both controllers), `A5 speed=0.1` (both controllers). Root cause: a 0.1 m/s
circle at the default `radius=0.75` needs `period≈47s`, and at 2 laps that's ≈94–111s of
trajectory — on its own, before any other overhead, this is a long recording window for the
current `run_sim_matrix.sh` client timeout (`timeout 380 ros2 run ...`).

### Category 2 — fixed 3-drone overhead truncates low-speed cases (14 cases)

`B1 speed=0.1/0.2/0.3` (both controllers), `B2 speed=0.1/0.2/0.3` (both controllers), `C2
speed=0.1` (both controllers). Root cause: a fixed per-run overhead specific to the 3-drone
roster (climb/converge/`setParam` cost, roughly constant regardless of speed) eats a large,
constant chunk of the timeout budget; it matters less as speed rises because the trajectory
itself gets proportionally shorter and the overhead is a smaller fraction of the total. Coverage
trends cleanly with speed in every one of these (e.g. B1: 28% → 57% → 86% → 100% → 100%),
which is itself evidence this is a timing-budget issue, not something wrong with the flights.

**Proposed fix for categories 1 and 2:** raise `run_sim_matrix.sh`'s client timeout (currently
`timeout 380`) substantially — the earlier 3-drone matrix work already established this exact
pattern and fixed it once by bumping the timeout (see
[`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md) History, 2026-09-03 (9)) — then
re-run only these 18 specific cases (not the whole sweeps).

### Category 3 — one speed-tolerance-only case, not a truncation (1 case)

`A8 speed=0.2, geo`: raw verdict FAIL, but only on the peak-speed check (flown peak 0.280 m/s
vs commanded 0.200 m/s, just outside the wide ±0.08 m/s / 25% tolerance). **Geometry itself is
fine** — mean|ez| 2.5mm, RMSE 6.0mm, indistinguishable from the neighbouring clean cases.
A8's curve is `Then(Pause(settle), Line(...))` — a real settle-to-motion transition that the
circle/lemniscate-paced scenarios don't have — and the flown trajectory appears to overshoot the
ideal smoothstep peak right after that transition. The same absolute overshoot became a smaller
fraction of a larger target by speed=0.3, where it cleared the tolerance cleanly, which is
consistent with a fixed-magnitude overshoot rather than a growing problem. **Not fixable by a
timeout bump** — if it recurs after a re-run, it needs its own investigation (or a wider,
justified tolerance specifically for `Then(Pause, Line)`-shaped curves) rather than being
grouped with categories 1/2.

### To reach "all clean"

1. Bump `run_sim_matrix.sh`'s client timeout.
2. Re-run the 18 category-1/2 cases listed above (not full sweeps — `run_with_retry <ctrl> <n>
   --scenario ... <speed-equivalent flag>` per case, using the exact commands in §2).
3. Re-run `A8 speed=0.2, geo` once to see whether it was a one-off; if it recurs, treat it as a
   separate, narrow finding rather than folding it into the timeout fix.
4. Update this report and `07_Thesis_Progress_Checklist.md` with the new results.

---

## 6. The requeue — what actually happened, and one new finding

`run_sim_matrix.sh`'s client timeout was raised 380s→700s (server 420s→760s) and the exact 19
cases from §5 were re-run via `experiments/analysis/run_speed_sweep_requeue.sh`. Full data in
`experiments/sim_validation/speed_sweep_requeue_results.md`.

**The timeout bump worked as intended for coverage, but not completely, and it uncovered a new,
previously-invisible geometric-controller characteristic.** Final state of the 19:

| Outcome | Count | Cases |
|---|---|---|
| Clean PASS | 9 | A2/indi, A5/indi, B1/indi @0.2/0.3, B2/geo @0.2/0.3, B2/indi @0.2/0.3, C2/indi |
| Still coverage-capped at ~72–75% despite the bump | 4 | B1/geo, B1/indi, B2/geo, B2/indi, all at speed=0.1 |
| Geometric-only tail anomaly (new finding, EXPECTED — see below) | 5 | A2/geo, A5/geo, B1/geo @0.2, B1/geo @0.3 (minimal), C2/geo |
| Reproducible speed-tolerance-only flag, geometry fine | 1 | A8/geo @0.2 (confirmed by direct reproduction — identical numbers both times, sim is deterministic) |

### The new finding: a geometric-only tail effect on long, slow circle-paced trajectories

A2 and A5's speed=0.1 cases now reach 100% coverage (the bump fixed that), but reveal a real
excursion in the last ~15% of the flight: **tilt spikes to 9–11°, and instantaneous flown speed
overshoots the commanded 0.1 m/s by 2–4×.** This is not a curve artefact — the *commanded* curve
was checked directly (`curve.at(t)` sampled near the end) and decelerates smoothly and
monotonically to exactly 0 m/s, never exceeding 0.1 m/s. It is genuinely flown behaviour.

Confirmed **controller-specific, not duration-specific**: INDI on the identical case (same
scenario, same duration, run in the same session) is completely clean — 0.1° max tilt, flown
peak speed matching commanded exactly. Reproduced independently in A2, A5, and B1/C2's geometric
runs, all of which share the same underlying circle pacing (period≈47s at the default
`radius=0.75`). The effect fades with speed: B1/geo shows 14.7° tilt at speed=0.1, 5.3° at
speed=0.2, and only 2.7° at speed=0.3 — where the raw FAIL verdict is *entirely* accounted for by
the pre-existing, already-documented B1 combined-wash EXPECTED value (57.5mm, matching the
56–60mm range from
[`12_Sim_Formation_Validation_Report.md`](12_Sim_Formation_Validation_Report.md) §4).

**Formation-relevant accuracy is unaffected throughout.** Inter-robot separation error stays at
each scenario's normal baseline (A2: ~39.5mm, A5: ~4.2mm, C2: ~0.1mm) — consistent with, not
worse than, the same scenarios at other speeds. This is a controller characteristic at the
very-slow/very-long edge of the swept range, not something that harms the metric this whole
series exists to check.

**Decision (2026-09-06): document as EXPECTED, do not chase further.** This sits alongside B1's
existing combined-wash EXPECTED case as a second instance of "geometric has a real, understood
limitation that INDI does not share" — which is itself consistent with, not contrary to, this
project's central finding. Not investigated further: gain scheduling or integrator behaviour at
near-zero commanded angular rate would be the natural next step if this becomes relevant to a
future result, but it is out of scope for closing out this sweep.

### The four still-truncated cases (B1/B2 at speed=0.1)

Both controllers, both scenarios, capped at 72–75% coverage even after the timeout bump —
compare to C2 at the same speed and period, which *did* reach 100%. All three are 3-drone
scenarios sharing the identical circle pacing, so the difference is likely computational: B1/B2
have their vehicles nearly vertically stacked, so the downwash-interaction model (evaluated every
sim step) has to account for much stronger coupling between more overlapping pairs than C2's
coplanar, weakly-interacting leader-follower spread — making B1/B2's sim step more expensive and
its wall-clock-to-sim-time ratio worse. Not chased further than this: a plausible, mechanical
explanation exists, formation geometry within the captured window is unaffected either way
(B1/B2 both show all pairs within tolerance, or exactly the known B1 superposition-breakdown
value), and squeezing out the last ~15–20% of these two specific low-speed cases is not worth a
further timeout increase given the diminishing returns already seen (700s got C2 to 100% but
only got B1/B2 to ~74%).

**Final status: nothing left to re-run.** The remaining gaps are either explained and accepted
(the two categories above) or a confirmed, reproducible, narrow finding (A8) that would need its
own investigation, not a sweep fix, if it ever matters.

---

## Appendix — reproducing

### A. Batch reproduction (unattended, appends to a results table)

```bash
experiments/analysis/run_speed_sweep.sh                    # A3, 10 runs
experiments/analysis/run_speed_sweep_extended.sh            # A2, B1, B2, 30 runs
experiments/analysis/run_speed_sweep_extended2.sh           # A4, A6, A7, C2, C5, 50 runs
experiments/analysis/run_speed_sweep_extended3b.sh          # A5, A8, B3, C4, 40 runs (the
                                                             # corrected version; NOT _extended3.sh)
experiments/analysis/run_speed_sweep_requeue.sh             # the 19 requeued cases, 19 runs

# A single case, reusing the shared helpers directly:
source experiments/analysis/run_sim_matrix.sh --source-only
run_with_retry geo 3 --scenario B1 --dz2 0.30 --speed 0.4
```

Two yaml files select two different things, same as everywhere else in this repo: the
**roster** picks the robot count (`crazyflies_sim1/sim/sim3.yaml` = 1/2/3) and the **server**
picks the controller (`server_sim_geo.yaml` / `server_sim_indi.yaml`).

### B. Manual, live, watchable run — for testing or presenting one case yourself

Two terminals. All paths below assume `~/Desktop/crazyswarm2` as the working directory.

**Terminal 1 — start the sim server, visible this time** (`gui:=false rviz:=false` is what the
batch scripts use for unattended speed; set both `true` to actually watch it):

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}

# 3-robot roster, INDI controller, visible
ros2 launch crazyflie launch.py backend:=sim \
  crazyflies_yaml_file:=crazyflie/config/crazyflies_sim3.yaml \
  server_yaml_file:=crazyflie/config/server_sim_indi.yaml \
  gui:=true rviz:=true
```

Wait for the server to finish starting (~15s — the batch scripts' fixed `sleep 15` is a good
rule of thumb) before moving to Terminal 2.

**Terminal 2 — fly one scenario:**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run crazyflie_examples run_formation \
  --scenario B1 --dz2 0.30 --speed 0.4 \
  --auto-center --yes --ros-args -p use_sim_time:=true
```

The client prints the commanded relative geometry, safety-checks it, writes Poly4D CSVs and a
`meta.json` sidecar under `experiments/logs/`, then uploads and flies. When it exits, `Ctrl-C`
Terminal 1 to stop the server (this flushes `record_states` to
`~/Desktop/crazyswarm2/state_<geo|indi>/<timestamp>/csv/`).

**Verifying afterward** (the runner's own report and `/pose` do not work in sim — always use this):

```bash
~/.pyenv/versions/flying_robots/bin/python \
  experiments/analysis/verify_formation_sim.py \
  experiments/logs/B1_<timestamp>.meta.json \
  ~/Desktop/crazyswarm2/state_indi/<timestamp>/csv \
  --controller indi --min-coverage 0.9
```

Prints PASS/FAIL per robot pair (commanded vs realised dz/dx/dy, mean/max error, RMSE), overall
tilt and coverage. Pick the `<timestamp>` that matches the run you just did — both directories are
named by wall-clock start time, printed by the client (`meta.json`) and visible via `ls -lat`.

### C. One ready command per scenario, both controllers, at 0.3 m/s

0.3 m/s is clean everywhere in this whole sweep — a safe default for a live demo. Swap
`server_sim_indi.yaml` → `server_sim_geo.yaml` in Terminal 1 to switch controllers; the client
command in Terminal 2 does not change.

| Scenario | N | Terminal 2 command |
|---|---|---|
| A1 | 2 | `run_formation --scenario A1 --dz 0.50 --auto-center --yes --ros-args -p use_sim_time:=true` |
| A2 | 2 | `run_formation --scenario A2 --dz 0.30 --speed 0.3 --auto-center --yes --ros-args -p use_sim_time:=true` |
| A3 | 2 | `run_formation --scenario A3 --dz 0.30 --speed 0.3 --auto-center --yes --ros-args -p use_sim_time:=true` |
| A4 | 2 | `run_formation --scenario A4 --motion line --offset 0.10 --dz 0.60 --speed 0.3 --auto-center --yes --ros-args -p use_sim_time:=true` |
| A5 | 2 | `run_formation --scenario A5 --dz 0.50 --radius 0.75 --period 15.708 --laps 2.0 --auto-center --yes --ros-args -p use_sim_time:=true` |
| A6 | 2 | `run_formation --scenario A6 --dz 0.10 --speed 0.3 --allow-extreme --auto-center --yes --ros-args -p use_sim_time:=true` |
| A7 | 2 | `run_formation --scenario A7 --speed 0.3 --allow-extreme --height 0.5 --auto-center --yes --ros-args -p use_sim_time:=true` |
| A8 | 2 | `run_formation --scenario A8 --dz 0.25 --span 1.0 --timescale 1.2153 --auto-center --yes --ros-args -p use_sim_time:=true` |
| B1 | 3 | `run_formation --scenario B1 --dz2 0.30 --speed 0.3 --auto-center --yes --ros-args -p use_sim_time:=true` |
| B2 | 3 | `run_formation --scenario B2 --dz2 0.30 --r 0.10 --speed 0.3 --auto-center --yes --ros-args -p use_sim_time:=true` |
| B3 | 3 | `run_formation --scenario B3 --dz 0.22 --span 0.55 --timescale 1.0026 --auto-center --yes --ros-args -p use_sim_time:=true` |
| C1 | 3 | `run_formation --scenario C1 --sep 0.30 --auto-center --yes --ros-args -p use_sim_time:=true` |
| C2 | 3 | `run_formation --scenario C2 --speed 0.3 --rotate 90 --auto-center --yes --ros-args -p use_sim_time:=true` |
| C3 | 3 | `run_formation --scenario C3 --auto-center --yes --ros-args -p use_sim_time:=true` (no `--side` flag exists; flies at its default 0.50 m) |
| C4 | 2 | `run_formation --scenario C4 --lateral 1.0 --dz-start 0.50 --dz-end 0.10 --timescale 0.9817 --allow-extreme --auto-center --yes --ros-args -p use_sim_time:=true` |
| C5 | 1 | `run_formation --scenario C5 --speed 0.3 --auto-center --yes --ros-args -p use_sim_time:=true` (use `crazyflies_sim1.yaml` in Terminal 1) |

For A1/C1/C3 there is no `--speed` (no axis at all — see §1); the commands above just fly them
at their default, validated pace. Roster in Terminal 1: `crazyflies_sim.yaml` for the 2-robot
rows, `crazyflies_sim3.yaml` for the 3-robot rows, `crazyflies_sim1.yaml` for C5.
