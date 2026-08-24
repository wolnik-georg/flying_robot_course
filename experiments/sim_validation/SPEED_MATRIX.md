# Speed & separation parameterisation — fix report and validation

**Date:** 24 August 2026 · **Scope:** three defects found in the Phase 0 audit. The formation
library itself is unchanged; no hardware configs were touched and nothing was flown.

---

## 1. What was wrong

| # | Defect | Evidence |
|---|---|---|
| 1 | `--speed` was **silently ignored** on `circle` and `lemniscate` paths, which are paced by a period. Those are the **defaults** for A2, A4, B1, B2 — the main tracking scenarios | A2 built at `speed=0.2` and `speed=0.5` produced byte-identical curves |
| 2 | The sidecar **recorded a speed that was never flown** | A2/B1/B2 logged `speed=0.4`, flew **0.63 m/s**; A4 logged 0.4, flew **0.50** |
| 3 | Every builder ends in `**_`, so **any unknown or inapplicable flag was swallowed** | `build('A3', spede=0.5)` accepted, ran at the 0.4 default |

Consequence: the tracking scenarios had only ever run at **0.63 m/s**, above the 0.2–0.5 m/s band
this work targets, and speed was never actually a variable. Verification could not catch it — it
rebuilds from `params`, and rebuilding with `speed=0.4` regenerates the same circle. Consistent,
and wrong.

**The geometry results are unaffected.** Separations were verified independently and correctly, and
the defaults are unchanged by the fix. Only the speed field was wrong.

---

## 2. What changed

### Fix 1 — speed reaches period-paced curves

`build()` converts a requested speed into the period that delivers it. Peak speed scales exactly as
`1/period` for fixed geometry, so one reference build gives the conversion.

`--period` still works and **wins over `--speed`** when both are given.

> **The conversion is not `T = 2πr/v`.** For a circle it reduces to that (r = 0.75 m → 0.30 m/s =
> 15.71 s). For a **lemniscate it does not** — that formula is **42% low** at the default radius,
> because the fastest point of a figure-of-eight is not on a circle of radius *a*. A4 would have
> flown 0.42 m/s when asked for 0.30. Scaling from a measurement of the actual curve is correct for
> both.

### Fix 2 — realised speed is recorded and verified

`Scenario.realised` carries the peak speed, peak acceleration, effective period, path kind and
whether the period was pinned. It is written to the sidecar and to the CSV metadata, **separate
from `params`**, because verification rebuilds from `params` and that rebuild must take exactly the
builder's own arguments.

`verify_formation_sim.py` now compares the commanded peak against the peak actually flown
(`SPEED_TOL_ABS = 0.08 m/s` or `SPEED_TOL_REL = 25%`, whichever is larger — wide because it compares
a commanded peak against a tracked one, which differ by lag, overshoot and differentiation noise).

### Fix 3 — inapplicable parameters are an error

`build()` rejects any flag the scenario does not take, and any `--speed` that has no effect on the
chosen path. `run_formation` reports it as a clean message, not a traceback.

> The effectiveness test is **not** `peak == request`. A7 traverses at 0.30 m/s while its other
> vehicle descends at 0.39 — legitimate, and an equality check rejects it. The test perturbs the
> request and asks whether the curves move, which is what actually separates *honoured* from
> *silently dropped*.

---

## 3. Backward compatibility

| Check | Result |
|---|---|
| Default `params` for all 16 scenarios vs pre-fix git version | **identical** |
| Default peak speed for all 16 scenarios | **identical** |
| `check_spec` self-test | **38 cases, 0 failures** |
| Round-trip: sidecar `params` rebuild the same scenario | **exact** |
| **40 real sidecars from the 34-case matrix replay** | **40/40 without the conversion firing** |

The replay property is load-bearing and deliberate: `effective()` records every signature
parameter, so a sidecar always carries an explicit `period`, which pins it — a past run rebuilt
from its own metadata can never re-run the conversion and shift the time base underneath its
recorded states. **If `effective()` ever stops recording defaults, that guarantee goes with it.**

---

## 4. Validation matrix — 16 cells, 16 PASS

`experiments/analysis/run_speed_matrix.sh` · results in `speed_matrix_results.md`

| Scenario | Ctrl | Δz | Speed | Lateral | Period | mean\|ez\| | RMSE | Tilt | Diverged |
|---|---|---|---|---|---|---|---|---|---|
| A2 | geo | 0.40 | 0.30 | – | 15.71 | 32.0 mm | 32.1 | 1.4° | no |
| A2 | geo | 0.40 | 0.50 | – | 9.42 | 30.7 mm | 30.8 | 3.8° | no |
| A2 | geo | 0.30 | 0.30 | – | 15.71 | 37.3 mm | 37.3 | 1.4° | no |
| A2 | geo | 0.30 | 0.50 | – | 9.42 | 36.0 mm | 36.1 | 3.8° | no |
| A4 | geo | 0.60 | 0.30 | 0.10 | 13.33 | 2.1 mm | 3.3 | 1.5° | no |
| A4 | geo | 0.60 | 0.30 | 0.20 | 13.33 | 0.9 mm | 1.2 | 1.5° | no |
| A1 | geo | 0.50 | hover | – | – | 27.6 mm | 27.6 | 0.1° | no |
| A1 | geo | 0.30 | hover | – | – | 39.8 mm | 39.8 | 0.1° | no |
| A2 | indi | 0.40 | 0.30 | – | 15.71 | 0.1 mm | 0.1 | 1.4° | no |
| A2 | indi | 0.40 | 0.50 | – | 9.42 | 0.2 mm | 0.3 | 3.8° | no |
| A2 | indi | 0.30 | 0.30 | – | 15.71 | 0.1 mm | 0.1 | 1.4° | no |
| A2 | indi | 0.30 | 0.50 | – | 9.42 | 0.2 mm | 0.3 | 3.8° | no |
| A4 | indi | 0.60 | 0.30 | 0.10 | 13.33 | 0.1 mm | 0.5 | 1.5° | no |
| A4 | indi | 0.60 | 0.30 | 0.20 | 13.33 | 0.0 mm | 0.5 | 1.5° | no |
| A1 | indi | 0.50 | hover | – | – | 0.0 mm | 0.0 | 0.0° | no |
| A1 | indi | 0.30 | hover | – | – | 0.0 mm | 0.0 | 0.0° | no |

Periods confirm the conversion: **15.71 s ↔ 0.30 m/s**, **9.42 s ↔ 0.50 m/s** on the r = 0.75 m
circle, **13.33 s ↔ 0.30 m/s** on the A4 lemniscate.

### Negative control — the speed check is not vacuous

The same recorded flight, verified twice:

| Metadata | Geometry | Speed | Verdict |
|---|---|---|---|
| Honest (A1 hover, dz 0.30) | dz −0.300, 0.0 mm | 0.000 vs cmd 0.000 | **PASS** |
| Doctored (claims A2 circle at 0.5 m/s) | dz −0.300, 0.0 mm — still passes | 0.000 vs cmd 0.500 | **FAIL — speed out of tolerance** |

Geometry passes and speed fails: precisely the error class that was previously invisible.

---

## 5. What the numbers suggest — and what they do not

**Separation dominates; speed barely registers.** Under geometric control, tightening Δz from 0.40
to 0.30 costs about 5 mm, while 0.30 → 0.50 m/s changes almost nothing (32.0 → 30.7 mm), at both
separations. Consistent with the RQ2 premise that the interaction is a strong function of distance
and a weaker one of lateral speed.

**Lateral offset moves the lower vehicle out of the wash core** — A4 sits at 0.9–2.1 mm against
A2's 30+ mm, and offset 0.20 is cleaner than 0.10.

**INDI holds ~0.1 mm across the whole speed range**, post-sign-fix, matching `docs/12`.

> **These are simulation numbers, and the interaction is the Neural-Swarm2 model.** They describe
> what that model does, not what the air does. The speed-insensitivity in particular is a property
> of a downwash model that is primarily a function of relative position — it is a hypothesis for
> the lab, not a finding. The A1 hover cells are the most suspect: their only signal is
> steady-state sag, which is exactly where the simulator's known position-damping mismatch bites.

---

## 6. Caveat on reproducing these runs

`run_sim_matrix.sh` does `rm -rf state_$CTRL` at the start of every run, so the `states` column in
the results files points at directories that **no longer exist for all but the last run of each
controller**. The verification happened at the time and its numbers stand, but the raw states
cannot be re-examined after the fact. Worth changing before any run whose states matter as data.
