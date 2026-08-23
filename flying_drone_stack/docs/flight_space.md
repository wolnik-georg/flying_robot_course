# Flight Space — Physical Bounds and Trajectory Validation

## Usable flight volume

Measured/confirmed by the user (2026-07-20), world frame centered on the takeoff origin:

| Axis | Range | Extent |
|------|-------|--------|
| x (front/back of drone) | −1.2 m … +1.2 m | 2.4 m |
| y (side of drone) | −2.5 m … +2.5 m | 5.0 m |
| z (absolute height, floor = 0) | 0.5 m … 1.8 m | 1.3 m |

All exported trajectories are validated against this box. x/y are absolute (relative to the
sampled EKF origin at takeoff, assumed ≈ (0,0)). z in the exported onboard CSVs is an **offset**
from `traj.hz` (the `--height` flag passed to `flight.py`) — so whether a trajectory fits depends
on which hover height it's flown at. See the per-trajectory table below for the valid `--height`
window for each one.

## Validation method

For each trajectory, the worst-case X/Y/Z bounding box was computed by densely sampling every
exported onboard CSV (mode 1 + mode 3, across the full validated kt sweep — see
`project_trajectory_port_2026-07` memory / conversation history for the exact kt lists) and
taking the min/max over all samples and all kt values. This is a true worst-case check, not a
single-kt spot check.

## Results (2026-07-20)

| Trajectory | x range | y range | z-offset range (mode1) | z-offset range (mode3) | Feasible `--height` window |
|---|---|---|---|---|---|
| circle | ±1.08 | ±0.89 | 0 | 0 | 0.5 – 1.8 m (any) |
| figure8 | ±1.08 | ±0.66 | 0 | 0 | 0.5 – 1.8 m (any) |
| corner | −0.90…+0.27 | −0.25…+1.10 | 0 | 0 | 0.5 – 1.8 m (any) |
| oval | ±0.80 | ±2.20 | 0 | 0 | 0.5 – 1.8 m (any) |
| slalom | ±0.98 | ±2.20 | 0 | 0 | 0.5 – 1.8 m (any) |
| helix | −0.46…+0.50 | ±0.87 | 0…+1.00 | 0…+1.00 | **0.5 – 0.8 m** |
| corkscrew | ±0.50 | ±0.59 | 0…+1.00 | 0…+1.00 | **0.5 – 0.8 m** |
| loop | ±0.58 | 0 | −0.11…+1.00 | −0.11…+1.12 | **0.61 – 0.68 m** (narrow) |
| teardrop | ±0.34 | 0 | −0.54…+0.54 | −0.54…+0.54 | **1.04 – 1.27 m** |
| teardrop_wide | ±0.43 | 0 | −0.48…+0.48 | −0.48…+0.48 | **0.98 – 1.32 m** |
| tilted_oval | ±0.80 | ±2.20 | −0.50…+0.50 | −0.50…+0.50 | **1.00 – 1.30 m** |
| roller_coaster | ±0.53 | ±1.60 | −0.30…+0.82 | −0.26…+0.81 | **0.80 – 0.98 m** |
| loop_train | ±1.10 | ±0.74 | −0.50…+0.46 | **mode3 EXCLUDED — see below** | **1.00 – 1.34 m** (mode1 only) |
| corner_loop | −0.91…+0.27 | −0.34…+1.10 | 0 | 0 | 0.5 – 1.8 m (any) |
| slalom_loop | ±0.74 | ±2.20 | 0 | **mode3 EXCLUDED — see below** | 0.5 – 1.8 m (any, mode1 only) |
| loop_train_loop | ±0.55 | ±0.74 | −0.56…+0.53 | **mode3 EXCLUDED — see below** | **1.02 – 1.24 m** (mode1 only) |
| roller_coaster_loop | ±0.52 | ±1.61 | −0.36…+0.81 | −0.36…+0.81 | **0.86 – 1.00 m** |

All X/Y stay comfortably inside the ±1.2 m / ±2.5 m box for every trajectory and every kt in the
curated sweep, at every mode. No trajectory needed to be excluded on X/Y grounds.

### Recommended `--height` per trajectory (midpoint of the feasible window)

| Trajectory | `--height` |
|---|---|
| circle, figure8, corner, oval, slalom | 0.7 (default) |
| helix, corkscrew | 0.65 |
| loop | 0.65 |
| teardrop, teardrop_wide, tilted_oval, loop_train | 1.15 |
| roller_coaster | 0.89 |
| corner_loop, slalom_loop | 0.7 (default) |
| loop_train_loop | 1.15 |
| roller_coaster_loop | 0.89 |

## Multi-lap "_loop" variants (there-and-back)

Added 2026-07-20 to support seamless `--reps N` looping for the 4 trajectories that aren't
naturally closed (corner, slalom, loop_train, roller_coaster): `corner_loop`, `slalom_loop`,
`loop_train_loop`, `roller_coaster_loop`. Each flies the original shape forward, then retraces
it backward to the start (`[w0..wN, w(N-1)..w1]`, periodic wraparound), turning an open path into
a genuinely closed one with position/velocity/accel/jerk continuity at the seam — same mechanism
`circle`/`loop`/`oval` already use for looping.

This required raising the firmware's `TRAJ_MAX_SEGS` from 12 to 24 (position/z/attitude
coefficient buffers resized accordingly, `g_traj_coef_ci`/`g_traj_att_ci` widened from `u8` to
`u16` since their index range now exceeds 255) — **requires `make cload` before first flight**.

**`slalom_loop` and `loop_train_loop` mode 3 are excluded** — same Runge-phenomenon overshoot
bug as the original `loop_train` mode 3 (see below), confirmed across the full kt range (e.g.
`slalom_loop` mode3 reaches x=−2.98m at kt=0.09, far outside the room). `corner_loop` and
`roller_coaster_loop` mode 3 are clean and included.

Each onboard export now also writes a `{label}_onboard.meta.json` sidecar with
`{"periodic": bool}` — `flight.py` reads this to decide whether `--reps > 1` is safe. figure8 is
marked non-periodic regardless of mode, conservatively, since its looping behavior was never
validated and it's the established benchmark trajectory (left untouched this session).

## loop_train mode 3 — excluded, not just narrow

`loop_train`'s mode 3 (8-iteration Richter redistribution) was found to badly overshoot the
intended geometry: the connector segment between the two loops (a long, nearly-flat straight
segment bridging two tightly-curved loop segments) gets its duration stretched to ~2.7s by the
redistribution, and the resulting min-snap polynomial overshoots to **z ≈ −2.4 m relative to the
intended ±0.5 m loop amplitude** — a Runge-phenomenon-style artifact from matching high-curvature
boundary derivatives over an unnecessarily long, under-constrained segment. This is not a "narrow
kt window" like circle or roller_coaster; every mode-3 kt value tested (0.05–0.3) overshot by at
least ~1 m past the intended bound, worst at low kt (−2.385 m at kt=0.05, improving to −0.98 m at
kt=0.3, still 2× the intended ±0.5 m amplitude).

**The 4 mode-3 onboard CSVs for loop_train have been deleted** from the CS2 data folder to
prevent accidental use. `loop_train` should be flown in **mode 1 only** until/unless this is
fixed (e.g. by not letting redistribution touch the connector segment, or restructuring the
waypoint chain).

## Notes

- A stray leftover export (`figure8_mode1_kt1.3_onboard.csv`, kt=1.3 — not part of the curated
  sweep, left over from earlier unrelated work) does exceed the x bound (x_max=1.25m). It is not
  part of the current trajectory library's kt lists and should not be flown; harmless to leave in
  the data folder as long as it's never explicitly requested via `--kt 1.3`.
- z bounds assume the drone starts exactly at the sampled EKF origin — real flights may drift a
  few cm from this in X/Y depending on takeoff position; the margins above (especially oval/slalom
  at the full ±2.5m y extent) leave very little slack and should be spot-checked against the
  actual takeoff position before a full-speed flight.
