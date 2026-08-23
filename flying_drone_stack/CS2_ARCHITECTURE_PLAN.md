# CS2 Architecture Integration Plan

> ## ⚠️ HISTORICAL (last updated 2026-05-17)
> This document describes the original CS2 integration, which it records as complete. The architecture has since moved
> from Mode D to Mode E -- see `docs/08_Trajectory_Upload_Paths.md`.
>
> **The current source of truth for project status is
> [`docs/07_Thesis_Progress_Checklist.md`](../docs/07_Thesis_Progress_Checklist.md);
> [`docs/00_README.md`](../docs/00_README.md) maps every other document.** Kept because it
> records what was decided and why -- do not act on it as if it were current.

## Status (as of 2026-05-17) — COMPLETE

### What is done ✅
- Geometric SE(3) controller on firmware (`controller=6`) — validated with mocap
- Mocap → CS2 server → `send_extpose` → drone EKF — validated, figure-8 flew cleanly
- Trajectory export: `export_poly4d.rs` writes Poly4D CSVs directly to `crazyswarm2/crazyflie_examples/data/`
- Generic flight script: `crazyswarm2/crazyflie_examples/crazyflie_examples/flight.py` — any trajectory/mode/kt via args
- Logging: CS2 custom topics at 20 Hz, same 18 columns as Rust, saves to `Controls/logs/` with meta headers
- `analyze_flight.py` works on CS2 logs without any changes

### What still needs work
- Gains tuning (Block S is current best, Block T to try next — needs logging data from real flight)
- INDI controller (next major milestone after gains are stable)

---

## Full CS2 Architecture

```
Rust motion planning (unchanged)
    │
    ▼ export_poly4d --trajectory figure8 --mode 1 --kt 0.008
    │  → crazyswarm2/crazyflie_examples/data/figure8_mode1_kt0.008.csv
    │
OptiTrack → motion_capture_tracking node → CS2 server
                                               │
                                    extpose → drone EKF (Kalman, estimator=2)
                                               │
                              geometric SE(3) controller (firmware_app, controller=6)
                                               │
                          Poly4D trajectory uploaded via CS2, evaluated by HLC at 1000 Hz
                                               │
                          Controls/logs/{label}_{timestamp}.csv (18 cols + meta headers)
```

---

## Key architectural decisions

| Decision | Choice | Reason |
|---|---|---|
| Radio ownership | CS2 only | Rust + CS2 simultaneously corrupts EKF (z=111m symptom) |
| Pose source | OptiTrack → motion_capture_tracking → CS2 | No drift; globally consistent |
| Controller | geometric SE(3) (`controller=6`) | INDI will build on top of it |
| Trajectory evaluation | HLC at 1000 Hz (8-coeff Poly4D) | Standard Bitcraze path; equivalent tracking to Mode D 500 Hz |
| Trajectory source | Rust motion planner → `export_poly4d` → CS2 upload | Preserves all 4 planning modes |
| Log format | Same 18 columns as Rust + meta headers | `analyze_flight.py` works unchanged |

---

## Gain tuning results (mocap, geometric SE3)

| Block | KP | KV | KR | KW | Result |
|---|---|---|---|---|---|
| O | 28 | 3 | 0.010 | 0.00110 | still wobbly |
| R | 28 | 2 | 0.010 | 0.00110 | slightly better |
| S | 28 | 2 | 0.007 | 0.00115 | ★ best so far |
| T | 36 | 3 | 0.007 | 0.00115 | to try next session |

**Key finding**: attitude gains `KR=0.010` contribute to wobble with mocap. Mellinger uses `KR=0.007, KW=0.00115` and works perfectly → those are now the mocap attitude baseline. With mocap, EKF differentiates positions for velocity → keep KV low (2–3) to avoid noise amplification.

**Strategy**: tune only enough to get stable hover + clean trajectory tracking, then move to INDI. The position gains (KP, KV) from the geometric controller carry over to INDI's outer loop; attitude gains (KR, KW) will be retuned for INDI anyway.

---

## Next steps

1. Fly with Block S, get logging CSV, run `analyze_flight.py` to see XY RMSE baseline
2. Try Block T if RMSE is still poor — one session max
3. Freeze gains, move to INDI
