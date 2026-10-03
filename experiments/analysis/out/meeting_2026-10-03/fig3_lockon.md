# Fig. 3 — NS2 2026-10-03 lock-on timeline

![fig3](docs/meetings/assets/2026-10-03/fig3_ns2_lockon_timeline.png)

## Rules (**high confidence**, `lock_on_scan.py`)
- Shared radio clock; interpolate cf_second onto cf5 timestamps.
- Liftoff: first cf5 `pos_z > 0.12` m (scan script default).
- Lock-on: earliest time such that |Δy|<5 cm and |Δz|<15 cm for remainder of flight (≥0.4 s sustained).
- Vertical lines: liftoff, first tilt>20°, lock-on.

| Flight | liftoff (s) | first tilt>20° (s) | lock-on (s) | max tilt (°) | lock after flip? |
|---|---:|---:|---:|---:|---|
| A8_cf5_2026-10-03_13-10-32.csv | 8.6 | 12.6 | 15.6 | 119.3 | yes |
| A8_cf5_2026-10-03_13-15-00.csv | 8.5 | 8.8 | 16.0 | 179.9 | yes |

## What this figure does not show
Does not prove mocap ID swap mechanism (radio positions only); no uSD estimator state; no RNN predictions.
