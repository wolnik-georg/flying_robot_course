# Table — geometric Z-integral z tracking

![fig](docs/meetings/assets/2026-10-03/fig_z_tracking_geometric.png)

## Window rule
- **Formation (radio):** liftoff = first `pos_z` > 0.05 m; steady = liftoff + 6.0 s … hold end; tilt ≤ 25°.
- **Solo (Controls):** segment start = first `z` ≥ 0.8 m (logs begin on ground at z≈−0.02 m; first z>0.05 m is **not** hover); steady = segment start + 6.0 s … hold end; tilt ≤ 25°.
- **Plot time axis:** seconds since that segment start (solo) or liftoff (formation).
- **Steady end:** walk forward from steady start; last sample with `z ≥ max(plateau_median − 5 cm, z_cmd − 2 cm)` before sustained descent (solo + formation); formation also capped by A1 `param_hold` / A8 `duration_s`.
- **Figure-8 time:** lap resets in raw CSV are stitched into one monotonic timeline before windows/plots.

## Exclusions (listed per panel; not drawn)
- tilt > 45°; wrong controller/ki_z; no segment start; no steady samples.

## Hover — summary (steady window only)
- n=3 | mean error 0.15 cm | mean |error| 0.22 cm | max |error| 1.99 cm

| flight | clean | mean (cm) | mean |e| (cm) | max |e| (cm) | note |
|---|---:|---:|---:|---:|---|
| `hover_mode1_kt0.008_2026-09-30_19-16-19.csv` | yes | 0.15 | 0.23 | 1.99 | steady end = last sample before z >5 cm below plateau median & falling; stitched lap time |
| `hover_mode1_kt0.008_2026-09-30_19-16-56.csv` | yes | 0.17 | 0.22 | 1.88 | steady end = last sample before z >5 cm below plateau median & falling; stitched lap time |
| `hover_mode1_kt0.008_2026-09-30_19-20-10.csv` | yes | 0.14 | 0.20 | 1.79 | steady end = last sample before z >5 cm below plateau median & falling; stitched lap time |

## Figure-8 — summary (steady window only)
- n=2 | mean error 0.68 cm | mean |error| 0.84 cm | max |error| 1.92 cm

| flight | clean | mean (cm) | mean |e| (cm) | max |e| (cm) | note |
|---|---:|---:|---:|---:|---|
| `figure8_mode1_kt0.05_2026-09-30_19-20-41.csv` | yes | 0.22 | 0.53 | 1.92 | steady end = last sample before z >5 cm below plateau median & falling; stitched lap time |
| `figure8_mode1_kt0.05_2026-09-30_19-21-14.csv` | yes | 1.15 | 1.15 | 1.72 | steady end = last sample before z >5 cm below plateau median & falling; stitched lap time |

## A1 — summary (steady window only)
- n=5 | mean error -1.65 cm | mean |error| 1.65 cm | max |error| 2.31 cm

| flight | clean | mean (cm) | mean |e| (cm) | max |e| (cm) | note |
|---|---:|---:|---:|---:|---|
| `A1_cf_second_2026-10-02_18-36-18.csv` | yes | -1.58 | 1.58 | 1.90 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A1_cf_second_2026-10-02_18-37-33.csv` | yes | -1.48 | 1.48 | 1.80 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A1_cf_second_2026-10-02_18-48-33.csv` | yes | -1.53 | 1.53 | 1.71 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A1_cf_second_2026-10-02_18-50-20.csv` | yes | -1.56 | 1.56 | 1.81 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A1_cf_second_2026-10-02_19-13-24.csv` | no | nan | nan | nan | no steady samples |
| `A1_cf_second_2026-10-02_19-15-47.csv` | yes | -2.10 | 2.10 | 2.31 | ki_z=16 from session yaml (not in cf_second radio # meta) |

## A8 top drone (cf_second) — summary (steady window only)
- n=8 | mean error -1.66 cm | mean |error| 1.66 cm | max |error| 2.41 cm

| flight | clean | mean (cm) | mean |e| (cm) | max |e| (cm) | note |
|---|---:|---:|---:|---:|---|
| `A8_cf_second_2026-10-02_18-23-23.csv` | yes | -1.12 | 1.12 | 1.41 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A8_cf_second_2026-10-02_18-24-57.csv` | yes | -1.25 | 1.25 | 1.51 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A8_cf_second_2026-10-02_18-45-09.csv` | yes | -1.60 | 1.60 | 1.91 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A8_cf_second_2026-10-02_18-46-45.csv` | yes | -1.55 | 1.55 | 1.81 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A8_cf_second_2026-10-02_18-57-49.csv` | no | nan | nan | nan | max tilt 179.7° > 45° |
| `A8_cf_second_2026-10-02_18-59-17.csv` | no | nan | nan | nan | max tilt 56.4° > 45° |
| `A8_cf_second_2026-10-02_19-08-13.csv` | no | nan | nan | nan | no steady samples |
| `A8_cf_second_2026-10-02_19-09-54.csv` | yes | -1.68 | 1.68 | 2.01 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A8_cf_second_2026-10-02_19-11-30.csv` | yes | -1.71 | 1.71 | 2.01 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A8_cf_second_2026-10-03_13-10-32.csv` | yes | -2.23 | 2.23 | 2.41 | ki_z=16 from session yaml (not in cf_second radio # meta) |
| `A8_cf_second_2026-10-03_13-15-00.csv` | yes | -2.10 | 2.10 | 2.31 | ki_z=16 from session yaml (not in cf_second radio # meta) |

## A8 bottom drone (cf5) — summary (steady window only)
- n=2 | mean error 0.15 cm | mean |error| 0.89 cm | max |error| 8.30 cm

| flight | clean | mean (cm) | mean |e| (cm) | max |e| (cm) | note |
|---|---:|---:|---:|---:|---|
| `A8_cf5_2026-10-02_17-23-14.csv` | yes | 0.15 | 0.88 | 7.87 | ki_z=16 from session yaml (not in cf5 radio # meta) |
| `A8_cf5_2026-10-02_17-28-37.csv` | yes | 0.16 | 0.90 | 8.30 | ki_z=16 from session yaml (not in cf5 radio # meta) |

## Change vs previous table (window end = log end − 2.5 s)
- Previous max |error| on hovers included **landing descent**; descent-based end gives max |error| **≈2 cm** on 09-30 hovers (see Hover rows).

## All exclusions
- `A1_cf_second_2026-10-02_19-13-24.csv` (A1): no steady samples
- `A8_cf_second_2026-10-02_18-57-49.csv` (A8 top drone (cf_second)): max tilt 179.7° > 45°
- `A8_cf_second_2026-10-02_18-59-17.csv` (A8 top drone (cf_second)): max tilt 56.4° > 45°
- `A8_cf_second_2026-10-02_19-08-13.csv` (A8 top drone (cf_second)): no steady samples
