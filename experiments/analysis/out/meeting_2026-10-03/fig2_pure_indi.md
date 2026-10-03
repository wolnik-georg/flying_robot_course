# Fig. 2 — Pure INDI variants (2026-10-02)

![fig2](docs/meetings/assets/2026-10-03/fig2_pure_indi_height_error.png)

![fig2b](docs/meetings/assets/2026-10-03/fig2b_a8_crossing_separation.png)

## Rules
Formation radio CSVs + sidecar meta; steady window = liftoff+6 s through hold, tilt≤25°.
**cf_second ref:** only paired flights where cf_second has controller=6, ctrl_mode=0, max tilt ≤ 25°, valid steady window.
**Ours A8 median:** only `19-09-54` and `19-11-30` (ki_z=0, post-fix). Hollow: `18-57-49`, `18-59-17` (pre-fix crash), `19-08-13` (abort).

## Summary table (**high confidence** on listed flights)

| Scenario | Variant | Median mean err (cm) | Flights in median |
|---|---|---:|---|
| A1 | Omar C (c=9) | 2.94 | median over all included flights: `A1_cf5_2026-10-02_18-36-18.csv`, `A1_cf5_2026-10-02_18-37-33.csv` |
| A1 | Omar Rust (c=10) | -9.14 | median over all included flights: `A1_cf5_2026-10-02_18-48-33.csv`, `A1_cf5_2026-10-02_18-50-20.csv` |
| A1 | Ours full INDI (c=6, mode=3) | -18.15 | median over all included flights: `A1_cf5_2026-10-02_19-13-24.csv`, `A1_cf5_2026-10-02_19-15-47.csv` |
| A8 | Omar C (c=9) | 20.59 | median over all included flights: `A8_cf5_2026-10-02_18-23-23.csv`, `A8_cf5_2026-10-02_18-24-57.csv` |
| A8 | Omar Rust (c=10) | 19.46 | median over all included flights: `A8_cf5_2026-10-02_18-45-09.csv`, `A8_cf5_2026-10-02_18-46-45.csv` |
| A8 | Ours full INDI (c=6, mode=3) | 3.43 | median over clean post-ki_z=0 only: `A8_cf5_2026-10-02_19-09-54.csv`, `A8_cf5_2026-10-02_19-11-30.csv` |

## Per-flight detail

| File | mean err (cm) | max tilt (°) | in median? |
|---|---:|---:|---|
| `A1_cf5_2026-10-02_18-36-18.csv` | 3.507598888888889 | 31 | yes |
| `A1_cf5_2026-10-02_18-37-33.csv` | 2.363074585635359 | 23 | yes |
| `A1_cf5_2026-10-02_18-48-33.csv` | -6.977093888888889 | 63 | yes |
| `A1_cf5_2026-10-02_18-50-20.csv` | -11.310220555555556 | 38 | yes |
| `A1_cf5_2026-10-02_19-13-24.csv` | -17.080312154696127 | 29 | yes |
| `A1_cf5_2026-10-02_19-15-47.csv` | -19.222109444444445 | 39 | yes |
| `A8_cf5_2026-10-02_18-22-05.csv` | — | 180 | no |
| `A8_cf5_2026-10-02_18-23-23.csv` | 18.895429797979798 | 56 | yes |
| `A8_cf5_2026-10-02_18-24-57.csv` | 22.277110101010102 | 34 | yes |
| `A8_cf5_2026-10-02_18-45-09.csv` | 19.465789873417723 | 33 | yes |
| `A8_cf5_2026-10-02_18-46-45.csv` | 19.463445428571426 | 34 | yes |
| `A8_cf5_2026-10-02_18-57-49.csv` | -35.077731168831164 | 169 | no |
| `A8_cf5_2026-10-02_18-59-17.csv` | -52.399930250000004 | 170 | no |
| `A8_cf5_2026-10-02_19-08-13.csv` | -17.128478333333334 | 18 | no |
| `A8_cf5_2026-10-02_19-09-54.csv` | 3.5167037499999996 | 16 | yes |
| `A8_cf5_2026-10-02_19-11-30.csv` | 3.344923940149626 | 11 | yes |

## cf_second reference (included n=12)
- `A1_cf_second_2026-10-02_18-36-18.csv` (pair `A1_cf5_2026-10-02_18-36-18.csv`): -1.58 cm
- `A1_cf_second_2026-10-02_18-37-33.csv` (pair `A1_cf5_2026-10-02_18-37-33.csv`): -1.48 cm
- `A1_cf_second_2026-10-02_18-48-33.csv` (pair `A1_cf5_2026-10-02_18-48-33.csv`): -1.53 cm
- `A1_cf_second_2026-10-02_18-50-20.csv` (pair `A1_cf5_2026-10-02_18-50-20.csv`): -1.56 cm
- `A1_cf_second_2026-10-02_19-15-47.csv` (pair `A1_cf5_2026-10-02_19-15-47.csv`): -2.10 cm
- `A8_cf_second_2026-10-02_18-23-23.csv` (pair `A8_cf5_2026-10-02_18-23-23.csv`): -1.13 cm
- `A8_cf_second_2026-10-02_18-24-57.csv` (pair `A8_cf5_2026-10-02_18-24-57.csv`): -1.26 cm
- `A8_cf_second_2026-10-02_18-45-09.csv` (pair `A8_cf5_2026-10-02_18-45-09.csv`): -1.59 cm
- `A8_cf_second_2026-10-02_18-46-45.csv` (pair `A8_cf5_2026-10-02_18-46-45.csv`): -1.54 cm
- `A8_cf_second_2026-10-02_19-08-13.csv` (pair `A8_cf5_2026-10-02_19-08-13.csv`): -1.60 cm
- `A8_cf_second_2026-10-02_19-09-54.csv` (pair `A8_cf5_2026-10-02_19-09-54.csv`): -1.68 cm
- `A8_cf_second_2026-10-02_19-11-30.csv` (pair `A8_cf5_2026-10-02_19-11-30.csv`): -1.71 cm
## cf_second excluded (n=4)
- `A1_cf_second_2026-10-02_19-13-24.csv`: no steady samples (tilt>25° or window empty)
- `A8_cf_second_2026-10-02_18-22-05.csv`: max tilt 73° > 25° (mean -79.1 cm)
- `A8_cf_second_2026-10-02_18-57-49.csv`: max tilt 180° > 25° (mean -32.2 cm)
- `A8_cf_second_2026-10-02_18-59-17.csv`: max tilt 56° > 25° (mean -102.4 cm)

## fig2b
Kept: separation at first y-crossing for clean A8 flights; interpret as radio Δz vs commanded dz only (medium confidence).

## What this figure does not show
uSD Omar error channels; pre-10-02 INDI history.
