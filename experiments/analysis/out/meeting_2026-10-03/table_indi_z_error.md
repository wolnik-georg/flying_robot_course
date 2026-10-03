# Table — INDI z error (cf5 bottom, 2026-10-02)

## Steady window
- Liftoff: pos_z > 0.05 m; steady = liftoff + 6.0 s … hold end (A1: param_hold; A8: duration_s − 2.5 s); tilt ≤ 25.0°.
- Representative plot: **clean** flight closest to variant median mean error; legend time = HH-MM-SS stamp.
- **pre-fix:** 18-57-49, 18-59-17 (ki_z=16 tumbles). **aborted:** 19-08-13 (~18 s). **crashed (flip/abort):** 18-22-05 or tilt≈180°.
- **completed, tilt excursion >45°:** flight ran but peak |roll|/|pitch| > 45° (e.g. 18-23-23, 18-48-33).

### A1 Ours `19-15-47` — z→0 dips at t≈16 s and ≈23–25 s
- Full trace shows **pos_z near 0 m** after the hold window; this is **post-scenario landing / ground contact** on the radio log (A1 hold ends ~15 s after liftoff; landing follows).
- **Mean/max in the table use only the steady window** (tilt ≤ 25°, before landing); the dips are **not** included in those statistics.

| scenario | variant | flight | flag | mean z error (cm) | max |z error| (cm) | max tilt (°) |
|---|---|---|---|---:|---:|---:|
| A1 | Omar C | `A1_cf5_2026-10-02_18-36-18.csv` | clean | 3.51 | 9.21 | 30.6 |
| A1 | Omar C | `A1_cf5_2026-10-02_18-37-33.csv` | clean | 2.36 | 6.31 | 22.8 |
| A1 | Omar Rust | `A1_cf5_2026-10-02_18-48-33.csv` | completed, tilt excursion >45° | -6.98 | 12.33 | 62.9 |
| A1 | Omar Rust | `A1_cf5_2026-10-02_18-50-20.csv` | clean | -11.31 | 18.08 | 38.1 |
| A1 | Ours | `A1_cf5_2026-10-02_19-13-24.csv` | clean | -17.08 | 28.96 | 28.8 |
| A1 | Ours | `A1_cf5_2026-10-02_19-15-47.csv` | clean | -19.22 | 38.18 | 39.4 |
| A8 | Omar C | `A8_cf5_2026-10-02_18-22-05.csv` | crashed (flip/abort) | nan | nan | 180.0 |
| A8 | Omar C | `A8_cf5_2026-10-02_18-23-23.csv` | completed, tilt excursion >45° | 19.36 | 25.12 | 55.8 |
| A8 | Omar C | `A8_cf5_2026-10-02_18-24-57.csv` | clean | 22.94 | 27.61 | 33.8 |
| A8 | Omar Rust | `A8_cf5_2026-10-02_18-45-09.csv` | clean | 20.18 | 25.51 | 33.4 |
| A8 | Omar Rust | `A8_cf5_2026-10-02_18-46-45.csv` | clean | 18.90 | 24.32 | 33.7 |
| A8 | Ours | `A8_cf5_2026-10-02_18-57-49.csv` | pre-fix | -1.59 | 94.89 | 168.9 |
| A8 | Ours | `A8_cf5_2026-10-02_18-59-17.csv` | pre-fix | -52.40 | 52.40 | 169.7 |
| A8 | Ours | `A8_cf5_2026-10-02_19-08-13.csv` | aborted | -8.10 | 51.45 | 17.9 |
| A8 | Ours | `A8_cf5_2026-10-02_19-09-54.csv` | clean | 3.66 | 8.50 | 16.0 |
| A8 | Ours | `A8_cf5_2026-10-02_19-11-30.csv` | clean | 3.57 | 7.96 | 11.3 |

## Variant medians — clean only
- **A1 / Ours:** n=2, median = -18.15 cm
- **A1 / Omar C:** n=2, median = 2.94 cm
- **A1 / Omar Rust:** n=1, median = -11.31 cm
- **A8 / Ours:** n=2, median = 3.61 cm
- **A8 / Omar C:** n=1, median = 22.94 cm
- **A8 / Omar Rust:** n=2, median = 19.54 cm

## Variant medians — clean + completed tilt excursion >45°
- **A1 / Ours:** n=2, median = -18.15 cm
- **A1 / Omar C:** n=2, median = 2.94 cm
- **A1 / Omar Rust:** n=2, median = -9.14 cm
- **A8 / Ours:** n=2, median = 3.61 cm
- **A8 / Omar C:** n=2, median = 21.15 cm
- **A8 / Omar Rust:** n=2, median = 19.54 cm
