# Table — INDI z error (cf5 bottom, 2026-10-02)

## Steady window
- Liftoff: pos_z > 0.05 m; steady = liftoff + 6.0 s … hold end (A1: param_hold; A8: landing descent (first z > 30 cm below command) − 1.5 s); tilt ≤ 25.0°.
- Representative plot: **clean, complete** flight (log ends on the ground) closest to variant median mean error; legend time = HH-MM-SS stamp.
- **pre-fix:** 18-57-49, 18-59-17 (ki_z=16 tumbles). **aborted:** 19-08-13 (~18 s). **crashed (flip/abort):** 18-22-05 or tilt≈180°.
- **completed, tilt excursion >45°:** flight ran but peak |roll|/|pitch| > 45° (e.g. 18-23-23, 18-48-33).

### A1 Ours `19-15-47` — z→0 dips at t≈16 s and ≈23–25 s
- Full trace shows **pos_z near 0 m** after the hold window; this is **post-scenario landing / ground contact** on the radio log (A1 hold ends ~15 s after liftoff; landing follows).
- **Mean/max in the table use only the steady window** (tilt ≤ 25°, before landing); the dips are **not** included in those statistics.

| scenario | variant | flight | flag | log | mean z error (cm) | max |z error| (cm) | max tilt (°) |
|---|---|---|---|---|---:|---:|---:|
| A1 | Omar C | `A1_cf5_2026-10-02_18-36-18.csv` | clean | complete | 3.51 | 9.21 | 30.6 |
| A1 | Omar C | `A1_cf5_2026-10-02_18-37-33.csv` | clean | ends in air (truncated) | 2.36 | 6.31 | 22.8 |
| A1 | Omar Rust | `A1_cf5_2026-10-02_18-48-33.csv` | completed, tilt excursion >45° | complete | -6.98 | 12.33 | 62.9 |
| A1 | Omar Rust | `A1_cf5_2026-10-02_18-50-20.csv` | clean | complete | -11.31 | 18.08 | 38.1 |
| A1 | Ours | `A1_cf5_2026-10-02_19-13-24.csv` | clean | complete | -17.08 | 28.96 | 28.8 |
| A1 | Ours | `A1_cf5_2026-10-02_19-15-47.csv` | clean | complete | -19.22 | 38.18 | 39.4 |
| A8 | Omar C | `A8_cf5_2026-10-02_18-22-05.csv` | crashed (flip/abort) | complete | nan | nan | 180.0 |
| A8 | Omar C | `A8_cf5_2026-10-02_18-23-23.csv` | completed, tilt excursion >45° | complete | 18.68 | 25.12 | 55.8 |
| A8 | Omar C | `A8_cf5_2026-10-02_18-24-57.csv` | clean | complete | 21.99 | 29.51 | 33.8 |
| A8 | Omar Rust | `A8_cf5_2026-10-02_18-45-09.csv` | clean | complete | 18.65 | 25.51 | 33.4 |
| A8 | Omar Rust | `A8_cf5_2026-10-02_18-46-45.csv` | clean | ends in air (truncated) | 18.90 | 24.32 | 33.7 |
| A8 | Ours | `A8_cf5_2026-10-02_18-57-49.csv` | pre-fix | complete | nan | nan | 168.9 |
| A8 | Ours | `A8_cf5_2026-10-02_18-59-17.csv` | pre-fix | complete | nan | nan | 169.7 |
| A8 | Ours | `A8_cf5_2026-10-02_19-08-13.csv` | aborted | complete | 2.61 | 5.11 | 17.9 |
| A8 | Ours | `A8_cf5_2026-10-02_19-09-54.csv` | clean | complete | 3.53 | 8.50 | 16.0 |
| A8 | Ours | `A8_cf5_2026-10-02_19-11-30.csv` | clean | complete | 3.42 | 7.96 | 11.3 |

## Variant medians — clean only
- **A1 / Ours:** n=2, median = -18.15 cm
- **A1 / Omar C:** n=2, median = 2.94 cm
- **A1 / Omar Rust:** n=1, median = -11.31 cm
- **A8 / Ours:** n=2, median = 3.48 cm
- **A8 / Omar C:** n=1, median = 21.99 cm
- **A8 / Omar Rust:** n=2, median = 18.78 cm

## Variant medians — clean + completed tilt excursion >45°
- **A1 / Ours:** n=2, median = -18.15 cm
- **A1 / Omar C:** n=2, median = 2.94 cm
- **A1 / Omar Rust:** n=2, median = -9.14 cm
- **A8 / Ours:** n=2, median = 3.48 cm
- **A8 / Omar C:** n=2, median = 20.33 cm
- **A8 / Omar Rust:** n=2, median = 18.78 cm
