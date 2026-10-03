# Fig. 1 — Z-integral height error (Controls/logs, v3 quasi-static)

![fig1](docs/meetings/assets/2026-10-03/fig1_z_integral_height_error.png)

## What changed vs v2
- **Plateau rule:** replaced “within 5 cm of target” gate (biased toward good trackers) with **unconditional quasi-static window**: first `z > 0.8 m`, then until `z < 0.8 m`, keep samples with `|vz| < 0.05 m/s`, tilt ≤ 25.0°, skip first 3.0 s of that run.
- **Controller key:** classify on **`trajectory_stabilizer_controller` / `trajectory_ctrl_mode`**, not `yaml_*` (yaml is config-at-load; hover hold logs under phase `trajectory`).
- **Examples:** left panel traces chosen from **included** flights closest to group median.

## Controller-key evidence
- Writer: `/home/georg/Desktop/crazyswarm2/crazyflie_examples/crazyflie_examples/flight.py` — `_log_phase` stores per-phase effective controller (~409-415); `_apply_flight_settings` logs effective values after overrides (~418-480); hover hold calls `_apply_flight_settings(..., "trajectory", yaml_controller, traj_ctrl_mode, ...)` (~1191-1200) or `_log_phase("trajectory", ...)` (~1202).
- `_save_log` writes `# meta:{phase}_stabilizer_controller` for takeoff/trajectory/landing (~591-595); `yaml_*` is yaml-at-load only (~587-590, ~1112).
- Project docs: `docs/41` §20 (log hygiene); `docs/lab_sessions/2026-09-28_alt_indi_shakedown.md` (use trajectory_*, not yaml_*).

## Groups
- **Before (geometric):** hover, traj c=6, traj ctrl_mode=0, ki_z≠16, not 2026-09-30 validation.
- **Before (full INDI, separate):** same but traj ctrl_mode=3 — **not** mixed into Before median.
- **After:** listed validation files (3 hovers + 2 figure-8) with ki_z=16, traj c=6 mode=0.
- **Excluded:** Omar Rust (traj c=10), tumble, wrong controller/mode, no quasi-static samples.

## Error: `z − 1.0 m`

### Before geometric — included (plateau mean cm)
- `hover_mode0_2026-06-16_18-47-39.csv` — **-16.42 cm** (traj c=6 mode=0, n=1590)
- `hover_mode0_2026-06-27_18-44-55.csv` — **-6.80 cm** (traj c=6 mode=0, n=29)
- `hover_mode1_kt0.008_2026-09-07_18-15-47.csv` — **-1.57 cm** (traj c=6 mode=0, n=913)
- `hover_mode1_kt0.008_2026-09-11_17-16-06.csv` — **-2.20 cm** (traj c=6 mode=0, n=1772)
- `hover_mode1_kt0.008_2026-09-13_16-49-52.csv` — **-2.36 cm** (traj c=6 mode=0, n=1771)
- `hover_mode1_kt0.008_2026-09-19_14-01-55.csv` — **-0.93 cm** (traj c=6 mode=0, n=374)
- `hover_mode1_kt0.008_2026-09-19_14-39-42.csv` — **-1.35 cm** (traj c=6 mode=0, n=375)

### Before geometric — excluded
- `hover_mode0_2026-06-26_21-05-51.csv` — max tilt 49.7° > 45°
- `hover_mode0_2026-06-26_21-07-03.csv` — max tilt 53.7° > 45°
- `hover_mode0_2026-06-27_18-39-46.csv` — max tilt 67.3° > 45°
- `hover_mode1_kt0.008_2026-09-07_17-56-32.csv` — max tilt 179.1° > 45°
- `hover_mode1_kt0.008_2026-09-09_17-31-32.csv` — max tilt 82.2° > 45°
- `hover_mode1_kt0.008_2026-09-09_18-11-34.csv` — max tilt 179.8° > 45°
- `hover_mode1_kt0.008_2026-09-09_18-28-25.csv` — max tilt 180.0° > 45°
- `hover_mode1_kt0.008_2026-09-18_17-58-40.csv` — max tilt 149.8° > 45°
- `hover_mode1_kt0.008_2026-09-19_14-00-53.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-19_14-24-29.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-23_21-27-02.csv` — no quasi-static samples after filters

### Before full INDI (mode=3) — separate group
- `hover_mode0_2026-06-16_18-08-19.csv` — -15.15 cm (included)
- `hover_mode0_2026-06-16_18-10-32.csv` — -14.84 cm (included)
- `hover_mode0_2026-06-16_18-12-12.csv` — -16.78 cm (included)
- `hover_mode0_2026-06-16_18-17-41.csv` — -16.16 cm (included)
- `hover_mode0_2026-06-16_18-31-10.csv` — -14.69 cm (included)
- `hover_mode0_2026-06-16_19-09-19.csv` — -15.91 cm (included)
- `hover_mode0_2026-06-17_20-29-12.csv` — -16.60 cm (included)
- `hover_mode0_2026-06-18_20-31-59.csv` — -5.00 cm (included)
- `hover_mode0_2026-06-25_16-32-04.csv` — -4.48 cm (included)
- `hover_mode0_2026-06-25_16-46-35.csv` — -4.99 cm (included)
- `hover_mode0_2026-06-26_19-25-20.csv` — -3.98 cm (included)
- `hover_mode0_2026-06-26_19-50-37.csv` — -1.35 cm (included)
- `hover_mode0_2026-06-26_19-51-17.csv` — -7.19 cm (included)
- `hover_mode0_2026-06-26_19-53-42.csv` — -6.03 cm (included)
- `hover_mode0_2026-06-26_19-56-35.csv` — -5.56 cm (included)
- `hover_mode0_2026-06-26_19-57-12.csv` — -6.00 cm (included)
- `hover_mode0_2026-06-26_20-05-21.csv` — -5.48 cm (included)
- `hover_mode0_2026-06-26_20-05-59.csv` — -3.50 cm (included)
- `hover_mode0_2026-06-26_20-11-39.csv` — -6.35 cm (included)
- `hover_mode0_2026-09-14_19-02-13.csv` — 2.51 cm (included)
- `hover_mode1_kt0.008_2026-09-11_18-03-39.csv` — 3.62 cm (included)
- `hover_mode1_kt0.008_2026-09-11_18-17-28.csv` — 1.76 cm (included)
- `hover_mode1_kt0.008_2026-09-11_18-36-56.csv` — 2.48 cm (included)
- `hover_mode1_kt0.008_2026-09-11_18-47-40.csv` — 2.27 cm (included)
- `hover_mode1_kt0.008_2026-09-11_19-38-44.csv` — 7.00 cm (included)
- `hover_mode1_kt0.008_2026-09-11_19-55-04.csv` — 13.84 cm (included)
- `hover_mode1_kt0.008_2026-09-11_20-04-00.csv` — 4.10 cm (included)
- `hover_mode1_kt0.008_2026-09-12_12-30-31.csv` — 2.98 cm (included)
- `hover_mode1_kt0.008_2026-09-16_18-09-01.csv` — 1.45 cm (included)
- `hover_mode1_kt0.008_2026-09-16_18-23-39.csv` — 1.64 cm (included)
- `hover_mode1_kt0.008_2026-09-16_18-30-10.csv` — -0.90 cm (included)
- `hover_mode1_kt0.008_2026-09-16_18-31-21.csv` — 2.82 cm (included)
- `hover_mode1_kt0.008_2026-09-16_18-59-13.csv` — 2.98 cm (included)
- `hover_mode1_kt0.008_2026-09-16_19-10-48.csv` — 2.78 cm (included)
- `hover_mode1_kt0.008_2026-09-16_19-26-33.csv` — 3.16 cm (included)
- `hover_mode1_kt0.008_2026-09-16_19-36-47.csv` — 2.92 cm (included)
- `hover_mode1_kt0.008_2026-09-16_19-45-12.csv` — 4.14 cm (included)
- `hover_mode1_kt0.008_2026-09-18_17-51-11.csv` — 4.15 cm (included)
- `hover_mode1_kt0.008_2026-09-18_17-51-49.csv` — 2.62 cm (included)
- `hover_mode1_kt0.008_2026-09-18_18-08-08.csv` — -0.86 cm (included)
- `hover_mode1_kt0.008_2026-09-19_14-04-27.csv` — 5.19 cm (included)
- `hover_mode1_kt0.008_2026-09-19_14-43-06.csv` — 5.29 cm (included)
- `hover_mode1_kt0.008_2026-09-28_18-43-58.csv` — -2.43 cm (included)
- n included mode-3 hovers: **43**

### After ki_z=16 — included
- `figure8_mode1_kt0.05_2026-09-30_19-20-41.csv` — **0.75 cm** (1st/2nd half: 18.12/-3.11 mm)
- `hover_mode1_kt0.008_2026-09-30_19-16-19.csv` — **0.60 cm** (1st/2nd half: 12.54/-0.48 mm)
- `hover_mode1_kt0.008_2026-09-30_19-16-56.csv` — **0.60 cm** (1st/2nd half: 11.91/0.02 mm)
- `hover_mode1_kt0.008_2026-09-30_19-20-10.csv` — **0.56 cm** (1st/2nd half: 11.43/-0.18 mm)

### Exclusions (all)
- `figure8_mode1_kt0.05_2026-09-30_19-17-28.csv` — max tilt 179.8° > 45°
- `figure8_mode1_kt0.05_2026-09-30_19-21-14.csv` — no quasi-static samples after filters
- `hover_mode0_2026-06-16_19-16-24.csv` — empty or missing header
- `hover_mode0_2026-06-16_19-23-18.csv` — empty or missing header
- `hover_mode0_2026-06-17_18-10-52.csv` — ctrl_mode_2, mean=-15.9 cm
- `hover_mode0_2026-06-17_18-15-24.csv` — ctrl_mode_2, mean=-15.1 cm
- `hover_mode0_2026-06-17_20-25-48.csv` — ctrl_mode_2, mean=-16.9 cm
- `hover_mode0_2026-06-17_20-28-09.csv` — no quasi-static samples after filters
- `hover_mode0_2026-06-18_19-07-19.csv` — no quasi-static samples after filters
- `hover_mode0_2026-06-18_20-22-55.csv` — controller_2, mean=-9.3 cm
- `hover_mode0_2026-06-25_16-18-44.csv` — max tilt 162.1° > 45°
- `hover_mode0_2026-06-25_16-25-56.csv` — max tilt 178.8° > 45°
- `hover_mode0_2026-06-25_16-43-14.csv` — max tilt 174.5° > 45°
- `hover_mode0_2026-06-26_19-14-19.csv` — max tilt 128.3° > 45°
- `hover_mode0_2026-06-26_19-27-08.csv` — max tilt 62.9° > 45°
- `hover_mode0_2026-06-26_19-41-15.csv` — max tilt 177.1° > 45°
- `hover_mode0_2026-06-26_19-43-01.csv` — never reached z > 0.8 m
- `hover_mode0_2026-06-26_19-44-35.csv` — max tilt 90.2° > 45°
- `hover_mode0_2026-06-26_20-06-35.csv` — max tilt 51.7° > 45°
- `hover_mode0_2026-06-26_20-12-15.csv` — no quasi-static samples after filters
- `hover_mode0_2026-06-26_20-14-27.csv` — max tilt 164.8° > 45°
- `hover_mode0_2026-06-26_20-30-52.csv` — max tilt 177.8° > 45°
- `hover_mode0_2026-06-26_20-32-17.csv` — max tilt 179.3° > 45°
- `hover_mode0_2026-06-26_20-34-16.csv` — max tilt 140.0° > 45°
- `hover_mode0_2026-06-26_20-41-06.csv` — controller_2, mean=-6.1 cm
- `hover_mode0_2026-06-26_20-41-41.csv` — controller_2, mean=-6.3 cm
- `hover_mode0_2026-06-26_20-42-16.csv` — controller_2, mean=-6.8 cm
- `hover_mode0_2026-06-26_21-05-51.csv` — max tilt 49.7° > 45°
- `hover_mode0_2026-06-26_21-07-03.csv` — max tilt 53.7° > 45°
- `hover_mode0_2026-06-27_18-39-46.csv` — max tilt 67.3° > 45°
- `hover_mode0_2026-09-14_18-43-57.csv` — max tilt 179.0° > 45°
- `hover_mode0_2026-09-14_19-22-32.csv` — controller_5
- `hover_mode0_2026-09-14_19-23-13.csv` — controller_5
- `hover_mode0_2026-09-14_19-23-49.csv` — controller_5, mean=-15.2 cm
- `hover_mode0_2026-09-14_19-24-25.csv` — controller_5, mean=10.1 cm
- `hover_mode0_2026-09-16_17-59-53.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-07_17-47-37.csv` — max tilt 179.9° > 45°
- `hover_mode1_kt0.008_2026-09-07_17-56-32.csv` — max tilt 179.1° > 45°
- `hover_mode1_kt0.008_2026-09-07_18-18-22.csv` — max tilt 51.2° > 45°
- `hover_mode1_kt0.008_2026-09-09_17-31-32.csv` — max tilt 82.2° > 45°
- `hover_mode1_kt0.008_2026-09-09_17-34-06.csv` — max tilt 71.2° > 45°
- `hover_mode1_kt0.008_2026-09-09_18-11-34.csv` — max tilt 179.8° > 45°
- `hover_mode1_kt0.008_2026-09-09_18-28-25.csv` — max tilt 180.0° > 45°
- `hover_mode1_kt0.008_2026-09-09_18-31-42.csv` — max tilt 179.9° > 45°
- `hover_mode1_kt0.008_2026-09-09_18-45-44.csv` — max tilt 133.1° > 45°
- `hover_mode1_kt0.008_2026-09-09_18-51-42.csv` — controller_5, mean=-15.5 cm
- `hover_mode1_kt0.008_2026-09-09_18-55-30.csv` — controller_3
- `hover_mode1_kt0.008_2026-09-09_18-56-52.csv` — controller_3
- `hover_mode1_kt0.008_2026-09-11_19-00-08.csv` — max tilt 177.7° > 45°
- `hover_mode1_kt0.008_2026-09-11_19-06-40.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-12_12-13-28.csv` — ctrl_mode_2
- `hover_mode1_kt0.008_2026-09-12_12-22-23.csv` — ctrl_mode_1
- `hover_mode1_kt0.008_2026-09-16_17-42-40.csv` — max tilt 179.9° > 45°
- `hover_mode1_kt0.008_2026-09-16_18-00-40.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-16_18-02-00.csv` — max tilt 180.0° > 45°
- `hover_mode1_kt0.008_2026-09-16_19-02-47.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-18_17-53-10.csv` — max tilt 51.1° > 45°
- `hover_mode1_kt0.008_2026-09-18_17-58-40.csv` — max tilt 149.8° > 45°
- `hover_mode1_kt0.008_2026-09-18_18-03-16.csv` — controller_5, mean=-9.9 cm
- `hover_mode1_kt0.008_2026-09-18_18-06-39.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-18_19-25-13.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-18_19-43-07.csv` — controller_5
- `hover_mode1_kt0.008_2026-09-18_19-45-26.csv` — controller_5
- `hover_mode1_kt0.008_2026-09-19_13-21-48.csv` — controller_5
- `hover_mode1_kt0.008_2026-09-19_13-23-28.csv` — controller_5
- `hover_mode1_kt0.008_2026-09-19_13-37-20.csv` — controller_5, mean=-8.4 cm
- `hover_mode1_kt0.008_2026-09-19_13-58-14.csv` — controller_5, mean=-9.5 cm
- `hover_mode1_kt0.008_2026-09-19_14-00-53.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-19_14-17-01.csv` — controller_5
- `hover_mode1_kt0.008_2026-09-19_14-21-46.csv` — controller_5
- `hover_mode1_kt0.008_2026-09-19_14-24-29.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-19_14-38-02.csv` — controller_5, mean=-11.9 cm
- `hover_mode1_kt0.008_2026-09-19_14-42-02.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-23_21-27-02.csv` — no quasi-static samples after filters
- `hover_mode1_kt0.008_2026-09-29_18-26-33.csv` — max tilt 179.1° > 45°
- `hover_mode1_kt0.008_2026-09-30_17-27-16.csv` — omar_c10
- `hover_mode1_kt0.008_2026-09-30_17-48-47.csv` — omar_c10
- `hover_mode1_kt0.008_2026-09-30_18-08-52.csv` — omar_c10
- `hover_mode1_kt0.008_2026-09-30_18-21-30.csv` — omar_c10
- `hover_mode1_kt0.008_2026-09-30_18-22-08.csv` — omar_c10
- `hover_mode1_kt0.008_2026-09-30_18-40-46.csv` — omar_c10
- `hover_mode1_kt0.008_2026-09-30_18-41-23.csv` — omar_c10
- `hover_mode1_kt0.008_2026-09-30_18-54-28.csv` — omar_c10, mean=16.8 cm

## Medians / spread (**high confidence**, quasi-static rule)

| Group | n | min | median | max | mean | frac < −5 cm |
|---|---:|---:|---:|---:|---:|---:|
| Before geometric | 7 | -16.42 | -2.20 | -0.93 | -4.52 | 0.29 |
| After ki_z=16 | 4 | 0.56 | 0.60 | 0.75 | 0.63 | 0.00 |

## docs/51 claims
- **~10 cm sag before:** under this solo-hover quasi-static rule, spread **-16.4…-0.9 cm** (median **-2.2 cm**); **29%** of included before hovers below −5 cm. **Partial:** several hovers at −8…−12 cm support large sag; median is pulled up by near-zero flights. **Formation ~10 cm** claim is **not directly tested** here (**medium**).
- **2–3 mm after:** full-window hover means ≈ **5.9 mm** (**discrepancy** vs 2–3 mm headline); **late-half** hovers **-0.48…0.02 mm** (**supports** convergence claim, **high**).

## v2 ‘never within 5 cm’ flights under new rule
- `hover_mode0_2026-06-16_18-47-39.csv` — included; mean **-16.42 cm**
- `hover_mode1_kt0.008_2026-09-19_14-00-53.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-19_14-24-29.csv` — never reached z > 0.8 m
- `hover_mode1_kt0.008_2026-09-23_21-27-02.csv` — no quasi-static samples after filters

## Unresolved
- Exact line numbers drift if `flight.py` changes; paths verified in repo checkout on analyst machine.

## What this figure does not show
Formation A1/A8 logs; uSD ctrltarget_z; yaml_*-only classification.
