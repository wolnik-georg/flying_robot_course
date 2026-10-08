# Cursor prompt — A8 variant comparison: plots + table (2026-10-09)

Repo: `~/Desktop/flying_robot_course`. You start fresh. Read first: `docs/plan_2026-10-09_comparison_and_study.md` (section A), `docs/study_parameter_decisions.md`, `docs/68_NS2_Sign_Test_Result_2026-10-08.md`, `docs/69_Omar_Iz_A8_Results_2026-10-08.md`, `docs/70_Omar_C_Iz_A8_Results_2026-10-08.md`, and the existing plot code of the last meeting: `experiments/analysis/meeting_simple_indi_z.py`, `meeting_simple_z_geometric.py`, `meeting_simple_common.py`, `meeting_hw_common.py` (reuse their style and helpers where sensible), plus `experiments/analysis/ns2_2026_10_05_crossing_dip.py` (crossing detection), `omar_iz_a8_2026_10_08.py`, `omar_c_iz_a8_2026_10_08.py`, `ns2_signtest_2026_10_08_radio.py`, `ns2_a1_2026_10_08.py` (the analyses whose numbers you must reproduce).

## Goal
Compare, on scenario **A8** only, the controller variants we now have data for, with the same kinds of plots as for the last meeting plus an all-in-one plot and a summary table. The Omar variants now have the z offset fixed, so they are finally comparable with ours and with the geometric baseline. The question each output answers: **how well does each variant track the commanded height on A8 (bottom drone cf5), and how do they differ?** Crossing dips are reported but are not the main criterion.

## Rules
- Read-only on `experiments/logs/**`, firmware, crazyswarm2, `docs/meetings/**`, `docs/07_*`, `docs/next_*`, `docs/lab_sessions/**`, `docs/lab_bench_cheatsheet_*`. **No commits, no pushes, no firmware changes.**
- New files only: `experiments/analysis/a8_variant_comparison.py` (single script, deterministic, runnable from scratch with `~/.pyenv/versions/flying_robots/bin/python`), outputs in `experiments/analysis/out/a8_compare_2026-10-09/` (figures `figs/*.png`, `table_a8_variants.csv`, `table_a8_variants.md`, `flights_used.csv`, `flights_excluded.csv`), and `docs/71_A8_Variant_Comparison.md`.
- Report only what the data supports. State n everywhere. No cherry-picking; every excluded flight is listed with the reason.

## Variants and flights (verify EVERY flight against its `.meta.json`: controller, ctrl_mode, ki_z, res_sign, params dz/duration/passes, height — do not trust this list blindly; report any mismatch)
Common inclusion rule: A8, `dz 0.5`, `height 0.5`, `passes 4`, duration 26 s, bottom drone = `cf5`, top = `cf_second` geometric; completed flight (no crash/pose swap/abort); airborne data for the whole scenario. Stamps are `<date>_<HH-MM-SS>` of `experiments/logs/A8_<date>_<stamp>.meta.json`.
| Variant label | controller / ctrl_mode / other | candidate flights |
|---|---|---|
| Geometric baseline | 6 / 0, ki_z 16, network off | 2026-10-03_13-15-00; 2026-10-05_17-39-27, 17-41-09; 2026-10-08_17-37-04, 17-38-50 |
| NS2 res_sign +1 | 6 / 0, ki_z 16, rnn.en 1, res_sign +1 (not in the meta; 10-05 yaml) | 2026-10-05_19-17-04, 19-19-27, 19-23-26 |
| NS2 res_sign −1 | 6 / 0, ki_z 16, res_sign −1 | 2026-10-08_17-16-27 (battery sag, keep but flag), 17-18-40, 17-22-13, 17-24-54 |
| Ours INDI | 6 / 3, ki_z 0 | 2026-10-02_19-08-13, 19-09-54, 19-11-30; 2026-10-03_13-10-32 (check which are aborted; 10-02_18-57-49 and 18-59-17 are pre-fix with ki_z 16 → exclude) |
| Omar C exact (Kpos_Iz 0) | 9 / 0 | 2026-10-02_18-23-23, 18-24-57 (18-22-05 has dz 0.25 → exclude) |
| Omar Rust exact (kpos_iz 0) | 10 / 0 | 2026-10-02_18-45-09, 18-46-45 |
| Omar Rust + Iz 1.0 / 1.5 / 2.0 | 10 / 0 | 2026-10-08_18-17-33, 18-19-07 (1.0); 18-23-52, 18-25-23 (1.5); 18-28-04, 18-29-35 (2.0) |
| Omar C + Iz 1.0 / 1.5 / 2.0 | 9 / 0 | 2026-10-08_18-40-18, 18-42-05 (1.0); 18-45-11, 18-47-48 (1.5); 18-49-37, 18-51-15 (2.0) |
The Iz values are NOT stored in the meta; they are assigned by flight order as stated in docs/69/70 (hard-code them with that comment). The NS2 +1/−1 and geometric rows have no network-off/on flag in the meta either (rnn.en comes from the yaml of the day) — use the table above and say so. Any other A8 flight in `experiments/logs/` that satisfies the inclusion rule and belongs to one of these variants should be reported in `flights_excluded.csv` as "candidate not used" with the reason, not silently added.

## Data sources
- **uSD (500 Hz) preferred:** `experiments/logs/usd_raw/cf5_A8_thesisNN_<date>_<stamp>.bin` (10-03, 10-05, 10-08). The 10-02 bins are named `cf5_thesis<N>_2026-10-02_<time>.bin` — match them to flights by `usd_run_tag` in the meta vs `run_tag` in the decoded log (`flying_drone_stack/tools/decode_usd_log.py`, `load()`; ignore its "unmapped channels" warning). Useful channels: `t, x, y, z, ctrltarget_x/y/z, roll_deg, pitch_deg, gyro_x, motor_m1..4, rnn_pred_z, a_res_z`.
- **Radio CSV (≈ 20 Hz) fallback:** `experiments/logs/A8_cf5_<date>_<stamp>.csv` (`pd.read_csv(..., comment="#")`; columns `time_s,pos_x,pos_y,pos_z,roll,pitch,gyro_x,vbat,...`; no setpoint → z setpoint is 0.5 m). Use it only where no uSD exists and mark such rows in the table (column `source`). Lateral tracking needs the uSD setpoints (skip lateral for radio-only flights).
- Do not use any cross-drone alignment; everything is per-drone.

## Metrics (fixed definitions — implement exactly)
Let `e_z = (z − ctrltarget_z)·100` [cm] (radio: `(pos_z − 0.5)·100`). `t0` = first time `z > 0.3 m`, `t1` = last time `z > 0.3 m`. Scenario window `W = (t0 + 6 s, t1 − 4 s)`. Crossings: `ns2_2026_10_05_crossing_dip.find_crossing_times(t[airborne], y[airborne])` (4 expected). Steady set = `W` minus ±1.5 s around each crossing.
Per flight: `steady_mean` and `steady_sd` of `e_z` on the steady set; `rms_z` = sqrt(mean(e_z²)) on `W`; `max_abs_z` on `W`; crossing dips = per crossing min of `e_z` in ±1 s (negative numbers; shallower = better), report mean of the 4 and the dip relative to `steady_mean`; `lat_rms` = RMS of sqrt((x−ctrltarget_x)²+(y−ctrltarget_y)²) on `W` (uSD only); `roll_p99`, `pitch_p99` = 99th percentile of |roll|,|pitch| on `W`; `gyro_x_sd` on `W`; `pwm_ceiling_frac` = fraction of airborne samples with any `motor_m*` ≥ 65000 (uSD only); `vbat_min` (radio, loaded samples > 2 V); firmware/day.
Per variant: n flights, mean ± sd **across flights** for each metric AND the pooled sample sd of `e_z` on the steady sets; also list the per-flight values in `flights_used.csv`.

## Outputs
1. `figs/indi_variants_z.png` — INDI variants: Ours, Omar C exact, Omar C + Iz 1.5, Omar Rust exact, Omar Rust + Iz 1.5; top panel `z` vs time since takeoff with the commanded height dashed, lower panel(s) `e_z`; successor of `docs/meetings/assets/2026-10-03/fig_indi_z_tracking.png` (look at it, keep the look; do not write into `docs/meetings/**`).
2. `figs/omar_iz_ladder.png` — Rust and C with Iz 0 / 1.0 / 1.5 / 2.0: `e_z` traces plus steady-mean-vs-Iz points (per flight and mean) for both ports in one panel.
3. `figs/geometric_z.png` — geometric baseline (successor of `fig_z_tracking_geometric.png`).
4. `figs/ns2_on_off.png` — geometric network off vs NS2 `res_sign +1` vs `−1`: `e_z` traces and a dip panel (per-crossing dips with cohort mean ± sd; expected ≈ −6.1 / −10.4 / −3.15 cm from the radio analysis).
5. `figs/all_variants.png` — ONE figure with all variants (A8, same dz/height/speed): left, median `e_z` trace of each variant over time since takeoff with a 10–90 % band for the variants that have ≥ 3 flights (common axes, distinct colours, legend with n); right, bar chart of `steady_mean ± steady_sd` per variant (sorted in the order of the table below). Colour-blind-safe palette; readable at slide size.
6. `table_a8_variants.md/.csv` — one row per variant: n, source (uSD/radio/mixed), steady z error mean ± sd (across flights), pooled steady sd, rms_z, max_abs_z, crossing dip abs (mean ± sd over all crossings) and relative, lat_rms, roll_p99, pitch_p99, gyro_x_sd, pwm_ceiling_frac, vbat_min, firmware/day. Row order: Geometric, NS2 off(=geometric), NS2 −1, NS2 +1, Ours INDI, Omar C exact, Omar C + Iz 1.0/1.5/2.0, Omar Rust exact, Omar Rust + Iz 1.0/1.5/2.0.
7. `docs/71_A8_Variant_Comparison.md` — what was compared and how, the table, the figures (relative links), a short factual reading (which variants are within ±2 cm, who has the smallest spread, how Iz moves each Omar port), and **caveats**: different days/firmware (10-02/03 vs 10-05 vs 10-08), n per variant, Iz values inferred from order, battery state (cf5 vbat min per variant), 10-08 17-16-27 battery sag, radio-vs-uSD mix.

## Numbers you must reproduce (cross-check; report each as "reproduced / differs by X")
Steady z error (cm) from uSD, steady set as defined: Omar Rust + Iz 1.0 → +6.5, +6.1 (mean +6.3); 1.5 → +0.8, +1.2 (+1.0); 2.0 → 0.0, +2.4 (+1.2). Omar C + Iz 1.0 → +0.9, +2.4 (+1.6); 1.5 → +2.4, +2.0 (+2.2); 2.0 → +1.1, +1.8 (+1.5). Omar exact on A8 (10-02, older note) ≈ +19 (Rust) / +22 (C) — compute it yourself and compare. Crossing dips from radio logs: NS2 network off (10-08 17-37-04, 17-38-50, 8 crossings) mean −6.10; NS2 −1 (4 flights, 16 crossings) mean −3.15; NS2 +1 (10-05 19-17-04, 19-19-27, 19-23-26, 12 crossings) mean −10.35; geometric steady error ≈ ±2 cm (docs/meetings/2026-10-05.md table: A8 bottom +0.13 cm; do not edit that file). Tolerance 0.3 cm; explain any larger difference (usually the window or the crossing exclusion).

## Report back
Per output: file path; the table; per variant n and source; list of reproduced / not reproduced numbers; every excluded flight with reason; anything surprising (e.g. a variant whose flights disagree with each other by more than 3 cm); what you could not do. Do not interpret beyond the data; flag any place where the comparison is confounded.
