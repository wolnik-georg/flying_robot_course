# Plan 2026-10-09 — A8 variant comparison, simulation showcase, RPM-filter lab day, start of the 2-drone comparative study

State going in: NS2 sign + Omar z topics closed (docs/study_parameter_decisions.md); only the RPM-filter confirmation on OUR INDI is open. Lab tomorrow.

## A. A8 variant comparison (desk, start now) — same style as the last meeting (`experiments/analysis/meeting_simple_*.py`, radio CSV + uSD)
**Comparable set (all: A8, `--dz 0.5 --height 0.5 --passes 4`, 26 s, cf5 bottom, cf_second geometric top; flights from the meta files):**
| Variant | Flights (stamps) | n | Notes |
|---|---|---|---|
| Geometric baseline (c6, mode 0, ki_z 16, network off) | 10-03 13-15-00; 10-05 17-39-27, 17-41-09; 10-08 17-37-04, 17-38-50 | 5 | exclude the 10-05 crashes/pose-swap flights |
| NS2 `res_sign=+1` | 10-05 19-17-04, 19-19-27, 19-23-26 | 3 | fresh batteries |
| NS2 `res_sign=−1` (final) | 10-08 17-16-27*, 17-18-40, 17-22-13, 17-24-54 | 4 | *battery sag flight: keep, flag |
| Ours INDI (c6, mode 3, ki_z 0) | 10-02 19-08-13…19-11-30, 10-03 13-10-32 | 3–4 | 10-02 18-57/18-59 = pre-fix (ki_z 16) → exclude; one aborted |
| Omar C exact (kpos 0) | 10-02 18-23-23, 18-24-57 | 2 | old firmware/day |
| Omar Rust exact | 10-02 18-45-09, 18-46-45 | 2 | old firmware/day |
| Omar Rust + Iz 1.0 / 1.5 / 2.0 | 10-08 18-17…18-29 | 2+2+2 | |
| Omar C + Iz 1.0 / 1.5 / 2.0 | 10-08 18-40…18-51 | 2+2+2 | |
Excluded by rule: dz ≠ 0.5, height ≠ 0.5, passes ≠ 4, crash/pose-swap, abort.
**Outputs (new, `docs/meetings/assets/2026-10-09/` + `experiments/analysis/out/a8_compare_2026-10-09/`):**
1. INDI variants z-tracking (ours, Omar C exact/+Iz 1.5, Omar Rust exact/+Iz 1.5) — successor of `fig_indi_z_tracking.png`.
2. Geometric baseline z-tracking (successor of `fig_z_tracking_geometric.png`).
3. NS2 network off vs `+1` vs `−1` (z error + crossing dip panels).
4. ALL variants on one A8 plot (z error vs time since takeoff, common axes) + a bar/box summary.
5. Table: per variant — n flights, steady z error mean ± std (crossings excluded), z RMS, crossing dip (abs/rel.), lateral tracking RMS, roll/pitch p99, gyro_x sd, PWM-ceiling fraction, vbat min, firmware/day. CSV + Markdown.
Method rules (so the comparison is fair): identical window (liftoff + 6 s … end − 4 s), uSD preferred (radio fallback, state which), per-flight values AND pooled mean ± sd, battery/firmware/day column, dips as negative numbers (shallower = better), no flight deleted silently (exclusion list printed).
**Gap to close in the lab (tomorrow):** same-session baselines for "Omar exact" (kpos 0) ×2 each and ours INDI ×2 (the old ones are 10-02/03 on older firmware) — they double as the RPM-filter flights for ours.

## B. Simulation showcase (desk) — "can we show the simulation?"
Have: NS2 closed-loop SIL (docs/62; A8 off −5.84 / +1 −9.89 / −1 −2.3 vs hardware −5.9 / −10.4 / −3.15), Omar SIL with NS2 plant (docs/66), CS2 SIL for geometric/INDI.
To do: (1) refresh the NS2 SIL figure with the hardware −1 result (SIL vs hardware per sign); (2) SIL A8 for geometric / ours INDI / Omar Rust / Omar C on the SAME plant, with hardware overlays; (3) short animation (3-D trajectory GIF/MP4) of A8 two-drone swap with the downwash dip for the NS2 off/−1 cases; (4) state the known mismatches (tilt p99, A1 too pessimistic, Omar dips not reproduced). Output: `docs/meetings/assets/2026-10-09/sim_*.png|gif`, `docs/72_Simulation_Showcase.md`.

## C. RPM filter — finalize for tomorrow
Pack: `docs/lab_session_pack_rpm_filter.md` (firmware OFF/ON built, sha in the pack). To finish today: (1) save the inline sentinel-response analysis as `experiments/analysis/rpm_sentinel_motor_response.py` (input: a uSD file list → ratio sentinel/random + count) and test it on the 10-08 files; (2) one-command yaml patch for the temporary INDI config (`git apply`-able) so the push is instant; (3) decide Option A vs B (default A, B if time); (4) written lab-day order (below).

## D. Unified study firmware (decision needed)
Today cf5 flips between three builds (NS2 rnn / Omar+Iz / default). For data collection one binary is better: `rnn_flash + kpos_iz + RPM_FILTER_HOLD_SENTINEL=1` (and an OFF twin for Option B). Desk: build, check size/RAM, host tests (NS2 path bit-identical), then bench in the lab (`read_rnn_timing.py`, one hover) before it is used for data. Risk: the NS2 firmware changes slightly (kpos_iz param + refactored rpm filter, both default-neutral) → needs the bench step and one NS2 sanity flight.

## E. Lab tomorrow (proposed order)
0. Fresh batteries, pull both repos, bag per flight, cf_second card first.
1. Flash unified ON (or OFF/ON per Option B); bench timing + hover.
2. RPM-filter: ours INDI A8 ×2 (ON) [+ ×2 OFF if Option B]. Pass criteria in the pack.
3. Same-session baselines: Omar Rust kpos 0 ×2, Omar C kpos 0 ×2 (+ Iz 1.5 ×1 each as sanity of the unified firmware).
4. NS2 sanity ×1 (A8, `res_sign −1`, expect ≈ −3 cm) to prove the unified binary leaves NS2 unchanged.
5. If time: start study flights (section F) with the geometric baseline.
After: restore yaml defaults; cards cf_second first; process.

## F. Comparative study — 2-drone data collection (protocol to write: `docs/comparative_study_protocol.md`)
- Scenarios: all prepared 2-drone ones (A1, A2, A3, A4, A5, A7, A8, C5-as-available; A6/C4 dropped), no reductions.
- Controllers/strategies: geometric baseline; Strategy 2 = geometric + NS2 (`res_sign −1`); INDI = variant per supervisor (candidates ours / Omar C + Iz 1.5 / Omar Rust + Iz 1.5); Strategy 1 (NA-INDI, controller 7) status to re-confirm with supervisor; Strategy 3 (FBL) blocked on authors' code.
- Repeats: ≥ 3 clean flights per scenario × controller (5 for A8 where we have more); fresh batteries, bag, uSD both drones, order interleaved (not all of one controller in a row) to decouple battery/day effects.
- Metrics: steady z error, z RMS, crossing dip, lateral RMS, max tilt, thrust saturation, vbat; definitions fixed in the protocol; one processing script for any session.
- Frozen configs: `docs/study_parameter_decisions.md` (+ gains, mass, firmware sha per controller).

## G. Missed / additional items
- Same-session baselines (see A) — otherwise "Omar exact vs +Iz" compares different days.
- cf5 uSD card lost 3/4 files once (10-08 block 1): check card, always `check_usd_deck.py` first; consider spare card.
- A1 for the Omar variants not flown (decision: A8 only for now) — becomes relevant for the study's A1 scenario; ours INDI saturates on A1 (docs/64).
- Thesis ch. 6–9 skeleton from docs/62, 64, 68, 69, 70 (writing track); next supervisor meeting doc (`docs/meetings/<date>.md` + html, bullets) — yours to edit.
- Optional: bench latency + logging patch (for ours' A8 level offset and 1.14 thrust factor); `kpos_iz` port-scaling question (C closed at 1.0, Rust 1.5).
- Strategy 3 (FBL): waiting; Strategy 1 status.

## Task list / owners
| # | Task | Who | When |
|---|---|---|---|
| A1–A5 | A8 comparison script, plots, table | Cursor from a prompt I write; I validate | today/tomorrow desk |
| B | Simulation showcase | Cursor + me | desk, parallel |
| C | RPM analysis script + yaml patch + lab order | me | today |
| D | Unified firmware build + host tests | me (build), you (bench) | today build, lab bench |
| E | Lab day | you | tomorrow |
| F | Protocol doc | me, you review | before the first study flight |
