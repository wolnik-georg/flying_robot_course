# Comparative study protocol — 2-drone scenarios (DRAFT v0.2, 2026-10-11)

> **UPDATE 2026-10-14 (night): the strategies S0–S3 of this protocol are redefined on ONE Omar C controller tree** (S0 Omar geometric `indi 0`, S1 Omar full INDI `indi 3`, S2 geometric + NS2 in C, S3 INDI + NS2; both drones controller 9, `Kpos_Iz 1.5`, shared position gains; FBL = S4). §1–§3 below describe the earlier controller-6 definitions and are superseded where they differ; tier 1 = 12 settings × 4 strategies = 48 cells (240 clean flights). Protocol v1.0 is re-frozen only after the tree validation flights. Details: `docs/plan_topics_2026-10-14.md`, `docs/plan_omar_tree_ns2_2026-10-14.md`, yaml `crazyswarm2/crazyflie/config/study/tree_S0..S3.patch`.

**Status:** draft for review, not frozen (v0.2 of 2026-10-11 folds in the desk preparation: readiness matrix, `--study` gate, logging superset, flight plan; changes listed in section 12). It is frozen as v1.0 after the shakedown session; later changes only as dated amendments (section 12).
**Builds on (does not replace):** [`05_Experimental_Protocol_2Robot.md`](05_Experimental_Protocol_2Robot.md) (goal, fairness rule, ≥ 5 repeats), [`10_Formation_Library.md`](10_Formation_Library.md) (scenario library), [`27_Analysis_and_Metrics_Plan.md`](27_Analysis_and_Metrics_Plan.md) (analysis policy, `metrics.py`, `aggregate.py`, `plot_comparison.py`), [`study_parameter_decisions.md`](study_parameter_decisions.md) (frozen parameters), [`roadmap_to_comparative_study.md`](roadmap_to_comparative_study.md).
**Where it differs from docs/05:** controller selection is no longer only `indi_gains.ctrl_mode` — Omar's INDI is `stabilizer.controller 9`, NS2 needs `rnn.en 1` + `res_sign −1`; scenario list, sweeps, tiers and the exclusion record are new.

## 1. Question and scope
Compare control strategies for interaction-force-aware multirotor teams under downwash on two Crazyflie 2.1 Brushless drones in vertical stacks.
| ID | Strategy | Status |
|---|---|---|
| S0 | geometric baseline (uncompensated) | frozen |
| S1 | pure INDI = **Omar C + z integral 1.5** (controller 9) | **decided 2026-10-14 (author; supervisor not yet consulted)**; ours INDI stays supplementary (10-09 data, frozen `docs/77`) |
| S2 | geometric + NS2 learned residual, `res_sign −1`, 100 Hz | frozen config; weights decision open (section 2) |
| S3 | FBL + residual | blocked (authors' code not received); added by amendment if it arrives |
**Subject drone:** `cf5` (bottom, carries the strategy). `cf_second` (top) is always geometric (S0 config) and identical in every flight. **Both drones' data are recorded and analysed** (per-drone and relative metrics).
**Out of scope:** 3-drone scenarios (B1–B3, C1–C3), single-drone C5, A6/C4 (dropped after repeated real crashes). NA-INDI/LINDI are not compared (supervisor decision).

## 2. Controllers and frozen configurations
| ID | cf5 yaml (stabilizer / parameters) | firmware cf5 |
|---|---|---|
| S0 | `controller 6`, `ctrl_mode 0`, `ki_z 16`, pos gains 40/30/8/10, `rnn.en 0` | unified ON |
| S1 | `controller 9`, `ctrlOmarIndi.indi 3`, `Kpos_Iz 1.5`, Omar gains unchanged, deck RPM (`deck.bcRpm 1`) | unified ON |
| S2 | `controller 6`, `ctrl_mode 0`, `ki_z 16`, `rnn.en 1`, `res_sign −1`, network at 100 Hz (`rnn.div 10`) | unified ON |
- **Firmware:** `cf21bl_study_rnn_iz_sentinel_ON.bin`, sha256 `989e5150f4d68690d319fa9320c1f947e36f7996caca329db669a512d3079121` (network + `kpos_iz` + RPM-sentinel fix). `cf_second`: firmware as flown 10-05…10-09 (hash recorded at the shakedown).
- **NS2 weights (open decision):** currently `experiments/analysis/out/c2_e2e_2026-10-01/full_bank_c1_complete.npz`, sha256 `8a43b5b54d0e2023f475a4314f64fe81120a38413d9f6cb6817b2bab6afc7b43` (26-flight bank). Retraining with more data is possible but **must be decided before the shakedown**; the weights are then frozen for the whole study. Rule: retrain **only with non-study data** (C.1, sign test, shakedown); study flights are never used for training.
- **Frozen = gains, yaml, firmware, weights.** Each strategy was tuned on the hardware gate; no retuning during the campaign (docs/05). The yaml of each strategy is a tagged crazyswarm2 commit (`study-S0`, `study-S1`, `study-S2`, created at the shakedown); the commit hash is written into every flight meta.
- `res_sign` rule: −1 only with `ctrl_mode 0`; never with `ctrl_mode 3` (`study_parameter_decisions.md`).

## 3. Scenarios, settings and tiers
All at `--height 0.5` (bottom drone 0.5 m, top 0.5 m + dz), `--auto-center`, two drones. Speed set **{0.2, 0.3, 0.4, 0.5} m/s**, static separation set **{0.25, 0.5, 0.75} m**.
| Scenario | Type | Controlling parameter(s) | Tier 1 | Tier 2 | Tier 3 |
|---|---|---|---|---|---|
| A1 vertical stack hover | static | `dz` ∈ {0.25, 0.5, 0.75}, `--hold 15` | 3 settings | | |
| A8 vertical swap | dynamic | `dz 0.5`; speed via `--duration` of one swap pass (flag added to `run_formation` 2026-10-11; ≈ 10.9 / 7.3 / 5.5 / 4.4 s give peak 0.20 / 0.30 / 0.40 / 0.50 m/s, checked with the scenario generator; **all A8 flights before 2026-10-11 used the default 6.0 s = 0.365 m/s**, fixed at the shakedown) | 4 | | |
| A2 stack tracking (circle) | dynamic | `dz 0.5`; speed via circle `--period` (r 0.4 m: 12.6 / 8.4 / 6.3 / 5.0 s) | 1 (0.4) | +3 | |
| A3 static top, bottom moves | dynamic | `dz 0.5`, `--speed` | 1 (0.4) | +3 | |
| A4 offset stack | dynamic | `dz 0.5`, `--offset 0.10`, `--speed` | 1 (0.4) | | +3 |
| A5 reverse-circle stack | dynamic | `dz 0.5`, `--period` as A2 | 1 (0.4) | | +3 |
| A7 dynamic merge | dynamic | `dz` 1.1 → 0.1 by design, `--speed` | 1 (0.4) | | +3 |
| **Settings** | | | **12** | **+6 (18)** | **+9 (27)** |
- **Cell = scenario setting × strategy.** Tier 1: 12 × 3 = 36 cells; tier 2: 54; tier 3: 81. **5 clean flights per cell** → **180 / 270 / 405 clean flights**.
- Exact scenario parameters per setting (and feasibility at speed 0.5: mocap boundary, gating, `--allow-extreme`) are fixed in the shakedown and written into the protocol v1.0 table; flight command lines are generated from that table.
- Tier 2/3 are flown only if time remains after tier 1 (decision at the end of each tier).

## 4. Flight plan
- **Repeats are repeated flights** (never phases or crossings of one flight, docs/27). **5 clean flights per cell.** Attempt cap **8 per cell**: if 5 clean flights are not reached, the cell is reported as "not flyable / not reached" — itself a result, never silently dropped.
- **Grouped by controller** inside a session (controller changes need a yaml change + relaunch); the controller order rotates between sessions (S0→S1→S2, S2→S0→S1, …); within a controller block the scenario settings follow a fixed pre-written order.
- **Anchor flights** against day/battery drift: 2 × S0 on A8 (default setting) at the start and 2 at the end of every session (these also feed the S0 cell).
- **Session routine:** fresh batteries (rest ≥ 4.1 V; swap if either drone < 4.0 V or after the flights agreed at the shakedown), `git pull` + build, `check_usd_deck.py` on both drones, unified firmware ON verified (sha), pose bag per flight (`/poses /cf5/pose /cf_second/pose`), firmware/yaml block printed by `run_formation` copied into the lab doc, verify `a_res` ≠ 0 on the first flight (docs/05).
- **Abort / safety:** |roll| or |pitch| > 25° for 0.5 s, or z < 0.25 m → abort; stop after 2 crashes; toggle `stabilizer.estimator` 1→2 after a crash; human pilot ready.
- **Throughput (estimate):** 10-09 flew ≈ 20 flights in 95 min including three firmware flashes and card swaps ≈ 12 flights/hour. Tier 1 ≈ 225 attempts ≈ 19 flying hours ≈ 8–10 sessions of 3 h; tiers 1+2 ≈ 12–14; all ≈ 17–19 sessions. To be replaced by measured numbers after the shakedown.

## 5. Data and logging
- **Source of record: uSD logs** of both drones (500 Hz). Radio CSVs are for monitoring only and are **not** used for reported results (docs/27 policy; the 10-09 A8 comparison still mixes a few radio-only flights — such flights are excluded or marked in the study).
- **Pose bag** is the common clock for relative metrics (the trajectory-based log alignment is weak for stationary scenarios such as A1): `experiments/analysis/pose_bag.py` reads the mcap bag (needs `pip install mcap mcap-ros2-support`), `study_metrics.py` adds `bag_sep_err_*` / `bag_min_separation_m` next to the uSD-pair values; on 13 matched A1/A8 flights min separation and mean agree within ~1–3 cm, the bag RMS is larger because its window still contains the climb/converge phase.
- **Logging changes before the shakedown:** (1) **prepared 2026-10-11, not yet flown:** one superset (96 variables incl. `rpmf.m1..m4` since 2026-10-13) `flying_drone_stack/tools/usd_study_config.txt` for every strategy (92 variables: Omar C group incl. his `rpy`/`rpyd`, battery `pm.vbat`, quaternion, reference velocity/acceleration, raw mocap pose) — needs the patched firmware (`MAX_USD_LOG_VARIABLES_PER_EVENT` 96, binary `cf21bl_study_usd96_rnn_iz_sentinel_ON.bin`, replaces the unified-ON hash above once flashed and checked with `check_usd_log_quality.py`; baseline gap rate of the old config 0.08 %) — copy the config to the SD cards of both drones; (2) **prepared and tested without hardware 2026-10-11 (crazyswarm2, uncommitted; needs the `cf_second` firmware hash in `study_manifest.json` and the cf5 hash of the image actually flashed):** `run_formation --study --strategy S0|S1|S2|ours` reads the parameters back from both drones, checks the frozen block before takeoff (and again after `apply(scenario)`, landing first on a mismatch) and writes the readback, git commit and firmware/weights hashes into the meta; originally specified as: `run_formation` writes the whole `cf5` yaml parameter block, yaml commit hash, firmware sha and weights sha into every meta (today the meta lacks `rnn.en`, `res_sign`, `kpos_iz`, `Kpos_Iz`).
- **Handling per session:** radio CSV + meta + bags pushed from the lab PC; cards: `cf_second` first, copy with explicit names + `cmp`, archive on the card, merge by run tag; one `flight_decisions.csv` (columns: date, stamp, scenario, setting, strategy, decision, reason, vbat min, notes).
- **Processing:** extend the existing `metrics.py` / `aggregate.py` / `plot_comparison.py` (docs/27) with the sweep cells and the scenario events below; `a8_crossings.py` replaces the old crossing detector (the old one was wrong in 8 of 29 flights).

## 6. Metrics (fixed definitions)
- **Unit of analysis = one flight.** Per-flight values are averaged over crossings/passes first; statistics run over flights (n = 5 per cell). Pooling crossings across flights is not allowed for inference.
- **Common, per drone (cf5 and cf_second):** position RMSE (total, x/y/z) and peak; steady z error (window: scenario start + 6 s … end − 4 s, event windows excluded as defined per scenario), z RMS; lateral RMS; measured roll/pitch/yaw RMS and p99/max; deviation from the nominal attitude of the commanded acceleration (controller-independent, `att_vs_nom_ref_*`; the controller-internal `e_R` is logged by ours and Omar Rust only, is not comparable across controllers and is not used for comparisons); gyro spread; control effort (`motor_*` PWM ratios, PWM-ceiling fraction); battery `vbat` min.
- **Relative (needs the common clock):** separation error against the commanded separation (mean, so a downwash bias shows, and RMS), minimum separation.
- **Learned strategy:** `rnn_pred_z` vs measured `a_res_z` (R², RMSE) on every flight, including S0/S1 flights where it is logged but unused.
- **Scenario-specific primary metric:** A1 steady z error + z/attitude spread; A8 crossing dip per crossing (zero-crossing detector), averaged per flight; A3 dip per pass; A2/A5 tracking RMSE + dz error; A4 lateral and z error; A7 dz error vs commanded dz(t) and minimum separation.
- **Robustness:** failure/abort rate per cell (flights attempted vs clean), behaviour as dz or speed changes.

## 7. Acceptance and exclusion
- You decide at the card pull which flights are clean and which are excluded (crash, abort, pose swap, battery, other) — as before. Each decision goes into `flight_decisions.csv` with a one-word reason; the lab doc lists counts.
- Automatic flags (advice only): tilt > 45°, z < 0.25 m, flight shorter than the scenario, vbat < 3.2 V, missing uSD file, crossing count ≠ 4 in A8.
- Excluded flights are never deleted from the repo and are reported (counts per cell) in the thesis.

## 8. Analysis plan
- Per scenario: table cells × strategies (mean ± sd over flights, n shown), figure z error vs time per strategy (median + band) and metric-vs-setting curves (speed or dz).
- Differences between strategies per setting: effect size and bootstrap CI on the ratio (as `aggregate.py`); with n = 5 no multiple-testing claims; p-values only where the pseudo-replication guard allows.
- Simulation (docs/62, 72) is shown as validation of the NS2 sign effect, not as a predictor for Omar or ours.

## 9. Known limits and risks
INDI on A1 (Omar C untested there, ours saturates/oscillates) — flown anyway · A1 under S0/S2 showed a ±35° oscillation on 2026-10-08 (`docs/68`), not expected — re-checked in the shakedown · NS2 in-distribution (A1–A5, A7) vs out-of-distribution (A8: the network acts during ~8 % of a flight), bank thin at dz 0.75 m (0.5 % of its samples within ±5 cm) · simulation screen of the settings: `docs/sil_study_screening.md` (A8 validated, others not) · battery dependence (Omar's no-integral offset +13…+23 cm; hence anchors) · only 2 drones, geometric top drone · Omar C reads the unfiltered deck RPM (dropout risk) · mocap tracker pose swaps (battery-related so far) · host SIL gaps (Omar dips, A1) · cf5 uSD card lost files once (always `check_usd_deck.py`) · S1 choice and NS2 weights still open · S3 blocked.

## 10. Before the first study flight
1. Protocol v1.0 frozen (after the shakedown) incl. exact parameter table. 2. Pure-INDI decision (S1) and NS2 weights decision. 3. Logging changes (section 5) done and tested. 4. yaml commits tagged `study-S0/S1/S2`. 5. Processing extended and tested on 10-09 data. 6. Shakedown done: every scenario once per strategy, go/no-go table per cell, throughput measured, speed-0.5 feasibility known.

**Status 2026-10-11:** item 3 (logging: superset config, firmware cap, `--study` gate, flight decisions) is prepared and tested without hardware (not flown), item 5 (processing: `study_metrics.py`, `aggregate.py --study-table`, `study_report.py`, pose bag) is done and tested on the 10-02…10-09 data; items 1, 2, 4, 6 are open (decisions, tags, shakedown: `docs/study_shakedown_checklist.md`, `docs/study_shakedown_card.md`). Earlier status 2026-10-09: none of the six items was done; readiness per controller and the open-work list are in `roadmap_to_comparative_study.md` §1b.

## 11. Open decisions
S1 final (Omar C + Iz 1.5 vs ours) · NS2 retrain or keep (numbers and options K / R1 / R2: `ns2_retrain_options.md`) · tier 2/3 go/no-go · exact speed↔parameter mapping per scenario · whether ours INDI is flown as a supplementary 4th column in tier 1 · cf_second firmware hash.

## 12. Change log
- v0.1 (2026-10-09): first draft from the decisions of 2026-10-09 (scenarios, sweeps, 5 flights per cell, controller-grouped flights, clean/excluded by the pilot, both drones measured, NS2 retraining left open).
- v0.2 (2026-10-11): desk preparation folded in — A8 speed flag `--duration`, pose-bag clock, attitude metrics defined (measured + nominal-reference deviation, `e_R` not cross-comparable), uSD superset config and `--study` gate (manifest, frozen blocks, pre-takeoff check, safe landing), flight plan and shakedown card generated and validated, readiness matrix, SIL screening of the settings. Open decisions unchanged (section 11).
