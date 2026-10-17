# NEXT LAB SESSION — flash study_tree_v2 and re-validate S2 / S3 (A, B) — PLANNED (date open)

*(The NS2 training-data collection that this file used to plan is DONE on 2026-10-16/17 — kept below as the record: `2026-10-16.md`, `2026-10-17.md`, `ns2_training_data_overview_2026-10-17.md`.)*

## Session 2026-10-?? : v2 validation (ordered)
1. **Prep:** pull both repos; flash `flying_drone_stack/firmware_app/build_artifacts/study_tree_v2.bin` on BOTH drones (sha256 `6f2d57d69c295e68…`, 533 152 B; contains `ns2_v2` weights and the `ns2_mode` switch; verify the hash with `sha256sum` before flashing), power-cycle; `ns2c_bench_check.py --readback` on both (expect `rnn.ready 1`, `rnn.div 10`, `ns2_mode` readable; controller 9 only after `ros2 launch`; confirm the console line `Using OutOfTree4 (9)` / `check_active_controller.py`); `check_usd_deck.py`; fresh batteries; restore the study SD config if the cards were reformatted.
2. **Hover check of all four strategies** (S0, S1, S2, S3): z tracking, `rnn.us_max` (NS2 timing, expect ≈ 526 µs).
3. **A8 and A1, 3 flights each, per strategy to compare:** S2 (`from_main_S2` = geometric + NS2), S3 Version A (`from_main_S3A.patch`), S3 Version B (`from_main_S3B.patch`); S0 and S1 as references (n = 2–3). Same card of checks as before (empty-file check on both cards after every block, `tree_log_check.py`, `check_usd_log_quality.py`, merge, `a8_crossings.py`/study metrics). Record `ns2_mode` per block in the lab doc (the uSD logs cannot tell A from B).
4. **After:** supervisor decision S3 A/B (meeting 2026-10-22), freeze protocol v1.0, tags, then the tier-1 study.

---
# (record) NS2 training data collection on the Omar C tree (S0) — DONE 2026-10-16/17

Previous session (tree validation, closed): `2026-10-15.md`. Plan and rationale: `docs/plan_ns2_retrain_data_2026-10-14.md`. Card: `docs/ns2_training_flight_card.md` (**119 flights** after the 2026-10-15 redesign: dz {0.25, 0.5, 0.75} × peak speed {0.1 … 0.5} m/s for A8 / A2 / A3 / A5, A7 and A4 speed sweeps, A1 dz grid, 20 R1; 66 min of flight, ≈ 10 h with overheads, i.e. about 4 sessions; ordered core-first so every session is a balanced subset). Speeds are requested directly (`--speed`, A8 and A5 now convert exactly to duration / period); no `--study` in any command.

## Next lab session (filtered after part 1 on 2026-10-16, see `2026-10-16.md`) — A1, R1, A4 (offset steps), A7
Chosen by coverage (gate time by lateral offset / real gap / speed, bank vs. each scenario): the bank has 267 s in the gate, but only 13 s with a real gap below 0.45 m, 39 s above 0.4 m/s and no usable A1 hold. **Skipped on purpose:** A5 (≈ A8), A2/A3 repeats at dz 0.25/0.75 (lateral offset < 0.05 m is already covered; label flat across dz), A4 offset-0.05 sweeps #76–90 (≈ A3, 134 of 170 s at < 0.1 m/s). A7 is kept by decision of the user although the study never goes below dz 0.25 (the evidence argued to skip it): fly it last, only with clean cards and fresh batteries.

| Order | Block | Cards | Why it is in | Flights | Flight time |
|---|---|---|---|---|---|
| 0 | Prep: back up + reformat both cards, `restore_study_sd_config.py`, `check_usd_deck.py` on both, fresh batteries (rest ≥ 4.1 V), bench `--readback` (controller 9, indi 0, rnn.en 0) | – | 4 of 5 A1 flights today were lost to empty uSD files | – | – |
| 1 | **A1** stacked hold, dz 0.5 / 0.25 / 0.75 ×2 (+ 0.375, 0.625 ×2 optional) | #66–75 | study scenario; only static / lateral-0 / speed-0 data; 1 of 10 usable | 6–10 | 15 s each |
| 2 | **R1** random waypoints, seeds 101–110 first (111–120 after a quick retrain check) | #100–119 | 2.7× the gate time of the bank, widest lateral/gap spread; never flown (seed 101 first, hand on the abort) | 10–20 | 43–68 s each |
| 3 | **A4** offset steps 0 / 0.1 / 0.15 m at 0.3 m/s, dz 0.5 / 0.25 / 0.75 | #91–99 | controlled lateral-offset dependence | 9 | 18 s each |
| 4 | **A7** descend-through | #61–65 | continuous gap sweep (only data < dz 0.25; outside the study range); needs `--allow-extreme`, no `--height`; ends 0.1 m above the bottom drone | 5 | 8–29 s each |
Optional if time remains: A2 at 0.4 / 0.5 m/s, dz 0.25 and 0.75 (#29, #30, #49, #50), the cheapest way to add fast in-gate time.

Per-flight tracker with all 48 commands (A1 10, R1 20, A4 9, A7 5, optional A2 4): `ns2_collection_part2_tracker.md`.

Commands (all start with `ros2 run crazyflie_examples run_formation`):
- A1: `--scenario A1 --dz <0.5|0.25|0.75|0.375|0.625> --hold 15.0 --height 0.5 --auto-center --yes` (#66/67 dz 0.5, #68/69 0.25, #70/71 0.75, #72/73 0.375, #74/75 0.625)
- R1: `--scenario R1 --seed <101…120> --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` (#100 = seed 101 … #119 = seed 120)
- A4: `--scenario A4 --dz <0.5|0.25|0.75> --speed 0.3 --offset <0.0|0.1|0.15> --motion line --rotate 90 --height 0.5 --auto-center --yes` (line along y, offset along x; #91–93 dz 0.5, #94–96 dz 0.25, #97–99 dz 0.75; offsets in the order 0.0, 0.1, 0.15)
- A7: `--scenario A7 --speed <0.1…0.5> --auto-center --yes --allow-extreme` (#61 = 0.1 … #65 = 0.5)
After each block: `copy_usd_log.py … --tag ns2train_<block>` (cf_second card first, then cf5), check that every flight has a file on BOTH cards before the next block (empty-file check), `check_usd_log_quality.py`, `merge_training_bank.py --date <day> --only-tag-contains ns2train`.

## Added 2026-10-17: S3 Version A / B
The Omar controller now has `ctrlOmarIndi.ns2_mode` (0 = Version A, plain sum, default; 1 = Version B, INDI on what NS2 left over; `docs/ns2_s3_versions_A_B_2026-10-17.md`). After `study_tree_v2` is flashed (it contains the parameter): fly S3 (`from_main_S3A.patch`) and S3B (`from_main_S3B.patch`) on A8 and A1, 3 flights each (the supervisor decides which is the study's S3, meeting 2026-10-22).

## 0. Goal
Dedicated training flights (never study repeats) for the retrained NS2 that serves both **S2 (geometric + NS2)** and **S3 (INDI + NS2)**. All flights on S0 (indi 0, NS2 off, deck RPM on, both drones controller 9). The label `a_res_z = a_imu_f − a_rpm_f` is reconstructed offline with Omar's own filter chain (`omar_labels.py`, validated r = 0.998), so it is the same signal the INDI term measures in S1/S3.

## 1. What is already in place
| Item | State |
|---|---|
| Image `study_tree_v1.bin` on both drones, 96-variable uSD logging, deck RPM | flashed, flown today (NS2 off in S0, no effect on the data) |
| yaml for S0 | yaml on `main` is S0 since crazyswarm2 `4b65435` (the `tree_S*.patch` files no longer apply); `git pull` is enough |
| 119-flight card | speed-based (exact), no `--study`; all commands validated by the scenario generator and geofence; A8 `--speed` / A5 `--speed` added to the scenario builder (crazyswarm2) |
| Old valid flights (v1 bank, 26) | **used for v1 (decision 2026-10-15), all 26 relabelled** with the tree definition (`relabel_v1_bank.py` → `experiments/logs/bank_v1_relabelled/`, label r 0.88–0.98 vs the old logged label); LOO on that bank: 22/26 OK, A6 and A1 09-28 17:47 hard fail (A6 is out of scope), details `docs/ns2_tree_retrain_dryrun_2026-10-15.md` §4. v1 is in-sample on these flights, so gate (b) compares only on new flights |
| R1 random-waypoint scenario | planned offline, goes through `safety.check()`, reproducible from the seed, 68 % of the time inside the NS2 gate; **never flown on hardware** |
| Label adapter | `omar_labels.py` + `merge_usd_logs.py --omar-labels deck`, validated against logged labels |
| Retrain chain | `experiments/analysis/run_c2_lab_retrain.py` → `dataset.py` → LOO gate → `export_weights_c.py` (unchanged downstream) |
| Log tools | `copy_usd_log.py --tag`, `check_usd_log_quality.py`, `tree_log_check.py`, `ns2c_bench_check.py` |

## 2. To prepare (ordered)
| # | What | Who | Why |
|---|---|---|---|
| 1 | Back up both cards, **reformat, then `restore_study_sd_config.py` on both**, then `check_usd_deck.py` before the first flight | lab | today's write gaps (1.3 % mean) and empty files; reformat removes the config |
| 2 | Batteries: charge everything, change below ≈ 4.1 V resting (four S2 flights were lost to 2.6 V cut-offs) | lab | each lost flight is lost training data |
| 3 | Bench check on both drones (`ns2c_bench_check.py`), confirm `Using OutOfTree4 (9)` after any flash/power-cycle | lab | known silent-wrong-controller failure mode |
| 4 | ~~Retrain acceptance rule~~ **DECIDED 2026-10-15 (b):** leave-one-flight-out with the existing script thresholds (R² ≥ 0.35 and ≥ 50 % vs zero), unchanged from v1, plus v1 weights scored on the same left-out flights (new must be at least as good); old 26 flights used (relabelled) | done | — |
| 5 | ~~Generic merge for the training bank~~ **DONE 2026-10-15:** `experiments/analysis/merge_training_bank.py` (pairs by `usd.runTag`, `--omar-labels deck`, writes the manifest) | desk | tested on today's 13 flights, the 3 outliers skipped |
| 6 | ~~Dry run of the retrain chain~~ **DONE 2026-10-15** (`docs/ns2_tree_retrain_dryrun_2026-10-15.md`): chain runs end to end in 4 min, 12/13 LOO folds R² 0.71–0.88; gate thresholds in the script are the old v1 ones | desk | — |
| 7 | ~~Clamped fraction~~ **DONE 2026-10-15:** 0.00 % in flight (bottom vehicle, all strategies, incl. takeoff); the 8.0 is a motors-off ground reading | desk | — |
| 8 | First-flight safety for R1: first seed with a hand on the abort, geofence, fresh batteries; abort rule protocol §4 | lab | R1 is new on hardware |

## 3. Session flow
1. Setup (items 1–3 above), S0 yaml, relaunch CS2 after the yaml change.
2. Fly the card in order of its blocks (A1 → A4 → A8 → A2/A3/A5/A7 → R1); copy the logs after each block (`copy_usd_log.py … --tag ns2_train_<block>`, cf_second card first), run `check_usd_log_quality.py`.
3. After the session: `tree_log_check.py` on every file (must read S0: INDI off, NS2 off), merge both cards, `run_c2_lab_retrain.py` dry-run on the new bank.
4. After the full bank: retrain → LOO gate → your sign-off → `export_weights_c.py` → image `study_tree_v2` → bench → re-validate S2/S3 (A8, A1) → re-freeze protocol.

## 4. Known limits / risks
- High-speed coverage inside the gate comes mostly from A4/A8, because R1 legs are rest-to-rest (median bottom speed in the gate 0.07 m/s); check the achieved coverage after the first session with `random_flight_coverage.py`-style statistics on the real data.
- `--study` gate not usable (NaN readback): training strategy is verified from the log, not from the radio meta.
- A7 needs `--allow-extreme`; if a block crashes twice, stop that block (protocol §4) and document, do not retry at the same setting.
- The retrained network is only valid for the tree's definition of the residual (mass 0.0427, Omar's k_t, 30 Hz, deck RPM); a change of image constants invalidates the labels.
