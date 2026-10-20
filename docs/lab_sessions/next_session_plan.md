# (DONE and closed 2026-10-20: `2026-10-20.md` §0) — more data on A1, A8 and A4 for all four strategies (plan of 2026-10-20)

*(The v2 validation and the first unified comparison were DONE on 2026-10-19: `2026-10-19.md` §0, `docs/tree_comparison_2026-10-19.md`. The older plans below are kept as records.)*

**Video flights 2026-10-20 added (A1 target now 5 per strategy, all at or above 5; `2026-10-20.md` §13).** Earlier: **Progress 2026-10-20 (COMPLETE):** the last S1 A4 0.15 flights are flown and merged — **all cells are at or above their targets** (`2026-10-20.md` §12); the pilot data set of this plan is finished. Earlier: **Progress 2026-10-20 (final):** all cells at or above their targets except **S1 A4 offset 0.15: 2 more flights** (`2026-10-20.md` §9–11, unified overview §10). Earlier: **Progress 2026-10-20 (latest):** S3 completed and the S0 A4 flights done (11 flights, all complete); **remaining: S0 A8 +1 and A1 +1, then the S1 block (A4 ×9, A8 +1, A1 +1)** = 13 flights (`2026-10-20.md` §7–8). Earlier: the S3 block is flown too (9 of 12 flights complete; 3 lost to battery collapses / empty files); with the new A1 / A8 target of 4 the remaining flights are S3: A4 0.10 +1 and 0.15 +1; S0 and S1: 11 each, `2026-10-20.md` §5–6. Earlier: the S2 block is flown (A4 ×0.0/0.05/0.10/0.15, A8, A1; `2026-10-20.md`); S3, S0 and S1 are still to fly. **Decision 2026-10-20: A4 offsets are 0.05 / 0.10 / 0.15 only (offset 0.0 dropped).**

## Goal
Mean and spread (std) per strategy on the three most informative scenarios, at one setting each (**dz 0.5 m, peak speed 0.5 m/s**), so the supervisor can see how big the spread between flights is compared with the difference between controllers. Pilot data for the unified comparison (not the frozen study); the cells A1 / A8 / A4 are also tier-1 cells, so the flights double as the shakedown of their parameters for protocol v1.0.
- **A1** stacked hold, dz 0.5 — steady downwash. **A8** vertical swap — transient crossing. **A4** offset line — lateral offset (edge of the plume and of the NS2 gate).
- A2 circle (sustained tracking in the plume) only if time remains; A3, A5, A7 and other settings are left out on purpose (A3, A5 ≈ A8; A7 is extreme and outside the study's gap range).

## Flights (uSD logs of both drones for every flight; clean flights wanted per cell)
**Decisions 2026-10-20 (user):** A4 offset 0.0 is dropped (it repeats A1 = plume centre at rest and A8 = crossing through the plume centre); A4 is flown at **0.05, 0.10 and 0.15 m**, **3 flights per offset and strategy**, for all four controllers. **A1 and A8 target: 4 clean flights per strategy** (not 5). The 4 flights of S2 at offset 0.0 that already exist stay as a record.
| Cell | S0 | S1 | S2 | S3 | target | have now (S0/S1/S2/S3) | to fly (S0/S1/S2/S3) |
|---|---|---|---|---|---|---|---|
| A1, dz 0.5, hold 15 s | 4 | 4 | 4 | 4 | 4 each | 3 / 3 / 6 / 4 | +1 / +1 / 0 / 0 |
| A8, dz 0.5, 0.5 m/s | 4 | 4 | 4 | 4 | 4 each | 3 / 3 / 4 / 5 | +1 / +1 / 0 / 0 |
| A4, dz 0.5, 0.5 m/s, offset **0.05** | 3 | 3 | 3 | 3 | 3 each | 0 / 0 / 3 / 3 | 3 / 3 / 0 / 0 |
| A4, offset **0.10** | 3 | 3 | 3 | 3 | 3 each | 0 / 0 / 3 / 2 | 3 / 3 / 0 / **+1** |
| A4, offset **0.15** | 3 | 3 | 3 | 3 | 3 each | 0 / 0 / 3 / 2 | 3 / 3 / 0 / **+1** |
| **Flights still to fly** | | | | | | | **S0 11, S1 11, S3 2 = 24 flights, about 2 h** |
Order: **S3 first (2 flights: A4 0.10 ×1, A4 0.15 ×1)**, then S0 and S1 (A4 0.05 ×3, 0.10 ×3, 0.15 ×3, then A8 ×1, then A1 ×1 in each block). Priority if time is short: A4 first, then the A8 and A1 flights.

## Commands (all validated with the scenario generator and the geofence check, 2026-10-20, `problems: []`)
- A1: `ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.5 --hold 15.0 --height 0.5 --auto-center --yes`
- A8 (same command as on 2026-10-19, peak 0.5 m/s): `ros2 run crazyflie_examples run_formation --scenario A8 --dz 0.5 --height 0.5 --passes 4 --auto-center --yes`
- A4 (line along y, offset along x as in the data collection): `ros2 run crazyflie_examples run_formation --scenario A4 --dz 0.5 --speed 0.5 --offset <0.05|0.10|0.15> --motion line --rotate 90 --height 0.5 --auto-center --yes`
- Optional A2: `ros2 run crazyflie_examples run_formation --scenario A2 --dz 0.5 --speed 0.5 --height 0.5 --auto-center --yes`

## Order and procedure
1. **Before the first flight:** drones and propellers checked after the 2026-10-19 crashes; both cards: count the `thesis` files (≥ 85 → archive, `2026-10-19.md` §12); batteries charged; `git pull` on both repos.
2. **Group by strategy, controller order rotated against the last session:** **S2 → S3 → S0 → S1** (last session S0 → S1 → S2 → S3). One yaml state and one launch per block (`S0`: `indi 0 / rnn.en 0`, `S1`: `3 / 0`, `S2`: `0 / 1`, `S3`: `3 / 1`; cf5 lines 225 and 246, `lab_check_strategy.py` before the launch, the cf5 `Update parameter` lines after it).
3. **Inside a block (12–13 flights, about 1 h):** A4 offset 0.05 ×3, 0.10 ×3, 0.15 ×3, then the A8 and A1 top-up flights of that strategy.
4. **Batteries (proposal, to be confirmed by you):** the same rule for all four strategies; note the resting voltage before every flight; INDI strategies sag the pack most (3.2–3.5 V loaded in A1), a fresh pack per INDI flight.
5. **After each block:** copy the cards (cf_second first), merge by run tag (`merge_training_bank.py --scenarios A1 A4 A8 …`), check the strategy of every flight from the logs (`ns2_applied_check.py`, `tree_log_check.py`), check that every flight has a uSD file on both cards (a missing file means the flight does not count).
6. **Analysis afterwards:** extend `docs/tree_comparison_2026-10-19.md` with A4 per offset and the larger A1 / A8 samples (mean, std, n, per-flight points); add the new overview to the meeting doc.

## Still to do before the real comparative study (checklist)
- Supervisor decisions (meeting 2026-10-22): tree approved; study size (tier 1 = 12 settings × strategies × 5 flights); two vehicles first; dz floor; FBL as strategy S4 in tier 1.
- Remaining tier-1 settings not flown yet: A1 dz 0.25 / 0.75, A8 at 0.2 / 0.3 / 0.4 m/s, A2, A3, A5, A7 at 0.4 m/s (flyability with all four controllers, exact parameters for protocol v1.0).
- Freeze protocol v1.0 (exact table, commands, tags `study-S0..S3`, image hash), clean/excluded decision log, `--study` gate, strategy check from the logs, battery rule.
- FBL: gain tuning in simulation, then bench/hover/A1/A8 (`docs/fbl/PROGRESS_AND_RESUME.md`); it joins as its own block.
- Decision for you: whether the pilot flights of 2026-10-19 and of this session count in the study (recommended: no, re-fly the cells under the frozen protocol).

---
*(The NS2 training-data collection that this file used to plan is DONE on 2026-10-16/17 — kept below as the record: `2026-10-16.md`, `2026-10-17.md`, `ns2_training_data_overview_2026-10-17.md`.)*

## Session 2026-10-?? : v2 validation (ordered)
0. **Both uSD cards: count `thesisNN` files first; at ≥ 85 move them into an `_archive_<date>` folder (at 100 the drone resets in the air when logging stops, root cause found 2026-10-19, `2026-10-19.md` §12).**
1. **Prep:** pull both repos; flash `flying_drone_stack/firmware_app/build_artifacts/study_tree_v2.bin` on BOTH drones (sha256 `3ae7304a31d0f01e…`, 532 952 B, contains the `ns2_v2` weights; verify the hash with `sha256sum` before flashing), power-cycle; `ns2c_bench_check.py --readback` on both (expect `rnn.ready 1`, `rnn.div 10`; controller 9 only after `ros2 launch`; confirm the console line `Using OutOfTree4 (9)` / `check_active_controller.py`); `check_usd_deck.py`; fresh batteries; restore the study SD config if the cards were reformatted.
2. **Hover check of all four strategies** (S0, S1, S2, S3): z tracking, `rnn.us_max` (NS2 timing, expect ≈ 526 µs).
3. **A8 and A1, 3 flights each, per strategy to compare:** S2 (`from_main_S2` = geometric + NS2), S3 (`from_main_S3.patch` = INDI + NS2); S0 and S1 as references (n = 2–3; **re-fly S1 on fresh batteries ≥ 3.8 V**, the 2026-10-15 S1 flights ran partly at 3.44–3.49 V). Look at the upward peak of S3 against S1. Same card of checks as before (empty-file check on both cards after every block, `tree_log_check.py`, `check_usd_log_quality.py`, merge, `a8_crossings.py`/study metrics).
4. **After:** freeze protocol v1.0, tags, then the tier-1 study. S3 is decided (two plain switches, `docs/s3_indi_ns2_decision_2026-10-18.md`). Optional diagnostic: S1 with `indi = 1` (headroom test), only if you want the cause of the weak INDI.

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
