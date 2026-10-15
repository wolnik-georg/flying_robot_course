# Next lab session — NS2 training data collection on the Omar C tree (S0) — PLANNED (date open)

Previous session (tree validation, closed): `2026-10-15.md`. Plan and rationale: `docs/plan_ns2_retrain_data_2026-10-14.md`. Card: `docs/ns2_training_flight_card.md` (66 flights, 32 min of flight, ≈ 5.5 h with overheads, i.e. 2–3 sessions).

## 0. Goal
Dedicated training flights (never study repeats) for the retrained NS2 that serves both **S2 (geometric + NS2)** and **S3 (INDI + NS2)**. All flights on S0 (indi 0, NS2 off, deck RPM on, both drones controller 9). The label `a_res_z = a_imu_f − a_rpm_f` is reconstructed offline with Omar's own filter chain (`omar_labels.py`, validated r = 0.998), so it is the same signal the INDI term measures in S1/S3.

## 1. What is already in place
| Item | State |
|---|---|
| Image `study_tree_v1.bin` on both drones, 96-variable uSD logging, deck RPM | flashed, flown today (NS2 off in S0, no effect on the data) |
| yaml for S0 | `git checkout crazyflie/config/crazyflies.yaml && git apply crazyflie/config/study/tree_S0.patch` (yaml in the tree is still S2) |
| 66-flight card | regenerated without `--study` (readback gate NaN, not needed here); commands validated by the scenario generator |
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
| 4 | Retrain acceptance rule and hold-out (e.g. which R1 seeds stay out) — **you define**; whether the old 26 flights are used at all (current plan: no) | you | the gate must be fixed before the data is seen |
| 5 | Generic merge for the training bank (pair cards by `usd.runTag`, `--omar-labels deck`, one command): `tree_validation_merge.py` is hard-wired to the tree labels | desk | one step per card pair instead of ad-hoc merges |
| 6 | Dry run of the retrain chain on today's merged flights (labels via the adapter, LOO on 13 flights) to prove the chain end to end before 66 flights exist | desk | finds adapter/dataset problems now |
| 7 | Clamped fraction of `rnn_clamped` in today's flights (the last prediction on the ground read 8.0 = clamp) | desk | if NS2 clamps often in flight the label range matters for training |
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
