> **CLOSED 2026-10-23** — flown and analysed, see `2026-10-23.md` (40 flights, retention 1.03). Kept as the record of the procedure.

# Lab session checklist — Topic 2: unseen-settings block (40 flights, S0–S3, ns2_v2 unchanged)

Plan and pass rule: `ns2_unseen_block_plan_2026-10-22.md` (decided 2026-10-22: FBL-containing logging image, thresholds fixed, no `--study`, as all pilot sessions). Goal of the session: flash the logging image, check it, fly 40 flights, copy and merge the logs. No new abort or exclusion rules: write down what happened, decide afterwards.

## 0. Before leaving the desk
- [ ] Image file is on the lab PC (`build_artifacts` is git-ignored, copy it by hand): `flying_drone_stack/firmware_app/build_artifacts/study_tree_v2_fbl_estlog_candidate.bin`
- [ ] `sha256sum` on the lab PC = `4fa0a130c698f217728d8b5dcf7464cb84a0b5bd4196b3ac4123a8100bad012b` (580 908 B)
- [ ] `git pull` in `flying_robot_course` and `crazyswarm2` (lab PC); yaml on `main` = S0
- [ ] Charged batteries: at least 2 fresh packs per drone per block (INDI strategies sag the pack to 3.0–3.5 V loaded: a fresh pack per INDI flight, rule for all strategies: note the resting voltage before every flight)
- [ ] Both uSD cards: count the `thesis` files; **at 85 or more move them into an `_archive_<date>` folder** (at 100 the drone resets in the air); both cards mounted

## 1. Flash and bench check (CS2 stopped, nothing flying)
- [ ] Flash cf5 (bottom): `cfloader flash ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build_artifacts/study_tree_v2_fbl_estlog_candidate.bin stm32-fw -w radio://0/80/2M/E7E7E7BB02`, power-cycle
- [ ] Flash cf_second (top): same file, `-w radio://0/80/2M/E7E7E7E7E9`, power-cycle
- [ ] `python3 flying_drone_stack/tools/ns2c_bench_check.py <uri> --readback` on both: `rnn.ready 1`, `rnn.div 10`, controller / indi / rnn.en as expected (CS2 stopped)
- [ ] `python3 flying_drone_stack/tools/check_usd_deck.py <uri>` on both (deck detected, no 0-byte logs; the URI is argv[1]); `restore_study_sd_config.py` if the cards were reformatted
- [ ] Verify with the console line that the controller slot really runs after launch (controller 9, Omar C tree)

## 2. Logging check (the point of the new image): one short S0 flight or hover
- [ ] Yaml = S0 (`lab_check_strategy.py` says S0); launch; one A1 flight (or a hover)
- [ ] Log (cf5): `ctrlOmarIndi.a_rpm_f*` and `a_imu_f*` are **non-zero** with `indi 0` (decode with `flying_drone_stack/tools/decode_usd_log.py`, check with `experiments/analysis/tree_log_check.py`)
- [ ] The flight looks like S0 (same behaviour as the pilot S0 flights); if the logging check fails: stop and report, do not fly the block

## 3. The 40 flights (blocks in the order S1 → S3 → S0 → S2, 10 flights per block)
Per block, before the launch: `cd ~/georg/ros2_ws/src/crazyswarm2 && git checkout crazyflie/config/crazyflies.yaml` then the patch of the block (S0: none), then `python3 flying_drone_stack/tools/lab_check_strategy.py` (must name the block's strategy and report the installed yaml as identical, not STALE; rebuild as in the earlier sessions if stale), then `ros2 launch crazyflie launch.py`; read the cf5 `Update parameter` lines.
Commands (all end with `--auto-center --yes`, prefix `ros2 run crazyflie_examples run_formation`):

- `A4a`: `--scenario A4 --dz 0.4 --speed 0.35 --offset 0.075 --motion line --rotate 90 --height 0.5`
- `A4b`: `--scenario A4 --dz 0.4 --speed 0.35 --offset 0.125 --motion line --rotate 90 --height 0.5`
- `A8`: `--scenario A8 --dz 0.4 --speed 0.35 --height 0.5 --passes 4`
- `A1`: `--scenario A1 --dz 0.45 --hold 15.0 --height 0.5`

### Block S1 — INDI on, NS2 off (`indi 3 / rnn.en 0`)
- [ ] yaml: `git apply crazyflie/config/study/from_main_S1.patch`; `lab_check_strategy.py` = S1; launch; cf5 parameter lines read; card file counts noted (cf_second ___ / cf5 ___)
| # | cell | command key | resting V cf5 / cf_second | uSD file on BOTH cards | notes |
|---|---|---|---|---|---|
| S1-1 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| S1-2 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| S1-3 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| S1-4 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| S1-5 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S1-6 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S1-7 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S1-8 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
| S1-9 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
| S1-10 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
- [ ] after the block: copy the cards (cf_second first): `python3 flying_drone_stack/tools/copy_usd_log.py <mount> cf_second --file thesisNN --tag unseen_S1` and `… <mount> cf5 --file thesisNN --tag unseen_S1` per flight (`--file` names the card file, e.g. `thesis84`; without it the latest file is copied); every flight has a file on both cards; `check_usd_log_quality.py` on the files

### Block S3 — INDI on, NS2 on (`indi 3 / rnn.en 1`)
- [ ] yaml: `git apply crazyflie/config/study/from_main_S3.patch`; `lab_check_strategy.py` = S3; launch; cf5 parameter lines read; card file counts noted (cf_second ___ / cf5 ___)
| # | cell | command key | resting V cf5 / cf_second | uSD file on BOTH cards | notes |
|---|---|---|---|---|---|
| S3-1 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| S3-2 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| S3-3 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| S3-4 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| S3-5 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S3-6 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S3-7 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S3-8 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
| S3-9 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
| S3-10 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
- [ ] after the block: copy the cards (cf_second first): `python3 flying_drone_stack/tools/copy_usd_log.py <mount> cf_second --file thesisNN --tag unseen_S3` and `… <mount> cf5 --file thesisNN --tag unseen_S3` per flight (`--file` names the card file, e.g. `thesis84`; without it the latest file is copied); every flight has a file on both cards; `check_usd_log_quality.py` on the files

### Block S0 — geometric, NS2 off (`indi 0 / rnn.en 0`)
- [ ] yaml: yaml on `main` as is; `lab_check_strategy.py` = S0; launch; cf5 parameter lines read; card file counts noted (cf_second ___ / cf5 ___)
| # | cell | command key | resting V cf5 / cf_second | uSD file on BOTH cards | notes |
|---|---|---|---|---|---|
| S0-1 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| S0-2 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| S0-3 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| S0-4 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| S0-5 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S0-6 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S0-7 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S0-8 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
| S0-9 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
| S0-10 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
- [ ] after the block: copy the cards (cf_second first): `python3 flying_drone_stack/tools/copy_usd_log.py <mount> cf_second --file thesisNN --tag unseen_S0` and `… <mount> cf5 --file thesisNN --tag unseen_S0` per flight (`--file` names the card file, e.g. `thesis84`; without it the latest file is copied); every flight has a file on both cards; `check_usd_log_quality.py` on the files

### Block S2 — geometric + NS2 (`indi 0 / rnn.en 1`)
- [ ] yaml: `git apply crazyflie/config/study/from_main_S2.patch`; `lab_check_strategy.py` = S2; launch; cf5 parameter lines read; card file counts noted (cf_second ___ / cf5 ___)
| # | cell | command key | resting V cf5 / cf_second | uSD file on BOTH cards | notes |
|---|---|---|---|---|---|
| S2-1 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| S2-2 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| S2-3 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| S2-4 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| S2-5 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S2-6 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S2-7 | A8 dz 0.4, 0.35 m/s | `A8` | ___ / ___ | [ ] | |
| S2-8 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
| S2-9 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
| S2-10 | A1 dz 0.45 | `A1` | ___ / ___ | [ ] | |
- [ ] after the block: copy the cards (cf_second first): `python3 flying_drone_stack/tools/copy_usd_log.py <mount> cf_second --file thesisNN --tag unseen_S2` and `… <mount> cf5 --file thesisNN --tag unseen_S2` per flight (`--file` names the card file, e.g. `thesis84`; without it the latest file is copied); every flight has a file on both cards; `check_usd_log_quality.py` on the files

## 4. After the last block
- [ ] Merge: `python3 experiments/analysis/merge_training_bank.py --date <YYYY-MM-DD> --out experiments/logs/bank_unseen_<date> --only-tag-contains unseen --rpm deck` (as for the earlier pilot banks); every flight merged, run tags match the radio metas
- [ ] Strategy of every flight verified from the logs (done automatically by the analysis script, `ns2_applied_check.py`)
- [ ] Final card counts written down, archive at 85 or more; yaml on `crazyswarm2` `main` back to S0 (`git checkout crazyflie/config/crazyflies.yaml`)
- [ ] Session doc `docs/lab_sessions/<date>.md` (5-section template): flights flown / lost, incidents, card state
- [ ] Analysis (desk, afterwards): `unseen_block_analysis.py --banks bank_unseen_<date> --out experiments/results/unseen_block_<date>`, then `nn_vs_indi_raw.py` / `nn_vs_indi_check.py` on the same bank (commands in the plan); verdict = retention R against the fixed thresholds; update `docs/meetings/2026-10-29.md`

## 5. Things to remember
- Pilot flights ran on the old image: the code differs (control byte-identical in the replay only); the first block also checks that S0/S1/S2/S3 still behave as in the pilot.
- A missing or empty uSD file means the flight does not count (listed, not backfilled from radio).
- Protocol v1.0 will have to name this image's hash; the live `study_manifest.json` is stale (still `study_tree_v1.bin`), not updated in this session (no `--study`).
- Not in this session: FBL flights (paused until after this block), thesis writing.
