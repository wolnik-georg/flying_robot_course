# Lab session checklist — 2026-10-26: FBL (S4) as the authors flew it

Plan and background: `2026-10-26.md` (session doc, analysis checklist), `fbl_stock_inner_session_plan.md` (plan), `docs/fbl/STOCK_INNER_LOOP_2026-10-24.md`.

Goal of the session:
- flash the rebuilt FBL image;
- refly the 2026-10-24 cells with both gain sets;
- fly C_D 0.236 vs 0.55;
- fly the paper's own hardware test (block V).

No new abort or exclusion rules: write down what happened, decide afterwards. The priority if time is short is at the end.

## 0. Before leaving the desk
- [ ] The image file is on the lab PC. `build_artifacts` is git-ignored, so copy it by hand: `flying_drone_stack/firmware_app/build_artifacts/study_tree_v2_fbl_stockinner_nn4_full_candidate.bin`
- [ ] `sha256sum` on the lab PC = `e3593a6f39a42a61789c5631784ae691d2ecb964f3a6d88bf14d02af6b673a09`
- [ ] `git pull` in `flying_robot_course`. In crazyswarm2 on the lab PC: `git fetch origin && git checkout study_FBL_stock && git pull`, then rebuild (`colcon build` from the workspace root, `cpp` only).
- [ ] Charged batteries: at least 3 fresh packs per drone per block. Note the resting voltage before every flight.
- [ ] Both uSD cards: count the `thesis` files. **This session is up to 62 flights:** archive into `_archive_<date>` now if a card holds more than 20 files, and recount after every block (at 100 files the drone resets in the air).
- [ ] cf5 card has the FBL config: `python3 flying_drone_stack/tools/restore_study_sd_config.py <mount> --fbl` (cf_second: study config, no `--fbl`).
- [ ] Ruler or caliper for the propeller.

## 1. Setup and bench check (CS2 stopped, nothing flying)
- [ ] cf5 propeller radius (tip to hub centre): ____ mm (the physics model assumes 45 mm)
- [ ] Flash cf5 (bottom): `cfloader flash <bin> stm32-fw -w radio://0/80/2M/E7E7E7BB02`, power-cycle
- [ ] Flash cf_second (top): same file, `-w radio://0/80/2M/E7E7E7E7E9`, power-cycle
- [ ] `python3 flying_drone_stack/tools/fbl_bench_check.py radio://0/80/2M/E7E7E7BB02` → `ALL PASS`, with `ctrlFbl.inner 1`, `nn_en 1`, `abg_on_new 0` at boot
- [ ] Same on cf_second (`…E7E7E7E7E9`)
- [ ] `python3 flying_drone_stack/tools/fbl_init_survival_check.py radio://0/80/2M/E7E7E7BB02` → `PASS`
- [ ] `python3 flying_drone_stack/tools/check_usd_deck.py <uri>` on both (deck detected, no 0-byte logs)
- [ ] `python3 flying_drone_stack/tools/lab_check_strategy.py` → FBL on cf5, S0 on cf_second, installed yaml identical (not STALE)
- [ ] `ros2 launch crazyflie launch.py`. Check the console line for controller 9 and the cf5 `Update parameter` lines.
- [ ] `python3 flying_drone_stack/tools/fbl_bench_check.py radio://0/80/2M/E7E7E7BB02 --readback` → `preset 1 (ours), inner loop: STOCK rate PID (inner 1)`
- [ ] A2 line command checked once: `ros2 run crazyflie_examples run_formation --scenario A2 --dz 0.4 --path line --speed 0.4 --height 0.5 --check` → "checks passed" (and `--dz 0.3`)

## 2. Flights
**Commands:** all start with `ros2 run crazyflie_examples run_formation` and end with `--auto-center --yes`.

| Key | Arguments |
|---|---|
| `A1` | `--scenario A1 --dz 0.45 --hold 15.0 --height 0.5` |
| `A8` | `--scenario A8 --dz 0.4 --speed 0.35 --height 0.5 --passes 4` |
| `A4a` | `--scenario A4 --dz 0.4 --speed 0.35 --offset 0.075 --motion line --rotate 90 --height 0.5` |
| `A4b` | `--scenario A4 --dz 0.4 --speed 0.35 --offset 0.125 --motion line --rotate 90 --height 0.5` |
| `V4` | `--scenario A2 --dz 0.4 --path line --speed 0.4 --height 0.5` |
| `V3` | `--scenario A2 --dz 0.3 --path line --speed 0.4 --height 0.5` |

**Settings:** change them on the ground only, between flights:
`ros2 param set /crazyflie_server cf5.params.ctrlFbl.<name> <value>`
- They are recorded in `meta.json` (`controller_variant`).
- Before every block, check the printed `controller variant: … inner 1 nn_en … preset … cd …` line.

**Every flight:** the uSD log runs from before takeoff. After landing, check that a new file is on BOTH cards and note the setting.

### Block 1 — preset 1 (ours), C_D 0.55, network on (firmware default)
- [ ] Settings: default (`preset 1`, `cd 0.55`, `nn_en 1`); readback ok; card counts (cf_second ___ / cf5 ___)

| # | cell | key | resting V cf5 / cf_second | file on BOTH cards | notes (yaw, sideways drift, wobble, trip) |
|---|---|---|---|---|---|
| 1-1 | A1 (shakedown) | `A1` | ___ / ___ | [ ] | |
| 1-2 | A1 | `A1` | ___ / ___ | [ ] | |
| 1-3 | A1 | `A1` | ___ / ___ | [ ] | |
| 1-4 | A8 | `A8` | ___ / ___ | [ ] | |
| 1-5 | A8 | `A8` | ___ / ___ | [ ] | |
| 1-6 | A8 | `A8` | ___ / ___ | [ ] | |
| 1-7 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| 1-8 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| 1-9 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| 1-10 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| 1-11 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| 1-12 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
- [ ] After the block: copy the cards, cf_second first: `python3 flying_drone_stack/tools/copy_usd_log.py <mount> cf_second --file thesisNN --tag fbl_nn4_b1`, then the same for `cf5`, per flight. `check_usd_log_quality.py` on the files.

### Block 2 — preset 0 (the authors' code gains)
- [ ] `ctrlFbl.preset 0`; readback shows preset 0; card counts (cf_second ___ / cf5 ___)
- [ ] First A1 only. Go on only if it holds; otherwise note what happened and set `preset 1` back.

| # | cell | key | resting V cf5 / cf_second | file on BOTH cards | notes |
|---|---|---|---|---|---|
| 2-1 | A1 (test) | `A1` | ___ / ___ | [ ] | |
| 2-2 | A1 | `A1` | ___ / ___ | [ ] | |
| 2-3 | A1 | `A1` | ___ / ___ | [ ] | |
| 2-4 | A8 | `A8` | ___ / ___ | [ ] | |
| 2-5 | A8 | `A8` | ___ / ___ | [ ] | |
| 2-6 | A8 | `A8` | ___ / ___ | [ ] | |
| 2-7 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| 2-8 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| 2-9 | A4 offset 0.075 | `A4a` | ___ / ___ | [ ] | |
| 2-10 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| 2-11 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
| 2-12 | A4 offset 0.125 | `A4b` | ___ / ___ | [ ] | |
- [ ] After the block: `ctrlFbl.preset 1` back. Copy the cards as above, with `--tag fbl_nn4_b2`.

### Block V — validation = the paper's hardware test (A2 straight line, preset 1)
The variants are set by `cd` and `nn_en` only:

| Variant | Settings |
|---|---|
| **N** (Nominal) | `cd 0`, `nn_en 0` |
| **R** (ROM) | `cd 0.55`, `nn_en 0` |
| **L** (Learned) | `cd 0.55`, `nn_en 1` |

- The variants are flown in rotation (N, R, L, N, R, L, …), so battery and drift spread over all three. This is a suggestion, not a rule.
- 5 flights per variant and dz (minimum 3). If time is short: dz 0.4 this session, dz 0.3 next session.
- [ ] Preset 1 readback; card counts (cf_second ___ / cf5 ___)

| # | dz | variant | key | resting V cf5 / cf_second | file on BOTH cards | notes |
|---|---|---|---|---|---|---|
| V-1 | 0.4 | N | `V4` | ___ / ___ | [ ] | |
| V-2 | 0.4 | R | `V4` | ___ / ___ | [ ] | |
| V-3 | 0.4 | L | `V4` | ___ / ___ | [ ] | |
| V-4 | 0.4 | N | `V4` | ___ / ___ | [ ] | |
| V-5 | 0.4 | R | `V4` | ___ / ___ | [ ] | |
| V-6 | 0.4 | L | `V4` | ___ / ___ | [ ] | |
| V-7 | 0.4 | N | `V4` | ___ / ___ | [ ] | |
| V-8 | 0.4 | R | `V4` | ___ / ___ | [ ] | |
| V-9 | 0.4 | L | `V4` | ___ / ___ | [ ] | |
| V-10 | 0.4 | N | `V4` | ___ / ___ | [ ] | |
| V-11 | 0.4 | R | `V4` | ___ / ___ | [ ] | |
| V-12 | 0.4 | L | `V4` | ___ / ___ | [ ] | |
| V-13 | 0.4 | N | `V4` | ___ / ___ | [ ] | |
| V-14 | 0.4 | R | `V4` | ___ / ___ | [ ] | |
| V-15 | 0.4 | L | `V4` | ___ / ___ | [ ] | |
| V-16 | 0.3 | N | `V3` | ___ / ___ | [ ] | |
| V-17 | 0.3 | R | `V3` | ___ / ___ | [ ] | |
| V-18 | 0.3 | L | `V3` | ___ / ___ | [ ] | |
| V-19 | 0.3 | N | `V3` | ___ / ___ | [ ] | |
| V-20 | 0.3 | R | `V3` | ___ / ___ | [ ] | |
| V-21 | 0.3 | L | `V3` | ___ / ___ | [ ] | |
| V-22 | 0.3 | N | `V3` | ___ / ___ | [ ] | |
| V-23 | 0.3 | R | `V3` | ___ / ___ | [ ] | |
| V-24 | 0.3 | L | `V3` | ___ / ___ | [ ] | |
| V-25 | 0.3 | N | `V3` | ___ / ___ | [ ] | |
| V-26 | 0.3 | R | `V3` | ___ / ___ | [ ] | |
| V-27 | 0.3 | L | `V3` | ___ / ___ | [ ] | |
| V-28 | 0.3 | N | `V3` | ___ / ___ | [ ] | |
| V-29 | 0.3 | R | `V3` | ___ / ___ | [ ] | |
| V-30 | 0.3 | L | `V3` | ___ / ___ | [ ] | |
- [ ] After the block: `cd 0.55`, `nn_en 1` back. Copy the cards as above, with `--tag fbl_nn4_bV`.

### Block 3 — C_D: the authors' 0.236 vs ours 0.55 (preset 1, network off)
The network is off for both values, because it was trained relative to C_D 0.55.
- [ ] `ctrlFbl.nn_en 0`; card counts (cf_second ___ / cf5 ___)

| # | cell | setting | key | resting V cf5 / cf_second | file on BOTH cards | notes |
|---|---|---|---|---|---|---|
| 3-1 | A1 | `cd 0.236` | `A1` | ___ / ___ | [ ] | |
| 3-2 | A1 | `cd 0.55` | `A1` | ___ / ___ | [ ] | |
| 3-3 | A1 | `cd 0.236` | `A1` | ___ / ___ | [ ] | |
| 3-4 | A1 | `cd 0.55` | `A1` | ___ / ___ | [ ] | |
| 3-5 | A8 | `cd 0.236` | `A8` | ___ / ___ | [ ] | |
| 3-6 | A8 | `cd 0.55` | `A8` | ___ / ___ | [ ] | |
| 3-7 | A8 | `cd 0.236` | `A8` | ___ / ___ | [ ] | |
| 3-8 | A8 | `cd 0.55` | `A8` | ___ / ___ | [ ] | |
- [ ] After the block: `cd 0.55`, `nn_en 1` back (= default). Copy the cards as above, with `--tag fbl_nn4_b3`.

## 3. After the flights
- [ ] CS2 stopped: `python3 flying_drone_stack/tools/fbl_bench_check.py radio://0/80/2M/E7E7E7BB02 --post-flight` → `steps > 0`, `faults 0`
- [ ] Every flight has a file on both cards. List lost or aborted ones in `2026-10-26.md` §1 (not backfilled from radio).
- [ ] Merge: `python3 experiments/analysis/merge_training_bank.py --date 2026-10-26 --out experiments/logs/bank_fbl_nn4_2026-10-26 --only-tag-contains fbl_nn4`
- [ ] Analysis (`~/.pyenv/versions/flying_robots/bin/python`):
  - `experiments/analysis/fbl_hw_overview.py --out experiments/results/fbl_hw_overview_2026-10-26 --fbl-banks bank_fbl_nn4_2026-10-26` (3D RMSE, z_max, groups by `controller_variant`)
  - `fbl_integral_windup.py --fbl-banks bank_fbl_nn4_2026-10-26`
  - `fbl_replay_check.py`
  - `review_fbl_yaw_budget.py`
  - `review_fbl_lateral_chain.py`
- [ ] Fill in `2026-10-26.md` §1–§5 and the "What to look for" table. Update `docs/07` and `next_steps_checklist.md` the same night.

## 4. Priority if time is short
1. Block 1
2. Block 2: the A1 test only
3. Block V at dz 0.4 (15 flights)
4. Block 3
5. The rest of block 2, then block V at dz 0.3

Full session: 12 + 12 + 30 + 8 = 62 flights.
