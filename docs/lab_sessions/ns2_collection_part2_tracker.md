# Lab part 2 — NS2 training collection on S0 (A1, R1, A4 offset steps, A7) — tracker + commands

Plan and reasons: `next_session_plan.md`; coverage evidence: `experiments/results/ns2_collection_2026-10-16/coverage.md`; part 1 result: `2026-10-16.md`. All flights S0 (yaml on crazyswarm2 `main` = S0, image `study_tree_v1.bin`, no reflash). Every command starts with `ros2 run crazyflie_examples run_formation`. Tick a cell only when BOTH uSD files exist, are non-empty and are copied.

## 0. Before the first flight (all must be done)
- [ ] `git pull` in `flying_robot_course` and `crazyswarm2`; yaml on `main` is S0 (`grep -n "en:" crazyflie/config/crazyflies.yaml` → rnn.en 0 for both drones)
- [ ] Back up both cards (copy `thesis*` somewhere), **reformat both, then `restore_study_sd_config.py` on both**, then `check_usd_deck.py` on both (4 of 5 A1 flights on 10-16 were lost to empty files)
- [ ] Fresh batteries on both drones (resting ≥ 4.1 V, change below that; the A2 block on 10-16 ran at 3.4–3.7 V)
- [ ] Bench `--readback` on both: `ns2c_bench_check.py radio://0/80/2M/E7E7E7BB02 --readback` and `…E7E7E7E7E9 --readback` → controller 9, indi 0, rnn.en 0
- [ ] `ros2 launch crazyflie launch.py` (restart after any yaml change); check cf5 propellers/guards (floor contact in the slow flights on 10-16)

## Rules
- After **every block**: confirm each flight has a non-empty file on **both** cards (a missing file means refly now, not later), then `copy_usd_log.py <mount> <cf_second|cf5> --file thesisNN --tag <tag>` (cf_second card first), `check_usd_log_quality.py`, `tree_log_check.py` (INDI off, NS2 off), then `merge_training_bank.py --date <day> --only-tag-contains ns2train` and `ns2_bank_coverage.py`.
- Abort rule: protocol §4 (two crashes at the same setting → stop that block, document, do not retry at the same setting). R1 seed 101 first with a hand on the abort; change batteries between blocks.
- Outliers (bottom drone reads z ≈ 1.0 m = lost/merged tracking, flights aborted early) are listed in the tracker and excluded in `ns2_bank_coverage.py --exclude`.

## A1 stacked hold (15 s) — copy tag `ns2train_A1`

| # | setting | s | command (after `run_formation`) | flown | uSD cf_second / cf5 | notes |
|---|---|---:|---|---|---|---|
| 66 | dz=0.5 hold=15.0 | 15 | `--scenario A1 --dz 0.5 --hold 15.0 --height 0.5 --auto-center --yes` | ✔ 12-27-58 (dz 0.5) | ✔ / ✔ | one repeat not flown |
| 67 | dz=0.5 hold=15.0 | 15 | `--scenario A1 --dz 0.5 --hold 15.0 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 68 | dz=0.25 hold=15.0 | 15 | `--scenario A1 --dz 0.25 --hold 15.0 --height 0.5 --auto-center --yes` | ✔ 12-30-03 (dz 0.25) | ✔ / ✔ | one repeat not flown |
| 69 | dz=0.25 hold=15.0 | 15 | `--scenario A1 --dz 0.25 --hold 15.0 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 70 | dz=0.75 hold=15.0 | 15 | `--scenario A1 --dz 0.75 --hold 15.0 --height 0.5 --auto-center --yes` | ✔ 12-26-41 (dz 0.75; also 10-16 19-46-03) | ✔ / ✔ | one repeat not flown |
| 71 | dz=0.75 hold=15.0 | 15 | `--scenario A1 --dz 0.75 --hold 15.0 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 72 | dz=0.375 hold=15.0 | 15 | `--scenario A1 --dz 0.375 --hold 15.0 --height 0.5 --auto-center --yes` | ✔ 12-31-27 (dz 0.375) | ✔ / ✔ | one repeat not flown |
| 73 | dz=0.375 hold=15.0 | 15 | `--scenario A1 --dz 0.375 --hold 15.0 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 74 | dz=0.625 hold=15.0 | 15 | `--scenario A1 --dz 0.625 --hold 15.0 --height 0.5 --auto-center --yes` | ✔ 12-34-20 (dz 0.625) | ✔ / ✔ | one repeat not flown |
| 75 | dz=0.625 hold=15.0 | 15 | `--scenario A1 --dz 0.625 --hold 15.0 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |

## R1 random waypoints (seeds 101–110 first, 111–120 after a quick retrain check) — copy tag `ns2train_R1`

| # | setting | s | command (after `run_formation`) | flown | uSD cf_second / cf5 | notes |
|---|---|---:|---|---|---|---|
| 100 | seed=101 | 53 | `--scenario R1 --seed 101 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ✔ 12-45-14 ⚠ near-collision 0.08 m (kept) | ✔ / ✔ | |
| 101 | seed=102 | 43 | `--scenario R1 --seed 102 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ✔ 12-54-15 (min 0.10 m) | ✔ / ✔ | |
| 102 | seed=103 | 46 | `--scenario R1 --seed 103 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ✔ 12-56-17 ⚠ bottom above top 2.6 s, top disturbed 28 cm (min 0.08 m) | ✔ / ✔ | |
| 103 | seed=104 | 55 | `--scenario R1 --seed 104 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ✔ 13-01-05 (bottom above top 0.6 s) | ✔ / ✔ | |
| 104 | seed=105 | 45 | `--scenario R1 --seed 105 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ✔ 13-03-07 | ✔ / ✔ | |
| 105 | seed=106 | 53 | `--scenario R1 --seed 106 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ✔ 13-29-16 (clean); 13-23-04 COLLISION, excluded | ✔ / ✔ | |
| 106 | seed=107 | 46 | `--scenario R1 --seed 107 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ✔ 13-31-17 | ✔ / ✔ | |
| 107 | seed=108 | 59 | `--scenario R1 --seed 108 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ✔ 13-34-05 (min 0.11 m) | ✔ / ✔ | |
| 108 | seed=109 | 59 | `--scenario R1 --seed 109 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | flown 13-52-38, LOST (empty file on both cards) | ✗ / ✗ | SKIPPED (user, 2026-10-17): the drone gets too close to the boundary and is lost; the plan has the widest y extent (±0.78 m vs ≤ 0.70 for the other flown seeds) and the highest z (1.40 m vs ≤ 1.33) |
| 109 | seed=110 | 46 | `--scenario R1 --seed 110 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ✔ 13-55-39 | ✔ / ✔ | |
| 110 | seed=111 | 47 | `--scenario R1 --seed 111 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 111 | seed=112 | 51 | `--scenario R1 --seed 112 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 112 | seed=113 | 53 | `--scenario R1 --seed 113 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 113 | seed=114 | 58 | `--scenario R1 --seed 114 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 114 | seed=115 | 45 | `--scenario R1 --seed 115 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 115 | seed=116 | 49 | `--scenario R1 --seed 116 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 116 | seed=117 | 55 | `--scenario R1 --seed 117 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 117 | seed=118 | 68 | `--scenario R1 --seed 118 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 118 | seed=119 | 50 | `--scenario R1 --seed 119 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 119 | seed=120 | 53 | `--scenario R1 --seed 120 --total 40.0 --bias 0.7 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |

## A9 = the nine A4 offset steps in ONE flight (replaces cards #91–99; 198 s plan, `--rotate 90`) — copy tag `ns2train_A9`

| # | setting | s | command (after `run_formation`) | flown | uSD cf_second / cf5 | notes |
|---|---|---:|---|---|---|---|
| A9 | 9 sections: dz 0.75 → 0.5 → 0.25, offset 0 / 0.1 / 0.15 m, 0.3 m/s line along y (2 passes each) | 198 | `--scenario A9 --rotate 90 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |

Sections (plan time): 1 dz 0.75 off 0 (0–17.5 s), 2 dz 0.75 off 0.1 (22.5–40), 3 dz 0.75 off 0.15 (45–62.5), 4 dz 0.5 off 0.15 (67.5–85), 5 dz 0.5 off 0.1 (90–107.5), 6 dz 0.5 off 0 (112.5–130), 7 dz 0.25 off 0 (135–152.5), 8 dz 0.25 off 0.1 (157.5–175), 9 dz 0.25 off 0.15 (180–197.5); 5 s rest-to-rest transitions in between (top moves vertically and/or the bottom drone sideways). dz only decreases, so every change increases the downwash on the lower drone.

## A7 descend-through (last, `--allow-extreme`, no `--height`) — copy tag `ns2train_A7`

| # | setting | s | command (after `run_formation`) | flown | uSD cf_second / cf5 | notes |
|---|---|---:|---|---|---|---|
| 61 | speed=0.1 | 29 | `--scenario A7 --speed 0.1 --auto-center --yes --allow-extreme` | skipped (user, 2026-10-17: same close-approach regime as R1, which already produced contact/near-collisions; dz below 0.25 is outside the study range) | – | |
| 62 | speed=0.2 | 16 | `--scenario A7 --speed 0.2 --auto-center --yes --allow-extreme` | skipped (user, 2026-10-17: same close-approach regime as R1, which already produced contact/near-collisions; dz below 0.25 is outside the study range) | – | |
| 63 | speed=0.3 | 12 | `--scenario A7 --speed 0.3 --auto-center --yes --allow-extreme` | skipped (user, 2026-10-17: same close-approach regime as R1, which already produced contact/near-collisions; dz below 0.25 is outside the study range) | – | |
| 64 | speed=0.4 | 10 | `--scenario A7 --speed 0.4 --auto-center --yes --allow-extreme` | skipped (user, 2026-10-17: same close-approach regime as R1, which already produced contact/near-collisions; dz below 0.25 is outside the study range) | – | |
| 65 | speed=0.5 | 8 | `--scenario A7 --speed 0.5 --auto-center --yes --allow-extreme` | skipped (user, 2026-10-17: same close-approach regime as R1, which already produced contact/near-collisions; dz below 0.25 is outside the study range) | – | |

## Optional: A2 fast, dz 0.25 / 0.75 — copy tag `ns2train_A2fast`

| # | setting | s | command (after `run_formation`) | flown | uSD cf_second / cf5 | notes |
|---|---|---:|---|---|---|---|
| 29 | dz=0.25 path=circle speed=0.4 | 15 | `--scenario A2 --dz 0.25 --path circle --speed 0.4 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 30 | dz=0.25 path=circle speed=0.5 | 12 | `--scenario A2 --dz 0.25 --path circle --speed 0.5 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 49 | dz=0.75 path=circle speed=0.4 | 15 | `--scenario A2 --dz 0.75 --path circle --speed 0.4 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |
| 50 | dz=0.75 path=circle speed=0.5 | 12 | `--scenario A2 --dz 0.75 --path circle --speed 0.5 --height 0.5 --auto-center --yes` | ☐ | ☐ / ☐ | |

## After the session
1. Merge + `ns2_bank_coverage.py` (gate time by lateral offset / real gap / speed), update `2026-10-16_ns2_collection_tracker.md` result blocks, write the session doc (date of the flight day).
2. Pilot retrain on the bank (`run_c2_lab_retrain.py --new-scenarios ALL`, v1 weights compared on the new flights only) after R1 seeds 101–110; decide there whether seeds 111–120 are worth flying.
3. If the gate passes: `export_weights_c.py` → `study_tree_v2` → bench → re-validate S2/S3 (A8, A1) → freeze protocol v1.0 → tier-1 study (2 drones).
