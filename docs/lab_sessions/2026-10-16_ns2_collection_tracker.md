# 2026-10-16 NS2 training-data collection — tracker (card: `docs/ns2_training_flight_card.md`, 119 flights, all S0)

Tick a cell when the flight is clean AND both uSD files are copied + checked. Cell = card number. `x` = flight lost / refly.

## Setup (once)
```bash
cd ~/Desktop/flying_robot_course && git pull && cd ~/Desktop/crazyswarm2 && git pull     # both repos, CS2 stopped
cd ~/Desktop/crazyswarm2 && git checkout crazyflie/config/crazyflies.yaml && git apply crazyflie/config/study/tree_S0.patch
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/ns2c_bench_check.py radio://0/80/2M/E7E7E7BB02 --readback   # expect controller 9 after power-cycle, indi 0, rnn.en 0
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/ns2c_bench_check.py radio://0/80/2M/E7E7E7E7E9 --readback
ros2 launch crazyflie launch.py
```

No reflash: `study_tree_v1.bin` stays on both drones (S0 does not use NS2).

## Speed × dz (peak speed m/s)

| Scenario | dz | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 |
|---|---|---|---|---|---|---|
| A8 swap | 0.5 | #1 ✔ (17-38-05, v 0.1) | #2 ✔ (17-34-59, v 0.2) | #3 ✔ (17-32-27, v 0.3) | #4 ✔ (17-29-10, v 0.4) | #5 ✔ ×3 (17-22-49, 17-24-15, 17-27-40; v 0.5) |
| A8 swap | 0.25 | #21 ☐ | #22 ☐ | #23 ☐ | #24 ☐ | #25 ☐ |
| A8 swap | 0.75 | #41 ☐ | #42 ☐ | #43 ☐ | #44 ☐ | #45 ☐ |
| A2 circle | 0.5 | #6 ☐ | #7 ☐ | #8 ☐ | #9 ☐ | #10 ☐ |
| A2 circle | 0.25 | #26 ☐ | #27 ☐ | #28 ☐ | #29 ☐ | #30 ☐ |
| A2 circle | 0.75 | #46 ☐ | #47 ☐ | #48 ☐ | #49 ☐ | #50 ☐ |
| A3 lemniscate | 0.5 | #11 ☐ | #12 ☐ | #13 ☐ | #14 ☐ | #15 ☐ |
| A3 lemniscate | 0.25 | #31 ☐ | #32 ☐ | #33 ☐ | #34 ☐ | #35 ☐ |
| A3 lemniscate | 0.75 | #51 ☐ | #52 ☐ | #53 ☐ | #54 ☐ | #55 ☐ |
| A5 reverse circle | 0.5 | #16 ☐ | #17 ☐ | #18 ☐ | #19 ☐ | #20 ☐ |
| A5 reverse circle | 0.25 | #36 ☐ | #37 ☐ | #38 ☐ | #39 ☐ | #40 ☐ |
| A5 reverse circle | 0.75 | #56 ☐ | #57 ☐ | #58 ☐ | #59 ☐ | #60 ☐ |
| A7 descend-through | – | #61 ☐ | #62 ☐ | #63 ☐ | #64 ☐ | #65 ☐ |

## A1 stack hold (15 s), 2 repeats each

| dz | rep 1 | rep 2 |
|---|---|---|
| 0.5 | #66 ☐ | #67 ☐ |
| 0.25 | #68 ☐ | #69 ☐ |
| 0.75 | #70 ☐ | #71 ☐ |
| 0.375 | #72 ☐ | #73 ☐ |
| 0.625 | #74 ☐ | #75 ☐ |

## A4 offset stack (line)

| dz | speed 0.1 | 0.2 | 0.3 | 0.4 | 0.5 | offset 0.0 @0.3 | offset 0.10 @0.3 | offset 0.15 @0.3 |
|---|---|---|---|---|---|---|---|---|
| 0.5 | #76 ☐ | #77 ☐ | #78 ☐ | #79 ☐ | #80 ☐ | #91 ☐ | #92 ☐ | #93 ☐ |
| 0.25 | #81 ☐ | #82 ☐ | #83 ☐ | #84 ☐ | #85 ☐ | #94 ☐ | #95 ☐ | #96 ☐ |
| 0.75 | #86 ☐ | #87 ☐ | #88 ☐ | #89 ☐ | #90 ☐ | #97 ☐ | #98 ☐ | #99 ☐ |

## R1 random waypoints (seeds 101–120; first one with a hand on the abort)

| s101 | s102 | s103 | s104 | s105 | s106 | s107 | s108 | s109 | s110 | s111 | s112 | s113 | s114 | s115 | s116 | s117 | s118 | s119 | s120 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| #100 ☐ | #101 ☐ | #102 ☐ | #103 ☐ | #104 ☐ | #105 ☐ | #106 ☐ | #107 ☐ | #108 ☐ | #109 ☐ | #110 ☐ | #111 ☐ | #112 ☐ | #113 ☐ | #114 ☐ | #115 ☐ | #116 ☐ | #117 ☐ | #118 ☐ | #119 ☐ |

## Per flight / per block
```bash
# commands: copy from docs/ns2_training_flight_card.md (row = card number); no --study, no --session
# after each block (CS2 stopped is NOT needed for the copy; drones landed):
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/copy_usd_log.py /media/georg/<cf_second card> cf_second --file thesisNN --tag ns2train_<block>   # cf_second first
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/copy_usd_log.py /media/georg/<cf5 card> cf5 --file thesisNN --tag ns2train_<block>
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/check_usd_log_quality.py experiments/logs/usd_raw/*ns2train*
python3 experiments/analysis/tree_log_check.py experiments/logs/usd_raw/cf5_ns2train_*.bin     # must read INDI off, NS2 off
```
Pull the radio logs/meta back into the repo as usual (`git pull`), then later: `python3 experiments/analysis/merge_training_bank.py --date 2026-10-16 --out experiments/logs/bank_ns2train_2026-10-16 --only-tag-contains ns2train`.

Order: top to bottom of the speed×dz table (dz 0.5 first), then A7, A1, A4, R1.

## Result block 1 (A8, dz 0.5, S0) — merged 2026-10-16
7 flights merged (bank `experiments/logs/bank_ns2train_2026-10-16`, not committed, regenerate with `merge_training_bank.py --date 2026-10-16 --only-tag-contains ns2train`). Both cards read INDI off / NS2 off. Two 0.5 m/s takes (17-16-44, 17-18-59) have no uSD log (cf_second card: empty file; cf5 card: 17-18-59 empty) and are skipped.

| v (m/s) | bottom z RMS (cm) | bottom xy RMS (cm) | top z RMS (cm) | in gate (s) | a_res_z in gate mean / min | z error peak in gate (cm) |
|---|---|---|---|---|---|---|
| 0.5 (×3) | 1.6 / 1.7 / 4.4 | 3.8–4.0 | 0.3–0.5 | 7.0–7.4 | −1.15…−1.34 / −3.4…−3.6 | 2.6–4.3 |
| 0.4 | 5.1 | 3.6 | 0.4 | 8.9 | −1.27 / −3.2 | 8.3 |
| 0.3 | 5.9 | 3.2 | 0.4 | 11.1 | −1.20 / −3.2 | 13.7 |
| 0.2 | 8.7 | 3.4 | 0.4 | 23.3 | −1.29 / −3.0 | 38.3 |
| 0.1 | 10.1 | 3.2 | 0.6 | 47.3 | −1.50 / −3.0 | 42.8 |
Outside the gate a_res_z mean is about −0.2. Slower crossings keep the bottom drone in the downwash longer, so the z error grows while the peak residual stays about the same.
