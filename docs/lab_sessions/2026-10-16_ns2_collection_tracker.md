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
| A8 swap | 0.25 | #21 ✔ (19-08-19, v 0.1; 19-06-19 aborted at 26 s, no cf_second log) | #22 ✔ (19-04-27) | #23 ✔ (19-02-56) | #24 ✔ (19-01-29) | #25 ✔ (18-59-19) |
| A8 swap | 0.75 | #41 ✔ (19-18-12, v 0.1) | #42 ✔ (19-21-36) | #43 ✔ (19-24-51; 19-23-34 aborted, no logs) | #44 ✔ (19-26-18) | #45 ✔ (19-28-05) |
| A2 circle | 0.5 | #6 ✔ (18-05-31, v 0.1; 18-02-54 outlier) | #7 ✔ (18-01-18) | #8 ✔ (17-58-53) | #9 ✔ (17-57-31) | #10 ✔ ×2 (18-24-22, 18-27-49; 18-26-13 outlier) |
| A2 circle | 0.25 | #26 ☐ | #27 ☐ | #28 ☐ | #29 ☐ | #30 ☐ |
| A2 circle | 0.75 | #46 ☐ | #47 ☐ | #48 ☐ | #49 ☐ | #50 ☐ |
| A3 static-top shuttle | 0.5 | #11 ✔ (18-36-13, v 0.1) | #12 ✔ (18-39-08) | #13 ✔ (18-42-13) | #14 ✔ (18-43-54) | #15 ✔ (18-57-51; 18-30-11 had no cf_second log) |
| A3 static-top shuttle | 0.25 | #31 ☐ | #32 ☐ | #33 ☐ | #34 ☐ | #35 ☐ |
| A3 static-top shuttle | 0.75 | #51 ☐ | #52 ☐ | #53 ☐ | #54 ☐ | #55 ☐ |
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
| 0.5 (×3) | 1.6 / 1.7 / 4.4 | 3.8–4.0 | 0.3–0.5 | 1.4–1.5 | −1.15…−1.34 / −3.4…−3.6 | 2.6–4.3 |
| 0.4 | 5.1 | 3.6 | 0.4 | 1.8 | −1.27 / −3.2 | 8.3 |
| 0.3 | 5.9 | 3.2 | 0.4 | 2.2 | −1.20 / −3.2 | 13.7 |
| 0.2 | 8.7 | 3.4 | 0.4 | 4.7 | −1.29 / −3.0 | 38.3 |
| 0.1 | 10.1 | 3.2 | 0.6 | 9.5 | −1.50 / −3.0 | 42.8 |
Outside the gate a_res_z mean is about −0.2. (Gate times corrected 2026-10-16 evening: merged files are 500 Hz, the first version divided by 100.) Slower crossings keep the bottom drone in the downwash longer, so the z error grows while the peak residual stays about the same.

## Result block 2 (A2 circle r 0.40, dz 0.5, S0) — merged 2026-10-16
5 flights merged, 4 usable (0.1, 0.2, 0.3, 0.4 m/s). Excluded: 18-02-54 (0.1 m/s): from t ≈ 33 s cf5 reads z = 1.00 m constant (error −50 cm, motor saturated 65535, a_res_z mean +3.8), i.e. lost/swapped tracking or collision with the top drone. Lost: 0.5 m/s (17-52-58: cf5 log only, no cf_second log; 17-55-42: no log on either card) — #10 needs a refly. Battery at start on cf5 was low (3.43–3.68 V vs 3.8+ in block 1).

| v (m/s) | bottom z median (m) / min | bottom z err RMS (cm) | bottom xy err RMS (cm) | top z err | a_res_z mean / min |
|---|---|---|---|---|---|
| 0.4 | 0.34 / 0.17 | 17.1 | 12.7 | ≈ 0.3 cm | −1.16 / −3.9 |
| 0.3 | 0.34 / 0.17 | 17.5 | 10.1 | | −1.21 / −3.8 |
| 0.2 | 0.31 / 0.18 | 19.7 | 8.7 | | −1.21 / −3.8 |
| 0.1 (18-05-31) | 0.28 / 0.15 | 22.1 | 8.7 | | −1.31 / −3.7 |
The merge flags the bottom drone (alignment RMS 19–24 cm vs the commanded path); the sag is real, not a misalignment: in a held stack the bottom drone sits 15–25 cm below 0.5 m and creeps up over 20–30 s.

## Result block 3 (A2 refly + A3 static-top shuttle, dz 0.5, S0) — merged 2026-10-16
19 flights in the bank in total (7 A8, 5 A2 + 2 usable A2 0.5 m/s refly, 4 A3 ... see table); excluded outliers: A2 18-02-54 and A2 18-26-13 (bottom drone reads z = 1.00 m, a_res_z +7.6, i.e. lost/merged tracking). Lost: A3 0.5 m/s (18-30-11: no cf_second log), A3 0.1 m/s 18-31-49 (empty cf5 file). tree_log_check: INDI off on all, NS2 off.

| Flight | v (m/s) | bottom z med / min (m) | z err RMS (cm) | in gate (s, % of flight) | a_res_z in gate / out | z err peak in gate (cm) |
|---|---|---|---|---|---|---|
| A2 18-24-22 | 0.5 | 0.40 / 0.17 | 13.2 | 10.8 (91 %) | −1.37 / −1.52 | 33 |
| A2 18-27-49 | 0.5 | 0.39 / 0.18 | 14.0 | 10.6 (90 %) | −1.14 / −1.28 | 32 |
| A3 18-43-54 | 0.4 | 0.54 / 0.13 | 10.6 | 4.6 (17 %) | −1.22 / −0.19 | 37 |
| A3 18-42-13 | 0.3 | 0.54 / 0.04 | 13.4 | 5.0 (14 %) | −1.19 / −0.19 | 46 |
| A3 18-39-08 | 0.2 | 0.52 / 0.11 | 11.9 | 9.7 (18 %) | −1.31 / −0.19 | 39 |
| A3 18-36-13 | 0.1 | 0.51 / 0.11 | 11.6 | 18.0 (17 %) | −1.22 / −0.20 | 39 |
A3: the bottom drone is pushed down to 4–13 cm above the floor under the top drone (watch for ground contact). A2 gate share is ~90 % (stacked), A3 ~15 %, A8 ~7–11 %.

## Result block 4 (A3 0.5 m/s refly + A8 dz 0.25, S0) — merged 2026-10-16
25 flights in the bank. Excluded: A2 18-02-54, A2 18-26-13 (bottom drone reads z = 1.00 m). Not mergeable: A8 dz 0.25 19-06-19 (0.1 m/s, aborted at 26 s of 89.5 s; cf_second file empty). Coverage so far: cards #1–15 and #21–25 complete (20 of 119); A5 (#16–20) deliberately skipped (counter-moving crossing, judged close to A8).

| A8 dz 0.25 | z err RMS (cm) | bottom z med / min (m) | in gate (s) | a_res_z in gate | z err peak in gate (cm) | gap top−bottom in gate: median / min (m) |
|---|---|---|---|---|---|---|
| 0.5 m/s | 3.5 | 0.51 / 0.40 | 1.5 | −1.22 | 3.9 | 0.25 / 0.21 |
| 0.4 | 4.3 | 0.51 / 0.36 | 1.7 | −1.36 | 9.1 | 0.25 / 0.21 |
| 0.3 | 5.2 | 0.51 / 0.31 | 2.6 | −1.14 | 17.8 | |
| 0.2 | 11.4 | 0.52 / 0.05 | 4.6 | −1.35 | 43.0 | 0.39 / 0.22 |
| 0.1 | 11.9 | 0.52 / 0.04 | 9.1 | −1.05 | 46.2 | 0.60 / 0.22 |
A3 0.5 m/s (18-57-51): z err RMS 6.5 cm, bottom z min 0.34 m, in gate 3.6 s, a_res_z in gate −0.69, z err peak 16.5 cm.
Observation: at 0.1–0.2 m/s the bottom drone is pushed to 4–7 cm above the floor in the gate (dz 0.5 and 0.25 alike), so the real gap in the gate is 0.4–0.76 m, not the nominal dz; label magnitude in the gate (≈ −1.2 m/s²) is about the same for dz 0.25 and 0.5 at equal speed.

## Result block 5 (A8 dz 0.75, S0) — merged 2026-10-16
30 flights in the bank. Not mergeable: A8 19-23-34 (0.3 m/s, aborted, no logs on either card). Cards done: #1–15, #21–25, #41–45 (25 of 119); A5 #16–20 deliberately skipped.

| v (m/s) | z err RMS (cm) | bottom z med / min (m) | in gate (s) | a_res_z in gate / out | z err peak in gate (cm) | gap in gate med / min (m) |
|---|---|---|---|---|---|---|
| 0.5 | 4.2 | 0.50 / 0.39 | 1.5 | −1.15 / −0.26 | 5.0 | 0.75 / 0.73 |
| 0.4 | 5.2 | 0.51 / 0.34 | 1.8 | −1.17 / −0.25 | 6.3 | 0.74 / 0.73 |
| 0.3 | 6.2 | 0.52 / 0.29 | 2.2 | −1.36 / −0.26 | 14.9 | 0.77 / 0.72 |
| 0.2 | 8.1 | 0.52 / 0.19 | 4.7 | −1.33 / −0.23 | 29.6 | 0.85 / 0.69 |
| 0.1 | 7.3 | 0.51 / 0.20 | 10.2 | −1.41 / −0.19 | 29.9 | 0.93 / 0.74 |
Top drone z error 0.2–0.6 cm RMS in all five. Observation: the gate-mean label (≈ −1.2…−1.4 m/s²) is nearly the same at dz 0.25, 0.5 and 0.75 for equal speed, while the z error peak does fall with dz (0.1 m/s: 46 / 43 / 30 cm). To be checked in the retrain: how much of the in-gate label is wash vs crossing manoeuvre.
