# Unseen-settings block — flight confirmation of the NS2 results (plan and pre-registered pass rule, 2026-10-22)

Why: supervisor feedback 2026-10-22 (NS2 trained on the cases it is tested on). Offline answer (`docs/ns2_heldout_rebalance_2026-10-22.md`): small effect for A1/A8, clear for A4. This block is the flight confirmation, **cheap version only** (decided by the user 2026-10-22; the strict version, a network trained without the pilot settings flown on them, is not planned). Network stays **ns2_v2**, nothing is retrained.

## What "unseen" means here
Settings between the trained values, not extrapolation. Trained: A8 dz 0.25/0.5/0.75 × speed 0.1–0.5 m/s; A1 dz 0.2, 0.25, 0.3, 0.375, 0.5, 0.625, 0.75; A4 dz 0.3/0.5 (+ A9 dz 0.25/0.5/0.75), offsets 0–0.15 m, speed 0.3–0.5 m/s.
| Cell | Setting (unseen) | Nearest trained | Note |
|---|---|---|---|
| A8 swap | dz 0.4, peak 0.35 m/s (duration 6.25 s) | dz 0.5 / 0.25, 0.3 and 0.4 m/s | unseen in both dz and speed |
| A1 hold | dz 0.45, 15 s | dz 0.5 (3 flights, 0.05 away) and 0.375 | **weakest cell**: close to a trained dz |
| A4 line | dz 0.4, 0.35 m/s, offset **0.075** | offsets 0.05 not trained, 0.1 trained; dz 0.3/0.5 | unseen in dz, speed, offset |
| A4 line | dz 0.4, 0.35 m/s, offset **0.125** | offsets 0.1 / 0.15 | same |

## Flights (S0, S1, S2, S3; uSD logs of both drones for every flight)
| Cell | per strategy | total |
|---|---|---|
| A8 dz 0.4, 0.35 m/s | 3 | 12 |
| A1 dz 0.45 | 3 | 12 |
| A4 offset 0.075 | 2 | 8 |
| A4 offset 0.125 | 2 | 8 |
| **all** | **10** | **40 (about 3.5 h at the pilot pace of ~5 min per flight)** |

## Pass rule (written before flying)
- Metrics as in the meeting doc: A8 crossing dip (mean of per-crossing minima; window ±0.75 s scaled with the duration), A1 mean z error (2–15 s), A4 z RMS.
- Relative improvement of NS2 over its base: **RI = (|S0| − |S2|) / |S0|** (primary, geometric, clean NS2 effect) and (|S1| − |S3|) / |S1| (secondary, INDI; negative in the pilot for A1 and A4 0.10/0.15, so only reported). Pilot reference = meeting-doc numbers (A4 interpolated linearly in the offset).
- **Retention R = RI_unseen / RI_pilot**, per cell and pooled (mean over the cells): **R ≥ 0.7 no material overfit; 0.4 ≤ R < 0.7 partial; R < 0.4 overfit confirmed.** Also reported: ranking S0 > S2 and S1 vs S3 unchanged or not, n and std per cell. (Thresholds are the proposal of 2026-10-22, change them before the first flight, not after.)
- Caveat: the unseen cells also differ from the pilot cells in difficulty (dz, speed), so RI is relative and the pilot reference is only the nearest available; the same four strategies are flown in the same cell, which is what the comparison rests on.
- Counts: only flights with a complete uSD file on both cards, strategy read from the log (`ns2_applied_check.py` via the analysis script), no new abort or exclusion rules.

## Lab procedure
0. Cards: count `thesis` files on both (≥ 85 → archive), fresh batteries (note resting voltage before each flight, INDI strategies: fresh pack per flight), `git pull` on both repos.
1. **Flash the logging image on both drones:** `flying_drone_stack/firmware_app/build_artifacts/study_tree_v2_fbl_estlog_candidate.bin` (sha256 `4fa0a130c698f217728d8b5dcf7464cb84a0b5bd4196b3ac4123a8100bad012b`, 580 908 B; v2 weights, INDI estimate logged under every strategy, FBL inside but inert at `ctrlFbl.en 0`; control byte-identical to `study_tree_v2` in the replay gates). **Open: whether the study flights should use this FBL-containing image (default here) or a separate image without FBL (not built).** Verify the hash before flashing, power-cycle, `ns2c_bench_check.py --readback` on both (`rnn.ready 1`, `rnn.div 10`).
2. **Bench/hover check of the logging:** one S0 flight (or hover): the log must show non-zero `ctrlOmarIndi.a_rpm_f*` and `a_imu_f*` with `indi 0`, and the flight must look like S0 (`tree_log_check.py`).
3. **Blocks, order S1 → S3 → S0 → S2** (INDI first on the freshest packs; differs from both earlier orders), one yaml state per block (`S0`: `indi 0 / rnn.en 0`, `S1`: `3 / 0`, `S2`: `0 / 1`, `S3`: `3 / 1`; `lab_check_strategy.py` before the launch). Inside a block: A4 0.075 ×2, A4 0.125 ×2, A8 ×3, A1 ×3.
4. After each block: copy the cards (cf_second first) with the tag `unseen_<block>`, `merge_training_bank.py --date <day> --only-tag-contains unseen`, check that every flight has a file on both cards.

## Commands (validated with `run_formation --check`, "checks passed", 2026-10-22; trajectories regenerate at flight time)
All start with `ros2 run crazyflie_examples run_formation` and end with `--auto-center --yes`:
- A8: `--scenario A8 --dz 0.4 --speed 0.35 --height 0.5 --passes 4`
- A1: `--scenario A1 --dz 0.45 --hold 15.0 --height 0.5`
- A4: `--scenario A4 --dz 0.4 --speed 0.35 --offset <0.075|0.125> --motion line --rotate 90 --height 0.5`

## Analysis (ready, dry-run on the pilot flights reproduces the meeting-doc numbers with retention 1.0)
```
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/unseen_block_analysis.py --banks <bank folders> --out experiments/results/unseen_block_<date>
PYTHONPATH=<crazyflie-firmware>/build python3 experiments/analysis/nn_vs_indi_raw.py <bank folders>
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/nn_vs_indi_check.py --banks <bank folders> --out experiments/results/unseen_nn_vs_indi_<date>
```
Outputs: per-flight and per-cell tables, retention table, verdict, figure; NN-vs-INDI quality at the unseen settings (S0/S1), and with the onboard INDI log also for S2/S3.
