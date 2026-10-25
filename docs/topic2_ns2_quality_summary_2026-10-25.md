# Topic 2 — NS2 quality (supervisor feedback 2026-10-22): summary of results and closed work

State 2026-10-25: **closed** (desk 2026-10-22, lab 2026-10-23). Written as a self-contained summary for an independent check. Meeting doc with the plots: `docs/meetings/2026-10-29.md` §1–4.

## The question (supervisor, after the meeting doc of 2026-10-22)
- (a) NS2 is trained on the cases it is tested on.
- (b) Imitation data weighs every sample equally; "boring" near-zero rows can dominate and overfit.
- (c) Log the NN output and the INDI output on every strategy and compare them (same magnitude, NN maybe faster).

Plan agreed 2026-10-22: `docs/ns2_quality_plan_2026-10-22.md`.

## Facts checked first
- ns2_v2 is trained on all 73 flights; the pilot settings A8 dz 0.5 / 0.5 m/s, A1 dz 0.5 and A4 dz 0.5 offsets 0.10 / 0.15 are inside the training data (point a confirmed). A4 offset 0.05 was flown in the pilot only (2026-10-20, after v2 was frozen 10-17), not in training.
- 54 % of the 1.29 M training rows have |a_res| < 0.3 m/s² (point b: the data are indeed mostly "boring"); 86 % of these lie outside the gate. Training loss = Huber (SmoothL1, beta 0.2).
- The NN output (`rnn.pred_*`) was already logged on all strategies; the INDI estimate was zero under S0 / S2 (its filters ran only with INDI on).

## Results

**1. NN vs INDI (offline, 37 pilot flights, NN not applied; `docs/nn_vs_indi_check_2026-10-22.md`)**
| Cell (z, in gate) | flights | corr | label lag behind NN | slope label / NN |
|---|---|---|---|---|
| A8 S0 / S1 | 4 / 4 | 0.93 / 0.96 | 0 / 0 ms | 0.85 / 1.01 |
| A1 S0 | 6 | 0.93 | −10 ms | 1.17 |
| A1 S1 | 5 | 0.71 | −44 ms | 0.54 |
| A4 S0 / S1 | 9 / 9 | 0.92 / 0.88 | 0 / −8 ms | 0.98 / 1.02 |
- Label = INDI residual smoothed offline with a zero-delay 5 Hz filter. The NN follows it closely, in phase with the label and so 0–8 ms ahead of the onboard causal INDI estimate (which is noisier); weakest in A1 under INDI. Rebuilt vs onboard-logged INDI estimate: RMS 0.03 m/s², corr 0.996. The NN predicts z only.

**2. Training on the test cases and boring rows (offline, 3 seeds each; `docs/ns2_heldout_rebalance_2026-10-22.md`)**
| In-gate RMSE [m/s²] | all | A1 | A4 | A8 |
|---|---|---|---|---|
| v2 recipe (pilot settings in training) | 0.435 ± 0.017 | 0.414 | 0.446 | 0.635 |
| pilot settings held out | 0.464 ± 0.018 | 0.426 | 0.545 | 0.619 |
| all data, boring weight 0.05 | 0.426 ± 0.014 | 0.415 | 0.403 | 0.630 |
| held out, boring weight 0.05 | 0.479 ± 0.034 | 0.442 | 0.553 | 0.628 |
- Holding the pilot settings out costs +3 % (A1), 0 % (A8), +22 % (A4, whole A4 / A9 family removed = pessimistic case).
- Down-weighting boring rows: pooled at most 2 % gain, within the seed spread; by scenario A4 about 10 % better (3/3 seeds), A1 worse at unseen settings → no uniform gain, no v3 retrain. The v2 recipe is reproducible (seed 0 gives exactly v2); the released v2 is one seed (0.453 vs 3-seed mean 0.435, spread ±0.02–0.03). `train.py --boring-weight` added (default unchanged).

**3. Logging of the INDI estimate on every strategy (firmware)**
- Patch `flying_drone_stack/firmware_patches/indi_estimate_always_logged.patch`: the estimate runs whenever the RPM deck is present, applied to the control only with INDI on. Replay byte-identical 4 / 4; estimate under `indi 0` equals the `indi 3` estimate exactly.
- Image `study_tree_v2_fbl_estlog_candidate.bin` (sha256 4fa0a130…) flashed on both drones 2026-10-23 and validated in flight: onboard log = offline rebuild, corr 0.96–1.00 in all four strategies.

**4. Flight confirmation at unseen settings (lab 2026-10-23; `docs/lab_sessions/2026-10-23.md`)**
- 40 flights with complete uSD logs, S0–S3, settings between the trained ones: A8 dz 0.4 at 0.35 m/s, A1 dz 0.45, A4 offsets 0.075 / 0.125; ns2_v2 unchanged.
- Pre-registered rule (fixed before the flights, user decision 2026-10-22): retention R = NS2 gain unseen / NS2 gain seen, geometric pair (S0 → S2), pooled; R ≥ 0.7 no material overfit, 0.4–0.7 partial, < 0.4 confirmed.

| Cell (unseen) | metric | S0 | S1 | S2 | S3 | n |
|---|---|---|---|---|---|---|
| A1 dz 0.45 | mean z error [cm] | −19.7 ± 0.9 | −0.9 ± 1.1 | −7.3 ± 0.2 | 2.5 ± 2.0 | 3 |
| A4 offset 0.075 | z RMS [cm] | 17.4 ± 0.5 | 6.0 ± 1.1 | 11.3 ± 1.1 | 2.9 ± 0.5 | 2 |
| A4 offset 0.125 | z RMS [cm] | 9.2 ± 0.4 | 2.2 ± 0.2 | 4.1 ± 0.8 | 2.0 ± 0.2 | 2 |
| A8 dz 0.4 | crossing dip [cm] | −17.0 ± 0.3 | −13.2 ± 5.5 | −7.9 ± 0.6 | −5.7 ± 2.2 | 3 |

| S0 → S2 (geometric) | seen gain | unseen gain | R |
|---|---|---|---|
| A1 | 50 % | 63 % | 1.27 |
| A4 0.075 | 40 % | 35 % | 0.89 |
| A4 0.125 | 56 % | 55 % | 0.98 |
| A8 | 55 % | 54 % | 0.97 |
| **pooled** | | | **1.03 → no material overfit** (95 % CI 0.93–1.17; leave one flight out 0.99–1.06) |

- A4 0.075: its seen reference interpolates the pilot offsets 0.05 (not trained) and 0.10; with trained offsets only, R = 0.68 (pooled 0.98).
- INDI pair (S1 → S3): NS2 helps in A8 (54 → 57 %) and A4 0.075 (10 → 51 %), not in A1 (S1 already at −0.9 cm) and A4 0.125. The INDI bases are negative or near zero in 2 of 4 cells, so INDI retention cannot be read either way; A8 seen 54 % excludes 2 drained S3 flights (28 % with them). The verdict rests on the geometric pair.
- NN vs INDI at the unseen settings (`experiments/results/unseen_nn_vs_indi_2026-10-23/summary_z_5Hz.csv`, label = offline-rebuilt INDI estimate, equal to the onboard log, smoothed zero-delay at 5 Hz):

| Cell (z, in gate) | flights | corr | lag [ms] | slope | mean NN / label [m/s²] |
|---|---|---|---|---|---|
| A1 S0 | 3 | 0.95 | -4 | 1.20 | -1.30 / -1.66 |
| A1 S1 | 3 | 0.84 | -32 | 0.86 | -2.18 / -2.79 |
| A4 S0 | 4 | 0.93 | -6 | 1.05 | -1.11 / -1.77 |
| A4 S1 | 4 | 0.68 | 2 | 0.70 | -0.27 / -0.70 |
| A8 S0 | 3 | 0.94 | 0 | 0.90 | -1.15 / -1.61 |
| A8 S1 | 3 | 0.95 | 0 | 0.99 | -1.16 / -1.53 |

- Under S0 and in A8 the agreement is high (corr 0.93–0.95). Weaker under INDI in A1 (0.84, −32 ms) and A4 (0.68). The NN's mean level is below the label in most cells (e.g. A1 S0 −1.30 vs −1.66, A4 S0 −1.11 vs −1.77 m/s²). **Mostly a label bias of the new bottom drone:** far from the neighbour the label is still −0.25 to −0.47 m/s² (NN −0.1 to −0.17); after removing it NN / label = 0.93–1.10 (A4, A8; A1 is always in the gate and cannot be split). Offline, the v2 in-gate RMSE is 0.453 seen vs 0.464 unseen (+2 %).

## Decisions taken
- ns2_v2 stays for the study (no v3).
- Study image = the FBL-containing logging image (option A, 2026-10-22); protocol v1.0 must name its hash.
- Yaml per strategy on crazyswarm2 branches `study_S1`, `study_S2`, `study_S3` (main = S0).

## Limits (stated in the meeting doc)
- Unseen = between trained settings (interpolation), not extrapolation; A1 dz 0.45 is 0.05 m from a trained dz.
- 2–3 flights per cell; the bootstrap CI covers flight-to-flight spread only, not session effects; block order S1, S3, S2, S0 not randomised (S1 A4 packs at 3.2–3.3 V); the seen reference is from another session (2026-10-19/20).
- Offline results use the IMU-based INDI residual as label (noise floor ≈ 0.115 m/s², plus the new-vehicle bias above), one architecture; hard subsampling and coverage-based weighting not tested.
- Open (desk): the label bias on the new bottom drone (k_t check from solo hover); in A1 S0 the z-integral term is not visible in the logged thrust (9/9 flights, seen and unseen; same on both sides, so R is not biased).

Independent review: `docs/topic2_ns2_quality_independent_review.md`.

## Still waiting for (supervisor, not ours)
- Is the unseen-settings confirmation sufficient, or a network trained without the pilot settings flown on them (strict version, not planned)?
- Keep ns2_v2 for the study (our recommendation: yes)?
- A4: re-fly S2 / S3 on offsets / speeds outside the training data?

## How to reproduce (for a check)
```
# offline NN vs INDI (system python3 with the firmware bindings for the raw part)
PYTHONPATH=~/Desktop/crazyflie-firmware/build python3 experiments/analysis/nn_vs_indi_raw.py <bank folders>
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/nn_vs_indi_check.py --banks <bank folders> --out <dir>
# held-out / boring-row study (trains; --seed)
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/ns2_heldout_pilot_study.py --out-dir <dir> --seed 0   # seeds 0 / 1 / 2; see docs/ns2_heldout_rebalance_2026-10-22.md
# unseen-settings block (pre-registered rule)
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/unseen_block_analysis.py --banks bank_unseen_S0_2026-10-23 bank_unseen_S1_2026-10-23 bank_unseen_S3_2026-10-23 bank_unseen_S3S2_2026-10-23 --out <dir>
```
Results in the repo: `experiments/results/nn_vs_indi_2026-10-22/`, `ns2_heldout_2026-10-22/`, `unseen_block_2026-10-23/` (`verdict.json`, `cells.csv`, `per_flight.csv`, `retention.csv`), `unseen_nn_vs_indi_2026-10-23/`. The merged `bank_*` folders are local only (6 GB, not in git); the raw uSD files they are built from are in `experiments/logs/usd_raw/`.
