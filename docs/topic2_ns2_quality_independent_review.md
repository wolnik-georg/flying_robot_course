# Topic 2 (NS2 quality): independent review (2026-10-25)

Scope: `docs/topic2_ns2_quality_summary_2026-10-25.md`, meeting doc `docs/meetings/2026-10-29.md` §1–4 (with the overview), and the code and data behind them. Everything below was recomputed from the uSD logs and the stored networks. Nothing was retrained, flown or edited. Confidence levels: **V** = verified (recomputed), **S** = strong evidence, **H** = hypothesis, **U** = unknown or not checkable.

## 1. Claim check

| # | Claim (where) | Result | Evidence | Conf. |
|---|---|---|---|---|
| 1 | Pilot settings are in the ns2_v2 training set: A8 dz 0.5 / 0.5 m/s, A1 dz 0.5, A4 dz 0.5 at offsets 0.05–0.15 (summary l.13, `ns2_quality_plan` Findings) | **Corrected** | A8 dz 0.5 at duration 4.375: 3 training flights. A1 dz 0.5: 3. A4 offset 0.10: yes, through A9 at 0.5 m/s and the A4 line at 0.3 m/s. Offset 0.15: yes, through A9. **Offset 0.05 is not a trained setting.** It was flown only in the pilot test block of 2026-10-20 (where it replaced 0.0), i.e. after ns2_v2 was frozen on 10-17. Every A4 meta file up to 10-17 has offset 0.1 / 0.15 / 0.2 / 0.0, and `A9_SEQUENCE` has used 0 / 0.10 / 0.15 since it was created (`e048019`). The NN *inputs* do include lateral offsets around 0.05 m as transients inside the A4/A9 passes (thousands of rows per flight). So 0.05 lies inside the training input range but was never a commanded training setting | V |
| 2 | 54 % of the 1.29 M training rows are "boring" (\|a_res\| < 0.3) | Confirmed | Train logs give 51–55 % of the training split; the all-data run gives 54.0 % (`ns2_heldout_2026-10-22/*.train.log`). Side correction: the loss is **Huber** (`SmoothL1Loss(beta=0.2)`, `train.py:121`), not "plain MSE" as the plan's Findings table says | V |
| 3 | The NN output is logged on all strategies. The INDI estimate was zero under S0 / S2 on the old image | Confirmed | On the pilot S0/S2 flights, the logged estimate differs from the rebuild by RMS 1.1 m/s² (it is logged as zeros). `rnn.pred_z` is non-zero in S0/S1 | V |
| 4 | Table 1, offline NN vs INDI (corr / lag / slope per cell) | Confirmed | `experiments/results/nn_vs_indi_2026-10-22/summary_z_5Hz.csv`. The values are medians over flights of the correlation at the best lag. 37 flights (the docstring's "43" is stale) | V |
| 5 | The NN is in phase, "not faster". The onboard causal estimate lags by 0–8 ms | Confirmed, reword | The NN is in phase with the zero-delay label, so it is **0–8 ms ahead of the onboard causal 30 Hz estimate**. That is the sense in which the supervisor expected "faster", and it holds (small, because the 30 Hz filter delay is only ≈ 7.5 ms) | V |
| 6 | Rebuilt vs onboard INDI estimate: RMS 0.03, corr 0.996 (39 INDI flights) | Confirmed | Median RMS 0.033, median corr 0.9964, n = 39 (`review_topic2_nn_offset.py`) | V |
| 7 | The NN predicts z only | Confirmed | `rnn.pred_x/y` are identically 0 | V |
| 8 | Table 2 (held out / boring weight, in-gate RMSE) | Confirmed, with a caveat | Numbers match `results.json`. "v2 recipe 0.435 ± 0.017" is the 3-seed mean. **The released v2 (= seed 0, weights bit-identical) scores 0.453, the worst of the three seeds.** Compared with the released v2, holding out costs +2.4 % (0.453 → 0.464) | V |
| 9 | Holding out costs +3 % (A1), 0 % (A8), +22 % (A4) | Confirmed, with context | Leakage-free at the flight level: 14 flights removed, both ego drones, including all 3 A9 flights. It is an **exact-setting** hold-out: A8 dz 0.5 at 0.4 m/s and A1 dz 0.375 / 0.625 stay in training. The pooled "all" column is 74 % A1 rows; A8 is only 5 % (5 823 of 112 271 in-gate rows) | V |
| 10 | Down-weighting boring rows gains "at most 2 %, within the seed spread" → no v3 | Partly | Pooled: true (seed-paired −0.039 / +0.021 / −0.007). By scenario: **A4 −10 % in 3/3 seeds**, on both seen and unseen flights. At unseen settings: A8 −9 %, **A1 +17 % (worse)**. "No v3" is defensible because there is no uniform gain, but the wording hides the scenario dependence | V |
| 11 | Seed 0 reproduces v2 exactly | Confirmed | Weight arrays are identical and all metrics are equal | V |
| 12 | Logging patch: replay byte-identical 4/4 | Confirmed (re-run today) | `test_omar_ns2_off_identical.py --old-dir build_pre_estlog_2026-10-22`: 4/4 PASS against the current build. It covers only `rnn.en 0` with indi 0/3 (the S0/S1 code path) | V |
| 13 | The estimate under `indi 0` equals the `indi 3` estimate exactly | Not re-run | In flight: the S0/S2 onboard log vs the indi-3 rebuild has corr ≥ 0.997 | S |
| 14 | Image flashed and validated in flight, corr 0.96–1.00 in all four strategies | Confirmed | Min corr 0.964 (S1), medians 0.997–0.999. RMS difference 0.02–0.08 m/s²; the lab doc's "0.04–0.05" understates the maximum of 0.076. The flash itself and the hash rest on the lab record only (U) | V |
| 15 | 40 flights, S0–S3, planned counts, strategy read from the log | Confirmed | The strategy read from every cf5 log matches its bank. NS2 coefficient −1.00…−1.06 when applied vs −0.20…+0.01 when not. The INDI flag is unambiguous. Counts are 3 / 2 / 2 / 3 per strategy and cell. No flight is double-counted or of the wrong strategy (`review_topic2_strategy.py`) | V |
| 16 | The rule was computed as pre-registered | Confirmed | `unseen_block_analysis.py` is unchanged since `2576025e` (10-22, before the flights). The thresholds are unchanged: `7298e374` only marks them as confirmed, before the flights. The re-run reproduces `per_flight.csv` byte for byte, pooled 1.0295. Deviations: block order flown S1, S3, S2, S0 instead of the planned S1, S3, S0, S2 (disclosed). `ns2_applied_check.py` was changed in the lab (`1bc546c7`) because the new image logs the estimate; the margins above show no ambiguity | V |
| 17 | Cell table (S0–S3 per unseen cell) | Confirmed | Reproduced to the last digit | V |
| 18 | Geometric retention A1 1.27, A4 0.075 0.89, A4 0.125 0.98, A8 0.97, pooled 1.03 | Confirmed, with a caveat | The seen reference for A4 0.075 interpolates the pilot offsets 0.05 (**not trained**) and 0.10, so it is half unseen. With trained offsets only as the reference: A4 0.075 = **0.68**, pooled 0.98 | V |
| 19 | INDI pair: NS2 helps in A8 (54 → 57 %) and A4 0.075 (10 → 51 %). "No sign of memorisation on either side" (meeting §4) | Numbers confirmed; claim **overstated** | The A8 seen 54 % depends on excluding 2 drained S3 flights (`unified_overview.py` DRAIN); with them it is 28 %. The INDI-side bases are negative or near zero in 2 of 4 cells, so INDI retention cannot be read either way. Only the geometric side carries the claim | V |
| 20 | Unseen NN-vs-INDI table, "label = onboard INDI estimate smoothed" | Confirmed, reword | The label is the offline rebuilt raw estimate (zero-delay 5 Hz), not the onboard log; the two agree (#14). Meeting §4 "corr 0.93–0.97 under S0": **0.93–0.95 in the gate**; 0.97 is A4 over all samples | V |
| 21 | "The NN's mean level is below the label" (summary l.73) | Confirmed, **reinterpreted** | See §3.2. Most of the offset is a bias of the label on the new bottom drone (−0.25 to −0.47 m/s² even far from the neighbour), not downwash the NN misses | S |
| 22 | Limits list (interpolation, n 2–3, order, other session) | Confirmed, apt | — | V |
| M1 | Meeting overview: "same magnitude" | **Corrected** | Raw in-gate NN/label mean ratio is 0.70–0.80. After removing the out-of-gate label bias it is 0.93–1.10 (§3.2) | S |
| M2 | Meeting overview and §3: "built, not flashed yet", "Not yet confirmed in flight: all results are offline prediction errors" | **Stale** | Contradicts §4 and the Next Steps (flashed 10-23, 40 flights) | V |
| M3 | Meeting §1: A1 S1 "NN level 13 % low" | Confirmed | −2.41 / −2.77 = 0.87 | V |

## 2. Were (a), (b), (c) and the goal addressed?

| Point | Answer | Why |
|---|---|---|
| (a) trained on the test cases | **Yes, for interpolated settings** | Closed loop: R 1.03, bootstrap 95 % CI [0.93, 1.17]. Offline, independently: the released v2 in-gate RMSE goes 0.453 on seen settings → **0.464 on unseen** (A1 0.438 → 0.422, A4 0.445 → 0.496, A8 0.648 → 0.628). Not covered: extrapolation and the strict version (a held-out network flown). The pilot S2/S3 numbers themselves were measured on trained settings, so the study protocol must declare that or use cells off the trained grid |
| (b) boring data | **Partly** | The fact is confirmed. Only re-weighting was tested (0.2 / 0.05), with a scenario-dependent effect (#10). Hard subsampling, coverage weighting and the threshold were not tested. Two points that support "not a problem" are missing from the docs: 86 % of the boring rows lie outside the gate (derived from the plan's own numbers: 54 % × 46 % split, 86 % of out-of-gate rows boring) (neighbour far, below, or top drone as ego), i.e. a different input region; and after removing the label bias there is no shrinkage toward zero (NN/label 0.93–1.10). Also relevant: Huber loss, not MSE |
| (c) NN vs INDI on every strategy | **Yes (logging), mostly (comparison)** | Logging was done and validated in flight. Under S0: corr 0.93–0.95, in phase, 0–8 ms ahead of the causal estimate, same magnitude once the vehicle bias is removed. Weaker under INDI: A1 S1 corr 0.71 / 0.84 with the NN 32–44 ms behind and slope 0.54–0.86; A4 S1 corr 0.68 in gate at unseen settings (small signal: label RMS 0.75) |
| Goal: is the NN trained well / is the dataset good? | **Partly yes** | It generalises to interpolated settings, with no degradation offline or in closed loop. Dataset gaps: no A4 offset 0.05; the label is collected on a different (bent) bottom drone, and the new drone's label has a −0.25…−0.47 m/s² bias that the dataset cannot know about; the quality under INDI is clearly lower than under S0 |

## 3. Errors and weak points

1. **A4 offset 0.05 was flown in the pilot (10-20), not in training** (claim #1; the pilot dropped 0.0 for 0.05 after v2 was frozen on 10-17). The A4 0.075 "seen" reference is therefore half a commanded-unseen setting, even though 0.05 offsets occur as transients in the training passes. With trained offsets only as the reference, R is 0.68 (#18), shown as a sensitivity only. The held-out "A4 +22 %" also partly scores a setting that was never commanded in training.
2. **The NN mean-level offset is mostly a label bias.** Out of the gate (neighbour |dx| or |dy| > 0.4 m or below, t > 3 s) the label is −0.26 / −0.37 m/s² (A8 / A4, pilot) and −0.44 / −0.47 (unseen 10-23); the NN predicts −0.09 to −0.17 there.

   | In gate, S0 / S1 | NN / label raw | after subtracting the out-of-gate label bias |
   |---|---|---|
   | seen (19 / 18 flights) | 0.80 / 0.77 | 0.93 / 0.99 |
   | unseen (10 / 10 flights) | 0.70 / 0.74 | 0.94 / 1.10 |

   A1 cannot be split this way, because the bottom drone is in the gate all the time; its offset of 0.16–0.60 has the size of that bias. Cause **H**: the rotor-speed model (k_t) does not fit the new bottom drone, and/or pack or propeller state; the bias grew from 10-19/20 to 10-23. Consequences:
   - "Same magnitude" holds for the downwash part only.
   - Every network's RMSE contains a common offset floor, so the relative differences quoted (+3 %, ≤ 2 %) are diluted.
   - In closed loop the z integral absorbs the bias.
3. **The pooled offline metric is A1-dominated** (74 % of in-gate rows; A8 5 %). The pooled "all" figures say little about A8.
4. **One seed of v2.** The released v2 is the worst of 3 seeds on the pilot flights (0.453 vs 0.420 / 0.431) and the best on the unseen flights (0.464 vs ~0.49–0.51 for seeds 1–2). The seed spread (±0.02–0.03) is the size of the effects being argued.
5. **INDI side.**
   - The A8 seen gain depends on the drained-flight exclusion (54 % vs 28 %).
   - The S1 block flew on low packs: A4 S1 voltage at takeoff was 3.19–3.30 V, vs 3.33–3.83 V in every other cell and block, despite "fresh pack per INDI flight".
   - The S1 → S3 gain at A4 0.075 (51 % vs pilot 10 %) may therefore be inflated by a weak S1 baseline (**H**).
6. **A1 S0: the z-integral term is not visible in the logged thrust.** Coefficient 0.00–0.04 in 9 of 9 A1 S0 flights, seen and unseen, vs 1.1–2.1 in every other cell; the motors are not saturated. Cause **U**. It is the same on both sides, so R is not biased, but A1 S0 is not "geometric with a working z integral". This needs to be understood before the thesis interprets the A1 S0 sag (−17.7 / −19.7 cm).
7. **Session and order confounds are not in any CI.** Seen = 10-19/20, unseen = 10-23 with a new image; the blocks were not randomised; the bootstrap only resamples flights within sessions.
8. **Minor.**
   - `nn_vs_indi_check.py` assumes 500 Hz; the logs are ≈ 521 Hz, so lags are overstated by 4 % and the filter sits at 5.2 Hz. Negligible.
   - The lag is computed on concatenated gate samples; for A8 the contiguous airborne window gives the same 0 ms.
   - The ranking check required by the rule ("S1 vs S3 unchanged or not") is not reported. It is unchanged except A4 0.125, where S1 > S3 on seen and S3 ≳ S1 on unseen.

## 4. Robustness of the verdict (geometric pair; the rule is not changed, the alternatives are shown separately)

| Variant | Pooled R | Note |
|---|---|---|
| Pre-registered | **1.03** | re-run reproduces it exactly |
| Leave one flight out (63 removals, S0/S2, seen and unseen) | 0.99 – 1.06 | worst single cell 0.77 (A4 0.075 without `19-29-33`) |
| Bootstrap of flights within cell × strategy, both sides, 4 000 draws | median 1.03, **95 % CI [0.93, 1.17]**, P(R < 0.7) = 0 | per cell: A1 [1.11, 1.49], A4 0.075 [0.68, 1.20] (P < 0.7: 4.7 %), A4 0.125 [0.77, 1.28], A8 [0.88, 1.10] |
| A4 reference = trained pilot offsets only | 0.98 | A4 0.075 → 0.68 |
| Without A1 (soft cell) | 0.95 | |
| Without A1 and A4 0.075 | 0.98 | |
| Ratio of mean RI / median over cells | 1.03 / 0.98 | |
| Offline, released v2, in-gate RMSE seen → unseen | 0.453 → 0.464 | independent of the closed loop |
| Offline, held-out network / all-data network at unseen settings | ×1.18 (A1 ×1.25, A4 ×1.15, A8 ×0.96) | the network leans on the nearest trained settings, as expected for interpolation |

The verdict "no material overfit" (R ≥ 0.7) is robust to every flight, to resampling and to the alternative references. Its strength is limited by what it tests: interpolation 0.05–0.1 m from trained settings, within-session spread only, and a test that can detect a large loss of the NS2 benefit (more than ~30 %), not a subtle one.

## 5. Changes before the supervisor sees it, and what needs flights

**Reword (desk only):**
1. Training overlap: "A4 offsets 0.10 and 0.15 are in training; 0.05 was flown in the pilot only (10-20, after v2 was frozen), it is not a trained setting". Say that the A4 0.075 reference is half unseen and give the trained-only value (0.68, pooled 0.98).
2. Replace "same magnitude" with the split in §3.2: raw NN 20–30 % below the label, of which ≈ 0.3–0.45 m/s² is a label bias present far from the neighbour.
3. Remove the stale overview bullets ("not flashed", "not yet confirmed in flight") and the "not flashed" in §3.
4. Add the bootstrap CI and the LOFO range (§4) to the retention table.
5. "No sign of memorisation on either side" → "geometric side; the INDI side cannot be read (negative or near-zero bases)". Mention the drained-flight dependence of the A8 seen INDI gain.
6. Boring data: give the per-scenario effect (A4 −10 % consistent, A1 worse at unseen), name the Huber loss, and add that 86 % of the boring rows lie outside the gate.
7. S0 correlation at unseen settings in the gate: 0.93–0.95. Mention the seed variance of v2 (single draw).

**Desk follow-ups (no flights):**
- The out-of-gate label bias on the new bottom drone: check k_t from the solo hover phases.
- The A1 S0 integral observation (§3.6).

**Flights / retrain:** **none needed for the conclusion as stated** (no overfit at interpolated settings; v2 stays). New flights only if the supervisor asks for one of these:
- the strict version (H1 network flown on the pilot settings);
- extrapolation (settings outside the trained grid, e.g. dz < 0.25 or offset > 0.15);
- randomised block order.

No retrain is justified by this evidence. For the study itself: declare which study cells lie on the trained grid, or choose cells off it.

## 6. Overall assessment
The supervisor's concern was understood correctly, and the work answers it with sensible and mostly correct analyses. The pre-registered rule was fixed before the flights, computed exactly as written, and applied to 40 flights whose strategies are unambiguous in the logs. Its result (R 1.03, CI 0.93–1.17) holds up under every robustness check, and an independent offline check agrees (v2 error +2 % at unseen settings). "No material overfit at settings between the trained ones; ns2_v2 stays" is therefore justified, as strong evidence for interpolation and not for extrapolation.

The weak spots are in the supporting statements, not the verdict:
- the A4 offset 0.05 was wrongly listed as trained, which weakens one of the four cells;
- "same magnitude" is stated without noting that the label carries a vehicle bias, which explains most of the NN offset;
- the boring-data answer reports a pooled 2 % and hides a consistent A4 gain and an A1 loss;
- the INDI-side claim is stronger than its data;
- the meeting doc still carries stale "not flashed / offline only" bullets.

All of this can be fixed by rewording. Nothing needs new flights or a retrain.

## Commands (reproduce)
```
# rule re-run (identical per_flight.csv) and dry run on the seen pilot banks
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/unseen_block_analysis.py --banks bank_unseen_S0_2026-10-23 bank_unseen_S1_2026-10-23 bank_unseen_S3_2026-10-23 bank_unseen_S3S2_2026-10-23 --out <scratch>/rerun
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/unseen_block_analysis.py --banks bank_v2val_2026-10-19 bank_a1clean_2026-10-19 bank_pilot2_2026-10-20 bank_pilot3_2026-10-20 bank_pilot4_2026-10-20 bank_pilot5_2026-10-20 bank_pilot6_2026-10-20 bank_video_2026-10-20 --out <scratch>/seen
diff <(sort experiments/results/unseen_block_2026-10-23/per_flight.csv) <(sort <scratch>/rerun/per_flight.csv)
git diff 2576025e 66e28d1d -- experiments/analysis/unseen_block_analysis.py docs/lab_sessions/ns2_unseen_block_plan_2026-10-22.md
# strategy of every flight with margins (also on the pilot banks for the A1 integral observation)
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_topic2_strategy.py bank_unseen_S1_2026-10-23 bank_unseen_S3_2026-10-23 bank_unseen_S3S2_2026-10-23 bank_unseen_S0_2026-10-23
# robustness of R (LOFO, bootstrap, alternative references, INDI side)
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_topic2_retention.py <scratch>/rerun/per_flight.csv <scratch>/seen/per_flight.csv
# NN mean level in / out of the gate, onboard log vs rebuild
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_topic2_nn_offset.py <scratch>
# existing 10-22 networks scored on the unseen flights (no training)
python3 experiments/analysis/review_topic2_unseen_offline.py
# replay gate of the logging patch
python3 flying_drone_stack/firmware_app/host/test_omar_ns2_off_identical.py --old-dir ~/Desktop/crazyflie-firmware/build_pre_estlog_2026-10-22
```
Training coverage: `experiments/results/ns2_pilot2_2026-10-17/training_manifest_pilot2.json` + `experiments/logs/<flight>.meta.json`, `A9_SEQUENCE` in `crazyswarm2/crazyflie_examples/crazyflie_examples/formations/scenarios.py`. Scratch outputs: session scratchpad `t2/` (not in the repo).
