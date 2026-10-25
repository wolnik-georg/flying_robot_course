# Handoff: independent review of Topic 2 — did we address the supervisor's NS2 concern properly and conclude correctly? (2026-10-25)

## Your role
You are an independent reviewer. Check, from the data and code, whether the supervisor's concern about the NS2 network was understood correctly, addressed with the right analyses, executed without errors, and concluded correctly — and whether anything in between was skipped, overstated or decided on weak evidence. Do not trust the summaries: recompute.

## Background (short)
- Master's thesis: comparative study of downwash compensation on two Crazyflie 2.1 Brushless drones (bottom drone cf5 = study drone, top drone cf_second). One firmware controller tree (Omar C controller, `stabilizer.controller 9`) with four strategies: **S0** geometric, **S1** INDI, **S2** geometric + NS2, **S3** INDI + NS2. (S4 FBL = Topic 1, not part of this review.)
- **NS2** = a Neural-Swarm2-type network (deep sets, onboard in C) that predicts the downwash acceleration on the bottom drone from the relative state of the neighbour; trained on our flights with the INDI residual as label (`a_res` = measured acceleration − rotor-speed model). Released version **ns2_v2**, trained on all 73 collection flights (`flying_drone_stack/tools/residual/weights/NS2_V2.md`, `docs/ns2_model_registry.md`, `docs/ns2_pilot_retrain_2026-10-16.md`).
- On 2026-10-22 the supervisor read the meeting doc `docs/meetings/2026-10-22.md` (pilot comparison of S0–S3, `docs/tree_comparison_2026-10-19.md` §9) and replied.

## The supervisor's concern (condensed by us on 2026-10-22; the original email is not in the repo — ask the user for it if the wording matters)
From `docs/ns2_quality_plan_2026-10-22.md`:
- Flight time is enough, but **(a)** the network is trained on the same cases it is tested on (overfit risk), **(b)** imitation-learning data weighs every sample equally, so "boring" data (f_a ≈ 0) in large quantity makes the network overfit.
- **(c)** Next step suggested: log the NN output **and** the INDI output on every strategy (even when NN or INDI is off), then compare. They should have the same magnitude, the NN a bit "faster". Offline the label can be smoothed with a zero-delay filter. **Goal: find out whether the NN is trained well, which tells whether the dataset is good.**

## What we did (chronological; check each step)
1. **Checked the facts (2026-10-22):** overlap confirmed (pilot settings inside ns2_v2's training set); 54 % of 1.29 M training rows have |a_res| < 0.3 m/s²; NN output already logged on all strategies; INDI estimate was zero under S0 / S2. → `docs/ns2_quality_plan_2026-10-22.md` (Findings table).
2. **Offline NN vs INDI** on 37 pilot flights with the NN not applied (S0 / S1), INDI estimate rebuilt offline from the logs, zero-delay 5 Hz label → `docs/nn_vs_indi_check_2026-10-22.md`, results `experiments/results/nn_vs_indi_2026-10-22/`, code `experiments/analysis/nn_vs_indi_raw.py`, `nn_vs_indi_check.py`, label code `flying_drone_stack/tools/residual/omar_labels.py`.
3. **Held-out settings and boring-row weighting** (offline, 3 seeds): networks retrained without the pilot settings and with boring rows down-weighted (0.2 / 0.05), scored on the 37 pilot flights → `docs/ns2_heldout_rebalance_2026-10-22.md`, results `experiments/results/ns2_heldout_2026-10-22/`, code `experiments/analysis/ns2_heldout_pilot_study.py`, `ns2_heldout_plot.py`, `flying_drone_stack/tools/residual/train.py` (`--boring-weight`).
4. **Decision:** no v3 retrain (held-out / rebalanced networks not clearly better offline). The plan's item "re-fly S2 / S3 with v3 on the pilot settings" was therefore dropped; replaced by a cheap flight block at **new settings** with ns2_v2 unchanged, with a **pre-registered** pass rule → `docs/lab_sessions/ns2_unseen_block_plan_2026-10-22.md`, checklist `docs/lab_sessions/ns2_unseen_block_checklist.md`.
5. **Logging change:** firmware patch `flying_drone_stack/firmware_patches/indi_estimate_always_logged.patch` (INDI estimate computed and logged under all strategies, applied only with INDI on); replay gate `flying_drone_stack/firmware_app/host/test_omar_ns2_off_identical.py`; image `study_tree_v2_fbl_estlog_candidate.bin` (sha256 4fa0a130…).
6. **Lab 2026-10-23:** image flashed on both drones, 40 flights S0–S3 at A8 dz 0.4 / 0.35 m/s, A1 dz 0.45, A4 dz 0.4 offsets 0.075 / 0.125 → `docs/lab_sessions/2026-10-23.md`; results `experiments/results/unseen_block_2026-10-23/` (`verdict.json`, `cells.csv`, `per_flight.csv`, `retention.csv`), `experiments/results/unseen_nn_vs_indi_2026-10-23/`; code `experiments/analysis/unseen_block_analysis.py`; strategy check per flight `experiments/analysis/ns2_applied_check.py`.
7. **Conclusion reported** (meeting doc `docs/meetings/2026-10-29.md` §1–4): pooled retention 1.03 ≥ 0.7 → "no material overfit"; boring data: no measurable effect; NN follows INDI in phase with the same magnitude; ns2_v2 stays. **All results in one place: `docs/topic2_ns2_quality_summary_2026-10-25.md` (read it completely; it is the list of claims to check).**

## Points we want you to examine in particular
- **Is (a) answered?** Retention compares the NS2 gain (S0 → S2) at unseen vs seen settings. The seen reference comes from other sessions (2026-10-19/20, `docs/tree_comparison_2026-10-19.md` §9). Unseen settings are between trained ones (interpolation; A1 dz 0.45 is 0.05 m from a trained dz). Is "no material overfit" justified, and how strong is the claim?
- **Is the rule computed exactly as pre-registered** (pairing, metric per cell, pooling, which flights; any flight excluded, double counted or of the wrong strategy)? Check the strategy of all 40 flights from the logs (`ns2_applied_check.py`, `rnn.pred_*`, INDI channels), not from bank names.
- **Is (b) answered?** Only re-weighting (0.2 / 0.05) was tested; hard subsampling and coverage-based weighting were not. Does "≤ 2 % gain, within seed spread" justify "no v3"? Is the hold-out leakage-free (no pilot-setting rows, incl. A9 chained flights)?
- **Is (c) answered, and does it show the NN is trained well?** Offline 10-22 and at unseen settings: high correlation under S0, weaker under INDI (A1 S1 0.71 offline / 0.84 unseen; A4 S1 0.53–0.68 unseen), and **the NN's mean level is below the label in most cells** (e.g. A1 S0 −1.30 vs −1.66, A4 S0 gate −1.11 vs −1.77 m/s²) while the meeting doc reports only the S0 correlation. The supervisor expected the NN to be "a bit faster"; we found "in phase". Is "same magnitude" correct? What does this say about the dataset?
- **Statistics:** 2–3 flights per cell, non-randomised block order (S1, S3, S2, S0), no confidence intervals. How sensitive is the verdict (leave-one-flight-out on R, bootstrap)?
- **Anything missing or overstated** that the supervisor or a thesis examiner would ask about; anything we concluded that the data do not support.

## Sources (all in the repo unless stated)
- Plans / results docs: `docs/ns2_quality_plan_2026-10-22.md`, `docs/nn_vs_indi_check_2026-10-22.md`, `docs/ns2_heldout_rebalance_2026-10-22.md`, `docs/lab_sessions/ns2_unseen_block_plan_2026-10-22.md`, `docs/lab_sessions/ns2_unseen_block_checklist.md`, `docs/lab_sessions/2026-10-23.md`, `docs/topic2_ns2_quality_summary_2026-10-25.md`, `docs/meetings/2026-10-22.md` (what the supervisor read), `docs/meetings/2026-10-29.md` §1–4 (what we answer), `docs/tree_comparison_2026-10-19.md` §9 (seen reference).
- NS2 background: `docs/ns2_model_registry.md`, `flying_drone_stack/tools/residual/weights/NS2_V2.md`, `docs/ns2_pilot_retrain_2026-10-16.md`, `docs/ns2_retrain_options.md`, `docs/13_Residual_Learning.md`, `docs/ns2_c_module_spec.md`.
- Code: listed per step above; training `flying_drone_stack/tools/residual/train.py`; decoder `flying_drone_stack/tools/decode_usd_log.py`.
- Data: manifests `experiments/logs/bank_*/training_manifest.json`; merged CSVs in the `bank_*` folders (local only, ~6 GB, not in git); raw uSD files `experiments/logs/usd_raw/` (in git). Banks of the unseen block: `bank_unseen_S0_2026-10-23`, `bank_unseen_S1_2026-10-23`, `bank_unseen_S3_2026-10-23`, `bank_unseen_S3S2_2026-10-23`; pilot / seen banks `bank_*_2026-10-19`, `bank_*_2026-10-20`. **Use complete uSD logs only, never radio CSVs.**
- Python: `~/.pyenv/versions/flying_robots/bin/python` for analysis; system `python3` (3.10) for anything importing the firmware bindings (`~/Desktop/crazyflie-firmware/build`).

## What NOT to do
- Never fly, flash or touch hardware. Never commit or push; never add a Co-Authored-By trailer.
- Do not overwrite or retrain a registered network (`docs/ns2_model_registry.md`); put new scripts in `experiments/analysis/review_topic2_*.py` and outputs in a scratch folder.
- Do not change the pre-registered rule after the fact; if you think it is weak, say so separately and show what an alternative gives.
- Do not edit the meeting docs, lab session docs or the summary; write your own file.

## Deliverable
`docs/topic2_ns2_quality_independent_review.md`, short, tables first:
1. Claim check: every claim of `docs/topic2_ns2_quality_summary_2026-10-25.md` and of the meeting doc §1–4 → confirmed / corrected / unverifiable, with evidence (file:line, script, number).
2. Did we address (a), (b), (c) and the supervisor's goal ("is the NN trained well / is the dataset good")? One row each: yes / partly / no, why.
3. Errors or weak points found (with numbers), incl. the NN mean-level offset and the INDI-side agreement.
4. Robustness of the verdict (R under leave-one-flight-out, alternative references / metrics).
5. What should be added or reworded before the supervisor sees it, and what (if anything) needs new flights or a retrain.
6. One-paragraph overall assessment.

## Reporting
- Confidence per claim: verified / strong evidence / hypothesis / unknown. Say plainly if a conclusion is wrong or overstated. List the commands you ran so the result can be reproduced.
