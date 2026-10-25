# Handover: independent check of Topic 2 (NS2 quality) results (2026-10-25)

## Context
- Master's thesis, comparative study of downwash compensation on two Crazyflie 2.1 Brushless drones (bottom cf5, top cf_second). Strategies: S0 geometric (Omar C controller 9), S1 INDI, S2 geometric + NS2 (Neural-Swarm2 network, version ns2_v2), S3 INDI + NS2; S4 FBL (Topic 1, separate).
- Topic 2 = the supervisor's feedback of 2026-10-22: (a) NS2 is trained on the cases it is tested on, (b) equal weights on "boring" near-zero rows can overfit, (c) compare the NN output with the INDI output on every strategy. Closed 2026-10-23.
- **Read first, completely:** `docs/topic2_ns2_quality_summary_2026-10-25.md` (the claims to check), then `docs/meetings/2026-10-29.md` §1–4 (plots), `docs/ns2_quality_plan_2026-10-22.md`, `docs/nn_vs_indi_check_2026-10-22.md`, `docs/ns2_heldout_rebalance_2026-10-22.md`, `docs/lab_sessions/ns2_unseen_block_plan_2026-10-22.md` (pre-registered rule), `docs/lab_sessions/2026-10-23.md`, `docs/tree_comparison_2026-10-19.md` §9 (the "seen" reference), `docs/ns2_model_registry.md`.

## Sources
- Code: `experiments/analysis/nn_vs_indi_raw.py`, `nn_vs_indi_check.py`, `ns2_heldout_pilot_study.py`, `ns2_heldout_plot.py`, `unseen_block_analysis.py`, `ns2_applied_check.py` (strategy check per flight); training `flying_drone_stack/tools/residual/train.py`; firmware patch `flying_drone_stack/firmware_patches/indi_estimate_always_logged.patch`; replay gate `flying_drone_stack/firmware_app/host/test_omar_ns2_off_identical.py`.
- Results: `experiments/results/nn_vs_indi_2026-10-22/`, `ns2_heldout_2026-10-22/`, `unseen_block_2026-10-23/`, `unseen_nn_vs_indi_2026-10-23/`.
- Data: manifests in `experiments/logs/bank_*/training_manifest.json`; merged CSVs in the `bank_*` folders (local, not in git); raw uSD files `experiments/logs/usd_raw/` (in git); decoder `flying_drone_stack/tools/decode_usd_log.py`. Use complete uSD logs only, never radio CSVs.
- Python: `~/.pyenv/versions/flying_robots/bin/python` for analysis; system `python3` (3.10) for anything that imports the firmware bindings (`~/Desktop/crazyflie-firmware/build`).

## What NOT to do
- Never fly, flash or touch hardware. Never commit or push; never add a Co-Authored-By trailer.
- Do not retrain or overwrite a registered network (`docs/ns2_model_registry.md`); write new outputs to a scratch folder or `experiments/analysis/review_topic2_*.py`.
- Do not change the pre-registered rule after the fact; if you think it is weak, say so separately.
- Do not edit meeting docs or lab session docs.

## What to do
1. Every number and claim in the summary: recompute it from the data / results and mark it confirmed / corrected / unverifiable (file, script, number).
2. The pre-registered retention rule: is it computed exactly as written (pairing, metrics per cell, pooling, which flights)? Are the 40 flights all the right strategy (check with `ns2_applied_check.py` and the logged `rnn.pred_*` / INDI channels, not only the bank names)? Any flight excluded or double counted?
3. The "seen" reference: same metric definitions, comparable conditions (other session, battery, block order)? How sensitive is R to that choice?
4. The held-out study: is the hold-out really leakage-free (no pilot-setting rows in training, no shared flights)? Is the seed spread reported correctly?
5. NN vs INDI: label construction (zero-delay filter), gate, lag estimation, slope; does the conclusion "in phase, same magnitude" follow?
6. Statistics: with 2–3 flights per cell, how much does the verdict depend on single flights (leave-one-flight-out on R, bootstrap if cheap)?
7. What is missing or overstated, and what a reviewer of the thesis would ask.

## Deliverable
- `docs/topic2_ns2_quality_independent_review.md`, short, tables first: (a) claim check table, (b) issues found with evidence, (c) robustness of the verdict (R under alternatives), (d) one-paragraph assessment.

## Reporting
- Confidence per claim (verified / strong evidence / hypothesis / unknown); say plainly if a conclusion is wrong; list the commands you ran.
