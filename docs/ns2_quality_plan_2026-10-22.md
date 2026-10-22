# NS2 network quality — supervisor feedback 2026-10-22 and plan (Task 2, started first)

Source: supervisor reply to the meeting doc 2026-10-22 (user-pasted). Status: plan confirmed by the user 2026-10-22; Task 2 is worked first, FBL training (Task 1, `docs/fbl/KNODE_TRAINING_SOURCES_2026-10-22.md`) afterwards.

## Feedback (condensed)
- Flight time is enough, but (a) the network is trained on the same cases it is tested on (overfit risk), (b) imitation-learning data weighs every sample equally, so "boring" data (f_a ≈ 0) in large quantity makes the network overfit.
- Next step suggested: log the NN output **and** the INDI output on every strategy (even when NN or INDI is off), then compare. They should have the same magnitude, the NN a bit "faster". Offline the label can be smoothed with a zero-delay filter. Goal: find out whether the NN is trained well, which tells whether the dataset is good.

## Findings (checked 2026-10-22)
| Point | Result |
|---|---|
| Train/test overlap | **Confirmed.** ns2_v2 is trained on all 73 flights. Pilot test settings are inside it: A8 dz 0.5 / 0.5 m/s, A1 dz 0.5, A4 dz 0.5 / 0.5 m/s offsets 0.05–0.15 (A9 chained flights contain them). S0 and S1 are unaffected (no network). |
| Earlier generalisation evidence | Leave-one-flight-out keeps near-identical repeats in training; held-out-setting tests (no A3, no dz 0.75, no R1/A9/A1) mostly passed; the deployed v2 itself was never tested on unseen settings. |
| Boring data | **Confirmed.** 1.29 M training rows (both drones as ego): 54 % have \|a_res\| < 0.3 m/s²; inside the gate (46 % of rows) median 1.12 m/s², outside the gate 86 % of rows < 0.3. Training is plain MSE, equal weights, no resampling. |
| NN output logged | Yes, on every strategy (`rnn.pred_x/y/z`, evaluated whether or not `rnn.en`). |
| INDI output logged | **No under `indi 0` (S0, S2):** the filters `a_rpm_f` / `a_imu_f` only update when `indi & 1` (`controller_omar_indi.c`), logged values are 0. Offline reconstruction exists (`flying_drone_stack/tools/residual/omar_labels.py`, same C code over a log) but is not the onboard value. |

## Plan
1. **Offline NN-vs-INDI check** on existing flights (reconstructed INDI estimate, zero-delay smoothing): magnitude, lag, per scenario. **DONE 2026-10-22 → `docs/nn_vs_indi_check_2026-10-22.md`** (corr 0.9–0.96, lag ±10 ms, slope ≈ 1 in A8/A1 S0/A4; weakest A1 under INDI).
2. **DONE 2026-10-22 → `docs/ns2_heldout_rebalance_2026-10-22.md`:** pilot settings held out: in-gate RMSE +3 % (A1), none (A8), +22 % (A4, whole family removed); down-weighting boring rows (0.2 / 0.05): at most −2 %, within seed spread. Original item: **Dataset fixes:** (a) held-out split by *setting* (not by flight), disjoint from the study test cases; (b) rebalance the boring rows (subsample or weight by label size and input-space coverage).
3. **Retrain v3 — not needed on this evidence (held-out and rebalanced networks are not clearly better than v2 offline); the held-out networks above are the answer to the question. Original item: Retrain v3** and compare with v2 on held-out data; cost of removing the held-out settings.
4. **Logging change (needs a reflash, next lab):** INDI estimate updated and logged on all four strategies without changing control; check with the byte-identical replay (as for the NS2 and FBL patches). **DONE on the desk 2026-10-22:** `firmware_patches/indi_estimate_always_logged.patch` (last in the README order; the rpm force and the two filters now run whenever the rpm deck is present, `a_indi` is applied only with `indi & 1`). Gates: replay byte-identical 4/4 (`test_omar_ns2_off_identical.py`, old build kept as `crazyflie-firmware/build_pre_estlog_2026-10-22`); estimate under `indi 0` equals the `indi 3` estimate exactly (max diff 0, 6000 of 6000 rows non-zero); ARM image builds clean: `build_artifacts/study_tree_v2_fbl_estlog_candidate.bin` (sha256 `4fa0a130…`, 580 908 B, flash 56 %, RAM 76 %, local only, **not flashed**) = v2 weights + FBL (inert at `ctrlFbl.en 0`, G4) + this patch. The existing uSD config already logs `ctrlOmarIndi.a_rpm_f*`/`a_imu_f*` and `rnn.pred_*`, no config change needed. Open: whether the study flights should use this FBL-containing image or a separate image without FBL (not built); bench check after flashing = the logged estimate on an S0 flight must be non-zero.
5. **Decision after 1–3:** re-fly S2/S3 with v3 (≈ 25 flights on the pilot settings) so their numbers are on unseen settings. Open question for the user: held-out set = the pilot settings (dz 0.5 across A1/A8/A4) or separate new settings.

Desk first, as far as possible; lab only for validation and data collection.

## Closing the topic (decided 2026-10-22): cheap unseen-settings block only
Plan, flight table, commands and the pre-registered pass rule: `docs/lab_sessions/ns2_unseen_block_plan_2026-10-22.md` (40 flights S0–S3: A8 dz 0.4 / 0.35 m/s ×3, A1 dz 0.45 ×3, A4 dz 0.4 / 0.35 m/s offsets 0.075 and 0.125 ×2; v2 stays, nothing retrained). Desk side ready: commands validated with `--check`, analysis `experiments/analysis/unseen_block_analysis.py` (dry run on the pilot flights reproduces the meeting-doc numbers), `nn_vs_indi_raw/check.py` take bank names. Lab: flash the logging image, bench check, fly the block. Open: FBL-containing image or a separate one.

**Decisions 2026-10-22 (user):** image = the FBL-containing logging image (option A); pass-rule thresholds (R ≥ 0.7 / 0.4–0.7 / < 0.4) confirmed and fixed. Nothing open before the lab session except the supervisor's own pending decisions (tree approval, tiers, two vehicles first, dz floor, FBL as S4, NS2 execution).
