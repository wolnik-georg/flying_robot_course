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
1. **Offline NN-vs-INDI check** on existing flights (reconstructed INDI estimate, zero-delay smoothing): magnitude, lag, per scenario.
2. **Dataset fixes:** (a) held-out split by *setting* (not by flight), disjoint from the study test cases; (b) rebalance the boring rows (subsample or weight by label size and input-space coverage).
3. **Retrain v3** and compare with v2 on held-out data; cost of removing the held-out settings.
4. **Logging change (needs a reflash, next lab):** INDI estimate updated and logged on all four strategies without changing control; check with the byte-identical replay (as for the NS2 and FBL patches).
5. **Decision after 1–3:** re-fly S2/S3 with v3 (≈ 25 flights on the pilot settings) so their numbers are on unseen settings. Open question for the user: held-out set = the pilot settings (dz 0.5 across A1/A8/A4) or separate new settings.

Desk first, as far as possible; lab only for validation and data collection.
