# Topic 2 — NS2 retrain and diverse training data on the Omar C tree (desk preparation, 2026-10-14)

Parent plans: `docs/plan_topics_2026-10-14.md`, `docs/plan_omar_tree_ns2_2026-10-14.md`. Status: **desk preparation; decisions of §6 taken by the author 2026-10-14 (all four proposals accepted); R1 random-waypoint scenario and the 66-flight training card written and desk-validated; label adapter and retrain not built; no flight.**
Depends on: lab phases 7–8 (`docs/lab_sessions/next_session_plan.md`) — the data must be flown on the validated tree.

## 1. Two findings that shape the plan
**(a) Omar's residual signals are not computed under `indi = 0`.** In `controller_omar_indi.c` the filters for `a_rpm_f` / `a_imu_f` (the logged `ctrlOmarIndi.a_rpm_f*`, `a_imu_f*`) are updated only inside `if ((self->indi & 1) && rpm_deck_available)`. Our own controller computed `a_res` passively in every mode; Omar's does not. So on S0 and S2 flights (indi 0) these log variables stay zero. The NS2 training label `a_res_z = a_meas − a_model` must therefore be **reconstructed offline** from the raw logs (IMU acceleration, attitude, deck RPM, Omar's mass 0.0427 and k_t, his 30 Hz Butterworth filters). The replay machinery already rebuilds exactly these inputs (`experiments/analysis/indi_replay_equal_worker.py`, side `omar_c`: runs Omar C offline with the logged state, gyro, accelerometer and RPM); it only has to expose `a_imu_filtered − a_rpm_filtered` as output columns. Check of the method: on S1/S3 flights (indi 3) the controller logs the same quantity, so offline label = logged label can be verified. Alternative (not recommended): fly the training flights with `indi = 1` so Omar logs it — that lets the INDI loop act on the very residual to be learned (contaminates the data; docs/13 §7). Adding passive filters to Omar's file is excluded by the "do not modify Omar's code" rule.

**(b) Training flights must be separate from the study repeats.** The study evaluates S2/S3 on the same scenarios and settings as the S0 flights. If NS2 were trained on the S0 study flights it would be evaluated on its own training data. v1 was trained on dedicated C.1 collection flights, not on later study repeats. Proposal: **dedicated data-collection flights** (grid + random waypoints) that never enter the study cells, with leave-one-flight-out cross-validation as in the existing gate (`run_c2_lab_retrain.py`). The study S0 repeats stay evaluation data.

## 2. Study size with four strategies (generator `study_flight_plan.py`, regenerated)
Tier 1: 12 settings × 4 strategies = **48 cells, 240 clean flights** (5 per cell); tier 1+2 = 72 cells (360); all = 108 cells (540). At ~12 flights/hour tier 1 is ≈ 20 h of flights plus rejects.

## 3. Coverage proposal (T2.1 — you define the numbers)
The network only acts inside the gate (|dx|, |dy| < 0.2 m, |dvx| < 1.5 m/s) with the neighbour ABOVE; v1 is thin at dz 0.75 (0.5 % of samples) and has no A8. Proposal for the dedicated training flights (S0, `indi 0`, NS2 off, deck RPM on, uSD logging):
| block | variation | flights |
|---|---|---|
| A1 stack hold (15 s) | dz ∈ {0.25, 0.375, 0.5, 0.625, 0.75} × 2 | 10 |
| A4 offset stack (line motion, speed 0.4) | lateral offset ∈ {0, 0.05, 0.10, 0.15} (gate edge 0.2) × dz ∈ {0.25, 0.5, 0.75} | 12 |
| A8 vertical swap | peak speed ∈ {0.2, 0.3, 0.4, 0.5} × 2 | 8 |
| A2 / A3 / A5 / A7 | speed ∈ {0.2, 0.3, 0.4, 0.5} × 1 each | 16 |
| random waypoints (§4) | 2 drones, 30 s each, stack-biased goals | 20 |
≈ 66 flights ≈ 5.5 h. v1 had 26 flights (R² 0.944). These numbers are a proposal; the cell sizes and whether to keep the old 26 flights in a combined fit (different residual definition: mass, k_t, filter, RPM source) are open — the old flights can only be re-labelled approximately.

## 4. Random-waypoint flights (T2.2 / T2.3)
Source script validated 2026-10-14 (`docs/plan_topics_2026-10-14.md` §Topic 2): IMRCLab `random_flight.py` (box sampling, speed 0.1–0.5 m/s, firmware `colAv` collision avoidance, 30 s). Required changes and safety review before any use: two drones only; remove the camera/AI-deck parts, `stabilizer.controller 4`, `hlCommander.yawrate`, hard-coded paths; our uSD start/stop sequence from `run_formation.py` and the `usec.reset` ground-only rule; our arena limits instead of the fixed box; per-drone stack bias (goals of one drone near the other's xy at ≥ 0.25 m vertical offset) so the pair is inside the gate far more than the ~9 % of unbiased sampling; `colAv.ellipsoidX/Y` 0.15, `ellipsoidZ` ≈ 0.12 (the module keeps centres ≥ 2× the radius apart per axis, so 0.12 allows ≈ 0.24 m vertical; the firmware default 0.3/0.3/0.9 is far too large); controller from the yaml (tree pin, Omar geometric on both drones). Not written yet — it needs your decision on the arena box and the bias before code is worth writing; a first run would be single-drone-far-apart to check the geofence.

## 5. Retrain pipeline (T2.5)
`run_c2_lab_retrain.py` → `dataset.py` expects merged logs with `<ego>.a_res_{x,y,z}`; for the tree this column is produced by the offline label step of §1(a) (a thin adapter that writes the same column name, so the downstream training, LOO gate and `export_weights_c.py` stay unchanged). Same architecture and weight layout (19 297, only 11 461 used by the C module). Frozen-network rule: the new weights replace `ns2_v1.npz` only after the LOO gate and your sign-off; then `export_weights_c.py` → new `ns2_weights.c` → new image → new manifest hash.

## 6. Decisions needed (nothing is built before these)
1. **Dedicated training flights separate from the study repeats** (recommended) — or accept training on study S0 flights with cross-validation by flight? (leakage risk).
2. **Label route:** offline replay reconstruction (recommended) vs flights with `indi = 1`.
3. **Coverage numbers** of §3, and whether the old 26 flights are used at all.
4. **Arena box and stack bias** for the random flights (§4).

## 7. Done 2026-10-14 (after the four decisions)
- **R1 scenario** (`crazyswarm2/.../formations/scenarios.py`, `run_formation --scenario R1 --seed N --total 40 --bias 0.7`): a random-waypoint flight **planned offline** instead of the IMRCLab script's online buffered-Voronoi avoidance (firmware `colAv`). Same data goal, but the complete trajectory goes through `safety.check()` before upload, is exactly reproducible from the seed (recorded in the meta like every scenario parameter), and the top vehicle is ≥ `dz_min` (0.25 m) above the bottom one at every waypoint, so with simultaneous legs it is above it at every instant; no reliance on a module we have never flown. Legs are rest-to-rest degree-7 smoothsteps (the library's standard curve) between random waypoints, both vehicles return to their slots at the end.
- **Coverage statistics** (`experiments/analysis/random_flight_coverage.py`, 40 seeds, 40 s each, auto-centred, height 0.5): all 40 pass the safety gate (min separation 0.27 m, peak speed 0.50 m/s, peak acceleration 0.82 m/s²). With stack bias 0.7 the pair is inside the NS2 gate **68 % of the time (34 s per flight)** against 17 % unbiased (the original script's behaviour). dz in the gate q5/q50/q95 = 0.29 / 0.47 / 0.68 m. **Limit:** because every leg is rest-to-rest, the bottom vehicle's speed in the gate is mostly low (median 0.07 m/s, q95 0.30 m/s); the high-speed coverage must come from the grid flights (A8, A2, A3, A5, A7 at 0.4–0.5 m/s).
- **Training card** (`experiments/analysis/ns2_training_flight_card.py` → `docs/ns2_training_flight_card.md`): 66 flights (A1 × 5 dz × 2, A4 offsets 0 / 0.05 / 0.10 / 0.15 × dz 0.25 / 0.5 / 0.75, A8 × 4 speeds × 2, A2 / A3 / A5 / A7 × 4 speeds, R1 seeds 101–120), strategy S0, session `ns2_train`, all commands validated; ≈ 32 min of flight time, ≈ 5.5 h with overheads.
- **Not done:** the label adapter (offline replay → `a_res_z` column), the retrain, any flight. The old 26 flights are kept out of the new fit (different residual definition) unless you say otherwise.
