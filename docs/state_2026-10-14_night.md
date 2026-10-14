# State 2026-10-14 (night) — what was achieved, what is open, what is next in the lab

Read this first for the study side; the ordered lab list is in `next_steps_checklist.md` ("NEXT LAB SESSION") and the commands in `lab_sessions/next_session_plan.md`.

## What happened today (in order)
1. Meeting results: finalize NS2 and INDI, follow up FBL, then data collection. **Decision (author): Pure INDI = Omar C + z integral 1.5.**
2. Professor's request: a unified foundation (geometric + switches). **Decision: one Omar C controller tree** — S0 geometric, S1 full INDI, S2 geometric + NS2, S3 INDI + NS2 (FBL = S4); both drones controller 9; Omar's code unchanged; shared position gains, z integral 1.5 in all four.
3. **FBL**: only the training script is asked from the authors (meeting doc updated; licence and singularity not asked).
4. **Thesis draft revision** (Opus, 7 phases + closing pass) finished, nothing merged: `thesis_revision/STATUS.md` (D-11 resolved by the tree decision).

## Topic 1 — NS2 in C on the Omar tree: DESK DONE
- `ns2_residual.{h,c}` in the firmware core + `firmware_app/ns2_weights.c` (frozen v1, 11 461 floats), hook in `controller_omar_indi.c` (`a_d − a_nn`, only when `rnn.en = 1`, evaluated and logged under every strategy).
- Verified: C == Rust flight code bit-identical (400 stateful cases); `rnn.en = 0` byte-identical to the old Omar C on four logged flights; closed-loop two-drone SIL: NS2 removes the geometric Omar's −11…−15 cm crossing dip. Results `ns2_c_phase3_5_results_2026-10-14.md`. SIL cannot show whether S3 helps (perfect RPM information); negative control inconclusive.
- Two host-simulator artifacts found and fixed (fake `usecTimestamp`, shared `rnn.ready`); hardware unaffected.
- Image `…_ns2c.bin` sha256 `dfac761a…2d5` (532 960 B; flash 52 %, RAM 75 %); patches in `firmware_patches/` (README order), applied on a clean tree; candidate manifest `docs/frozen/omar_ns2c_tree/` (live manifest NOT changed).
- Study yaml `crazyswarm2/crazyflie/config/study/tree_S0..S3.patch`, strategy blocks S0–S3 in `study_strategies.json` (old = L0–L2), `run_formation --strategy S3`.
- Bench tool `flying_drone_stack/tools/ns2c_bench_check.py` (static; this firmware has no arming gate, so no setpoints on the bench).

## Topic 2 — NS2 retrain with diverse data: DESK DONE
- Decisions: dedicated training flights (separate from study repeats), labels reconstructed offline, 66-flight plan, old 26 flights not used.
- Finding: Omar's `a_rpm_f`/`a_imu_f` are not computed under `indi 0` → `omar_labels.py` rebuilds `a_res = a_imu_f − a_rpm_f` with Omar's own controller code (validated: corr 0.998 vs logged); `merge_usd_logs.py --omar-labels deck`; dataset and training chain unchanged.
- R1 random-waypoint scenario (offline-planned, safety-gated, seeded; `run_formation --scenario R1 --seed N`); in-gate time 68 % vs 17 % unbiased.
- Cards: `ns2_training_flight_card.md` (66 flights, S0), `study_shakedown_card.md` (tree validation); study size now 48 cells / 240 clean flights (tier 1).

## What is NOT done (all needs the lab)
1. Flash both drones with `_ns2c`, bench check, manifest swap.
2. Tree validation flights S0 → S2 → S1 → S3 (INDI takeoff from the ground with Omar's controller is untested).
3. NS2 timing on the drone (`rnn.us_max`, limit to be defined).
4. 66 training flights → labels → retrain → LOO gate → new weights → new image → re-validate S2/S3.
5. Re-freeze protocol v1.0 for S0–S3, then tier 1 (240 clean flights).
6. Thesis draft updates (Ch1 scope/gap, Ch4 strategies and fairness) after validation.
Small open items: send the FBL question; consult the supervisor about Pure INDI = Omar C and the unified tree; RPM source for the final labels (deck vs DShot).

## Where things are
Plans: `plan_topics_2026-10-14.md`, `plan_omar_tree_ns2_2026-10-14.md`, `ns2_c_module_spec.md`, `plan_ns2_retrain_data_2026-10-14.md`. Code: `crazyflie-firmware` (patched tree, patches preserved), `flying_drone_stack/firmware_app/ns2_weights.c`, `tools/residual/omar_labels.py`, `experiments/analysis/{omar_ns2_closed_loop_sil,random_flight_coverage,study_yaml_tree_variants,ns2_training_flight_card}.py`. Memory: `reference_omar_tree_ns2_facts_2026-10-14.md`.
