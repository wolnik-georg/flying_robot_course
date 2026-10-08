# Cursor prompt — state sync (2026-10-08). Read, summarise, do NOT change anything yet.

You are the executor in a Claude-plans / Cursor-executes workflow on the master's thesis "Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams" (Crazyflie 2.1 Brushless, deadline 26 Feb 2027). Repo `~/Desktop/flying_robot_course` (branch `main`, pushed up to `78041acd`), firmware tree `~/Desktop/crazyflie-firmware` (local, patch-preserved, not pushed), crazyswarm2 `~/Desktop/crazyswarm2`. Claude independently validates every result you deliver; several of your earlier results were corrected (see "Lessons" below).

## Standing rules
- No commits/pushes unless asked; **never add a Co-Authored-By trailer** to commits. Never touch the user's hand-edited `docs/meetings/*` or `docs/lab_bench_cheatsheet_2026-10-03.md`.
- Read-only on `experiments/logs/**`. Do not edit `docs/07_*`, `docs/next_*`, `docs/lab_sessions/**` unless the prompt says so.
- Host python: system `python3` (3.10) with `PYTHONPATH=~/Desktop/crazyflie-firmware/build`; the pyenv `flying_robots` (3.12) is for plotting only. **Host bindings must be rebuilt with the NS2 feature**: `cd flying_drone_stack/firmware_app && DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" cargo build --release --target x86_64-unknown-linux-gnu --features residual_nn`, then `cd ~/Desktop/crazyflie-firmware && rm -f build/_cffirmware*.so build/cffirmware_wrap.c && make bindings_python` (a rebuild without `residual_nn` silently breaks every network-on SIL run).
- Fresh process per SIL episode, cache results, run long jobs in the background. Dips are negative numbers; shallower is better.
- A documented "not validated / does not fix it" is an acceptable outcome; an unvalidated number presented as a result is not.

## State by topic (read these docs, in this order)
1. `docs/07_Thesis_Progress_Checklist.md` (row "Next action — lab") and `docs/next_steps_checklist.md` (section "LAB SEQUENCE") — single source of truth.
2. **NS2 (Neural-Swarm2, geometric + onboard network, 100 Hz, z-only):** hardware 10-05 (`docs/lab_sessions/2026-10-05.md`): crashes were tracker pose swaps and a cf_second battery collapse, not the network; with `res_sign=+1` the A8 crossing dip is −10.9 cm vs −5.9 cm without the network. **Closed-loop SIL validated for A8 only** (`docs/62`): mirror-symmetric bank force + one torque scalar c = 0.0032 m² (fitted on network-off only): off −5.85 cm, `+1` test −9.96 cm (hardware −10.9 ± 1.5), prediction `res_sign=−1` ≈ −2.3 cm (stable, direction supported, magnitude ±1.5 cm). Not validated: A1 with network on (SIL tilt 38–48°), tilt p99 13° vs 22.9°.
3. **INDI replay (`docs/63`, `docs/64`):** valid where the comparison is valid; Omar C ≡ Omar Rust on identical inputs (~1e-7/1e-9); the old A1 "failure" was motor saturation (96 % of samples at PWM max) in the reference. Ours vs Omar on A8: thrust −19 %, corr 0.97, torques loosely correlated.
4. **Omar C/Rust z offset (`docs/65`, `docs/66`):** +16…+25 cm = commanded-vs-delivered thrust DC gap (~1.14), invisible to INDI (IMU ≈ RPM force). Opt-in z integral `ctrlOot5.kpos_iz` (Rust, controller 10) / `ctrlOmarIndi.Kpos_Iz` (C, controller 9), default 0 = bit-identical to Omar (verified on replay), ARM CF21BL build OK. SIL recommends 1.5 (1.0 cautious). **Not removed by the integral: Omar's hardware crossing dips (11–26 cm below its own level on A8; ours ≈ 6 cm)** — the SIL does not reproduce them. Omar C reads the optical deck RPM directly (unfiltered); Omar Rust and ours read DShot via `rpm_get_all()` with the spike filter (28 000 RPM cap, 10 000 RPM jump, hold-last-good; 0xFFFF → 0 is an open item).
5. Decisions (user, 10-08): all prepared 2-drone scenarios stay (A1 included); 100 Hz network is enough; Omar's controllers stay as close to the original as possible; INDI variant choice depends on the z-offset fix (and the dip finding) — supervisor decision pending; the professor already followed up the FBL email (Strategy 3 blocked on author code).

## Lab sequence (what happens next on hardware — do not change it)
1. NS2 sign test: fresh batteries (rest ≥ 4.1 V), pull crazyswarm2 + check the printed cf5 config, pose bag; A8 `res_sign=-1` ×3 (abort ready on the first), A8 `rnn.en=0` ×2, A1 `rnn.en=0` ×1; A1 network-on `-1` only if A8 passes and the A1 baseline is clean. Then `res_sign: 1` is restored and `post_flight_check.py` + `ns2_signtest_analysis.py` run. Pack: `docs/lab_session_pack_signtest.md` §0–§4.
2. Only after 1: INDI "Omar + Iz" (pack §4b): Omar Rust hover `kpos_iz` 0 → 1.0 → 1.5, A8 ×2, A1 ×2; Omar C only if chosen.
3. Later: command→thrust latency bench, logging patch (`f_d`, applied gains, rpm source — also decides the 1.14 factor and the ours-vs-Omar A8 offset), A1 yaml flights for our oscillation.

## Open desk items (Claude decides what you get; candidates)
RPM sentinel (0xFFFF → hold-last-good) + host unit test; SIL realism (tilt p99, Omar crossing dips, A8 matched-thrust fall); thesis chapters 6–9 skeleton data; Omar+Iz entry in the comparison; keeping docs consistent.

## Lessons from earlier rounds (do not repeat)
Compare against the right reference (saturated motors invalidated a replay reference); a number calibrated to the target is not evidence; never regenerate a patch file without checking it is non-empty (`controller_omar_indi.c.patch` was emptied once); rebuild bindings with `--features residual_nn`; do not stop at a grid edge — extend the search; a deterministic simulation needs noise before "3 seeds" mean anything; circular fixes are rejected.

## Your task now
Reply with (a) ≤ 15 bullets stating your understanding of the current state and the lab sequence, (b) any inconsistency you find between the docs listed above, (c) questions, if any. Change nothing.
