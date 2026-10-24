# FBL (Strategy 3) — what is left is lab work (state 2026-10-23: images already flashed, branch `study_FBL`)

Everything below is desk-prepared and untested on hardware. Order is a proposal for you to confirm.

## NEXT (prepared 2026-10-24 night): FBL with the stock rate-PID inner loop (`ctrlFbl.inner 1`, `STOCK_INNER_LOOP_2026-10-24.md`)
Proposal, to be confirmed by you; desk-tested only (host 1:1 vs `controller_pid.c`, simulation), never flown.

| # | Step | Exact action | Pass criterion |
|---|---|---|---|
| S1 | Flash | `BIN=~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build_artifacts/study_tree_v2_fbl_stockinner_candidate.bin` (sha256 `5b20fe3a112066996ca44be7c6131531afb296217cd55c9a887f86886e4f4f7b`, 583 580 B); `sha256sum $BIN`; `cfloader flash $BIN stm32-fw -w <uri>` on cf5 and cf_second, power-cycle | hash matches, flash OK |
| S2 | Bench (CS2 stopped) | `fbl_bench_check.py <uri>` on both; `fbl_init_survival_check.py radio://0/80/2M/E7E7E7BB02` | `ALL PASS`; prints `stock-inner image params present: ['inner', 'abg_on_new', 'nn_en']`, all 0 at boot |
| S3 | Yaml | crazyswarm2 (lab PC `~/georg/ros2_ws/src/crazyswarm2`): `git fetch origin && git checkout study_FBL_stock` (pushed 2026-10-24), rebuild, `lab_check_strategy.py`; optionally copy `study/study_manifest_FBL_stock_candidate.json` over `study_manifest.json` | `ctrlFbl.en 1`, `inner 1`, `cd 0.55`, no gains / preset written |
| S4 | Readback after launch | `fbl_bench_check.py <cf5 uri> --readback` | `FBL ON … gains: preset 1 (ours), inner loop: STOCK rate PID (inner 1)` |
| S5 | uSD | cards archived / counted as before; **start the uSD log before takeoff** (to see the takeoff, the FBL switch-on and the integral) | — |
| S6 | Flights | A1 first (hand on the abort, fresh packs), then A8, A4 0.075, A4 0.125 (unseen-block settings as on 10-24) | takes off, holds, lands; `steps > 0`, `faults 0` |
| S7 | Acceptance (analysis, not a protocol rule) | `fbl_log_check.py`, `fbl_hw_overview.py` (add the new bank), `fbl_replay_check.py`, `review_fbl_yaw_budget.py`, `review_fbl_lateral_chain.py` | yaw held (simulation: < 2°), no slow lateral drift, rate error has no steady part |
| S8 | Optional | C_D like the paper (manual): A1 with `cd` 0.43 / 0.55 / 0.65 under `inner 1`; force-network data card (`STOCK_INNER_LOOP_2026-10-24.md` §5: A3 and A2 at dz 0.4, 0.4 m/s, ≥ 28 s; dz 0.3 OOD) | — |

uSD logging (not changed, cap 96): to also log the stock loop, swap `motor.m1_rpm`…`motor.m4_rpm` (deck RPM; DShot `rpm.m*` is the standing source) for `ctrlFbl.yaw_sp`, `pid_rate.roll_outI`, `pid_rate.pitch_outI`, `pid_rate.yaw_outI` in `usd_fbl_config.txt` — your decision.

Fallback: `ctrlFbl.inner 0` (live parameter) = exactly the controller flown on 2026-10-24.

| # | Step | Exact action | Pass criterion |
|---|---|---|---|
| 0 | Pull | `git pull` in both repos; in crazyswarm2 `git fetch origin` (branch `study_FBL` = main + FBL switches + `ctrlFbl.cd 0.55`) | `study_FBL` exists |
| 1 | **Flash both drones with the init-fix image** (2026-10-24) | `BIN=~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build_artifacts/study_tree_v2_fbl_initfix_candidate.bin` (sha256 `27e0e100…`, 580,780 B; local file, copy by hand if the machine is another one); `cfloader flash $BIN stm32-fw -w <uri>` for cf5 and cf_second, power-cycle both | flash OK; then `fbl_init_survival_check.py` (step 3b) passes |
| 2 | SD cards | **archive both cards first** (cf_second at thesis64, cf5 at thesis65 after 2026-10-23; ~19 flights would reach the 85 limit); `restore_study_sd_config.py <cf5 mount> --fbl` on the bottom drone's card (cf_second keeps `usd_study_config.txt`); `check_usd_deck.py` | 96 variables, deck OK |
| 3 | Bench, both drones, CS2 stopped | `python3 flying_drone_stack/tools/fbl_bench_check.py <uri>` (cf5 `radio://0/80/2M/E7E7E7BB02`, cf_second `radio://0/80/2M/E7E7E7E7E9`) | `ALL PASS`: controller 9, preset 1, `en` 0 at boot |
| 3b | **Init-survival check, once after the flash, right after a power cycle (CS2 stopped)** | `python3 flying_drone_stack/tools/fbl_init_survival_check.py radio://0/80/2M/E7E7E7BB02` (writes `cd 0.55`, `en 1` on controller 6, switches to 9, reads fresh; restores) | `PASS: ctrlFbl.en 1 and cd 0.55 survived the controller init`. With this image no priming and no double launch is needed, also not after battery swaps |
| 4 | Yaml | `cd ~/georg/ros2_ws/src/crazyswarm2 && git checkout study_FBL`, then `python3 flying_drone_stack/tools/lab_check_strategy.py` (rebuild if STALE) and optionally `lab_preflight.py --study FBL --sd-config <card>/config.txt` (copy `study/study_manifest_FBL_candidate.json` over `study_manifest.json` first; it names the estlog image) | prints `FBL (Strategy 3, S4)`; cf_second stays S0 |
| 5 | Launch + check | `ros2 launch crazyflie launch.py`; `fbl_bench_check.py <cf5 uri> --readback` | `FBL ON … gains: preset 1 (ours)`, `cd 0.55` |
| 6 | First FBL flight | A1: `ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.5 --hold 15.0 --height 0.5 --auto-center --yes` (fresh packs, hand ready on the abort, no `--study`) | takes off, holds, lands |
| 7 | Verify it ran | stop CS2: `fbl_bench_check.py <cf5 uri> --post-flight`; copy both cards (`copy_usd_log.py <mount> <drone> --file thesisNN --tag fbl_S4`), `experiments/analysis/fbl_log_check.py <log>` | `steps > 0`, `faults 0` (`tripped 1` = latch fell back to Omar) |
| 8 | Then, same settings as the pilot (dz 0.5, 0.5 m/s) so S4 sits in the S0–S3 tables | A1 ×5 total; A8 ×4: `--scenario A8 --dz 0.5 --height 0.5 --passes 4`; A4 ×3 per offset 0.05 / 0.10 / 0.15: `--scenario A4 --dz 0.5 --speed 0.5 --offset <o> --motion line --rotate 90 --height 0.5` (all with `--auto-center --yes`); copy after every flight or few | per-cell n like the pilot (18 flights) |

- **Reading guide for the first flights (expected channel values, what to change): `FIRST_FLIGHT_CHECKLIST.md`.**
- **If the drone sags or oscillates in the first flights:** the knobs are `ctrlFbl.ki_z` (default 12), `ctrlFbl.cd` (firmware default 0.43; KNODE on our logs gives ≈ 0.55 with calibrated thrust, `KNODE_TRAINING_STEP3_2026-10-22.md`; downwash strength, 0 = no residual) and `ctrlFbl.preset`; all settable per flight without reflashing.
- **First uSD FBL log is also the shakedown of the logging chain:** decode + merge have never seen an FBL config (11 INDI/oot5 variables swapped for `ctrlFbl.*`); if a downstream script misses a column, fix it then.
- **Expected from simulation (optimistic):** in the calibrated plant (measured thrust, downwash 1.3×) A8 dip about 1 cm, A1 bias about 0.2 cm (RMS 3 cm), A4 2–3 cm RMS (`SIM_COMPARISON_2026-10-20.md`); real values will be worse.
- **Knob order if the downwash compensation looks too weak:** `ctrlFbl.cd` 0.43 → 0.55 (KNODE on our logs) → 0.6–0.65 (best in simulation; **first flights start at 0.55 (decided 2026-10-22)**), `ctrlFbl.ki_z` 12 → 16 (`ROBUSTNESS_2026-10-21.md`).
- **Waiting for the authors:** their gains/checkpoint become a third preset; nothing else changes.

## Desk-side tests of the lab tools (2026-10-21)
- `fbl_bench_check.py`: dry run against a mocked drone (`flying_drone_stack/tools/test_fbl_bench_check_mock.py`): passes on a correct image, fails on missing log variables, missing controller-9 line, a preset that does not load the gains, and `steps = 0` after a flight. Not run against a real drone or the real cflib.
- `usd_fbl_config.txt`: all 96 variables exist in the firmware sources (scan of the LOG groups); `restore_study_sd_config.py --fbl` tested on a fake card (copy, backup, idempotent).
- FBL uSD layout: a synthetic log with the FBL variables (`make_synthetic_fbl_usd_log.py`, real study log with the 11 variables swapped) decodes, merges with `merge_usd_logs.py`, passes `fbl_log_check.py`, and `pilot_cell_summary.py` now adds `fbl_ran` / `fbl_tripped` columns.
- **Step time on the drone, estimated:** one FBL step (residual + law + command) costs about 1/70 of the NS2-sized network on the host (`c/bench_step.c`, `make bench`), i.e. roughly 10 µs on the drone (NS2 evaluation: 526 µs there); even 10× worse is far below the 2 ms slot. To be confirmed in the lab from the logs.

## Found 2026-10-24 (first FBL lab day): controller-init wipes the FBL yaml values
- `fbl_bench_check.py` printed FAIL on preset/gains at a fresh boot. Two causes: (1) cflib `get_value` returns the value cached at connect time, so reads after the controller switch are stale; (2) at a fresh boot the `ctrlFbl.*` values really are zero until the first switch to controller 9 (`fbl_defaults()` runs inside the Omar controller init only).
- Real risk: CS2 applies the yaml firmware_params in alphabetical order (std::map in `crazyflie_server.cpp`), so `ctrlFbl.*` is set BEFORE `stabilizer.controller 9`; on a freshly powered drone the init then resets `ctrlFbl.en` / `cd` / gains to the defaults, FBL is silently off and the flight is a plain S0 (`steps 0` post-flight). Workaround: `fbl_prime_controller.py` before the launch (drone already on controller 9, the yaml switch is a no-op).
- Proper fix (not done): static initial values for `g_self.fbl` and an init that only resets the runtime state (law, peers, active/tripped), so the config survives the init; needs a rebuilt image, replay gates and a reflash.
- `fbl_bench_check.py` static check still reads cached values (preset / gains FAIL at a fresh boot is expected until cf5 was primed once); the readback after priming shows the truth.

**Resolution 2026-10-24:** firmware fix `fbl_config_survives_init.patch` (static FBL config, init resets runtime state only; gates: replay bit-identical 4/4, host tests, C-in-the-loop identical), image `study_tree_v2_fbl_initfix_candidate.bin`. `fbl_prime_controller.py` and the double launch are only needed with the old estlog image.

## State after the first hardware day (2026-10-24)
- Image `study_tree_v2_fbl_initfix_candidate.bin` on both drones; crazyswarm2 branch `study_FBL` = frozen proposal (xy position loop ×0.7 of preset 1, cd 0.55, preset not written); `study_FBL_kr` (rate loop ×0.5) is worse (2 of 3 flights tripped).
- Found: the FBL tracks the planned horizontal path 6–10× worse than S0–S3 on A4 / A8 (25–31 cm and 18 cm rms against 3–5 cm) although z is the best of all strategies; 3 latch trips in 12 flights (A4 0.125, A8) none with the proposal; details `docs/lab_sessions/2026-10-24.md` §0, `experiments/results/fbl_all_2026-10-24/`. **Before the comparison flights: understand the lateral behaviour (reference / feedforward in the port, residual lateral components, peer feed), then decide whether the gains are frozen.**
