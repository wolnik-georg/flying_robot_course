# FBL (Strategy 3) — what is left is lab work (state 2026-10-23: images already flashed, branch `study_FBL`)

Everything below is desk-prepared and untested on hardware. Order is a proposal for you to confirm.

| # | Step | Exact action | Pass criterion |
|---|---|---|---|
| 0 | Pull | `git pull` in both repos; in crazyswarm2 `git fetch origin` (branch `study_FBL` = main + FBL switches + `ctrlFbl.cd 0.55`) | `study_FBL` exists |
| 1 | **No flash needed** | both drones already run `study_tree_v2_fbl_estlog_candidate.bin` (sha256 `4fa0a130…`, flashed 2026-10-23; FBL inside, inert at `ctrlFbl.en 0`; same params and log variables as the old presets image) | `fbl_bench_check.py <uri>` in step 3 passes |
| 2 | SD cards | **archive both cards first** (cf_second at thesis64, cf5 at thesis65 after 2026-10-23; ~19 flights would reach the 85 limit); `restore_study_sd_config.py <cf5 mount> --fbl` on the bottom drone's card (cf_second keeps `usd_study_config.txt`); `check_usd_deck.py` | 96 variables, deck OK |
| 3 | Bench, both drones, CS2 stopped | `python3 flying_drone_stack/tools/fbl_bench_check.py <uri>` (cf5 `radio://0/80/2M/E7E7E7BB02`, cf_second `radio://0/80/2M/E7E7E7E7E9`) | `ALL PASS`: controller 9, preset 1, `en` 0 at boot |
| 3b | **Prime cf5 (needed after EVERY power cycle / battery swap of cf5, CS2 stopped)** | `python3 flying_drone_stack/tools/fbl_prime_controller.py radio://0/80/2M/E7E7E7BB02` | prints `PASS: cf5 is on controller 9 with the FBL defaults in place` |
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
