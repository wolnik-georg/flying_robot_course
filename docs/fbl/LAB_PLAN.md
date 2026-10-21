# FBL (Strategy 3) — what is left is lab work (state 2026-10-20 night)

Everything below is desk-prepared and untested on hardware. Order is a proposal for you to confirm.

| # | Step | Exact action | Pass criterion |
|---|---|---|---|
| 0 | Pull | `git pull` in both repos (`flying_robot_course`, `crazyswarm2`) | files below exist |
| 1 | Flash both drones | `flying_drone_stack/firmware_app/build_artifacts/study_tree_v2_fbl_candidate_presets.bin` (local file, sha256 `956479a7…`; rebuild recipe `PROGRESS_AND_RESUME.md` §6) | flash OK |
| 2 | SD cards | count `thesis` files on both cards (archive at ≥ 85); `restore_study_sd_config.py <mount> --fbl` on the bottom drone's card (cf_second keeps `usd_study_config.txt`); `check_usd_deck.py` | 96 variables, deck OK |
| 3 | Bench, both drones, CS2 stopped | `python3 flying_drone_stack/tools/fbl_bench_check.py <uri>` (cf5 `radio://0/80/2M/E7E7E7BB02`, cf_second `radio://0/80/2M/E7E7E7E7E9`) | `ALL PASS`: controller 9, preset 1, `en` 0 at boot |
| 4 | Yaml | in crazyswarm2: `git apply crazyflie/config/study/from_main_FBL.patch`, then `python3 flying_drone_stack/tools/lab_check_strategy.py` | prints `FBL (Strategy 3, S4)`; cf_second stays S0 |
| 5 | Launch + check | `ros2 launch crazyflie launch.py`; `fbl_bench_check.py <cf5 uri> --readback` | `FBL ON … gains: preset 1 (ours)` |
| 6 | First FBL flight | A1, dz 0.5, hold 15 s: `ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.5 --hold 15.0 --height 0.5 --auto-center --yes`; hand ready on the abort | takes off, holds, lands |
| 7 | Verify it ran | stop CS2: `fbl_bench_check.py <cf5 uri> --post-flight`; later `experiments/analysis/fbl_log_check.py <log>` | `steps > 0`, `faults 0` (`tripped 1` = latch fell back to Omar) |
| 8 | Then | A1 repeats, A8 (`--scenario A8 --dz 0.5 --height 0.5 --passes 4`), A4 offsets 0.05/0.10/0.15; compare in the same tables as S0–S3 | per-cell n like the pilot |

- **If the drone sags or oscillates in the first flights:** the knobs are `ctrlFbl.ki_z` (default 12), `ctrlFbl.cd` (0.43, downwash strength, 0 = no residual) and `ctrlFbl.preset`; all settable per flight without reflashing.
- **First uSD FBL log is also the shakedown of the logging chain:** decode + merge have never seen an FBL config (11 INDI/oot5 variables swapped for `ctrlFbl.*`); if a downstream script misses a column, fix it then.
- **Expected from simulation (optimistic):** in the calibrated plant (measured thrust, downwash 1.3×) A8 dip about 1 cm, A1 bias about 0.2 cm (RMS 3 cm), A4 2–3 cm RMS (`SIM_COMPARISON_2026-10-20.md`); real values will be worse.
- **Knob order if the downwash compensation looks too weak:** `ctrlFbl.cd` 0.43 → 0.6 (best in simulation), `ctrlFbl.ki_z` 12 → 16 (`ROBUSTNESS_2026-10-21.md`).
- **Waiting for the authors:** their gains/checkpoint become a third preset; nothing else changes.
