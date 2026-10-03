# Cursor prompt — two last text fixes in the lab sheet (2026-10-03) — then desk is closed

Repo `~/Desktop/flying_robot_course`. Touch only `docs/lab_bench_cheatsheet_2026-10-03.md`; append "Appendix H" (3–6 lines, append only) to `docs/lab_sessions/2026-10-03.md`. Same constraints (never fly/flash/commit/push, no `Co-Authored-By`, don't edit `docs/07`).

## Context (Claude verified your last round by running it: mock capture exits 0/2/1, `ctrltarget_z` one-liner 0.5/1.0, merge refuses with RMS 90.7 cm — all accepted)

## Fixes
1. **Step C uses the wrong `run_formation` command.** The sheet says `ros2 run crazyflie_examples run_formation --scenario A8 --auto-center --yes`. The A8 flights on 10-03 (and the meta `experiments/logs/A8_2026-10-03_13-15-00.meta.json`: `dz 0.5, span 1.0, passes 4, height 0.5`) were flown with **`--dz 0.5 --height 0.5 --passes 4 --auto-center --yes`**. `--height` defaults to **1.0** (see `run_formation.py` argparse), which would put the top drone at 1.5 m — risk of the mocap ceiling/geofence. Use the exact 10-03 flags in Step C, and say "do not drop `--height 0.5`". Do the same wherever the sheet runs a scenario.
2. **Step D has no flight command.** Add the exact command and scenario for the `ki_z` A/B (use the A1 invocation documented in `docs/lab_prep_geometric_kiz0_test.patch` header / `docs/ns2_next_lab_protocol.md`; if no exact flags are documented, grep `docs/lab_sessions/2026-10-0*.md` for the A1 command that was actually flown and reuse it verbatim, stating its source). Put it after the "confirm `ki_z`" line, with the pass criterion.

## Deliverable / reporting
Edited sheet + Appendix H (what changed, source of the exact flags with file:line). Confidence per claim; "could not verify" for anything not checked by running/grep.
