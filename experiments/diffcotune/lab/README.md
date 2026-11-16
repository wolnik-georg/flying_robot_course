# lab/: DiffCoTune S0 desk tools (Stage 4a / 4b, nothing flown)

Python: `/usr/bin/python3` (3.10; the frsim firmware build is for 3.10). Plots: `~/.pyenv/versions/flying_robots/bin/python`.

| File | Purpose |
|---|---|
| `round.py` | `python round.py --controller S0 --round <l> --logs <folder> [--top auto\|logged\|planned]`; `--init` writes round 0. Ingest, gate, fit β (not in the last round), tune θ (A + B), writes `S0/round_<l>/report.{json,md}`, `S0/round_<l+1>/{state.json,gains.yaml}` (last round `gains_theta_best.yaml`) |
| `../usd_ingest.py` | merged uSD file -> model trace, quality gate, frame and lag estimate, reference check |
| `lab_pool.py` | worker pool with the logged top-drone path |
| `fake_usd.py`, `dry_test.py` | frsim flights written as fake merged uSD files; `dry_test.py [--reuse]` compares one round with `loop_lib`. Stage 4b: `dry_test.py --from-round 1` keeps `dry/` and runs round 1 only |
| `dry_round1_strict.py` | Stage 4b: round 1 once more with `strict=True` on a copy (`dry_strict/`); run after `dry_test.py`, never at the same time (one worker pool at a time, about 6 GB) |
| `gyro_check*.py`, `attitude_check.py`, `tilt_abort_check.py` | Stage 4b: gyro / attitude check of the 23 Oct flights (sign, axes, frequency, phase) and real tilt against the abort rule (scored part only). `attitude_check.py [--top auto\|logged\|planned]` |
| `test_ingest_oct23.py`, `lag_a8.py`, `test_frame_fig8.py` | tests on the 23 Oct S0 flights and the turned figure 8 |
| `plot_protocol.py`, `figs/` | figures of the protocol draft |

`<folder>` holds merged uSD files and `flights.json`: list of `{file, scenario, cell, role (tuning|test), attempt, rotate_deg, lag_s}`.
Results: `results/`. Protocol draft: `docs/diffcotune_lab_protocol_S0_DRAFT.md`.

## Stage 4b (13 Nov 2026)
Dry test round 1 passed the gate (θ₂ and β identical to `loop_P1.json`, real cost 9.713 cm). The `strict=True` run gives the same θ₂ (difference 0.0); no failed flight in that round, so the repeat path is not exercised by it. Gyro, attitude and abort-rule findings and the P2 stall: `../dryrun/stage4b/report_stage4b.md`. The first Stage 4b run was killed by systemd-oomd at 14:52:53 because two jobs ran two worker pools at once; since then one pool at a time (`../dryrun/stage3c/job_runner.py`).

## Stage 4c (13 Nov 2026, proposals, desk only)
| File | Purpose |
|---|---|
| `cmd_check.py` | lab commands for the cells, checked against the frsim reference (`results/cmd_check.json`) |
| `apply_gains.py` | `merge` (gain file into the cf5 block of `crazyflies.yaml`, verified), `readback` / `set` (cflib, fresh connection; not tested on hardware) |
| `fly_file.py` | flies `../flightfiles/*.csv` through `simple_flight.py` without copying; `--check` offline (pieces, path about the hover point = file − file(0), pad window, z, distance to the cf_second pad); CIRCLE (turned) and FIG8; OVAL refused; Stage 4d corrected; not tested with ROS |

Report: `../dryrun/stage4c/report_stage4c.md`.

## Stage 4d (13 Nov 2026)
| File | Purpose |
|---|---|
| `circle_turn_choice.py` | +90° or −90° for the circle at the cf5 pad; pad windows of all files (path about the hover point) |
| `test_frame_circle.py` | frame test of the turned circle: the fit alone is degenerate (rotation = time shift), `rotate_deg = -90` must be given |

Report: `../dryrun/stage4d/report_stage4d.md`. Flight files: `../flightfiles/` (`checks.json` has `lab_use` and the pad windows).

## Stage 5 (14 Nov 2026): S1, S2, S3
| File | Purpose |
|---|---|
| `round.py --controller S0\|S1\|S2\|S3` | the round with the model of the controller (default S0, behaviour identical to Stage 4; `--n-rounds` default 5 since Stage 5b; `--grad-mode reverse|forward`, default reverse for all four controllers); state in `lab/<controller>` |
| `apply_gains.py` | `merge --strategy`, `readback`, `set --strategy` check `stabilizer.controller` = 9 and the switches (S0 0/0, S1 3/0, S2 0/1, S3 3/1) |
| `preflight.py --controller S0..S3` | every offline check (`results/preflight_<C>.txt`, all four PASSED) |
| `dry_test5.py <C>` | round-0 dry test against `loop_lib` (θ₁ difference 0.0 for S0–S3; `results/dry_test5_<C>.json`) |
| `test_apply_gains_mock.py` | readback, set, switch checks and the yaml merges of main / study_S1 / study_S2 / study_S3 against a fake cflib (not ROS, not hardware) |
| `flights_template.py` | `flights.json` template: 4 tuning + 6 test flights |
| `S0_LAB_CARD.md` | one page for the first S0 block, a source on each line |

**Status:** S1 and S3 are not lab-ready on the desk side (S1 A1 gradient explosion, open); S2 is open. Report: `../dryrun/stage5/report_stage5.md`. Protocol: `docs/diffcotune_lab_protocol_S0_DRAFT.md`, section "S1–S3".

## Stage 5b (14 Nov 2026)
`round.py`: `--n-rounds` default 5 (was 6), `--grad-mode reverse|forward` (default per controller in `GRAD_MODE_DEFAULT`, all `reverse`); `preflight.py` checks both. Status unchanged: S0 lab-ready; S1, S2, S3 not (exact A1 gradient too slow so far). Report: `../dryrun/stage5b/report_stage5b.md`.

## Stage 5c (15 Nov 2026) and "Stage 5c, RPM delay fixed (user, 14 Nov)"
`round.py`: `--grad-mode reverse|forward|mixed` (default table: S0 reverse, S1 / S2 / S3 mixed), `--fix-rd` / `--fit-rd` (default: S1, S3 RPM delay fixed at 4.0502 ms). `lab_pool.worker_rollout_top` uses the compiled forward sensitivity for the mixed A1 fit (otherwise the round would not equal the loop). Status (desk side): S0, S1, S2 lab-ready; S3 not (flat start). Round time: S0 about 20, S1 22.5, S2 25.9, S3 28.8 min.

## Stage 6 (15 Nov 2026, proposed)
`session.py` (`status` / `switch` / `manifest` / `desk` / `cards`), `session_timeline.py`, `INTERLEAVED_LAB_CARD.md` (every item "Stage 6, proposed"), mock test `test_session_mock.py` (options `--fake-frame`, `--lenient`: tests only). `round.py` defaults unchanged by Stage 6 (no speed-up passed the identity gate).

## Stage 7 and Stage 8 (15 Nov 2026)
`round.py`: tuning set of 5 scenarios (A4 cell 1 is a tuning flight, A4 cell 2 the held-out test cell), gradient `cfwd`, `--method paper|ours` (default paper, 6 rounds). `preflight.py` item 7 (NS2 weights hash and yaml switches) and a method-aware settings check; `test_preflight_ns2.py`. Lab cards `S0_LAB_CARD.md` and `INTERLEAVED_LAB_CARD.md` updated for 6 rounds and 5 flights per round. `session.py` / `flights_template.py` / `fake_round_flights.py` / `test_session_mock.py` follow the 5 flights.

## Stage 8b (15 Nov 2026, evening)
`round.py` defaults to `--method ours` (frozen lab variant); after the tuning step it runs the safety preview (`preview.py`, `--no-preview` to skip) and writes `round_<l+1>/PREVIEW.txt`; `preflight.py` item 4b fails unless the frozen variant is active. Cards `S0_LAB_CARD.md` and `INTERLEAVED_LAB_CARD.md`: preview line, `--method ours`, uSD rule (empty both cards, archive once after round 2 of both controllers).
