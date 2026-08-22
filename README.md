# Flying Robot — Master's Thesis Repository

This repository originally grew out of the *Advanced Flying Robots* course project and now
serves as the codebase for the Master's thesis:

**Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams**
Georg Wolnik, supervised by Prof. Wolfgang Hönig.

Historical course material — the finished course final report/slides, the Python flight/analysis
scripts (`Controls/`), old course-phase logs and trajectory plots, and the earlier assignment
folders — has been moved to [`archive/`](archive/) and left otherwise unchanged. Nothing was
deleted.

## Start here

| | |
|---|---|
| **Thesis planning docs** | [`docs/`](docs/) — thesis snapshot, literature matrix, gap/contribution statement, residual-wrench model, experimental protocol, references |
| **Active codebase** | [`flying_drone_stack/`](flying_drone_stack/) — Rust controller/estimator/planning stack |
| **Simulation library** | [`bitcraze_rs_lib/`](bitcraze_rs_lib/) |
| **Thesis experiments (new)** | [`experiments/`](experiments/) — logs, analysis, protocols for the ongoing thesis work |
| **Flying a trajectory** | [`docs/08_Trajectory_Upload_Paths.md`](docs/08_Trajectory_Upload_Paths.md) — Mode D vs Mode E, which trajectories are valid on each, how to run them |
| **Archived course project** | [`archive/`](archive/) — see below |

## `archive/` — historical course material

| Folder | Contents |
|---|---|
| `archive/presentations/` | Course final report (`FINAL_REPORT.*`), final/progress presentation decks (`FINAL_PRESENTATION_BEAMER.*`, `PROGRESS_PRESENTATION*`), and `FINALIZE_PROJECT.md` (the course wrap-up plan behind them) — the finished output of the course project, superseded going forward by `docs/` |
| `archive/Controls/` | Python flight/analysis scripts, per-flight logs, tuning notes (`Controls/CLAUDE.md`, `INDI_tuning_guide.md`, etc.) used during the course project |
| `archive/assignments_and_course/` | Earlier-in-course assignment material: `Introduction and Dynamics/`, `Motion Planning/`, `Simulation and Dynamics/`, `State Estimation/`, and `ASSIGNMENTS.md` (the writeup describing them) |
| `archive/old_logs/` | Course-phase `logs/` and `trajectory_plots/` |
| `archive/old_scripts/` | One-off verification scripts (`verify_equivalence.sh`, `verify_numerical.py`) whose paths (`multirotor_simulator/`, `Simulation and Dynamics/`) no longer exist at their original locations |
| `archive/misc/` | `readme_multirotor_simulator_orig.md` — an older README describing a `multirotor_simulator/` crate layout that predates and no longer matches `bitcraze_rs_lib/`'s actual structure |

The course report itself documents the end-to-end pipeline, controllers, and codebase map as they
stood at the end of the course; see `archive/presentations/FINAL_REPORT.pdf` and
`archive/Controls/CLAUDE.md` for that historical detail.

## Branches — work on `main`, never on the preserved ones

Thesis work happens on **`main`**. The branches below are frozen snapshots kept so earlier
results stay reproducible; they must not be committed to, merged into, or rebased.

| Branch | Preserves |
|---|---|
| `finalized-version-for-INDI-project` | The completed INDI project exactly as submitted — report, slides, and the trajectory CSVs as flown |
| `stable-crazyflie-standard` | Standard CF2.1 working baseline |
| `stable-jun30-working` | Clean state before the upgraded/INDI work |
| `stable-june20`, `stable-brushless-full-indi` | Further tuning baselines |
| `brushless-port` | CF21BL brushless support |
| `lab-results-v2` | SD-card logging + raw flight logs (452 files, not merged) |
| `alt-branch` | One-off experiment (local only) |

The same branch names exist in the `crazyswarm2` repo and carry the same meaning.

As of 2026-08-22, thirteen fully-merged branches were deleted locally and on the remote after
confirming every commit was already contained in `main`. Nothing was lost; only redundant
labels were removed.

## Flying a trajectory — two paths

| | **Mode D** (legacy, frozen) | **Mode E** (current standard) |
|---|---|---|
| How | coefficients pushed one float at a time into out-of-tree firmware params | stock Crazyswarm2 `uploadTrajectory` / `startTrajectory` |
| Run | `flight -- --trajectory loop --mode 1 --kt 0.15 --onboard` | `flight -- --trajectory figure8 --mode 1 --kt 0.05` |
| Multi-robot | no | **yes** — `formation_flight` |
| Use for | aggressive manoeuvres (loop, flip, immelmann, splits) and anything with degree-8 content | everything the thesis needs: hover, figure8, circle, oval, tilted_oval |

Mode D is preserved byte-for-byte and is still required for the manoeuvres Mode E cannot
represent. Full details, measurements and the trajectory allowlist:
[`docs/08_Trajectory_Upload_Paths.md`](docs/08_Trajectory_Upload_Paths.md).

## Quick build/test

```bash
cd flying_drone_stack && cargo build --release
cd flying_drone_stack && cargo test              # 479 tests, 0 failures expected
```

## Still at repo root (actively useful)

- `scripts/` — generates a personal study canvas from the live `flying_drone_stack/docs/INDI_STUDY.md`
- `cache/` — cflib radio/param TOC cache used by the flight connection scripts (regenerable, but live)
- `CONTROL_NOTES.md` — running design notes on trajectory scaling, yaw handling, and INDI concepts/tuning — directly relevant to the thesis's INDI work
