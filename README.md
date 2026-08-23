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

**Read [`docs/07_Thesis_Progress_Checklist.md`](docs/07_Thesis_Progress_Checklist.md) first** — the
single source of truth for what is done and what comes next. [`docs/00_README.md`](docs/00_README.md)
is the map of every other document.

> ### ⇄ Two parallel tracks
>
> **🔬 Lab track** — software preparation is FINISHED; blocked only on lab access.
> **C.0** Hardware Gate ⬅️ next → **C.1** Residual Data Collection → **C.2** Train the Residual
> Model → **C.3** Integrate the Strategies → **C.4** Systematic Comparison
>
> **✍️ Writing track** — blocked by nothing. W.1 problem statement ✅ · W.2 related-work structure ✅
> · W.3 strategy descriptions ✅ · W.4 contribution statement ✅ · W.2b paper library ✅ · W.2c all 15
> read at claim level ✅ · W.5 Ch. 2 ✅ · W.6 Ch. 3 ✅ · W.8 Ch. 4 ✅ · W.7 Ch. 5 ✅ · **W.9 Ch. 1 ✅ — all lab-unblocked chapters drafted (35 pp, ~12,600 w)** · next: close 7 verification items
>
> Full plan: [`docs/07_Thesis_Progress_Checklist.md`](docs/07_Thesis_Progress_Checklist.md)


| I want to | Go to |
|---|---|
| Know where the project stands | [`docs/07`](docs/07_Thesis_Progress_Checklist.md) |
| Find any document | [`docs/00`](docs/00_README.md) |
| Find my way around the three repos | [`docs/14`](docs/14_Repository_Map.md) |
| Know what the thesis is asking | [`docs/15`](docs/15_Problem_Statement_and_Research_Questions.md) |
| Write the Related Work chapter | [`docs/16`](docs/16_Related_Work_Structure.md) |
| Cite a paper — check first | [`docs/17`](docs/17_Source_Ledger_and_Citation_Discipline.md), quote from [`docs/20`](docs/20_Verified_Claims.md) |
| See what the 7 strategies do differently | [`docs/18`](docs/18_Strategy_Descriptions.md) |
| Fly a single-robot trajectory | [`docs/08`](docs/08_Trajectory_Upload_Paths.md) |
| Fly a formation (2–3 robots) | [`docs/10`](docs/10_Formation_Library.md) |
| Run something in simulation | [`docs/09`](docs/09_Simulation.md) |
| Work on the learned residual model | [`docs/13`](docs/13_Residual_Learning.md) |
| Prepare for the first real flights | [`docs/11`](docs/11_Hardware_Readiness_Checklist.md) |

### Where the code is

| | |
|---|---|
| **Rust stack** — planning, controllers, offline analysis | [`flying_drone_stack/`](flying_drone_stack/) |
| **Onboard controller** — `no_std`, 500 Hz on the drone | [`flying_drone_stack/firmware_app/`](flying_drone_stack/firmware_app/) |
| **Residual training pipeline** | [`flying_drone_stack/tools/residual/`](flying_drone_stack/tools/residual/) |
| **Experiments** — logs, analysis scripts, results | [`experiments/`](experiments/) |
| **Flight + formation scripts, simulator** | `~/Desktop/crazyswarm2/` (separate repo, also ours) |
| **Firmware** | `~/Desktop/crazyflie-firmware/` (bitcraze upstream — deliberately **not** forked; local changes are listed in [`LOCAL_MODIFICATIONS.md`](flying_drone_stack/firmware_app/host/LOCAL_MODIFICATIONS.md)) |
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

## Quick build / test

```bash
# Rust stack
cd flying_drone_stack && cargo build --release
cd flying_drone_stack && cargo test                    # 479 tests, 0 failures expected

# Onboard firmware (brushless CF21BL is the default target)
cd flying_drone_stack/firmware_app && make             # build; `make cload` flashes

# Onboard residual network, checked against an independent reference
cd flying_drone_stack/firmware_app && python3 host/test_residual_nn.py

# Residual training pipeline, checked end to end against the compiled controller
cd flying_drone_stack/tools/residual && python3 test_pipeline.py

# Formation scenario library: spec check, no hardware or ROS needed
cd ~/Desktop/crazyswarm2/crazyflie_examples/crazyflie_examples \
   && python3 -m formations.scenarios --self-test
```

**Use system `python3`** for anything touching torch or the simulator bindings. The pyenv
`flying_robots` environment has neither; it is for the older cflib flight scripts.

## Still at repo root (actively useful)

- `scripts/` — generates a personal study canvas from the live `flying_drone_stack/docs/INDI_STUDY.md`
- `cache/` — cflib radio/param TOC cache used by the flight connection scripts (regenerable, but live)
- `CONTROL_NOTES.md` — running design notes on trajectory scaling, yaw handling, and INDI concepts/tuning — directly relevant to the thesis's INDI work
