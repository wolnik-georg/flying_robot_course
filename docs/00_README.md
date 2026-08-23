# Thesis Documentation — start here

**Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams**
Georg Wolnik, supervised by Prof. Wolfgang Hönig.

**If you read one file, read [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md)** —
it is the single source of truth for what is done, where we are, and what comes next. Everything
else is detail hanging off it.

> ### ★ Core Thesis Workflow — the master plan
> **Software preparation is FINISHED. Next actionable phase: C.0 — Hardware Gate.**
>
> | # | Phase | Detail |
> |---|---|---|
> | **1** | **C.0 — Hardware Gate** ⬅️ **next** | [`11`](11_Hardware_Readiness_Checklist.md) |
> | 2 | C.1 — Residual Data Collection | [`10`](10_Formation_Library.md), [`11`](11_Hardware_Readiness_Checklist.md) |
> | 3 | C.2 — Train the Residual Model | [`13`](13_Residual_Learning.md) |
> | 4 | C.3 — Integrate the Strategies | [`13`](13_Residual_Learning.md), [`01`](01_Thesis_Project_Snapshot.md) |
> | 5 | C.4 — Systematic Comparison | [`05`](05_Experimental_Protocol_2Robot.md) |

---

## I want to… → read this

| I want to | Go to |
|---|---|
| Know the project status, what's next | [`07`](07_Thesis_Progress_Checklist.md) |
| **Start the next phase (C.0)** | [`11`](11_Hardware_Readiness_Checklist.md) |
| **Find my way around the three repos** | [`14`](14_Repository_Map.md) |
| Understand the thesis idea and the 7 strategies | [`01`](01_Thesis_Project_Snapshot.md), [`03`](03_Gap_and_Contribution_Statement.md) |
| Understand the physics being measured | [`04`](04_Unified_Residual_Wrench_Model.md) |
| **Fly a trajectory on hardware** | [`08`](08_Trajectory_Upload_Paths.md) — Mode D vs Mode E |
| **Fly a formation** (2–3 robots) | [`10`](10_Formation_Library.md) |
| **Run anything in simulation** | [`09`](09_Simulation.md) |
| Know whether the formations are validated | [`12`](12_Sim_Formation_Validation_Report.md) |
| Work on the learned residual model | [`13`](13_Residual_Learning.md) |
| Prepare for the first real flights | [`11`](11_Hardware_Readiness_Checklist.md) |
| Find a paper | [`02`](02_Literature_Matrix.md), [`06`](06_References_Overview.md) |
| Know the 2-robot experiment protocol | [`05`](05_Experimental_Protocol_2Robot.md) |

---

## The documents

### Planning and science
| | |
|---|---|
| [`01_Thesis_Project_Snapshot.md`](01_Thesis_Project_Snapshot.md) | The idea, the 7 control strategies, timeline, status |
| [`02_Literature_Matrix.md`](02_Literature_Matrix.md) | Structured overview of the relevant papers |
| [`03_Gap_and_Contribution_Statement.md`](03_Gap_and_Contribution_Statement.md) | Gap analysis and contribution claim |
| [`04_Unified_Residual_Wrench_Model.md`](04_Unified_Residual_Wrench_Model.md) | The residual-force model — the maths behind `indi.a_res_*` |
| [`05_Experimental_Protocol_2Robot.md`](05_Experimental_Protocol_2Robot.md) | Protocol for the 2-robot comparison |
| [`06_References_Overview.md`](06_References_Overview.md) | Every paper, with reading status |

### Status
| | |
|---|---|
| [`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md) | **Single source of truth.** Update it in the same change that alters state |

### Engineering reference — how the system actually works
| | |
|---|---|
| [`08_Trajectory_Upload_Paths.md`](08_Trajectory_Upload_Paths.md) | Mode D (onboard params) vs Mode E (stock Crazyswarm2 upload). Which trajectories are valid on each, how to run them |
| [`09_Simulation.md`](09_Simulation.md) | Running the thesis controllers in the CS2 simulator with the downwash model — and what the simulator can and cannot show |
| [`10_Formation_Library.md`](10_Formation_Library.md) | The 16 formation scenarios (A1–A8, B1–B3, C1–C5), how to run them, coverage gaps |
| [`13_Residual_Learning.md`](13_Residual_Learning.md) | The learned residual model 5 of the 7 strategies share: onboard network, weight upload, training pipeline, sim dry run |

### Results and checklists
| | |
|---|---|
| [`11_Hardware_Readiness_Checklist.md`](11_Hardware_Readiness_Checklist.md) | Checklists A/B/C: inventory, C.0 acceptance, freeze-the-gains |
| [`12_Sim_Formation_Validation_Report.md`](12_Sim_Formation_Validation_Report.md) | 34 sim cases, 33 pass — and the two flight-code bugs it found |

### Orientation
| | |
|---|---|
| [`14_Repository_Map.md`](14_Repository_Map.md) | The three repositories, the end-to-end signal flow, which file to open for a given task, and the traps that have cost this project time |

---

## Where the code lives

| | |
|---|---|
| **Rust stack** (planning, controllers, offline analysis) | `flying_drone_stack/` — see its [`CLAUDE.md`](../flying_drone_stack/CLAUDE.md) |
| **Onboard controller** (`no_std`, runs at 500 Hz on the drone) | `flying_drone_stack/firmware_app/` — see its [`CLAUDE.md`](../flying_drone_stack/firmware_app/CLAUDE.md) |
| **Residual training pipeline** | `flying_drone_stack/tools/residual/` — see its [`README.md`](../flying_drone_stack/tools/residual/README.md) |
| **Flight + formation scripts** (ROS 2) | `~/Desktop/crazyswarm2/crazyflie_examples/` |
| **Simulator** | `~/Desktop/crazyswarm2/crazyflie_sim/` |
| **Firmware** (bitcraze upstream, deliberately not forked) | `~/Desktop/crazyflie-firmware/` — local changes listed in [`LOCAL_MODIFICATIONS.md`](../flying_drone_stack/firmware_app/host/LOCAL_MODIFICATIONS.md) |
| **Experiments** (logs, analysis scripts, results) | `experiments/` — see its [`README.md`](../experiments/README.md) |

---

## Conventions

- **Numbering:** `01`–`06` planning, `07` status, `08`–`10` + `13` engineering reference,
  `11`–`12` checklists and results, `14` orientation. New documents get the next number.
- **Update in the same change that alters state**, not afterwards. A doc that lags reality is
  worse than no doc, because it is trusted.
- **Never tick a box for something that has not been verified.** Code that compiles is not code
  that flew. Several things in this project are marked *built, unflown* on purpose.
