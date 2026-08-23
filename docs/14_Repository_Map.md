# Repository Map — where everything is and how it fits together

**Last updated:** 23 August 2026

Three repositories on this machine make one system. This document says which is which, what may
be committed where, and which file to open for a given task. Status lives in
[`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md); this is geography.

> **★ Core Thesis Workflow:** **C.0** Hardware Gate ⬅️ next → **C.1** Residual Data Collection →
> **C.2** Train the Residual Model → **C.3** Integrate the Strategies → **C.4** Systematic
> Comparison. Software preparation is **finished**; the project is waiting on lab access.

---

## 1. The three repositories

| Path | What it is | Commit & push? |
|---|---|---|
| `~/Desktop/flying_robot_course` | **This repo.** Rust stack, onboard controller, training pipeline, thesis docs, experiments | **Yes** |
| `~/Desktop/crazyswarm2` | ROS 2 flight layer: flight scripts, formation library, simulator, robot configs | **Yes** |
| `~/Desktop/crazyflie-firmware` | Bitcraze upstream firmware | **No — deliberately not forked.** Local edits stay local and are recorded in [`LOCAL_MODIFICATIONS.md`](../flying_drone_stack/firmware_app/host/LOCAL_MODIFICATIONS.md) |

All work is on **`main`** in the two repos we own. Everything was merged there on 2026-08-22 and
thirteen stale branches were deleted — notes pointing at `brushless-port` or
`tune-indi-on-crazyflie-thrust-upgraded` are out of date.

⚠️ The `crazyflie-firmware` edits are **uncommitted by design**, which means a `git checkout`,
`stash` or upstream pull in that tree destroys them. Four files are affected; the most dangerous
to lose is `usddeck.c`, whose raised log-variable limit (20 → 40) the 35-variable thesis logging
config depends on. Below the stock limit the log is **silently truncated** — no error, just
missing columns.

---

## 2. Signal flow, end to end

```
                    flying_drone_stack (Rust)
    trajectory planning ──► export_poly4d ──► Poly4D CSV
                                                  │
                    crazyswarm2 (ROS 2)           ▼
    OptiTrack ──► motion_capture_tracking ──► crazyflie_server ──► uploadTrajectory
                                                  │                      │
                                                  ▼                      ▼
                                            send_extpose            startTrajectory
                                                  │                      │
                    ══════════ radio ═════════════╪══════════════════════╪══════════
                                                  ▼                      ▼
                    on the drone            Kalman/EKF  ────────►  HLC @ 1 kHz
                                                                         │
                                        firmware_app (our Rust, 500 Hz)  ▼
                                        geometric SE(3) / INDI  ◄─── setpoint
                                                  │
                                                  ├──► indi.a_res_*  (the thesis signal)
                                                  ├──► residual_nn   (predicted a_res)
                                                  └──► motors
                                                  │
                    logging                       ▼
                    uSD 500 Hz × 35 vars (the dataset)  +  radio 100 Hz (monitoring only)
                                                  │
                    back on the laptop            ▼
                    merge_usd_logs ──► tools/residual ──► weights ──► upload_residual_weights
```

**The same `firmware_app` source also compiles for x86_64** and links into the Crazyswarm2
simulator, so simulation runs the controller that flies, not a re-implementation.

---

## 3. Which file do I open?

### To fly something

| Task | File / command |
|---|---|
| Single-robot trajectory | `ros2 run crazyflie_examples flight -- ...` → [`08`](08_Trajectory_Upload_Paths.md) |
| Formation, 2–3 robots | `ros2 run crazyflie_examples run_formation -- ...` → [`10`](10_Formation_Library.md) |
| Check a formation without flying | `run_formation -- --scenario A3 --dz 0.30 --check` |
| Anything in simulation | [`09`](09_Simulation.md) |

### To change control behaviour

| Task | File |
|---|---|
| The control laws themselves | `flying_drone_stack/firmware_app/src/lib.rs` |
| Which law runs | `indi_gains.ctrl_mode` in `crazyswarm2/crazyflie/config/crazyflies.yaml` (runtime; no reflash) |
| Gains | same yaml — pushed over CRTP at connect, so **the yaml wins over firmware defaults** |
| The learned residual model | `flying_drone_stack/firmware_app/src/residual_nn.rs` → [`13`](13_Residual_Learning.md) |

### To work with data

| Task | File |
|---|---|
| Decode a uSD log | `flying_drone_stack/tools/decode_usd_log.py` |
| Merge per-drone uSD logs onto one clock | `flying_drone_stack/tools/merge_usd_logs.py` |
| Train a residual model | `flying_drone_stack/tools/residual/train.py` |
| Score a simulated formation run | `experiments/analysis/verify_formation_sim.py` |
| Everything else | [`experiments/README.md`](../experiments/README.md) |

### Instruction files for working in a directory

`CLAUDE.md` files carry the local rules and gotchas:
`flying_drone_stack/`, `flying_drone_stack/firmware_app/`, `flying_drone_stack/src/*/`,
`crazyswarm2/crazyflie_examples/`.

⚠️ **The root `.gitignore` excludes `**/CLAUDE.md`, so none of them is tracked in this repo.**
That appears deliberate, but the consequence is worth knowing: these files exist **only on this
machine**. They are not in any clone, not on the remote, and not in any backup that follows git.
If they should survive a disk failure or reach the lab machine, the rule has to change.

---

## 4. Things that have bitten this project

Kept here because each one cost real time and none is obvious from the code.

| Trap | What happens |
|---|---|
| **`a_res` reads exactly 0.0** | No RPM source reached the controller. Not "no interaction" — *no measurement*. A dataset of zeros trains a model that predicts nothing and looks converged |
| **Poly4D CSV written with `%.6f`** | 0.21 m/s² became 241 m/s² and crashed a drone. Always `%.12g`; `poly4d.verify_csv()` guards it |
| **`firmware_params` not applied in sim** | The controller silently runs on compile-time defaults. Check the startup line `applied N firmware_params` before blaming a control law |
| **Shared controller state across simulated drones** | One `State` for N vehicles destroys every filter and integrator. Geometric looks fine (near-memoryless); INDI never leaves the ground |
| **Scoring a run against the wrong sidecar** | Taking the newest `*.meta.json` pairs a flight with the *previous* scenario's geometry if a run died. Use a pre-run marker |
| **Unmapped uSD channel names** | `merge_usd_logs.py` guards lookups with `if key in dict`, so an unmapped channel makes the merge **succeed while producing nothing** |
| **A blanket `*.md` gitignore** | Hid 14 documents in `flying_drone_stack/` from git, including two whole investigations. Removed 2026-08-23. The separate root rule for `CLAUDE.md` still stands |

---

## 5. Which Python

| Use | Interpreter |
|---|---|
| torch, simulator bindings, residual pipeline, SIL tests | **system `python3`** (3.10) |
| Older cflib flight/analysis scripts | `~/.pyenv/versions/flying_robots/bin/python` |

The pyenv environment has neither torch nor `cffirmware`; the system one has both. Getting this
wrong produces a `ModuleNotFoundError`, which is at least loud.

---

## 6. Standing rules

- **Commit and push only when asked.** Never add a `Co-Authored-By:` or AI-attribution trailer.
- **Never delete files** — move them (`archive/` exists for this).
- **Do not fork `crazyflie-firmware`.**
- **Brushless attitude gains (KR/KW): changeable ONLY inside C.0, then frozen.** The original
  rule was a blanket "do not change"; it was scoped on 2026-08-23 — an attitude retune to remove
  the remaining oscillation is explicitly allowed during the Hardware Gate. **After the freeze
  (Checklist C) no gain may change for the rest of the campaign**, or the comparison measures
  tuning effort instead of the methods.
- **Do not start the RL policy networks** (strategies 5 and 7) — deliberately deferred.
- **Never tick a checklist box for something unverified.** Code that compiles is not code that
  flew; several things here are marked *built, unflown* on purpose.
