# DiffCoTune for FBL, Stage F2b: answers to the stop, A1 crash under noise, which firmware the drones run (2026-11-18)

Paste everything below the line into the working agent (F1/F2 rules still apply unless this prompt changes them).

---

You are the **working agent**. Facts only from files, with paths; "not stated" otherwise. Never invent numbers or rules. Same isolation as F1/F2 (worktree `~/Desktop/flying_robot_course_fbl`, branch `feature/diffcotune-fbl`, lab checkout untouched, no push, no Co-Authored-By). Lab guard as in F2: today is a lab day.

## 0. What the planning agent read (your stop at 06:22 was right)

- Regressions `regressF2_S0` / `regressF2_S2`: 0.0, accepted. The F2 desk part (`d15b35e3`) and D4 (`fbl/D4_lab_rate.md`: lab = 500 Hz) are accepted.
- **The main finding is not the gate. It is this:** in `fbl/noise_forward_check_fbl.log`, **every A1 cell crashes with sensor noise**, in frsim itself and in the model:
  - all 3 variants, all 3 gain sets (preset 1, gains1, gains2), both seeds (18/18 cells, crash flag True, 3-D RMS 30–160 cm);
  - without noise, A1 is fine (3-D RMS about 0.18 cm);
  - A8, CIRCLE, FIG8 and A4 with noise mostly do not crash;
  - all noisy forward-check failures and the `gateF_base` failure (`J_task_noise` A1: J 0.49, gradient 4.7e10) are in these crashing A1 cells.
- So the model copies frsim correctly, but **in frsim, FBL with sensor noise cannot fly A1 at θ_0.** A dry run on that basis would have A1 as R3 in every round, with a meaningless A1 gradient. Running dry runs before this is understood would waste nights.

## 1. Answers to your two questions

1. **Gate:** no exception and no gate change for now. The `gateF_base` result stays FAIL. Task F2b.1 decides what it means.
2. **Memory and queue** (resource and tooling settings, not method):
   - `mk_f2.py` / job files: **3 pool workers** for FBL;
   - `f2.sh`: check `MemAvailable ≥ 3 GB` before every job, and every 5 min during a job. Below that, stop the job by its exact PIDs and log it;
   - **a FAIL of any gate or regression stops the queue** (it does not go on to the next job);
   - **do not start the queue.** It waits for the planning agent after F2b.

## 2. Tasks (all light, single process, allowed on lab days; one core, `nice -n 19`, `OMP_NUM_THREADS=1`)

### F2b.1 Why FBL crashes on A1 with noise (frsim, then model)
- For base, preset 1, seeds 1000/1001, plants calibration and P1:
  - which drone crashes (top or bottom);
  - time of onset;
  - tilt, rate and thrust history;
  - plot `fbl/figs/a1_noise_crash_*.png`.
- **Which noise does it?** Switch on one sensor group at a time (position, velocity, gyro: frsim `sensors.py`, FBL path), each at full level, then the levels ×0.5 and ×0.25. Report the crash yes/no table.
- **Same noisy A1 with S0** (same seeds, same plant; frsim): does S0 survive? (The S0–S3 dry runs say yes; show it on these seeds.)
- Where in FBL does the noise enter, and is it amplified? For example the α-β-γ estimator (`beta`, `gamma`), velocity used directly, the jerk term, or the integrator. Cite file:line. Report it, do not fix it.
- **Is frsim's noise for FBL right?** Compare the noise levels frsim applies to FBL with those it applies to the tree, and both with real uSD data (position / velocity / gyro noise in hover, from existing S0 uSD logs). Is the FBL sensor path in frsim the same signal as on the drone (EKF state vs raw)? Facts with paths.

### F2b.2 Real FBL A1 flights (read only, uSD only, never radio CSVs)
- List every FBL flight on hardware (24 Oct and later: `docs/fbl/`, the lab session docs, the uSD archives): scenario, preset, `inner`, `cd`, outcome (crash, trip, tilt oscillation, clean), files.
- For the A1 flights: tilt RMS / max, and whether the oscillation looks like the frsim noisy A1 run (frequency, onset).
- If no complete uSD log of an FBL A1 flight exists, say so.

### F2b.3 Which firmware the drones' FBL image was built from (read only, build nothing)
D4 found that the `fbl_v2b` source (25 Oct: hand-over, tilt latch, rate limit) and the simulator build (4 Nov, authors-aligned) differ by 116 lines. The model follows the simulator build. Find out:
- the image named in `docs/fbl/LAB_PLAN.md:10` (sha256 `e3593a6f…`): its build date and source tree;
- whether an image built from the 4 Nov authors-aligned source exists (`flying_drone_stack/firmware_app/build_artifacts/`, `~/Desktop/fbl_v2_firmware`, `fbl_v2b_firmware`, the docs `EQUIVALENCE_CHECK_2026-11-02.md`, `HANDOVER_FBL_AUTHORS_ALIGNMENT_2026-11-04.md`; plan line 124 "flash fbl_v2b"): path, sha256, date, what it was checked against ("199/199");
- what is on the drones now: not stated unless a file says it.

Result: "image X matches the simulator build" or "no image matches yet; one would have to be built from <source>". **Do not build or flash anything.**

### F2b.4 Report `experiments/diffcotune/fbl/report_F2b.md` plus a commit on the branch
Sections F2b.1–F2b.3, the changed `mk_f2.py` / `f2.sh` (diff), and the open decisions with options. For example:
- if frsim's FBL noise is too high: correct it in the worktree simulator copy, as a patch file;
- if FBL is truly noise-sensitive on A1: smallest documented change of θ_0 under the starting-gain rule, or A1 stays and R3 handles it;
- if no matching image exists: build one.

Decide none of these. Then tell the user "Stage F2b done: experiments/diffcotune/fbl/report_F2b.md" and wait.
