# DiffCoTune for FBL, Stage F1: isolated branch and folder, inventory, differentiable FBL model, forward and gradient checks (desk only, 2026-11-17)

Paste everything below the line into the working agent.

---

You are the **working agent**. The planning agent and the user plan and validate; you execute. Facts only from files, with paths; "not stated" / "not yet" otherwise. Never invent numbers or rules.

## 0. Context

- Thesis: comparative study of quadrotor controllers. Before the study, each controller gets one gain set from the same auto-tuning method: **DiffCoTune's** Split-Alternate loop (Krishna et al. 2025) with **DiffTune's** quadrotor settings (Cheng et al. 2024), frozen lab variant "ours". Definition: `docs/diffcotune_spec.md` §24; flight rules R3 / RS and the starting-gain rule: `docs/diffcotune_lab_protocol_S0_DRAFT.md`, last section.
- S0–S3 (Omar C tree) are **finished and lab-ready** on branch `feature/auto-tuning` (commits `6e4f663c`, `db052851`). The user flies them in the lab from **18 Nov**, on this laptop, from `~/Desktop/flying_robot_course`. **That checkout and everything it uses must not change.**
- The plan (`docs/plan_diffcotune_2026-11-11.md`, line 13) has **4 FBL controllers**, each tuned on its own with the same method: FBL base, FBL + downwash model (ROM), FBL + learned part (net), FBL + both.
- FBL starting gains θ_0 (decided, user 17 Nov): **preset 1** = `FBL_DEFAULT_GAINS` in `simulator/frsim/controllers.py:36`: kp_xy 14.0, kv_xy 9.5, ki_xy 12.0, kp_z 11.25, kv_z 5.4, ki_z 12.0; `FBL_FIXED` kr_xy 59.5, kr_z 35.0 are not tuned.
- This stage: isolation, inventory, the differentiable FBL model, its forward check and gradient check. **Not** this stage: wiring into `loop_lib.py` / `round.py`, dry runs, lab cards, flashing.

## 1. Isolation (do this first; the user's rule)

1. Create a git worktree on a new branch from `feature/auto-tuning` at `db052851` (not from `main`: `main` has no DiffCoTune code):
   `git -C ~/Desktop/flying_robot_course worktree add ~/Desktop/flying_robot_course_fbl -b feature/diffcotune-fbl db052851`
2. `simulator/` is **untracked** in the repo (848 MB), so the worktree does not have it. Copy it: `cp -a ~/Desktop/flying_robot_course/simulator ~/Desktop/flying_robot_course_fbl/simulator`. All your simulator changes go into this copy only. Record every simulator change also as a patch file under `experiments/diffcotune/fbl/simulator_patches/` (committed), because `simulator/` stays untracked.
3. Check and report: from the worktree, `experiments/diffcotune/reference.py` resolves `SIMULATOR` to `~/Desktop/flying_robot_course_fbl/simulator` (it is `parents[2] / 'simulator'`), and `frsim` is imported from the worktree copy (print `frsim.__file__`).
4. **Firmware bindings:** frsim loads `cffirmware` from `FRSIM_FIRMWARE_BUILD` or the default (`config.firmware_build_dir()`; the S0–S3 preflight uses `~/Desktop/crazyflie-firmware/build`). **Never rebuild, clean or modify `~/Desktop/crazyflie-firmware/build` or any firmware repo.** If FBL needs other bindings (for example from `~/Desktop/fbl_v2b_firmware`), build them with `simulator/firmware/build_bindings.sh` into the **worktree's** `simulator/_build` (the script clones the source read-only) and set `FRSIM_FIRMWARE_BUILD` only in your own worktree commands. First report what is needed before building.

**Absolute rules for isolation:**
- Work only inside `~/Desktop/flying_robot_course_fbl`. Never edit, create or delete files in `~/Desktop/flying_robot_course` (except the git metadata that `git worktree add` writes), never switch its branch, never stash or reset there.
- Git: commits only on `feature/diffcotune-fbl`; no push; no Co-Authored-By; no merge or rebase into any other branch.

## 2. Lab days (this laptop runs `round.py`)

The user flies in the lab with this laptop. Before starting anything heavier than a unit test or an editor action:
- if the file `~/Desktop/LAB_SESSION_ACTIVE` exists, **or** `pgrep -f "round.py|session.py"` finds a process: run only light single processes (`nice -n 19`, `OMP_NUM_THREADS=1`, no pool, no compile with more than 1 thread, at most 1 core). Postpone pools, gradient checks and binding builds until neither condition holds.
- Otherwise the usual limits: `nice -n 19`, one pool at a time, load ≤ 22, MemAvailable ≥ 3 GB, `OMP_NUM_THREADS=2`, `TORCHINDUCTOR_COMPILE_THREADS=1`, no GPU.

## 3. Tasks, in this order

### F1.1 Inventory (desk, light), file `experiments/diffcotune/fbl/INVENTORY_F1.md`
Answer from files, with paths:
- **frsim FBL:** how frsim runs FBL (Python or firmware bindings; which firmware it represents, e.g. `fbl_v2b` "199/199 equivalence", plan line 124); whether the 4 variants (ROM on/off, net on/off) are switches in frsim; mass and inertia used (must be the Omar values of S0–S3); inner loop (stock rate PID, `inner 1`); C_D (0.55); the net (trained 4-neuron network, weights file).
- **Old PyTorch copies:** `experiments/tuning/diff/fbl_plant_torch.py`, `fbl_plant_torch_v2.py`, `fbl_plant_torch_v2o.py` and their checks (`check_fbl_*.py`): what each models, which inertia (the plan says the old copy uses J_FBL and must move to the Omar inertia and mass), which firmware version, what was checked, which results exist.
- **Interfaces to match:** how `experiments/diffcotune/s0_model.py` / `s1_model.py` expose θ (log-factors relative to the defaults), β (thrust gain, motor lag, downwash scale, plume scale), the rollout, `fsens.py` / `cfwd`, and what `loop_lib.py` expects from a model (`N_BETA`, `GAIN_NAMES`, `GAIN_DEFAULT`, `gains_of`, the pool interface). List what is controller-specific for FBL (6 gains instead of 10, fixed kr).
- **Lab side (read only):** FBL firmware parameter names for the 6 gains and the variant switches (`ctrlFbl.*`), the crazyswarm2 branch `study_FBL_stock`, the image the drones need. What `round.py`'s gains-to-yaml would have to write for FBL.
- **Open decisions** (list, do not decide): e.g. whether the plume / downwash β apply unchanged to FBL; whether ROM and net use the same flown-plant β as S0–S3; anything else not stated.

### F1.2 Differentiable FBL model `experiments/diffcotune/fbl_model.py`
- Same interface as `s0_model.py` (so the loop can use it later): θ = 6 log-factors relative to preset 1; β = the same 4 values as S0 (calibration as reference); `N_BETA` 4; variant switches ROM / net (4 variants), selected by an environment variable `DIFFCOTUNE_FBL_VARIANT` = `base | rom | net | both`.
- Mass and inertia: the exact Omar values of frsim (as `s0_model.py` uses). Inner loop, C_D, net weights: as frsim's FBL.
- Start from the old torch copy if the inventory shows it is the closest match; state what was changed.
- Do not touch `s0_model.py` … `s3_model.py`, `costs.py`, `fsens.py`, `loop_lib.py`, `round.py`, `METHOD_SETTINGS`.

### F1.3 Forward check (model vs frsim), per variant
- Same procedure and thresholds as the S0 forward check of Stage 2 (find it in `experiments/diffcotune/checks/` and `docs/diffcotune_spec.md`; cite the threshold, do not invent one): noise off, preset 1, the 5 tuning scenarios (A1, A8, A4 cell 1, CIRCLE, FIG8) and the test scenarios (A4 cell 2, oval), plant at the calibration and at P1.
- Report per variant and scenario: position / velocity / attitude difference, PASS / FAIL.

### F1.4 Gradient check, per variant
- As `dryrun/stage8/gate9.py` (cfwd vs finite difference along the gradient, relative ≤ 1e-3; reverse mode for information), at preset 1 and the calibration β, task cost and J_sysId, the 5 tuning scenarios. Heavy: only outside lab sessions (section 2).

### F1.5 Report `experiments/diffcotune/fbl/report_F1.md`, plus commit on `feature/diffcotune-fbl`
1. Isolation check (worktree path, branch, `frsim.__file__`, bindings path; confirmation that `~/Desktop/flying_robot_course` is unchanged: `git -C ~/Desktop/flying_robot_course status --porcelain` before and after, same output).
2. Inventory summary and open decisions.
3. Model: files, what it copies from where, deviations from frsim's FBL.
4. Forward check and gradient check tables per variant.
5. What the next stage (F2: loop wiring, `round.py` FBL support, preflight, dry runs) would need.

Tell the user "Stage F1 done: experiments/diffcotune/fbl/report_F1.md (branch feature/diffcotune-fbl)". If an isolation rule cannot be kept, stop and report before doing anything else.

## 4. Rules

- Python: `/usr/bin/python3` (3.10, torch 2.5.1) for frsim and models; `~/.pyenv/versions/flying_robots/bin/python` for plots; no pip installs.
- Shells: no command that waits on standard input; heredocs or the edit tools.
- Processes: stop only processes you started, by exact PID (`ps -o pid,ppid,args -p <PID>` first); never `pkill` / `killall`; never touch the user's lab processes (`round.py`, `session.py`, crazyswarm2).
- Hardware: nothing flashed or flown, no drone or radio connection, no reboot, NVIDIA driver untouched.
- Data: complete uSD logs only, never radio CSVs.
- Do not touch: `~/Desktop/flying_robot_course` (the lab checkout), `~/Desktop/crazyflie-firmware` and its `build`, `~/Desktop/fbl_v2_firmware`, `~/Desktop/fbl_v2b_firmware` (read only), `~/Desktop/crazyswarm2` (read only), `docs/` in the lab checkout.
- Wording: real `date` time stamps; "DiffCoTune" / "DiffTune" in full; FBL is just "FBL"; "test error"; DiffTune settings are cited as DiffTune's.
