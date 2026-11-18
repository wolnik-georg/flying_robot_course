# FBL worker prompt: Stage F2e continue (run the queue now that the lab is closed), 2026-11-18 evening

ROLE: fbl agent (FBL worker). Worktree `~/Desktop/flying_robot_course_fbl`, branch `feature/diffcotune-fbl`.
Read first: `~/Desktop/agent_snapshots/fbl_worker_2026-11-18.md` (your snapshot) and `docs/HANDOVER_DIFFCOTUNE_FBL_STAGE_F2D_CONTINUE_2026-11-18.md` on `feature/auto-tuning`. All rules there stay in force (nice 19, pool 3, 3 GB MemAvailable watchdog, exact-PID stops only, no hardware, no push, no co-author trailer, `.before_<stage>` copies, facts only).

## 1. State accepted by the planning agent
- Your last commit is `04ba95ca` (F2d/F2e: `reference.py` `_FLAT` leak fix, gateF_base PASS, `fbl_prime_v2b.py`, leak check). The leak fix is **accepted**: the 2-round leak check reproduced rounds 0–1 of the stopped run with max abs diff 0.0, and workers stayed flat at ~1.3 GB.
- The reboot fixed the NVIDIA mismatch (driver 580.178.04; the planning agent checked that a small `torch.compile` works with `CUDA_VISIBLE_DEVICES=-1`).

## 2. Lab state: what changed for you
- The lab session of 2026-11-18 is **over**. S0 round 0 was flown but produced no uSD logs (a separate problem, handled by the planning agent and a Cursor job in crazyswarm2 + `feature/auto-tuning` lab tools). **That work does not touch your worktree or the shared DiffCoTune model code.**
- The lock file `~/Desktop/LAB_SESSION_ACTIVE` still exists (stale, from 16:09). **The user removes it**; you do not. Once `ls ~/Desktop/LAB_SESSION_ACTIVE` fails and `ps aux | grep -E "[r]ound.py|[s]ession.py"` is empty, heavy work is allowed. Until then: light work only (step 3a).
- **If the lock file reappears while your queue runs** (a new lab session): f2.sh's guard only checks before each job. Let the running job finish, but do not let a new heavy job start. Confirm that the guard does this, and report it if it doesn't.

## 3. Do this, in order
a. **Light (allowed now):** verify the reboot state per snapshot §6 steps 1–3:
   - `git log --oneline -1` = `04ba95ca`.
   - The `reference.py` fix is present.
   - torch on CPU works.
   - No job of yours is running.
   - Lab-checkout isolation: if `/tmp/claude-1000/status_before_F2.txt` is gone, say so and use the current `git status --porcelain` of `~/Desktop/flying_robot_course` as the new baseline. The planning agent added `docs/cursor_prompt_usd_gate_2026-11-18.md` and this file; the lab agent added files under `experiments/diffcotune/lab/`. None of those are yours.
b. **Queue (after the lock file is gone):**
   - Check that each job env has `CUDA_VISIBLE_DEVICES=-1`: f2.sh does not export it. Add it to f2.sh (after a `.before_F2e2` copy) or confirm the job jsons carry it. Never set `TORCHDYNAMO_DISABLE`.
   - The first `torch.compile` after the reboot rebuilds the cache (a few minutes; expected, not a hang).
   - Restart per snapshot §6 step 5, with the marker line `--- restart after laptop reboot`.
   - Queue order unchanged: `gateF_base regressF2_S0 regressF2_S2 dryF_base rtestF_base gate9F_base gateF_rom dryF_rom rtestF_rom gate9F_rom gateF_both dryF_both rtestF_both gate9F_both`.
c. **Watch** with a background until-loop on `logs/f2.log` (no polling sleeps in the foreground).
d. **Gates you must check yourself (facts only):**
   - `gateF_base`: PASS again (the code is unchanged since 12:01; any difference = stop and report).
   - `regressF2_S0` / `regressF2_S2`: **max abs diff 0.0** against the S0/S2 references (shared code changed with the leak fix). Anything else = stop and report. **This is the gate for the lab S0/S2 path; report it as soon as it is known**, before the FBL jobs finish.
   - `dryF_base` from scratch: rounds 0–3 identical to `loop_FBL_FBLbase_ours_strict_stopped_mem.json` (abs diff 0.0, `seconds_*` excluded), round 4 now completes; record duration per round and peak memory (`logs/memlog.csv`). Workers must stay flat (~1.3 GB each). Growth > 0.2 GB/h per worker = report.
   - rom/both: the F1 exception covers only the 4 known `both` A1 cells (preset 1, gains1, seeds 1000/1001). Any other FAIL stops the queue.
e. **Interim messages** (one line each, as they happen):
   - `F2e: regressF2 S0/S2 = <diff>/<diff>`
   - `F2e: dryF_base done, <RMS per round>, peak <GB>`
   - and on any stop: `F2e stopped: <job> <reason>`.
f. **At the end:**
   - Finish `experiments/diffcotune/fbl/report_F2c.md`. Include the queue table, per-variant lab-ready tables as in `dryrun/stage8/report_stage8e.md`, the leak section, round.py memory (S0–S3 path: measured per-worker peak, not an estimate), and the open items from your snapshot §5.
   - Check lab-checkout isolation.
   - Commit (`git -c core.hooksPath=/dev/null commit`, explicit paths, no push).
   - Send: `Stage F2c/F2d/F2e done: experiments/diffcotune/fbl/report_F2c.md`.

## 4. Do not
- Flash, connect to or fly anything.
- Edit `~/Desktop/crazyswarm2`, the lab checkout, `flying_drone_stack/`, or any firmware tree.
- Remove the lock file yourself.
- Change gate rules or thresholds, or add exceptions beyond the 4 F1 cells.
- Resume dryF_base from the stopped run (from scratch only).
