# FBL worker prompt: Stage F2g — rule B for the `both` variant (noise-free A1 gradient), 2026-11-19

ROLE: fbl agent. Worktree `~/Desktop/flying_robot_course_fbl`, branch `feature/diffcotune-fbl`. All rules from F2d/F2e stay in force (nice 19, pool 3, 3 GB watchdog, exact-PID stops, no hardware, no push, no co-author trailer, `.before_<stage>` copies, facts only, lab guard via `~/Desktop/LAB_SESSION_ACTIVE`).

## 1. Decision (user, 2026-11-19, on the planning agent's recommendation)
**Rule B:** for the `both` variant only, the tuning step uses the **noise-free** A1 cost (`J_task_noisefree`, value and gradient) in place of the noisy A1 cost (`J_task_noise`).
- **Why:** the noisy A1 cost of `both` is chaotic, in frsim and the model alike. J moves 5–10 % for a gain change down to 1e-12; the finite differences do not converge; the tangent grows to 1e8–1e20; this holds for every seed pair tested; ~all of it is in gain index 1 (your diagnosis, `logs/diag_gateF_both_A1_both.json`). The noisy gradient carries no information there.
- **Scope, exactly:**
  - Variant `both`; scenario A1; tuning step (task gradient and J used for the tuning update) only.
  - Unchanged: every other cell of `both` (A8, CIRCLE, FIG8, A4 cell 1, with noise as now); the sysId/beta fit; the F2f model-crash guard; the base and rom variants; S0–S3; all scoring and test evaluation (rtestF_both and any test or real cost still use the **noisy** costs as now, so the results show noise robustness honestly).
- No other exception. Gate tolerances unchanged.

## 2. Implement (`.before_F2g` copies first)
1. A flag (e.g. `DIFFCOTUNE_NOISEFREE_TASK_CELLS=A1`, or a loop argument), **OFF by default**, set ON only in the FBLboth job jsons (gateF_both, dryF_both, rtestF_both, gate9F_both).
   - When ON, the named cells use the noise-free task cost in the tuning step.
   - Log once per job: `rule B active: cells [A1]`.
2. **Monitoring (no effect on tuning):** per round in dryF_both, also compute and log the noisy A1 J (value only, no gradient) at the round's gains. Then we can see whether noise-free tuning makes noisy A1 worse.
3. Keep the code path minimal; show the full diff in the report.

## 3. Tests: all must pass before the queue restarts
a. **regressF2_S0 and regressF2_S2** with the flag OFF: max abs diff 0.0 (shared code changed; re-run both).
b. **base round 0 with the flag OFF**, guard ON as in its job: diff 0.0 against `results/loop_FBL_FBLbase_guard_r0.json`.
c. **Unit test** (`fbl/test_rule_b_F2g.py`, logged to `logs/f2g/`):
   - Flag ON → the A1 tuning entry equals the noise-free A1 cost (value + gradient).
   - Every other cell is identical to flag OFF.
   - Flag OFF → identical to the current code.
d. **gateF_both with the flag ON:**
   - The A1 `J_task_noise` row is reported as `replaced (rule B)`, not as PASS or FAIL.
   - The A4 crash rows are dropped by the guard as before.
   - **Every other row passes the unchanged tolerance (1e-3).**
   - Any other FAIL stops the queue as before.
- Send `F2g: rule B built, tests a-d <results>` before restarting.

## 4. Queue
- After a–d pass: restart from **dryF_both** (from scratch; marker line `--- restart after F2g rule B`), then rtestF_both, gate9F_both.
- In gate9F_both, the A1 noisy-task rows are reported as `replaced (rule B)`; all other rule handling is as in F2e/F2f.
- Per round in dryF_both, report:
  - 3-D RMS (all cells);
  - noisy A1 J (monitoring);
  - guard drops;
  - crash flags.
- Stop and report on: any gate FAIL outside the replaced A1 row and the guard-dropped crash cells; noisy A1 J rising over 2 rounds in a row; the memory watchdog; non-finite values.

## 5. Also owed (from the planning agent's previous message, if not done yet)
- Name gain index 1 (which FBL gain, file:line, its preset-1 value).
- gate9 tolerance and rule (file:line), and whether any A1 row with rel up to 1.4e-2 passed by mistake.
- Test c of F2f re-run into `logs/f2f/`.
- F2f section in `report_F2c.md`.
- Commit of the finished base + rom work.

## 6. Report
- New section `F2g rule B` in `experiments/diffcotune/fbl/report_F2c.md`, containing:
  - the rule, the why (with the diagnosis numbers), the scope, the test results;
  - per-round dryF_both tables;
  - the final per-variant lab-ready tables for base, rom and both.
- At the end, commit (explicit paths, no push) and send: `Stage F2c/F2d/F2e/F2f/F2g done: experiments/diffcotune/fbl/report_F2c.md`.
