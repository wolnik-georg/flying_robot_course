# DiffCoTune for FBL, Stage F2: decisions from F1, loop wiring, lab tools, dry runs (isolated branch, 2026-11-17)

Paste everything below the line into the working agent (it continues after Stage F1; the F1 rules still apply unless this prompt changes them).

---

You are the **working agent**. The planning agent and the user plan and validate; you execute. Facts only from files, with paths; "not stated" / "not yet" otherwise. Never invent numbers or rules.

## 0. Stage F1: validated

The planning agent checked `experiments/diffcotune/fbl/report_F1.md` (commit `bf8207fc`) against its logs, and the facts hold. F1 is accepted with the decisions in section 1. Same isolation as F1: work only in `~/Desktop/flying_robot_course_fbl` on branch `feature/diffcotune-fbl`. No push, no Co-Authored-By. The lab checkout `~/Desktop/flying_robot_course` stays untouched: check `git status --porcelain` there before and after, as in F1.

**Lab days have started (18 Nov onwards).** The user flies S0–S3 with this laptop. The F1 lab-day guard (F1 prompt section 2) is now the normal case:
- while `~/Desktop/LAB_SESSION_ACTIVE` exists or `pgrep -f "round.py|session.py"` finds a process: light single processes only (`nice -n 19`, `OMP_NUM_THREADS=1`, one core, no pool, no gradient check, no dry run);
- heavy jobs only in a queue that checks the guard **before every job** and waits (poll every 5 min) while it holds.

## 1. Decisions on the F1 open items (planning agent's recommendation; the user accepted them by sharing this prompt)

| # | item | decision |
|---|---|---|
| D1 | `SG_R` stop-gradient | **Off**: `FBL_SG_R` default becomes `0`, so the gradient is exact (F1: 18/18 rows PASS). Safeguard: any non-finite cost or gradient in a fit, tuning step or gradient check is logged with scenario, round and r0; the run **stops and reports**. No silent clamp. |
| D2 | forward check, 4 crashed A4 cells (rom/both, calibration plant) | **Known exception, gate unchanged.** Same treatment as the S2 gradient-gate exception: both frsim and the model crash, the crash flags agree, and the paths agree to 0.01 mm before the crash. Report it as "FAIL, known exception (post-crash phase)". |
| D3 | net-only variant | **Dropped from this campaign.** There is no firmware path (`dw_en` switches ROM and net together), no weights and no frsim reference, and the earlier result was "net adds nothing over ROM". FBL variants to tune: **base, rom, both** (3). `net` stays in `fbl_model.py` and is never tuned. |
| D4 | lab control rate (500 Hz?) | **Check from source (task F2.2).** Do not change the model before the planning agent has seen the result. |
| D5 | quantisation | **Off in the fit and the tuning step** (`FBL_QUANT=0`), the same as S0–S3 (`s0_model.py:17`: quantisation not modelled). The forward check is re-run with `FBL_QUANT=0` (task F2.1). |
| D6 | β | The same 4 β as S0, unchanged: thrust-law gain, motor lag, downwash scale, plume scale. They describe the plant, not the controller, so they are the same for all 3 variants. |
| D7 | crash tilt | FBL 45° (`config.py:54`) for the crash flag, the strict dry-run rule and R3, as frsim defines it. |
| D8 | method | Exactly the frozen variant "ours" with rules R3 and RS: ±25 % box, 6 flown sets, 5 tuning scenarios, keep-best, strict flight rule (`docs/diffcotune_lab_protocol_S0_DRAFT.md`, last section). The same settings as S0–S3, applied to the 6 FBL gains. θ_0 = preset 1. |

## 2. Tasks, in this order

### F2.1 Model settings and forward check without quantisation (light, allowed on lab days)
- `fbl_model.py`: `FBL_SG_R` default `0`, plus the non-finite guard of D1 (in the model or in `fsens_fbl.py`, wherever it catches both modes).
- Re-run `forward_check_fbl.py` with `FBL_QUANT=0` for base, rom and both (single process, light). Gate unchanged. Report the table next to the F1 table.

### F2.2 Lab control rate (read only, light)
- Find in the firmware source of the lab FBL image how often the FBL controller state advances on the drone: the stabilizer loop rate, the rate gate in `controllerOmarFbl`, and the `dt` used. Sources: `~/Desktop/fbl_v2b_firmware`, and the source the candidate `.bin` of `docs/fbl/LAB_PLAN.md:10` was built from, as far as the files say. Cite file:line.
- Also trace why frsim's odd-tick call does not advance the state (F1 left it open).
- Result: "lab = 500 Hz like frsim" or "lab differs: …". If it differs, **stop after this task and report.** The model would be wrong for the lab.

### F2.3 Sensor noise for FBL (light code; the noisy forward check is a single process)
- `fbl_model.frsim_noise(seed, n_ticks, plant)`, matching frsim's FBL sensors (`frsim/sensors.py`: 9 numbers per 1 ms tick; only the even ticks reach the controller). Read the sensor code and cite it.
- Forward check **with noise**, by the Stage 3d procedure (find it in `checks/` and the spec; cite the gate) for base, rom and both, on the 5 tuning scenarios at P1.

### F2.4 Loop wiring (`dryrun/loop_lib.py`, `fsens.py` dispatch, pool worker)
- `DIFFCOTUNE_CONTROLLER=FBL` plus `DIFFCOTUNE_FBL_VARIANT=base|rom|both`, and `FRSIM_FBL_VARIANT` set to match for the frsim flights. Result tags `FBLbase`, `FBLrom`, `FBLboth`.
- Replace the hard-wired 10 by the model's `N_GAINS`: `x` length, `PSI_FIT_IDX`, the `adam_inner` index, gain names and defaults. Crash tilt from the model. Gradient through `fbl/fsens_fbl.py` for FBL.
- Keep `.before_F2` copies first. **S0–S3 must stay bit-identical**: regression = round 0 of S0 and S2 "ours_strict" with RS (as `regress9r_S0/S2`), max abs diff of θ₁ and β = 0.0. These are pool jobs, so they go in the queue (F2.6), as its first jobs.
- Do not touch the fit, the tuning step, `costs.py`, `METHOD_SETTINGS` or the S0–S3 models.

### F2.5 Lab tools for FBL (`lab/round.py`, `lab/session.py`, `lab/preflight.py`, `lab/usd_ingest.py`, cards)
- `round.py` gains-to-yaml for FBL. Write a `ctrlFbl` fragment with the 6 gains (`kp_xy, kv_xy, ki_xy, kp_z, kv_z, ki_z`; kr not written) and `en 1`, `inner 1`, `cd 0.55`, with the variant switches set as follows:

  | variant | `dw_en` | `nn_en` |
  |---|---|---|
  | base | 0 | 0 |
  | rom | 1 | 0 |
  | both | 1 | 1 |

  Check that `session.py`'s merge step handles the `ctrlFbl` group, and that the yaml base is crazyswarm2 `study_FBL_stock` (read only).
- `usd_ingest.py`: check from the uSD logging config of the FBL image that every field the fit needs (position, velocity, attitude, rates; whatever `usd_ingest` reads for S0) is logged under FBL. If one is missing, **report it**. Do not change the logging.
- `preflight.py --controller FBL --variant <v>`: the FBL model import, the variant, the yaml fragment, the R3/RS markers. Extra check: FBL `fbl.en`/image facts, where the files allow it.
- Mock tests with fake uSD files (as `lab/test_r3_mock.py`) for FBL base: a normal round, R3, RS, and all 5 scenarios R3.
- S0–S3 regression: the existing mocks (a/ext/regress/rs/t3b) must pass unchanged. `round.py` output with all flights usable must be identical (diff 0.0).
- Lab card `lab/FBL_LAB_CARD.md`: image, crazyswarm2 branch, priming (`fbl_prime_controller.py`: the controller init wipes the `ctrlFbl` values on a fresh boot), the variant switch per block, and `check_active_controller.py` after a flash.

### F2.6 Queue `dryrun/fbl/f2.sh` (heavy; guard before every job; one pool at a time)
In this order:
1. `regressF2_S0`, `regressF2_S2` (F2.4 regression; must be 0.0, otherwise stop the queue);
2. `gateF_base`: gradient check as `gate9.py` at preset 1 + calibration β (SG_R 0, QUANT 0);
3. `dryF_base`: strict dry run, `DIFFCOTUNE_DRY_STRICT=1`, `DIFFCOTUNE_RESTART_BEST=1`, plant P1, the same seeds as S0's strict run; then `rtestF_base` (round test `dry_test5.py --strict-dry`, must reproduce θ₁ exactly) and `gate9F_base` (gradient check at the fitted β of every round);
4. the same three for `rom`, then for `both`.

Time estimate: not stated. Measure the first job and write the estimate into the report.

### F2.7 Report `experiments/diffcotune/fbl/report_F2.md`, plus a commit on `feature/diffcotune-fbl`
1. Isolation check (as F1).
2. D4 result (lab rate) with sources.
3. Forward checks: no quantisation, and with noise.
4. Loop / lab-tool changes: diffs, mock tests, S0–S3 regressions (all 0.0).
5. Per variant: real cost per round, keep-best round, θ_0 → θ_best on the tuning flights and on held-out A4 cell 2, R3/RS cases, round test, gates. The same table format as `dryrun/stage8/report_stage8e.md`'s lab-ready table.
6. Lab-ready table for FBL base / rom / both; open items, and who decides each.

Tell the user "Stage F2 step done: <what>" after F2.1–F2.5 (desk part), and "Stage F2 done: experiments/diffcotune/fbl/report_F2.md" at the end. Stop and report at once if: D4 shows a different lab rate; a non-finite value occurs; an S0–S3 regression is not 0.0; a uSD field for the fit is missing under FBL; or an isolation rule cannot be kept.

## 3. Rules (as F1, plus)

- Edits allowed on the FBL branch only: `fbl_model.py`, `fbl/*`, `dryrun/loop_lib.py` (F2.4 only), `fsens.py` (dispatch only, the tree path unchanged), `lab/round.py`, `lab/session.py`, `lab/preflight.py`, `lab/usd_ingest.py` (F2.5 only), new FBL cards, and the worktree's `simulator/` copy (patch file for every change). Keep a `.before_F2` copy first, every time.
- Never: the method (fit, tuning step, costs, `METHOD_SETTINGS`, the R3/RS rules); the S0–S3 models; anything in `~/Desktop/flying_robot_course`; firmware repos or their `build`; crazyswarm2 (read only).
- Resources: `nice -n 19`; one pool at a time; load ≤ 22; MemAvailable ≥ 3 GB; `OMP_NUM_THREADS=2`; `TORCHINDUCTOR_COMPILE_THREADS=1`; no GPU. The lab guard comes first.
- Processes: stop only processes you started, by exact PID; never `pkill`/`killall`; never touch `round.py`, `session.py` or crazyswarm2 processes.
- Hardware: nothing flashed or flown, no drone or radio connection, no reboot.
- Wording: real `date` time stamps; "DiffCoTune" / "DiffTune" in full; FBL is just "FBL"; "test error".
