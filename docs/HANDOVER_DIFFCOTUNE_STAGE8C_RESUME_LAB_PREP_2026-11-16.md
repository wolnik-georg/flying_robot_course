# DiffCoTune, Stage 8c: resume after the crash, finish the evidence, session 1 (S0 + S2) lab-ready (desk only, 2026-11-16)

Paste everything below the line into the working agent.

---

You are the **working agent**. The planning agent (Claude Code) and the user plan and validate; you execute. Work only from the facts in the files named here. Do not re-derive decisions, do not propose method changes, do not invent numbers or rules: write "not stated" / "not yet" and list it as open. Every number you report must come from a file, with its path.

## 0. Context in five lines

- Thesis: comparative study of four quadrotor controllers S0–S3 (S0 geometric, S1 INDI, S2 S0 + NS2 network, S3 INDI + NS2) on two Crazyflies. Before the study, each controller gets **one gain set** from an auto-tuning method that uses real flights.
- Method: **DiffCoTune** (Krishna et al. 2025) Split-Alternate loop with **DiffTune's** quadrotor settings (Cheng et al. 2024). Lab variant **"ours"**, frozen 15 Nov evening, the same for S0–S3. Full definition: `docs/diffcotune_spec.md` §24 (read §24 only; §5 and §17 for the deviation list if needed).
- Last handover (for the rules and the decisions): `docs/HANDOVER_DIFFCOTUNE_STAGE8B_LAB_VARIANT_FROZEN_2026-11-15.md`.
- Before any lab flight, every controller needs a complete **dry run** (simulator to simulator, plant P1 = wrong thrust and lag on purpose) that passes criteria a–c, an exact **round test** and a **gradient check**.
- Session 1 in the lab = S0 + S2 interleaved; session 2 = S1 + S3.

## 1. State at handover (2026-11-16 12:10, checked by the planning agent)

- The desktop session crashed on 16 Nov 03:17 and killed the job queue. Committed afterwards: `63cfd594` on `feature/auto-tuning` (all of `experiments/diffcotune/` and the DiffCoTune docs; generated `lab/**/*.csv` ignored).
- **Queue running now:** `experiments/diffcotune/dryrun/stage8/c7.sh` (PID 3110006, started 11:56, log `dryrun/stage8/logs/slotC7.log`), one job at a time, through `../stage3c/job_runner.py`:
  `dry8o_S2 → rtest8o_S2 → gate9o_S2 → gate9o_S0 → dry8o_S1 → rtest8o_S1 → gate9o_S1 → dry8o_S3 → rtest8o_S3 → gate9o_S3 → gate9_S1 (paper)`.
  Expected (estimate from earlier round times): dry8o_S2 ~3.5 h, round test ~45 min, each gate ~2.5 h. S2 round 0 already reproduced 14.38 cm (same as before the crash).
- Partial files from the crash are kept as `*_stopped2104*` and `*_crash0205*`. Do not delete or overwrite them.
- **Done:**
  - S0 "ours" dry run, 6 rounds: real 3-D cost 18.24 → 9.69 → 9.14 → 15.96 → 4.78 → 3.31 cm (`results/loop_S0_ours.json`); held-out A4 cell 2 13.05 → 5.23 cm (`results/eval_S0_ours.json`);
  - S0 round test exact (θ₁ and β difference 0.0, `logs/rtest8o_S0.log`);
  - preflight PASSED for S0–S3 (`logs/pre_<C>.log`);
  - lab cards updated (preview line, `--method ours`, uSD rule);
  - `round.py` default method is `ours`.
- **S0 criteria a / b / c = no / no / yes.** Both failures come only from round 3 (A1 floor flight 15.85 → 34.45 cm while the other 4 scenarios improved; the preview PASSED that round; it gave a false alarm in round 2). **This is a decision for the user; do not change any criterion.**
- **Fixed but not re-tested:** `lab/preview.py:24` (S1 / S3: `L.model().RD_START` → `cal.rpm_delay_ms`; old copy `lab/preview.py.before_rdfix`). The paper S3 round test crashed on it (`logs/rtest8_S3.log`, 00:42). The queued `rtest8o_S1` / `rtest8o_S3` are the first real test.
- `LAB_TOMORROW.md` is regenerated from results by `dryrun/stage8/make_lab_tomorrow.py` (read-only on the results). Previous version: `LAB_TOMORROW.before_crash_resume.md`.
- `report_stage8.md` does **not exist** yet; only `report_stage8_part1.md` … `part4.md` (§1, §2, §3–4, §6).
- **Five stuck python3 processes** (PIDs 3059125, 3059126, 3059127, 3059129, 3060180, plus zombies 3059128, 3060176–3060179) are stuck in kernel exit from 15 Nov. `kill -9` has no effect. **Ignore them**: no memory, ~5 cores. The user reboots after the queue. Do not try to kill them.

## 2. Tasks, in this order

### T1. Watch the queue (whole time)
- Do **not** stop, restart or reorder `c7.sh`. Check `logs/slotC7.log` and the current job log about every 30–60 min (use waits, not busy loops).
- After every finished job: record rc, end time and the key numbers, then run `nice -n 19 /usr/bin/python3 make_lab_tomorrow.py` (in `dryrun/stage8/`).
- If a job ends with rc ≠ 0: read its log, find the cause, write it in the report (file and line), and **do not fix and rerun it yourself if the fix touches method code** (`loop_lib.py`, models, `costs.py`, `fsens.py`, `round.py`). Report it and stop that line of work; the next queue job starts anyway. A pure tooling or reporting fix (preview output, report scripts) is allowed; state it.
- If MemAvailable < 3 GB or load > 22 (from `/proc/meminfo`, `/proc/loadavg`): stop only the process **you** started, by exact PID, and report.

### T2. `report_stage8.md`: one complete report, re-runnable (start now)
- Write `dryrun/stage8/make_report8.py`. It concatenates parts 1–4 in section order and generates the missing sections from the result files, so it can be re-run after every job:
  - **§5: "ours" dry runs S0–S3.** Per controller:
    - real cost per round, crashes / floor / tilt per round (with scenario names);
    - largest gain change per round, criteria a / b / c with the numbers behind each;
    - tuning / held-out A4 cell 2 / in-sample A4 cell 1 means (θ₀ → θ_best);
    - round test result, gradient check per round (from `logs/gate9o_<C>.log` / its result json: PASS / FAIL per scenario and round, max relative error);
    - desk minutes per round.
    - Paper columns next to it from the existing paper results (evidence, not adopted).
  - **§7: lab-readiness per controller.** Yes / no, with the exact reason. "Not yet" if a job is missing.
- Check the generated numbers against `LAB_TOMORROW.md` (same source; they must agree).

### T3. S0 round-3 decision sheet (start now, desk only, no new runs)
- File `dryrun/stage8/S0_ROUND3_DECISION.md`, one page, facts only:
  - per-scenario real cost of rounds 2, 3, 4 and the gains of θ₃ vs θ₂ (factors);
  - why criteria a and b fail (thresholds from `make_lab_tomorrow.py:23–25`);
  - what keep-best returns (round 5, 3.31 cm), and whether round 3 influenced the final θ;
  - what the lab protocol does with a floor flight (repeat rule, "not used for the fit": cite the protocol draft / `round.py` `--lenient` text), and whether the dry run fitted round 3's A1 floor flight (it uses `--lenient`; state what that means for the fit of round 3);
  - preview result per round (round 2 false alarm, round 3 missed).
- End with the options, without a recommendation: (A) accept S0 as is, (B) a criterion that allows one round that later recovers (needs the user to define it; you do not), (C) other. **The user decides.**

### T4. S2 preview check (after `dry8o_S2` ends, before or between later jobs only if memory allows)
- Re-run the preview replay on the new `results/loop_S2_ours.json` (same as `jobs_prev_S2_ours.json`; the old partial result is `results/preview_S2_ours_partial.json`; keep it). Single process, `nice -n 19`, only while MemAvailable ≥ 6 GB with the queue job running; otherwise run it in a gap between queue jobs.
- Add the per-round preview vs real-cost table for S2 to §6 / §5 (like the S0 table in `report_stage8_part4.md`).

### T5. Preview fix check for S1 / S3 (light)
- Run `lab/test_preview_mock.py` for S1 (and S3 if it takes the controller as an argument), single process, `nice -n 19`. Report PASSED / FAILED. If the mock does not cover S1 / S3, state that; the queued `rtest8o_S1` / `rtest8o_S3` then remain the test.

### T6. Session timeline with the Stage 8b numbers (light)
- Re-run `lab/session_timeline.py` (output `lab/results/session_timeline.md`, current one is from 15 Nov 13:01 with older desk times) with 6 rounds × 5 tuning flights + test flights and the Stage 8b desk minutes: S0 "ours" 40 / 38 / 36 / 36 / 29 (`LAB_TOMORROW.md`), S2 from `loop_S2_ours.json` once it exists. Keep the old file as `session_timeline.before_stage8c.md`. Report the estimated session-1 hours.

## 3. Report back

File `experiments/diffcotune/dryrun/stage8/report_stage8c.md`, short, updated after each finished queue job:
1. Queue: per job rc, start / end time (from `date` / the logs), key result.
2. T2–T6: done / not done, with paths.
3. **Lab-ready table** S0 / S1 / S2 / S3: dry run, a / b / c, round test, gradient check, preview, verdict.
4. Open items, with who decides (user / planning agent).

Then stop and tell the user: "Stage 8c report ready: experiments/diffcotune/dryrun/stage8/report_stage8c.md". The planning agent validates before anything is committed or flown.

## 4. Rules

- **Resources:**
  - `nice -n 19` for everything;
  - **one worker pool at a time**: the queue owns it. You start only single light processes (no `Pool`), and only while MemAvailable ≥ 6 GB;
  - torch threads 2 (`OMP_NUM_THREADS=2`), `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - no GPU (driver/library mismatch, do not touch the NVIDIA driver).
- **Python:** `/usr/bin/python3` (3.10, torch 2.5.1) for frsim and the models; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs.
- **Shells:** no command that waits on standard input; use heredocs or the edit tools.
- **Edits only in:** `experiments/diffcotune/dryrun/stage8/` (new scripts and reports), `experiments/diffcotune/lab/results/`, and reporting/tooling fixes in `experiments/diffcotune/lab/preview.py` (keep a `.before_stage8c` copy first).
- **Do not touch:**
  - method code (`loop_lib.py`, `*_model.py`, `costs.py`, `fsens.py`, `reference.py`, `round.py` logic), `c7.sh` and the job json files;
  - Stage 3b–8 result files and logs (read only);
  - `simulator/`, `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`, `experiments/tuning/`;
  - `docs/` (spec, inventory, supervisor summary, protocol, `docs/07_*`, meetings, thesis): the planning agent updates them after validation.
- **Git and hardware:**
  - read-only git only: no add, commit, stash, checkout, branch switch or push;
  - no Co-Authored-By;
  - nothing flashed or flown, no drone or radio connection;
  - no reboot.
- **Stopping processes:** no `pkill` / `killall` / pattern kills. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Data:** complete uSD logs only, never radio CSVs (dry runs use frsim traces).
- **Wording:**
  - time stamps only from real `date` output;
  - method names in full ("DiffCoTune", "DiffTune");
  - DiffTune settings are cited as DiffTune's, never as DiffCoTune's;
  - "test error", never "study score";
  - FBL is just "FBL" (and is out of scope here).
