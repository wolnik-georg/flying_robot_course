# DiffCoTune, Stage 8d: dry run with the lab's flight rule + rule R3 (3 failed attempts), reruns S0 / S2 (desk only, 2026-11-16)

Paste everything below the line into the working agent (it continues after the Stage 8c prompt; Stage 8c rules still apply unless this prompt changes them).

---

You are the **working agent**. The planning agent and the user plan and validate; you execute. Facts only from files, with paths; "not stated" / "not yet" otherwise. Do not change the tuning method (fit, tuning step, costs, gradients, settings of the frozen variant "ours").

## 0. Why this stage (user decision, 16 Nov, option C)

In the S0 "ours" dry run, round 3 got worse (A1 floor flight 34.45 cm, then recovered to 3.31 cm in round 5). The dry run is **not** run like the lab:

| | Dry run now (`dryrun/loop_lib.py`) | Lab (`lab/round.py`, strict) |
|---|---|---|
| Repeat a flight | only a crash (tilt ≥ 70°), `fly_set` lines 211–229 | crash, tilt **or floor** flight, attempt ≤ 3 |
| Floor flight used for the β fit | **yes** (`floor_failed` only records, lines 204–208; `run_loop` passes every final trace to the fit, line 366) | **no** (`good()`, `round.py` ~line 113) |
| All 3 attempts fail | crash: the last trace is used anyway | `round.py:128`: "nothing fitted", the round stops |

The user wants the dry run to work like the lab, and wants to see whether the bad round still happens. The user also decided a rule for 3 failed attempts (**R3**), which goes into both the dry run and `round.py`.

**User question this stage answers:** with the lab's rule, does S0 (and S2) still get worse before it gets better?

## 1. Rule R3 and the lab flight rule (decided by the user 16 Nov; implement exactly this)

Per round l, for each tuning scenario s (A1, A8, A4 cell 1, CIRCLE, FIG8), with the round's gains θ_l:

1. **Usable flight** = not crashed, not tilted (protocol abort), not floor (floor rule of Stage 3d), and passes the quality gate (lab only).
2. A flight that is not usable is **repeated with the same θ_l**, at most 2 repeats (attempt ≤ 3). The first usable attempt is used. In the dry run a repeat uses a new noise seed: keep the existing scheme `seed + 1000 × tries`.
3. **R3:** if all 3 attempts on s fail:
   a. **Fallback flight:** s is flown once more with **θ_best** (the keep-best set so far). It is logged and reported. It is **not** used for the fit, **not** used for θ_l's real cost, and does not change θ_best. In round 0 there is no θ_best yet (θ_0 is the set being flown), so there is no fallback flight.
   b. **Fit on the other good flights:** the β fit of round l uses only the usable flights of the other scenarios.
   c. **Tuning step: unchanged.** It runs in the model on all 5 scenarios, with no real traces, as now.
4. **Keep-best, consequence of R3** (needed because θ_l's real cost is missing a scenario; state it in the report as "consequence of R3, for the user to confirm"):
   - a set with a scenario that failed 3 attempts is **not eligible** for keep-best;
   - the real cost for keep-best is the mean over the 5 used (usable) flights, as now;
   - if no flown set is eligible after round 5, the result is θ_0 (default gains) and the run is marked "no eligible set".
5. **Reporting, for comparability with the old runs:**
   - keep the old real cost "as flown" per round = mean over the last attempt of each scenario (this is what criteria a/b/c used so far);
   - add the cost over the used flights;
   - per scenario, all attempts (cost, floor, crashed, tilt, seed), the used flag and the fallback flight;
   - criteria a/b/c keep their definitions (`make_lab_tomorrow.py:23–25`), computed on the cost "as flown" and the per-scenario fail flags (a scenario counts as failed in round l if no usable attempt exists). Do not invent other criteria.

## 2. Tasks, in this order

### T1. Dry run: strict switch in `dryrun/loop_lib.py`
- New environment switch `DIFFCOTUNE_DRY_STRICT` (`1` = section 1; default `0` = old behaviour, which must stay **bit-identical**).
- Change only what section 1 needs:
  - `fly_set`: repeat also floor flights; return all attempts;
  - `run_loop`: fit traces = used flights only; R3 fallback flight with θ_best; keep-best eligibility; record fields as in 1.5.
- Do not touch the fit, the tuning step, `costs.py`, `fsens.py`, the models, `METHOD_SETTINGS`.
- Keep `dryrun/loop_lib.py.before_stage8d` first.
- **Tests:**
  - (a) unit test `dryrun/stage8/test_strict_rule.py` with a **fake pool** (no frsim, no torch pool): one scenario floor on all 3 attempts → 3 attempts, fallback flown with θ_best, fit gets 4 traces, set not eligible; floor on attempt 1 only → attempt 2 used; round 0 with 3 fails → no fallback; no eligible set → θ_0 returned and marked;
  - (b) regression with strict = 0: round 0 of S0 "ours" (stop after round 0) must give θ₁ and β **identical** (max abs diff 0.0) to `results/loop_S0_ours.json` rounds[1].phi / rounds[0].psi_after_fit. This is a pool job: queue it as the first job of T4, not in parallel.

### T2. Lab tool: R3 in `lab/round.py`
- Keep `round.py.before_stage8d` first.
- Replace the stop at `round.py:128` by R3:
  - when a scenario has 3 failed attempts in the manifest, `round.py` needs the fallback flight (manifest role `fallback`, flown with θ_best). If the fallback file is missing, print exactly which scenario to fly with which gains file (θ_best), and stop;
  - with the fallback present: fit on the usable flights of the other scenarios, run the tuning step as usual, mark θ_l not eligible for keep-best (`state.json`), record everything in `report.json` / `report.md` / `flights.json`;
  - the preview runs as before.
- `--lenient` (dry tests) stays as it is.
- Add `--strict-dry` to `lab/dry_test5.py`: the round test against a strict dry run uses the strict rule, as the lab does. The old lenient round tests must still pass unchanged.
- **Tests:**
  - (a) mock test `lab/test_r3_mock.py` with fake uSD files (`lab/fake_usd.py`): A1 floor ×3 + fallback → round completes, 4 flights fitted, not eligible; A1 floor ×3 without fallback → clear message, nothing written as the next round;
  - (b) with all flights usable, `round.py` output identical to before (θ_{l+1}, β diff 0.0) on an existing dry test folder.
- Update `lab/INTERLEAVED_LAB_CARD.md` and `lab/S0_LAB_CARD.md`: the R3 step (what to fly, which gains file, it is not used for the fit), plus the flight count (+1 per R3 case).
- `preflight.py`: add a check that `round.py` has R3, i.e. the version marker; do not change the other checks.

### T3. Report and LAB_TOMORROW readers
- `make_lab_tomorrow.py` (keep `.before_stage8d`) and `make_report8.py` / `make_report8c.py`: read the new fields when present (strict runs: tag `ours_strict`, files `results/loop_<C>_ours_strict.json`, `eval_<C>_ours_strict.json`) and show strict next to the old "ours" run:
  - cost as flown, cost over used flights, attempts, R3 cases;
  - eligibility, criteria a/b/c.
- Old files must render as before.

### T4. New queue `dryrun/stage8/c8.sh` (one job at a time, same pattern as `c7.sh`)
- The planning agent stopped `c7.sh` at 12:42; **`dry8o_S2` (old mode) is still running** (job_runner PID 3110009, run PID 3110012). Do not stop it. It is the "before" run for S2.
- `c8.sh` first waits until `logs/dry8o_S2.rc` exists, then runs:
  1. `regress8s_S0`: T1 test (b), strict = 0, round 0 only;
  2. `dry8s_S0`: strict dry run S0, `DIFFCOTUNE_DRY_STRICT=1`, tag `ours_strict`, same seeds as `dry8o_S0`;
  3. `rtest8s_S0`: round test `dry_test5.py --strict-dry` against `dry8s_S0`;
  4. `dry8s_S2`, then `rtest8s_S2`;
  5. `gate9s_S0`, then `gate9s_S2`: gradient check at the fitted β of every round of the strict runs;
  6. then `dry8s_S1`, `rtest8s_S1`, `gate9s_S1`, `dry8s_S3`, `rtest8s_S3`, `gate9s_S3` (RPM delay fixed, `DIFFCOTUNE_FIX_RD=1` as in `mk_ours.sh`).
- Make the job json files with `make_job.py` (template: `mk_ours.sh`; write a `mk_strict.sh`). Start `c8.sh` detached: `setsid nohup ./c8.sh > logs/slotC8.log 2>&1 < /dev/null &`.
- **Before starting `c8.sh`:** T1 test (a) and T2 tests (a) and (b) must pass. If they don't, do not start the queue; report.
- The old-mode jobs `rtest8o_S2`, `gate9o_S2`, `gate9o_S0` and the old S1 / S3 "ours" runs are dropped (superseded by the strict runs). The paper gate `gate9_S1` is dropped too. List this in the report.
- Expected time (estimate): about 4 h per dry run, 45 min per round test, 2.5 h per gate. S0 + S2 strict incl. gates ≈ 15 h.

### T5. Keep Stage 8c going
- Continue T1 (queue watch, now `c8.sh` + `dry8o_S2`), T4 (S2 preview replay on `loop_S2_ours.json` when `dry8o_S2` ends) and T6 (session timeline; add the strict S0 desk minutes when available).

## 3. Report back

File `experiments/diffcotune/dryrun/stage8/report_stage8d.md`, generated, re-run after each job:
1. T1–T3 changes: files, diff summary, test results (each test: PASS / FAIL, numbers).
2. Queue table (c8.sh): job, start, end, rc, key result.
3. **The answer to the user's question, per controller (S0, S2 first):** old "ours" vs "ours_strict":
   - real cost per round (as flown / used);
   - attempts and R3 cases per round, fallback flights;
   - eligible sets, keep-best round and cost;
   - criteria a/b/c;
   - held-out A4 cell 2 and tuning means (θ₀ → θ_best);
   - whether a round got worse than round 1 + 1 cm.
4. Lab-ready table S0 / S1 / S2 / S3 (strict runs).
5. Open items, who decides. Always include "keep-best eligibility rule = consequence of R3, user to confirm".

Tell the user "Stage 8d step done: <what>" after T1–T3 tests pass and `c8.sh` is started, and again when S0 + S2 strict (dry run, round test, gate) are complete.

## 4. Rules (changes to Stage 8c in bold)

- **Edits allowed now also in:** `dryrun/loop_lib.py` (only the strict switch of T1), `lab/round.py` (only R3 of T2), `lab/dry_test5.py` (`--strict-dry`), `lab/preflight.py` (R3 marker check), the lab cards, `make_lab_tomorrow.py`. A `.before_stage8d` copy first, every time. **The default behaviour (strict = 0, all flights usable) must stay bit-identical: prove it with the tests.**
- Still not to touch: the fit, the tuning step, costs, gradients, the models, `METHOD_SETTINGS`, `simulator/`, `flying_drone_stack/`, crazyswarm2, firmware repos, `experiments/tuning/`, `docs/`, Stage 3b–8 result files and logs (the old "ours" results stay as they are).
- Resources:
  - `nice -n 19`;
  - one pool at a time (the queue's); your tests are single light processes, no pool, MemAvailable ≥ 6 GB;
  - `OMP_NUM_THREADS=2`, `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - no GPU.
- Python: `/usr/bin/python3` for frsim / models; `~/.pyenv/versions/flying_robots/bin/python` for plots.
- Processes:
  - **you may stop the queue scripts you start (`c8.sh`) by exact PID**;
  - never `pkill` / `killall`;
  - never touch PIDs 3110009 / 3110010 / 3110012 (`dry8o_S2`) or the stuck processes from 15 Nov.
- Git and hardware: read-only git only, no commit; no Co-Authored-By; no drone, no radio, no flashing; no reboot.
- Wording:
  - real `date` time stamps;
  - "DiffCoTune" / "DiffTune" in full;
  - "test error";
  - never invent numbers or rules: R3 and section 1 are the user's rules, everything else is "not stated".
