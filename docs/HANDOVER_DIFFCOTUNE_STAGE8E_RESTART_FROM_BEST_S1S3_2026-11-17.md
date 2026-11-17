# DiffCoTune, Stage 8e: restart from θ_best after an R3 round, S1 / S3 reruns, session 1 sign-off (desk only, 2026-11-17)

Paste everything below the line into the working agent (it continues after Stage 8d; the Stage 8c / 8d rules still apply unless this prompt changes them).

---

You are the **working agent**. The planning agent and the user plan and validate; you execute. Facts only from files, with paths; "not stated" / "not yet" otherwise. Do not change the tuning method beyond the rule in section 1.

## 0. State at handover (checked by the planning agent, 17 Nov 10:55)

`c8.sh` finished: `ALL DONE` at 08:03:36, all 13 jobs rc 0 (`dryrun/stage8/logs/slotC8.log`). `report_stage8d.md` was **not** regenerated after 16 Nov 15:25: do that first (T0).

Strict runs ("ours_strict", `results/loop_<C>_ours_strict.json`, `eval_<C>_ours_strict.json`, `grad_gate9_<C>_ours_strict.json`):

| | Real 3-D cost per round (cm, as flown) | θ_best | Round test | Gradient check |
|---|---|---|---|---|
| S0 | 18.26, 9.24, 7.35, 6.39, 7.18, 5.53 | round 5 | exact | PASS |
| S2 | 14.47, 9.64, 7.85, 6.72, 5.98, 5.92 | round 5 | exact | FAIL: A1 task only (round 0: 5.7e-3, round 4: 1.9e-3, limit 1e-3) |
| S1 | 9.54, 6.22, 94.81, nan, nan, nan | round 1 | exact | PASS |
| S3 | 10.07, 9.71, 7.21, nan, nan, nan | round 2 | exact | PASS |

- **S1 from round 2 and S3 from round 3:** all 5 scenarios fail all 3 attempts (R3 on every scenario), so no usable flight exists, β is not refitted (psi unchanged), and the tuning step **starts again from the crashed θ_l** (`loop_lib.py:429`, `x` = θ_l) and moves another ±25 %. The run never recovers. In the lab this would be 15 crashes + 5 fallback flights per round.
- S1 / S3 use the plant `P1RD` (RPM delay 8 ms, `loop_lib.py:71`) while the model keeps the RPM delay fixed at 4.0502 ms (`FIX_RD`, user decision 14 Nov), so that part of the mismatch cannot be fitted (psi error −0.681 in every round). The real drones were measured at 4.05 ms.
- In S0 / S2 the only R3 case is A1 in round 0 (θ_0 floor ×3), where no θ_best exists yet.

## 1. User decisions (17 Nov)

1. **Rule RS ("restart from θ_best"), the same for S0–S3, in the dry run and in `round.py`:**
   a. If round l has at least one R3 scenario (θ_l not eligible) **and** a θ_best exists, the tuning step of round l starts from θ_best instead of θ_l. The ±25 % box is around θ_best. The β fit of round l is unchanged (usable flights only, as in Stage 8d; with 0 usable flights β stays as it is).
   b. **Stop:** if round l has an R3 scenario and round l − 1 also had one (and θ_best exists), no further tuning: the result is θ_best, marked "stopped: two R3 rounds in a row". No new gain set is flown.
   c. Everything else is unchanged: R3, fallback flight, keep-best eligibility, extension rounds, preview.
   d. Expected consequence, to be proven: S0 and S2 strict runs are **bit-identical** with RS on (their only R3 round is round 0, which has no θ_best).
2. **S1 / S3 dry runs are rerun with RS twice:** with the plant `P1` (RPM delay = the calibrated 4.05 ms, what the lab assumes) **and** with `P1RD` (8 ms, stress test). The **P1 runs decide lab readiness** for S1 / S3; the P1RD runs are evidence only.
3. **S2's gradient check:** the A1 task failure (0.57 % and 0.19 %, every other check passes) is **accepted as a known exception** (A1 is the known weak spot since Stage 2). Report it as "FAIL on A1 task only, accepted by the user 17 Nov". Do not change the gate or its threshold.

## 2. Tasks, in this order

### T0. Finish Stage 8d reporting
- Regenerate `report_stage8d.md`, `LAB_TOMORROW.md` (`make_lab_tomorrow.py`), and `make_report8d.py` output from the finished `c8.sh` results. Numbers must match the table in section 0.

### T1. RS in `dryrun/loop_lib.py`
- Keep `loop_lib.py.before_stage8e` first.
- New environment switch `DIFFCOTUNE_RESTART_BEST` (`1` = rule RS; default `0` = Stage 8d behaviour, **bit-identical**). Only active together with `DIFFCOTUNE_DRY_STRICT=1`.
- In `run_loop`, before the tuning step (line ~429): if RS and `r['r3_scenarios']` and `keep` exists → set `x[:10] = keep['phi']` before `adam_inner(tune_fn, ...)`. Stop rule 1b: set the halt reason `'two R3 rounds in a row'` and end the loop; θ_best is the result.
- Record per round: `restart_from_best` (bool, θ_best round), and in the run: `halt` = the stop reason.
- Do not touch the fit, the tuning step itself, costs, gradients, models, `METHOD_SETTINGS`.

### T2. Plant switch for the dry-run tools
- `dryrun/stage8/run_8.py` and `lab/dry_test5.py` (keep `.before_stage8e`): environment `DIFFCOTUNE_DRY_PLANT` = `P1` | `P1RD`; default = today's choice (`P1RD` for S1 / S3, `P1` for S0 / S2), so the old jobs are unchanged. The chosen plant is written into the result json.

### T3. RS in `lab/round.py`
- Keep `round.py.before_stage8e` first. Marker `RS_VERSION = 'RS-2026-11-17'`.
- If the round has an R3 scenario and `keep0` exists: the tuning step starts from θ_best (`x = np.r_[keep0 phi, psi]` before `L.adam_inner(... tune ...)`, line ~173); print and record it (`report.json`, `report.md`, `state.json`: `restart_from_best`, `prev_r3`).
- Stop rule 1b: if this round and the previous one (from `state.json` history) both have R3 → write `gains_theta_best.yaml`, `last_round = true`, no `phi_next`, message "Tuning stopped: two R3 rounds in a row. Final gains = θ_best (round k). Fly the test flights with θ_best."
- `preflight.py`: add a check for the RS marker (like check 6b). Lab cards (`S0_LAB_CARD.md`, `INTERLEAVED_LAB_CARD.md`): one line each for RS and the stop rule.

### T4. Tests (all must pass before the queue starts; light single processes, no pool)
- (a) `dryrun/stage8/test_restart_rule.py`, fake pool: R3 round with θ_best → tuning starts from θ_best (box around θ_best); R3 in round 0 → starts from θ_0 (no θ_best); two R3 rounds in a row → stop, result θ_best; RS = 0 → Stage 8d behaviour.
- (b) `lab/test_r3_mock.py` extended (or `lab/test_rs_mock.py`): R3 + fallback with θ_best present → next θ tuned from θ_best; two R3 rounds → stop message, `gains_theta_best.yaml` written, no next round; existing 8d mock cases still pass unchanged.
- (c) `round.py` with all flights usable on an existing dry test folder: θ_{l+1}, β diff 0.0 vs before (as Stage 8d test b).

### T5. Queue `dryrun/stage8/c9.sh` (one job at a time, pattern of `c8.sh`, jobs via `make_job.py`, write `mk_rs.sh`)
All jobs: `DIFFCOTUNE_DRY_STRICT=1 DIFFCOTUNE_RESTART_BEST=1`, same seeds as the `dry8s_*` runs, S1 / S3 with `DIFFCOTUNE_FIX_RD=1`.
1. `regress9r_S0`: full S0 strict run with RS, tag `ours_rs_regress`; compare with `loop_S0_ours_strict.json` (every round's phi, psi, real cost, eligibility): max abs diff must be 0.0.
2. `regress9r_S2`: same for S2.
3. `dry9r_S1p1` (plant P1, tag `ours_rs_p1`), `rtest9r_S1p1` (`dry_test5.py --strict-dry`, plant P1), `gate9r_S1p1`.
4. `dry9r_S3p1`, `rtest9r_S3p1`, `gate9r_S3p1`.
5. `dry9r_S1rd` (plant P1RD, tag `ours_rs_rd`), `dry9r_S3rd`: dry runs only (evidence).
- Start detached: `setsid nohup ./c9.sh > logs/slotC9.log 2>&1 < /dev/null &`. Estimate: ~2.5 h per dry run, ~40 min per round test, ~1–1.5 h per gate (c8.sh times): ≈ 20 h in total. If both regressions pass (about 5 h after the start), say so at once: that is the sign-off for session 1.

## 3. Report back: `dryrun/stage8/report_stage8e.md`, generated, re-run after each job

1. T0–T4: files, diff summary, each test PASS / FAIL with numbers.
2. Queue table (`c9.sh`): job, start, end, rc, key result.
3. **S0 / S2:** regression result (diff per field). If not 0.0: stop, report, the planning agent decides.
4. **S1 / S3, per plant:** real cost per round (as flown / used), attempts and R3 cases per round, restarts from θ_best, stop reason, θ_best round and cost, criteria a/b/c, whether a round got worse than round 1 + 1 cm, θ_0 → θ_best on tuning / held-out A4 cell 2 / oval, number of crashed and fallback flights (= what the lab would have flown).
5. Lab-ready table S0 / S1 / S2 / S3: dry run, a/b/c, round test, gradient check (S2: "FAIL on A1 task only, accepted by the user 17 Nov"), preview, RS marker in preflight, verdict.
6. Open items, who decides.

Tell the user "Stage 8e step done: <what>" (a) after T0–T4 pass and `c9.sh` is started, (b) when both regressions are done, (c) when the queue is done.

## 4. Rules (changes to Stage 8d in bold)

- **Edits allowed now:** `dryrun/loop_lib.py` (only RS, T1), `lab/round.py` (only RS, T3), `dryrun/stage8/run_8.py` and `lab/dry_test5.py` (plant switch, T2), `lab/preflight.py` (RS marker), the lab cards, report / reader scripts in `dryrun/stage8/`. A `.before_stage8e` copy first, every time. Default behaviour (both switches off) must stay bit-identical.
- Not to touch: the fit, the tuning step, costs, gradients, models, `METHOD_SETTINGS`, `gate9*.py` thresholds, `simulator/`, `flying_drone_stack/`, crazyswarm2, firmware repos, `experiments/tuning/`, `docs/`, all earlier result files and logs (Stage 3b–8d stay as they are).
- Resources: `nice -n 19`; one pool at a time (the queue's); your tests are single light processes, MemAvailable ≥ 6 GB; `OMP_NUM_THREADS=2`, `TORCHINDUCTOR_COMPILE_THREADS=1`; no GPU. Job runner limits: load ≤ 22, MemAvailable ≥ 3 GB.
- Python: `/usr/bin/python3` for frsim / models; `~/.pyenv/versions/flying_robots/bin/python` for plots.
- Processes: you may stop `c9.sh` by exact PID; never `pkill` / `killall`; never touch the stuck processes from 15 Nov (if still present).
- Git and hardware: read-only git only, no commit; no Co-Authored-By; no drone, no radio, no flashing; no reboot.
- Wording: real `date` time stamps; "DiffCoTune" / "DiffTune" in full; "test error"; never invent numbers or rules: section 1 is the user's rule set, anything else is "not stated".

## 5. Change of scope (user, 17 Nov, overrides sections 1.2, T2, T5.3–T5.5 and report item 4)

Focus is the real lab sessions. **No more S1 / S3 dry runs.**
- **Dropped:** decision 1.2 (S1 / S3 reruns with P1 and P1RD), T2 (plant switch), T5 items 3–5 (`dry9r_*`, `rtest9r_*`, `gate9r_*`), report item 4.
- **Kept:** T0 (Stage 8d report), T1 + T3 (RS in `loop_lib.py` and `round.py`; it is a lab safety rule), T4 tests, and T5 items 1–2 only (`regress9r_S0`, `regress9r_S2`, about 5 h). `c9.sh` = these two jobs.
- S1 / S3 lab readiness is judged on what already exists: round tests exact (`logs/rtest8s_S1.log`, `logs/rtest8s_S3.log`), gradient checks PASS (`grad_gate9_S1/S3_ours_strict.json`), preflight, RS mock tests. The Stage 8d crashes from round 2/3 are reported as "dry-run behaviour under an unfittable 8 ms delay; in the lab covered by RS + stop rule".
- Report: `report_stage8e.md` with T0, T1/T3/T4 results, the two regressions, and the lab-ready table S0–S3. Tell the user "Stage 8e done" when both regressions are finished.
