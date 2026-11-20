# FBL worker prompt: Stage F2h — per-round A4 fit rule for `both` + new stop rules, 2026-11-20

ROLE: fbl agent. Worktree `~/Desktop/flying_robot_course_fbl`, branch `feature/diffcotune-fbl`. All rules from F2d–F2g stay in force (nice 19, pool 3, 3 GB watchdog, exact-PID stops, no hardware, no push, no co-author trailer, `.before_<stage>` copies, facts only, lab guard via `~/Desktop/LAB_SESSION_ACTIVE`). **Lab day:** check the lock before starting and between jobs; while it exists, start nothing.

## 1. Decisions (planning agent recommendation, based on 9c05826a + c9061854)
- **Rule B stays unchanged.** The A1 rise is a noise-free trade-off visible in frsim, not chaos.
- **Fit: F0 (per-call guard drop) is the problem for `both`.** It switches A4 in and out of the objective from call to call (29/51 in r0, 48–50/51 later), beta stalls (L2 0.530), and the model keeps crashing on A4.
- **F1/F3 (truncate) rejected:** they underpredict A1/CIRCLE/FIG8 by 3–20× (ratio 0.05–0.33) and move comp 1 away from truth.
- **F2 (no A4) is the best single fit** (L2 0.366) and the model no longer crashes on A4 at G0 (13.3°). But a fit that never sees A4 can never learn from it. So the rule is **per round**, not permanent:

**F2h fit rule (`both` only, flag OFF by default, ON in the *_both job jsons):**
at the start of each round's beta fit, run the model noise-free on A4 cell 1 at the current beta and the round's gains.
- Crash (tilt ≥ 45°) → this round's fit excludes **all A4 cells** (as F2).
- No crash → this round's fit includes A4 with the F2f per-call guard as now.
- Log once per round: `F2h fit: A4 excluded|included (model A4 max tilt X°)`.
The tuning step and its F2f guard are unchanged.

## 2. New stop rules for dryF_both (replace "noisy A1 J rises 2 rounds in a row")
- **Rule-B harm:** stop if the noisy A1 J (fixed seeds 6000/6001) rises while the frsim **noise-free** A1 J does not rise in the same round.
- **General:** stop if the frsim all-cell 3-D RMS rises 2 rounds in a row.
- Unchanged: gate FAIL outside the rule-B row and the guard-dropped crash cells, the memory watchdog, non-finite values.

## 3. Tests before the restart
a. regressF2_S0 and regressF2_S2, flag OFF: max abs diff 0.0.
b. base round 0 and rom round 0 with the flag OFF: diff 0.0 against their finished records.
c. Unit test `fbl/test_fit_rule_F2h.py` (logs to `logs/f2h/`): crash → A4 excluded = F2 fit (diff 0.0 vs c9061854 F2 beta on r0 data); no crash → identical to current code; flag OFF → identical to current code.
d. Send `F2h: fit rule built, tests a-c <results>` before restarting.

## 4. Queue
Restart **dryF_both from scratch** (marker `--- restart after F2h fit rule`), then rtestF_both, gate9F_both. Per round, report:
- the F2h decision and the model's A4 tilt;
- beta and its L2 error vs true P1 (compare: rom r0–r5 0.592, 0.440, 0.131, 0.053, 0.119, 0.119);
- frsim 3-D RMS for all cells, without A4, per cell;
- noisy A1 J (fixed seeds) and frsim noise-free A1 J;
- guard drops (fit/tune), crash flags.

## 5. Report and finish
- Section `J. F2h fit rule` in `experiments/diffcotune/fbl/report_F2c.md`, plus the owed `F2g rule B` section and the final per-variant lab-ready tables for base, rom and both.
- Commit explicit paths, no push, and send `Stage F2c/F2d/F2e/F2f/F2g/F2h done: experiments/diffcotune/fbl/report_F2c.md`.
- **Success criterion for `both`:** beta L2 error falls over the rounds (as in rom) and the final all-cell 3-D RMS is below the round-0 value (7.15 cm). If not, stop at the end and report; do not invent a further rule.
