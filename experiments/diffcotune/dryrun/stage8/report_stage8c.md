# Stage 8c report (generated 17 Nov 2026 11:16 by `make_report8c.py`; the planning agent validates before anything is committed or flown)

## 1. Queue (`c7.sh`, PID 3110006)

| Job | start | end | rc | key result |
|---|---|---|---|---|
| dry8o_S2 | 11:56:00 | 14:52:09 | 0 | real cost 14.38 → 8.09 → 6.88 → 6.55 → 6.38 → 5.63 |
| rtest8o_S2 | not yet | not yet | – | not yet |
| gate9o_S2 | not yet | not yet | – | not yet |
| gate9o_S0 | not yet | not yet | – | not yet |
| dry8o_S1 | not yet | not yet | – | not yet |
| rtest8o_S1 | not yet | not yet | – | not yet |
| gate9o_S1 | not yet | not yet | – | not yet |
| dry8o_S3 | not yet | not yet | – | not yet |
| rtest8o_S3 | not yet | not yet | – | not yet |
| gate9o_S3 | not yet | not yet | – | not yet |
| gate9_S1 | not yet | not yet | – | not yet |

Slot log (`logs/slotC7.log`):
```
11:56:00 start dry8o_S2
12:42:50 c7.sh stopped by the planning agent (user decision C, 16 Nov); dry8o_S2 keeps running; strict queue c8.sh follows
```

## 2. Tasks

| Task | Status | Path |
|---|---|---|
| T1 queue watch | running; see §1 | `logs/slotC7.log` |
| T2 `report_stage8.md` | script done, re-run after every job (§5 and §7 are filled as results arrive) | `make_report8.py`, `report_stage8.md` |
| T3 S0 round-3 decision sheet | done | `S0_ROUND3_DECISION.md` (`make_s0_round3_sheet.py`) |
| T4 S2 preview replay | not yet (waits for `dry8o_S2`) | `results/preview_S2_ours_partial.json` kept |
| T5 preview mock S1 / S3 | PASSED for S1 and S3 (mock covers both; A8 only, same set → PASSED, crashing set → FAILED; rc 0 both) | `logs/prevmock_S1S3_stage8c.log`; `rtest8o_S1` / `rtest8o_S3` remain the full test |
| T6 session timeline | not yet (needs 6 rounds of `loop_S2_ours.json`; old files kept as `lab/results/session_timeline.before_stage8c.*`) | `session_timeline8c.py` |

## 3. Lab-ready table ("ours")

| Controller | dry run | a / b / c | round test | gradient check | preview | Lab-ready | Reason |
|---|---|---|---|---|---|---|---|
| S0 | complete | no / no / yes | exact (θ₁ 0e+00, β 0e+00) | not yet | r1 P / r2 F / r3 P (P = passed, F = failed) | **no** | criterion a not met; criterion b not met; gradient check not yet |
| S2 | complete | yes / yes / yes | not yet | not yet | r1 P / r2 P / r3 P / r4 P / r5 P (P = passed, F = failed) | **not yet** | round test not yet; gradient check not yet |
| S1 | not yet | – | not yet | not yet | not yet | **not yet** | dry run missing |
| S3 | not yet | – | not yet | not yet | not yet | **not yet** | dry run missing |

S0 criteria a and b: see `S0_ROUND3_DECISION.md` (decision of the user). Preflight (`logs/pre_<C>.log`): see the Stage 8c report.

## 4. Open items

* S0 criteria a and b (round 3 A1 floor flight): user (`S0_ROUND3_DECISION.md`).
* `loop_S2_ours.json` / `LAB_TOMORROW.md` showed 2 S2 rounds until the queue run wrote its own: that was the stopped run's file (byte-identical copy `loop_S2_ours_stopped2104.json`); flagged in the generated reports: planning agent.
* Two-at-once desk times for "ours" are not measured (session timeline uses cases A / B): user.
* Switch and `--solo` relaunch overhead, flights per pack, which PC runs `round.py`: not stated (user).
