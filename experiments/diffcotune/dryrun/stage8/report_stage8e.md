# Stage 8e report (generated 17 Nov 2026 17:45 by `make_report8e.py`; the planning agent validates before anything is committed or flown)

Scope (user, 17 Nov): focus is the real lab sessions; no S1 / S3 dry-run reruns, no plant switch. Rule RS, both S0 / S2 regressions, tests, readers.

## 0. T0 Stage 8d reporting

`report_stage8d.md`, `LAB_TOMORROW.md`, `report_stage8.md` regenerated from the finished `c8.sh` (ALL DONE 08:03:36, 13 jobs rc 0, `logs/slotC8.log`). Real 3-D cost per round (cm, as flown):

| Controller | Rounds | θ_best | Round test | Gradient check |
|---|---|---|---|---|
| S0 | 18.26, 9.24, 7.35, 6.39, 7.18, 5.53 | round 5 | exact | PASS |
| S2 | 14.47, 9.64, 7.85, 6.72, 5.98, 5.92 | round 5 | exact | FAIL on A1 task only, accepted by the user 17 Nov |
| S1 | 9.54, 6.22, 94.81, nan, nan, nan | round 1 | exact | PASS |
| S3 | 10.07, 9.71, 7.21, nan, nan, nan | round 2 | exact | PASS |

Numbers match the handover table of 17 Nov. Reader fix: the queue-time cutoff in `make_report8c.py` / `make_report8d.py` used today's date; it is now 16 Nov.

## 1. T1 / T3 / T4: rule RS (user decision 17 Nov)

Rule: (a) a round with an R3 scenario and an existing θ_best starts its tuning step from θ_best (box ± 25 % around θ_best; the β fit is unchanged); (b) R3 in round l and in round l − 1 with θ_best existing: no further tuning, result θ_best, "stopped: two R3 rounds in a row"; everything else unchanged.

| Item | Files (copies `*.before_stage8e` kept) | Change | Test | Result |
|---|---|---|---|---|
| T1 RS in the dry run | `dryrun/loop_lib.py`, `stage8/run_8.py` | `DIFFCOTUNE_RESTART_BEST` (default 0 = Stage 8d, only with `DIFFCOTUNE_DRY_STRICT=1`); per round `restart_from_best`, `restart_theta_best_round`, `prev_r3`, `rs_stop`; run `halt = rs_stop = "two R3 rounds in a row"`; `run_8.py` keeps that stop reason | `stage8/test_restart_rule.py` (fake pool: restart, round 0, two in a row, not consecutive, no θ_best, RS = 0) | 11 checks, all PASS |
| T1 strict rule unchanged with RS off | same | – | `stage8/test_strict_rule.py` | 16 checks, all PASS |
| T3 RS in `round.py` | `lab/round.py`, marker `RS_VERSION = 'RS-2026-11-17'` | R3 round with θ_best: tuning from θ_best (printed, `report.json` / `report.md` / `state.json` history `restart_from_best`, `prev_r3`); stop rule from the state history: message "Tuning stopped: two R3 rounds in a row. Final gains = θ_best (round k). Fly the test flights with θ_best.", `gains_theta_best.yaml` written, no next round | `lab/test_r3_mock.py --rs` | 6 checks, all PASS |
| T3 earlier mock cases unchanged | – | – | `lab/test_r3_mock.py` (R3), `--ext` (extension rounds, fallback, manifest) | 7 checks, all PASS; 9 checks, all PASS |
| T4 (c) regression, all flights usable | `lab/test_r3_mock.py --regress` | `round.py` now against `round.py.before_stage8d`, same folder, θ₁ and β after the fit | `lab/results/test_r3_mock_8e_regress.txt` | PASS b regression, all flights usable: theta_1 diff 0.0e+00, beta diff 0.0e+00, real cost equal , PASS b new fields only added (eligible True, no r3 block)  |
| T3 `preflight.py` | `lab/preflight.py` | check 6d (RS marker and stop text in `round.py`) | `preflight.py --controller S0 / S1 / S2 / S3` (`lab/results/preflight_<C>_8e.txt`) | 6d PASS, PREFLIGHT PASSED for all four |
| T3 cards | `lab/S0_LAB_CARD.md`, `lab/INTERLEAVED_LAB_CARD.md` | rule RS and the stop rule (item 9a / 5b) | – | done |
| Dropped (user 17 Nov) | – | plant switch, S1 / S3 reruns, `dry9r_*` / `rtest9r_*` / `gate9r_*`; the code of the plant switch was reverted (`dry_test5.py` identical to `.before_stage8e`) | – | – |

## 2. Queue c9.sh (the two RS regressions)

| Job | start | end | rc | key result |
|---|---|---|---|---|
| regress9r_S0 | 11:28:21 | 14:04:53 | 0 | PASS (bit-identical): max abs diff {'phi': '0.0e+00', 'psi': '0.0e+00', 'psi_after_fit': '0.0e+00', 'real_cost': '0.0e+00', 'real_used_cost': '0.0e+00', 'attempt_costs': '0.0e+00', 'tune_J_best': '0.0e+00'}; structure equal |
| regress9r_S2 | 14:04:59 | 16:51:33 | 0 | PASS (bit-identical): max abs diff {'phi': '0.0e+00', 'psi': '0.0e+00', 'psi_after_fit': '0.0e+00', 'real_cost': '0.0e+00', 'real_used_cost': '0.0e+00', 'attempt_costs': '0.0e+00', 'tune_J_best': '0.0e+00'}; structure equal |

Comparison (`stage8/compare_rs.py`): per round phi, psi, psi after the fit, real cost as flown and over used flights, attempt costs, tuning J_best; structure: eligibility, R3 scenarios, attempts, keep-best set, no RS action. PASS only if every difference is 0.0.

## 3. Lab-ready table

| Controller | Stage 8d strict dry run: real cost per round (cm, as flown) | a / b / c | round test (strict, round 0) | gradient check | preview (old runs) | preflight (R3, extension, RS markers) | RS mock tests | Remark |
|---|---|---|---|---|---|---|---|---|
| S0 | 18.26 → 9.24 → 7.35 → 6.39 → 7.18 → 5.53 | yes / yes / yes | exact (θ₁ 0e+00, β 0e+00) | PASS | 2 of 3 rounds PASSED | PASS (6b, 6c, 6d; PREFLIGHT PASSED) | 6 checks, all PASS (`--rs`); 7 checks, all PASS (R3 / 8d cases); 9 checks, all PASS (`--ext`); 2 checks, all PASS (`--regress`) | criteria a / b / c = yes / yes / yes |
| S2 | 14.47 → 9.64 → 7.85 → 6.72 → 5.98 → 5.92 | yes / yes / yes | exact (θ₁ 0e+00, β 0e+00) | FAIL on A1 task only, accepted by the user 17 Nov | 5 of 5 rounds PASSED | PASS (6b, 6c, 6d; PREFLIGHT PASSED) | same (controller-independent) | criteria a / b / c = yes / yes / yes |
| S1 | 9.54 → 6.22 → 94.81 → nan → nan → nan | no / no / yes | exact (θ₁ 0e+00, β 0e+00) | PASS | not run | PASS (6b, 6c, 6d; PREFLIGHT PASSED) | same (controller-independent) | dry-run behaviour under an unfittable 8 ms delay (plant P1RD, RPM delay fixed at 4.0502 ms in the model); in the lab covered by RS + stop rule. Crashed rounds: [2, 3, 4, 5]; dry-run criteria a / b / c = no / no / yes |
| S3 | 10.07 → 9.71 → 7.21 → nan → nan → nan | no / no / yes | exact (θ₁ 0e+00, β 0e+00) | PASS | not run | PASS (6b, 6c, 6d; PREFLIGHT PASSED) | same (controller-independent) | dry-run behaviour under an unfittable 8 ms delay (plant P1RD, RPM delay fixed at 4.0502 ms in the model); in the lab covered by RS + stop rule. Crashed rounds: [3, 4, 5]; dry-run criteria a / b / c = no / no / yes |

Gradient check S2: FAIL on A1 task only (round 0: 5.7e-3, round 4: 1.9e-3, limit 1e-3), every other check passes; accepted by the user 17 Nov; gate and threshold unchanged.
S1 / S3: the Stage 8d dry-run crashes (all five scenarios R3 from round 2 / round 3, β not refitted) are dry-run behaviour under an unfittable 8 ms RPM delay (the real drones were measured at 4.05 ms); in the lab covered by RS + stop rule. Lab readiness of S1 / S3 is judged on the round tests, gradient checks, preflight and the RS mock tests (user, 17 Nov).

## 4. Open items (who decides)

* **Keep-best eligibility rule** confirmed by the user 16 Nov (a set with an R3 scenario is never θ_best).
* Which controller order, and the lab session plan: user / planning agent (not stated).
* Two-at-once desk times, the switch overhead, flights per pack and which PC runs `round.py`: not stated (carried from Stage 8c).
* Dry-run S1 / S3 results under RS: not run (scope change); RS in the dry run is proven by the fake-pool test and, for S0 / S2, by the regressions above.
