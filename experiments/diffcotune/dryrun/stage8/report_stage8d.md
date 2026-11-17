# Stage 8d report (generated 17 Nov 2026 11:16 by `make_report8d.py`; the planning agent validates before anything is committed or flown)

## 1. Changes (T1–T3) and tests

| Item | Files (copies `*.before_stage8d` kept) | Change | Test | Result |
|---|---|---|---|---|
| T1 strict switch | `dryrun/loop_lib.py` | `DIFFCOTUNE_DRY_STRICT` (default 0 = old path): `fly_set_strict` (repeat crash and floor flights, ≤ 3 attempts, seed + 1000 × tries, all attempts returned); `run_loop`: fit on usable flights only (scenario subset), R3 fallback flight with θ_best (not used), keep-best over eligible sets with cost over used flights, "no eligible set" → θ_0 marked; as-flown cost kept as `real`, `real_used`, `attempts`, `used_attempt`, `failed_all`, `fallback`, `eligible`, `r3_scenarios` added | `dryrun/stage8/test_strict_rule.py` (fake pool): all-3-fail → 3 attempts + fallback with θ_best + 4-scenario fit + not eligible; floor on attempt 1 only → attempt 2 used; round 0 with 3 fails → no fallback; no eligible set → θ_0 marked; fallback uses θ_best of the eligible sets | ALL PASS |
| T1 regression strict = 0 | `stage8/regress8s.sh`, `compare_regress8s.py` | round 0 of S0 "ours" rerun against `loop_S0_ours.json` (θ₁ from `tune.log[best_epoch].x`, β after the fit, real cost) | queue job `regress8s_S0` | PASS: θ₁ diff 0.0e+00, β diff 0.0e+00, real cost diff 0.0e+00 |
| T2 R3 in `round.py` | `lab/round.py` | stop at "no usable flight" replaced by R3 for a scenario with attempts 1, 2, 3 all failed (manifest role `fallback`, attempt 4, flown with θ_best); missing fallback → exact message with the θ_best gains file, nothing written; fit on the usable flights, set not eligible (state.json keep unchanged, history `eligible: false`), `report.json` fields `eligible`, `r3`, `r3_version`; `cell_of` treats role `fallback` like `tuning`; version marker `R3-2026-11-16` | `lab/test_r3_mock.py` (fake uSD files, epochs 2, one worker): `lab/results/test_r3_mock_a.txt` | a: 8 PASS lines, ALL PASS |
| T2 regression (all flights usable) | `lab/test_r3_mock.py --regress` | `round.py` now against `round.py.before_stage8d`, same folder (`dry5_S0_8o/round0_logs`), lenient as the old dry test, epochs 2 | `lab/results/test_r3_mock_b.txt` | PASS b regression, all flights usable: theta_1 diff 0.0e+00, beta diff 0.0e+00, real cost equal , ALL PASS |
| T2 `dry_test5.py` | `lab/dry_test5.py` | `--strict-dry`: flights repeated like the lab (≤ 3 attempts), `round.py` strict; reference `loop_<C>_ours_strict.json`; outputs `dry_test5_<C>_8s.json`; default path unchanged | round tests `rtest8s_<C>` (queue) | see §2 |
| T2 `preflight.py` | `lab/preflight.py` | new check 6b (R3 version marker in `round.py`) | `preflight.py --controller S0` | 6b PASS, PREFLIGHT PASSED (13:18) |
| T2 cards | `lab/S0_LAB_CARD.md`, `lab/INTERLEAVED_LAB_CARD.md` | item 9a / 5b (R3 step, +1 flight per R3 case), rule rows | – | done |
| **8d2** extension rounds, dry run | `dryrun/loop_lib.py` (`*.before_stage8d2`), `stage8/run_8.py` (no evaluation when there is no eligible set) | strict mode only: no eligible set after round 5 → extension rounds l = 6, 7 (same method, fit and tuning step after round 5 as in any round); record `extension_rounds_used`; still none → `best` has no gains, note "no eligible set after extension: ask the user", never θ_0; default path bit-identical (strict = 0) | `stage8/test_strict_rule.py` (fake pool) | 16 checks, all PASS |
| **8d2** extension rounds, lab | `lab/round.py` (`*.before_stage8d2`), `lab/session.py` status, `lab/preflight.py` 6c | last planned round without eligible set: no final gains file, `round_<l+1>/` written, `extension_round_added`; after round 7 without eligible set: `NO_ELIGIBLE_SET.md`, message "no eligible set after extension: ask the user", no gains file | `lab/test_r3_mock.py --ext` (c–f, 2 epochs, one worker) | 4 checks, all PASS |
| **8d2** fallback-flight failure rule + `session.py manifest --fallback` | `lab/round.py` R3 message, `lab/session.py` (`--fallback SCEN=file --fallback-gains <θ_best gains.yaml> --append`: role fallback, attempt 4, one per scenario), both lab cards | message and cards: if the fallback flight fails, pause, fix drone and setup (battery, props, mocap, EKF toggle), fly it once more, the round continues either way (`report.json` r3.fallback_flags); a stopped lab day resumes from the saved round state | `lab/test_r3_mock.py --ext` (g, h1–h3, i) | 5 checks, all PASS |
| T3 readers | `make_lab_tomorrow.py`, `make_report8.py`, `make_report8c.py` unchanged, new `make_report8d.py` | tag `ours_strict` rendered next to old "ours" (cost as flown / used, attempts, R3 cases, eligibility, a/b/c); old files render as before | – | in use |

## 2. Queue c8.sh (started 13:18:47; waits for `logs/dry8o_S2.rc`)

Old-mode S2 run `dry8o_S2` (before run): rc 0, ended 14:52:09

| Job | start | end | rc | key result |
|---|---|---|---|---|
| regress8s_S0 | 14:52:18 | 15:24:43 | 0 | PASS: θ₁ diff 0.0e+00, β diff 0.0e+00, real cost diff 0.0e+00 |
| dry8s_S0 | 15:24:50 | 17:57:45 | 0 | cost as flown 18.26 → 9.24 → 7.35 → 6.39 → 7.18 → 5.53; used 8.43 → 9.24 → 7.35 → 6.39 → 7.18 → 5.53 |
| rtest8s_S0 | 17:57:55 | 18:31:38 | 0 | θ₁ diff 0.0e+00, β diff 0.0e+00 (R3 in round 0: ['A1']) |
| dry8s_S2 | 18:31:48 | 21:22:21 | 0 | cost as flown 14.47 → 9.64 → 7.85 → 6.72 → 5.98 → 5.92; used 7.08 → 9.64 → 7.85 → 6.72 → 5.98 → 5.92 |
| rtest8s_S2 | 21:22:28 | 22:00:36 | 0 | θ₁ diff 0.0e+00, β diff 0.0e+00 (R3 in round 0: ['A1']) |
| gate9s_S0 | 22:00:42 | 22:49:02 | 0 | PASS at the fitted β of 6 rounds (max adopted-vs-FD 1.7e-04) |
| gate9s_S2 | 22:49:09 | 23:55:18 | 0 | FAIL at the fitted β of 6 rounds (max adopted-vs-FD 5.7e-03; failing rounds [0, 4]) |
| dry8s_S1 | 23:55:26 | 01:59:20 | 0 | cost as flown 9.54 → 6.22 → 94.81 → nan → nan → nan; used 9.54 → 6.22 → n/a → n/a → n/a → n/a |
| rtest8s_S1 | 01:59:29 | 02:37:03 | 0 | θ₁ diff 0.0e+00, β diff 0.0e+00 (R3 in round 0: None) |
| gate9s_S1 | 02:37:13 | 03:35:41 | 0 | PASS at the fitted β of 6 rounds (max adopted-vs-FD 7.8e-05) |
| dry8s_S3 | 03:35:47 | 06:09:06 | 0 | cost as flown 10.07 → 9.71 → 7.21 → nan → nan → nan; used 10.07 → 9.71 → 7.21 → n/a → n/a → n/a |
| rtest8s_S3 | 06:09:13 | 06:52:32 | 0 | θ₁ diff 0.0e+00, β diff 0.0e+00 (R3 in round 0: None) |
| gate9s_S3 | 06:52:38 | 08:03:26 | 0 | PASS at the fitted β of 6 rounds (max adopted-vs-FD 7.1e-05) |

Dropped (superseded by the strict runs): `rtest8o_S2`, `gate9o_S2`, `gate9o_S0`, `dry8o_S1`, `rtest8o_S1`, `gate9o_S1`, `dry8o_S3`, `rtest8o_S3`, `gate9o_S3` and the paper gate `gate9_S1`. `c7.sh` was stopped at 12:42:50.

```
13:18:47 c8.sh waits for logs/dry8o_S2.rc
14:52:18 dry8o_S2 rc 0
14:52:18 start regress8s_S0
15:24:50 done regress8s_S0 rc 0
15:24:50 start dry8s_S0
17:57:55 done dry8s_S0 rc 0
17:57:55 start rtest8s_S0
18:31:48 done rtest8s_S0 rc 0
18:31:48 start dry8s_S2
21:22:28 done dry8s_S2 rc 0
21:22:28 start rtest8s_S2
22:00:42 done rtest8s_S2 rc 0
22:00:42 start gate9s_S0
22:49:09 done gate9s_S0 rc 0
22:49:09 start gate9s_S2
23:55:26 done gate9s_S2 rc 0
23:55:26 start dry8s_S1
01:59:29 done dry8s_S1 rc 0
01:59:29 start rtest8s_S1
02:37:13 done rtest8s_S1 rc 0
02:37:13 start gate9s_S1
03:35:47 done gate9s_S1 rc 0
03:35:47 start dry8s_S3
06:09:13 done dry8s_S3 rc 0
06:09:13 start rtest8s_S3
06:52:38 done rtest8s_S3 rc 0
06:52:38 start gate9s_S3
08:03:36 done gate9s_S3 rc 0
08:03:36 ALL DONE

```

## 3. Answer: does the bad round still happen with the lab's rule?

Controllers S0 and S2 first. Cost in cm, 3-D RMS, mean of the 5 tuning flights of a round; "as flown" = last attempt of each scenario (the old definition), "used" = mean over the usable flights.

### S0: old "ours" against "ours_strict" (lab flight rule + R3)

Old "ours" (as flown): 18.24 → 9.69 → 9.14 → 15.96 → 4.78 → 3.31; a / b / c = no / no / yes; keep-best round 5.

| Round | cost as flown (cm) | cost over used flights (cm) | attempts (5 scenarios) | R3 cases | fallback flights | eligible | floor / crash flags of the last attempts | desk min |
|---|---|---|---|---|---|---|---|---|
| 0 | 18.26 | 8.43 | 7 | A1 | 0 | no | 1 floor / 0 crash | 31 |
| 1 | 9.24 | 9.24 | 5 | – | 0 | yes | 0 floor / 0 crash | 29 |
| 2 | 7.35 | 7.35 | 5 | – | 0 | yes | 0 floor / 0 crash | 31 |
| 3 | 6.39 | 6.39 | 5 | – | 0 | yes | 0 floor / 0 crash | 31 |
| 4 | 7.18 | 7.18 | 5 | – | 0 | yes | 0 floor / 0 crash | 30 |
| 5 | 5.53 | 5.53 | 5 | – | 0 | yes | 0 floor / 0 crash | 0 |

Per scenario, all attempts (cm; F = floor, C = crash; * = used; fallback in brackets):

| Round | A1 | A8 | CIRCLE | FIG8 | A4 |
|---|---|---|---|---|---|
| 0 | 37.1F / 37.2F / 37.2F | 11.8* | 6.5* | 6.1* | 8.0* |
| 1 | 16.3* | 7.2* | 5.5* | 5.2* | 7.2* |
| 2 | 13.0* | 5.6* | 3.2* | 3.2* | 7.0* |
| 3 | 10.9* | 4.9* | 2.7* | 2.7* | 6.8* |
| 4 | 13.9* | 5.5* | 1.9* | 2.0* | 5.1* |
| 5 | 9.2* | 5.8* | 2.0* | 2.0* | 5.1* |

Status: complete.
* Keep-best: round 5; eligible sets: rounds [1, 2, 3, 4, 5].
* Criteria (definitions of make_lab_tomorrow.py:23–25; b on "no usable attempt"): a **yes**, b **yes**, c **yes**.
* A round worse than round 1 + 1 cm (10.24 cm as flown): none so far.
* Means θ₀ → θ_best (cm, seeds 2000–2004): tuning 14.04 → 4.87; held-out A4 cell 2 13.05 → 8.31; in-sample A4 cell 1 8.80 → 5.11.
* Round test (strict): θ₁ difference 0.0e+00, β difference 0.0e+00, R3 in round 0: {'scenarios': ['A1'], 'fallback_files': {}, 'theta_best_round': None, 'fallback_flags': {}, 'note': 'fallback flights are not used for the fit, the cost or theta_best; this set is not eligible for keep-best; a failed fallback flight is only recorded (pause, fix the setup, fly it once more, the round continues either way)'}.
* Gradient check: PASS.

### S2: old "ours" against "ours_strict" (lab flight rule + R3)

Old "ours" (as flown): 14.38 → 8.09 → 6.88 → 6.55 → 6.38 → 5.63; a / b / c = yes / yes / yes; keep-best round 5.

| Round | cost as flown (cm) | cost over used flights (cm) | attempts (5 scenarios) | R3 cases | fallback flights | eligible | floor / crash flags of the last attempts | desk min |
|---|---|---|---|---|---|---|---|---|
| 0 | 14.47 | 7.08 | 7 | A1 | 0 | no | 1 floor / 0 crash | 39 |
| 1 | 9.64 | 9.64 | 5 | – | 0 | yes | 0 floor / 0 crash | 32 |
| 2 | 7.85 | 7.85 | 5 | – | 0 | yes | 0 floor / 0 crash | 33 |
| 3 | 6.72 | 6.72 | 5 | – | 0 | yes | 0 floor / 0 crash | 33 |
| 4 | 5.98 | 5.98 | 5 | – | 0 | yes | 0 floor / 0 crash | 32 |
| 5 | 5.92 | 5.92 | 5 | – | 0 | yes | 0 floor / 0 crash | 0 |

Per scenario, all attempts (cm; F = floor, C = crash; * = used; fallback in brackets):

| Round | A1 | A8 | CIRCLE | FIG8 | A4 |
|---|---|---|---|---|---|
| 0 | 28.9F / 29.0F / 29.1F | 9.1* | 6.1* | 5.8* | 6.8* |
| 1 | 18.9* | 6.6* | 3.9* | 3.8* | 5.7* |
| 2 | 13.5* | 6.8* | 2.2* | 2.4* | 8.4* |
| 3 | 12.0* | 4.7* | 2.1* | 2.3* | 7.1* |
| 4 | 10.0* | 4.3* | 2.0* | 2.2* | 7.2* |
| 5 | 10.8* | 4.4* | 2.0* | 2.2* | 5.6* |

Status: complete.
* Keep-best: round 5; eligible sets: rounds [1, 2, 3, 4, 5].
* Criteria (definitions of make_lab_tomorrow.py:23–25; b on "no usable attempt"): a **yes**, b **yes**, c **yes**.
* A round worse than round 1 + 1 cm (10.64 cm as flown): none so far.
* Means θ₀ → θ_best (cm, seeds 2000–2004): tuning 11.25 → 4.85; held-out A4 cell 2 8.71 → 6.57; in-sample A4 cell 1 6.81 → 5.36.
* Round test (strict): θ₁ difference 0.0e+00, β difference 0.0e+00, R3 in round 0: {'scenarios': ['A1'], 'fallback_files': {}, 'theta_best_round': None, 'fallback_flags': {}, 'note': 'fallback flights are not used for the fit, the cost or theta_best; this set is not eligible for keep-best; a failed fallback flight is only recorded (pause, fix the setup, fly it once more, the round continues either way)'}.
* Gradient check: FAIL, see results/grad_gate9_S2_ours_strict.json.

### S1: old "ours" against "ours_strict" (lab flight rule + R3)


| Round | cost as flown (cm) | cost over used flights (cm) | attempts (5 scenarios) | R3 cases | fallback flights | eligible | floor / crash flags of the last attempts | desk min |
|---|---|---|---|---|---|---|---|---|
| 0 | 9.54 | 9.54 | 5 | – | 0 | yes | 0 floor / 0 crash | 36 |
| 1 | 6.22 | 6.22 | 5 | – | 0 | yes | 0 floor / 0 crash | 33 |
| 2 | 94.81 | n/a | 15 | A1, A8, CIRCLE, FIG8, A4 | 5 | no | 5 floor / 5 crash | 18 |
| 3 | nan | n/a | 15 | A1, A8, CIRCLE, FIG8, A4 | 5 | no | 3 floor / 5 crash | 17 |
| 4 | nan | n/a | 15 | A1, A8, CIRCLE, FIG8, A4 | 5 | no | 0 floor / 5 crash | 18 |
| 5 | nan | n/a | 15 | A1, A8, CIRCLE, FIG8, A4 | 5 | no | 2 floor / 5 crash | 0 |

Per scenario, all attempts (cm; F = floor, C = crash; * = used; fallback in brackets):

| Round | A1 | A8 | CIRCLE | FIG8 | A4 |
|---|---|---|---|---|---|
| 0 | 17.7* | 7.0* | 5.9* | 5.2* | 5.4* |
| 1 | 12.0* | 5.3* | 2.7* | 2.4* | 3.1* |
| 2 | 50.9FC / 50.5FC / 51.2FC [fallback θ_best round 1: 12.4] | 70.2FC / 68.4FC / 76.2FC [fallback θ_best round 1: 6.3] | 112.0FC / 148.1FC / 149.6FC [fallback θ_best round 1: 2.5] | nanC / 115.8FC / 103.6FC [fallback θ_best round 1: 2.6] | 66.5FC / 73.0FC / 58.4FC [fallback θ_best round 1: 3.2] |
| 3 | nanC / nanC / nanC [fallback θ_best round 1: 12.8] | nanC / nanC / 50.0FC [fallback θ_best round 1: 6.0] | nanC / nanC / nanC [fallback θ_best round 1: 2.7] | nanC / nanC / 118.9FC [fallback θ_best round 1: 2.6] | nanC / 79.7FC / 72.5FC [fallback θ_best round 1: 3.0] |
| 4 | 50.0FC / 50.0FC / nanC [fallback θ_best round 1: 12.4] | 60.8FC / 50.0FC / nanC [fallback θ_best round 1: 5.7] | 131.1FC / 130.2FC / nanC [fallback θ_best round 1: 2.6] | 129.2FC / 125.8FC / nanC [fallback θ_best round 1: 2.4] | nanC / 72.8FC / nanC [fallback θ_best round 1: 3.3] |
| 5 | nanC / 50.0FC / 50.0FC [fallback θ_best round 1: 12.9] | 57.4FC / 50.0FC / nanC [fallback θ_best round 1: 5.6] | nanC / nanC / nanC [fallback θ_best round 1: 2.4] | 125.3FC / nanC / 127.1FC [fallback θ_best round 1: 2.3] | 56.3FC / 99.8FC / nanC [fallback θ_best round 1: 3.2] |

Status: complete.
* Keep-best: round 1; eligible sets: rounds [0, 1].
* Criteria (definitions of make_lab_tomorrow.py:23–25; b on "no usable attempt"): a **no**, b **no**, c **yes**.
* A round worse than round 1 + 1 cm (7.22 cm as flown): round 2 (94.81).
* Means θ₀ → θ_best (cm, seeds 2000–2004): tuning 8.30 → 5.26; held-out A4 cell 2 7.38 → 3.28; in-sample A4 cell 1 5.33 → 3.03.
* Round test (strict): θ₁ difference 0.0e+00, β difference 0.0e+00, R3 in round 0: None.
* Gradient check: PASS.

### S3: old "ours" against "ours_strict" (lab flight rule + R3)


| Round | cost as flown (cm) | cost over used flights (cm) | attempts (5 scenarios) | R3 cases | fallback flights | eligible | floor / crash flags of the last attempts | desk min |
|---|---|---|---|---|---|---|---|---|
| 0 | 10.07 | 10.07 | 5 | – | 0 | yes | 0 floor / 0 crash | 42 |
| 1 | 9.71 | 9.71 | 5 | – | 0 | yes | 0 floor / 0 crash | 35 |
| 2 | 7.21 | 7.21 | 5 | – | 0 | yes | 0 floor / 0 crash | 34 |
| 3 | nan | n/a | 15 | A1, A8, CIRCLE, FIG8, A4 | 5 | no | 4 floor / 5 crash | 20 |
| 4 | nan | n/a | 15 | A1, A8, CIRCLE, FIG8, A4 | 5 | no | 0 floor / 5 crash | 20 |
| 5 | nan | n/a | 15 | A1, A8, CIRCLE, FIG8, A4 | 5 | no | 0 floor / 5 crash | 0 |

Per scenario, all attempts (cm; F = floor, C = crash; * = used; fallback in brackets):

| Round | A1 | A8 | CIRCLE | FIG8 | A4 |
|---|---|---|---|---|---|
| 0 | 18.9* | 6.6* | 5.8* | 5.1* | 6.7* |
| 1 | 18.3* | 6.2* | 6.6* | 5.3* | 5.4* |
| 2 | 12.1* | 6.4* | 4.7* | 4.4* | 5.5* |
| 3 | 43.9FC / 49.8FC / 45.7FC [fallback θ_best round 2: 13.1] | 67.0FC / 78.0FC / 60.9FC [fallback θ_best round 2: 4.9] | 145.9FC / 109.1FC / 111.0FC [fallback θ_best round 2: 4.9] | 96.3FC / nanC / nanC [fallback θ_best round 2: 4.2] | 68.3FC / 88.0FC / 67.2FC [fallback θ_best round 2: 5.5] |
| 4 | nanC / nanC / nanC [fallback θ_best round 2: 13.1] | nanC / nanC / nanC [fallback θ_best round 2: 4.6] | nanC / nanC / nanC [fallback θ_best round 2: 4.8] | nanC / nanC / nanC [fallback θ_best round 2: 4.0] | nanC / nanC / nanC [fallback θ_best round 2: 5.6] |
| 5 | nanC / nanC / nanC [fallback θ_best round 2: 12.2] | nanC / nanC / nanC [fallback θ_best round 2: 4.6] | nanC / nanC / nanC [fallback θ_best round 2: 4.7] | nanC / nanC / nanC [fallback θ_best round 2: 4.2] | nanC / nanC / nanC [fallback θ_best round 2: 5.7] |

Status: complete.
* Keep-best: round 2; eligible sets: rounds [0, 1, 2].
* Criteria (definitions of make_lab_tomorrow.py:23–25; b on "no usable attempt"): a **no**, b **no**, c **yes**.
* A round worse than round 1 + 1 cm (10.71 cm as flown): none so far.
* Means θ₀ → θ_best (cm, seeds 2000–2004): tuning 8.38 → 6.30; held-out A4 cell 2 7.02 → 5.94; in-sample A4 cell 1 5.82 → 5.41.
* Round test (strict): θ₁ difference 0.0e+00, β difference 0.0e+00, R3 in round 0: None.
* Gradient check: PASS.

## 4. Lab-ready table (strict runs)

| Controller | strict dry run | a / b / c | round test (strict) | gradient check | eligible sets / keep-best | Lab-ready | Reason |
|---|---|---|---|---|---|---|---|
| S0 | complete | yes / yes / yes | exact | PASS | [1, 2, 3, 4, 5] / 5 | **yes** | all conditions met |
| S2 | complete | yes / yes / yes | exact | FAIL | [1, 2, 3, 4, 5] / 5 | **no** | gradient check FAIL |
| S1 | complete | no / no / yes | exact | PASS | [0, 1] / 1 | **no** | criterion a not met; criterion b not met |
| S3 | complete | no / no / yes | exact | PASS | [0, 1, 2] / 2 | **no** | criterion a not met; criterion b not met |

## 5. Open items

* **Keep-best eligibility rule = consequence of R3, user to confirm** (a set with an R3 scenario is not eligible; cost over the used flights; no eligible set after round 5 → up to 2 extension rounds (l = 6, 7, same method), then "no eligible set after extension: ask the user", never θ_0). **Rule confirmed by the user 16 Nov** (Stage 8d2).
* S0 criteria a / b / c are evaluated on the strict runs; no criterion was changed (user).
* `session.py manifest` does not write the `fallback` entry: it is added to `flights.json` by hand (card item 9a / 5b); whether `session.py` should do it: user / planning agent.
* In the dry run a fallback flight with θ_best is itself recorded but not judged (no rule stated for a failed fallback flight): user.
* Preview, session timeline (T6) with the strict desk minutes, and the S2 preview replay (T4) follow when the runs are complete.
