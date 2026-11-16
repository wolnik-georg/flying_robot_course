# Stage 5b report: forward-mode gradient for S1, S2, S3 (DiffCoTune; desk only; 14 Nov 2026)

Desk only. Nothing flown, flashed or connected; no commit, checkout or stash; no Co-Authored-By. Every job ran through `dryrun/stage3c/job_runner.py`, one pool at a time, `nice -n 19`, load and memory read before and one minute after each start (load stayed ≤ 4.1, MemAvailable ≥ 18 GB; a forward-mode pool needs no more memory than a reverse-mode pool). Sources: `results/diag_b1.json`, `diag_b2.json`, `fwd_checks_S*.json`, `noise_hscan_S*.json`, logs in `logs/`.

**Result in one paragraph.** The forward-mode gradient (`torch.func.jacfwd`, eager) is exact: it passes the combined gradient gate (b) for S1, S2, S3 (and S0, for information). Without noise, compiled reverse mode equals forward mode to 1e-14 on A8, CIRCLE and FIG8 and fails on A1 only (1e27 to 1e32 times too large for S1 / S3, 0.2 relative for S2, 4e-3 for S0). With noise, reverse mode on A1 equals forward mode to ≤ 2e-11. Forward mode costs 450–590 s per 4-worker call against 15–38 s for reverse, so one round (about 101 gradient calls) would take about 14–17 h, far above the 45 min limit. **By the stop rule the reruns were not started and forward mode is not adopted in this stage.** Which controller is lab-ready on the desk side is therefore unchanged: S0 yes; S1, S2, S3 no.

## 1. Hypothesis test (S1, A1, whole scenario 23.5 s, eager, noise off, theta = 0, beta = calibration)

| Test | Result |
|---|---|
| Reverse mode, float64, J_task gradient | 1e20 to 1e25 on the 10 non-zero components (e.g. thrust gain −3.5e22, rpm delay −8.3e24); finite difference 1e-4 to 4e-3 |
| Reverse mode, float32 | **NaN** in all 15 components |
| Finite difference (float64, h = 1e-6) | smooth, as before |

The gap is not a small precision loss: float64 gives 1e22 and float32 overflows. The measured growth of the unstable mode is about ×400 per second (S1 / S3, one-block eigenvalue 1.006 to 1.012 per 2 ms). The cost window of A1 is 2 s to 15 s, so the adjoint is carried backward over about 13 s: 400^13 is about 1e34, and a rounding seed of 1e-16 (float64) would become about 1e18; the observed 1e22 to 1e25 is in the same range given that the rate is only roughly known (it rises from 1.0073 to 1.0121 per block between 1 s and 4 s). In float32 (seed 1e-7) the growth overflows. This supports the working hypothesis that the explosion is **rounding driven**; it is a consistency estimate from the growth rate, not a separate proof, and the earlier 5-s window checks of Stage 5 saw only the beginning of it.

Observability of the unstable mode (S1, A1, one-block state Jacobian A of 156 states, blocks 500 / 1250 / 2000 = 1.0 / 2.5 / 4.0 s; unit-norm eigenvectors; ratios to the largest value over all modes; `results/diag_b2.json`):

| Block | \|λ\|max | modes with \|λ\| > 1 | adjoint of the position cost on the right eigenvector r | parameter excitation by the left eigenvector l (θ / β) | position cost gradient on l (literal pairing) | B on r (literal pairing, θ / β) |
|---|---|---|---|---|---|---|
| 500 | 1.00726 | 4 | 1.1e-2 | 5.7e-15 / 1.7e-14 | 3.8e-14 | 9.3e-2 / 2.2e-2 |
| 1250 | 1.01106 | 4 | 4.1e-3 | 6.3e-17 / 1.0e-15 | 3.6e-15 | 1.3e-1 / 4.1e-2 |
| 2000 | 1.01207 | 4 | 6.6e-3 | 9.4e-16 / 2.8e-15 | 3.5e-14 | 1.7e-1 / 5.3e-2 |

The task text pairs the projections the other way round (cost gradient with the left, parameter columns with the right eigenvector); both pairings are listed. The mathematically relevant pairing (reverse mode carries the adjoint aᵀ, whose coefficient on mode i is a·r_i; forward mode carries the tangent, whose coefficient is l_i·B) shows: **parameters do not excite the unstable mode at all** (l·B ≈ 1e-15 of the largest, i.e. zero to rounding), the adjoint of the position cost has a small but non-zero component along it (0.4–1.1 % of the largest). The mode lives in the INDI filter states (eigenvector largest in `fs[36..38]`) and does not reach the position through the parameters; the adjoint's component along it is not exactly zero, so in reverse mode it is amplified, while forward mode never sees it. The hypothesis "no exact component" is refined: the component exists in the adjoint (0.4–1.1 %) but is exactly cancelled in the parameter gradient (l·B = 0).

## 2. Forward mode: implementation and checks

**Switch.** `loop_lib.GRAD_MODE` (`DIFFCOTUNE_CONTROLLER`-style environment variable `DIFFCOTUNE_GRAD_MODE`, default `reverse`), kinds `task_F` / `sysid_F` (forward) and `task_R` / `sysid_R` (reverse) for checks. `forward`: `torch.func.jacfwd` over all parameters (one tangent per gain and per β, batched by `vmap`), **eager**: `torch.compile` cannot compile `torch.func.jvp` in torch 2.5.1 (dynamo skips the frame: graph count 0). Reverse mode stays the default; `lab/round.py --grad-mode` and a per-controller default table (all four `reverse`).

**(b) combined gradient gate** (all 10 gains and all β, J_task and J_sysId = p+v+q, mean over A1, A8, CIRCLE, FIG8, theta = 0, beta = calibration, J_sysId against a noise-off frsim flight of the default gains on the dry-run plant; forward against central finite differences, h = 1e-6, float64; gate: same sign and relative ≤ 1 % or absolute ≤ 0.1 % of the largest component):

| Controller | J_task | J_sysId |
|---|---|---|
| S1 | PASS (components agree to 1e-6 relative or better) | PASS |
| S3 | PASS | PASS |
| S2 | PASS | PASS |
| S0 (information) | PASS | PASS |

(In the S1 table the two near-zero components KRz and Kwz, 1e-10 in size, differ by 2e-3 / 3e-2 relative and pass by the absolute rule.)

**(a) forward against reverse, per scenario** (max |forward − reverse| / max |forward|, noise off):

| | A1 task / sysId | A8 | CIRCLE | FIG8 |
|---|---|---|---|---|
| S0 | 4.0e-3 / 2.3e-3 | 1e-14 | 7e-15 | 2e-15 |
| S1 | 3.3e29 / 5.8e26 | 2.5e-14 | 1e-14 | 1.2e-14 |
| S2 | 0.21 / 3.3e-2 | 1.4e-14 | 2.7e-15 | 1.2e-14 |
| S3 | 5.5e31 / 1.9e32 | 2.1e-14 | 2.9e-15 | 1.6e-15 |

Reverse mode is correct on A8, CIRCLE, FIG8 and wrong on A1 for all four controllers (S0 4e-3, S2 0.2, S1 and S3 explosion): the S0 "accepted A1 row" and the S2 "1 to 14 % mismatch" of Stage 5 are the same defect at smaller growth.

**(c) with noise** (seeds 1000 / 1001, theta = 0, beta = true plant, mean over the two rows; `fwd_checks.py`): reverse against forward, max relative difference over the 14 or 15 components per scenario:

| | A1 | A8 | CIRCLE | FIG8 |
|---|---|---|---|---|
| S0 | 2.1e-12 | 3e-15 | 4e-15 | 2e-15 |
| S1 | 1.8e-11 | 4e-15 | 4e-15 | 2e-15 |
| S2 | 1.7e-13 | 2e-15 | 3e-15 | 2e-15 |
| S3 | 6.1e-12 | 3e-14 | 4e-15 | 4e-15 |

With noise reverse mode is correct on A1 as well (why noise removes the explosion was not tested). The finite-difference gate (h = 1e-5) of check (c) FAILS for S0, S1, S3 and passes for S2; the failure is the finite difference: **h-scan** (`noise_hscan.py`, finite difference against the forward gradient, gate failures out of 10): S1 h 1e-4 / 1e-5 / 1e-6 / 1e-7 / 1e-8 = 9 / 6 / 0 / 0 / 0, S3 = 5 / 1 / 0 / 0 / 0, S0 = 6 / 4 / 1 / 0 / 0 (largest difference 4e-1 / 1.2 / 1.4e-3 / 5e-4 / 3e-5 of the largest component for S1; S3 6e-2 / 3e-3 / 1e-3 / 6e-4 / 1e-4; S0 0.2 / 2.0 / 7e-3 / 9e-4 / 2e-4). The finite difference converges to the forward gradient as h falls, so the noisy gradient is right and the h = 1e-5 finite difference is rough (as the Stage 3d diagnostic found).

**Timing** (4-worker call, one scenario per worker, all four scenarios; reverse after one warm-up call):

| | forward, J_task | forward, J_sysId | forward with noise | reverse, J_task | reverse, J_sysId | reverse with noise |
|---|---|---|---|---|---|---|
| S0 | 453 s | 471 s | 482 s | 23 s | 21 s | 24 s |
| S1 | 560 s | 578 s | 581 s | 30 s | 15 s | 30 s |
| S2 | 480 s | 480 s | 492 s | 33 s | 16 s | 30 s |
| S3 | 585 s | 581 s | 590 s | 32 s | 31 s | 38 s |

Where the time goes (S1, 300 blocks of A1, `bench3.py`): primal eager 1.6 ms per block; reverse eager 7.8 ms; reverse compiled 0.6 ms; forward `jvp`, 1 tangent 33 ms; forward `vmap(jvp)`, 5 tangents 38 ms, 15 tangents 38 ms. Forward mode is dominated by the eager dual-number dispatch, not by the number of tangents (`bench_blk.py`: the compile call of the batched jvp returned in 1 s and gave no speed-up, dynamo skipped it). **Minutes per round:** about 101 gradient calls (51 fit + 50 tune epochs) × 450–590 s = about 13–17 h, against about 22 min with reverse mode (Stage 5). More than 45 min: **not started** (stop rule), forward mode not adopted.

## 3. Reruns of S1, S3, S2: not started

By the stop rule (a round above 45 min is reported with the numbers, not started). Identifiability, dry runs and round-0 tests with forward mode were therefore **not rerun**; the Stage 5 numbers for S1, S2, S3 stand unchanged, **and the RPM-delay verdict and the S3 flat start remain unjudged** (they need a gradient that is exact on A1). The Stage 5 gradient checks with the rerun are likewise open. Per controller: lab-ready on the desk side: S1 no, S2 no, S3 no (the exact gradient exists but is not usable at a lab-compatible speed in this stage).

## 4. Small fixes (done)

- `lab/round.py`: default `--n-rounds` 5 (was 6); new `--grad-mode reverse|forward`, per-controller default table (all `reverse`, written in `GRAD_MODE_DEFAULT`); the setting is part of `SETTINGS`. `lab/preflight.py`: the reminder line is now a check (round.py default n-rounds = 5; grad mode per controller consistent with loop_lib); lab card and `lab/README.md` updated. Old files kept as `*.before_stage5b`.
- Protocol draft: S0 lab-time table, last-round desk time **0.1 min** (measured) instead of about 10 min: S0 desk 81.3 min, loop 110.2 min, with 12 test flights 127.5 min; all four controllers 372.0 min desk, 487.6 min loop, 556.8 min with tests; section S1–S3 updated (gradient mode per controller, thrust-arrival risk as a known risk with the lab order S0, S2, S1, S3, the RPM-delay proposal).
- S0 round-0 dry test unchanged: the S0 worker path in reverse mode is untouched (switch default `reverse`, no `_F` / `_R` suffix); proof: `s0_identity.py` (worker_rollout of the Stage 5 `loop_lib.py` against the new one: J and gradient, bit for bit): see section 5.

## 5. S0 identity proof

`results/s0_identity.json`, job `s0id` (19:33–19:36, after a loader fix of the first attempt): S0, A1 and A8, J_task, J_task with 2 noise seeds, J_sysId: difference of the value and of the gradient between the Stage 5 `loop_lib.worker_rollout` and the new one is **exactly 0.0 in all six cases**. Result: S0 in reverse mode is bit-identical to Stage 5, so the S0 round-0 dry test (θ₁ difference 0.0) is unchanged without the 20-min run.

## 6. Open points for the user

1. Forward mode is exact but about 20× slower than reverse in eager form: decision whether Stage 5c (compiled forward-sensitivity variants, mixed per-scenario gradient) goes ahead (it did in the next stage on your instruction).
2. RPM-delay β (S1, S3): proposal "keep if its error after the round-0 fit is < 25 %, otherwise fix at 4.0502 ms": not judged, needs the exact A1 gradient.
3. S3 flat start: not judged.
4. Times not in the repository: CS2 stop / launch, yaml change, readback, battery change, `--solo` relaunch (unchanged).
