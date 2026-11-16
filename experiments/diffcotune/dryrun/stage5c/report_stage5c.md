# Stage 5c report: exact gradient at near-normal speed for S1, S2, S3 (DiffCoTune; desk only; 14 Nov 2026)

Desk only; nothing flown or flashed; no commit. Jobs through `dryrun/stage3c/job_runner.py`, one pool at a time. Sources in `results/`, logs in `logs/`. Updated after each step.

## Step 1: reverse mode with noise across gain sets (tuning gradient)

Noisy J_task (mean over 2 noise seeds as in the tuning step), compiled reverse against forward (`jacfwd`, eager), at theta_l and beta_l (`psi_after_fit`) of the Stage 5 dry run of the controller, l = 0, 1, 3, with the loop pair of the round and 7000 / 7001 on A1 (6 points), and one point (l = 1, pair 6002 / 6003) on A8, CIRCLE, FIG8. Number: max |reverse − forward| / max |forward| over all 14 or 15 components (the gains alone give the same or a smaller number). Gate ≤ 1e-6.

| Controller | Scenario | l | noise pair | rel. difference (gains / all) | passed |
|---|---|---|---|---|---|
| S1 | A1 | 0 | 6000 / 6001 | 4.0e-12 / 4.0e-12 | yes |
| S1 | A1 | 0 | 7000 / 7001 | 1.7e-12 / 1.6e-12 | yes |
| S1 | A1 | 1 | 6002 / 6003 | 3.3e-11 / 2.5e-11 | yes |
| S1 | A1 | 1 | 7000 / 7001 | 6.9e-12 / 5.7e-12 | yes |
| S1 | A1 | 3 | 6006 / 6007 | 7.2e-15 / 7.2e-15 | yes |
| S1 | A1 | 3 | 7000 / 7001 | 2.6e-14 / 2.5e-14 | yes |
| S1 | A8 | 1 | 6002 / 6003 | 2.2e-15 / 2.2e-15 | yes |
| S1 | CIRCLE | 1 | 6002 / 6003 | 4.7e-15 / 3.1e-15 | yes |
| S1 | FIG8 | 1 | 6002 / 6003 | 4.9e-15 / 1.6e-14 | yes |
| S3 | A1 | 0 | 6000 / 6001 | 1.2e-08 / 1.2e-08 | yes |
| S3 | A1 | 0 | 7000 / 7001 | 1.8e-09 / 1.8e-09 | yes |
| S3 | A1 | 1 | 6002 / 6003 | 4.1e-12 / 4.1e-12 | yes |
| S3 | A1 | 1 | 7000 / 7001 | 7.1e-10 / 7.1e-10 | yes |
| S3 | A1 | 3 | 6006 / 6007 | 1.7e-10 / 2.3e-10 | yes |
| S3 | A1 | 3 | 7000 / 7001 | 1.7e-12 / 2.8e-12 | yes |
| S3 | A8 | 1 | 6002 / 6003 | 7.9e-13 / 8.3e-13 | yes |
| S3 | CIRCLE | 1 | 6002 / 6003 | 3.2e-15 / 3.8e-15 | yes |
| S3 | FIG8 | 1 | 6002 / 6003 | 5.6e-15 / 1.0e-14 | yes |
| S2 | A1 | 0 | 6000 / 6001 | 9.2e-11 / 6.0e-11 | yes |
| S2 | A1 | 0 | 7000 / 7001 | 2.5e-11 / 2.5e-11 | yes |
| S2 | A1 | 1 | 6002 / 6003 | 1.6e-13 / 5.2e-14 | yes |
| S2 | A1 | 1 | 7000 / 7001 | 1.5e-13 / 5.0e-14 | yes |
| S2 | A1 | 3 | 6006 / 6007 | 5.7e-11 / 2.0e-11 | yes |
| S2 | A1 | 3 | 7000 / 7001 | 3.2e-11 / 3.2e-11 | yes |
| S2 | A8 | 1 | 6002 / 6003 | 1.4e-14 / 7.3e-15 | yes |
| S2 | CIRCLE | 1 | 6002 / 6003 | 4.1e-15 / 1.2e-14 | yes |
| S2 | FIG8 | 1 | 6002 / 6003 | 3.3e-15 / 1.1e-14 | yes |

Worst per controller: S1 2.5e-11, S3 1.2e-08, S2 6.0e-11. Forward mode needed 316–571 s per task (one worker), reverse 11–40 s.

**Decision for the tuning step:** the gate passes everywhere (all scenarios, all three gain sets, both noise pairs), so the tuning step keeps compiled reverse mode for S1, S2, S3 (same speed as in Stage 5). Not tested (not stated): other gain sets than theta_0, theta_1, theta_3 of the Stage 5 runs, and the noisy case at points where the unstable mode could grow more (e.g. gain sets near the ±25 % limit): the loop checks (gradient gate with the adopted mode, step 5) cover the rest.

## Steps 2 and 3: exact forward gradient at compiled speed, and the mixed mode

**Step 3 variant tried first and kept: (a') forward sensitivity inside the compiled model, built from the jvp traced to a plain graph** (`experiments/diffcotune/fsens.py`).

* Why not the other variants first: `torch.compile` cannot compile `torch.func.jvp` in torch 2.5.1 (dynamo skips the frame, Stage 5b), so the jvp of ONE 2-ms step (S1: INDI block; S2 / S3: block with the NS2 evaluation of every 5th block as a second graph; vmapped over the tangents) is traced ONCE with `torch.fx.experimental.proxy_tensor.make_fx`. The functorch transforms are executed at trace time, so the result is a plain aten graph (about 5600 nodes for S1, 5 tangents) that `torch.compile` compiles. The state tangents are carried next to the state through the rollout; the cost is evaluated outside on the recorded rows (x, v, q, w) by autograd of the cost function only (cheap, exact) and contracted with the recorded tangent rows. No approximation is made: same gradient as torch.func.jvp.
* **One-block test (`fsens_try.py`, S1, 5 tangents):** compiled against eager `vmap(jvp)`: max difference 1.1e-16; 38.9 ms per block eager against **0.58 ms** compiled (67 times faster; reverse compiled is 0.6 ms per block). **3000 blocks of A1:** compiled 1.8 s (0.59 ms per block), eager 115.1 s; tangent of the position at the end: max difference 1.7e-16 (largest tangent 0.21), all 156 state tangents 1.4e-14. A full A1 flight (11750 blocks) therefore takes about 7 s for the 5 β tangents (estimate from 0.59 ms per block; the measured times in the pool are below).
* **Correctness on full flights (`fsens_check.py`, job `fchk_<C>`, theta = 0, beta = calibration, all 14 / 15 parameters, against the eager forward gradient of Stage 5b, `stage5b/results/fwd_checks_<C>.json`):** max |compiled − eager| / max |eager| per scenario A1 / A8 / CIRCLE / FIG8 (gate ≤ 1e-8):

| Controller | J_sysId | J_task | seconds per 4-worker call, J_sysId (2nd call) | J_task |
|---|---|---|---|---|
| S1 | 2e-15 / 3e-14 / 6e-16 / 3e-15 | 1e-15 / 3e-14 / 6e-16 / 1e-15 | 10.2 s | 10.3 s |
| S3 | 3e-14 / 7e-15 / 1e-15 / 2e-15 | 2e-15 / 1e-14 / 5e-16 / 8e-16 | 10.8 s | 11.3 s |
| S2 | 3e-15 / 5e-15 / 8e-16 / 4e-16 | 8e-16 / 1e-14 / 2e-15 / 1e-15 | 10.2 s | 9.9 s |

All pass (worst 3e-14). Reverse mode in the same job reproduces the Stage 5b result: A1 5.8e26 (S1), 1.9e32 (S3), 3.3e-2 (S2) for J_sysId, exact on A8, CIRCLE, FIG8. The compiled forward call (all 15 tangents, all four scenarios, one scenario per worker) takes **9.8–11.4 s**, against 450–590 s eager (about 45 times faster) and 11.6–44.5 s for reverse in the same pool (the larger reverse numbers include the compile of the reverse graph in a worker that had not run it yet; the first forward call costs 35–79 s of graph compile once per worker and graph).
* **Switch `grad_mode = 'mixed'`** (`dryrun/loop_lib.py`, `DIFFCOTUNE_GRAD_MODE=mixed`; `lab/round.py --grad-mode`, default table now S0 reverse, S1 / S2 / S3 mixed): in the noise-free β fit, A1 (kind `sysid`) uses `fsens.gradient` with the β tangents only (5 for S1 / S3, 4 for S2); A8, CIRCLE, FIG8 and every tuning gradient (noise, 2 seeds) use compiled reverse. The summed cost's gradient is the sum, so it stays exact. **Split of the work:** no split of the A1 tangents over the workers is needed: the A1 gradient with 5 tangents takes about as long as the reverse gradient of another scenario, so the 4 workers keep one scenario each (the 2 / 1 / 1 / 1 split of the plan was not built). Explicit kinds `task_C` / `sysid_C` (compiled forward, all parameters, with or without noise), `_F` (eager forward), `_R` (reverse) remain for checks.
* **Check that the mixed gradient equals the pure forward gradient** (job `fmix_<C>`, the gradient of the fit as the loop calls it, theta = 0, beta = calibration, β components, per scenario A1 / A8 / CIRCLE / FIG8, against the eager forward gradient of Stage 5b; gate ≤ 1e-8; the 10 gain components of A1 are zero by construction in this mode, since the fit updates β only):

| Controller | max relative difference per scenario | seconds per fit-gradient call (4 workers; first / second call) |
|---|---|---|
| S1 | 2e-15 / 6e-14 / 4e-15 / 5e-15 | 43 / 36 s |
| S3 | 3e-14 / 8e-15 / 1e-14 / 9e-15 | 69 / 56 s |
| S2 | 3e-15 / 8e-15 / 1e-15 / 5e-15 | 68 / 47 s |

All pass (worst 6e-14). The seconds in the last column are the wall time of the whole pool call: they are probably dominated by the compile of the reverse graphs in workers that ran another scenario before (`pool.map` may assign the four scenarios to the four workers in changing order; not verified); the per-epoch time of the loop is measured in step 5.
* Other step-3 variants **not tried** because (a') met the goal and passed the check: (b) double vjp on the compiled reverse block, (c) eager forward mode on the GPU, (d) others. Round time: see step 5.

## Step 5: reruns with grad_mode mixed (S1, S3, S2; written 15 Nov 2026, offline from the logs)

Jobs: ident_<C>, dry_<C> (5 rounds, same seeds and plants as Stage 5), rtest_<C> (round 0 with lab/round.py), gate_<C>. Per-controller tables: `results/sections_<C>.md` (made with sections5.py). Step 4 (truncated reverse) was **not run**: step 3 passed.

### Round test and gate (all three controllers)

| | theta_1 and beta after fit: max abs difference round.py vs loop (Stage 5c run) | round 0 minutes (fit / tune / total) | yaml roundtrip | gradient gate (adopted mode) |
|---|---|---|---|---|
| S1 | 0.0 / 0.0 | 10.9 / 11.0 / 22.0 | 2e-16 | PASS |
| S3 | 0.0 / 0.0 | 14.6 / 13.8 / 28.4 | 8e-17 | PASS |
| S2 | 0.0 / 0.0 | 13.2 / 12.6 / 25.9 | 3e-16 | **FAIL**, one check: noisy J_task, reverse against central FD h=1e-7, KRxy rel 4.0e-2 (all other components 1e-9 ... 9e-3) |

Round time target (<= 30 min, hard 45): met by all three (S3 28.4 min is the longest). Note: the S2 gate failure is on the noisy task gradient in reverse mode, which step 1 showed equal to forward mode to <= 2e-11; FD with noise needs h <= 1e-6 (Stage 5b) and h=1e-7 was used, so the FD value is the likelier outlier, **but this is not shown**: a one-component h-scan on S2 KRxy is open (not run).

### Dry runs (criteria a to c; 'minutes' = desk minutes per round l = 0 ... 4)

| | a (rounds 2-4 <= round 1 + 1 cm) | b | c (theta_best beats theta_0, tuning / test means) | real 3-D RMS rounds 0..4 (cm) | minutes per round |
|---|---|---|---|---|---|
| S1 | yes | yes | yes (2.25 / 1.79 against 9.04 / 5.73) | 10.32, 6.93, 5.26, 4.11, 3.04 | 22.6, 20.9, 20.6, 20.7, 0.1 |
| S3 | yes | yes | **no** (8.45 / 5.82 against 9.02 / 5.78) | 10.75, 10.56, 10.28, 10.26, 9.26 | 28.7, 25.2, 25.2, 25.3, 0.1 |
| S2 | yes | yes | yes (5.25 / 5.26 against 12.36 / 6.77) | 15.71, 8.92, 7.28, 7.00, 6.81 | 26.1, 23.5, 23.6, 23.7, 0.1 |

S3 flat start: the tuning-flight RMS falls only from 10.75 to 9.26 cm over five rounds (S1: 10.32 to 3.04); the test mean does not improve (5.82 against 5.78). The mixed gradient did not remove it: criterion c fails for S3 as in Stage 5.

### beta after each fit (error against the plant truth, %)

| | round 0 | round 3 | remark |
|---|---|---|---|
| S1 (thrust / lag / downwash / plume / RPM delay) | -1.3 / -29.6 / -42.6 / +9.2 / -43.8 | +4.5 / +59.2 / -74.9 / -15.0 / +82.1 | downwash drifts to -75 %, lag and RPM delay swing to +59 / +82 % |
| S3 | +41.3 / -46.2 / -29.9 / +65.6 / -63.1 | +6.3 / -78.8 / -60.2 / +174.0 / -89.1 | lag, plume and RPM delay drift away from the truth |
| S2 (thrust / lag / downwash / plume) | -12.9 / -37.6 / +15.0 / +34.7 | -1.0 / +0.8 / +33.0 / -2.7 | converges except downwash (+33 %) |

RPM delay: error after the round-0 fit S1 -44 %, S3 -63 % (identifiability job: S1 -44.4 %, S3 -63.3 %), both above 25 %: **user decision 14 Nov: fixed at the calibrated 4.0502 ms for S1 and S3** (see `dryrun/stage5c_rdfix/`, rerun of dry run, round test and gradient gate; this section is the fitted-delay reference).

The two identifiability jobs ident_S3 and ident2_S1 exited rc 1 only at the last print line (missing RD_START constant in the S3 model / import of config); the fit and results/identifiability_<C>.json were complete.

### Lab-ready on the desk side (fitted-delay runs; status before the fixed-delay rerun)

| | lab-ready on the desk side | reason |
|---|---|---|
| S1 | yes (provisional) | criteria a to c yes, round test identical, gate PASS, 22 min per round; the beta drift (downwash -75 %) is a point to review in the fixed-delay rerun |
| S3 | **no** | criterion c fails (flat start), as in Stage 5 |
| S2 | **no (open)** | criteria yes, round test identical, but the gate check failed on one noisy FD comparison (KRxy); h-scan not yet run |

S0 unchanged (reverse, lab-ready as before). Open: h-scan S2 KRxy; fixed-delay reruns of S1, S3 (running, chain6).

### S2 gradient gate: KRxy step-size scan (15 Nov 2026, job hscan_S2, `s2_krxy_hscan.py`, `results/s2_krxy_hscan.json`)

Noisy J_task of S2 (seeds 6000 / 6001, theta = 0, beta = truth), reverse gradient against central finite differences, KRxy component (reverse −5.376e-3):

| h | 1e-4 | 1e-5 | 1e-6 | 1e-7 (gate) | 1e-8 |
|---|---|---|---|---|---|
| relative difference of the finite difference on KRxy | 3.9e-2 | 1.4e-1 | 8.0e-2 | 3.8e-2 | **3.2e-8** |

At h = 1e-8 the finite difference equals the reverse gradient on KRxy to 3e-8 and the worst of all ten gains is 1.6e-4 (Kwxy); for h ≥ 1e-7 the finite difference of S2 with noise is not converged (it varies between 4 % and 14 % without a trend). So the gate failure of S2 was the finite-difference step, not the gradient: the reverse gradient is confirmed. The gate script (h = 1e-7) was **not changed and its recorded result stays FAIL**; with h = 1e-8 the check passes. (S1 and S3 passed at h = 1e-7.)

Updated line: **S2 lab-ready on the desk side: yes** (criteria a to c yes, round test identical, 25.9 min per round, gradient confirmed at h = 1e-8; recorded gate result at h = 1e-7: FAIL, explained above).
