# Stage 5c, RPM delay fixed (user, 14 Nov 2026): S1 and S3 reruns (written 15 Nov 2026)

User decision: the RPM delay error after the round-0 fit was above 25 % (S1 −44 %, S3 −63 %), so the RPM delay of S1 and S3 is **fixed at the calibrated 4.0502 ms** (the plant truth is 8.0 ms, so the model keeps a −49.4 % offset in this β by construction) and the other four β are fitted. Everything else as the Stage 5c runs with the delay fitted (`../stage5c/`): same seeds, plants, frozen method, `grad_mode` mixed. Implementation: `loop_lib.FIX_RD` / `PSI_FIT_IDX` (env `DIFFCOTUNE_FIX_RD`, default off), `lab/round.py` (`--fix-rd` / `--fit-rd`, default table S1 and S3 fixed, S0 and S2 unchanged), settings record `fix_rd`. Jobs: `dry_<C>`, `rtest_<C>`, `gate_<C>` (chain6, all rc 0). Tables: `results/sections_<C>.md`.

## Criteria and round test, against the Stage 5c runs with the delay fitted

| | S1 fixed | S1 fitted | S3 fixed | S3 fitted |
|---|---|---|---|---|
| a. rounds 2–4 ≤ round 1 + 1 cm | yes | yes | yes | yes |
| b. no new failed scenario | yes | yes | yes | yes |
| c. theta_best beats theta_0 (tuning / test mean, cm) | **yes** (2.30 / 1.81 against 9.04 / 5.73) | yes (2.25 / 1.79) | **no** (9.02 / 5.78 against 9.02 / 5.78: theta_best = theta_0) | **no** (8.45 / 5.82 against 9.02 / 5.78) |
| real 3-D RMS rounds 0…4 (cm) | 10.32, 6.80, 5.05, 3.82, 2.93 | 10.32, 6.93, 5.26, 4.11, 3.04 | 10.75, 10.51, 10.95, 10.91, 10.79 | 10.75, 10.56, 10.28, 10.26, 9.26 |
| desk minutes per round (round 0 … 3) | 22.7, 20.4, 20.4, 20.4 | 22.6, 20.9, 20.6, 20.7 | 28.9, 25.6, 25.6, 25.4 | 28.7, 25.2, 25.2, 25.3 |
| round test (θ₁, β after the fit against the loop) | 0.0 / 0.0, 22.5 min | 0.0 / 0.0, 22.0 min | 0.0 / 0.0, 28.8 min | 0.0 / 0.0, 28.4 min |
| gradient gate (adopted mode) | PASS | PASS | PASS | PASS |

## β error after each fit (% against the plant truth)

| | round 0 | round 1 | round 2 | round 3 |
|---|---|---|---|---|
| S1 fixed (thrust / lag / downwash / plume / RPM delay) | −1.4 / −29.2 / −42.5 / +9.4 / −49.4 | −0.1 / −4.1 / −60.0 / +15.3 / −49.4 | +3.7 / +26.8 / **−73.9** / +5.8 / −49.4 | +1.8 / +19.8 / −62.7 / +8.1 / −49.4 |
| S1 fitted | −1.3 / −29.6 / −42.6 / +9.2 / −43.8 | −0.1 / −1.3 / −61.2 / +10.4 / −21.6 | +4.7 / +63.0 / −74.8 / −12.9 / +28.9 | +4.5 / +59.2 / −74.9 / −15.0 / +82.1 |
| S3 fixed | +41.0 / −46.3 / −29.7 / +70.0 / −49.4 | +9.0 / −52.8 / −47.0 / +101.4 / −49.4 | +0.6 / −63.4 / −47.7 / +148.1 / −49.4 | +2.3 / −74.6 / −56.2 / +157.7 / −49.4 |
| S3 fitted | +41.3 / −46.2 / −29.9 / +65.6 / −63.1 | +10.1 / −50.7 / −42.9 / +101.8 / −74.5 | −0.1 / −69.1 / −46.7 / +151.6 / −83.1 | +6.3 / −78.8 / −60.2 / +174.0 / −89.1 |

## Findings

* **S1 downwash drift is not caused by the RPM delay.** With the delay fixed the downwash error is still −42 % after round 0 and peaks at −74 % in round 2 (fitted: −75 %). What fixing the delay removes is the swing of the lag error (fitted +63 %, fixed +27 %) and of the delay itself (fitted +82 %). Test error and criteria are essentially unchanged (θ_best 2.30 / 1.81 against 2.25 / 1.79 cm).
* **S3 flat start is not caused by the RPM delay either, and is sharper with it fixed.** In round 0 the tuning step finds no descent from θ₀ in the fixed run (best iterate = epoch 0, J_task 0.01209 unchanged), so θ₁ = θ₀ (all ten gain factors 0); keep-best then selects round 1, whose gains are the defaults, and criterion c fails by identity. In the fitted run the round-0 step found a small descent (epoch 13, 0.01173 → 0.01047). In both runs the tuning cost at θ_l grows from round 0 to round 2 (fixed: 0.0121, 0.0346, 0.0429; fitted: 0.0117, 0.0272, 0.0424) while the β errors for lag and plume grow (plume +158 %, lag −75 % after round 3): the model with the fitted β describes the flights less well, not better, as rounds go on. Cause not established in this stage.
* Round times are unchanged by the switch (the fixed run does the same work: five-entry β, four fitted).

## Lab-ready on the desk side (RPM delay fixed)

| | lab-ready on the desk side | reason |
|---|---|---|
| S1 | **yes** | criteria a to c yes, round test identical, gate PASS, 22.5 min per round; open point: downwash β drifts to −74 % (does not affect the criteria in the dry run) |
| S3 | **no** | criterion c fails (θ_best = θ_0; round-0 tuning step finds no descent) |

Open points: cause of the S3 flat start and of the β drift of S3 and S1 (downwash, lag, plume) not established; whether S3 should be run with a different β set or start is a user decision (not decided here).
