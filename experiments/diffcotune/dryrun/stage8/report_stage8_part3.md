## 2. Why the body rates break J_sysId for us (step 1b; S0, plant P1, true β, default gains, 5 scenarios)

**Error floors per state component** (`results/floors.json`; mean over the sysId window; units p m², v (m/s)², q (unit-quaternion difference)², w (rad/s)²; cells = p / v / q / w and the share of the body-rate part):

| scenario | A: noise-free truth vs noise-free model | B: noisy truth (seed 1) vs noise-free model | C: noisy seed 1 vs noisy seed 2 |
|---|---|---|---|
| A1 | 9.21e-11 / 9.18e-12 / 0 / 0 (w 0 %) | 0.0114 / 0.507 / 0.045 / 10.6 (w 95 %) | 0.0211 / 1.05 / 0.0944 / 23.2 (w 95 %) |
| A8 | 6.76e-10 / 3.3e-09 / 7.24e-12 / 7.74e-09 (w 66 %) | 0.00106 / 0.00617 / 0.000113 / 0.427 (w 98 %) | 0.000351 / 0.00458 / 0.000282 / 0.557 (w 99 %) |
| CIRCLE | 2.85e-11 / 1.1e-11 / 6.68e-13 / 2.33e-09 (w 98 %) | 4.2e-05 / 0.00016 / 1.74e-05 / 0.0586 (w 100 %) | 1.57e-05 / 0.000227 / 3.61e-05 / 0.124 (w 100 %) |
| FIG8 | 2.28e-11 / 1.77e-11 / 8.24e-13 / 2.1e-09 (w 98 %) | 4.07e-05 / 0.000246 / 1.76e-05 / 0.0577 (w 99 %) | 2.95e-05 / 0.000423 / 3.91e-05 / 0.129 (w 100 %) |
| A4 | 1.24e-09 / 8.31e-09 / 2.48e-10 / 4.27e-08 (w 81 %) | 0.00151 / 0.0129 / 0.000551 / 0.262 (w 95 %) | 0.000377 / 0.00422 / 0.000233 / 0.392 (w 99 %) |

A: the noise-free model reproduces the noise-free truth to 1e-10 … 5e-8, so there is no model-mismatch floor at the true β. B and C: against a noisy flight the floor is **95–100 % body rate** in every scenario (A1: 11.2 for one noisy flight against the noise-free model, 24.4 between two noise seeds; the position part is 0.011 and 0.021). The same share holds for the J_sysId parts at the start of the fit (p 0.018, v 0.126, q 0.010, w 2.51 on average over the scenarios, `ident8.json`).

**Phase decorrelation** (`results/floors.json`, `rates`; A1, A8, A4, two noise seeds at the true β):

| scenario | axis | body-rate RMS noise-free truth / noisy 1 / noisy 2 (rad/s) | dominant frequency noisy 1 (Hz) | correlation noisy 1 vs noisy 2 (lag 0 / best, lag) | correlation noisy 1 vs noise-free (lag 0) |
|---|---|---|---|---|---|
| A1 | x | 0.000 / 2.36 / 2.22 | 1.13 | 0.46 / -0.91, -0.57 s | 0.00 |
| A1 | y | 0.000 / 2.16 / 2.28 | 1.13 | -0.60 / 0.87, +0.32 s | 0.00 |
| A1 | z | 0.000 / 0.65 / 0.71 | 0.07 | -0.31 / -0.60, +0.53 s | 0.00 |
| A8 | x | 0.913 / 1.06 / 1.12 | 0.94 | 0.97 / 0.97, +0.00 s | 0.91 |
| A8 | y | 0.000 / 0.47 / 0.30 | 7.67 | -0.45 / -0.45, -0.12 s | 0.00 |
| A8 | z | 0.000 / 0.12 / 0.13 | 7.22 | -0.08 / 0.25, +1.00 s | 0.00 |
| A4 | x | 0.519 / 0.59 / 0.57 | 1.20 | 0.81 / 0.85, +0.02 s | 0.84 |
| A4 | y | 0.413 / 0.59 / 0.58 | 7.60 | 0.66 / 0.67, -0.01 s | 0.77 |
| A4 | z | 0.095 / 0.16 / 0.17 | 7.60 | 0.36 / 0.48, +0.06 s | 0.70 |

On A1 the noise-free truth has body rate 0.000 (a hover hold) while both noisy flights oscillate at 2.2–2.4 rad/s about x and y at about 1.1 Hz; the two noise seeds correlate at 0.46 / −0.60 at lag 0, their best correlation (about 0.9) sits at different random lags (−0.57 s, +0.32 s), and neither correlates with the noise-free truth (0.00). **The A1 rate oscillation is noise-driven (a limit cycle excited by the sensor noise, random phase).** On A8 and A4 the x-rate has a deterministic part (seed-seed 0.97 / 0.81, with the noise-free truth 0.91 / 0.84); the y and z rates are noise-dominated (about 7.2–7.7 Hz).

**Units.** The unweighted sum adds m², (m/s)², a dimensionless quaternion term and (rad/s)². The reference body rates of the scenarios are small (|w*| ≤ 0.26 rad/s by flatness), but the measured rates are 1–2 rad/s: numerically the rate term is about 16 times the sum of the other three (2.51 against 0.154) at the start of the fit.

**Paper-exact identifiability** (`results/ident8.json`; fit only, noisy flights of θ₀ with seeds 0 and 1, lr 0.01, forward gradient, 5 scenarios; truth: thrust 0.8, lag 1.5, downwash 1.3, plume 0.7):

| J_sysId form | β error: thrust gain / motor lag / downwash / plume (%) | epochs (stop) | J start → returned |
|---|---|---|---|
| p+v+q (ours), 50 epochs, best iterate | -6.3 / -56.4 / -35.3 / +64.8 | 51 (max epochs) | 0.1539 → 0.1421 |
| p+v+q+w paper-exact (true body rate of the flight), 50 epochs, best iterate | +0.9 / -30.7 / +20.9 / +14.4 | 51 (max epochs) | 2.662 → 2.448 |
| p+v+q+w paper-exact (true rate), paper stop rule | +7.0 / -31.6 / +14.2 / +17.7 | 45 (converged |dJ| < 1e-3) | 2.662 → 2.471 |
| p+v+q+w, measured gyro (true rate + gyro noise), paper stop rule | +7.6 / -31.7 / +14.8 / +17.7 | 46 (converged |dJ| < 1e-3) | 2.796 → 2.604 |

(Stage 4e, 4 scenarios, p+v+q best iterate: −1.7 / −34.5 / +5.7 / +22.3 %; with the measured gyro in the 4-scenario set it was −11.4 / −37.9 / −16.1 / +39.4 %.)

**Conclusion in plain words.** The body-rate part of J_sysId is almost entirely a noise-driven oscillation that a noise-free model cannot reproduce, so it is 95–100 % of the cost and sets its floor. Even so, in this sim-to-sim identifiability test the paper form (all four parts, unweighted) is **not worse** than our p+v+q: with the true rate of the flight it gives +0.9 / −30.7 / +20.9 / +14.4 % against −6.3 / −56.4 / −35.3 / +64.8 % for p+v+q on the same flights, and the measured-gyro form with the paper stop rule gives +7.6 / −31.7 / +14.8 / +17.7 %. So the body rates do not break the **fit** in this test (the deterministic x-rate parts on A8 and A4 carry information); what fails in the paper variant is the **tuning**: the real cost gets worse with every round (Section 5). Why the paper form works for the paper's systems: its simulations are noise-free sim-to-sim, so the rates are predictable. **Smallest change that keeps the paper form and is robust to the noise: not established**; candidates (proposal, not adopted): a noise model in the simulator with several noise seeds averaged in J_sysId (as the frozen method does for the task cost), or fitting only the deterministic rate part (x-rate on A8 and A4).

**Share of the body rates in the task cost per round and fit epochs per round:** fit and tune epochs per round are in the per-controller tables of Section 5; the share of p, v, q, w in the paper task cost per round (`taskshare.py`) is listed in Section 5 where it was computed (not computed if the line is missing).
