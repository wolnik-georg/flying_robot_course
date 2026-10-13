# 76 — Equal-parameter replay: ours, Omar C and Tal & Karaman on the same inputs (2026-10-13)

**Question (meeting 2026-10-05 / 2026-10-12):** `docs/64` fed the same logged inputs to ours, Omar C and Omar Rust with *native* gains and filters and found different outputs. If every settable gain, constant and filter of one controller is set to the other's value and the outputs still differ, the difference is the law or its implementation. This page walks that ladder one parameter group at a time and adds the Tal & Karaman controllers.

**Correction of `docs/64` / `docs/74` §4 (found 2026-10-13).** The earlier replay harness never set `state->acc` (the estimator's world-frame acceleration, Gs, gravity removed: `kalman_core.c`, `R·acc_body − [0,0,1]`), which Omar C reads as its measured acceleration. Omar's residual `a_rpm − a_imu` therefore ran without a measurement. The ours-vs-Omar numbers of `docs/64` (thrust correlation 0.97, −19 %) are superseded by the tables below (ours native vs Omar C: thrust RMS 0.090 N, correlation 0.62). Also: ours' torque output on the same input differs from the 10-07 files of `docs/64` (RMS 0.5–0.9 mN m), because the host library changed since; the present worker reproduces the old worker run today exactly (0.0).

## Method
- Code: `experiments/analysis/indi_replay_equal.py` (driver), `indi_replay_equal_worker.py` (one controller, one parameter set, one input file), `indi_replay_equal_plot.py`. One process per run, host-compiled firmware through the Python bindings, 1 kHz ticks, steady part after 0.8 s.
- Inputs: the 2026-10-02 A8 flight (`out/indi_replay/A8`, ours on-policy, 26,060 ticks) and today's A8 flight with our INDI (`2026-10-13` 17-59-48, cf5 uSD, 26,060 ticks). Open loop, so off-policy for every other controller: read output differences as law differences, never as flight performance.
- Reference: **Omar C native**, and **Omar C with the attitude integral `K_I` = 0**. With `K_I` 0.03 the integral winds up in the open-loop replay (RMS torque 12 mN m on x, std 5.5 mN m) and swamps the torque comparison, so torques are compared against the `K_I` = 0 run. Thrust is identical in both references.
- Omar C ≡ Omar Rust (0.0000 on thrust and all torques), Tal & Karaman Rust ≡ C (0.0000), re-confirmed on these inputs.
- Ladder for ours (cumulative): flown configuration → mass 0.0427 → `k_t` 4.29e-10 (all motors) → position gains `K_P` 7, `K_D` 4, `k_i,z` 0 → residual sign −1 (Omar subtracts) → attitude gains `k_R` 421 / `k_ω` 69 (z: 273 / 68; Omar's torque gains divided by his inertia) → inertia `j_scale` 0.693 (matches x/y; z would need 0.905, one scalar only) → filters 30 Hz (`fc_bw`, `res_fc`, rate-then-filter order, notch off) → clamps and torque limits off.
- Tal & Karaman: native = paper gains and 30 Hz with the flown mass and `k_t`; "Omar's values" = mass, `k_t`, `K_P` 7, `K_D` 4, acceleration feedback 0, rotor-inertia term 0, attitude gains as Omar's torque gains divided by the controller's own inertia (its inertia is a constant, 23.95e-6, not settable).

## Result, A8 2026-10-02 (reference: Omar C with K_I = 0)
| run | thrust RMS diff [N] | thrust mean diff [N] | thrust corr | torque RMS diff x / y / z [mN m] | torque corr x / y / z |
|---|---|---|---|---|---|
| ours native | 0.090 | −0.078 | 0.62 | 1.77 / 2.71 / 1.18 | 0.37 / 0.42 / 0.69 |
| + mass | 0.068 | −0.047 | 0.62 | 1.70 / 2.59 / 1.18 | 0.39 / 0.43 / 0.70 |
| + k_t | 0.081 | −0.066 | 0.62 | 1.75 / 2.73 / 1.18 | 0.37 / 0.43 / 0.70 |
| + position gains | 0.039 | −0.006 | 0.23 | 2.82 / 2.16 / 1.16 | 0.22 / 0.49 / 0.71 |
| + residual sign −1 | **0.007** | 0.000 | **0.98** | 2.56 / 2.18 / 1.16 | 0.70 / 0.30 / 0.71 |
| + attitude gains | 0.007 | 0.000 | 0.98 | 0.67 / 0.80 / 1.30 | 0.77 / 0.43 / 0.40 |
| + inertia | 0.007 | 0.000 | 0.98 | 0.60 / 0.77 / 1.18 | 0.78 / 0.38 / 0.42 |
| + filters 30 Hz | **0.005** | 0.000 | **0.99** | **0.35 / 0.39 / 1.01** | **0.94 / 0.89 / 0.78** |
| + limits off | 0.005 | 0.000 | 0.99 | 0.35 / 0.39 / 1.01 | 0.94 / 0.89 / 0.78 |
| Tal & Karaman native (Rust = C) | 0.029 | −0.026 | 0.99 | 0.81 / 0.84 / 1.62 | 0.64 / 0.33 / −0.44 |
| Tal & Karaman with Omar's values (Rust = C) | **0.0013** | +0.0006 | **1.00** | 0.81 / 0.86 / 1.35 | 0.66 / 0.32 / −0.26 |
| Omar C with ours' gains (reference: Omar C) | 0.076 | −0.057 | 0.87 | 4.55 / 3.12 / 0.97 | 0.83 / 0.75 / 0.98 |

Figure: `docs/meetings/assets/2026-10-14/indi_equal_parameter_replay_ladder.png`. Raw numbers: `…/indi_equal_replay_A8_summary.json`.

## Same ladder on today's flight (ours on-policy, A8 17-59-48)
Thrust RMS diff to Omar C: native 0.074 N (corr 0.63) → after residual sign 0.005 N (corr 0.99) → after filters 0.0036 N (corr 1.00); Tal & Karaman with Omar's values 0.0014 N (corr 1.00). Torque x / y after all steps: 0.61 / 0.49 mN m (corr 0.92 / 0.90), z 0.74 mN m (corr 0.77). Same picture (`…/indi_equal_replay_2026-10-13_ours_3_summary.json`).

## What it says
1. **Thrust (the position / residual law): no implementation difference.** Once ours has Omar's mass, position gains and residual sign, its thrust equals his to 0.005 N (about 1 % of the 0.41 N mean, correlation 0.99). Tal & Karaman, an independent implementation, reproduces Omar's thrust to 0.0013 N with his values. The earlier "−19 % and different shape" was parameters: the residual sign (`+1` against `−1`), the position gains (64 / 5 against 7 / 4) and the mass.
2. **Torque x and y: nearly the same law.** After the attitude gains and the 30 Hz filters the difference is 0.35–0.6 mN m with correlation 0.9; what remains are terms that cannot be switched off (Omar's gyroscopic and rate feed-forward terms, his unfiltered reference, loop rate).
3. **Yaw: still different** (≈ 1 mN m RMS, correlation 0.78): inertia can only be matched on x / y with one scalar, and the heading handling differs; not isolated.
4. **Tal & Karaman's attitude law is a different law:** its torques correlate poorly with Omar's even with his values (0.66 / 0.32 / −0.26), because of its own attitude-error definition, feed-forward and fixed inertia.
5. **Consequence for the INDI choice:** ours and Omar C are the same thrust law with different parameter sets (and different attitude-loop stiffness), not two different controllers. The flight difference (A8 dips −9.4 vs −9.1 cm, ours with an overshoot, Omar's slower recovery) comes from the parameters: ours flies with the opposite residual sign and 7–9× stiffer position gains. Open: ours with `res_sign −1` diverged in hover on 2026-09-09 (cause never isolated), while Omar's textbook sign flies. A flight of ours with Omar's parameter set (position gains 7 / 4, `res_sign −1`, attitude gains 421 / 69, 30 Hz filters) would show whether the two then fly the same.

## Final comparison with all parameters equal (reference Omar C, K_I = 0; thrust RMS [N] / corr, torque RMS x / y / z [mN m] / corr)
| | A8 2026-10-02 | A8 today (ours on-policy) |
|---|---|---|
| ours native | 0.090 / 0.62 · 1.8 / 2.7 / 1.2 · 0.37 / 0.42 / 0.69 | 0.074 / 0.63 · 2.4 / 3.6 / 1.0 · 0.89 / 0.63 / 0.70 |
| **ours adjusted** (mass, k_t, position gains, residual sign, attitude gains, inertia, 30 Hz filters) | **0.005 / 0.99** · 0.35 / 0.39 / 1.0 · 0.94 / 0.89 / 0.78 | **0.004 / 1.00** · 0.61 / 0.49 / 0.74 · 0.92 / 0.90 / 0.77 |
| Tal & Karaman native | 0.029 / 0.99 · 0.8 / 0.8 / 1.6 · 0.64 / 0.33 / −0.44 | 0.025 / 0.99 · 1.4 / 1.1 / 1.3 · 0.49 / 0.30 / −0.53 |
| **Tal & Karaman with Omar's values** | **0.0013 / 1.00** · 0.8 / 0.9 / 1.35 · 0.66 / 0.32 / −0.26 | **0.0014 / 1.00** · 1.4 / 1.1 / 1.0 · 0.51 / 0.29 / −0.22 |
| ours adjusted vs Tal & Karaman with Omar's values | 0.005 / 0.99 · 0.6 / 0.7 / 0.4 · 0.81 / 0.62 / 0.03 | 0.004 / 0.99 · 1.0 / 0.9 / 0.3 · 0.78 / 0.65 / 0.07 |

Thrust: all three agree to 0.001–0.005 N (≤ 1 %). Torque: ours adjusted is the closest to Omar C (correlation 0.9 on x / y); Tal & Karaman's attitude law stays further away (correlation 0.3–0.65) and its yaw is uncorrelated (≈ 0) with both. The gyroscopic term is 0.002 mN m RMS here (negligible), the rate feed-forward is inactive in the replay (no jerk / snap in the inputs); the remaining torque difference is mostly high-frequency (smoothing both over 50 ms leaves 0.24–0.29 mN m on x / y and 0.05 on yaw), consistent with Omar's unfiltered attitude reference against ours filtered at 30 Hz (not proven).

## Closed-loop check before the lab (single drone, 2026-10-13)
`experiments/analysis/indi_variants_sil_check.py` (airborne start, measurement noise on, real vehicle as plant incl. 44 ms motor lag; hover, A8 bottom path, circle; `out/indi_variants_sil_check_airborne/summary.md`) and `indi_ours_sil_sweep.py` (one parameter group at a time).
- **Ours with Omar's values and ours' own 206 Hz filters is stable and flies like Omar C:** xy RMS 0.4–0.5 cm, z offset −1.5 cm (0.0 cm with the z integral 1.5), tilt ≤ 4°; Omar C: 0.4 cm, −0.3 cm (0.0 with the integral).
- **Lowering ours' filter cutoff destabilises it** (30, 60 and 100 Hz all diverge, faster than the frozen 206 Hz); ours with all of Omar's values including the 30 Hz filters diverges in the three scenarios. Candidate reason (not proven): ours low-passes its attitude reference as well, which adds lag in the feedback path; Omar's reference is unfiltered.
- Tal & Karaman, native and with Omar's values: stable (0.2–0.5 cm, tilt ≤ 5°).
- Ours frozen falls in this simulation after 1–5 s although it flies 27 s on the drone: the simulation is harsher than the hardware for ours; only the ordering is used.
- Consequence for the lab: first flight of ours with Omar's values keeps ours' own filters (`ours_omar_values_keepfilters.patch`); the 30 Hz variant last, if at all (`docs/lab_sessions/next_session_plan.md`).

## Limits
Open loop and off-policy; one scenario (A8) and two flights; the `K_I` = 0 reference differs from Omar's shipped configuration in the torque only; inertia z and Tal & Karaman's inertia not matched.
