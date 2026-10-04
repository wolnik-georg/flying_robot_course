# 56 — INDI oscillation: overview tables (ours vs Omar C / Rust)

Status 2026-10-04. Source: docs 53, 53a, 54, 55 (Cursor) plus my independent checks against code and flight metadata.
**Nothing here is a proven root cause.** Open items and the planned simulation/gain-history work are in section 8.

## 1. What is the same (so not the cause)

| Item | Ours | Omar C / Rust | Same? |
|---|---|---|---|
| Motor output path (mixer, torque-to-thrust ratio, arm length) | `controlModeForceTorque` → `power_distribution_quadrotor.c` | same | yes |
| Mass | 0.041 kg | 0.0427 kg | ~4 % |
| Thrust constant | per-motor ≈ 4.1e-10 | one scalar | ~4 % |
| Motor lag (bench) | ≈ 44 ms | same hardware | same |
| Stabilizer, motors, EKF, setpoint rates | 1 kHz, 1 kHz, 100 Hz, 100 Hz | same | same |

## 2. What differs

| Item | Ours | Omar C / Rust | Size of difference |
|---|---|---|---|
| Attitude stiffness (J·kr) | 0.0575 N·m/rad | 0.007 N·m/rad | **ours ≈ 8×** |
| Attitude damping (J·kw) | 0.0041 | 0.00115 | **ours ≈ 3.5×** |
| Natural frequency | 7.8 Hz | 3.3 Hz | stable ceiling ≈ 4.5 Hz |
| Position gain KP (xy / z) | 64 / 48 | ≈ 7 | **ours ≈ 7–9×** |
| Position damping KV | 5 / 7 | ≈ 4 | similar |
| **Residual sign in position loop** | **+1: adds it** | **subtracts it** | **opposite** |
| Law structure | replaces the torque with τ_cur + J·(α_ref − α_meas) | keeps the geometric torque, adds (τ_rpm − J·α) | different |
| Attitude integral | off | on (0.03) | different |
| Inertia (roll / pitch) | 23.95e-6 | 16.57e-6 | ours 1.45× |
| Control rate | 1 kHz, no hold | 500 Hz, output held | 2× |
| Angular-acceleration filter | 206 Hz | 30 Hz | ours lets more noise through |
| Total phase lag at 6.3 Hz | 113° | 129° | ours is lower |

## 3. What the flights show (A1 hover, 2026-10-02)

| | Ours | Omar C | Omar Rust |
|---|---|---|---|
| Mean height error | −17 to −19 cm | +2 to +4 cm | **−11 cm** |
| Gyro RMS (500 Hz cards) | 260–290 °/s | 55–97 °/s | 55–97 °/s |
| Main oscillation frequency | 4.7–5.7 Hz | 3.5–3.7 Hz | 3.2–3.9 Hz |
| Share of energy in 5.5–7.5 Hz | 14–36 % | 1–12 % | 1–12 % |
| A8 mean height error (comparison) | +3 cm | +22 cm | +19 cm |

Omar Rust also sags on A1 (−11 cm). So the residual sign alone cannot explain the height error.

## 4. Ranking of possible reasons (plausible, none proven)

| Rank | Difference | Why it matters |
|---|---|---|
| 1 | Attitude stiffness 8× and natural frequency 7.8 Hz | The stable ceiling is about 4.5 Hz. Flown `kr` is about 3× past it (investigation). |
| 2 | Residual sign `+1` | Our code comment records 2× the sag under a known disturbance. It feeds the commanded attitude. |
| 3 | Position gains 7–9× stiffer | Stiff loop around the lagged residual. |
| 4 | Inertia 1.45× | Scales the torque increment; smaller effect (Omar flies calmly). |
| 5 | Rates and filters | Small, and ours has less total lag. |
| 6 | Motor and mixer path | No difference. |

## 5. What is known and unknown

| Known | Unknown |
|---|---|
| Omar's controllers fly this hardware calmer on A1. | Whether the correct residual sign (`res_sign=-1`) combined with low attitude gains and a soft position loop fixes it. |
| **Correction (validation of doc 57):** `kr=483/kw=76` (close to Omar-equivalent 290–420) **was flown twice with INDI on 2026-09-11**, always with `res_sign=+1`: with matched KP 13 the attitude peak improved but **position holding broke** (z overshoot/drift); with geometric position gains (KP 40/30) position holding was fixed but a **small hover growth trend** remained. `kr=987/kw=109` with matched KP 26 gave the best hover of that day. Sources: `docs/FLIGHT_CARD_VALIDATION.md` L154–158, `docs/lab_sessions/2026-09-11.md`. | Whether the broken position holding at low kr was the `+1` residual sign (it fits: the sign amplifies the sag). |
| `res_sign=-1` has never been flown alone as a runtime change. The earlier flip on 2026-09-09 diverged in hover. | Whether the three changes together (gain level, sign, softer position loop) fix it. |
| Harness is a toy and its "500 Hz hold" result is an artifact. | The real effect of the 500 Hz output hold. |
| Card-log pairing is per variant, not per flight. | The exact oscillation frequency per flight. |

## 6. Errors in Cursor's docs 55 and 57

- **Doc 57:** Omar geometric ζ is shown as 0.082; it is ≈ 1.7 (ζ = KW / (2·√(KR·J))).
- **Doc 57:** "KP < 30 never flown with INDI" holds only for the CSV meta; the 2026-09-11 lab runs flew KP 13 and 26 (and 40/30) with INDI.
- **Doc 57:** residual-to-tilt phase margins (89° / 269°, crossover 0.14 Hz) are toy artifacts and carry no information.
- **Doc 57:** it says a full SIL run with motor lag would need edits to existing scripts. It does not: the CS2 SIL plant already has `sim.physics.motor_tau` (0.044 s default, `docs/09_Simulation.md`).
- **Doc 55:**
  - **`ki_z` row:** it says `ki_z=16` on the ours flights, but the meta shows `pos_ki_z=0.0`.
  - **Gain units:** it calls them "not comparable", but multiplying by J makes them directly comparable.
  - **Residual sign:** it ranks the sign only MEDIUM, but our own code records the 2× effect.

## 7. Next step, not yet planned

- First test in simulation: `res_sign=-1`, `kr` ≈ 483 / `kw` ≈ 76 (already flown with `+1`), and geometric-like position gains (KP 40 / 30).
- Then a lab flight on A1, using only yaml parameters, so no reflash.
- It would come after the NS2 100 Hz test.

## 8. Simulation result (CS2 SIL with motor lag and downwash, doc 58, checked 2026-10-04)

The SIL does **not** reproduce the flight oscillation (gyro RMS ≈ 0.02 °/s vs 260–290 °/s). The grid is exploratory. One effect is nevertheless systematic:

| Mean height error on A1 (cm), SIL | res_sign +1 | res_sign −1 |
|---|---:|---:|
| kr 2400, position gains flown (64/48) | −7.1 | 0.0 |
| kr 2400, geometric-like (40/30) | −10.4 | 0.0 |
| kr 2400, matched low (26) | −11.6 | 0.0 |
| kr 987, any of the three position settings | −7.1 / −10.4 / −11.6 | 0.0 |
| kr 483, geometric-like (40/30) = 2026-09-11 attempt 4 | −10.4 | 0.0 |
| kr 483, matched (26) = 2026-09-11 attempt 3 | −11.6 (flight: z drift) | 0.0 |
| kr 483, flown position gains (64/48) | −19.7, unstable (gyro 102 °/s, lateral 42 cm) | −8.4, unstable (gyro 110 °/s, lateral 65 cm) |
| Omar C (reference) | ≈ 0 | — |

- **Reading:** with the residual added (`+1`) the SIL shows a height sag of 7–12 cm; with it subtracted (`−1`) the sag is gone. The attitude gain level does not change this. This matches the flights (ours −18 cm, Omar C +3 cm) in direction, not in size.
- **Caveat:** the SIL's residual estimate is noise-free, so `−1` looks perfect there. On hardware the residual is noisy and delayed.
- **Not explained by the SIL:** the attitude oscillation. The investigation points to the rotor-speed (RPM) measurement being stale against the gyro; the SIL feeds the controller the true post-lag rotor speed, so that mechanism is missing.
- The "regime" column in doc 58 labels almost every run "divergence" even when the vehicle hovers calmly; ignore it.

## 8b. RPM-delay simulation and log timing (doc 59, checked 2026-10-04)

| Question | Result |
|---|---|
| Is the logged rotor-speed torque time-aligned with the gyro? (Oct-02 A1 card logs, DShot) | **Yes for ours: lag ≈ −1 ms** (coherence ≈ 0.99 at 5.5–5.9 Hz). Omar C ≈ −9 ms (oscillation at a different frequency, 3.0–3.5 Hz). |
| Does the card log show a slow (≈ 20 Hz) rotor-speed update? | **No**: a new value arrives at every 500 Hz log sample. |
| Does adding an RPM delay / hold / noise to the full SIL reproduce the limit cycle? | **No.** 20 delay × rate combinations: ours stays at 1–2 °/s (flight 260–290). The single "36 °/s" point (50 ms, 20 Hz) is a corner where the Omar run returned NaN, so ignore it. |

- **Reading:** a stale rotor-speed measurement, the earlier explanation for the limit cycle, is **not supported**: the logged RPM torque lines up with the gyro within about 1 ms, and it is not updated slowly. (Tested with DShot; the optical deck used in the July flights was not tested.)
- **Bigger point:** the full compiled controller with the 44 ms linear motor lag stays calm at kr 2400 in the SIL, while the old 1-axis toy harness limit-cycles at 7.9 Hz. So the July explanation "linear 44 ms motor lag alone causes the limit cycle" is **not confirmed** by the compiled controller. What is still missing in the SIL: amplitude-dependent actuator behaviour, gyro noise and filter, estimator delay, the 500 Hz output hold.
- **Doc 53 correction:** its delay budget assumed a 50 Hz rotor-speed hold (23° of phase). The logs show a 1-sample hold, so that term is too large; it is the same for every variant, so the comparison between variants does not change.
- **Cursor errors in doc 59:** it says the regime label was fixed, but `sweep_summary.md` still tags calm runs "divergence"; its "best case" row uses a run where the Omar result is NaN.

## 8c. Harness vs SIL bisection (doc 60, checked 2026-10-04)

| Question | Result |
|---|---|
| Is the old 1-axis harness faithful to the compiled law? | **No** (scalar attitude error, a 2-tick command dead time, a one-step-old torque base, no position loop or residual). |
| What does the harness limit cycle need? | **Motor lag ≈ 44 ms AND a command dead time of ≥ 2 ms.** Dead time 0 → calm; 1 ms → calm; 2 ms → limit cycle (σ 4.3 rad/s, 7.9 Hz). The one-step-old rotor-speed base is not needed. |
| Full SIL (compiled controller, motor lag 44 ms) | Calm (≈ 0.7 °/s): it applies the command with **zero** extra delay. |
| Full SIL + 2 ms command dead time | **Ours: 233 °/s (flight 260–290), 7.4 Hz (flight 4.7–5.7). Omar C: stays calm (≈ 0 °/s).** |
| RPM delay/hold, 500 Hz hold, spool asymmetry added alone | No effect. |

- **Reading:** at kr 2400 the loop is within about **1–2 ms** of instability. The motor lag alone is not enough; a small extra delay anywhere in the loop tips ours into the limit cycle, while Omar's softer gains stay calm with the same delay. The July explanation ("44 ms lag alone") is **refined**: lag **plus a couple of milliseconds of pure delay**.
- **Real hardware has such delays** (estimate 2–4 ms: DShot/ESC frame and latency, gyro 80 Hz low-pass ≈ 2 ms, compute at the end of the tick). The SIL has none of them. The SIL did **not** test the gyro low-pass, sensor delay or noise, nor dead times other than 2 ms.
- **Not yet validated:** frequency (7.4 vs 4.7–5.7 Hz) and the height error do not match flight; the "2 ms" is not independently measured.
- **Cursor errors in doc 60:** the "regime" label still reads "divergence" for calm runs (0.7 °/s); only dead time 2 ms was tried in the SIL (not 1, 3, or the gyro low-pass).
- **Consequence for the goal:** gain level matters because of the ~2 ms margin; lowering kr (or removing delay, e.g. a higher gyro low-pass cutoff — speculative, untested) buys margin. The residual sign (height sag) is a separate effect.

## 8d. Delay-margin simulation (doc 61, checked 2026-10-04)

| Our INDI (kr 2400) in the full SIL | Gyro RMS | Dominant frequency | Height error |
|---|---:|---:|---:|
| No added delay, no gyro filter | 0.7 °/s (calm) | — | −0.4 cm |
| **Only the real 80 Hz gyro low-pass (0 ms extra delay)** | **161 °/s** | 8 Hz | **−47 cm** |
| Gyro low-pass off, dead time 0 / 0.5 / 1 ms | calm (0.7–1.7 °/s) | — | −0.4 cm |
| Gyro low-pass off, dead time 1.5 / 2 / 3 / 4 ms | 124–155 °/s | 6–8.7 Hz | −22 to −50 cm |
| Flight (A1, ours) | 260–290 °/s | 4.7–5.7 Hz | −18 cm |
| Flight, top drone (geometric), gyro RMS | 4–9 °/s (calm) | — | — |

- **Strongest independent result of the whole series:** the gyro low-pass in the firmware (80 Hz, a FACT) alone is enough to push our high-gain loop into a limit cycle in the simulator. No guessed delay is needed. Without it, about 1.5 ms of command dead time does the same.
- **Hardware delay budget (doc 61):** gyro low-pass 3.25 ms (group delay at 6 Hz) + about 2.5 ms command path (ESC, DShot; judgement) ≈ 5.75 ms (range 2–8 ms) — larger than the margin, so the delay is not too small to matter.
- **Plant is NOT validated:**
  - the oscillating runs lose height (−47 to −50 cm; flight −18 cm) and the frequency is 5–8 Hz (flight 4.7–5.7 Hz);
  - **every Omar-bottom run shows a constant 220 °/s on the top drone** (and 0.0 °/s on the bottom), independent of any delay setting, while the flown top drone is calm (4–9 °/s). This is a set-up artifact (likely shared global parameters between the two drones in one process), so the "Omar stays calm" statement from the SIL is **not** trustworthy;
  - the ESC and DShot delay values are judgement, not measurement.
- **Lever check:** a 250 Hz gyro low-pass (SIL only) did not fix the height error; it was not reported separately for the oscillation.
- **Doc 61 caveat wording:** it blames "top-drone partner gyro ~220 °/s in many runs" on the two-robot set-up; in the data it appears in **all** Omar-role runs and in none of the ours-role runs.

## 9. Final status (investigation closed for now, 2026-10-04)

### 9.1 Updated ranking after the simulations

| Rank | Difference / effect | Evidence now | Status |
|---|---|---|---|
| 1 | **Loop on a delay knife edge at kr 2400** (stiff attitude gain + motor lag 44 ms + a few ms of delay) | Real 80 Hz gyro low-pass alone makes ours oscillate in the SIL; without it ≈ 1.5 ms dead time does; Omar's gains are ≈ 8× softer | **Supported (simulation + firmware fact)**, plant not validated |
| 2 | **Residual sign `+1`** (adds the residual) | SIL height sag −7…−12 cm with `+1`, 0 with `−1`; code comment records 2× sag; matches flight direction | **Supported for the height sag**; not for the oscillation |
| 3 | Position gains 7–9× stiffer | Low `kr` with stiff position gains is unstable in the SIL; 2026-09-11 flights needed softer position gains | Plausible |
| 4 | Inertia 1.45×, 500 Hz hold, filters, structure | Small or untested | Minor / unknown |
| 5 | Stale rotor-speed measurement | Logged RPM torque aligned with gyro within ≈ 1 ms, new value every sample | **Not supported** |
| 6 | Motor/mixer path, "linear 44 ms lag alone" | Same path for all; lag alone does not oscillate the compiled controller | **Refuted** |

### 9.2 What we learned, in one table

| Topic | Result |
|---|---|
| Same | Motor path, mass (~4 %), thrust constants (~4 %), lag, platform rates |
| Differs most | Attitude stiffness 8×, position gains 7–9×, residual sign, structure, attitude integral |
| Delay | Hardware delay estimate ≈ 5.75 ms (2–8 ms): gyro low-pass 3.25 ms + command path ≈ 2.5 ms (judgement) |
| Margin | Ours at kr 2400 has ≈ 0–1.5 ms margin; any hardware delay tips it over |
| Not explained | Oscillation frequency (SIL 5–8 Hz vs flight 4.7–5.7 Hz), height collapse in oscillating SIL runs (−50 cm vs −18 cm) |
| Simulation limits | Omar-bottom runs have a constant 220 °/s on the top drone (set-up artifact); ESC/DShot delay is judgement; the 1-axis harness was not faithful |

### 9.3 Open items (not done)

| Item | Why it matters |
|---|---|
| Bench: command → thrust latency (and gyro-to-controller latency) | The one number that would pin down the delay budget |
| Lab flights from doc 58 / doc 61: baseline → `res_sign=-1` only → package (`res_sign -1`, kr 483 / kw 76, KP 40/30) | Separates sign from gain level; run the package first (the sign-only flip at kr 2400 diverged on 2026-09-09) |
| Log `res_sign`, `filt_dt_us`, `notch_en`, `filt_prewarp`, rpm source in the flight meta | They are not recorded today (`docs/lab_prep_log_filter_params.patch`) |
| Fix the shared-parameter artifact in the SIL | Only if the simulation is continued |

Plan: these lab items come after the NS2 100 Hz test (`docs/next_flight_card.html`).

Details and evidence: [`53`](53_INDI_Loop_Rates_and_Oscillation_Ledger.md), [`53a`](53a_INDI_Loop_Comparison_Overview.md), [`54`](54_INDI_Loop_Rates_Followup.md), [`55`](55_INDI_Motor_Model_and_Structure_Comparison.md), [`57`](57_INDI_Gain_History_and_Package_Simulation.md), [`58`](58_INDI_SIL_Package_Run.md), [`59`](59_INDI_SIL_RPM_Delay.md), [`60`](60_INDI_Harness_vs_SIL_Bisection.md), [`61`](61_INDI_Delay_Margin_SIL.md).
