# Validation of the S3 versions A and B (adversarial review, 2026-10-18)

> **Outcome (2026-10-18):** the review's recommendation was followed. Version B was removed, S3 = two plain switches; decision and removal record in `docs/s3_indi_ns2_decision_2026-10-18.md`. The proposed lab test of B is no longer needed.


Scope: claims C1–C7 of the review brief about S3 (INDI + NS2), Version A (plain sum, flown) and Version B (`ns2_mode = 1`,
SIL only). Sources:
- the live controller (`crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c` L225–300) and power
  distribution (`power_distribution_quadrotor.c` L95–190);
- the 13 tree-validation flights of 2026-10-15, as merged CSV and 500 Hz uSD;
- new analyses (`experiments/analysis/validation_s3_*.py`, Appendix B).

Nothing in the firmware, yaml or existing docs was changed.

**Bottom line:**
- The *measurement* argument of C1 is right.
- The *double-counting magnitude* is wrong for the hardware as flown. On the crossing time scale INDI realises only about
  a fifth to a third of the push, NS2 about two thirds, and the two add to roughly one.
- Version B, whose justification rests on a strong INDI, would on the same data remove almost all of NS2's benefit
  (predicted dip ≈ INDI alone, −10 … −11 cm).
- **Keep A as the S3 of the study.** Fly B only as the discriminating test of §5.

## 1. Verdicts

| Claim | Verdict | Evidence (one line) |
|---|---|---|
| C1 mechanism | **PARTLY** | The measurement part holds: the gate push is the same in all four strategies, S0 −1.0/−1.1, S1 −1.0/−1.1/−1.1, S2 −1.1/−1.2, S3 −1.1/−1.25 m/s² above a baseline of −0.05 (App. A1), so compensation does not change what INDI measures. The consequence "thrust ≈ 2× push, net +1× push" holds only for an *ideal* INDI. The data-fitted model gives INDI an effectiveness of 0.2–0.35 on the crossings (App. A3), so the terms complement rather than double. |
| C2 data | **PARTLY** | The numbers reproduce exactly with gate g1. With the tighter g2 the A8 values are S1 −1.36/−1.58/−1.45 and S3 −1.63/−1.72: S3 slightly *larger*, within noise. Comparability is limited: two of the three S1 A8 flights ran at 3.44–3.49 V against 3.61–3.67 V for S3, and the A1 flights are single runs at different batteries. The inference "S3 = S1 ⇒ A double-counts" does not follow: equality is expected whether or not A over-compensates, and S0 and S2 show the same push. |
| C3 hardware result | **CORRECT** (n small) | A8 dip, mean of crossing means: S0 −15.7 (sd 2.8), S1 −10.6 (1.8), S2 −5.5 (1.4), S3 −1.9 (2.4) cm. Upward peak: S3 +1.5 mean, max +5.7. A1 z error mean: S1 −0.6, S3 +2.6 cm. Within S1 the dip tracks battery: −9.2 cm at 3.86 V against −11.1/−11.6 cm at 3.44–3.49 V. |
| C4 B algebra | **PARTLY** | The algebra is right: in steady state B = INDI alone; B only adds the 30 Hz high-pass of the prediction; a biased NS2 is corrected in steady state. But B's net NS2 contribution measured on the logged prediction is about 0.04 m/s² RMS (App. A4). B therefore also removes NS2's 20–40 ms lead over the filtered measurement. Its benefit exists only if INDI itself is strong, which the flights contradict. |
| C5 SIL | **CORRECT as a SIL statement, not informative for the hardware** | The SIL's INDI is near-ideal: INDI alone dips 0.75 cm (hardware 10.6) and NS2 alone 0.45 cm (hardware 5.5). The plant force is the NS2 model itself, so the SIL sits in the regime where A must over-count; the hardware does not. Extra runs (plant scale 0 and −1): App. A5. |
| C6 implementation | **CORRECT** | Code review: filter initialised at zero (L138), updated every step inside the `indi & 1` branch with 0 when NS2 is off, so switching `ns2_mode` or `rnn.en` at run time starts clean. The current build reproduces all four recorded replay hashes with `ns2_mode = 0` (App. A6). |
| C7 mental model | **PARTLY** | For the geometric baseline, "NS2 first, feedback on the rest" happens automatically: PD acts on the error that NS2 left. For INDI it does not, because INDI acts on the measured force, not the error. B is the faithful implementation of the model. On this hardware, however, INDI is the *weaker* partner, so "INDI only on the rest" hands the leftover to the weak loop and throws away the strong one. |

## 2. What is solid, what is plausible, what is wrong (plain words)

**Solid.**
- INDI's measurement `a_imu_f − a_rpm_f` measures the downwash push itself. It does not shrink when NS2 or INDI add
  thrust: same value in all four strategies.
- The RPM thrust model is calibrated (RPM thrust / weight = 1.00 in solo hover).
- The push is a short pulse at each A8 crossing: about −2.5 to −3 m/s² peak, half-width 130–190 ms, impulse
  ≈ −0.6 m/s. The dip comes 0.5–0.7 s later, as the slow position loop's response
  ($\omega_n = \sqrt7$ ≈ 2.6 rad/s) to that impulse.

**Plausible (fits all four strategies, not yet proven).**
- On the crossing time scale INDI delivers only a fraction of its correction; NS2's feed-forward is about twice as
  effective.
- The best concrete mechanism is **motor headroom**. With the moment INDI term on, motor commands run at a median peak
  of 0.80–0.85 of full range (0.59 without INDI) and saturate (5.5–8.8 % of samples above 0.95 in the low-battery S1
  flights). The power distribution then cuts all four motors equally, keeping the torques and sacrificing collective
  thrust.
- Around the crossings only 67–68 % of the commanded thrust increase arrives in S1/S3, against 94 % in S0/S2.
- The S1 flight on a fresh battery, with 0.3 % saturation, has the smallest S1 dip.

**Wrong.**
- "A corrects the push twice": true only for an ideal INDI. Such an INDI would leave S1 with almost no dip, not
  −10.6 cm, and would push S3 up by ≈ +8 cm, not +1.5 (model, App. A3).
- "B keeps NS2's fast part, so it can only be better than A on the dip": B's surviving NS2 part is about 0.04 m/s²
  RMS. With the INDI as flown, B ≈ S1.

**For the supervisor.**
- With both switches on, NS2 does most of the work at the crossings, because INDI's correction is clipped by the
  motors. That is why the plain sum (A) works well: −1.9 cm dip against −10.6 cm for INDI alone, with a small overshoot.
- The "leftover" version (B) would make INDI responsible again, and on our hardware INDI is the part that cannot
  deliver.
- If INDI's headroom problem were fixed, A would start to overshoot and B would become the right choice. That is a
  prediction we can test.

## 3. Recommended supervisor question and default

**Question:** "S3 = INDI + NS2 as two plain switches (A) has the smallest crossing dip (−1.9 cm against −10.6 cm for
INDI alone) and a small overshoot (up to +5.7 cm). The analysis suggests this works because INDI's own correction is
largely clipped by motor saturation, so the two terms complement rather than double. A version in which INDI handles
only what NS2 leaves (B) is implemented but would, on the current data, lose most of the improvement. Do we keep A as
S3 of the study, and do you want B flown once as a check (6 flights)?"

**Default: A** — it is the simple-switch design the supervisor asked for, it is what was flown, and it is the better of
the two on every model consistent with the data. Report B only if the check in §5 contradicts this analysis.

## 4. Statements in existing docs that are overstated (quoted; do not edit those files — replacement text proposed)

`docs/meetings/2026-10-22.md`, Questions:

| Quoted | Proposed replacement |
|---|---|
| "with both on, a push of 1 is corrected twice (2)" | "with an ideal INDI, a push of 1 would be corrected twice (2); on our hardware INDI delivers only a fraction of its correction at the crossings, so the two terms add to about 1" |
| "flights confirm it: the measured push in S3 equals the one in S1 …" | "flights confirm that INDI measures the push itself: it is the same in all four strategies (≈ −1.1 m/s² in the A8 gate), whatever compensates" |
| "A: NS2 and INDI both fix the same push, so it is fixed twice" | "A: NS2 and INDI both act on the same push; because INDI's correction is largely clipped by the motors, together they fix about all of it" |
| "push of 1 / NS2 20 % too high: A 2 / 2.2, B 1 / 1" | "(ideal INDI) push of 1 / NS2 20 % too high: A 2 / 2.2, B 1 / 1 — not what the flights show" |
| "B: … the filter only lines up the timing" | "B: INDI works on the rest; this also removes NS2's lead over the filtered measurement, so B keeps almost none of NS2's effect" |
| "the simulation cannot show B's advantage on the dip: its INDI has no dip (hardware: −10.4 cm)" | "the simulation's INDI is near-ideal (no dip); on a model fitted to the hardware, B's dip is about that of INDI alone (−10 … −11 cm)" |

`docs/ns2_s3_versions_A_B_2026-10-17.md`:

| Quoted | Proposed replacement |
|---|---|
| "B behaves like INDI alone (no double counting); A adds an upward bias … the signature of the doubled correction" | "In the SIL, whose INDI is near-ideal, B behaves like INDI alone and A adds an upward bias; on the hardware INDI is far from ideal, and this ranking need not hold" |
| Heading "Evidence from the flights of 2026-10-15 (why A double-counts)" and "NS2 on top does not reduce what INDI measures" | "Evidence that INDI measures the push itself (same in S0–S3)"; the equality does not by itself show double counting |

## 5. The discriminating lab test

**Fly, in one session on fresh batteries (≥ 3.8 V at take-off, swap after every 2 flights), A8 at the tier-1 speed
0.4 m/s, interleaved:**
- S1 × 3;
- S3-A × 3;
- S3-B × 3;
- **S1 with `indi = 1`** (position INDI only, no moment term) × 3 — the headroom test.

Log `a_rpm_f`, `a_imu_f`, `a_nn_fz`, `rnn.pred_z`, `thrustSi`, the four motor commands and the z error. That is 12
flights.

**Predictions:**

| | This analysis (weak INDI from motor headroom) | Brief's picture (strong INDI, A double-counts) |
|---|---|---|
| S3-B dip | ≈ S1 (−9 … −11 cm) | ≤ S3-A, little overshoot |
| S3-A peak | small | large |
| `indi = 1` dip | much smaller than S1 (−3 … −6 cm), with less motor saturation | ≈ S1 |

**Statistic:**
- Per flight, the mean dip over its 4 crossings; per-flight sd ≈ 1.5–2.5 cm.
- The predicted A–B difference is ≈ 8 cm, so 3 flights per arm separate it with a two-sample *t* (or a permutation
  test on 6 flight means) at large effect size.
- Secondary: per crossing, the delivered share Δ(RPM thrust)/Δ(commanded thrust) and the saturation fraction in
  ±0.3 s.

**What proves this analysis wrong:**
- S3-B mean dip within 3 cm of S3-A; or
- `indi = 1` not improving on S1 while saturation drops.

**What proves the brief's picture wrong:** S3-B dip ≥ 7 cm larger than S3-A.

## 6. Simpler alternatives considered

| Option | Assessment |
|---|---|
| A (sum) | Simplest, as flown, best on the data. Risk: overshoot grows if INDI becomes effective. |
| B (leftover) | Faithful to C7, but on the current hardware ≈ INDI alone. |
| NS2 only when INDI is off | That is S2; S3 would then not exist as a strategy. |
| Scale NS2 by (1 − η) in S3 | Needs η, which depends on battery and saturation. Not simple, not recommended. |
| Remove the cause | Reduce the moment term's noise (`indi = 1`, or a lower moment-term filter) so INDI has headroom. This is a controller change for the study (all INDI strategies), not an S3 option; fly the `indi = 1` test first. |

---

## Appendix

### A1. C2 recomputed (`validation_s3_c2_gates.py`)

| Item | Result |
|---|---|
| Offline residual (controller definition, 500 Hz uSD) vs onboard | correlation 0.991–0.998, bias ≤ 0.001 m/s² on S1/S3 |
| Baseline far from the neighbour (horizontal distance > 0.35 m), all A8 flights | −0.05 … −0.07 m/s² |
| RPM thrust / weight in solo hover | 0.996–0.999 |
| Commanded thrust / weight in solo hover | S0 1.018, S1 0.86: the commanded-to-delivered thrust ratio varies with the flight or battery, and INDI's feed-forward of the residual does not correct it (the integral does) |

| Flight (A8) | V | g1 mean / median | g2 mean / median | pred g1 |
|---|---|---|---|---|
| S0 1731, 1735 | 3.83, 3.72 | −1.19/−1.10, −1.06/−0.91 | −1.83, −1.81 | −1.38, −1.35 |
| S1 1743, 1744, 1746 | 3.49, 3.44, 3.86 | −1.05/−0.96, −1.14/−1.10, −1.18/−0.97 | −1.36, −1.58, −1.45 | −1.10, −1.17, −1.03 |
| S2 1814, 1816 | 3.85, 3.79 | −1.22/−1.12, −1.22/−1.01 | −1.88, −1.95 | −1.29, −1.25 |
| S3 1754, 1756 | 3.67, 3.61 | −1.21/−1.05, −1.31/−1.23 | −1.63, −1.72 | −1.09, −0.99 |

A1 (single flights): the push is S1 −2.89 and S3 −2.86, but S0 −1.25 and S2 −1.10. In S0/S2 the vehicle sags
14–20 cm away from the top vehicle, so A1 is not comparable across strategies.

### A2. Crossing profiles (`validation_s3_events.py`, `fig_events.png`)

Mean over crossings:

| | Push min (m/s²) | Push impulse ±0.8 s (m/s) | Pred min | Dip time after crossing | Thrust command / RPM rise (m/s²) |
|---|---|---|---|---|---|
| S0 | −2.7 | −0.61 | −2.7 | 600 ms | +1.58 / +1.49 |
| S1 | −2.5 | −0.63 | −2.4 | 690 ms | +3.16 / +2.13 |
| S2 | −3.0 | −0.63 | −2.5 | 560 ms | +2.04 / +1.92 |
| S3 | −2.9 | −0.74 | −2.3 | 910 ms | +4.75 / +3.24 |

The prediction's minimum comes 20–40 ms before the measured residual's.

Motor command, peak of the four (fraction of full range):

| | Median | p99 | Share > 0.95 |
|---|---|---|---|
| S0, S2 | 0.58–0.62 | 0.73–0.75 | 0 |
| S1 | 0.74–0.85 | 0.91–1.00 | 0.3–8.8 % |
| S3 | 0.79–0.80 | 0.98–1.00 | 1.8–2.6 % |

### A3. Vertical model (`validation_s3_model.py`)

The controller's z equations at 500 Hz:
- PD+I as in the code; NS2 as a 100 Hz hold of the pooled logged prediction; 30 Hz filters;
- first-order motors; push = the pooled measured residual.

Fitted by grid search to the four measured dips and peaks:

| Family | Fit error | S0 | S1 | S2 | S3 (dip / peak) |
|---|---|---|---|---|---|
| F1 mechanistic (RPM and IMU delays up to 160 ms, motor lag) | 60 | −15.3 | −7.5 | −8.1 | −3.3 / **+10.4** |
| F2 INDI effectiveness η = 0.2–0.35 | 4 | −14.3 | −11.5 | −5.0 | −2.6 / +1.7 |

F1 cannot fit: an INDI strong enough for S1 always makes A overshoot. Predictions under F2: S3-B −11.4 cm (≈ S1);
NS2 ×0: A = B = S1; NS2 ×1.3: A −0.9 / +4.7, B −11.4. Ideal INDI (no lag): S1 −0.4, S3-A 0.0 / **+7.8**, S3-B
−0.3 / +0.1.

Caveats:
- In both families the push must be scaled by s_p ≈ 1.5 to make the model's geometric dip match S0. This absorbs
  outer-loop effects (estimator and attitude dynamics) that the model lacks.
- η is phenomenological; the motor-headroom data above give it a plausible cause.

### A4. Version B's added term on the logged prediction (`validation_s3_staircase.py`)

| Quantity | Value |
|---|---|
| Net NS2 contribution of B ($a_{nn} - f_{30}(a_{nn})$), RMS | 0.04–0.06 m/s² |
| of which the 100 Hz hold steps, RMS | 0.02–0.05 m/s² |
| Largest hold-step pulse | 0.4–1.5 m/s² for one or two samples |
| INDI term's own sample-to-sample change, RMS | 0.03 m/s² |

Values are for the A8 flights. The A1 flights run at 0.06–0.12 m/s². S2 A8 1809 (0.19, stair max 5.5) is an
outlier, not investigated and not in the C3 dip numbers.

The staircase cannot excite the loops: its pulses last a few milliseconds and are of the INDI term's own noise level.
It is also a direct measure of how little of NS2 B keeps.

Alignment: $f_{30}(a_{nn})$ and the measurement use the same filter and rate. The prediction leads the physical push by
20–40 ms at its minimum and runs on 100 Hz relative-state updates, so in B the leading part is cancelled except for
what lies above 30 Hz.

### A5. Extra SIL runs (`omar_ns2_closed_loop_sil.py --mode-matrix 0,-1 --tag _validation_2026-10-18`)

Plant force scale = the scale of the *physical* downwash force in the simulated plant. NS2 predicts as usual, so
scale 0 is "NS2 sees a push that does not exist" and scale −1 is "NS2 has the wrong sign". The summary file repeats
the −1 rows; the copies are identical.

| indi | NS2 | Plant scale | Dip (cm) | Bump (cm) |
|---|---|---|---|---|
| 3 | off | 0 | 0.59 | +1.03 |
| 3 | A | 0 | 0.26 | +1.13 |
| 3 | B | 0 | 0.59 | +1.05 |
| 0 | A = B | 0 | 0.23 | +1.11 |
| 3 | off | −1 | 0.27 | +1.37 |
| 3 | A | −1 | 0.07 | +1.02 |
| 3 | B | −1 | 0.44 | +1.55 |
| 0 | A = B | −1 | −0.05 | +1.41 |

`max |ez|` is 50 cm in every row, the same value everywhere (not crossing-related); offsets ≤ 0.2 cm.

Reading:
- With `indi = 0` the two modes give identical results, as the code requires.
- B ≈ INDI alone whatever NS2 predicts, as C4 says.
- A wrong NS2 (scale 0 or −1) changes every result by **≤ 0.5 cm** in either mode. The SIL's INDI absorbs any NS2
  error almost completely, which confirms that the SIL sits in the ideal-INDI regime (C5).
- These runs cannot rank A against B for the hardware, where INDI alone leaves a 10.6 cm dip.
- The delayed or noisy INDI case was run in the vertical model rather than the SIL. Family F1 has RPM and IMU delays
  and does not reproduce the flights (App. A3); `noise` in `validation_s3_model.py` was left at 0.

### A6. C6 byte identity

The current host build (`.so` of 2026-10-17 17:02, contains `ns2_mode`) was compared against itself on the four
recorded replays:

| Replay | sha256 (current build) | Recorded value |
|---|---|---|
| A8 indi 3 | a300021ed3b0 | a300021ed3b0 |
| A8 indi 0 | 404a9d4373d3 | 404a9d4373d3 |
| A1 indi 3 | 9466b5a18dea | 9466b5a18dea |
| A1 indi 0 | 81e66789889c | 81e66789889c |

All four are identical. The recorded hashes were produced by the pre-`ns2_mode` code, so the build with Version B
reproduces them exactly with `ns2_mode = 0`.

Limits of the check:
- The pre-NS2 build itself is not available, so this is a comparison against recorded hashes, not against a second
  binary.
- It covers the host build, not the ARM image `study_tree_v2.bin`. Flash identity follows only if both are built from
  the same source.

Together with the code review (filter updated with 0 when NS2 is inactive, applied only under `ns2_mode == 1 &&
ns2_active`) and the SIL rows with `indi = 0` and A = B in A5, **C6: CORRECT**.

### B. Reproduce

```
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/validation_s3_c2_gates.py   # A1, writes out/validation_s3/aligned_*.npz
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/validation_s3_events.py     # A2
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/validation_s3_model.py      # A3 (about 70 s)
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/validation_s3_staircase.py  # A4
CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build /usr/bin/python3 experiments/analysis/omar_ns2_closed_loop_sil.py --mode-matrix 0,-1 --jobs 6 --tag _validation_2026-10-18   # A5
CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build /usr/bin/python3 flying_drone_stack/firmware_app/host/test_omar_ns2_off_identical.py --old-dir ~/Desktop/crazyflie-firmware/build   # A6 (self-compare, prints the hashes)
```
