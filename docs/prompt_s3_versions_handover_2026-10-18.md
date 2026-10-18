# Handover prompt: S3 Version A vs B, what we now believe (2026-10-18)

> **Closed 2026-10-18:** Version B was removed and S3 decided (`docs/s3_indi_ns2_decision_2026-10-18.md`); kept as history.


Paste everything below the line into the other agent.

---

You are picking up the S3 (INDI + NS2) Version A/B question in `~/Desktop/flying_robot_course`. An independent
adversarial review was done on 2026-10-18, and it **changes the conclusion** of the earlier docs. Read this whole
message first, then `docs/validation_s3_versions_2026-10-18.md` (the full review, with numbers, appendix and
reproduction commands). Do not act on `docs/ns2_s3_versions_A_B_2026-10-17.md` or the Questions section of
`docs/meetings/2026-10-22.md` until you have read the review: parts of both are now known to be overstated (list in
§4 of the review).

## 1. The two versions, in one line each

Controller: `controller_omar_indi.c` (slot 9). At an A8 crossing the top drone's downwash pushes the bottom drone
down.
- **NS2** predicts the push and adds thrust (`a_d -= a_nn`).
- **INDI** measures the push (`a_rpm_f − a_imu_f`) and adds thrust.

| Version | What it does | Status |
|---|---|---|
| **A** (`ns2_mode = 0`) | Both corrections are summed. | Flown 2026-10-15 |
| **B** (`ns2_mode = 1`) | INDI is meant to fix only what NS2 missed: adds `filter30(a_nn)` to the INDI term, so in steady state only INDI acts and NS2 keeps only its fast (> ~30 Hz) part. | SIL only, never flown |

## 2. Why we first believed B was right

1. **The theory.** With an *ideal* INDI, INDI alone already cancels the full push, so A would correct it twice and
   push the drone up. B avoids that.
2. **The SIL agreed.** The SIL's INDI is near-ideal (INDI alone dips 0.75 cm).
3. **A misread of the flight data.** INDI measured the same push in S1 (INDI only) and S3 (INDI + NS2). This was
   read as proof of double counting. It is not: the measured push is the same in **all four** strategies, S0 and S2
   included (≈ −1.0 to −1.25 m/s² above baseline in the A8 gate). It only shows that INDI measures the push itself,
   which is true under either picture.

## 3. What we now believe

The hidden assumption, "INDI is strong", is false on the hardware. A8 crossing dips (mean, cm):

| Strategy | Dip |
|---|---|
| S0 (nothing) | −15.7 |
| S1 (INDI only) | −10.6 |
| S2 (NS2 only) | −5.5 |
| S3, Version A (both, summed) | −1.9 (peak +1.5, max +5.7) |

- INDI alone removes only about ⅓ of the dip and NS2 alone about ⅔. Summed (A) they fix about one push, not two.
- A vertical model fitted to all four strategies only fits if INDI delivers about 20–35 % of its correction at the
  crossings. A model with an ideal INDI predicts S1 ≈ 0 cm and S3-A ≈ +8 cm overshoot, neither of which happened.
- Under the fitted model, **Version B dips ≈ −11 cm, i.e. like INDI alone**. B hands the leftover to the weak
  partner and leaves only about 0.04 m/s² RMS of NS2.
- **Likely cause, not proven: the motors run out of headroom.** With INDI on, the busiest motor peaks at a median
  0.80–0.85 of full range (0.59 without INDI), with up to 8.8 % of samples above 0.95. Only 67–68 % of the
  commanded thrust rise actually arrives at the crossings (94 % without INDI), and the power distribution cuts
  collective thrust to keep the torques.
- **C6 (B does not change A) is confirmed.** The current host build reproduces all four recorded replay hashes with
  `ns2_mode = 0`.
- **The SIL cannot decide A vs B for the hardware.** Even a wrong NS2 (plant scale 0 or −1) changes its results by
  ≤ 0.5 cm, because its INDI absorbs everything.

**Working decision:** Version A is S3 for the study (most likely right, simplest, best flown result). This is not
final until the lab test below.

## 4. The deciding lab test (not yet flown)

One session, A8 at tier-1 speed, fresh batteries (≥ 3.8 V at take-off, swap every 2 flights), runs interleaved:

| Run | Flights | Purpose |
|---|---|---|
| S1 (pure INDI, NS2 off) | 3 | Re-fly: the old S1 A8 flights were mostly at 3.44–3.49 V |
| S3-A (`ns2_mode = 0`) | 3 | |
| S3-B (`ns2_mode = 1`) | 3 | |
| Optional: S1 with `indi = 1` (thrust INDI only, no moment term) | 3 | Tests the motor-headroom explanation; not needed for the A/B choice |

Log: z error, `a_rpm_f`, `a_imu_f`, `a_nn_fz`, `rnn.pred_z`, `thrustSi`, the four motor commands.

**Verdict rule** (per flight, the mean dip over its crossings):

| Result | Conclusion |
|---|---|
| B dips about as much as S1 (−9 to −11 cm), about 8 cm worse than A | **A is right** (expected) |
| B dips within ~3 cm of A, with less overshoot | **B is right**; the review was wrong |
| Optional `indi = 1` run: smaller dip than S1, with less motor saturation | Supports the headroom explanation |

## 5. Files

| File | Content |
|---|---|
| `docs/validation_s3_versions_2026-10-18.md` | Full review: verdicts C1–C7, quoted overstatements with replacement text, lab test, reproduction commands |
| `experiments/analysis/validation_s3_c2_gates.py` | Gate push numbers, offline residual |
| `experiments/analysis/validation_s3_events.py` | Crossing profiles, thrust delivery, motor saturation |
| `experiments/analysis/validation_s3_model.py` | Vertical model fit and A/B predictions |
| `experiments/analysis/validation_s3_staircase.py` | Version B's net NS2 term |
| `experiments/analysis/out/validation_s3/` | Outputs of the four scripts |
| `experiments/analysis/out/omar_ns2_closed_loop_sil/summary_validation_2026-10-18.md` | Extra SIL runs |

## 6. Rules

- Do not edit `docs/meetings/2026-10-22.md` or `docs/ns2_s3_versions_A_B_2026-10-17.md` unless the user asks; the
  proposed replacement wording is in §4 of the review.
- Do not change firmware, controller or yaml for this.
- No `Co-Authored-By` trailers. Commit only when asked, never push unless asked.
- When the user asks for the current state, answer in plain words: **A is most likely right; one lab session
  (S1 / A / B, 3 flights each) decides it.**
