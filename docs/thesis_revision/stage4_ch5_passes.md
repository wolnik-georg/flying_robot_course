# Stage 4 — Chapter 5 (Experimental setup and protocol): passes A–F

Revised file: `revised/ch5_experimental_setup.tex`. Combined build (revised Ch3–Ch5, original Ch1–2, 6–9) compiles with
no errors, no undefined references or citations (57 pages).

## Verdict (5 lines)
1. The chapter's design logic (frozen library, null scenarios, onboard logging, gate → freeze → interleave) is sound and kept.
2. Four design facts were stale against the code (500 Hz loop, 35 variables, $e_R$ "not logged", rotor-speed source) and are corrected.
3. Three passages reported project findings (simulation dry run, simulation defects, radio packet numbers); they are commented out (`% REMOVED (H1)`) and replaced by design-level statements.
4. The metrics are now defined as formulas from recorded variables, with windows taken from the commanded geometry; statistics are a skeleton with `\todo`s — the author must fix $n$, the test and the windows before the campaign.
5. Open risks: the campaign-status paragraph (D-05), the data-collection rationale (D-04) and the sign check in the gate (D-01).

## Rubric (0–5), before → after

| Criterion | Before | After | Note |
|---|---|---|---|
| 1 Correctness | 2 | 4 | rates, variable count, $e_R$, C2 regime, rotor source; D-04 open |
| 2 Completeness | 2 | 4 | metric definitions, statistics skeleton, scenario parameters |
| 3 Depth | 3 | 4 | gate interaction with the network input band |
| 4 Academic standard | 3 | 4 | H1 passages removed; status ¶ D-05 |
| 5 Explainability | 4 | 4 | |
| 6 Related work / referencing | 2 | 3 | NS2 split precedent, Gielis null-scenario basis, simulator model cited |
| 7 Cross-chapter consistency | 2 | 4 | with revised Ch3/Ch4 (network gate, prediction-off flights, runtime selection) |
| 8 Reference correctness | 3 | 4 | three new cites checked against records |

## A. Claim re-check

### A1. Literature claims

| Location | Claim | Key | Record | Verdict |
|---|---|---|---|---|
| §5.3.1 (new) | downwash concentrated beneath; negligible outside downwash at small separations; small chaotic lateral forces at larger distances | gielis2023aggregate | R03 §5 | supported (stand measurement; wording kept to the source) |
| §5.6 | simulator interaction model is Neural-Swarm2's | shi2022neuralswarm2 | R02; sim backend `neuralswarm.py` (logic copied from aerorobotics/neural-swarm) | supported; weights `[VERIFY]` |
| §5.7 | NS2 splits by ~20 s pieces, not samples | shi2022neuralswarm2 | R02 §VII-C | supported |

### A2. Implementation statements vs code

| Location | Statement | Code | Verdict |
|---|---|---|---|
| §5.1 | control loop 1 kHz | `stabilizer.c` RATE_MAIN_LOOP; `lib.rs:2350–2351` | corrected (was 500 Hz) |
| §5.1 | dataset 500 Hz; network ≈100 Hz | `usd_study_config.txt` header; `traj_iface.c` `g_rnn_div=10` | supported |
| §5.1 | default rotor source DShot; deck alternative | `traj_iface.c:508–520, 701–730` | corrected |
| §5.1 | 9 cm rotor-to-rotor | `lib.rs:261` arm 0.050 m ⇒ ≈0.10 m diagonal | `[VERIFY]` (text unchanged) |
| §5.1 | $a_{\mathrm{res}}$ reads zero without rotor source | `lib.rs:1781–1834` | supported |
| §5.3 / Tab 5.1 | team sizes, defaults, gating | `scenarios.py:134–470` | added from code |
| §5.3.2 | C2 moves at 0.4 m/s | `scenarios.py` C2 builder | corrected |
| §5.3.2 (new) | 0.2 m lateral gate | `residual_nn.rs:139–140, 281–286` | supported |
| §5.4.1 | 96 variables at 500 Hz | `usd_study_config.txt` | corrected (was 35) |
| §5.4.2 | recorded list | `usd_study_config.txt` | completed |
| §5.5.4 | $e_R$ logged every mode | `lib.rs:1972–1976`; config lines | corrected (was "not logged") |
| §5.4.3 | cross-correlation merge | not in listed code | `[VERIFY]` |
| Tab 5.2 | 15 cm RMS role-check threshold | not in listed code | `[VERIFY]` |

## B. Defect log (new)

| # | Defect | Sev | Handling |
|---|---|---|---|
| B5-1 | metric "within the window" with no window definition; crossing time could depend on controller response | S1 | APPLY: windows from commanded geometry + `\todo` |
| B5-2 | RQ3 "transfer penalty" never defined in the chapter that measures it | S1 | APPLY definition + `\todo` pairing |
| B5-3 | prediction-off flights described only in Ch4 | S2 | APPLY paragraph in §5.5.3 |
| B5-4 | Table 5.1 never referenced | S3 | APPLY |
| B5-5 | §5.5.1 Stage 3 sign check presupposes the D-01 outcome | S2 | `% DECISION D-01` comment |

## C. Rewrite
`revised/ch5_experimental_setup.tex` (complete). Labels unchanged; new label `sec:setup-stats`.

## D. Citation audit

| `\cite` | Exists | Claim | Location | Context |
|---|---|---|---|---|
| gielis2023aggregate | ✓ | downwash localisation | R03 §5 | ✓ (stand, not flight) |
| shi2022neuralswarm2 (×2) | ✓ | simulator model; segment split | R02, R02 §VII-C | ✓ |

No failed row.

## E. Examiner simulation

| # | Question | Where answered |
|---|---|---|
| 1 | How many repetitions per cell, and why that number? | §5.5.3 `\todo`, §5.5.5 (author) |
| 2 | Which test, and is it paired? | §5.5.5 skeleton |
| 3 | How is the analysis window chosen; could it favour a controller? | §5.5.4 Notation and windows |
| 4 | How is "transfer penalty" computed? | §5.5.4 Residual rejection |
| 5 | Why collect under S0 if compensation does not change $a_{\mathrm{res}}$? | **D-04** |
| 6 | How do you know the gate's sign check is the right sign? | **D-01** |
| 7 | Is the logged position the estimate or mocap? | `\todo` in §5.5.4 |
| 8 | What does C1 test if the network is gated off at 0.30 m? | §5.3.2 gate paragraph |
| 9 | How are clocks aligned, measured how? | §5.4.3 (`[VERIFY]` method) |
| 10 | Does simulation support any claim? | §5.6 |
| 11 | Battery drift? | §5.7 limitations |
| 12 | How is data-quantity cost (RQ4) measured? | **D-09** |

## F. Student-reader test

| Stumbling point | Resolution |
|---|---|
| "window" | defined |
| separation error sign | formula with $e_3$ |
| prediction-on/off | explicit paragraph |
| why coplanar controls matter | physical basis + gate paragraph |
| scenario parameters | in Table 5.1 |

## Change log
P5-01 ✓; P5-02 ✓; P5-03 ✓; P5-04 ✓; P5-05 ✓ (H1 comment); P5-06 ✓ (H1 comment); P5-07 ✓ (+VERIFY); P5-08 → D-05 comment;
P5-09 → D-05 comment; P5-10 → D-04 comment; P5-11 ✓ (+todos); P5-12 ✓ (+todo, D-09); P5-13 ✓; P5-14 ✓; P5-15 ✓;
P5-16 ✓ (+VERIFY); P5-17 ✓ (+VERIFY); P5-18 VERIFY comment; P5-19 VERIFY comment; P5-20 ✓; P5-21 ✓; P5-22 ✓; P5-23 ✓.
Additional: Gielis basis for null scenarios (from R03, Stage 2 gap G-list); prediction-off paragraph (consistency with revised Ch4).

## DECISION rows left open
D-01 (§5.5.1), D-04 (§5.5.2), D-05 (status ¶, §5.4.3), D-09 (§5.5.4 robustness).

## [VERIFY]
§5.1 9 cm rotor-to-rotor; §5.4.1 radio numbers if restored; §5.4.3 alignment method; §5.6 simulator weights; Tab 5.2 15 cm threshold.
