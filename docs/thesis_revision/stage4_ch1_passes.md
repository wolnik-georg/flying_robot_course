# Stage 4 — Chapter 1 (Introduction): passes A–F

Revised file: `revised/ch1_introduction.tex`. Combined build (revised Ch1–Ch5) compiles with no errors, no undefined
references or citations (59 pages).

## Verdict (5 lines)
1. The introduction promises only what the body delivers and claims no result; that discipline is intact.
2. Two factual errors about the hybrid (single-vehicle only) are corrected minimally; the italic gap and contribution 1 are unchanged and flagged (D-03).
3. Uncited physics claims now carry sources (Kiran, Gielis, Shi 2020 magnitude); "exactly two ways" and "unconstrained" are softened to what holds.
4. The apparatus paragraph matches the code (flash-resident or uploaded weights) and no longer reads as a result ("verified").
5. Ch6–9 are named in prose (H6); Strategy 3's status sentence and "four strategies are compared" are DECISION D-05.

## Rubric (0–5), before → after

| Criterion | Before | After | Note |
|---|---|---|---|
| 1 Correctness | 3 | 4 | hybrid sentences fixed; gap D-03 open |
| 2 Completeness | 4 | 4 | |
| 3 Depth | 3 | 3 | appropriate for an introduction |
| 4 Academic standard | 4 | 4 | |
| 5 Explainability | 4 | 4 | |
| 6 Related work / referencing | 3 | 4 | uncited physics claims sourced |
| 7 Cross-chapter consistency | 3 | 4 | consistent with revised Ch2/Ch3 |
| 8 Reference correctness | 3 | 5 | |

## A. Claim re-check (changed or new)

| Location | Claim | Key | Record | Verdict |
|---|---|---|---|---|
| §1.1 ¶1 | ≈9 g at 30 cm, within thrust capability | shi2020neuralswarm | R01 §I | supported |
| §1.1 ¶3 | nonlinear, lateral decay, asymmetric | kiran2025downwash, gielis2023aggregate | R11 §III-A; R03 §5 | supported |
| §1.1 ¶4 | heterogeneous sixteen; 56 % from abstract | shi2022, smith2023 | R02 Fig. 1; R04 abstract | corrected |
| §1.2 ¶3 | output can be bounded but not made correct | code | `residual_nn.rs:134` (OUT_CLAMP) | consistent with Ch3 |
| §1.2 ¶4 | team hybrid inside off-board MPC, combination best | hsieh2025onlineadaptation | R10 §3 | corrected |
| §1.3 ¶3 | residual sources as listed by Cobo-Briesewitz; team instance | cobo, hsieh2025 | R07 §1; R10 | corrected |
| §1.4 S2 | vertical component | code | `residual_nn.rs` scalar output | corrected |
| §1.6 apparatus | flash-resident or uploaded weights; numerical check | code | `Cargo.toml:21–23`, `build.rs:111–117`, `residual_nn.rs:154–171` | corrected |

## B. Defect log (new)

| # | Defect | Sev | Handling |
|---|---|---|---|
| B1-1 | "Four control strategies are compared" with one strategy unimplemented (H5) | S2 | DECISION D-05 comment |
| B1-2 | contribution 1 "first evaluation" contested by Hsieh 2025 | S1 | DECISION D-03 comment |

## C. Rewrite
`revised/ch1_introduction.tex` (complete). No label removed.

## D. Citation audit

| `\cite` | Exists | Claim | Location | Context |
|---|---|---|---|---|
| shi2020neuralswarm (new) | ✓ | 9 g | §I | ✓ |
| kiran2025downwash, gielis2023aggregate (new) | ✓ | structure of the interaction | R11, R03 | ✓ |
| hsieh2025onlineadaptation (×2 new) | ✓ | team hybrid in MPC | §2–3 | ✓ |
| cobobriesewitz2026lindi (new in ¶3) | ✓ | residual sources | §1 | ✓ |
| unchanged cites | ✓ | verified in Phase 1 | — | ✓ |

## E. Examiner simulation

| # | Question | Where answered |
|---|---|---|
| 1 | Hsieh 2025 already compares the families — what is new? | **D-03** |
| 2 | Is Strategy 3 compared or not? | **D-05** |
| 3 | Why is a 9 g force hard? | §1.1 ¶1 + ¶3 |
| 4 | "Exactly two ways" — what about physics models? | §1.2 softened |
| 5 | Can a learned model's output be unbounded? | §1.2 ¶3 |
| 6 | What is the contribution if strategies do not separate? | §1.6 negative result |
| 7 | Why only force residual? | Tab 1.1 |
| 8 | Single airframe — generalisation? | §1.6 does-not-claim |
| 9 | Is the network NS2? | §1.6 does-not-claim; Ch4 |
| 10 | Why three vehicles max? | Tab 1.1 |
| 11 | RQ3 "superposition does not hold" — overclaim? | RQ3 kept (Gielis linear regime noted in Ch2) |
| 12 | Who tuned what? | contribution 1 |

## F. Student-reader test
Glossed: downwash, blade flapping, slung payload (kept); "INDI-based hybrid" is defined by the preceding paragraph.

## Change log
P1-01 ✓; P1-02 ✓; P1-03 ✓; P1-04 ✓; P1-05 ✓; P1-06 → D-03 comment; P1-07 → D-03 comment; P1-08 ✓ (minimal) + D-03;
P1-09 ✓ (S2) / D-05 (S3); P1-10 ✓; P1-11 ✓; P1-12 ✓; P1-13 no change (noted for D-09).

## DECISION rows left open
D-03 (§1.2 ¶5, italic gap, contribution 1, provenance todo); D-05 (§1.4 intro sentence, S3 status).

## [VERIFY]
None new.
