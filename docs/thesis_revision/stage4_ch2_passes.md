# Stage 4 — Chapter 2 (Background and related work): passes A–F

Revised file: `revised/ch2_related_work.tex`. Combined build (revised Ch2–Ch5) compiles with no errors, no undefined
references or citations (59 pages).

## Verdict (5 lines)
1. The chapter's strength — numbers tied to what they are numbers of — is kept; nine context corrections from Phase 1 are applied.
2. Two factual errors about Hsieh et al. 2025 are corrected (it is a three-vehicle measured/predicted/hybrid comparison inside MPC); the main gap sentence is not changed (D-03) and is now the chapter's single largest exposure.
3. Background the student reader lacked (rotations and Ψ, what flatness fixes, how INDI forms the residual and why filters must match, spectral normalisation) is added in short paragraphs.
4. An evaluation-practice paragraph makes §2.8's "cannot be composed" argument concrete and links it to Ch5's metric definitions.
5. Terminology clash on "reactive" (Chee et al.) is footnoted.

## Rubric (0–5), before → after

| Criterion | Before | After | Note |
|---|---|---|---|
| 1 Correctness | 3 | 4 | Hsieh 2025, Gielis, NS2, Smeur, Shankar, Cobo fixed; first gap D-03 |
| 2 Completeness | 3 | 4 | Hsieh 2026 SE(3), Bauersfeld demo, Cobo PWM, evaluation practice |
| 3 Depth | 4 | 4 | |
| 4 Academic standard | 4 | 4 | |
| 5 Explainability | 3 | 4 | rotations, flatness, INDI mechanism, SN |
| 6 Related work / referencing | 3 | 4 | held corpus used fully; external additions in proposed_references.md |
| 7 Cross-chapter consistency | 3 | 4 | Ψ/e_R as Ch4; filters as Ch4; D-03 open in Ch1/Ch2 |
| 8 Reference correctness | 3 | 5 | every changed sentence re-checked against R01–R18 |

## A. Claim re-check (changed or new sentences only; unchanged sentences were verified in Phase 1)

| Location | Claim | Key | Record | Verdict |
|---|---|---|---|---|
| §2.1 ¶Rotations | Ψ, $e_R$; coordinate-free avoids singularities | lee2010geometricse3 | R15 eqs. 6, 8; §I | supported (v4) |
| §2.1 flatness | acceleration → thrust/axis; jerk → ω; snap → α, τ | tal2018indi, hsieh2026flatness | R13 §II-B; R05 §3 | supported |
| §2.2 ¶1 | 65 %/90 % at 13 cm; lateral Δx/l≈3, vertical Δz/l≈19 | kiran2025downwash | R11 §III-A | supported (rig) |
| §2.2 ¶1 | downwash concentrated; negligible outside | gielis2023aggregate | R03 §5 | supported |
| §2.2 ¶2 | PIV sheet through front two rotors; 700 g sufferer; Gielis evaluate own models | kiran, gielis | R11 §II-A; R03 §5, §4.4 | corrected |
| §2.2 ¶3 | jet-model feedforward 1 m/2 m, ≈3–5× height RMSE | bauersfeld2024airflow | R12 §VII, Tab. III | supported |
| §2.3 fn | "reactive" in Chee; L1 as online estimator | chee2024, hsieh2025 | R09 §I; R10 §2.2 | supported |
| §2.3 INDI ¶ | increment; identical filters | tal2018indi, smeur2017 | R13 §III-B; R14 §2 | supported |
| §2.3 Smeur | 4 Hz within wind-tunnel experiment | smeur2017 | R14 §5 | corrected |
| §2.3 | PWM-thrust INDI worse than Lee | cobobriesewitz2026lindi | R07 Tab. 1 | supported |
| §2.3 | "without first observing"; observer convergence; NS2 crash remark (caveated) | gielis, shankar, shi2022 | R03 §1; R17 §1; R02 §VII-E | supported |
| §2.4 Shi 2020 | attitude loop open; experiments use Lee | shi2020 | R01 §IV, §V-C | corrected |
| §2.4 NS2 | SN bounds Lipschitz; 3× in abstract; 24 cm in intro/Fig. 1; 15→10 cm upgraded motors | shi2020, shi2022 | R01 eq. 7; R02 abstract, Fig. 1, §VII-E | corrected |
| §2.4 Smith | 56 % abstract/contributions; 55/51 % body; 55 %/49 % pos/vel; leader 2.5 m NED | smith2023 | R04 | corrected |
| §2.4 superposition | "explore"; K = 3 → 4 total; caution beyond K = 3 | gielis | R03 abstract, §6 | corrected |
| §2.4 superposition | Hsieh 2025 cites Gielis for pairwise sum | hsieh2025 | R10 §2.1 | supported |
| §2.5 FF | SE(3) + learned residual −43.7 % RMSE, off-board | hsieh2026 | R05 §5.2 | supported |
| §2.5 FBL | intro "identical hardware setup" vs body "reported" | hsieh2026 | R05 §1.1.1, §5.2 | corrected |
| §2.5 MPC | Hsieh 2025 arms; centre-vehicle result; slower entering/leaving wake; optimiser off-board | hsieh2025, chee2024 | R10 §3–4; R09 §IV-B | corrected |
| §2.5 docking | failure attribution | shankar2023 | R17 §3.3, §4 | corrected |
| §2.7 ¶1, ¶3 | team hybrid only inside off-board MPC | hsieh2025 | R10 | corrected (minimal) |
| §2.7 ¶3 | residual sources as listed by the authors | cobobriesewitz2026lindi | R07 §1 | corrected |
| §2.8 practice ¶ | ablations; 3–20 repeats; achieved separation; splits; sim/PIL; tuning | several | Stage 2 §5 table | supported |
| Tab 2.1 caption, rows | caption claim narrowed; Hsieh 2025, Smith, Gielis, Hsieh 2026 rows | — | R03, R04, R05, R10 | corrected |
| §2.9 second gap | INDI hybrid single vehicle; MPC team instance | cobo, hsieh2025 | R07, R10 | corrected (minimal) |
| §2.9 third gap | two-vehicle-trained summed model flown on three vehicles; penalty not isolated | hsieh2025 | R10 §2.1 | supported |

## B. Defect log (new)

| # | Defect | Sev | Handling |
|---|---|---|---|
| B2-1 | Table 2.1 caption claimed no two rows share platform, team size, separation and trajectory class — Chee 2024 and Hsieh 2026 nearly do | S2 | APPLY: caption narrowed to what holds |
| B2-2 | "Feedforward into a geometric controller" had no geometric-controller instance | S2 | APPLY: Hsieh 2026 SE(3) configuration |
| B2-3 | third gap ignored a deployed summed model on three vehicles | S2 | APPLY (factual sentence) |

## C. Rewrite
`revised/ch2_related_work.tex` (complete). No label removed.

## D. Citation audit

| `\cite` | Exists | Claim (changed/new) | Location | Context |
|---|---|---|---|---|
| lee2010geometricse3 (×3) | ✓ | Ψ, e_R, singularities | eqs. 6, 8, §I | ✓ |
| tal2018indi (×4 new) | ✓ | flatness chain; INDI increment; filters | §II-B, §III-B | ✓ |
| hsieh2026flatness (×2 new) | ✓ | flat maps; SE(3) −43.7 % | §3, §5.2 | ✓ |
| kiran2025downwash (new) | ✓ | 65/90 %; lateral/vertical recovery | §III-A | ✓ rig |
| gielis2023aggregate (×2 new) | ✓ | concentrated downwash; "without first observing" | §5, §1 | ✓ |
| bauersfeld2024airflow (new) | ✓ | FF demo | §VII | ✓ |
| hsieh2025onlineadaptation (×6 new) | ✓ | L1 estimate; arms; pairwise sum; team hybrid; two→three | §2–4 | ✓ |
| chee2024knodedwmpc (×3 new) | ✓ | "reactive" usage; achieved separation | §I, §IV-B | ✓ |
| smeur2017cascadedindi (×2 new) | ✓ | increment; same filter | §2 | ✓ |
| cobobriesewitz2026lindi (×3 new) | ✓ | PWM INDI; held-out trajectories; repeats | Tab. 1, §5 | ✓ |
| shankar2023docking (×2 new) | ✓ | observer convergence; repeats | §1, §3.3 | ✓ |
| shi2022neuralswarm2 (×3 new) | ✓ | crash remark; split; ablation | §VII-C, §VII-E | ✓ caveated |
| shi2020neuralswarm (×2 new) | ✓ | SN; ablation, repeats | §III, §V-C | ✓ |
| li2023ndpnmpc (×3 new) | ✓ | ablation; 3 rounds; shuffled split | §III-B/C | ✓ |
| gu2025comparative (×2 new) | ✓ | processor-in-the-loop; BO tuning | §5.5, §1 | ✓ |
| yang2025flatnessresiduals (new) | ✓ | simulation evidence | §6 | ✓ |
| abro2025fractional | ✓ | unchanged | no PDF | `[VERIFY]` |
| lee2010geometrictracking | ✓ | unchanged | no PDF | `[VERIFY]` CDC numbering |

No failed row.

## E. Examiner simulation

| # | Question | Where answered |
|---|---|---|
| 1 | Hsieh 2025 compares measured, predicted and hybrid on three vehicles — how is your gap new? | **D-03** (comment; recommended wording) |
| 2 | Is INDI really model-free? | §2.3 "narrower than it sounds" |
| 3 | Why must INDI's filters match? | §2.3 mechanism ¶ |
| 4 | Is the Smith 56 % a body result? | §2.4 + footnote |
| 5 | Did NS2 fly 16 robots at 24 cm with a 3× reduction? | §2.4 (separate reports) |
| 6 | Does Gielis show summation fails? | §2.4 regimes; both readings |
| 7 | What evidence that reactive methods are too slow? | §2.3 (Gielis, Shankar, NS2 caveated; Hsieh 2025 wake remark) |
| 8 | Can INDI run without rotor speeds on a Crazyflie? | §2.3 Cobo PWM result |
| 9 | Has anyone fed a learned residual into SE(3) control? | §2.5 FF ¶ |
| 10 | Why not MPC? | §2.5 MPC ¶ (onboard feasibility) |
| 11 | "Reactive" means something else in MPC papers | §2.3 footnote |
| 12 | How do others evaluate? | §2.8 practice ¶ |

## F. Student-reader test

| Stumbling point | Resolution |
|---|---|
| SO(3), hat map, Ψ | ¶ Rotations |
| flat outputs | flatness chain |
| incremental control | mechanism ¶ |
| spectral normalisation | one-sentence definition |
| "reactive" ambiguity | footnote |

## Change log
P2-01 ✓; P2-02 ✓; P2-03 ✓; P2-04 ✓; P2-05 ✓; P2-06 ✓; P2-07 ✓; P2-08 ✓; P2-09 ✓; P2-10 ✓; P2-11 ✓; P2-12 ✓; P2-13 ✓;
P2-14 ✓; P2-15 ✓; P2-16 ✓; P2-17 ✓; P2-18 → D-03 comment; P2-19 → D-03 comments (gap ¶, first gap, todo); P2-20 ✓ (minimal)
+ D-03; P2-21 ✓; P2-22 VERIFY comment; P2-23 ✓; P2-24 ✓; P2-25 ✓; P2-26 ✓; P2-27 ✓; P2-28 ✓.

## DECISION rows left open
D-03 (§2.8 comparisons ¶ opening, gap ¶, §2.9 first gap, provenance todo).

## [VERIFY]
§2.1 CDC 2010 stability regions/numbering; §2.8 Abro paragraph (no PDF).
