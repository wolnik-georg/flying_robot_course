# Stage 4 — Chapter 3 (System modelling): passes A–F

Revised file: `revised/ch3_system_modelling.tex`. Builds without errors or undefined references in the combined build
(`/tmp/claude-1000/build.sh`, revised Ch3 + original other chapters, 2026-10-14).

## Verdict (5 lines)
1. The chapter's core (rigid body + residual wrench, the measurement equation, the sign derivation) was sound and is kept.
2. Its description of the learned model was wrong in every code-settled detail (architecture, size, gate, output, rate, deployment) and is now corrected to `residual_nn.rs`/`lib.rs`.
3. Two items that touch the argument are left for the author: the configured residual sign (D-01) and the integral-action assumption behind "approximately twice" (D-02), both now visible in the text through an explicit assumption and comments.
4. The student reader gains the specific-force explanation, two illustrative magnitudes, the one-line sign arithmetic, a deep-sets toy and a normalisation example.
5. A fourth property of the measurement (it measures the force, not the controller's success) is added because the rest of the thesis relies on it and it is supported by Neural-Swarm.

## Rubric (0–5), before → after

| Criterion | Before | After | Note |
|---|---|---|---|
| 1 Correctness | 2 | 4 | network facts fixed; sign/integral behind D-01/D-02 |
| 2 Completeness | 3 | 4 | filters, bias, gate consequences, input sources added |
| 3 Depth | 3 | 4 | sign arithmetic, magnitudes, fold example |
| 4 Academic standard | 4 | 4 | |
| 5 Explainability | 3 | 4 | specific force, toy examples |
| 6 Related work / referencing | 4 | 4 | added supporting cites from held papers only |
| 7 Cross-chapter consistency | 2 | 4 | Ch4/Ch5 must follow (P4, P5 rows) |
| 8 Reference correctness | 4 | 5 | all cites re-checked (pass D) |

## A. Claim re-check

### A1. Literature claims

| Location (revised) | Claim | Key | Record | Verdict |
|---|---|---|---|---|
| §3.2 scope | Neural-Swarm compensates the vertical interaction force | shi2020 | R01 §V-B | supported |
| §3.2 scope | Hsieh hardware controller commands thrust and angular rates; torque to rate loop | hsieh2026 | R05 §5.2 | supported |
| §3.2 scope | Kiran and Gielis measure force and moment on a rig | kiran2025, gielis2023 | R11, R03 | supported |
| §3.3 magnitudes | lower of two stacked Crazyflies at 13 cm, constant hover voltage, ≈65 % hover thrust | kiran2025 | R11 §III-A (Δz/l = 4, l = 32.5 mm) | supported |
| §3.3.1 | Cobo-Briesewitz remove the rotor-speed prerequisite | cobo2026 | R07 abstract | supported |
| §3.3.1 | INDI with thrust from motor command tracked worse than uncompensated geometric on CF2.1; authors' reading | cobo2026 | R07 Table 1, §5.4.1 | supported ("PWM" rendered "motor command") |
| §3.3.1 delay | identical filters on measured acceleration and thrust | tal2018, smeur2017 | R13 §III-B; R14 §2–§3 | supported |
| §3.3.1 bias | INDI position loop sensitive to accelerometer bias | smeur2017 | R14 §4.6 | supported |
| §3.3.1 new ¶ | compensating a learned force "will only affect" commanded thrust "and not the observed" force | shi2020 | R01 §III-C (quote) | supported |
| §3.3.2 | NS/NS2 subtract the predicted force; Tal's increment = commanded − gravity − external; Cobo and Hsieh subtract before thrust | shi2020, shi2022, tal2018, cobo2026, hsieh2026 | R01 eq. 9; R02 eq. 18; R13 eqs. 18–20; R07 eq. 8a; R05 eq. 14a | supported |
| §3.4.1 | NS2: one φ per neighbour type, one ρ per own type, environment as a neighbour | shi2022 | R02 §IV-B, §VII-B | supported |
| §3.4.1 | widths 6→25→40→40→H, H→40→40→40→1, H = 20 | shi2022 | R02 §VII-C, Table III | supported |
| §3.4.1 | NS and NS2 apply spectral normalisation | shi2020, shi2022 | R01 §III-B; R02 §IV-C | supported |
| §3.4.2 | with >2 vehicles not a simple superposition | shi2020 | R01 §V-C | supported |
| §3.4.2 | Gielis motivation | gielis2023 | R03 abstract | supported |
| §3.4.2 | 2-trained network worse than baseline at 3–4; "since the training data does not include data on superpositions" | shi2020 | R01 Table I, §V-C | supported |
| §3.4.2 | Hsieh 2025 pairwise summation of a two-vehicle-trained model on three vehicles, citing Gielis | hsieh2025 | R10 §2.1 | supported |
| §3.4.2 | Gielis linear vs nonlinear regimes | gielis2023 | R03 §4 | supported |
| §3.4.3 | NS2 interaction volume "a cuboid based on observed interactions" | shi2022 | R02 §III-B (quote) | supported |
| §3.4.3 | downwash concentrated within about a body radius over long vertical distance (load stand) | gielis2023 | R03 §5 | supported (platform larger; "consistent in kind") |
| §3.4.3 | Gu: RBF crash, ELM unstable, ESN/GP fine; "may be attributed"; quote | gu2025 | R18 §5.3 | supported |
| Tab 3.1 | superposition known imperfect | shi2020, gielis2023 | R01, R03 | supported |
| §3.5 | Cobo LINDI circle OOD, no payload | cobo2026 | R07 §5.4.2 | supported |

### A2. Implementation statements vs code

| Statement | Code | Verdict |
|---|---|---|
| accelerometer reading rotated, scaled by g, minus $ge_3$ | `lib.rs:1790` | ✓ |
| accelerometer Butterworth prefilter | `lib.rs:545, 2374–2382` | ✓ |
| rotor speeds from DShot telemetry or optical deck, selectable; plausibility filter | `traj_iface.c:508–520, 701–730` | ✓ |
| per-motor $k_{t,k}$ in N/RPM² | `traj_iface.c:285–288`; `lib.rs:1782–1785` | ✓ |
| $a_{\mathrm{res}}$ computed every control step when rotor speeds available, every mode; zero otherwise | `lib.rs:1781–1834` | ✓ |
| control 1 kHz; logging 500 Hz | `lib.rs:2350–2351`; `stabilizer.c`; `usd_study_config.txt` | ✓ |
| thrust not filtered by default; both-sides filter is an option | `lib.rs:1801–1828`; `traj_iface.c:493–494` | ✓ |
| neighbour poses from broadcast; velocity by finite differences; ≤3 neighbours | `lib.rs:839–873`; `traj_iface.c:146–166`; `residual_nn.rs:129` | ✓ |
| eq. (model-deepsets): ground branch, gated sum, scalar ρ, g/1000, ÷m, clamp 8, $e_3$ | `residual_nn.rs:262–309, 134, 136` | ✓ |
| layer widths; 19 297 / 11 461 | `residual_nn.rs:91–125` (3675 + 3625 + 4161 = 11 461) | ✓ |
| weights compiled into the image; upload alternative | `Cargo.toml` features; `lib.rs:922–937`; `traj_iface.c:116–214` | ✓ |
| Huber loss on $a_{\mathrm{res},z}$; no SN | `tools/residual/train.py:17, 118`; `model.py` (no SN) | ✓ |
| every 10th step, held; computed whenever weights loaded | `traj_iface.c:132`; `lib.rs:884–903, 1752–1756` | ✓ |
| gate values and asymmetry; no cut-off; no clamp | `residual_nn.rs:36–44, 139–140, 281–286` | ✓ |
| training pipeline applies the same gate | `tools/residual/dataset.py:normalisation_rel` ("gated neighbour observations") | ✓ (statistics over gated rows) |
| fold into first layers of $\phi_S$ and $\phi_G$ separately | `tools/residual/model.py:224–241` | ✓ |
| both terms carry the same sign | `lib.rs:1930–1931` | ✓ (value of the sign: D-01) |

## B. Defect log (new findings while rewriting)

| # | Defect | Sev | Handling |
|---|---|---|---|
| B3-1 | Original §3.3.2 said the sign error is "invisible altogether" in single-vehicle flight "since $f_{\mathrm{res}}\approx 0$", contradicting §3.2 (isolated residual is not zero) | S2 | APPLY (reworded) |
| B3-2 | Original §3.3.1 said "Three properties"; the thesis relies on a fourth (residual independent of compensation at a given state) without stating it | S2 | APPLY (new ¶) + D-04 for the rationale elsewhere |
| B3-3 | "approximately twice" holds only without integral action on that axis — unstated | S2 | APPLY (assumption stated) + D-02 |
| B3-4 | Specific-force sentence ("excludes gravity by construction") misleading for the student | S3 | APPLY |
| B3-5 | Fallback `\squared` inside math breaks the build without siunitx | S3 | fixed (math/units separated) |

## C. Rewrite
See `revised/ch3_system_modelling.tex` (complete, compilable). Markers: `% NEW`, `% CHANGED` (with plan row and
code location), `% REMOVED` (original text preserved as comment), `% DECISION D-01/D-02/D-04/D-06`.

## D. Citation audit

| `\cite` | Key exists | Claim at that place | Location in paper | Context right? |
|---|---|---|---|---|
| shi2020neuralswarm (×6) | ✓ | vertical force; quote §III-C; subtracts; SN; superposition; 2-trained worse | §V-B; §III-C; eq. 9; §III-B; §V-C; Table I | ✓ |
| shi2022neuralswarm2 (×4) | ✓ | subtracts; architecture; SN; cuboid | eq. 18; §VII-C; §IV-C; §III-B | ✓ |
| hsieh2026flatness (×2) | ✓ | HW thrust + rates; subtracts residual | §5.2; eq. 14a | ✓ |
| kiran2025downwash (×2) | ✓ | rig force+moment; 65 % at 13 cm | §II-A; §III-A | ✓ (constant voltage, rig) |
| gielis2023aggregate (×4) | ✓ | rig; motivation; regimes; wake concentration | §3; abstract; §4; §5 | ✓ |
| cobobriesewitz2026lindi (×4) | ✓ | removes RPM; PWM-INDI worse; subtracts; OOD | abstract; Table 1; eq. 8a; §5.4.2 | ✓ |
| tal2018indi (×2) | ✓ | identical LPFs; increment sign | §III-B | ✓ |
| smeur2017cascadedindi (×2) | ✓ | identical filters; bias | §2–§3; §4.6 | ✓ |
| hsieh2025onlineadaptation (×1) | ✓ | pairwise summation citing Gielis | §2.1 | ✓ |
| gu2025comparative (×1) | ✓ | crash, quote | §5.3 | ✓ |

No failed row.

## E. Examiner simulation (12 questions)

| # | Question | Answer in revised text |
|---|---|---|
| 1 | Your residual includes model error; how do you separate it from interaction? | §3.2 ¶2; §3.3 magnitudes; §3.3.1 "absorbs model error"; null scenarios (Ch5) |
| 2 | Why does the accelerometer give $\dot v$ if it measures specific force? | §3.3 specific-force sentences |
| 3 | Eq. (model-ares): which filters, which delays, and are the two sides matched? | §3.3.1 "It is delayed" (+ D-06 for numbers) |
| 4 | Eq. (model-sign): derive it, and what happens with the opposite sign? | §3.3.2 substitution, PD arithmetic, assumption on integral action |
| 5 | Your Strategy 1 configuration — which sign does it fly? | **D-01** (comment; author must answer in text) |
| 6 | Eq. (model-deepsets): why can a sum represent a non-additive aggregate? | §3.4.1 toy example; §3.4.2 |
| 7 | Why a scalar output — what about lateral forces? | §3.4.1 (NS/NS2 found horizontal components small); Tab 3.1 |
| 8 | Your gate excludes neighbours 0.2 m to the side — is that physical? | §3.4.3 consequences; Tab 3.1 row; Gielis |
| 9 | Eq. (model-fold): prove the fold is exact | §3.4.4 equation + numeric example |
| 10 | Is the network you fly the one in the cited paper? | §3.4.1 "What is borrowed and what is ours" |
| 11 | RQ3: a model trained on pairs has never seen a sum of two embeddings — is the question fair? | §3.4.2 (Shi Table I quote) |
| 12 | Limitations: what does this formulation not capture? | §3.2 scope (moment), Tab 3.1 |

## F. Student-reader test

| Stumbling point | Resolution |
|---|---|
| "specific force" | defined with a resting-vehicle example (§3.3) |
| how big is $a_{\mathrm{res}}$? | two illustrative magnitudes (§3.3) |
| why the sign matters numerically | PD arithmetic with numbers (§3.3.2) |
| hat on $\hat a$ vs hat map | convention sentence (§3.1) |
| vee map | defined (§3.1) |
| deep sets / latent space | toy example (§3.4.1) and plain-words paragraph (§3.4.2) |
| gram-equivalent unit | explained with the conversion factor (§3.4.1) |
| what "flash-resident" means | "compiled into the flight image" (§3.4.1) |
| Butterworth delay | formula + value for scale (§3.3.1) |
| what "INDI" needs from filters | identical filters sentence (§3.3.1); full derivation in Ch4 |

## Proposed additional references (UNVERIFIED) for this chapter
Zaheer et al. 2017 (deep sets theorem) — for §3.4.1; Jain et al. 2019 (physics interaction model) — §3.4; see
`proposed_references.md`.

## [VERIFY] for this chapter
- `% VERIFY` none added; numerical values all code-settled or illustrative. D-06: the realised filter cut-off.

## Change log (plan rows)
P3-01 ✓, P3-02 ✓, P3-03 ✓, P3-04 ✓, P3-05 ✓, P3-06 ✓, P3-07 ✓, P3-08 ✓, P3-09 ✓, P3-10 ✓, P3-11 ✓, P3-12 ✓ (number → D-06),
P3-13 ✓, P3-14 ✓, P3-15 ✓, P3-16 ✓, P3-17 → DECISION D-01 (comment), P3-18 ✓, P3-19 ✓, P3-20 ✓, P3-21 ✓, P3-22 (no change
needed), P3-23 ✓, P3-24 ✓.

## DECISION rows left open
D-01 (§3.3.2, §3.4.1 ¶"These equations", §3.6), D-02 (§3.3.2), D-04 (§3.3.1 new ¶), D-06 (§3.3.1).
