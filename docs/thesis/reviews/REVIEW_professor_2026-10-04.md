# Professor review of the thesis draft — 2026-10-04

**Role:** examiner-style review (correctness, rigour, honesty, defensibility). **Status:** read-only review; no `.tex`, `.bib` or existing file was modified, nothing was committed or pushed, no hardware was touched. Ch.1–5 are treated as frozen: findings are reported, not fixed. Results-in-repo are used only where Ch.1–5 contradict them.

Build used for evidence: temp-dir build outside `docs/thesis/` (copy of `docs/thesis` + `docs/references.bib` in `/tmp/tb`), `latexmk -pdf`, exit 0, **50 pages, 0 undefined references, 0 undefined citations**, 8 overfull boxes (one of 186 pt). Source PDFs were converted with `pdftotext -layout` to `/tmp/pdftxt`; the line numbers quoted for papers are line numbers in that extraction.

## Executive summary (top 10 findings)

1. **P-021** (Ch5, MUST-FIX) — The 'gains frozen before the dataset' doctrine is not what the repository shows for the geometric z-gain
2. **P-001** (Ch1, MUST-FIX) — Chapter 1 promises falsifiable hypotheses that no chapter states
3. **P-023** (Ch5, MUST-FIX) — No statistical plan: repeats, test, interval or decision rule are unspecified
4. **P-036** (Ch7, MUST-FIX) — The RQ3 fallback needs the same third vehicle that the main plan needs
5. **P-022** (Ch5, MUST-FIX) — The time-alignment method described in Ch.5 is not the method the merge tool now uses
6. **P-038** (Ch9, MUST-FIX) — The conclusion says Strategies 0–3 'were implemented'; Strategy 3 is not
7. **P-016** (Ch4, SHOULD-FIX) — The training target of the predictive strategy is not defined (raw `a_res` or interaction-only)
8. **P-027** (Ch5, SHOULD-FIX) — Train/test separation is stated, but 'held-out scenarios' are not defined and segments of one flight still share conditions
9. **P-019** (Ch4, SHOULD-FIX) — Strategy 2 status and 'evaluated at the control rate' contradict the latest hardware attempt
10. **P-024** (Ch5, SHOULD-FIX) — 'A1: nothing moves, so any relative displacement is interaction' is contradicted by the z-bias finding

Overall: the prose and the prior-work handling are careful and unusually well qualified (see section F). The risk is not wording but design closure and currency: no statistical plan, hypotheses promised but absent, a fairness doctrine that the repository's own history strains (z-gain change after the training bank), a stale description of time alignment, and an RQ3 fallback that needs the resource that is missing. Ch.6–9 are honest skeletons.

### Scoring (1 = weak, 5 = strong)

| Chapter | Correctness | Rigour | Honesty | Clarity for experts | Completeness | Mean | Comment |
|---|---|---|---|---|---|---|---|
| Ch1 Introduction | 4 | 3 | 5 | 4 | 4 | 4.0 | Argument is clean and calibrated. Promised hypotheses missing; hybrid in gap sentence. |
| Ch2 Related work | 4 | 4 | 5 | 3 | 4 | 4.0 | Numbers carefully qualified. One MISMATCH (7×), one imprecise onboard claim (Li), clipped Table 2.1. |
| Ch3 Modelling | 4 | 4 | 5 | 4 | 4 | 4.2 | Derivations hold. `a_des` clash, `a_meas` wording, unsupported 'moment residual logged'. |
| Ch4 Control architectures | 4 | 3 | 5 | 3 | 3 | 3.6 | Honest implemented/reference split. Variant choice, symbols and status stale; no block diagram. |
| Ch5 Experimental setup | 3 | 2 | 4 | 3 | 3 | 3.0 | Good design logic; no statistical plan, outdated alignment text, stale status, doctrine-vs-history gap, no parameters. |
| Ch6 Results (2-robot) | 3 | 2 | 5 | 3 | 2 | 3.0 | Honest skeleton. Statistics inconsistent; hybrid slot against doctrine. |
| Ch7 Results (multi-robot) | 2 | 2 | 4 | 3 | 2 | 2.6 | Skeleton; the fallback depends on the missing resource. |
| Ch8 Discussion | 3 | 3 | 5 | 3 | 2 | 3.2 | Skeleton; reference types broken. |
| Ch9 Conclusion | 2 | 2 | 4 | 3 | 2 | 2.6 | Skeleton; false 'implemented' statement; mixed future work. |

Scores for Ch.6–9 judge structure and framing only; there are no results to be correct about. Judgement, not measurement.

**Overall readiness.** Ch.1–5 are at 'defensible draft' level *conditional on the six MUST-FIX items* (hypotheses, z-gain/freeze record, time-alignment text, statistical plan, RQ3 fallback, Strategy 3 statement). Without them an examiner will find the same gaps in the first defence questions. Ch.6–9 cannot be assessed beyond structure. The empirical risk (n per cell, third vehicle, Strategy 2 on hardware) is larger than the textual risk.

## Step 0 — Preparation

- **Build:** see header. Undefined refs/cites: **0 / 0**. Overfull boxes: **8** (ch2 ×3 incl. one 186 pt table, ch3 ×1, ch4 ×2, ch5 ×1, ch9 ×1).
- **`\todo` notes per chapter** (grep of `\todo` in the `.tex`): Ch1 1 (provenance, green), Ch2 2 (one is a comment), Ch3 0, Ch4 0, Ch5 2, Ch6 9, Ch7 4, Ch8 8, Ch9 4. Page count: 50.
- **Citation keys:** 20 distinct keys in non-comment `\cite` calls; unresolved against `docs/references.bib`: **none**.

### One-page claims inventory

| Item | Where | Content |
|---|---|---|
| RQ1 | `ch1_introduction.tex` L120 | reactive vs predictive vs uncompensated: tracking accuracy and residual-force rejection |
| RQ2 | `ch1_introduction.tex` L124 | does the advantage depend on regime (separation, relative speed, quasi-static vs transient)? |
| RQ3 | `ch1_introduction.tex` L128 | transfer penalty 2-vehicle-trained → 3-vehicle; predictive strategies only |
| RQ4 | `ch1_introduction.tex` L136 | cost of sensing, compute and training data against performance |
| Contribution 1 | `ch1_introduction.tex` L159 | controlled comparison, same airframe/trajectories/formations, residual logged everywhere, null flights |
| Contribution 2 | `ch1_introduction.tex` L168 | magnitude of the summation penalty on a deployed model |
| Contribution 3 | `ch1_introduction.tex` L173 | cost reported next to performance |
| Compared set | `ch4_control_architectures.tex` L41 | Strategy 0 geometric baseline; 1 pure INDI; 2 geometric + NN; 3 FBL + NN (not implemented) |
| Headline prior-work number | `ch1_introduction.tex` L54 | NS factor 2–4 (9.4/2.4/2.7 cm); NS2 24 cm; Smith 56 % vertical / 36 % 3D |
| Headline prior-work number | `ch2_related_work.tex` L385 | Cobo about 40 % (no payload), 19–24 % (payload); Hsieh 31 % abstract, 24/29 % body |

## A. Problem, gap, contribution

**Verified facts.** The problem statement (measure vs predict) is stated in two ways only and is consistent across Ch.1, Ch.2 §2.9 and Ch.3. The gap is carefully worded with 'to the best of our knowledge' and a provenance note. Contributions are marked as intended, not achieved, and a 'what this thesis does not claim' paragraph exists. Gap vs scope: the gap sentence includes the hybrid; the compared set does not. Hypotheses: absent.

**Judgement.** The contribution (controlled comparison on one airframe) is real but narrow; it survives only if the six MUST-FIX items are closed, in particular the statistics and the freeze record. Related findings: P-002, P-001, P-003, P-020.

## B. Related work and citations

**Checked: 42 claims against the source PDFs in `docs/papers/`** (39 VERIFIED, 2 MISMATCH, 1 UNVERIFIABLE); all 20 cite keys used in Ch.1–9 resolve in `docs/references.bib` (unresolved: none). The ledger path in the task (`docs/20_Verified_Claims.md`) does not exist; the archived copy is stale in places, so the PDFs were used as the primary evidence (WRITING_RULES §3 agrees). Quotes are verbatim from the `pdftotext` extraction; the line numbers are lines of that extraction.

| Claim in the thesis | Thesis location | Source quote | Verdict | Note |
|---|---|---|---|---|
| NS: baseline 9.4 cm vs 2.4 cm (two-vehicle swap) | `ch2_related_work.tex` L184 | “worst-case z-error by a factor of two to four (e.g., 2.4 cm” (`shi2020neuralswarm`, line 353) | **VERIFIED** | prose says 'instead of 9.4 cm for the two vehicle case' |
| NS: 2.7 cm for the two-vehicle-trained network | `ch2_related_work.tex` L186 | “Trained w/ 2 CF 0.027 0.150 0.294” (`shi2020neuralswarm`, line 306) | **VERIFIED** | table value in metres; 4-vehicle-trained row reads 0.024 |
| NS: swap distances 0.20–0.25 m | `ch2_related_work.tex` L431 | “vertical distances of 0.2 m to 0.25 m between neighbors.” (`shi2020neuralswarm`, line 354) | **VERIFIED** |  |
| NS: 60 cm planning gap for Crazyflie 2.0 (cited by NS from earlier work) | `ch2_related_work.tex` L35 | “e.g., 60 cm for” (`shi2020neuralswarm`, line 50) | **VERIFIED** | attributed to earlier planning work, as the thesis says |
| NS: non-superposition for more than two vehicles | `ch3_system_modelling.tex` L245 | “effect is not a simple superposition of each pair” (`shi2020neuralswarm`, line 361) | **VERIFIED** |  |
| NS2: 16 robots, 24 cm minimum vertical distance | `ch2_related_work.tex` L202 | “(minimum vertical distance 24 cm) of heterogeneous aerial teams (16 robots)” (`shi2022neuralswarm2`, line 47) | **VERIFIED** |  |
| NS2: separate 'up to threefold' abstract claim | `ch2_related_work.tex` L203 | “with up to three times reduction in worst-case tracking errors.” (`shi2022neuralswarm2`, line 46) | **VERIFIED** | kept separate from the 16-robot flight, as the locked rule requires |
| NS2: 15 cm → 10 cm, factor about 1.5 | `ch2_related_work.tex` L207 | “motors is still reduced from 15 cm to 10 cm.” (`shi2022neuralswarm2`, line 1292) | **VERIFIED** |  |
| Bauersfeld: jet beyond about 2.5 motor-to-motor distances | `ch2_related_work.tex` L120 | “After 2.5 motor-to-motor distances” (`bauersfeld2024airflow`, line 33) | **VERIFIED** |  |
| Smith: 36 % 3D, 56 % vertical (abstract) | `ch1_introduction.tex` L54 | “36 % on average (and vertical tracking by 56 %).” (`smith2023so2equivariant`, line 24) | **VERIFIED** | source's own intro says 'lateral' for 36 % (see finding on Smith) |
| Smith: 0.154 m → 0.098 m 3D position error | `ch2_related_work.tex` L220 | “positioning error to 0.098 m (from 0.154 m), corresponding” (`smith2023so2equivariant`, line 609) | **VERIFIED** |  |
| Smith: platform 0.26 m span, 0.7 kg, Raspberry Pi | `ch2_related_work.tex` L223 | “span 0.26 m on the longest body-diagonal, and weigh ≈” (`smith2023so2equivariant`, line 579) | **VERIFIED** |  |
| Smith: 50 Hz controller, 45 Hz network | `ch2_related_work.tex` L224 | “iterations are performed at 50 Hz and 45 Hz respectively.” (`smith2023so2equivariant`, line 588) | **VERIFIED** |  |
| Smith: closest training stage about 0.5 m | `ch2_related_work.tex` L225 | “≈ 0.5 m (approx. two body-lengths).” (`smith2023so2equivariant`, line 403) | **VERIFIED** |  |
| Hsieh 2026: 31 % in the abstract | `ch2_related_work.tex` L288 | “our framework reduces average tracking errors by 31% compared to nominal baselines” (`hsieh2026flatness`, line 24) | **VERIFIED** | abstract-only, as the thesis says |
| Hsieh 2026: body 24 % RMSE, 30 % max-z | `ch2_related_work.tex` L288 | “baseline by an average of 24% in RMSE and 30% in” (`hsieh2026flatness`, line 566) | **VERIFIED** |  |
| Hsieh 2026: further 29 % / 43 % for learned FBL | `ch2_related_work.tex` L290 | “additional improvements of 29% in RMSE and 43% in” (`hsieh2026flatness`, line 568) | **VERIFIED** |  |
| Hsieh 2026: 'introduction separately quotes a sevenfold compute reduction' | `ch2_related_work.tex` L296 | “order of magnitude less computation. We” (`hsieh2026flatness`, line 26) | **MISMATCH** | v2 says order of magnitude; no 'seven' anywhere (see P finding) |
| Hsieh 2026: 28 s hardware training data | `ch2_related_work.tex` L297 | “The training dataset D spans 28 seconds (6,720 points)” (`hsieh2026flatness`, line 557) | **VERIFIED** |  |
| Hsieh 2026: controller off-board, 400 Hz Crazyradio | `ch2_related_work.tex` L301 | “the quadrotors at 400 Hz via Crazyradio PA.” (`hsieh2026flatness`, line 524) | **VERIFIED** |  |
| Chee: two Crazyflie 2.1, laptop base station (off-board) | `ch2_related_work.tex` L437 | “the base station and communication between the Crazyflies” (`chee2024knodedwmpc`, line 360) | **VERIFIED** | tight case 0.08 m: 'commanded to be 0.8 body lengths (0.08m)' |
| Hsieh 2025: three Crazyflie 2.1 | `ch2_related_work.tex` L439 | “We conducted physical experiments using three Crazyflie 2.1 quadrotors.” (`hsieh2025onlineadaptation`, line 225) | **VERIFIED** | pairwise 0.2 m not re-checked |
| Cobo: about 40 % without payload | `ch2_related_work.tex` L385 | “by around 40%” (`cobobriesewitz2026lindi`, line 431) | **VERIFIED** |  |
| Cobo: payload reductions 19–24 % | `ch2_related_work.tex` L386 | “Figure8: Lee: 7.06, INDI: 5.38, LINDI: 5.46, NA-INDI: 5.39.” (`cobobriesewitz2026lindi`, line 450) | **VERIFIED** | computed: Figure8 23.8/22.7/23.6 %, Circle (Lee 7.26; 5.91/5.78/5.74) 18.6/20.4/20.9 % |
| Cobo: hybrid margin is small / similar with payload | `ch1_introduction.tex` L80 | “although with a small margin” (`cobobriesewitz2026lindi`, line 435) | **VERIFIED** | payload text: 'remains similar in performance' |
| Cobo: LINDI drop on circle is out of distribution | `ch3_system_modelling.tex` L355 | “likely because the trajectory is out of distribution relative to the training data.” (`cobobriesewitz2026lindi`, line 433) | **VERIFIED** |  |
| Gielis: load-stand 'sufferer', not free flight | `ch2_related_work.tex` L108 | “sufferer fixed to the load-stand (right).” (`gielis2023aggregate`, line 65) | **VERIFIED** |  |
| Gielis: sufferer at hover thrust | `ch2_related_work.tex` L108 | “sets its motor thrust to hover thrust” (`gielis2023aggregate`, line 225) | **VERIFIED** |  |
| Tal: 609 g custom vehicle | `ch2_related_work.tex` L147 | “flying mass is 609 g.” (`tal2018indi`, line 857) | **VERIFIED** |  |
| Tal: rotor speed from optical encoders (Ch.4) | `ch4_control_architectures.tex` L226 | “on measurements from optical encoders attached to the motors.” (`tal2018indi`, line 30) | **VERIFIED** |  |
| Smeur: Bebop, 512 Hz | `ch2_related_work.tex` L140 | “sample frequency of 512 Hz.” (`smeur2017cascadedindi`, line 282) | **VERIFIED** |  |
| Smeur: waypoint every 14 s into a 10 m/s flow | `ch2_related_work.tex` L143 | “waypoints every 14 seconds, one being in the center of” (`smeur2017cascadedindi`, line 619) | **VERIFIED** | 10 m/s flow: 'out of a 10 m/s windtunnel exhaust' |
| Kiran: two Crazyflies on an x–z translation stage with 6-DOF load cells | `ch2_related_work.tex` L103 | “quadrotors mounted on a x − z translation stage.” (`kiran2025downwash`, line 157) | **VERIFIED** | ATI Nano-17 transducers also in text |
| Shankar: vehicle about 0.27 m diagonal | `ch2_related_work.tex` L328 | “hardware. They span ≈ 0.27 m on the longest diagonal, and have a stock mass” (`shankar2023docking`, line 230) | **VERIFIED** | mass 0.7 kg in the next line |
| Gu: no real-quadrotor validation (future work) | `ch2_related_work.tex` L490 | “to validate the approach experimentally on a real quadro-” (`gu2025comparative`, line 909) | **VERIFIED** | abstract also says 'hardware experiments' (Jetson in the loop); reading is defensible |
| Gu: RBFNN instability/crashes, ELM unstable (Ch.3) | `ch3_system_modelling.tex` L278 | “RBFNN encountered instability and crashes during the sim-” (`gu2025comparative`, line 720) | **VERIFIED** | ELM landing instability on the following lines |
| Yang: simulation on a planar 2D quadrotor, no hardware | `ch2_related_work.tex` L281 | “approach is validated in simulation on a 2D quadrotor subject to unmodeled aerodynamic effects.” (`yang2025flatnessresiduals`, line 22) | **VERIFIED** |  |
| Yang: lower-triangular residuals preserve flatness | `ch4_control_architectures.tex` L283 | “that residuals with a lower-triangular structure preserve both the flatness of the system and” (`yang2025flatnessresiduals`, line 17) | **VERIFIED** |  |
| ProxFly: no inter-vehicle communication | `ch2_related_work.tex` L442 | “and does not rely on any communication between quad-” (`zhang2024proxfly`, line 19) | **VERIFIED** | dock 10 cm and release 5 cm also in the PDF: 'less than 5cm, the SQ shuts down' |
| Li: controller ran 'off-board or on a companion computer rather than on the vehicle' | `ch2_related_work.tex` L317 | “both an onboard TX2 NX computer and a desktop PC.” (`li2023ndpnmpc`, line 402) | **MISMATCH** | NDP-NMPC ran on the onboard TX2 NX; see finding |
| Lee: exponential stability for Ψ<1; attractiveness for 1≤Ψ<2 | `ch4_control_architectures.tex` L171 | “Ψ(R(0), Rc (0)) < 1,” (`lee2010geometricse3`, line 371) | **VERIFIED** | arXiv companion: position-tracking mode (Props. 3 and 4) |
| Abro: methods compared, motivation (abstract figure deliberately unused) | `ch2_related_work.tex` L506 | — (no PDF in `docs/papers/`) | **UNVERIFIABLE** | no PDF in docs/papers |

MISMATCH items: P-006, P-007. Related: P-009, P-010, P-008, P-004.

## C. Mathematics and modelling

**10 derivations/checks done by hand** (not by running code).

| Check | Thesis location | What I did | Result |
|---|---|---|---|
| M1  `model-ares` | `ch3_system_modelling.tex` L113 | Rearranged `m v' = -m g e3 + f R e3 + f_res` for `f_res/m`. Result: `v' - (f/m R e3 - g e3)`. | agrees with the printed equation |
| M2  accelerometer to `a_meas` | `ch3_system_modelling.tex` L120 | A specific-force sensor reads `f_spec = R^T (a - g_vec)`, with `g_vec = -g e3`. Hence `a = R f_spec - g e3` (rotate, then subtract `g e3`). | agrees with the text ('subtracting g e3' in the ENU frame with gravity along -e3) |
| M3  sign in the control law `model-sign` | `ch3_system_modelling.tex` L164 | From the translational equation with desired acceleration `a`: `f R e3 = m a + m g e3 - f_res`. | agrees; the residual is subtracted (see the `a_des` finding for the symbol clash with Ch.4) |
| M4  inverted sign gives about twice the displacement | `ch3_system_modelling.tex` L178 | PD loop, constant disturbance `d`: uncompensated steady displacement `d/(m K_p)`; inverted-sign compensation adds `+d` of commanded error, net `2d`. | agrees in the linear steady state only (finding on 'approximately twice') |
| M5  first-layer normalisation folding `model-fold` | `ch3_system_modelling.tex` L298 | `W (x-mu)/sigma + b = (W/sigma) x + (b - (W/sigma) mu)`, with the division applied column-wise to `W`. | agrees; the identity is exact |
| M6  parameter count 987 | `ch3_system_modelling.tex` L225 | phi: 6→16 (112) + 16→16 (272) + 16→8 (136) = 520. rho: 8→16 (144) + 16→16 (272) + 16→3 (51) = 467. Total 987. | agrees |
| M7  attitude error function thresholds | `ch4_control_architectures.tex` L171 | `Psi = (1/2) tr(I - R_d^T R) = 1 - cos(theta)`; `Psi<1` ⇔ `cos(theta)>0` ⇔ `theta<90°`; `Psi<2` ⇔ `theta<180°`. | agrees with the printed angles |
| M8  Lee torque law and `e_R` | `ch4_control_architectures.tex` L158 | Compared with Lee et al. eq. (11) and eq. (8) in the PDF: terms `-k_R e_R - k_Omega e_Omega + Omega x J Omega - J(...)` and `e_R = (1/2)(R_d^T R - R^T R_d)^v`. | agrees; the thesis's reduced form drops the feedforward bracket and adds an integral, and says so |
| M9  consistency of `a_des` across chapters | `ch4_control_architectures.tex` L97 | Substituted Ch.3 eq. (model-sign) into Ch.4 eq. (ctrl-ades). | INCONSISTENT symbol: differs by `g e3 - f_res/m` (finding) |
| M10 informative inputs of a vertical stack | `ch5_experimental_setup.tex` L167 | Count inputs `(dx,dy,dz,dvx,dvy,dvz)` that vary when only z changes. | four of six lack variation; Ch.5 says half and two (finding) |

Verified facts: M1–M3, M5–M8 agree with the printed equations. Judgement: the Ch.3/Ch.4 `a_des` clash (P-011), the `a_meas` wording (P-012), the 'twice' statement (P-014), and input counting (P-032). Not done: no numerical check of the firmware, no re-derivation of the Ch.2 INDI equations (Ch.2 does not print any).

## D. Method and experimental design

Threats an examiner would add or press that are missing or thin: statistics (P-023); freeze doctrine vs history (P-021); A1 as the interaction reference (P-024); alignment (P-022, P-033); train/test separation (P-027); RPM source (P-028); vehicle identity vs role (P-029); training target (P-016); undefined scenario parameters (P-030); C2 as a null (P-031). Data leakage between C.1 and C.4 is partially addressed (segment split, held-out scenarios) but under-specified. Battery drift and the single airframe are honestly listed as unmitigated.

## E. Internal consistency

Cross-chapter contradictions found (each is a finding): Strategy 3 'implemented' in Ch.9 vs 'not implemented' in Ch.4 (P-038); RQ4 content (P-003); `a_des` (P-011); channel count 35 vs 54 (P-026); campaign status (P-025); null set (P-031); half vs two of six inputs (P-032); Ch.8/9 reference types (P-037); 'moment residual measured and logged' (P-013). Consistent and checked: Strategies 0–3 and the 'residual is subtracted' rule across Ch.1, Ch.3, Ch.4, Ch.6; the 987-parameter count; 500 Hz control/log rate; 'No claim rests on simulation' in Ch.1, Ch.5, Ch.8.

## F. Honesty and calibration

**Strong points (verified by reading):** every prior-work number is qualified by axis, platform and abstract-vs-body; the reduced torque law is labelled as reduced and its theory is said not to transfer; Strategy 3 is stated as not implemented in Ch.4; the 'what this thesis does not claim' paragraph; the statement that simulation never validates a method; negative results are declared a result. **Over- or under-claims found:** the hybrid in the gap sentence (P-002); Ch.9 'were implemented' (P-038); 'evaluated onboard at the control rate' (P-019); 'moment residual measured and logged' (P-013); stale status text (P-025, P-005).

## G. Ch.6–9 skeleton structure

Each chapter states what evidence would answer each RQ and what a negative result would mean (Ch.8), and marks every empty slot with a red `\todo`. Problems: statistics inconsistent and unspecified (P-023); hybrid slot (P-035); **RQ3 fallback depends on the missing third vehicle** (P-036); wrong `\ref` types (P-037); conclusion and future-work content (P-038, P-039); repository jargon in the text (P-040). Plausibility of the planned RQ2 analysis: a scenario-class table is planned (Table 6.3), but no test for a 'crossover' is specified.

## H. Presentation

No figures in the whole draft (P-043); Table 2.1 clipped (P-008); stale README (P-041); stale or false header comments (P-005); `K_R` unit clash (P-018). Positive: consistent chapter openers, an outline in Ch.1, a clean hyperlinked contents page.

## I. The 15 hardest defence questions

Tally: **YES 3, PARTLY 7, NO 5.**

**Q1.** Your gap lists reactive, predictive and hybrid families, but your compared set has no hybrid. What exactly do you close?

- (ii) A good answer contains: States the gap as reactive-vs-predictive, says hybrid is literature-only, and states the Cobo result is single-vehicle.
- (iii) Answered in the draft: **PARTLY** — scope stated, headline sentence not aligned (`ch1_introduction.tex` L110; `ch2_related_work.tex` L550)

**Q2.** How do you know a difference between strategies is not a tuning artefact when each has its own gains?

- (ii) A good answer contains: Per-strategy tuning at a gate, freeze, interleaving, a tuning-effort account in RQ4, and what a frozen gain disadvantage would look like.
- (iii) Answered in the draft: **PARTLY** — doctrine is clear; no tuning-effort measure and RQ4 text omits it (`ch4_control_architectures.tex` L323)

**Q3.** How many repeats per cell, and what difference is detectable above your measurement floor?

- (ii) A good answer contains: n per cell, a paired estimator, an interval, a decision rule tied to C1–C3.
- (iii) Answered in the draft: **NO** — no n, test or rule (`ch5_experimental_setup.tex` L272)

**Q4.** What exactly is the regression target of the learned model? Does it learn the controller's own steady-state error?

- (ii) A good answer contains: Defined label (raw vs differenced vs solo-subtracted), treatment of the z-offset, and why the label is not contaminated by the closed loop.
- (iii) Answered in the draft: **NO** — label never stated (`ch4_control_architectures.tex` L244)

**Q5.** Is Strategy 2 evaluated out of sample, or on the scenarios it was trained on?

- (ii) A good answer contains: Named held-out classes, split by flight, separate reporting of in- and out-of-distribution cells.
- (iii) Answered in the draft: **PARTLY** — intent stated, held-out set undefined (`ch5_experimental_setup.tex` L337)

**Q6.** How are logs from two vehicles aligned in time and with what error?

- (ii) A good answer contains: Method actually used, measured lag/RMS per merge, effect on `a_res` pairing.
- (iii) Answered in the draft: **PARTLY** — text describes a superseded method (`ch5_experimental_setup.tex` L217)

**Q7.** Why should I trust `a_res` as an estimate of the interaction force, given the RPM thrust model, filtering delay and model error?

- (ii) A good answer contains: Error budget: RPM-source accuracy, thrust-constant identification, delay, and the null-scenario check.
- (iii) Answered in the draft: **PARTLY** — failure modes listed qualitatively, no numbers; RPM source scatter not in thesis (`ch3_system_modelling.tex` L132)

**Q8.** You have two drones. How do you answer RQ3, and what is your fallback?

- (ii) A good answer contains: Third vehicle plan or an honest two-vehicle proxy with stated limits.
- (iii) Answered in the draft: **NO** — fallback needs the third vehicle too (`ch7_results_multirobot.tex` L71)

**Q9.** Is your geometric baseline a straw man given its steady-state z error?

- (ii) A good answer contains: The z-gain policy, the reason, and the sensitivity of the conclusions to it.
- (iii) Answered in the draft: **PARTLY** — K_i is in the equation, its state is not stated; repo shows the change (`ch4_control_architectures.tex` L89)

**Q10.** Only one predictive family member is implemented and it is your own small network. What does the result say about 'predictive compensation'?

- (ii) A good answer contains: Explicit instantiation-level conclusions; Strategy 3 as scope limitation.
- (iii) Answered in the draft: **PARTLY** — warned once in Ch.4, not carried to RQ wording (`ch4_control_architectures.tex` L265)

**Q11.** Does the network run on the vehicle at the control rate?

- (ii) A good answer contains: Measured execution time, rate used, overload margin, behaviour when it overruns.
- (iii) Answered in the draft: **NO** — first onboard flights crashed; 100 Hz fix untested (`ch4_control_architectures.tex` L242)

**Q12.** The implemented torque law is not Lee's law. What do the cited stability results say about it?

- (ii) A good answer contains: That nothing transfers, and that the baseline is a reference condition.
- (iii) Answered in the draft: **YES** — stated plainly, with the exact differences (`ch4_control_architectures.tex` L167)

**Q13.** What happens if the residual sign is wrong, and how would you notice?

- (ii) A good answer contains: Derivation, expected doubling, a check scenario.
- (iii) Answered in the draft: **YES** — derived and testable (steady-state caveat noted) (`ch3_system_modelling.tex` L172)

**Q14.** One airframe, two vehicles, one lab: what generalises?

- (ii) A good answer contains: Honest limits.
- (iii) Answered in the draft: **YES** — limits stated; vehicle-identity confound missing (`ch5_experimental_setup.tex` L353)

**Q15.** Is vehicle identity separate from role? cf5 is always the bottom drone.

- (ii) A good answer contains: Role-swap scenario (A8), per-vehicle null flights, per-vehicle constants.
- (iii) Answered in the draft: **NO** — not discussed (`ch6_results_2robot.tex` L31)

## J. Per-chapter scoring and readiness

See the table in the executive summary. Readiness statement as given there.

## Findings (ordered by chapter, then severity)

Counts by chapter and severity:

| Chapter | MUST-FIX | SHOULD-FIX | NICE | Total |
|---|---|---|---|---|
| Ch1 | 1 | 3 | 1 | 5 |
| Ch2 | 0 | 3 | 2 | 5 |
| Ch3 | 0 | 3 | 2 | 5 |
| Ch4 | 0 | 5 | 0 | 5 |
| Ch5 | 3 | 10 | 1 | 14 |
| Ch6 | 0 | 1 | 0 | 1 |
| Ch7 | 1 | 0 | 0 | 1 |
| Ch8 | 0 | 1 | 0 | 1 |
| Ch9 | 1 | 1 | 0 | 2 |
| Cross | 0 | 4 | 0 | 4 |
| **Total** | 6 | 31 | 6 | 43 |

### P-001 — Chapter 1 promises falsifiable hypotheses that no chapter states

- **Severity:** MUST-FIX
- **Location:** `docs/thesis/ch1_introduction.tex` L145 (§Research questions `sec:intro-rq`)
- **Quote:** “The questions are stated with falsifiable hypotheses in Chapter~\ref{ch:related} and”
- **Problem:** Ch.1 says the RQs are stated with falsifiable hypotheses in Ch.2 and Ch.5, and that a negative answer is a result. Neither chapter states a hypothesis. No RQ is turned into a prediction with a decision rule, so nothing says what outcome would count as the negative answer the thesis promises to report. This is the first thing an examiner checks against a comparison thesis.
- **Evidence:** A whole-tree search of the nine chapters for `hypothes|falsif` finds only: this Ch.1 sentence, `docs/thesis/ch3_system_modelling.tex` L350: “Two entries deserve emphasis because they are not merely caveats but hypotheses the experiments” (about two table rows, not the RQs), `docs/thesis/ch6_results_2robot.tex` L88: “controllers as part of the primary hypothesis test unless explicitly added to the C.4 matrix.” (refers to a hypothesis never stated), and a comment in Ch.8. The hypotheses exist in the repository: `docs/15_Problem_Statement_and_Research_Questions.md` L114: “**H1.** All compensating strategies reduce tracking error relative to an uncompensated geometric” (H1–H3 are written there, with a 'measured by' line each) but were never lifted into the `.tex`. Note docs/15 still words RQ1 with a hybrid family; the thesis RQ1 does not.
- **Suggested direction:** Add a short 'Hypotheses and decision rules' subsection (Ch.5 is the natural home) with one prediction per RQ, the metric that decides it, and what counts as 'not separated' (for example a difference smaller than the C1–C3 floor or the repeat spread). Re-word H1 so it matches the Strategies 0–3 scope. Point Ch.1 to it. No qualifier needs to be removed anywhere.
- **Confidence:** HIGH — absence established by a complete grep of ch1–ch9; source of the missing text is docs/15

### P-002 — The gap sentence is wider than the gap the thesis closes (hybrid)

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch1_introduction.tex` L94 (§Gap `sec:intro-gap`)
- **Quote:** “\emph{Representative methods exist for reactive compensation, for predictive compensation, and for”
- **Also at:** `docs/thesis/ch1_introduction.tex` L110 (§Gap `sec:intro-gap`): “evaluation of such hybrids is outside this thesis.”
- **Also at:** `docs/thesis/ch2_related_work.tex` L541 (§Summary and gap `sec:rw-gap`): “First, and principally, to the best of our knowledge the reactive, predictive and hybrid families”
- **Problem:** The emphasised gap statement (Ch.1) and the first gap in Ch.2 say that the reactive, predictive and hybrid families have not been compared. The compared set is Strategies 0–3, which contains no hybrid, and the same chapters state that hybrid evaluation is outside scope. A reader (and an examiner) can conclude that the thesis claims to close a gap it explicitly leaves open.
- **Evidence:** The scope limit is stated in the same paragraph (`docs/thesis/ch1_introduction.tex` L110: “evaluation of such hybrids is outside this thesis.”) and in Ch.2 (`docs/thesis/ch2_related_work.tex` L550: “evaluation of a reactive--predictive hybrid is outside the scope of this thesis, as already stated”), so the text is not dishonest, but the headline sentence and the contribution do not match.
- **Suggested direction:** Keep the hybrid in the literature statement (WRITING_RULES: Cobo stays in §2.7) and add one clause to the emphasised sentence, along the lines of 'of which this thesis compares the first two'. Do not delete §2.7 and do not add a Strategy 4.
- **Confidence:** HIGH — three quoted lines; wording issue only

### P-003 — RQ4 does not list tuning effort, but Ch.4 and Ch.5 put tuning effort inside RQ4

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch1_introduction.tex` L136 (§Research questions `sec:intro-rq`)
- **Quote:** “\item[RQ4] For each compared strategy, what sensing, onboard compute and training data does it”
- **Also at:** `docs/thesis/ch4_control_architectures.tex` L324 (§What makes the comparison fair, and what it does not guarantee `sec:ctrl-fairness`): “The cost of per-strategy tuning is in scope for RQ4.”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L334 (§Threats to validity `sec:setup-validity`): “Per-strategy tuning effort is in scope for RQ4”
- **Problem:** Per-strategy tuning is the thesis's answer to the fairness objection, and its cost is declared to be part of RQ4 in Ch.4, Ch.5 and (implicitly) Ch.8. RQ4 in Ch.1 names only sensing, onboard compute and training data. The research question and its answer plan disagree.
- **Evidence:** Ch.8 also lists 'tuning time per strategy' under RQ4 (`docs/thesis/ch8_discussion.tex` L74: “upload vs flash-resident weights, tuning time per strategy (Table~\ref{tab:setup-threats}),”). Nothing in Ch.5's protocol says how tuning effort will be measured (hours, iterations, flights).
- **Suggested direction:** Add 'and tuning effort' to RQ4 (and to the contribution-3 sentence), and one line in Ch.5 saying how it is recorded.
- **Confidence:** HIGH — four quoted locations agree on one side, Ch.1 stands alone

### P-004 — Ch.1 uses the 31 % figure without source or 'abstract' qualifier; the 9 cm size has no source

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch1_introduction.tex` L98 (§Gap `sec:intro-gap`)
- **Quote:** “close proximity and a \SI{31}{\percent} improvement measured in a tight formation are not the same”
- **Also at:** `docs/thesis/ch1_introduction.tex` L30 (§Motivation `sec:intro-motivation`): “Crazyflie~2.0 (\SI{9}{\centi\metre} rotor-to-rotor) that conservative planning gap has been taken”
- **Problem:** WRITING_RULES §3 requires every number to carry its source and qualifier (abstract vs body, platform, axis). In Ch.1 the 31 % (Hsieh et al.) appears for the first time in the gap paragraph with no citation and no 'abstract' qualifier, although the locked rule says it is abstract-only (the body reports 24 % / 29 % splits). The 9 cm Crazyflie 2.0 size has no source in Ch.1, Ch.2 or Ch.5. Ch.2 itself handles the 31 % correctly (see evidence), so this is a Ch.1 glue issue only.
- **Evidence:** Ch.2 qualifies it properly: `docs/thesis/ch2_related_work.tex` L287: “Their abstract reports a \SI{31}{\percent} reduction in average tracking error against nominal”. Source: `docs/papers/hsieh2026flatness.pdf` (pdftotext line 24): “our framework reduces average tracking errors by 31% compared to nominal baselines” (abstract). The 36 % in Ch.1 is cited a few lines earlier (`docs/thesis/ch1_introduction.tex` L55: “vertical and \SI{36}{\percent} in three-dimensional tracking error for a pair of”), the 31 % is not.
- **Suggested direction:** Add `\cite{hsieh2026flatness}` and 'in its abstract' at the Ch.1 comparison sentence (or a forward reference to Sec. 2.6); cite or footnote the 9 cm size (vendor data sheet). No number is changed.
- **Confidence:** HIGH — rule text and quoted lines compared directly

### P-005 — Stale header comments in Ch.1 ('Nothing has flown')

- **Severity:** NICE
- **Location:** `docs/thesis/ch1_introduction.tex` L10 (front matter / chapter opener)
- **Quote:** “% 1. No result is claimed. Nothing has flown. The contribution is stated as what the thesis”
- **Problem:** A source-file comment says nothing has flown. Hardware campaigns have run since September 2026. It does not render, but the next editor reads it as current status.
- **Evidence:** Same pattern in ch5 header ('HONESTY NOTE (updated 21 September 2026)'). Campaign state is in `docs/07_Thesis_Progress_Checklist.md` L21: “C.1 collection COMPLETE”.
- **Suggested direction:** Update or delete the stale comment lines when Ch.1 next opens.
- **Confidence:** HIGH — comment text only

### P-006 — Ch.2 attributes a 'sevenfold compute reduction' to Hsieh et al.; the PDF says 'an order of magnitude'

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch2_related_work.tex` L296 (§Injecting the prediction: the controller matters `sec:rw-inject`)
- **Quote:** “residual) is a \emph{simulation} figure; the introduction separately quotes a sevenfold compute”
- **Problem:** The sentence says the introduction of Hsieh et al. (2026) separately quotes a sevenfold compute reduction. I could not find any 'seven' / '7×' claim in the PDF in the repo (arXiv:2607.12275v2). The v2 introduction says an order of magnitude, as does the abstract.
- **Evidence:** Source text: `docs/papers/hsieh2026flatness.pdf` (pdftotext line 26): “computation. We” (abstract) and `docs/papers/hsieh2026flatness.pdf` (pdftotext line 62): “baselines (Chee et al., 2024) while requiring an order of magnitude less computation” (introduction); no occurrence of 'seven', 'sevenfold' or '7x' (grep of the full text). Simulation table: `docs/papers/hsieh2026flatness.pdf` (pdftotext line 590): “NMPC 0.020 / 0.032 27.6” against 0.99 ms for the proposed controller. The bib entry has no version: `docs/references.bib` L61: “eprint = {2607.12275},”. A 'sevenfold' statement may come from an earlier arXiv version (UNVERIFIABLE here).
- **Suggested direction:** Replace with what v2 says, or check v1 and cite the version explicitly. Pin the arXiv version in the ledger and the bib entry (separate pass: bib is not edited on style passes).
- **Confidence:** HIGH — negative grep of the PDF text in the repo; v1 not available

### P-007 — Li et al. are grouped with off-board controllers, but their NMPC ran on an onboard computer

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch2_related_work.tex` L317 (§Injecting the prediction: the controller matters `sec:rw-inject`)
- **Quote:** “off-board or on a companion computer rather than on the vehicle. This injection point is not among”
- **Problem:** Ch.2 says all three NMPC works (Chee, Li, Hsieh 2025) run the optimisation off-board or on a companion computer 'rather than on the vehicle'. For Li et al. the controller ran on a computer carried by the quadrotor. The table calls it 'Jetson'; the PDF says only 'TX2 NX'.
- **Evidence:** `docs/papers/li2023ndpnmpc.pdf` (pdftotext line 402): “both an onboard TX2 NX computer and a desktop PC.” `docs/papers/li2023ndpnmpc.pdf` (pdftotext line 404): “and hover throttle estimator ran on the TX2 NX, while the”: the NDP-NMPC controller ran onboard. 'Jetson' does not occur in the Li text (outside knowledge that TX2 NX is a Jetson module; UNVERIFIABLE from the PDF). Chee and Hsieh 2026 are genuinely laptop base stations (`docs/papers/chee2024knodedwmpc.pdf` (pdftotext line 360): “the base station and communication between the Crazyflies”). The thesis's point (not a Crazyflie-class MCU) survives, but 'rather than on the vehicle' is wrong for Li.
- **Suggested direction:** Say 'off-board or on a companion computer (Li et al.: an onboard TX2 NX)', and write 'TX2 NX' in the table. The claim 'onboard feasibility on a Crazyflie-class platform is open' stays true.
- **Confidence:** HIGH — PDF text read at the quoted lines

### P-008 — Table 2.1 (evaluation conditions) overflows the text width; the last column is clipped on page 16

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch2_related_work.tex` L420 (§How these methods are evaluated `sec:rw-eval`)
- **Quote:** “\caption{Evaluation conditions across the interaction-force literature. Each result is reported”
- **Problem:** The table is 186 pt wider than the line (LaTeX 'Overfull \hbox (186.01447pt too wide)' at lines 425–445). On the rendered page 16 the 'Reported quantity' column header is cut off at the right edge.
- **Evidence:** Build log of a temp-dir build (50 pages, 0 undefined references/citations): `Overfull \hbox (186.01447pt too wide) in paragraph at lines 425--445` while `ch2_related_work.tex` is open. Checked visually on page 16 of the rebuilt PDF.
- **Suggested direction:** Use `tabularx`/`p{}` columns or `\footnotesize` with `\resizebox`, or drop the 'Reported quantity' text into shorter labels. This is the only Overfull larger than 30 pt; the other seven are < 30 pt.
- **Confidence:** HIGH — build log and page image

### P-009 — Smith et al. are internally inconsistent about the 36 %; the writing rule states one reading as fact

- **Severity:** NICE
- **Location:** `docs/thesis/ch2_related_work.tex` L217 (§Predictive compensation: learning the residual from relative state `sec:rw-predictive`)
- **Quote:** “three-dimensional or lateral: we follow the abstract and the translation-result body, which give”
- **Problem:** The thesis (correctly) reports 36 % as 3D tracking and 56 % as vertical, following the abstract and body. The paper's own contribution list says 'lateral tracking errors by 36 %'. WRITING_RULES locks 'Not 36 % lateral' without noting that the source says it once. Ch.2 does note the ambiguity; Ch.1 and the rule do not.
- **Evidence:** Abstract: `docs/papers/smith2023so2equivariant.pdf` (pdftotext line 24): “36 % on average (and vertical tracking by 56 %).”. Intro contribution 3: `docs/papers/smith2023so2equivariant.pdf` (pdftotext line 115): “errors by 56 % and lateral tracking errors by 36 %.”. Body: `docs/papers/smith2023so2equivariant.pdf` (pdftotext line 609): “positioning error to 0.098 m (from 0.154 m), corresponding” (3D). The thesis's reading is supported by two of three statements.
- **Suggested direction:** Add a short footnote at the Ch.1 mention (as Ch.2 already does) and extend the WRITING_RULES line with 'the paper's own intro says lateral; abstract and body say 3D'. The rule itself is not wrong.
- **Confidence:** HIGH — three PDF lines read

### P-010 — Abro et al. cannot be verified from the repository

- **Severity:** NICE
- **Location:** `docs/thesis/ch2_related_work.tex` L506 (§How these methods are evaluated `sec:rw-eval`)
- **Quote:** “for that motivation; the abstract's quantitative improvement figure and its claim about larger”
- **Problem:** No PDF for Abro et al. (2025) exists in `docs/papers/`. The thesis already limits itself to the methods compared and the motivation, which is the safe use.
- **Evidence:** `ls docs/papers | grep -i abro` returns nothing. The archived ledger says the full text was consulted but the PDF is not kept: `docs/_archive/citation_notes/20_Verified_Claims.md` L10: “The Abro PDF is not in”. Claim status: UNVERIFIABLE by me.
- **Suggested direction:** Add the PDF to `docs/papers/` (or record the DOI plus the pages used) so the next audit can re-check.
- **Confidence:** HIGH — file absence

### P-011 — `a_des` has two different meanings in Ch.3 and Ch.4

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch3_system_modelling.tex` L161 (§Measuring the residual `sec:model-measure` › §Sign convention in the control law)
- **Quote:** “flight. From \eqref{eq:model-trans}, holding a desired acceleration $a_{\mathrm{des}}$ requires”
- **Also at:** `docs/thesis/ch4_control_architectures.tex` L97 (§The position and attitude loop `sec:ctrl-common` › §Position loop): “in the horizontal and vertical axes. The commanded thrust vector is $m\,a_{\mathrm{des}}$; its”
- **Problem:** In Ch.3 eq. (model-sign), `a_des` is the acceleration to be held and the thrust vector is `m a_des + m g e3 - f_res`, so `a_des` excludes gravity and the residual. In Ch.4 eq. (ctrl-ades), `a_des` already contains `+ g e3`, `- a_res` and `- \hat a_res`, and 'the commanded thrust vector is m a_des'. The same symbol differs by `g e3 - f_res/m`.
- **Evidence:** Re-derived from eq. (model-trans): hold acceleration a requires f R e3 = m a + m g e3 - f_res (Ch.3, correct). Ch.4 assembles the whole bracket into one symbol. Both are correct in isolation; together they are inconsistent notation. Ch.3 even says the vector 'passed to the thrust-and-attitude allocation carries -f_res/m', i.e. the passed vector is not a_des.
- **Suggested direction:** Rename the Ch.4 quantity (for example `a_cmd`, 'commanded acceleration including gravity compensation') and say once that `a_des` of Ch.3 is the trajectory-demanded part.
- **Confidence:** HIGH — algebra checked line by line

### P-012 — Ch.3 says `a_meas` comes from the accelerometer and, a page later, that it is differentiated

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch3_system_modelling.tex` L146 (§Measuring the residual `sec:model-measure` › §What this measurement requires, and how it fails)
- **Quote:** “\paragraph{It is delayed.} $a_{\mathrm{meas}}$ is differentiated from noisy inertial measurement”
- **Also at:** `docs/thesis/ch3_system_modelling.tex` L119 (§Measuring the residual `sec:model-measure`): “Both terms are available onboard. The measured acceleration comes from the accelerometer, which”
- **Problem:** Eq. (model-ares) is introduced with `a_meas` = rotated accelerometer minus gravity (no differentiation). The delay paragraph then says `a_meas` is differentiated. Differentiation is true for the angular acceleration (Ch.4: gyro is differentiated), not for translational `a_meas`. An expert reads this as a conceptual error.
- **Evidence:** Ch.4 does it correctly for the rotational channel: `docs/thesis/ch4_control_architectures.tex` L200: “differentiating the gyro signal and low-pass filtering it, and the torque currently being applied,”.
- **Suggested direction:** Change to 'is filtered from a noisy accelerometer signal' for the force channel and keep 'differentiated' for angular acceleration.
- **Confidence:** HIGH — both lines quoted; code not re-read

### P-013 — The moment residual is said to be 'measured and logged', but no such channel exists

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch3_system_modelling.tex` L89 (§Rigid-body model with a residual wrench `sec:model-dynamics`)
- **Quote:** “$\tau_{\mathrm{res}}$ is measured and logged, but no strategy compensates it separately: the”
- **Also at:** `docs/thesis/ch1_introduction.tex` L222 (§Scope `sec:intro-scope`): “The force residual & The moment residual, which is measured but not separately compensated \\”
- **Problem:** Ch.1 (scope table) and Ch.3 state that the moment residual is measured and logged. The logged channel list has torque and angular acceleration but no moment-residual quantity, and I found none in the firmware sources.
- **Evidence:** Search `tau_res|torque_res|res_torque|alpha_res|alp_res` in `*.rs`, `*.c`, `*.h` of the repository: 0 hits. The 54 channels in `flying_drone_stack/tools/usd_thesis_config.txt` L26: “indi.tau_x” are the INDI torque command, not a residual. Ch.5's recorded-channel paragraph also lists no moment residual: `docs/thesis/ch5_experimental_setup.tex` L197: “the INDI torque and filtered angular acceleration; the network prediction”. It could be reconstructed offline from logged alpha and torque (not checked).
- **Suggested direction:** Either say 'could be reconstructed from the logged torque and angular acceleration' or add the channel; delete 'measured and logged' until true.
- **Confidence:** MEDIUM — grep is negative but offline reconstruction or a different source file is possible

### P-014 — 'Approximately twice' for an inverted sign holds only in a linear, unsaturated steady state

- **Severity:** NICE
- **Location:** `docs/thesis/ch3_system_modelling.tex` L178 (§Measuring the residual `sec:model-measure` › §Sign convention in the control law)
- **Quote:** “the sign inverted, the displacement must be approximately twice the uncompensated value.”
- **Problem:** This is offered as a check the results chapter can run. Re-derivation: with a PD loop and a constant disturbance d, uncompensated steady displacement is d/(m K_p); with the inverted term the net disturbance is 2d, so the displacement doubles. That is the steady-state, linear case. During transients, with thrust saturation, or with filter delay in the measured-residual path, 'twice' is not guaranteed.
- **Evidence:** Derivation done in this review (see section C, row M4); no data involved.
- **Suggested direction:** State the condition ('in a quasi-static stack, for a linear regime') or phrase it as 'larger, approximately twice in steady state'.
- **Confidence:** HIGH — elementary algebra

### P-015 — The 2 m neighbour cut-off is justified by an unsupported statement about the noise floor

- **Severity:** NICE
- **Location:** `docs/thesis/ch3_system_modelling.tex` L265 (§Predicting the residual from relative state `sec:model-deepsets` › §Input conditioning)
- **Quote:** “interaction is below the noise floor of the measurement, and including such neighbours would ask”
- **Problem:** The justification for excluding neighbours beyond 2 m is that their interaction is below the measurement noise floor. No number, measurement or citation is given.
- **Evidence:** The tracked volume is about 2 m x 4 m (`docs/thesis/ch5_experimental_setup.tex` L71: “The tracked volume measures approximately \SI{2}{\metre} $\times$ \SI{4}{\metre} horizontally and”), so a 2 m cut-off is nearly always inactive in a two-vehicle flight; the claim is therefore low-stakes. Far-field measurements exist in the literature (Kiran et al., PIV) but are not tied to the 2 m figure here.
- **Suggested direction:** Cite, or say 'chosen conservatively' and note that C1–C3 check it.
- **Confidence:** MEDIUM — no number to check against

### P-016 — The training target of the predictive strategy is not defined (raw `a_res` or interaction-only)

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L244 (§Strategy 2: geometric with a learned residual `sec:ctrl-s2`)
- **Quote:** “Weights are trained offline on data collected under Strategy~0”
- **Also at:** `docs/thesis/ch3_system_modelling.tex` L82 (§Rigid-body model with a residual wrench `sec:model-dynamics`): “by \emph{differencing}, comparing the residual measured in formation against the residual”
- **Problem:** Ch.3 says the interaction can only be isolated by differencing against a solo reference. Ch.4/Ch.5 say the network is trained on data from Strategy 0. Nowhere is the label fixed: raw measured `a_res` (which contains the single-vehicle bias, thrust-scale error, and the controller's own z offset) or `a_res` minus a solo-flight reference for the same trajectory. This decides what the network learns and whether Strategy 2 can double-count a bias that Strategy 0 also has.
- **Evidence:** grep of Ch.4/Ch.5 for `differenc|solo|isolated|label|target` finds no definition (only LaTeX `\label`). Ch.5's collection subsection says what to fly (`docs/thesis/ch5_experimental_setup.tex` L253: “Collection flights are flown under the \emph{uncompensated} baseline of”) but not what is regressed. Repo evidence that the issue is real: all 27 merged C.1 flights show a negative mean z error (`docs/50_Position_Integral_Z_Bias_Investigation.md` L37: “**Sign:** **27/27** flights have mean Z error”).
- **Suggested direction:** State the label explicitly in Ch.4 §Strategy 2 and in Ch.5 §Data collection, including whether C5 / solo flights are used as the reference.
- **Confidence:** MEDIUM — absence by grep; the implementation may define it in code not read here

### P-017 — Strategy 1 is described as one controller; the repo has three INDI variants and no choice yet

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L188 (§Strategy 1: pure INDI `sec:ctrl-s1`)
- **Quote:** “\paragraph{Implemented.} The measured residual of \eqref{eq:model-ares} enters the position loop as”
- **Problem:** Ch.4 describes a single 'pure INDI'. The latest meeting notes compare three INDI variants with no winner and defer the choice. 'Tune on the hardware gate, then freeze' presupposes a chosen variant; which one is Strategy 1 determines the scenario classes in which the comparison favours reactive control.
- **Evidence:** `docs/meetings/2026-10-05.md` L143: “Which indi variant shall we use for comparative study?” / `docs/meetings/2026-10-05.md` L145: “Only scenarios with temporary downwash affecting drone george's indi seems better suited.” / `docs/meetings/2026-10-05.md` L146: “If scenarios with constant downwash affect shall be included omar's indi is better again.”. The variant choice therefore interacts with RQ2.
- **Suggested direction:** Record the selection rule and its date in Ch.4 §Strategy 1 once decided; until then add an explicit 'variant to be fixed before the gate' sentence and avoid describing the INDI result as family-level.
- **Confidence:** HIGH — meeting notes quoted; thesis text read

### P-018 — Symbols in the attitude loop: `omega_des` is undefined and `K_R`, `K_omega` change units between two equations

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L113 (§The position and attitude loop `sec:ctrl-common` › §Attitude loop)
- **Quote:** “$e_\omega = \omega - \omega_{\mathrm{des}}$. In the geometric configuration the torque is”
- **Also at:** `docs/thesis/ch4_control_architectures.tex` L194 (§Strategy 1: pure INDI `sec:ctrl-s1`): “\alpha_{\mathrm{ref}} = \alpha_{\mathrm{des}} - K_R e_R - K_\omega e_\omega,”
- **Problem:** `omega_des` appears once, undefined (Lee's definition is `e_Omega = Omega - R^T R_d Omega_d`, which the chapter prints later with different symbols). `K_R, K_omega` multiply `e_R, e_omega` in the torque law (N m) and again in the angular-acceleration law (rad/s^2), so they are two different gain sets under one name. Source-law symbols (Omega, R_d) vs implementation symbols (omega, R_des) are never mapped.
- **Evidence:** Torque law: `docs/thesis/ch4_control_architectures.tex` L116: “\tau = -K_R e_R - K_\omega e_\omega + \omega \times J\omega”; source: `docs/thesis/ch4_control_architectures.tex` L163: “with $e_\Omega = \Omega - R^\top R_d \Omega_d$.”
- **Suggested direction:** Define `omega_des` (the body-rate target from the planned trajectory), give the two gain sets different subscripts (`K_{R}^{\tau}`, `K_{R}^{\alpha}`) or note the units, add a one-line symbol map for the Lee equation.
- **Confidence:** HIGH — equations compared directly

### P-019 — Strategy 2 status and 'evaluated at the control rate' contradict the latest hardware attempt

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L242 (§Strategy 2: geometric with a learned residual `sec:ctrl-s2`)
- **Quote:** “\eqref{eq:ctrl-ades}. The network is evaluated onboard at the control rate from the relative states”
- **Also at:** `docs/thesis/ch4_control_architectures.tex` L57 (front matter / chapter opener): “2 & Geometric + NN & predicted & $a_{\mathrm{des}}$ & no & implemented, unflown \\”
- **Also at:** `docs/thesis/ch3_system_modelling.tex` L225 (§Predicting the residual from relative state `sec:model-deepsets` › §Deep-sets parameterisation): “This is \num{987} parameters, evaluated onboard within the control period. The sizing is discussed”
- **Problem:** Ch.4 says the network runs at the control rate and Strategy 2 is 'implemented, unflown'. The 5 October notes record a first onboard attempt that crashed every time, a suspected overload from running the network at 1 kHz, and a proposed 100 Hz network rate that is untested. If 100 Hz is adopted, 'at the control rate' (500 Hz) is false, and the status is 'flown, crashed, under test'. The 'within the control period' claim (Ch.3) is unproven.
- **Evidence:** `docs/meetings/2026-10-05.md` L134: “bottom drone with ns2 controller crashed each time.” `docs/meetings/2026-10-05.md` L136: “Likely cause: the network ran at 1 kHz → overloads the processor leading to the crash.” `docs/meetings/2026-10-05.md` L137: “Fix: rate reduced to 100 Hz, tested in simulation.” `docs/meetings/2026-10-05.md` L138: “Next (lab): test on the drone whether 100 Hz fixes the crashes.”. Control loop rate in Ch.5: `docs/thesis/ch5_experimental_setup.tex` L65: “The control loop runs at \SI{500}{\hertz} onboard.”
- **Suggested direction:** Reword to 'at a network rate of N Hz, interpolated/held between evaluations' after the lab test; update the status cell; add the network-rate to the list of frozen settings for Strategy 2.
- **Confidence:** HIGH — repo evidence quoted; outcome of the 100 Hz test not yet known

### P-020 — Each family is represented by one home-made instantiation; the generalisation sentence is missing

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L265 (§Strategy 2: geometric with a learned residual `sec:ctrl-s2`)
- **Quote:** “representative of the predictive family, and should be read as such rather than as an evaluation of”
- **Problem:** RQ1/RQ2 are phrased about 'reactive vs predictive compensation'. The compared predictive member is a 987-parameter network of the thesis's own sizing on a geometric controller; the second predictive member (FBL + flatness-preserving residual) is not implemented, and the reactive member is one of three INDI variants. Ch.4 warns the reader once, but Ch.1 RQ wording, Ch.6 summary table and Ch.9 headline slots do not carry that limit.
- **Evidence:** `docs/thesis/ch4_control_architectures.tex` L260: “Neural-Swarm2 applies spectral normalisation to obtain stability” / `docs/thesis/ch4_control_architectures.tex` L274: “\textbf{Not implemented.} The controller code was requested from the”.
- **Suggested direction:** Add one sentence to Ch.1 scope and to the Ch.8/9 placeholders: conclusions are about these instantiations of the families, and say so in the RQ2 'rule' (Ch.1 line on RQ2).
- **Confidence:** MEDIUM — judgement about how far the conclusions will be read

### P-021 — The 'gains frozen before the dataset' doctrine is not what the repository shows for the geometric z-gain

- **Severity:** MUST-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L242 (§Protocol `sec:setup-protocol` › §Hardware gate)
- **Quote:** “Flying two vehicles before the freeze means the gains move underneath the dataset.”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L367 (§Summary `sec:setup-summary`): “gains are frozen before the comparison and training-collection flights; strategies are selected at”
- **Also at:** `docs/thesis/ch4_control_architectures.tex` L89 (§The position and attitude loop `sec:ctrl-common` › §Position loop): “+ K_p e_p + K_v e_v + K_i \!\int\! e_p \,\mathrm{d}t”
- **Problem:** Ch.5 states that tuning happens once, at the hardware gate, and that moving gains after the dataset exists is the failure mode to avoid. The repository shows: (1) C.1 training data were collected with the geometric controller without z-integral action and all 27 merged flights had a systematic negative z error (mean −7.1 cm); (2) afterwards a z-integral gain (`ki_z=16`) was introduced, reducing steady z error to about 2 cm; (3) which z-gain Strategy 0 uses in C.4 is still being A/B-tested. So either the training bank was collected under a Strategy 0 that differs from the one evaluated, or the evaluation Strategy 0 deliberately keeps the old gain. The thesis states neither, and Ch.4's eq. (ctrl-ades) contains a `K_i` term without saying whether it is on.
- **Evidence:** `docs/50_Position_Integral_Z_Bias_Investigation.md` L37: “**Sign:** **27/27** flights have mean Z error”; `docs/50_Position_Integral_Z_Bias_Investigation.md` L17: “enable** position integral on hardware — bias reads as formation/downwash physics, not an integral” (status 2026-09-27). `docs/07_Thesis_Progress_Checklist.md` L84: “**Geometric + Z-integral (`ki_z=16`):** steady z within ≈2 cm” and `docs/meetings/2026-10-05.md` L47: “**Goal:** Show that the new Z-integral gain value improves the z-error (before around 10cm - now <2cm).”; `docs/07_Thesis_Progress_Checklist.md` L170: “the Z-integral (`ki_z=16`) was flown repeatedly under plain geometric on both `cf5` and” with the tumble history in the same file. Ch.5 itself asks for deviations to be recorded: `docs/thesis/ch5_experimental_setup.tex` L372: “Record any deviation from a per-strategy gain block, frozen after the hardware gate, or from scenario parameters in this chapter.”
- **Suggested direction:** Decide and write down which z-gain defines Strategy 0 for C.4 and for the training bank. If C.4 uses `ki_z=16`, either re-collect the affected C.1 cells or document the mismatch as a deviation and show (offline) that the network's inputs and labels are insensitive to it. Add the deviation record Ch.5 asks for.
- **Confidence:** MEDIUM — repo facts quoted; whether the evaluation baseline will use ki_z=16 is undecided in the repo

### P-022 — The time-alignment method described in Ch.5 is not the method the merge tool now uses

- **Severity:** MUST-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L217 (§Measurement and logging `sec:setup-logging` › §Time alignment across vehicles)
- **Quote:** “The merge tool therefore \emph{measures} the offset by cross-correlating each vehicle's onboard”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L340 (§Threats to validity `sec:setup-validity`): “Residual-time alignment across vehicles depends on merge quality; \texttt{merge\_usd\_logs.py} aborts above”
- **Problem:** Ch.5 §Time alignment says offsets are measured by cross-correlating each vehicle's onboard trace with a radio trace, and that the predicted alignment is a few milliseconds, to be confirmed. The merge tool's own comments say the cross-drone z-correlation is unreliable for near-constant-altitude scenarios, and that the preferred method aligns each drone against its own commanded trajectory (full 3D position fit). The same chapter's threat table describes yet a third thing (a 15 cm RMS 'role check'). Time alignment feeds `a_res` pairing and every separation-error metric, so an examiner will ask exactly this.
- **Evidence:** `flying_drone_stack/tools/merge_usd_logs.py` L395: “the cross-drone z-correlation below is unreliable for any scenario that” (`flying_drone_stack/tools/merge_usd_logs.py` L397: “+282 ms at corr 0.39 on a flight whose true cross-drone offset was 30 ms.”). `flying_drone_stack/tools/merge_usd_logs.py` L398: “each drone against its own commanded trajectory instead is unambiguous (<2 cm RMS on”. Quality gate: `flying_drone_stack/tools/merge_usd_logs.py` L438: “refusing to merge on an alignment this poor -- fix the” at RMS ≥ 15 cm.
- **Suggested direction:** Rewrite §Time alignment to describe the method actually used (per-drone fit to the commanded trajectory, run-tag pairing), report the measured lag/RMS statistics from the banked merges instead of 'a few milliseconds, to be confirmed', and state the uncertainty carried into the metrics.
- **Confidence:** HIGH — tool source and thesis text both read

### P-023 — No statistical plan: repeats, test, interval or decision rule are unspecified

- **Severity:** MUST-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L272 (§Protocol `sec:setup-protocol` › §Comparison campaign)
- **Quote:** “Each condition is repeated to allow a distribution rather than a single number to be reported.”
- **Problem:** The comparison thesis rests on differences between strategies per scenario, but the design gives no number of repeats, no primary metric per RQ, no test or interval, no minimum detectable difference relative to the C1–C3 floor, and no rule for excluded or aborted flights. Ch.4 calls the design 'paired' through interleaving, but no paired analysis is specified. Ch.6 prescribes 'mean +- std' in the table and 'SEM' in the figure.
- **Evidence:** grep of ch1–ch9 for `significan|confidence interval|bootstrap|Wilcoxon|t-test|p-value|effect size|power`: no hit. Only generic repeat language: `docs/thesis/ch6_results_2robot.tex` L40: “(scenario, strategy) cell is intended to be repeated $n$ times under interleaved runtime controller”. Inconsistent summary statistic: `docs/thesis/ch6_results_2robot.tex` L61: “the comparison matrix. Statistics: mean $\pm$ std across repeats.” vs `docs/thesis/ch6_results_2robot.tex` L79: “error bars; source \texttt{plot\_comparison.py} on phase CSV. One panel per metric family (position”. Ch.4: `docs/thesis/ch4_control_architectures.tex` L139: “be interleaved within a single session, which is what makes the comparison a paired one.”
- **Suggested direction:** Add to Ch.5 §Metrics: n per cell (and the budget check against the remaining flight days), the primary metric for each RQ, the paired estimator (per-session difference to Strategy 0), a bootstrap or exact interval, the rule for 'separate' (e.g. interval excludes zero and exceeds the floor from C1–C3), and the aborted-flight policy. Pick one dispersion statistic for Ch.6.
- **Confidence:** HIGH — absence by grep; budget check not done

### P-024 — 'A1: nothing moves, so any relative displacement is interaction' is contradicted by the z-bias finding

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L116 (§The formation library `sec:setup-scenarios`)
- **Quote:** “A1 & Vertical stack, hover & Baseline. Nothing moves, so any relative displacement is interaction rather than tracking error \\”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L291 (§Protocol `sec:setup-protocol` › §Metrics): “against the commanded separation. Downwash acts as a bias, the lower vehicle sags, closing the”
- **Problem:** In A1 the commanded setpoint is constant, but the controller can still have steady-state error. The repository shows a consistent z offset on every merged C.1 flight and, after a z-integral was added, a much smaller one on hover and A1. So a relative displacement in A1 is interaction plus controller steady-state error plus model error (thrust scale). The metric text then reads the mean separation error as downwash ('the lower vehicle sags').
- **Evidence:** `docs/50_Position_Integral_Z_Bias_Investigation.md` L37: “**Sign:** **27/27** flights have mean Z error”; `docs/07_Thesis_Progress_Checklist.md` L84: “**Geometric + Z-integral (`ki_z=16`):** steady z within ≈2 cm” (hover and A1 top within a few cm). Ch.3 itself says the residual is a catch-all: `docs/thesis/ch3_system_modelling.tex` L81: “the nominal model gets wrong about a single vehicle. Second, the interaction can only be isolated”.
- **Suggested direction:** Phrase A1 as 'the reference against which interaction is differenced (with a solo reference flight)', and state that the mean separation error is interpreted after subtracting the solo/controller offset. Do not delete the 'bias, lower vehicle sags' mechanism; add the reference.
- **Confidence:** MEDIUM — interpretation of two repo documents; no re-analysis of the logs

### P-025 — Campaign status in Ch.5 is stale relative to the checklist (C.1 'has begun' / 'partly filled')

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L37 (front matter / chapter opener)
- **Quote:** “Residual-model training data collection (C.1) \emph{has begun} under”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L39 (front matter / chapter opener): “but the planned dz/speed grid is only partly filled (see \texttt{docs/25} coverage table).”
- **Problem:** Ch.5 describes C.1 as begun and partly filled, citing a repo document. The progress checklist now says C.1 collection is complete (28 Sep). The 28 Sep meeting prep, however, says A4 x4 still remained. The thesis cannot cite either as current, and the repo's own sources disagree.
- **Evidence:** `docs/07_Thesis_Progress_Checklist.md` L21: “C.1 collection COMPLETE” vs `docs/meetings/2026-09-28.md` L9: “Only **A4 ×4** remains.”. The header comment already announces the need: `docs/thesis/ch5_experimental_setup.tex` L11: “when this draft was last edited. Revise tense section-by-section as the campaign completes;”.
- **Suggested direction:** At the next Ch.5 pass, replace the status paragraph by a factual table of what was flown (scenarios, n, dates) taken from one authoritative manifest, in past tense, with no repo-doc pointer. Resolve the A4 question first.
- **Confidence:** MEDIUM — two repo statements conflict; which is final is a project fact I cannot decide

### P-026 — Logged-channel count and the missing `e_R` note are out of date; the logging configuration changed after the first bank

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L189 (§Measurement and logging `sec:setup-logging` › §Why the dataset is recorded onboard)
- **Quote:** “The micro SD deck records \num{35} variables at \SI{500}{\hertz} on each vehicle independently of”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L288 (§Protocol `sec:setup-protocol` › §Metrics): “attitude but not $e_R$ itself; that signal will be added before the comparison campaign.”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L206 (§Measurement and logging `sec:setup-logging` › §What is recorded): “The logging configuration is fixed before data collection and shared by the collection and”
- **Problem:** Ch.5 says 35 variables and that `e_R` will be added later. The current configuration has 54 channels including the four `indi.e_r_*` channels; the 23 Sep configuration already had 47. Ch.5 also says a configuration change between collection and evaluation would make the halves incomparable, yet the configuration grew by 7 channels after the first bank (additive, with the earlier 47 unchanged).
- **Evidence:** Count (lines matching `^name.name$`): current `flying_drone_stack/tools/usd_thesis_config.txt` L54: “indi.e_r_norm” file has 54; `experiments/logs/usd_raw/2026-10-03_THESIS1/config.txt.bak-2026-09-23` has 47, and a diff shows only the seven added lines (`indi.oot5_*` and `usd.runTag`). Whether older C.1 flights were logged with an even older configuration is not checked.
- **Suggested direction:** Update the number and state the policy as 'channels may be added; no channel used by the metrics changes meaning'. List the channel-set per bank date in an appendix.
- **Confidence:** HIGH — files counted and diffed

### P-027 — Train/test separation is stated, but 'held-out scenarios' are not defined and segments of one flight still share conditions

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L337 (§Threats to validity `sec:setup-validity`)
- **Quote:** “Training/test leakage & Held-out scenarios; split by contiguous flight segment, never by sample & --- \\”
- **Also at:** `docs/thesis/ch6_results_2robot.tex` L39 (§Experimental campaign overview `sec:res2-overview`): “A3, A4, A7) together with null coplanar controls (C1--C3) that bound the measurement floor. Each”
- **Problem:** The table says scenarios are held out and the text says data are split by contiguous segment, but nowhere says which scenarios are held out. Ch.6 plans closed-loop C.4 on the same scenario types (A1, A3, A4, A7) as C.1 training, so Strategy 2 is evaluated in-distribution by design; out-of-distribution behaviour (the core of RQ2) is not tied to any named held-out class. Splitting by segment of the same flight still shares battery state, session, bias and mocap calibration; the generalisation claim needs at least a split by flight.
- **Evidence:** `docs/thesis/ch5_experimental_setup.tex` L348: “Splitting the training and test data by contiguous flight segment rather than by sample deserves a” / `docs/thesis/ch6_results_2robot.tex` L180: “Train/test splits by scenario class (Section~\ref{sec:setup-validity}) feed RQ2 directly: a crossover”. The data-collection subsection lists the classes the bank covers but not which are reserved.
- **Suggested direction:** Name the held-out scenario class(es) before C.2 training, split by flight (and ideally by session) for the quoted open-loop error, and say that in-distribution C.4 flights are reported separately from held-out ones.
- **Confidence:** MEDIUM — text read; actual C.2 split lives in analysis code not read here

### P-028 — The rotor-speed source is a first-order threat to `a_res` and INDI, but not in the threat table

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L56 (§Platform `sec:setup-platform`)
- **Quote:** “\paragraph{A rotor-speed source}, optical RPM deck or DShot telemetry. Without it the applied”
- **Problem:** Ch.5 treats the optical RPM deck and DShot telemetry as interchangeable ('or'). The 5 Oct notes report measured differences between them (small bias but a few-percent-level scatter, lag up to a few ms, and a spike filter). Both sources are in the logging configuration, but the thesis does not say which feeds Strategy 1 and `a_res`, nor that its error enters the measured residual. The threat table (§5.7) has no RPM row.
- **Evidence:** `docs/meetings/2026-10-05.md` L125: “Bias −0.11…−0.32 %, RMSE 95–121 RPM (plotted: −0.21 %, 104 RPM).” `docs/meetings/2026-10-05.md` L126: “Range 0.78–0.98, median 0.92 (plotted: 0.92).” (correlation r across 32 motors). Both sources logged: `flying_drone_stack/tools/usd_thesis_config.txt` L43: “motor.m1_rpm” and `flying_drone_stack/tools/usd_thesis_config.txt` L47: “rpm.m1”.
- **Suggested direction:** Add a threat row (RPM-source error and lag, effect on `a_res` and INDI, mitigation: dual-source check, spike filter) and say which source defines the thrust model in each strategy.
- **Confidence:** HIGH — numbers quoted from meeting notes; whether a_res uses deck or DShot not verified in code

### P-029 — Vehicle identity is confounded with role (cf5 always bottom, cf_second always top) and is not in the threats

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch6_results_2robot.tex` L31 (§Experimental campaign overview `sec:res2-overview`)
- **Quote:** “The comparison flights use the brushless pair \textbf{cf5} (study vehicle, bottom role in vertical”
- **Problem:** In vertical stacks the same physical vehicle is always the bottom one. Vehicle-specific offsets (motors, thrust constant, battery, mass) are therefore indistinguishable from role-specific interaction. The null scenarios help only if both vehicles fly them in comparable roles. A8 (vertical swap) swaps sides and can break the confound, but the threat table does not use it.
- **Evidence:** `docs/thesis/ch5_experimental_setup.tex` L30: “the brushless pair \textbf{cf5} (study vehicle) and \textbf{cf\_second} (top). Micro-SD merges”; threat table has 'Single airframe' (platform generalisation) but no 'vehicle identity' row.
- **Suggested direction:** Add the threat and list the mitigation (role-swap in A8, C1 with both orders, airframe constants per vehicle).
- **Confidence:** MEDIUM — design inference; a mitigation may already exist in the lab protocol

### P-030 — The scenario library has no numerical parameters in the thesis (separations, speeds, durations, n)

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L94 (§The formation library `sec:setup-scenarios`)
- **Quote:** “Sixteen scenarios, fixed before any data collection and not modified afterwards. Freezing them is”
- **Problem:** The library is described by purpose only. A reader cannot tell which separations, relative speeds or durations are flown, so the 2.5-l near-field argument (Ch.1) cannot be checked against the actual separations, and the experiments cannot be reproduced from the text.
- **Evidence:** Ch.5 §5.3 contains no `\SI` values apart from the volume and floor. Ch.1 argues that formation flight happens inside 2.5 motor distances: `docs/thesis/ch2_related_work.tex` L120: “beyond about \num{2.5} motor-to-motor distances $l$ below the vehicle, a length” (about 22 cm for a 9 cm vehicle).
- **Suggested direction:** Add a parameter table (per scenario: separations, speed, duration, repeats planned/flown) or an appendix. State which separations fall inside 2.5 l.
- **Confidence:** HIGH — section read in full; parameters may exist in the repo formation docs

### P-031 — C2 is marked a null condition (no interaction expected) but its purpose is the wake

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L132 (§The formation library `sec:setup-scenarios`)
- **Quote:** “C2 & Leader--follower line $\ast$ & Wake rather than wash \\”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L143 (§The formation library `sec:setup-scenarios` › §The null scenarios): “C1--C3, the coplanar controls in which the vehicles fly side by side at equal height, are the most”
- **Problem:** The caption defines the asterisk as 'null conditions in which no inter-vehicle interaction is expected', and §null says C1–C3 are side by side at equal height. C2 is a leader–follower line whose stated purpose is a wake; a follower in a leader's wake is not a no-interaction condition. If C2 stays in the null set, the measurement-floor argument is weakened.
- **Evidence:** `docs/thesis/ch5_experimental_setup.tex` L107: “those marked $\ast$ are null conditions in which no”
- **Suggested direction:** Either remove C2 from the null set (floor = C1, C3) or describe the in-line geometry as one with negligible interaction and justify it.
- **Confidence:** HIGH — three quoted lines contradict each other

### P-032 — 'Half of its six inputs' and 'two of the six inputs' disagree; a vertical stack leaves four of six nearly constant

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L167 (§The formation library `sec:setup-scenarios` › §Regimes, and why the library spans them)
- **Quote:** “trained only on such data has seen no variation in half of its six inputs and is extrapolating the”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L174 (§The formation library `sec:setup-scenarios` › §Regimes, and why the library spans them): “produced training data in which two of the six network inputs never varied, which the training”
- **Problem:** The input is 6-dimensional (relative position 3 + relative velocity 3). Ch.5 says half (3) in the requirement paragraph and two in the simulation anecdote. In a purely vertical stack, dx and dy are constant and dvx, dvy are about zero; only dz and dvz vary. That is four of six (two thirds) with variation lacking.
- **Evidence:** Re-derived from `docs/thesis/ch3_system_modelling.tex` L188: “f_{\mathrm{res},i} \;\approx\; \mathcal{F}\!\left( \{\, p_j - p_i,\; v_j - v_i \,\}_{j \neq i} \right),”. 'Two' is correct only if the degenerate normalisation statistic fired on the two position inputs and not on the (noisy or zero) velocity inputs.
- **Suggested direction:** Check the training-pipeline report and fix the counts so that the sentence and the anecdote agree (probably 'four of six' or 'two, the lateral positions').
- **Confidence:** MEDIUM — counting argument; pipeline report not inspected

### P-033 — Aligning each drone to its commanded trajectory couples the time base to the quantity being evaluated

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L340 (§Threats to validity `sec:setup-validity`)
- **Quote:** “Residual-time alignment across vehicles depends on merge quality; \texttt{merge\_usd\_logs.py} aborts above”
- **Problem:** The preferred alignment minimises the position misfit to the commanded trajectory. Tracking error is also a headline metric, and a quality override exists for cases where the lower vehicle is expected to sag. A constant offset hardly biases a lag estimate, but with hover-like scenarios and sag the lag is poorly constrained, and accepting fits above the 15 cm gate by hand needs a documented rule.
- **Evidence:** `flying_drone_stack/tools/merge_usd_logs.py` L351: “help="do not refuse merge when alignment RMS >= 15 cm (still printed). "” and `flying_drone_stack/tools/merge_usd_logs.py` L353: “where bottom tracking error vs nominal command is expected downwash.”
- **Suggested direction:** Report lag and RMS per merged flight as a table; state the accept rule; show on one dynamic scenario that metrics are insensitive to ± the lag uncertainty.
- **Confidence:** LOW — reasoned risk; no numerical test run

### P-034 — Tracked-volume `\todo` still says 'before the first session'

- **Severity:** NICE
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L86 (§Flight volume `sec:setup-volume`)
- **Quote:** “The volume figures are corroborated from two independent statements but have not been”
- **Problem:** The note asks for a tape measurement before the first session; sessions have taken place. Either measured and the note is stale, or not measured and the figures need a qualifier.
- **Evidence:** Ch.5 also says some scenarios sit within about 10 cm of a wall: `docs/thesis/ch5_experimental_setup.tex` L82: “approximately \SI{10}{\centi\metre} of a wall at their extremes, and the scenario definitions are”.
- **Suggested direction:** Close the todo with the measured values.
- **Confidence:** HIGH — text only

### P-035 — Ch.6 keeps a conditional 'Hybrid (NA-INDI)' results slot that contradicts the locked doctrine

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch6_results_2robot.tex` L142 (§Hybrid (NA-INDI), if in scope `sec:res2-hybrid`)
- **Quote:** “(Strategy~4 in the wider project numbering) is \emph{not} in that set unless explicitly restored to”
- **Also at:** `docs/thesis/ch9_conclusion.tex` L71 (§Future work `sec:conclusion-future`): “\item \textbf{Residual RL strategies (Strategy~5/7).} Explicitly deferred in the project convention:”
- **Problem:** WRITING_RULES: hybrid is literature only, 'not a Strategy section ... do not add Strategy 4 / hybrid as a campaign row'. Ch.6 §6.4 is exactly a campaign row shell with an unnumbered-in-the-thesis 'Strategy 4'. 'Strategy 4' and 'Strategy 5/7' refer to a numbering the thesis never defines.
- **Evidence:** `docs/thesis/ch6_results_2robot.tex` L138: “Hybrid (NA-INDI), if in scope” (section title) and `docs/thesis/WRITING_RULES.md` L101: “- Do not add Strategy 4 / hybrid as a campaign row.”
- **Suggested direction:** Delete §6.4 (and the 'Strategy 4' wording) from Ch.6; in Ch.9 describe the deferred work in words ('hybrid with a measured and a learned term', 'residual reinforcement learning') without project numbers. Keep Ch.2 §2.7.
- **Confidence:** HIGH — rule text and chapter text compared

### P-036 — The RQ3 fallback needs the same third vehicle that the main plan needs

- **Severity:** MUST-FIX
- **Location:** `docs/thesis/ch7_results_multirobot.tex` L71 (§Limitation if three-robot data are incomplete `sec:res3-limitation`)
- **Quote:** “If three-robot comparison flights are not obtained in time, RQ3 is answered from \emph{logged}”
- **Also at:** `docs/thesis/ch5_experimental_setup.tex` L51 (§Platform `sec:setup-platform`): “Two vehicles are the minimum for the protocol. A third is required for the three-robot scenarios,”
- **Problem:** The fallback says that if three-robot comparison flights are missing, RQ3 is answered from logged three-robot data under Strategy 0. Those logs require three vehicles flying anyway. The third vehicle is listed as a not-yet-checked hardware requirement. It also claims support from 'C.1 logs plus offline evaluation', but C.1 logs are two-vehicle. RQ3 is one of the four research questions and contribution 2, so its fallback must work without a third vehicle or the thesis must say RQ3 may be unanswered.
- **Evidence:** `docs/11_Hardware_Readiness_Checklist.md` L156: “- [ ] **3rd CF21BL** — required for B1/B2/B3 and C1–C3; the three-robot scenarios” (unchecked); Ch.5: `docs/thesis/ch5_experimental_setup.tex` L52: “which cannot be run without it.”; Ch.7: `docs/thesis/ch7_results_multirobot.tex` L72: “three-robot data only: open-loop prediction and residual statistics on B1/B2/B3 (or a subset) under”; `docs/thesis/ch7_results_multirobot.tex` L75: “This matches the project note that RQ3 can be resolved from C.1 logs plus”. Note C3 (null, equilateral) also needs three vehicles.
- **Suggested direction:** Decide now: (a) obtain a third brushless vehicle with RPM source and uSD; (b) define a two-vehicle proxy for RQ3 whose limits are stated (what it can and cannot show about overlapping wakes); or (c) re-scope RQ3/contribution 2 as conditional in Ch.1 and Ch.9. Do not leave a fallback that depends on the missing resource.
- **Confidence:** HIGH — text and checklist agree on the dependency; roster status from the checklist only

### P-037 — Six `Chapter~\ref{sec:...}` references render as 'Chapter 7.2', 'Chapter 6.3', 'Chapter 5.7'

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch8_discussion.tex` L61 (§RQ3 --- Two-robot training on three-robot formations `sec:disc-rq3`)
- **Quote:** “Chapter~\ref{sec:res3-transfer} supplies the numbers; here they are interpreted against deployment”
- **Also at:** `docs/thesis/ch9_conclusion.tex` L34 (§Answers to the research questions `sec:conclusion-rq`): “baseline under identical conditions, once Chapter~\ref{sec:res2-tracking} is populated.}”
- **Also at:** `docs/thesis/ch8_discussion.tex` L46 (§RQ2 --- Regime dependence `sec:disc-rq2`): “was designed to isolate these regimes (Section~\ref{sec:res2-summary}); a crossover supports”
- **Problem:** Section labels are referenced with the word 'Chapter', so the PDF reads 'Chapter 7.2' etc. Two more references point at the wrong target: the scenario library is described in Ch.5, not in the Ch.6 summary section.
- **Evidence:** Rendered text of the temp build: 'Chapter 7.2 supplies the numbers', 'Chapter 6.3 and the', 'Chapter 5.7 and Table 5.2 listed threats', 'once Chapter 6.2 is populated', 'referencing Chapter 7.2'. Source lines: ch8 L31, L61, L72, L103; ch9 L34, L40 (`grep -n 'Chapter~\\ref{sec:'`).
- **Suggested direction:** Change `Chapter~\ref{sec:...}` to `Section~\ref{...}`; point the RQ2 reference to `sec:setup-scenarios`.
- **Confidence:** HIGH — build output checked

### P-038 — The conclusion says Strategies 0–3 'were implemented'; Strategy 3 is not

- **Severity:** MUST-FIX
- **Location:** `docs/thesis/ch9_conclusion.tex` L24 (§Summary `sec:conclusion-summary`)
- **Quote:** “were implemented and specified in Chapters~\ref{ch:model}--\ref{ch:control}; the experimental”
- **Also at:** `docs/thesis/ch4_control_architectures.tex` L274 (§Strategy 3: feedback linearisation with a flatness-preserving residual `sec:ctrl-s3`): “\paragraph{Implemented.} \textbf{Not implemented.} The controller code was requested from the”
- **Problem:** A false statement of fact in the thesis's closing summary, contradicting Ch.4 and Table 4.1 (FBL+NN 'not implemented'). Also contradicts the locked doctrine that Strategy 3 is table-only until author code arrives.
- **Evidence:** Ch.4 status table: `docs/thesis/ch4_control_architectures.tex` L58: “3 & FBL + NN & predicted & linearising law & no & not implemented \\”.
- **Suggested direction:** Write 'Strategies 0–2 were implemented; Strategy 3 is specified but unimplemented pending author code'.
- **Confidence:** HIGH — contradiction between two quoted lines

### P-039 — The future-work list mixes core campaign items with extensions and is internally inconsistent

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch9_conclusion.tex` L59 (§Future work `sec:conclusion-future`)
- **Quote:** “Several extensions are already scoped in project documentation and do not depend on the C.4 outcome:”
- **Also at:** `docs/thesis/ch9_conclusion.tex` L74 (§Future work `sec:conclusion-future`): “\item \textbf{Hardware validation of onboard prediction.} SIL and desk parity do not substitute for”
- **Problem:** The list is introduced as extensions that do not depend on C.4, but its first item is 'C.1 coverage and C.4 repeats' (the core campaign) and one item is hardware validation of onboard prediction, which Strategy 2 in the primary comparison requires.
- **Evidence:** `docs/thesis/ch9_conclusion.tex` L62: “\item \textbf{C.1 coverage and C.4 repeats.} Complete the planned dz/speed grid under geometric”. Strategy 2 flight status: see finding on Ch.4 (crashes on 5 Oct).
- **Suggested direction:** Move the C.1/C.4 completion and onboard validation to a 'remaining work' paragraph; keep true extensions (n ≥ 4, retrained hybrid, residual RL) in the future-work list.
- **Confidence:** HIGH — text read

### P-040 — Repository-internal references appear in thesis text (docs numbers, script names, 'Priority D/E')

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch5_experimental_setup.tex` L39 (front matter / chapter opener)
- **Quote:** “but the planned dz/speed grid is only partly filled (see \texttt{docs/25} coverage table).”
- **Also at:** `docs/thesis/ch7_results_multirobot.tex` L58 (§Extension to four and five vehicles `sec:res3-n4sketch`): “A Priority~D/E scenario sketch (proposed IDs F4a, D1, E1, etc.) describes how n$\geq$4 formations”
- **Also at:** `docs/thesis/ch9_conclusion.tex` L66 (§Future work `sec:conclusion-future`): “optional Priority~D/E sketches (docs/36) for n$\geq$4 remain”
- **Problem:** The rendered thesis points readers to files and labels they cannot see (docs/25, docs/36, docs/26, docs/45, `run_formation --check`, `plot_comparison.py`, `full_bank_*.npz`, 'Priority D/E', 'supervisor sign-off').
- **Evidence:** grep of non-comment lines in ch3–ch9: ch5 L39 and L88, ch7 L58 and L60, ch9 L66, L68, L76 (and `plot_comparison.py` in ch6 placeholders). `firmware_app/src/lib.rs` in ch3 is a deliberate provenance footnote and fine.
- **Suggested direction:** Move to an appendix 'Reproducibility and artefacts' with the repository URL, or replace with the object (for example 'the coverage table in Appendix X').
- **Confidence:** HIGH — grep of rendered source

### P-041 — `docs/thesis/README.md` and `docs/00_README.md` are stale and point to a ledger path that does not exist

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/README.md` L16 (Building)
- **Quote:** “latexmk -pdf thesis_draft.tex # everything drafted so far, 35 pages -- USE THIS”
- **Also at:** `docs/thesis/README.md` L65: “**Nothing has flown.** Chapter 5 describes a design, not a completed campaign. As results arrive,”
- **Problem:** The README says 35 pages, nothing has flown, five contributions C1–C5, and tells the reader to verify against `../20_Verified_Claims.md`. The current draft is 50 pages, flights have happened, there are three contributions, and the ledger lives at `docs/_archive/citation_notes/20_Verified_Claims.md` (the top-level path does not exist). `docs/00_README.md` row for `20_Verified_Claims.md` has the same dead path. This misled this very review.
- **Evidence:** `docs/thesis/README.md` L71: “Its five contributions correspond one-to-one to”; `docs/00_README.md` L94: “| `20_Verified_Claims.md` | **Every claim we may cite, in each paper's own words**”. WRITING_RULES (and the chapter headers) say verify against the paper itself, never the ledger, which is the right rule.
- **Suggested direction:** Update the README (page count, status, three contributions, archive path) and fix the 00_README row. Not a thesis-text issue.
- **Confidence:** HIGH — files and paths checked

### P-042 — WRITING_RULES §1 'do not write as if C.0 or C.1 is done' is superseded by the repository state

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/WRITING_RULES.md` L33 (1. Campaign doctrine (do not drift))
- **Quote:** “- Nothing in Ch.1–5 is a measured flight result. Do not write as if C.0”
- **Problem:** A rule can itself be wrong. It prevents the tense update that Ch.5 asks for (its header and final `\todo` say to revise to past tense as the campaign completes), and Ch.5 already reports flown facts in its status paragraph. The 07 checklist says C.0 passed and C.1 collection is complete. Following the rule literally forces false present-tense plans.
- **Evidence:** `docs/07_Thesis_Progress_Checklist.md` L21: “C.1 collection COMPLETE”; `docs/thesis/ch5_experimental_setup.tex` L371: “\todo{Continue revising to past tense for each scenario and controller row as C.4 repeats land.”
- **Suggested direction:** Replace with 'C.0 and C.1 are done as of <date>; C.4 is not. Ch.1–5 report design and protocol and, in Ch.5, a dated campaign status; no results appear before Ch.6.' Report only; no silent deviation.
- **Confidence:** HIGH — rule text vs checklist text

### P-043 — The draft contains no figure at all (no setup photo, block diagram, scenario geometry or wake sketch)

- **Severity:** SHOULD-FIX
- **Location:** `docs/thesis/ch4_control_architectures.tex` L74 (§The position and attitude loop `sec:ctrl-common`)
- **Quote:** “\section{The position and attitude loop}”
- **Problem:** grep of `\includegraphics|\begin{figure}|tikzpicture` over ch1–ch9 returns 0 for every chapter. Ch.4 describes three controllers with their injection points and has no block diagram; Ch.5 describes 16 geometries and has no picture; Ch.1 argues about a downwash cone with no sketch. Ch.6 reserves figures only as `\todo` placeholders.
- **Evidence:** Count per file: `ch1..ch9: 0` (the grep output). The ch4 table 'Enters at' column is the only structural summary.
- **Suggested direction:** Add at least: a block diagram of the shared position/attitude loop with the three injection points; a scenario-geometry figure (A1, A4, A8, B1); a setup photo or schematic of the tracked volume and the logging chain.
- **Confidence:** HIGH — grep count

## Report counts

- Findings: **43** (MUST-FIX 6, SHOULD-FIX 31, NICE 6). Table above gives the per-chapter split.
- Citations checked: **42** claims (VERIFIED 39, MISMATCH 2, UNVERIFIABLE 1); cite keys resolved: 20 of 20.
- Equations / derivations checked: **10**.
- Defence questions: 15 (YES 3 / PARTLY 7 / NO 5).

## What could not be done

- Abro et al.: no PDF in the repository (UNVERIFIABLE).
- Hsieh et al. v1 not available; the 'sevenfold' claim might match an earlier version.
- The firmware and the analysis scripts were not re-read for most claims; where a finding rests on the repository, the file read is cited. Whether the training label is raw or differenced, which RPM source feeds `a_res`, and the real C.2 data split live in code I did not read.
- No numerical experiment was run; derivations were done by hand. The final page layout was checked only on the clipped-table page.
- Findings marked MEDIUM/LOW confidence depend on repository statements that may be superseded.

## Verified facts vs judgement

Verified facts are those with a quoted line and a located file (all `Quote`, `Also at` and `Evidence` items; every line number was found by exact string search at generation time). Judgement is in the `Problem` interpretation, `Suggested direction`, `Confidence`, the scoring and the readiness statement.

