# Stage 0 — whole-draft comprehension and audit (dry run, 2026-10-14)

**Run by:** Claude (Sonnet 5.5) as a dry run of the STAGE 0 prompt of `docs/thesis_revision_prompt.md`, with project knowledge of the code (not available to an external model). No edits to the thesis were made. Mechanical inventories are in the appendices (A claims, B numbers, C equations and labels); this file holds the judgement.

Items marked **[needs project knowledge]** are findings an external model could only make if the fact sheet of the prompt told it; they led to additions of the prompt (listed at the end).

## 1. Reading record
| File | Read |
|---|---|
| `ch1_introduction.tex`, `ch2_related_work.tex`, `ch3_system_modelling.tex`, `ch4_control_architectures.tex`, `ch5_experimental_setup.tex` | **fully**, line by line |
| `ch6_results_2robot.tex`, `ch7_results_multirobot.tex`, `ch8_discussion.tex`, `ch9_conclusion.tex` | **fully** (skeletons; comments skipped in the analysis, read in ch6) |
| `thesis_draft.tex`, `preamble_draft.tex` | fully |
| `references.bib` | all 20 entries (key, type, year, venue / eprint, notes); not every field |
| `docs/15`, `docs/19` | the first sections only (research questions, gap, claims) |
| source PDFs, `20_Verified_Claims.md` | **not read** (stage 1 / 1b) |

## 2. Thesis map
| Ch. | Title | Words (LaTeX tokens) | Sections | Tense / register | State |
|---|---|---|---|---|---|
| 1 | Introduction | 2,287 | Motivation · Problem · Gap · Compared strategies · Research questions · Contributions · Scope · Outline | present; impersonal | draft; 1 provenance `\todo` |
| 2 | Background and Related Work | 5,713 | 2.1 dynamics and baseline control · 2.2 aerodynamic interaction · 2.3 reactive · 2.4 predictive (+ superposition) · 2.5 injecting the prediction · 2.6 learning directly · 2.7 hybrid · 2.8 evaluation (+ existing comparisons) · 2.9 summary and gap | present / past for cited work | draft; 1 provenance `\todo`, 1 misplaced `% UNVERIFIED` marker |
| 3 | System Modelling and Residual-Force Formulation | 3,083 | 3.1 notation · 3.2 rigid body with residual wrench · 3.3 measuring the residual (+ failure modes, sign convention) · 3.4 predicting from relative state (deep sets, superposition, guards, normalisation) · 3.5 assumptions · 3.6 summary | present | draft; **§3.4 stale (see 10, 13)** |
| 4 | Control Architectures | 2,873 | 4.1 position and attitude loop · 4.2–4.5 Strategies 0–3 · 4.6 fairness · 4.7 summary | present; status column and summary in "at the time of writing" | draft; **Strategy 2 deviations stale** |
| 5 | Experimental Setup and Protocol | 3,091 | platform · volume · formation library (null scenarios, regimes) · measurement and logging · protocol (gate, collection, campaign, metrics) · role of simulation · threats · summary | **mixed** present / present perfect; a "Campaign status" paragraph in the past | draft; 2 `\todo` |
| 6 | Two-robot results | 1,122 | overview · tracking · effort · predictive strategy · hybrid (if in scope) · summary | skeleton, placeholders (`\todo` red) | skeleton |
| 7 | Multi-robot results | 583 | three-robot campaign · transfer penalty · extension to 4–5 vehicles · limitation · summary | skeleton | skeleton |
| 8 | Discussion | 775 | RQ1–RQ4 · contributions revisited · threats revisited | skeleton | skeleton |
| 9 | Conclusion | 520 | summary · answers to RQs · engineering takeaway · future work · closing remark | skeleton | skeleton |

## 3. Argument chain (as the draft makes it)
1. Dense formation flight needs the interaction force handled instead of avoided (Ch. 1 §1.1; Ch. 2 intro). *Supported by literature citations; the "60 cm planning gap" is one example from Shi et al.*
2. The force is nonlinear, asymmetric, and has no near-field closed form (Ch. 1 §1.1; Ch. 2 §2.2). *Supported: Bauersfeld et al. (jet model only beyond about 2.5 motor distances).*
3. A controller can **measure** the residual (reactive) or **predict** it (predictive); the two are complementary (Ch. 1 §1.2; Ch. 2 §2.3–2.4). *Asserted as an organising claim ("exactly two ways") that is not argued to be exhaustive; Ch. 2 §2.6 then adds a third stance (learn the action directly), so "exactly two ways" is contradicted by the thesis' own survey.*
4. Strong results exist for each, but under non-comparable conditions (Ch. 1 §1.3; Ch. 2 §2.8, Table 2.1). *Supported by Table 2.1; the table is the load-bearing evidence.*
5. Two systematic comparisons exist in adjacent territory (Gu et al., Abro et al.) and bound the gap (Ch. 2 §2.8.1). *Abro et al. is abstract-level / no PDF held.*
6. Hence the gap: families defined by how the force is obtained have not been compared under common conditions; hybrid on teams untested; penalty of summation unquantified (Ch. 1 §1.3; Ch. 2 §2.9). *Hedged "to the best of our knowledge"; supported by two audits that are recorded as provenance `\todo`s inside the text.*
7. Therefore: a controlled comparison (4 strategies, one airframe, frozen library, per-strategy frozen gains, null scenarios) with RQ1–RQ4 (Ch. 1 §1.5; Ch. 4; Ch. 5). *Fairness argument present in Ch. 1 §1.6, Ch. 4 §4.6, Ch. 5 §5.4.3; consistent with the current rule.*
8. Expected evidence (Ch. 6–9): placeholders only. *Falsifiability of each RQ stated, "negative result is a result", but the decision criteria (what counts as "separate", effect-size threshold, number of repeats) are not stated anywhere.*

Step 3 ("exactly two ways", "organises the entire field") is the argument's weakest premise; step 8 lacks pre-specified decision criteria.

## 4. Promise–delivery matrix
| Promise (Ch. 1) | Delivered in | Reported in | Gap |
|---|---|---|---|
| RQ1 (compare strategies vs baseline: accuracy, rejection) | Ch. 4 (strategies), Ch. 5 §5.4.3 and metrics | Ch. 6 §6.2–6.3, Ch. 8 §8.1 | metrics are named but **window / detector / statistics not defined** (e.g. what "residual-force rejection" is computed from; crossing-based transient metric absent) |
| RQ2 (regime dependence) | Ch. 5 §5.3.2 (regimes) | Ch. 6 §6.5 summary table, Ch. 8 §8.2 | no definition of a regime boundary (what is "quasi-static" vs "transient") |
| RQ3 (transfer penalty 2 → 3 vehicles) | Ch. 3 §3.4.2 (superposition), Ch. 5 (B-scenarios) | Ch. 7 §7.2 | needs the third vehicle; limitation handled in Ch. 7 §7.4; the penalty metric itself is not defined |
| RQ4 (cost: sensing, compute, data, tuning) | Ch. 4 "Requirements" paragraphs | Ch. 6 §6.3 (effort), Ch. 8 §8.4 | tuning effort has no measurable definition; compute cost has no stated measurement method |
| Contribution 1 controlled comparison | Ch. 4–5 | Ch. 8 §8.5 | ok |
| Contribution 2 transfer penalty | Ch. 3, 5 | Ch. 7, 8 | ok (same gap as RQ3) |
| Contribution 3 engineering cost account | Ch. 4 | Ch. 6, 8 | ok |
| "apparatus" (library, residual path, onboard network with weight upload, training pipeline, simulator) | Ch. 5 | — | the **training pipeline and weight-upload protocol are never described in Ch. 3–5**; only mentioned |
| Outline | Ch. 1 §1.8 | — | outline matches chapters |
**Orphans / mismatches:** Chapter 1 lists **three** contributions; `docs/19` and the project convention list **five** (C1–C5): the prompt must not say "C1–C5 in Chapter 1". Ch. 6 §6.5 and Ch. 9 future work retain NA-INDI/LINDI items that Chapter 1 says are out of scope.

## 5. Claim inventory
See **appendix A** (53 sentences with `\cite`, 96 citation uses, 20 keys; chapter:line, keys, sentence). Observations that need judgement:
- Highest density of numeric claims about other work: Ch. 2 §2.4 (Neural-Swarm, Neural-Swarm2, Smith), §2.5 (Hsieh 2026, Shankar), §2.7 (Cobo-Briesewitz). These are the rows to verify first in stage 1.
- **Context given vs needed:** most numbers carry platform / vehicle count / sim-vs-hardware context in Ch. 2; Ch. 1 repeats three of them without context (36 %, 56 %, 31 %) and without the footnote that Ch. 2 attaches to the 36 % figure.
- **Inconsistent context between chapters (reference-correctness defect):** Ch. 2 L293–296 says that *on hardware* Hsieh et al. match the error published by Chee et al.; Ch. 4 L304–306 says *in simulation* they match it ("an order of magnitude less computation … off-board"). One of them is wrong or conflates two results.
- **Unverified markers:** Ch. 2 L500 carries `% UNVERIFIED (review P-010): no PDF of Abro et al.` placed *before the paragraph about Gu et al.* (misplaced); the Abro paragraph itself (L510–518) rests on an abstract-level reading.
- Opinion statements flagged by the author in the text ("in our reading", "our observation, not theirs"): Ch. 2 L112–127, L345–347; acceptable but should be listed as the author's inferences.
- **"exactly two ways", "organises the entire field"** (Ch. 1 L62–63) are universal claims not supported by a citation (see 3).

## 6. Number inventory
See **appendix B** (132 `\SI`/`\SIrange`/`\num` occurrences). Repeated numbers and agreement:
| Number | Places | Agree? |
|---|---|---|
| 9 cm rotor-to-rotor | Ch. 1 L30 (CF2.0), Ch. 2 L33 (CF2.0), Ch. 3 L275 (vehicles "9 cm across"), Ch. 5 L45 (CF2.1 Brushless) | the value is repeated for **two different airframes**; the brushless figure needs a source (**VERIFY**) |
| 60 cm planning gap | Ch. 1 L31, Ch. 2 L35 | yes |
| 36 % / 56 % (Smith) | Ch. 1 L54–55, Ch. 2 L214–216, L415–416, Ch. 5 none | yes; footnote only in Ch. 2 |
| 31 % (Hsieh) | Ch. 1 L99, Ch. 2 L287, L417, Ch. 4 L301 | yes; "abstract only" only stated in Ch. 2 §2.8 and Ch. 4 |
| 24 cm, 16 robots, 3× | Ch. 1 L53, Ch. 2 L202, L412, Table 2.1 | yes; Ch. 1 does not separate the 3× and 24 cm results (Ch. 2 does) |
| 987 parameters | Ch. 3 L219, L227; Ch. 4 L271 | internally yes; **stale against the code** (see 10) |
| 35 variables at 500 Hz | Ch. 5 L190, L368 | yes; **stale** (current configuration records 96) |
| 500 Hz control loop | Ch. 5 L66 | single place; **VERIFY** against the flight code (stabiliser and attitude loop rates) |
| 2 m × 4 m × 1.7 m volume | Ch. 5 L72, L367 | yes; not tape-measured (open `\todo`) |
| 2 m neighbour exclusion, 4 cm clamp, 8 m/s² output bound | Ch. 3 L268–279 | **stale** exclusion / clamp (see 10); output bound matches the code (8.0) |
| 16 scenarios | Ch. 5 L94, L367 | yes (A1–A8, B1–B3, C1–C5 = 16) |

## 7. Notation table (principal symbols; full inventory is `$…$` content of Ch. 2–5)
| Symbol | Meaning | Defined | Issue |
|---|---|---|---|
| $p_i, v_i, R_i, \omega_i$, $m$, $J$ | state, mass, inertia | Ch. 3 §3.1 (Ch. 2 §2.1 uses them first, forward reference) | Ch. 2 uses before definition |
| $f_i$, $\tau_i$ | collective thrust, torque | Ch. 2 L67, Ch. 3 | none |
| $f_{\mathrm{res}}, \tau_{\mathrm{res}}$ | residual wrench | Ch. 2, Ch. 3 | none |
| $a_{\mathrm{res}}$, $a_{\mathrm{meas}}$, $a_{\mathrm{model}}$, $\hat a_{\mathrm{res}}$ | residual accel., measured, model, predicted | Ch. 3 §3.3, §3.4 | $f_{\mathrm{res}}/m$ vs $a_{\mathrm{res}}$ used interchangeably in Ch. 3 / Ch. 4 |
| hat on a vector | **two meanings**: hat map $\hat{\omega}$ (Ch. 3 §3.1) and "predicted" $\hat{a}$ (Ch. 3 §3.4) | Ch. 3 | **notation overload** |
| $\varpi_k, k_{t,k}$ | rotor speed, thrust constant | Ch. 3 L57 | $\omega$ (body rate) vs $\varpi$ (rotor) easy to confuse |
| $a_{\mathrm{cmd}}, a_{\mathrm{ff}}, a_{\mathrm{des}}$ | commanded, feed-forward, desired accel. | Ch. 4 L90–102 | $a_{\mathrm{des}}$ of Ch. 3 (no gravity, no residual) vs $a_{\mathrm{cmd}}$ (with both): explained once, easy to misread |
| $K_p, K_v, K_i$, $K_R^{\mathrm{geo}}$, $K_\omega^{\mathrm{geo}}$, $k_{i,\mathrm{att}}$ | geometric gains | Ch. 4 | units differ between geo and INDI gains (stated) |
| $K_R^{\mathrm{INDI}}, K_\omega^{\mathrm{INDI}}$, $\alpha_{\mathrm{ref}}, \alpha_{\mathrm{des}}, \alpha_{\mathrm{meas}}$, $\tau_{\mathrm{cur}}$ | INDI law | Ch. 4 §4.3 | $\tau_{\mathrm{cur}}$ (τ current) vs $\tau$ command |
| $k_R, k_\Omega, \Omega, \Omega_d, M$ | Lee et al. symbols | Ch. 4 L132–133, L168 | mapping given once |
| $\phi, \rho$, $\mathcal{F}$ | deep-sets embedding, readout, interaction function | Ch. 3 §3.4 | stale architecture (10) |
| $\mu, \sigma, W_1, b_1$ | normalisation | Ch. 3 §3.4.4 | none |

## 8. Terminology
Defined at first use: INDI, NN, FBL (Ch. 1 §1.4), differential flatness (Ch. 2 §2.1), deep sets (explained only in Ch. 3 §3.4; used from Ch. 2 §2.4 "heterogeneous deep sets"). **Used before or without definition:** *deep sets* (Ch. 2), *SE(3)* (Ch. 2), *spectral normalisation* (Ch. 2, Ch. 3, Ch. 4), *hardware gate* and *null scenarios* (Ch. 1, defined in Ch. 5), *DShot*, *EKF*, *mocap* (Ch. 5), *Lipschitz* (Ch. 2), *C.1 / C.4* (internal campaign labels in Ch. 5, Ch. 6, Ch. 9 — **not thesis terminology**), *cf5, cf_second* (vehicle names, Ch. 5, Ch. 6), *uSD*, *roles (bottom/top)*.

## 9. Cross-reference check
105 labels, 169 `\ref` uses, **0 undefined references**, 47 labels never referenced (section labels; list in appendix C). 20 distinct citation keys, **0 unresolved**, 0 unused bibliography entries. Forward references to Chapters 6–9 are made by `\ref` in Ch. 1 and Ch. 6–9 (they exist as skeletons); Ch. 2 and Ch. 3 refer to results chapters by name in prose, consistent with the convention. **Internal file names appear in the text:** `docs/25` (Ch. 5 L39), `manifest_2026_09_21…json` (Ch. 5 L34), `firmware_forward`, `full_bank_*.npz` (Ch. 6), `firmware_app/src/*.rs` (Ch. 3 footnote).

## 10. Stale-scope scan (including lab-status passages, H9)
| Location | What | Class |
|---|---|---|
| Ch. 3 L219–229, Ch. 4 L270–278 | the network is "ours, **987 parameters**, φ 6→16→16→8, ρ 8→16→16→3, vector output, 2 m exclusion and 4 cm clamp guards, no spectral normalisation" **[needs project knowledge]**: the onboard model is a port of the Neural-Swarm2 architecture (19,297 weights, φ 25→40→40→20, ρ 20→40→40→40→1, scalar vertical force, gate on relative x, y and x-velocity, ground-effect term, up to 3 neighbours) since 2026-09-14 | **stale design facts; "these equations are the code" is false for §3.4** |
| Ch. 3 L268–279 | neighbour exclusion at 2 m and clamp at 4 cm | stale (see above) |
| Ch. 4 L254 | network "evaluated at the control rate" | stale (evaluated at about 100 Hz, every tenth control step) **[needs project knowledge]** |
| Ch. 4 L56–59, L350–352 | "Status" column and summary: "flying", "implemented, unflown" | lab-derived status, stale (H9) |
| Ch. 5 L28–39 | "Campaign status (September 2026)": flights executed, vehicle names, merges, manifest file, "indicative comparisons exist" | lab-status passage (H9), stale |
| Ch. 5 L174–177 | "established in simulation before any flight: a dry run … two of the six inputs never varied" | simulation-derived statement (H1) |
| Ch. 5 L314–320 | simulation "established that every scenario executes … exposed two defects in flight code" | simulation / lab-derived statement (H1, H9) |
| Ch. 5 L190, L368 | 35 recorded variables | stale design number |
| Ch. 5 tense | not future tense: present design description with past-perfect campaign claims; a `\todo` says "revise to past tense" | prompt rule H1 must be restated (see 13) |
| Ch. 6 L45–50, §6.5 | "Indicative single flights and partial C.1 collection exist"; Hybrid (NA-INDI) section "if in scope", "Strategy 4 in the wider project numbering" | status passage; out-of-scope strategy retained |
| Ch. 6 tables | row labels S0–S3 (project abbreviations) | vocabulary |
| Ch. 7 L78 "decision rests on … simulation sanity checks" | contradicts "No result … rests on simulation" (Ch. 5 L322, Ch. 1 L212–214) | internal inconsistency |
| Ch. 9 L84 closing remark | "remaining work is to replace the placeholders … from the lab campaign" | author's to-do inside the text |
| Ch. 9 L68, L75 future work | "Our notes describe a retrain path …", "SIL and desk parity do not substitute for `rnn.en=1` flight logs" | internal project vocabulary / status |
| Ch. 4 L80–83 vs L64–67 | "Strategies 0, 1 and 2 **share one** geometric position loop" vs "each strategy is its own controller" | tension in wording (fairness rule) |
| strategies > 3 in main chapters | none in Ch. 1–5 (only related work); Ch. 6 §6.5 as above | — |
| "shared substrate" / "one gain set" | only in negated form | ok |

## 11. Equation inventory
12 equation blocks (appendix C). All use the z-up frame with gravity along $-e_3$ and the residual **subtracted** (eq. 3.7, 4.1). Findings:
- Eq. 2.1 and 3.1 are the same model; Ch. 2 §2.1 states it before Ch. 3 defines the notation (forward dependence).
- Eq. 3.2 (measured residual) is consistent with eq. 3.1; the text derives $a_{\mathrm{meas}}$ from the accelerometer (specific force, rotated, minus $g e_3$) correctly.
- Eq. 3.7 sign derivation correct from eq. 3.1. **[needs project knowledge]** The flight code applies the residual through a configurable sign (`res_sign`): the geometric+network configuration uses the textbook sign, our INDI path was flown with the opposite sign in the July campaign, and the textbook sign diverged once in hover (2026-09-09, cause not isolated). Chapter 3's statement that the residual "is subtracted whether measured or predicted" and "these equations are the code" therefore needs the author's reconciliation; the audit cannot decide it.
- Eq. 4.1 includes both residual terms in one expression although no strategy uses both (Strategy 0: none, 1: measured, 2: predicted); stated in the text.
- Eq. 4.2 (reduced geometric law) and eq. 4.3 (Lee's law) are labelled as such; eq. 4.3 matches the cited source at equation level according to the author's notes; **not verified here** (no PDF read in this stage).
- Eq. 4.4–4.5 (INDI increment): consistent with the cited approach; `K^INDI` units stated.
- Eq. 3.5 (deep sets) matches the *family* but **not the current network** (no ground-effect branch, scalar vertical output).
- No equation for: the INDI position-loop term, the Strategy 3 feedback-linearising law, the metrics of Ch. 5.

## 12. Open `\todo` and `% VERIFY` markers
Ch. 1 L221 (provenance, green), Ch. 2 L571 (provenance, green), Ch. 2 L500 (`% UNVERIFIED P-010`, misplaced), Ch. 5 L87 (tape-measure the volume), Ch. 5 L374 (revise to past tense), Ch. 5 L289–290 ("e_R is not recorded; will be added"), plus 24 red placeholder `\todo` in Ch. 6–9 (8 + 4 + 8 + 4).

## 13. First impressions — the ten weakest places, in order
1. **Ch. 3 §3.4 and Ch. 4 Strategy 2: the described network is not the implemented one** (987 parameters vs a port of the Neural-Swarm2 architecture with 19,297 weights); the chapter's own claim "these equations are the code" is then false. Blocker.
2. **Residual sign vs implementation** (eq. 3.7 vs the configurable sign of the INDI path): the thesis asserts one sign; the code has two configurations. Needs the author's decision before any rewriting. Blocker.
3. **Lab- and simulation-derived statements inside Ch. 5** (campaign status; two simulation findings) and status columns in Ch. 4 / Ch. 6: conflict with the no-results rule and are stale.
4. **Reference-correctness conflict Ch. 2 vs Ch. 4** on Hsieh et al. (hardware vs simulation match with Chee et al.).
5. **Ch. 5 metrics are named but not defined** (window, detector, statistic, repeats, decision criteria); RQ2 regimes have no operational boundary; the training pipeline and weight-upload protocol promised in Ch. 1 are not described.
6. **Premise "exactly two ways"** (Ch. 1) contradicted by Ch. 2 §2.6; "organises the entire field" unsupported.
7. **Background insufficient for the student reader**: no derivation of INDI, no explanation of SE(3) / rotation matrices, deep sets, flatness, EKF / mocap latency; Ch. 2 uses the dynamics before Ch. 3 defines the notation.
8. **Bibliography**: 14 of 20 entries are `@misc` preprints; years and venues inconsistent with the keys (`shi2022neuralswarm2` year 2020; `cobobriesewitz2026lindi` year 2025; Tal and Karaman 2018 with a note about the published version); two entries without a PDF (Lee CDC 2010, Abro 2025).
9. **Duplication and unequal hedging across Ch. 1 and Ch. 2** (36 % / 31 % passage, 60 cm gap, "not a criticism" paragraph); Smith footnote only in Ch. 2; Ch. 1 does not separate the NS2 3× and 24 cm results.
10. **Skeleton chapters carry internal vocabulary** (S0–S3, C.1/C.4, cf5, file names, NA-INDI slot), an internal to-do as a "closing remark", and one statement that contradicts "no claim rests on simulation".

---

## What this dry run changed in the prompt (for the author)
- The fact sheet gained the **current network facts**, the **recorded-variable count**, the **residual-sign / implementation reconciliation** as an author decision, and the **loop-rate verification** item, because the audit could not have found items 1, 2 and the stale numbers without project knowledge.
- Rule H1 was corrected: Chapter 5 is **not** written in the future tense; the rule is now "design description in present / future tense, no past-tense claim of what was done or found".
- "Contributions C1–C5 in Chapter 1" was wrong: Chapter 1 lists three; `docs/19` has five.
- Stage 0 got an explicit item for **statements derived from the project's own simulations or lab sessions** (H1/H9) and for **internal vocabulary** (file names, nicknames, S0–S3, C.1/C.4).

---

## Addendum (2026-10-14): code-vs-draft check, part 14 of the prompt (done after the code access was added)
| Item | Draft says | Code says (location) | Verdict |
|---|---|---|---|
| (a) network | φ 6→16→16→8, ρ 8→16→16→3, 987 parameters, vector output, 2 m exclusion, 4 cm clamp, no spectral normalisation (Ch. 3 §3.4, Ch. 4 Strategy 2) | `residual_nn.rs` header and constants: φ 6→25→40→40→20, ρ 20→40→40→40→1, ground-effect branch (4 inputs), 19,297 weights, scalar vertical force, gate \|dx\|<0.2 \|dy\|<0.2 \|dvx\|<1.5, `MAX_NEIGHBOURS = 3`, `OUT_CLAMP = 8.0`; the 987-parameter network is stated to be gone | **stale** |
| (b) residual sign | residual subtracted (Ch. 3 eq. 3.7, Ch. 4 eq. 4.1) | `lib.rs` ~L1856–1915: `res_sign` from `g_indi_res_sign`, default `+1` ("frozen / flight-proven"), `-1` "derivation-correct sign"; configuration decides | **author decision** |
| (c) integrals | eq. 4.1 has $K_i\int e_p$ (vector), eq. 4.2 has $k_{i,\mathrm{att}}\int e_R$; text says the attitude integral is an addition "the source does not have" | `lib.rs` L525 `ENABLE_POSITION_INTEGRAL = false`, L535 `ENABLE_Z_INTEGRAL = true` (vertical only, `accumulate_z_integral`, `ki_z`), L540 `ENABLE_ATTITUDE_INTEGRAL = false` | **mismatch**: only a vertical integral is enabled; wording is an author decision |
| (d) INDI increment and filters | eq. 4.4–4.5 incremental law with a filtered angular acceleration | consistent in structure (`controller_step`); filter orders and cutoffs are parameters (`fc_bw`, `filt_order`, …) | consistent; the draft names no cutoff, no filter order (completeness) |
| (e) loop rate | "control loop runs at 500 Hz" (Ch. 5 L66) | `lib.rs` ~L2248: "The loop actually runs at 1000 Hz"; some services (network upload) at 500 Hz | **verify / stale** |
| (f) logged variables | 35 at 500 Hz (Ch. 5 L190, L368) | `usd_study_config.txt`: 96 variables at 500 Hz | **stale** |
| (g) scenario library | 16 scenarios (A1–A8, B1–B3, C1–C5) | `crazyswarm2` repo not part of this audit | not checked |
