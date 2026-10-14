# Stage 0 — Whole-draft comprehension and audit (no edits)

Run: 2026-10-14, one continuous seven-phase run (prompt `docs/thesis_revision_prompt_full.md`, pasted in full).
Model: Claude Opus 5.5. This file replaces the earlier Stage 0 dry run (commits 2542863e, c9fe9280); that version
is still in git history. The earlier appendices (`stage0_appendix_*.md`) were **not** used as input for this audit and
are superseded by the sections below.

Severity scale used in all stage files: **S1** = an examiner would fail or require major revision on it (wrong
fact, wrong equation, stale design, result leakage); **S2** = must fix before submission (missing definition, unsupported
claim, inconsistency); **S3** = quality (explanation, style, signposting).

---

## 1. Reading record

| File | Read | Notes |
|---|---|---|
| `docs/thesis/thesis_draft.tex` | fully | build order ch1–ch9, `plain` style, `../references.bib` |
| `docs/thesis/preamble_draft.tex` | fully | report class, siunitx with fallback, todonotes inline, `\chapref` trap macro |
| `docs/thesis/ch1_introduction.tex` | fully (275 lines) | |
| `docs/thesis/ch2_related_work.tex` | fully (579 lines) | |
| `docs/thesis/ch3_system_modelling.tex` | fully (379 lines) | |
| `docs/thesis/ch4_control_architectures.tex` | fully (353 lines) | |
| `docs/thesis/ch5_experimental_setup.tex` | fully (376 lines) | |
| `docs/thesis/ch6_results_2robot.tex` | fully | skeleton |
| `docs/thesis/ch7_results_multirobot.tex` | fully | skeleton |
| `docs/thesis/ch8_discussion.tex` | fully | skeleton |
| `docs/thesis/ch9_conclusion.tex` | fully | skeleton |
| `docs/references.bib` | fully (20 entries) | |
| `flying_drone_stack/firmware_app/src/lib.rs` | fully (2591 lines) | |
| `flying_drone_stack/firmware_app/src/residual_nn.rs` | fully (373 lines) | |
| `flying_drone_stack/firmware_app/traj_iface.c` | fully (833 lines) | |
| `flying_drone_stack/tools/usd_study_config.txt` | fully (103 lines) | 7 header lines + 96 variables |
| `crazyswarm2/crazyflie_examples/crazyflie_examples/formations/scenarios.py` | header, every builder signature and docstring, registry (lines 1–470) | `check_spec`/self-test body (470–801) not read |
| `crazyswarm2/.../formations/safety.py` | grep only (floors, limits) | |
| `crazyswarm2/crazyflie/config/crazyflies.yaml` | grep only (`res_sign`, `ctrl_mode`, `ki_z`, `mass`) | the study configuration; comments in it are lab notes and were not used as content |
| `crazyswarm2/crazyflie/config/study/*.patch` | grep only | |
| `crazyflie-firmware/src/modules/src/stabilizer.c`, `.../interface/stabilizer_types.h` | grep only (loop rate) | read to settle the 500 vs 1000 Hz question; code, not a result document |
| `flying_drone_stack/firmware_app/Cargo.toml`, `Makefile`, `Kbuild` | features section, grep | flash-resident weights feature |
| `flying_drone_stack/tools/residual/model.py`, `train.py`, `dataset.py`, `test_pipeline.py` | docstrings + grep (normalisation, spectral, loss, target) | settles normalisation fold, training target, absence of spectral normalisation |
| `docs/15_...`, `docs/19_...` | `docs/19` fully; `docs/15` RQ section (lines 100–215) | context only |
| `docs/18`, `docs/04`, `docs/10`, `docs/05` | **not read** in Phase 0 | not needed for the audit; the code and `scenarios.py` settle the facts they would describe |
| `docs/papers/*.pdf` | **not read** (by rule) | Phase 1 |
| `docs/_archive/citation_notes/*` | **not read** (by rule) | Phase 1B |
| `flying_drone_stack/CLAUDE.md`, `firmware_app/CLAUDE.md` | loaded automatically by the harness | **not used as a source**; `firmware_app/CLAUDE.md` is itself stale (says 987 weights) |

---

## 2. Thesis map

Word counts are prose words with LaTeX commands stripped (approximate).

| Ch | Job (one sentence) | Section tree | Words | Tense | State |
|---|---|---|---|---|---|
| 1 Introduction | Motivates dense formation flight, states the measure/predict distinction, the gap, the strategies, RQ1–RQ4, three contributions, scope and outline. | 1.1 Motivation · 1.2 Problem · 1.3 Gap · 1.4 The compared strategies · 1.5 Research questions · 1.6 Contributions (+ "does not claim", "negative result", provenance todo) · 1.7 Scope (Tab. 1.1) · 1.8 Outline | ≈2 090 | present / future ("sets out to establish") | draft |
| 2 Background and related work | Surveys how the interaction force is measured, predicted, injected, learned around, or combined, and argues the results cannot be composed. | 2.1 Multirotor dynamics and baseline control · 2.2 Aerodynamic interaction · 2.3 Reactive compensation · 2.4 Predictive compensation (+ ¶ superposition) · 2.5 Injecting the prediction (¶ FF geometric, ¶ FBL flatness, ¶ MPC, ¶ docking LQR) · 2.6 Learning the compensation directly · 2.7 Hybrid · 2.8 How these methods are evaluated (Tab. 2.1; ¶ comparisons that exist) · 2.9 Summary and gap | ≈5 410 | present | draft |
| 3 System modelling | Defines the residual acceleration, how it is measured and predicted, its sign in the control law, the network, and the assumptions. | 3.1 Notation · 3.2 Rigid body + residual wrench (¶ force not moment) · 3.3 Measuring the residual (3.3.1 requirements/failure; 3.3.2 sign) · 3.4 Predicting (3.4.1 deep sets; 3.4.2 superposition; 3.4.3 input conditioning; 3.4.4 normalisation) · 3.5 Assumptions (Tab. 3.1) · 3.6 Summary | ≈2 850 | present | draft |
| 4 Control architectures | Describes each strategy as implemented and as the reference method, and what makes the comparison fair. | Tab. 4.1 · 4.1 Position and attitude loop (4.1.1 position; 4.1.2 attitude; 4.1.3 runtime selection) · 4.2 S0 · 4.3 S1 · 4.4 S2 · 4.5 S3 · 4.6 Fairness · 4.7 Summary | ≈2 530 | present + status in past/present perfect | draft |
| 5 Experimental setup | Platform, volume, library, logging, protocol, metrics, simulation, threats to validity. | ¶ Campaign status · 5.1 Platform · 5.2 Volume · 5.3 Library (Tab. 5.1; 5.3.1 null; 5.3.2 regimes) · 5.4 Logging (5.4.1 why onboard; 5.4.2 recorded; 5.4.3 time alignment) · 5.5 Protocol (5.5.1 gate; 5.5.2 collection; 5.5.3 campaign; 5.5.4 metrics) · 5.6 Simulation · 5.7 Threats (Tab. 5.2) · 5.8 Summary | ≈2 910 | **mixed**: protocol present/future; campaign status and simulation findings in past tense | draft |
| 6 Two-robot results | Placeholder structure for the two-vehicle results. | 6.1 Overview · 6.2 Tracking · 6.3 Effort · 6.4 Predictive (NS2) · 6.5 Hybrid (NA-INDI) · 6.6 Summary | ≈990 | future | skeleton |
| 7 Multi-robot results | Placeholder structure for RQ3. | 7.1 Campaign · 7.2 Transfer penalty · 7.3 4–5 vehicles · 7.4 Limitation · 7.5 Summary | ≈490 | future | skeleton |
| 8 Discussion | Placeholder per RQ, contributions and threats. | 8.1–8.4 RQ1–RQ4 · 8.5 Contributions · 8.6 Threats | ≈690 | future | skeleton |
| 9 Conclusion | Placeholder answers and future work. | 9.1 Summary · 9.2 Answers · 9.3 Engineering takeaway · 9.4 Future work · 9.5 Closing | ≈440 | past ("set out") / future | skeleton |

---

## 3. Argument chain

| # | Step | Where made | Supported? |
|---|---|---|---|
| A1 | Close multirotor flight produces a downwash disturbance strong enough to destabilise a lower vehicle. | 1.1 ¶1; 2 intro ¶1; 2.2 ¶1 | **Asserted without citation** in 1.1 ¶1, 2 intro ("large enough to destabilise") and 2.2 ¶1 ("strongly nonlinear… asymmetric"). Supportable from held PDFs (Shi 2020, Bauersfeld, Kiran) — Phase 1. |
| A2 | Conventional practice keeps vehicles far apart; the margin costs density. | 1.1 ¶1–2; 2 intro | 60 cm figure cited (Shi 2020 citing earlier work). "The margin is expensive… applications…" uncited reasoning (acceptable). |
| A3 | The interaction is hard to model from first principles in the near field. | 1.1 ¶3; 2.2 ¶3 | Bauersfeld cited; "formation flight happens closer than that" is flagged as "our observation" in Ch2 but stated flatly in Ch1. |
| A4 | Data-driven compensation has worked (numbers). | 1.1 ¶4; 2.4; 2.8 ¶1 | Cited; context checks in Phase 1. "The margin has been substantially reduced" uncited summary. |
| A5 | Information about the force comes in exactly two ways: measure or predict. | 1.2 ¶1; 2.9 ¶1 | **Overstated in Ch1** ("exactly two ways … organises the entire field"): Ch2 itself adds a third stance (2.6, no force estimate) and the MPC/planning stance; Ch2.9 says "two ways" then "A third approach". Internal inconsistency (S2). |
| A6 | Measurement is late but general; prediction early but bounded by training data. | 1.2 ¶2–4; 2.3; 2.4; 2.7 | Reasoned; the "unconstrained" behaviour outside training data (1.2 ¶3) is an uncited strong claim — the deployed network has an 8 m/s² output bound (Ch3), so "unconstrained" contradicts Ch3 (S2). |
| A7 | A hybrid exists on one vehicle with a small margin. | 1.2 ¶5; 2.7; 2.9 | Cobo-Briesewitz cited; Phase 1. |
| A8 | Comparisons exist but not across these families; results cannot be composed. | 1.2 ¶6; 1.3; 2.8 | Gu (PDF held) and Abro (no PDF) cited; "to the best of our knowledge" hedge present; provenance todo. |
| A9 | Gap: reactive vs predictive not evaluated under identical conditions. | 1.3; 2.9 | Hedged. **Scope inconsistency**: 1.3 italic gap statement names "reactive, predictive and hybrid"; the thesis compares only the first two. Ch1 says so in the next sentence; acceptable but the italic sentence states the gap more broadly than what the thesis fills (S3). |
| A10 | Two narrower gaps: hybrid untested on teams (out of scope); size of summation penalty unreported. | 1.3 ¶3; 2.9 | The first narrower gap is one the thesis explicitly does **not** fill — listing it as a "gap that follows" in Ch1 invites the examiner question "then why state it?" (S3). Second supported by RQ3. |
| A11 | Method: controlled comparison of S0–S3 on one airframe, frozen library, one residual definition, null flights, per-strategy frozen gains. | 1.4–1.6; 4.6; 5.* | Described. Two code-level facts undermine "one residual definition entering with one sign" (residual sign differs by configuration; integral terms differ by configuration) — see §14 and §10. |
| A12 | Expected evidence: tracking/separation/attitude metrics, residual left, prediction vs measurement, cost. | 5.5.4; 6–9 skeletons | Metrics are named but **not defined mathematically** (no window, no detector, no aggregation rule) (S2). RQ2 needs a regime classification, defined only by scenario list (5.3.2). No statistical procedure (repeats count, test, effect size) (S2). |
| A13 | Negative result is informative. | 1.6 ¶"negative result"; 8 | Reasoned. |

---

## 4. Promise–delivery matrix

| Promise (Ch1) | Delivered in 2–5 | Reported in 6–9 (skeleton) | Gap / orphan |
|---|---|---|---|
| RQ1 (S0 vs reactive vs predictive; tracking + residual rejection) | 4 (strategies), 5.5.3–5.5.4 (campaign, metrics) | 6.2, 6.3, 6.6; 8.1; 9.2 | Metric definitions incomplete (no formula/window); "residual-force rejection" defined in 5.5.4 as separation error + remaining residual — OK but should be formal. |
| RQ2 (regime dependence) | 5.3.2 (regimes), Tab. 5.1 | 6.6 (Tab. 6.3), 8.2 | No operational regime definition (thresholds for separation / relative velocity / quasi-static vs transient). "Train/test splits by scenario class (Section 5.7)" in 6.6 refers to a split 5.7 only mentions in one row. |
| RQ3 (2→3 transfer penalty) | 3.4.2, Tab. 3.1; 5.3 (B1–B3) | 7.2, 8.3 | The **network-in-code gates neighbours with \|Δx\|,\|Δy\|<0.2 m and \|Δv_x\|<1.5 m/s** (§14a): in the V-stack B2 with r = 0.10 m both neighbours pass, but whether a neighbour is summed at all depends on lateral offset — this conditions RQ3 and is not stated. Penalty metric not defined (5.5.4 has no RQ3 metric). |
| RQ4 (sensing, compute, data, tuning effort) | 4 (requirements ¶), 5.5.4 (robustness and cost) | 6.3, 8.4, 9.3 | "Tuning effort" has no unit or protocol (hours? number of flights?). "Training data required to reach a given accuracy" implies a data-size sweep that Ch5 never schedules (S2). |
| Contribution 1 (controlled comparison) | 4.6, 5 | 8.5 | OK; wording "first evaluation" hedged. |
| Contribution 2 (summed-model penalty) | 3.4.2, 7 | 7.2, 8.5 | OK subject to RQ3 metric. |
| Contribution 3 (engineering account of cost) | 4 requirements; 5.5.4 | 8.4, 8.5 | OK subject to RQ4 protocol. |
| Apparatus ("weight-upload protocol", "offline training pipeline verified against the compiled controller", "simulation environment running the same controller source") | 5.6 (simulation) only; upload protocol not described in Ch5 | — | Upload protocol is no longer the study deployment path: the study build uses **flash-resident weights** (`Cargo.toml` feature `residual_nn_flash`; `lib.rs:922–926`). "Verified against the compiled controller" is a project result statement (H1-adjacent). |
| Outline: Ch3 "Strategies 1–3 use the same definition where they apply" | 3 | — | OK. |
| Outline: Ch4 "Strategies 0–3 … separating what is implemented from the published method" | 4 | — | OK. |
| Outline: Ch6–9 | — | 6–9 | Ch1 outline uses `\ref{ch:results2}` etc. (H6 says name in prose until they exist; they exist as skeletons — acceptable in the combined build, but flagged). Ch6 contains a "Hybrid (NA-INDI), if in scope" section — **orphan / stale scope** (hybrid is not compared). |
| Scope table: "moment residual measured but not separately compensated" | 3.2 ¶scope | 8 (promised "discussion chapter returns to it") | Ch8 skeleton has **no section** on the moment residual (orphan promise from 3.2). |
| 3.1/3.5 promise "discussion chapter interprets a negative result as evidence about an assumption" | Tab. 3.1 | Ch8 has no assumption-by-assumption section | Orphan (S3). |
| 4.6 promise "The discussion chapter reports where a frozen gain appears to have disadvantaged a strategy" | 4.6 | Ch8 ¶"Tuning and fairness" partly | Needs its own Ch8 section (prompt brief requires it). |
| 4.4 promise "Both flights are therefore needed, and Chapter 5 schedules them" (prediction on/off) | 5.4.2 mentions recording; **5.5 never schedules prediction-off evaluation flights explicitly** | 6.4 | Gap (S2). |
| 5.4.3 "predicted alignment … to be confirmed on the first two-vehicle flight" | — | — | Stale status (H9). |

Orphans in 6–9 not promised by 1–5: Ch6 §6.5 hybrid; Ch7 §7.3 four/five-vehicle sketch (fine as future work but should not be a results section); Ch9 future-work bullets referencing internal workflow ("C.1 coverage and C.4 repeats", "cf5", "rnn.en=1", "Our notes describe a retrain path").

**Chapter 1 vs `docs/19` drift** (Chapter 1 is authoritative): `docs/19` lists five claims C1–C5 including C2 "first multi-robot evaluation of the hybrid" (now out of scope), "seven strategies", "gains … shared across strategies wherever the architecture permits" (stale fairness rule), "onboard deep-sets network with a weight-upload protocol". Chapter 1 lists three numbered contributions plus apparatus. `docs/15` RQ1 and RQ4 still name the hybrid; Chapter 1's RQ1/RQ4 do not. **Drift reported; `docs/19` and `docs/15` are stale, not the thesis.**

---

## 5. Claim inventory (literature claims; work list for Phase 1)

Hedge levels: **F** flat statement; **H** hedged ("report", "describe", "in our reading"); **A** attributed with abstract/body distinction.
Context columns: P platform, N vehicles, Sep separation, S/HW simulation vs hardware, Tr training.

| ID | Loc | Claim (condensed) | Key(s) | Context given | Hedge |
|---|---|---|---|---|---|
| C1.01 | 1.1 L30–32 | Crazyflie 2.0 ≈9 cm rotor-to-rotor (vendor data sheet); conservative planning gap ≈60 cm cited by Shi et al. from earlier planning work | shi2020neuralswarm | P: CF2.0 | H |
| C1.02 | 1.1 L45–47 | Mean flow below a hovering quadrotor ≈ turbulent jet beyond ≈2.5 motor-to-motor distances | bauersfeld2024airflow | hover | F |
| C1.03 | 1.1 L47–48 | In the near field those authors report no closed-form model | bauersfeld2024airflow | — | H |
| C1.04 | 1.1 L51–52 | Worst-case height error reduced by factor two or better, up to four | shi2020neuralswarm | none | H |
| C1.05 | 1.1 L52–54 | Sixteen vehicles at minimum vertical separation 24 cm | shi2022neuralswarm2 | none (heterogeneous not said) | H |
| C1.06 | 1.1 L54–56 | Average reductions 56 % vertical, 36 % 3-D tracking error for a pair | smith2023so2equivariant | N=2 | H |
| C1.07 | 1.2 L77–80 | Hybrid built on a single vehicle; gain over reactive small without payload, similar with payload | cobobriesewitz2026lindi | N=1 | H |
| C1.08 | 1.2 L84–86 | Gu: online-learning estimators on a single vehicle; Abro: fractional-order and classical regulators for close-proximity swarms | gu2025comparative, abro2025fractional | N=1 / swarm | F |
| C1.09 | 1.3 L98–100 | 36 % on two vehicles in close proximity vs 31 % in the abstract of a tight-formation study | smith2023so2equivariant, hsieh2026flatness | abstract flagged for Hsieh | A |
| C1.10 | 1.3 L108–111 | Hybrid demonstrated only where residual is self-induced: single vehicle, blade flapping, drag, **ground forces**, optional slung payload | (implicit cobobriesewitz2026lindi; no cite) | N=1 | F — **uncited**; "ground forces" to check |
| C1.11 | 1.3 L113–115; 1.5 L156–157; 1.6 L193–194 | Per-neighbour summation known to be an approximation / superposition does not hold, already established | shi2020neuralswarm, gielis2023aggregate | — | F ("it does not") |
| C2.01 | 2 L33–36 | Same as C1.01 (CF2.0 9 cm; 60 cm) | shi2020neuralswarm | | H |
| C2.02 | 2.1 L74–77 | Lee: geometric tracking on SE(3), position + one heading direction, coordinate-free, avoids singularities | lee2010geometrictracking | no PDF | F |
| C2.03 | 2.1 L77–79 | Exponential stability for initial attitude errors <90°, almost-global exponential attractiveness <180° | lee2010geometrictracking | no PDF | F |
| C2.04 | 2.1 L80–83 | Attitude error function verified equation-by-equation against companion arXiv paper | lee2010geometricse3 | | F |
| C2.05 | 2.1 L87–90 | Tal & Karaman exploit flatness; track position and yaw derivatives up to fourth order; flatness-derived FF for angular rate and acceleration to follow jerk and snap | tal2018indi | | F |
| C2.06 | 2.2 L97–100 | Downwash strongly nonlinear, decays quickly with lateral offset, asymmetric | **uncited** | | F |
| C2.07 | 2.2 L103–107 | Kiran: load cells, forces and torques, mounted Crazyflies with roll and yaw locked, constant hover voltage, PIV centre-plane, single + pair; dynamic cases traverse on a rail | kiran2025downwash | P: Crazyflie; rig | F |
| C2.08 | 2.2 L107–111 | Gielis: 6-DoF exogenous forces on a multirotor on a load stand at hover thrust, up to three further vehicles flying freely; strong nonlinearities in aggregation | gielis2023aggregate | rig | F |
| C2.09 | 2.2 L111–115 | Both measure transducer load; actuation present at hover thrust; neither claims ground-truth role; **neither validates a learned model of ours or anyone else's** | kiran, gielis | | H ("in our reading") — **"neither validates a learned model of anyone else's" to check: Gielis train and evaluate learned aggregation models** |
| C2.10 | 2.2 L117–122 | Bauersfeld: ≈16 h hover data, six platforms 0.23–6.3 kg; turbulent jet beyond ≈2.5 motor-to-motor distances l; abstract says "drone-diameters" | bauersfeld2024airflow | hover | A |
| C2.11 | 2.2 L122–127 | Estimates time-averaged magnitude in hover, not instantaneous wrench; near field: no closed-form model, point to CFD | bauersfeld2024airflow | | H |
| C2.12 | 2.3 L138–141 | Smeur: inner angular-acc INDI + outer linear-acc INDI; single Parrot Bebop; Paparazzi at 512 Hz; position/velocity loops PD | smeur2017cascadedindi | P: Bebop; HW | F |
| C2.13 | 2.3 L142–143 | Indoors in/out of 10 m/s open-jet exhaust every 14 s | smeur2017cascadedindi | | F |
| C2.14 | 2.3 L143–146 | Two separate experiments: mocap downsampled to 4 Hz; outdoors GNSS ≈5.1 m/s wind | smeur2017cascadedindi | | F |
| C2.15 | 2.3 L146–148 | Tal: INDI, aggressive flight, single custom 609 g vehicle; no parametric model of aerodynamic effects required | tal2018indi | | F |
| C2.16 | 2.3 L149–150 | Mass, inertia, motor time constant, control effectiveness, throttle map still identified; drag model dropped | tal2018indi (implicit) | | F |
| C2.17 | 2.3 L152–155 | Snap tracking requires direct torque control, achieved with closed-loop motor speed control based on optical encoders | tal2018indi | | F |
| C2.18 | 2.3 L156–158 | Reconstructing applied wrench generally requires rotor-speed feedback; **Smeur et al. likewise use motor-speed measurement** | smeur2017cascadedindi | | F — to check |
| C2.19 | 2.3 L164–166 | NN generates smooth approximations of INDI's residual estimate without rotor-speed measurement | cobobriesewitz2026lindi | | F |
| C2.20 | 2.4 L177–182 | Shi 2020: nominal model + regularised permutation-invariant DNN; FF in decentralised nonlinear tracking position controller; geometric SE(3) only optional attitude inner loop; learned force vertical f_a,z; evaluated onboard STM32 | shi2020neuralswarm | | F |
| C2.21 | 2.4 L182–184 | Separate networks on 2, 3, 4 vehicles; worst-case height error ×2 or better, up to ×4 | shi2020neuralswarm | Tr | F |
| C2.22 | 2.4 L184–186 | Two-vehicle swap: baseline 9.4 cm; 4-vehicle-trained 2.4 cm; 2-vehicle-trained 2.7 cm | shi2020neuralswarm | | F |
| C2.23 | 2.4 L186–188 | Swap vertical spacings 0.20–0.25 m; 25 cm explicit for three-vehicle case | shi2020neuralswarm | Sep | F |
| C2.24 | 2.4 L188–191 | 3- or 4-vehicle-trained improve 5-vehicle flight; 2-vehicle-trained worse than baseline on 3 and 4 | shi2020neuralswarm | | F |
| C2.25 | 2.4 L193–194 | NS2: heterogeneous deep sets, any number of neighbours | shi2022neuralswarm2 | | F |
| C2.26 | 2.4 L195–197 | Trained on at most three vehicles (two neighbours); 16-robot prediction "relatively less accurate" | shi2022neuralswarm2 | Tr | A |
| C2.27 | 2.4 L197–200 | Spectral normalisation; abstract: stability and generalisation guarantees; body: Lipschitz bounds + empirical generalisation + bounded-residual assumption | shi2022neuralswarm2 | | A |
| C2.28 | 2.4 L200–201 | Learned residual used in interaction-aware motion planning and tracking | shi2022neuralswarm2 | | F |
| C2.29 | 2.4 L201–204 | 16 robots at minimum vertical distance 24 cm; separately up to ×3 worst-case tracking error reduction vs baseline nonlinear tracking with interaction force zero | shi2022neuralswarm2 | abstract | A |
| C2.30 | 2.4 L204–207 | Body on 16-robot flight: max z-error of a small vehicle 15 → 10 cm, factor ≈1.5 | shi2022neuralswarm2 | body | A |
| C2.31 | 2.4 L209–213 | Smith: SO(2)-equivariant; 5 min real data lower validation error than baselines trained on >15 min; prediction-accuracy result | smith2023so2equivariant | Tr | H |
| C2.32 | 2.4 L213–216 | Deployed in LQR on two identical vehicles; 3-D −36 %, vertical −56 % on average | smith2023so2equivariant | N=2 | F |
| C2.33 | 2.4 fn L216–219 | Internal inconsistency: contributions list calls 36 % lateral | smith2023so2equivariant | | A |
| C2.34 | 2.4 L219–221 | Lateral transect beneath hovering vehicle: mean 3-D error 0.154 → 0.098 m; vertical improvements 55 % and 49 % | smith2023so2equivariant | | F |
| C2.35 | 2.4 L222–224 | Custom quadrotor ≈0.26 m span, 0.7 kg; OptiTrack; Raspberry Pi at 50 Hz; network 45 Hz | smith2023so2equivariant | P | F |
| C2.36 | 2.4 L224–227 | Closest training stage ≈0.5 m (≈two body-lengths); evaluation 0.6–0.8 m; lemniscate beneath at z = −2.5 m | smith2023so2equivariant | Sep | F |
| C2.37 | 2.4 L227–228 | Authors attribute efficiency to exploiting latent geometry | smith2023so2equivariant | | H |
| C2.38 | 2.4¶ L236–237; 3.4.2 L249–250 | With more than two vehicles the effect is not a simple superposition of each pair | shi2020neuralswarm | | H |
| C2.39 | 2.4¶ L237–239; 3.4.2 L250–253 | Gielis motivation: accurate single-vehicle downwash model unlikely to suffice for aggregate | gielis2023aggregate | | H |
| C2.40 | 2.4¶ L239–241 | Abstract frames as graph; implementation sum-pooling deep-sets predictor vs linear-summation baseline | gielis2023aggregate | | A |
| C2.41 | 2.4¶ L241–244 | Generalisation to a fourth vehicle claimed in abstract; tables stop at three neighbours; conclusion pessimistic | gielis2023aggregate | | A |
| C2.42 | 2.4¶ L246–252 | Three regimes: chaotic/noise-dominated; linear (well estimated by summation); nonlinear | gielis2023aggregate | rig | H |
| C2.43 | 2.4¶ L254–257 | Summed architectures remain in wide use; penalty size for deployed model not reported | (gap claim, uncited) | | F |
| C2.44 | 2.5 L268–271 | Simplest injection adds predicted force to the desired force of a nonlinear tracking controller | shi2020, shi2022 | | F |
| C2.45 | 2.5 L274–281 | Yang: lower-triangular residuals preserve local flatness and original flat output for pure-feedback systems; constructive diffeomorphism recovery; residual is a structured correction; worked example only in velocity equation | yang2025flatnessresiduals | | F |
| C2.46 | 2.5 L281–283 | Validation in simulation on planar 2-D quadrotor; no hardware; physical quadrotor listed as future work | yang2025flatnessresiduals | S | F |
| C2.47 | 2.5 L283–287 | Hsieh 2026: physics-informed residual keeps joint system flat; FBL cancels disturbance by feedforward | hsieh2026flatness | | F |
| C2.48 | 2.5 L287–291 | Abstract: 31 % avg tracking error reduction vs nominal baselines; body hardware: 24 % RMSE, 30 % max-z ROM-FBL over FBL; further 29 % RMSE, 43 % max-z learned FBL over ROM-FBL | hsieh2026flatness | A | A |
| C2.49 | 2.5 L293–296 | On hardware they match ≈5 cm RMSE and 10 cm max-z **published** by Chee rather than same rig; in simulation flatness controller's max-z can be worse than NMPC | hsieh2026flatness, chee2024knodedwmpc | HW vs S | A |
| C2.50 | 2.5 L298–301 | 20-fold solve-time reduction (≈1 vs 27 ms at equal residual) is a simulation figure; abstract and intro: order of magnitude less computation; training 28 s HW, 18 s sim | hsieh2026flatness | S | A |
| C2.51 | 2.5 L303–309 | Hardware: two CF2.1 with thrust-upgrade bundles; controller off-board on Intel i5 laptop; Crazyradio at 400 Hz; 5 ms loop is abstract-level off-board figure | hsieh2026flatness | P, HW | A |
| C2.52 | 2.5 L311–313 | Hardware compensates residual forces only (thrust + angular velocity), torques to rate controller; simulation models residual torques | hsieh2026flatness | | F |
| C2.53 | 2.5 L318–319 | Chee: learned component inside prediction constraint of NMPC | chee2024knodedwmpc | | F |
| C2.54 | 2.5 L319–320 | Li: predicted downwash force sequence injected into optimiser's prediction model | li2023ndpnmpc | | F |
| C2.55 | 2.5 L320–321 | Hsieh 2025: L1 online estimate added to same family | hsieh2025onlineadaptation | | F |
| C2.56 | 2.5 L321–323 | Chee and Hsieh 2025 run optimisation off-board or on a companion computer; Li on TX2 NX carried by vehicle; **none runs on a Crazyflie-class vehicle** | chee, li, hsieh2025 | | F — wording ambiguous (Chee flies Crazyflies) |
| C2.57 | 2.5 L330–338 | Shankar: learned lumped downwash force online in infinite-horizon LQR on feedforward-linearised follower beneath inner attitude loop; vertical docking within 1–2 body lengths; two custom ≈0.27 m diagonal, 0.7 kg; dock at ≈0.36 m vertical; curriculum to ≈0.5 m | shankar2023docking | P, HW | F |
| C2.58 | 2.5 L338–339 | SO(2)-equivariant model of prior work, trained offline from ≈5 min, queried online on onboard Raspberry Pi; mocap; hardware only | shankar2023docking | Tr | F |
| C2.59 | 2.5 L339–342 | Abstract: terminal error <0.06 m; body: vertical typically <0.1 m hovering leader; moving leader <0.05 m with model vs >0.2 m without | shankar2023docking | A | A |
| C2.60 | 2.5 L342–345 | Physical docks 8/10 vs hovering leader at 0.36 m; moving leader at 0.25 m/s limited success, attributed to gripper latency | shankar2023docking | | F |
| C2.61 | 2.6 L353–356 | Zhang: residual RL on cascaded controller, compensating disturbance and thrust loss from downwash, no explicit force estimate | zhang2024proxfly | | F |
| C2.62 | 2.6 L357–359 | Policy observes only tracking error, baseline command, own previous action; no neighbour state, no communication | zhang2024proxfly | | F |
| C2.63 | 2.7 L378–380 | LINDI: NN replaces INDI's residual estimate, trained on smoothed (spline) data | cobobriesewitz2026lindi | | F |
| C2.64 | 2.7 L380–382 | NA-INDI: f_a = f_a,NN + f_a,INDI | cobobriesewitz2026lindi | | F |
| C2.65 | 2.7 L382–384 | Single quadrotor, with/without slung payload; figure-eight and circle, neither in training set | cobobriesewitz2026lindi | N=1 | F |
| C2.66 | 2.7 L386–390 | Headline concerns sensing; rotor speeds needed for training labels; NA-INDI retains incremental branch | cobobriesewitz2026lindi | | H |
| C2.67 | 2.7 L390–392 | No payload: ≈40 % reduction vs geometric baseline; payload: 19–24 % | cobobriesewitz2026lindi | | F |
| C2.68 | 2.7 L393–395 | Hybrid best without payload, small margin; with payload similar to INDI | cobobriesewitz2026lindi | | H |
| C2.69 | 2.7 L395–398 | Slight drop on circular trajectory (LINDI, no payload), attributed to OOD | cobobriesewitz2026lindi | | H |
| C2.70 | 2.7 L400–402 | Residual self-induced: blade flapping, drag, payload swing; predicted from own state | cobobriesewitz2026lindi (implicit) | | F |
| C2.71 | 2.8 L411–419 | Recap numbers (Shi ×4, NS2 ×3, Smith 36/56, Hsieh 31, Cobo 40) | multiple | | A |
| C2.72 | Tab 2.1 caption | No two rows share platform, team size, separation and trajectory class | all | | F |
| C2.73 | Tab 2.1 rows | 14 rows of team size / separation / reported quantity (see Ch2 L435–448) | each | | F |
| C2.74 | Tab 2.1 fn ‡ | Hsieh 2026 HW comparisons at commanded Δz 0.3 and 0.4 m; 0.2–0.5 m range is simulation/prediction | hsieh2026flatness | | A |
| C2.75 | 2.8 L481–490 | Gu: four online-learning estimators (ELM, RBF, ESN, GP), modular framework with swappable residual module; lumped additive wrench via inverting discrete Newton–Euler; accuracy, tracking, compute; GP most accurate but cubic training; parametric fast but initialisation-sensitive; ESN most balanced | gu2025comparative | S | F |
| C2.76 | 2.8 L492–498 | Single quadrotor, generic uncertainty, all online; simulation throughout; hardware = learning modules on embedded computer in loop with simulator (latency); real quadrotor future work | gu2025comparative | S | F |
| C2.77 | 2.8 L501–504 | Gu tune ELM, RBF, ESN to their optima by Bayesian optimisation under consistent conditions | gu2025comparative | | F — **comment `% UNVERIFIED (review P-010): no PDF of Abro` sits above this Gu paragraph (misplaced)** |
| C2.78 | 2.8 L510–518 | Abro: FO-PID, FO-LQR, integral state feedback; close proximity; downwash and safety-distance motivation; only integral state feedback classical; feedback regulators, no predictor | abro2025fractional | **no PDF** | F |
| C3.01 | 3.2 L93–94 | Neural-Swarm compensates the vertical interaction force | shi2020neuralswarm | | F |
| C3.02 | 3.2 L94–96 | Hsieh hardware controller commands thrust and angular rates, residual torque to rate loop | hsieh2026flatness | HW | F |
| C3.03 | 3.2 L96–97 | Kiran and Gielis measure both force and moment, on a rig | kiran, gielis | rig | F |
| C3.04 | 3.3.1 L139–141 | Cobo-Briesewitz remove rotor-speed prerequisite by learning the residual | cobobriesewitz2026lindi | | F |
| C3.05 | 3.4.1 L217–221 | Family = NS and NS2 deep sets; not Cobo-Briesewitz's network; not a copy of NS2's layers | shi2020, shi2022, cobo | | F — **"not a copy of NS2's layers" is now false per code (§14a)** |
| C3.06 | 3.4.2 L249–253 | = C2.38, C2.39 (load stand) | shi2020, gielis | | H |
| C3.07 | 3.4.3 L281–287 | Gu (simulation): RBF crashes under periodic wind; ELM unstable on landing under ground effect; ESN, GP did not; failure "may be attributed" to random input-to-hidden initialisation "can produce excessively large prediction values" | gu2025comparative | S, N=1 | A (quotation) |
| C3.08 | Tab 3.1 | Superposition known imperfect | shi2020, gielis | | F |
| C3.09 | 3.5 L358–360 | Cobo-Briesewitz: drop on circle for purely learned method, no payload, OOD | cobobriesewitz2026lindi | | H |
| C4.01 | 4.1.2 L129–133 | Our torque law is reduced form of Lee; drops angular-FF bracket, adds integral; symbol mapping | lee2010geometrictracking | | F |
| C4.02 | 4.2 L158–162 | Lee tracks position + one heading, four outputs; e_R = ½(R_dᵀR − RᵀR_d)^∨ verified against the source | lee2010geometrictracking | no PDF (companion) | F |
| C4.03 | 4.2 eq (ctrl-tau-lee) | Lee torque law M = −k_R e_R − k_Ω e_Ω + Ω×JΩ − J(Ω̂RᵀR_dΩ_d − RᵀR_dΩ̇_d), e_Ω = Ω − RᵀR_dΩ_d | lee2010geometrictracking | companion check | F |
| C4.04 | 4.2 L180–183 | Ψ(0)<1 exponential stability (<90°); almost-global exponential attractiveness for 1 ≤ Ψ < 2 (<180°) | lee2010geometrictracking | no PDF | F |
| C4.05 | 4.3 L233–235 | Incremental formulation + flatness FF follows Tal & Karaman; cascaded position/attitude INDI follows Smeur | tal2018indi, smeur2017 | | F |
| C4.06 | 4.3 L238–239 | Tal & Karaman obtain rotor speed from optical encoders | tal2018indi | | F |
| C4.07 | 4.4 L266–268 | Predicting interaction from relative states with permutation-invariant network as FF follows Neural-Swarm line | shi2020, shi2022 | | F |
| C4.08 | 4.4 L272–276 | NS2 applies spectral normalisation to obtain stability and generalisation guarantees; NS2 uses residual in planning and control | shi2022neuralswarm2 | | F — **drops the abstract/body hedge of C2.27** |
| C4.09 | 4.5 L291–298 | = C2.45–C2.47 (Yang theory in sim on planar quadrotor; HW demo is Hsieh's) | hsieh2026, yang2025 | | F |
| C4.10 | 4.5 L301–308 | Abstract 31 %; body HW 24 % RMSE ROM-FBL over FBL, further 29 %; **"In simulation they match the tracking error published by Chee et al. for NMPC at an order of magnitude less computation, from under 30 s of training data, at a 5 ms loop rate obtained off-board"** | hsieh2026flatness | | **Contradicts C2.49** (Ch2: the match is on hardware vs Chee's published numbers; in simulation max-z can be worse than NMPC) — S1 consistency |
| C5.01 | 5.6 L310–311 | Simulator uses "a published downwash model" for the interaction force | **uncited** | S | F — missing citation (S2) |
| C5.02 | 5.6 L317–320 | Interaction force in simulation produced by a learned model of the same family as the one being fitted | (self) | | F |

Total: 13 Ch1 rows, 78 Ch2 rows, 9 Ch3 rows, 10 Ch4 rows, 2 Ch5 rows (several rows bundle 2–4 sentences with the same key and context).

---

## 6. Number inventory

Literature numbers are listed in §5 with their claims; this table lists every number with its context and every repetition, flagging non-identical repeats. **Bold** = mismatch or stale.

| Number | Context | Locations | Repeats identical? |
|---|---|---|---|
| ≈9 cm rotor-to-rotor | Crazyflie 2.0, vendor data sheet | 1.1 L30; 2 L33 | yes |
| 9 cm rotor-to-rotor | **our Crazyflie 2.1 Brushless** | 5.1 L46; 3.4.3 L275 ("vehicles are 9 cm across") | **unsourced for the Brushless; code uses arm length 0.050 m (`lib.rs:261`, ARM_M = √2/2·0.050), i.e. 0.10 m motor-to-motor diagonal — VERIFY** |
| 60 cm | conservative planning gap, cited by Shi 2020 | 1.1 L31; 2 L35 | yes |
| 2.5 motor-to-motor distances | Bauersfeld jet onset | 1.1 L46; 2.2 L120 | yes |
| 16 h, 6 platforms, 0.23–6.3 kg | Bauersfeld | 2.2 L118 | — |
| ×2 or better, up to ×4 | Shi 2020 worst-case height error | 1.1 L51; 2.4 L183; 2.8 L412 | yes |
| 9.4 / 2.4 / 2.7 cm | Shi 2020 swap | 2.4 L185–186 | — |
| 0.20–0.25 m; 25 cm | Shi 2020 spacing | 2.4 L187; Tab 2.1 | yes |
| 16 robots, 24 cm | NS2 | 1.1 L53; 2.4 L202; Tab 2.1 | yes |
| ×3 | NS2 worst-case tracking | 2.4 L203; 2.8 L413; Tab 2.1 fn | yes |
| 15 → 10 cm, ×1.5 | NS2 body 16-robot | 2.4 L206; Tab 2.1 fn | yes |
| 5 min vs >15 min | Smith data efficiency | 2.4 L211 | — |
| 36 % 3-D, 56 % vertical | Smith | 1.1 L55; 1.3 L98; 2.4 L215; 2.8 L415, L462 | yes |
| 0.154 → 0.098 m; 55 %, 49 % | Smith transect | 2.4 L220–221 | — |
| 0.26 m, 0.7 kg, 50 Hz, 45 Hz | Smith platform | 2.4 L222–224 | — |
| ≈0.5 m train; 0.6–0.8 m eval; z = −2.5 m | Smith | 2.4 L225–227; Tab 2.1 | yes |
| 31 % | Hsieh 2026 abstract | 1.3 L100; 2.5 L287; 2.8 L417, L463; 4.5 L301 | yes |
| 24 % / 30 %; 29 % / 43 % | Hsieh 2026 HW | 2.5 L288–291; 4.5 L302–303 (only 24 %, 29 %) | consistent subset |
| ≈5 cm RMSE, 10 cm max-z | Chee published, matched by Hsieh | 2.5 L293–294 | — |
| 20-fold; 1 vs 27 ms | Hsieh sim | 2.5 L298–299 | — |
| order of magnitude | Hsieh abstract/intro | 2.5 L300; 4.5 L305 | yes |
| 28 s HW, 18 s sim | Hsieh training data | 2.5 L300–301 | **4.5 L305 says "under 30 s" in a sentence that attributes it to simulation — context drift** |
| 400 Hz; 5 ms | Hsieh off-board loop | 2.5 L306–307; 4.5 L306 | 400 Hz vs "5 ms loop" are different numbers (2.5 ms vs 5 ms) — to check in PDF |
| 512 Hz; 10 m/s; 14 s; 4 Hz; 5.1 m/s | Smeur | 2.3 | — |
| 609 g | Tal | 2.3 L148 | — |
| 0.27 m, 0.7 kg, 0.36 m, 0.5 m, 0.06/0.1/0.05/0.2 m, 8/10, 0.25 m/s | Shankar | 2.5 L334–345; Tab 2.1 | yes |
| ≈40 %; 19–24 % | Cobo-Briesewitz | 2.7 L391–392; 2.8 L419 | yes |
| 90°, 180° | Lee regions | 2.1 L78–79; 4.2 L182–183 | yes |
| 0.3 / 0.4 m; 0.2–0.5 m | Hsieh 2026 separations | Tab 2.1 | — |
| 0.2–0.4 m; ≈0.08 m | Chee | Tab 2.1 | — |
| 0.2 m pairwise | Hsieh 2025 | Tab 2.1 | — |
| 0.50 m; 0.10; 0.05 m | Zhang | Tab 2.1 | — |
| 157 works | citation-graph scan | 2.9 todo | — (provenance) |
| ≈41 g | our vehicle mass | 5.1 L45 | matches study yaml `mass: 0.041` (`crazyflies.yaml:653`); firmware compiled default 0.0364 (`traj_iface.c:297`) is overridden |
| **500 Hz control loop** | our loop | 5.1 L66 | **code: controller runs at 1000 Hz** (`stabilizer.c:303,350`; `stabilizer_types.h:383`; `lib.rs:2351`) |
| 500 Hz onboard logging | uSD rate | 5 L29; 5.4.1 L190; 5.7 L343, L352; 5.8 L369; 6.1 | yes; matches `usd_study_config.txt` line 6 |
| 100 Hz mocap | EKF input | 5.1 L66 | not checkable in the listed code (fact sheet) |
| **35 variables** | uSD | 5.4.1 L190; 5.8 L368 | **code: 96 variables** (`usd_study_config.txt`, lines 8–103) |
| ≈800 packets/s; ceiling ≈1000 | radio telemetry | 5.4.1 L186–187 | **project-derived number, no source (H1-suspect)** |
| 2 × 4 × 1.7 m | tracked volume | 5.2 L72–73; 5.8 L367–368 | yes; open todo (tape-measure) |
| 0.30 m floor; 0.10 m near-ground | operational floors | 5.2 L75, L78 | match `safety.py:53,60` |
| ≈10 cm from a wall | scenario extremes | 5.2 L83 | design check (not checked here) |
| 16 scenarios | library | 5.3 L94; 5.8 L367 | matches `scenarios.py:467–469` |
| 6 network inputs | per neighbour | 5.3.2 L166 | matches `residual_nn.rs:91`; **but the network also has a 4-input ground branch (`residual_nn.rs:92`)** |
| 15 cm RMS | merge-tool abort threshold | Tab 5.2 | internal tool detail |
| a few milliseconds | predicted alignment | 5.4.3 L220 | stale status |
| **2 m neighbour cut-off** | input guard | 3.4.3 L268 | **code: no distance cut-off; gate \|Δx\|<0.2 m, \|Δy\|<0.2 m, \|Δv_x\|<1.5 m/s (`residual_nn.rs:139–140, 281–283`)** |
| **4 cm minimum-separation clamp** | input guard | 3.4.3 L273 | **code: no clamp (removed, `residual_nn.rs:40–44`)** |
| 8 m/s² | output bound | 3.4.3 L279 | matches `residual_nn.rs:134` (`OUT_CLAMP`), applied to the scalar vertical output |
| **987 parameters** | network size | 3.4.1 L219, L227; 4.4 L271 | **code: 19 297 weights (`residual_nn.rs:65,125`)** |
| **φ 6→16→16→8, ρ 8→16→16→3** | layer widths | 3.4.1 L224–225 | **code: φ_S 6→25→40→40→20, φ_G 4→25→40→40→20, ρ_S 20→40→40→40→1 (`residual_nn.rs:17–18, 91–100`)** |
| **3-D output** | ρ output | 3.4.1 L225; eq (model-deepsets) "residual acceleration in world frame" | **code: scalar vertical force, x,y ≡ 0 (`residual_nn.rs:38, 100, 302`)** |
| "at the control rate", "within the control period" | network rate | 3.4.1 L227; 4.4 L254 | **code: evaluated every `rnn.div` = 10 ticks (≈100 Hz) and held (`traj_iface.c:132`; `lib.rs:884–903`)** |
| two neighbours (three-robot team) | neighbour count | 3.4 L197 | code limit `MAX_NEIGHBOURS = 3` (`residual_nn.rs:129`) |
| ≈2× | sign-error consequence | 3.3.2 L179–180 | derived prediction, not a result |

---

## 7. Notation table

| Symbol | Meaning | First use | Defined | Conflict |
|---|---|---|---|---|
| $p_i, v_i$ | position, velocity | 2.1 eq (rw-trans) | 3.1 | used in Ch2 before Ch3 defines it (Ch2 says "developed in Ch3") — acceptable with forward pointer |
| $R_i$ | body-to-world rotation | 2.1 | 3.1 | — |
| $\omega_i$ | body angular velocity | 2.1 | 3.1 | Lee's $\Omega$ mapped in 4.1.2 |
| $\hat{\cdot}$ | hat map | 2.1 ($\hat\omega_i$) | 3.1 | **also used for "predicted" ($\hat a_{\mathrm{res}}$, 3.4.1 onward) — same accent, two meanings (S2)** |
| $(\cdot)^\vee$ | vee map | 4.1.2 | **never defined** | S2 |
| $m, J$ | mass, diagonal inertia | 2.1 | 3.1 | — |
| $g, e_3$ | gravity, third basis vector (up) | 2.1 | 2.1 / 3.1 | — |
| $f_i, \tau_i$ | collective thrust, body torque | 2.1 | 2.1, 3.2 | $\tau$ in Ch4 also for $\tau_{\mathrm{cur}}$; fine |
| $f_{\mathrm{res},i}, \tau_{\mathrm{res},i}$ | residual force, moment | 2.1 | 3.2 | Ch2 uses $f_a$, $f_{a,z}$, $f_{a,\mathrm{NN}}$, $(f_a,\tau_a)$ for the papers' symbols — not mapped to $f_{\mathrm{res}}$ (S3) |
| $\varpi_k$ | rotor speed | 3.1 | 3.1 | unit not stated (code: RPM, `traj_iface.c:285`) |
| $k_{t,k}$; $k_t$ | per-rotor thrust coefficient; unsubscripted in 3.2, 3.3.1, Tab 3.1 | 3.1 | 3.1 | **two forms ($k_{t,k}$ vs $k_t$) without saying $k_t$ means "the set"** (S3) |
| $a_{\mathrm{meas}}, a_{\mathrm{model}}$ | measured / model acceleration | 3.3 eq (model-ares) | 3.3 | — |
| $a_{\mathrm{res}}$ | residual acceleration $= f_{\mathrm{res}}/m$ | 3 intro | 3.3 (implicitly via underbrace) | defined as a quantity in the intro before its equation (S3) |
| $\hat a_{\mathrm{res}}$ | predicted residual | 3.4.1 | 3.4.1 | hat conflict (above) |
| $a_{\mathrm{des}}$ | trajectory-demanded acceleration, no gravity, no residual | 3.3.2 | 3.3.2, 4.1.1 | — |
| $a_{\mathrm{cmd}}$ | commanded acceleration incl. gravity and residual terms | 4.1.1 | 4.1.1 | — |
| $a_{\mathrm{ff}}$ | trajectory feedforward acceleration | 4.1.1 | 4.1.1 (underbrace only) | — |
| $K_p, K_v, K_i$ | position gains | 4.1.1 | 4.1.1 | $K_i$ full-vector integral (code: disabled; vertical-only integral with its own gain) |
| $e_p, e_v$ | position, velocity error | 4.1.1 | 4.1.1 | sign convention $e_p = p_{\mathrm{des}} - p$ (desired minus actual) vs Lee's $e_x = x - x_d$ — not mentioned (S3) |
| $R_{\mathrm{des}}$ vs $R_d$ | desired attitude | 4.1.1 / 4.2 | 4.1.1 | **two symbols for one quantity** ($R_{\mathrm{des}}$ in 4.1, $R_d$ in 4.2 and eq ctrl-tau-lee) (S3; partly Lee's notation) |
| $e_R, e_\omega$ | attitude and rate error | 4.1.2 | 4.1.2 | $e_\omega = \omega - \omega_{\mathrm{des}}$ (ours) vs Lee's $e_\Omega = \Omega - R^\top R_d\Omega_d$ — **difference not stated** (code: `lib.rs:2221`, no transport term) |
| $\omega_{\mathrm{des}}$ | desired body rate | 4.1.2 | 4.1.2 | — |
| $K_R^{\mathrm{geo}}, K_\omega^{\mathrm{geo}}$ | geometric gains → torque | 4.1.2 | 4.1.2 | units not stated (code: N·m per rad, N·m·s per rad; `traj_iface.c:266–280`) |
| $k_{i,\mathrm{att}}$ | attitude integral gain | 4.1.2 | eq only | **not defined in words**; code value 0 (disabled) |
| $M, k_R, k_\Omega, \Omega, \Omega_d, e_\Omega$ | Lee's symbols | 4.2 | 4.1.2 mapping, 4.2 | OK |
| $\Psi$ | attitude error function | 2.1 (implicit) / 4.2 | 4.2 ("the attitude error function Ψ") | formula never given (S3) |
| $\alpha_{\mathrm{ref}}, \alpha_{\mathrm{des}}, \alpha_{\mathrm{meas}}$ | angular-acceleration reference / FF / measured | 4.3 | 4.3 | — |
| $K_R^{\mathrm{INDI}}, K_\omega^{\mathrm{INDI}}$ | INDI gains → angular acceleration | 4.3 | 4.3 | units stated qualitatively |
| $\tau_{\mathrm{cur}}$ | torque from measured rotor speeds | 4.3 | 4.3 | mapping from rotor speeds to torque (arm, drag ratio) not given |
| $\mathcal{F}$ | interaction map | 3.4 | 3.4 | — |
| $\phi, \rho$ | deep-sets networks | 3.4.1 | 3.4.1 | code has two $\phi$ ($\phi_S$, $\phi_G$) |
| $W_1, b_1, \mu, \sigma, \oslash, x$ | normalisation fold | 3.4.4 | 3.4.4 | $x$ (network input) vs Lee's $x$ (position) — Lee's $x$ never used, OK |
| $l$ | motor-to-motor distance (Bauersfeld) | 2.2 | 2.2 | — |
| $f_a, f_{a,z}$ etc. | papers' residual symbols | 2.4, 2.7, 2.8 | in passing | see above |

---

## 8. Terminology list

| Term | Defined where | Used before definition? |
|---|---|---|
| downwash | 1.1 L28 (parenthetical) | no |
| residual / residual force | 1.2 L65 (implicitly) ; 3.2 formally | yes — "residual" used throughout Ch1–2 informally; acceptable with forward pointer |
| interaction force | never defined as distinct from residual | used from title onward (S3) |
| INDI | 1.4 L126 (spelled out) | no |
| geometric tracking controller / SE(3) | 2.1 (no definition of SE(3), SO(3)) | **SE(3) never defined** (S2 for the student reader) |
| differential flatness / flat output | 2.1 L85–87 (informal) | definition given informally; "flat output" used in 2.5 without definition |
| feedback linearisation (FBL) | 1.4 (abbrev.) | **never defined** (S2) |
| deep sets / permutation invariance | 2.4 (in passing), 3.4 | partly; "deep sets" term never explained before 3.4.1 |
| latent space / embedding | 3.4.2 L246 | used in 2.4 (superposition ¶) before defined |
| spectral normalisation | 2.4 L197 | never defined (S3) |
| Lipschitz | 2.4 L199 | never defined (S3) |
| LQR, NMPC, MPC | 2.4 L214 (LQR spelled out); 2.5 L296 NMPC spelled out; "MPC" heading L315 "model predictive control" spelled out at L296 | OK |
| ROM-FBL | 2.5 L289 (spelled out) | no |
| L1 adaptation | 2.5 L321 ($\mathcal L_1$) | never explained (S3) |
| PIV | 2.2 L105 spelled out | no |
| CFD | 2.2 L123 spelled out | no |
| RMSE | 2.5 L289 | **never spelled out** (S3) |
| EKF | 5.1 L65 spelled out | no |
| DShot | 5.1 L56 parenthetical | no |
| uSD | 5.1 L62 | no |
| OptiTrack | 5.0 L29 (brand), 6.1 | brand name, never introduced |
| null scenario | 3.2 L83 ("coplanar control scenarios"), 3.3.1 L154 ("null scenarios") | defined in 5.3.1 — used in Ch1 ("null flights") and Ch3 before Ch5 (forward use; S3) |
| measurement floor | 1.6 L186 | never defined operationally (S2) |
| hardware gate | 2.8 L504 | defined in 5.5.1 — forward use in Ch2/Ch4 (S3) |
| transfer penalty | 1.5 RQ3 | defined by RQ3 itself; no metric (S2) |
| reactive / predictive | 1.2 | fine |
| superposition | 1.3 | 2.4¶ / 3.4.2 | OK |
| "Strategy 0–3" | 1.4 | OK |
| "C.1", "C.4" | Ch5, Ch6–9 | **internal vocabulary** (never defined; not thesis terms) |
| "cf5", "cf_second" | Ch5 status ¶, Ch6.1, Ch9 | **internal vocabulary** |
| "S0–S3" | Ch6 tables | internal shorthand (S3) |

---

## 9. Cross-reference check

- **Undefined labels:** none (every `\ref`/`\eqref` target exists in the combined build).
- **Duplicate labels:** none.
- **Labels never referenced:** `eq:ctrl-alpharef`, `eq:model-fold`, `eq:rw-rot`, `eq:rw-trans`, `tab:intro-scope` (**table never referenced in text — style rule: every table referenced**), `tab:setup-scenarios` (**never referenced**), many section labels (normal).
- **Label placement:** `\label{sec:rw-superposition}` (2.4) and `\label{sec:rw-existing-comparisons}` (2.8) are placed before `\paragraph`; they resolve to the enclosing section number (2.4, 2.8), which is what the text needs ("§2.8"). OK.
- **Forward references from Ch1–5 into Ch6–9 by `\ref`:** Ch1 L271–273 (`ch:results2`, `ch:results3`, `ch:discussion`, `ch:conclusion`). H6 says name them in prose until they exist; they exist as skeleton files and resolve in the combined build — flag as S3, keep `\ref` (the build resolves) unless the author wants prose.
- **Promises "the X chapter returns to…":** 3.2 (moment residual → discussion; **no Ch8 slot**), 3.0 (assumptions → discussion; no slot), 3.4.2 (penalty → multi-robot chapter; Ch7.2 ✓), 3.3.2 (sign prediction → two-robot chapter; **no Ch6 slot**), 4.6 (frozen gain → discussion; partial), 4.4 (prediction on/off flights → Ch5 schedules; **not scheduled**), 2.4¶ (penalty → multi-robot chapter ✓), 4.5 (S3 absence → discussion ✓ implicit).
- **Citations used but not in bib:** none. **Bib entries never cited:** none (all 20 cited).
- **Bib-entry issues (suspected, Phase 1 to confirm):** `shi2022neuralswarm2` `year = 2020` while the key says 2022 and `yearnote` says cite T-RO 2022 (the `plain` style will print 2020); `cobobriesewitz2026lindi` `year = 2025` vs key/yearnote L4DC 2026; all arXiv entries are `@misc` with `howpublished` — published venues (T-RO, RA-L, L4DC, Control Eng. Practice) not used as the citation; `gu2025comparative` note "full text not held" although `docs/papers/gu2025comparative.pdf` exists; `bauersfeld2024airflow` note says RA-L — year of RA-L publication to check; `tal2018indi` yearnote mentions a later revision — the held PDF version to check; non-standard fields `file`, `yearnote` ignored by BibTeX (harmless).

---

## 10. Stale-scope scan

| # | Location | Text (excerpt) | Class | Sev |
|---|---|---|---|---|
| SS-01 | ch4 header comment L21–30 | "Strategy 1 = Pure INDI … Strategy 7 = Geometric + Residual RL … seven-method count … Strategy 6 (MPC) …" | strategies above 3 (comment only, not printed) | S3 |
| SS-02 | ch6 §6.5 L139–149 | "Hybrid (NA-INDI), if in scope … (Strategy 4 in the wider project numbering)" | strategy above 3 printed in body | **S1** |
| SS-03 | ch9 §9.4 L71–77 | "Retrained NA-INDI / LINDI reference. Our notes describe a retrain path…"; "Residual reinforcement-learning strategies. Explicitly deferred in the project convention" | out-of-scope strategies as plans; "our notes" | S2 |
| SS-04 | ch5 L28–39 | "Campaign status (September 2026). Two-robot flights … have been executed on the brushless pair cf5 … cf_second … On 2026-09-21, eight verified geometric merges were archived … manifest c1_2026_09_21_merged/manifest_2026_09_21_c1.json … Indicative Strategy 0 vs 1 comparisons exist … (see docs/25 coverage table)" | **H9 lab status; internal file names, dates, vehicle nicknames, C.1/C.4** | **S1** |
| SS-05 | ch4 Tab 4.1 "Status" column | "flying", "implemented, unflown", "not implemented" | **H9** | **S1** |
| SS-06 | ch4 L46 caption | "implementation status of each at the time of writing" | H9 | S2 |
| SS-07 | ch4 §4.7 L350–352 | "At the time of writing, two strategies fly (the baseline and pure INDI), one is implemented and unflown…" | **H9** | **S1** |
| SS-08 | ch4 §4.5 L286–289 | "The controller code was requested from the authors and had not been received at the time of writing." | H9 (project status, past tense) | S2 |
| SS-09 | ch5 L174–177 | "This was established in simulation before any flight: a dry run collecting only a vertical stack produced training data in which two of the six network inputs never varied…" | **H1 (project simulation finding)** | **S1** |
| SS-10 | ch5 §5.6 L311–317 | "It was used throughout development … It established that every scenario in the library executes … under both controllers, and it exposed two defects in flight code that single-vehicle testing could not have found." | **H1 (project simulation findings)** | **S1** |
| SS-11 | ch5 L185–188 | "Radio telemetry saturates at approximately two vehicles: each drone transmits on the order of 800 packets per second … ceiling near 1000" | H1-suspect (project lab-derived numbers, unsourced) | S2 |
| SS-12 | ch5 L219–221 | "The predicted alignment is a few milliseconds; that figure is to be confirmed against real logs on the first two-vehicle flight." | H9 stale status | S2 |
| SS-13 | ch5 L289–290 | "The logging configuration … records attitude but not $e_R$ itself; that signal will be added before the comparison campaign." | **stale design fact** (code logs `indi.e_r_*`, `usd_study_config.txt` lines 56–59; `lib.rs:1975–1976`) | S1 |
| SS-14 | ch5 L190, L368 | "35 variables" | stale design fact (96) | S1 |
| SS-15 | ch5 L66 | "The control loop runs at 500 Hz onboard." | stale design fact (1000 Hz) | S1 |
| SS-16 | ch3 L217–229, L268–277; ch4 L270–272 | 987 parameters, 6→16→16→8 / 8→16→16→3, 2 m cut-off, 4 cm clamp, "not a copy of NS2's layers", "of our own sizing" | stale design facts (network) | **S1** |
| SS-17 | ch3 L227; ch4 L254 | network "evaluated onboard within the control period" / "at the control rate" | stale (≈100 Hz) | S1 |
| SS-18 | ch5 Tab 5.2 L343 | "analysis uses merged uSD and firmware `ctrltarget.*` where available … merge tool aborts above 15 cm RMS in its role check" | internal vocabulary (log-variable names, tool behaviour) | S2 |
| SS-19 | ch5 Tab 5.2 L344 | "Pre-takeoff EKF-vs-mocap gate; role verification from merge" | internal procedure vocabulary; acceptable if defined | S3 |
| SS-20 | ch5 header comment L9–12; L374–375 todo | "Hardware flights began September 2026 (C.0/C.4 indicative runs…)"; "Continue revising to past tense … as C.4 repeats land" | internal workflow vocabulary; comment/todo | S3 (todo is an open author item, keep) |
| SS-21 | ch6 §6.1 L28–48 | "The comparison flights use the brushless pair cf5 (study vehicle…) and cf_second (top), with … four-camera OptiTrack … the C.1/C.4 matrix prioritises … Indicative single flights and partial C.1 collection exist" | **H9 + internal vocabulary**; "four-camera" lab fact | **S1** |
| SS-22 | ch6 L81, L131, L134 | "source: the analysis scripts on the phase metrics"; "`firmware_forward` replay"; "(`full_bank_*.npz` provenance)"; `effort_proxy`, `motor_*`, `min_sep_m`, `min_dz_m` | internal tool and file names | S2 |
| SS-23 | ch6 L86–89 | "contrast geometric tracking (Strategy 0), our INDI implementation (Strategy 1), and any reference INDI flights included for context" | **H5 risk** (alludes to several INDI implementations) | S2 |
| SS-24 | ch6 Tab 6.1–6.3 | "S0", "S1 (INDI)", "S2 (pred.+geo)", "S3 (pred.+FBL)" | internal shorthand | S3 |
| SS-25 | ch7 L29–31 | "require the third brushless vehicle in the roster" | lab status | S2 |
| SS-26 | ch7 §7.4 L74–76 | "the summed-architecture decision rests on the magnitude of open-loop penalty and simulation sanity checks" | H1 risk (sim used as evidence) | S2 |
| SS-27 | ch8 §8.6 L116–118 | "any sim-only diagnosis (e.g. backend choice for SIL) is mentioned only as implementation context" | internal vocabulary (SIL backend) | S3 |
| SS-28 | ch8 L84–86 | "cost includes RPM sourcing, SD logging, upload vs flash-resident weights, … C.1 collection burden for Strategy 2/3" | internal vocabulary | S3 |
| SS-29 | ch9 §9.4 L62–80 | "C.1 coverage and C.4 repeats … cf5 … rnn.en=1 flight logs on the brushless pair … SIL and desk parity" | internal vocabulary + lab status | S2 |
| SS-30 | ch9 §9.1 L24–27 | "This thesis set out … Strategies 0–2 are implemented" | implementation status (neutral, acceptable per H9 option 3) | S3 |
| SS-31 | ch2 §2.8 L503–505 | "This thesis tunes each compared strategy on the hardware gate and then freezes those gains before the comparison **and training-collection** flights" | fairness rule current ✓; "training-collection" ordering: collection is under S0, frozen before collection — consistent with 5.8 | OK |
| SS-32 | ch1 L203–206 | apparatus "an onboard network with a weight-upload protocol, an offline training pipeline verified against the compiled controller" | stale (flash-resident weights) + H1-adjacent ("verified") | S2 |
| SS-33 | ch3 L125–130; ch4 L256–257 | "Weights are trained offline … and uploaded before flight" | stale deployment path (flash-resident) | S2 |
| SS-34 | ch4 L80–83 | "Strategies 0, 1 and 2 … share one geometric position loop" | not "shared substrate" in the stale sense (describes code structure); OK but check against fairness wording | OK |
| SS-35 | Fairness wording search | "shared substrate", "one gain set", "shared gains" | **none found in printed text** (ch4 header comment mentions substrate removal) | OK |
| SS-36 | Equations vs code | ch4 eq (ctrl-ades) full vector integral $K_i\int e_p$; eq (ctrl-tau-geo) attitude integral; residual subtracted | code differs — see §14(b),(c) | **S1 (DECISION)** |
| SS-37 | ch3 L311 | simulator "with a published downwash model providing the interaction force" | uncited | S2 |
| SS-38 | Tense | Ch5 §5.6 past tense ("was used throughout development", "established"); status ¶ past; rest present | tense mismatch (H1 brief: present/future) | S2 |

---

## 11. Equation inventory

| Label | Ch | Claim | Derived / quoted | Source | Frame / sign | Code check |
|---|---|---|---|---|---|---|
| (rw-trans), (rw-rot) | 2.1 | rigid body + residual wrench | quoted (repeated in Ch3) | none cited in Ch2 | z-up, gravity −e₃ ✓ | — |
| (model-trans), (model-rot) | 3.2 | same | stated | — | z-up ✓ | consistent with `lib.rs:1788–1790` |
| (model-ares) | 3.3 | $f_{\mathrm{res}}/m = \dot v - (f/m\,Re_3 - ge_3)$ | derived (rearrangement) ✓ | — | ✓ | `lib.rs:1781–1833`: $a_{\mathrm{meas}} = R\,(g\,\tilde a) - ge_3$ with $\tilde a$ the accelerometer in g units **Butterworth-prefiltered** (`lib.rs:545, 2374–2382`); $a_{\mathrm{model}} = (\sum k_{t,k}\varpi_k^2/m)Re_3 - ge_3$; rotor speeds pass a sanity filter (`traj_iface.c:726`); optional pre-difference filter/clamp default off (`traj_iface.c:493–494`). Prefilter not stated in Ch3. |
| (model-sign) | 3.3.2 | $fRe_3 = ma_{\mathrm{des}} + mge_3 - f_{\mathrm{res}}$ | derived ✓ (algebra correct) | — | residual subtracted | **code: `f_d = … + res_sign·a_indi + res_sign·a_nn` (`lib.rs:1930–1931`), default `res_sign = +1` (`traj_iface.c:442`), i.e. added by default; the study configuration sets −1 for geometric+network and +1 for our INDI (`crazyflies.yaml:227`, `study/ours.patch:13–14`). DECISION.** |
| (model-F) | 3.4 | $f_{\mathrm{res},i} \approx \mathcal F(\{p_j-p_i, v_j-v_i\})$ | modelling assumption | — | — | code input also includes own height and velocity (ground branch) |
| (model-deepsets) | 3.4.1 | $\hat a = \rho(\sum\phi(\cdot))$, world-frame acceleration | stated | NS family | — | **code: $\hat a = e_3\,\mathrm{sat}_8\{(g/1000)\,m^{-1}\rho_S(\phi_G(-z,-v) + \sum_{j\in\mathcal G}\phi_S(p_j-p_i, v_j-v_i))\}$, $\mathcal G$ = gated neighbours (`residual_nn.rs:262–309`)**; APPLY |
| (model-fold) | 3.4.4 | normalisation fold | derived ✓ | — | — | correct; code folds into two first layers (`tools/residual/model.py:224–241`) |
| (ctrl-ades) | 4.1.1 | $a_{\mathrm{cmd}} = a_{\mathrm{ff}} + K_pe_p + K_ve_v + K_i\int e_p + ge_3 - a_{\mathrm{res}} - \hat a_{\mathrm{res}}$ | stated | — | subtracted | **code: vector integral disabled (`lib.rs:525, 1865–1869`); vertical-only integral $k_{i,z}\int e_z$ enabled at compile time (`lib.rs:535, 1545–1577, 1880`), gain is a runtime parameter (study: 16 for geometric, 0 for our INDI); XY velocity error clamped to ±0.6 m/s and low-passed at 10 Hz (`lib.rs:564, 570, 573–574, 1504–1521`); residual terms enter with `res_sign`. DECISION (integral, sign); APPLY (velocity-error conditioning as a stated detail).** |
| (ctrl-tau-geo) | 4.1.2 | $\tau = -K_Re_R - K_\omega e_\omega + \omega\times J\omega - k_{i,\mathrm{att}}\int e_R$ | stated | Lee (reduced) | — | code `lib.rs:2219–2235`: matches; $k_{i,\mathrm{att}} = 0$ (`ENABLE_ATTITUDE_INTEGRAL = false`, `lib.rs:540, 2225`); optional torque clamp (default off). DECISION (integral wording). Also $e_\omega = \omega - \omega_d$ without transport term. |
| (ctrl-tau-lee) | 4.2 | Lee's torque law | quoted | lee2010 (companion check) | Lee's frame (z-down in Lee) — **not mentioned: Lee et al. use $e_3$ pointing down (NED-like) with $-fRe_3$ thrust; the thesis uses z-up. The torque law itself is frame-independent in body frame, but $e_R$ sign and $f$ convention differ; mapping statement missing** | Phase 1 to check |
| (ctrl-alpharef) | 4.3 | $\alpha_{\mathrm{ref}} = \alpha_{\mathrm{des}} - K_R^{\mathrm{INDI}}e_R - K_\omega^{\mathrm{INDI}}e_\omega$ | stated | Tal & Karaman | — | `lib.rs:2069–2073` ✓ (separate z gains) |
| (ctrl-indi-tau) | 4.3 | $\tau = \tau_{\mathrm{cur}} + J(\alpha_{\mathrm{ref}} - \alpha_{\mathrm{meas}})$ | stated | Tal & Karaman | — | `lib.rs:2204–2213`: $\tau = \mathrm{clamp}(\tau_{\mathrm{cur}} + s_JJ(\mathcal B[\alpha_{\mathrm{ref}}] - \mathcal B[\dot\omega]))$ — $\alpha_{\mathrm{ref}}$ is passed through the same Butterworth as $\alpha_{\mathrm{meas}}$ (`lib.rs:2076–2080`), effectiveness scale $s_J$ (default 1), optional filtering of $\tau_{\mathrm{cur}}$ (`filt_tau`), optional notch, optional clamp. APPLY: state the reference filter. |

Frame and sign: all thesis equations are z-up with gravity $-e_3$ ✓. "Residual subtracted" is stated in 3.3.2 and 4.1.1 and holds in the code only for `res_sign = −1`.

---

## 12. Open `\todo` and `% VERIFY` markers

| Loc | Marker | Type |
|---|---|---|
| ch1 L221–223 | green provenance todo (gap audits) | provenance — keep |
| ch2 L571–574 | green provenance todo (gap audits, 157 works) | provenance — keep |
| ch2 L500 | `% UNVERIFIED (review P-010): no PDF of Abro et al. …` | **misplaced**: sits above the Gu fairness paragraph, not the Abro paragraph (L510) |
| ch5 L87–88 | todo: tape-measure the volume | open author item — keep |
| ch5 L374–375 | todo: revise to past tense as repeats land | open author item — keep (reword without "C.4") |
| ch6 L58, 78, 82, 98, 129, 133, 147, 159 | red PLACEHOLDER todos | skeleton — keep, revise |
| ch7 L35, 47, 51, 85 | red PLACEHOLDER todos | skeleton |
| ch8 L37, 51, 64, 77, 91, 94, 97, 123 | red PLACEHOLDER todos | skeleton |
| ch9 L33, 36, 39, 42 | red PLACEHOLDER todos | skeleton |
| `% VERIFY` | none in the draft | — |

---

## 13. First impressions — the ten weakest places

1. **Ch3 §3.4 and Ch4 §4.4 describe a network that no longer exists** (987 parameters, 3-D output, 2 m/4 cm guards, control-rate evaluation); the code runs a 19 297-weight Neural-Swarm2 port with a scalar vertical output, a lateral/relative-velocity gate, a ground branch and ≈100 Hz evaluation — while the chapter says "these equations are the code".
2. **The residual sign** stated as "subtracted" in Ch3/Ch4 is configuration-dependent in the code, and the study configuration for our INDI uses the opposite (+1) sign — the Ch3 derivation then predicts that Strategy 1's position loop reinforces the disturbance.
3. **Ch4's position and attitude equations carry integral terms the code disables** (vector position integral, attitude integral) and omit the one it enables (vertical-only integral, configuration-dependent gain).
4. **Lab-status and simulation-derived passages** in Ch4 (status column, "two strategies fly"), Ch5 (campaign status, dry-run finding, simulation findings, radio numbers), Ch6 (cf5/cf_second, partial collection) — violate H1/H9.
5. **Metrics are named, not defined** (Ch5 §5.5.4): no formulas, windows, phase segmentation, crossing detector, aggregation across repeats, statistical test or minimum detectable effect; RQ2's regimes and RQ3's penalty have no operational definition.
6. **Ch4 §4.5 contradicts Ch2 §2.5 on Hsieh et al.** (simulation vs hardware match with Chee et al.; "under 30 s" vs 28 s/18 s; 5 ms vs 400 Hz).
7. **Background for the student reader is thin**: SE(3)/SO(3), the vee map, the attitude error function Ψ, INDI derivation (why the increment rejects disturbances), feedback linearisation, deep sets, spectral normalisation, flatness are used but not explained; Lee's z-down convention vs the thesis z-up not mapped.
8. **Ch1 §1.2 overstates** ("exactly two ways … organises the entire field"; prediction outside training data "unconstrained") against Ch2's own third/fourth stance and Ch3's 8 m/s² bound.
9. **Ch5 control loop 500 Hz and 35 logged variables** are stale (1000 Hz, 96); Ch5 says $e_R$ is not logged (it is).
10. **Ch6–9 skeletons carry stale scope and internal vocabulary** (hybrid section, "Strategy 4", "C.1/C.4", file names, "our notes"), and Ch8 lacks the promised sections (moment residual, assumption-by-assumption reading, frozen-gain disadvantage).

---

## 14. Code-vs-draft check (verification checklist (a)–(g))

### (a) Network

| Item | Code (location) | Draft says | Verdict |
|---|---|---|---|
| Architecture | $\phi_S$: 6→25→40→40→20; $\phi_G$: 4→25→40→40→20; $\rho_S$: 20→40→40→40→1; ReLU hidden, linear last layer; also unused $\phi_L$, $\rho_L$ (`residual_nn.rs:14–30, 91–100, 312–339`) | $\phi$ 6→16→16→8, $\rho$ 8→16→16→3 (3.4.1) | **MISMATCH — APPLY** |
| Parameter count | 19 297 weights in the flat array (`residual_nn.rs:65, 125`); of which the active path ($\phi_S$+$\phi_G$+$\rho_S$) = 3 675 + 3 625 + 4 161 = 11 461 | 987 (3.4.1, 4.4) | **MISMATCH — APPLY** (state both: 19 297 stored, 11 461 on the evaluated path) |
| Input gate | neighbour summed iff $\lvert\Delta x\rvert<0.2$ m ∧ $\lvert\Delta y\rvert<0.2$ m ∧ $\lvert\Delta v_x\rvert<1.5$ m/s (`residual_nn.rs:139–140, 281–286`); no distance cut-off, no near-field clamp (`residual_nn.rs:40–44`) | 2 m exclusion; 4 cm clamp preserving direction (3.4.3) | **MISMATCH — APPLY** |
| Ground branch | $\phi_G([-z, -v_x, -v_y, -v_z])$ always added (`residual_nn.rs:271–276`) | absent | **MISSING — APPLY** |
| Neighbour limit | `MAX_NEIGHBOURS = 3` (`residual_nn.rs:129`); peers from peer-localisation, position only; relative velocity by onboard finite differences of broadcast positions (`lib.rs:839–873`) | "a three-robot team has two neighbours"; relative states "the flight stack already broadcasts" | partially stated — APPLY (finite-difference velocity) |
| Output clamp | $\lvert\hat a_z\rvert \le 8$ m/s², clamp flag logged; non-finite → 0 and flag (`residual_nn.rs:134, 298–307`) | 8 m/s², logged (3.4.3) | ✓ |
| Output dimension | scalar vertical; $x,y$ components identically 0 (`residual_nn.rs:38, 100, 302`) | 3-D world-frame acceleration (3.4.1, eq model-deepsets) | **MISMATCH — APPLY** |
| Units | network output in gram-equivalent force units, converted ×9.81/1000 N, ÷ mass (`residual_nn.rs:48–55, 136, 295–296`) | acceleration "so it is comparable with $a_{\mathrm{res}}$" | **APPLY** (state conversion; comparable with $a_{\mathrm{res},z}$ only) |
| Evaluation rate | every `rnn.div` ticks, default 10 at 1 kHz ≈ 100 Hz, output held between evaluations (`traj_iface.c:132`; `lib.rs:884–903`) | "at the control rate", "within the control period" | **MISMATCH — APPLY** |
| Weights | flash-resident build feature `residual_nn_flash` (`Cargo.toml` features; `lib.rs:922–926`); alternative RAM-upload path exists (`residual_nn` feature; `traj_iface.c:116–214`) | "uploaded before flight"; apparatus "weight-upload protocol" | **stale — APPLY** (describe both, flash-resident as the deployment form) |
| Normalisation | folded into first layers of $\phi_S$ and $\phi_G$ separately (`tools/residual/model.py:224–241`) | folded into first layer of $\phi$ | APPLY (two first layers) |
| Spectral normalisation | none in training code (`tools/residual/model.py`, `train.py`: no occurrence) | "not used here" | ✓ |
| Training target | $a_{\mathrm{res},z}$ only, Huber loss (`train.py:17, 118`) | not stated | add (APPLY, design) |
| Computed when | whenever weights are loaded (`rnn.ready`), whatever `rnn.en` (`lib.rs:1752–1756`); used in control only if `rnn.en = 1` (`lib.rs:1851–1855`) | "computed and logged whether enabled or not" (4.4) | ✓ |

### (b) Residual sign

`lib.rs:1857–1861, 1930–1931`: `f_d = a_d + K_p e_p + K_v e_v + … + g e_3 + res_sign·a_indi + res_sign·a_nn`. `a_indi = a_res` iff ctrl_mode bit 0 set (`lib.rs:1837`); `a_nn` = prediction iff `rnn.en = 1` and `rnn.ready` (`lib.rs:1851`). Firmware default `res_sign = +1` (`traj_iface.c:442`) → residual **added**; code comments call −1 "derivation-correct". Study configuration: geometric + network `res_sign: -1` (`crazyswarm2/crazyflie/config/crazyflies.yaml:227`); our full INDI `res_sign: 1` (`study/ours.patch:13–14`). Draft (3.3.2, 4.1.1): residual **subtracted** for both measured and predicted. **Verdict: MISMATCH for Strategy 1 as configured; matches for Strategy 2. DECISION D-01 (prominent).** The Ch3 falsifiable consequence ("with the sign inverted, the displacement must be larger, approximately twice") then applies to Strategy 1's position loop as configured.

### (c) Integral terms

| Term | Code | Draft | Verdict |
|---|---|---|---|
| Vector position integral $K_i\int e_p$ | `ENABLE_POSITION_INTEGRAL = false` (`lib.rs:525`); term zero (`lib.rs:1865–1869`); accumulator still updated and clamped ±2 (`lib.rs:1761–1766`) but unused | in eq (ctrl-ades) for S0–S2 | **MISMATCH — DECISION D-02** |
| Vertical-only integral $k_{i,z}\int e_z$ | `ENABLE_Z_INTEGRAL = true` (`lib.rs:535`); gain `pos_gains.ki_z` (default 8, `traj_iface.c:572`), limit 1.5 m·s, conditional anti-windup on thrust saturation / ground (`lib.rs:1548–1569`); study: `ki_z 16` geometric, `0` our INDI (`crazyflies.yaml:237`, `study/ours.patch:22–23`) | absent | **MISSING — DECISION D-02** |
| Attitude integral $k_{i,\mathrm{att}}\int e_R$ | `ENABLE_ATTITUDE_INTEGRAL = false` (`lib.rs:540`) → `ki_att = 0` (`lib.rs:2225`) | in eq (ctrl-tau-geo), and "adds an integral term the source does not have" (4.1.2, 4.2) | **MISMATCH — DECISION D-02** |

### (d) INDI increment and filters

`lib.rs:1986–2016, 2046–2080, 2095–2213`: $\alpha_{\mathrm{meas}} = \mathcal B[\Delta\omega/\Delta t]$ (default order: differentiate raw gyro then 2nd-order Butterworth; option filter-then-differentiate); $\alpha_{\mathrm{ref}} = \alpha_{\mathrm{des}} - K_Re_R - K_\omega e_\omega$ then **the same Butterworth** applied to $\alpha_{\mathrm{ref}}$; $\tau_{\mathrm{cur}}$ from per-motor $k_{t,k}\varpi_k^2$ through the mixer inverse (arm 0.0354 m, drag ratio; `lib.rs:1481–1493`), optionally filtered (`filt_tau`), optional notch, optional clamp; $\tau = \tau_{\mathrm{cur}} + s_J J(\alpha_{\mathrm{ref,f}} - \alpha_{\mathrm{meas}})$, then optional clamp; gyroscopic term absent in INDI branch (correct). Filter coefficients are designed for `filt_dt_us` (default 2000 µs) while the loop runs at 1 kHz, and the default discretisation is not pre-warped (`traj_iface.c:444–478`; `lib.rs:69–93`): the nominal cutoff parameter `fc_bw` is **not** the realised −3 dB frequency at the defaults. Accelerometer prefilter uses the same Butterworth (`lib.rs:545, 2374–2382`). Draft (4.3): "differentiating the gyro signal and low-pass filtering it"; eq (ctrl-indi-tau) without reference filtering. **Verdict: increment structure ✓; reference-side filter missing in text — APPLY; realised cutoff vs nominal — VERIFY/DECISION D-06 (which values the study configuration sets is a configuration fact the author must state).**

### (e) Loop rates

Controller: called every main-loop step at `RATE_MAIN_LOOP = 1000 Hz` (`crazyflie-firmware/src/modules/src/stabilizer.c:303, 350`; `stabilizer_types.h:383`), $\Delta t$ = tick difference × 1 ms (`lib.rs:2350–2351`). uSD logging at 500 Hz (`usd_study_config.txt` line 6; gated at `stabilizer.c:371`). Network ≈100 Hz (`rnn.div` 10). Draft: "The control loop runs at 500 Hz onboard" (5.1). **MISMATCH — APPLY (1 kHz).**

### (f) Recorded variables

`usd_study_config.txt`: header (version, buffer 8192, file name `thesis`, mode `on:fixedFrequency`, **500 Hz**) + **96 variables**: state estimate position/velocity (6) and quaternion (4); Euler attitude (3); gyro (3); accelerometer (3); residual $a_{\mathrm{res}}$ (3); INDI torque (3) and filtered angular acceleration (3); network prediction (3) + clamp flag (1); commanded position (3), velocity (3), acceleration (3); motor commands (4); rotor speeds from DShot telemetry (4), optical deck (4), and the filtered values the controller uses (4); attitude error $e_R$ (3) + norm (1); battery voltage (1); external-pose input (3) + its tick (1); controller-internal diagnostic variables of other controller implementations (6 + 23); run tag (1). Draft: 35 variables (5.4.1, 5.8); recorded list (5.4.2) lacks commanded velocity/acceleration, quaternion, rotor speeds, $e_R$, battery voltage, pose input; 5.5.4 says $e_R$ not logged. **MISMATCH — APPLY (96; list; $e_R$ logged).** Note: some variables belong to controller implementations outside the compared set; the thesis need not name them (H5).

### (g) Formation library

`scenarios.py:134–469`: 16 builders A1–A8, B1–B3, C1–C5, registry at L466–470; separations are parameters with defaults (A1 dz 0.50; A2 0.30; A3 0.30; A4 dz 0.60 offset 0.10; A5 dz 0.50 radius 0.40; A6 dz 0.10 (gated); A7 1.10→0.10 (gated when end < 0.15); A8 dz 0.25 span 1.0; B1 0.20/0.30; B2 0.20/0.30, r 0.10; B3 0.22; C1 sep 0.30, n 3; C2 gap 0.50, n 3; C3 side 0.50; C4 lateral 1.0, dz 0.50→0.10 (gated when end < 0.15); C5 z 0.15, single robot). ENU, z-up, robot 0 lowest (L11–14). Floors 0.30 m / 0.10 m (`safety.py:53, 60`). Draft Tab 5.1: 16 scenarios ✓; gated set A6, A7, C4 ✓; C1–C3 coplanar ✓; C5 one vehicle ✓. **Verdict ✓.** Gaps in the draft: default parameter values are not given anywhere (a reader cannot reproduce a scenario), and C1/C2 default to three vehicles although listed under "Controls" without team size.

### Fact-sheet vs code disagreements

None material: the fact sheet's network, gate, rate, sign, integral, loop-rate and variable-count statements all match the code. Fact sheet "≈41 g" matches the study configuration mass (`crazyflies.yaml:653`); the compiled firmware default 36.4 g (`traj_iface.c:297`) is overridden at connect.
