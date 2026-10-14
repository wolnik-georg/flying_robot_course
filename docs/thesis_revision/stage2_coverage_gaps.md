# Stage 2 — Literature coverage and gap analysis (no edits)

Basis: the 18 held PDFs, their reference lists and related-work sections (Stage 1 records R01–R18), and general
knowledge. **Every work not among the 18 held PDFs is `UNVERIFIED — author must check`.** Where bibliographic data
below come from a held paper's reference list, the list is named ("via R0x"); that is secondary evidence, not
verification. Nothing in this file is to be added to `references.bib` without the author checking the primary source.

---

## 1. Family map

| Family | What it contributes to the thesis question | Held papers representing it |
|---|---|---|
| F1 Rotor-wake and inter-vehicle aerodynamics (measurement, flow models) | what the disturbance is, its magnitude, asymmetry, lateral decay, regimes, transient behaviour | Kiran 2025 (R11), Gielis 2023 (R03), Bauersfeld 2024 (R12); partially Shi 2020 (−9 g at 30 cm, R01) |
| F2 Close-proximity / formation flight control with interaction compensation | the application; who flew how close, with what | Shi 2020, Shi 2022, Smith 2023, Shankar 2023, Chee 2024, Hsieh 2025, Hsieh 2026, Li 2023, Zhang 2024 (ProxFly) |
| F3 INDI / NDI foundations (incremental control, rotor-speed sensing, filtering) | Strategy 1 and the reactive family | Tal & Karaman (R13), Smeur 2017 (R14); used in Cobo-Briesewitz (R07) |
| F4 Geometric control on SE(3) and flatness-based trajectory tracking | Strategy 0 and the shared position loop | Lee 2010 companion (R15); CDC 2010 (no PDF); flatness used in Tal & Karaman, Hsieh 2026, Yang 2025 |
| F5 Learned residual / dynamics models (supervised, offline) | Strategy 2 and the predictive family | Shi 2020, Shi 2022, Smith 2023, Li 2023, Cobo-Briesewitz (LINDI), Hsieh 2026 (learned + ROM), Chee 2024 (KNODE) |
| F6 Permutation-invariant set functions (deep sets) and aggregation | the summed per-neighbour model; RQ3 | Shi 2020, Shi 2022 (theory, Thm 2/4), Gielis 2023 |
| F7 Differential flatness and flatness-preserving residuals; feedback linearisation | Strategy 3 | Yang 2025 (R06), Hsieh 2026 (R05); Tal & Karaman (flatness FF) |
| F8 Learning-based / adaptive MPC | out-of-scope injection point; evaluation conditions | Chee 2024, Hsieh 2025, Li 2023, Hsieh 2026 (NMPC baseline) |
| F9 Online adaptation / disturbance observers | reactive family beyond INDI; hybrid | Hsieh 2025 ($\mathcal L_1$), Gu 2025 (online learning), Smeur (INDI as disturbance rejection) |
| F10 Residual reinforcement learning | the no-force-estimate stance | Zhang 2024 (ProxFly) |
| F11 Benchmarking and evaluation practice in aerial robotics | how to build and report a comparison | Gu 2025; same-rig ablations in all F2 papers |
| F12 Safety and uncertainty of learned components (Lipschitz bounds, output bounds, OOD) | output clamp, generalisation claims | Shi 2020/2022 (SN), Li 2023 (SN, γ study), Gu 2025 (crashes from large predictions), Cobo-Briesewitz (OOD) |
| F13 Ground effect (confound, C5) | the near-ground scenario and the 0.30 m floor | NS2 (environment as neighbour), Gu 2025 (ground-effect case) |

---

## 2. Coverage matrix

Status of each family in the current Chapter 2 (draft), with missing works. Confidence: **H** = I am confident the
work exists with these details (often because a held paper cites it); **M** = confident it exists, details from memory.

| Family | Ch2 coverage | Missing foundational / comparison works (UNVERIFIED — author must check) | Where | Conf. |
|---|---|---|---|---|
| F1 | **thin**: Kiran, Gielis, Bauersfeld described as methods; magnitudes not used; C2.06 (nonlinear, asymmetric) uncited | **Jain, Fortmuller, Byun, Mäkiharju, Mueller, "Modeling of aerodynamic disturbances for proximity flight of multirotors", ICUAS 2019, pp. 1261–1269** (via R03, R05, R08, R09, R11) — the physics model under Chee/Hsieh's ROM and ProxFly's FB-AeroComp; **Preiss, Hönig, Ayanian, Sukhatme, "Downwash-aware trajectory planning for large quadrotor teams", IROS 2017, pp. 250–257** (via R11) — origin of the 60 cm/24 cm ellipsoid; **Kiran, Harutyunyan, Ayanian, Breuer, "Downwash dynamics: impact of quadrotor separation on forces, moments, and velocities for dense formation flight", AIAA Aviation 2024, p. 4145** (via R05, R09) | §2.2 | H |
| F2 | **covered** (10 papers) | **Hönig, Preiss, Kumar, Sukhatme, Ayanian, "Trajectory planning for quadrotor swarms", IEEE T-RO 34(4):856–869, 2018** (via R01, R02, R16) — the source Shi et al. cite for 60 cm; **Jain & Mueller, "Flying batteries: in-flight battery switching to increase multirotor flight time", ICRA 2020, pp. 3510–3516** (via R08) — docking application | §1.1, §2.4 | H |
| F3 | **covered, thin on foundations** | **Smeur, Chu, de Croon, "Adaptive incremental nonlinear dynamic inversion for attitude control of micro air vehicles", J. Guid. Control Dyn. 39(3):450–461, 2016** (via R07, R13, R14); **Sieberling, Chu, Mulder, "Robust flight control using incremental nonlinear dynamic inversion and angular acceleration prediction", J. Guid. Control Dyn. 33(6):1732–1742, 2010** (via R13) — INDI origin; **Sun, Romero, Foehn, Kaufmann, Scaramuzza, "A comparative study of nonlinear MPC and differential-flatness-based control for quadrotor agile flight", IEEE T-RO 38(6):3357–3373, 2022** (via R06, R07, R16) — a controller comparison that finds INDI essential for both | §2.3; §2.8 | H |
| F4 | **covered** (Lee) but no flatness source | **Mellinger & Kumar, "Minimum snap trajectory generation and control for quadrotors", ICRA 2011, pp. 2520–2525** (via R05, R06, R16, R18) — flat outputs $[p,\psi]$ and the $R_d$ construction used in Ch4; **Faessler, Franchi, Scaramuzza, "Differential flatness of quadrotor dynamics subject to rotor drag …", RA-L 3(2):620–626, 2018** (via R05, R06, R07, R13) — the frame construction the code calls "Faessler" | §2.1; Ch4 §4.1 | H |
| F5 | **covered** | **Shi et al., "Neural Lander: stable drone landing control using learned dynamics", ICRA 2019, pp. 9784–9790** (via R01, R02, R05, R06, R08, R16, R18) — origin of the residual-learning + SN line; **O'Connell et al., "Neural-Fly enables rapid learning for agile flight in strong winds", Science Robotics 7(66), 2022** (via R16, R18) — learned + adaptive hybrid on one vehicle; **Bauersfeld, Kaufmann, Foehn, Sun, Scaramuzza, "NeuroBEM: hybrid aerodynamic quadrotor model", RSS 2021** (via R07, R12, R18); **Spitzer & Michael, "Feedback linearization for quadrotors with a learned acceleration error model", ICRA 2021, pp. 6042–6048** (via R05) — FBL + learned residual, single vehicle | §2.4; §2.5 FBL ¶ | H |
| F6 | **thin**: deep sets used without the source theorem | **Zaheer, Kottur, Ravanbakhsh, Póczos, Salakhutdinov, Smola, "Deep sets", NeurIPS 2017, pp. 3391–3401** (via R01, R02, R03) — the representation theorem that justifies Ch3 eq. (model-deepsets) | §2.4¶; Ch3 §3.4.1 | H |
| F7 | **covered** (Yang, Hsieh) but no foundation | **Fliess, Lévine, Martin, Rouchon, "Flatness and defect of non-linear systems: introductory theory and examples", Int. J. Control 61(6):1327–1361, 1995** (via R05, R06, R13) | §2.1 | H |
| F8 | **covered** | **Chee, Jiahao, Hsieh, "KNODE-MPC: a knowledge-based data-driven predictive control framework for aerial robots", RA-L 7(2):2819–2826, 2022** (via R05, R06, R09, R10); **Torrente, Kaufmann, Föhn, Scaramuzza, "Data-driven MPC for quadrotors", RA-L 6(2):3769–3776, 2021** (via R06) | §2.5 MPC ¶ | H |
| F9 | **absent as a family** (Hsieh 2025 treated as MPC only) | **Wu, Cheng, Zhao, Gahlawat, Ackerman, Lakshmanan, Yang, Yu, Hovakimyan, "L1 Quad: L1 adaptive augmentation of geometric control for agile quadrotors with performance guarantees", arXiv:2302.07208** (via R10) — $\mathcal L_1$ on a geometric controller; **Hanover, Foehn, Sun, Kaufmann, Scaramuzza, "Performance, precision, and payloads: adaptive nonlinear MPC for quadrotors", RA-L 7(2):690–697, 2021/2022** (via R10) | §2.3 / §2.7 | H |
| F10 | covered (ProxFly) | — (in scope only as related work; no addition needed) | §2.6 | — |
| F11 | **thin**: §2.8 argues non-composability but gives no account of evaluation practice | Sun et al. 2022 (above, F3) — same-hardware controller comparison methodology; **Martini et al., "A benchmark framework for testing, evaluation, and comparison of quadrotor linear and nonlinear controllers", ICUAS 2023, pp. 471–478** (via R18) | §2.8 | H |
| F12 | thin (SN mentioned; Gu crashes in Ch3) | **Bartlett, Foster, Telgarsky, "Spectrally-normalized margin bounds for neural networks", NeurIPS 2017** (via R01, R02); **Miyato, Kataoka, Koyama, Yoshida, "Spectral normalization for generative adversarial networks", ICLR 2018** (via R16) | §2.4 NS2 ¶ | H |
| F13 | absent (C5 scenario motivated only in Ch5) | **Cheeseman & Bennett, "The effect of ground on a helicopter rotor in forward flight", ARC R&M, 1955** (via R01, R02); **Kan et al., "Analysis of ground effect for small-scale UAVs in forward flight", RA-L 4(4):3860–3867, 2019** (via R01, R02); **Carter, Bouchard, Quinn, "Influence of the ground, ceiling, and sidewall on micro-quadrotors", AIAA J. 59(4):1398–1405, 2021** (via R03, R11, R16) — wall/ceiling effects, relevant to Ch5's "10 cm from a wall" | Ch5 §5.2; Ch2 §2.2 (one sentence) | H |
| Background (textbook-level) | absent | **Mahony, Kumar, Corke, "Multirotor aerial vehicles: modeling, estimation, and control of quadrotor", IEEE RAM 19(3):20–32, 2012** (via R12) — tutorial for the student reader; **Slotine & Li, *Applied Nonlinear Control*, Prentice Hall, 1991** (via R13) — NDI/feedback linearisation; **Khalil, *Nonlinear Systems*, 3rd ed., 2002** (via R01) — ISS, Lyapunov | Ch2 §2.1 or Ch3 §3.1 | H |
| Platform | absent | **Busetto et al., arXiv:2512.14450** (named in `lib.rs:230` as the source of the brushless inertia constants) — `[VERIFY]` title, authors, content; **Preiss, Hönig, Sukhatme, Ayanian, "Crazyswarm: a large nano-quadcopter swarm", ICRA 2017, pp. 3299–3304** (via R01, R02, R07, R08) — the software stack | Ch5 §5.1 | M (Busetto), H (Crazyswarm) |

---

## 3. Gap-claim stress test

**Current gap (Ch1 §1.3 italic; Ch2 §2.9 first gap):** "Representative methods exist for reactive compensation, for
predictive compensation and for their hybrid. Each is validated on its own. To the best of our knowledge, they have not
been evaluated against one another under identical conditions."

**Works that weaken it (held corpus):**

| Work | What it compares on one rig | Why it does not fully close the gap |
|---|---|---|
| **Hsieh, Chee, Hsieh 2025 (R10)** | **online residual estimate ($\mathcal L_1$) vs learned+physics prediction (KNODE-DW / DW) vs both**, nominal MPC as baseline, **three** Crazyflies, V-stack and I-stack, 5 runs each | all inside one **off-board NMPC**; the online estimate is a velocity-predictor disturbance observer using commanded thrust, not rotor-speed INDI; two formation types, few separations; no geometric/onboard controllers; the two-to-three transfer is not isolated as a quantity |
| Cobo-Briesewitz 2026 (R07) | uncompensated Lee vs INDI (measured) vs LINDI (learned) vs NA-INDI (hybrid), one Crazyflie | **single vehicle**, self-induced residual, no relative state |
| Chee 2024 (R09) | nominal vs physics (DW) vs learned (KNODE) vs physics+learned inside NMPC, two Crazyflies | all predictive; no measured family |
| Hsieh 2026 (R05) | nominal vs ROM vs learned, each in FBL and SE(3), two Crazyflies, 60 runs | all predictive; off-board |
| Zhang 2024 ProxFly (R08) | basic vs model-based relative-position compensation (FB-AeroComp) vs residual RL, same hardware pair | no measured-residual family; heterogeneous pair |
| Bauersfeld 2024 (R12) | with/without a physics feedforward compensation, two vehicles | single method vs baseline |
| Gu 2025 (R18) | four online estimators | single vehicle, simulation |
| Sun et al. 2022 (UNVERIFIED, via R06/R07/R16) | NMPC vs differential-flatness control, both with INDI | single vehicle, agile flight, no interaction force |
| Abro 2025 (no PDF) | three regulators, close proximity | as in the draft; unverifiable here |

**Assessment.** The draft's broad statement — families "not evaluated against one another under identical conditions" —
**does not survive Hsieh et al. 2025**, which is a same-rig, multi-vehicle comparison of exactly the three information
sources (measure online, predict, both), albeit inside MPC. The Ch1/Ch2 sentences "The hybrid has been demonstrated only
where the residual is self-induced: a single vehicle" are **false** for the same reason.

**Narrowest defensible wording (recommended, DECISION D-03):**

> To the best of our knowledge, no study has compared rotor-speed-based reactive compensation (INDI) with learned
> per-neighbour feedforward prediction in onboard controllers on one airframe, across a fixed library of
> interaction regimes — static and transient, vertical and lateral, two and three vehicles — with the residual logged
> under every controller and the two-to-three-vehicle transfer of the summed model quantified. The closest study,
> Hsieh et al. (2025), compares an online $\mathcal L_1$ estimate, a learned downwash model and their combination
> inside one off-board model predictive controller on three vehicles in two stacked formations; it finds the
> combination best and the online estimate alone ahead of the learned model alone for the middle vehicle.

The three things that still make the thesis distinct: (1) *onboard* geometric/INDI controllers rather than an off-board
MPC; (2) a *frozen multi-regime scenario library* (RQ2) rather than two formation types; (3) *quantification* of the
summed-model transfer penalty (RQ3) and a cost account (RQ4). Each must be stated as a property of the design, not of
its results.

**Further consequences.**
- Ch1 §1.3 "Two narrower gaps follow. The hybrid has been demonstrated only where the residual is self-induced …" — the
  first narrower gap disappears; the second (summation penalty) survives, narrowed: Hsieh et al. 2025 *use* a summed
  two-vehicle-trained model on three vehicles without isolating the penalty.
- Ch2 §2.9 second gap — rewrite as "the hybrid has been compared on teams only inside MPC (Hsieh et al. 2025)" (or delete).
- The two provenance todos must be revised by the author: the audits did not recognise Hsieh 2025 as spanning the
  families.
- The thesis' terminology must accommodate Chee et al.'s use of "reactive" for current-time-step feedforward (R09)
  and the $\mathcal L_1$ estimate as a reactive member.

---

## 4. Background sufficiency (for the stated student reader)

| Topic | Where now | Sufficient? | What is missing, at what depth |
|---|---|---|---|
| SE(3), SO(3), rotation matrix as body→world map | Ch3 §3.1 (one line); Ch2 §2.1 "geometric tracking controller on SE(3)" | **No** | ½ page: what SO(3)/SE(3) are; why a rotation matrix (not Euler angles) — singularities; hat/vee maps with a 2-line example; how $R e_3$ is the thrust direction. |
| Rotation matrices vs quaternions | absent | **No** | 3–4 sentences: the EKF delivers a quaternion, converted to $R$ (code `lib.rs:177–183, 2361–2365`); Tal & Karaman and Smeur use quaternion errors; double cover; the thesis' laws use $R$. |
| Attitude error function Ψ and $e_R$ | Ch2 §2.1, Ch4 §4.2 (named, not defined) | **No** | define $\Psi = \tfrac12\mathrm{tr}(I - R_d^\top R)$ and its relation to the rotation angle ($\Psi = 1 - \cos\theta$); one worked value (90° ⇒ Ψ = 1). |
| Geometric position loop and $R_d$ construction from thrust vector and yaw | Ch4 §4.1.1 ("usual flatness construction") | **Partly** | the three-line construction ($z_d = a/\lVert a\rVert$, $y_d = z_d\times x_c/\lVert\cdot\rVert$, $x_d = y_d\times z_d$) — it is in the code (`lib.rs:1463–1473`) and Lee/Mellinger. |
| Differential flatness | Ch2 §2.1 (informal) | **Partly** | flat outputs $[p,\psi]$; why $a$ fixes thrust and $z_b$; why jerk fixes $\omega$ and snap fixes $\dot\omega$ (one equation each, from Hsieh 2026 eqs. 3–11 or Tal & Karaman eqs. 9–15). |
| NDI vs INDI derivation; why the increment rejects disturbances | Ch2 §2.3 (verbal), Ch4 §4.3 (formula) | **No** | ½–1 page: NDI cancels a model; INDI replaces the unknown part with the measured acceleration (Tal & Karaman eqs. 18–20 / 29–31); the result that closed-loop angular acceleration ≈ actuator dynamics; the cost (needs current actuation and its measurement, matched filters). A toy 1-D example: $\ddot x = u + d$, increment $u_k = u_{k-1} + (a_{\mathrm{ref}} - \ddot x_{k-1})$. |
| Feedback linearisation | absent (Ch1/Ch2 use the term) | **No** | 3–4 sentences + reference (Slotine & Li, UNVERIFIED). |
| Deep sets, permutation invariance, why summation | Ch3 §3.4.1–2 | **Partly** | state the representation result (Zaheer et al., Shi et al. Lemma 1) in one sentence; a two-neighbour numerical toy showing order invariance. |
| Spectral normalisation, Lipschitz constant | Ch2 (named) | **No** | 2 sentences: definition of Lipschitz constant and that SN bounds it by the product of layer spectral norms (Shi 2020 eq. 7; Li eq. 11). |
| Residual force estimation and its noise/delay | Ch3 §3.3 | **Mostly** | add: the accelerometer measures specific force; matched filtering requirement (Tal & Karaman, Smeur); delay of a 2nd-order Butterworth at cutoff $f_c$ ≈ $\sqrt2/(2\pi f_c)$ s group delay at DC (≈7.5 ms at 30 Hz — textbook value, illustrative); bias sensitivity (Smeur §4.6). |
| Filtering and delay (Butterworth, notch, discretisation) | Ch4 §4.3 (one sentence) | **No** | ½ page: what a 2nd-order Butterworth does; why the same filter on both terms; that the realised cutoff depends on the discretisation sample time (code fact: `filt_dt_us`, `filt_prewarp`) — DECISION D-06 for the values. |
| State estimation (EKF with mocap), latency | Ch5 §5.1 (one line) | **No** | 4–5 sentences; latency and why it matters for prediction (relative velocity from finite differences of broadcast positions, `lib.rs:839–873`). |
| Motor dynamics, DShot telemetry, thrust model $k_t\varpi^2$ | Ch3 §3.1, Ch5 §5.1 | **Partly** | units (RPM), per-motor constants, sanity filter on telemetry (`traj_iface.c:726`), battery/thrust drift. |
| Stability notions (exponential stability, attractiveness, ISS) | Ch2/Ch4 use the terms | **No** | 3 sentences with Khalil (UNVERIFIED). |
| Experimental design and statistics (repeats, paired comparison, effect size) | absent | **No** | ½ page in Ch5 §5.5.4 (see §5 below). |

---

## 5. Evaluation practice in the held papers

| Paper | Baseline | Metric | Repeats | Separation reported as | Sim vs HW | Controller location |
|---|---|---|---|---|---|---|
| Shi 2020 | same controller, $\hat f_a = 0$ | max \|z\| error, any vehicle | 6 swaps | commanded 0.2–0.25 m | HW | onboard |
| Shi 2022 | same controller, $f_a = 0$ | worst-case z error; box plots; "statistically significant in most cases" | 6 swaps | commanded 0.2–0.3 m | HW (+ planning sim) | onboard |
| Smith 2023 | no model; deep non-equivariant | mean 3-D / vertical position and velocity error | "across experiments" (n not stated) | commanded 0.6–0.8 m | HW | onboard (RPi) |
| Cobo-Briesewitz | Lee (no residual) | mean position tracking error | 20 (10 with payload) | n/a | HW | onboard |
| Chee 2024 | nominal MPC | RMSE, z_max | 5 per case | commanded and achieved separation | sim + HW | off-board |
| Hsieh 2025 | MPC family | RMSE, z_max | 5 per case | commanded | HW | off-board |
| Hsieh 2026 | nominal FBL/SE(3) | RMSE, z_max | 5 per case (60 runs) | commanded | sim + HW | off-board |
| Li 2023 | NMPC without prediction | height RMSE | 3 rounds | not quantified | HW | onboard (TX2) |
| ProxFly | basic; model-based compensation | position and attitude RMSE | not stated | commanded 0.5 m | sim (train) + HW | off-board high level |
| Shankar | no model | terminal vertical error; dock success | 10 | commanded (cable) | HW | onboard (RPi) |
| Smeur | PID | max position deviation | 7 | n/a | HW | onboard |
| Gu | estimators vs each other | estimation error, tracking, time | 50 sim trials | n/a | sim (+ Jetson timing) | — |
| Gielis | linear vs nonlinear models | integrated prediction error over arena | — | rig geometry | rig | — |

**Observations for §2.8 (methodological, not critical of authors):**
1. The dominant design is a **within-paper ablation**: one controller with the learned term on and off, so each
   "improvement" is relative to that paper's own nominal controller (NS, NS2, Hsieh 2026, Chee, Li).
2. Metrics converge on **RMSE and a maximum vertical error**, but windows differ (whole flight, final 0.5 s, crossing
   only) and are rarely defined precisely enough to reproduce.
3. **Repetitions are few (3–20)** and distributions are reported in only some cases; formal tests are rare (NS2 mentions
   significance; Bauersfeld uses a t-test on model residuals).
4. Separations are usually **commanded**, not achieved; Chee reports achieved separation; achieved separation under
   compensation can differ (the controller changes the geometry).
5. **Simulation vs hardware vs processor-in-the-loop** must be distinguished (Gu's "hardware" is a Jetson running a
   simulator; Yang is simulation; Hsieh 2026's 20× is simulation).
6. **On-board vs off-board** execution changes what "cheap" means (Hsieh 2026/Chee/Hsieh 2025 off-board; NS/NS2/Cobo
   onboard STM32; Li onboard TX2; Smith/Shankar onboard RPi).
7. **Data splits** differ: NS2 by ≈20 s pieces; Li by shuffled samples; Smith/Hsieh by trajectory or separation (OOD).
8. Baselines are rarely **tuned for themselves** — the thesis' "each tuned for itself, then frozen" rule is closer to
   Gu's BO-tuned fairness than to the within-paper ablation norm.

What §2.8 should add: a paragraph with points 1–8 (as practice, not fault), the explicit requirement list that follows
(already present), and a sentence that the thesis' metrics are defined in Ch5 with windows and detectors.

---

## 6. Ranked list — the 15 most valuable additions or corrections

| Rank | Item | Type | Ch | Basis |
|---|---|---|---|---|
| 1 | Correct the gap and the "hybrid only on a single vehicle" statements for Hsieh et al. 2025 | correction (gap) | 1, 2 | R10, §3 |
| 2 | Replace the "residual that survived compensation" rationale with a correct one | correction (argument) | 4, 5 | R01, R02 |
| 3 | Correct Ch4 §4.5 on Hsieh 2026 (simulation vs hardware, 30 s, 5 ms) to match Ch2 | correction | 4 | R05 |
| 4 | Add a ½–1 page INDI derivation with a 1-D toy example and the matched-filter requirement | background | 2 or 4 | R13, R14 |
| 5 | Add SE(3)/SO(3), hat/vee, Ψ definition, $R_d$ construction, quaternion remark | background | 3 §3.1 / 4 §4.1 | R15, code |
| 6 | Cite a source for nonlinearity, asymmetry and lateral decay (Kiran 65 %/90 %, Gielis lateral concentration) and add an illustrative magnitude | source + example | 1, 2, 3 | R11, R03 |
| 7 | Add Cobo-Briesewitz Table 1 (INDI with PWM worse than Lee) as evidence for the rotor-speed prerequisite | evidence | 2, 3 | R07 |
| 8 | Add Hsieh 2026's learned-residual SE(3) result as the closest published analogue of Strategy 2 | related work | 2 | R05 |
| 9 | Add §2.8 evaluation-practice paragraph (points 1–8 above) | related work / method | 2 | §5 |
| 10 | Correct NS2 "both in the abstract", Smith 56 % (abstract), Smeur 4 Hz, Gielis wording, Shankar attribution, Cobo "swing" / "ground forces" | reference context | 1, 2 | R02, R04, R14, R03, R17, R07 |
| 11 | Cite deep-sets theory (Zaheer et al., UNVERIFIED; or Shi et al. Lemma 1/Thm 2, held) for eq. (model-deepsets) | background | 3 | R01, R02 |
| 12 | Add the accelerometer-bias failure mode and the matched-filter requirement to Ch3 §3.3.1 | missing failure mode | 3 | R14, R13 |
| 13 | Add Sun et al. 2022 (UNVERIFIED) and Mellinger & Kumar 2011 (UNVERIFIED) as foundations | related work | 2 | via R06/R07/R16 |
| 14 | Add a terminology note on "reactive" (Chee's usage; $\mathcal L_1$ as a reactive estimator) | terminology | 1 or 2 | R09, R10 |
| 15 | Add NS2's split-by-piece precedent to Ch5 §5.7 and NS2's cuboid interaction volume to Ch3 §3.4.3 | supporting evidence | 3, 5 | R02 |
