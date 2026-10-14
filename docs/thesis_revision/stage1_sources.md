# Stage 1 — Source records, one per paper, from the PDFs

Independent reading: no `docs/*.md` note, no `papers/summaries/`, no `20_Verified_Claims.md` was opened in this stage.
Text was extracted with `pdftotext -layout` (all pages, references and appendices included); page images were
viewed where a table or figure had to be read. Claim IDs (C1.xx … C5.xx) refer to the claim inventory in
`stage0_draft_audit.md` §5. Verdicts: **supported** / **supported with changed context** / **overstated** / **wrong** /
**cannot tell**.

---

## R01 — shi2020neuralswarm

1. **Reading record.** arXiv:2003.02992v1 (6 Mar 2020), 7 pages, all read incl. references. Figures 1–4 are
   images (heat maps, time series); only captions and text read. Table I read in full.
2. **Bibliographic check.** Authors Guanya Shi, Wolfgang Hönig, Yisong Yue, Soon-Jo Chung ✓; title ✓; year 2020 ✓;
   arXiv ✓. The PDF carries no venue; the published version is ICRA 2020 (not verifiable from the PDF — `[VERIFY]` if
   a venue is wanted; the bib currently cites arXiv, which is safe).
3. **What the paper does.** Learns the inter-vehicle interaction force with a deep-sets network
   $\hat f_a = \rho(\sum_j\phi(x^{(ij)}))$ on relative position and velocity, trained offline on flight data where the
   label is $f_a = m\dot v - mg - Rf_u$ computed from a calibrated thrust model. Both networks are spectrally
   normalised for a Lipschitz bound. The prediction is subtracted in a decentralised nonlinear (composite-variable,
   integral) position controller; any attitude controller can follow, the baseline uses a nonlinear attitude tracking
   controller [24 = Lee et al. CDC 2010]. A stability theorem gives exponential convergence to an error ball under a
   bounded-rate learning-error assumption. Flight tests with 2–5 Crazyflie 2.0 on a swapping task.
4. **Setup facts.**
   - Platform: "slightly modified Crazyflie 2.0 … small (9 cm rotor-to-rotor) and lightweight (34 g)" (§V); motors
     replaced by more powerful ones, thrust-to-weight 2.6 (§V-A). Mocap 100 Hz; controller, EKF and network on the
     STM32 (§V). Each network ≈550 µs, $f_a$ in <4 ms for ≤6 neighbours (§V-B).
   - Network: φ 6→25→40→40→40, ρ 40→40→40→40→1, ReLU; **only the z-component of $f_a$ learned** ("we only learn the z
     component of fa since we found the other two components … very small", §V-B).
   - Data: random walk and swapping, 2–4 vehicles, "one minute of data for each scenario" (§V-B); curriculum: data
     re-collected with the learned model as feed-forward (§III-C). uSD logging ≈ every 10 ms (§V).
   - Controller: $f_d = m\dot v_r - Ks - mg - \hat f_a$ (eq. 9), with $g = [0,0,-g]^\top$ — the prediction is
     **subtracted** in a z-up frame. Baseline: same controller, $\hat f_a\equiv 0$, nonlinear attitude tracking
     controller [24]; "all controllers, including the baseline controller, always have integral control compensation
     parts" (§V-C).
   - Experiment: swapping task, vertical distances "0.2 m to 0.25 m between neighbors" (§V-C); Fig. 2: three vehicles
     "25 cm vertical separation". Six repetitions per controller; metric = maximum z-error of any vehicle over the
     flight.
   - Headline (Table I, body): 2 CF baseline 0.094 m, trained 2 CF 0.027, 3 CF 0.026, 4 CF 0.024; 3 CF swap baseline
     0.139, trained-2 0.150; 4 CF 0.209 vs trained-2 0.294; 5 CF 0.314 vs trained-3 0.159, trained-4 0.150.
     Text: "reduces the worst-case z-error by a factor of two to four (e.g., 2.4 cm instead of 9.4 cm …)". Abstract:
     "up to four times smaller worst-case height tracking errors". Conclusion: "by a factor of two or better".
   - Physics remark (§I): "the downwash for two Crazyflie quadrotors hovering 30 cm on top of each other is only −9 g".
5. **Stated limitations.** Cannot guarantee $\tilde p\to 0$ since $\hat f_a\ne f_a$ (§IV-B); larger errors for 3–5
   vehicles "because we occasionally saturate the motors" (§V-C); for vehicles 3–4 in the 4-CF swap "$f_{a,z}$ is so
   high such that we cannot fully compensate it within our thrust limits" (§V-D); future work: heterogeneous swarms,
   planning, learn $\tau_a$ (§VI).
6. **Quotable claims.**
   - "one must require a large safety distance between vehicles, e.g., 60 cm for the small Crazyflie 2.0 quadrotor (9 cm rotor-to-rotor) [4]" (§I; [4] = Hönig et al., T-RO 2018, trajectory planning for quadrotor swarms).
   - "the downwash for two Crazyflie quadrotors hovering 30 cm on top of each other is only −9 g, which is well within their thrust capabilities" (§I).
   - "Note that our data collection and learning are independent of the controller used and independent of the fa compensation. In particular, if we actively compensate for a learned fa, this will only affect fu in (1a) and not the observed fa." (§III-C)
   - "for the case with more than two vehicles, the aerodynamic effect is not a simple superposition of each pair" (§V-C).
   - "the performance degrades significantly when only trained on two quadrotors, since the training data does not include data on superpositions" (§V-C).
   - "In this work, we only learn the z component of fa" (§V-B).
   - "Though an integral gain can cancel steady-state error during set-point regulation, it can struggle with complex time-variant interactions between vehicles." (§V-C)
   - "We observe that the interaction between quadrotors is non-stationary and sensitive to relative velocity, as well as not a simple superposition between pairs." (§V-D)
7. **What it does NOT show.** No comparison against a reactive/INDI method; no lateral (x, y) compensation; no
   moment compensation; the 60 cm figure is the planning literature's, not measured here; the "factor two to four"
   is for networks trained on enough vehicles (the 2-CF-trained network is worse than baseline at 3–4 CF); no
   heterogeneous teams (that is NS2).
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C1.01, C2.01 (CF2.0 9 cm; 60 cm cited from planning work) | supported | §I, ref [4]. "conservative planning gap" is a fair gloss of "large safety distance … [4]". |
| C1.04 (×2 or better, up to ×4) | supported | abstract + §V-C + §VI. |
| C1.11, C2.38, C3.06 (not a simple superposition) | supported | §V-C quote; also §V-D. |
| C2.20 (nominal model + regularised permutation-invariant DNN; FF in decentralised nonlinear tracking position controller; geometric only optional attitude inner loop; vertical $f_{a,z}$; onboard STM32) | **supported with changed context** | All true, but "geometric SE(3) control appears only as an optional attitude inner loop" understates it: the paper lists [24] as one possible attitude controller, and its **baseline and experiments use** "a nonlinear attitude tracking controller [24]" (= Lee et al.). Corrected: "… the attitude loop is left open in the design ('any attitude controller'), and the experiments use the nonlinear attitude tracking controller of Lee et al.". The position controller is a composite-variable integral controller ("nonlinear feedback linearization controller", §IV). |
| C2.21 (separate networks on 2, 3, 4; ×2 or better, up to ×4) | supported | Table I; §V-C. |
| C2.22 (9.4 → 2.4 cm 4-CF-trained; 2.7 cm 2-CF-trained) | supported | Table I column "2 CF Swap". |
| C2.23 (0.20–0.25 m; 25 cm for three) | supported | §V-C; Fig. 2 caption. |
| C2.24 (3/4-trained improve 5-CF; 2-trained worse than baseline at 3 and 4) | supported | Table I. |
| C2.44 (FF into nonlinear tracking controller) | supported | eq. (9). |
| C3.01 (Neural-Swarm compensates the vertical interaction force) | supported | §V-B. |
| C4.07 (NS line: predict from relative states with permutation-invariant net, as FF) | supported | |
| Tab 2.1 row (2–4 train, 2–5 test; 0.20–0.25 m vert.; max $\lvert z\rvert$ error) | supported | "max. z-error of any vehicle"; training scenarios 2–4 vehicles (random walk + swap). |
| **Ch4 §4.2 "Role" L190–192; Ch5 §5.5.2 L255–258; Ch4 §4.4 L262–264** ("a residual observed under a compensating controller is the residual that survived compensation, which is not what a predictive model should be fitted to") | **contradicted by this source** (thesis argument, not a citation) | Shi et al. §III-C: with $f_a$ computed from the applied thrust, compensation "will only affect fu … and not the observed fa". The thesis' own definition (eq. model-ares, thrust from **measured** rotor speeds, `lib.rs:1781–1790`) has the same property. Valid reasons to collect under the baseline remain (state-distribution and excitation differences, saturation, not feeding a model's own errors back into its training states, a clean prediction-vs-measurement comparison when the prediction does not move the vehicle), but "the residual that survived compensation" is not one of them. → DECISION row (argument). |
| Ch3 §3.3.2 sign convention (residual subtracted) | **supported by analogy** | eq. (9) subtracts $\hat f_a$ in a z-up frame with $g=[0,0,-g]^\top$ — the same sign the thesis derives. Usable as a literature anchor for the sign derivation. |

9. **Confidence and gaps.** High. Venue (ICRA 2020) not stated in the PDF.

---

## R02 — shi2022neuralswarm2

1. **Reading record.** arXiv:2012.05457**v2** (16 Jul 2021), 16 pages with IEEE T-RO running header, all read incl.
   references and biographies. Figures 1, 3, 5–9 are images (captions read); Tables I–III read.
2. **Bibliographic check.** Authors Guanya Shi, Wolfgang Hönig, Xichen Shi, Yisong Yue, Soon-Jo Chung ✓; title ✓.
   `references.bib` has `year = 2020` (arXiv v1); the held PDF is v2 (2021) formatted for IEEE T-RO; the T-RO volume/year
   is not printed in the PDF. The bib's own `yearnote` says "cite the T-RO version" but the entry is `@misc` with year 2020,
   so `plain` prints "2020" while the key says 2022 → **suspected bib error** (`[VERIFY]` T-RO 2022, vol. 38, no. 2 —
   from memory, not from the PDF).
3. **What the paper does.** Extends Neural-Swarm to heterogeneous teams with "heterogeneous deep sets" (one φ per
   neighbour type, one ρ per own type, the environment as a special neighbour for ground effect and drag), proves the
   representation result, trains five spectrally normalised networks on 12 scenarios with 1–3 vehicles, and uses the
   learned $f_{a,z}$ both in a two-stage interaction-aware motion planner (sampling + SCP) and in a delay-compensated
   nonlinear tracking controller. Flight tests: swaps of 2–3 vehicles of two sizes, a 5-robot ring and a 16-robot
   three-ring task.
4. **Setup facts.**
   - Platforms (Table I, §VII): small = Crazyflie 2.X, 34 g, "9 cm rotor-to-rotor", diameter 12 cm, max thrust 65 g;
     large = Crazyflie 2.1 board on a larger frame, 67 g, 145 g thrust, 19 cm. Some small vehicles with upgraded motors
     ("S*", thrust-to-weight 2.6). Single marker, mocap 100 Hz; controller, EKF and networks on the STM32 (§VII).
   - Model: $f_a$ only, **z-component only** ("we only learn the z-component of $f_a$ … the output of our learning model … is a scalar", §VII-B);
     $\phi$: 6→25→40→40→H, $\rho$: H→40→40→40→1, ReLU, **H = 20, L = 4** chosen by ablation (Table III, §VII-C);
     relative state $x^{(ij)} = [p^{(j)}-p^{(i)}; v^{(j)}-v^{(i)}]$ in world frame; environment neighbour with
     $p^{(j)}=0, v^{(j)}=0$ (§VII-B). Neighbour set defined by an **interaction volume V, "a cuboid based on observed interactions"** (§III-B).
   - Data: 12 scenarios (Table II), "a trajectory with a duration around 1000 seconds" each, 1.4 million pairs; speeds
     0–2 m/s, 7 % ≥0.5 m/s; **split: each scenario cut into 50 pieces of ≈20 s, 80 %/20 % by piece** (§VII-C); training
     vehicles "1-3" ("training data (up to 2)" neighbours, §VII-E).
   - Label: $y = \dot x - \Phi$ from calibrated nominal dynamics incl. motor delay (§VII-B); load-cell calibration of thrust
     (§VII-A).
   - Controller: position law (18) $f_d = -mg + m\dot v_r - \hat f_a - \dots$ — prediction **subtracted**, z-up gravity
     $g = [0;0;-g]$; attitude law (20) from Bandyopadhyay et al. [41] (quaternion error), **not** a geometric SE(3) law;
     explicit motor-delay compensation (21). Baseline identical incl. gains with interaction force set to zero (§VII).
   - Results (body): swaps with vertical distances 0.2–0.3 m (§VII-E); L2S median max-z error 17 → 7 cm (§VII-E,
     Fig. 8); 5-robot ring: small vehicle with powerful motors 10 → 5 cm (Fig. 9); 16 robots: "up to 15" neighbours vs
     "up to 2" in training, "making the prediction of $f_{a,z}$ relatively less accurate. However, the maximum z-error of a
     small multirotor in one of the vertical rings with powerful motors is still reduced from 15 cm to 10 cm" (§VII-E).
   - Abstract: "spectral normalization for stability and generalization guarantees of unseen data"; "up to three times
     reduction in worst-case tracking errors". **The 24 cm figure is in the Fig. 1 caption and the contribution list
     (§II end, item iv), not in the abstract.**
5. **Stated limitations.** Theoretical analysis of generalisation "an interesting future research direction"
   (§IV-B); prediction at 16 robots relatively less accurate (§VII-E); compensation helps "if the maximum thrust
   capabilities of the robots are sufficient" (§VIII); stability rests on Assumption 1 (bounded approximation error and
   its rate), which "relies on spectral normalization" (§IV-C, §VI-B).
6. **Quotable claims.**
   - "Without accurate downwash interaction modeling, a large safety distance between vehicles is necessary … e.g., 60 cm for the small Crazyflie 2.0 quadrotor (9 cm rotor-to-rotor) [4]." (§I)
   - "the downwash for two small Crazyflie quadrotors hovering 30 cm on top of each other is only 9 g" (§I)
   - "In this paper, we design V as a cuboid based on observed interactions in experiments." (§III-B)
   - "Note that our data collection and learning are independent of the controller used and independent of the $f_a$ or $\tau_a$ compensation." (§IV-D)
   - "In this work, we only learn the z-component of $f_a$ since we found the other two components, x and y, are much smaller and less varied" (§VII-B)
   - "robots can have significantly more neighbors (up to 15) compared to the training data (up to 2), making the prediction of $f_{a,z}$ relatively less accurate." (§VII-E)
   - "We note that a conceptually-simpler method is to estimate and compensate for $f_{a,z}$ online without learning. However, online estimation will not only introduce significant delays, but also be very noisy especially in close-proximity flight. … In experiments, we observe that the online estimation and compensate method would quickly crash the drone." (§VII-E, last paragraph)
   - "We can also observe that the interactions are not a simple superposition of different pairs." (§VII-C; maximum gap to the superposition version 11.4 g)
   - "we retrain neural networks neglecting relative velocity and the mean squared validation error degrades by 18%" (§VII-C)
   - "The bounded approximation error assumption (Assumption 1) in our control stability and robustness analysis (Sec. VI-B) also relies on spectral normalization of DNNs." (§IV-C)
7. **What it does NOT show.** No reactive (INDI) comparison — the online-estimation remark is one sentence with no
   estimator, filter or data given; no formal generalisation guarantee on unseen data in the body; no lateral force
   model; the 16-robot "15 → 10 cm" is one small vehicle with upgraded motors, not the team; the "three times" figure is
   abstract-level and not tied in the body to one experiment.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C1.05 (16 vehicles, min vertical 24 cm) | supported (add "heterogeneous") | Fig. 1 caption; §II item iv. |
| C2.25 (heterogeneous deep sets, any number of neighbours) | supported | §IV-B, Theorem 4. |
| C2.26 (trained on ≤3 vehicles / ≤2 neighbours; 16-robot prediction relatively less accurate) | supported | §VII-C, §VII-E. |
| C2.27 (SN; abstract guarantees; body Lipschitz + empirical + bounded-residual assumption) | supported | abstract; §IV-C; Assumption 1. |
| C2.28 (planning and control) | supported | §V, §VI. |
| C2.29 ("The two figures are reported in the abstract") | **wrong in detail** | Only the "three times" figure is in the abstract; 24 cm/16 robots are in Fig. 1 and §II. Corrected: "the threefold figure is the abstract's; the sixteen-robot, 24 cm flight is reported in the introduction and Fig. 1, and we do not assume the two describe the same experiment". |
| C2.30 (16-robot body: max z-error small vehicle 15 → 10 cm, ≈1.5) | supported with changed context | add "with upgraded motors, in one of the vertical rings". |
| C2.44 (FF into nonlinear tracking controller) | supported | eq. (18). Note the heading "Feedforward into a **geometric** controller" (2.5) is not NS2's attitude law (Bandyopadhyay et al. quaternion law); the paragraph text says "nonlinear tracking controller" — keep the text, retitle or qualify. |
| C3.05 ("not a copy of NS2's layers", "of our own sizing") | **wrong for the current code** | The firmware network is exactly NS2's $\phi$ 6→25→40→40→20 / $\rho$ 20→40→40→40→1 with H = 20, z-only output, small/large/environment networks (§VII-C; `residual_nn.rs:17–30`). |
| C4.08 ("NS2 applies spectral normalisation to obtain stability and generalisation guarantees") | **overstated** (drops C2.27's hedge) | Corrected: "applies spectral normalisation, which its abstract associates with stability and generalisation guarantees and on which its bounded-error assumption rests". |
| Tab 2.1 row (16, heterogeneous; 24 cm vert.; worst-case tracking error †) | supported | footnote correct. |
| Ch3 3.4.3 guard (2 m cut-off) / code gate | **source for the code's gate** | NS2 defines neighbours by a cuboid interaction volume (§III-B); the dimensions $\lvert\Delta x\rvert,\lvert\Delta y\rvert<0.2$ m, $\lvert\Delta v_x\rvert<1.5$ m/s are not in the paper; they are in the authors' public code as copied into `crazyswarm2/crazyflie_sim/crazyflie_sim/backend/neuralswarm.py:78` ("Logic copied from https://github.com/aerorobotics/neural-swarm/blob/master/planning/robots.py"). Cite the paper for the cuboid concept and the code for the numbers. |
| Ch5 §5.7 split by contiguous segment | **supporting precedent** | NS2 splits each scenario into 50 ≈20 s pieces and assigns pieces, not samples (§VII-C). |
| Ch5 §5.6 C5.01 "a published downwash model" | **source** | the simulator backend evaluates the NS2 architecture with weights loaded from a model folder (`neuralswarm.py:62–72`); cite shi2022neuralswarm2 for the model; `[VERIFY]` that the weight files are the authors' published ones. |
| Ch2 §2.3 limitations of reactive compensation (missing) | **addition candidate** | NS2's §VII-E remark that online estimate-and-compensate "would quickly crash the drone" — one sentence, no method given; usable only with that caveat. |
| Ch4 §4.2 / Ch5 §5.5.2 rationale "residual that survived compensation" | **contradicted** (as R01) | §IV-D quote. |

9. **Confidence and gaps.** High for content. T-RO volume/year not in the PDF.

---

## R03 — gielis2023aggregate

1. **Reading record.** arXiv:2312.03488v1 (6 Dec 2023), 12 pages, all read. Figures 1–7 images (captions read); Table 1 read.
2. **Bibliographic check.** Jennifer Gielis, Ajay Shankar, Ryan Kortvelesy, Amanda Prorok ✓; title ✓; 2023 ✓; arXiv ✓.
   Springer-style (LNCS-like) layout; venue not printed — `[VERIFY]` if a venue is wanted.
3. **What the paper does.** Builds a load stand with eight load cells that measures the 6-DOF exogenous wrench on a
   multirotor ("sufferer") spinning at hover thrust while one to three other multirotors fly patterns around it (side
   by side, leader–follower, stack, triangle). It characterises the forces and compares three predictors of the
   aggregate: a naïve sum of a single-neighbour model, a learnt linear (sum of a learnt per-neighbour model) and a learnt
   nonlinear deep-sets model; the nonlinear model has the lowest error, mainly in the vertical axis.
4. **Setup facts.**
   - Rig (§3): 8 shear-beam load cells (10 kg), HX711 at 80 Hz, Raspberry Pi 4B; stand 1.4 m above ground; NED frame;
     self-calibration zeroes the D-axis with the sufferer at hover thrust. **Sufferer platform 700 g** (§5: "Given our
     700 g platform"); originating multirotor "< 30cm in total diameter" (§5).
   - Flights: formations flown "in a grid-like manner" over 5.6 m³, 2 m square laterally, 1.4 m vertically above the
     stand; 0.5 m/s, 30–40 legs per experiment; mocap 180 Hz (§3).
   - Evaluated cases (Table 1): Side by Side K = 2, Stack K = 2, Leader Follower K = 3, at relative altitude 1.3 m;
     conclusion: "formations of up to 4 multirotors in total" (§6).
   - Results: three regimes (§4): "Chaotic Interactions: those that are dominated by noise; Linear Interactions: those
     that are well estimated by linear summation; and, Non-linear Interactions: those that can only be estimated with
     non-linear models"; linear when one neighbour is much closer (§4.2); summation errors "up to ≈ 3 N … ≈ 4 m/s² … ≈ 75 %
     of peak forces" (§5); measurement noise ±0.05 N (§4.1).
5. **Stated limitations.** "only analysed a limited sample of velocities" (§6); "our deep-sets based non-linear model may
   be incapable of generalising to cases where K > 3" (§6); the stand itself modifies the airflow ("observer effect",
   §4.1); N-axis vs E-axis differences attributed to the setup (§4.4).
6. **Quotable claims.**
   - "even in a homogeneous team, an accurate model of downwash forces from one vehicle is unlikely to be sufficient for predicting aggregate forces from multiple vehicles in formation." (abstract)
   - "reactive controllers are not able to reject them without first observing the disturbance." (§1)
   - "This makes the assumption that downwash forces from the K neighbours are additive, or at least, linearly summable. While the assumption might hold for a variety of flight regimes/formations, there is a dearth of empirical studies that explore this further." (§1)
   - "To a (learnt) non-linear model, on the other hand, an input with a different K is completely out of distribution." (§2)
   - three-regime list (§4, quoted above).
   - "The downwash airflow of multirotors in near proximity tend to merge laterally, and reduces in overall cross-sectional area with increasing distance from the generating multirotors." (§4.3)
   - "Multirotor downwash fields expand very little with vertical distance, and remain concentrated approximately within the body radius up to at least 1.5m" (§5)
   - "The aerodynamic influence of a multirotor is negligible to its neighbours outside of its downwash (Fig. 4), including at very small (< 1 vehicle diameter) separations." (§5)
   - "simple summation of forces from single vehicle models is insufficient when a sufferer is subjected to multiple downwash zones because it can result in force estimation errors up to ≈ 3 N in our datasets." (§5)
7. **What it does NOT show.** No closed-loop control and no free-flying sufferer; no Crazyflie-class sufferer (700 g); no
   held-out generalisation test beyond K = 3; no comparison with a flight-controller residual.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C1.11, C3.08 (summation known approximation) | supported | abstract; §1; §5. |
| C2.08 (6-DOF on load stand at hover thrust, up to three further vehicles flying freely; strong nonlinearities) | supported (add platform) | §3; abstract; add "a 700 g sufferer". |
| C2.09 ("neither validates a learned model of ours or anyone else's") | **overstated / wrong for Gielis** | Gielis et al. train and evaluate **their own** learnt linear and nonlinear models against the stand (§4.4, Table 1, Fig. 7). Corrected: "Gielis et al. evaluate their own learnt aggregation models against the stand; neither paper evaluates a model learned from flight-controller residuals." |
| C2.39, C3.06 (motivation quote) | supported | abstract. |
| C2.40 (graph framing in abstract; deep-sets sum-pooling vs linear summation) | supported | abstract; eqs. (1)–(2). |
| C2.41 ("Generalisation to a fourth vehicle is claimed at abstract level, while the reported tables stop at three neighbours") | **overstated** | The abstract says the authors "explore how our proposed models generalize when a fourth vehicle is added to the formation" — "explore", not a claim; and K = 3 neighbours **is** four vehicles in total (Table 1 Leader Follower K = 3; §6 "up to 4 multirotors in total"). Corrected: "The abstract announces an exploration of generalisation when a fourth vehicle is added; the largest reported case is three neighbours around the stand (four vehicles in total), and the conclusion cautions that the deep-sets model 'may be incapable of generalising to cases where K > 3'." |
| C2.42 (three regimes) | supported | §4 verbatim. |
| C3.03 (measure force and moment on a rig) | supported | 6-DOF incl. pitch, roll, yaw torques (§3). |
| Tab 2.1 row ("up to 4 total (rig sufferer + ≤3 flyers)"; "test rig, sufferer not in free flight"; "force-prediction accuracy") | supported | add "700 g sufferer". |
| C2.06 (decays quickly with lateral offset) — uncited in the draft | **usable source** | §5 bullets (concentrated within body radius; negligible outside the downwash). |
| Ch5 null scenarios C1 (side-by-side) "no significant inter-vehicle interaction should exist" | **supported with a caveat** | §5: negligible outside the downwash "including at very small separations", but "lateral forces being exerted at significant distance … generally chaotic and of a small magnitude". |

9. **Confidence and gaps.** High. Venue not in PDF.

---

## R04 — smith2023so2equivariant

1. **Reading record.** arXiv:2305.18983**v3** (25 Mar 2024), 8 pages, all read. Figures 1–7 images (captions and
   axis text read); the Fig. 3 summary table read.
2. **Bibliographic check.** H. Smith, A. Shankar, J. Gielis, J. Blumenkamp, A. Prorok ✓; title ✓; arXiv 2023 (v3 2024) ✓;
   no venue printed — `[VERIFY]` venue if wanted.
3. **What the paper does.** Proposes a downwash model for a follower ("Bravo") under a leader ("Alpha") that is
   translation-invariant and SO(2)-equivariant about the leader's vertical axis: inputs are mapped to six invariant
   features, a shallow network (258 parameters) predicts two components, and the lateral direction is restored from the
   polar angle. Labels are $f_{\mathrm{ext}} = a - u$ from an LQR on a feedback-linearised double integrator. Data are
   collected in three stages of decreasing separation; the model is compared with shallow and deep (Neural-Swarm-style)
   non-equivariant networks offline and in two flight experiments.
4. **Setup facts.**
   - Platform (§V-C): two identical custom quadrotors, "span 0.26 m on the longest body-diagonal, and weigh ≈ 0.7 kg";
     Raspberry Pi 4B runs control, estimation and model; OptiTrack; "controller and the model iterations are performed
     at 50 Hz and 45 Hz respectively"; NED frame (§II).
   - Controller: LQR on (2) with compensation $u_f = -K(x - x_r) - f_{\mathrm{ext}}$ (§II) — prediction subtracted.
   - Data (§V-A): stage-0 vertical separation 1.75 m to 1.35 m; three stages; "at stage-2 the vehicles have a vertical
     separation of only ≈ 0.5 m (approx. two body-lengths)"; full dataset ≈17 min.
   - Offline (Fig. 3 table): validation RMSE at 5 min: equivariant 0.222 vs shallow 0.263, deep 0.255; at 15 min:
     0.218 vs 0.243, 0.229.
   - Flights (§V-C): lemniscate with Alpha hovering at $\hat e_3 = -2.5$ m (Fig. 6): equivariant 3-D position error
     0.181 → 0.113 m (37 %), velocity 36 %, vertical "nearly 51 % in both position and velocity"; translation transect at
     $\hat e_1 = 0.2$ m, vertical separation 0.6 m (Fig. 7): "reduction in the mean 3D positioning error to 0.098 m (from
     0.154 m), corresponding to an improvement of almost 36 %"; velocity 34 %; vertical only 55 % (position) and 49 %
     (velocity). Fig. 5: predictions during a transition at 0.8 m separation.
   - Abstract: "improves 3D trajectory tracking by nearly 36 % on average (and vertical tracking by 56 %)";
     Contributions (§I-B): "reduce vertical tracking errors by 56 % and **lateral** tracking errors by 36 %".
5. **Stated limitations.** Assumption 2 "is unlikely to hold when Alpha's rotor speeds are highly asymmetric (ex.
   during aggressive maneuvering)" (§III-B); near-hover leader assumed in experiments (§V); multi-vehicle extension
   open; summing a two-vehicle model "may" be insufficient because fields "interact in highly nonlinear ways" (§VI).
6. **Quotable claims.**
   - "when trained with only 5 minutes of real-world flight data, our geometry-aware model outperforms state-of-the-art baseline models trained with more than 15 minutes of data." (abstract)
   - "In dense real-world flights with two vehicles, deploying our model online improves 3D trajectory tracking by nearly 36 % on average (and vertical tracking by 56 %)." (abstract)
   - "our model's predictions reduce vertical tracking errors by 56 % and lateral tracking errors by 36 %." (§I-B)
   - "Across experiments, we observe a reduction in the mean 3D positioning error to 0.098 m (from 0.154 m), corresponding to an improvement of almost 36 %" (§V-C)
   - "The labels that our model (5) learns to approximate are obtained from the feedback control equation (2), $f_{\mathrm{ext}} = a - u(t)$." (§IV-C)
   - "at stage-2 the vehicles have a vertical separation of only ≈ 0.5 m (approx. two body-lengths)." (§V-A)
   - "One naïve approach to modelling multi-vehicle downwash would be to employ our two-vehicle model and sum the individual force contributions … However, it may be the case that individual downwash fields interact in highly nonlinear ways" (§VI)
7. **What it does NOT show.** No Crazyflie-class vehicle; no comparison with a reactive/INDI method; no multi-vehicle
   (>2) flights; the 56 % vertical figure appears in the abstract and contributions only — the body reports 55 % and
   ≈51 %; residual labels from commanded acceleration, not rotor speeds.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C1.06, C2.32, C2.71 (36 % 3-D, 56 % vertical, average, pair) | **supported with changed context** | 36 % 3-D: abstract + body (translation, 0.154 → 0.098 m). **56 %: abstract and contributions only**; body gives 55 % (translation) and ≈51 % (lemniscate). Corrected: "… by 36 % in 3-D and, as stated in the abstract, 56 % vertically (the body reports 55 % for the translation experiment)". |
| C1.09 (36 % on two vehicles in close proximity) | supported | |
| C2.31 (5 min vs >15 min, prediction accuracy) | supported | abstract; Fig. 3. |
| C2.33 (contributions list says lateral) | supported | §I-B. |
| C2.34 (0.154 → 0.098 m; 55 % and 49 %) | supported (clarify) | add "(vertical position and velocity, respectively)". |
| C2.35 (0.26 m span, 0.7 kg, OptiTrack, RPi 50 Hz, 45 Hz model) | supported | "longest body-diagonal". |
| C2.36 (train ≈0.5 m, two body-lengths; eval 0.6–0.8 m; lemniscate beneath hovering vehicle at z = −2.5 m) | supported | "z = −2.5 m" is the leader's NED position; Bravo's lemniscate altitude is not given. Suggest "beneath the leader hovering at 2.5 m (NED $z = -2.5$ m)". |
| C2.37 (latent geometry) | supported | abstract. |
| Tab 2.1 row | supported | |

9. **Confidence and gaps.** High. Venue not in PDF.

---

## R05 — hsieh2026flatness

1. **Reading record.** arXiv:2607.12275**v2** (17 Aug 2026; dated 19 Aug 2026), 15 pages incl. Appendix A, all read.
   Figures 1–3 are images (captions read); Tables 1–4 read.
2. **Bibliographic check.** Pei-An Hsieh†, Fengjun Yang† (equal contribution), Nikolai Matni, M. Ani Hsieh ✓; title ✓;
   2026 ✓; arXiv ✓. No venue. (Its reference list gives venues for other bib entries: Yang et al. 2025a = **CDC 2025,
   pp. 3035–3042**; Bauersfeld et al. = **RA-L 10(2):1241–1248, 2024**; Neural-Swarm2 = T-RO 38(2):1063–1079, "2021" —
   secondary evidence only, `[VERIFY]`.)
3. **What the paper does.** Restricts a learned residual force/torque to depend only on the team's positions and
   velocities, which keeps each vehicle (and the team jointly) differentially flat with the usual flat outputs
   $[p,\psi]$; derives the modified flat maps; builds a cascaded feedback-linearisation (FBL) controller with PID on
   position, thrust and attitude from the modified maps, and a proportional angular-rate loop. The residual is a
   physics reduced-order model (jet theory from Bauersfeld et al. + drag from Jain et al., adapted from Chee et al.)
   plus a small network, trained with KNODE. Simulation (sweeps, NMPC head-to-head) and hardware (two Crazyflies,
   stacked formation, FBL and SE(3) variants).
4. **Setup facts.**
   - Frame: world ENU (§2). Dynamics (1)–(2) with residual added; thrust (14a) $u = \lVert m(a-g) - \hat f^a\rVert$ —
     the residual is **subtracted** in the thrust/attitude computation.
   - Controller: PID on position (17) with integral term; angular-velocity reference from (15),(10); torque (19)
     proportional on rate error with residual-torque feedforward (§3.3).
   - Simulation (§5.1): Crazyflie mass/inertia, downwash model of Chee et al. ($C_D$ 0.7 sim vs 0.3 ROM), networks of one
     hidden layer of nine neurons; **18 s** training data at 0.3 and 0.4 m (static-top and stacked); evaluation 0.2–0.5 m
     (prediction, Table 1), sweeps 0.2–0.6 m and 0.2–0.5 m/s (tracking, Tables 2–3); FBL at 100 Hz; full noise-free state.
     NMPC head-to-head with the same learned residual (Table 4): RMSE/zmax 0.016/0.042 vs NMPC 0.020/0.032 at 0.3 m;
     0.016/0.036 vs 0.018/0.019 at 0.5 m; solve time 0.99 vs 27.6 ms and 1.3 vs 26.2 ms ("20× reduction").
   - Hardware (§5.2): **two Crazyflie 2.1 with thrust upgrade bundles, 10 cm body length, 36 g**; Vicon at 120 Hz to a
     laptop (Intel i5) that "computes control commands that are transmitted to the quadrotors at 400 Hz via Crazyradio
     PA"; FBL outputs thrust and angular velocity, "we compensate exclusively for residual forces and rely on the
     high-frequency PID controller onboard to handle the residual torques"; network 2 layers × 4 neurons; **28 s (6 720
     points)** training at 0.4 m separation, 0.4 m/s, collected under an FBL controller "unaugmented by learned residual
     dynamics"; commanded separations 0.4 m (in-distribution) and 0.3 m (OOD); 60 runs, five repetitions per case.
   - Hardware results (body): ROM-FBL vs Nominal-FBL −24 % RMSE, −30 % zmax; Learned-FBL vs ROM-FBL a further −29 % RMSE,
     −43 % zmax; **Learned-SE(3) and ROM-SE(3) −43.7 % and −34 % RMSE vs nominal SE(3)**; Learned-FBL average RMSE 5 cm,
     zmax 10 cm, which "match the reported tracking performance of KNODE-DW MPC (Chee et al., 2024)"; window demo merging
     1.1 m → 0.1 m.
   - Abstract: "reduces average tracking errors by 31% compared to nominal baselines"; "matches the tracking performance of
     … NMPC while requiring an order of magnitude less computation"; "under 30 seconds of training data and a 5ms loop
     rate". Introduction (§1.1.1): "We directly compare our approach with this baseline **in an identical hardware
     setup** and show that our proposed controller matches their tracking performance while using 20× less compute" —
     in tension with §5.2, where the hardware match is against **reported** numbers and the head-to-head is simulation.
5. **Stated limitations.** Exact FBL torque would need second derivatives of the residual — replaced by a high-gain
   rate loop (§3.3); restricted expressivity of kinematic features (§2, §4); ROM slightly better than learned at 0.5 m
   for angular acceleration, "mildly overfit" (§5.1); future work: relax structural assumptions, scale up (§6).
6. **Quotable claims.**
   - "Hardware experiments demonstrate our framework reduces average tracking errors by 31% compared to nominal baselines." (abstract)
   - "We are the first to show that stable, tight formation flight can be achieved with under 30 seconds of training data and a 5ms loop rate" (abstract)
   - "we restrict our residual models … to depend exclusively on the joint kinematic state of the swarm" (§3.2)
   - "the FBL controller was configured to output thrust and angular velocity commands rather than thrust and torque. Therefore, we compensate exclusively for residual forces and rely on the high-frequency PID controller onboard to handle the residual torques." (§5.2)
   - "This base station computes control commands that are transmitted to the quadrotors at 400 Hz via Crazyradio PA." (§5.2)
   - "Overall, our Learned-FBL controller achieved an average RMSE of 5 cm and a zmax of 10 cm. These results match the reported tracking performance of KNODE-DW MPC (Chee et al., 2024)" (§5.2)
   - "the Learned-SE(3) and ROM-SE(3) configurations reduced RMSE by 43.7% and 34%, respectively, compared to the nominal baseline." (§5.2)
   - "our JIT-compiled PyTorch-based controller still achieved a 20× reduction in solve time." (§5.1)
   - "rather than explicitly modeling residuals from data offline, an alternative is to actively estimate and reject the unmodeled dynamics online (Hsieh et al., 2025 …). Integrating online estimation with our framework remains a promising direction" (§1.1.2)
7. **What it does NOT show.** No onboard execution; no three-vehicle hardware; no head-to-head NMPC on hardware (in
   the body); no reactive/INDI comparison; the 31 % aggregate is not restated in the body; the 5 ms loop rate is not
   tied to a body measurement (the body reports 400 Hz command transmission).
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C1.09 (31 % in the abstract of a tight-formation study) | supported | abstract. |
| C2.47 (residual keeps joint system flat; FBL cancels by feedforward) | supported | abstract; §3. |
| C2.48 (abstract 31 %; body 24 %/30 %, 29 %/43 %) | supported | §5.2. |
| C2.49 (hardware match vs published Chee; sim max-z can be worse than NMPC) | supported, **add the introduction's contrary wording** | §5.2 vs §1.1.1; Table 4. Suggested: "…the body's hardware comparison is against Chee et al.'s reported figures (the introduction describes an identical-setup comparison); the head-to-head with NMPC is in simulation, where the flatness-based controller's max-z error is larger." |
| C2.50 (20-fold, ≈1 vs 27 ms simulation; order of magnitude in abstract/intro; 28 s HW, 18 s sim) | supported | Table 4; §5.1; §5.2. |
| C2.51 (two CF2.1 thrust-upgrade; off-board i5; Crazyradio 400 Hz; 5 ms abstract-level off-board) | supported | §5.2; abstract. |
| C2.52, C3.02 (forces only on HW; torques to onboard rate/PID loop; sim models torques) | supported | §5.2; §5.1. |
| C4.09 | supported | |
| **C4.10** ("**In simulation** they match the tracking error **published** by Chee et al. … from under 30 s of training data, at a 5 ms loop rate obtained off-board") | **wrong context** (and contradicts C2.49) | The match with *published* Chee figures is the **hardware** result (§5.2); the simulation result is a head-to-head with NMPC using the same residual (Table 4), where FBL's zmax is worse; "under 30 s" and "5 ms" are abstract-level. Corrected per C2.49–C2.51. → APPLY (reference-context fix). |
| Tab 2.1 row and footnote ‡ | supported | |
| Ch2 §2.5 "Feedforward into a geometric controller" (missing evidence) | **addition candidate** | §5.2: learned residual as feedforward in an SE(3) geometric controller (Lee et al.) reduced RMSE by 43.7 % on hardware — the published instance closest to Strategy 2's structure. |

9. **Confidence and gaps.** High. The paper is a 2026 preprint (v2), not peer-reviewed as far as the PDF shows.

---

## R06 — yang2025flatnessresiduals

1. **Reading record.** arXiv:2504.04324**v3** (27 Jan 2026), 17 pages incl. Appendices A–B, all read. Figures 1–3
   images (captions read); Tables 1–3 read.
2. **Bibliographic check.** Fengjun Yang, Jake Welde, Nikolai Matni ✓; title ✓; arXiv 2025 ✓. No venue printed; R05's
   reference list gives **IEEE CDC 2025, pp. 3035–3042** (`[VERIFY]` against the proceedings).
3. **What the paper does.** For nominal models in (MIMO) pure-feedback form, shows that an additive residual whose
   $i$-th block depends only on $x_1,\dots,x_i$ ("lower-triangular") keeps the system locally flat with the **same flat
   output**, and gives a recursive construction of the augmented flatness diffeomorphism from the nominal one
   (Theorems 1–3). Parameterises such residuals with smooth networks and learns them from one-step finite-difference
   labels. Validates on a simulated planar (2-D) quadrotor with drag.
4. **Setup facts.** Simulation only (§6): planar quadrotor, $m = 1$, $I = 0.1$, drag $\Delta_2(x_2) = -C_rx_2 - C_p\lVert x_2\rVert x_2$
   (residual only in the velocity equation, §6.1; worked example §4 uses $\Delta_2 = -(C_r/m)x_2$); 3 000 random
   trajectories of 0.5 s at 100 Hz; 1 hidden layer of 32 GeLU neurons; 30 seeds. Closed loop: flatness controller vs
   NMPC with the same learned model: tracking 0.0051/0.0061 m vs 0.0062/0.0063 m, nominal 0.0478/0.0258 m (Table 2);
   compute 3.3 ± 0.3 ms vs 85.2 ± 1.74 ms (§6.3). Abstract: "tracking error 5× lower than the nominal flat model, while
   being 20× faster over a structure-agnostic alternative".
5. **Stated limitations.** Local flatness (around an operating point); restrictive structural assumption ("may seem
   overly restrictive", §4); "Future work includes relaxing the structural assumptions, extending the algorithm to
   allow online adaptation …, and deploying the algorithm on quadrotors in physical experiments" (§7).
6. **Quotable claims.**
   - "generic residual parameterizations may destroy flatness, limiting the applicability of flatness-based planning and control methods." (abstract)
   - "We show that residuals with a lower-triangular structure preserve both the flatness of the system and the original flat outputs." (abstract)
   - "Our approach is validated in simulation on a 2D quadrotor subject to unmodeled aerodynamic effects." (abstract)
   - "Note that this class of residual dynamics also preserves the flat outputs of the nominal system." (§4)
   - future-work sentence (§7, quoted above).
7. **What it does NOT show.** No hardware; no 3-D quadrotor; no multi-vehicle or downwash case; global flatness not
   claimed.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence |
|---|---|---|
| C2.45 (lower-triangular preserve local flatness and flat output for pure-feedback; constructive procedure; structured correction; worked example only in the velocity equation) | supported | abstract; Thms 2–3; §4 example; §6.1. |
| C2.46 (simulation on planar quadrotor; no hardware, no 3-D, no multi-robot; physical quadrotor future work) | supported | abstract; §6; §7. |
| C4.09 (theory in simulation on planar quadrotor; HW demo is Hsieh et al.'s) | supported | together with R05 §1.1.2 ("demonstrate its first successful hardware application"). |
| Tab 2.1 row ("1, planar (sim. only)") | supported | |

9. **Confidence and gaps.** High. Venue from a secondary source only.

---

## R07 — cobobriesewitz2026lindi

1. **Reading record.** arXiv:2503.09441**v2** (14 Apr 2026), PMLR/L4DC 2026 layout, 15 pages incl. Appendices A–B, all
   read. Figures 1–6 images (captions with mean values read); Table 1 read.
2. **Bibliographic check.** Eckart Cobo-Briesewitz, Khaled Wahba, Wolfgang Hönig ✓; title ✓. The PDF header reads
   "Proceedings of Machine Learning Research vol vvv:1–15, **2026** 8th Annual Conference on Learning for Dynamics and
   Control". `references.bib` has `year = 2025` (arXiv v1) while the key and `yearnote` say L4DC 2026 → **suspected bib
   error** (`plain` prints 2025). Note: the third author is the thesis supervisor.
3. **What the paper does.** Compares, on a Crazyflie 2.1 with and without a 5 g slung payload, (i) a geometric (Lee)
   controller, (ii) "INDI" = the same controller with the residual force/torque estimated online from IMU and rotor
   speeds and fed forward, (iii) LINDI = a small MLP trained offline on spline-smoothed INDI residuals, run without RPM
   input, and (iv) NA-INDI = network plus an INDI term for the remaining mismatch. Two unseen test trajectories
   (figure-eight, circle).
4. **Setup facts.**
   - Platform (§5.1): Crazyflie 2.1, standard motors, arm 4.6 cm, 34.7 g; microSD deck; custom IR RPM board; OptiTrack
     100 Hz; 7.5 × 4 × 2.75 m flight space; Crazyswarm2. Networks run onboard; MLP inference 203 µs (§5.2).
   - Controller (8a): $f_u = (-K_pe_p - K_ve_v + mge_z + m\ddot p_d - f_a)\cdot Re_z$; (8b) with $-\tau_a$ — residual
     **subtracted**, z-up. INDI residual (10a): $f_a = m\dot v - f_uRe_z + mge_z - Tq$, $f_u$ from RPM via (3), Butterworth
     filtering (§4.2).
   - LINDI network (§4.3): MLP 19 inputs ($v, \dot v, \omega$, first two columns of $R$, motor PWM), 6 outputs, 2 hidden
     layers × 24, Leaky-ReLU; payload version 28 inputs, 16 hidden; labels spline-smoothed offline.
   - Data (§5.2): random spline trajectories in 1.6 × 1.6 × 0.4 m, up to 2 m/s (1.75 m/s with payload); 64 996 samples
     ≈ 12 min (no payload), 58 538 (payload); L1 loss.
   - Results: no payload (Fig. 5, 20 trials): Lee 7.07/7.45 cm, INDI 4.28/4.33, LINDI 4.41/4.87, NA-INDI 4.14/4.26
     (figure-eight/circle) — "around 40%"; payload (Fig. 6, 10 trials, payload position): Lee 7.06/7.26, INDI 5.38/5.91,
     LINDI 5.46/5.78, NA-INDI 5.39/5.74 (reductions 18.6–23.8 %). **INDI with PWM-derived thrust instead of RPM (Table 1):
     18.27/18.89 cm vs Lee 7.52/7.29 cm** — worse than the uncompensated geometric controller.
5. **Stated limitations.** "the RPM measurements are still required for training our learning methods" (§5.4.1);
   circle drop "likely because the trajectory is out of distribution relative to the training data" (§5.4.2); full
   cable-dynamics INDI "was not experimentally validated" (App. B.1.2); external aerodynamic disturbances left to future
   work (§6).
6. **Quotable claims.**
   - "we show that a neural network can generate smooth approximations of INDI outputs without requiring specialized rotor RPM sensor inputs." (abstract)
   - "The remaining forces, called residual forces, can arise from diverse sources such as blade flapping, drag, or ground forces" (§1)
   - "In the context of Unmanned Aerial Vehicles (UAVs), INDI typically relies on rotor RPM measurements to approximate the nominal model." (§1)
   - "residual prediction can reduce the tracking error to the desired flight trajectory by around 40%." (§5.4.2)
   - "for the circular trajectory, we observe a slight drop in performance, likely because the trajectory is out of distribution relative to the training data." (§5.4.2)
   - "the combination of sensor measurements plus learned residuals (NA-INDI) leads to the best performance, although with a small margin." (§5.4.2)
   - "NA-INDI shows the potential to outperform INDI, but still remains similar in performance." (§5.4.3)
   - "using LINDI and NA-INDI to learn the residual force calculations from the PWM-based INDI greatly improves flight performance compared to the PWM-based INDI but still underperformed when compared to the basic Lee controller with no residual prediction. This highlights INDI's dependence on accurate sensor measurements." (§5.4.1)
   - "For practical purposes, it is important to filter the measured values, e.g., by using a butterworth filter (Tal and Karaman, 2021)." (§4.2)
7. **What it does NOT show.** No multi-vehicle or downwash experiment; no inter-vehicle relative state; "ground forces"
   are listed as a generic source of residuals, not studied; no comparison under identical conditions with a
   predictive-from-neighbours model.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C1.07, C2.68 (hybrid on one vehicle; small margin without payload; similar with payload) | supported | §5.4.2–5.4.3 quotes. |
| C1.10 (hybrid demonstrated only where the residual is self-induced: single vehicle, blade flapping, drag, **ground forces**, optional slung payload) — uncited | **overstated detail; add citation** | "Blade flapping, drag, or ground forces" is the authors' generic list of residual sources (§1, citing others), not the conditions of their experiments. Corrected: "…on a single vehicle, where the residual consists of the vehicle's own unmodelled dynamics (the authors list blade flapping, drag and ground forces among the sources of such residuals) and, optionally, a slung payload~\cite{cobobriesewitz2026lindi}". |
| C2.19, C3.04 (NN approximates INDI residual without rotor-speed measurement) | supported | abstract. |
| C2.63 (LINDI trained on spline-smoothed data) | supported | §4.3. |
| C2.64 (NA-INDI split) | supported | §4.4. |
| C2.65 (single quadrotor, with/without slung payload; figure-eight and circle not in training set) | supported | §5. |
| C2.66 (headline is sensing; rotor speeds needed for labels; NA-INDI keeps incremental branch) | supported | abstract; §5.4.1. |
| C2.67 (≈40 % vs geometric; 19–24 % with payload) | supported | Fig. 5/6 means. |
| C2.69, C3.09 (LINDI circle drop, OOD) | supported | §5.4.2. |
| C2.70 (self-induced; predicted from own state) | supported with nuance | inputs are own state, own acceleration and PWM. |
| Tab 2.1 row | supported | |
| Ch3 §3.3.1 "It requires rotor-speed sensing" | **strong supporting evidence (addition candidate)** | Table 1: INDI with PWM-derived thrust performed worse than the uncompensated geometric controller on a Crazyflie 2.1 (18.27 vs 7.52 cm, circle). |
| Ch3 §3.3.2 sign | supported by analogy | (8a) subtracts $f_a$ in a z-up frame. |

9. **Confidence and gaps.** High.

---

## R08 — zhang2024proxfly

1. **Reading record.** arXiv:2409.13193**v2** (30 Apr 2025), 7 pages, all read. Figures 1–5 images (captions read);
   Tables I–II read.
2. **Bibliographic check.** Ruiqi Zhang, Dingqi Zhang, Mark W. Mueller ✓; title ✓; arXiv 2024 ✓; no venue printed.
3. **What the paper does.** Adds a PPO-trained residual policy to a cascaded (position–attitude) controller; the
   residual adds mass-normalised thrust and body-rate commands. The policy observes only the ego tracking error, the
   basic controller's command and its own previous action — no neighbour state, no communication — and is trained in
   simulation with randomised mass, inertia, propeller efficiency and synthetic (triangular) force and Gaussian torque
   disturbances. Real-world tests with a small (0.28 kg) and a large (0.85 kg) quadcopter: hover, circling (same and
   reversed direction) and mid-air docking.
4. **Setup facts.**
   - Platforms (Table I): small 0.280 kg, arm 0.058 m; large 0.850 kg, arm 0.165 m. Mocap 200 Hz; high-level controller
     and residual on a laptop at 50 Hz; low-level onboard at 500 Hz (§II-A).
   - Observation $o = \{\Delta s, u_{\mathrm{cas}}, a_{\mathrm{res}}\}\in\mathbb R^{20}$ (§II-B.1).
   - Tasks (§III-B): SQ hovers 50 cm above LQ for 10 s; circles of 1.5 m diameter, period 7.5 s, SQ 50 cm above, same and
     reversed direction; docking: SQ descends at 0.1 m/s "until it is 10cm above the LQ", motors off when horizontal
     distance < 5 cm, "falls freely from 5cm above".
   - Results (Table II): average position RMSE basic 0.1506 m, FB-AeroComp (model-based aerodynamic compensation from
     relative position, after Jain et al.) 0.0976 m, ProxFly 0.1069 m; text: −29.0 % position and −43.7 % attitude vs
     basic; "comparable" to FB-AeroComp.
5. **Stated limitations.** Residual "can potentially cause oscillations, which might lead to motor overheating";
   parameters must stay "in a reasonable range"; "we only validated the performance on two regular-sized quadcopters"
   (§IV); "ProxFly is mainly trained for hovering rather than trajectory tracking" (§III-B).
6. **Quotable claims.**
   - "our method takes only the ego state and controllers' commands as inputs and does not rely on any communication between quadcopters" (abstract)
   - "The observation information $o = \{\Delta s, u_{cas}, a_{res}\} \in \mathbb{R}^{20}$ consists of the error between the desired state and current state …, the high-level command … from the basic cascaded controller, and the action from residual module" (§II-B.1)
   - "the residual module reduces the averaged RMSE of position control by 29.0% and averaged attitude error by 43.7% across the three tasks." (§III-B)
   - "compared to FB-AeroComp with complex aerodynamic modeling, ProxFly achieves comparable performance." (§III-B)
   - "these methods are only validated on micro quadcopters with weak downwash effect and large spacing, so it is uncertain whether the conclusions can be generalized to regular quadcopters" (§I, about NS/NS2 and Li et al.)
7. **What it does NOT show.** No explicit force estimate (by design); no Crazyflie; no INDI; the comparison with
   FB-AeroComp is a same-hardware cross-method comparison (learned residual policy vs a model-based relative-position
   compensator) — relevant to the gap claim (Phase 2).
8. **Thesis' use of it.**

| Claim | Verdict | Evidence |
|---|---|---|
| C2.61 (residual RL on cascaded controller; disturbance and thrust loss; no explicit force estimate) | supported | abstract; §II. |
| C2.62 (observes only tracking error, baseline command, previous action; no neighbour state, no comms) | supported | §II-B.1. |
| Tab 2.1 row (2 heterogeneous; 0.50 m vert. hover/circle; dock to 0.10 m, release at 0.05 m; position and attitude error) | supported | §III-B; Table II. |
| Ch2 §2.8 gap ("the families … have not been placed under common conditions") | **narrows it** | ProxFly compares, on one hardware pair, a residual-RL controller with a model-based relative-position compensator (FB-AeroComp) and a basic controller — a cross-family comparison, though not reactive-vs-learned-predictive. Add to §2.8 "Comparisons that do exist". |

9. **Confidence and gaps.** High.

---

## R09 — chee2024knodedwmpc

1. **Reading record.** arXiv:2410.09727v1 (13 Oct 2024), 7 pages, all read. Figures 2–5 are images (heat maps and
   bar plots; exact per-case values **not readable** from text); captions and text read.
2. **Bibliographic check.** Kong Yao Chee*, Pei-An Hsieh* (equal contribution), George J. Pappas, M. Ani Hsieh ✓;
   title ✓; 2024 ✓; arXiv ✓; no venue printed.
3. **What the paper does.** Builds a "KNODE-DW" model of the bottom vehicle's dynamics: nominal rigid body + an
   analytical downwash model (jet velocity field after Bauersfeld et al., drag integration after Jain et al.) + a
   small neural ODE trained on one-step state prediction; uses it as the prediction model of a nonlinear MPC that also
   propagates the top vehicle's state over the horizon. Compared against nominal, KNODE-only, DW-only and (in
   simulation) omniscient MPC.
4. **Setup facts.**
   - Frame ENU (§II-A). Hardware (§IV-B): two Crazyflie 2.1, 10 cm body length, ≈34 g, bottom vehicle with thrust
     upgrade bundle; laptop (Intel i5) base station, Crazyradio PA at ≈400 Hz, Vicon ≈120 Hz; MPC outputs thrust and
     angular-rate commands to the firmware PID loops; forces compensated, not torques.
   - Data: simulation 3 532 points ≈18 s at 0.3 and 0.4 m, 0.4 m/s; hardware 6 720 points, 28 s, at 0.4 m/s and 0.4 m,
     collected under nominal MPC; "a total of 46 seconds of flight data" (abstract).
   - Tests: hardware static-top and stacked at commanded 0.2 m and 0.4 m (5 runs each); lemniscate in stacked
     formation at "2 body lengths"; straight line at 0.8 body lengths (0.08 m) commanded → average 0.12 m achieved vs
     0.28 m with nominal MPC.
   - Headline (abstract + §IV-B): "40.1% improvement in the average trajectory tracking errors and a 57.5% reduction
     in the maximum vertical separation errors" vs nominal MPC.
5. **Stated limitations.** Analytical flow model accurate only in the far field (§II-A); top vehicle assumed at
   constant velocity over the horizon (§III-B); collision constraints and uncertainty quantification left to future work.
6. **Quotable claims.**
   - "our framework significantly outperforms nominal MPC in physical experiments, achieving a 40.1% improvement in the average trajectory tracking errors and a 57.5% reduction in the maximum vertical separation errors." (abstract)
   - "the derived models are often deployed in **reactive controllers**, e.g., [13], [17]. These controllers consider the aerodynamic effects or disturbances at the current time step, but neglect potential disturbances that may occur within the next few time steps." (§I; [13] = Smith et al., [17] = Neural-Swarm) — **terminology clash** with the thesis, where "reactive" means measured (INDI).
   - "Instead of generating thrust and moment commands, the framework in physical experiments generates thrust and angular rate commands, which are passed to the PID controllers running in the Crazyflie firmware." (§IV-B)
   - "They are commanded to be 0.8 body lengths (0.08m) vertically apart … the average vertical separation between the drones is 0.12m" (§IV-B)
7. **What it does NOT show.** No onboard MPC; no three-vehicle case; no INDI baseline; per-case hardware RMSE values only
   in figures.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C2.53 (learned component inside the NMPC prediction constraint) | supported | eq. (8c), (15). |
| C2.56 ("Chee et al. … run the optimisation off-board"; "None of the three runs on a Crazyflie-class vehicle") | supported with changed wording | Chee **flies** Crazyflies; the optimiser runs on a laptop. Corrected: "none of the three runs its optimiser onboard a Crazyflie-class vehicle". |
| C2.49 (≈5 cm RMSE, 10 cm max-z "published" by Chee) | **cannot tell from this PDF** | values are plotted in Fig. 4, not stated; R05 reports them as "the reported tracking performance". Keep the attribution to Hsieh et al. ("which they report as matching …"). |
| Tab 2.1 row (2, Crazyflie 2.1; commanded 0.2–0.4 m; tight ≈0.08 m; RMSE / max z vs nominal MPC; off-board) | supported | |
| Ch1/Ch2 terminology "reactive" | **clash to flag** | Chee et al. call feedforward-of-a-learned-model controllers "reactive" (current-time-step) in contrast with MPC; the thesis uses "reactive" for measured residuals. A one-sentence footnote avoids an examiner question. |

9. **Confidence and gaps.** Medium-high: numeric per-case hardware values not readable.

---

## R10 — hsieh2025onlineadaptation  ⚠️ gap-relevant

1. **Reading record.** arXiv:2506.17488**v3** (30 Oct 2025), 10 pages (Springer proceedings layout), all read.
   Figures 1–4 images (captions read; per-case values not readable).
2. **Bibliographic check.** Pei-An Hsieh, Kong Yao Chee, M. Ani Hsieh ✓; title ✓; 2025 ✓; arXiv ✓; venue not printed.
3. **What the paper does.** Adds an $\mathcal L_1$-adaptive module — a state predictor on velocity, a piecewise-constant
   adaptation law that **estimates the residual acceleration online** from the predicted-vs-measured velocity error
   using the commanded thrust, and a low-pass filter — to the KNODE-DW MPC of Chee et al., whose prediction model sums
   pairwise learned/physics downwash forces from all higher neighbours. Six MPC variants are flown on **three**
   Crazyflies: nominal, DW, KNODE-DW, and each with $\mathcal L_1$ — i.e. no compensation, **online estimation only
   ($\mathcal L_1$ MPC), prediction only (DW / KNODE-DW MPC) and their combination ($\mathcal L_1$ KNODE-DW MPC)** on the
   same hardware and formations.
4. **Setup facts.**
   - Hardware (§3): three Crazyflie 2.1 with thrust-upgrade bundles, 0.1 m diameter, ≈34 g; Vicon 120 Hz; controllers
     off-board (Intel NUC13 i7 for the bottom vehicle via Crazyradio PA at 200 Hz; laptop i5 for the others at 400 Hz);
     inputs thrust and angular rates; acados.
   - Aggregation (§2.1, eq. 1): total force = **pairwise summation** over higher neighbours within a range α — "Based on
     existing empirical downwash force measurements [9 = Gielis et al.], we assume that … the aggregated force … can be
     approximated as a pairwise summation".
   - Training data: "the same training data as in [5 = Chee et al.]", i.e. **two-vehicle** static-top and stacked data,
     used on three-vehicle formations (§2.1).
   - Formations (§3, Fig. 1): V-stack ($r$ = 0.1 m, $z_1$ = 0.2 m, $z_2$ = 0.4 m or 0.3 m) and I-stack ($z_1$ = 0.2 m,
     $z_2$ = 0.4/0.3 m; tight case $z_1 = z_2$ = 0.2 m); five runs per case; metrics RMSE and $z_{\max}$.
   - Results (body): centre vehicle: $\mathcal L_1$ KNODE-DW vs KNODE-DW −47.9 % RMSE, −40.4 % $z_{\max}$; **$\mathcal L_1$ MPC
     (estimation only) −28.5 % RMSE vs KNODE-DW MPC (prediction only)**; nominal MPC caused collisions; bottom vehicle:
     $\mathcal L_1$ KNODE-DW −15 % RMSE, −6.6 % $z_{\max}$ vs KNODE-DW; at $z_2$ = 0.4 m several variants match it, at 0.3 m
     only $\mathcal L_1$ KNODE-DW keeps its performance; tight I-stack RMSE 0.034/0.057/0.049 m (top/centre/bottom).
5. **Stated limitations.** "since the L1 adaptive module lacks a downwash model, it is not as responsive as L1
   KNODE-DW MPC when entering or exiting downwash regions … a strategy that relies solely on the L1 adaptive controller
   may be too slow to respond to fast changing and/or very large disturbances" (§4); collision constraints future work (§5).
6. **Quotable claims.**
   - "we present L1 KNODE-DW MPC, an adaptive, mixed expert learning based control framework that allows individual quadrotors to accurately track trajectories while adapting to time-varying aerodynamic interactions" (abstract)
   - "the adaptive module estimates and compensates for unmodeled disturbances online through a sequential procedure consisting of a state estimator, an adaptation law, and a low-pass filter" (§2.2)
   - "Based on existing empirical downwash force measurements [9], we assume that for all m ∈ Ni, the aggregated force of on i can be approximated as a pairwise summation of the force from each m ∈ Ni." (§2.1)
   - "Notably, the L1 MPC also achieved a 28.5 % improvement in RMSE and a 26.3 % improvement in zmax compared to KNODE-DW MPC." (§3)
   - "However, since the L1 adaptive module lacks a downwash model, it is not as responsive as L1 KNODE-DW MPC when entering or exiting downwash regions, as the latter has a highly predictive model of the downwash forces." (§4)
   - "individual vehicles are impacted by both the cummulative downwash forces generated from vehicles above and the aerodynamics created by vehicles below." (§4)
   - "To the best of our knowledge, this is the first demonstration of such a compact formation involving more than two quadrotors." (§1)
7. **What it does NOT show.** No rotor-speed-based INDI (the online estimate uses commanded thrust and a velocity
   predictor); no onboard execution; no geometric/feedforward controller; no quantification of the two-to-three
   summation penalty as such (although a two-vehicle-trained model is used on three vehicles); one airframe; small
   number of formations.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C2.55 ("Hsieh et al. add an $\mathcal L_1$ online estimate to the same family") | **understated / incomplete** | It is a measured-plus-predicted hybrid on three vehicles, with estimation-only and prediction-only arms. Corrected (APPLY, reference-context): "Hsieh et al. add an $\mathcal L_1$-adaptive online estimate of the residual acceleration to this MPC and fly three Crazyflies in two stacked formations, comparing estimation only, prediction only and both; the combination performs best, and estimation alone outperforms prediction alone for the middle vehicle." |
| C2.56 (off-board) | supported | §3. |
| Tab 2.1 row ("3; V-stack and I-stack; pairwise 0.2 m; tracking vs MPC family") | supported with changed context | separations $z_1$ = 0.2 m, $z_2$ = 0.3–0.4 m, $r$ = 0.1 m; tight case 0.2/0.2 m; add "online-estimation, prediction and hybrid arms; off-board". |
| **C1.10, Ch1 §1.3 ¶3 ("The hybrid has been demonstrated only where the residual is self-induced: a single vehicle … It has not been shown where other vehicles produce the residual")** | **wrong** | §2–§3: online estimation + learned downwash model on a three-vehicle team. Minimal factual correction (APPLY): restrict the sentence to "The hybrid of Cobo-Briesewitz et al. …" and add Hsieh et al. as the multi-vehicle MPC instance; the re-statement of the gap is a DECISION (D-03). |
| **Ch2 §2.9 second gap ("demonstrated only for a single vehicle … remains untested for such hybrids")** | **wrong** | same. |
| **Ch1 §1.3 gap; Ch2 §2.8 ¶"Comparisons that do exist" and §2.9 first gap ("reactive, predictive and hybrid families have not been evaluated against one another under identical conditions")** | **weakened** | Hsieh et al. evaluate, on one airframe and formation set, an online estimate, a learned prediction and both, inside one MPC. What survives: no comparison of *rotor-speed-based* reactive control (INDI) against a *learned per-neighbour feedforward* in onboard non-MPC controllers, across a frozen multi-regime scenario library, with the residual logged under every controller and a quantified two-to-three transfer penalty. DECISION D-03 with recommended wording (Phase 2 §3). |
| Ch1/Ch2 RQ3 framing (summation penalty "not reported") | **narrowed** | Hsieh et al. deploy a summed two-vehicle-trained model on three vehicles but do not isolate the penalty; the narrower claim survives. |
| Ch2 §2.4¶ superposition | **counter-use to note** | Hsieh et al. cite Gielis et al. **in support of** pairwise summation, while the thesis cites Gielis et al. for its limits — both readings exist (Gielis' linear regime vs nonlinear regime). |
| The two provenance todos (Ch1 L221, Ch2 L571) | **audit gap** | the audits "found adjacent comparisons but none spanning these families"; this held paper spans them in an MPC setting. The provenance note should be revised by the author (DECISION D-03). |

9. **Confidence and gaps.** High for the method and arms; per-case values only in figures.

---

## R11 — kiran2025downwash

1. **Reading record.** arXiv:2507.09463v1 (13 Jul 2025), 12 pages, all read. Figures 1–16 images (captions read);
   Tables I–II read.
2. **Bibliographic check.** Anoop Kiran, Nora Ayanian, Kenneth Breuer ✓; title ✓; 2025 ✓; arXiv ✓; no venue printed.
3. **What the paper does.** Measures forces, moments and flow fields of a single Crazyflie and of a pair, both mounted
   on 6-DOF force/torque transducers on a precision x–z translation stage (roll and yaw prevented, motors at constant
   hover voltage), plus PIV of the wake; fits turbulent-jet scaling to the far field; adds a linear traverse
   ("CrazyRail") for dynamic approach/retreat of one vehicle above another.
4. **Setup facts.**
   - Rig (§II-A): two Crazyflies on an x–z stage (100 µm accuracy), ATI Nano-17 6-DOF transducers, "preventing roll and
     yaw", 4 V DC tethered supply at a hover PWM, rotors ≈320 Hz; static test matrix 0.1 ≤ Δx ≤ 0.6 m, 0.1 ≤ Δz ≤ 1.1 m,
     20 kHz for 30 s, three repetitions; PIV laser sheet "passing through the center of the front two rotors"; $l$ = 32.5 mm
     (body centre to motor); weight $W$ ≈ 0.265 N.
   - Results (§III-A): lower vehicle thrust "significantly drops to 65% of its nominal hover thrust" at Δz/l = 4
     (13 cm, stacked); upper vehicle retains 90 %; lower-vehicle thrust unaltered beyond Δx/l ≈ 3 and Δz/l ≈ 19;
     unsteadiness lingers to Δz/l ≈ 17, Δx/l ≈ 4.5; upper vehicle's fluctuations "approximately two orders of magnitude
     smaller"; pitch moment peaks at Δx/l = 2 (offset).
   - Far field (§III-B): merge at z/l = 6.5 ≈ 2.3 motor-to-motor distances, "consistent with Bauersfeld et al., which
     reports a similar transition occurring after 2.5 motor-distance length scales".
   - Dynamic (§III-C): approach phase lower thrust than retreat (hysteresis); offset approach produces stronger pitch moments.
   - §I-B: Preiss et al. planned with "∆z = 60 cm … and a horizontal separation of ∆x = 24 cm".
5. **Stated limitations.** Homogeneous small quadrotors only (§V); dynamic traverse limited to 0.15 ≤ Δz/l ≤ 7.5 (§V);
   constant voltage, not constant rotor speed (§II-A).
6. **Quotable claims.**
   - "at the closest vertical separation (∆z/l = 4), the thrust significantly drops to 65% of its nominal hover thrust." (§III-A)
   - "Even at the closest vertical separation (∆z/l = 4) from the lower quadrotor, the upper quadrotor retains 90% of its original hover thrust." (§III-A)
   - "The lower quadrotor's thrust attains unaltered levels at (∆x/l ≈ 3) and (∆z/l ≈ 19)." (§III-A)
   - "the upper quadrotor experiences turbulence approximately two orders of magnitude smaller than the lower quadrotor." (§III-A)
   - "Notably, the approach phase of each interaction cycle has lower thrust in comparison to its retreating counterpart." (§III-C)
   - "For collision avoidance, Preiss et al. used a vertical separation, ∆z = 60 cm … and a horizontal separation of ∆x = 24 cm … between two small quadrotors (Crazyflies)" (§I-B)
7. **What it does NOT show.** No free flight, no closed-loop residual, no controller, no learned model.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence |
|---|---|---|
| C2.07 (load cells, forces and torques, mounted Crazyflies roll/yaw locked, constant hover voltage, PIV centre-plane sheet, single and pair; dynamic on a rail) | supported (minor) | the PIV plane passes "through the center of the front two rotors" rather than the body centre plane. |
| C2.09 (transducer load; actuation at hover; not a ground-truth claim; no learned model) | supported for Kiran | |
| C3.03 (force and moment on a rig) | supported | |
| C2.06 (strongly nonlinear, decays with lateral offset, asymmetric) — uncited | **usable source** | §III-A: 65 % vs 90 % thrust; lateral recovery by Δx/l ≈ 3 vs vertical Δz/l ≈ 19. |
| C1.02, C2.10 (2.5 motor-to-motor distances) | **corroborated** | §III-B: 2.3 measured, citing 2.5. |
| Ch3 magnitude example (missing) | **usable illustrative magnitude** | a 35 % thrust loss of a ≈27 g Crazyflie at Δz = 13 cm in a static rig corresponds to ≈0.35 g ≈ 3.4 m/s² — label as "rig measurement at constant voltage, not a flight residual". |
| Ch5 regime RQ2 (transient vs quasi-static) | **supporting** | §III-C: approach/retreat hysteresis. |

9. **Confidence and gaps.** High.

---

## R12 — bauersfeld2024airflow

1. **Reading record.** arXiv:2403.13321**v3** (12 Dec 2024), header "IEEE Robotics and Automation Letters. Preprint
   version. Accepted Dec, 2024", 8 pages, all read. Figures 1–10 images (captions read); Tables I–III read.
2. **Bibliographic check.** Leonard Bauersfeld, Koen Muller, Dominic Ziegler, Filippo Coletti, Davide Scaramuzza ✓;
   title ✓; RA-L ✓ (bib `note`). Year: the PDF says accepted Dec 2024; R05/R11 reference lists give RA-L **10(2):1241–1248**
   with year 2024 (R05) or 2025 (R11) — a volume-10 issue is 2025; bib `year = 2024` is the arXiv year → `[VERIFY]`.
3. **What the paper does.** Measures the time-averaged flow speed below six hovering quadrotors (0.23–6.3 kg) with a
   hot-ball probe in a large motion-capture hall (>16 h), normalises by induced velocity and motor distance $l$, and
   shows that after ≈2.5 $l$ the merged flow follows classical turbulent-jet similarity with fitted parameters; packages
   this as a closed-form "pen-and-paper" model; demonstrates a feedforward throttle correction when one drone passes
   1–2 m below another.
4. **Setup facts.** Vicon hall 25 × 25 × 7 m; Testo 440 hot-ball probe, 1 s sampling, magnitude only; points with speed
   > 0.1 m/s filtered out; ≈10 000 s per vehicle (§IV); near field: rotor flows merge "between 1.5 and 2.5
   motor-distances below the drone" (§V); fit $Bd$ = 10.11, $S$ = 0.07668, $s_0$ = −5.817 (eq. 9); controller demo: Kolibri
   passes 1 m and 2 m below Offboard 2; height RMSE 69.1 → 24.8 mm and 134.2 → 25.4 mm (Table III); abstract: "four
   times lower altitude deviation".
5. **Stated limitations.** Hover / near-hover only; canted propellers deviate (§VI-B); "In this near-flow region, the
   interactions between the flow and the drone are dominant which can only be fully captured through CFD simulations"
   (§VIII); theory over-predicts for ξ > 2.
6. **Quotable claims.**
   - "This paper introduces a computationally lightweight model for estimating the time-averaged magnitude of the induced flow below quadrotors in hover." (abstract)
   - "we show for the first time that the combined flow from all drone propellers is well-approximated by a turbulent jet after 2.5 drone-diameters below the vehicle." (abstract)
   - "After 2.5 motor-to-motor distances l the individual flows merge into one turbulent jet (far-field)" (Fig. 1 caption)
   - "In this near-flow region, the interactions between the flow and the drone are dominant which can only be fully captured through CFD simulations." (§VIII)
   - "we integrate it into a controller that automatically compensates for downwash when passing below another drone, yielding a four times smaller altitude tracking error." (§I)
7. **What it does NOT show.** No near-field model; no instantaneous force on a follower (a power/throttle scaling is
   derived via momentum theory); no Crazyflie; no formation at tight separations (demo at 1–2 m).
8. **Thesis' use of it.**

| Claim | Verdict | Evidence |
|---|---|---|
| C1.02, C2.10 (16 h; six platforms 0.23–6.3 kg; jet beyond ≈2.5 motor-to-motor distances; abstract says "drone-diameters") | supported | abstract; Fig. 1; §V. |
| C1.03, C2.11 (time-averaged magnitude in hover; near field: CFD) | supported | abstract; §VIII. |
| Ch1 L48 "In that near field those authors report no closed-form model" | supported | §VIII. |
| Ch2 §2.5 (physics-based predictive compensation, missing) | **addition candidate** | §VII: feedforward downwash compensation from the jet model, 3–5× lower height RMSE at 1–2 m vertical separation — a first-principles predictive instance. |

9. **Confidence and gaps.** High.

---

## R13 — tal2018indi

1. **Reading record.** arXiv:1809.04048**v2** (7 Jun 2020), 16 pages incl. biographies, all read. Figures are block
   diagrams and plots (captions and text read); Tables I–V read.
2. **Bibliographic check.** Ezra Tal, Sertac Karaman ✓; title ✓. The held version is the **extended journal version**
   ("A preliminary version of this article was presented at 57th IEEE CDC 2018 [1]"); R07's reference list gives
   **IEEE Transactions on Control Systems Technology 29(3):1203–1218, 2021** for it (`[VERIFY]`). Bib key `tal2018indi`
   with `year = 2018` points readers to the CDC version while the content cited (e.g. response analysis, quaternion
   form, 12.9 m/s) is from the extended version → **suspected bib mismatch**; recommend citing T-CST 2021 (DECISION, bib).
3. **What the paper does.** A trajectory-tracking controller for position and yaw with derivatives up to snap: PD
   position/velocity loop → INDI linear-acceleration loop (thrust vector increment from filtered acceleration and
   motor-speed-derived thrust) → attitude/rate PD with flatness feedforward of angular rate and acceleration → INDI
   angular-acceleration loop (moment increment) → nonlinear inversion of control effectiveness with motor-speed
   closed loop using optical encoders. Response analysis of robustness to disturbances and modelling errors; flights up
   to 12.9 m/s and 2.1 g; drag-plate and tether-pull tests.
4. **Setup facts.**
   - Frame: NED inertial frame (§II-A).
   - INDI acceleration (§III-B): $f_{\mathrm{ext}} = m(a_f - (\tau b_z)_f - gi_z)$ (18); $(\tau b_z)_c = (\tau b_z)_f + a_c - a_f$ (20)
     — algebraically $a_c - gi_z - f_{\mathrm{ext}}/m$, i.e. the estimated external force is **subtracted**; "Identical LPFs
     must be used to ensure that equal phase lag is incurred by acceleration and thrust measurements"; "This principle
     eliminates the need for integral action anywhere in the control design."
   - INDI angular acceleration (§III-D): $\mu_{\mathrm{ext}} = J\dot\Omega_f - \mu_f + \Omega_f\times J\Omega_f$ (29); $\mu_c = \mu_f + J(\dot\Omega_c - \dot\Omega_f)$ (31),
     $\mu_f$ from measured motor speeds **and low-pass filtering**; $\dot\Omega_c = K_\xi\xi_e + K_\Omega(\Omega_{\mathrm{ref}} - \Omega_f) + \dot\Omega_{\mathrm{ref}}$ (28).
   - Filtering (§III, §V-A): angular acceleration by numerical differentiation of the measured rate with an LPF;
     second-order Butterworth, 30 Hz (188.5 rad/s), on motor speeds and IMU.
   - Platform (§V-A): custom quadrotor, motors 18 cm apart, **609 g**; STM32H7 at 2000 Hz (16 µs per update); IMU 2000 Hz;
     OptiTrack 360 Hz (18 ms latency, propagated); optical encoders ≈5000 Hz.
   - Parameters needed (§V-A): "vehicle mass m, moment of inertia J, motor time constant τm, control effectiveness
     matrices G1 and G2, and the gain and polynomial fit used by the motor speed controller".
   - Results: RMS 6.6 cm at up to 12.9 m/s, 2.1 g (Table IV); roulette: 9.0 cm, 16.8 cm without jerk/snap tracking,
     7.6 cm with drag plate (Table V); tether pull: within 4 cm under ≈3.7 N (§V-F).
5. **Stated limitations.** Bandwidth of LPF and motors limits disturbance rejection (§IV-A); linearised inversion can
   cause yaw oscillations if τm and Δt differ (§V-E); feedforward thrust derivatives unused (§II-B).
6. **Quotable claims.**
   - "Snap tracking requires direct control of body torque, which we achieve using closed-loop motor speed control based on measurements from optical encoders attached to the motors." (abstract)
   - "The controller utilizes incremental nonlinear dynamic inversion (INDI) for robust tracking of linear and angular accelerations despite external disturbances, such as aerodynamic drag forces. Hence, prior modeling of aerodynamic effects is not required." (abstract)
   - "Identical LPFs must be used to ensure that equal phase lag is incurred by acceleration and thrust measurements [18]." (§III-B)
   - "The incremental nature of (20) enables the controller to achieve the commanded acceleration despite possible disturbances or modeling errors. … This principle eliminates the need for integral action anywhere in the control design." (§III-B)
   - "the ability to reject disturbances is limited by the bandwidth of both the LPFs and the motors." (§IV-A)
   - "the controller requires several platform-specific parameters, namely: vehicle mass m, moment of inertia J, motor time constant τm, control effectiveness matrices G1 and G2, and the gain and polynomial fit used by the motor speed controller." (§V-A)
7. **What it does NOT show.** No multi-vehicle or downwash case; INDI does not by itself require encoders (encoders
   serve the snap/torque path and accurate thrust); no integral term.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C2.05 (flatness; position and yaw derivatives to fourth order; FF angular rate and acceleration for jerk and snap) | supported | abstract; §II-B. |
| C2.15 (single custom 609 g vehicle; no parametric model of aerodynamics required) | supported | §V-A; abstract. |
| C2.16 (mass, inertia, motor time constant, control effectiveness, throttle map identified; drag model dropped) | supported | §V-A quote. |
| C2.17, C4.06 (optical encoders; snap and torque path) | supported | abstract; §III-E. |
| C4.05 (incremental formulation with flatness FF follows Tal & Karaman) | supported | |
| Ch4 eq. (ctrl-indi-tau) $\tau = \tau_{\mathrm{cur}} + J(\alpha_{\mathrm{ref}} - \alpha_{\mathrm{meas}})$ | supported (form) | (31). **Deviations of our code** to state (APPLY, code): Tal & Karaman filter $\mu_f$ with the same LPF as $\dot\Omega_f$ and use $\Omega_f$ in the reference; the code by default leaves $\tau_{\mathrm{cur}}$ unfiltered (`filt_tau = 0`, `traj_iface.c:342`) and instead filters $\alpha_{\mathrm{ref}}$ with the same Butterworth (`lib.rs:2076–2080`); the code differentiates first and filters after by default (`filt_order = 0`, `traj_iface.c:334`). Which options the study configuration sets is a configuration fact (`[VERIFY]`). |
| Ch4 §4.1 position INDI (measured residual subtracted from $a_{\mathrm{cmd}}$) | supported in form | (20) = subtraction of $f_{\mathrm{ext}}/m$. **Code deviation**: the accelerometer is Butterworth-prefiltered (`lib.rs:545, 2374–2382`) while the rotor-speed thrust is not filtered with the same filter unless `res_fc` is set (default 0, `traj_iface.c:493`) — the opposite of "identical LPFs" (APPLY as a stated deviation; study value `[VERIFY]`). |
| Ch3 §3.3.2 sign | **supported** | (18)–(20). |
| Ch4 S1 integral terms (DECISION D-02) | **relevant** | Tal & Karaman state that INDI removes the need for integral action — supports the study configuration's `ki_z = 0` for our INDI. |

9. **Confidence and gaps.** High. Venue/year of the held version from a secondary reference.

---

## R14 — smeur2017cascadedindi

1. **Reading record.** arXiv:1701.07254**v2** (12 Jan 2022), 17 pages incl. references, all read. Figures 1–16
   (block diagrams and plots; captions read); Tables 1–3 read.
2. **Bibliographic check.** Ewoud J. J. Smeur, Guido C. H. E. de Croon, Qiping Chu ✓; title ✓; arXiv 2017 ✓. Journal
   version per R13's reference list: Control Engineering Practice 73:79–90, 2018 — matches the bib `yearnote` ✓.
3. **What the paper does.** Extends INDI from the attitude loop to the linear-acceleration loop of a quadrotor: the
   accelerometer measures both control and disturbance forces, so the thrust vector is incremented by the inverse
   control effectiveness times the acceleration error; all incremented terms pass the same second-order filter; a PD
   position loop supplies the acceleration reference. Tested on a Parrot Bebop (Paparazzi, 512 Hz) flying into and out
   of a 10 m/s wind-tunnel jet against a PID baseline, outdoors with GPS, and with a nonlinear thrust-vector increment.
4. **Setup facts.**
   - Platform: Parrot Bebop, Paparazzi, control and IMU at 512 Hz (§5); NED frame (§3); thrust from a quadratic
     thrust-vs-rotor-speed curve with "the internal rpm measurement" (§4.3); motor feedback filtered with the same filter
     as the angular acceleration (§2, Fig. 3); filter ωn = 50 rad/s, ζ = 0.55 (§4.5, Table 2).
   - Wind tunnel (§5): 10 m/s open jet; waypoints alternated "every 14 seconds"; **OptiTrack data "only sent to the
     drone at a frequency of 4 Hz"** in the same experiment; max position deviation 0.21/0.20 m (INDI) vs 1.51 m (PID),
     seven repetitions.
   - Outdoor (§6): take-off in "average wind speed of 5.1 m/s" (KNMI), u-blox Neo M8N GPS; 12 PID and 13 INDI flights;
     average max error 0.24 m vs 0.85 m.
   - §4.6: "The outer loop INDI controller is somewhat sensitive to accelerometer biases … an offset in the measurement
     will result in an offset in the actual acceleration as well."
5. **Stated limitations.** Accelerometer bias sensitivity (§4.6); saturation and control allocation (§8.1); small-ψ
   assumption (§3, §8.1); GPS errors in outdoor runs (§6.1).
6. **Quotable claims.**
   - "The idea is that both disturbances as well as control forces are measured by the accelerometer." (§1)
   - "all terms with subscript 0 will be filtered with the same filter … such that they are equally delayed." (§2, §3)
   - "the INDI loop replaces the integral of the PID controller." (§5)
   - "The outer loop INDI controller is somewhat sensitive to accelerometer biases. Because the accelerometer measurement is fed back to control the acceleration, an offset in the measurement will result in an offset in the actual acceleration as well." (§4.6)
   - "the data was only sent to the drone at a frequency of 4 Hz." (§5)
7. **What it does NOT show.** No multi-vehicle; no trajectory tracking at speed (that is Tal & Karaman); no encoders.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C2.12 (inner angular-acc INDI + outer linear-acc INDI; single Bebop; Paparazzi 512 Hz; PD position/velocity loops) | supported | §2–§5. |
| C2.13 (10 m/s jet, every 14 s) | supported | §5. |
| C2.14 ("In two **separate** experiments … indoors with motion capture deliberately downsampled to 4 Hz, and outdoors … ≈5.1 m/s wind") | **supported with changed context** | the 4 Hz down-sampling is part of the wind-tunnel experiment itself, not a separate indoor experiment. Corrected: "The wind-tunnel experiment itself fed motion-capture positions at only 4 Hz, and a separate outdoor take-off in ≈5.1 m/s average wind used a consumer GNSS module." |
| C2.18 (Smeur et al. use motor-speed measurement) | supported | §4.3 (internal rpm measurement; thrust curve); Fig. 3. |
| C4.05 (cascaded arrangement follows Smeur) | supported | |
| Tab 2.1 row | supported | |
| Ch3 §3.3.1 "It absorbs model error" (missing failure mode) | **addition** | accelerometer bias enters the measured residual and, through position INDI, the steady-state acceleration (§4.6). |
| Ch4 S1 integral (D-02) | relevant | "the INDI loop replaces the integral". |

9. **Confidence and gaps.** High.

---

## R15 — lee2010geometricse3 (companion of the CDC 2010 paper)

1. **Reading record.** arXiv:1003.2005**v4** (9 Sep 2011), 12 pages; main text, Propositions 1–5 and numerical section
   read in full; Appendices A (hat-map properties) and B (proof of Proposition 1) read; Appendices C–E (remaining proofs)
   skimmed. Figures are simulation plots.
2. **Bibliographic check.** Taeyoung Lee, Melvin Leok, N. Harris McClamroch ✓; title ✓; arXiv 1003.2005 ✓. **The held PDF
   is v4**, while the bib `note` for this key and for `lee2010geometrictracking` describes equation checks against
   "arXiv:1003.2005v1" and reserves v4-only content (three flight modes, velocity mode, heading projection, switched
   720°/360° example) for this key. The checks below are against v4; the objects the thesis uses (Ψ, $e_R$, $e_\Omega$, the
   torque law, the position-mode thrust, the stability regions) appear in v4 Sections IV–V. Whether v1/CDC carries the same
   numbering cannot be checked (`[VERIFY]`: the bib note's "Prop. 2 / Prop. 3" numbering is the CDC's; v4 numbers them
   Propositions 3 and 4).
3. **What the paper does.** Models the quadrotor on SE(3) and gives geometric tracking controllers for three flight
   modes (attitude, position with heading direction, velocity), with exponential stability and almost-global exponential
   attractiveness results, and simulated aggressive manoeuvres (recovery from upside-down, 720° flips).
4. **Setup facts.** Frame and model (§II, eqs. 2–5): $m\dot v = mge_3 - fRe_3$, i.e. **$e_3$ points down** (gravity along
   $+e_3$, thrust along $-b_3$); $J\dot\Omega + \Omega\times J\Omega = M$. Error function (6) $\Psi = \tfrac12\mathrm{tr}(I - R_d^\top R)$;
   (8) $e_R = \tfrac12(R_d^\top R - R^\top R_d)^\vee$; (9) $e_\Omega = \Omega - R^\top R_d\Omega_d$; torque (11)/(20)
   $M = -k_Re_R - k_\Omega e_\Omega + \Omega\times J\Omega - J(\hat\Omega R^\top R_c\Omega_c - R^\top R_c\dot\Omega_c)$; position mode (19)
   $f = (k_xe_x + k_ve_v + mge_3 - m\ddot x_d)\cdot Re_3$ with $e_x = x - x_d$; $b_{3c}$ from (23); heading via projection of
   $b_{1d}$ (38). Proposition 3: exponential stability if $\Psi(R(0),R_c(0)) < 1$ ("initial attitude error is less than
   90°"); Proposition 4: almost-global exponential attractiveness for $1 \le \Psi < 2$ (< 180°). Simulation vehicle 4.34 kg.
5. **Stated limitations.** Topological obstruction to global asymptotic stability on SO(3) (§IV); assumptions (24)–(25)
   (non-zero, bounded commanded force).
6. **Quotable claims.**
   - "It is coordinate-free. Therefore, it completely avoids singularities, complexities, discontinuities, or ambiguities that arise when using local coordinates or quaternions." (§I)
   - "Since the quadrotor UAV has four inputs, it is possible to achieve asymptotic output tracking for at most four quadrotor UAV outputs." (§III)
   - "Proposition 3 requires that the initial attitude error is less than 90° to achieve exponential stability for this flight mode." (§V)
   - "by combining the results of Proposition 1 and 3, we can show almost global exponential attractiveness when Ψ(R(0), Rc(0)) < 2." (§V)
7. **What it does NOT show.** No experiments in this version (simulation only); no disturbance/residual term; no
   integral action.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C2.04 (attitude error function verified against the companion) | supported (v4) | eqs. (6), (8). Wording: "companion arXiv paper" — say which version is held (v4). |
| C4.02 ($e_R$ formula; position + one heading; four outputs) | supported (v4) | (8); §III; Prop. 5. |
| C4.03 (Lee torque law with feedforward bracket; $e_\Omega$) | supported (v4) | (11), (20), (9). |
| C4.04 (Ψ(0) < 1 exponential, < 90°; almost-global exponential attractiveness 1 ≤ Ψ < 2, < 180°) | supported (v4) | Props. 3–4. Attribution to CDC 2010 (no PDF): `[VERIFY]`. |
| C2.02, C2.03 (cited to CDC) | cannot tell from CDC; consistent with v4 | keep as in the draft (H2: no PDF of CDC). |
| Ch4 §4.1 / §4.2 frame mapping (missing) | **addition (APPLY)** | Lee et al. use $e_3$ down and thrust $-fRe_3$, position error $e_x = x - x_d$; the thesis uses $e_3$ up, thrust $+fRe_3$ and $e_p = p_{\mathrm{des}} - p$. The torque law is written in body coordinates and carries over; the position-loop signs map accordingly. One sentence prevents an examiner sign question. |
| Ch4 $e_\omega = \omega - \omega_{\mathrm{des}}$ vs $e_\Omega = \Omega - R^\top R_d\Omega_d$ | **deviation to state (APPLY, code)** | the code uses $\omega - \omega_d$ without the transport $R^\top R_d$ (`lib.rs:2221`); the two coincide when $R = R_d$. |

9. **Confidence and gaps.** High for v4 content; CDC 2010 not held.

---

## R16 — li2023ndpnmpc

1. **Reading record.** arXiv:2304.07794**v2** (12 Sep 2023), 7 pages, all read. Figures 1–7 images (captions read);
   Tables I–III read.
2. **Bibliographic check.** Jinjie Li, Liang Han, Haoyang Yu, Yuheng Lin, Qingdong Li, Zhang Ren ✓; title ✓; 2023 ✓;
   arXiv ✓. Venue per R08/R03-style reference lists: **IEEE CDC 2023, pp. 2122–2128** (R08 ref [12]) — `[VERIFY]`.
3. **What the paper does.** Trains a spectrally normalised MLP to predict the downwash force on a lower quadrotor from
   relative position and velocity, using labels from odometry acceleration minus a nominal force computed from **DShot
   rotor-speed telemetry**; feeds a predicted disturbance sequence (from neighbours' predicted states) into an NMPC
   (acados) running on an onboard TX2 NX; flies two custom 1.53 kg quadrotors under OptiTrack.
4. **Setup facts.** ENU/FLU frames (§II-B); nominal $^If_n = R\,{}^Bf_u + m\,{}^Ig$ with rotor speeds measured "during the
   training phase" and ESCs that "supported rotor speed measurement through the DShot protocol" (§II-D.3, §III-A);
   network 6-128-64-128-3 (Table II); 570 s of piloted data, **"shuffled with a ratio of 0.75 for training and 0.25 for
   testing"** (§III-B); trajectory server, NMPC, hover throttle estimator on the TX2 NX, trajectory generation and mocap on
   a PC (§III-C); room 7 × 4 × 3 m; baseline NMPC without prediction [PAMPC]; result: "75.37% reduction in the tracking
   Root-Mean-Square Error (RMSE)" in height (§III-C, abstract).
5. **Stated limitations.** False predictions cause an upward jump "before it enters the downwash area" (§III-C);
   "the proposed approach should be compared with non-prediction methods to better highlight the advantage of integrating
   predictions" (§IV).
6. **Quotable claims.**
   - "the existence of the downwash disturbance can be predicted based on the states of neighboring quadrotors. If this prediction is considered, the control loop can proactively handle the disturbance" (abstract)
   - "The resultant force can also be estimated directly using the inertial measurement unit (IMU), but the noise level is unacceptable." (§II-D, Remark)
   - "the neural network predicts a disturbance that does not exist in reality and has no corresponding counterbalancing force, which results in an unexpected ascent of the lower quadrotor." (§III-C)
   - "In the future, the proposed approach should be compared with non-prediction methods to better highlight the advantage of integrating predictions." (§IV)
7. **What it does NOT show.** No reactive/INDI comparison; no Crazyflie; sample-wise random split; separations not
   reported numerically.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence |
|---|---|---|
| C2.54 (predicted downwash force sequence in the optimiser's prediction model) | supported | §II-E, Fig. 4. |
| C2.56 (Li et al. run on a TX2 NX carried by the vehicle) | supported | §III-C. |
| Tab 2.1 row ("2, custom; neighbour-relative; NMPC with predicted force sequence; onboard TX2 NX + mocap") | supported | |
| Ch2 §2.9 gap claim | **supporting** | authors' own future-work sentence asks for exactly a prediction vs non-prediction comparison. |
| Ch5 §5.7 split-by-segment rationale | **counter-example to cite carefully** | Li et al. split by shuffled samples — the practice §5.7 warns against (state as a methodological contrast, not a criticism; H3/§2.8 tone). |
| Ch3 §3.3 rotor-speed source | **precedent** | DShot telemetry used to form the nominal force (§III-A). |
| Ch3 §3.5 / RQ2 prediction failure mode | **supporting** | spurious predicted disturbance before entering the wake (§III-C). |

9. **Confidence and gaps.** High.

---

## R17 — shankar2023docking

1. **Reading record.** arXiv:2311.13988v1 (23 Nov 2023), 10 pages, all read. Figures 1–5 images (captions read); Table 1 read.
2. **Bibliographic check.** Ajay Shankar, Heedo Woo, Amanda Prorok ✓; title ✓; 2023 ✓; arXiv ✓. Venue per R09's reference
   list: International Symposium on Experimental Robotics (ISER), Springer 2023, pp. 427–437 — `[VERIFY]`.
3. **What the paper does.** Uses the SO(2)-equivariant downwash model of Smith et al., trained by curriculum on ≈5 min
   of data, inside an onboard LQR on a feedforward-linearised follower to dock vertically onto a platform suspended
   below a leader; studies cable lengths on a fixed-leader rig and docks in free flight.
4. **Setup facts.** Model $m\vec a = -R T + mg\hat e_3$ (NED-like), torques to "a high-bandwidth inner loop"; infinite-horizon
   LQR (§2); labels "difference between Bravo's observed and commanded lateral accelerations" (§2); curriculum from
   1.8–1.5 m in 0.3 m steps to ≈0.5 m ("roughly 2 body-lengths") (§2); vehicles ≈0.27 m longest diagonal, ≈0.7 kg;
   everything onboard a Raspberry Pi 4B; motion capture indoors (§3.1); fixed-leader study: six offsets 0.36–1.35 m, all
   missed without the model and all docked with it (Table 1); free flight: $d_p$ = 0.36 m ("a little over one
   body-length"), 8 of 10 docks (§3.3); vertical error "typically less than 0.1 m"; moving leader 0.25 m/s: "very limited
   success, usually attributed to Bravo's oscillatory behavior that prevents the servo actuation from clasping"; mean
   error >0.2 m without vs <0.05 m with the model (§3.3); abstract: "less than 0.06 m for the follower (a 3-4x
   reduction) when approaching vertically within two body-lengths".
5. **Stated limitations.** Gripper response time and ≈15 ms digital latency (§4); model trained for near-hover leader;
   observer convergence too slow to rely on (§4).
6. **Quotable claims.**
   - "the follower must hover and keep formation directly underneath the leader at some point, it will experience strong downwash that can be hard for a disturbance observer to estimate online since they have a finite (non-zero) convergence time." (§1)
   - "Our evaluations show a tracking error of less than 0.06 m for the follower (a 3-4x reduction) when approaching vertically within two body-lengths of the leader." (abstract)
   - "even at 5 body-lengths (BL), Bravo is still likely to experience noticeable disturbance from Alpha." (§3.2)
   - "Bravo is successful in effecting a final dock repeatably in 8 out of 10 instances." (§3.3)
   - "in practice, the downwash function changes so rapidly that a convergence is not guaranteed." (§4, about the LQG observer)
7. **What it does NOT show.** No Crazyflie; no comparison with an online residual estimator beyond the LQG observer
   remark; no multi-follower case.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence / corrected formulation |
|---|---|---|
| C2.57 (learned lumped force online in infinite-horizon LQR on feedforward-linearised follower beneath inner attitude loop; docking within 1–2 BL; ≈0.27 m, 0.7 kg; dock ≈0.36 m; curriculum to ≈0.5 m) | supported | §1–§3. |
| C2.58 (SO(2) model of prior work; ≈5 min; onboard Raspberry Pi; mocap; hardware only) | supported | §1, §3.1. |
| C2.59 (abstract <0.06 m; body <0.1 m hovering; moving <0.05 m vs >0.2 m) | supported | |
| C2.60 (8/10 at 0.36 m; moving 0.25 m/s limited, attributed to gripper latency) | **supported with changed context** | §3.3 attributes the moving-leader failures to "Bravo's oscillatory behavior that prevents the servo actuation from clasping"; §4 to physical actuation limits (gripper response, latency). Corrected: "…which the authors attribute to the follower's oscillation together with the gripper's actuation latency". |
| Tab 2.1 row | supported | |
| Ch2 §2.3 reactive limitation (missing evidence) | **addition candidate** | §1: disturbance observers' finite convergence time under the leader. |

9. **Confidence and gaps.** High.

---

## R18 — gu2025comparative

1. **Reading record.** Journal version, J. Intell. Robot. Syst. (2025) 111:98, 20 pages (open access), all read incl.
   references and biographies. Figures 1–12 images (captions read); Tables 1–2 read.
2. **Bibliographic check.** Weibin Gu, Jiance Zhao, Alessandro Rizzo ✓; title ✓; JIRS 2025 ✓; DOI 10.1007/s10846-025-02305-5 ✓.
   The bib `note` "Abstract read via OpenAlex; full text not held (paywalled)" is **stale** — the open-access PDF is held
   (`docs/papers/gu2025comparative.pdf`). Topical collection ICUAS 2024.
3. **What the paper does.** A modular framework in which a nominal geometric-style controller (after Mellinger & Kumar)
   is augmented by an online residual-learning module (circular buffer, retrained every step) that estimates lumped
   force/torque uncertainties from a discrete Newton–Euler model; compares ELM, RBFNN, deep ESN and GP on estimation
   accuracy, tracking and compute, with Bayesian-optimised hyperparameters for the parametric models; evaluates in
   MATLAB simulation and runs the learning modules on a Jetson Orin NX in the loop with the gym-pybullet-drones simulator.
4. **Setup facts.** NED/FRD (§3); uncertainty $(f_a,\tau_a)$ lumped, labels from (10) (discrete Newton–Euler); control
   (15)–(16) $f_u = \bar f_u - \hat f_a$, $\tau_u = \bar\tau_u - \hat\tau_a$ (residual subtracted); BO "To ensure fair comparison
   among parametric models … guaranteeing that each method operates near its best achievable performance" (§1, §5.1);
   simulated disturbances: periodic wind, ground effect, constant and Gaussian wind (§5.2–§5.3); simulated vehicle
   2.95 kg; RBFNN "encountered instability and crashes during the simulation", ELM unstable "for landing tasks under
   ground effect"; offline FNN comparison fails out of distribution (§5.4); embedded: Jetson Orin NX with
   gym-pybullet-drones, 200 Hz threshold (§5.5).
5. **Stated limitations.** "we plan … to validate the approach experimentally on a real quadrotor platform" (§6); noisy
   sensors future work; GP needs approximations for real time.
6. **Quotable claims.**
   - "we systematically evaluate four representative learning methods, namely Extreme Learning Machines (ELMs), Radial Basis Function Neural Networks (RBFNNs), Echo State Networks (ESNs), and Gaussian Processes (GPs)" (abstract)
   - "GPs yield highest accuracy but require approximations for real-time use. ELMs and RBFNNs enable fast inference but suffer initialization sensitivity. ESNs provide the best balance" (abstract)
   - "To ensure fair comparison among parametric models, Bayesian Optimization (BO) is adopted for hyperparameter selection, thereby eliminating manual bias in configuration and guaranteeing that each method operates near its best achievable performance." (§1)
   - "This instability may be attributed to the random initialization of weights between the input and hidden layers of RBFNN and ELM, which can produce excessively large prediction values." (§5.3)
   - "we deployed all online learning modules directly on an NVIDIA Jetson Orin NX (16 GB) and integrated them with the open-source gym-pybullet-drones simulator" (§5.5)
   - "We also aim to validate the approach experimentally on a real quadrotor platform" (§6)
7. **What it does NOT show.** No real flight; no multi-vehicle or downwash; no offline-trained predictive family other
   than one FNN baseline; no reactive/INDI baseline.
8. **Thesis' use of it.**

| Claim | Verdict | Evidence |
|---|---|---|
| C1.08 (online-learning estimators on a single vehicle) | supported | |
| C2.75 (four estimators; modular framework; lumped wrench via discrete Newton–Euler; accuracy, tracking, compute; GP accurate but cubic; parametric fast but initialisation-sensitive; ESN balanced) | supported | abstract; §4–§5. |
| C2.76 (single quadrotor; all online; simulation throughout; "hardware" = Jetson in the loop with a simulator; real quadrotor future work) | supported | §5.5; §6. |
| C2.77 (BO tuning of ELM, RBFNN, ESN to their optima) | **supported** | §1, §5.1 — the `% UNVERIFIED (review P-010): no PDF of Abro et al.` comment placed above this paragraph is **misplaced**; it belongs above the Abro paragraph. |
| C3.07 (RBF crash under periodic wind; ELM unstable on landing in ground effect; ESN and GP did not; "may be attributed"; "can produce excessively large prediction values") | supported | §5.3 verbatim. |
| bib `note` | **stale** | full text held. |

9. **Confidence and gaps.** High.

---

## R19 — lee2010geometrictracking (CDC 2010) — **no PDF held**

Nothing is written about its content beyond what the draft already says. All equation-level checks are done against the
held companion (R15, arXiv v4). `[VERIFY]` that the CDC paper (i) contains the torque law with the feedforward
bracket as quoted in Ch4 eq. (ctrl-tau-lee), (ii) states exponential stability for initial attitude errors below 90°
and almost-global exponential attractiveness below 180° — the bib `note` says "Prop. 2 / Prop. 3"; the v4 companion
numbers them Propositions 3–4.

## R20 — abro2025fractional — **no PDF held**

Nothing is written about its content beyond what the draft already says (Ch2 L510–518, which the draft itself marks as
abstract-based in a footnote). `[VERIFY]` every clause of Ch2 L510–518 against the full text (methods compared:
FO-PID, FO-LQR, integral state feedback; close-proximity motivation; "downwash"; number of vehicles; simulation or
hardware).

---

## Summary of Phase 1 verdicts (rows of `stage0_draft_audit.md` §5)

| Verdict | Count | IDs |
|---|---|---|
| supported | 86 | all rows not listed below |
| supported with changed context | 9 | C1.06/C2.32/C2.71 (Smith 56 % abstract-only), C2.14 (Smeur 4 Hz in the wind-tunnel run), C2.20 (Shi attitude loop), C2.30 (NS2 upgraded motors), C2.36 (Smith z = −2.5 m is the leader), C2.56 (Chee flies Crazyflies), C2.60 (Shankar failure attribution), C2.70 (Cobo inputs), Tab 2.1 Hsieh 2025 row |
| overstated | 4 | C1.10 (Cobo "ground forces"), C2.09 (Gielis validates its own models), C2.41 (Gielis "claimed"), C4.08 (NS2 guarantees without hedge) |
| wrong | 5 | C2.29 (NS2: "both figures in the abstract"), C3.05 (network "not a copy of NS2's layers"), C4.10 (Hsieh 2026 simulation vs hardware), C1.10 + Ch2 §2.9 second gap (hybrid "only on a single vehicle" — contradicted by Hsieh 2025), C2.55 (Hsieh 2025 understated — it is a hybrid with estimation-only and prediction-only arms) |
| cannot tell | 3 | C2.49 numbers (Chee figures plotted only), C2.02–C2.03/C4.04 (CDC not held; consistent with v4), C2.78 (Abro, no PDF) |
| uncited claims that now have a source | 4 | C2.06 (Kiran, Gielis), C5.01 (NS2 model in the simulator), Ch3 sign derivation (Shi 2020/2022, Tal & Karaman, Cobo-Briesewitz, Hsieh 2026, Gu all subtract), Ch3 rotor-speed necessity (Cobo-Briesewitz Table 1) |
| thesis arguments contradicted by a source | 1 | "a residual observed under a compensating controller is the residual that survived compensation" (Ch4 §4.2 Role, §4.4; Ch5 §5.5.2) — Shi et al. 2020 §III-C and 2022 §IV-D state the opposite for this residual definition |
