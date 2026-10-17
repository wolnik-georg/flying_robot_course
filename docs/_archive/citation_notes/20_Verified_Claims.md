# Verified Claims Ledger

**Track:** Writing (W.2c). **Created:** 23 August 2026.

What we have actually read in each paper, and therefore what may be cited. Rules and verification
status: [`17_Source_Ledger_and_Citation_Discipline.md`](17_Source_Ledger_and_Citation_Discipline.md).

> ### Status, 24 August 2026, extended 27 August — every source read in full
> **Every source has now been read in full**, Abro et al. included (full text consulted
> 27 Aug; the earlier abstract-only status is obsolete). **The Abro PDF is not in
> `papers/`**, so nothing may be quoted from it. Each entry below reflects a full-text reading, and
> the discrepancies that reading surfaced are recorded in place. Holding the full text of Abro et
> al. does **not** license importing its numbers — see its correction block for what may and may
> not be used.

> ### Status, 26–27 August 2026 — external verification pass over eight papers
> An independent close reading checked **Neural-Swarm, Neural-Swarm2, Hsieh 2026, Cobo-Briesewitz,
> Smith, Chee and Gielis** (26 Aug), then **Hsieh 2025** (27 Aug), against the draft and returned
> corrections for all eight. They are recorded below under **⚠️ Corrections** in each paper's
> entry, and every one has been applied to the chapters.
>
> **Three of the eight share authors and a platform and are repeatedly confused. Keep them apart:**
> **Chee 2024** = two robots, KNODE-DW NMPC · **Hsieh 2025** = three robots, \(\mathcal{L}_1\) on
> top of it, hardware-only · **Hsieh 2026** = two robots, flatness-based FBL, source of the
> simulation-only 20× figure.
>
> **Provenance differs from the rest of this file.** These findings come from that external pass,
> not from a fresh reading on our side. Where it supplied the paper's own words they appear in
> quotation marks; everything else is stated as fact without quotation, and should be re-checked
> against the PDF before being quoted verbatim in the thesis.
>
> The recurring failure they expose is not misreading a number but **attaching it to the wrong
> thing** — the wrong axis, the wrong training set, the wrong platform, the wrong experiment, or
> an abstract-level summary presented as a body result.

**Depth of reading is recorded per paper.** "Abstract-level" supports *what the paper proposes and
claims*; it does **not** support a claim about experimental detail unless that detail is quoted
below. Quotation marks mean the words are the paper's, taken from the PDF in
[`papers/`](papers/) — not paraphrased from memory.

---

## Shi, Hönig, Yue, Chung — *Neural-Swarm* (2020), arXiv:2003.02992
**Read:** abstract + experimental section.

| Claim | Evidence |
|---|---|
| Permutation-invariant DNN learning multi-vehicle interactions | *"combines a nominal dynamics model with a regularized permutation-invariant Deep Neural Network (DNN)"* |
| Reduces worst-case height error 2–4× | *"reduces the worst-case z-error by a factor of two to four (e.g., 2.4 cm instead of 9.4 cm for the two vehicle case)"* |
| **Separate** networks per team size | *"f̂a computed using different neural networks (trained on data flying 2, 3, and 4 quadrotors, respectively)"* — **not one model trained on the combined set** |
| Generalises beyond the training team size | *"generalizes to cases with more vehicles"* |
| Baseline is a **nonlinear** tracking controller | *"our position tracking controller (9) … and a nonlinear attitude tracking controller [24]"*, where [24] is Lee/Leok/McClamroch. Do not describe the baseline simply as "a geometric controller" |
| **Vertical separation in the swapping task: 25 cm** | *"Each quadrotor flies at a different fixed height (25 cm vertical separation)"* |
| Prior practice required ~60 cm separation for Crazyflie 2.0 | *"a large safety distance between vehicles, e.g., 60 cm for the small Crazyflie 2.0 quadrotor"* |

> ### ⚠️ **Superposition is explicitly stated NOT to hold**
> *"for the case with more than two vehicles, the aerodynamic effect is **not a simple
> superposition of each pair**"*
>
> This materially affects our **RQ3**. Non-superposition is not something this thesis would
> discover — it is already reported. See §"Consequences" below.

> ### ⚠️ Corrections, 26 Aug
>
> | Was | Is |
> |---|---|
> | "2.4 cm instead of 9.4 cm **for the two vehicle case**" read as the two-vehicle-*trained* result | The **9.4 cm baseline** and **2.4 cm** are the two-vehicle *task*; the 2.4 cm net was **trained on four** vehicles. The net trained on **two** reaches **2.7 cm**. The abstract's phrasing does not distinguish task from training set |
> | "a factor of two to four" as a uniform improvement | **"up to four times smaller"** worst-case height error; **a factor of two or better** in general. Do not imply 4× at every team size |
> | 9 cm quadrotor "**commonly flown**" at 60 cm | **Crazyflie 2.0, 9 cm rotor-to-rotor.** 60 cm is an **example of a conservative planning safety distance** when interaction is unmodelled, cited by Shi from prior planning work — not a Neural-Swarm measurement and not "commonly flown" |
> | 25 cm as *the* separation | Swap experiments use **0.20–0.25 m**; 25 cm is explicit for the **three-vehicle** figure |
> | "Generalises to larger teams" unqualified | **Directional.** Nets trained on **3 or 4** improved 5-vehicle flight; a net trained on **2 did not generalise** — worse than baseline on 3- and 4-vehicle tests |
> | Position controller described as geometric-style | Learned force is **feed-forward in a decentralised nonlinear tracking position law**. Geometric \(SE(3)\) is an **optional attitude inner loop** only |
> | Team size "2–4 (trained)" in the evaluation table | **Trained on 2–4, tested on 2–5**; metric is **max \|z\| error**; Δz **0.20–0.25 m** |
>
> Also true, and safe to state: they learn **\(f_{a,z}\) only**, and the network runs **onboard the
> STM32**.

---

## Shi, Hönig, Shi, Yue, Chung — *Neural-Swarm2* (arXiv 2020; T-RO 2022), arXiv:2012.05457
**Read:** full text (2026-08-24). The threefold figure appears only in the abstract; no body experiment ties it to the 16-robot result.

| Claim | Evidence |
|---|---|
| Heterogeneous deep sets, any number of neighbours, permutation-invariant | *"heterogeneous deep sets for supporting any number of heterogeneous neighbors in a permutation-invariant manner without reducing expressiveness"* |
| **Spectral normalisation** for stability/generalisation | *"spectral normalization for stability and generalization guarantees of unseen data"* — **not in our matrix; an architectural choice we did not adopt** |
| Residual used in **both** planning and control | *"benefit both the proposed interaction-aware multirobot motion planning and the nonlinear tracking control design"* |
| Up to 3× reduction in worst-case tracking error | *"up to three times reduction in worst-case tracking errors"* |
| **16 robots, minimum vertical distance 24 cm** ✓ our matrix | *"(minimum vertical distance 24 cm) of heterogeneous aerial teams (16 robots)"* |
| Also models **ground effect** | *"such as downwash generated by nearby drones and ground effect"* |

> ### ⚠️ Corrections, 26 Aug
>
> | Was | Is |
> |---|---|
> | 16 robots + 24 cm + 3× liable to be read as one experiment | **Never put them in one cell or one sentence.** On the 16-robot flight the **body** reports the max \(z\)-error of a small vehicle falling **15 cm → 10 cm**, i.e. **≈1.5×**, not 3× |
> | "Up to three times" as a body result | **Abstract-level summary**, against a baseline nonlinear tracking controller **with the interaction force set to zero**. The body never assigns 3× to the 16-robot trial |
> | "Any number of neighbours" as an accuracy claim | **Architectural / permutation-invariant property.** Accuracy at 16 robots is degraded — **"relatively less accurate"**; trained on **≤3 vehicles / ≤2 neighbours** |
> | Spectral normalisation gives "guarantees on unseen data" | That is the **abstract's** wording. The body rests on **Lipschitz bounds + empirical generalisation + a bounded-residual assumption** used in the proofs. **Do not upgrade to a PAC or formal unseen-data theorem** |
> | — | Planning **and** control both use the residual in that paper. Our Strategy 2 dropping the planning half stays labelled **our deviation** |

---

## Gielis, Shankar, Kortvelesy, Prorok — *Modeling Aggregate Downwash Forces* (2023), arXiv:2312.03488
**Read:** full text (2026-08-24).

| Claim | Evidence |
|---|---|
| Single-vehicle downwash models do not compose | *"an accurate model of downwash forces from one vehicle is unlikely to be sufficient for predicting aggregate forces from multiple vehicles in formation"* |
| Force measurement on a **test rig**, not free flight | *"an experimental test rig designed to capture 6-DOF exogenic forces acting on a multirotor frame"* |
| Graph-based aggregation, contrasted with linear models | *"by modeling the formation as a graph, we present a novel approach for learning the force aggregation function, and contrast it against simpler linear models"* |
| Generalisation tested by adding a fourth vehicle | *"how our proposed models generalize when a fourth vehicle is added"* |
| Strong nonlinearity in aggregation | *"strong non-linearities present in the aggregation of these interactions"* |

| **Interactions fall into three regimes** | *"our results … generally fall into one of three categories: 1. Chaotic Interactions: those that are dominated by noise; 2. Linear Interactions: those that are well estimated by linear summation; and, 3. Non-linear Interactions: those that can only be estimated with non-linear models"* |

> **This refines the superposition story rather than simply refuting it.** Some interactions *are*
> well estimated by linear summation; which regime applies depends on the formation and its location
> in the state space. Recorded in Chapter 2 §2.4.1.

❌ **Our matrix's "Usable onboard" is REFUTED.** The full text contains no discussion of onboard
deployment, real-time inference or embedded execution. Removed from the matrix.

> ### ⚠️ Corrections, 26 Aug
>
> | Was | Is |
> |---|---|
> | "Test rig, not free flight" — ambiguous about *whose* flight | The vehicle **experiencing** the disturbance is **held on a load stand at hover thrust**. The neighbours **do** fly freely. The sufferer is **not** in free flight |
> | "Up to 4 vehicles" | **4 in total = 1 constrained sufferer + up to 3 free neighbours (K ≤ 3).** Not four free-flying robots, and not K = 4 neighbours |
> | "Modelling the formation as a graph" as the method | **Abstract language.** The implementation is **Deep Sets (sum-pool)** contrasted against linear summation. **Do not write GNN or graph learning as the method** |
> | "Generalisation tested by adding a fourth vehicle" as a demonstrated result | **Abstract-level.** Body tables stop at **K = 3**, and the conclusion is **pessimistic about K > 3**. Do not claim a successful held-out four-neighbour result |
> | Three regimes | Keep the paper's own labels: **chaotic / noise-dominated**, **linear / well estimated by linear summation**, **nonlinear / requires a nonlinear model** |
> | Superposition support | The aggregation failure here is **Gielis's own load-stand result**. **Do not attach Shi's free-flight superposition numbers to this citation** — keep the two as separate supports |
>
> "In our reading, the natural ground truth for a learned model" **remains our reading.** The paper
> validates its own linear and Deep-Sets predictors on the stand; it does not claim that role.

---

## Smith, Shankar, Gielis, Blumenkamp, Prorok — *SO(2)-Equivariant Downwash Models* (2023), arXiv:2305.18983
**Read:** full text (2026-08-24).

| Claim | Evidence |
|---|---|
| **36 % / 56 %** — with the context that matters | *"In dense real-world flights with two vehicles, deploying our model online improves 3D trajectory tracking by nearly 36 % on average (and vertical tracking by 56 %)"* |
| Strong data efficiency from geometry | *"trained with only 5 minutes of real-world flight data, our geometry-aware model outperforms state-of-the-art baseline models trained with more than 15 minutes of data"* |

| **Tightest separation reached: ≈0.5 m** | *"at stage-2 the vehicles have a vertical separation of only ≈ 0.5 m (approx. two body-lengths)"* |

| ⚠️ **The paper is internally inconsistent about the 36 %** | Abstract: *"improves 3D trajectory tracking by nearly 36 % on average (and vertical tracking by 56 %)"*. Contributions list: *"reduce **vertical** tracking errors by 56 % and **lateral** tracking errors by 36 %"*. Results (Fig. 7, a lateral transect beneath the upper vehicle): *"reduce mean position and velocity tracking errors by almost 36 % … with over 55 % improvement in ê3 axis alone"* |
| Two identical vehicles, controllers onboard | *"two identical multirotor vehicles … They have similar estimation and control stacks onboard"* |
| A different manoeuvre gives 51 % | *"The equivariant model produces an improvement of nearly 51 % in both position and velocity tracking"* |

> ### ⚠️ Corrections, 26 Aug — this reverses the guidance that stood here
>
> The box previously here said to use **"56 % vertical, 36 % lateral"** as the specific reading.
> **That is now reversed.** The primary assignment follows the **abstract and the
> translation/lemniscate results body**:
>
> **≈36 % improvement in 3D tracking and 56 % in vertical tracking, two-vehicle flights, "on
> average".** Do **not** use "36 % lateral" as the headline. The contributions list's "lateral 36 %"
> is the paper's internal inconsistency, not the reading to follow.
>
> Footnote wording to use: *the paper is internally inconsistent on whether 36 % is 3D or lateral;
> we follow the abstract and the translation-result body (3D + vertical); the contributions list
> instead calls the 36 % figure lateral.*
>
> | Was | Is |
> |---|---|
> | Transect sentence carrying **both** 56 % and 36 % | On the lateral transect beneath the hovering vehicle: **mean 3D position error 0.154 → 0.098 m (≈36 %)**, with vertical improvements of **55 % and 49 %** |
> | "5 min outperforms baselines trained on 15 min" | That is **validation RMSE against state-of-the-art learning baselines** — **not** a closed-loop tracking result |
> | Controller unnamed / "controllers onboard" | **LQR**, an "optimal feedback controller", deployed **online**. Two **identical** vehicles |
> | "Tightest separation ≈0.5 m" applied to all tests | **0.5 m is the closest *training* stage** (≈two body-lengths). **Evaluation uses 0.6–0.8 m**, plus a lemniscate beneath the upper vehicle at \(z = -2.5\) m |
> | Platform assumed Crazyflie-like | **Not a Crazyflie.** Custom quadrotor, **≈0.26 m span, ≈0.7 kg**, OptiTrack, **Raspberry Pi at 50 Hz control / 45 Hz network** |
> | Evaluation-table quantity "lateral and vertical" | **"3D and vertical tracking"** |

---

## Cobo-Briesewitz, Wahba, Hönig — *Learned INDI* (L4DC 2026), arXiv:2503.09441
**Read:** abstract, results, conclusion. Fully verified — see [`17`](17_Source_Ledger_and_Citation_Discipline.md) §1.

| Claim | Evidence |
|---|---|
| Headline is **sensor elimination** | *"a neural network can generate smooth approximations of INDI outputs without requiring specialized rotor RPM sensor inputs"* |
| Two methods: LINDI and NA-INDI | LINDI replaces INDI's residual estimate; NA-INDI is the hybrid |
| Compensation ≈ 40 % better than geometric | *"residual prediction can reduce the tracking error … by around 40 %"* |
| Hybrid margin is **small** (no payload) | *"NA-INDI leads to the best performance, although with a small margin"* — 4.14 vs 4.28 cm |
| Hybrid ≈ INDI (with payload) | *"remains similar in performance"* — 5.39 vs 5.38 cm |
| Circle trajectory was out of distribution | *"a slight drop in performance, likely because the trajectory is out of distribution relative to the training data"* — **direct support for our RQ2** |
| Test trajectories were unseen | *"two unseen flight trajectories which are not present in the dataset used to train the neural network"* |
| LINDI trains on smoothed data | *"trains a neural network on smooth data based on spline fitting"* (conclusion) |

> ### ⚠️ Corrections, 26 Aug
>
> | Was | Is |
> |---|---|
> | "≈40 % better than geometric" as a general result | **No-payload only.** **With a payload the reductions are ≈19–24 %.** Never write it as "any residual compensation in all conditions" |
> | OOD circle drop stated generally | Applies to **LINDI, no-payload only**. **Not** to NA-INDI on the circle, and **not** to the payload circle |
> | Network predicts "the bulk" | The paper writes \(f_a = f_{a,\mathrm{NN}} + f_{a,\mathrm{INDI}}\) and INDI **reasons about the remaining mismatch**. Prefer **"remaining mismatch"**; keep "bulk" at most as one short gloss. **Our Strategy 4 form is unchanged** |
> | "No RPM required" | True at **LINDI runtime**. RPM is still needed **for the training labels** and **for the classical / NA-INDI INDI branch** |
> | Residual described as self-induced incl. "payload swing", predicted from ego state | The introduction names **blade flapping, drag, ground forces**. **Swing is not named as a residual** — cable tension sits in the **nominal payload model**. Payload network input is **28-D**: vehicle **and payload** velocity, acceleration, and **cable direction** — not ego state alone |
> | Hybrid margin "small" applied generally | **"Small" is the authors' word for NA-INDI vs INDI *without* payload.** With payload they say **"similar"**, not "small margin" |
>
> **Single vehicle only.** Nothing here supports a multi-robot reading.

---

## Zhang, Zhang, Mueller — *ProxFly* (2024), arXiv:2409.13193
**Read:** full text (2026-08-24). The no-communication claim is confirmed in the body: *"it takes only ego states and command outputs but doesn't rely on any forms of communication."*

| Claim | Evidence |
|---|---|
| Residual RL on top of a cascaded controller | *"a residual module on top of a cascaded controller … to generate high-level control commands"* |
| **Requires no inter-vehicle communication** | *"takes only the ego state and controllers' commands as inputs and does not rely on any communication between quadcopters, thereby reducing the bandwidth requirement"* |
| Domain randomisation relaxes system-ID accuracy | *"Through domain randomization, our method relaxes the requirement for accurate system identification"* |
| **Separation 50 cm; docking from 5 cm** | *"hovers 50cm above the large quadcopter (LQ)"*; docking stage: *"falls freely from 5cm above"*. Note the pair is **heterogeneous** (small + large quadcopter) |

> **Architecturally distinct from every other strategy here.** Strategies 2, 3, 4 and 6 all consume
> *relative state* and therefore need neighbour positions. ProxFly consumes only ego state. That is
> a genuine advantage for **RQ4** (no communication, no localisation of neighbours) and a genuine
> limitation: it cannot anticipate a neighbour it cannot sense.

---

## P.-A. Hsieh, Yang, Matni, M. A. Hsieh — *Flatness-Preserving Residual Learning* (2026), arXiv:2607.12275
**Read:** full text (2026-08-24).

| Claim | Evidence |
|---|---|
| Residual learned while **preserving differential flatness** | *"captures complex aerodynamic interactions while ensuring the joint multi-quadrotor system remains differentially flat"* |
| Feedback linearisation with feedforward cancellation | *"a computationally efficient feedback linearization controller … cancels aerodynamic disturbances via feedforward compensation"* |
| **31 %** average tracking-error reduction | *"reduces average tracking errors by 31% compared to nominal baselines"* |
| **Matches NMPC at ~10× less computation** | *"matches the tracking performance of state-of-the-art nonlinear model predictive control (NMPC) while requiring an order of magnitude less computation"* |
| **Under 30 s of training data; 5 ms loop** | *"the first to show that stable, tight formation flight can be achieved with under 30 seconds of training data and a 5ms loop rate"* |
| **Two quadrotors, 0.2–0.5 m vertical** | *"flying two quadrotors in a stacked formation at vertical separations of 0.2, 0.3, 0.4, and 0.5m"* (§ evaluation) |

The last two rows are the most consequential findings in this ledger for **RQ4** — they set the
bar for what "cheap" means, and they are strong claims by the authors.

> ### ⚠️ Corrections, 26 Aug — most of the rows above are **abstract-level**
>
> | Was | Is |
> |---|---|
> | **31 %** cited as a result | **Abstract only** — *"reduces average tracking errors by 31% compared to nominal baselines."* **The body does not restate it.** Hardware body figures: **ROM-FBL vs nominal FBL — 24 % RMSE, 30 % \(z_{\max}\)**; **learned FBL vs ROM-FBL — a further 29 % RMSE, 43 % \(z_{\max}\)**. Prefer the body numbers if a figure is needed |
> | **20× solve time** | **Simulation only** — ≈1 ms vs ≈27 ms at equal residual (Table 4). The **introduction separately says 7×**; the **abstract says "an order of magnitude"**. **Never write 20× as a hardware result** |
> | "On par with Chee" | On hardware they **match Chee's *published* ≈5 cm RMSE / ≈10 cm \(z_{\max}\)** — **not** a same-rig timing comparison. In **sim Table 4 the FBL \(z_{\max}\) can be worse than NMPC's.** Do not flatten this |
> | "Under 30 s of training data" | **28 s hardware, 18 s simulation.** "Under 30 s" is fine only if not implied to be one run |
> | **5 ms loop** implying onboard | **Off-board, confirmed** — Intel **i5 laptop**, commands over **Crazyradio at 400 Hz**. The 5 ms figure is **abstract-level and off-board**, **not an onboard measurement** |
> | Force-only compensation stated generally | **Hardware only** (commands thrust + angular velocity). The **simulation still models torque residuals** |
> | Evaluation table "0.2–0.5 m" | That range is **sim / prediction evaluation** (closed-loop sim 0.2–0.6 m). **Hardware tracking comparisons are commanded Δz = 0.3 m and 0.4 m** |
>
> Unchanged and safe: **team size 2**; **Crazyflie 2.1 with thrust-upgrade bundles** — very nearly
> this thesis's platform.

---

## Yang, Welde, Matni — *Learning Flatness-Preserving Residuals for Pure-Feedback Systems* (2025), arXiv:2504.04324
**Read:** full text (2026-08-24). The **theory** underneath the 2026 paper. Both claims confirmed in the body; validation is explicitly simulation-only, on a 2D quadrotor.

| Claim | Evidence |
|---|---|
| Generic residuals can destroy flatness | *"generic residual parameterizations may destroy flatness, limiting the applicability of flatness-based planning and control methods"* |
| Lower-triangular residuals preserve flatness | *"residuals with a lower-triangular structure preserve both the flatness of the system and the original flat outputs"* |
| **Validated in simulation only** | *"validated in simulation on a 2D quadrotor subject to unmodeled aerodynamic effects"* — no hardware |

---

## Chee, P.-A. Hsieh, Pappas, M. A. Hsieh — *Learning-based MPC for Tight Formations* (2024), arXiv:2410.09727
**Read:** full text (2026-08-24). Physical experiments confirmed; platform is Crazyflie.

| Claim | Evidence |
|---|---|
| Hybrid first-principles + data-driven residual | *"combines the benefits of first-principles modeling and data-driven approaches"* |
| Data-driven part kept light for optimisation | *"lightweight, making it amenable for optimization-based control design"* |
| Evaluated in simulation **and** hardware | *"Through simulations and physical experiments"* |
| Hardware is **Crazyflie** quadrotors | *"Crazyflie (CF) quadrotors flying in lemniscate trajectories"* — the same platform family as this thesis |

> ### ⚠️ Corrections and additions, 26 Aug
>
> **This is the two-vehicle stacked-formation KNODE-DW NMPC paper.** It is **not** three robots, the
> NMPC does **not** run onboard, and it is **not** the 2025 \(\mathcal{L}_1\) paper. The three-robot
> I-stack / V-stack work is **Hsieh 2025** — keep them apart.
>
> | Item | Verified |
> |---|---|
> | Where the residual enters | **Inside the NMPC prediction constraint**, as a mixed-expert KNODE-DW model — **not** an instantaneous feedforward term |
> | Platform | **Two Crazyflie 2.1**, stacked; **bottom vehicle carries a thrust-upgrade bundle**; body length **10 cm**, mass **≈34 g** |
> | Off-board | **Intel i5 laptop**, **Crazyradio ≈400 Hz**, **Vicon ≈120 Hz**. Onboard firmware is **PID on thrust and body rates** |
> | Hardware result vs nominal MPC | **40.1 % better average RMSE**; **57.5 % smaller \(z_{\max}\)**. **Chee's own hardware numbers** — usable in Chee's paragraph or table, **never assigned to Hsieh** |
> | Training data | **46 s total = 18 s simulation + 28 s hardware.** Not 46 s of hardware |
> | Tightness | Average separation **< 1.5 body lengths**; one tight case averages **Δz = 0.12 m (≈1.2 BL)**. Commanded separations **0.2–0.4 m**, plus a very tight **0.08 m** test |
> | ❌ **20× solve time is NOT in Chee** | That figure belongs to **Hsieh 2026, in simulation**. Never write it as a Chee result |
>
> Safe one-sentence form: *Chee et al. embed a mixed-expert KNODE-DW model in the NMPC prediction
> constraint and demonstrate, in simulation and off-board hardware with two Crazyflie 2.1 vehicles,
> stacked flight with average separation below 1.5 body lengths.*

---

## P.-A. Hsieh, Chee, M. A. Hsieh — *Online Adaptation for Flying Quadrotors in Tight Formations* (2025), arXiv:2506.17488
**Read:** full text (2026-08-24). The two-three-quadrotor-formations claim is confirmed in the body.

| Claim | Evidence |
|---|---|
| **Method name is L1 KNODE-DW MPC** — the paper's own | *"we present L1 KNODE-DW MPC, an adaptive, mixed expert learning based control framework"* |
| **Two three-quadrotor formations** | *"We evaluate L1 KNODE-DW MPC in two different three-quadrotor formations"* |
| L1 works best paired with an accurate model | *"the L1 adaptive module compensates for unmodeled disturbances most effectively when paired with an accurate dynamics model"* |

> Correction to our own earlier note: "L1 KNODE-DW MPC" is **the paper's method name**, not our
> shorthand. The paper's *title* is *Online Adaptation for Flying Quadrotors in Tight Formations*.
> Both are correct; do not treat the method name as an error.

> ### ⚠️ Corrections, 27 Aug
>
> **Keep three papers apart.** This is the **three-robot, hardware-only, V-stack / I-stack** paper.
> It is **not** Chee 2024 (two robots, KNODE-DW NMPC) and **not** Hsieh 2026 (two robots,
> flatness-based FBL). All three share authors and a platform, which is exactly why they get
> confused.
>
> | Item | Verified |
> |---|---|
> | Team and platform | **Three Crazyflie 2.1.** **Hardware-only evaluation** — do **not** write "simulation and hardware" for this cite |
> | Formations | Named **V-stack** and **I-stack**. "Two different three-quadrotor formations" is the paper's phrasing for these two |
> | Separations | \(z_1 = \SI{0.2}{\metre}\) in the reported tests; \(z_2 \in \{0.2, 0.3, 0.4\}\) m; **V-stack \(r = \SI{0.1}{\metre}\)**. The tight I-stack is \(z_1 = z_2 = \SI{0.2}{\metre}\) — **two body-lengths pairwise** |
> | Where the estimate enters | An **\(\mathcal{L}_1\) residual-acceleration estimate inside the KNODE-DW prediction model** |
> | What adapts online | **Only the \(\mathcal{L}_1\) estimate (\(\hat\sigma\)).** KNODE-DW **reuses Chee's training data**; the NODE is **not retrained online** |
> | Off-board | **Bottom vehicle: Intel NUC13 i7 at \SI{200}{\hertz}. Top/centre: i5 laptop at \SI{400}{\hertz}.** Vicon \SI{120}{\hertz}, **acados**, commanding **thrust + body rates** |
> | ❌ **20× solve time is NOT in this paper** | That figure is **Hsieh 2026, in simulation**. Never attach it here |
> | Evaluation-table row | team **3**; **V-stack and I-stack**; tracking against the **MPC family** (nominal / DW / KNODE-DW / L1 / L1-DW / L1-KNODE-DW) |
>
> **The abstract sentence is a quotation, and our gloss is not.** Use
> *"the L1 adaptive module compensates for unmodeled disturbances most effectively when paired with
> an accurate dynamics model"* as the paper's words. **"Rather than substituting for one" is our
> paraphrase of their ablation** — it must stay outside the quotation marks and marked as our
> reading, which Chapter 2 now does.
>
> Safe one-sentence form: *Hsieh et al. (2025) insert an \(\mathcal{L}_1\) residual-acceleration
> estimate into the KNODE-DW NMPC prediction model and evaluate it on three Crazyflie 2.1 vehicles
> in V-stack and I-stack formations, including a vertically aligned stack with pairwise separations
> of \SI{0.2}{\metre}.*

---

## Kiran, Ayanian, Breuer — *Static and Dynamic Downwash Interactions* (2025), arXiv:2507.09463
**Read:** full text (2026-08-24). Forces, torques and PIV confirmed in the body.

| Claim | Evidence |
|---|---|
| Forces, moments **and** PIV flow fields | *"measurements of forces and torques … and particle image velocimetry (PIV) to quantify the spatial features of the downwash wake"* |
| PIV covers a single quadrotor and a **pair** | *"for a single quadrotor and an interacting pair of quadrotors"* |
| Intended to inform physics-based models | *"This data can be used to inform physics-based …"* |

---

## Bauersfeld, Muller, Ziegler, Coletti, Scaramuzza — *Robotics meets Fluid Dynamics* (RA-L 2024), arXiv:2403.13321
**Read:** full text (2026-08-24). Found during W.2b. The 2.5-diameter and 16-hour figures are confirmed in the body, which uses the term *far-field* for the region beyond that distance.

| Claim | Evidence |
|---|---|
| Combined propeller flow ≈ **turbulent jet beyond 2.5 drone-diameters** below | *"the combined flow from all drone propellers is well-approximated by a turbulent jet after 2.5 drone-diameters below the vehicle"* |
| Built from **16 h** of flight data across drone sizes | *"over 16 hours of flight data from drones of varying sizes"* |
| Lightweight alternative to CFD | *"computationally lightweight model … Unlike related approaches that rely on expensive computational fluid dynamics (CFD) simulations"* |
| ⚠️ **The model is for HOVER** | *"a computationally lightweight model for estimating the time-averaged magnitude of the induced flow below quadrotors **in hover**"*. Do not state the turbulent-jet result unconditionally |

A useful physical scaling law for §2.2, and a natural sanity check on our own measured residual.

---

## Tal, Karaman — *Accurate Tracking of Aggressive Quadrotor Trajectories using INDI and Differential Flatness* (2018), arXiv:1809.04048
**Read:** full text (2026-08-24). Both quoted claims confirmed in the body.

| Claim | Evidence |
|---|---|
| Tracks position/yaw derivatives to 4th order | *"up to fourth order, specifically, velocity, acceleration, jerk, and snap along with yaw rate and yaw acceleration"* |
| Jerk/snap feedforward via differential flatness | *"tracked using feedforward inputs for angular rate and angular acceleration based on the differential flatness of the quadcopter dynamics"* |
| **Snap tracking requires motor-speed feedback** | *"Snap tracking requires direct control of body torque, which we achieve using closed-loop motor speed control based on measurements from optical encoders attached to the motors"* |
| **No prior aerodynamic model needed** | *"prior modeling of aerodynamic effects is not required"* |

The last two rows are exactly the reactive family's defining trade-off in our framing: general
because it needs no model, but dependent on rotor-speed sensing.

---

## Smeur, de Croon, Chu — *Cascaded INDI for MAV Disturbance Rejection* (2017; journal 2018), arXiv:1701.07254
**Read:** full text (2026-08-24). The 10 m/s wind-tunnel experiment is confirmed in the body.

| Claim | Evidence |
|---|---|
| INDI cascaded for attitude **and** position | *"the cascaded integration of Incremental Nonlinear Dynamic Inversion (INDI) for attitude control and INDI for position control"* |
| Demonstrated against a **10 m/s wind-tunnel exhaust** | *"an experiment where the quadrotor flies in and out of a 10 m/s windtunnel exhaust"* |
| Tolerates infrequent position updates | *"does not rely on frequent position updates … using a standard GPS module"* |

---

## Lee, Leok, McClamroch — *Control of Complex Maneuvers … on SE(3)* (2010), arXiv:1003.2005
**Read:** equation level (2026-08-24) — this is where the control law we implement was verified,
character by character. See the dedicated Lee entry above for the primary-citation split.

| Claim | Evidence |
|---|---|
| Nonlinear tracking on SE(3) per flight mode | *"A nonlinear tracking controller is developed on the special Euclidean group SE(3) for each flight mode"* |

> ✅ **Resolved.** CDC 2010 is the primary citation; this arXiv paper is secondary and is where the
> equations were verified. See the dedicated Lee entry above.

---

## The "first systematic comparison" audit (W.10, 2026-08-23)

**Searched:** arXiv, via its API, with keyword combinations covering downwash + comparison /
benchmark / comparative study; close-proximity + benchmark; INDI + learning-based; surveys of
aerodynamic interaction; disturbance rejection + comparison; formation flight + downwash +
learning; interaction force + compensation.

**Found:** no study comparing reactive, predictive and hybrid interaction-force compensation
against one another under common conditions. No survey of the area.

**Also found — two relevant papers not previously in our set,** now added:

| Paper | Relevance |
|---|---|
| Li et al., *NDP-NMPC* (2023), arXiv:2304.07794 | Neural downwash predictor inside NMPC, trained with **spectral normalisation**. Directly relevant to Strategy 6, and a second instance of the injection point in §2.5 |
| Shankar, Woo, Prorok, *Docking Multirotors using Learnt Downwash Models* (2023), arXiv:2311.13988 | Learned downwash inside an optimal feedback controller for **vertical docking**, leader–follower, 1–2 body-lengths. Was in our `06` list as unread |

### Audit 2 — cross-database sweep (2026-08-23, same day)

**Searched:** OpenAlex (title+abstract filters, 2017 onward), Crossref, DBLP and Semantic Scholar
— which index IEEE Xplore, the ACM Digital Library, Springer and Elsevier, and so reach the venues
arXiv cannot. Plus, more decisively, a **citation-graph scan**: every work citing Neural-Swarm,
Neural-Swarm2 or the Gielis aggregate-downwash paper — **157 citing works** — filtered for titles
containing *compare / benchmark / evaluate / survey / review / versus / trade-off*.

The citation-graph scan is the stronger test. Any systematic comparison of these methods would
almost certainly cite at least one of those three papers, so this asks the question structurally
rather than by guessing keywords.

**Found: two genuine comparison studies, both adjacent, neither ours.** Both are now cited in
Chapter 2 §2.8.

| Study | What it compares | Why it is not this thesis |
|---|---|---|
| Gu, Zhao & Rizzo, *Learning uncertainties online for quadrotor flight control: a comparative study*, JIRS 2025 | Four **online-learning** uncertainty estimators (ELM, RBFNN, ESN, GP) — accuracy, tracking, real-time cost, in simulation **and** hardware | **Single vehicle**, generic uncertainty, **online** learning. Compares estimators *within* one family, not the reactive/predictive/hybrid families. Full text obtained (open access) |
| Abro, Alsayegh & Mustafa, *High-Precision UAV Swarm Control: Evaluating Fractional-Order Methods for Close-Proximity Operations*, IEEE JESTIE 2025 | **FOPID, FOLQR and integral state feedback (ISFA)** on close-proximity, downwash-motivated flight | Multi-vehicle and downwash-aware, but all three are **feedback regulators**: they do not present a **learned or first-principles predictor** of the interaction force in the Neural-Swarm sense. **Full text now held** — see the correction block below |

**These sharpen the claim rather than threaten it — and they improve it.** The defensible statement
is no longer "nobody compares controllers for close-proximity flight", which Abro et al. would
falsify. It is that the families defined by *how they obtain information about the interaction
force* — measure it, predict it, or both — have not been placed under common conditions. Gu et al.
also supplies a methodological precedent: a comparison whose conclusion is a set of trade-offs
rather than a winner, which is the shape this thesis should aim for.

### ⚠️ Abro et al. — corrections, 27 Aug (full text now held)

**The "paywalled / abstract only" status is obsolete.** The full text is in hand, and the footnote
claiming otherwise has been removed from Chapter 2. **This does not license importing its numbers.**

| Was | Is |
|---|---|
| Footnote: *"only the abstract was available; full text behind a subscription"* | **Deleted.** Replaced with: cited for the methods compared and the downwash / safety-distance motivation; the abstract's quantitative improvement and "larger swarm" claims are **not used** |
| "Benchmarks **classical regulators**" | Compares **FOPID, FOLQR and integral state feedback (ISFA)**. **Only ISFA is classical**; the fractional-order laws are not. **Do not add plain LQR** as a compared method — it is named in the contributions but not in the result tables |
| "**None estimates or predicts** the interaction force" | Softened: all three are **feedback regulators**; they do **not present a learned or first-principles predictor** of the interaction force in the Neural-Swarm sense. **Do not** write "disturbance-blind PID only" — §II does write a **linear \(f_a\) model** — and **do not** write that they learn the residual |

**Do not import into the thesis:** the abstract's **4× vs a nonlinear tracking controller** (the
body tables compare FOLQR against ISFA, FOPID and a "Reference Line", **not** against Lee's
\(SE(3)\) law; \(n = 2\), **Z-axis only**); the **0.5 / 0.85 / 1.1 m** tables; the **CoDrone**
platform details. **They do not fly Crazyflies, and they did not demonstrate large swarms.**

**Keep:** the work is motivated by downwash and by the large standoffs that follow from ignoring it.

### ⚠️ What these audits do **not** establish

Both are keyword- and citation-based. Neither is exhaustive: a study using different terminology,
or one that cites none of the three seed papers, would be missed. Direct search of IEEE Xplore and
the ACM DL through their own interfaces was not possible (API keys / subscription), though their
content is indexed by the databases used. **Absence of evidence in a structured search is stronger
than in a keyword search, but it is still not proof of absence.**

**Consequence for the wording.** The claim is retained as *"to the best of our knowledge"* and is
now stated against the two adjacent comparisons rather than in a vacuum, which is both more honest
and more informative to a reader who knows the field.

---

## Gu, Zhao & Rizzo — *Learning uncertainties online: a comparative study* (JIRS 2025)
**Read: deep** — problem setup, method, all of §5, conclusion. `papers/gu2025comparative.pdf`.

The closest methodological precedent to this thesis, and the only paper we hold that is *itself* a
systematic comparison. Read in full for that reason.

| Claim | Evidence |
|---|---|
| Same model form as ours | Eqs. (1a)–(1d): `ṗ=v`, `mv̇ = mge₃ + Rf_u + f_a`, `Jω̇ = −ω^Jω + τ_u + τ_a`, with `f_a, τ_a` the *"lumped uncertainties … unmodeled dynamics and disturbances"*. Note **e₃ points downward** (NED); ours is z-up |
| Modular framework, swappable estimator | *"a nominal controller with an online residual learning module"*; *"supports the seamless incorporation of various online learning methods"* |
| Four estimators compared | ELM, RBFNN, ESN, GP — parametric and non-parametric |
| **Fairness by per-method tuning** | *"To ensure a fair comparison, we employed BO to tune the hyperparameters of all parametric models under consistent conditions, allowing each to operate near its best achievable performance"* |
| Matched capacity for the offline baseline | *"For fair comparisons, we implemented a 5-layer cascaded FNN, mirroring the network complexity of the ESN"* |
| Conclusion is trade-offs, not a winner | GP most accurate but cubic training cost; ELM/RBFNN fast but initialisation-sensitive; ESN *"the most balanced trade-off"* |
| **A learned residual crashed the vehicle** | *"the online-learning-based controller with RBFNN encountered instability and crashes during the simulation due to significant prediction errors"*, attributed to random initialisation *"which can produce excessively large prediction values"* |

### ⚠️ The abstract is misleading about hardware, and we repeated the error

The abstract says *"Through extensive simulation and hardware experiments"*. **No real flight took
place.** Every result is simulation: MATLAB-generated datasets, *"50 simulation campaigns"*, *"50
sec of simulated flight data"*. The *"hardware experiments"* are the learning modules running on an
NVIDIA Jetson Orin NX **in the loop with the gym-pybullet-drones simulator** — a compute-latency
benchmark, not a flight. The conclusion is explicit: *"We also aim to validate the approach
experimentally on a real quadrotor platform."*

Our first note on this paper, written from the abstract, said "in simulation **and** hardware" and
implied real flight. Corrected in Chapter 2. **Third time in this project that reading the source
corrected an abstract-level claim** — see [`17`](17_Source_Ledger_and_Citation_Discipline.md) §1.

### What this changes for us

**1. Their fairness choice is the opposite of ours, and that is worth saying.** They tune every
method to its own optimum via Bayesian optimisation; we freeze shared gains. Both answer the same
confound — that a difference might reflect tuning effort rather than method — and neither is
free of criticism. Chapter 4 §4.9 previously presented per-method tuning as the alternative *we
rejected*; it can now cite a real study that chose it, which makes the discussion a genuine fork
rather than a straw man.

**2. Our comparison is on real hardware; theirs is not.** A legitimate and concrete differentiator,
now stated in Chapter 2 rather than implied.

> ### ⚠️ Corrections, 27 Aug
>
> **Keep the "not real flight" reading.** The abstract's *"hardware experiments"* still must not be
> upgraded. The body's hardware is a **Jetson Orin NX 16 GB running gym-pybullet-drones on the same
> device**, timing wall-clock cost against a **5 ms / 200 Hz** budget (§5.5). Real-quadrotor
> validation is **future work (§6)**.
>
> | Was | Is |
> |---|---|
> | "Their residual is defined **exactly as ours is**" | They treat uncertainty as a **lumped additive wrench \((f_a, \tau_a)\)** recovered by **inverting the discrete Newton–Euler dynamics**. **Analogous** to our \(a_\mathrm{res}\), **not identical a priori** |
> | "They tune **every method** by Bayesian optimisation" | BO was applied to the **parametric models only — ELM, RBFNN, ESN**. The Gaussian process is not among them. The paper's own words are *"all parametric models"* |
> | Crash sentence | Must stay **simulation-only** and keep **"may be attributed"**. Body facts: **RBFNN crashed in simulation under periodic wind**; **ELM went unstable in a simulated landing under ground effect** (*"not detailed here"*); **ESN and GP did not**. Cause: random initialisation of the **input-to-hidden** weights, which *"can produce excessively large prediction values"* |
> | — | **Do not blend with Neural-Swarm, INDI or multi-robot downwash.** The disturbances here are **ground effect and wind on a single vehicle** |

**3. It justifies our output clamp.** The `OUT_CLAMP = 8` m/s² ceiling in `residual_nn.rs` was
added defensively. This paper documents the exact failure it guards against: a learned residual
producing excessively large predictions and crashing the vehicle. That is a citable justification
for a design decision we had only reasoned our way to.

**4. Its conclusion shape is the one to aim for.** Trade-offs with conditions attached, not a
winner. Note also its framing — *"We demonstrate the superiority of online learning"* — sets out to
demonstrate rather than to test. Our H1 is deliberately written so that a null result is
informative; that contrast is worth keeping.

---

## Lee, Leok & McClamroch — geometric tracking on SE(3)
**Read: deep (equations).** Primary citation: **CDC 2010**, *Geometric tracking control of a
quadrotor UAV on SE(3)*, pp. 5420–5425, DOI 10.1109/cdc.2010.5717652 — metadata verified via
Crossref. Equation-level verification was performed against the companion arXiv paper
**arXiv:1003.2005**, `papers/lee2010geometricse3.pdf`, which we hold.

| Claim | Evidence |
|---|---|
| Our attitude error is theirs | eq. (8): *"eR = ½ (Rdᵀ R − Rᵀ Rd)∨"* — matches `vee_half(...)` in `lib.rs` |
| Our torque law is theirs | eq. (11): *"M = −kR eR − kΩ eΩ + Ω × JΩ"* — matches the geometric torque in `lib.rs` |

> **Why two entries.** The CDC 2010 paper is the canonical citation and is used as the single
> primary reference in Chapters 2 and 4. We do not hold its PDF; we hold the companion arXiv paper,
> which is where the equations above were checked character by character. The bibliography records
> this split in both entries' `note` fields, so no claim rests on an unread document.

---

## Li, Han, Yu, Lin, Li & Ren — *NDP-NMPC* (2023), arXiv:2304.07794
**Read:** full text (2026-08-24). Found in the W.10 audit. All three claims confirmed in the body.

| Claim | Evidence |
|---|---|
| Neural downwash predictor coupled to NMPC | *"an approach that integrates a Neural network Downwash Predictor with Nonlinear Model Predictive Control (NDP-NMPC)"* |
| Trained with spectral normalisation | *"The neural network is trained with spectral normalization to ensure robustness"* |
| Prediction from neighbours' states enables proactive handling | *"the existence of the downwash disturbance can be predicted based on the states of neighboring quadrotors. If this prediction is considered, the control loop can proactively handle the disturbance"* |

⚠️ Abstract-level only. Claim nothing about their experiments or results.

---

## Shankar, Woo & Prorok — *Docking Multirotors using Learnt Downwash Models* (2023), arXiv:2311.13988
**Read:** full text (2026-08-24). Found in the W.10 audit. All three claims confirmed in the body.

| Claim | Evidence |
|---|---|
| Learned downwash inside an optimal feedback controller | *"we employ a learnt downwash model online within an optimal feedback controller to accurately track a docking maneuver and then hold formation"* |
| Leader--follower vertical docking, 1--2 body-lengths | *"certain missions require two multirotors to approach each other within 1-2 body-lengths … vertically docking two multirotors in the air"* |
| The follower takes the interference | *"the follower experiences significant downwash interference from the leader in its final docking stages"* |

⚠️ Abstract-level only. The framing of docking as a regime where compensation becomes a
precondition is **our reading**, not a claim of theirs, and Chapter 2 marks it as such.

---

## Consequences for our own documents

### 1. RQ3 must be reframed — superposition is already known to fail

Neural-Swarm (2020) states outright that the effect *"is not a simple superposition of each pair"*,
and Gielis et al. (2023) exists specifically to model aggregation because single-vehicle models
*"unlikely to be sufficient"*. Our RQ3 as originally written — *"do pairwise interactions
superpose?"* — would be asking a question the literature has already answered.

**The defensible version** is narrower and still worth asking: *how much accuracy does our
2-robot-trained deep-sets model lose on 3-robot formations, and is the loss small enough that the
simpler architecture remains the right engineering choice?* That is a quantitative question about
**our** model on **our** platform, not a claim to discover non-superposition.

Corrected in [`15`](15_Problem_Statement_and_Research_Questions.md) and
[`19`](19_Contribution_Statement.md).

### 2. Two more errors in `02_Literature_Matrix.md`

| Entry | Our matrix | Verified |
|---|---|---|
| Neural-Swarm formations | "2–5 Crazyflies, **~30 cm** vertical" | **0.20–0.25 m** in the swapping task (25 cm explicit for three vehicles); **trained** on 2/3/4, **tested** on 2–5 |
| Aggregate Downwash compute | "Usable onboard" | Not supported by anything we have read |

#### `02_Literature_Matrix.md` — five rows corrected 26 Aug ✅

The Smith and Hsieh 2026 rows were brought in line with this ledger: Smith now reads **36 % 3D /
56 % vertical on average** with the "lateral" reading marked as the paper's own inconsistency, with
evaluation at **0.6–0.8 m** (0.5 m being the closest *training* stage) and the platform flagged as
**not a Crazyflie**; Hsieh 2026 now marks **31 % as abstract-only** against body figures of
24 %/30 % and 29 %/43 %, the **20× as simulation** rather than a same-rig comparison with Chee, and
hardware tracking at **Δz = 0.3 and 0.4 m**.

The **Neural-Swarm** and **Gielis** rows were brought in line on the same day: Neural-Swarm now reads
**trained 2–4 / tested 2–5** at **0.20–0.25 m**, with the 2.4 cm figure attributed to the
**4-vehicle-trained** net (2.7 cm for the 2-trained one), **f_a,z only**, onboard the STM32, and
generalisation marked **directional**; Gielis now reads **Deep Sets (sum-pool)** with "graph"
marked as the abstract's language, the sufferer **on a load stand and not in free flight**, and
**4 total = 1 constrained sufferer + ≤3 free flyers (K ≤ 3)** with the fourth-vehicle
generalisation marked abstract-level.

The **Hsieh 2025** row was brought in line on 27 Aug: **three** CF2.1, **V-stack and I-stack** with
the separation sweep, **hardware-only and off-board** with the two host machines, **only the
\(\mathcal{L}_1\) estimate adapting online** while KNODE-DW reuses Chee's data, and an explicit
note that **the 20× figure is not in this paper**.

**All eight papers from the 26–27 Aug passes are now consistent between the chapters, this ledger
and `02`.**

> **`02` remains a working note, not a source. Do not cite it.** Three of the misattributions in
> [`17`](17_Source_Ledger_and_Citation_Discipline.md) §1 came from treating our own summary files
> as sources — this ledger is the record, and where the two disagree, this file wins.

### 3. Bars our own work is measured against

Worth stating plainly in Chapter 2, because they are stronger than we had assumed:

- Flatness-preserving residual: **31 %** error reduction *(abstract; body gives 24 %/30 % and a
  further 29 %/43 %)*, **NMPC-matching at ~10× less compute** *(simulation)*, **< 30 s** of
  training data *(28 s hardware + 18 s sim)*, **5 ms** loop *(abstract, off-board)*.
- SO(2)-equivariant: **36 %** 3D / **56 %** vertical on two vehicles, from **5 minutes** of data
  *(a validation-error result, not closed-loop tracking)*.

Each still sets a real bar, but every one of them is weaker as an *onboard, hardware, closed-loop*
claim than the bare number suggests. §2.8 should not imply the field is short of strong results —
it is short of *comparable* ones — but nor should it quote these as if they were all measured the
way ours will be.

Our contribution is the controlled comparison, not beating these numbers — but §2.8 must not
imply the field is short of strong results. It is short of *comparable* ones.
