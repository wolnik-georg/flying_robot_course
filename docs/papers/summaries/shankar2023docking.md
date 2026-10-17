# Docking + learnt downwash — Shankar et al., 2023

## 0. Identity

- Full title: Docking Multirotors in Close Proximity using Learnt Downwash Models
- Authors: Ajay Shankar, Heedo Woo, Amanda Prorok. Affiliation on this file: Department of Computer Science & Technology, University of Cambridge, UK.
- Year / venue / arXiv / DOI: arXiv footer on this file is `arXiv:2311.13988v1 [cs.RO] 23 Nov 2023`. Conference or journal name is **not in this paper**. DOI is **not in this paper**.
- Version of THIS file: arXiv v1, 23 Nov 2023. Keywords printed: Multirotors, Aerial Docking, Geometric Learning.
- Suggested cite key: `shankar2023docking`
- What this paper IS: a two-quad hardware paper that puts a learnt lumped downwash force into an onboard LQR so a follower can track a short vertical docking path under a leader and, in hover, physically clasp a suspended bar.
- What this paper is NOT: not the SO(2) learning paper (they cite that as prior work [4]); not a swarm paper; not Crazyflie; not INDI / NA-INDI / L-INDI; not MPC; not outdoor GNSS docking (they only say the architecture could be moved).

## 1. Problem they pose

Their wording (Motivation + Problem Statement): unmodeled aerodynamic disturbances are a key challenge in close proximity. Some missions "require two multirotors to approach each other within 1-2 body-lengths of each other and hold formation." They take vertical in-air docking as that instance. The follower sees "significant downwash interference from the leader in its final docking stages." A downwash-aware planner can be avoided in other missions; here close formation is required, so the disturbance cannot be planned around.

Problem statement (body): leader Alpha flies a predetermined path and tows a suspended docking platform at vertical distance $d_p$. Follower Bravo has a gripper that docks when within tolerance $\varepsilon_g$ of the platform. Objective: use an accurate model of $\vec{f}_{\mathrm{ext}}$ online so $\|\vec{p}^B - \vec{p}^B_{\mathrm{des}}\| \le \varepsilon_g$ for all $t$ in a horizon $t_h$ when $\|\vec{p}^A(t)-\vec{p}^B(t)\| < D$.

Plain restatement: get the follower close enough under the leader to grab a hanging bar, and keep position error inside the gripper window while the leader's wake is strong.

## 2. Paper map (follow their TOC)

### 1 Motivation

- Purpose: why close docking is useful, why downwash blocks it, state the two-vehicle problem, survey related docking and downwash papers.
- Method / construction: none yet. Use-case they name: a larger leader tows smaller followers with targeted sensors. Prior docking they cite used long cables (~4 m) or approach from above.
- Key equation: none numbered in this section.
- Assumption: missions that need 1–2 body-length vertical proximity cannot dodge the wake. Long cables add swing risk and hurt leader agility.
- Result claimed: positioning only. They say their prior SO(2) formulation [4] gave "dramatic boosts in training efficiency." That boost is not re-measured here.

### 2 Technical Approach

- Purpose: write the follower plant, put $\vec{f}_{\mathrm{ext}}$ into it, recall the learnt map from [4], describe curriculum data, describe cable + gripper + planner.
- Method / construction: Bravo is a 6-DoF rigid body of mass $m$. Inner loop holds commanded rotation. Outer loop is a feedforward-linearised 7-state model. Control is infinite-horizon LQR. Downwash is a lumped force on the linearised plant. Labels for learning are acceleration residuals from nominal flight. Network input is a 6-D SO(2) feature map of a 9-D state. Data collected in height stages. Planner is a smooth parametric path of horizon $t_h$ aimed at a predicted future platform pose.
- Key equations:
  - Unnumbered rigid-body force: $m\vec{a} = -R_E^B \cdot T + mg\,\hat{e}_3$.
  - (1) $\dot{x} = Ax + Bu$, $y = Cx$, with $x \equiv [\vec{p},\vec{v},\psi]^\top$, $u \equiv [a_n,a_e,a_d,\dot{\psi}]^\top \equiv [\vec{a},\dot{\psi}]^\top$. Then $u_{\mathrm{ap}} = [R_E^B, T]^\top$ by nonlinear inversion. Feedback: $u = -K_{\mathrm{lqr}} x$.
  - Disturbed plant (unnumbered): $\dot{x} = Ax + Bu + \bar{B}\,\vec{f}_{\mathrm{ext}}$, with $B = \begin{bmatrix}\bar{B} & 0 \\ 0 & 1\end{bmatrix}$. Yaw torques ignored. Moments folded into lateral effects.
  - Labels (unnumbered): $\hat{\vec{f}}_{\mathrm{ext}}(t)$ from $\vec{a}_{\mathrm{obs}} - u_{[1:3]}$.
  - State for the map: $\mathbf{x} \equiv [\Delta\vec{p},\,\vec{v}^A,\,\vec{v}^B]$ in $\mathbb{R}^9$. Naive net $\vec{f}_{\mathrm{ext}}:\mathbb{R}^9\to\mathbb{R}^3$.
  - (2) feature map $h:\mathbb{R}^9\to\mathbb{R}^6$ (SO(2) invariant part). Angle $\varphi(\mathbf{x})$ between $\mathrm{proj}_A(\Delta\vec{p})$ and the $+a_1$ axis in $\mathcal{S}_A$ is used for equivariance. $\mathcal{S}_A = \mathrm{span}\{a_1,a_2\}$.
- Assumption: commanded vs observed acceleration differs by "constant biases and offsets that exist with/without $\vec{f}_{\mathrm{ext}}$." Translation invariance via $\Delta\vec{p}$. Absolute velocities kept so hover-together is not confused with matched motion. Model from [4] is reused, not re-derived.
- Result claimed in this section: method only. "merely 5 mins of real-world data" is stated here as the training budget for that learnt force.

### 3 Experiments and Results

- Purpose: deploy the model online on docking tasks. They send learning-pipeline metrics back to [4]. This section is deployment and clasp success.
- Method / construction: see 3.1–3.3.
- Key equation: none new.
- Assumption: "docking with Alpha presents a make-or-break scenario, where insufficient accuracy in tracking causes a direct mission failure."
- Result claimed: split below by subsection. Do not flatten them into one number.

### 3.1 Technical Setup

- Purpose: name the vehicles, gripper, computers, and sensing.
- Method / construction: two in-house quads. Servo rotary-gate gripper with a strain-gauge trigger. Wheatstone + HX711 into a Raspberry Pi 4B. Resting-bias calibration at the start of each flight. Flight stack: customised ArduCopter inner firmware; Freyja over ROS2 for high-level control and planning. Indoor mocap gives each vehicle its own position. Vehicles talk over an infrastructure network. No centralised controller.
- Key equation: none.
- Assumption: architecture "easily transported to the field" with GNSS. That transport is not demonstrated in this file.
- Result claimed: setup only. Printed sizes: span "≈ 0.27 m on the longest diagonal"; stock mass "≈ 0.7 kg (including batteries)." Strain patch: 6.35 mm, 350 Ω, ±5 % strain range.

### 3.2 Studying Effects of Cable Length

- Purpose: measure docking with and without the model at six cable lengths, with Alpha fixed on a stand and commanded hover thrust.
- Method / construction: platform hung under fixed Alpha. Offsets sampled "between one and five body-lengths." Bravo starts on the same lateral station. Missions repeated with and without the model. They call the setup realistic for Bravo but ideal for Alpha (no leader position error).
- Key equation: table statistics are "integrated over the final 0.5 s," written $\int_{t-0.5}^{t}(\cdot)\,dt$.
- Assumption: longer cables swing more and give weaker contact force (more false negatives). Shorter cables are stiffer and safer for Alpha if Bravo can hold station.
- Result claimed (body + Fig. 2 + Table 1): without the model, **Miss** at every offset 0.36, 0.47, 0.64, 0.87, 1.11, 1.35 m. With the model, **Dock** at every listed offset. Predicted force with model at 0.36 m is 6.13 m/s² over that 0.5 s integral; it "trails off quite sharply" at larger offsets. They write that even at 5 body-lengths Bravo still sees a "noticeable disturbance." Vertical error "plateaus in both cases after about 3 body-lengths." Largest with-model positioning error is still smaller than the smallest without-model error. Fig. 2 left: Alpha drawn at altitude 2.5 m. Fig. 2 right: combined vertical positioning error, one standard deviation shaded. Docking "right before the 4 s mark."

### 3.3 Docking Mid-Air

- Purpose: Alpha free-flying in hover. Then a separate moving-Alpha test.
- Method / construction: shortest offset from Table 1, $d_p \approx 0.36$ m, "a little over one body-length." $\varepsilon_g = 0.1$ m (gripper aperture). Bravo starts from several poses, plans a ≈ 4 s parametric path to a predicted platform pose. Ten evaluations from four start locations. Airspace assumed empty. Later: Alpha commanded at a fixed 0.25 m/s horizontal speed. Bravo meets the target at 6 s and holds 5 s.
- Key equation: none new.
- Assumption: downwash model "trained to handle cases where Alpha is in near-hover." Moving-Alpha is an extra probe, not the training regime.
- Result claimed:
  - Hover leader: "successful in effecting a final dock repeatably in **8 out of 10** instances."
  - Average vertical tracking error in the final docking period "typically less than **0.1 m**."
  - After a successful clasp, Bravo cuts collective and Alpha carries extra weight, so Alpha dips. That dip shows up as extra vertical error for Bravo. Lateral error falls after clasp.
  - Fig. 4: success = both vehicles lose altitude after dock; miss = only Bravo loses altitude.
  - Moving Alpha (Fig. 5): without model, mean error after entering the wake (4–6 s) is "over **0.2 m**, nearly 1 body-length." With model, error "reduced to less than **0.05 m**, and is nearly zero at 6 s." Physical clasp in this moving test: "very limited success," blamed on Bravo oscillation and gripper timing (see §4).

### 4 Discussion and Conclusion

- Purpose: explain missed clasps, comment on LQG observer lag, state the claim of first close-vertical dock, list future work.
- Method / construction: none new. Failure analysis of the gate.
- Key equation: none.
- Assumption / limit they state: contact sensor has finite response time. "digital latency of approximately **15 ms**" to recognise contact. Filtering, IPC, and servo torque-curve add more delay. In a miss, contact is seen, servo starts, bar "slips through" before the gate shuts. Observer can be tuned faster only at the cost of "robustness in regular undisturbed flight." Downwash "changes so rapidly that a convergence is not guaranteed."
- Result claimed: "first demonstrations of two multirotors docking in such close vertical proximity" (body §4). Future work: models for rapidly moving Alpha, or a different geometry.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Two in-house quads. Longest diagonal ≈ 0.27 m. Stock mass ≈ 0.7 kg with batteries. Names: Alpha (leader), Bravo (follower). No commercial airframe name. |
| n robots min/max | 2 and 2. No third vehicle in this file. |
| Sim / hardware / both | Hardware only in the reported docking tests. Simulation is not in this paper. |
| Onboard vs off-board (Hz) | Estimation, LQR, planning, model query, and comms: onboard Raspberry Pi 4B. Loop rates in Hz: **not in this paper**. Inner loop: customised ArduCopter. High level: Freyja + ROS2. |
| Sensing (mocap, RPM, GPS, …) | Indoor motion-capture position to each vehicle separately. Strain gauge on the gripper arm. RPM sensing: **not in this paper**. GPS flights: **not in this paper**. |
| Controller family | Feedforward-linearised outer model + infinite-horizon LQR. Nonlinear inversion to $[R_E^B, T]$. Inner attitude by thrust differentials. Discussion later calls the closed loop an LQG observer artefact. Not MPC. Not INDI. |
| How interaction enters the law | Lumped force $\vec{f}_{\mathrm{ext}}\in\mathbb{R}^3$ added to the linearised plant as $\bar{B}\vec{f}_{\mathrm{ext}}$. Network prediction is the compensation the LQR sees. |
| Learned model? inputs → output | Yes. Raw state $\mathbf{x}=[\Delta\vec{p},\vec{v}^A,\vec{v}^B]\in\mathbb{R}^9$. Features $h(\mathbf{x})\in\mathbb{R}^6$ plus angle $\varphi(\mathbf{x})$. Output $\vec{f}_{\mathrm{ext}}\in\mathbb{R}^3$. Architecture details live in cited [4], not in this file. |
| Training data amount / source | This file says "merely 5 mins of real-world data" and points to [4]. Labels: $\vec{a}_{\mathrm{obs}}-u_{[1:3]}$ from nominal-controller flights at staged separations. Curriculum: 1.8–1.5 m down in 0.3 m steps to ≈ 0.5 m. |
| Formations + min separation | Leader–follower, follower below. Free-flight dock at $d_p\approx 0.36$ m. Stand-test offsets 0.36–1.35 m. Curriculum floor ≈ 0.5 m ("roughly 2 body-lengths"). |
| Speeds | Hover-dock path ≈ 4 s long. Moving-Alpha test: 0.25 m/s commanded horizontal. Other speeds: **not in this paper**. |
| Metrics | Position error (vertical and 3D), predicted force (m/s²), Dock vs Miss, 8/10 clasp count. Table 1 uses 0.5 s integrals, not raw mean error. |
| Baseline they compare to | Same stack **without** the learnt model. No INDI, no geometric SE(3) paper, no Neural-Swarm stack compared here. |

## 4. Formations and flights (table, one row per experiment)

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Stand + cable sweep, no model | 2 | Bravo under fixed Alpha | 0.36, 0.47, 0.64, 0.87, 1.11, 1.35 m | docking approach, ~4 s; terminal stats last 0.5 s | hw, Alpha on a stand | Miss at every offset. Table 1 Error Down 1.67–2.28 (integral). |
| Stand + cable sweep, with model | 2 | same | same six offsets | same | hw, Alpha on a stand | Dock at every offset. Predicted force 6.13 m/s² at 0.36 m, then smaller. Fig. 2. |
| Mid-air hover dock | 2 | Bravo under free-flying hovering Alpha | $d_p=0.36$ m, $\varepsilon_g=0.1$ m | ≈ 4 s planned path; 10 runs, 4 start poses | hw, mocap | 8/10 physical docks. Terminal vertical error typically < 0.1 m. Fig. 3, Fig. 4. |
| Alpha in motion | 2 | same vertical stack | same 0.36 m target alignment | Alpha 0.25 m/s horizontal; meet at 6 s, hold 5 s | hw, mocap | With model: error < 0.05 m, near 0 at 6 s. Without: mean error > 0.2 m in 4–6 s. Physical clasp: "very limited success." Fig. 5. |

No other formations in this file. No 3-robot tests. No side-by-side or above-leader docks in the experiments.

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where (abstract/body/fig) | Caveat |
|---|---|---|---|---|
| 1–2 body-lengths | "1-2 body-lengths" | stated required proximity for the mission class | abstract and Motivation | Not a measured metre by itself. |
| Vehicle size | ≈ 0.27 m | longest diagonal ("body-length" unit they use) | body §3.1 | Not rotor-to-rotor in so many words. |
| Vehicle mass | ≈ 0.7 kg | stock mass including batteries | body §3.1 | Gripper mass split is not in this paper. |
| Tracking error headline | "less than 0.06 m" and "3-4x reduction" | follower tracking error when approaching vertically within two body-lengths | **abstract only** | Body hover-dock text says typically < 0.1 m. Body moving-Alpha text says < 0.05 m vs > 0.2 m. Abstract does not retract. Do not pick a winner. |
| Hover-dock terminal error | "typically less than 0.1 m" | average vertical tracking error in the final docking period | body §3.3 | After clasp, Alpha dips. That inflates Bravo's apparent vertical error. |
| Moving-Alpha errors | < 0.05 m with model; > 0.2 m without | mean altitude error after Bravo enters the wake (4–6 s), 10 runs | body §3.3 + Fig. 5 | Positioning result, not clasp result. |
| Free-flight cable | $d_p \approx 0.36$ m | platform hang distance used for mid-air tests | body §3.3 | "a little over one body-length." |
| Gripper window | $\varepsilon_g = 0.1$ m | gate aperture / dock tolerance | body §3.3 | Same 0.1 m also appears as typical hover vertical error. Do not fuse them. |
| Clasp count | 8 out of 10 | physical docks, hovering Alpha, $d_p=0.36$ m | body §3.3 | Not the moving-Alpha test. |
| Moving-Alpha speed | 0.25 m/s | commanded horizontal speed of Alpha | body §3.3 | Not a demonstrated clasp speed. |
| Training time | "merely 5 mins" | real-world data for the learnt force, via [4] | body §2 | Data volume of *this* docking campaign is not separately stated. |
| Curriculum start / step / floor | 1.8–1.5 m; 0.3 m steps; ≈ 0.5 m | staged data collection heights | body §2 | 0.5 m is "roughly 2 body-lengths of Alpha." |
| Contact latency | "approximately 15 ms" | digital latency to recognise contact in software | body §4 | Not the full actuation delay. Servo + filter + IPC extra, unquantified. |
| Stand-test table integrals | e.g. 1.06 m "Error Down" with model at 0.36 m | $\int_{t-0.5}^{t}$ of the listed quantity | Table 1 caption + §3.2 | Units printed as [m] but the caption says time integral. Do not quote as mean tracking error next to 0.06 m. |
| Predicted force at closest stand offset | 6.13 m/s² | table integral, with model, offset 0.36 m | Table 1 | Not a pointwise peak. |

## 6. Assumptions and failure modes

- Two vehicles only. Follower below the leader.
- Downwash on Bravo is a lumped $\vec{f}_{\mathrm{ext}}\in\mathbb{R}^3$. Yaw torque from the wake is ignored. Moments are folded into lateral acceleration.
- Labels treat $\vec{a}_{\mathrm{obs}}-u_{[1:3]}$ as the force, up to constant bias.
- Learnt map uses $\Delta\vec{p}$ plus both absolute velocities. Training regime is near-hover Alpha.
- SO(2) symmetry about the leader's vertical axis is assumed useful (inherited from [4]).
- Inner attitude loop is fast enough that the 7-state linear model is the right outer plant.
- Cable length $d_p$ is known. Platform may swing and yaw within cable limits.
- Gripper needs contact force and enough time for the gate. Miss mode: detect, actuate, still lose the bar.
- Indoor mocap. Empty airspace. No obstacles.
- Moving-Alpha at 0.25 m/s is outside the stated training distribution.
- LQG/LQR observer lag: they say faster observer tuning hurts undisturbed flight, and the wake changes too fast for guaranteed convergence without the model.
- Field/GNSS use is a claim, not a test in this file.

## 7. Relation to our thesis (clearly labelled OUR READING)

- Which of our strategies it supports:
  - Closest to **3 geo+NN residual** in spirit: a learned force/acceleration is added inside a model-based feedback law.
  - Also adjacent to **4 FBL+residual**: they linearise first, then add $\bar{B}\vec{f}_{\mathrm{ext}}$. That is not our FBL, and they never say "feedback linearisation residual" in those words.
  - Does **not** support **2 full INDI** or **5 hybrid NA-INDI/L-INDI**. Those words are not in this file.
  - Does **not** support a pure **1 geometric baseline** paper. The geometric object here is the SO(2) feature map from [4], not a geometric SE(3) tracking law.
- What we may cite:
  - Hardware fact: two small quads, follower under leader, ~1–2 body-lengths, learned force inside onboard LQR.
  - Qualitative result: without the model they never clasp in the stand sweep; with it they clasp at every listed offset.
  - 8/10 hover docks at $d_p=0.36$ m.
  - Author language that close tracking is "pass/fail" / "make-or-break."
  - Limit: moving leader at 0.25 m/s gives good height tracking with the model but poor clasp.
- What we must NOT cite from it:
  - Crazyflie, 3-robot teams, OptiTrack-by-name, STM32, INDI, frozen-gain campaign vs geometric/INDI/hybrid.
  - Smith et al. percentages (36 %, 56 %) or the full SO(2) training study. Those live in [4], not in this file.
  - Abstract 0.06 m as if the hover-dock body also said 0.06 m.
  - Outdoor docking.
  - "Precondition for the manoeuvre" or "closest to Chapter 5." Those sentences are ours.
- Formations we should reproduce if we use it as a reference condition:
  - Two robots, follower directly below, commanded vertical offset near one body-length of *their* 0.27 m frame (0.36 m here).
  - A hover-leader approach of a few seconds, then a slow translate-and-hold if we want their Fig. 5 comparison.
  - Report Dock/Miss separately from centimetre tracking error.
  - Do not treat their hanging-bar clasp as the same task as our interaction-force tracking on a free team.
- Compute: would this run on an STM32?
  - They ran the model + LQR + planner on a **Raspberry Pi 4B**, not an STM32. Network size, FLOPs, and control Hz are **not in this paper**. Pi 4B is a different class than an STM32 flight MCU. Inner loop is ArduCopter on the flight controller; that split is the closest analogue to "MCU + companion." Whether the net fits a Crazyflie brushless MCU is not answered by this file.

## 8. Equations worth keeping

Symbols in one line each: $m$ follower mass; $T$ collective thrust; $R_E^B$ rotation inertial $\to$ body as printed; $\hat{e}_3$ inertial up; $x=[\vec{p},\vec{v},\psi]^\top$; $u=[\vec{a},\dot{\psi}]^\top$; $\vec{f}_{\mathrm{ext}}$ lumped wake force; $\Delta\vec{p}=\vec{p}^B-\vec{p}^A$ as used in the 9-D state; $\mathcal{S}_A=\mathrm{span}\{a_1,a_2\}$ leader horizontal plane.

1. Rigid-body force (unnumbered, §2):

$$
m\vec{a} = -R_E^B \cdot T + mg\,\hat{e}_3.
$$

2. Linearised plant (1):

$$
\dot{x}=Ax+Bu,\quad y=Cx.
$$

3. LQR (unnumbered, §2):

$$
u=-K_{\mathrm{lqr}}x.
$$

4. Disturbed plant (unnumbered, §2):

$$
\dot{x}=Ax+Bu+\bar{B}\,\vec{f}_{\mathrm{ext}}.
$$

5. Residual label (unnumbered, §2):

$$
\hat{\vec{f}}_{\mathrm{ext}} \leftarrow \vec{a}_{\mathrm{obs}}-u_{[1:3]}.
$$

6. Learning state (unnumbered, §2):

$$
\mathbf{x}\equiv[\Delta\vec{p},\,\vec{v}^A,\,\vec{v}^B]\in\mathbb{R}^9.
$$

7. Feature map (2), as printed:

$$
h(\mathbf{x})=\Bigl(
\frac{\mathrm{proj}_{\mathcal{S}_A}(\Delta\vec{p})^\top\mathrm{proj}_{\mathcal{S}_A}(\vec{v}^B)}{\|\mathrm{proj}_{\mathcal{S}_A}(\Delta\vec{p})\|_2\|\mathrm{proj}_{\mathcal{S}_A}(\vec{v}^B)\|_2},
\|\mathrm{proj}_{\mathcal{S}_A}(\Delta\vec{p})\|_2,
\|\mathrm{proj}_{\mathcal{S}_A}(\vec{v}^B)\|_2,
[R_E^A\Delta\vec{p}]_3,
[R_E^A\vec{v}^B]_3,
\|\mathrm{proj}_{\mathcal{S}_A}(\vec{v}^A)\|_2
\Bigr).
$$

8. Dock specification (Problem Statement):

$$
\|\vec{p}^B-\vec{p}^B_{\mathrm{des}}\|\le\varepsilon_g \text{ while } \|\vec{p}^A-\vec{p}^B\|<D, \text{ for } t\in t_h.
$$

## 9. Figures / tables worth remembering

- Fig. 1: two-vehicle frames; stills of the docked pair; CAD of the servo gate on top of one vehicle (highlighted red).
- Fig. 2 left: vertical trajectories at several cable offsets. Alpha drawn at 2.5 m. With-model vs without-model.
- Fig. 2 right: combined vertical position error vs time, mean and 1σ. Dock just before 4 s.
- Table 1: six offsets × (without model | with model). Columns: Error Down, Error 3D, Predicted Force, Result. Without: all Miss. With: all Dock. Caption: integrals over the last 0.5 s.
- Fig. 3a: plan/3D paths of 10 hover-dock runs. Alpha hover uncertainty as a shaded ellipsoid. Docking platform marked.
- Fig. 3b: error vs time and a plan view with predicted-force arrows. Grey band = likely clasp window.
- Fig. 4: altitude of Alpha and Bravo on a successful dock vs a miss. Success: both drop after clasp. Miss: only Bravo drops.
- Fig. 5: Bravo altitude tracking with Alpha moving. With model vs without vs reference, ~0–10 s.

## 10. Open uncertainties

- Control and estimator rates (Hz): not in this paper.
- LQR weights $Q,R$ and gain $K_{\mathrm{lqr}}$: not in this paper.
- Network width, depth, activations, inference time: not in this paper (pointed to [4]).
- Exact definition of "body-length" beyond "≈ 0.27 m on the longest diagonal."
- How many flights sit behind the "5 mins" figure in *this* campaign vs [4].
- Cable mass, stiffness number, bar diameter, exact $d_p$ construction tolerance.
- Filter cutoffs on mocap, strain, or the force prediction.
- Whether $\vec{f}_{\mathrm{ext}}$ is in newtons or specific force. Table 1 header says m/s² for "Predicted Force."
- How the 0.5 s integral in Table 1 was normalised, given the [m] column header.
- GNSS/outdoor docking: not tested.
- Exact count of moving-Alpha clasp successes: only "very limited success."
- Bravo thrust reduction law after clasp: stated qualitatively, not as a number.

## 11. 12-line abstract I can reread

Shankar, Woo, Prorok, arXiv:2311.13988v1 (23 Nov 2023). Two in-house quads, ≈ 0.27 m diagonal, ≈ 0.7 kg. Leader Alpha tows a bar on cables. Follower Bravo clasps it from below. They add a learnt lumped force $\vec{f}_{\mathrm{ext}}$ to an onboard LQR on a Pi 4B. Model and features come from their prior SO(2) paper [4]; this file is the docking deployment. Training budget stated here: 5 min of residual labels, curriculum from ~1.8 m down to ~0.5 m. Stand tests at 0.36–1.35 m: without model all Miss, with model all Dock. Free-flight hover dock at $d_p=0.36$ m: 8/10 clasps, terminal vertical error typically < 0.1 m. Abstract separately says < 0.06 m and a 3–4× cut within two body-lengths. Moving Alpha at 0.25 m/s: height error < 0.05 m with the model vs > 0.2 m without; clasp still mostly fails on gate timing. Indoor mocap. Not Crazyflie, not INDI, not three robots.

Source: attached PDF only. No other paper used.
