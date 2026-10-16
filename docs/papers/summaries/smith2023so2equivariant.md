# SO(2) Equivariant Downwash — Smith et al., 2023

## 0. Identity

- Full title: SO(2)-Equivariant Downwash Models for Close Proximity Flight
- Authors: Henry Smith, Ajay Shankar, Jennifer Gielis, Jan Blumenkamp, Amanda Prorok (Cambridge)
- Year / venue / arXiv / DOI: 2023. This file is arXiv:2305.18983v1 [cs.RO], 25 May 2023. No journal/conference name is printed on the PDF. DOI: not in this paper.
- Version of THIS file: arXiv v1 preprint.
- Suggested cite key: `smith2023so2equivariant`
- What this paper IS: a pairwise, geometry-aware residual force model (SO(2) about the leader vertical axis) trained on real two-vehicle flight and subtracted inside an LQR/LQG loop.
- What this paper is NOT: not Neural-Swarm / Neural-Swarm2, not Crazyflie, not 16-robot, not INDI, not MPC, not RL, not a multi-vehicle aggregation study.

## 1. Problem they pose

Their words: conventional methods "have fallen short of providing adequate 3D force-based models that can be incorporated into robust control paradigms for deploying dense formations." They want a "fast and accurate model of these exogenic forces" for onboard compensation.

Plain restatement: learn the 3-D downwash force one vehicle puts on another, with few minutes of real data, then subtract it in the follower's controller.

## 2. Paper map (follow their TOC)

### I. Introduction

- Purpose: motivate close-proximity downwash and sample-efficient learning.
- Method: learn a geometry-aware force model instead of CFD.
- Key equation: none in this section.
- Assumption: CFD is too expensive for onboard use.
- Result claimed: 5 min beats SOTA trained on more than 15 min (abstract). Online two-vehicle flights: "improves 3D trajectory tracking by nearly 36% on average (and vertical tracking by 56%)" (abstract). First learning-based claim of consistent *lateral-plane* downwash patterns (intro body).

### I-A. Related Work

- Purpose: place against large-separation planners, DNN residuals (they treat [14] Neural Lander / related as SOTA for learned downwash), GPs, RNNs, neural-ODEs, and geometric DL.
- Method: literature only.
- Key equation: none.
- Assumption: they treat the model in [14] as SOTA for learning-based downwash prediction.
- Result: "no previous work that utilizes geometric priors to efficiently learn multirotor downwash forces."

### I-B. Contributions

1. Equivariant model in a lower-dimensional feature space.
2. 5 min data more accurate than SOTA with 15 min.
3. Online in an "optimal feedback controller": "reduce vertical tracking errors by 56 % and lateral tracking errors by 36 %."

- Note: contribution 3 says *lateral* 36 %. Abstract says *3D* 36 %. Body does not retract either line.

### II. Problem Formulation

- Purpose: two-vehicle setup, NED dynamics, LQR plant, residual as additive force.
- Vehicles: identical. Alpha = leader. Bravo = follower in Alpha's downwash.
- Frames: NED, right-handed. $A=\{a_1,a_2,a_3\}$ Alpha body. $\mathcal{M}=\{\hat{e}_1,\hat{e}_2,\hat{e}_3\}$ inertial.
- Dynamics (eq. 1):

$$
m\mathbf{a} = -R_{\mathcal{M}}^{C}\,T + \hat{e}_3 mg
$$

- Feedback-linearized plant (eq. 2): $\dot{\mathbf{x}}=A\mathbf{x}+B\mathbf{u}$, $\mathbf{x}=[\mathbf{p},\mathbf{v},\psi]^\top$, $\mathbf{u}=[\mathbf{a},\dot{\psi}]^\top$.
- LQR: $\mathbf{u}(t)=-K(\mathbf{x}(t)-\mathbf{x}_r(t))$.
- Downwash: additive $\mathbf{f}_{\mathrm{ext}}(\mathbf{x})$, $\mathbf{x}=[\mathbf{p}^A,\mathbf{p}^B,\mathbf{v}^A,\mathbf{v}^B]$.
- Compensated law:

$$
\mathbf{u}_f(t)\equiv -K(\mathbf{x}(t)-\mathbf{x}_r(t))-\mathbf{f}_{\mathrm{ext}}
$$

- Assumption: high gain margin so the residual can be subtracted this way. Narrow-margin case: they mention MPC as an option, not used here.
- Torques from aero are *not* modelled separately. Low-level autopilot takes attitude. Residual is a collective force (mass-normalized acceleration in later labels).
- Problem statement: learn a sample-efficient $\mathbf{f}_{\mathrm{ext}}(\mathbf{x})$ for online compensation.

### III. Establishing Geometric Priors

- Purpose: state invariance / equivariance they will hard-code.
- Def. 1: invariance $f(x)=f(g\star_1 x)$. Equivariance $g\star_2 f(x)=f(g\star_1 x)$.
- Assumption 1, translation invariance: $\mathbf{f}_{\mathrm{ext}}$ depends only on $\Delta\mathbf{p},\mathbf{v}^A,\mathbf{v}^B$. Then $\mathbf{x}\in\mathbb{R}^9$.
- Assumption 2, rotational equivariance about Alpha's body-down $a_3$. Group $H_A\simeq\mathrm{SO}(2)$ (eq. 3). Action: $H\star_1\mathbf{x}=[H\Delta\mathbf{p},\mathbf{v}^A,H\mathbf{v}^B]$, $H\star_2\mathbf{w}=H\mathbf{w}$.
- They say Assumption 2 need not hold for aggressive, highly asymmetric rotor speeds.
- Fallback: one can instead rotate about inertial down and set $R_{\mathcal{M}}^A=I$.

### IV. Geometry-Aware Learning

- Purpose: build an invariant feature map + equivariant reconstruction.
- Feature map $h:\mathbb{R}^9\to\mathbb{R}^6$ (eq. 4): cosine of angle between $\mathrm{proj}_{S_A}(\Delta\mathbf{p})$ and $\mathrm{proj}_{S_A}(\mathbf{v}^B)$, those two norms, the two $a_3$ components of $\Delta\mathbf{p}$ and $\mathbf{v}^B$, and $\|\mathrm{proj}_{S_A}(\mathbf{v}^A)\|_2$.
- Polar angle $\varphi(\mathbf{x})$ of $\mathrm{proj}_{S_A}(\Delta\mathbf{p})$ in the $a_1$-$a_2$ plane.
- Network $f_\Theta:\mathbb{R}^6\to\mathbb{R}^2$. Full model (eq. 5):

$$
F_\Theta(\mathbf{x})=(R_{\mathcal{M}}^A)^\top
\begin{bmatrix}
[f_\Theta(h(\mathbf{x}))]_1\cdot[\cos\varphi(\mathbf{x}),\,\sin\varphi(\mathbf{x})]\\
[f_\Theta(h(\mathbf{x}))]_2
\end{bmatrix}.
$$

- Theorem 1: $F_\Theta$ satisfies Assumption 2. Proof uses parameterization (eq. 6).
- Shallow net (eq. 7): one ReLU hidden layer, $W^{(1)}\in\mathbb{R}^{32\times 6}$, $W^{(2)}\in\mathbb{R}^{2\times 32}$. 256 parameters (Fig. 3 table).
- Labels: $\mathbf{f}_{\mathrm{ext}}=\mathbf{a}-\mathbf{u}(t)$ from (2). MSE on all three inertial axes.
- Assumption in the flown special case of §V: $R_{\mathcal{M}}^A=I$, Alpha near hover, no wind.

### V. Real-World Flight Experiments

- Purpose: data collection, sample-efficiency study, closed-loop tests.
- Sequential stages (similar to [16]): stage-0 vertical sep 1.75 m to 1.35 m with uncompensated LQR. Then deploy stage-$i$ and tighten. Three stages. Stage-2 ≈ 0.5 m, "approx. two body-lengths." Full set "approximately 17 minutes."
- Labels at stage $i$: residual after subtracting stage-$(i-1)$ predictions.
- Baselines: shallow non-equivariant on $[\Delta\mathbf{p},\mathbf{v}^B]\in\mathbb{R}^6$ (321 params); eight-layer spectrally normalized net from [16], [15] (9043 params), singular values bounded by 2.
- Near-hover: $\mathbf{v}^A$ dropped from the non-equivariant input.
- Sample efficiency (Fig. 3, §V-B): with 5 min the equivariant model is more accurate on lateral and downward forces than both baselines with 15 min.
- Fig. 4 viz: Alpha 1 m above Bravo, or 0.1 m east. Bravo velocity $[0.5,0,0]^\top$. Downwash pattern: first push down, then pull up.
- Fig. 5: vertical sep 0.8 m. Lateral pattern: push out, pull in under Alpha, push out again.
- Hardware: two identical custom quads, 0.26 m longest body-diagonal, ≈ 0.7 kg with batteries. Raspberry Pi 4B (8 GB). Freyja LQG at 50 Hz. Network 45 Hz in PyTorch. OptiTrack. Decentralized.
- Stability check: stationary hover of Bravo under Alpha with the model on, to rule out unbounded oscillation.
- Lemniscate: Bravo figure-eight under Alpha hovering at $\hat{e}_3=-2.5$ m.
- Translation: back-and-forth parallel to $\hat{e}_2$, $\hat{e}_1=0.2$ m fixed, $\Delta z=0.6$ m, Alpha at $\mathbf{p}^A=[0,0,-2.5]$.

### VI. Conclusion

- Equivariance about the leader vertical axis. Biggest gain when data is scarce.
- Future they name: outdoor / wind / higher speed; multi-vehicle. They say naive pairwise summation may fail if fields interact nonlinearly.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Custom COTS quadrotor. 0.26 m longest body-diagonal. ≈ 0.7 kg with batteries. Not a Crazyflie. |
| n robots min/max | 2 / 2 |
| Sim / hardware / both | Hardware only in the reported experiments |
| Onboard vs off-board (Hz) | Control, estimation, and NN on the vehicle's Raspberry Pi 4B. Controller 50 Hz. Network 45 Hz. |
| Sensing | OptiTrack. Rotor-speed sensing: not in this paper as a named sensor. Force labels from $\mathbf{a}-\mathbf{u}(t)$. |
| Controller family | Feedback-linearized second-order plant + LQR (text also says LQG / Freyja in §V-C) |
| How interaction enters the law | Predicted $\mathbf{f}_{\mathrm{ext}}$ subtracted from the LQR command: $\mathbf{u}_f=-K(\mathbf{x}-\mathbf{x}_r)-\mathbf{f}_{\mathrm{ext}}$ |
| Learned model? inputs → output | Yes. Invariant $h(\mathbf{x})\in\mathbb{R}^6$ plus polar angle → $f_\Theta\to\mathbb{R}^2$ → rotated 3-D force/accel on Bravo |
| Training data amount / source | Full set ≈ 17 min real two-vehicle flight. Sequential stages. Validation set "roughly equal" in size to full training set. |
| Formations + min separation | Vertical stack, Alpha above Bravo. Training down to ≈ 0.5 m. Eval: 0.8 m (Fig. 5), 0.6 m (translation), Alpha at $z=-2.5$ m (lemniscate). Offset 0.1 m east (Fig. 4), 0.2 m north in translation. |
| Speeds | Force-field plots use $\mathbf{v}^B=[0.5,0,0]^\top$ m/s. Closed-loop peak speed: not in this paper. |
| Metrics | Validation RMSE of force. Position / velocity tracking error. % improvement vs no-model and vs deep non-equivariant. |
| Baseline they compare to | Uncompensated LQR. Shallow non-equivariant NN. Deep 8-layer spectrally normalized NN ([16], [15]). |

## 4. Formations and flights (table, one row per experiment)

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Stage-0 collect | 2 | Alpha above Bravo | 1.75–1.35 m, Δx=Δy≈0 | not in this paper | hw | residual labels $\mathbf{a}-\mathbf{u}$ |
| Stage-2 collect | 2 | same | ≈ 0.5 m (≈ two body-lengths) | not in this paper | hw | closest training sep |
| Force viz down | 2 | stack / 0.1 m east | 1 m; 0.1 m east | Bravo at 0.5 m/s | model eval | Fig. 4 downwash +/− pattern |
| Lateral viz | 2 | Alpha hover at origin | 0.8 m vertical | Bravo ±0.5 m/s | model + tracking overlay | Fig. 5 out–in–out lateral forces |
| Hover-under stability | 2 | Bravo stationary under Alpha | not in this paper | hover | hw | no unbounded oscillation |
| Lemniscate | 2 | Bravo under hovering Alpha | Alpha at $\hat{e}_3=-2.5$ m | figure-eight, one period | hw | no-model 0.181 m / 0.128 m/s; equiv. 0.113 m (37%), 0.081 m/s (36%); vertical ~51% |
| Translation / transect | 2 | Bravo under Alpha | Δz = 0.6 m, $\hat{e}_1=0.2$ m | back-forth along $\hat{e}_2$ | hw | 3D pos 0.154 → 0.098 m (~36%); vel ~34%; vertical 55% / 49% |

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| 5 min vs >15 min | 5 min; more than 15 min | training flight time vs SOTA *validation RMSE*, lateral + downward forces | abstract, contrib. 2, §V-B, Fig. 3 | Not a closed-loop tracking number. Stages shortened proportionally. |
| Full training set | ≈ 17 min | concatenated staged real flights | §V-A | Three stages. |
| 36% / 56% headline | "nearly 36%" 3D; 56% vertical | "dense real-world flights with two vehicles", "on average" | abstract | Contrib. 3 instead says 56% vertical and 36% *lateral*. Body does not retract either. |
| Lemniscate 3D | 0.181 → 0.113 m (37%); vel 0.128 → 0.081 m/s (36%) | one figure-eight period under Alpha at $z=-2.5$ m | §V-C, Fig. 6 | Separate from the 56% vertical headline. Vertical on this run is "nearly 51%". |
| Translation 3D | 0.154 → 0.098 m, "almost 36%" | mean 3D position on the 0.6 m transect | §V-C | Peak reduced "similarly". Vel 34%. |
| Translation vertical | 55% position, 49% velocity | "only the vertical tracking performance in these tests (not shown in figures)" | §V-C | Fig. 7 caption: "over 55% improvement in $\hat{e}_3$ alone." Closest body support for the 56% headline. |
| Params | 256 / 321 / 9043 | equiv. / shallow non-eq. / deep non-eq. | Fig. 3 table | Deep net is the [16]/[15] architecture with spectral norm ≤ 2. |
| Rates | 50 Hz control, 45 Hz network | onboard Pi | §V-C | Not STM32. |
| Vehicle size | 0.26 m diagonal, ≈ 0.7 kg | custom platform | §V-C | Do not call this a Crazyflie. |
| Closest training sep | ≈ 0.5 m | stage-2 vertical | §V-A | Eval seps are 0.6 m and 0.8 m, not 0.5 m. |

## 6. Assumptions and failure modes

- Translation invariance of $\mathbf{f}_{\mathrm{ext}}$.
- SO(2) equivariance about leader body-down. They say this is approximate. Unlikely under "highly asymmetric" rotor speeds / aggressive flight.
- Experiments use $R_{\mathcal{M}}^A=I$: Alpha near hover, frame aligned with inertial.
- No wind / outdoor flow in the reported tests.
- Residual treated as a 3-D force (or mass-normalized accel). Aero torques left to the low-level autopilot.
- High LQR gain margin so subtracting $\mathbf{f}_{\mathrm{ext}}$ is valid.
- Pairwise only. They warn that summing two-vehicle fields may be wrong if wakes interact nonlinearly.
- Labels $\mathbf{a}-\mathbf{u}$ absorb any model error in the nominal plant.
- Sequential collection: later labels are residuals *after* the previous model.

## 7. Relation to our thesis (clearly labelled OUR READING)

- Supports our strategy **3 (geometric / tracking + NN residual)**. Closest published cousin of "predict $\hat{a}_{\mathrm{res}}$ and subtract." Not INDI. Not hybrid NA-INDI. Not FBL-flatness. Not RL.
- What we may cite: SO(2) feature construction; 5 min vs 15 min sample-efficiency on real pairwise data; first ML report of the lateral out–in–out pattern; sequential staged collection; subtract-the-force injection into a high-margin tracking law.
- What we must NOT cite from it as if it were our setup: Crazyflie / STM32; 3-robot superposition; 16-robot / 24 cm; INDI; onboard MCU inference. Their computer is a Raspberry Pi 4B at 45 Hz PyTorch.
- Formations worth reproducing as a *reference condition* if we cite them: Alpha hover + Bravo transect at Δz 0.6–0.8 m with a small lateral offset; Bravo lemniscate under a fixed Alpha; staged tightening from ~1.75 m toward ~0.5 m.
- Compute: they ran the net on a Pi 4B, 256 weights, 45 Hz. STM32 feasibility: not in this paper.

## 8. Equations worth keeping

(1) Nominal NED rigid-body (collective thrust only):

$$
m\mathbf{a}=-R_{\mathcal{M}}^{C}T+\hat{e}_3 mg
$$

(2) Linearized plant: $\dot{\mathbf{x}}=A\mathbf{x}+B\mathbf{u}$.

Compensation (unnumbered, after (2)):

$$
\mathbf{u}_f(t)\equiv -K(\mathbf{x}-\mathbf{x}_r)-\mathbf{f}_{\mathrm{ext}}
$$

Label used in training:

$$
\mathbf{f}_{\mathrm{ext}}=\mathbf{a}-\mathbf{u}(t)
$$

(4) Invariant features $h(\mathbf{x})\in\mathbb{R}^6$.

(5) Equivariant reconstruction $F_\Theta(\mathbf{x})$ via $\varphi(\mathbf{x})$ and $f_\Theta:\mathbb{R}^6\to\mathbb{R}^2$.

(7) Shallow net:

$$
f_\Theta(\mathbf{w})=W^{(2)}\sigma(W^{(1)}\mathbf{w}+b^{(1)})+b^{(2)},\quad \sigma=\mathrm{ReLU}.
$$

## 9. Figures / tables worth remembering

- Fig. 1: snapshot, Bravo path with model (green) vs without (red).
- Fig. 2: cartoon of Assumption 2. Rotate $\Delta\mathbf{p}$ and $\mathbf{v}^B$ in Alpha's $a_1 a_2$ plane; force rotates the same way.
- Fig. 3: validation RMSE vs minutes of training (5-trial mean) plus a summary table of params / RMSE / tracking errors on the full set.
- Fig. 4: predicted downward accel. Top-down at 1 m stack. Sagittal with 0.1 m east offset. Bravo at 0.5 m/s.
- Fig. 5: 0.8 m stack. (a) Bravo position error while translating. (b) lateral force field, $+\mathbf{v}^B$ and $-\mathbf{v}^B$; non-equivariant field breaks the assumed symmetry.
- Fig. 6: lemniscate time series and histograms, down-axis and 3-D, three controllers.
- Fig. 7: translation path with predicted force arrows, plus error histograms.

## 10. Open uncertainties

- No peer-reviewed venue printed on this file.
- LQR vs LQG: both words appear. Gains $K$ not given.
- Exact closed-loop speeds / accelerations of lemniscate and transect: not in this paper (0.5 m/s is only the viz case).
- Hover-under Δz for the stability check: not in this paper.
- Horizontal path of the lemniscate (how far off-axis each pass is): not in this paper.
- IMU / RPM / estimator details beyond "Freyja" and OptiTrack: not in this paper.
- Whether Bravo's NN runs using Alpha pose from radio or mocap-as-if-onboard: they say decentralized OptiTrack. Comms protocol: not in this paper.
- Fig. 3 table cell-by-cell tracking numbers: the PDF table is dense; use the running-text numbers in §V-C unless you re-read the figure.

## 11. 12-line abstract I can reread

Smith et al. learn the 3-D downwash force Alpha puts on Bravo. They assume translation invariance and SO(2) equivariance about Alpha's body-down, map $\mathbf{x}$ to 6 invariant features, and rotate a 2-D network output by the polar angle of $\Delta\mathbf{p}$. A 256-parameter shallow ReLU net is enough because geometry is hard-coded. Labels are $\mathbf{a}-\mathbf{u}$ from an LQR/LQG loop. Data are collected in three real-flight stages down to ≈ 0.5 m. Full set ≈ 17 min. With 5 min they beat a deep spectrally normalized baseline trained on 15 min. Hardware is two 0.26 m, 0.7 kg custom quads, Pi 4B, OptiTrack, 50 / 45 Hz. Closed loop: subtract $\mathbf{f}_{\mathrm{ext}}$ from the LQR command. Lemniscate under Alpha at $z=-2.5$ m: 3-D position 0.181 → 0.113 m (37%). Transect at 0.6 m / 0.2 m offset: 3-D 0.154 → 0.098 m (~36%), vertical 55%. Abstract says 36% 3-D and 56% vertical. Contribution 3 says 36% lateral and 56% vertical. Keep both wordings. Pairwise only. Not our airframe.

Source: attached PDF only. No other paper used.
