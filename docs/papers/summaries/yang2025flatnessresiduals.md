# Flatness-preserving residuals — Yang et al., 2026 (arXiv 2025)

## 0. Identity

- Full title: Learning Flatness-Preserving Residuals for Pure-Feedback Systems
- Authors: Fengjun Yang, Jake Welde, Nikolai Matni (GRASP Lab, University of Pennsylvania)
- Year / venue / arXiv / DOI: dated 28 January 2026; arXiv:2504.04324v3 [eess.SY], 27 Jan 2026. No journal or conference venue printed on this file. No DOI for this paper in this file.
- Version of THIS file: arXiv v3 (27 Jan 2026). First arXiv id 2504.04324 implies April 2025 posting. This file is the long preprint. Whether a shorter conference version exists is not in this paper.
- Suggested cite key: `yang2025flatnessresiduals`
- What this paper IS: a theory-plus-simulation paper that learns a lower-triangular residual so a pure-feedback flat nominal model stays locally flat, with an explicit construction of the new flat maps, tested on one 2D quadrotor in simulation.
- What this paper is NOT: not hardware; not Crazyflie; not 3D; not multi-robot; not an INDI / NA-INDI / L1 paper; not a flight-test of interaction wrenches. Downwash is only a cited motivation from other papers.

## 1. Problem they pose

Their wording (Intro): flatness is usually proved on idealized nominal models. Real residuals (rotor drag, downwash, etc.) are learned with generic function classes that "can lead to augmented dynamics that no longer enjoy the flatness property." Even if flatness survives, "it is often unclear how to construct the flatness diffeomorphism."

Plain restatement: they want a residual you can fit from data without breaking flatness, and a recipe to rebuild the flat maps from the nominal ones.

## 2. Paper map (follow their TOC)

### 1 Introduction

- Purpose: state the gap, place the work, list three contributions.
- Method / construction: none yet. Motivation examples of residuals include rotor drag and "another quadrotor's downwash" (citations only).
- Key equation: none numbered here.
- Assumption: nominal models are flat; residuals matter for tracking.
- Result claimed: "first work to address learning flatness-preserving residual dynamics for differentially flat systems" (body §1). Abstract also claims 5× tracking and 20× speed vs a structure-agnostic alternative.

### 1.1 Related Work

- Purpose: split prior art into residual learning vs learning-to-linearize / adaptive pure-feedback / model-free flat control.
- Method: literature only. Residual papers they name use system-specific laws or NMPC. They want to keep flatness-based compute.
- Key equation: none.
- Assumption: NMPC is "drastically more computationally expensive than flatness-based control methods for quadrotors."
- Result: positioning only. Complementary to Fliess–Join model-free and to GP / RL feedback linearization.

### 1.2 Contributions

- Purpose: three bullets.
- (i) Class of residuals that keep flatness of pure-feedback systems (Thm. 2) and a construction of the new diffeomorphism from the nominal one (Thm. 3).
- (ii) A parameterization so that class is learnable from trajectories.
- (iii) 2D-quadrotor simulation: lower tracking error than nominal flat control, order-of-magnitude faster than NMPC on the same learned model.

### 2 Preliminaries and Problem Formulation

- Purpose: define flatness and the residual split.
- Method: true dynamics $f = \bar f + \Delta$. Learn $\hat\Delta$. Restrict $\bar f$ to MIMO pure-feedback form.
- Key equations:
  - (1) $\dot{\mathbf{x}} = f(\mathbf{x},\mathbf{u})$
  - (2) $\dot{\mathbf{x}} = \bar f(\mathbf{x},\mathbf{u}) + \Delta(\mathbf{x},\mathbf{u})$
  - (3) $\Delta(\mathbf{x},\mathbf{u}) := f(\mathbf{x},\mathbf{u}) - \bar f(\mathbf{x},\mathbf{u})$
  - (4) $\hat f = \bar f + \hat\Delta$
  - Def. 1 (Hagenmeyer and Delaleau 2003): flat output $\mathbf{y}=\Lambda(\mathbf{x},\mathbf{u},\dot{\mathbf{u}},\ldots)$ and maps $\mathbf{x}=\Phi(\mathbf{y},\ldots)$, $\mathbf{u}=\Psi(\mathbf{y},\ldots)$. Local if it holds in a neighborhood $\mathcal{N}$ of $(\mathbf{x}^*,\mathbf{u}^*)$.
- Assumption 1: nominal dynamics in pure-feedback form (5), each $\mathbf{x}_i\in\mathbb{R}^m$ same relative degree $r$ as $\mathbf{u}\in\mathbb{R}^m$. Remark 1: unequal relative degrees need dynamic extension; they do not develop that case.
- Result: problem statement only.

### 3 Differentially Flat Pure-Feedback Systems

- Purpose: recall why pure-feedback + regularity is locally flat with $y=x_1$.
- Method: implicit function theorem. Solve $x_{i+1}=h_i(x_1,\ldots,x_i,\dot x_i)$ and recurse.
- Key equations:
  - Assump. 2: $\bar f_i$ are $C^1$; $|D_{x_{i+1}}\bar f_i|\neq 0$ and $|D_u \bar f_r|\neq 0$ at the operating point.
  - Thm. 1: then $\bar f$ is locally differentially flat around $(\mathbf{x}^*,\mathbf{u}^*)$ with flat output $\mathbf{y}=\mathbf{x}_1$.
  - Running example (6)–(7): planar quadrotor, thrust dynamically extended twice.
    $\mathbf{x}_1=\mathbf{p}$, $\mathbf{x}_2=\mathbf{v}$, $\mathbf{x}_3=[F,\theta]^\top$, $\mathbf{x}_4=[\dot F,\omega]^\top$, $\mathbf{u}=[\ddot F,\tau]$.
    Flat except when $F=0$.
- Assumption: regularity of the partial Jacobians. Local, not global.
- Result: Thm. 1. Full proof in Appendix A.

### 4 Flatness-Preserving Perturbations

- Purpose: sufficient structure on $\Delta$ so the sum stays flat and the old $y$ is still a flat output. Then build new $\Phi,\Psi$.
- Method: force $\Delta$ to be lower-triangular in the pure-feedback coordinates. Jacobians w.r.t. the next state are unchanged. Reconstruct maps by subtracting $\hat\Delta_{k-1}$ before applying the old $h_{k-1}$.
- Key equations:
  - Def. 2 (8): $\Delta(\mathbf{x})=\bigl[\Delta_1(\mathbf{x}_1),\,\Delta_2(\mathbf{x}_1,\mathbf{x}_2),\,\ldots,\,\Delta_r(\mathbf{x}_1,\ldots,\mathbf{x}_r)\bigr]^\top$, each $\Delta_i$ of class $C^1$. No $\mathbf{u}$ in this definition.
  - Thm. 2: Assumps. 1–2 + lower-triangular $\hat\Delta$ $\Rightarrow$ $\bar f+\hat\Delta$ locally flat around $(\mathbf{x}^*,\mathbf{u}^*)$ with $\mathbf{y}=\mathbf{x}_1$.
  - Assump. 3: the nominal implicit maps $h_k$ in (9) are known.
  - Thm. 3 / (10): recursive $\hat\Phi_k$ from $h_{k-1}$ and $\hat\Delta_{k-1}$. Then $\mathbf{x}_k=\hat\Phi_k(\mathbf{y},\ldots,\mathbf{y}^{(k-1)})$, $\mathbf{u}=\hat\Phi_{r+1}(\mathbf{y},\ldots,\mathbf{y}^{(r)})$.
  - Example residual for 2D quad: $\Delta_1=\Delta_3=\Delta_4=0$, $\Delta_2(\mathbf{x}_2)=-(C_r/m)\mathbf{x}_2$. Closed form for $\hat\Phi_3$ given on p. 8.
- Assumption: residual does not depend on the next state (or on $u$ in Def. 2). Nominal $h_k$ available. Same neighborhood regularity.
- Result: Thms. 2–3 in the body. Same flat outputs "often carry semantic meaning."

### 5 Learning Flatness-Preserving Residuals

- Purpose: turn Def. 2 into a trainable model.
- Method: one network (or block) per $\hat\Delta_i(\mathbf{x}_1,\ldots,\mathbf{x}_i;\theta_i)$. Smooth activations. Fit by squared error on finite-difference residuals. After fit, build (10) offline with autodiff Jacobians.
- Key equations:
  - (11) $L(\Theta)=\sum_i\sum_k \bigl\|\Delta(\tau^i_k)-\hat\Delta(\tau^i_k;\Theta)\bigr\|_2^2$
  - Finite difference: $\Delta(\tau^i_k):=\frac{\mathbf{x}^i(t_{k+1})-\mathbf{x}^i(t_k)}{t_{k+1}-t_k}-\bar f(\mathbf{x}^i(t_k),\mathbf{u}^i(t_k))$
- Assumption: trajectories from the true system (2) are available. $\hat\Delta_i$ $C^1$ in each $x_j$. They write $\hat\Delta_\Theta(\mathbf{x},\mathbf{u})$ once, then restrict to $\hat\Delta_\Theta(\mathbf{x})$ lower-triangular (no $u$).
- Result: algorithm description only. No test numbers in this section.

### 6 Numerical Experiments

- Purpose: open-loop consistency, then closed-loop tracking vs NMPC, both in simulation.
- Method / construction: see 6.1–6.3 below.
- Assumption: true residual is itself lower-triangular (drag only in $\dot{\mathbf{v}}$). Full state observed in closed loop.
- Result: Tables 1–2, Figs. 2–3. Code: https://github.com/FJYang96/flat-residual/

### 6.1 Setup

- Purpose: plant, data, network, references.
- True perturbation (12): $\dot{\mathbf{x}}_2=\bar f_2(\mathbf{x}_3)-C_r\mathbf{x}_2-C_p\|\mathbf{x}_2\|\mathbf{x}_2$. Linear + parasitic drag, simplified from 3D non-isotropic models they cite.
- Data: 3000 trajectories, 0.5 s each, 100 Hz. Init $\mathbf{p}$ in $[-1,1]^2$ in the section text; App. B gives a 6-D box. Random $F,\tau$.
- Network: feedforward, 1 hidden layer, 32 neurons, GeLU. 20 epochs ADAM on Nvidia A5000, under 1 minute. 30 seeds.
- References: circle $\mathbf{p}_{\mathrm{circ}}(t)=[\cos(\omega t),\sin(\omega t)]^\top$; Gerono lemniscate with $a=1$, $b=0.6$; $T=14$, $\omega=2\pi/T$.
- Parameters (App. B Table 3): $m=1$, $I=0.1$, $g=9.81$, $C_p=0.1$, $C_r=0.01$.
- Result: setup only.

### 6.2 Open-Loop Trajectory Evaluation

- Purpose: feed a position path through nominal vs learned flat maps, apply the resulting $u$ to the true $f$, measure position drift.
- Method: RK4, step $\tau=10^{-2}$ s. Error is mean $\|\mathbf{x}_1(k\tau)-\mathbf{p}(k\tau)\|$ over the horizon.
- Key equation: the error sum on p. 11 (unnumbered).
- Result (Table 1, mean ± std over 30 training runs):
  - Circle: nominal 0.312 m; learned $0.0265\pm 0.0058$ m.
  - Lemniscate: nominal 0.238 m; learned $0.0417\pm 0.0066$ m.
  - Body: "on average … reduces position error by nearly an order of magnitude."
  - Ground-truth residual + Thm. 3 map: open-loop error "less than $10^{-5}$." Construction check, not a learned-model score.
- Fig. 2: example open-loop paths.

### 6.3 Closed-Loop Tracking Control

- Purpose: flatness PD on Brunovsky form vs NMPC, same learned model.
- Method: $\mathbf{y}^{(4)}=\boldsymbol{\nu}$ with (13). Luenberger observer for higher derivatives of $y$. $u=\hat\Psi(y,\dot y,\ddot y,y^{(3)},\nu)$. Both laws at 100 Hz zero-order hold. Full state assumed known.
- NMPC: horizon $H=1$ s, 100 RK4 steps, $Q=Q_T=I$, $R=0.001 I$, CasADi in Python, warmstart, solve to convergence.
- Gains (App. B): $k_0=3.96$, $k_1=12.08$, $k_2=13.16$, $k_3=6.03$ (pole placement).
- Result (Table 2, mean closed-loop tracking error in metres, 30 seeds):
  - Circle: Flat-nominal 0.0478; Flat-learned $0.0051\pm 0.0009$; NMPC $0.0062\pm 0.0001$.
  - Lemniscate: Flat-nominal 0.0258; Flat-learned $0.0061\pm 0.0008$; NMPC $0.0063\pm 0.0001$.
  - Body: both learned controllers showed "more than 5× less position tracking error than the nominal flatness-based controller."
  - Compute (Fig. 3c / body): flat+learned $3.3\pm 0.3$ ms per update; NMPC $85.2\pm 1.74$ ms. Caption: "20× less time." They call this suitable for "computationally constrained hardware" but they did not run on a flight controller.

### 7 Conclusion

- Purpose: recap and future work.
- Future work in this file: relax structural assumptions; online adaptation to slowly varying residuals; "deploying the algorithm on quadrotors in physical experiments."
- Result claimed: "first step towards more principled methods for learning residual models for differentially flat systems."

### A Proof of Theorem 1

- Lemma 4: existence of each $h_k$ by IFT. Then induction for $\Phi$.
- No extra experimental claims.

### B Experiment Details

- Table 3 parameters (above).
- Init box in App. B: $\mathbf{x}\in\mathbb{R}^6$, $x_{ub}=-x_{lb}=[1,\,1,\,0.5,\,0.5,\,0.05,\,0.1]^\top$.
- At each data-collection step: $F$ uniform in $[8.98,10.98]$, torque uniform in $[-0.5,0.5]$.
- Controller gains listed above.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Simulated planar ("2D") quadrotor. $m=1$, $I=0.1$. No airframe name. No arm length in this file. |
| n robots min/max | 1 / 1. No multi-robot experiment. |
| Sim / hardware / both | Simulation only. Physical deployment is future work. |
| Onboard vs off-board (Hz) | Not a vehicle. Control updates at 100 Hz in sim. Training on Nvidia A5000. NMPC in Python/CasADi. Flat law timed in the same desktop setup (3.3 ms vs 85.2 ms). |
| Sensing (mocap, RPM, GPS, …) | Assumed full state $\mathbf{x}$. Luenberger observer only for extra derivatives of the flat output. No mocap/IMU/RPM described. |
| Controller family | Differential-flatness feedforward + linear Brunovsky feedback (13). Baseline: same learned model inside NMPC. |
| How interaction enters the law | There is no inter-robot interaction law. Residual is an additive $\hat\Delta(\mathbf{x})$ on $\dot{\mathbf{x}}$. In the example it is a drag force in $\dot{\mathbf{v}}$ only. Then the flat maps (10) absorb $\hat\Delta$. |
| Learned model? inputs → output | Yes. Lower-triangular $\hat\Delta_i(\mathbf{x}_1,\ldots,\mathbf{x}_i;\theta_i)$. Implemented as one 1×32 GeLU MLP on $\mathbf{x}$. Output is the residual vector field, not a body wrench map in 3D. |
| Training data amount / source | 3000 simulated trajectories, 0.5 s × 100 Hz, random init and random $F,\tau$. Finite-difference residual labels. 20 epochs. 30 seeds. |
| Formations + min separation | not in this paper |
| Speeds | not in this paper as a speed in m/s. Circle and lemniscate use $\omega=2\pi/14$. Path amplitude: circle radius 1; lemniscate $a=1$, $b=0.6$. |
| Metrics | Mean position error (m). Open-loop Table 1. Closed-loop Table 2. Wall-clock ms per control update Fig. 3c. |
| Baseline they compare to | (i) nominal flat maps / nominal flat controller (no residual). (ii) NMPC on the same learned $\bar f+\hat\Delta_\Theta$. (iii) open-loop check with ground-truth $\Delta$. |

## 4. Formations and flights (table, one row per experiment)

No formations. No hardware flights. Two simulated reference paths on one planar vehicle.

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Circle open-loop | 1 | planar circle radius 1 | not in this paper (2D) | $\omega=2\pi/14$, $T=14$ | sim | Table 1 position error 0.312 vs $0.0265\pm0.0058$ m |
| Lemniscate open-loop | 1 | Gerono lemniscate $a=1,b=0.6$ | not in this paper | same $\omega,T$ | sim | Table 1 0.238 vs $0.0417\pm0.0066$ m |
| Circle closed-loop | 1 | same circle | not in this paper | 100 Hz ZOH | sim | Table 2: 0.0478 / $0.0051\pm0.0009$ / $0.0062\pm0.0001$ m (nom / flat-learn / NMPC) |
| Lemniscate closed-loop | 1 | same lemniscate | not in this paper | 100 Hz ZOH | sim | Table 2: 0.0258 / $0.0061\pm0.0008$ / $0.0063\pm0.0001$ m |
| Compute timing | 1 | same tasks | — | 100 Hz | sim, Python | $3.3\pm0.3$ ms flat-learn vs $85.2\pm1.74$ ms NMPC |

Do not import Neural-Swarm / downwash stacks. Those citations are motivation only (p. 4).

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| "tracking error 5× lower than the nominal flat model" | 5× | closed-loop position error vs nominal flat controller | Abstract; body §6.3 says "more than 5× less" | Table 2 ratios: circle $0.0478/0.0051\approx 9.4$; lemniscate $0.0258/0.0061\approx 4.2$. 30 seeds. Two paths only. Sim. |
| "20× faster over a structure-agnostic alternative" | 20× | wall-clock time per control update vs NMPC | Abstract; Fig. 3c caption "20× less time" | Body times: $3.3\pm0.3$ ms vs $85.2\pm1.74$ ms $\approx 26\times$. Same learned model. CasADi Python NMPC, horizon 1 s, 100 steps. Not an STM32 number. |
| "order of magnitude less computation time" | order of magnitude | same compute comparison | Intro, contributions, §6 | Softer than 20×. Matches the ms table. |
| "nearly an order of magnitude" open-loop | ~10× | open-loop position error vs nominal maps | §6.2 | Table 1, not the closed-loop 5×. |
| Ground-truth diffeomorphism check | $<10^{-5}$ | open-loop position error with true $\Delta$ plugged into (10) | §6.2 | Proves Thm. 3 on this plant. Says nothing about learning quality. |
| Training set | 3000 traj × 0.5 s @ 100 Hz | simulated random rollouts | §6.1 | Not flight logs. |
| Network | 1×32 GeLU, 20 epochs, <1 min on A5000 | training cost | §6.1 | Desktop GPU. |
| Drag coeffs | $C_p=0.1$, $C_r=0.01$ | true-plant parameters | App. B Table 3 | The network is not given these numbers; it fits $\hat\Delta$. |
| Mass / inertia | $m=1$, $I=0.1$ | sim plant | App. B | Dimensionless-looking units; not a Crazyflie. |

Abstract and body do not retract each other. Abstract 5× / 20× are rounded slogans of Table 2 + Fig. 3c.

## 6. Assumptions and failure modes

- Assump. 1: nominal model is MIMO pure-feedback with equal relative degree $r$ per channel (or you dynamically extend, as they do for $F$).
- Assump. 2: $C^1$ and the next-state / input Jacobians are nonsingular at the operating point. Local IFT neighborhood only.
- Assump. 3: you already know the nominal implicit maps $h_k$.
- Def. 2: residual is lower-triangular in those coordinates and $C^1$. In the definition it does not depend on $u$ or on "future" states.
- Thm. 2 is sufficient, not necessary. A non-triangular residual can destroy the Jacobian test they use.
- 2D quadrotor flatness fails at $F=0$.
- Labels $\Delta$ come from a one-step difference. Differentiation noise is not analyzed.
- Closed-loop tests assume full-state observation.
- The simulated "true" residual already has the structure they learn. They do not test a residual that violates Def. 2.
- No robustness to parametric error in $m,I$ is reported.
- NMPC and flat timings are unoptimized Python. They say so.
- No actuator delay, no estimation error model, no contact, no 3D aero.

## 7. Relation to our thesis (clearly labelled OUR READING)

- Supports strategy 4 (FBL / flatness + residual) as a *citation for the idea* "structure the residual so flatness survives." Weak support for strategy 3 (geo+NN residual) only as "people learn residuals." Does not support 1 geo, 2 INDI, or 5 hybrid NA-INDI/L-INDI. Those families are not in this file.
- What we may cite:
  - lower-triangular residual $\Rightarrow$ local flatness + same $y$ (Thm. 2);
  - constructive $\hat\Phi$ from nominal $h_k$ (Thm. 3 / eq. 10);
  - learning that class from trajectory finite differences (eq. 11);
  - 2D sim evidence that a drag-like $\Delta_2(\mathbf{v})$ can be absorbed into the flat maps.
- What we must NOT cite from it:
  - Crazyflie hardware, STM32, OptiTrack, 2- or 3-robot teams;
  - interaction-force / downwash flight data (only cited as other people's residuals);
  - 3D wrench residual;
  - INDI, L1, hybrid INDI;
  - onboard timings;
  - "they flew formations."
- Formations to reproduce if used as a reference condition: none in this paper. Closest reproducible objects are the planar circle (radius 1) and Gerono lemniscate ($a=1,b=0.6$), $T=14$. Those are single-vehicle paths, not our team geometry.
- Compute: they did not run on an STM32. Flat law is a few Jacobians of a tiny MLP plus closed-form $h_k$; in principle that is MCU-plausible, but that is OUR READING, not a result. What they actually ran: Python, autodiff, CasADi NMPC, A5000 for training, 3.3 ms vs 85.2 ms on that stack.

## 8. Equations worth keeping

Symbols: $\mathbf{x}=[\mathbf{x}_1^\top,\ldots,\mathbf{x}_r^\top]^\top$, $\mathbf{x}_i\in\mathbb{R}^m$, $\mathbf{u}\in\mathbb{R}^m$, $\mathbf{y}=\mathbf{x}_1$.

1. Residual split (2)–(3):

$$
\dot{\mathbf{x}}=\bar f(\mathbf{x},\mathbf{u})+\Delta(\mathbf{x},\mathbf{u}),\quad \Delta:=f-\bar f.
$$

2. Pure-feedback nominal (5):

$$
\dot{\mathbf{x}}_i=\bar f_i(\mathbf{x}_1,\ldots,\mathbf{x}_{i+1}) \text{ for } i<r,\qquad
\dot{\mathbf{x}}_r=\bar f_r(\mathbf{x}_1,\ldots,\mathbf{x}_r,\mathbf{u}).
$$

3. Lower-triangular residual, Def. 2 (8):

$$
\Delta(\mathbf{x})=\bigl[\Delta_1(\mathbf{x}_1),\,\Delta_2(\mathbf{x}_1,\mathbf{x}_2),\,\ldots,\,\Delta_r(\mathbf{x}_1,\ldots,\mathbf{x}_r)\bigr]^\top.
$$

4. Nominal implicit maps, Assump. 3 (9):

$$
h_k(\mathbf{x}_1,\ldots,\mathbf{x}_k,\bar f_k(\mathbf{x}_1,\ldots,\mathbf{x}_{k+1}))=\mathbf{x}_{k+1}.
$$

5. Augmented diffeomorphism, Thm. 3 (10):

$$
\hat\Phi_1(\mathbf{y})=\mathbf{y},
$$

$$
\hat\Phi_k = h_{k-1}\bigl(\hat\Phi_1,\ldots,\hat\Phi_{k-1},\, D\hat\Phi_{k-1}\,[\dot{\mathbf{y}};\ldots;\mathbf{y}^{(k-1)}]-\hat\Delta_{k-1}(\hat\Phi_1,\ldots,\hat\Phi_{k-1})\bigr).
$$

6. Learning loss (11) and finite-difference label (unnumbered, §5):

$$
L(\Theta)=\sum_i\sum_k\|\Delta(\tau^i_k)-\hat\Delta(\tau^i_k;\Theta)\|_2^2,
$$

$$
\Delta(\tau^i_k)=\frac{\mathbf{x}(t_{k+1})-\mathbf{x}(t_k)}{t_{k+1}-t_k}-\bar f(\mathbf{x}(t_k),\mathbf{u}(t_k)).
$$

7. Plant residual used in sim (12):

$$
\dot{\mathbf{x}}_2=\bar f_2(\mathbf{x}_3)-C_r\mathbf{x}_2-C_p\|\mathbf{x}_2\|\mathbf{x}_2.
$$

8. Brunovsky law (13):

$$
\boldsymbol{\nu}=\mathbf{y}_r^{(4)}-k_0(\mathbf{y}-\mathbf{y}_r)-\cdots-k_3(\mathbf{y}^{(3)}-\mathbf{y}_r^{(3)}).
$$

## 9. Figures / tables worth remembering

- Fig. 1: sketch of the planar quadrotor (thrust $F$, torque $\tau$, angle $\theta$, gravity).
- Fig. 2: open-loop circle and lemniscate. Learned maps stay closer to the dashed reference than nominal maps.
- Fig. 3a–b: closed-loop circle and lemniscate. Flat-learned and NMPC both hug the reference; Flat-nominal lags.
- Fig. 3c: bar chart of mean compute time per update. Flat vs NMPC.
- Table 1: open-loop mean position error (m) ± std, 30 runs. Circle / lemniscate × Nominal / Learned.
- Table 2: closed-loop mean tracking error (m) ± std. Nominal flat / learned flat / NMPC.
- Table 3 (App. B): $m,I,g,C_p,C_r$.

## 10. Open uncertainties

- No DOI / published venue on this file.
- Whether v3 differs from v1 in results: not stated.
- Exact MLP input vector (full $\mathbf{x}$ vs only $\mathbf{x}_2$) after they say "parameterize $\hat\Delta_\Theta(x)$" as one net.
- Separate $\theta_i$ per block vs one net: both wordings appear.
- Luenberger observer gains: not in this paper.
- NMPC solver tolerances, CPU model for the 3.3 ms / 85.2 ms numbers: not in this paper.
- Integration scheme of the *plant* during data collection: not named (closed-loop eval uses RK4, $\tau=10^{-2}$).
- Why App. B init is $\mathbb{R}^6$ while the extended state is 8-D: $F$ is sampled separately; $\dot F$ init not stated.
- Units of $m=1$: SI or normalized, not stated.
- No min distance, no 3D radius, no RPM, no battery, no rate limits beyond the random $F,\tau$ box.
- No statement that the learned $\hat\Delta$ stayed lower-triangular after training beyond the parameterization.

## 11. 12-line abstract I can reread

Yang, Welde, Matni (arXiv:2504.04324v3) learn residuals for flat pure-feedback plants without killing flatness.
Nominal model: MIMO pure-feedback plus Jacobian regularity (Assumps. 1–2). Flat output is $y=x_1$ (Thm. 1).
Allowed residual: lower-triangular in those coordinates (Def. 2). Same $y$, local flatness (Thm. 2).
New maps: subtract $\hat\Delta_{k-1}$ then apply the old implicit $h_{k-1}$ (Thm. 3, eq. 10).
Fit: squared error on finite-difference $\dot x-\bar f$, smooth nets (eq. 11).
Testbed: one simulated 2D quadrotor. True extra term is drag in $\dot v$ only (eq. 12). No hardware.
Open-loop maps: position error drops from 0.312/0.238 m to about 0.03/0.04 m (Table 1).
Closed-loop: learned flat and NMPC both beat nominal flat by more than 5× on two paths (Table 2).
Compute on their desktop stack: 3.3 ms vs 85.2 ms (Fig. 3c). Abstract calls this 20×.
This is not a Crazyflie, not a team, not INDI, not an interaction-wrench paper.
Cite for structure of $\Delta$ and the map construction. Do not cite for our flights.
Physical quadrotor tests are listed as future work.

Source: attached PDF only. No other paper used.
