# L1 KNODE-DW MPC — Hsieh et al., 2025

## 0. Identity

- Full title: Online Adaptation for Flying Quadrotors in Tight Formations
- Authors: Pei-An Hsieh, Kong Yao Chee, M. Ani Hsieh
- Year / venue / arXiv / DOI: 2025. arXiv:2506.17488 [cs.RO]. No journal name or DOI printed on this file.
- Version of THIS file: arXiv **v3**, 30 Oct 2025 (header on p. 1)
- Suggested cite key: `hsieh2025onlineadaptation`
- What this paper IS: off-board $\mathcal{L}_1$-augmented mixed-expert NMPC for **three** Crazyflie 2.1 vehicles in V-stack and I-stack hardware flights.
- What this paper is NOT: not the two-robot KNODE-DW paper [5]; not onboard STM32 control; not INDI / geometric / FBL; no closed-loop simulation campaign in this file.

## 1. Problem they pose

**Their wording.** Tight formation flight is hard because "complex aerodynamic wake interactions can destabilize individual team members as well as the team." Effects are "highly nonlinear and fast-paced." Mixed-expert models "are often trained offline and cannot adapt to unseen conditions." Prior $\mathcal{L}_1$+MPC work [13] added no extra disturbance model, so "it remains unclear whether $\mathcal{L}_1$ adaptive control can benefit from more accurate dynamic models."

**Plain restatement.** They want three stacked Crazyflies to hold small vertical gaps while downwash changes. Offline KNODE-DW is not enough by itself. They add an online $\mathcal{L}_1$ residual inside the MPC prediction model and test whether a better downwash model makes that residual work better.

## 2. Paper map (follow their TOC)

### Abstract

- Purpose: name the method and the headline result.
- Method: $\mathcal{L}_1$ KNODE-DW MPC. Mixed-expert downwash plus online adaptation, inside MPC.
- Result (abstract): evaluated in "two different three-quadrotor formations"; "outperforms several MPC baselines"; three-vehicle team "remain vertically aligned in close proximity"; "$\mathcal{L}_1$ adaptive module compensates for unmodeled disturbances most effectively when paired with an accurate dynamics model."

### 1 Introduction

- Purpose: gap and contribution.
- Construction: cite DNN downwash [8,9] as data-heavy; cite mixed-expert KNODE-DW [5] as accurate but offline; cite $\mathcal{L}_1$ [11–13] as online but previously used without a good disturbance model.
- Assumption: pairwise / above-only downwash is the dominant effect they will model; the rest is residual.
- Result claimed here (contribution paragraph): first demonstration, to their knowledge, of a compact formation "involving more than two quadrotors" with "pair-wise separation distance of two body lengths." Hardware only ("extensive physical experiments").

### 2 Technical Approach

- Purpose: models + controller.

### 2.1 Modeling Complex Aerodynamic Interactions

- Purpose: write the interaction force that later enters MPC.
- Method: neighborhood $\mathcal{N}_i$; pairwise sum of forces from neighbors **above** only.
- Key equation (1):

$$
T_{d,i}(\mathbf{x}_i,\mathcal{S}_i)=\sum_{m\in\mathcal{N}_i,\,p_{z_m}>p_{z_i}}\tau_{d,m}(\mathbf{x}_i,\mathbf{x}_m).
$$

- Two models for $\tau_{d,m}$: DW (airflow + drag) and KNODE-DW (DW + Neural ODE). Details deferred to [5].
- Training in **this** file: "the same training data as in [5]" from **two** vehicles, static-top and stacked. Seconds / points: **not in this paper**.
- Assumption: aggregated force $\approx$ pairwise sum; only $p_{z_m}>p_{z_i}$ counts. Neighborhood radius $\alpha$: value **not in this paper**.

### 2.2 Formation Controller Synthesis

- Inputs: $\mathbf{u}=[u_\gamma,\boldsymbol{\omega}^\top]^\top\in\mathbb{R}^4$ (thrust + body rates).
- Nominal dynamics (2):

$$
\dot{\mathbf{p}}=\mathbf{v},\quad
\dot{\mathbf{v}}=-\mathbf{g}+\frac{1}{\eta}R\gamma,\quad
\dot{\mathbf{q}}=\frac12 G(\mathbf{q})^\top\boldsymbol{\omega},
$$

with $\gamma=[0,0,u_\gamma]^\top$, mass $\eta$.

- Six controllers: nominal MPC, DW MPC, KNODE-DW MPC, and the three $\mathcal{L}_1$ versions.

**Baseline MPC (3a–3d).** Standard quadratic tracking MPC. Prediction uses $f_{\mathrm{nom}}$ only. Receding horizon: apply first input $u_{i,0}^\star$.
Horizon $N$, period $T$, weights $Q,R,P$: **not in this paper**.

**$\mathcal{L}_1$ Adaptive Module.** Predictor on velocity $\mathbf{z}_i=\mathbf{v}_i$ only (4):

$$
\dot{\hat{\mathbf{z}}}_i=-\mathbf{g}+\frac1\eta T_{d,i}(\mathbf{x}_i,\mathcal{S}_i)+B\bar{u}_{\gamma,i}+\hat{\sigma}_i+A(\hat{\mathbf{z}}_i-\mathbf{z}_i).
$$

$B:=[(1/\eta)R\mathbf{e}_3]$. $A$ Hurwitz, values **not in this paper**.

Piecewise-constant adaptation (5):

$$
\hat{\sigma}(t):=\hat{\sigma}(kT):=(e^{AT}-\mathbb{I})^{-1}Ae^{AT}(\mathbf{z}_i-\hat{\mathbf{z}}_i).
$$

Low-pass residual used in MPC (6):

$$
U_{\sigma,i}(t):=\bigl(U_{\sigma,i}((k-1)T)+\hat{\sigma}(kT)\bigr)e^{-\alpha T}-\hat{\sigma}(kT).
$$

Cutoff $\alpha$: **not in this paper**.

**$\mathcal{L}_1$ KNODE-DW MPC.** Replace (3b) by (7):

$$
\mathbf{x}_{i,j+1}:=f_{\mathrm{nom}}(\mathbf{x}_{i,j},\mathbf{u}_{i,j})+f_{d,i}(\mathbf{x}_{i,j},\mathcal{S}_i)+f_{\sigma,i}(\ldots).
$$

$f_{d,i}$ puts $(1/\eta)T_{d,i}$ in the velocity channels. $f_{\sigma,i}$ puts $U_{\sigma,i}$ in the same channels. Variants differ only in how $T_{d,i}$ is built (KNODE-DW / DW / omitted). Without $\mathcal{L}_1$, drop $f_{\sigma,i}$ to get (8).

### 3 Results

- Purpose: hardware tests on three Crazyflie 2.1.
- Platform: diameter $0.1$ m, mass $\approx 34$ g, Bitcraze thrust-upgrade bundles.
- Sensing: Vicon **120 Hz**.
- Compute: **off-board**. Bottom: Intel NUC13 (i7) via Crazyradio PA at **200 Hz**. Top and center: laptop (i5) via another Crazyradio PA at **400 Hz**. Solver: acados. Commands: thrust + rates.
- Formations (Fig. 1): **V-stack** and **I-stack**.
- Metrics: RMSE and $z_{\max}$. Five repeats. Mean and std in Figs. 2–3.
- Protocol: test the law on the **center** vehicle first, then the **bottom**. Neighbors are not always the same controller (see table below). Then one tight I-stack demo.

### 4 Discussions

- Purpose: interpret, not new numbers.
- Claims: center vehicle feels **above and below**; two-vehicle models miss that. $\mathcal{L}_1$ helps every model fidelity. Ranking they state: $\mathcal{L}_1$ KNODE-DW best, then $\mathcal{L}_1$ DW, then $\mathcal{L}_1$ MPC. DW vs no-model gap under $\mathcal{L}_1$ is "quite small." Tight I-stack "only achievable with the center quadrotor using $\mathcal{L}_1$ KNODE-DW MPC." Pure $\mathcal{L}_1$ is "not as responsive" when entering/leaving downwash and can be "too slow" for very large / fast residuals.

### 5 Conclusion and Future Work

- Repeat: best baseline overall; $\mathcal{L}_1$ works better with a more accurate plant model. Future: add collision-avoidance constraints. None in this controller.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Crazyflie 2.1; diameter $0.1$ m; $\approx 34$ g; thrust-upgrade bundle |
| n robots min/max | 3 / 3 in the reported flights |
| Sim / hardware / both | Hardware only in Sec. 3. No closed-loop sim results in this file |
| Onboard vs off-board (Hz) | Off-board. Bottom NUC13 i7 @ 200 Hz. Top/center laptop i5 @ 400 Hz |
| Sensing | Vicon pose @ 120 Hz |
| Controller family | Learning-based NMPC + $\mathcal{L}_1$ residual acceleration |
| How interaction enters the law | Predicted downwash $T_{d,i}$ and filtered residual $U_{\sigma,i}$ added to the MPC **prediction** model (7), not a separate INDI subtract |
| Learned model? inputs → output | KNODE-DW from [5]. This file does not list NODE inputs/width. Output used as force on $i$ from neighbors above |
| Training data amount / source | "Same training data as in [5]": two-vehicle static-top and stacked. Duration **not in this paper** |
| Formations + min separation | V-stack and I-stack. Tightest pairwise $\Delta z=0.2$ m (= two body lengths on this airframe) |
| Speeds | **not in this paper** |
| Metrics | RMSE; $z_{\max}$; 5 repeats |
| Baseline they compare to | nominal MPC, DW MPC, KNODE-DW MPC, $\mathcal{L}_1$ MPC, $\mathcal{L}_1$ DW MPC |

## 4. Formations and flights (table, one row per experiment)

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Center, V-stack | 3 | top+center offset; bottom centered under the pair | $z_1=0.2$ m, $z_2=0.4$ m, $r=0.1$ m | tracking a reference; type/speed **not in this paper** | hw | Fig. 2 RMSE and $z_{\max}$. Test law on **center**. Neighbors: $\mathcal{L}_1$ MPC and KNODE-DW MPC on top and bottom. Nominal MPC omitted (collisions) |
| Center, I-stack | 3 | pure vertical | $z_1=0.2$ m, $z_2=0.4$ m, offset 0 | same | hw | Fig. 2. Same neighbor protocol |
| Bottom, V-stack | 3 | same V geometry | $z_1=0.2$ m, $z_2=0.3$ or $0.4$ m, $r=0.1$ m | same | hw | Fig. 3. Test law on **bottom**. Top and center run $\mathcal{L}_1$ KNODE-DW MPC |
| Bottom, I-stack | 3 | pure vertical | $z_1=0.2$ m, $z_2=0.3$ or $0.4$ m | same | hw | Fig. 3 |
| Tight I-stack demo | 3 | pure vertical | $z_1=z_2=0.2$ m | time history ~10 s in Fig. 4; speed **not in this paper** | hw | Fig. 4. "vertical separation of less than six body lengths throughout." RMSE 0.034 / 0.057 / 0.049 m (top / center / bottom). $z_{\max}$ 0.069 / 0.179 / 0.115 m |

Training formations named in Sec. 2.1 (static top, stacked) are **two-vehicle data from [5]**, not new flights in this file.

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Pair-wise sep. in the contribution claim | "two body lengths" | intended pair-wise gap on 0.1 m vehicles $\Rightarrow 0.2$ m | p. 2 contribution | Not a measured mean gap |
| Tight I-stack stack height | "less than six body lengths throughout the flight" | whole 3-vehicle stack, not pair-wise | body p. 7 | 6 body lengths = 0.6 m on their diameter |
| Center: $\mathcal{L}_1$ KNODE-DW vs KNODE-DW | 47.9% RMSE; 40.4% $z_{\max}$ | "on average" over the center V- and I-stack cases in Fig. 2 | body p. 6 | Neighbors were $\mathcal{L}_1$ MPC and KNODE-DW, not all-identical |
| Center: $\mathcal{L}_1$ MPC vs KNODE-DW | 28.5% RMSE; 26.3% $z_{\max}$ | same center tests | body p. 7 | Adaptation without a downwash model still beat offline KNODE-DW **on the center vehicle in those tests** |
| Bottom: $\mathcal{L}_1$ KNODE-DW vs KNODE-DW | 15% RMSE; 6.6% $z_{\max}$ | "best overall" over the four bottom configs in Fig. 3 | body p. 7 | At $z_2=0.4$ m several methods "match"; the gap shows up at $z_2=0.3$ m |
| Tight I-stack RMSE | 0.034, 0.057, 0.049 m | top, center, bottom | body p. 7 / Fig. 4 | One demonstrated flight set; n of repeats for this tight case **not stated separately** from the "five times" rule |
| Tight I-stack $z_{\max}$ | 0.069, 0.179, 0.115 m | top, center, bottom | body p. 7 | Center is the worst vehicle |
| Vehicle size | 0.1 m diameter; $\approx 34$ g | Crazyflie 2.1 | body p. 5 | Diameter, not motor-to-motor spelled as "body length" in this file; they still say "two body lengths" = 0.2 m |
| Vicon / radios | 120 Hz; 200 Hz bottom; 400 Hz others | pose rate and command links | body p. 5 | Not the MPC horizon period $T$ |
| Repeats | five | each "test scenario" | body p. 6 | Tight-demo repeats not separately numbered |

Abstract and body do not conflict on the complementarity sentence. Abstract has no 47.9% / 15% figures. Those live in the body.

## 6. Assumptions and failure modes

- Pairwise superposition of downwash (justified by citing [9], not re-derived here).
- Only vehicles **above** enter $T_{d,i}$. Below-vehicle aero is unmodeled and left to $\hat{\sigma}$.
- KNODE-DW trained on **two-vehicle** data; three-vehicle use is zero-shot plus $\mathcal{L}_1$.
- Residual is **acceleration** on the velocity channels only. No torque residual in (4)–(7).
- Top/center/bottom do **not** always run the same law during ablations.
- No collision-avoidance constraint in the OCP. Tight 0.2/0.2 m I-stack failed with other center laws ("typically resulted in collisions").
- Nominal MPC "lead to collisions between the center and bottom" in the center-vehicle tests.
- $\mathcal{L}_1$ alone "not as responsive" on enter/exit of downwash; "may be too slow" for fast / very large residuals.
- Off-board radios and Vicon. Latency **not characterized** in this file.
- Filter cutoff $\alpha$ and Hurwitz $A$ are user-chosen; values not given.

## 7. Relation to our thesis (OUR READING)

- Which of our strategies it supports: closest to **6 / learning-based MPC**, not 1–5. The $\mathcal{L}_1$ residual is an **acceleration** injected into the predictor, which is conceptually near our "residual force/acceleration, subtracted," but the law is NMPC, not INDI or geometric.
- What we may cite: 3-Crazyflie V-stack and I-stack geometry; 0.2 / 0.3 / 0.4 m $\Delta z$; RMSE + $z_{\max}$; statement that adaptation works **best with** a good interaction model; center vehicle is harder than bottom because of above+below aero.
- What we must NOT cite from it: onboard STM32 results; OptiTrack (they use Vicon); brushless CF not named as brushless; 2-robot 1.2-body-length Chee numbers; any 20× solve-time figure; "sim + hardware" for **this** paper; INDI / FBL / geometric performance.
- Formations to reproduce as a reference condition: I-stack $z_1=z_2=0.2$ m; I-stack $z_2=0.3$ and $0.4$ m; V-stack $r=0.1$ m, $z_1=0.2$ m, $z_2=0.3/0.4$ m. Hold formation while tracking. Report center and bottom separately.
- Compute: they ran acados on a **NUC13 i7** and a **laptop i5**, 200–400 Hz command rates. This file does **not** claim STM32 feasibility. An NMPC + NODE + $\mathcal{L}_1$ stack of this form would not be "what they ran onboard," because they did not run it onboard.

## 8. Equations worth keeping

**(1)** $T_{d,i}=\sum_{m\in\mathcal{N}_i,\,p_{z_m}>p_{z_i}}\tau_{d,m}$. Pairwise downwash from above.

**(2)** Nominal rigid-body quadrotor, mass $\eta$, input thrust $u_\gamma$ and rates $\boldsymbol{\omega}$.

**(3a–d)** Tracking NMPC with $f_{\mathrm{nom}}$ only.

**(4)** Velocity predictor with known $T_{d,i}$, last thrust $\bar{u}_{\gamma,i}$, residual $\hat{\sigma}_i$, Hurwitz injection $A(\hat{\mathbf{z}}-\mathbf{z})$.

**(5)** Piecewise-constant $\hat{\sigma}(kT)$ from predictor error.

**(6)** Filtered compensation $U_{\sigma,i}$ at cutoff $\alpha$.

**(7)** MPC step: $f_{\mathrm{nom}}+f_{d,i}+f_{\sigma,i}$. This is the whole method.

**(8)** Same without $f_{\sigma,i}$.

## 9. Figures / tables worth remembering

- **Fig. 1:** V-stack vs I-stack schematic. Labels: $z_1$ top–center, $z_2$ center–bottom. Caption uses $\tau$ for the V-stack lateral gap; body text uses $r=0.1$ m.
- **Fig. 2:** Center-vehicle RMSE (top panel) and $z_{\max}$ (bottom). Mean ± std. Two geometries.
- **Fig. 3:** Bottom-vehicle same metrics. Includes nominal MPC. Four geometry/separation combos.
- **Fig. 4 left:** $z$ and thrust vs time for tight I-stack. Blue top, orange center, green bottom. Time axis to ~10 s.
- **Fig. 4 right:** composite photo of the same flight.
- No numbered results table. Numbers live in prose + error-bar plots.

## 10. Open uncertainties

- Trajectory shape, speed, and acceleration: not stated.
- MPC horizon $N$, step $T$, weights $Q,R,P$: not stated.
- $\mathcal{L}_1$ matrices $A$ and filter cutoff $\alpha$: not stated.
- Neighborhood radius $\alpha$ in $\mathcal{N}_i$: not stated (same symbol as the filter cutoff; two different $\alpha$s in the paper).
- NODE architecture / inputs: deferred to [5].
- How many repeats specifically for the tight 0.2/0.2 m demo.
- Radio latency, packet loss, solver time.
- Whether I-stack had any residual lateral error around the intended zero offset.
- Caption $\tau$ vs body $r$ for the V-stack offset (same 0.1 m quantity, two symbols).

## 11. 12-line abstract I can reread

Hsieh, Chee, Hsieh (arXiv:2506.17488v3) add an $\mathcal{L}_1$ residual-acceleration loop to KNODE-DW NMPC.
The residual is filtered and inserted into the MPC prediction model with the pairwise "vehicles above" downwash term.
Evaluation is hardware only: three Crazyflie 2.1, Vicon 120 Hz, off-board acados (NUC i7 @ 200 Hz bottom; laptop i5 @ 400 Hz others).
Formations: V-stack ($r=0.1$ m) and I-stack. $z_1=0.2$ m. $z_2=0.2$, $0.3$, or $0.4$ m.
Metrics: RMSE and $z_{\max}$, five repeats.
Center vehicle: $\mathcal{L}_1$ KNODE-DW beat KNODE-DW by 47.9% RMSE and 40.4% $z_{\max}$ on average.
Bottom vehicle: 15% and 6.6%. At $z_2=0.3$ m only the full method held the looser-gap performance.
Tight I-stack 0.2/0.2 m flew; RMSE 0.034 / 0.057 / 0.049 m; $z_{\max}$ 0.069 / 0.179 / 0.115 m. Other center laws collided.
They state $\mathcal{L}_1$ works best when paired with an accurate disturbance model.
KNODE-DW weights are frozen from two-vehicle [5] data. Only $\hat{\sigma}$ adapts online.
This is not onboard, not INDI, not two-robot Chee 2024.

Source: attached PDF only. No other paper used.
