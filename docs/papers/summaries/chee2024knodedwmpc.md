# KNODE-DW MPC — Chee et al., 2024

## 0. Identity

- Full title: Flying Quadrotors in Tight Formations using Learning-based Model Predictive Control
- Authors: Kong Yao Chee*, Pei-An Hsieh*, George J. Pappas, M. Ani Hsieh (* equal contribution). GRASP Lab, University of Pennsylvania
- Year / venue / arXiv id / DOI if printed: 2024; this file is arXiv:2410.09727v1 [cs.RO] 13 Oct 2024. No conference name printed on the PDF. No DOI for this paper printed
- Version of THIS file: arXiv v1, 7 pages including references
- Suggested cite key: `Chee2024KnodeDwMpc`
- One sentence: what this paper IS: hybrid analytical-downwash + KNODE residual model inside off-board nonlinear MPC for two Crazyflie 2.1 vehicles in stacked / static-top flight
- One sentence: what this paper is NOT: not onboard INDI, not geometric control, not 3+ robots, not Neural-Swarm, not a docking paper. Compensation does not run on the STM32

## 1. Problem they pose

Their wording: "Flying quadrotors in tight formations is a challenging problem." Near-field propeller airflow is "complex and difficult to characterize." Pure first-principles models are "often overly simplified." Pure data-driven models are "significantly less sample efficient" and may need "onerous data collection." Reactive controllers "consider the aerodynamic effects or disturbances at the current time step, but neglect potential disturbances that may occur within the next few time steps." Prior learned MPC [14] is limited to "short-duration traversals" at "one particular speed and separation."

Plain restatement: they want a model of neighbor downwash that is accurate in the near field, cheap to train, and usable inside a receding-horizon optimizer so the bottom vehicle can anticipate the wake, not only react to it.

## 2. Paper map (follow their TOC)

### Abstract

- Purpose: headline claims
- Method: first-principles + lightweight data-driven residual, then learning-based NMPC
- Key equation: none numbered
- Assumption: not in this paper beyond "formation flight"
- Result claimed: hardware "40.1% improvement in the average trajectory tracking errors" and "57.5% reduction in the maximum vertical separation errors" vs nominal MPC; "only a total of 46 seconds of flight data for training across both simulations and physical experiments"; "average separation of less than 1.5 body lengths throughout the flight"

### I. Introduction

- Purpose: motivate close-proximity flight; related work; three contributions
- Method: position vs first-principles [9–11], data-driven [8,13,14], and reactive controllers [13,17]
- Key equation: none
- Assumption: stacked wake is the hard case because effects are "highly transient"
- Result claimed (body): sim MPC "comparable to an omniscient MPC"; hardware "average separation distance of less than 1.5 body lengths"; "never achieved before for a quadrotor team of this size and weight class" (authors' claim)

### II. Preliminaries

#### A. Quadrotor Dynamics and Aerodynamic Disturbances

- Purpose: write nominal 13-state dynamics and an analytical downwash map
- Method: rigid-body model (1)–(3). Flow frame $\mathcal{F}$ from top-vehicle body $z$ and $\Delta p$ (4). Velocity field from [11] as (5). Force/torque integrals from [9] as (6). Packed as $f_d$ in (7)
- Key equations: (1), (3), (4), (5), (6a–b), (7)
- Assumption: planform is a disk $A_b=\pi(\lambda/2)^2$. Model (7) "is unlikely to be highly accurate" in the near field. Flow model (5) "likely to provide accurate predictions only in the far-field" when vertical separation is "less than 2 body lengths"
- Result claimed: none experimental; this section only defines knowledge

#### B. Learning-based MPC

- Purpose: write the finite-horizon OCP
- Method: receding-horizon quadratic tracking cost with learned dynamics in the constraint
- Key equation: (8a–e)
- Assumption: state measurement $x(k)$ available; sets $\mathcal{X},\mathcal{X}_f,\mathcal{U}$ exist. Horizon length $N$ and weights $Q,R,P$ not given numerically
- Result claimed: none yet

### III. Model and Control Design

#### A. Model Formulation and Training

- Purpose: define residual neural ODE and training loss
- Method: $\Upsilon$-branch, $L$-layer tanh MLP. Inputs $[z,r,v_z^\mathcal{F}]^\top$ so the residual is axially symmetric about $\mathcal{F}_z$. Outputs rotated/scaled by $H$ and $M$ outside the net. Hybrid plant is (12). Train by one-step integrated prediction MSE (13)–(14) with Adam / PyTorch. No force or torque labels
- Key equations: (9a–c), (10), (11), (12), (13), (14)
- Assumption: velocity and orientation estimates are usable; IMU accel/torque reconstruction is not required
- Result claimed: architectural claim only (residual = leftover aero after $f_{\mathrm{nom}}+f_d$)

#### B. Incorporating into Learning-based MPC

- Purpose: put (12) into the OCP
- Method: discretize (12) with e.g. RK4 → constraint (15). Treat $x(k)$, $\{x_{r,i}\}$, $\{x_{\mathrm{top},i}\}$ as parameters. Top vehicle over the horizon: constant-velocity position propagation
- Key equation: (15)
- Assumption: top state known; constant-velocity neighbor forecast. Collision constraints "left as part of future work"
- Result claimed: they say prior KNODE-MPC [20] "does not necessarily lead to accurate closed-loop performance under all test cases"

### IV. Experiments and Results

- Purpose: five questions (prediction accuracy, closed-loop gain, generalization, sim-to-real, tightness)
- Baselines: nominal MPC ($f_{\mathrm{nom}}$); KNODE-MPC ($f_{\mathrm{nom}}+d$); DW-MPC ($f_{\mathrm{nom}}+f_d$); omniscient MPC (sim only); proposed KNODE-DW MPC ($f_{\mathrm{nom}}+f_d+d$)

#### A. Simulations

- Purpose: mismatched-$C_D,S$ plant vs controller; force/torque prediction + tracking
- Method: plant = (3)+(7), RK45, $\Delta t=5\,\mathrm{ms}$. Dist. integrals by first-order 2D trapezoid. acados + CasADi. Two-branch 1-layer residual, 9 neurons/branch. Train: 4 segments, 2 static-top + 2 stacked, 3532 points ≈ 18 s, nominal-MPC closed loop, 0.4 m/s, separations 0.3 m and 0.4 m. Tests also at 0.3 and 0.5 m/s and 0.35 m and 0.2 m
- Key numbers in this subsection: plant $C_D=1.3$, $S=0.0697$; controller $C_D=1.18$, $S=0.0997$; $W_x=\mathrm{diag}(I_{10},0.1 I_3)$
- Result claimed (body + Fig. 2–3): KNODE-DW "89.5% improvement over the nominal model, on average" in force/torque prediction. Closed loop: "reducing the average RMSEs and $z_{\max}$ by 89% and 96.5% over nominal MPC"; "comparable to omniscient MPC"

#### B. Physical Experiments

- Purpose: two Crazyflie 2.1 in the same two geometries, plus lemniscate and an extreme stack
- Method: body length 10 cm, mass ≈ 34 g. Bottom vehicle has thrust-upgrade kit. Intel i5 laptop = base station. Crazyradio PA "average rate of 400 Hz." Vicon pose "approximate rate of 120 Hz." CrazyROS. Same acados/CasADi stack "operates on the base station." Outputs thrust + angular-rate to onboard firmware PID. Hardware residual is force-only: $M:=M_{11}$. Single-branch 1-layer net, 4 hidden neurons. $C_D=0.236$ after manual sys-id so DW-MPC is "reasonable" on training cases. Train: static-top and stacked, 0.4 m/s, 0.4 m sep., 6720 points = 28 s under nominal MPC. Extra test: 0.4 m/s at 0.2 m. Five runs per test case
- Result claimed (body + Fig. 4–5 + Fig. 1): "average of 40.1% improvement in RMSE and 57.5% reduction in $z_{\max}$ over nominal MPC across the test cases." Lemniscate stacked at "commanded separation of 2 body lengths" (Fig. 1 caption), trajectory "not seen" in training. Extreme stack: commanded "0.8 body lengths (0.08 m)"; under KNODE-DW "average vertical separation … is 0.12 m, which is 57.1% smaller than … nominal MPC … 0.28 m"; they call 0.12 m "1.2 body lengths"; "average radial separation $r$ … is at 0.03 m"

### V. Conclusion

- Purpose: restatement + future UQ
- Result claimed: model and controller "highly accurate" for disturbance prediction and tracking. Future work: uncertainty quantification [30]

### VI. Acknowledgments

- Funding ONR N00014-22-1-2157 and DSO National Laboratories. Thanks Cowley (Fig. 1 photo) and Kunapuli (CF parts)

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Crazyflie 2.1; body length 10 cm; ≈ 34 g; bottom has thrust-upgrade bundle |
| n robots min/max | 2 / 2 |
| Sim / hardware / both | both |
| Onboard vs off-board (Hz) | NMPC on Intel i5 laptop. Radio to vehicles "average rate of 400 Hz." Onboard firmware = PID on thrust and angular rate. NMPC is not onboard |
| Sensing (mocap, RPM, GPS, …) | Vicon pose to base station "approximate rate of 120 Hz." Both vehicles' states assumed available to the MPC. RPM not stated |
| Controller family | nonlinear MPC (acados + CasADi), parametric in neighbor state |
| How interaction enters the law | predicted in the MPC dynamics: $f_{\mathrm{nom}}+f_d+d(\theta^\star)$. Hardware $d$ acts on force channels only |
| Learned model? inputs → output | yes. Residual MLP inputs $[z,r,v_z^\mathcal{F}]^\top$. Output: residual force (hw) or force+torque (sim), mapped into $\dot{x}$ by $M,H$ |
| Training data amount / source | sim 3532 pts ≈ 18 s; hw 6720 pts = 28 s; abstract total "46 seconds … across both." Source: closed-loop nominal MPC flights |
| Formations + min separation | static-top and stacked. Train sep. 0.3–0.4 m (sim), 0.4 m (hw). Tests down to commanded 0.2 m and 0.08 m (0.8 body lengths) |
| Speeds | sim 0.3 / 0.4 / 0.5 m/s; hw training and extra test 0.4 m/s. Accel not in this paper |
| Metrics | RMSE of bottom vehicle vs reference; $z_{\max}$ = longest distance between bottom vehicle and reference height; realized mean $z$ and $r$ |
| Baseline they compare to | nominal MPC, KNODE-MPC, DW-MPC; omniscient MPC in sim only |

## 4. Formations and flights (table, one row per experiment)

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Static-top | 2 | top vehicle fixed; bottom flies under it | commanded 0.3 m, 0.4 m train (sim); also 0.35 m, 0.2 m test (sim); hw train 0.4 m; hw also 0.2 m (Fig. 4 legend) | straight path; sim 0.3–0.5 m/s; hw 0.4 m/s | both | prediction heatmaps Fig. 2; tracking heatmaps Fig. 3; hw stats Fig. 4 |
| Stacked | 2 | both on same horizontal path; bottom in wake | same Δz grid as above | straight path; same speeds | both | same figures; they call this the hard case vs "crossing trajectories" |
| Crossing trajectories | 2 | 2D paths overlap "momentarily" | not in this paper | straight path | discussed only | they say it is "significantly more benign"; not a reported campaign |
| Stacked lemniscate | 2 | stacked while both fly vertical lemniscates | Fig. 1 caption: "commanded separation of 2 body lengths" | "vertical lemniscate trajectories"; speed not in this paper | hw only | qualitative + composite photo Fig. 1; "not seen" in training |
| Extreme stacked | 2 | stacked, bottom "well in the wake" | commanded 0.08 m = 0.8 body lengths | straight line; speed not restated in that paragraph | hw only | realized mean $z=0.12\,\mathrm{m}$ (they say 1.2 body lengths) vs nominal 0.28 m; mean $r=0.03\,\mathrm{m}$; time series Fig. 5 |

No V-stack, triangle, side-by-side, or 3-robot formation in this paper.

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Tracking improvement, hardware | 40.1% | "improvement in the average trajectory tracking errors" / "improvement in RMSE … across the test cases" | abstract and §IV.B | average over the hw test cases in Fig. 4, vs nominal MPC, 5 runs each |
| Max vertical-separation error, hardware | 57.5% | "reduction in the maximum vertical separation errors" / "reduction in $z_{\max}$" | abstract and §IV.B | same average-over-cases qualifier |
| Sample budget, combined | 46 s | "flight data for training across both simulations and physical experiments" | abstract | 18 s sim + 28 s hw. Do not treat 46 s as hardware-only |
| Sim training set | 3532 points ≈ 18 s | closed-loop nominal-MPC sim | §IV.A | 0.4 m/s; 0.3 m and 0.4 m sep.; 2+2 segments |
| Hw training set | 6720 points = 28 s | closed-loop nominal-MPC flight | §IV.B | 0.4 m/s and 0.4 m only |
| Contrast they draw to [14] | 570 s | training data used in [14], not in this method | §IV.B | do not cite 570 s as their budget |
| Sim prediction gain | 89.5% | improvement of KNODE-DW force/torque predictions "over the nominal model, on average" | §IV.A, Fig. 2 | sim, mismatched $C_D,S$ |
| Sim closed-loop RMSE | 89% | "reducing the average RMSEs … over nominal MPC" | §IV.A, Fig. 3 | heatmap is normalized to nominal = 100% |
| Sim closed-loop $z_{\max}$ | 96.5% | reduction of $z_{\max}$ vs nominal MPC | §IV.A | same |
| Body length | 10 cm | Crazyflie 2.1 | §IV.B | use this for all BL conversions in this paper |
| Mass | ≈ 34 g | each CF 2.1 | §IV.B | |
| Extreme commanded Δz | 0.08 m = 0.8 body lengths | commanded vertical sep. | §IV.B, Fig. 5 | commanded, not realized |
| Extreme realized mean Δz, proposed | 0.12 m | "average vertical separation between the drones" | §IV.B | they also write "1.2 body lengths" |
| Extreme realized mean Δz, nominal | 0.28 m | same quantity under nominal MPC | §IV.B | 57.1% smaller is 0.12 vs 0.28 |
| Extreme realized mean radial | 0.03 m | average $r$ under proposed | §IV.B | implies bottom vehicle stays in the wake |
| Abstract tightness | "less than 1.5 body lengths" | "average separation … throughout the flight" | abstract + intro | which flight is not named in the abstract. Extreme case realized 1.2 BL vertical. Fig. 1 commanded 2 BL |
| Radio | 400 Hz | "average rate" Crazyradio PA | §IV.B | comms rate, not proven NMPC solve rate |
| Mocap | ≈ 120 Hz | Vicon to base station | §IV.B | |
| Plant step (sim) | 5 ms | RK45 integration | §IV.A | |
| Hw $C_D$ | 0.236 | analytical force model after manual sys-id | §IV.B | not the same as sim $C_D$ |
| Runs per hw case | 5 | repeats | §IV.B | error bars = mean ± std |

Abstract and body agree on 40.1% and 57.5%. Body does not retract the abstract.

## 6. Assumptions and failure modes

- Two vehicles only. Bottom vehicle is the controlled plant. Top state is an exogenous parameter
- Neighbor forecast over the horizon is constant velocity
- Full state of both vehicles available to the laptop MPC (Vicon + assumption)
- Analytical $f_d$ is known to be wrong below 2 body lengths; residual is expected to patch it
- Hardware residual does not compensate torques. Sim residual does
- Collision constraints not implemented
- Training distribution is nominal-MPC flight at one speed (hw) and two separations (sim)
- KNODE-MPC (no $f_d$) "fails to provide consistent performance" on hw cases outside the training set (§IV.B)
- No UQ in the controller (they flag this as future work)
- External mocap + ground-station computer required for the method as run

## 7. Relation to our thesis (clearly labelled OUR READING)

- Which of our strategies it supports: none of 1–5 as implemented. It is the closest published neighbor to "learning-based MPC / KNODE-style," which is outside our five-controller campaign but is the right Related Work foil for residual-in-the-model vs residual-subtracted-from-the-command
- What we may cite: hybrid $f_{\mathrm{nom}}+f_d+d$ vs residual-only vs physics-only; stacked and static-top as hard tests; sample budget 18 s sim / 28 s hw; hardware gains 40.1% RMSE and 57.5% $z_{\max}$ vs nominal MPC; realized 0.12 m mean stack on CF 2.1; force-only residual on hardware because the inner loop is rate PID
- What we must NOT cite from it: onboard STM32 execution; OptiTrack (they use Vicon); Crazyflie brushless (they use CF 2.1, mass ≈ 34 g); 3-robot teams; INDI / geometric / FBL laws; "onboard NMPC"; our residual-subtraction architecture (theirs sits inside the prediction model, not as an $\hat F$ subtracted from the command)
- Formations we should reproduce if we use it as a reference condition: 2-robot static-top; 2-robot continuous stacked straight line; commanded Δz grid 0.4 m, 0.2 m, and an 0.8-BL stress case; optional stacked vertical lemniscate. Same path for both vehicles in stacked. Report RMSE, $z_{\max}$, realized mean $z$ and $r$
- Compute: would this run on an STM32? Not as published. They run acados/CasADi NMPC on an Intel i5 laptop and stream thrust + rate setpoints. Onboard compute is stock Crazyflie firmware PID. A frozen MLP residual of 4 hidden neurons could in principle run on an MCU. The optimizer they actually used would not

## 8. Equations worth keeping

(1) Nominal rigid body:

$$
\dot p=v,\quad
\dot v=-g+(1/m)R\tau,\quad
J\dot\omega=\alpha-\omega\times J\omega,\quad
\dot q=\Omega(q/2).
$$

$\tau=[0,0,\eta]^\top$. Compact form (3): $\dot x=f_{\mathrm{nom}}(x,u)$, $x=[p^\top,v^\top,q^\top,\omega^\top]^\top$, $u=[\eta,\alpha^\top]^\top$.

(4) Flow frame: $e_3=-e'_3$ (top body $z$), $e_2=(e_3\times\Delta p)/\|e_3\times\Delta p\|_2$, $e_1=e_2\times e_3$, $\Delta p=p-p_{\mathrm{top}}$.

(5) Relative airflow in $\mathcal{F}$ (from [11]; constants $U_H,B_d,z_0,S,\lambda$ not fully defined here):

$$
V(z,r)=\frac{U_H}{B_d}\frac{\tilde z-z_0}{\bigl(1+(\sqrt{2}-1)(\tilde r/(S(\tilde z-z_0)))^2\bigr)^2}-v_z^\mathcal{F}.
$$

(6) Disturbance force and torque, $\zeta=\tfrac12\rho V(z,r)^2$:

$$
\tau_d=\int_{A_b}e_3 C_D\zeta\,dA,\qquad
\alpha_d=\int_{A_b}(p_f-R_W^\mathcal{F}\Delta p)\times e_3 C_D\zeta\,dA.
$$

(7) Disturbance vector field $f_d(x,x_{\mathrm{top}})$ injecting $\tau_d/m$ and $J^{-1}R^\top\alpha_d$ into $\dot x$.

(8) NMPC: track $x_r$ with weights $Q,R,P$, dynamics $\hat f(\cdot;\theta^\star)$, receding horizon $N$.

(12) Hybrid model:

$$
\dot x(\theta)=f_{\mathrm{nom}}(x,u)+f_d(x,x_{\mathrm{top}})+d(x,x_{\mathrm{top}};\theta).
$$

(13)–(14) Train by weighted one-step integrated prediction error. No $\tau,\alpha$ labels.

(15) Discrete constraint used at runtime: $x_{i+1}=\hat f(x_i,u_i,x_{\mathrm{top},i}(k);\theta^\star)$.

## 9. Figures / tables worth remembering

- Fig. 1 top: composite photo, two CF 2.1 on stacked vertical lemniscates, commanded sep. 2 body lengths. Bottom: block diagram $f_{\mathrm{nom}}+f_d+d$ inside $\min J$ NMPC
- Fig. 2: heatmaps of force $\tau_d$ and torque $\alpha_d$ prediction RMSE for DW / KNODE / KNODE-DW, stacked and static-top, several speeds and separations, normalized to nominal
- Fig. 3: heatmaps of closed-loop RMSE and $z_{\max}$ for omniscient / DW / KNODE / KNODE-DW, all normalized to nominal MPC = 100%
- Fig. 4: hw mean ± std of RMSE and $z_{\max}$ for nominal, KNODE, DW, KNODE-DW. Legend cases include static-top 0.2 m, static-top 0.4 m, stacked 0.2 m, stacked 0.4 m
- Fig. 5: time series of $z$, $r$, and commanded thrust $T_{\mathrm{cmd}}$ for the 0.8-BL stacked straight-line case
- No numbered results table of raw meters; numbers live in text and in these figures

## 10. Open uncertainties

- Numerical $N$, $Q$, $R$, $P$, input limits, and NMPC sample time: not in this paper
- Exact laptop solve rate: not in this paper (only radio 400 Hz and Vicon ≈ 120 Hz)
- Lemniscate amplitude, period, plane, and speed: not in this paper
- Commanded horizontal offset in stacked/static-top: not in this paper except realized $r=0.03\,\mathrm{m}$ on the extreme run
- Values of $\lambda,U_H,B_d,z_0,\rho$ in (5): referred to [11], not printed
- Whether $\theta$ is frozen after the 28 s set or updated online: not in this paper
- Filter cutoffs, estimator, and firmware PID gains: not in this paper
- How "average separation of less than 1.5 body lengths throughout the flight" is computed (which flight, vertical vs Euclidean): formula not in this paper
- Conference vs journal version: this file is arXiv v1 only. Whether a shorter published version exists is not in this paper

## 11. 12-line abstract I can reread

Chee et al. 2024 put an analytical downwash map $f_d$ plus a small axially-symmetric KNODE residual $d(\theta)$ inside off-board NMPC. The plant is two Crazyflie 2.1 (10 cm, ≈ 34 g). Hard tests are static-top and continuous stacked wake, not brief crossings. Training is closed-loop nominal MPC: 18 s sim and 28 s hardware, 46 s combined. Hardware NMPC runs on an Intel i5 laptop and sends thrust + rate to firmware PID; the residual on hardware is force-only. Versus nominal MPC they report 40.1% mean RMSE improvement and 57.5% $z_{\max}$ cut on hardware, averaged over the Fig. 4 cases (5 runs each). Sim tracking approaches an omniscient MPC. An unseen stacked lemniscate at 2 body lengths commanded is shown in Fig. 1. An extreme stacked line at 0.08 m commanded yields 0.12 m mean vertical and 0.03 m mean radial under their controller, versus 0.28 m mean vertical under nominal MPC. Collision constraints and UQ are future work. This is not an onboard STM32 result and not a 3-robot result.

Source: attached PDF only. No other paper used.
