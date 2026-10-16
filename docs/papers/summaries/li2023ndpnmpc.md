# NDP-NMPC downwash — Li et al., 2023

## 0. Identity

- Full title: Nonlinear MPC for Quadrotors in Close-Proximity Flight with Neural Network Downwash Prediction
- Authors: Jinjie Li, Liang Han (corresponding), Haoyang Yu, Yuheng Lin, Qingdong Li, Zhang Ren. Beihang University.
- Year / venue / arXiv / DOI: 2023. 62nd IEEE Conference on Decision and Control (CDC). arXiv:2304.07794 [cs.RO]. DOI is not printed on this PDF. External IEEE record exists; do not treat the DOI as coming from this file.
- Version of THIS file: arXiv v2, 12 Sep 2023 (header on page 1). Comments on the preprint: accepted for the invited session "Control with Learning for Autonomous Robots" at CDC 2023. This file is 7 pages. Code/dataset link is in the PDF: https://github.com/Li-Jinjie/ndp_nmpc_qd
- Suggested cite key: `li2023ndpnmpc`
- What this paper IS: two similar ~1.53 kg custom PX4 quadrotors; spectrally normalised MLP predicts inertial disturbance force from relative position and velocity; that force sequence is injected as NMPC model parameters; hardware tracking with OptiTrack and an onboard TX2 NX.
- What this paper is NOT: not Crazyflie; not STM32; not INDI / NA-INDI / L1; not geometric control; not heterogeneous swarm; not 3+ robots; not a network used as the plant (they reject that, citing Matei et al.). Not Neural-Swarm2. They only cite Shi et al. as the FBL current-state baseline they want to beat with prediction.

## 1. Problem they pose

Their wording (Intro): downwash from "upper neighbors" is "highly nonlinear and hard to describe through mathematical modeling." Planning that treats each vehicle as a tall ellipsoid "reduces the reachable set." They want the disturbance in the control layer so planning only uses a collision radius.

They also say Shi et al. [5] put an NN in a "hierarchical feedback-linearization controller, which generates control inputs based on only the current states." Downwash "can be predicted through the exchange of reference trajectories." MPC can "look forward" and handle thrust saturation.

Plain restatement: learn the downwash force, feed a future sequence of that force into NMPC, so the lower vehicle prepares before the airflow hits and stays inside input limits.

## 2. Paper map (follow their TOC)

### I. Introduction

- Purpose: gap, related work, three contributions.
- Method / construction: none. Positioning only.
- Key equation: none numbered.
- Assumption: close vertical flight is desirable; downwash is a control problem, not only a planner keep-out.
- Result claimed: contributions (1) NDP-NMPC, (2) real-time tracking experiments, (3) open code and dataset. Abstract (not body) already states "75.37% reduction in tracking error in height."

### II. Methodology

Paper text says the workflow is in "Section III-A." That is a numbering slip. The blocks below are printed as II-A to II-E.

### II-A. System Overview

- Purpose: pipeline from waypoints to motors (Fig. 2).
- Method: Path Planner → min-snap Trajectory Generator → Trajectory Server (flat full state) → NMPC → hover-throttle map → PX4. States back from the autopilot.
- Key equation: none. Fig. 2 caption says the NN uses "the error state sequence of its own and other quadrotors." Body II-D uses relative position and velocity, not an "error state." Use the body definition.
- Assumption: neighbour reference trajectories are available to the ego NN over the horizon.
- Result: architecture claim only.

### II-B. Notation and Coordinate Systems

- Purpose: frames and quaternion algebra.
- Method: inertial ENU, body FLU, Hamilton quaternion, MAVROS/PX4 convention. Fig. 3.
- Key equation: rotation matrix $R(q)$ printed under II-B (unnumbered).
- Assumption: unit quaternion. Body origin at the CoM (stated in II-C).
- Result: notation only.

### II-C. Nominal Quadrotor Model

- Purpose: 6-DoF plant used by both the residual label and NMPC.
- Method: rigid-body translation and rotation. Rotor thrust along body $z$. Quadratic rotor map. Allocation matrix $G$.
- Key equations:
  - (1) ${}^{I}\dot{\mathbf{p}} = {}^{I}\mathbf{v}$
  - (2) ${}^{I}\dot{\mathbf{v}} = \bigl({}^{I}_{B}R(q)\cdot {}^{B}\mathbf{f}_{u} + {}^{I}\mathbf{f}_{d}\bigr)/m + {}^{I}\mathbf{g}$
  - (3) ${}^{I}_{B}\dot{q} = \tfrac{1}{2}\,{}^{I}_{B}q \circ \mathcal{V}^{*}({}^{B}\omega)$
  - (4) ${}^{B}\dot{\omega} = I^{-1}\bigl(-{}^{B}\omega \times (I\cdot {}^{B}\omega) + {}^{B}\boldsymbol{\tau}_{u} + {}^{B}\boldsymbol{\tau}_{d}\bigr)$
  - (5) $f_{i}=k_{t}\Omega^{2}$, $\tau_{i}=k_{q}\Omega^{2}$ ($\Omega$ in kRPM)
  - (6)–(7) allocation through $G(L,\alpha,k_{q}/k_{t})$
- Assumption: four rotors in the body XY-plane. Inertia diagonal (symmetry). Thrust purely vertical in body frame. ${}^{B}\boldsymbol{\tau}_{d}$ left to the PX4 body-rate loop. Only ${}^{I}\mathbf{f}_{d}$ is learned.
- Result: model used later. No flight result here.

### II-D. Neural Network Observer for Downwash Effect

- Purpose: MLP + spectral norm + how labels are built.
- Method: MLP (8). Input $\mathbf{x}=[{}^{I}\mathbf{p}_{\mathrm{rela}},{}^{I}\mathbf{v}_{\mathrm{rela}}]^{\top}$. Output $\mathbf{y}={}^{I}\hat{\mathbf{f}}_{d}$. Spectral norm on each $W^{l}$ every epoch (12), Lipschitz bound (13). Labels: measured $\Omega_{i}$ → nominal force (14); odometry velocity differentiated by Tustin → inertial force (15); residual (16). IMU force rejected as too noisy. Batch predict a sequence in flight.
- Key equations: (8)–(16). Lipschitz definition (9). Layer bound (10). Product bound (11).
- Assumption: identified physical model is accurate enough that (16) is the downwash force. Training data will not cover the full state space, so the Lipschitz cap is a safety device.
- Result: method only. $\gamma$ choice is in III-B.

### II-E. Nonlinear MPC with Network Disturbance Prediction

- Purpose: min-snap reference + NMPC that takes the predicted force sequence.
- Method: min-snap on flat outputs $[x,y,z,\psi]$. Time allocation = distance / user-defined average speed. Server publishes a horizon of references at the control rate. Each vehicle is assumed to run NMPC and share predictions. Predicted $\mathbf{f}_{d}$ are "parameters for the optimizer" (Fig. 4), i.e. they enter the discrete dynamics, not the cost and not as a disturbance constraint. Cost (17) is tracking + input effort. Constraints (18): RK4 dynamics of (1)–(3), $x_{0}=x_{\mathrm{init}}$, box on $u$. State $x_{k}=[{}^{I}p_{k},{}^{I}v_{k},{}^{I}q_{k}]^{\top}$. Input $u_{k}=[f_{c},{}^{B}\omega]^{\top}$. Quaternion cost (19). Warm-start, RTI, multiple shooting in ACADOS. First optimal input (20) sent out. Thrust then mapped to $[0,1]$ throttle.
- Key equations: (17)–(20).
- Assumption: neighbour planned trajectory is known over the horizon. Dynamics in the solver are (1)–(3) only; (4) is not in the NMPC state. Discrete model is RK4 of that reduced model plus the NN force.
- Result: algorithm. Timing numbers are in Table III / §III-C.

### III. Experiments

Preamble: ID, then data/training, then open-loop prediction, then closed-loop tracking.

### III-A. Parameter Identification for Quadrotors

- Purpose: airframe and Table I.
- Method: two self-made vehicles (Fig. 5a). DShot RPM. Load-cell pair-rotor thrust/torque ID, including body-on-rotor flow. Bifilar pendulum for inertia. Parameters used in PX4 SITL and in control.
- Key equation: none new. Table I values listed in §5 and §8.
- Assumption: three-axis symmetry for inertia.
- Result: $m=1.5344\,\mathrm{kg}$, $L=0.1372\,\mathrm{m}$, $\alpha=45^{\circ}$, thrust/weight $=4.3100$, $\Omega\in[2.6,24.0]$ kRPM, listed flight time $705\,\mathrm{s}$ (endurance, not a test duration).

### III-B. Data Collection, Training, and Testing

- Purpose: dataset, MLP hyperparameters, choose $\gamma$.
- Method: two pilots. Lower vehicle hover. Upper vehicle flown over it at several heights. 100 Hz rosbag. "A total of 570-second data were collected for both quadrotors," doubled by symmetry. Hover-bias removed. Shuffle 0.75 / 0.25 train/test. Table II: layers 6-128-64-128-3, normal init, ReLU, Adam, 20 000 epochs, lr $1\times 10^{-4}$, MSE. Fig. 6: predicted $Z$ force on a $1\,\mathrm{m}\times 1\,\mathrm{m}$ XY patch, zero relative velocity, several relative heights.
- Key equation: none new.
- Assumption: two similar vehicles so data may be pooled. Points with no training coverage must not explode.
- Result: $\gamma=2$ too conservative (Loss=0.518, L=1.022 on the figure labels). $\gamma=4$ chosen. No-$\gamma$ ($\gamma=\infty$) "suddenly increases at 1.3 m due to the lack of data." Fig. 6 relative heights shown: $-0.5, 0.5, 0.7, 0.9, 1.1, 1.3, z=1.5\,\mathrm{m}, 2$. Negative height means the other vehicle is above.

### III-C. Trajectory Tracking under Downwash Effect

- Purpose: closed-loop hardware vs NMPC without the predictor.
- Method: Python + ROS. Onboard TX2 NX: trajectory server, NDP-NMPC, hover-throttle estimator. Desktop PC: trajectory generation, MoCap broadcast, ROS master. ACADOS, Table III. Room "$7\times 4\times 3$" with OptiTrack. Two vehicles, same side, different altitudes, back-and-forth with an overlap zone. "Multiple runs." Fig. 7: mean of three rounds, shaded min/max. Baseline = NMPC without disturbance prediction, cited as [25] Falanga PAMPC.
- Key equation: none new.
- Assumption: MoCap is the localisation source. Embedded compute and latency are the relevant limits.
- Result (body): "noteworthy 75.37% reduction in the tracking Root-Mean-Square Error (RMSE)." Conclusion restates this as "reduced 75.37% tracking error in the Z axis." Fig. 7 also prints four RMSE numbers (see §5). They report a false-positive: the lower vehicle "suddenly jumps before it enters the downwash area" because the net predicts a force that is not there yet. Horizontal fluctuations in Fig. 7 from the same error.

### IV. Conclusion

- Purpose: recap and future work.
- Method: none.
- Key equation: none.
- Assumption: none new.
- Result claimed again: network "robust"; "75.37% tracking error in the Z axis." Future: compare more cleanly to non-prediction methods; large-scale swarms; put prediction uncertainty inside NMPC. Those three are not done in this paper.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Two self-made quadrotors. $m=1.5344\,\mathrm{kg}$. Arm $L=0.1372\,\mathrm{m}$, $\alpha=45^{\circ}$. PX4 + DShot ESCs. No commercial name. |
| n robots min/max | 2 and 2. No 3+ test. |
| Sim / hardware / both | ID parameters "employed for both PX4 SITL simulation and control." Reported tracking is hardware. Open-loop prediction plots from the trained net (Fig. 6). |
| Onboard vs off-board (Hz) | NDP-NMPC on TX2 NX. $dt_{\mathrm{nmpc}}=1/60\,\mathrm{s}$. Prediction parameter $dt_{\mathrm{pred}}=0.1\,\mathrm{s}$. Trajectory generation and MoCap on a desktop PC. PX4 runs the body-rate loop. Control publication rate = NMPC rate. Training logs at 100 Hz. |
| Sensing (mocap, RPM, GPS, …) | OptiTrack for flight. DShot motor speed for training labels. Autopilot odometry velocity for (15). IMU force rejected. |
| Controller family | Nonlinear MPC (ACADOS, RTI, multiple shooting) + PX4 rate loop. Min-snap + differential flatness for the reference. |
| How interaction enters the law | Additive inertial force ${}^{I}\mathbf{f}_{d}$ in (2). NN output is a parameter sequence in the NMPC model. Not a cost term. Not a state constraint on the disturbance. Torque residual not in the NN. |
| Learned model? inputs → output | Yes. MLP 6-128-64-128-3. Rel. pos. (3) + rel. vel. (3) → force (3). Spectral norm, $\gamma=4$. |
| Training data amount / source | 570 s on two vehicles @ 100 Hz, poolable by similarity. Piloted overlap, lower vehicle stationary. 75/25 split. Hover bias subtracted. |
| Formations + min separation | Two vehicles, different altitudes, overlapping XY, back-and-forth. Numeric $\Delta z$ and min separation: not in this paper. |
| Speeds | "User-defined average velocity" for min-snap timing. Numeric speed: not in this paper. |
| Metrics | Z (and plotted Y) RMSE. Fig. 7 mean ± range over three rounds. Open-loop force maps and training loss vs $\gamma$. |
| Baseline they compare to | Same NMPC without the disturbance prediction, cited as Falanga et al. PAMPC [25]. Not INDI. Not geometric. Not Shi FBL on this airframe. |

## 4. Formations and flights (table, one row per experiment)

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Data collection | 2 | lower hover, upper flown over it | several heights; numeric list only in Fig. 6 grid, not a commanded formation | piloted, speed not given | hw, 100 Hz bags | 570 s raw; labels via (14)–(16) |
| Open-loop prediction maps (Fig. 6) | 2 in the model input | other vehicle relative to ego, zero rel. vel. | plotted relative z: −0.5, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 2 m | none (static grid) | net evaluation, not a flight | Z-force field on 1 m × 1 m; $\gamma$ study |
| Closed-loop tracking (Fig. 7) | 2 | same side, different altitudes, XY overlap | Δz not in this paper | back-and-forth; speed not in this paper | hw, OptiTrack, room 7×4×3 | 3-round mean; 75.37% Z RMSE cut vs no-prediction NMPC; pre-entry jump |

No other formations in this file. Do not import Neural-Swarm stacks.

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Height tracking improvement | 75.37% | "reduction in tracking error in height" (abstract); "reduction in the tracking Root-Mean-Square Error (RMSE)" (III-C); "reduced 75.37% tracking error in the Z axis" (IV) | abstract, body III-C, conclusion | Same number in all three. Body does not retract the abstract. It is Z RMSE vs no-prediction NMPC, lower vehicle under downwash, 3-round mean. Not XY. Not vs INDI or FBL. |
| Fig. 7 Z RMSE pair | 0.0544 m and 0.2209 m | printed on the z-panel of Fig. 7 | figure | $1-0.0544/0.2209=0.7537$. This pair is the 75.37%. Assign 0.2209 m to baseline, 0.0544 m to NDP-NMPC. |
| Fig. 7 other RMSE pair | 0.0777 m and 0.0445 m | printed on the y-panel of Fig. 7 | figure | Body never names these two numbers. Do not call them Z. Do not fold them into 75.37%. |
| Mass | 1.5344 kg | identified vehicle mass | Table I | Not a Crazyflie. |
| Arm / angle | $L=0.1372\,\mathrm{m}$, $\alpha=45^{\circ}$ | geometry in $G$ | Table I, Fig. 3 | |
| Inertia | $I_{xx}=0.0094$, $I_{yy}=0.0134$, $I_{zz}=0.0145$ kg·m² | bifilar pendulum | Table I | Symmetry assumed. |
| Rotor maps | $k_{t}=2.8158\times 10^{-2}$ N/kRPM², $k_{q}=3.7611\times 10^{-4}$ N·m/kRPM² | load cell, diagonal pair | Table I | $\Omega$ in kRPM. |
| RPM box / T/W | [2.6, 24.0] kRPM; thrust/weight 4.3100 | identified limits | Table I | |
| Table I "flight time" | 705 s | platform endurance | Table I | Not the length of the tracking test. |
| Dataset | 570 s, both vehicles, 100 Hz; 0.75/0.25 split | rosbag duration | III-B | "Doubled due to the similarity of drones" is their claim, not extra recorded seconds. |
| $\gamma$ | 4 | spectral-norm scale | III-B | Also plotted: $\gamma=2$ and $\gamma=\infty$. |
| MLP | 6-128-64-128-3, 20 000 epochs, lr $1\mathrm{e}{-4}$ | Table II | body | Input dim 6, output dim 3. |
| NMPC | $N=20$, $dt_{\mathrm{nmpc}}=1/60\,\mathrm{s}$, $dt_{\mathrm{pred}}=0.1\,\mathrm{s}$ | Table III | body | Horizon length in seconds = $20/60\approx 0.33\,\mathrm{s}$. They do not write that product. |
| Weights | $Q_{p,xy}=300$, $Q_{p,z}=400$, $Q_{v,xyz}=1$, $Q_{q,xyz}=0.1$, $R_{\omega,xyz}=10$, $R_{f_{c}}=10$ | Table III | body | "High weights to positions." |
| Room | 7 × 4 × 3 | flight volume | III-C | Unit not printed. Metres are implied by the rest of the paper. |
| Repeats | three rounds | Fig. 7 caption | figure / III-C | Shaded band is the range, not a std. |
| Fig. 6 patch | 1 m × 1 m | XY window for predicted Z force | Fig. 6 caption | Zero relative velocity by construction. |

## 6. Assumptions and failure modes

- Identified nominal model is accurate enough that (16) isolates downwash.
- Only force residual is learned. Torque residual is left to PX4 rate control.
- Two vehicles are similar enough to pool data.
- Each vehicle runs NMPC and can share future references, otherwise the horizon prediction is empty.
- NMPC plant is (1)–(3) plus $\mathbf{f}_{d}(k)$, RK4, not the full (4).
- Spectral norm + $\gamma=4$ is enough for uncollected states. They show $\gamma=\infty$ blowing up at 1.3 m.
- Training coverage is incomplete. They say so.
- Failure they report: predictor fires early → lower vehicle climbs before the overlap, then is pushed back; extra XY wiggle (III-C).
- IMU-based force labels rejected (noise).
- Hover-bias subtraction: residual is relative to hover, not to a raw force sensor.
- Uncertainty of the NN is not inside the NMPC (IV, future work).
- Large swarms not shown (IV).

## 7. Relation to our thesis (clearly labelled OUR READING)

- Which of our strategies it supports: closest in spirit to **3 geo+NN residual** (learned force/acceleration residual, then subtracted or injected). Mechanism is not geometric: the residual goes into **NMPC dynamics**. It also motivates why we might beat a **current-state FBL+residual (4)** with a preview of the neighbour. It does **not** support **2 full INDI** or **5 NA-INDI/L-INDI**. It is not a geometric baseline paper (**1**).
- What we may cite: SN-MLP downwash force from relative $p,v$; inject predicted force into a preview controller; two-vehicle stacked overlap as a test idea; OOD blow-up without Lipschitz control; the reported early-trigger failure; hardware evidence that a force residual helps Z tracking vs the same controller without it.
- What we must NOT cite from it: Crazyflie results; STM32 timing; 3-robot teams; INDI/L1 numbers; heterogeneous airframes; "onboard" as if MoCap and planning ran on the vehicle; 75.37% as a number against our baselines; any formation spacing or speed; Matei-style network-as-dynamics; Shi Neural-Swarm2 data.
- Formations we should reproduce if we use it as a reference condition: two vehicles, different height, XY overlap, back-and-forth. Pick and **state our own** $\Delta z$ and speed. Theirs are not in the file.
- Compute: as written, no. Python ACADOS NMPC + MLP batch on a **TX2 NX**, horizon $N=20$ at 60 Hz, plus PX4 below. Our STM32 stack is a different computer class. Off-board pieces they actually used: desktop planner, OptiTrack, ROS master.

## 8. Equations worth keeping

Symbols: $m$ mass; ${}^{I}\mathbf{g}=[0,0,-g]^{\top}$; ${}^{B}\mathbf{f}_{u}=[0,0,f_{c}]^{\top}$; $\Omega$ motor speed in kRPM.

- (2) force channel where the residual lives:

$$
{}^{I}\dot{\mathbf{v}}=\bigl({}^{I}_{B}R(q)\cdot{}^{B}\mathbf{f}_{u}+{}^{I}\mathbf{f}_{d}\bigr)/m+{}^{I}\mathbf{g}
$$

- (5) rotor map used for labels:

$$
f_{i}=k_{t}\Omega^{2},\quad \tau_{i}=k_{q}\Omega^{2}
$$

- (8) MLP:

$$
f(\mathbf{x};\theta)=W^{H+1}\phi(\cdots\phi(W^{1}\mathbf{x}+b^{1})\cdots)+b^{H+1}
$$

- (12)–(13) spectral norm they actually train:

$$
\overline{W}^{l}=\gamma\,W^{l}/\sigma(W^{l}),\qquad \|f\|_{\mathrm{Lip}}\le\gamma^{H+1}
$$

- (14)–(16) label pipeline:

$$
{}^{I}\mathbf{f}_{n}={}^{I}_{B}R(q)\cdot{}^{B}\mathbf{f}_{u}+m\cdot{}^{I}\mathbf{g}
$$

$$
{}^{I}\mathbf{f}=m\cdot{}^{I}\dot{\hat{\mathbf{v}}}
$$

$$
{}^{I}\mathbf{f}_{d}={}^{I}\mathbf{f}-{}^{I}\mathbf{f}_{n}
$$

- (17)–(18) NMPC:

$$
\min_{u_{k}}\;\bar{x}_{N}^{\top}Q_{N}\bar{x}_{N}+\sum_{k=0}^{N-1}\bigl(\bar{x}_{k}^{\top}Q\bar{x}_{k}+\bar{u}_{k}^{\top}R\bar{u}_{k}\bigr)
$$

$$
x_{k+1}=f(x_{k},u_{k}),\quad x_{0}=x_{\mathrm{init}},\quad u_{k}\in[u_{\min},u_{\max}]
$$

- (20) applied command:

$$
u_{\mathrm{NDP\text{-}NMPC}}=u_{0}^{*}=[f_{c}',\omega_{x}',\omega_{y}',\omega_{z}']
$$

## 9. Figures / tables worth remembering

- Fig. 1: two-vehicle hardware snapshot. Left "Baseline," right "Our Method." Lower vehicle under the upper one, left-to-right track.
- Fig. 2: planner → min-snap → server → NMPC(+NN) → throttle → PX4.
- Fig. 3: ENU / FLU frames and rotor numbering.
- Fig. 4: one NMPC iteration. Warm start, $x_{0}$ pinned, NN disturbances as parameters, ACADOS.
- Fig. 5: (a) two vehicles with companion computers; (b) load-cell ID rig.
- Fig. 6: predicted Z force vs XY at several relative heights, three $\gamma$ rows. X/Y force "negligible" so not plotted.
- Fig. 7: closed-loop $z(x)$ and $y(x)$. Mean of 3 runs, shaded range. Grey band = "Strong Downwash Effect Aera" (their spelling). Four RMSE labels (see §5).
- Table I: identified parameters.
- Table II: MLP training settings.
- Table III: NMPC weights and timings.

## 10. Open uncertainties

- Commanded $\Delta z$, lateral offset, and min separation in the tracking test: not in this paper.
- Numeric flight speed / average velocity used in min-snap: not in this paper.
- Which exact segment the 75.37% uses (whole lap vs grey band only): body says "under the downwash effect" in the abstract and points at Fig. 7; it does not write the integration window.
- What 0.0777 m and 0.0445 m are, in words: not in this paper. Only printed on Fig. 7.
- NMPC input bounds $u_{\min},u_{\max}$ numeric values: not in this paper (RPM box is in Table I, not copied into Table III).
- Hover-throttle estimator: KF vs calibration map. Both mentioned. Which one flew: not in this paper.
- Filter / differentiator settings for Tustin $\dot{v}$ besides the method name: not in this paper.
- TX2 NX CPU timing / solve time: not in this paper. They only say real-time on that board.
- Whether SITL tracking numbers exist: parameters were used in SITL; no SITL RMSE in this file.
- Room units and OptiTrack rate: not in this paper.
- Yaw profile during the overlap: not in this paper.
- How neighbour states are transported (which ROS topics, delay): not in this paper.
- DOI / CDC page numbers: not printed on this PDF.

## 11. 12-line abstract I can reread

Li, Han et al., CDC 2023 arXiv v2. Two similar 1.53 kg PX4 quads, OptiTrack, NMPC on a TX2 NX.
MLP 6-128-64-128-3 maps relative position and velocity to inertial force. Spectral norm, $\gamma=4$.
Labels = odometry mass×acceleration minus model force from DShot RPM. 570 s piloted overlap, lower vehicle still.
Force sequence is an NMPC parameter, not a cost term. Plant in the solver is (1)–(3)+RK4. Torque residual stays in PX4.
Baseline is the same NMPC with the predictor off, cited as Falanga [25].
Hardware back-and-forth, two heights, overlap. Three-run mean. They quote 75.37% less Z RMSE.
Fig. 7 numbers that match that percent: 0.2209 m → 0.0544 m.
They also show the net can fire early and lift the lower vehicle before the real downwash.
Not Crazyflie, not STM32, not INDI, not 3 robots, not a spacing or speed you can copy.
Cite for predicted force-into-NMPC and for the SN safety plot. Do not steal their 75.37% into our campaign.

Source: attached PDF only. No other paper used.
