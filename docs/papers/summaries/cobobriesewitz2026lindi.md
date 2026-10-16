# LINDI / NA-INDI — Cobo-Briesewitz et al., 2026

## 0. Identity

- Full title: Learned Incremental Nonlinear Dynamic Inversion for Quadrotors with and without Slung Payloads
- Authors: Eckart Cobo-Briesewitz, Khaled Wahba, Wolfgang Hönig
- Year / venue: 2026, Proceedings of Machine Learning Research, 8th Annual Conference on Learning for Dynamics and Control (L4DC). Pages printed as 1–15. Volume number printed as "vol vvv".
- arXiv: 2503.09441v2 [cs.RO], 14 Apr 2026
- DOI: not in this paper
- Version of THIS file: arXiv v2 / L4DC-style 15-page PDF with Appendix A–B (full cable dynamics + unused geometric-INDI). Not a journal version.
- Suggested cite key: `cobobriesewitz2026lindi`
- What this paper IS: a single-Crazyflie study that trains a small MLP on offline-smoothed INDI residuals so runtime residual compensation does not need rotor RPM, plus a hybrid that still uses INDI on the leftover mismatch. Also tested with one slung point-mass payload.
- What this paper is NOT: not a multi-robot or downwash paper. Not Neural-Swarm. Not online adaptation (GP / meta-learning INDI). Not FBL. Not MPC. Not an unknown-payload estimator.

## 1. Problem they pose

Their wording: conventional controllers "model a large part of these forces but are unable to model all of them." The rest are residual forces (blade flapping, drag, ground forces). Direct computation is "computationally complex and intractable for real time control." INDI estimates residuals from model–sensor mismatch but "typically relies on rotor RPM measurements" and "may still be affected by measurement noise." Slung cables add "new types of unmodeled residual forces."

Plain restatement: they want INDI-level residual compensation without wearing an RPM board in flight, including when a small hanging mass is attached.

## 2. Paper map (follow their TOC)

### 1. Introduction

- Purpose: state the residual-force problem, INDI's RPM cost, and the two proposals.
- Construction: three blocks in Fig. 1: (a) INDI with RPM, (b) LINDI with NN only, (c) NA-INDI = LINDI + RPM-INDI.
- Equations: none numbered here.
- Assumption: residuals exist and can be treated as additive $\mathbf{f}_a,\boldsymbol{\tau}_a$.
- Claim (abstract + intro): a network can give "smooth approximations of INDI outputs without requiring specialized rotor RPM sensor inputs." Hybrid "integrates learning-based predictions with INDI."

### 2. Related Work

- Purpose: place INDI, residual NNs, online INDI+learning, and slung-load learning.
- Construction: literature only. Closest contrasts they name: Bauersfeld et al. (residual NN), Shi et al. (downwash DeepSets), Ignatyev & Tsourdos (online GP), Zhang & Ran (meta-learning INDI matrix), Jin et al. (unknown payload).
- Their difference, in their words: model "trained offline and directly predicts INDI outputs," no special sensors at runtime, "two different quadrotor modalities."
- Assumption: those prior works are the right neighbors.
- Result: none experimental.

### 3. Problem Description

- Purpose: write the two plants and the tracking objective.
- Free rigid-body Newton–Euler, action = rotor angular velocities $\mathbf{u}=[r_1,r_2,r_3,r_4]^\top$.
- Payload: point mass on a cable from the CoG. Tension from payload acceleration.
- Control problem (7): minimise mean tracking distance over a trajectory. Free flight uses vehicle position. Payload case uses payload position.
- Assumption: cable taut, attached at CoG, masses known. Residuals ignored by the nominal controller unless added later.

Key equations:

$$
\dot{\mathbf{p}}=\mathbf{v},\qquad
m\dot{\mathbf{v}}=f_u\mathbf{R}\mathbf{e}_z-mg\mathbf{e}_z+\mathbf{f}_a \tag{1}
$$

$$
\dot{\mathbf{R}}=\mathbf{R}\hat{\boldsymbol{\omega}},\qquad
\mathbf{J}\dot{\boldsymbol{\omega}}=\mathbf{J}\boldsymbol{\omega}\times\boldsymbol{\omega}+\boldsymbol{\tau}_u+\boldsymbol{\tau}_a \tag{2}
$$

$$
f_i=\kappa_F r_i^2,\qquad
[f_u,\boldsymbol{\tau}_u]^\top=\mathbf{B}_0[f_1,f_2,f_3,f_4]^\top \tag{3a,b}
$$

$$
m\dot{\mathbf{v}}=f_u\mathbf{R}\mathbf{e}_z-mg\mathbf{e}_z+T\mathbf{q}+\mathbf{f}_a \tag{4}
$$

$$
T=-m_p\mathbf{q}^\top(\dot{\mathbf{v}}_p+g\mathbf{e}_z) \tag{6}
$$

### 4. Approach

- Purpose: define geometric laws, INDI, LINDI, NA-INDI.

### 4.1 Geometric control laws

- Lee et al. (2010) for free flight. Yu et al. (2020) cascade for payload.
- New terms: subtract $\mathbf{f}_a$ from the desired force, subtract $\boldsymbol{\tau}_a$ from the desired torque.

$$
f_u=(-K_p\mathbf{e}_p-K_v\mathbf{e}_v+mg\mathbf{e}_z+m\ddot{\mathbf{p}}_d-\mathbf{f}_a)\cdot\mathbf{R}\mathbf{e}_z \tag{8a}
$$

$$
\boldsymbol{\tau}_u=-K_R\mathbf{e}_R-K_\omega\mathbf{e}_\omega-\mathbf{J}\boldsymbol{\omega}\times\boldsymbol{\omega}
-\mathbf{J}(\hat{\boldsymbol{\omega}}\mathbf{R}^\top\mathbf{R}_d\boldsymbol{\omega}_d-\mathbf{R}^\top\mathbf{R}_d\dot{\boldsymbol{\omega}}_d)-\boldsymbol{\tau}_a \tag{8b}
$$

$$
f_u=(\mathbf{u}_0-\mathbf{f}_a)\cdot\mathbf{R}\mathbf{e}_z \tag{9}
$$

- Assumption: cable at CoG so rotational law stays (8b).

### 4.2 INDI

- Recover residuals from IMU + RPM + (payload) tension.

$$
\mathbf{f}_a=m\dot{\mathbf{v}}-f_u\mathbf{R}\mathbf{e}_z+mg\mathbf{e}_z-T\mathbf{q} \tag{10a}
$$

$$
\boldsymbol{\tau}_a=\mathbf{J}\dot{\boldsymbol{\omega}}-\mathbf{J}\boldsymbol{\omega}\times\boldsymbol{\omega}-\boldsymbol{\tau}_u \tag{10b}
$$

- $f_u,\boldsymbol{\tau}_u$ from RPM via (3). $\dot{\mathbf{v}}$ from accelerometer, rotated to world. $\boldsymbol{\omega}$ from gyro. $\dot{\boldsymbol{\omega}}$ and $\dot{\mathbf{v}}_p$ by numerical differentiation. Online filter: Butterworth (they cite Tal & Karaman, cutoff not given).
- They keep tension coupling (4), not the full cable-ODE INDI of the appendix.
- Assumption: RPM is an accurate thrust proxy. Filters are required.

### 4.3 LINDI

- Offline: compute (10) on logged flights. Smooth labels with cubic spline segments that minimise $L_2$ to the points (not exact interpolation). Train an MLP to map ordinary sensors to those labels.
- Free net: 19 → 24 → 24 → 6, Leaky-ReLU. Inputs: $\mathbf{v},\dot{\mathbf{v}},\boldsymbol{\omega}$, first two columns of $\mathbf{R}$, motor PWM (not $r_i$). Output $[\mathbf{f}_a,\boldsymbol{\tau}_a]^\top\in\mathbb{R}^6$. Min-max to $[-1,1]$.
- Payload net: 28 inputs (adds payload velocity, acceleration, cable direction). Hidden width 16.
- Assumption: PWM + kinematics predict the same residual INDI would have computed from RPM. Domain of training trajectories is enough.

### 4.4 NA-INDI

- Split: $\mathbf{f}_a=\mathbf{f}_{a,\mathrm{NN}}+\mathbf{f}_{a,\mathrm{INDI}}$ and same for $\boldsymbol{\tau}_a$.
- NN is the §4.3 network. INDI only estimates the leftover mismatch.
- They do not say "bulk." That word is ours.

### 5. Experiments

- Purpose: closed-loop tracking on two held-out paths, free and with payload.

### 5.1 Physical setup

- Bitcraze Crazyflie 2.1. Arm length 4.6 cm. Mass 34.7 g. STM32 168 MHz, 1 MB flash, 192 KB RAM.
- Add-ons: micro-SD logger, custom IR RPM board (also an OptiTrack active marker).
- OptiTrack 100 Hz. Room 7.5 m × 4 m × 2.75 m. Crazyswarm2 / ROS 2.
- Payload: 5 g on 0.5 m string, magnet at CoG, IR LED on the mass. Size and mass fixed.

### 5.2 Data and training

- Random points in a 1.6 × 1.6 × 0.4 m³ box → splines. Speed up to 2 m/s free, 1.75 m/s payload.
- 64,996 timestamps (~12 min) free. 58,538 payload.
- Adam, $L_1$, initial LR $3\cdot10^{-1}$, ×0.92 every 10 epochs, 128 epochs, batch 512.
- Inference "203 microseconds on the Crazyflie hardware."
- No architecture ablation. They say the goal is sensor replacement, not net tuning.

### 5.3 Testing

- Two paths not in the training set: Figure8 and Circle (Fig. 4).
- Metric: mean of (7). Free: vehicle $\|\mathbf{p}-\mathbf{p}_r\|_2$. Payload: payload position error.
- Free max speed 2 m/s. Payload "around 1.75 m/s."

### 5.4 Results

- PWM-INDI ablation (Table 1, 10 trials, no payload): Lee 7.52 / 7.29 cm vs INDI-PWM 18.27 / 18.89 cm. Learning those bad labels still loses to Lee. Their point: INDI needs real RPM.
- No payload (Fig. 5, 20 trials each): "residual prediction can reduce the tracking error … by around 40%." Means (cm): Figure8 Lee 7.07, INDI 4.28, LINDI 4.41, NA-INDI 4.14. Circle Lee 7.45, INDI 4.33, LINDI 4.87, NA-INDI 4.26.
- LINDI "similar performance to INDI" without RPM. Circle: "slight drop … likely because the trajectory is out of distribution." That sentence is LINDI, no-payload only.
- NA-INDI "best performance, although with a small margin."
- Payload (Fig. 6, 10 trials): Figure8 Lee 7.06, INDI 5.38, LINDI 5.46, NA-INDI 5.39. Circle Lee 7.26, INDI 5.91, LINDI 5.78, NA-INDI 5.74. "LINDI is able to match … INDI." "NA-INDI … remains similar in performance."

### 6. Conclusion

- LINDI can replace INDI at runtime. Smoother residuals. One RPM-equipped vehicle for data, then copies without RPM boards.
- Future they name: external aerodynamic disturbances. They did not test those.

### Appendix A–B

- Full Hamilton cable dynamics (11) and a geometric INDI with $\mathbf{f}_{a_p},\mathbf{f}_{a_c}$.
- They did not fly this law. Reason they give: $\dot{\boldsymbol{\Omega}}$ from double differentiation of $\mathbf{q}$ was too noisy or too delayed. STM32 EKF "computationally demanding."
- Main-text INDI is the practical compromise: IMU already sees cable force through tension.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Crazyflie 2.1, arm 4.6 cm, 34.7 g |
| n robots min/max | 1 / 1 |
| Sim / hardware / both | Hardware only |
| Onboard vs off-board (Hz) | Residual MLP onboard, 203 µs. Pose from OptiTrack 100 Hz. Control-loop rate not in this paper |
| Sensing (mocap, RPM, GPS, …) | OptiTrack; IMU; custom IR RPM board for INDI and for labels; PWM always; payload LED |
| Controller family | Geometric SE(3) (Lee 2010) / payload cascade (Yu 2020) + additive residual |
| How interaction enters the law | Inter-vehicle interaction does not. Self residuals and cable tension do. $\mathbf{f}_a,\boldsymbol{\tau}_a$ subtracted in (8)–(9) |
| Learned model? inputs → output | MLP. Free: 19 → 6. Payload: 28 → 6. Output residual force and torque |
| Training data amount / source | 64,996 free / 58,538 payload timestamps from RPM-INDI on random indoor splines |
| Formations + min separation | not in this paper |
| Speeds | up to 2 m/s free; ≈1.75 m/s payload |
| Metrics | mean position error (7), cm, mean ± trial spread in Table 1 / violin captions |
| Baseline they compare to | Lee geometric; classical RPM-INDI; PWM-INDI ablation |

## 4. Formations and flights (table, one row per experiment)

No multi-robot formation exists in this file. Do not import stacks from other papers.

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Free tracking | 1 | single vehicle | — | Figure8, Circle / ≤2 m/s | hw | mean vehicle position error, 20 trials |
| Slung payload tracking | 1 | 5 g on 0.5 m string at CoG | cable 0.5 m vertical when hanging | Figure8, Circle / ≈1.75 m/s | hw | mean payload position error, 10 trials |
| Training flights | 1 | random splines in 1.6×1.6×0.4 m³ box | — | ≤2 / 1.75 m/s | hw | labels only, not a reported metric |
| PWM-INDI ablation | 1 | free | — | Circle, Figure8 | hw | Table 1, 10 trials |

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| "around 40%" error cut | ~40% | no-payload residual methods vs Lee | body §5.4.2 | not payload; payload cut is ~19–24% from their means |
| Figure8 free means | 7.07 / 4.28 / 4.41 / 4.14 cm | Lee / INDI / LINDI / NA-INDI | Fig. 5 caption | 20 trials. Vehicle position |
| Circle free means | 7.45 / 4.33 / 4.87 / 4.26 cm | same four | Fig. 5 caption | LINDI worse than INDI here |
| Figure8 payload means | 7.06 / 5.38 / 5.46 / 5.39 cm | same four | Fig. 6 caption | 10 trials. Payload position |
| Circle payload means | 7.26 / 5.91 / 5.78 / 5.74 cm | same four | Fig. 6 caption | LINDI slightly better than INDI |
| PWM-INDI Circle / Figure8 | 18.27±0.97 / 18.89±1.82 cm | INDI using PWM not RPM | Table 1 | worse than Lee |
| Training set size | 64,996 / 58,538 | timestamps | §5.2 | ~12 min free; payload minutes not restated |
| Bounding box | 1.6×1.6×0.4 m³ | training workspace | §5.2 | not the test path size |
| Flight hall | 7.5×4×2.75 m | mocap volume | §5.1 | not a formation gap |
| Inference time | 203 µs | MLP on this STM32 | §5.2 | this net only |
| Payload | 5 g, 0.5 m | fixed mass and string | §5.1 | no mass sweep |
| "small margin" | qualitative | NA-INDI vs INDI, no payload | §5.4.2 | payload text is "similar," not "small margin" |
| OOD circle drop | qualitative | LINDI vs INDI, no payload, Circle | §5.4.2 | do not attach to NA-INDI or payload |

Abstract and body do not fight on the 40% figure. The 40% sentence is only in the no-payload results body, not in the abstract.

## 6. Assumptions and failure modes

- Known constant mass, inertia, $\kappa_F$, $\mathbf{B}_0$.
- Cable at CoG, length fixed, payload mass known and constant.
- Training needs an RPM board. LINDI at runtime does not.
- Learned map is for "similar specifications." No transfer claim across airframes.
- No wind. No second vehicle. No downwash.
- Circle can be OOD for LINDI.
- Full cable-INDI in the appendix failed on this hardware because $\dot{\boldsymbol{\Omega}}$ was unusable.
- PWM as a thrust sensor is a stated failure mode for INDI (Table 1).
- Residual net is feed-forward, not an online learner.

## 7. Relation to our thesis (OUR READING)

- Supports strategy 2 (pure INDI) as their reactive baseline.
- Supports strategy 3 (geometric + NN residual) as LINDI.
- Supports strategy 5 (hybrid NA-INDI): one residual split $\mathbf{f}_{a,\mathrm{NN}}+\mathbf{f}_{a,\mathrm{INDI}}$, not two full residuals subtracted twice.
- Does not support FBL. Does not support residual RL. Does not support multi-robot interaction.
- We may cite: RPM-free runtime residual; spline-smoothed INDI labels; hybrid leftover-INDI; onboard 203 µs MLP on Crazyflie STM32; held-out Figure8 / Circle protocol; 40% only for their no-payload vs Lee.
- We must NOT cite: team size >1, formation gaps, downwash, 40% on payload, OOD drop for NA-INDI, "payload swing" as a named residual, training without RPM.
- Formations to reproduce from this paper: none. Reference conditions we can copy: single-vehicle Figure8 and Circle at ≤2 m/s, and optional 5 g / 0.5 m payload at ≈1.75 m/s.
- Compute: they ran this MLP on the Crazyflie STM32. We can treat onboard NN residual as demonstrated on that class of MCU. They did not run a payload-cable EKF on it.

## 8. Equations worth keeping

- (1)–(2): nominal rigid body plus additive $\mathbf{f}_a,\boldsymbol{\tau}_a$.
- (3a,b): $f_i=\kappa_F r_i^2$, allocation $\mathbf{B}_0$.
- (4), (6): tension $T\mathbf{q}$ and $T=-m_p\mathbf{q}^\top(\dot{\mathbf{v}}_p+g\mathbf{e}_z)$. $\mathbf{q}=(\mathbf{p}_p-\mathbf{p})/\|\mathbf{p}_p-\mathbf{p}\|$.
- (7): mean tracking cost. Free $d=\|\mathbf{p}_\pi-\mathbf{p}_r\|_2$. Payload uses payload position.
- (8a,b), (9): geometric laws with residuals subtracted.
- (10a,b): INDI residual recovery.
- §4.4 split: $\mathbf{f}_a=\mathbf{f}_{a,\mathrm{NN}}+\mathbf{f}_{a,\mathrm{INDI}}$ (same for torque).

## 9. Figures / tables worth remembering

- Fig. 1: three residual pipelines (INDI / LINDI / NA-INDI).
- Fig. 2: Crazyflie + RPM IR board + payload LED.
- Fig. 3: raw-label MLP vs spline-label MLP on residual acceleration $y$ during a payload Figure8. Spline labels look cleaner.
- Fig. 4: (a) random training spline, (b) test Figure8, (c) test Circle. No numeric radii on the page.
- Fig. 5: free-flight violins, 20 trials, vehicle error in cm.
- Fig. 6: payload violins, 10 trials, payload error in cm.
- Table 1: PWM-INDI vs Lee / LINDI-PWM / NA-INDI-PWM, 10 trials, cm.

## 10. Open uncertainties

- Geometric gains $K_p,K_v,K_R,K_\omega$ (and payload cascade gains): not in this paper.
- Control rate on the STM32: not in this paper.
- Butterworth cutoff and order: not in this paper.
- Figure8 / Circle radius, period, exact duration $D$: not in this paper.
- How $\dot{\boldsymbol{\omega}}$ is discretised beyond "numerically."
- Residual prediction error vs INDI labels in closed loop: they report tracking error only.
- Whether ground or wall effect was present in the 2.75 m hall: not discussed.
- Exact string elasticity: modelled as direction $\mathbf{q}$ and tension (6); appendix uses a rigid-rod cable. Which elastic model sat on the hardware is not measured.

## 11. 12-line abstract I can reread

Cobo-Briesewitz et al. fly one Crazyflie 2.1, with and without a 5 g mass on a 0.5 m CoG string.
Classical INDI builds $\mathbf{f}_a,\boldsymbol{\tau}_a$ from IMU and rotor RPM and subtracts them in a Lee / Yu geometric law.
LINDI is an onboard MLP trained on those INDI residuals after offline cubic-spline smoothing.
At runtime LINDI uses state, IMU, and PWM. It does not use RPM. Training still needed RPM.
NA-INDI adds the network output to a leftover INDI term that still uses RPM.
Tests are held-out Figure8 and Circle, ≤2 m/s free and ≈1.75 m/s with payload.
No-payload mean errors (cm): Lee 7.07/7.45, INDI 4.28/4.33, LINDI 4.41/4.87, NA-INDI 4.14/4.26.
That no-payload cut vs Lee is the paper's "around 40%."
Payload means (cm): Lee 7.06/7.26, INDI 5.38/5.91, LINDI 5.46/5.78, NA-INDI 5.39/5.74.
Authors call NA-INDI's extra gain a "small margin" without payload and "similar" with payload.
PWM in place of RPM breaks INDI. No second robot and no formation appear.
Appendix cable-INDI was not flown: cable angular acceleration was too noisy on this platform.

Source: attached PDF only. No other paper used.
