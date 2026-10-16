# Neural-Swarm — Shi et al., 2020

## 0. Identity

- Full title: Neural-Swarm: Decentralized Close-Proximity Multirotor Control Using Learned Interactions
- Authors: Guanya Shi, Wolfgang Hönig, Yisong Yue, Soon-Jo Chung
- Year / venue / arXiv id / DOI if printed: 2020. Attached file prints `arXiv:2003.02929v1 [cs.RO] 6 Mar 2020`. Conference/journal name is **not in this paper**. DOI is **not in this paper**.
- Version of THIS file: arXiv v1 (7 pages including references)
- Suggested cite key: `shi2020neuralswarm`
- What this paper IS: a decentralized nonlinear tracking controller that learns multi-vehicle residual interaction force $f_a$ with a spectrally normalized deep-set network and injects it as feedforward on Crazyflie hardware.
- What this paper is NOT: Neural-Lander (single vehicle vs ground). Not INDI. Not geometric position control. Not heterogeneous swarms. Not torque learning. Not planning.

## 1. Problem they pose

Their wording: "Close-proximity control is challenging due to the complex aerodynamic interaction effects between multirotors, such as downwash from higher vehicles to lower ones. Conventional methods often fail to properly capture these interaction effects, resulting in controllers that must maintain large safety distances."

Plain restatement: stacked or dense flight needs a model of the force one quadrotor puts on another. Without that model you keep large gaps and still get height error.

They also write that, "in lieu of better downwash interaction modeling, one must require a large safety distance between vehicles, e.g., 60 cm for the small Crazyflie 2.0 quadrotor (9 cm rotor-to-rotor) [4]."

## 2. Paper map (follow their TOC)

### Abstract

- Purpose: claim, method one-liner, headline result.
- Method: nominal dynamics + "regularized permutation-invariant Deep Neural Network (DNN)" + "stable nonlinear tracking controller."
- Result claimed here: "up to four times smaller worst-case height tracking errors." Also: model "generalize[s] to larger swarm sizes."
- Assumption: homogeneous team. Decentralized neighbor inputs only.

### I. Introduction

- Purpose: why dense flight, why downwash is hard, how they differ from Neural-Lander and from conservative ellipsoids/cylinders.
- Method sketch: learn residual forces from relative position and velocity of neighbors. Offline training. Onboard use as feedforward.
- Claim: "To our knowledge, our approach is the first that models interactions between more than two multirotor vehicles."
- Related-work split in this section: Neural-Lander = UAV–ground. Propeller-field models = two vehicles, often stationary. Planning papers use conservative keep-out shapes.

### II. Problem Statement: Swarm Interactions

- Purpose: write the plant and name what is unknown.

**II-A. Single Multirotor Dynamics**

- Eq. (1a)–(1b). Residual force $f_a$ and torque $\tau_a$ sit next to the nominal wrench.
- Implied: rigid body, $\eta = B_0 u$ with $u$ = squared motor speeds.

**II-B. Swarm Dynamics**

- Eq. (2)–(3). Same maps $f, f_a, \tau_a$ on every vehicle (homogeneous).
- $f_a^{(i)} = f_a(\mathcal{N}^{(i)})$, $\mathcal{N}^{(i)}$ = relative states of neighbors.

**II-C. Problem Statement & Approach**

- They "focus on the position dynamics (1a) so $f_a$ is our primary concern."
- Offline learning, then onboard in an "exponentially-stabilizing controller."

### III. Learning Approach

- Purpose: architecture constraints (permutation invariance, Lipschitz, compact).

**III-A. Permutation-Invariant Neural Networks**

- Lemma 1 (from Deep Sets): $h = \rho(\sum_i \phi(x_i))$.
- Eq. (5): $\hat f_a^{(i)} = \rho\big(\sum_{x^{(ij)}\in\mathcal{N}^{(i)}} \phi(x^{(ij)},\theta_\phi),\theta_\rho\big)$.
- Claim: complexity linear in neighbor count. Same $\phi,\rho$ for varying $n$. Data from $n$ vehicles reusable $n$ times under homogeneity.
- Body claim: "a few minutes flight data is sufficient to accurately learn interactions between two to five multirotors."
- Body claim: trained on three vehicles, "good predictions for five."

**III-B. Spectral Normalization**

- Eq. (6)–(7). ReLU nets. After each batch step, $W^i \leftarrow W^i/\|W^i\|_2 \cdot \gamma^{1/(L+1)}$.
- Purpose they state: Lipschitz bound $\gamma$, robustness to input noise, used later for stability.

**III-C. Data Collection**

- Curriculum: 2 vehicles without NN → learn → fly closer → add vehicles.
- Compensation of $\hat f_a$ changes $f_u$ only, "not the observed $f_a$."

### IV. Nonlinear Decentralized Controller Design

- Purpose: put $\hat f_a$ into a tracking law and prove an error ball.

**IV-A. Reference Trajectory Tracking**

- Eq. (8): composite $s^{(i)} = \dot{\tilde p}^{(i)} + 2\Lambda\tilde p^{(i)} + \Lambda^2\int\tilde p^{(i)}\,dt$.
- Eq. (9): $f_d^{(i)} = m\dot v_r^{(i)} - K s^{(i)} - mg - \hat f_a^{(i)}$.
- Desired thrust/attitude from $f_d$. Attitude may be "robust nonlinear tracking [23]" or "geometric tracking control on SE(3) [24]."
- Then $u_d^{(i)} = B_0^\dagger \eta_d^{(i)}$. Decentralized.

**IV-B. Nonlinear Stability and Robustness Analysis**

- Assumptions 1–2: bounded reference $p_d,\dot p_d,\ddot p_d$; learning error = constant bias + time-varying term whose derivative is bounded by $d_m^{(i)}$.
- Theorem 2 / Eq. (10): $\lim_{t\to\infty}\|\tilde p^{(i)}(t)\| = d_m^{(i)} / \big(\lambda_{\min}^2(\Lambda)\,\lambda_{\min}(K)\big)$.
- They do **not** claim $\tilde p\to 0$. ISS / exponential convergence to a ball.

### V. Experiments

- Purpose: platform, calibration, data, Table I, figures.

**V-A. Calibration and System Identification**

- Modified Crazyflie 2.0. Stock: "9 cm rotor-to-rotor", "34 g". Stock total force "about 48 g", TWR "1.4". Replaced motors: TWR "2.6". Same physical motor size.
- Load-cell + battery-voltage thrust maps. $J$ and thrust-to-torque "from the existing literature [29]."

**V-B. Data Collection and Learning**

- Tasks: random walk in a "small cube" with onboard artificial potentials; swapping on a cylinder.
- Data amount: "one minute of data for each scenario," scenarios 2–4 vehicles.
- Residual target from (1a): $f_a = m\dot v - mg - R f_u$.
- Inputs used: relative global position and velocity in $\mathbb{R}^6$. "attitude information $R$ and $\omega$ are not dominant for $f_a$."
- "we only learn the $z$ component of $f_a$."
- $\phi$: $6\to25\to40\to40\to40$. $\rho$: $40\to40\to40\to40\to1$.
- Eq. (12): stacked squared loss.
- Onboard: STM32 168 MHz, "about 550 µs" per network, "less than 4 ms for 6 or less neighbors." C-code from PyTorch weights.

**V-C. Neural-Swarm Control Performance**

- Test: swapping, vertical neighbor spacing "0.2 m to 0.25 m."
- Four controllers: baseline $\hat f_a=0$ (same gains, still has integral); same law with nets trained on 2, 3, or 4 CF.
- Metric: "maximum z-error that occurred for any vehicle over the whole flight." Six repeats.
- Table I is the quantitative result (see §5).
- Body example: "2.4 cm instead of 9.4 cm for the two vehicle case."
- Motors saturate on some 3–5 vehicle tests. That is why max $z$-error grows with $n$.

**V-D. Learned Neural Network Visualization**

- Fig. 3 heatmaps: $\hat f_{a,z}$ vs $(y,z)$ with neighbors as blue stars.
- Claims: interaction is non-stationary; sensitive to relative velocity; "not a simple superposition of different pairs."
- Fig. 4: 3-CF-trained net vs ground-truth $f_{a,z}$ on a 4-vehicle swap.

### VI. Conclusion

- Repeat: decentralized, relative pos/vel only, "factor of two or better," first >2-vehicle interaction model (their claim).
- Future they list: heterogeneous teams, planning with learned forces, also learn $\tau_a$.

### References

- Not a method section. Neural-Lander is [7]. Deep Sets is [8]. Spectral norm is [6]. Crazyswarm is [27]. Geometric SE(3) attitude is [24].

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | "slightly modified Crazyflie 2.0"; "9 cm rotor-to-rotor"; "34 g"; motors swapped, best-case TWR "2.6" |
| n robots min/max | Training flights 2–4. Hardware tests 2–5. |
| Sim / hardware / both | Hardware only. No simulation results in this file. |
| Onboard vs off-board (Hz) | Position/attitude controller, EKF, and NN "running on-board the STM32." Mocap pose "100 Hz." Data log "roughly every 10 ms." NN "~550 µs" each; "< 4 ms" for ≤6 neighbors. |
| Sensing (mocap, RPM, GPS, …) | External motion capture, four markers. Onboard IMU. Battery voltage for thrust map. No GPS. RPM not used as NN input. |
| Controller family | Nonlinear tracking / feedback-linearizing position law with integral action + residual feedforward. Attitude: they allow robust nonlinear [23] or geometric SE(3) [24]. |
| How interaction enters the law | $\hat f_a^{(i)}$ subtracted in the desired-force equation (9). Feedforward. Not an INDI filter. |
| Learned model? inputs → output | Deep sets $\phi,\rho$, spectral norm. Input: relative $p,v$ of each neighbor ($\mathbb{R}^6$). Output: $\hat f_{a,z}$ only. |
| Training data amount / source | Curriculum real flights. Random walk + swap. "one minute of data for each scenario," 2–4 vehicles. "a few minutes" stated as enough in III-A. |
| Formations + min separation | Swap: vehicles on a cylinder, then opposite side; vertical neighbor gap "0.2 m to 0.25 m." Fig. 2 (3 CF): "25 cm vertical separation." Random-walk cube size **not in this paper**. |
| Speeds | Swap speeds **not in this paper**. Fig. 3(b) visualization only: one neighbor drawn at "0.8 m/s." |
| Metrics | Maximum (worst-case) $z$-error over the flight. x/y errors "similar across the different controllers" and not tabulated. |
| Baseline they compare to | Same position law and same gains with $\hat f_a=0$. Baseline still has integral action. |

## 4. Formations and flights (table, one row per experiment)

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Vertical swap | 2 | different fixed heights; align vertically once per pass | 0.20–0.25 m between neighbors | horizontal swap on a cylinder; speed not in this paper | hw | Table I col. "2 CF Swap"; Fig. 1 |
| Vertical swap | 3 | fixed heights; align at t≈2 s and t≈6.5 s (Fig. 2) | Fig. 2: "25 cm vertical separation" | same class of swap; speed not in this paper | hw | Table I; Fig. 2 height traces |
| Vertical swap | 4 | same class; stacked alignment | 0.20–0.25 m | same; speed not in this paper | hw | Table I; Fig. 4 $f_{a,z}$ traces |
| Vertical swap | 5 | same class | 0.20–0.25 m | same | hw | Table I only. No 5-CF figure. |
| Random walk (training only) | 2–4 | random goals in a "small cube" + APF avoidance | cube size not in this paper | successive random goals | hw | training data, not a reported error table |

No side-by-side, triangle, V, hover-only, ground-effect, or docking flights in this file.

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Rotor span | 9 cm | Crazyflie 2.0 **rotor-to-rotor** | Intro | Not "body across." |
| Conservative gap cited | 60 cm | planning safety distance when interaction is unmodelled | Intro, cited from [4] | Not a Neural-Swarm measured result. |
| Two-vehicle downwash example | "−9 g" at 30 cm hover stack | force they say is "well within their thrust capabilities" | Intro | Motivating example, not a Table I metric. |
| Headline factor | "up to four times smaller" | worst-case **height** tracking error vs their baseline | Abstract | Body/conclusion also say "a factor of two or better." Both stand. Body does not retract the abstract. |
| 2 CF example | "2.4 cm instead of 9.4 cm" | max $z$-error, two-vehicle swap | Body §V-C | 9.4 cm = baseline 0.094 m. 2.4 cm = **trained w/ 4 CF** (0.024 m). Trained w/ 2 CF on same test is 0.027 m. |
| Table I baseline | 0.094 / 0.139 / 0.209 / 0.314 m | max $z$-error, swap, n=2/3/4/5 | Table I | Worst vehicle, whole flight, six repeats. |
| Table I best reported | 0.024 / 0.061 / 0.102 / 0.150 m | same metric, net trained on 4 CF | Table I | 2-CF-trained net on 3 CF is 0.150 m (worse than baseline 0.139). 2-CF net does **not** generalize. |
| Vertical spacing | 0.2 m to 0.25 m | neighbor Δz on the swap | Body §V-C | 25 cm is explicit only in Fig. 2 (3 CF). |
| TWR | 1.4 stock → 2.6 after motor swap | best-case thrust-to-weight | §V-A | After hardware change. |
| Mass / force | 34 g vehicle; stock total force "about 48 g" | platform | §V-A | |
| Compute | 550 µs / net; <4 ms for ≤6 neighbors; 168 MHz STM32 | onboard eval time | §V-B | Two nets ($\phi$ and $\rho$). |
| Mocap | 100 Hz | pose rate | §V | Compensation runs onboard. Pose is external. |
| Log rate | ~10 ms | uSD binary log | §V | |
| Data per scenario | one minute | each of 2–4 vehicle random-walk and swap sets | §V-B | Total minutes summed across all runs is **not in this paper**. |
| Heatmap example | $\hat f_{a,z}=4$ g | third vehicle at one plotted pose in Fig. 3(c) | Fig. 3 caption | Visualization, not a flight-test metric. |
| Viz speed | 0.8 m/s | neighbor speed in Fig. 3(b) only | Fig. 3 | Not a stated swap command speed. |

## 6. Assumptions and failure modes

- Homogeneous vehicles. Same $f$, $f_a$, $\tau_a$.
- $f_a$ depends only on the neighbor relative-state set.
- Only $f_{a,z}$ is learned. $f_{a,x}, f_{a,y}$ "very small." $\tau_a$ unused.
- Relative inputs are $p$ and $v$ only. $R,\omega$ dropped because "not dominant."
- Assumption 1: $p_d,\dot p_d,\ddot p_d$ bounded.
- Assumption 2: residual error = bias + time-varying term with bounded derivative $d_m$.
- Tracking does not go to zero. Error ball scales with $d_m / (\lambda_{\min}^2(\Lambda)\lambda_{\min}(K))$.
- Integral action alone is not enough when $f_a$ changes fast (their reason the baseline fails on swap).
- Motor saturation on lower vehicles in 3–5 stacks. Compensation cannot exceed thrust limits.
- Curriculum needed: without $\hat f_a$, vehicles "move apart," so close data is hard to collect.
- Neighbor relative states must be available. Communication protocol is **not in this paper**.
- Short flights ("less than 2 min") so clock drift is ignored.
- Crazyflie IMU "directly mounted on the PCB frame" is noisier; they treat spectral norm as important for that.

## 7. Relation to our thesis (OUR READING)

- Supports strategy **4 FBL + NN residual**. Position law is Eq. (9), not INDI, not a geometric position controller.
- Geometric SE(3) appears only as an optional **attitude** inner loop [24]. Do not cite this paper as "geometric + NN" unless you mean attitude only.
- What we may cite: multi-vehicle residual $f_a$ is learnable from relative $p,v$; deep sets + spectral norm; superposition fails for $n>2$; swap at 0.20–0.25 m as a stress test; max $|z|$ as a metric; 2–4× class of height-error cut vs an integral-only nonlinear tracker; onboard STM32 feasibility; generalization only after training on ≥3 vehicles.
- What we must NOT cite from it: INDI results; ground effect (that is Neural-Lander, which they cite as different); heterogeneous teams; $\tau_a$ compensation; a single 25 cm number for every $n$; "any network generalizes upward"; 2.4 cm as the 2-vehicle-trained number; 60 cm as their own flown separation; simulation numbers; OptiTrack by name (they say "motion capture"); brushless as a product name (they say modified CF 2.0).
- Formations to reproduce if used as a reference condition: constant-height vertical swap, neighbor Δz 0.20 m and 0.25 m, $n=2$ then $n=3$, report max $|z|$ over the pass. Do not invent a cylinder radius.
- Compute: they **did** run $\phi$ and $\rho$ on the STM32 at 168 MHz. That is direct evidence a compact residual net can live onboard. Gains $\Lambda,K$ and $\gamma$ are **not in this paper**, so we cannot copy their loop numbers.

## 8. Equations worth keeping

Single-vehicle translation, Eq. (1a):

$$
\dot{\mathbf{p}}=\mathbf{v},\qquad
m\dot{\mathbf{v}}=m\mathbf{g}+R\mathbf{f}_u+\mathbf{f}_a.
$$

$\mathbf{f}_a$: unmodelled interaction force.

Neighbor residual, Eq. (3):

$$
\mathbf{f}_a^{(i)}=\mathbf{f}_a(\mathcal{N}^{(i)}),\qquad
\mathcal{N}^{(i)}=\{\mathbf{x}^{(ij)}\mid j\in\mathrm{neighbor}(i)\}.
$$

Deep-set approximator, Eq. (5):

$$
\hat{\mathbf{f}}_a^{(i)}
=\rho\Bigg(\sum_{\mathbf{x}^{(ij)}\in\mathcal{N}^{(i)}}
\phi(\mathbf{x}^{(ij)},\theta_\phi),\theta_\rho\Bigg).
$$

Composite tracking error, Eq. (8):

$$
\mathbf{s}^{(i)}=\dot{\tilde{\mathbf{p}}}^{(i)}+2\Lambda\tilde{\mathbf{p}}^{(i)}+\Lambda^2\int\tilde{\mathbf{p}}^{(i)}\,dt
=\dot{\mathbf{p}}^{(i)}-\mathbf{v}_r^{(i)}.
$$

$\tilde{\mathbf{p}}^{(i)}=\mathbf{p}^{(i)}-\mathbf{p}_d^{(i)}$. $\mathbf{v}_r^{(i)}$ is the reference velocity.

Desired force with residual feedforward, Eq. (9):

$$
\mathbf{f}_d^{(i)}=m\dot{\mathbf{v}}_r^{(i)}-K\mathbf{s}^{(i)}-m\mathbf{g}-\hat{\mathbf{f}}_a^{(i)}.
$$

Error ball, Theorem 2 / Eq. (10):

$$
\lim_{t\to\infty}\|\tilde{\mathbf{p}}^{(i)}(t)\|
=\frac{d_m^{(i)}}{\lambda_{\min}^2(\Lambda)\,\lambda_{\min}(K)}.
$$

$d_m^{(i)}$ bounds $\|\dot\epsilon^{(i)}(t)\|$, the time derivative of the time-varying part of $\hat f_a-f_a$.

Training loss, Eq. (12):

$$
\sum_i\sum_{\mathcal{D}^{(i)}}
\Big\|
\rho\Big(\sum_{\mathbf{x}^{(j)}\in\mathcal{N}^{(i)}}\phi(\mathbf{x}^{(j)},\theta_\phi),\theta_\rho\Big)
-y^{(i)}
\Big\|_2^2.
$$

$y^{(i)}$ is the observed $f_a$ from (1a).

Spectral-norm step, Eq. (7):

$$
W^i\leftarrow\frac{W^i}{\|W^i\|_2}\,\gamma^{1/(L+1)}.
$$

## 9. Figures / tables worth remembering

- Fig. 1: pipeline sketch plus a two-vehicle vertical swap photo/sequence. Shows the close stack they care about.
- Fig. 2: 3-CF swap height vs time. Setpoint vs baseline vs nets trained on 2 / 3 / 4 CF. Caption states 25 cm vertical separation and alignment at t = 2 s and 6.5 s.
- Fig. 3: heatmaps of $\hat f_{a,z}$ from the 3-CF-trained net. Color = predicted vertical force. Blue stars = neighbor positions. Panel (b) neighbor at 0.8 m/s. Used to argue velocity dependence and non-superposition.
- Fig. 4: 4-CF swap. Ground-truth $f_{a,z}$ vs prediction from the **3-CF-trained** net. Lower vehicles show forces large enough that they "cannot fully compensate it within our thrust limits."
- Table I: max $z$-error (m) for baseline and three trained nets, columns 2 / 3 / 4 / 5 CF swap. This is the only numeric comparison table.

## 10. Open uncertainties

- Conference venue and DOI (not printed on this file).
- Gains $\Lambda$, $K$. Spectral bound $\gamma$.
- Cylinder radius and exact start/end $(x,y)$ of the swap.
- Commanded speed / acceleration of the swap.
- Exact size of the "small cube."
- Neighbor definition and radio protocol for relative state.
- Whether they used [23] or [24] for attitude on the reported flights.
- Filter or differentiation method for $\dot v$ when building $y = f_a$.
- How many neighbors were declared at runtime (bound given as "6 or less" for timing only).
- Number of distinct flights beyond "six" repeats of each swap controller.
- Whether x/y residual was measured and discarded, or never fit.

## 11. 12-line abstract I can reread

Neural-Swarm learns the vertical residual interaction force between homogeneous Crazyflies and subtracts it inside a nonlinear tracking law. The learner is a spectrally normalized deep-set pair $(\phi,\rho)$ on relative position and velocity. Training is curriculum real flight: random walk plus cylinder swaps, one minute per 2–4 vehicle scenario. Tests are the same swap family at 0.20–0.25 m neighbor height (25 cm drawn for three vehicles) on 2–5 modified CF 2.0s. Pose is mocap at 100 Hz. Controller, EKF, and NN run on the STM32. Baseline is the identical law with $\hat f_a=0$ and the same gains, integral included. Table I max $z$-error falls from 0.094 m to 0.024 m on two vehicles for the best net, and by about 2× on 3–5 vehicles. A 2-vehicle net does not help larger stacks. A 3- or 4-vehicle net does help five vehicles. They state the multi-vehicle field is not a pairwise sum. They do not learn $\tau_a$ or horizontal $f_a$. They do not claim zero tracking error.

Source: attached PDF only. No other paper used.
