# Flatness-Preserving Residual FBL — Hsieh et al., 2026

## 0. Identity

- Full title: Flatness-Preserving Residual Learning for Real-Time Tight Quadrotor Formation Flight
- Authors: Pei-An Hsieh†, Fengjun Yang†, Nikolai Matni, M. Ani Hsieh (GRASP, UPenn; † equal contribution)
- Year / venue / arXiv / DOI: 15 Jul 2026; arXiv:2607.12275v1 [cs.RO] 14 Jul 2026; DOI not in this file
- Version of THIS file: arXiv v1 preprint (15 pages + Appendix A). No journal version in this file.
- Suggested cite key: `hsieh2026flatness`
- What this paper IS: A two-Crazyflie stacked-formation paper that learns a kinematics-only residual (ROM + tiny NN) so the team stays differentially flat, then runs off-board feedback linearization with residual feedforward.
- What this paper is NOT: Not onboard STM32 control. Not INDI. Not 3+ robots. Not NMPC as the proposed law (NMPC is the baseline). Not Neural-Swarm. Not a published journal article in this file.

## 1. Problem they pose

Their words: tight formations produce "turbulent aerodynamic interactions, such as downwash, that can cause catastrophic collisions if left unmodeled." Unstructured residuals are usually put into NMPC, which "demands substantial computational resources and scales poorly."

Plain restatement: they want a residual that is accurate enough for downwash, but structured enough that a cheap flatness-based controller can cancel it, instead of running NMPC.

## 2. Paper map (follow their TOC)

### Abstract

- Purpose: claims and headline numbers.
- Construction: physics-informed residual that keeps joint flatness; FBL with feedforward disturbance cancel.
- Result claimed here only: "reduces average tracking errors by 31% compared to nominal baselines"; "matches" SOTA NMPC "while requiring an order of magnitude less computation"; "under 30 seconds of training data and a 5ms loop rate."
- Assumption: those three numbers are not recomputed in the body as a single table.

### 1 Introduction

- Purpose: motivate residual learning vs NMPC cost; state the kinematic restriction.
- Construction: residual depends "strictly on the joint positions and velocities of the quadrotor team" so augmented dynamics stay differentially flat.
- Result: "on par with NMPC baselines … order of magnitude less computation." In §1.1.1 they also write "matches their tracking performance while using 7× less compute" on "an identical hardware setup."
- Mix-up risk: 7× (intro) vs order of magnitude (abstract/conclusion) vs 20× (sim §5.1) are three different sentences.

### 1.1 Related Works

- Purpose: place vs NeuralLander / Neural-Swarm, Spitzer–Michael FBL, KNODE-MPC, Faessler drag-flatness, Yang et al. 2025a theory.
- Claim: first hardware use of Yang-style flatness-preserving residuals for tight formation flight.
- Claim: first FBL-with-learned-residual on tight multi-quadrotor formation (their words).

### 1.2 Contributions

- Four items: (i) joint flatness + reconstruction maps §3.2; (ii) physics-informed learner §4; (iii) cheap controller §3.3; (iv) sim + hardware vs NMPC. They point the NMPC compute claim at "Sec. 6"; Sec. 6 in this file is the conclusion. The tables live in Sec. 5.

### 2 Problem Formulation

- Purpose: rigid-body team model + residual slot.
- Dynamics (their (1)):

$$
\dot p^i = v^i,\quad
m^i\dot v^i = u^i R^i e_3 + m^i g + d^{a,i},
$$

$$
\dot R^i = R^i[\omega^i]_\times,\quad
J^i\dot\omega^i = J^i\omega^i\times\omega^i + \tau^i + d^{\tau,i}.
$$

- Augmented residual form (2): $\hat f^{a,i}(\xi)$ in force, $(R^i)^\top\hat f^{\tau,i}(\xi)$ in body torque.
- Assumption: when vehicles are "sufficiently separated," $d^{a,i}=d^{\tau,i}=0$. Tight stack is the regime where that fails.

### 3 Flatness-Preserving Residuals

- Purpose: recall nominal quadrotor flatness, then show the residual does not break it if $\xi$ is kinematic only.

### 3.1 Background on Quadrotor Flatness

- Nominal flat outputs $y=[p,\psi]$. Thrust $u=m\|a-g\|$, body $z$ from acceleration, $\omega$ from jerk. Standard Mellinger–Kumar chain (3)–(11).

### 3.2 Flatness under Augmented Dynamics

- Purpose: reuse the same algebraic chain after subtracting the residual.
- Feature lock (13): $\xi=[p^1,v^1,\dots,p^N,v^N]^\top$.
- Corrected maps (14)–(16):

$$
u=\|m(a-g)-\hat f^a(\xi)\|,\qquad
z=\frac{m(a-g)-\hat f^a(\xi)}{u},
$$

$$
\omega_{xy}^B=\frac1u e_3\times R^\top\bigl(mj-\dot{\hat f}^a(\xi)\bigr),\qquad
\tau=J\dot\omega+\omega\times J\omega-R^\top\hat f^\tau(\xi).
$$

- Assumption: residual must not depend on thrust, attitude, or $\omega$, or the algebra stops being explicit.
- Scope they state: "we focus on two-quadrotor teams for this paper."

### 3.3 Flatness-based Controller Design

- Purpose: cascaded FBL for tracking $p_r$.
- Position PID → commanded $a$ (17); differentiate → jerk (18); map to $\omega_r$ via (15)+(10); torque as high-gain P plus residual feedforward (19), not exact $\ddot{\hat f}$.
- Acceleration estimated with an $\alpha$-$\beta$-$\gamma$ filter, not raw IMU $a$.
- Assumption: they reject full dynamic-extension FBL on hardware (noise / low-frequency loss).

### 4 Physics-Informed Residual Learning

- Purpose: ROM + NN on an equivariant pair feature so the kinematic restriction still fits data.
- Pair geometry: $\Delta p=p^2-p^1$, $\Delta r=\sqrt{\Delta p_x^2+\Delta p_y^2}$, $\Delta z=\Delta p_z$.
- Near-hover for the upper vehicle: $z^1\approx e_3$. Local frame $\mathcal F$ (20). Feature $h_0=[\Delta z,\Delta r]^\top$.
- Split (21): $\hat f(\xi)=\hat f_d(h_0)+\hat f_\Theta(h_0)$.
- NN: small MLP, $\tanh$, output rotated by $R_W^F$.
- Train with KNODE (integrated-state MSE), Adam. Loss (23).
- ROM: turbulent-jet velocity + disk drag, details in Appendix A. Adapted from Chee et al. 2024.

### 5 Experiment

- Purpose: residual accuracy, closed-loop FBL and SE(3), ablation Nominal / ROM / Learned w/o ROM / Learned+ROM, then NMPC timing.

### 5.1 Simulation Experiments

- Crazyflie mass/inertia. Plant downwash from Chee et al. 2024 with $C_D=0.7$; ROM uses $C_D=0.3$.
- Plant residual depends on variables outside $\xi$ (e.g. orientation). That is the stress test.
- Nets: two MLPs, one hidden layer, nine neurons each.
- Train: 18 s, FBL+ROM controller, four cases (static-top and stacked at 0.3 m and 0.4 m).
- FBL at 100 Hz; RK45 at 10 ms; full noise-free state.
- Table 1 prediction RMSE. Tables 2–3 tracking. Fig. 2 heatmaps. Table 4 vs Acados NMPC: "20× reduction in solve time."

### 5.2 Physical Experiments

- Two Crazyflie 2.1 + thrust-upgrade bundles. Body 10 cm, 36 g with battery and Vicon markers.
- Vicon 120 Hz → laptop (Intel i5) → Crazyradio PA commands at 400 Hz. CrazyROS.
- Hardware FBL outputs thrust + $\omega$, not torque. "we compensate exclusively for residual forces and rely on the high-frequency PID controller onboard to handle the residual torques."
- ROM $C_D=0.236$. Force net: 2-layer, 4 neurons.
- Train: 28 s (6,720 points), static + stacked, 0.4 m, 0.4 m/s, FBL without learned residual.
- Test: 0.4 m in-distribution, 0.3 m OOD. 60 runs, five repeats each. Fig. 3.
- Learned-FBL: "average RMSE of 5 cm and a $z_{\max}$ of 10 cm." They say this matches the *reported* numbers of KNODE-DW MPC (Chee et al., 2024).
- Demo: merge 1.1 m → 0.1 m through a 0.4 m high window (Fig. 1).

### 6 Conclusion

- Repeats "on par with NMPC … order of magnitude less computation."
- Future work they name: relax the residual structure; scale the experiments.

### Appendix A — Physics prior

- Jet velocity $V(\Delta z,\Delta r)$ (24). Disk integrals for force/torque (25). Constants $B_d,S,U_H,z_0$ from Bauersfeld et al. 2024 airflow measurements. Half-body length $\lambda$ used only to normalize $\Delta\tilde z,\Delta\tilde r$.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Crazyflie 2.1 + thrust-upgrade; 10 cm body; 36 g with battery + Vicon markers |
| n robots min/max | 2 / 2 (theory written for $N$; experiments are pairs) |
| Sim / hardware / both | Both |
| Onboard vs off-board (Hz) | Compensation on laptop. Vicon 120 Hz in, radio commands 400 Hz out. Sim FBL 100 Hz. Onboard piece is the stock attitude PID only. |
| Sensing | Vicon pose. Accel from $\alpha$-$\beta$-$\gamma$ on $p,v$. No RPM/GPS in this file. |
| Controller family | Cascaded FBL (proposed). Also SE(3) geometric with the same residual as feedforward. |
| How interaction enters the law | Learned $\hat f^a$ subtracted in the thrust / $z$-axis map; $\hat f^\tau$ as torque feedforward in sim. Hardware: force residual only. |
| Learned model? inputs → output | Inputs: $h_0=[\Delta z,\Delta r]$. Output: residual accel + (sim) residual torque. ROM + MLP. |
| Training data amount / source | Sim 18 s. Hardware 28 s / 6,720 points. Own flights under FBL±ROM. |
| Formations + min separation | Vertical stack and static-top. Hardware tests 0.3 m and 0.4 m. Demo 0.1 m. Sim down to 0.2 m. |
| Speeds | Sim references 0.2–0.5 m/s. Hardware train 0.4 m/s. Other HW speeds not in this paper. |
| Metrics | Residual RMSE $e^a/e^\tau$; 3-D position RMSE; $z_{\max}$; solve time (ms). |
| Baseline they compare to | Nominal (no residual), ROM-only, learned-without-ROM; SE(3) variants; KNODE-style NMPC (Chee et al., 2024) in Acados. |

## 4. Formations and flights (table, one row per experiment)

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Stacked tracking sweep | 2 | bottom under top | Δz = 0.2–0.6 m; lateral offset not stated (treated as stacked) | both fly same-direction straight min-snap; 0.2–0.5 m/s | sim | Fig. 2, Tables 2–3 |
| Residual prediction eval | 2 | stacked | 0.2 / 0.3 / 0.4 / 0.5 m | 100 Hz samples of bottom state | sim | Table 1 |
| NMPC bake-off | 2 | same stacked setups as above | 0.3 m and 0.5 m in Table 4 | same as sim sweep | sim | Table 4 RMSE/$z_{\max}$ + solve time |
| Train set (sim) | 2 | static-top and stacked | 0.3 m and 0.4 m | FBL+ROM | sim | 18 s |
| Train set (hw) | 2 | static-top and stacked | 0.4 m | 0.4 m/s, FBL without learned residual | hw | 28 s, 6,720 points |
| Closed-loop hw tests | 2 | stacked | commanded 0.4 m (train) and 0.3 m (OOD); lateral offset not stated | not in this paper beyond the train speed | hw | Fig. 3; 60 runs, 5 each; 24%/30% then +29%/+43%; Learned-FBL "5 cm" RMSE, "10 cm" $z_{\max}$ |
| Window merge | 2 | stacked, time-varying Δz | 1.1 m → 0.1 m; window height 0.4 m | translating merge | hw | Fig. 1 demo only; no RMSE table |

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Average tracking-error cut vs nominal | 31% | "average tracking errors" vs "nominal baselines" | abstract only | Body never rebuilds 31%. Body: ROM-FBL vs Nominal-FBL "24% in RMSE and 30% in $z_{\max}$"; Learned-FBL vs ROM-FBL extra "29% … RMSE and 43% … $z_{\max}$"; Learned-SE(3) vs nominal RMSE "43.7%", ROM-SE(3) "34%". |
| Compute vs NMPC | "order of magnitude" | computation / solve time | abstract, intro, §5 open, conclusion | Three different numbers in one file. |
| Compute vs NMPC | 7× | "less compute" | §1.1.1 | Tied to "identical hardware setup." No 7× table. |
| Compute vs NMPC | 20× | "reduction in solve time" | §5.1 body | JIT PyTorch FBL vs Acados C + RTI, **simulation**. Table 4: 0.99 ms vs 27.6 ms (0.3 m); 1.3 ms vs 26.2 ms (0.5 m). |
| Loop rate | 5 ms | "5ms loop rate" | abstract only | Not an onboard measurement. Laptop + 400 Hz radio. Sim FBL 100 Hz. Table 4 solve ~1 ms. |
| Training time | "under 30 seconds" | training data duration | abstract | Sim 18 s. Hardware 28 s. |
| Hardware accuracy | 5 cm RMSE, 10 cm $z_{\max}$ | Learned-FBL average | §5.2 body | Compared to Chee's *reported* numbers, not a same-run hardware timing table. |
| Sim FBL Learned+ROM at 0.3 m | 0.016 / 0.042 m | RMSE / $z_{\max}$ | Table 2 | In-distribution sim, noise-free state. |
| Same vs NMPC at 0.3 m | Ours 0.016/0.042; NMPC 0.020/0.032 | RMSE / $z_{\max}$ | Table 4 | RMSE similar or better; $z_{\max}$ **worse** than NMPC. "On par" is not "better on every metric." |
| Team size | 2 | vehicles flown | whole experimental section | $N$ is in the equations only. |
| Closest stack | 0.1 m | vertical separation in the window demo | Fig. 1, §5.2 last paragraph | Not in the 0.3/0.4 m statistics table. |
| Body length | 10 cm | Crazyflie 2.1 | §5.2 | Use if you convert 0.1 m = 1 body length. They report metres. |

## 6. Assumptions and failure modes

- Residual features are positions and velocities only. Orientation-dependent aero is approximated, not modelled.
- Upper vehicle near hover when building frame $\mathcal F$ ($z^1\approx e_3$).
- Pairwise stacked / static-top geometry. No side-by-side campaign in this file.
- Hardware cannot command torque through default firmware, so torque residual is not closed in the off-board law.
- Exact $\ddot{\hat f}^a$ dropped because of noise and cost.
- Training distribution is FBL without the learned net; closed-loop shift is not quantified.
- Tiny nets (9 neurons sim, 4 neurons hw). They themselves note possible angular-accel overfitting at 0.5 m in Table 1.
- Other agents treated as exogenous when writing the ego flatness maps.
- No communication, delay, or estimator analysis beyond "noisy $p,v,R$" and the $\alpha$-$\beta$-$\gamma$ filter.

## 7. Relation to our thesis (OUR READING)

- Supports strategy **4 FBL + residual**, and secondarily **3 geometric + NN residual** (their SE(3) feedforward tests). Does not implement INDI or hybrid NA-INDI.
- May cite: kinematic feature lock that preserves flatness; ROM + small NN on $(\Delta z,\Delta r)$; residual subtracted in the thrust / $z$-axis map; hardware proof that a cheap FBL can reach Chee-reported centimetre errors on a two-Crazyflie stack; ablation Nominal → ROM → learned.
- Must NOT cite as: onboard STM32 result; 3-robot result; 20× hardware speedup; "31%" as a body table; force-and-torque compensation on the real vehicles; OptiTrack (they use Vicon); brushless-vs-brushed comparison (not in this file).
- Reference conditions worth copying if we use this paper as a yardstick: two-vehicle vertical stack at 0.3 m and 0.4 m; optional OOD 0.2 m / 0.5 m in sim; report RMSE and $z_{\max}$; keep a no-residual geometric/FBL baseline.
- Compute: they ran the residual + FBL on an Intel i5 laptop in PyTorch (JIT). Not shown on STM32. The only onboard loop is Crazyflie stock rate PID. Porting is a claim they *suggest*, not a measurement they *show*.

## 8. Equations worth keeping

- (1b) force channel: $m\dot v = uRe_3 + mg + d^a$.
- (13) allowed features: $\xi=[p^1,v^1,\dots,p^N,v^N]^\top$.
- (14a–b) residual-corrected thrust and body $z$.
- (15) residual-corrected $\omega_{xy}^B$ using $\dot{\hat f}^a = (D_\xi\hat f^a)\dot\xi$.
- (17) position PID → commanded $a$.
- (19) hardware-style torque: $\tau=-K_\omega(\omega-\omega_r)-R^\top\hat f^\tau(\xi)$.
- (21) $\hat f=\hat f_d(h_0)+\hat f_\Theta(h_0)$, $h_0=[\Delta z,\Delta r]^\top$.
- (23) KNODE integrated-state loss.

## 9. Figures / tables worth remembering

- Fig. 1: two Crazyflies, 1.1 m stack shrinking to 0.1 m, 0.4 m window. Photo, not a metric.
- Fig. 2: sim FBL heatmaps, RMSE and $z_{\max}$ vs desired speed (0.2–0.5 m/s) and $z$ (0.2–0.6 m), four model fidelities.
- Fig. 3: hardware bars, RMSE and $z_{\max}$, mean±std over five runs. X labels: FBL 0.3 m, FBL 0.4 m, SE(3) 0.3 m, SE(3) 0.4 m. Colors: Nominal / ROM / Learned+ROM.
- Table 1: residual prediction RMSE $e^a/e^\tau$, in-dist 0.3/0.4 m and OOD 0.2/0.5 m.
- Table 2: sim FBL tracking RMSE / $z_{\max}$.
- Table 3: sim SE(3) tracking RMSE / $z_{\max}$.
- Table 4: sim Ours vs NMPC at 0.3 m and 0.5 m, errors and solve time in ms.

## 10. Open uncertainties

- PID / $K_\omega$ gains: not in this paper.
- Exact lateral offset in "stacked": not in this paper (implied under the top vehicle).
- Hardware test speed except the 0.4 m/s training set: not in this paper.
- Trajectory length and duration of each run: not in this paper.
- $\alpha$-$\beta$-$\gamma$ coefficients: not in this paper.
- How the abstract 31% was averaged: not in this paper.
- What "7× less compute" on the "identical hardware setup" measures: not in this paper.
- Whether residual inference ran at 400 Hz or slower inside the laptop loop: not in this paper.
- NN hidden widths besides "2-layer … 4 neurons" (hw) and "one hidden layer of nine neurons" (sim): not in this paper.
- Weight matrix $W_x$ on hardware: sim only (diag, last three entries 0.1).

## 11. 12-line abstract I can reread

Hsieh et al. 2026 learn a downwash residual that is allowed to depend only on team positions and velocities, so two-quadrotor dynamics stay differentially flat. The residual is a Chee-style ROM plus a tiny NN on $(\Delta z,\Delta r)$. They plug it into cascaded FBL (and also into SE(3)) as feedforward. Training data: 18 s sim, 28 s hardware. Platform: two Crazyflie 2.1, 36 g, Vicon, Intel i5 laptop, 400 Hz radio. Hardware compensates residual force only; torque residual is left to onboard PID. Hardware tests are stacked 0.3 m and 0.4 m; a demo goes 1.1 m → 0.1 m through a 0.4 m window. Abstract 31% vs nominal is not rebuilt in the body. Body hardware cuts are 24%/30% then +29%/+43% on FBL, and 43.7% RMSE on Learned-SE(3) vs nominal. Learned-FBL is quoted as 5 cm RMSE and 10 cm $z_{\max}$, matching Chee's reported NMPC numbers. Solve-time 20× is simulation (≈1 ms vs ≈27 ms). Intro also says 7×. Not an STM32 result.

Source: attached PDF only. No other paper used.
