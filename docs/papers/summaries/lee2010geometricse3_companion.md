# Geometric SE(3) quadrotor tracking — Lee et al., 2010/2011

> **NOT the CDC paper. Do not cite as the Strategy 1 source.**
> This file summarizes only arXiv:1003.2005v4, "Control of Complex Maneuvers for a Quadrotor UAV
> using Geometric Methods on SE(3)" — the extended companion manuscript. The primary Strategy 1
> reference is Lee, Leok & McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3),"
> 49th IEEE CDC 2010, pp. 5420–5425, DOI 10.1109/CDC.2010.5717652, cite key `lee2010geometrictracking`
> in `references.bib`. No summary card for that CDC paper exists in this folder yet — its PDF is
> not held in `docs/papers/` (IEEE-paywalled), so one has not been fabricated here. The three
> flight modes, velocity-mode law, heading projection (eq. 38), and the 720°/360° Case II
> maneuvers documented below are companion-only material and must not be attributed to CDC 2010.

## 0. Identity

- Full title: Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3)
- Authors: Taeyoung Lee, Melvin Leok, N. Harris McClamroch
- Year / venue / arXiv / DOI: this file is arXiv:1003.2005v4 [math.OC], 9 Sep 2011. No journal name and no DOI printed on this file. NSF grants named in the footer: CMMI-1029551, DMS-0726263, DMS-1001521, DMS-1010687, CMMI-1029445.
- Version of THIS file: arXiv v4 (9 Sep 2011). Title differs from the short CDC 2010 paper ("Geometric tracking control of a quadrotor UAV on SE(3)"). This file is the companion / extended manuscript. It contains three flight modes, hybrid switching numerics, and full appendices that the 6-page CDC version may not.
- Suggested cite key: `lee2010geometricse3`
- What this paper IS: a single-vehicle geometric tracking paper on SE(3). Three output-tracking modes (attitude, position, velocity). Almost-global Lyapunov proofs. Simulation only.
- What this paper is NOT: not multi-robot. Not Crazyflie. Not hardware. Not INDI / NA-INDI / L1. Not residual learning. Not interaction-force compensation. Not FBL-with-NN.

## 1. Problem they pose

Their wording (Intro / Abstract): "little attention has been paid to constructing nonlinear control systems that can achieve complex aerobatic maneuvers." Controllers based on Euler angles "exhibit singularities." Quaternions "have ambiguities" and can cause inconsistency, discontinuity, or unwinding. Hybrid mode papers they cite are "sensitive to switching conditions" and need "complicated reachability set analyses."

Plain restatement: they want one coordinate-free tracker on SE(3) that can flip, track position or velocity, and switch modes without Euler-angle blow-ups or quaternion double-cover bugs.

## 2. Paper map (follow their TOC)

### Abstract

- Purpose: state the claim in one block.
- Method / construction: model on SE(3). Three output-tracking problems. One nonlinear controller per mode. Closed loop "almost global" in each case.
- Key equation: none numbered.
- Assumption: four rotor thrusts are the inputs.
- Result claimed (abstract): almost-global properties per mode. Numerics include recover-from-upside-down and mode switching.

### I. Introduction

- Purpose: quadrotor motivation, attack local-coordinate and quaternion controllers, place geometric control.
- Method: literature contrast only. Unique features they list: (i) almost-global tracking on SO(3), called "unprecedented" for a quadrotor on SE(3); (ii) hybrid switching robust because basins are almost global; (iii) underactuated rigid body with translation coupled to rotation; (iv) coordinate-free.
- Key equation: none numbered.
- Assumption: complex maneuvers need a global attitude representation.
- Result: positioning only.

### II. Quadrotor Dynamics Model

- Purpose: write the SE(3) rigid-body model and the 4×4 allocation map.
- Method: inertial frame $\{\bar e_1,\bar e_2,\bar e_3\}$, body frame $\{\bar b_1,\bar b_2,\bar b_3\}$ at the CoM. Four identical rotors at square vertices. Thrust of each propeller along $-\bar b_3$. Torque of rotor $i$ written $\tau_i=(-1)^i c_{\tau f}f_i$. Inputs used later are total thrust magnitude $f=\sum f_i$ and body moment $M$.
- Key equations:
  - (1) allocation

$$
\begin{bmatrix} f \\ M_1 \\ M_2 \\ M_3 \end{bmatrix}
=
\begin{bmatrix}
1 & 1 & 1 & 1 \\
0 & -d & 0 & d \\
d & 0 & -d & 0 \\
-c_{\tau f} & c_{\tau f} & -c_{\tau f} & c_{\tau f}
\end{bmatrix}
\begin{bmatrix} f_1 \\ f_2 \\ f_3 \\ f_4 \end{bmatrix}
$$

    Determinant $8c_{\tau f}d^2$. Invertible if $d\neq 0$ and $c_{\tau f}\neq 0$.
  - (2) $\dot x=v$
  - (3) $m\dot v=mge_3-fRe_3$
  - (4) $\dot R=R\hat\Omega$
  - (5) $J\dot\Omega+\Omega\times J\Omega=M$
- Assumption: thrust of each propeller is "directly controlled." No rotor dynamics in this file (they say the law "can readily be extended" to linear rotor dynamics as in a cited paper). Gravity term is $+mge_3$. Preferred thrust in inertial frame is $-fRe_3$.
- Result: model only. Figure 1 is the vehicle sketch.

### III. Geometric Tracking Controls

- Purpose: explain why they split into modes. Four inputs, six DoF. Cannot track attitude and position at once.
- Method: a complex maneuver is a concatenation of modes plus switching conditions plus time-varying commands.
- Key equation: none numbered.
- Assumption: at most four outputs can be tracked asymptotically.
- Result claimed (body): almost-global basins make switching "robust" and avoid "complex reachability analyses." No theorem in this section.

### IV. Attitude Controlled Flight Mode

- Purpose: track a smooth $R_d(t)\in\mathrm{SO}(3)$. Translation is ignored for the reduced attitude loop. Thrust is free; one optional choice tracks altitude.
- Method: error function on SO(3). Geometric PD+feedforward moment. Reduced closed loop is (4)–(5) with (11).
- Key equations:
  - (6) $\Psi(R,R_d)=\frac12\operatorname{tr}(I-R_d^\top R)$
  - (8) $e_R=\frac12(R_d^\top R-R^\top R_d)^\vee$
  - (9) $e_\Omega=\Omega-R^\top R_d\Omega_d$, with $\hat\Omega_d=R_d^\top\dot R_d$
  - (11) $M=-k_Re_R-k_\Omega e_\Omega+\Omega\times J\Omega-J(\hat\Omega R^\top R_d\Omega_d-R^\top R_d\dot\Omega_d)$
  - (15) optional altitude thrust

$$
f=\frac{k_x(x_3-x_{3d})+k_v(\dot x_3-\dot x_{3d})+mg-m\ddot x_{3d}}{e_3\cdot Re_3}
$$

- Assumption: (12) $\Psi(R(0),R_d(0))<2$ (rotation angle $<180^\circ$). (13) bound on $\|e_\Omega(0)\|$ using $\lambda_M(J)$. For (15): $e_3\cdot Re_3\neq 0$.
- Result: Prop. 1, exponential stability of $(e_R,e_\Omega)=(0,0)$ on that set; $\Psi(t)\le\min\{2,\alpha_2 e^{-\beta_2 t}\}$. Attitudes outside the basin are $\exp(\pi\hat s)R_d$, a 2-D subset of SO(3). Prop. 2: same plus asymptotic altitude tracking. Proofs in App. B–C.

### V. Position Controlled Flight Mode

- Purpose: track smooth $x_d(t)$. Attitude is slaved to a computed $R_c$ whose third column is the desired thrust direction.
- Method: backstepping reading they give: pick $b_{3c}$ so the thrust axis tracks the force that would stabilize translation; use the attitude law to track $R_c$; set $f$ by projecting that force onto current $Re_3$. Extra freedom: choose $b_{1c}$ in the plane normal to $b_{3c}$.
- Key equations:
  - (17)–(18) $e_x=x-x_d$, $e_v=v-\dot x_d$
  - (19) $f=(k_x e_x+k_v e_v+mge_3-m\ddot x_d)\cdot Re_3$
  - (20) same moment as (11) but with $R_c,\Omega_c$
  - (21) $e_R,e_\Omega$ vs $R_c$
  - (22) $R_c=[b_{1c},\; b_{3c}\times b_{1c},\; b_{3c}]$, $\hat\Omega_c=R_c^\top\dot R_c$
  - (23)

$$
b_{3c}=-\frac{-k_x e_x-k_v e_v-mge_3+m\ddot x_d}{\|-k_x e_x-k_v e_v-mge_3+m\ddot x_d\|}
$$

  - (38) heading projection

$$
b_{1c}=-\frac{1}{\|b_{3c}\times b_{1d}\|}\bigl(b_{3c}\times(b_{3c}\times b_{1d})\bigr)
$$

- Assumption: (24) denominator $\neq 0$. (25) $\|-mge_3+m\ddot x_d\|<B$. Prop. 3: $\Psi(R(0),R_c(0))<1$ and $\|e_x(0)\|<e_{x,\max}$, plus gain conditions (31)–(33) and RoA (34)–(35). Prop. 4: if $1\le\Psi(0)<2$, only exponential attractiveness. $b_{1d}$ not parallel to $b_{3c}$.
- Result: Prop. 3 exponential stability of $(e_x,e_v,e_R,e_\Omega)=0$. Prop. 4 almost-global exponential attractiveness. Prop. 5: $b_1$ asymptotically in the plane spanned by $b_{1d}$ and $ge_3-\ddot x_d$; if $\ddot x_d=0$ and $b_{1d}$ horizontal, $b_1\to b_{1d}$. $\Omega_c,\dot\Omega_c$ "summarized in [22]" (an earlier arXiv version of this same manuscript). Figure 2 is the cascade block diagram. Figure 3 is the $b_{1c}$ projection sketch.

### VI. Velocity Controlled Flight Mode

- Purpose: track smooth $v_d(t)$. Same cascade idea without a position error.
- Method: replace the force command by $k_v e_v+mge_3-m\dot v_d$. Build $R_c$ from that direction.
- Key equations:
  - (39) $e_v=v-v_d$
  - (40) $f=(k_v e_v+mge_3-m\dot v_d)\cdot Re_3$
  - (41) moment as in (20)
  - (44) $b_{3c}=-(k_v e_v-mge_3+m\dot v_d)/\|\cdots\|$
    Note: the printed (44) numerator and the "we assume" line (45) do not use the same sign pattern. Use the numbered (44) as printed if you implement from this file.
- Assumption: (45) a non-zero force norm (printed with a mix of $e_v$ and $\ddot x_d$ symbols). (46) $\|-mge_3+m\ddot x_d\|<B$.
- Result claimed (body): $(e_v,e_R,e_\Omega)=0$ exponentially stable if $\Psi(R(0),R_c(0))<1$, exponentially attractive if $\Psi<2$. "Due to page limitations, the explicit statements of propositions and proofs … are relegated to [22]." Those proofs are not in this file.

### VII. Numerical Results Illustrating Complex Flight Maneuvers

- Purpose: two simulation cases on parameters taken from a cited large quadrotor [23].
- Method: integrate the closed-loop model. No hardware.
- Parameters printed:
  - $J=\operatorname{diag}(0.0820,0.0845,0.1377)\,\mathrm{kg\cdot m^2}$, $m=4.34\,\mathrm{kg}$
  - $d=0.315\,\mathrm{m}$, $c_{\tau f}=8.004\times 10^{-3}\,\mathrm{m}$
  - gains: $k_x=16m$, $k_v=5.6m$, $k_R=8.81$, $k_\Omega=2.54$
- Case (I): hover $x_d=[0,0,0]$, $b_{1d}=[1,0,0]$. Initial $R(0)$ nearly upside down. They report initial eigen-axis error "$178^\circ$", $\Psi(0)=1.995$. $\Psi$ falls below 1 at $t=0.88\,\mathrm{s}$. After that, position error "converges to zero exponentially" (Fig. 4).
- Case (II): five-mode sequence, same initial state as Case I.
  - (a) $t\in[0,4]$: velocity mode, $v_d=[1+0.5t,\;0.2\sin(2\pi t),\;-0.1]$, $b_{1d}=[1,0,0]$
  - (b) $t\in[4,6]$: attitude mode, $R_d(t)=\exp(2\pi(t-4)\hat e_2)$ (720° about body $e_2$)
  - (c) $t\in[6,8]$: position mode, $x_d=[14-t,0,0]$, $b_{1d}=[1,0,0]$
  - (d) $t\in[8,9]$: attitude mode, $R_d(t)=\exp(2\pi(t-8)\hat e_1)$ (360° about body $e_1$)
  - (e) $t\in[9,12]$: position mode, $x_d=[20-\frac53 t,0,0]$, $b_{1d}=[0,1,0]$
  - During (b) they set $f$ as in (19) with a frozen waypoint $x_c=[8,0,0]$. During (d) the same with $x_c=[6,0,0]$. They say this does not guarantee convergence to $x_c$.
- Assumption: numerical model equals the paper model. No noise, no delay, no rotor dynamics in the reported runs.
- Result: Figs. 4–6. Animation URL printed: `http://my.fit.edu/~taeyoung` (link status not in this file).

### VIII. Conclusions

- Purpose: restatement.
- Result they repeat: exponential stability if initial attitude error $<90^\circ$; almost-global exponential attractiveness if $<180^\circ$; complex maneuvers by switching modes.

### Appendix A. Properties of the Hat Map

- Purpose: identities used in proofs.
- Key equations: (47) hat matrix; (48)–(52) cross-product, trace, conjugation $R\hat x R^\top=(Rx)^\wedge$.

### Appendix B. Proof of Proposition 1

- Purpose: attitude-error Lyapunov proof.
- Method: $\dot\Psi=e_R\cdot e_\Omega$, $\dot e_R=C(R_d^\top R)e_\Omega$ with $\|C\|_2\le 1$, $J\dot e_\Omega=-k_Re_R-k_\Omega e_\Omega$. Candidate (57): $V_2=\frac12 e_\Omega\cdot Je_\Omega+k_R\Psi+c_2 e_R\cdot e_\Omega$. Bound $\Psi<2$ from the $c_2=0$ restriction. Then exponential decay of $V_2$.
- Result: Prop. 1.

### Appendix C. Proof of Proposition 2

- Purpose: altitude loop after substituting (15) into $e_3^\top$ of (3).
- Result: linear second-order altitude error, exponentially stable for $k_x,k_v>0$. Rotation still decoupled.

### Appendix D. Proof of Proposition 3

- Purpose: coupled translation+attitude Lyapunov proof inside domain $D$ where $\Psi<1$ and $\|e_x\|<e_{x,\max}$.
- Method: write $m\dot e_v=-k_x e_x-k_v e_v-X$ with mismatch $X$ bounded by attitude error times $\alpha=\sqrt{\psi_1(2-\psi_1)}$. $V=V_1+V_2$. Matrices $W_1,W_{12},W_2$ as (28)–(30). Condition (33) makes $\dot V$ negative definite.
- Result: Prop. 3. RoA they call "conservative."

### Appendix E. Proof of Proposition 4

- Purpose: while $\Psi$ drops from $[1,2)$ into the Prop. 3 set in finite time $t^*$, show $\|e_x\|,\|e_v\|$ stay bounded.
- Method: comparison function $V_3=\frac12\|e_x\|^2+\frac12 m\|e_v\|^2$. Bound $\dot V_3\le d_1 V_3+d_2\sqrt{V_3}$. On intervals where $V_3\ge 1$, exponential growth bound, hence boundedness on $[0,t^*]$.
- Result: Prop. 4 (attractiveness, not the stronger exponential stability).

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Unnamed "typical" quadrotor. Numbers taken from cited [23]: $m=4.34\,\mathrm{kg}$, $d=0.315\,\mathrm{m}$, $J=\operatorname{diag}(0.0820,0.0845,0.1377)\,\mathrm{kg\cdot m^2}$. Not a Crazyflie. |
| n robots min/max | 1 / 1 |
| Sim / hardware / both | Simulation only |
| Onboard vs off-board (Hz) | not in this paper (no computer, no rate) |
| Sensing (mocap, RPM, GPS, …) | not in this paper (full state $x,v,R,\Omega$ is assumed known) |
| Controller family | Geometric tracking on SE(3). Three modes: attitude, position, velocity. Cascade / backstepping reading for position and velocity. |
| How interaction enters the law | It does not. No interaction force, no downwash, no residual. |
| Learned model? inputs → output | No |
| Training data amount / source | not in this paper |
| Formations + min separation | not in this paper (single vehicle) |
| Speeds | Case II velocity command starts at $1\,\mathrm{m/s}$ in $x$ and ramps; peak numbers other than the printed $v_d(t)$ are not in this paper |
| Metrics | Time histories of $\Psi$, position, velocity, $\Omega$, per-rotor thrust, body-axis $b_1$. No RMSE table. |
| Baseline they compare to | None in the numerics. Contrast in the intro is to Euler PD/LQR, backstepping/sliding mode, quaternion attitude control, and hybrid reachability papers. |

## 4. Formations and flights (table, one row per experiment)

No multi-robot formations in this paper.

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Case I hover flip | 1 | point hover at origin, $b_{1d}=e_1$ | none | hold $x_d=0$; initial attitude $\approx 178^\circ$ from $R_c(0)$ | sim | $\Psi(0)=1.995$; $\Psi<1$ at $0.88\,\mathrm{s}$; then position error decays (Fig. 4) |
| Case II (a) velocity | 1 | single body | none | $v_d=[1+0.5t,\;0.2\sin(2\pi t),\;-0.1]$ for $4\,\mathrm{s}$ | sim | velocity error "exponentially converges"; $b_1$ stays near $\operatorname{span}\{e_1,e_3\}$ because $\|v_d\|\ll g$ (Fig. 6) |
| Case II (b) 720° pitch | 1 | attitude command about $e_2$ | waypoint $x_c=[8,0,0]$ used only inside $f$ | $R_d=\exp(2\pi(t-4)\hat e_2)$, $2\,\mathrm{s}$ | sim | full 720°; position "stays close" to $x_c$, not proven |
| Case II (c) line | 1 | straight line | none | $x_d=[14-t,0,0]$, $2\,\mathrm{s}$ | sim | follows the line (Fig. 5–6) |
| Case II (d) 360° roll | 1 | attitude command about $e_1$ | waypoint $x_c=[6,0,0]$ used only inside $f$ | $R_d=\exp(2\pi(t-8)\hat e_1)$, $1\,\mathrm{s}$ | sim | full 360°; position "remain close" to $x_c$, not proven |
| Case II (e) line + heading | 1 | straight line, $b_{1d}=e_2$ | none | $x_d=[20-\frac53 t,0,0]$, $3\,\mathrm{s}$ | sim | $b_1\to e_2$ (left of the path), via Prop. 5 |

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Mass | $4.34\,\mathrm{kg}$ | simulated vehicle mass, copied from [23] | §VII | Not a Crazyflie. Not measured here. |
| Arm length $d$ | $0.315\,\mathrm{m}$ | CoM to rotor center in the $b_1,b_2$ plane | §VII | Same source [23]. |
| Inertia diag | $0.0820,\;0.0845,\;0.1377\,\mathrm{kg\cdot m^2}$ | body inertia used in sim | §VII | Same source [23]. |
| $c_{\tau f}$ | $8.004\times 10^{-3}\,\mathrm{m}$ | torque-to-thrust coefficient | §VII | Units printed as metres. |
| $k_x$ | $16m$ | position gain | §VII | Equals $16\times 4.34$ only if you multiply; they do not print the product. |
| $k_v$ | $5.6m$ | velocity gain | §VII | Same. |
| $k_R$ | $8.81$ | attitude gain | §VII | Dimension not stated. |
| $k_\Omega$ | $2.54$ | rate gain | §VII | Dimension not stated. |
| Initial eigen-axis error | $178^\circ$ | angle between $R(0)$ and $R_c(0)$ in Case I | §VII | Computed for that one IC. |
| Initial $\Psi$ | $1.995$ | attitude error function at $t=0$ in Case I | §VII | Just below the $\Psi=2$ ceiling. |
| Time to enter $\Psi<1$ | $0.88\,\mathrm{s}$ | Case I | §VII / Fig. 4(a) | One trajectory, one gain set. |
| Attitude-mode basin | $\Psi<2$ ($<180^\circ$) | Prop. 1 | §IV | Almost-global, not global. Topological obstruction cited as [20]. |
| Coupled exponential basin | $\Psi<1$ ($<90^\circ$) | Prop. 3 | §V | Plus $\|e_x\|<e_{x,\max}$ and several gain inequalities. $e_{x,\max}$ is "a fixed constant," no number. |
| Allocation invertibility | $\det=8c_{\tau f}d^2$ | 4×4 map (1) | §II | Needs $d\neq 0$, $c_{\tau f}\neq 0$. |

Abstract and body do not disagree on a quantitative metric. The abstract has no RMSE / "× smaller" number.

## 6. Assumptions and failure modes

- Each propeller thrust is a direct input. No motor lag in the theorems.
- Torque strictly proportional to thrust with one constant $c_{\tau f}$.
- Full state $x,v,R,\Omega$ available. No estimator.
- Commands $R_d(t)$, $x_d(t)$, $v_d(t)$ are smooth. Position mode also needs $\ddot x_d$. Velocity mode needs $\dot v_d$.
- Force command never zero: (24), (45). If it hits zero, $b_{3c}$ is undefined.
- Commanded acceleration bounded: (25), (46).
- Attitude-mode altitude law (15) dies when $e_3\cdot Re_3=0$ (vehicle on its side). They refuse to use Prop. 2 during the 720° flip for that reason.
- Smooth controllers on SO(3) cannot be globally asymptotically stable (they cite the topological obstruction).
- Exponential stability of the coupled loop needs $\Psi<1$. Between $1$ and $2$ they only claim attractiveness, after a finite-time wait, with a conservative Lyapunov RoA.
- $b_{1d}$ must not be parallel to $b_{3c}$ or (38) divides by zero.
- No disturbance, no interaction wrench, no actuator saturation in the theorems. Per-rotor thrust is plotted in Fig. 4(d) but no saturation model is stated.
- Switching: they claim robustness from almost-global basins. They do not prove a hybrid-system theorem (no dwell time, no common Lyapunov function across modes).

## 7. Relation to our thesis (clearly labelled OUR READING)

- Which of our strategies it supports: **strategy 1, geometric baseline**, as the classical single-vehicle geometric tracker. It does not support 2 INDI, 3 geo+NN, 4 FBL+residual, or 5 hybrid NA-INDI/L-INDI.
- What we may cite: SE(3) kinematics (2)–(5); $\Psi,e_R,e_\Omega$; moment (11); position thrust (19) and $b_{3c}$ (23); heading projection (38) if we constrain yaw that way; the $\Psi<1$ vs $\Psi<2$ distinction; the fact that Euler / quaternion representations are why they moved to rotation matrices.
- What we must NOT cite from it: multi-robot interaction; Crazyflie / STM32 / OptiTrack results; onboard rates; residual subtraction; INDI; learned models; formation geometry; any "hardware tracking error." Also do not cite CDC 2010 for three flight modes, velocity mode, Prop. 5, or Case II switching. Those are in this companion.
- Formations we should reproduce if we use it as a reference condition: none as a team formation. Useful single-vehicle reference conditions: hover from a large attitude error; a straight-line $x_d$ with a frozen $b_{1d}$. Do not treat Case II waypoints $x_c=[8,0,0]$ or $[6,0,0]$ as proven position tracking.
- Compute: would this run on an STM32? The law is a few 3×3 products, one normalize, and a 4×4 allocation solve. That class of arithmetic can run on a microcontroller. What they actually ran: desktop numerical integration. No MCU, no timing, no fixed-point discussion in this file.

## 8. Equations worth keeping

Symbols: $R\in\mathrm{SO}(3)$ body-to-inertial; $\Omega$ body rate; $x,v$ inertial position/velocity; $f$ total thrust magnitude; $M$ body moment; $e_3=(0,0,1)$; $\hat\cdot$ hat map (47).

1. Dynamics (3)–(5):

$$
m\dot v=mge_3-fRe_3,\qquad \dot R=R\hat\Omega,\qquad J\dot\Omega+\Omega\times J\Omega=M.
$$

2. Attitude error (6), (8), (9):

$$
\Psi=\frac12\operatorname{tr}(I-R_d^\top R),\qquad
e_R=\frac12(R_d^\top R-R^\top R_d)^\vee,\qquad
e_\Omega=\Omega-R^\top R_d\Omega_d.
$$

3. Attitude moment (11):

$$
M=-k_Re_R-k_\Omega e_\Omega+\Omega\times J\Omega-J(\hat\Omega R^\top R_d\Omega_d-R^\top R_d\dot\Omega_d).
$$

4. Position thrust (19) and computed axis (23):

$$
f=(k_x e_x+k_v e_v+mge_3-m\ddot x_d)\cdot Re_3,
$$

$$
b_{3c}=(-k_x e_x-k_v e_v-mge_3+m\ddot x_d)/\|\cdots\| \text{ after cancelling the printed double minus in (23).}
$$

5. Position moment (20): same as (11) with $R_d\to R_c$, $\Omega_d\to\Omega_c$.

6. Heading completion (38):

$$
b_{1c}=-\|b_{3c}\times b_{1d}\|^{-1}\bigl(b_{3c}\times(b_{3c}\times b_{1d})\bigr).
$$

7. Velocity thrust (40):

$$
f=(k_v e_v+mge_3-m\dot v_d)\cdot Re_3.
$$

8. Allocation (1): four rotor thrusts from $(f,M)$.

## 9. Figures / tables worth remembering

- Fig. 1: vehicle sketch. Frames, four rotors, $x$, $R$.
- Fig. 2: position-mode cascade. Force block outputs $b_{3c}$ and $f$; moment block tracks $R_c$; plant returns $x,v,R,\Omega$.
- Fig. 3: geometry of $b_{1c}=\mathrm{Proj}[b_{1d}]$ onto the plane normal to $b_{3c}$.
- Fig. 4: Case I. (a) $\Psi(t)$ crosses 1 at $0.88\,\mathrm{s}$. (b) position vs $x_d=0$. (c) angular velocity. (d) four rotor thrusts in newtons.
- Fig. 5: Case II cartoon. Path with mode labels (a)–(e), including the 720° and 360° loops.
- Fig. 6: Case II time histories. Velocity tracking, position, first body axis vs $b_{1d}$, plus the other panels printed on that page.
- No numerical results table. No RMSE table.

## 10. Open uncertainties

- CDC 2010 vs this file: this file does not reprint the CDC page count or a "differences from the conference version" list. Treat three modes, velocity mode, Prop. 5, Case II, and Apps. B–E as belonging to this companion unless you open CDC separately.
- $\Omega_c$ and $\dot\Omega_c$ explicit formulae: not printed. Pointed to [22] = arXiv 1003.2005v3.
- Velocity-mode propositions and proofs: not printed. Same [22].
- Sign clash in §VI: numbered (44) vs assumption (45) / the $\ddot x_d$ symbol in a velocity section.
- Sample rate, integrator, language, ODE solver: not in this paper.
- How they pick $b_{1c}$ in Case I before using (38): Case I states $b_{1d}=[1,0,0]$ and invokes the position mode, so Prop. 5 is the intended rule.
- Sensor model, noise, delay, quantization: not in this paper.
- Motor saturation and propeller map $f_i(\omega_i)$: not in this paper. Fig. 4(d) has thrust in N but no limit line.
- Numerical values of $e_{x,\max}$, $B$, $c_1$, $c_2$, $\psi_1$ used (if any) in the plotted runs: not in this paper. Only $k_x,k_v,k_R,k_\Omega$ are given.
- Whether gravity axis $e_3$ is "down" in their inertial frame: implied by $+mge_3$ in (3), not named NED/ENU.

## 11. 12-line abstract I can reread

Lee–Leok–McClamroch, arXiv:1003.2005v4 (2011), companion to CDC 2010.
Single rigid quadrotor on SE(3). Inputs: total thrust $f$ and body moment $M$, mapped to four rotor thrusts by (1).
Three modes: track $R_d$; track $x_d$ via computed $R_c$; track $v_d$ via a similar $R_c$.
Attitude law is geometric PD plus cancellation of $\Omega\times J\Omega$ and the $\Omega_d$ feedforward. Error function $\Psi=\frac12\operatorname{tr}(I-R_*^\top R)$.
Coupled position loop is exponentially stable only if initial $\Psi<1$. For $1\le\Psi<2$ they claim exponential attractiveness, not exponential stability.
Simulation vehicle: $4.34\,\mathrm{kg}$, $d=0.315\,\mathrm{m}$, inertia from a cited large quadrotor. Not a Crazyflie.
Case I: recover from $\Psi(0)=1.995$ to hover. $\Psi<1$ at $0.88\,\mathrm{s}$.
Case II: velocity, 720° about $e_2$, line, 360° about $e_1$, line with $b_{1d}=e_2$. Switching is demonstrated, not proven as a hybrid system.
No hardware, no team, no interaction force, no learned residual, no INDI.
Cite CDC only for the shared geometric tracker. Cite this file for modes, velocity law, heading projection (38), and the flip / switching plots.
Use it as the geometric baseline reference. Do not use it as evidence for residual compensation on STM32 teams.

Source: attached PDF only. No other paper used.
