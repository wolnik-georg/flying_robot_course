# Tal–Karaman INDI+flatness — Tal et al., 2019 (arXiv 2018)

## 0. Identity

- Full title: Accurate Tracking of Aggressive Quadrotor Trajectories using Incremental Nonlinear Dynamic Inversion and Differential Flatness
- Authors: Ezra Tal, Sertac Karaman (MIT AeroAstro / LIDS)
- Year / venue / arXiv / DOI: arXiv:1809.04048v2 [cs.RO], 7 Jun 2019. Preliminary version: IEEE CDC 2018 [1]. No IEEE journal pagination or DOI is printed on this file.
- Version of THIS file: arXiv v2 (7 Jun 2019). Relative to the CDC 2018 short version, this file adds quaternion attitude, a fuller architecture write-up, the entire response-analysis section, and faster flight experiments.
- Suggested cite key: `tal2018indi`
- What this paper IS: a single-quadrotor trajectory tracker that pairs differential-flatness feedforward of jerk/snap with INDI on linear and angular acceleration, plus encoder-based closed-loop motor speed for torque/snap.
- What this paper is NOT: not multi-robot; not Crazyflie; not a learned residual; not Smeur cascaded position-INDI / wind-tunnel work; not GPS outdoor flight; not an interaction-force team paper.

## 1. Problem they pose

Their wording (abstract / §I): accurate tracking of aggressive (high-speed, high-acceleration) quadrotor trajectories. At high speed, aerodynamic drag is "hard to model" and becomes dominant. Fast-changing acceleration also requires jerk and snap. Low-speed VTOL design usually drops both.

Plain restatement: keep centimetre tracking when the vehicle is fast and the aero force is unknown, by feeding forward high-order flatness terms and incrementing thrust/torque from sensors instead of fitting drag.

## 2. Paper map (follow their TOC)

### Abstract

- Purpose: claim, method headline, one performance sentence, robustness tests.
- Method / construction: track position and yaw and derivatives up to fourth order. Jerk/snap via flatness feedforward on angular rate and angular acceleration. Snap via body torque from closed-loop motor speeds (optical encoders). INDI for linear and angular acceleration. "Hence, prior modeling of aerodynamic effects is not required."
- Key equation: none numbered.
- Assumption: optical encoders available; disturbances exist.
- Result claimed (abstract): "speeds up to 12.9 m/s and accelerations up to 2.1g" with "root-mean-square tracking error down to 6.6 cm" in a volume "roughly 18 m by 7 m and 3 m tall." Also drag plate in flight and rope pull in hover. Video: https://youtu.be/K15lNBAKDCs.

### I. Introduction

- Purpose: drone-racing / high-speed navigation context. NDI robustness problem. Place INDI and flatness. List four novelty claims.
- Method / construction: none yet. They say existing INDI on quadrotors is for stabilization / hover, "not for trajectory tracking."
- Key equation: none.
- Assumption: aero drag is the main unmodelled force at speed; snap needs torque, not a body-rate-only inner loop.
- Result claimed: contribution list. (i) snap via encoder motor-speed control. (ii) INDI tailored to trajectory tracking. (iii) nonlinear (not linearised) INDI control-effectiveness inversion. (iv) experiments plus response analysis. Quaternion rewrite vs CDC 2018 avoids Euler singularities.

### II. Preliminaries

- Purpose: rigid-body model and the flatness maps used as feedforward.
- Method / construction: NED inertial frame. Thrust along $b_z$. External force $f_{\mathrm{ext}}$ and moment $\mu_{\mathrm{ext}}$ lumped. Motors $\omega>0$. Flat outputs $\sigma_{\mathrm{ref}}=[x_{\mathrm{ref}}^\top,\psi_{\mathrm{ref}}]^\top$. They drop aero from the flatness transform on purpose (contrast [29] in their refs).
- Key equations:
  - (1) $\dot{x}=v$
  - (2) $\dot{v}=g i_z+\tau b_z+m^{-1}f_{\mathrm{ext}}$
  - (3) $\dot{\xi}=\tfrac12\xi\circ\Omega$
  - (4) $\dot{\Omega}=J^{-1}(\mu+\mu_{\mathrm{ext}}-\Omega\times J\Omega)$
  - (5) $\begin{bmatrix}\mu\\T\end{bmatrix}=G_1\omega^{\circ 2}+G_2\dot{\omega}$
  - (8) $\sigma_{\mathrm{ref}}(t)=[x_{\mathrm{ref}}(t)^\top\,\psi_{\mathrm{ref}}(t)]^\top$
  - (9) $j=\tau R[i_z]_\times^\top\Omega+\dot{\tau}b_z$
  - (10) snap, includes $\dot{\Omega}$
  - (14) $\Omega_{\mathrm{ref}},\dot{\tau}_{\mathrm{ref}}$ from $j_{\mathrm{ref}},\dot{\psi}_{\mathrm{ref}}$
  - (15) $\dot{\Omega}_{\mathrm{ref}},\ddot{\tau}_{\mathrm{ref}}$ from $s_{\mathrm{ref}},\ddot{\psi}_{\mathrm{ref}}$
- Assumption: $x_{\mathrm{ref}}\in C^4$, $\psi_{\mathrm{ref}}\in C^2$. Propeller axes parallel to $b_z$. Motor inertia not in total angular momentum. Variations of $f_{\mathrm{ext}}$ neglected inside (9)–(10). $\dot{\tau}_{\mathrm{ref}},\ddot{\tau}_{\mathrm{ref}}$ unused because they cannot command $\dot{\omega},\ddot{\omega}$ that way.
- Result claimed: feedforward $\Omega_{\mathrm{ref}},\dot{\Omega}_{\mathrm{ref}}$ exist from jerk/snap/yaw derivatives.

### III. Trajectory Tracking Control

- Purpose: full cascade. Table II lists each block.
- Method / construction: outer PD position/velocity → INDI linear accel + yaw → flatness jerk/snap → PD attitude/rate → INDI angular accel → nonlinear inversion to $\omega_c$ → integrative throttle. State: pose from mocap+IMU; $\omega$ from encoders; $\dot{\Omega}$ by differentiating rate; LPF subscript $f$.
- Key equations: see §III-A–E below and §8.
- Assumption: identical LPF on paired signals. $f_{\mathrm{ext}}$ and $\mu_{\mathrm{ext}}$ slow vs LPF.
- Result claimed: architecture only in this heading; performance in §V.

### III-A. PD Position and Velocity Control

- Purpose: commanded inertial acceleration.
- Method: two cascaded PDs written as one law, plus $a_{\mathrm{ref}}$ feedforward.
- Key equation: (17)
  $a_c=K_x(x_{\mathrm{ref}}-x)+K_v(v_{\mathrm{ref}}-v)+K_a(a_{\mathrm{ref}}-a_f)+a_{\mathrm{ref}}$
- Assumption: same horizontal gains on $i_x,i_y$; separate vertical gains.
- Result: $a_c$ is the input to INDI thrust/attitude.

### III-B. INDI Linear Acceleration and Yaw Control

- Purpose: get $T_c$ and incremental attitude $\xi_c$ without modelling $f_{\mathrm{ext}}$.
- Method: estimate $f_{\mathrm{ext}}$ from filtered accel and filtered specific thrust. Increment $(\tau b_z)_c$. Magnitude → thrust. Align $-b_z$ with the thrust command, then rotate about $b_z$ for yaw. Quaternion, relative to current attitude.
- Key equations:
  - (18) $f_{\mathrm{ext}}=m\bigl(a_f-(\tau b_z)_f-g i_z\bigr)$
  - (20) $(\tau b_z)_c=(\tau b_z)_f+a_c-a_f$
  - (21) $T_c=-m\|(\tau b_z)_c\|_2$
  - (26) $\xi_c=\bar{\xi}_c\circ\xi_\psi$
- Assumption: $f_{\mathrm{ext}}$ in inertial frame is slow vs LPF. Filter accel and thrust after rotating to inertial. Singularities of (23),(25) exist but are moved away by incremental attitude; they say detect-and-pick-a-direction.
- Result claimed: no integral term needed anywhere if increments keep running.

### III-C. PD Attitude and Angular Rate Control

- Purpose: $\dot{\Omega}_c$ from attitude error plus rate/accel feedforward.
- Method: kinematics only. No $J$ in this loop. Error angles from $\xi_c$.
- Key equations:
  - (27) $\xi_e$ from $\xi_c$
  - (28) $\dot{\Omega}_c=K_\xi\xi_e+K_\Omega(\Omega_{\mathrm{ref}}-\Omega_f)+\dot{\Omega}_{\mathrm{ref}}$
- Assumption: $\Omega_{\mathrm{ref}},\dot{\Omega}_{\mathrm{ref}}$ from (14),(15). Filtered rate in the rate error.
- Result claimed: this is how jerk and snap enter the inner loop. Body-rate-only inner loops "cannot truly consider reference snap."

### III-D. INDI Angular Acceleration Control

- Purpose: $\mu_c$ that tracks $\dot{\Omega}_c$ despite $\mu_{\mathrm{ext}}$.
- Method: estimate $\mu_{\mathrm{ext}}$ from filtered $\dot{\Omega}$, $\mu$, gyroscopic term. Invert incrementally.
- Key equations:
  - (29) $\mu_{\mathrm{ext}}=J\dot{\Omega}_f-\mu_f+\Omega_f\times J\Omega_f$
  - (31) $\mu_c=\mu_f+J(\dot{\Omega}_c-\dot{\Omega}_f)$
- Assumption: gyroscopic term minus its filtered copy is small (slow, second-order). $\mu_{\mathrm{ext}}$ slow vs LPF. $\mu_f$ from (5) and encoder $\omega$.
- Result claimed: closed-loop angular acceleration, not open moment command.

### III-E. Moment/Thrust Inversion and Motor Speed Control

- Purpose: turn $\mu_c,T_c$ into ESC throttle $\zeta$.
- Method: first-order motor model (32). Solve nonlinear (33) (Newton). If saturated, first clip yaw moment $\mu_c^z$ using (34); then maybe change $T_c$; else clip $\omega_c$. Throttle: static polynomial plus integral on speed error.
- Key equations:
  - (32) $\dot{\omega}=\tau_m^{-1}(\omega_c-\omega)$
  - (33) $\begin{bmatrix}\mu_c\\T_c\end{bmatrix}=G_1\omega_c^{\circ 2}+\tau_m^{-1}G_2(\omega_c-\omega)$
  - (36) $\zeta=p(\omega_c)+K_{I\omega}\int(\omega_c-\omega)\,dt$
- Assumption: $\tau_m$ known. $p(\cdot)$ from static tests. Integral covers battery voltage sag. Raw (unfiltered) $\omega$ in (36) to cut lag.
- Result claimed: better than linearised inversion (52) in principle; in their flights the main practical win is including $\tau_m$ (avoids fast yaw oscillation if $\tau_m$ and $\Delta t$ differ a lot).

### IV. Response Analysis

- Purpose: linearised pitch/forward hover analysis. Compare INDI vs non-incremental. Show jerk/snap zeros.
- Method: linearise (2),(4),(20),(28),(33) about hover. Fig. 6 block diagram. Simulate with Table III gains, 30 Hz Butterworth, $\tau_m=20$ ms.
- Key equations:
  - (44) $\alpha/\alpha_c=M(s)$ for the incremental plant (nominal $\Delta=1$)
  - (45) disturbance moment path has factor $(1-H(s)M(s))$
  - (47) model-error $\Delta$: $\alpha/\alpha_c=\Delta M(s)/((\Delta-1)H M+1)$
  - (48) non-incremental: $\alpha/\alpha_{c,\mathrm{NI}}=\Delta M(s)$
  - (50) $a_x/a_{x,\mathrm{ref}}=M(s)(s^2+k_q s+k_\theta)/(s^2+k_q H M s+k_\theta M)$
- Assumption: linear hover. Qualitative claims meant to carry to the nonlinear law.
- Result claimed: INDI rejects force/moment steps to zero error without an extra integrator (Fig. 7). Model error $\Delta$ hurts transients, not the final accel (Figs. 8–9). Jerk/snap feedforward speeds the accel response (Fig. 10).

### V. Experimental Results

- Purpose: hardware tracking and ablations.
- Method / construction: see §V-A–F and tables in §4–5.
- Key equation in this part: (51) roulette curve.
- Assumption: OptiTrack + IMU latency compensation; encoder speeds valid in flight.
- Result claimed: numbers in Tables IV–V and hover wire test. Same gains on a second (unspecified) quadrotor after swapping only platform parameters; "no loss of tracking accuracy" (qualitative).

### V-A. Experimental Setup

- Purpose: vehicle, sensors, rates, gains.
- Platform: machined carbon/balsa. T-Motor F35A + F40 Pro II 2400 KV + Gemfan Hulkie 5055. Adjacent motors 18 cm apart. 4S LiPo. Flying mass 609 g.
- Compute: STM32H7 400 MHz, custom firmware, 2000 Hz, 16 µs per update at 32-bit float.
- Sensing: ADIS16477-3 IMU 2000 Hz. OptiTrack 360 Hz, average latency 18 ms, IMU-propagated. Optical encoders ~5000 Hz. Butterworth LPF 188.5 rad/s (30 Hz) on IMU and motor speed.
- Gains (Table III): $K_x=\mathrm{diag}([18,18,13.5])$, $K_v=\mathrm{diag}([7.8,7.8,5.9])$, $K_a=\mathrm{diag}([0.5,0.5,0.3])$, $K_\xi=\mathrm{diag}([175,175,82])$, $K_\Omega=\mathrm{diag}([19.5,19.5,19.2])$.
- Platform-specific list they retune: $m$, $J$, $\tau_m$, $G_1,G_2$, motor-map gain/polynomial. Gains called "platform-independent."

### V-B–F. Flight cases

- V-B: 3-D waypoint path [50] with forward yaw and constant yaw. Roulette (51), one lap 22.4 s.
- V-C: same roulette with $\Omega_{\mathrm{ref}}=\dot{\Omega}_{\mathrm{ref}}=0$. RMS position 9.0 cm → 16.8 cm.
- V-D: 16 cm × 32 cm cardboard plate on the underside. "More than triples the frontal surface area." Controller not retuned. Position RMS 7.6 cm vs 9.0 cm without plate. Yaw can spike (max 51.7 deg) when the plate yaws the vehicle into motor saturation.
- V-E: nonlinear (33) vs linearised (52). "No significant differences in tracking performance" on this vehicle. Linearised form can oscillate in yaw if $\tau_m$ and $\Delta t$ mismatch.
- V-F: hover. No extra disturbance: "sub-centimeter" position error. Wire pull: error "within at most 4 cm"; largest error near 10 s at "approximately 3.7 N."

### VI. Conclusions

- Purpose: restate design and headline numbers.
- Result claimed again: 6.6 cm RMS, 12.9 m/s, 2.1 g, 18×7×3 m volume. No aero-parameter model.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Custom carbon/balsa quadrotor, 609 g flying mass. Motors 18 cm apart. No commercial name. |
| n robots min/max | 1 / 1 |
| Sim / hardware / both | Hardware flights. Linearised response analysis in simulation (§IV). |
| Onboard vs off-board (Hz) | Full control law onboard STM32H7 at 2000 Hz (16 µs/update). Mocap pose is off-board at 360 Hz, fused onboard. |
| Sensing (mocap, RPM, GPS, …) | OptiTrack pose/vel/att. ADIS16477-3 IMU. Optical encoders on each motor hub. No GPS. |
| Controller family | Differential flatness feedforward + cascaded PD + INDI on linear and angular acceleration + nonlinear allocation + integrative RPM loop |
| How interaction enters the law | Not inter-vehicle. External wrench is $f_{\mathrm{ext}},\mu_{\mathrm{ext}}$. Cancelled by INDI increments (18),(20),(29),(31). Drag plate and rope are single-body disturbances. |
| Learned model? inputs → output | No. Static bench fit of $G_1,G_2,\tau_m,p(\cdot)$. |
| Training data amount / source | not in this paper (no learning) |
| Formations + min separation | none. Single vehicle. |
| Speeds | Max 12.9 m/s (3-D, forward yaw). Roulette max 7.3 m/s (proposed). |
| Metrics | RMS and max $\|x-x_{\mathrm{ref}}\|_2$, yaw error, $\|v\|_2$, $\|a-g i_z\|_2$ |
| Baseline they compare to | Same law without jerk/snap FF. Linearised vs nonlinear allocation. INDI vs non-incremental in simulation only. No geometric/SE(3) or NN residual baseline. |

## 4. Formations and flights (table, one row per experiment)

No multi-robot formation exists in this paper.

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Single 3-D, forward yaw | 1 | free | n/a | waypoint 3-D; max 12.9 m/s, max 20.8 m/s² | hw | Table IV: RMS pos 6.6 cm, max 10.8 cm; RMS yaw 5.1 deg |
| Single 3-D, constant yaw | 1 | free | n/a | same family; max 11.3 m/s, max 20.0 m/s² | hw | Table IV: RMS pos 6.1 cm, max 11.9 cm; RMS yaw 1.9 deg |
| Roulette, proposed | 1 | free, $r_z$ constant | n/a | (51), lap 22.4 s; max 7.3 m/s | hw | Table V(i): RMS pos 9.0 cm, max 14.3 cm |
| Roulette, no jerk/snap FF | 1 | same | n/a | same (51) | hw | Table V(ii): RMS pos 16.8 cm, max 28.9 cm |
| Roulette + drag plate | 1 | plate 16 cm × 32 cm under body | n/a | same (51); max 7.7 m/s | hw | Table V(iii): RMS pos 7.6 cm; max yaw 51.7 deg |
| Hover, no pull | 1 | fixed $\sigma_{\mathrm{ref}}$ | n/a | hover | hw | "sub-centimeter" position error |
| Hover + tensioned wire | 1 | wire on bottom plate | n/a | hover; force ~3.7 N peak | hw | position error "at most 4 cm" (Fig. 16–18) |
| Linearised analysis | 1 | hover linearisation | n/a | steps and (49) accel profile | sim | Figs. 7–10 |

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Top speed | 12.9 m/s | max $\|v\|_2$ on 3-D forward-yaw run | abstract, Table IV | That trajectory and yaw mode only. Constant-yaw max is 11.3 m/s. |
| Top acceleration | 2.1 g / 20.8 m/s² | max $\|a-g i_z\|_2$ same run | abstract says 2.1 g; Table IV says 20.8 m/s² (2.12 g) | Abstract rounds. Constant-yaw max 20.0 m/s². |
| RMS tracking | 6.6 cm | RMS $\|x-x_{\mathrm{ref}}\|_2$, 3-D forward yaw | abstract, Table IV | Constant yaw 6.1 cm. Roulette proposed 9.0 cm. Not a multi-run statistic. |
| Max position error | 10.8 cm | max $\|x-x_{\mathrm{ref}}\|_2$, 3-D forward yaw | Table IV | Constant yaw 11.9 cm. Roulette proposed 14.3 cm. |
| Volume | ~18 m × 7 m × 3 m | indoor flight room | abstract, §V | Room size, not trajectory bounding box. |
| Mass | 609 g | flying mass | §V-A | Custom vehicle, not Crazyflie. |
| Control rate | 2000 Hz / 16 µs | onboard update | §V-A | Does not include mocap delay. |
| Mocap | 360 Hz, 18 ms latency | pose stream | §V-A | Corrected by IMU integration. |
| LPF cutoff | 30 Hz (188.5 rad/s) | 2nd-order Butterworth | §V-A and §IV | Same number in flight and analysis. |
| Encoder rate | ~5000 Hz | optical motor speed | §V-A | Approximate. |
| Motor spacing | 18 cm | adjacent motors | §V-A | Not a formation separation. |
| Drag plate | 16 cm × 32 cm | cardboard under body | §V-D, Fig. 14 | Area change qualitative ("more than triples frontal surface"). |
| Wire force | ~3.7 N | INDI estimate (18), not a load cell | §V-F | "Approximately." Direction checked vs photos. |
| Hover error under wire | ≤ 4 cm | Euclidean position | §V-F | Time-varying pull. Undisturbed hover is "sub-centimeter." |
| Roulette lap | 22.4 s | one lap of (51) | §V-B | Parameters listed under (51). |
| Sim motor lag | $\tau_m=20$ ms | analysis only | §IV | Flight $\tau_m$ value not printed. |

Abstract and Table IV agree on 12.9 m/s and 6.6 cm. Acceleration is 2.1 g in the abstract and 20.8 m/s² in Table IV. No retraction.

## 6. Assumptions and failure modes

- $f_{\mathrm{ext}},\mu_{\mathrm{ext}}$ slow compared with the 30 Hz LPF.
- Motor axes aligned with $b_z$; $\omega>0$.
- Encoder $\omega$ available at high rate. Without it, their $\mu_f,T_f$ and snap path break.
- $\tau_m$, $G_1$, $G_2$, $J$, $m$, $p(\cdot)$ identified offline. INDI recovers large gain error in simulation, but transients change.
- Gyroscopic mismatch after filtering neglected in (30).
- $\dot{\tau},\ddot{\tau}$ references from flatness are computed then discarded.
- Motor saturation: yaw moment first, then thrust, then clip. Yaw can still blow up (drag-plate max 51.7 deg).
- Quaternion increment singularities exist if $i_z=(-b_z)_c^b$ or the yaw normal $x$-component is zero. Incremental form moves them off the nominal path.
- Pose depends on mocap. Not a vision-only or GPS paper.
- Analysis is linearised hover. Flights are one vehicle in still indoor air plus two staged disturbances (plate, rope).
- No proof that encoder-grade RPM equals ESC telemetry RPM.

## 7. Relation to our thesis (clearly labelled OUR READING)

- Which of our strategies it supports: **Strategy 2 (full INDI)**. Flatness feedforward of jerk/snap is part of their INDI stack, not a separate learned residual. It is not geo+NN, not FBL+NN, not hybrid NA-INDI unless we add a residual on top (they do not).
- What we may cite: INDI on linear and angular acceleration; no parametric drag model; snap needs a torque/RPM inner loop; encoder-based RPM in *their* hardware; single-vehicle aggressive tracking numbers as *their* result; drag-plate and rope as disturbance-rejection tests; $f_{\mathrm{ext}}$ estimate (18) as a wrench observer idea.
- What we must NOT cite from it: multi-robot formations or downwash stacks; Crazyflie mass/gains; our RMS as theirs; optical encoders as our sensor (thesis deviation: platform RPM); Smeur wind-tunnel/GPS/cascaded position INDI; "no model at all" (they still use $m,J,G_1,G_2,\tau_m$); onboard-only navigation (mocap is in the loop).
- Formations we should reproduce if we use it as a reference condition: none from this paper. Useful single-vehicle reference conditions only: aggressive 3-D tracking, high-jerk curve, added drag, hover wrench. Do not invent a stack geometry and pin it on Tal–Karaman.
- Compute: the *control law* ran on STM32H7 at 2 kHz / 16 µs. That supports "INDI fits a Cortex-M MCU." Their vehicle is 609 g with a 400 MHz H7 and a tactical IMU, not a stock Crazyflie brushless stack. Mocap is still required for $x,v,\xi$.

## 8. Equations worth keeping

Paper numbering. Symbols as in their Table I.

1. Translation (2): $\dot{v}=g i_z+\tau b_z+m^{-1}f_{\mathrm{ext}}$. $\tau=T/m$, thrust along $b_z$.
2. Rotation (4): $\dot{\Omega}=J^{-1}(\mu+\mu_{\mathrm{ext}}-\Omega\times J\Omega)$.
3. Allocation (5): $\begin{bmatrix}\mu\\T\end{bmatrix}=G_1\omega^{\circ 2}+G_2\dot{\omega}$.
4. Flatness rate (14): $\begin{bmatrix}\Omega_{\mathrm{ref}}\\\dot{\tau}_{\mathrm{ref}}\end{bmatrix}=\begin{bmatrix}\tau R[i_z]_\times^\top & b_z\\ S & 0\end{bmatrix}^{-1}\begin{bmatrix}j_{\mathrm{ref}}\\\dot{\psi}_{\mathrm{ref}}\end{bmatrix}$.
5. Outer accel (17): $a_c=K_x(x_{\mathrm{ref}}-x)+K_v(v_{\mathrm{ref}}-v)+K_a(a_{\mathrm{ref}}-a_f)+a_{\mathrm{ref}}$.
6. INDI force (18)+(20): $f_{\mathrm{ext}}=m(a_f-(\tau b_z)_f-g i_z)$, then $(\tau b_z)_c=(\tau b_z)_f+a_c-a_f$.
7. Attitude/rate (28): $\dot{\Omega}_c=K_\xi\xi_e+K_\Omega(\Omega_{\mathrm{ref}}-\Omega_f)+\dot{\Omega}_{\mathrm{ref}}$.
8. INDI moment (31): $\mu_c=\mu_f+J(\dot{\Omega}_c-\dot{\Omega}_f)$.
9. Motor inversion (33): $\begin{bmatrix}\mu_c\\T_c\end{bmatrix}=G_1\omega_c^{\circ 2}+\tau_m^{-1}G_2(\omega_c-\omega)$.

## 9. Figures / tables worth remembering

- Fig. 1: vehicle, body axes $b_x,b_y,b_z$, moment arms $\ell_x,\ell_y$.
- Fig. 2: outer PD position/velocity (blue) into acceleration/attitude block.
- Fig. 3: middle loop. Blue INDI accel/yaw; green flatness jerk/snap; red attitude/rate; yellow INDI angular accel.
- Fig. 4: inner allocation + RPM loop + LPF/IMU/encoder signal paths.
- Fig. 5: motor hub, optical encoder lens, reflective strip. Propeller off.
- Fig. 6: linearised forward-accel / pitch block diagram for §IV.
- Fig. 7: simulated position step under force and moment disturbance; INDI vs non-incremental.
- Fig. 8: angular-accel step vs model-error $\Delta$.
- Fig. 9: linear-accel tracking of (49) vs $\Delta$; incremental vs non-incremental.
- Fig. 10: same accel profile with vs without jerk/snap FF.
- Fig. 11–12: 3-D trajectory tracking, two yaw modes, error/speed/accel time series.
- Fig. 13: roulette path and errors; proposed / no FF / drag plate.
- Fig. 14: vehicle with cardboard plate.
- Fig. 15: estimated $f_{\mathrm{ext}}$ on roulette, with and without plate (plate ~3×).
- Fig. 16–18: hover wire-pull error, estimated force, photos of wire direction.
- Table I: nomenclature.
- Table II: controller block list.
- Table III: gains (printed values in §V-A above).
- Table IV: 3-D metrics, two yaw modes.
- Table V: roulette metrics, three columns.

## 10. Open uncertainties

- Exact waypoint list for the 3-D path: not in this paper (generated by [50]).
- Numerical $G_1,G_2$, $J$, $\tau_m$ used in flight: not printed (static-test source only). Analysis uses $\tau_m=20$ ms.
- Polynomial $p(\cdot)$ coefficients: not printed.
- $K_{I\omega}$ value: not printed.
- How many flights per table cell: not in this paper.
- Second quadrotor they retuned: identity and specs not in this paper.
- Wire attachment coordinates and independent force sensor: not in this paper (force is the INDI estimate).
- Whether ESC telemetry could replace the optical encoder: not tested here.
- Outdoor / GPS / onboard SLAM: not in this paper.

## 11. 12-line abstract I can reread

Tal and Karaman track a single indoor quadrotor on aggressive 3-D paths with INDI plus differential-flatness feedforward of jerk and snap.
Position and yaw are the flat outputs; references go through fourth-order position derivatives and second-order yaw derivatives.
INDI increments specific thrust and body moment from filtered IMU and encoder-based motor speeds, so drag is not identified.
Snap is treated as torque, which they apply by closing a motor-speed loop on optical encoders.
The law runs at 2 kHz on an STM32H7. Pose still comes from OptiTrack.
Hardware: 609 g custom frame, not a Crazyflie. One robot only.
Headline 3-D run: 12.9 m/s, 20.8 m/s², 6.6 cm RMS position (forward yaw).
Ablation: dropping jerk/snap FF raises roulette RMS from 9.0 cm to 16.8 cm.
A large underside drag plate does not wreck position tracking; a hover rope pull of about 3.7 N stays within 4 cm.
They still need $m,J,G_1,G_2,\tau_m$ from bench tests.
Cite for Strategy 2 (pure INDI + flatness FF). Do not cite for teams, stacks, or our RPM hardware.
Do not paste their centimetre numbers as our result.

Source: attached PDF only. No other paper used.
