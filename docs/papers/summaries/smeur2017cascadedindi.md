# Cascaded INDI — Smeur et al., 2018 (this file: arXiv v2 2022)

## 0. Identity

- Full title (this PDF page 1): Cascaded incremental nonlinear dynamic inversion for MAV disturbance rejection
- Authors: Ewoud J.J. Smeur, Guido C.H.E. de Croon, Qiping Chu (TU Delft Aerospace)
- Year / venue / arXiv / DOI printed on THIS file: arXiv:1701.07254v2 [cs.RO] 12 Jan 2022. No journal name, volume, or DOI is printed on the attached pages.
- Version of THIS file: arXiv v2 (12 Jan 2022). The body says this is an extension of IROS 2016 (Smeur et al., 2016b). Extra content vs that short version, as listed in §1 of THIS file: (1) large open-jet wind tunnel instead of a fan; (2) propeller thrust curve for total thrust; (3) outdoor GPS experiment; (4) nonlinear thrust-vector increment.
- Suggested cite key: `smeur2018cascadedindi`
- What this paper IS: cascaded INDI on one Parrot Bebop. Inner loop tracks angular acceleration. Outer loop tracks linear acceleration. Wind-tunnel jet entry/exit and outdoor takeoff tests vs a PID outer loop.
- What this paper is NOT: not multi-robot; not Crazyflie; not a learned residual; not geometric SE(3) / snap tracking; not Tal & Karaman; not interaction-force identification between vehicles.

## 1. Problem they pose

Their wording (§1): outdoor MAVs must hold position under wind gusts, especially near obstacles. PID "do not perform well under the influence of gusts." PID gust rejection "scale[s] with magnitude of the gains, which is often limited by the GPS update frequency." The integrator is "generally slow in compensating persistent wind disturbances." Extra Pitot tubes or detailed aero models add sensors, cost, or parameters that break when the airframe changes.

Plain restatement: they want fast gust rejection from the accelerometer, without a wind model and without needing high-rate position.

## 2. Paper map (follow their TOC)

### 1 Introduction

- Purpose: motivate gusts, reject PID / Pitot / full aero-model options, state the cascaded-INDI idea.
- Method / construction: none yet. Idea sentence: both disturbances and control forces are measured by the accelerometer; increment the last input from desired minus measured acceleration.
- Key equation: none numbered here.
- Assumption: outdoor gusts and urban flow can reach several m/s (they cite Orr et al. 0–7.6 m/s around buildings for 4.6 m/s free stream).
- Result claimed (abstract and §1): wind-tunnel entry/exit at 10 m/s with "only 21 cm maximum position deviation on average" for INDI vs "151 cm" for a controller that uses integrated error instead of the incremental law.

### 1.1 Related work

- Purpose: separate prior acceleration-feedback and INDI work from this paper.
- Method: literature only. Hoffmann altitude accel gain without thrust–accel physics. Simplicio vertical INDI on a helicopter in simulation. Wang et al. 2012 acceleration-feedback DI, not incremental, gusts not shown.
- Result: positioning only.

### 2 Incremental nonlinear dynamic inversion for attitude control

- Purpose: recap inner-loop INDI from Smeur et al. 2016a, with the thrust channel added.
- Method: write angular accel and thrust as $F(\Omega,v)+G(\omega,\dot{\omega})$. Taylor-expand. Drop aero/rate partials. Keep static $G_1,G_2$. Same filter on gyro-derived angular accel and actuator states. Invert for motor-rate command.
- Key equations:
  - (1) $\begin{bmatrix}\dot{\Omega}\\ T\end{bmatrix}=F(\Omega,v)+G(\omega,\dot{\omega})$
  - (3) $\begin{bmatrix}\dot{\Omega}-\dot{\Omega}_0\\ T-T_0\end{bmatrix}=G_1(\omega-\omega_0)+T_s G_2(\dot{\omega}-\dot{\omega}_0)$
  - (4) $H(s)=\dfrac{\omega_n^2}{s^2+2\zeta\omega_n s+\omega_n^2}$
  - (7) $\omega_c=\omega_f+(G_1+G_2)^{-1}\Bigl(\begin{bmatrix}\nu_{\dot{\Omega}}\\ \dot{T}\end{bmatrix}-\begin{bmatrix}\dot{\Omega}_f\\ T_f\end{bmatrix}+G_2 L(\omega_c-\omega_f)\Bigr)$
- Assumption: $G_1,G_2$ constant. Terms with index 0 / $f$ share one delay.
- Result: Fig. 3. Inner input is virtual angular accel plus thrust increment from the outer loop.

### 2.1 Attitude control

- Purpose: quaternion PD from attitude to $\nu_{\dot{\Omega}}$. Gains from a linearised loop that includes first-order actuators.
- Key equations: (8) $q_{\mathrm{err}}=q_{\mathrm{ref}}\otimes q_s^*$; (9) $\Omega_{\mathrm{ref}}=K_\eta[q_{\mathrm{err}1},q_{\mathrm{err}2},q_{\mathrm{err}3}]^T$; (11) $A(z)=\alpha/(z-(1-\alpha))$, $\alpha=0.1$ at 512 Hz; (12) closed-loop TF.
- Body text: "With $K_\Omega=28.0$ and $K_\eta=21.4$" poles at 0.964 and $0.965\pm 0.0445i$.
- Result: Fig. 5, 25 steps. Max error "6.4 % at 0.14 s" of the step.

### 2.2 Determining the control effectiveness

- Offline LS on one flight log, equation (13). Numbers later in Table 1.

### 3 Incremental nonlinear dynamic inversion applied to linear accelerations

- Newton (14), thrust vector (15), Taylor (16). Drop $\partial F/\partial\dot{\xi}$, $\partial F/\partial w$, $\partial T_N/\partial\psi$.
- (17) $\ddot{\xi}=\ddot{\xi}_0+\frac{1}{m}G(\eta_0,T_0)(u-u_0)$, $u=[\phi,\theta,T]^T$
- (19) $u_c=u_f+m G^{-1}(\eta_0,T_0)(\nu_{\ddot{\xi}}-\ddot{\xi}_f)$
- Assumption: yaw changes small. Inner and outer filters identical so $\Delta T$ matches.

### 4 Implementation / 4.1 Position control

- Fig. 6 outer INDI. Fig. 7 PD → $\nu_{\ddot{\xi}}$.
- Table 2: $K_\xi=0.7$ (m/s)/m, $K_{\dot{\xi}}=1.5$ (m/s²)/(m/s). Manual tune.

### 4.2 Adaptive control effectiveness

- LMS (20)–(21) on full $G$, including thrust row. 10 flights, read at 30 s. Table 1 vs 2 min offline LS. Motors not identical.

### 4.3 Estimation of the thrust

- Do not use body-$z$ accel as $T/m$. Scale test, internal RPM, quadratic fit Fig. 8. Incremental $G$ still static.

### 4.4 Linearization

- Nonlinear option: (24)/(28) then (25)–(27) for $T,\phi_c,\theta_c$.

### 4.5 Filtering

- Shared $H$. Flight: $\omega_n=50$ rad/s, $\zeta=0.55$. Attitude disturbance path written as $1-A(z)H(z)z^{-1}$.

### 4.6 Accelerometer bias

- Differentiate GPS/mocap velocity, rotate, LPF difference at "0.25 rad/s." Bias makes hover sit at a position offset.

### 5–5.1 Windtunnel experiment

- Open Jet 2.85 m × 2.85 m at 10 m/s. Waypoints: jet centre vs 2 m outside, switch every 14 s. OptiTrack sent at 4 Hz. Control 512 Hz. Same inner INDI for both outer laws.
- 7 runs. INDI "0.21 m" entering, "0.20 m" leaving. PID "1.51 m." Abstract: "21 cm" and "151 cm." Fig. 10 display filter is 20 rad/s, 0.7. Not the control filter.

### 6–6.1 Outdoor takeoff with wind

- External U-blox Neo-M8N (stock first-gen GPS "low quality"). KNMI "5.1 m/s." 12 PID then 13 INDI. One INDI run dropped (estimator not converged).
- "average a maximum position error of 0.24 m" INDI vs "0.85 m" PID.

### 7–7.1 Nonlinear increment

- Hover → $(0,4,0)$ m/s² for 0.5 s → $(0,-4,0)$ m/s² for 0.5 s. 25 runs each. No position feedback during the move.
- Linear law drops thrust when reversing bank. Nonlinear stays closer to zero mean $\ddot{z}$ but still not zero. Not stated that this invert was used in §5 or §6.

### 8–8.1 Conclusions / future work

- "more than 7 times lower maximum position deviation" in the 10 m/s jet. Saturation, small-$\psi$ assumption, inner-loop nonlinear RPM–thrust map named as open issues.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Parrot Bebop. Mass not in this paper. $l,b$ drawn, values not in this paper. |
| n robots min/max | 1 / 1 |
| Sim / hardware / both | Hardware only |
| Onboard vs off-board (Hz) | Control + IMU on vehicle at 512 Hz (Paparazzi). Position in from OptiTrack at 4 Hz or external GPS. |
| Sensing | Gyro, accelerometer, motor RPM, OptiTrack indoor, Neo-M8N outdoor. No Pitot. |
| Controller family | Cascaded INDI + PD position. Optional nonlinear $T_N$ invert. Optional LMS on $G$. |
| How interaction enters the law | It does not. Force is whatever the accelerometer sees. |
| Learned model? | No NN. LMS / LS on $G$. |
| Training data | One ID flight offline. LMS: 10 flights at 30 s. One 2 min log offline. |
| Formations + min separation | None. Waypoint gap 2 m. |
| Speeds | Tunnel 10 m/s. Outdoor wind 5.1 m/s (KNMI). Manufacturer max 13 m/s. §7: ±4 m/s². |
| Metrics | Max/mean position error, 1σ, vertical accel, adapted $G$. |
| Baseline | PID outer loop, same inner INDI. |

## 4. Formations and flights

No multi-robot formation in this paper.

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Jet entry/exit | 1 | none | height not in this paper; 2 m horizontal gap | 14 s waypoint switch; 10 m/s jet | hw, OptiTrack 4 Hz | 0.21 m / 0.20 m INDI vs 1.51 m PID; 7 runs |
| Outdoor takeoff | 1 | none | height not in this paper | takeoff + hold | hw, Neo-M8N | 0.24 m vs 0.85 m mean max horiz. |
| Linear vs nonlinear $T_N$ | 1 | none | not in this paper | ±4 m/s² on Y, 0.5 s + 0.5 s | hw indoor | Figs. 15–16; 25 runs |
| Attitude step | 1 | none | not in this paper | step, 25 repeats | hw | Fig. 5; 6.4 % at 0.14 s |
| LMS $G$ | 1 | none | n/a | piloted | hw | Table 1 |

## 5. Numbers that must stay qualified

Keep 10 m/s on the jet, 5.1 m/s on the outdoor takeoff. Do not merge those trials.

Internal clash: §2.1 has $K_\Omega=28.0$, $K_\eta=21.4$. Table 2 has $K_\Omega=10.7$, $K_\eta=28.0$. This file does not resolve it.

## 6. Assumptions and failure modes

- No model for $F$ or $w$.
- $\partial T_N/\partial\psi$ dropped.
- $G$ static despite quadratic thrust curve.
- Shared filter delay required.
- First-order actuators. Same PD unstable if $\alpha=0.02$.
- Accel bias → position offset.
- Saturation after $G^{-1}$ is "suboptimal" (§8.1).
- LMS needs excitation.
- GPS/estimator errors can look like control error.

## 7. Relation to our thesis (OUR READING)

- Supports **strategy 2 (full INDI)** and the increment + shared-filter block of **strategy 5 (hybrid NA-INDI / L-INDI)**.
- Cite: laws (7)+(19); 10 m/s jet vs PID; 4 Hz position; RPM feedback; LMS thrust row; linear vs nonlinear $T_N$.
- Do not cite: formations, Crazyflie, STM32 timing, inter-vehicle downwash, learned force residual, 10 m/s as an outdoor GPS result, "IMU-only / no rotor speed."
- Reproduce from this paper: single-vehicle jet entry/exit, takeoff-into-wind, ±4 m/s² vertical-coupling pulse. No $\Delta z$ or $\Delta xy$ for 2–3 robot stacks.
- Compute: Paparazzi 512 Hz on the Bebop. 4×4 and 3×3 inverses plus two second-order filters. No STM32 claim in this file.

## 8. Equations worth keeping

(1), (4), (7), (11), (14), (19), (25)–(27), (20). Symbols as in the paper. $\omega$ is propeller rate, not body rate.

## 9. Figures / tables worth remembering

Figs. 1–16 and Tables 1–3 as listed in the saved file. Fig. 12: jet at $-1.425<Y_N<1.425$, blowing $-X_N$.

## 10. Open uncertainties

Mass, $l$, $b$, full $G$ matrices, waypoint heights, which $T_N$ invert flew in §5–§6, which attitude gains flew, outdoor estimator details, CPU load.

## 11. 12-line abstract I can reread

Smeur, de Croon and Chu fly one Bebop with cascaded INDI: inner inversion of rotor-speed increments to track angular acceleration, outer inversion of roll/pitch/thrust increments to track linear acceleration. Position and velocity only set the accel reference through a PD. The same second-order filter sits on IMU and motor feedback so the thrust increment is consistent. No drag model. Disturbance is whatever the accelerometer measures. Indoor, they enter and leave a 10 m/s open jet with position at 4 Hz. Mean peak error 0.21 m / 0.20 m versus 1.51 m for a PID outer loop on the same inner INDI. Outdoor takeoff with a Neo-M8N in about 5.1 m/s wind: 0.24 m versus 0.85 m mean max horizontal error. They also adapt the thrust row of $G$ online and compare a linear $G$ inverse to an algebraic $T_N$ inverse on ±4 m/s² steps. Residual vertical accel remains. This file is one vehicle, Paparazzi at 512 Hz, RPM feedback included. It is not a formation paper and not a learned residual paper.

Source: attached PDF only. No other paper used.
