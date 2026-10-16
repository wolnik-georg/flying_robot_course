# ProxFly — Zhang et al., 2024/2025

## 0. Identity

- Full title: ProxFly: Robust Control for Close Proximity Quadcopter Flight via Residual Reinforcement Learning
- Authors: Ruiqi Zhang, Dingqi Zhang, Mark W. Mueller
- Year / venue / arXiv id / DOI if printed: arXiv:2409.13193. Printed header on this file: `arXiv:2409.13193v2 [cs.RO] 30 Apr 2025`. No conference or journal name, no DOI in this file.
- Version of THIS file: arXiv v2, 30 Apr 2025
- Suggested cite key: `zhang2024proxfly`
- One sentence: what this paper IS: a residual PPO policy added to a cascaded high-level thrust/body-rate controller to compensate downwash and model error on a heterogeneous pair, with no inter-vehicle comms.
- One sentence: what this paper is NOT: not INDI, not geometric control, not a learned force model, not Crazyflie, not onboard residual inference, not a 3+ robot swarm paper.

## 1. Problem they pose

Their wording: close-proximity flight is needed for confined collaborative missions and aerial docking, but downwash causes thrust loss plus external forces and torques that "are difficult to model using conventional model-based approaches." High-fidelity aero models need precise identification and fine-tuned gains. End-to-end RL overfits a model. Multi-agent RL needs communication. Black-box commands are hard to explain.

Plain restatement: keep a simple cascaded controller and learn only the leftover command that cancels downwash and model mismatch, using only the ego vehicle.

## 2. Paper map (follow their TOC)

### Abstract

- Purpose: state the method and the three evaluation claims.
- Method / construction: residual module on a cascaded "basic controller." Inputs: ego state and controller commands. Domain randomization. No comms.
- Key equation(s): none numbered here.
- Assumption stated or implied: residual commands plus a working baseline are enough; relative pose is not required at runtime.
- Result claimed: sim at several proximities; real hover and circling vs basic and vs FB-AeroComp; mid-air docking succeeds. Source: abstract.

### I. INTRODUCTION

- Purpose: motivate close proximity, survey model-based aero, supervised residuals, learned downwash for planning, and RL.
- Method / construction: none. Positioning only.
- Key equation(s): none.
- Assumption stated or implied: prior learned downwash work was on micro quads with "weak downwash" and "large spacing," so it may not transfer to regular quads or tighter flight.
- Result claimed: none yet. They announce residual RL (cite residual RL / residual policy learning) on a cascaded high-level thrust and body-rate stack, ego-only, domain-randomized.

### II. METHODOLOGY

- Purpose: pipeline, residual learning argument, training recipe.

### II.A Residual Policy Learning

- Purpose: define the superposition and why gradients only hit the residual.
- Method / construction: cascaded high-level position–attitude controller outputs mass-normalized thrust and body rates. Residual MLP adds corrections. Sum goes to an onboard low-level rate-to-motor loop. Mocap 200 Hz. High-level and residual 50 Hz. Low-level 500 Hz.
- Key equation:

$$
\pi(o\mid\theta)=u_{\mathrm{cas}}(s)+\pi_{\mathrm{res}}(o\mid\theta)\tag{1}
$$

Gradient identity they state: $\nabla_\theta\pi(o\mid\theta)=\nabla_\theta\pi_{\mathrm{res}}(o\mid\theta)$ because $u_{\mathrm{cas}}$ does not depend on $\theta$.

- Assumption: baseline is already flyable. Residual is a command residual, not a force residual.
- Result claimed: residual learning skips from-scratch exploration and cuts the black-box share of the total command. Source: body §II.A.

### II.B Training Settings

- Purpose: PPO setup, spaces, reward, randomization.
- Method / construction: two independent "full-enveloped" policies (one per airframe). MLP 3×128. LeakyReLU then tanh. Adam. PPO. Episode = 20 s = 10 000 steps at 500 Hz sim. First 15 s takeoff/hover, last 5 s land at 0.2 m/s. Init position uniform in 2 m × 2 m on the ground. Hover set-point $(0,0,1.2)$. 500 epochs × 10 episodes. "15 minutes" on Intel i7-13700H with parallel rollouts.
- Assumption: sim frequency 500 Hz matches the onboard low-level rate. Perfect mocap-like state in sim.

#### II.B.1 Action and Observation Space

- Action: $a_{\mathrm{res}}=\{c_{\mathrm{res}},\omega_{\mathrm{res}}\}\in\mathbb{R}^4$.
  Residual mass-normalized thrust range $[-10,10]\,\mathrm{m\,s^{-2}}$.
  Residual body rates $[-1,1]\,\mathrm{rad\,s^{-1}}$.
- Observation: $o=\{\Delta s,\,u_{\mathrm{cas}},\,a_{\mathrm{res}}\}\in\mathbb{R}^{20}$.
  $\Delta s=(s_{\mathrm{des}}-s)\in\mathbb{R}^{12}$, $u_{\mathrm{cas}}\in\mathbb{R}^4$, last residual $a_{\mathrm{res}}\in\mathbb{R}^4$.
- Fig. 2 caption also lists current state $s$, $p_{\mathrm{des}}$, $\eta_{\mathrm{des}}$, and last basic command. Body text uses the $\Delta s$ packing. Both are in this file.
- Assumption: no neighbour state, no relative pose, no aero features.

#### II.B.2 Reward Shaping

$$
J(\pi)=\mathbb{E}_{\tau\sim p(\tau\mid\pi)}\Big[\sum_{t=0}^{T}\gamma^t r_t\Big]\tag{2}
$$

$$
r_t=\alpha\cdot[e_{\mathrm{pos}},\,e_{\mathrm{att}},\,P_c,\,P_\omega,\,r_{\Delta t}]^T\tag{3}
$$

$\alpha=[-1\,\mathrm{m^{-1}},\,-1,\,-0.01\,\mathrm{kg\,N^{-1}},\,-0.1\,\mathrm{s\,rad^{-1}},\,1]$.

- $e_{\mathrm{pos}}=\|p_{\mathrm{des}}-p_t\|$
- $e_{\mathrm{att}}=3-\mathrm{trace}(R_t^T R_{\mathrm{des}})=2-2\cos(\omega)$
- $P_c=\|c_t\|+2\|c_t-c_{t-1}\|$
- $P_\omega=\|\omega_t\|+2\|\omega_t-\omega_{t-1}\|$
- $r_{\Delta t}=0.1$ survival bonus
- Assumption: those $\alpha$ weights and the factor 2 on rate penalties are empirical.

#### II.B.3 Randomization

- Purpose: stop overfitting one plant.
- Table I (this file): SQ mass 0.280 kg randomized $\mathcal{U}(0.8,1.2)$; LQ mass 0.850 kg randomized $\mathcal{U}(0.5,1.5)$. Inertia gets an extra $\mathcal{U}(0.8,1.2)$ factor on top of the mass factor. Propeller thrust factor per rotor $\mathcal{U}(0.6,1.2)$. Force in $x,y$ and $z$ as independent triangles with randomized period and amplitude; $z$ larger. Torque = independent Gaussians. Force truncated in $z$ (SQ 1.00 N, LQ 4.00 N).
- Assumption they state: this numerical force/torque randomization stands in for CFD/wind-tunnel downwash and "gets rid of extensive randomization of aerodynamic and trajectory parameters of two quadcopter."
- They do **not** put a second vehicle in the training episode. Disturbance is injected on the ego plant.

### III. EXPERIMENTS

- Purpose: sim check that the cheap disturbance transfers to a high-fidelity wake; then real comparison; then docking.

### III.A Simulation Experiments and Analysis

- Purpose: apply the aero model of Jain et al. 2019 [6] as a test disturbance, not as the training model.
- Geometry: LQ hover $(0,0,1.2)$. SQ starts $(-1,0,0)$, flies $+x$ at 0.2 m/s, hovers 5 s above LQ, leaves to $(1,0,0)$. Relative height $H\in\{0.25,0.50,0.75\}$ m. Relative yaw 0.
- Result claimed (Fig. 3): basic controller leaves altitude steady-state error that grows with downwash; residual adds thrust and rate without using SQ–LQ relative pose. Source: body + Fig. 3.

### III.B.1 Comparative Experiments

- Three real tasks, SQ 50 cm above LQ:
  - hover 10 s
  - both circle, same sense, diameter 1.5 m, period 7.5 s
  - same circle, opposite senses
- Baselines: Basic Controller; FB-AeroComp (model-based aero compensation from [4], same hardware family as this lab).
- Metrics:

$$
E_{\mathrm{pos}}=\sqrt{\frac{1}{T}\sum_{t=1}^{T}(p_{\mathrm{des}}-p_t)^2}\tag{4}
$$

$$
E_{\mathrm{att}}=\sqrt{\frac{1}{T}\sum_{t=1}^{T}\left[\cos^{-1}\left(\frac{\mathrm{trace}(R_t^T R_{\mathrm{des}})-1}{2}\right)\right]^2}\tag{5}
$$

- Table II numbers (body):

| Task | Metric | Basic | FB-AeroComp | ProxFly |
|---|---|---|---|---|
| Hovering | $E_{\mathrm{pos}}$ | 0.1199 m | 0.1113 m | 0.0882 m |
| Hovering | $E_{\mathrm{att}}$ | 0.1710 rad | 0.1818 rad | 0.0794 rad |
| Circling (same) | $E_{\mathrm{pos}}$ | 0.1867 m | 0.0832 m | 0.1385 m |
| Circling (same) | $E_{\mathrm{att}}$ | 0.1976 rad | 0.1238 rad | 0.1252 rad |
| Circling (reversed) | $E_{\mathrm{pos}}$ | 0.1451 m | 0.0983 m | 0.0940 m |
| Circling (reversed) | $E_{\mathrm{att}}$ | 0.1714 rad | 0.0930 rad | 0.0996 rad |
| Average | $E_{\mathrm{pos}}$ | 0.1506 m | 0.0976 m | 0.1069 m |
| Average | $E_{\mathrm{att}}$ | 0.1800 rad | 0.1329 rad | 0.1014 rad |

- Body claim vs basic: "reduces the averaged RMSE of position control by 29.0% and averaged attitude error by 43.7%."
- Body claim vs FB-AeroComp in hover only: "reduces position error by 20.8% and attitude error by 56.3%."
- They also say they are "mainly trained for hovering rather than trajectory tracking," and that residual commands "still oscillate at a high frequency."

### III.B.2 Quadcopter Aerial Docking

- LQ takeoff and hover $(0,0,1.2)$.
- SQ from $(0,-1,1.7)$, then "descends vertically at a speed of 0.1 m/s until it is 10 cm above the LQ."
- When horizontal distance "less than 5 cm," SQ "shuts down its motors and falls freely onto the top of the LQ" from "5 cm" (Fig. 5 caption) / "5cm above the LQ" (body).
- Docking changes mass and applies an impulse. Undock on a joystick command. SQ climbs vertically. Downwash returns.
- Result claimed: residual thrust and rates stabilize LQ through approach, dock, and undock. "Docking would fail if only the model-based basic controller were used." Source: body §III.B.2 and Fig. 5. No RMSE table for docking.

### IV. CONCLUSION

- Purpose: recap. Residual + cascade. No comms. No accurate parameters. Lightweight MLP.
- Limitations they write: residual "can potentially cause oscillations, which might lead to motor overheating"; parameters must stay "in a reasonable range"; "only validated the performance on two regular-sized quadcopters."

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Unnamed "regular-sized" pair. SQ mass 0.280 kg, arm 0.058 m. LQ mass 0.850 kg, arm 0.145 m. Not a Crazyflie. |
| n robots min/max | 2 / 2 |
| Sim / hardware / both | Both. Train in randomized sim. Test in sim with [6]-style aero and on hardware. |
| Onboard vs off-board (Hz) | Residual + cascade high-level: **off-board laptop, 50 Hz**. Low-level motor mixer: **onboard, 500 Hz**. Mocap 200 Hz. |
| Sensing (mocap, RPM, GPS, …) | Motion-capture pose, attitude, linear velocity, body rates. State estimator in the loop. No RPM or force sensor named. |
| Controller family | Cascaded position–attitude + residual RL (PPO). Command is thrust + body rate, not INDI or geometric. |
| How interaction enters the law | It does not, at runtime. No relative pose. Compensation is a learned command residual on the ego error. |
| Learned model? inputs → output | Policy, not a dynamics model. $o\in\mathbb{R}^{20}\to a_{\mathrm{res}}\in\mathbb{R}^{4}$ (residual $c,\omega$). |
| Training data amount / source | Sim only. 500 epochs × 10 episodes × 20 s. "15 minutes" on i7-13700H. No real-flight data in this file. |
| Formations + min separation | Vertical stack 0.50 m (hover, both circles). Sim stacks 0.25 / 0.50 / 0.75 m. Dock: 0.10 m then 0.05 m free-fall. |
| Speeds | Sim approach 0.2 m/s. Dock descent 0.1 m/s. Circle: diameter 1.5 m, period 7.5 s. Land in training 0.2 m/s. Accel not in this paper. |
| Metrics | $E_{\mathrm{pos}}$, $E_{\mathrm{att}}$ RMSE. Reward terms in training only. |
| Baseline they compare to | Basic cascaded controller. FB-AeroComp [4] (explicit aero compensation). |

## 4. Formations and flights (table, one row per experiment)

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Sim approach–hover–leave | 2 | SQ above LQ, yaw 0 | Δz = 0.25 / 0.50 / 0.75 m. Horizontal 1 m → 0 → 1 m | +x at 0.2 m/s, hover 5 s | sim | Fig. 3 force/torque and residual traces |
| Close proximity hovering | 2 | SQ above LQ | 0.50 m vertical, 0 lateral | hover 10 s | hw | Table II $E_{\mathrm{pos}},E_{\mathrm{att}}$ |
| Circling same direction | 2 | SQ above LQ, same circle | 0.50 m vertical | circle Ø 1.5 m, T = 7.5 s | hw | Table II + Fig. 4(a) |
| Circling reversed | 2 | SQ above LQ, opposite sense | 0.50 m vertical | same circle | hw | Table II + Fig. 4(b) |
| Mid-air docking | 2 | approach then contact | start SQ at (0,−1,1.7) vs LQ (0,0,1.2); powered to 0.10 m; free-fall 0.05 m; horiz trigger < 0.05 m | descent 0.1 m/s, then free-fall, then undock climb | hw | Fig. 5 traces. No RMSE row |

No side-by-side formation, no triangle, no 3-robot set, no ground-effect pad, no ceiling run in this file.

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Average pos improvement vs basic | 29.0% | mean of three-task $E_{\mathrm{pos}}$ RMSE | body §III.B.1 | Not vs FB-AeroComp. Same-direction circle pos is worse than FB-AeroComp. |
| Average att improvement vs basic | 43.7% | mean of three-task $E_{\mathrm{att}}$ RMSE | body §III.B.1 | Same caveat. |
| Hover vs FB-AeroComp pos / att | 20.8% / 56.3% | hover task only | body §III.B.1 | Do not paste onto circling. |
| Stack height in hover/circles | 0.50 m / "50cm" | SQ above LQ | body §III.B.1 | Not the docked state. |
| Dock powered floor | 0.10 m | last powered height | body §III.B.2 | Separate from free-fall. |
| Dock free-fall | 0.05 m / "5cm" | motor-off drop | body + Fig. 5 caption | Also horiz gate < 0.05 m. |
| Circle geometry | Ø 1.5 m, T = 7.5 s | both circling tasks | body | Speed not printed as m/s. |
| Residual action box | ±10 m/s², ±1 rad/s | policy output limits | §II.B.1 | Not the plant thrust limit. |
| Train time | 15 minutes | laptop i7-13700H, parallel sim | §II.B | Sim only. |
| Rates | 200 / 50 / 500 Hz | mocap / high-level / onboard low-level | §II | Residual is the 50 Hz off-board piece. |
| SQ / LQ mass | 0.280 / 0.850 kg | Table I nominal | Table I | Randomized in train. |

Abstract and body do not fight on these numbers. Abstract has no RMSE table.

## 6. Assumptions and failure modes

- Baseline cascade is stable enough to fly while the residual learns.
- Runtime needs no neighbour state and no aero model.
- Training disturbance (triangles + Gaussians + per-rotor thrust factor) is a valid proxy for real downwash.
- Mocap-grade state at 200 Hz is available.
- Plant stays inside the randomization box. They say parameters must stay "in a reasonable range."
- Two regular-sized airframes only. They say more types are needed.
- Residual can oscillate and "might lead to motor overheating."
- Policies "mainly trained for hovering." Circling is a moving hover set-point and they call the residual "highly sensitive," "more aggressive."
- Docking impulse and mass jump are handled only because randomization included mass/inertia error. No contact model is written down.
- Low-level 500 Hz loop is assumed correct and is not learned.

## 7. Relation to our thesis (clearly labelled OUR READING)

- Which of our strategies it supports (1 geo / 2 INDI / 3 geo+NN / 4 FBL / 5 hybrid): **none as a replica.** It is residual **RL on a cascaded thrust/rate command**, not a force/acceleration residual subtracted inside INDI/FBL/geo. If you still keep an unimplemented "ProxFly-style residual RL" column in related work, this is that paper. Do not file it under geo+NN or hybrid NA-INDI.
- What we may cite: no-comms ego-only downwash compensation; residual-on-baseline idea; domain randomization instead of CFD at train time; 0.50 m vertical stack hover and co-/counter-rotating circles; mid-air dock as a hard interaction case; Table II as a published 2-robot hetero benchmark; explicit admission of residual oscillation.
- What we must NOT cite from it: Crazyflie results; onboard residual on STM32; 3-robot teams; INDI or geometric baseline; a learned interaction force; RPM-based residual; fair frozen-gain comparison on our airframe; "better than model-based aero on every task."
- Formations we should reproduce if we use it as a reference condition: 0.50 m vertical stack hover; 1.5 m circle, T = 7.5 s, same and opposite sense, 0.50 m stack; optional dock-like approach to 0.10 m then contact, only if our hardware allows. Do not import Neural-Swarm spacings.
- Compute: they ran the MLP **off-board on a laptop at 50 Hz**. They did not run it on an STM32. The net is small (3×128), so a 50 Hz forward pass is plausible on a modern MCU, but that is our inference, not a result in this file.

## 8. Equations worth keeping

$$
\pi(o\mid\theta)=u_{\mathrm{cas}}(s)+\pi_{\mathrm{res}}(o\mid\theta)\tag{1}
$$

Total command = cascade + residual. $\theta$ lives only in the residual.

$$
J(\pi)=\mathbb{E}_{\tau\sim p(\tau\mid\pi)}\Big[\sum_{t=0}^{T}\gamma^t r_t\Big]\tag{2}
$$

PPO objective. $\gamma$ not given a number in this file.

$$
r_t=\alpha\cdot[e_{\mathrm{pos}},e_{\mathrm{att}},P_c,P_\omega,r_{\Delta t}]^T\tag{3}
$$

Five-term reward. $\alpha$ listed in §II.B.2.

$$
e_{\mathrm{att}}=3-\mathrm{trace}(R_t^T R_{\mathrm{des}})=2-2\cos(\omega)
$$

Train-time attitude penalty. $\omega$ = smallest rotation angle.

$$
E_{\mathrm{pos}}=\sqrt{\frac{1}{T}\sum_{t=1}^{T}(p_{\mathrm{des}}-p_t)^2}\tag{4}
$$

Position RMSE. Paper writes $(p_{\mathrm{des}}-p_t)^2$ without an explicit norm; treat as they printed.

$$
E_{\mathrm{att}}=\sqrt{\frac{1}{T}\sum_{t=1}^{T}\left[\cos^{-1}\left(\frac{\mathrm{trace}(R_t^T R_{\mathrm{des}})-1}{2}\right)\right]^2}\tag{5}
$$

Attitude RMSE in radians.

Action box (unnumbered): $c_{\mathrm{res}}\in[-10,10]\,\mathrm{m\,s^{-2}}$, $\omega_{\mathrm{res}}\in[-1,1]\,\mathrm{rad\,s^{-1}}$.

## 9. Figures / tables worth remembering

- Fig. 1: two real scenes. (a) stacked circular tracking. (b) small quad docking on large quad.
- Fig. 2: pipeline. Off-board high-level cascade + residual. Onboard low-level. Mocap in, motor speeds out.
- Table I: nominal SQ/LQ parameters and all randomization ranges.
- Fig. 3: sim. Row 1 external $F_z$, $\tau_x$, $\tau_y$ from the [6] model at three heights. Row 2 altitude/roll/pitch, basic vs ProxFly. Row 3 residual $c$, roll rate, pitch rate.
- Table II: real RMSE for hover, same-circle, reverse-circle, and averages. Three controllers.
- Fig. 4: real circle traces plus residual thrust. (a) same sense, lasting positive residual thrust. (b) reverse sense, ~1 s thrust bump when SQ passes over LQ. Label "LPF: low-pass filtered." Cutoff not stated.
- Fig. 5: docking stages 1–4 on LQ: takeoff, SQ approach, docked (mass change, residual thrust peak), SQ leaving. Residual thrust/rates over time.

## 10. Open uncertainties

- Conference/journal venue. Not printed on this file.
- Airframe product names, rotor diameter, motor KV.
- Cascade gains, natural frequency, damping. They say the baseline "only requires the predefined natural frequency, damping parameters and basic model constants" but print no numbers.
- PPO $\gamma$, clip, learning rate, batch size.
- Exact contents of the 12-vector $\Delta s$ (likely pos, att, vel, rate, but not itemized).
- LPF cutoff on plotted commands.
- Whether the two circling paths share one identical center (text says SQ "flies 50 cm above" the same 1.5 m circle).
- Contact/docking hardware geometry beyond "docking platform."
- Onboard estimator error. All real metrics use mocap-fed high-level control.
- Whether any real-world fine-tuning of the MLP happened. File says training is sim + domain randomization and "does not rely on … accurate knowledge of system parameters."
- What the short conference version would drop. This file is the arXiv v2. No companion short version is in this PDF.

## 11. 12-line abstract I can reread

ProxFly adds a PPO residual to a cascaded high-level thrust and body-rate controller.
The residual sees ego tracking error, the baseline command, and its last action. No comms.
Training is simulation only, with mass, inertia, rotor efficiency, and fake force/torque randomized.
The learned object is a command residual, not a force model and not INDI.
High-level residual runs off-board at 50 Hz. Low-level mixing is onboard at 500 Hz. Mocap 200 Hz.
Hardware is a heterogeneous pair, 0.280 kg and 0.850 kg, not Crazyflies. Always n = 2.
Real stack for hover and circling is 0.50 m vertical. Circles are 1.5 m diameter, 7.5 s period.
Vs the same cascade, mean $E_{\mathrm{pos}}$ drops 29.0% and mean $E_{\mathrm{att}}$ drops 43.7% over three tasks.
Vs FB-AeroComp the picture is mixed. Hover is better. Same-direction circle position is worse.
Docking: powered descent to 0.10 m, free-fall from 0.05 m, then a mass jump. Cascade alone is said to fail.
They warn about residual oscillation and overheating, and about staying inside the randomized plant range.
Cite for residual-on-cascade and no-comms downwash. Do not cite as our geo/INDI/NN-force campaign.

Source: attached PDF only. No other paper used.
