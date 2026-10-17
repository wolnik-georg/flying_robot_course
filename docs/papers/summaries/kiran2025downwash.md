# Kiran downwash — Kiran et al., 2025

## 0. Identity

- Full title: Influence of Static and Dynamic Downwash Interactions on Multi-Quadrotor Systems
- Authors: Anoop Kiran, Nora Ayanian, Kenneth Breuer (Brown University)
- Year / venue / arXiv / DOI: this file is **arXiv:2507.09463v1 [cs.RO] 13 Jul 2025**. Conference name and paper DOI are **not in this paper**. Dataset footnote: https://doi.org/10.26300/6444-wa17
- Version of THIS file: arXiv v1, 12 pages, 16 figures
- Suggested cite key: `kiran2025downwash`
- What this paper IS: a **lab-rig characterisation** of Crazyflie pair downwash. Load-cell forces/moments plus centre-plane PIV. Static maps and a forced-oscillation traverse (CrazyRail). Plus an algebraic far-field jet model.
- What this paper is NOT: a flight-controller paper. Not free flight. Not INDI, FBL, or a trained NN. Not a 3-robot team. Not onboard compensation.

## 1. Problem they pose

Their wording: dense formations fail because rotor-wake interference destabilises vehicles; planners use "predefined arbitrary separation … without factoring in flight physics." Prior velocity data are often dimensional, point-probe, or assumed Gaussian. Gap: "understanding the spatial influence of downwash on quadrotor interactions at a fundamental level," near-field and far-field.

Plain restatement: they want measured $F$, $M$, and wake velocity versus relative pose, so later models and controllers have numbers instead of keep-out ellipsoids.

## 2. Paper map (follow their TOC)

### Abstract

- Purpose: state the measurement claim.
- Method: forces/torques + PIV on a single vehicle and an interacting pair.
- Key equation: none.
- Assumption: close-proximity multi-quadrotor problem is dominated by downwash.
- Result: data "can be used to inform physics-based strategies … and improve the robustness of multi-quadrotor control." No controller is demonstrated.

### I. Introduction

- Purpose: motivate close-proximity flight and why avoid-the-wake strategies are conservative.
- Method: literature framing only.
- Assumption: small-scale quadrotor downwash science is "still in its infancy."
- Result claimed: characterisation (forces, moments, velocity, unsteadiness) is a basis for physics-based **and** learning-based models. They cite others [29, 6] for sample-efficient learning. They do not train a model here.

### I.A Contributions

- Two claims: (1) map pair forces, moments, velocities vs separation; algebraic reduced-order downwash model with turbulent-round-jet scaling; open dataset. (2) quantify unsteadiness (turbulence) for stability/planning.

### I.B Related Work

- Purpose: contrast cylinders/ellipsoids, anemometer decay fits, CFD box limits, Bauersfeld probe data.
- They quote Preiss *et al.* keep-out on Crazyflies: $\Delta z = 60\,\mathrm{cm}$ ($\Delta z/l \approx 18.5$), $\Delta x = 24\,\mathrm{cm}$ ($\Delta x/l \approx 7.4$). That is **Preiss**, not their flight envelope.
- Their stance: measure inside the downwash region instead of only avoiding it.

### II. Methodology

- Purpose: define Case I (stacked) and Case II (horizontal offset).
- Construction: two Crazyflies on an $x$–$z$ stage. Roll and yaw **prevented**. Wind-free room. "Closely mimics" indoor free flight. It is still a mount.
- Case I: aligned rotors, same rotation sense. Upper jet hits lower inflow. Expect large $\Delta F_z$ on the lower vehicle.
- Case II: offset grows $M_y$ to a peak at the largest moment arm, then decays.

### II.A Experiment Methods

- Platform: two Crazyflies. B fixed. A moved by steppers, accuracy $100\,\mu\mathrm{m}$. Both on ATI Nano-17 6-DOF cells.
- Actuation: tethered **constant 4 V DC**, hover bit-PWM via Crazyradio. Rotors "approximately 320 Hz." Hardware holds **voltage**, not rpm.
- Force/moment grid: $0.1\,\mathrm{m} \le \Delta x \le 0.6\,\mathrm{m}$, $0.1\,\mathrm{m} \le \Delta z \le 1.1\,\mathrm{m}$. 20 kHz, 30 s, 3 repeats. Std from those three trials.
- PIV cases: stacked and offset at $\Delta z/l = 4$ and $\Delta z/l = 12.5$ (four pair fields) + one isolated vehicle (five total).
- PIV kit: DEHS tracers; Quantel Evergreen Nd:YAG 532 nm, 200 mJ @ 15 Hz; sheet through centres of the front two rotors; LaVision sCMOS 2560×2160, Nikon 35 mm, FOV $400\times 300\,\mathrm{mm}$; 1000 pairs ≈ 70 s; DaVis v10, final window 32×32, 75% overlap.
- Stitching: move vehicles on the stage; laser and camera fixed; ≥ 100 mm overlap between sections.

### III.A Forces & Moments

- Normalisation: thrust by weight $W \approx 0.265\,\mathrm{N}$; $M_y$ by $W l$; $l = 32.5\,\mathrm{mm}$ (body centre to motor).
- Lower vehicle, stacked, $\Delta z/l = 4$: thrust "drops to 65% of its nominal hover thrust."
- Recovery: "unaltered levels at $(\Delta x/l \approx 3)$ and $(\Delta z/l \approx 19)$."
- $\Delta z/l > 19$: lower thrust "95%–98%" of hover. $\Delta x/l > 3.7$: thrust effect "nearly diminished."
- Upper vehicle at $\Delta z/l = 4$: "retains 90% of its original hover thrust."
- Unsteadiness (they write $F'_x$ in body text; figure captions use thrust std): lingers to $\Delta z/l \approx 17$, $\Delta x/l \approx 4.5$. Strong while stacked to $\Delta z/l \approx 14$. Upper fluctuations "approximately two orders of magnitude smaller" than lower. Negligible for $\Delta z/l > 17$ and $\Delta x/l > 4.5$.
- $M_y$ peaks at $\Delta x/l = 2$ (upper left rotors over lower right). Upper $|M_y|$ "only one-third" of lower. $M'_y$ also peaks at $\Delta x/l = 2$, negligible by $\Delta x/l \approx 4$. Steady-ish for $\Delta z/l > 16$. $M'_y$ nonzero even at zero horizontal offset.

### III.B Velocity fields

- Axes: $u$ axial, positive toward the ground; $v$ lateral. Pair plots use speed $\bar U = \sqrt{\bar u^2+\bar v^2}$.
- Normaliser: actuator-disk induced velocity, Eq. (1).
- Spatial resolution: 1.6 mm ($0.05l$). Velocity uncertainty 0.6 mm/s (0.2 px, 15 Hz).

### III.B.1 Single vehicle near-field

- Inflow above rotors ≈ $U_i/5$.
- Near field: annular jets, hub dead cores, body dead zone between rotors.
- Merge: $z/l = 6.5$ (they also write $\approx 2.3$ motor-to-motor distances). After merge, max speed is on the centreline.
- Profile sequence: many peaks → bimodal on $5 \le z/l \le 6.5$ → single peak for $z/l \ge 6.5$.

### III.B.2 Single vehicle far-field / jet theory

- Half-width definition Eq. (2). Linear spread Eq. (3). Centreline decay Eq. (4). Self-similar profiles Eqs. (5)–(6).
- Table I (this study vs Bauersfeld): $Bd = 5.868$ vs 10.11; $S = 0.0667$ vs 0.07668; $z_0 = -6.0585$ vs $-5.813$.
- $u_c$ peaks at "half the induced velocity" at $z/l = 6.5$, then decays.
- $\bar v \approx \bar u/20$.
- $\mathrm{Re} = u_c r_{1/2}/\nu$, Eq. (7). At merge: $u_c = 4.25\,\mathrm{m/s}$, $r_{1/2} \approx l = 0.0325\,\mathrm{m}$, $\nu \approx 1.5\times 10^{-5}\,\mathrm{m}^2/\mathrm{s}$ → $\mathrm{Re} \approx 9000$, Eq. (8). They argue jet scaling still holds below the usual $10^4$–$10^5$ textbook range.

### III.B.3 Multi-vehicle PIV

- Stacked $\Delta z = 40.6\,\mathrm{cm}$ ($\Delta z/l = 12.5$): wider, smoother inflow on the lower vehicle; **larger** speed magnitudes under the lower vehicle than at close stack.
- Stacked $\Delta z = 13\,\mathrm{cm}$ ($\Delta z/l = 4$): lower vehicle sits in the upper near-field; multi-lobe inflow; **lower** downwash speed and thrust.
- Offset (body: $\Delta x/l = 2$): asymmetric wake; peak speed off centreline for $\Delta z/l \ge 5$. Fig. 13 caption text in this file says "offset configuration $(\Delta z/l = 2)$" in one place. Treat $\Delta x/l = 2$ as the geometry the body defines.

### III.C Dynamic Quadrotor Interactions

- CrazyRail: upper Crazyflie (C) on a linear traverse, lower (D) on a Nano-17. Stacked and offset $\Delta x/l = 2$.
- Path: $\Delta z(t) = \Delta z_{\min} + A(1+\sin(\omega t))$, Eq. (9), $\omega = 2\pi f$, $\Delta\dot z = A\omega\cos(\omega t)$.
- Table II ranges: $\Delta z_{\min}$ 0.5–7 cm; $f$ 0.1–1 Hz; $A$ 2.5–9 cm; $\Delta\dot z$ $\pm 0.565\,\mathrm{m/s}$.
- Protocol: 48 cycles = 4 ramp-up + 40 analysed + 4 ramp-down. Phase average. Static points: 10 trials per $\Delta z_{\min}$.
- Result: hysteresis loops. Approach (blue, $\Delta\dot z < 0$) → **lower** thrust on D than retreat (red). Stacked: wide $F_z$ swing, $M_y \approx 0$. Offset: flatter $F_z$, large $M_y$, worse on approach.

### IV. Conclusions

- Body wording: stacked close hover → "strong reduction of approximately **65–70%** of its hover thrust." Abstract/III.A used **65%** at $\Delta z/l = 4$. Both stand; body does not retract the 65% point.
- Four scaling parameters they keep: $z_m/l$, $U_i$, $l$, $z_0$.
- They claim far-field self-similarity still applies if those four are refit, even for other rotor counts. Near-field pair maps are Crazyflie-homogeneous in this file.

### V. Limitations and Future Work

- Homogeneous small–small Crazyflies only.
- Near-field merge can change for heterogeneous layouts; far-field jet scaling is what they say transfers.
- Dynamic $\Delta z$ limited by rail: $0.15 \le \Delta z/l \le 7.5$.

### VI. Acknowledgments

- Brown Seed Award. NSF GRFP 2040433. Alexander Wang on setup/data.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Crazyflie. $l = 32.5\,\mathrm{mm}$. $W \approx 0.265\,\mathrm{N}$. Propeller radius as a number: **not in this paper**. Brushless vs brushed: **not in this paper**. |
| n robots min/max | 1 (PIV baseline) or 2. Never 3. |
| Sim / hardware / both | Hardware rig only. CFD cited, not run here. |
| Onboard vs off-board (Hz) | Not flying. Tethered 4 V. Crazyradio hover PWM. Load cells 20 kHz. PIV laser 15 Hz. No onboard compensator. |
| Sensing | ATI Nano-17 on both (static) / lower (dynamic). PIV. Stage position. No OptiTrack, GPS, or RPM sensor reported. |
| Controller family | None. Constant-voltage hover command. |
| How interaction enters the law | It does not. Interaction is the **measured** $F,M$. |
| Learned model? inputs → output | No model trained. Text only says data can feed learning later. |
| Training data amount / source | Open dataset DOI above. Byte counts / split: **not in this paper**. |
| Formations + min separation | Stacked $\Delta x = 0$. Offset $\Delta x/l = 2$. Static min in grid 0.1 m. Dynamic $\Delta z_{\min}$ down to 0.5 cm. |
| Speeds | Rotor ~320 Hz. Dynamic $\lvert\Delta\dot z\rvert \le 0.565\,\mathrm{m/s}$. Free-flight path speed: none. |
| Metrics | Mean and std of $F_z$, $M_y$; PIV $\bar u,\bar v,\bar U$; jet $r_{1/2}$, $u_c$, Re. No tracking RMSE. |
| Baseline they compare to | Isolated vehicle PIV. Static points vs dynamic loops. Jet theory [25] and Bauersfeld Table I. Avoidance ellipsoids as *literature*, not a flown baseline. |

## 4. Formations and flights (table, one row per experiment)

No free-flight trajectories in this file.

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Static F/M grid | 2 | stacked + many $\Delta x$ | $0.1\le\Delta x\le 0.6\,\mathrm{m}$, $0.1\le\Delta z\le 1.1\,\mathrm{m}$ | fixed pose, hover PWM | hw rig | mean/std $F_z$, $M_y$ both vehicles (Figs. 5–6) |
| PIV single | 1 | isolated | wake in $z$ | hover PWM | hw PIV | $\bar u,\bar v$, merge at $z/l=6.5$, jet fits (Figs. 7–11, Table I) |
| PIV pair stacked close | 2 | $\Delta x=0$ | $\Delta z/l=4$ (13 cm) | hover PWM | hw PIV | $\bar U$ and profiles under lower (Fig. 12c,d) |
| PIV pair stacked far | 2 | $\Delta x=0$ | $\Delta z/l=12.5$ (40.6 cm) | hover PWM | hw PIV | wider, stronger-looking lower wake (Fig. 12a,b) |
| PIV pair offset close | 2 | $\Delta x/l=2$ | $\Delta z/l=4$ | hover PWM | hw PIV | asymmetric $\bar U$ (Fig. 13c,d) |
| PIV pair offset far | 2 | $\Delta x/l=2$ | $\Delta z/l=12.5$ | hover PWM | hw PIV | milder asymmetry (Fig. 13a,b) |
| Dynamic CrazyRail stacked | 2 | $\Delta x=0$ | $\Delta z_{\min}$ 0.5–7 cm; $A$ 2.5–9 cm | sine, $f=0.1$–1 Hz, $\lvert\Delta\dot z\rvert\le 0.565\,\mathrm{m/s}$ | hw traverse | $F_z$ loops; $M_y\approx 0$ (Fig. 15) |
| Dynamic CrazyRail offset | 2 | $\Delta x/l=2$ | same Table II | same sine | hw traverse | flatter $F_z$; large $M_y$, worse on approach (Fig. 16) |

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Lower thrust at closest stack | 65% of hover | mean $F_z/W$ at $\Delta z/l=4$, $\Delta x=0$ | III.A, Fig. 6a | Mounted, constant voltage, not free hover |
| Same effect, conclusions | "approximately 65–70%" | stacked, "small ranges of vertical separations" | IV | Broader wording than the 65% grid point |
| Upper thrust at $\Delta z/l=4$ | 90% of hover | upper $F_z/W$ | III.A, Fig. 5a | Same rig |
| Thrust recovered | $\Delta x/l\approx 3$, $\Delta z/l\approx 19$ | where lower $F_z$ returns to isolated hover | III.A | "Unaltered levels," not a hard cut |
| Residual thrust far below | 95%–98% | lower $F_z/W$ for $\Delta z/l>19$ | III.A | Not zero interaction, just small |
| Lateral thrust fade | $\Delta x/l>3.7$ | thrust effect "nearly diminished" | III.A | Moments fade on a different scale |
| Unsteady $F$ gone | $\Delta z/l>17$, $\Delta x/l>4.5$ | their "negligible" std band | III.A | $M'_y$ uses $\Delta x/l\approx 4$, $\Delta z/l>16$ |
| Peak moment geometry | $\Delta x/l=2$ | max $\lvert M_y\rvert$ and $\lvert M'_y\rvert$ | III.A | Two-rotor overlap, not four |
| Upper vs lower $\lvert M_y\rvert$ | "one-third" | upper relative to lower | III.A | Qualitative ratio, no table of Nm |
| Fluctuation ratio | "two orders of magnitude smaller" | upper vs lower $F'$ and $M'$ | III.A | Order-of-magnitude, not a fitted σ |
| Weight | $W\approx 0.265\,\mathrm{N}$ | used to normalise $F_z$ | III.A | Mass in grams not printed |
| Arm | $l=32.5\,\mathrm{mm}$ | centre to motor | III.A / Fig. 3 | Also used as $r_{1/2}\approx l$ at merge |
| Merge station | $z/l=6.5$ ≈ 2.3 motor-to-motor | where $u_c=u_{\max}$ | III.B, Figs. 8–9 | Definition of their far-field start |
| $u_c$ at merge | "half the induced velocity"; 4.25 m/s in Eq. (8) | centreline speed at $z/l=6.5$ | III.B.2 | 4.25 m/s is the Re example, not a second independent campaign |
| Re at merge | $\approx 9000$ | $u_c r_{1/2}/\nu$ | Eq. (8) | Uses $r_{1/2}\approx l$, $\nu\approx 1.5\times 10^{-5}$ |
| Inflow above disk | "roughly one-fifth" of $U_i$ | single-vehicle PIV | III.B.1 | Visual from Fig. 7a |
| $\bar v / \bar u$ | $\approx 1/20$ | far-field | III.B.2 | Order of magnitude |
| PIV uncertainty | 0.6 mm/s | displacement 0.2 px @ 15 Hz | III.B.2 | Not a full uncertainty budget |
| Rotor frequency | ≈ 320 Hz | at 4 V hover PWM | II.A | Not closed-loop rpm hold |
| Dynamic $\lvert\Delta\dot z\rvert$ | 0.565 m/s | Table II max | III.C | Traverse kinematics, not a flown speed |
| Dynamic $\Delta z/l$ window | $0.15$ to $7.5$ | rail length limit | V | Static grid goes much higher in $z$ |
| Preiss keep-out (cited) | 60 cm / 24 cm | **their** reference to [26], not this experiment | I.B | Do not attribute as Kiran's flown spacing |

Abstract vs body on thrust drop: abstract has no percentage. III.A = 65% at one point. IV = 65–70% for "small" stacked separations. Keep both.

## 6. Assumptions and failure modes

- Quasi-steady actuator disk for $U_i$ (Eq. 1). Disk flow is not actually uniform.
- Constant voltage ≠ constant rpm. Interaction can change rpm.
- Mounted vehicles. No roll, no yaw, no body-rate response.
- Indoor, wind-free. No ambient flow.
- Pair only, identical Crazyflies, same mass and rotor layout.
- PIV is one vertical plane through two front rotors, not a 3-D volume.
- Jet self-similarity only for $z/l>6.5$.
- $u_0=1$ in Eq. (4) "as in [4]" (normalised).
- Dynamic path is imposed sine on a rail, not two free vehicles.
- Dynamic envelope truncated by rail length.
- They themselves: near-field pair results may not carry to heterogeneous airframes.

## 7. Relation to our thesis (OUR READING)

- Supports which of our strategies: **none as a flown competitor**. Indirect support for **3 geo+NN** and **4 FBL+residual** as *why a residual exists* (measured $\Delta F_z$, $\Delta M_y$, approach/retreat hysteresis). Does not implement or benchmark 1 geometric, 2 INDI, or 5 hybrid NA-INDI/L-INDI.
- What we may cite: rig $F$ and $M$ vs $(\Delta x,\Delta z)$; moments exist and peak at $\Delta x/l=2$; unsteadiness scales; PIV merge at $z/l=6.5$; far-field jet algebra; dynamic approach worse than retreat; dataset as characterisation ground truth **in our reading**.
- What we must NOT cite: onboard STM32 timing; OptiTrack flight errors; 3-robot teams; INDI/FBL/NN residual performance; "they flew close formations"; compensation subtracted in acceleration units; fair controller bake-off on the same airframe.
- Formations to reproduce if we use it as a reference condition: stacked $\Delta x=0$; offset $\Delta x/l=2$; vertical stations near $\Delta z/l=4$ and $12.5$; do not treat $\Delta z/l\approx 19$ as a flown keep-out unless we re-measure in free flight.
- Compute / STM32: nothing to port. No onboard law. Lab PC + DaVis + load-cell DAQ. Crazyradio only sends hover PWM.

## 8. Equations worth keeping

**(1)** induced velocity

$$
U_i=\sqrt{F_z/(2\rho A n)}
$$

$F_z$ thrust, $\rho$ air density, $n=4$ propellers, $A=\pi r^2$.

**(2)** half-width definition

$$
\bar u(r_{1/2}(z),z)=\tfrac12 u_c(z)
$$

$u_c$ centreline axial speed.

**(3)** linear jet growth (far field only)

$$
r_{1/2}(z)=S(z-z_0)
$$

$S$ spread rate, $z_0$ virtual origin. This study: $S=0.0667$, $z_0=-6.0585$ (Table I; units as fitted in that figure).

**(4)** centreline decay

$$
u_c(z)=u_0\, Bd/(z-z_0)
$$

$u_0=1$ as they set it; this study $Bd=5.868$.

**(5)** scaled axial profile

$$
\bar u(\xi)=u_c(z)\big/\big(1+(\sqrt{2}-1)\xi^2\big)^{2/3}
$$

$\xi=$ lateral coordinate $/\,r_{1/2}$.

**(6)** scaled lateral profile

$$
\bar v(\xi)=\tfrac12 u_c(z)\,(\xi-\xi^3)/(1+\xi^2)^2
$$

**(7)–(8)** Reynolds number at merge

$$
\mathrm{Re}=u_c r_{1/2}/\nu\approx (4.25\,\mathrm{m/s})(0.0325\,\mathrm{m})/(1.5\times 10^{-5})\approx 9000
$$

**(9)** CrazyRail separation

$$
\Delta z(t)=\Delta z_{\min}+A(1+\sin(\omega t)),\quad \omega=2\pi f,\quad \Delta\dot z=A\omega\cos(\omega t)
$$

Pair speed they plot: $\bar U=\sqrt{\bar u^2+\bar v^2}$.

## 9. Figures / tables worth remembering

- **Fig. 1:** left, single-vehicle $\tilde u$ field, merge $z/l>6.5$; right, long-exposure CrazyRail approach.
- **Fig. 2:** Case I stacked vs Case II offset. $F$ and $M$ arrows.
- **Fig. 3:** stage, Nano-17s, $\Delta x,\Delta z$, $l$, pitch about $y$.
- **Fig. 4:** PIV plane through two front rotors.
- **Fig. 5:** upper vehicle maps: mean $F_z$, std $F'$, mean $M_y$, std $M'$.
- **Fig. 6:** same four maps for the **lower** vehicle. This is the money figure for residual size.
- **Fig. 7:** single-vehicle $\bar u,\bar v$ fields and profiles vs $z/l$.
- **Fig. 8:** $r_{1/2}(z)$; linear only after $z/l=6.5$.
- **Fig. 9:** $u_c$ and $u_{\max}$ vs $z$; they meet after merge.
- **Figs. 10–11:** self-similar $\bar u(\xi)$, $\bar v(\xi)$ vs Eq. (5)–(6).
- **Table I:** $Bd$, $S$, $z_0$ vs Bauersfeld.
- **Fig. 12:** stacked pair $\bar U$ at $\Delta z/l=12.5$ and 4.
- **Fig. 13:** offset pair $\bar U$ at the same two heights.
- **Fig. 14 + Table II:** CrazyRail geometry and sine parameter ranges.
- **Fig. 15:** stacked dynamic loops $F_z$, $M_y$ vs $\Delta z$, coloured by $\Delta\dot z$.
- **Fig. 16:** offset dynamic loops. Static means as black points with error bars.

## 10. Open uncertainties

- Exact Crazyflie revision and mass in grams.
- Propeller radius $r$ and disk area $A$ as numbers.
- Which PWM bit value is "hover."
- True rpm vs the 320 Hz estimate when the lower disk is immersed.
- Load-cell filter / anti-aliasing.
- Air density used in $U_i$.
- How they chose $z_0$ beyond "trace back."
- Exact $\Delta x$ of every PIV "offset" frame (body says $\Delta x/l=2$).
- Which Table II $(A,f,\Delta z_{\min})$ triples produced which loop colour.
- Whether both vehicles stayed at the same 4 V during dynamic runs.
- Dataset file layout, units, and coordinate conventions (only the DOI is here).
- RSS vs arXiv pagination differences: **not stated in this file**.

## 11. 12-line abstract I can reread

Kiran et al. mount two Crazyflies on a stage and a rail. They do not fly a team.
They log 6-DOF loads and a centre-plane PIV sheet.
Lower stacked thrust is 65% of hover at $\Delta z/l=4$; conclusions also say 65–70% for small stacked gaps.
Thrust looks recovered near $\Delta x/l\approx 3$ and $\Delta z/l\approx 19$.
Pitch moment peaks at $\Delta x/l=2$. Upper loads are much smaller than lower.
Isolated jets merge at $z/l=6.5$ and then follow a round-jet fit at $\mathrm{Re}\approx 9000$.
Close stack puts the lower disk in a lumpy near-field. Farther stack is a wider, smoother jet.
Offset wakes are asymmetric.
CrazyRail sines show approach hits the lower vehicle harder than retreat.
Offset dynamics add a restoring-moment problem. Stacked dynamics mostly change $F_z$.
No controller, no NN, no STM32, no 3-robot run.
Use it as a measured disturbance atlas, not as a method we compete against.

Source: attached PDF only. No other paper used.
