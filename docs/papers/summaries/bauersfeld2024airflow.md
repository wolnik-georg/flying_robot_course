# Quadrotor downwash as a turbulent jet — Bauersfeld et al., 2024

## 0. Identity

- Full title: Robotics meets Fluid Dynamics: A Characterization of the Induced Airflow below a Quadrotor as a Turbulent Jet
- Authors: Leonard Bauersfeld, Koen Muller, Dominic Ziegler, Filippo Coletti, Davide Scaramuzza
- Year / venue / arXiv / DOI: IEEE Robotics and Automation Letters, preprint accepted Dec 2024. Manuscript received 29 Aug 2024, revised 31 Oct 2024, accepted 28 Nov 2024. Editor: Giuseppe Loianno. This file header also prints `arXiv:2403.13321v2 [cs.RO] 2 Dec 2024`. DOI line on the file says only "see top of this page." No numeric DOI in the body.
- Version of THIS file: RA-L preprint + arXiv v2 (2 Dec 2024). Short letter. Video link on the title page: `https://youtu.be/-erimxW1ZPs`. Whether a longer companion exists is not in this paper.
- Suggested cite key: `bauersfeld2024airflow`
- What this paper IS: a hover flow-measurement paper. It fits a closed-form time-averaged turbulent-jet model to the merged downwash far below one quadrotor, then shows one under-pass throttle correction.
- What this paper is NOT: not Crazyflie; not INDI / NA-INDI / L1 / FBL; not a learned residual; not a formation-control paper; not a near-field first-principles model; not CFD of the rotors.

## 1. Problem they pose

Their wording (abstract / §I): "the need to understand and characterize the strong downwash." CFD is "expensive and time-consuming" and usually limited to "millisecond time scales" and flow "only close to the vehicle." Pointwise mapping of each airframe in 3-D is "tedious." They ask: "How much wind does a drone generate when hovering or flying slowly in a near-hover state?"

Plain restatement: they want a cheap mean-speed map under a hovering quadrotor that works across sizes, without a new CFD run per vehicle.

## 2. Paper map (follow their TOC)

### I. Introduction

- Purpose: motivate far-field downwash (safety, inspection, spray, multi-agent, sensor placement). Contrast CFD and per-vehicle empiricism.
- Method / construction: none yet. Fig. 1 smoke + measured field on Kolibri.
- Key equation: none numbered here.
- Assumption: hover / near-hover is the regime that matters for the claimed model.
- Result claimed (abstract): combined propeller flow "well-approximated by a turbulent jet after 2.5 drone-diameters below the vehicle." "over 16 hours of flight data" from drones of "varying sizes." Model needs "mass, propeller size, and drone size." Controller demo: "four times lower altitude deviation when passing below."

### Contribution (still §I)

- Purpose: name the claim.
- Method: join turbulent-jet theory with robotics flight data. "six differently sized drones ranging from 230 g to 6.3 kg." "over 16h."
- Key equation: none.
- Assumption: a single characteristic length plus $U_H$ can collapse different airframes.
- Result: "unified, computationally lightweight model" for "mean velocity of the induced flow below a drone at hover." Plus a "pen-and-paper algorithm." Plus the under-pass controller.

### II. Related Work

- Purpose: separate rotor force/torque models (quadratic, BEM) from volumetric flow (CFD, probes, PIV, smoke).
- Method: literature only. They say force models "do not compute the combined total induced volumetric flow." Most PIV/smoke work is tethered; they flag that as a limit on free-flight physics.
- Key equation: none.
- Assumption: none extra.
- Result: positioning only. "no studies have looked into a comprehensive analysis of the flow below drones in hover while focusing on a scaling analysis, and on closed-form solutions from turbulent flows."

### III. Theory

#### III-A Coordinate System and Notation

- Purpose: body frame $\mathcal{B}$ and flow frame $\mathcal{F}$.
- Method: hover only, so $\hat z_B$ stays opposite gravity. Flow axis $\hat s$ points down. Cylindrical $(s,r,\theta)$. Velocities $U,V,W$. Geometry: propeller radius $r_{\mathrm{prop}}$, propeller diameter $d$, motor distance $l$ (Fig. 2).
- Key equation: none numbered.
- Assumption: no roll/pitch rotation of the flow frame.
- Result: notation only.

#### III-B Propeller-Induced Flow

- Purpose: velocity scale from momentum theory.
- Method: four propellers, total thrust $mg$. Actuator-disk / flow-tube average just below the disk.
- Key equations:
  - (1) $U_H=\sqrt{T_H/(2\rho A_{\mathrm{prop}})}=\sqrt{mg/(2\rho\pi r_{\mathrm{prop}}^2 N_P)}$ with $N_P=4$
  - (2) $\rho=pM/(R\vartheta)$, $M=28.966\,\mathrm{g/mol}$, $R=8.3144\,\mathrm{J/(K\cdot mol)}$
- Assumption: air above the disk at rest. Tube width equals propeller diameter. They call this an "oversimplification" of the profile, but a valid average by momentum balance.
- Result: $U_H$ is the later normalization speed. Table I lists $U_H$ per vehicle.

#### III-C Turbulent Jet Flow

- Purpose: far-field mean field from classical jets (Pope).
- Method: time average $\langle\cdot\rangle$. Self-similar radial profile. Linear half-width. Centerline $\propto 1/(s-s_0)$. Swirl dropped because rotors counter-rotate. Side flow treated as entrainment.
- Key equations:
  - (3) $\langle U\rangle(\xi)=U_C(s)/\bigl(1+(\sqrt{2}-1)\xi^2\bigr)^2$
  - (4) $\xi=r/r_{1/2}(s)$
  - (5) $U_C(s)=U_J\, B d/(s-s_0)$
  - (6) $r_{1/2}(s)=S(s-s_0)$, opening $\theta=2\arctan(S)$
- Assumption: turbulent mean, not instantaneous. Spreading "commonly around 12 deg" and "weakly dependent" on $\mathrm{Re}_d$. Independent of viscosity in their use.
- Result: template for later fit. No drone-specific constants yet.

### III-D Normalization

- Purpose: collapse different masses and sizes.
- Method: speed by $U_H$. Length by motor distance $l$ ("closest equivalent to a nozzle's diameter").
- Key equations:
  - (7) $\langle\tilde U\rangle=\langle U\rangle/U_H$, $\tilde U_C=U_C/U_H$
  - (8) $\tilde s=s/l$, $\tilde r=r/l$, $\tilde r_{1/2}=r_{1/2}/l$
- Assumption: $l$ is the right geometric scale for all planar uncanted quads they test.
- Result: claimed unification. Validated later in Figs. 7–9.

### IV. Flight Experiments

#### IV-A Experimental Setup

- Purpose: how they measured.
- Method: indoor hall, 36-camera Vicon, tracking volume "25×25×7 m." Omnidirectional Testo 440 hot-ball probe. Accuracy "±0.03 m/s ±5% (up to 20 m/s)." Sample period 1 s. Measures speed magnitude, not a vector, not vortices.
- Key equation: none.
- Assumption: large hall limits recirculation. Probe is enough because they want mean magnitude.
- Result: setup only. Fig. 3: Matrice 300 next to the probe.

#### IV-B Quadrotors

- Purpose: list airframes (Table I).
- Method: three research platforms (Kolibri, Offboard 1, Offboard 2) fly autonomous mocap missions. Offboard 1 and 2 share a frame; Offboard 2 has about double the mass. DJI Matrice 300 is the large uncanted case. DJI Mavic 3E (inward cant) and Flyability Elios 3 (outward cant) are flown manually to test limits.
- Key equation: none. Numbers in Table I (see §5).
- Assumption: uncanted planar layout is the model class. Canted frames are stress tests, not the fit set.
- Result: six vehicles, 0.230 kg to 6.300 kg.

#### IV-C Data Collection

- Purpose: sampling pattern and cleaning.
- Method: grid. Approach a point, hover 5 s, move. Autonomous: coarse xy grid "10 × 10 length-scales (resolution 0.66 l)" and fine "3 × 3 length-scales (resolution 0.33 l)." z "up to 3 m above the probe." Matrice 300 manual box "5 × 6 × 4.5 m." Drop points with speed "greater than 0.1 m/s." Ambient probe reading "0.06 m/s to 0.12 m/s" subtracted. Bin by median in each cell.
- Key equation: none.
- Assumption: 5 s hover is enough for the mean to re-develop. Ambient is facility flow, not drone recirculation.
- Result: "around 10000 s of flight data" per vehicle after filtering. Abstract/contribution still say "over 16 hours" / "over 16h" for the campaign.

### V. Results: Near Field

- Purpose: when four jets become one.
- Method: yz slices (Fig. 4). Track offset $\tilde\delta$ of each core from the body $-z$ axis with a symmetric bimodal Gaussian (Fig. 5).
- Key equation: none numbered.
- Assumption: motor distance $l$ is the right plot scale. Body blockage explains the on-axis deficit.
- Result (body): jets "start to converge at about one length scale." "From about 2.5 length scales below the drone, the jets appear to have fully merged." "merge between 1.5 and 2.5 motor-distances." Fig. 4 dotted line at 2.5.

### VI. Results: Far Field

#### VI-A Radial Velocity Profiles

- Purpose: show self-similar radial decay after merge.
- Method: bin in $(s,r)$. Fig. 6 Kolibri raw. Fig. 7 all planar drones at $\tilde s=3.0$, normalized.
- Key equation: overlay of (3) on Fig. 7.
- Assumption: far field starts near $\tilde s=2.5$.
- Result: peak speeds "exceed the calculated induced velocity at hover by 25% at maximum" (expected: profile is not flat). After $U_H$ and $l$ scaling, planar curves "overlap" within measurement accuracy and match (3).

#### VI-B Jet Flow Scaling

- Purpose: fit half-width and centerline laws.
- Method: fit (3) at each $\tilde s$. Use only far-field points (circles, not triangles) and only uncanted drones. Joint fit of (5) and (6).
- Key equation:
  - (9) $Bd=10.11$, $S=0.07668$, $s_0=-5.817$
- Assumption: virtual origin $s_0$ may sit above the disk. Canted frames excluded from the fit.
- Result: half-width linear; they "recover a characteristic spreading angle of 10 deg" (Fig. 8 caption). Centerline follows $Bd/(s-s_0)$. Fig. 9: uncanted data vs (3) across the far field. Theory overpredicts for $\xi>2$ (Table II $t$-test, $\alpha=0.05$). Mavic inward: narrower, higher centerline. Elios outward: wider, lower centerline.

#### VI-C Unified Model

- Purpose: six-step recipe to evaluate $\langle U\rangle$ at a point $\mathbf{p}=[s,r,\theta]^\top$.
- Method: compute $U_H$ and $l$; normalize; evaluate (5)–(6) with (9) and $U_J=1$; form $\xi$; scale $U_C=\tilde U_C U_H$; plug into (3). Radial mean $\langle V\rangle$ "through continuity" if wanted. Cite Pope, no extra formula printed.
- Key equations: (1), (3), (5), (6), (9) used as a procedure. No new numbered law.
- Assumption: far field, hover, uncanted planar constants.
- Result: the "pen-and-paper algorithm."

### VII. Controller Integration

#### VII-A Downwash Compensation

- Purpose: show the jet model can feed a throttle correction when one vehicle flies under another.
- Method: incoming speed $U_D$ from the unified model. Momentum theory with non-zero inflow gives extra induced speed $U'_H$. Relative power $\beta$. Kolibri ESC "quadratic relationship between the throttle command and the power consumption," so they scale throttle by $\sqrt{\beta}$. Implemented "on top of an MPC controller [46]."
- Key equations:
  - (10) $U'_H=U_D/2+\sqrt{(U_D/2)^2+U_H^2}$
  - (11) $\beta:=P'_H/P_H=\alpha/2+\sqrt{\alpha^2+4}/2$, $\alpha=U_D/U_H$ (body text: "$\alpha=U_D/U_H$"; also writes $\alpha=U_D/U'_H$ in one sentence. See §10.)
- Assumption: extra hover power is the right correction. Quadratic ESC map. Relative pose $\Delta p$ known.
- Result: law only. No gain table.

#### VII-B Experimental Results

- Purpose: one under-pass test.
- Method: Offboard 2 hovers at 3 m. Kolibri passes "exactly below" at 1 m and 2 m height, "translation speed of 1 length-scale per second."
- Key equation: none new. Table III, Fig. 10.
- Assumption: lower vehicle stays on the upper vehicle's axis. Far-field half-width drawn from 2.5 $l$ down (Fig. 10).
- Result: without correction, "deviates up to 30 cm." RMSE "3 to 5-fold improvement." Abstract: "four times lower altitude deviation." Table III numbers in §5.

### VIII. Conclusion and Discussion

- Purpose: restate scope and point to fluid-mechanics extensions they did not test.
- Method: none new. They name cross-flow jets, rotor vortices, impinging jets, wall jets as future routes (citations only).
- Key equation: none.
- Assumption: far field is what matters "on the environment and other agents."
- Result: near field "can only be fully captured through CFD." Far-field jet model is the contribution.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Six quads. Kolibri 0.230 kg, $l=11.8\,\mathrm{cm}$. Offboard 1 0.572 kg and Offboard 2 1.207 kg, same frame $l=26.6\,\mathrm{cm}$. Matrice 300 6.300 kg, $l=89.4\,\mathrm{cm}$. Mavic 3E 0.958 kg inward cant. Elios 3 2.398 kg outward cant. Prop diameters 7.37–53.34 cm (Table I). |
| n robots min/max | Flow maps: 1 vehicle vs a fixed probe. Controller demo: 2 (one hover, one pass under). No 3-robot test in this paper. |
| Sim / hardware / both | Hardware only for the claimed field and the demo. No CFD numbers in the results. |
| Onboard vs off-board (Hz) | Autonomous research drones fly in Vicon. Probe at 1 s. Where the jet formula and MPC ran (onboard MCU vs off-board) is not in this paper. No Hz for the controller in this paper. |
| Sensing (mocap, RPM, GPS, …) | 36-camera Vicon, "millimeter accuracy." Testo 440 hot-ball speed. No RPM, IMU, or GPS used in the reported reduction. |
| Controller family | Demo only: MPC from their Agilicious stack [46], plus a scalar throttle scale $\sqrt{\beta}$. Not INDI, not FBL, not NN. |
| How interaction enters the law | $U_D(\Delta p)$ from the jet model → (10)–(11) → multiply throttle. Feed-forward. |
| Learned model? inputs → output | No learned model. Analytic jet. Inputs: mass, $r_{\mathrm{prop}}$, $N_P$, $l$, point $(s,r)$. Output: mean speed $\langle U\rangle$. |
| Training data amount / source | Not training. Identification: ~10 000 s filtered per vehicle; campaign "over 16 hours" / "over 16h." Fit of (9) on uncanted far-field slices only. |
| Formations + min separation | No formation. Under-pass heights 1 m and 2 m below a vehicle at 3 m. Merge distance 2.5 $l$, not a formation spacing. |
| Speeds | Identification: keep points with speed $\le 0.1\,\mathrm{m/s}$. Demo: "1 length-scale per second." |
| Metrics | Flow: collapse of $\langle\tilde U\rangle(\tilde r)$, $\tilde r_{1/2}(\tilde s)$, $\tilde U_C(\tilde s)$, residuals vs (3), Table II $t$-test. Control: height RMSE and mean error (Table III). |
| Baseline they compare to | Same MPC without the $\sqrt{\beta}$ scale. No geometric / INDI / NN baseline. |

## 4. Formations and flights (table, one row per experiment)

No formation-flight matrix in this paper. Do not import Neural-Swarm layouts.

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Identification grids (Kolibri, Offboard 1/2) | 1 + probe | hover over xy grid | z up to 3 m above probe | hover 5 s per cell; discard speed $>0.1\,\mathrm{m/s}$ | hw, Vicon | binned mean speed field; near-field merge; far-field jet collapse |
| Identification (Mavic 3E, Elios 3) | 1 + probe | same idea, manual | "similar pattern" | manual | hw | canted jets: narrower (Mavic) or wider (Elios) |
| Identification (Matrice 300) | 1 + probe | hover box | volume 5 × 6 × 4.5 m | manual | hw | same normalized near/far pattern as smaller uncanted quads |
| Under-pass, 1 m | 2 | Kolibri on the vertical axis under Offboard 2 | upper at 3 m; lower at 1 m | straight pass, 1 length-scale/s | hw | Table III: RMSE 69.1 mm → 24.8 mm; mean −37.6 → 18.9 mm |
| Under-pass, 2 m | 2 | same | upper at 3 m; lower at 2 m | same | hw | Table III: RMSE 134.2 mm → 25.4 mm; mean −87.1 → 8.1 mm. Uncorrected peak "up to 30 cm." |

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Flight time | "over 16 hours" / "over 16h" / "over 16 h" | campaign duration | abstract, contribution, §VIII | After filters: "around 10000 s" per vehicle. Six vehicles. Do not treat 16 h as one continuous hover. |
| Mass range | 230 g to 6.3 kg | vehicle mass | abstract / contribution | Table I: 0.230, 0.572, 1.207, 6.300, 0.958, 2.398 kg. |
| Merge distance | "2.5 drone-diameters" | abstract length | abstract, Fig. 1 uses "2.5 motor-to-motor distances $l$" | Body uses $l$, not propeller diameter $d$. Also "1.5 and 2.5 motor-distances." |
| Peak vs $U_H$ | 25% higher | max measured peak vs momentum $U_H$ | §VI-A | Profile is peaked. $U_H$ is a disk average. |
| Jet constants | $Bd=10.11$, $S=0.07668$, $s_0=-5.817$ | joint far-field fit | (9), Fig. 8 | Uncanted only. $s_0$ is a virtual origin. |
| Spreading | "10 deg" vs theory "around 12 deg" | jet opening | Fig. 8 caption vs §III-C | Their fit is 10 deg. Do not cite 12 deg as their measurement. |
| Ambient | 0.06–0.12 m/s | facility background speed | §IV-C | Subtracted. "about 2 orders of magnitudes below" $U_H$. |
| Probe spec | ±0.03 m/s ±5%, ≤20 m/s, 1 s | instrument | §IV-A | Magnitude only. |
| Speed gate | 0.1 m/s | max allowed vehicle speed in the map | §IV-C | Near-hover, not cruise. |
| Grids | 10×10 at 0.66 $l$; 3×3 at 0.33 $l$; Matrice 5×6×4.5 m | sample domain | §IV-C | Autonomous vs manual differ. |
| Under-pass RMSE | 69.1 / 24.8 mm at 1 m; 134.2 / 25.4 mm at 2 m | height RMSE | Table III | Two heights, one pair of airframes, one speed. |
| Mean height error | −37.6 / 18.9 mm at 1 m; −87.1 / 8.1 mm at 2 m | signed mean | Table III | Sign flips when correction is on. |
| Abstract vs body on control | "four times lower altitude deviation" vs "3 to 5-fold improvement" RMSE vs "up to 30 cm" uncorrected | different error statements | abstract / §VII-B / Fig. 10 | Do not collapse into one factor. Body does not retract the abstract. |
| Table II | reject $H_0$ for $\xi\in[2,5)$ | theory overpredicts $\langle U\rangle/U_C$ | Table II | They call the mismatch "negligible" vs ambient. |
| Kolibri $U_H$ | 7.41 m/s | Table I | Table I | From (1), not a probe peak. |

## 6. Assumptions and failure modes

- Hover or "slowly in a near-hover state." Identification uses speed $\le 0.1\,\mathrm{m/s}$.
- Time-averaged speed magnitude. No instantaneous vortex, no swirl, no full vector from the probe.
- Far-field jet only after merge ($\tilde s\gtrsim 2.5$ in units of $l$).
- Length scale is motor distance $l$, not tip-to-tip span.
- Fit constants (9) from uncanted planar rotors. Inward cant focuses the jet. Outward cant widens it.
- Air above an isolated disk at rest when defining $U_H$. Density from dry-air ideal gas (2).
- Counter-rotating rotors cancel swirl. Other components "entrained."
- Indoor, low ambient wind. Background subtracted.
- Near field: body blockage and four separate cores. They say this "can only be fully captured through CFD."
- Controller: extra hover power from axial inflow (10)–(11). Quadratic ESC. Relative position known. One vehicle on the axis under another.
- Theory (3) overpredicts for $\xi>2$.
- Ground effect, walls, cross-flow, forward flight: named as future work, not validated here.

## 7. Relation to our thesis (clearly labelled OUR READING)

- Which of our strategies it supports: none as a controller family. It does not implement geometric, INDI, geo+NN, FBL+residual, or hybrid NA-INDI/L-INDI. It can support a physics check on the residual we subtract: the far-field mean axial speed under a hoverer.
- What we may cite: hover far-field mean downwash as a turbulent jet after $\approx 2.5\,l$; 16 h / six sizes; $U_H$ and $l$ scaling; closed-form (3)+(5)+(6)+(9); near field not closed-form; time-averaged magnitude only; canted layouts break the unified constants; one 2-robot under-pass as an existence proof that a mean-flow feed-forward can cut height error.
- What we must NOT cite from it: Crazyflie brushless teams; 3-robot formations; OptiTrack (they used Vicon); STM32; onboard rates; INDI or learned force residuals; formation geometry; cruise interaction; "they designed a multi-agent controller" as the paper's result.
- Formations we should reproduce if we use it as a reference condition: not a formation paper. If we only match their demo: leader hover, follower on-axis at 1 m and 2 m below a 3 m hover, slow pass. Better scientific use: report our spacings in units of $l$ and say whether we sit in their near field ($<2.5\,l$) or far field.
- Compute: the six-step jet evaluation is a few flops. It would run on an STM32. What they actually ran is not in this paper. The demo sits "on top of an MPC controller [46]." MPC compute platform and rate: not in this paper.

## 8. Equations worth keeping

- (1) Hover induced speed $U_H=\sqrt{mg/(2\rho\pi r_{\mathrm{prop}}^2 N_P)}$. Disk-average, isolated hover.
- (2) $\rho=pM/(R\vartheta)$. Dry air. $M=28.966\,\mathrm{g/mol}$, $R=8.3144\,\mathrm{J/(K\cdot mol)}$.
- (3) Mean axial jet $\langle U\rangle(\xi)=U_C(s)/\bigl(1+(\sqrt{2}-1)\xi^2\bigr)^2$.
- (4) $\xi=r/r_{1/2}(s)$.
- (5) $U_C(s)=U_J Bd/(s-s_0)$. In the recipe they set $U_J=1$ then multiply by $U_H$.
- (6) $r_{1/2}(s)=S(s-s_0)$.
- (9) Fitted constants $Bd=10.11$, $S=0.07668$, $s_0=-5.817$ (uncanted far field).
- (10) Inflow-corrected induced speed $U'_H=U_D/2+\sqrt{(U_D/2)^2+U_H^2}$.
- (11) Relative hover power $\beta=\alpha/2+\sqrt{\alpha^2+4}/2$. They set throttle scale $\sqrt{\beta}$ because ESC power $\propto$ throttle$^2$. Check $\alpha$ definition against the PDF before coding (see §10).

## 9. Figures / tables worth remembering

- Fig. 1: Kolibri smoke vs measured field. Separate cores near the vehicle. Merged jet and half-width in the far field.
- Fig. 2: frames $\mathcal{B}$ and $\mathcal{F}$; $r_{\mathrm{prop}}$, $d$, $l$; components $U,V,W$.
- Fig. 3: Matrice 300 and Testo hot-ball probe.
- Table I: six vehicles, cant, prop diameter, motor distance, $U_H$.
- Fig. 4: near-field yz slices, Offboard 1 and Matrice 300, axes in units of $l$. Merge line at 2.5.
- Fig. 5: core offset $\tilde\delta(\tilde s)$. Merge 2–3 length scales.
- Fig. 6: Kolibri radial profiles at several heights. Peak above $U_H$. Spreading with $s$.
- Fig. 7: planar drones at $\tilde s=3.0$. Normalized collapse onto (3).
- Fig. 8: $\tilde r_{1/2}(\tilde s)$, $\tilde U_C(\tilde s)$, Kolibri field with half-width overlay. Far-field points only in the fit.
- Fig. 9: $\langle U\rangle/U_C$ vs $\xi$ for all six. Uncanted follow (3) near $\xi=0$. Canted off the curve.
- Table II: $t$-test that theory overpredicts for $\xi\in[2,5)$.
- Table III: height RMSE and mean error, 1 m and 2 m, with/without correction.
- Fig. 10: under-pass tracks and a time-lapse at 2 m. Half-width cone from 2.5 $l$.

## 10. Open uncertainties

- Numeric DOI: not printed in the body.
- Exact Vicon rate: not in this paper.
- Controller / MPC rate, computer, and whether (3) ran onboard: not in this paper.
- Filter on the probe beyond the 1 s sample: not in this paper.
- Air temperature and pressure used in (2) for Table I $U_H$: not in this paper.
- Which motor-pair defines $l$ on non-square frames: Fig. 2 only.
- Whether $d$ in $Bd=10.11$ is propeller diameter or a fitted product. They print the product $Bd$, not $B$ and $d$ apart.
- $\alpha$ in §VII-A: one sentence says $\alpha=U_D/U'_H$, the next block uses $\alpha=U_D/U_H$ to get (11). The printed (11) matches $\alpha=U_D/U_H$ if $P'_H=U'_H T_H$ and $P_H=U_H T_H$. Do not guess beyond that. Flag before implementing.
- How they chose 5 s hover as developed-flow time: not derived.
- Ground clearance of the probe and hall-floor effect: not specified.
- Repeat count per grid cell: not in this paper.
- Gains of the underlying MPC: not in this paper.

## 11. 12-line abstract I can reread

Bauersfeld et al. (RA-L 2024 / arXiv:2403.13321v2) map time-averaged downwash speed under hovering quadrotors with a hot-ball probe and Vicon.
Six airframes, 0.23–6.3 kg. Campaign wording: over 16 h. After filters: ~10 000 s per vehicle.
Near the disk the four rotor jets are separate. They merge at about 1.5–2.5 motor distances $l$. Abstract says "2.5 drone-diameters."
After that, uncanted planar vehicles collapse onto one turbulent-jet profile when speed is scaled by momentum $U_H$ and length by $l$.
Fit (far field, uncanted): $Bd=10.11$, $S=0.07668$, $s_0=-5.817$. Opening in their figure: 10 deg.
Model output is mean magnitude, hover / near-hover only. Probe is not a vector PIV field.
Canted Mavic (in) and Elios (out) deviate. Near field stays CFD-class in their own words.
They then scale Kolibri throttle by $\sqrt{\beta}$ from momentum theory plus this $U_D$, on top of MPC, while passing under Offboard 2.
Height RMSE drops 69.1→24.8 mm at 1 m and 134.2→25.4 mm at 2 m. Abstract: four times lower deviation. Body: 3 to 5-fold RMSE.
This is a flow model plus one under-pass demo. It is not an INDI, NN-residual, or formation paper.
Cite for far-field hover mean flow and for the near-field limitation. Do not cite for STM32 teams or 3-robot geometry.
If our spacing is below ~2.5 $l$, we are outside their closed-form regime.

Source: attached PDF only. No other paper used.
