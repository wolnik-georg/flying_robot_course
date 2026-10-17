# Gielis-aggregate — Gielis et al., 2023

## 0. Identity

- **Full title:** Modeling Aggregate Downwash Forces for Dense Multirotor Flight
- **Authors:** Jennifer Gielis, Ajay Shankar, Ryan Kortvelesy, Amanda Prorok
- **Year / venue / arXiv / DOI:** 2023. This file is arXiv:2312.03488v1 [cs.RO], 6 Dec 2023. No conference or journal name is printed. No DOI is printed.
- **Version of THIS file:** arXiv v1 (12 pages).
- **Suggested cite key:** `gielis2023aggregate`
- **What this paper IS:** A load-stand measurement study of how downwash forces from 1–3 free-flying neighbours add (or fail to add) on one constrained sufferer, plus linear vs Deep-Sets predictors of that net force.
- **What this paper is NOT:** Not a flight controller paper. Not INDI, not MPC, not Neural-Swarm. Not Crazyflie. Not free-flight force sensing on the sufferer. Not a demonstrated GNN. Not a successful $K>3$ neighbour test.

## 1. Problem they pose

**Their wording (abstract / §1):** Even in a homogeneous team, "an accurate model of downwash forces from one vehicle is unlikely to be sufficient for predicting aggregate forces from multiple vehicles in formation." Reactive controllers cannot reject these "exogenic" forces without first observing them. Dense flight here means vehicles "frequently fly within 1–3 body-lengths."

**Two questions they write down:**
- **Q1:** Given a single-neighbour model $\vec{f}$, can pairwise linear combination estimate the collective force from $K$ neighbours?
- **Q2:** If not, does a learned nonlinear set architecture have enough expressivity?

**Plain restatement:** One-vehicle downwash maps exist. Nobody had measured whether you can just sum them when several vehicles sit in the same wake, or whether the wakes merge and the sum is wrong.

## 2. Paper map (follow their TOC)

### Abstract

- Purpose: state the gap, the rig, the two model classes, and the fourth-vehicle sentence.
- Method: 6-DOF test rig; formations as a "graph"; learn an aggregation function; contrast with linear models.
- Equations: none numbered.
- Assumption: homogeneous team; sufferer on the rig.
- Result claimed: "strong non-linearities" in aggregation; models "generalize when a fourth vehicle is added" (this last clause is **abstract-only**; body is weaker).

### 1 Motivation

- Purpose: why dense formation flight needs a force model before the controller sees the hit.
- Construction: contrast single-neighbour analytic [4] and data-driven [3] models with the naïve sum $f'=\sum_j \vec{f}(x_j)$.
- Assumption: additivity "might hold for a variety of flight regimes," but this had not been tested.
- Result: none yet. Sets Q1 and Q2.

### Related Work (still under §1 in this file)

- Purpose: place the paper against CFD, single-vehicle load-cell work, and Shi et al. Neural-Swarm.
- What they actually say about Shi [8]: small quadrotors; "implicitly learn the impact of neighbour's presence"; "some improvement in trajectory tracking through a learning-based downwash prediction." They do **not** attribute a superposition test to Shi.
- Closest prior they name: Li et al. [11] — learning-based single-quadrotor downwash correction with load-cell ground truth. They say they go further: up to 3 neighbour multirotors, 6-DOF stand, several model classes.
- Claim: "To our knowledge, no previous work has empirically quantified the characteristics of downwash for a range of specific dense swarm formations to this depth."

### 2 Technical Approach

- Purpose: write the two predictors.
- Linear model (their Eq. 1). Trained on single-vehicle data if desired. Can be queried at a new $K$.
- Nonlinear model (their Eq. 2). Deep Sets [12]. $\phi$ embeds each relative state; sum; $\Phi$ maps the sum to force. Can represent $K$-wise effects. A new $K$ is out of distribution for this class.
- Assumption: relative state $x_j-x_i$ is a sufficient input. Fluid mixing may make $K$ and $K+1$ dynamically different.
- Result claimed here: none measured. Capacity argument only.

### 3 Experimental Setup

- Purpose: how they measure 6-DOF force without a free-flight controller in the loop.
- Construction: eight shear-beam load cells (10 kg), HX711 ADCs at 80 Hz, Raspberry Pi 4B **on the stand**. Four cells on motor booms → D-force, pitch, roll. Four lateral cells → N, E, yaw. Rubber dampers. Selectively compliant transfer elements. Stand on a pole 1.4 m above the floor "to avoid ground interaction aerodynamic effects."
- Sufferer is mechanically coupled at each motor joint. Propellers spin at hover thrust. Then cells are zeroed. Goal: "flying-like" mixing without controller response bias.
- Exploration volume: 5.6 m³ inside a 96 m³ space. Square 2 m on a side laterally, 1.4 m vertically above the stand. Trajectories 0.5 m/s, roughly E-axis. 30–40 straight legs per experiment. Mocap 180 Hz.
- Formations named: Side by Side, Leader Follower, Stack, plus 3 Hybrid (equilateral triangle, coplanar). Also an extension of Leader Follower to a third vehicle.
- Assumption: stand intrusion is constant ("observer effect" is admitted later). Straight legs avoid extra variable-thrust forces.
- Result: none yet. This section is the apparatus.

### 4 Results

- Purpose: answer Q1 and Q2 against stand readings.
- Three categories they name (§4 opening):
  1. Chaotic: dominated by noise
  2. Linear: well estimated by linear summation
  3. Non-linear: only estimated with non-linear models
- §4.1 Chaotic: N, E, yaw "generally dominated by noise." Measurement noise "readily quantifiable at +/- 0.05N." Environment and the stand itself also perturb flow.
- §4.2 Linear: tends to hold when one neighbour is much closer than the others, especially at "extremely low altitude offsets." Little room for wakes to mix before they hit the stand.
- §4.3 Non-linear: Leader–Follower $K=3$, D-axis. At low $\Delta z$ three peaks match a naïve sum (largest error "less than 20% of the measured forces"). At 0.8 m and 1.3 m the measured force concentrates away from the formation edges. Their boxed observation: "The downwash airflow of multirotors in near proximity tend to merge laterally, and reduces in overall cross-sectional area with increasing distance from the generating multirotors."
- §4.4 Models / Table 1: learned linear vs learned nonlinear, integral error over the lateral plane at 1.3 m relative altitude. Nonlinear better on D for all three tabulated formations. N/E/yaw similar or mixed.
- Fig. 7: naïve linear, learned linear, learned nonlinear, ground truth for Leader–Follower $K=3$ at 1.3 m. Only the nonlinear model follows the "reduced forces for negative E-axis positions."
- Assumption: 1.3 m slice is the comparison altitude for Table 1.
- Result location: body + Fig. 4–7 + Table 1.

### 5 Discussion

- Downwash "expand very little with vertical distance" and "remain concentrated approximately within the body radius up to at least 1.5m," originating vehicle "< 30cm in total diameter."
- Influence "negligible to its neighbours outside of its downwash (Fig. 4), including at very small (< 1 vehicle diameter) separations."
- Enclosed lab: some lateral forces at long range, "generally chaotic and of a small magnitude." Lateral position keeping of the formation "decreased with increasing $K$."
- Leader–Follower gives the largest prediction errors. They posit lead-vehicle turbulence. Stack also nonlinear, but "smaller initial lateral distance … reduces the observed nonlinearity versus Leader Follower … because the airflow undergoes less lateral contraction."
- Linear sum "insufficient" when the sufferer sees multiple downwash zones: "force estimation errors up to ≈ 3N." "700g platform" → "acceleration of ≈ 4m/s²" and "≈ 75 % of peak forces out of all experiments." Sharp downwash boundary → high rate of change.

### 6 Conclusion and Future Work

- Scope they claim: "formations of up to 4 multirotors in total," "close formation (< 1 body radius separation)."
- Deep-Sets "capable of outperforming simple linear summation for force prediction."
- Limits they write: only a "limited sample of velocities"; "lower velocities tend to exaggerate downwash effects." Nonlinear model "may be incapable of generalising to cases where $K > 3$."

### Acknowledgment / References

- Funding lines only. Not experimental.

## 3. Method card (table)

| Item | This paper |
|---|---|
| Platform (size, mass, name) | Multirotor, name **not in this paper**. Sufferer "700g platform." Originating vehicle "< 30cm in total diameter." |
| n robots min/max | 1 constrained sufferer + $K=1$ to $K=3$ free neighbours. "Up to 4 multirotors in total." |
| Sim / hardware / both | Hardware only. No simulation in this file. |
| Onboard vs off-board (Hz) | **No onboard compensator.** Load-stand Pi reads cells at 80 Hz. Mocap 180 Hz. Logging on the stand Pi. Neighbours fly; how their low-level loops run is **not in this paper**. |
| Sensing (mocap, RPM, GPS, …) | Motion-capture "positioning data to all entities at 180 Hz." Eight shear load cells. No RPM, IMU residual, or GPS stated. |
| Controller family | None on the sufferer. Sufferer is constrained. Neighbour flight controller "not in this paper." |
| How interaction enters the law | It does not. They **predict force**, they do not close a control loop. |
| Learned model? inputs → output | Yes. Input: relative states $x_j-x_i$. Output: predicted 6-axis force/torque $\hat f_i$. Two maps: $\psi$ (linear sum) and $\phi,\Phi$ (Deep Sets). Layer sizes **not in this paper**. |
| Training data amount / source | Stand flights, grid of straight legs, 30–40 legs per experiment. Dataset size in samples **not in this paper**. |
| Formations + min separation | Side by Side, Leader Follower, Stack, 3 Hybrid (equilateral triangle). Close formation "< 1 body radius." Exact fixed inter-vehicle spacing except the 0.5 m centroid offset in one Leader–Follower plot: **not in this paper**. |
| Speeds | 0.5 m/s. Other speeds not reported. |
| Metrics | Integral of model error minus ground truth over the lateral plane, at 1.3 m relative altitude (Table 1). Qualitative contour / line plots. |
| Baseline they compare to | Naïve linear sum of single-neighbour models; learned linear model; learned nonlinear (Deep Sets). |

## 4. Formations and flights (table, one row per experiment)

| Name | n | geometry | Δz / offset | path / speed | sim or hw | what they report |
|---|---|---|---|---|---|---|
| Side by Side | sufferer + $K=2$ in Table 1 | coplanar, perpendicular to travel | mobiles coplanar; formation-to-sufferer examples 0.3 / 0.8 / 1.3 m | straight legs, 0.5 m/s, ~E-axis | hw, sufferer on stand | Table 1 errors; simultaneous downwash arrival |
| Leader Follower | sufferer + $K=2$ described; Table 1 uses $K=3$ | coplanar, parallel to travel | same altitude set; $K=3$ mobiles at "0.5 m offset from the formation centroid" | same | hw, stand | strongest nonlinearity; Figs. 5–7; D-axis merge |
| Stack | sufferer + $K=2$ in Table 1 | distinct horizontal planes, small lateral offset so each sits in the wake above | inter-mobile $\Delta z$ **not in this paper**; formation-to-sufferer inside 1.4 m | same | hw, stand | Table 1; discussion: less lateral contraction than Leader–Follower |
| 3 Hybrid | 3 mobiles + sufferer | equilateral triangle, one lateral plane | coplanar mobiles; $\Delta z$ to sufferer varied; triangle side **not in this paper** | same | hw, stand | named and drawn (Fig. 3); **no row in Table 1** |
| Single neighbour (Fig. 4) | sufferer + 1 | one vehicle scanned over N–E | 0.3 / 0.8 / 1.3 m | same family of passes | hw, stand | D-axis field stays narrow; little lateral expansion |

No hover-only campaign. No circle / figure-8 / swap / docking. No free-flight sufferer.

## 5. Numbers that must stay qualified

| Claim | Number | What it is a number OF | Where | Caveat |
|---|---|---|---|---|
| Mocap rate | 180 Hz | positioning of "all entities" | §3 | Not the force sample rate. |
| Load-cell / ADC rate | 80 Hz | HX711 read rate | §3 | Stand, not flight stack. |
| Cell capacity | 10 kg | shear-beam rating | §3 | Not measured peak force. |
| Measurement noise | +/- 0.05N | "ADC read inaccuracy and mechanical vibrations," their phrase | §4.1 | They call this "readily quantifiable." Other noise is environmental. |
| Stand height | 1.4 m | pole height above floor | §3 | Chosen to avoid ground effect. |
| Exploration box | 2 m × 2 m lateral, 1.4 m vertical, 5.6 m³ | volume scanned above the stand | §3 | Inside a 96 m³ room. |
| Speed | 0.5 m/s | commanded leg speed | §3 | Only speed reported. |
| Relative altitudes in figures | 0.3 m, 0.8 m, 1.3 m | formation-to-sufferer height in Figs. 4–5 and Table 1 | Figs. 4–5, Table 1 caption | Table 1 is **only** the 1.3 m slice. |
| K=3 LF centroid offset | 0.5 m | each vehicle from formation centroid | §4.3 / Fig. 5 | One stated spacing. Do not reuse as the Side-by-Side or Hybrid spacing. |
| Low-altitude linear-sum error | "less than 20% of the measured forces" | largest error, LF $K=3$, low $\Delta z$ (top of Fig. 5) | §4.3 | Not the 1.3 m case. |
| Peak linear-model force error | "up to ≈ 3N" | force estimation error of simple summation vs stand | §5 | Discussion, not Table 1. |
| Platform mass | 700g | "our 700g platform" | §5 | Vehicle name not given. |
| Implied acceleration | "≈ 4m/s²" | $3\,\mathrm{N}/0.7\,\mathrm{kg}$ | §5 | Their conversion. |
| Fraction of peak | "≈ 75 % of peak forces out of all experiments" | 3 N relative to their global peak | §5 | Peak force itself not tabulated. |
| Body size | "< 30cm in total diameter" | originating multirotor | §5 | Not Crazyflie. |
| Wake width | "within the body radius up to at least 1.5m" | D-axis concentration | §5 | From Fig. 4 plus discussion. |
| Close formation | "< 1 body radius separation" | conclusion wording | §6 | Exact commanded spacing per formation mostly not given. |
| Dense-flight slogan | "1–3 body-lengths" | motivation regime | §1 | Slogan, not a tabled spacing. |
| Table 1 D-axis, LF $K=3$ | Linear 0.439 / Non-Linear 0.255 | integral error, lateral plane, 1.3 m | Table 1 | Units are that integral, not newtons at a point. |
| Fourth vehicle | "fourth vehicle is added" vs "up to 4 … in total" vs "$K>3$" | team size / generalisation | abstract / §6 | Abstract is stronger than the body. Table 1 max is $K=3$. |

## 6. Assumptions and failure modes

- Sufferer is **clamped**. Forces are not free-flight residuals after a position loop.
- Propellers fixed at hover thrust. No attitude or thrust transients on the sufferer.
- Relative Cartesian state $x_j-x_i$ is the only model input. RPM, thrust, attitude of neighbours: **not in this paper**.
- Linear model assumes forces add. They show this fails when wakes have distance to merge.
- Deep Sets with a new $K$ is out of distribution (they say this themselves).
- Stand and room objects change the flow ("observer effect"). Bias is assumed constant across tests, not zero.
- Indoor mocap arena. No wind.
- One speed, 0.5 m/s. They warn slower flight may exaggerate downwash.
- N, E, yaw often too noisy to model. Do not treat 6-DOF as equally trustworthy.
- Homogeneous team. No mixed airframes.
- Ground effect deliberately avoided (1.4 m). Ceiling effect not studied.

## 7. Relation to our thesis (OUR READING)

- **Supports as evidence, not as a controller:** strategies 3 geo+NN residual, 4 FBL+residual, 5 hybrid NA-INDI/L-INDI — only as "linear superposition of single-neighbour force maps is not always enough; a set aggregator is one way to represent that." Strategy 1 geometric baseline and 2 full INDI are **not** tested here.
- **What we may cite:** 6-DOF stand protocol; three regimes (chaotic / linear / nonlinear); wake merge + lateral contraction with $\Delta z$; Table 1 D-axis gap; ≈ 3 N / ≈ 4 m/s² scale on **their** 700 g vehicle; formations Side by Side, Leader Follower, Stack, coplanar triangle; 0.5 m/s straight legs; sufferer-independent-of-controller measurement.
- **What we must NOT cite from it:** Crazyflie results; onboard STM32 residual; closed-loop tracking improvement; OptiTrack numbers; INDI or FBL gains; "they flew 4 free robots"; "graph neural net"; Shi/Neural-Swarm superposition; successful $K>3$ generalisation; ground-effect or ceiling-effect data.
- **Formations to reproduce if we use it as a reference condition:** Leader–Follower first (their hardest nonlinear case), then Side-by-Side and Stack. Hold geometry while translating at constant speed. Include at least one low $\Delta z$ and one $\Delta z \approx 1\,\mathrm{m}$ class. Scale spacings to Crazyflie size. Do not copy 700 g / 30 cm numbers as if they were ours.
- **Compute / STM32:** they ran Deep Sets **off-board on logged stand data**. No inference rate, no embedded net, no flash/RAM figure. A tiny Deep-Sets residual *could* be small, but **this paper did not run it on a flight MCU**.

## 8. Equations worth keeping

Naïve sum they attack (§1, unnumbered):

$$
f'=\sum_{j=0}^{K}\vec{f}(x_j)
$$

Single-neighbour map queried per neighbour, then added.

Linear model, Eq. (1):

$$
\hat{f}_i=\sum_{j=1}^{K}\psi(x_j-x_i),\qquad \psi:\mathbb{R}^{d_{\mathrm{in}}}\to\mathbb{R}^{d_{\mathrm{out}}}
$$

Same pairwise map, summed. Can be trained on $K=1$ data.

Nonlinear / Deep Sets, Eq. (2):

$$
\hat{f}_i=\Phi\Biggl(\sum_{j=1}^{K}\phi(x_j-x_i)\Biggr),\qquad \phi:\mathbb{R}^{d_{\mathrm{in}}}\to\mathbb{R}^{h},\;\Phi:\mathbb{R}^{h}\to\mathbb{R}^{d_{\mathrm{out}}}
$$

Sum of embeddings, then a second net. Permutation-invariant. Not a full graph net in the body.

Frame: outputs in NED. $D$ is down (positive down force on the stand).

No control law. No residual-subtraction equation in this file.

## 9. Figures / tables worth remembering

- **Fig. 1:** problem sketch; load-stand photo; still of several vehicles over the clamped sufferer.
- **Fig. 2:** 8-cell stand diagram. White airframe, purple couplers, blue dampers, red cells, yellow compliant elements, grey transfer plate and pole.
- **Fig. 3:** four cartoons. Side by Side, Leader Follower, Stack, 3 Hybrid. Green = fixed sufferer. Arrows = travel direction.
- **Fig. 4:** single-neighbour D-force contours at 0.3 / 0.8 / 1.3 m. Field spreads little laterally; column stays about body-wide.
- **Fig. 5:** D-force vs formation E-position, Leader–Follower $K=3$, three altitudes. Three peaks at 0.3 m; peaks merge and the naïve sum over-predicts at 1.3 m.
- **Fig. 6:** N–E contours at 1.3 m, same case. (a) linear-sum model still shows three lobes. (b) measured field is merged / shifted; last vehicle's wake "appear[s] to move forwards."
- **Fig. 7:** line plot of naïve linear, learned linear, learned nonlinear, ground truth vs E. Nonlinear is the only one that tracks the reduced force on the negative-E side.
- **Table 1:** integral errors at 1.3 m for Side by Side $K=2$, Stack $K=2$, Leader Follower $K=3$, six axes, linear vs nonlinear. Lower is better. D is the axis with a clear nonlinear win.

## 10. Open uncertainties

- Airframe name, propeller diameter, motor type.
- Exact $\phi/\Phi/\psi$ widths, activations, optimiser, epochs, loss.
- Train/test split and number of samples.
- Exact Side-by-Side, Stack, and Hybrid inter-vehicle spacings (metres).
- Stack inter-mobile $\Delta z$.
- Hybrid triangle side length.
- Whether legs keep a perfectly rigid formation or allow tracking error (they note lateral keeping got worse as $K$ grew).
- Neighbour onboard controller and rate.
- How N, E, D, torques are linearly combined from the eight cells (matrix **not printed**).
- Filter on the 80 Hz cell stream.
- Whether "fourth vehicle added" is a real extra flight or just $K=3$ + sufferer = 4 total.
- Any number at a speed other than 0.5 m/s.

## 11. 12-line abstract I can reread

Gielis et al. 2023 clamp one multirotor to an 8-cell 6-DOF stand, spin it at hover thrust, and fly 1–3 neighbours through Side-by-Side, Leader–Follower, Stack, and a coplanar triangle. Speed is 0.5 m/s on straight legs. Mocap 180 Hz, cells 80 Hz. They ask whether single-neighbour downwash maps may be summed. At small height offset the sum is often close. At about 1.3 m, wakes merge, the D-force field shrinks and shifts, and the sum is wrong. They fit a linear sum and a Deep Sets net on the same stand data. The net cuts D-axis integral error in Table 1. N, E, and yaw stay noisy. Biggest linear-sum miss they quote is about 3 N on a 700 g vehicle, about 4 m/s². Abstract says "graph" and "fourth vehicle"; the body is Deep Sets and $K\le 3$ neighbours (4 vehicles total), and they doubt $K>3$. No closed-loop controller. No Crazyflie. No STM32 residual.

Source: attached PDF only. No other paper used.
