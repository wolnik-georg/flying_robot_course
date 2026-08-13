# Advanced Flying Robots - INDI Controller: Current Progress CW4

## Main Message

The current stack has progressed from early geometric-controller tracking tests toward onboard Rust trajectory execution and a richer motion-planning comparison across modes 0-3. The main technical contribution is not only generating new reference trajectories, but also validating them with feasibility/limits plots, closed-loop simulation, and real-flight log analysis.

## Geometric Controller On Hardware

### Results of the geometric controller running on Crazyflie 2.1 with optical flow sensor

#### 1) Initial State: `figure8_20260423_145011.csv`

![Initial figure8 analysis](Controls/logs/figure8_20260423_145011_analysis.png)
![Initial figure8 orientation](Controls/logs/figure8_20260423_145011_3d_orientation.png)

#### 2) Intermediate Improvement: `figure8_20260427_200834.csv`

![Intermediate figure8 analysis](Controls/logs/figure8_20260427_200834_analysis.png)
![Intermediate figure8 orientation](Controls/logs/figure8_20260427_200834_3d_orientation.png)

#### 3) One of the best Results Yet (Geo Controller + Onboard Rust): `figure8_onboard_m0_s1-0x_r1_20260506_162613.csv`

![Best figure8 onboard main analysis](Controls/logs/figure8_onboard_m0_s1-0x_r1_20260506_162613_analysis.png)
![Best figure8 onboard per-axis](Controls/logs/figure8_onboard_m0_s1-0x_r1_20260506_162613_analysis_axes.png)
![Best figure8 onboard kinematics](Controls/logs/figure8_onboard_m0_s1-0x_r1_20260506_162613_analysis_kinematics.png)
![Best figure8 onboard orientation](Controls/logs/figure8_onboard_m0_s1-0x_r1_20260506_162613_3d_orientation.png)


---

## Motion Planning: Modes 0-3

| Mode | Core Idea | Attitude Source | Time Scaling | Presentation Evidence |
|---|---|---|---|---|
| 0 | Base spline trajectory | Flatness-derived | Manual/basic | Figure-8 appendix baseline |
| 1 | Richter-like minimum-snap with time allocation | Flatness-derived | `k_t`-based | Main Figure-8 reference plot set |
| 2 | Explicit SE(3): authored attitude + position | Uploaded/explicit | Richter timing + SO(3)/SLERP attitude | Vertical-loop appendix evidence |
| 3 | Paper-faithful flat-output planning with stronger timing coupling | Flatness-derived | Iterative/paper-style | Main Figure-8 reference plot set |

### Concise Implementation Notes

- Modes 0/1/3 use the same minimum-snap spline backbone: 8th-order polynomials with one small QP solved per axis (`x`, `y`, `z`, `yaw`) and then combined into a multi-axis trajectory. This keeps the implementation robust, debuggable, and fast enough for iteration.
- The current per-axis QP design is sufficient because cross-axis physics is handled downstream through differential flatness, controller logic, and planning-side feasibility checks. A larger joint QP would mainly become valuable if hard coupled constraints (tilt cones, corridor constraints, actuator limits) must be enforced inside the optimizer itself.
- Mode 1 timing is allocated from one aggressiveness knob:
  $$
  v_{avg}=0.5+1.5\sqrt{k_t}, \qquad
  T_i=\max\left(\frac{\lVert p_{i+1}-p_i\rVert}{v_{avg}},\ 0.15s\right)
  $$
- Mode 3 is the same Richter-family planner with iterative time redistribution enabled:
  $$
  T_i^{target}=\max\left(T_{total}\frac{J_i}{\sum_j J_j},T_{\min}\right), \qquad
  T_i \leftarrow T_i + 0.5(T_i^{target}-T_i)
  $$
  where `J_i` is the segment snap cost. The implementation also applies safety clamps to avoid numerically bad segment times.
- Mode 2 still uses the spline/minimum-snap stack for position, but attitude comes from explicit quaternion/SO(3) interpolation and is uploaded as attitude polynomials for onboard evaluation.
- Feasibility plots and limits dashboards are planning-side model/proxy checks. They are useful for comparing modes and spotting risk, but hardware saturation claims still need closed-loop logs or actuator telemetry.

### Latest Figure-8 `k_t` Scan Summary

The latest scan selected the largest sampled `k_t` values that stayed below the configured planning-side utilization limit (`target_utilization_max = 0.99`). Values marked `below_band` are still safe in the scan but did not reach the tighter 0.95-0.99 target band. The final slides focus this table on the Figure-8 planners shown in the main evidence; Mode 2 is instead shown with a vertical loop because explicit attitude planning is most meaningful for inversion-capable maneuvers.

| Mode | Figure-8 `k_t` | Scan Status | Interpretation |
|---|---:|---|---|
| 1 | `0.3107` | `below_band` | Highest safe sampled Richter timing value found for Figure-8. |
| 3 | `0.7116` | `below_band` | Highest safe sampled paper-style timing value found for Figure-8. |

### Common Planning Backbone

All four modes are easiest to compare if we separate the common trajectory-generation backbone from the mode-specific timing and attitude choices.

| Shared Aspect | More Specific Formulation / Meaning |
|---|---|
| Waypoint objective | Given waypoints `w_i`, generate one polynomial segment between each pair and enforce that the segment endpoints pass through the waypoints. |
| Segment representation | The implementation stores each segment on normalized time: $$\hat t=\tau/T_i,\qquad \sigma_i(\hat t)=\sum_{k=0}^{8}c_{i,k}\hat t^k,\qquad \hat t\in[0,1].$$ Physical-time derivatives scale as $$d^r\sigma_i/d\tau^r=(1/T_i^r)d^r\sigma_i/d\hat t^r.$$ |
| Minimum-snap objective | The common smoothness objective is based on snap. For a normalized segment, the physical-time per-axis cost is $$J_i=\frac{1}{T_i^7}\int_0^1\left(\frac{d^4\sigma_i}{d\hat t^4}\right)^2d\hat t.$$ This `1/T_i^7` scaling is why short segments become aggressive/numerically sensitive. |
| Per-axis QP structure | For each axis, the problem becomes: $$\min_{\mathbf{c}}\frac{1}{2}\mathbf{c}^\top Q\mathbf{c}+\mathbf{q}^\top\mathbf{c}\quad \text{s.t.}\quad A_{eq}\mathbf{c}=b_{eq}$$. We solve smaller independent QPs for `x`, `y`, `z`, and yaw where applicable instead of one large coupled QP. |
| Boundary / continuity constraints | `A_eq c = b_eq` contains waypoint endpoint constraints, rest-to-rest endpoint constraints for non-periodic paths, periodic-loop derivative matching for closed paths, and physical-time derivative continuity for `r=1..4` between adjacent segments. |
| Flat-output interpretation | The planned flat outputs are `x, y, z, yaw` and their derivatives. For modes 0/1/3, full attitude/thrust/rates are derived afterwards using differential flatness. |
| Flatness force direction | The desired force direction is approximately $$f=m(a+g e_3),\qquad z_B=\frac{f}{\lVert f\rVert}$$, where `z_B` is the desired body z-axis. |
| Flatness attitude construction | With yaw reference `ψ`, $$x_C=[\cos\psi,\sin\psi,0]^\top,\quad y_B=\frac{z_B\times x_C}{\lVert z_B\times x_C\rVert},\quad x_B=y_B\times z_B,\quad R=[x_B\ y_B\ z_B]$$. |
| Feasibility checking | The sampled plots check proxy limits: thrust, angular velocity, torque/motor mixing, kinematic derivatives, state axes, and 3D orientation. These are planning-side checks before hardware validation. |
| Downstream use | All modes feed the same simulator/controller/log-analysis chain, so the resulting plots can be compared mode-by-mode. |

### What Makes Each Mode Different

| Mode | What Stands Out | Exact / Useful Formulation | What To Point Out In The Plots |
|---|---|---|---|
| 0 — Spline | Baseline minimum-snap spline with manually chosen/fixed segment times. | Segment times `T_s` are provided directly. The optimizer only solves for coefficients `c`; it does not decide the timing. Objective and constraints still follow the shared per-axis QP form. | This is the reference baseline. Differences in limits/kinematics come mainly from fixed timing rather than automatic aggressiveness tuning. |
| 1 — Richter | Same minimum-snap backbone, but segment times are allocated automatically from `k_t`. | $$v_{avg}=0.5+1.5\sqrt{k_t},\qquad T_i=\max\left(\frac{\lVert p_{i+1}-p_i\rVert}{v_{avg}},0.15\right)$$. Larger `k_t` means larger `v_avg`, shorter `T_i`, and more aggressive references. | Compare against Mode 0: similar path shape, but timing/derivatives/limits change because segments are no longer manually timed. |
| 2 — SE(3) | Position uses the same planning family, but attitude is explicitly authored instead of purely flatness-derived. | Position timing comes from Richter-like durations. Attitude waypoints use quaternion interpolation: $$q(s)=\operatorname{SLERP}(q_i,q_{i+1},s),\qquad s=\tau/T_i$$. Roll/pitch attitude polynomials are then fitted for onboard evaluation/upload. | The 3D orientation plot matters most. The final presentation shows a vertical loop because this mode is most meaningful when attitude inversion is intentionally authored. |
| 3 — Paper | Same Richter-family flat-output planner, but with iterative segment-time redistribution based on segment snap cost. | Start from the Mode 1-style `k_t` allocation, then iterate using segment costs `J_i`: $$T_i^{target}=\max\left(T_{total}\frac{J_i}{\sum_j J_j},T_{\min}\right),\qquad T_i\leftarrow T_i+0.5(T_i^{target}-T_i)$$. In the implementation this paper-style path uses multiple redistribution iterations and safety clamps. | Compare against Mode 1: same flatness-derived attitude and same planner family, but segment times shift, so local kinematics/limits can improve or worsen per segment. |

The key presentation takeaway is: the modes do not represent four completely unrelated planners. They share the same smooth-polynomial planning philosophy; the main differences are how segment times are chosen and whether attitude is derived from flatness or explicitly authored.

### Mode 0 (SplineTrajectory) — Figure-8

![Mode0 figure8 reference overview](flying_drone_stack/results/planning_sim/reference/mode0/figure8/overview.png)
![Mode0 figure8 kinematics](flying_drone_stack/results/planning_sim/reference/mode0/figure8/kinematics_axes.png)
![Mode0 figure8 state axes](flying_drone_stack/results/planning_sim/reference/mode0/figure8/state_axes.png)
![Mode0 figure8 limits dashboard](flying_drone_stack/results/planning_sim/reference/mode0/figure8/limits_dashboard.png)
![Mode0 figure8 3D trajectory orientation](flying_drone_stack/results/planning_sim/reference/mode0/figure8/trajectory_3d_orientation.png)

- Description: baseline piecewise polynomial trajectory with fixed segment durations.
- Formulation: minimum-snap polynomial solve with waypoint + continuity constraints.
- QP view: solve quadratic cost over polynomial coefficients with equality constraints.
- Core objective:
  $$
  \min \sum_{s}\int_{0}^{T_s}\left\lVert \frac{d^4 p_s(t)}{dt^4} \right\rVert^2 dt
  $$
- Canonical QP form:
  $$
  \min_{\mathbf{c}} \frac{1}{2}\mathbf{c}^\top \mathbf{Q}\mathbf{c} + \mathbf{q}^\top\mathbf{c}
  \quad \text{s.t.} \quad \mathbf{A}_{eq}\mathbf{c} = \mathbf{b}_{eq}
  $$
- Continuity constraints: physical-time derivative continuity from velocity through snap (`r=1..4`) across segment boundaries.
- Attitude: derived via differential flatness from planned `x,y,z,yaw` (flat outputs converted to attitude/thrust/rates for feasibility and control references).
- Code references:
  - planner core: `flying_drone_stack/src/planning/mod.rs`, `flying_drone_stack/src/planning/spline.rs`
  - simulation/reference: `flying_drone_stack/src/bin/planning_sim.rs`, `flying_drone_stack/scripts/plot_planning_reference.py`, `flying_drone_stack/scripts/plot_planning_sim.py`
  - onboard/controller/analytics: `flying_drone_stack/src/bin/onboard_*.rs`, `flying_drone_stack/firmware_app/src/lib.rs`, `Controls/analyze_flight.py`

### Mode 1 (RichterTrajectory) — Figure-8

![Mode1 figure8 reference overview](flying_drone_stack/results/planning_sim/reference/mode1/figure8/overview.png)
![Mode1 figure8 kinematics](flying_drone_stack/results/planning_sim/reference/mode1/figure8/kinematics_axes.png)
![Mode1 figure8 state axes](flying_drone_stack/results/planning_sim/reference/mode1/figure8/state_axes.png)
![Mode1 figure8 limits dashboard](flying_drone_stack/results/planning_sim/reference/mode1/figure8/limits_dashboard.png)
![Mode1 figure8 3D trajectory orientation](flying_drone_stack/results/planning_sim/reference/mode1/figure8/trajectory_3d_orientation.png)

- Description: Richter-style minimum-snap with automatic segment timing from `k_t`.
- Formulation: same polynomial/QP structure as mode 0, with time allocation coupled via aggressiveness.
- QP view: same constrained per-axis minimum-snap QP after automatic timing allocation.
- Core objective:
  $$
  \min \sum_{s}\int_{0}^{T_s}\left\lVert \frac{d^4 p_s(t)}{dt^4} \right\rVert^2 dt
  $$
- Canonical QP form:
  $$
  \min_{\mathbf{c}} \frac{1}{2}\mathbf{c}^\top \mathbf{Q}\mathbf{c} + \mathbf{q}^\top\mathbf{c}
  \quad \text{s.t.} \quad \mathbf{A}_{eq}\mathbf{c} = \mathbf{b}_{eq}
  $$
- Time allocation: segment durations depend on geometry and aggressiveness parameter `k_t`.
- Attitude: differential-flatness-derived; no explicit attitude waypoint authoring.
- Code references:
  - planner core: `flying_drone_stack/src/planning/mod.rs`, `flying_drone_stack/src/planning/richter.rs`
  - simulation/reference: `flying_drone_stack/src/bin/planning_sim.rs`, `flying_drone_stack/scripts/plot_planning_reference.py`, `flying_drone_stack/scripts/plot_planning_sim.py`
  - onboard/controller/analytics: `flying_drone_stack/src/bin/onboard_*.rs`, `flying_drone_stack/firmware_app/src/lib.rs`, `Controls/analyze_flight.py`

### Mode 2 (Se3Trajectory) — Vertical Loop

![Mode2 vertical loop reference overview](flying_drone_stack/results/planning_sim/reference/mode2/loop/overview.png)
![Mode2 vertical loop kinematics](flying_drone_stack/results/planning_sim/reference/mode2/loop/kinematics_axes.png)
![Mode2 vertical loop state axes](flying_drone_stack/results/planning_sim/reference/mode2/loop/state_axes.png)
![Mode2 vertical loop limits dashboard](flying_drone_stack/results/planning_sim/reference/mode2/loop/limits_dashboard.png)
![Mode2 vertical loop 3D trajectory orientation](flying_drone_stack/results/planning_sim/reference/mode2/loop/trajectory_3d_orientation.png)

- Description: explicit SE(3) trajectory generation with authored attitude path capability.
- Formulation: position uses the same minimum-snap spline/QP family, while attitude is represented explicitly by quaternion waypoints and SLERP.
- QP view: position still uses the spline QP; attitude is not solved by that QP but authored/interpolated separately on SO(3).
- Time allocation: timing comes from Richter-like allocation, then explicit attitude interpolation/evaluation is performed.
- Attitude: explicit (uploaded/authored) SE(3) orientation pipeline.
- Code references:
  - planner core: `flying_drone_stack/src/planning/mod.rs`, `flying_drone_stack/src/planning/se3_traj.rs`, `flying_drone_stack/src/planning/richter.rs`
  - simulation/reference: `flying_drone_stack/src/bin/planning_sim.rs`, `flying_drone_stack/scripts/plot_planning_reference.py`, `flying_drone_stack/scripts/plot_planning_sim.py`
  - onboard/controller/analytics: `flying_drone_stack/src/bin/onboard_*.rs`, `flying_drone_stack/firmware_app/src/lib.rs`, `Controls/analyze_flight.py`

### Mode 3 (Paper / PaperRichterTrajectory) — Figure-8

![Mode3 figure8 reference overview](flying_drone_stack/results/planning_sim/reference/mode3/figure8/overview.png)
![Mode3 figure8 kinematics](flying_drone_stack/results/planning_sim/reference/mode3/figure8/kinematics_axes.png)
![Mode3 figure8 state axes](flying_drone_stack/results/planning_sim/reference/mode3/figure8/state_axes.png)
![Mode3 figure8 limits dashboard](flying_drone_stack/results/planning_sim/reference/mode3/figure8/limits_dashboard.png)
![Mode3 figure8 3D trajectory orientation](flying_drone_stack/results/planning_sim/reference/mode3/figure8/trajectory_3d_orientation.png)

- Description: paper-faithful flat-output minimum-snap planning with stronger timing coupling behavior.
- Formulation: flat-output polynomial minimum-snap with continuity/endpoint constraints and iterative timing refinement.
- QP view: Richter-family constrained QP structure with paper-style timing redistribution emphasis.
- Core objective:
  $$
  \min \sum_{s}\int_{0}^{T_s}\left\lVert \frac{d^4 p_s(t)}{dt^4} \right\rVert^2 dt
  $$
- Canonical QP form:
  $$
  \min_{\mathbf{c}} \frac{1}{2}\mathbf{c}^\top \mathbf{Q}\mathbf{c} + \mathbf{q}^\top\mathbf{c}
  \quad \text{s.t.} \quad \mathbf{A}_{eq}\mathbf{c} = \mathbf{b}_{eq}
  $$
- Time allocation: segment durations depend on geometry and aggressiveness parameter `k_t`, with stronger paper-style iterative timing redistribution behavior.
- Attitude: emergent from differential flatness (not explicitly authored in waypoints).
- Code references:
  - planner core: `flying_drone_stack/src/planning/mod.rs`, `flying_drone_stack/src/planning/richter.rs` (`plan_paper` path)
  - simulation/reference: `flying_drone_stack/src/bin/planning_sim.rs`, `flying_drone_stack/scripts/plot_planning_reference.py`, `flying_drone_stack/scripts/plot_planning_sim.py`
  - onboard/controller/analytics: `flying_drone_stack/src/bin/onboard_*.rs`, `flying_drone_stack/firmware_app/src/lib.rs`, `Controls/analyze_flight.py`

---

## Current Limitations And Next Steps

- The planning plots are model/proxy checks; final hardware claims should be based on real flight logs and actuator telemetry where available.
- The next validation step is to fly selected Mode 1/3 Figure-8 trajectories and selected Mode 2 attitude maneuvers with the updated `k_t` values, then compare their logs against the existing onboard Mode 0 baseline where appropriate.
- The INDI controller remains the next major control-side topic: the current results mainly demonstrate geometric-controller behavior and planning-stack progress.
- The Beamer presentation should remain a shorter story: motivation, hardware progress, planning modes, evidence plots, `k_t` tuning, and roadmap.
