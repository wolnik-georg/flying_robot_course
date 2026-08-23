# Planning Visualization Guide

This guide explains the figures generated for planning references and planning closed-loop simulations.
It is intentionally focused on **plot interpretation** (what is shown, units, and how to read it), not on solver internals.

---

## 1) Where the plots come from

Recommended command (uses `flying_robots` pyenv by default):

- `scripts/plot_planning_all.sh`

If needed, override interpreter explicitly:

- `PYENV_PYTHON=~/.pyenv/versions/flying_robots/bin/python scripts/plot_planning_all.sh`

Reference-only plots (planner output, no controller/simulation tracking):

- Source data: `results/planning_sim/reference/mode*/<trajectory>/reference.csv`
- Script: `scripts/plot_planning_reference.py`
- Main outputs per folder:
  - `overview.png`
  - `trajectory_3d_orientation.png`
  - `frenet_sanity.png`
  - `limits_dashboard.png` — actuation feasibility dashboard (see Section 5b)
- Cross-mode overlays: `results/planning_sim/cross_mode/<trajectory>/comparison.png`
  - Shows Modes 0/1/2/3 overlaid on the same axes (3D path, thrust, |ω|, speed)
  - Mode 3 shown in purple; only modes that have a reference for that trajectory appear

Closed-loop plots (reference vs simulated tracking — Geo controller only by default):

- Source data: `results/planning_sim/closed_loop/mode*/<trajectory>/geo.csv`
- Script: `scripts/plot_planning_sim.py`
- Outputs: `results/planning_sim/closed_loop/images/fig*.png`
  - Split 3D figures: `fig1_split/mode{0,1,2,3}_{traj}_3d.png` (one file per run)

---

## 2) Quick metric glossary

- `p = [x,y,z]` [m]: position.
- `v = p_dot` [m/s]: velocity.
- `a = p_ddot` [m/s^2]: acceleration.
- `j = p_dddot` [m/s^3]: jerk (rate of change of acceleration).
- `s = p^(4)` [m/s^4]: snap (rate of change of jerk).
- `yaw` [rad], `yaw_dot` [rad/s], `yaw_ddot` [rad/s^2]: flat yaw and derivatives.
- `omega = [omega_x, omega_y, omega_z]` [rad/s]: body angular velocity.
- `angacc = [angacc_x, angacc_y, angacc_z]` [rad/s^2]: body angular acceleration (`omega_dot`).
- `thrust_N` [N]: collective thrust magnitude required by flatness mapping.
- `torque = [torque_x, torque_y, torque_z]` [N*m]: body torque from rigid-body dynamics.

Interpretation:

- Higher `|j|` and `|s|` usually means more aggressive attitude and motor commands.
- `|omega|` and `|angacc|` indicate rotational aggressiveness (how hard and how quickly the attitude changes).
- `thrust_N` and `torque` approaching feasibility limits indicate little control margin.

---

## 3) Reference plot: `overview.png`

`overview.png` is the main "all-in-one" planner-reference figure for one mode and one trajectory.

### Position and geometry rows

- **3D position panel**: world-frame path; waypoints shown as markers.
  - Orientation glyphs show sampled body axes along the path.
- **Position vs time** (`px, py, pz`): useful for checking continuity and segment transitions.
- **Top/side views** (`x-y`, `x-z`, `y-z`): useful for quickly seeing trajectory shape distortion.

What to look for:

- Smooth path through waypoints without discontinuities.
- Expected geometric shape (circle/figure8/helix/loop) in the projected views.

### Translational derivatives row

- **Velocity panel**: `|v|` plus components (`vx, vy, vz`).
- **Acceleration/Jerk/Snap panel**:
  - Solid: `|a|`, `|j|`, `|s|`
  - Dotted: acceleration components (`ax, ay, az`)

What to look for:

- Peaks in `|j|` and especially `|s|` indicate where aggressiveness is concentrated.
- Sharp spikes near segment junctions can happen, but persistent large spikes suggest overly tight timing.

### Attitude and yaw row

- **Flat yaw** panel: `yaw`, `yaw_dot`, `yaw_ddot`.
- **Euler panel**: `roll`, `pitch`, `yaw` from orientation.
- **Quaternion panel**: `qw, qx, qy, qz` (continuous attitude representation).

What to look for:

- Prefer quaternion continuity for robust interpretation.
- Euler angles can look odd near singular configurations (pitch near +/- 90 deg); this is a representation issue, not always a physical discontinuity.

### Rotational dynamics and force/torque row

- **Thrust panel**: `thrust_N` with feasibility limit lines.
- **Body rate panel**: `|omega|` and components, with `|omega|` limit line.
- **Body angular acceleration panel**: `|angacc|` and components.
- **Torque panel**: `torque_x, torque_y, torque_z`.

What to look for:

- Time intervals where signals approach red feasibility bands are likely to be hardest to track.
- Large `|angacc|` typically aligns with rapid changes in tilt/yaw behavior and can imply higher torque demand.

### Segment-junction vertical lines

Dotted vertical lines in time-series panels come from `segment_junctions.csv` and indicate piecewise-polynomial boundaries.

How to use them:

- Mild shape changes at junctions are normal.
- Repeated large derivative spikes exactly at many junctions may indicate over-constrained timing or too short segment durations.

---

## 4) Reference plot: `trajectory_3d_orientation.png`

Purpose: a clean, large 3D trajectory view with orientation triads.

- Black curve: reference path.
- Waypoint markers: planned waypoint locations.
- Triads:
  - red = body-x
  - green = body-y
  - blue = body-z

How to read it:

- Body-z (blue) indicates thrust direction in world frame.
- Rapid triad rotation over short path distance means aggressive attitude change.
- For near-planar paths, this figure helps distinguish heading/yaw evolution from translational motion.

---

## 5) Reference plot: `frenet_sanity.png`

Purpose: geometric diagnostics + derivative consistency checks.

- `kappa` [1/m]: curvature, computed from `v` and `a`.
- Tangential/normal acceleration estimates: split of acceleration relative to path direction.
- Arc length `s(t)`: integrated path length.
- Relative derivative consistency:
  - compares finite-difference `gradient(position)` against CSV velocity,
  - compares finite-difference `gradient(velocity)` against CSV acceleration.

How to read it:

- Small consistency errors indicate numerically coherent exported derivatives.
- Local spikes can appear near boundaries/junctions or very low-speed points.

---

## 5b) Reference plot: `limits_dashboard.png`

A consolidated actuation feasibility dashboard, generated alongside `overview.png` for every reference trajectory folder.

### Panels (4×2 layout)

| Panel | What it shows |
|-------|--------------|
| **Thrust / rate / torque utilisation** | Normalised peak usage vs limits (0–1 bar chart); >0.9 = high-risk |
| **Violation windows** | Time intervals where thrust, |ω|, or |τ| exceed configured limits |
| **Motor-mix feasibility** | Pre-clamp per-motor ω² — negative values indicate the mixer would require negative thrust on that motor |
| **Per-motor speed & thrust** | Individual motor angular speeds [kRPM] and thrust [mN] vs time |
| **Rate-of-change proxies** | dT/dt, d|τ|/dt, d|ω|/dt — steep slopes stress actuators even within static limits |
| **Sustained violation bars** | Fraction of trajectory in violation + longest continuous violation window [s] per axis |
| **Battery sag margin** | Model-based estimate of voltage sag under peak load; margin to minimum cell voltage |

### How to read it

- All bars in the utilisation panel should ideally be below 0.85 for robust tracking margin.
- Negative pre-clamp ω² in the motor-mix panel means the trajectory is geometrically infeasible for a + or × mixer — the drone cannot produce the required net wrench.
- The rate-of-change proxies are the planning-side analogue of actuator bandwidth: high dT/dt requires fast ESC response; high d|ω|/dt requires fast attitude dynamics.
- Battery sag: the estimate uses `motor_kf`, `motor_kt`, `motor_arm_length_m` from `planning_meta.txt`. It is model-based — actual sag depends on battery state of charge and cell age.

---

## 6) Closed-loop figures (`fig1` ... `fig6`)

These compare reference trajectories against simulated tracking for controllers:

- **Geo**: solid lines
- **INDI**: dotted lines

### `fig1_split/mode{0,1,2,3}_{traj}_3d.png`

- One file per (mode, trajectory) combination: reference path (grey) vs Geo controller (solid colour).
- Split output avoids the overcrowded combined figure. Covers Modes 0/1/2/3 × all trajectories.
- Primary visual check of path-following quality and gross deviations per run.

### `fig2_position_errors.png`

- Component-wise position errors over time and 3D error summaries.
- Best for identifying axis-specific tracking weaknesses.

### `fig3_attitude.png`

- Reference vs simulated `roll/pitch/yaw`.
- For inversion maneuvers (Mode 2), check expected excursions near `pi` or `2*pi`.

### `fig4_thrust_mode01.png` and `fig4b_thrust_mode2.png`

- Reference and commanded thrust profiles.
- Mode 2 figure highlights negative-thrust regions (important for interpretation of inversion-like segments).

### `fig5_rms_summary.png`

- Aggregated RMS 3D tracking error by mode/trajectory/controller.
- Quick comparative quality metric across all runs.

### `fig6_cross_mode.png`

- Same shape overlaid across available modes.
- Best for understanding how planner mode changes trajectory execution difficulty and tracking outcome.

---

## 7) Practical interpretation checklist

For one trajectory, read in this order:

1. `overview.png`: check shape, derivative peaks, feasibility bands.
2. `trajectory_3d_orientation.png`: inspect attitude evolution along path.
3. `frenet_sanity.png`: verify geometric smoothness and derivative consistency.
4. Closed-loop figures (`fig1`/`fig2`/`fig5`): evaluate tracking quality.

If tracking degrades:

- First inspect reference aggressiveness (`|j|`, `|s|`, `|omega|`, `|angacc|`, thrust proximity to limits).
- Then inspect where in time errors spike and whether they align with segment junctions.

---

## 8) Notes and caveats

- Units and sign conventions come from exported CSV headers and planning scripts.
- Euler plots are helpful for intuition but can be misleading near singular attitudes; use quaternions and body-axis glyphs for continuity.
- "Feasible" in planning reports means constraints are met under configured limits; it does not guarantee perfect closed-loop tracking under all controller settings.
