# Trajectory — Reference Signal Generators

## 1. Overview

The trajectory module produces the reference signal that the controller tracks. At each timestep the controller asks: *"where should the drone be right now, and how fast should it be moving?"* The trajectory module answers that question.

All trajectory types implement a single trait: `Trajectory`. This means any trajectory can be dropped into any control loop without changing the controller, the simulator, or the main flight binary — the controller only ever sees a `TrajectoryReference`.

The module also contains `SequencedTrajectory`, which chains multiple trajectories into a single flight program (takeoff → stabilise → manoeuvre → hold).

---

## 2. The `Trajectory` Trait

**`src/trajectory/mod.rs:11`**

```rust
pub trait Trajectory {
    fn get_reference(&self, time: f32) -> TrajectoryReference;
    fn duration(&self) -> Option<f32> { None }
}
```

`get_reference(t)` is the one required method. It maps a wall-clock time `t` [s] to a complete `TrajectoryReference`. The controller calls this every loop iteration.

`duration()` is optional — finite trajectories return `Some(T)` so that simulation loops can terminate automatically. `SequencedTrajectory` and `CsvTrajectory` return `Some`; analytical trajectories that repeat forever return `None`.

### What `TrajectoryReference` Contains

```
position     [m]       desired XYZ
velocity     [m/s]     desired velocity (feedforward for kv term)
acceleration [m/s²]    desired acceleration (feedforward for F_des)
jerk         [m/s³]    desired jerk (feedforward for angular velocity)
yaw          [rad]     desired heading
yaw_rate     [rad/s]   desired yaw rate (feedforward)
yaw_acceleration [rad/s²]  desired yaw acceleration
```

The richer the reference (more non-zero feedforward fields), the tighter the tracking — the controller generates commands that anticipate future motion rather than only reacting to current error.

---

## 3. Trajectory Types

### 3.1 `CircleTrajectory` — Horizontal circle, analytical

**`src/trajectory/mod.rs:260`**

Uniform circular motion in the XY plane at constant height. All derivatives are computed analytically from the circular motion equations:

```
θ(t) = ω·t
p(t) = [cx + r·cos θ,  cy + r·sin θ,  z]

v(t) = [−r·ω·sin θ,   r·ω·cos θ,    0]       (tangential, |v| = r·ω)
a(t) = [−r·ω²·cos θ,  −r·ω²·sin θ,  0]       (centripetal, toward center)
j(t) = [r·ω³·sin θ,   −r·ω³·cos θ,  0]       (jerk, tangential)
```

Yaw is set to point tangent to the circle (`θ + π/2`) and yaw rate equals `ω`.

**Key parameters:**

| Parameter | Meaning |
|-----------|---------|
| `radius` | Circle radius [m] |
| `height` | Flight altitude [m] |
| `omega` | Angular velocity [rad/s]; `2π/T` for period `T` |
| `center` | (x, y) center of circle |

**Example** — 0.3m radius circle at 0.25m height, period ~16 s:
```rust
CircleTrajectory::new(0.30, 0.25, 2.0*PI/16.0)
```

---

### 3.2 `Figure8Trajectory` — Polynomial figure-8 from Bitcraze

**`src/trajectory/mod.rs:22`**

A 10-segment, 7th-order polynomial figure-8 trajectory. The coefficients were extracted directly from the Bitcraze `autonomous_sequence_high_level.py` example — the same trajectory used in official Crazyflie demos.

Each segment has a duration (in seconds) and 8 coefficients for x, y, z, and yaw. The segment is parameterised on `s ∈ [0, 1]` (normalised time within that segment) and each physical-time derivative is recovered by dividing by the segment duration raised to the appropriate power.

**Time scaling**: `time_scale > 1` stretches time (slower flight, same shape). The total duration is `Σ segment_durations × time_scale`. All velocities/accelerations scale accordingly.

**Segment durations** (base, ×time_scale): `1.05, 0.71, 0.62, 0.70, 0.56, 0.56, 0.70, 0.62, 0.71, 1.05` seconds (≈7.3 s total at `time_scale = 1`).

The trajectory loops — `segment_time = time % self.duration` wraps it.

---

### 3.3 `SmoothFigure8Trajectory` — Lissajous figure-8, analytical

**`src/trajectory/mod.rs:40`**

A mathematically cleaner alternative to the polynomial figure-8, using a Lissajous parametrisation:

```
x(t) = a · sin(ω·t)
y(t) = b · sin(2ω·t)
```

where `ω = 2π/duration`. This is C∞ (all derivatives continuous), loops at period `duration`, and all derivatives are analytical:

```
vx = a·ω·cos(ωt)                 ax = −a·ω²·sin(ωt)
vy = 2bω·cos(2ωt)                ay = −4bω²·sin(2ωt)
jx = −a·ω³·cos(ωt)               jy = −8bω³·cos(2ωt)
```

Yaw points in the direction of velocity (`atan2(vy, vx)`) and yaw rate is computed via the quotient rule. This trajectory is smoother than the polynomial version and better for testing the jerk feedforward path of the controller.

---

### 3.4 `TakeoffTrajectory` — Minimum-jerk ascent

**`src/trajectory/mod.rs:476`**

Lifts the drone from `z = 0` to `z = target_z` in `duration` seconds with zero velocity and acceleration at both ends. The Z profile is the classic minimum-jerk polynomial:

```
p(s) = 10s³ − 15s⁴ + 6s⁵       s = t / duration ∈ [0, 1]
```

Why this polynomial? It is the unique 5th-order polynomial satisfying:
```
p(0) = 0,  ṗ(0) = 0,  p̈(0) = 0
p(1) = 1,  ṗ(1) = 0,  p̈(1) = 0
```
Minimising jerk keeps the thrust command smooth during the ascent, preventing the drone from slamming into hover at the top.

X and Y are held constant at `(start_x, start_y)`. Yaw is held constant at the specified value throughout.

All four derivatives (position through jerk) are available analytically and are provided to the controller.

---

### 3.5 `CsvTrajectory` — Externally defined waypoints

**`src/trajectory/mod.rs:330`**

Loads a CSV file with columns `time, x, y, z, yaw` and linearly interpolates between rows at query time. Velocity is computed as finite-difference `Δpos / Δt` between the bracketing waypoints. Acceleration and jerk are set to zero (no smoothing).

Because acceleration is zero, the controller's feedforward term is absent — the trajectory is tracked by feedback only. This makes CsvTrajectory suitable for slow, pre-recorded waypoint sequences but not for high-speed dynamic manoeuvres.

After the last waypoint, the trajectory holds the final position indefinitely.

---

### 3.6 `SequencedTrajectory` — Phase chaining

**`src/trajectory/mod.rs:554`**

Takes a list of `(duration, Box<dyn Trajectory>)` pairs and chains them:

```rust
SequencedTrajectory::new(vec![
    (3.0, Box::new(TakeoffTrajectory::new(0.0, 0.0, 0.25, 3.0, 0.0))),
    (5.0, Box::new(CircleTrajectory::new(0.0, 0.25, 0.0))),   // hover
    (30.0, Box::new(CircleTrajectory::new(0.30, 0.25, PI/8.0))), // circle
])
```

At any time `t`, `active_phase(t)` finds which phase is active and computes the *local* time within that phase (`t − phase_start`). The local time is passed to the phase's own `get_reference`. After the last phase, the last phase is held indefinitely.

This is the standard pattern used in `main.rs` for real flights: takeoff → stabilise → manoeuvre.

---

## 4. Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│  Control loop (bin/assignment*.rs, main.rs)                      │
│                                                                  │
│  let ref = trajectory.get_reference(t)                          │
│  let ctrl = controller.compute_control(state, &ref, &params, dt)│
└────────────────────────────┬─────────────────────────────────────┘
                             │ get_reference(t)
                             ▼
┌──────────────────────────────────────────────────────────────────┐
│  Trajectory trait  (src/trajectory/mod.rs:11)                    │
│                                                                  │
│  ┌─────────────────┐  ┌──────────────────┐  ┌────────────────┐  │
│  │ CircleTrajectory│  │ Figure8Trajectory│  │TakeoffTrajecto.│  │
│  │ analytical      │  │ polynomial coeff │  │ min-jerk poly  │  │
│  └─────────────────┘  └──────────────────┘  └────────────────┘  │
│                                                                  │
│  ┌─────────────────┐  ┌──────────────────┐  ┌────────────────┐  │
│  │ SmoothFigure8   │  │ CsvTrajectory    │  │SequencedTraj.  │  │
│  │ Lissajous       │  │ linear interp    │  │ chains phases  │  │
│  └─────────────────┘  └──────────────────┘  └────────────────┘  │
└────────────────────────────┬─────────────────────────────────────┘
                             │
                             ▼
                  TrajectoryReference
          (pos, vel, acc, jerk, yaw, yaw_rate, yaw_acc)
```

---

## 5. How the Feedforward Derivatives Work

The richer the `TrajectoryReference`, the better the controller tracks:

**Position only** (CsvTrajectory): controller reacts purely on error. Tracking lag proportional to `1/(kp·kv)`.

**Position + velocity** (all analytical): `kv·ev` term provides damping anticipation. Velocity error is small for well-tracked trajectories.

**+ acceleration** (CircleTrajectory, Figure8Trajectory): `F_des = m·(a_ref + kp·ep + kv·ev)` — the centripetal acceleration of a circle is pre-commanded. Without it, the controller would have to "discover" the needed centripetal force through error buildup.

**+ jerk** (all except CsvTrajectory): drives the `h_ω` term in the angular velocity feedforward. Without jerk, the attitude loop only reacts to angular velocity error; with jerk, it anticipates the required rotation rate change. Critical for fast manoeuvres.

---

## 6. Parameters & Tuning

**CircleTrajectory `omega`**: Controls both speed and centripetal acceleration. The centripetal acceleration is `r·ω²` — at `r=0.3 m, ω=PI/8 rad/s` this is only `0.3·(0.39)² ≈ 0.046 m/s²`, well within the controller's capability. Doubling `ω` quadruples the required centripetal force.

**Figure8Trajectory `time_scale`**: Values from `assignments` use `time_scale = 0.5` (half speed) for comfortable indoor flight. `time_scale = 1.0` produces peak speeds around 1 m/s per segment.

**TakeoffTrajectory `duration`**: Minimum-jerk means the peak acceleration is `10·h/T²` at `s = 0.5`. For `h = 0.25 m, T = 3 s`: peak `az ≈ 0.28 m/s²` — very gentle. For `T = 1 s`: peak `az ≈ 2.5 m/s²` — still safe but more aggressive.

**SequencedTrajectory phase durations**: The hover phase (flat `CircleTrajectory` with `ω=0`) duration should be long enough for the controller to settle (typically 2–5 s at the Crazyflie's bandwidth).

---

## 7. Connections to Other Modules

| Direction | Module | What is exchanged |
|-----------|--------|------------------|
| Produces | `controller/` | `TrajectoryReference` per timestep |
| Used by | `bin/assignment*` | Each assignment picks one or more trajectory types |
| Used by | `bin/main.rs` | `SequencedTrajectory` wraps takeoff + manoeuvre phases |
| Replaced by | `planning/` | `SplineTrajectory::eval(t)` also produces `FlatOutput → TrajectoryReference` |

---

## 8. Common Pitfalls

**`Figure8Trajectory` segment time wrapping**: The trajectory wraps via `time % self.duration`. If `duration` is computed incorrectly (e.g., not including `time_scale`), only the first segment ever runs. This was a real bug: `with_time_scale` constructs `duration = base_total * time_scale` specifically to fix this.

**`CsvTrajectory` zero acceleration**: Because CsvTrajectory provides zero acceleration and jerk, the feedforward is absent. For fast waypoint sequences this causes the controller to lag behind — it must build up position error before generating enough force. For slow sequences (< 0.3 m/s) this is unnoticeable.

**`CircleTrajectory` yaw convention**: Yaw is set to `θ + π/2` — pointing tangent to the circle in the direction of travel. If you want the drone to always face the center, set `yaw = θ + π`. If you want a fixed heading, override to `yaw = constant`.

**`SequencedTrajectory` phase transitions**: Phases transition at their configured duration. The active phase's `get_reference` receives *local* time, so a trajectory that itself uses `time % duration` will correctly loop within its phase. A trajectory that doesn't loop (like `TakeoffTrajectory`) simply holds its terminal value after `local_t > duration`.

**Time starting at zero per phase**: Each phase receives `t_local = t_global − phase_start`. Make sure the inner trajectory expects time starting at 0. `TakeoffTrajectory` clamps `s = t / duration` to `[0,1]`, so it handles `t_local > duration` gracefully.

---

## 9. Related Tests

| Test file | What it covers |
|-----------|---------------|
| `src/trajectory/mod.rs` (inline) | `TakeoffTrajectory`: z=0 at start, z=target at end, zero velocity at both; `SequencedTrajectory`: correct phase start times; `Figure8Trajectory`: non-zero spatial extent; `CircleTrajectory`: starts at (r, 0, z); `CsvTrajectory`: linear interpolation |
| `tests/test_hover_control_loop.rs` | `TakeoffTrajectory` + `CircleTrajectory` in closed-loop sim → drone converges to target height and tracks circle |
