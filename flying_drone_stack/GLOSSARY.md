# Glossary

Short precise definitions of technical terms used throughout this project.
Add new terms here whenever a non-obvious keyword is introduced.

---

## Derivatives of position

**Position**
Where something is in space. Units: metres [m]. The zeroth derivative — everything else is derived from it.

**Velocity**
Rate of change of position. First derivative of position with respect to time. Units: m/s.

**Acceleration**
Rate of change of velocity. Second derivative of position. Units: m/s². Directly linked to the forces acting on the drone (F = ma).

**Jerk**
Rate of change of acceleration. Third derivative of position. Units: m/s³. High jerk means the acceleration is changing rapidly — relevant because the drone's attitude (tilt) must change to produce changes in acceleration, so jerk determines how fast the drone needs to rotate.

**Snap**
Rate of change of jerk. Fourth derivative of position. Units: m/s⁴. The quantity minimised in min-snap trajectory optimisation. High snap requires rapid changes in angular velocity.

**Min-snap**
The optimisation objective used by the spline planner: find a polynomial trajectory that minimises the integral of snap² over the entire path, subject to passing through all waypoints. Produces smooth, dynamically efficient trajectories. Not a derivative itself — it names the criterion.

---

## Planning terms

**Waypoint**
A prescribed position (and optionally yaw) that the trajectory must pass through exactly. The planner fits a polynomial between consecutive waypoints.

**Segment**
One polynomial piece of a piecewise trajectory, connecting two consecutive waypoints. Each segment has its own set of coefficients and a fixed duration.

**Duration**
The time allocated to one segment [s]. In Mode 0 (Spline) durations are specified manually. In Mode 1 (Richter) they are computed automatically from the inter-waypoint distances and the aggressiveness parameter k_t.

**Periodic trajectory**
A trajectory whose end connects smoothly back to its start — not just the same position, but also matching velocity, acceleration, jerk, and snap at the junction. Used for closed loops (circle, figure-8) so the drone can repeat laps indefinitely without a jolt.

**Flat output**
The set of quantities — position (x, y, z) and yaw — from which the full drone state (attitude, angular velocity, thrust, torques) can be computed analytically without integrating any differential equations. Multirotors are differentially flat systems, which is what makes polynomial trajectory planning practical.

**Differential flatness**
The property of a dynamical system that allows all states and inputs to be expressed as algebraic functions of a small set of outputs (the flat outputs) and their derivatives. For a multirotor: given position + yaw up to their 4th derivatives, the full rigid-body state is uniquely determined.

**Feasibility**
Whether a planned trajectory can physically be executed by the drone — i.e. whether the required thrust and angular rates stay within the hardware limits (e.g. max thrust ≤ 0.55 N, max |ω| ≤ 12 rad/s). A trajectory can be geometrically valid but dynamically infeasible if it demands too much acceleration.

---

## Angular motion

**Angular rate**
Common shorthand for angular velocity. In this project it usually refers to the same quantity as \(\omega\), i.e. orientation change per second. Units: typically rad/s in planning/control math, sometimes deg/s in logs and plots (check axis labels).

**Angular velocity (ω)**
Rate of change of the drone's orientation. Units: rad/s. A vector pointing along the axis of rotation; its magnitude is the rotation speed. In the body frame it is what the gyroscope measures.

**Angular acceleration (α)**
Rate of change of angular velocity. Units: rad/s². Produced by net torques acting on the drone (τ = J α, where J is the inertia matrix).

**Torque (τ)**
A rotational force. Units: N·m. The motors produce differential thrust which creates torques around the roll, pitch, and yaw axes, causing the drone to rotate.

**Thrust**
The total upward force produced by all four motors combined. Units: N. At hover, thrust equals weight (mg). During aggressive manoeuvres thrust can briefly exceed mg (to accelerate upward) or drop below (to accelerate downward). In flip/loop trajectories the reference thrust passes through zero — a physical limit the drone cannot cross (propellers cannot push downward).

---

## Attitude representation

**Roll, Pitch, Yaw**
Three Euler angles describing orientation. Roll: rotation around the forward (x) axis. Pitch: rotation around the lateral (y) axis. Yaw: rotation around the vertical (z) axis. Intuitive but suffer from gimbal lock near ±90° pitch.

**Rotation matrix**
A 3×3 orthogonal matrix (R^T R = I, det = +1) representing a rigid rotation. In this codebase stored as `[[f32; 3]; 3]`. Column vectors are the body x/y/z axes expressed in the world frame. No singularities, but 9 numbers for 3 degrees of freedom.

**Quaternion**
A 4-number representation of orientation (w, x, y, z) with no singularities and compact storage. Convention in this codebase: scalar w first, `[w, x, y, z]`. Must satisfy w² + x² + y² + z² = 1. The firmware uses scalar-last convention (w is the 4th component).

**Body frame**
A coordinate system fixed to the drone and rotating with it. Angular velocity ω and gyroscope readings are expressed in the body frame. The z-axis of the body frame points upward out of the drone (along the thrust direction).

**World frame**
A fixed, inertial coordinate system. Position, velocity, and acceleration are expressed in the world frame. z points upward.
