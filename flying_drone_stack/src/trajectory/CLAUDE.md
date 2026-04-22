# trajectory/ — Trajectory Generators

## What this module does

Provides the `Trajectory` trait and several concrete trajectory implementations.
Used by the geometric controller (simulation/shadow track) and by assignment binaries.

For **min-snap spline** trajectories, see `src/planning/` instead.

## Trait

```rust
pub trait Trajectory {
    fn get_reference(&self, time: f32) -> TrajectoryReference;
    fn duration(&self) -> Option<f32> { None }  // None = infinite
}
```

## Implementations

### CircleTrajectory

```rust
pub struct CircleTrajectory {
    pub center: Vec3,    // circle center [m]
    pub radius: f32,     // [m]
    pub omega: f32,      // angular speed [rad/s]
    pub height: f32,     // [m]
    pub yaw_tracking: bool,  // if true, yaw follows tangent
}

impl CircleTrajectory {
    pub fn new(center: Vec3, radius: f32, omega: f32, height: f32) -> Self
}
// Outputs smooth analytical position, velocity, acceleration, jerk, yaw
```

### Figure8Trajectory (piecewise polynomial)

```rust
pub struct Figure8Trajectory {
    pub duration: f32,   // [s] full loop
    pub height: f32,     // [m]
    pub scale: f32,      // spatial scale
    pub time_scale: f32, // temporal scale (>1 = slower)
}
// Uses professor's pre-computed degree-7 Poly4D coefficients
// ±1.0m x, ±0.5m y at scale=1.0; total ~7.28 s
```

### SmoothFigure8Trajectory (Lissajous, C^∞)

```rust
pub struct SmoothFigure8Trajectory {
    pub duration: f32,
    pub height: f32,
    pub a: f32,          // x amplitude
    pub b: f32,          // y amplitude
    pub time_scale: f32,
}
// Analytical Lissajous curve: x=a·sin(2ωt), y=b·sin(ωt)
// Infinitely differentiable — better for aggressive maneuvers
```

### CsvTrajectory

```rust
pub struct CsvTrajectory { /* loaded from file */ }
impl CsvTrajectory {
    pub fn from_file(path: &str) -> Result<Self, String>
    // CSV columns: time [s], x, y, z [m], yaw [rad]
    // Interpolates linearly between rows
}
```

### SequencedTrajectory

```rust
pub struct SequencedTrajectory { /* Vec of (duration, Box<dyn Trajectory>) */ }
impl SequencedTrajectory {
    pub fn new() -> Self
    pub fn add(mut self, duration: f32, traj: Box<dyn Trajectory>) -> Self
}
// Chains trajectories sequentially; switches at duration boundaries
```

### TakeoffTrajectory

```rust
pub struct TakeoffTrajectory {
    pub target_height: f32,  // [m]
    pub duration: f32,       // [s]
}
// Smooth polynomial ramp from 0 to target_height
// Returns zero velocity/acceleration at end
```

## Usage

```rust
// Circle
let traj = CircleTrajectory::new(Vec3::zero(), 0.25, 0.5, 0.5);
let ref_ = traj.get_reference(t);  // t [s]

// Figure-8 with slower speed
let traj = Figure8Trajectory {
    duration: 7.28, height: 0.5, scale: 1.0, time_scale: 2.0, // 2× slower
};

// Sequence: takeoff → circle → land
let traj = SequencedTrajectory::new()
    .add(3.0, Box::new(TakeoffTrajectory { target_height: 0.5, duration: 3.0 }))
    .add(30.0, Box::new(CircleTrajectory::new(Vec3::zero(), 0.25, 0.5, 0.5)));

// In control loop
let ref_ = traj.get_reference(t);
let output = ctrl.compute_control(state, &ref_, &params, dt);
```

## Difference from SplineTrajectory (planning/)

| | trajectory/ implementations | planning/SplineTrajectory |
|---|---|---|
| Definition | Analytical or polynomial | QP-optimized min-snap |
| Smoothness | Varies (C^∞ for Lissajous, C3 for poly) | C3 (degree 8) |
| Derivatives | Analytical | Numerical from spline eval |
| Output | `TrajectoryReference` | `FlatOutput` (then → `FlatnessResult`) |
| Use case | Simulation, shadow track | Full-state CRTP, HLC export |
