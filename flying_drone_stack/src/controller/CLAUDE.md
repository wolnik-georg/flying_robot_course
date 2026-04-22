# controller/ — SE(3) Geometric Controller

## What this module does

Implements the Lee et al. (2010) geometric tracking controller on SE(3).
Used in simulation (assignment binaries), offline shadow track (`main.rs`),
and onboard firmware (`firmware_app/src/lib.rs` — separate copy, same math).

## Key types

```rust
pub struct TrajectoryReference {
    pub position: Vec3,        // [m]
    pub velocity: Vec3,        // [m/s]
    pub acceleration: Vec3,    // [m/s²]
    pub jerk: Vec3,            // [m/s³]
    pub yaw: f32,              // [rad]
    pub yaw_rate: f32,         // [rad/s]
    pub yaw_acceleration: f32, // [rad/s²]
}

pub struct ControlOutput {
    pub thrust: f32,   // [N] total thrust
    pub torque: Vec3,  // [Nm] body-frame torques
}

pub trait Controller {
    fn compute_control(
        &mut self,
        state: &MultirotorState,
        reference: &TrajectoryReference,
        params: &MultirotorParams,
        dt: f32,
    ) -> ControlOutput;
}
```

## GeometricController

```rust
pub struct GeometricController {
    pub kp: Vec3,       // position proportional [m/s²/m]
    pub kv: Vec3,       // position derivative [m/s²/(m/s)]
    pub ki_pos: Vec3,   // position integral (anti-windup)
    pub kr: Vec3,       // attitude proportional [Nm/rad]
    pub kw: Vec3,       // attitude derivative [Nm/(rad/s)]
    pub ki: Vec3,       // attitude integral (usually zero)
}

impl GeometricController {
    pub fn default() -> Self      // Crazyflie-tuned gains (see table below)
    pub fn reset(&mut self)       // clear integral accumulators
}
```

## Default gains (offboard simulation/shadow track)

| Gain | Value | Notes |
|------|-------|-------|
| `kp` | (12.0, 12.0, 7.0) | XY position P |
| `kv` | (8.0, 8.0, 4.0) | XY position D |
| `ki_pos` | (0.05, 0.05, 0.05) | position I |
| `kr` | (0.007, 0.007, 0.008) | attitude P |
| `kw` | (0.00115, 0.00115, 0.002) | attitude D |
| `ki` | (0.0, 0.0, 0.0) | attitude I (disabled) |

**Firmware gains are different** — see `firmware_app/CLAUDE.md` for the actual onboard values
(KP_X/Y=7.5, KP_Z=26.0, KV_X/Y=9.0, KV_Z=14.0).

## Control law

```
ep = pd - p          (position error)
ev = vd - v          (velocity error)
eR = ½ vee(Rd^T R - R^T Rd)   (rotation error, SO(3))
eΩ = Ω - R^T Rd Ωd  (angular velocity error)

F_des = m(a_d + Kp·ep + Kv·ev + Ki·∫ep + g·e3)
thrust = F_des · b3  (project onto body z)
Rd = desired_rotation(F_des, yaw_d)
τ = -KR·eR - KW·eΩ + Ω×(J·Ω)   (gyroscopic compensation)
```

## Usage in simulation

```rust
let mut ctrl = GeometricController::default();
let params = MultirotorParams::crazyflie();
let mut sim = MultirotorSimulator::new(params.clone(), Box::new(RK4Integrator));

let reference = TrajectoryReference {
    position: Vec3::new(0.0, 0.0, 0.5),
    velocity: Vec3::zero(),
    acceleration: Vec3::zero(),
    jerk: Vec3::zero(),
    yaw: 0.0, yaw_rate: 0.0, yaw_acceleration: 0.0,
};

let output = ctrl.compute_control(sim.state(), &reference, &params, dt);
let action = MotorAction::from_thrust_torque(output.thrust, output.torque, &params);
sim.step(&action);
```

## Usage in main.rs shadow track

The shadow controller runs every loop but never touches the drone:
```rust
let shadow_output = shadow_ctrl.compute_control(&mekf_state_ms, &trajectory_ref, &params, dt);
// → logged to CSV as our_thrust, our_roll_cmd, our_pitch_cmd, our_yaw_rate_cmd
// → motors NOT affected
```

## Relationship to firmware_app

`firmware_app/src/lib.rs` is an **independent reimplementation** of the same SE(3) math
in `no_std` Rust. The gains and algorithm are the same; the code is separate because the
firmware crate cannot use std. If you update gains in one place, manually update the other.
