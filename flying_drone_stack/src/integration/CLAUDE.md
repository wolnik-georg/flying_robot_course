# integration/ — Numerical Integrators

## What this module does

Four numerical integrators for the multirotor dynamics ODE. All implement the `Integrator` trait.

## Trait

```rust
pub trait Integrator {
    fn step(
        &self,
        params: &MultirotorParams,
        state: &mut MultirotorState,
        action: &MotorAction,
    );
}
```

## Available integrators

| Type | Order | Notes |
|------|-------|-------|
| `EulerIntegrator` | O(dt) | Simplest; largest error at coarse dt |
| `RK4Integrator` | O(dt⁴) | Standard; recommended for simulation |
| `ExpEulerIntegrator` | O(dt) | Quaternion propagated via exponential map; better attitude accuracy than Euler |
| `ExpRK4Integrator` | O(dt⁴) | RK4 + exponential map; best overall |

## Usage

```rust
// Construct
let integrator: Box<dyn Integrator> = Box::new(RK4Integrator);
let mut sim = MultirotorSimulator::new(params, integrator);

// Or choose at runtime:
fn make_integrator(name: &str) -> Box<dyn Integrator> {
    match name {
        "euler"    => Box::new(EulerIntegrator),
        "rk4"      => Box::new(RK4Integrator),
        "exp_euler"=> Box::new(ExpEulerIntegrator),
        "exp_rk4"  => Box::new(ExpRK4Integrator),
        _ => panic!("unknown integrator"),
    }
}
```

## Performance comparison (Assignment 1 results)

**Synthetic spin test** (constant torque, dt=10ms, t=5s):
| Integrator | Position error |
|-----------|---------------|
| Euler | 68.0 mm |
| RK4 | 0.03 mm (~2000× better) |
| ExpEuler | ~65 mm |
| ExpRK4 | ~0.03 mm |

**Real-flight k-step prediction** (20 Hz CSV, k=1..10 horizon):
| Integrator | Error at k=1 | Error at k=10 |
|-----------|-------------|--------------|
| All four | 13.7 mm | 63.2 mm |
| Spread between integrators | < 1 mm | < 1 mm |

Key insight: on real data at 20 Hz, action reconstruction uncertainty (IMU noise) dominates
over integration order. Integrator choice matters for long simulations with exact inputs, not
for short-horizon prediction from noisy sensor data.

## Exponential integrators

The `Exp*` variants propagate the quaternion using the axis-angle exponential map:
```
q(t+dt) = q(t) ⊗ exp(ω·dt/2)
```
This keeps the quaternion on the unit sphere exactly, avoiding the drift that occurs with
naive Euler addition of the quaternion derivative. For position and velocity, the same
first-order or RK4 scheme is used as in the non-exponential variants.

## Gotchas

- All integrators use `params.dt` from `MultirotorParams` as the step size.
- `RK4Integrator` evaluates `compute_derivatives` four times per step (k1, k2, k3, k4).
- The quaternion is automatically renormalized after each step to prevent numerical drift.
