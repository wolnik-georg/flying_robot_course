# Debugging Status - Multirotor Simulator

## Current Status Summary

### ✅ What Works Perfectly

1. **Assignment 1 - Integrator Comparison** 
   - All 4 integrators (Euler, RK4, ExpEuler, ExpRK4) produce consistent results
   - Drone climbs steadily to ~2m as expected
   - No numerical instabilities

2. **Assignment 2 - Hover Scenario**
   - Perfect hover at (0, 0, 0.5m)
   - Final position error: 0.000m
   - Final velocity error: 0.000m/s
   - Controller maintains stability

3. **All Unit Tests** (60/60 passing)
   - Integration tests ✓
   - Geometric controller tests ✓
   - Motor mixing tests ✓

### ❌ What Fails

1. **Assignment 2 - Figure-8 Trajectory**
   - Final errors: **inf m** (infinity)
   - System diverges completely
   - Happens during dynamic maneuvers

2. **Assignment 2 - Circle Trajectory**
   - Final position error: **8.38 billion meters**
   - Final velocity error: **854 million m/s**
   - Catastrophic instability

### Key Finding

**The controller is NOT broken!**
- Hover works perfectly (0.000m error)
- Controller produces correct torque signs
- Motor mixing is correct
- Coordinate system verified

**The problem:** Controller cannot handle **trajectory tracking with significant accelerations/velocities**

## Root Cause Analysis

### CORRECTION: Controller Sign is Actually Correct!

After verifying coordinate system with `test_coordinates.rs`:
- **Positive pitch rotation** (+y axis) → nose DOWN → thrust points FORWARD (+x)
- **Negative pitch rotation** (-y axis) → nose UP → thrust points BACKWARD (-x)

For drone at x=0.1m needing to return to x=0:
- Need thrust pointing BACKWARD (-x component) ✓
- Need nose to tilt UP ✓
- Need NEGATIVE pitch torque ✓
- Controller produces τ_y = -0.000102 Nm ✓ **CORRECT!**

The controller IS working correctly! The instability must be from:
1. **Gains too weak** - controller can't respond fast enough
2. **Coupling effects** - position control affects attitude which affects position
3. **Some other bug** we haven't found yet

### Re-investigation Needed

The controller torque sign is correct. Need to investigate why system still goes unstable with position offsets.

**For drone at x=0.1m, hovering at z=0.5m:**

1. **Position Control** ✅
   ```
   Position error ep = (-0.1, 0, 0)  [need to go -0.1m in x]
   Feedforward acc = (-0.01, 0, 9.81) m/s²
   Thrust force = (-0.00027, 0, 0.265) N
   ```
   - Position control produces correct thrust force
   - Small -x component to pull drone back
   - Mostly +z to counteract gravity

2. **Desired Rotation** ✅
   ```
   Desired z-axis (thrust direction): (-0.001019, 0, 0.999999)
   Desired rotation matrix Rd:
     [1.00000,  0.00000, -0.00102]
     [0.00000,  1.00000,  0.00000]
     [0.00102,  0.00000,  1.00000]
   ```
   - Thrust direction is correct (slightly backward and up)
   - Rotation matrix correctly represents desired orientation

3. **Current Rotation** ✅
   ```
   Current rotation R = Identity (drone level)
   ```

4. **Rotation Error Calculation** ⚠️
   ```
   Rd^T*R - R^T*Rd = 
     [0.00000,  0.00000,  0.00204]
     [0.00000,  0.00000,  0.00000]
     [-0.00204, 0.00000,  0.00000]
   
   eR = vee(Rd^T*R - R^T*Rd)/2 = (0, 0.002039, 0)
   ```
   - Rotation error is POSITIVE in y-axis
   - Means: need to rotate positively about y (pitch nose UP)

5. **Torque Calculation** ❌
   ```
   Controller law: τ = -KR * eR
   Proportional torque = -0.05 * (0, 0.002039, 0) = (0, -0.000102, 0) Nm
   ```
   - **NEGATIVE pitch torque produced**
   - This would pitch nose DOWN (wrong direction!)
   - Should be POSITIVE to pitch nose UP

6. **Motor Response** ❌
   ```
   From debug_rotation.rs:
   Torque: (0, -0.000102, 0) Nm
   → Front motors faster than back motors
   → Nose pitches DOWN instead of UP
   → Makes position error WORSE
   ```

### The Sign Error

**What we need:**
- Drone at x=0.1 → need thrust with -x component
- To get thrust pointing backward → need to tilt nose UP
- To tilt nose up → need POSITIVE pitch torque (back motors faster)

**What controller does:**
- Computes eR = (0, +0.002039, 0) ✓ Correct
- Applies τ = -KR * eR = (0, -0.000102, 0) ✗ WRONG SIGN!
- Produces negative pitch torque → nose pitches DOWN
- This tilts thrust FORWARD, making error worse!

## The Bug

**Location:** `src/controller/mod.rs`, lines 200-203

```rust
let torque_proportional = Vec3::new(
    -self.kr.x * er.x,   // ← SIGN ERROR HERE
    -self.kr.y * er.y,   // ← SIGN ERROR HERE
    -self.kr.z * er.z,   // ← SIGN ERROR HERE
);
```

**Issue:** The control law uses `τ = -KR * eR`, but based on our testing:
- When eR > 0 (need to rotate positive), we get τ < 0 (wrong direction)
- This causes the controller to fight against itself

**Hypothesis:** Either:
1. The sign in the control law is wrong (should be `τ = +KR * eR`), OR
2. The rotation error definition is wrong (sign flipped)

## Next Steps

1. **Check Lee et al. (2010) paper** - verify the exact control law formula
2. **Test sign flip** - change to `τ = +KR * eR` and verify behavior
3. **Alternative**: Check if rotation error vee operator has wrong sign convention
4. **Verify fix** - run all scenarios after fix:
   - Assignment 1 (should still work)
   - Assignment 2 hover
   - Assignment 2 figure-8
   - Assignment 2 circle

## Bugs Fixed So Far

1. ✅ **RK4 quaternion integration** (`src/integration/rk4.rs:22`) - was using multiplication instead of addition
2. ✅ **Controller feedforward** (`src/controller/mod.rs:214`) - was subtracting angular velocity from acceleration  
3. ✅ **Motor mixing lever arm** (`src/dynamics/state.rs:111-130`) - now uses l/√2 for X-configuration
4. ✅ **Torque formulas** (`src/dynamics/params.rs:77-82`) - now use all 4 motors (was only using 2)
5. ✅ **Trajectory derivatives** (`src/trajectory/mod.rs:119-127`) - now use segment_duration (was using total duration)

## Remaining Issue: Trajectory Tracking Instability

**Symptom:** Hover works, but any trajectory with motion causes divergence

**Possible Causes:**
1. **Controller gains too weak** - Cannot respond to trajectory errors
2. **Feedforward term missing/wrong** - Not accounting for desired accelerations properly
3. **Coupling bug** - Position control interfering with attitude control  
4. **Numerical issue** - Small errors compound during motion

## Next Steps

1. **Check controller gains** - Current gains may be too conservative
2. **Examine feedforward path** - Verify desired acceleration is properly used
3. **Add more detailed logging** - Track errors during figure-8 to see what diverges first
4. **Test with simpler trajectories** - Try slow circle or straight line motion
