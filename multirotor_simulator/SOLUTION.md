# Solution: Controller Gain Issue

## Problem
Assignment 2 trajectories (figure-8 and circle) were diverging with massive position errors (>4m), while Assignment 1 and hover scenarios worked perfectly.

## Root Cause
**Controller gains were too weak for trajectory tracking.**

The simulator was using arbitrarily chosen "weak" gains:
- Position: Kp = 1.0, Kv = 0.5
- Attitude: KR = 0.5, Kω = 0.1

## Solution
**Use official Crazyflie Lee controller gains from the actual Crazyflie firmware.**

Our implementation uses the Lee et al. (2010) geometric controller on SE(3), which is EXACTLY the same as the Lee controller in the official Crazyflie firmware (`controller_lee.c` in bitcraze/crazyflie-firmware).

### Official Gains (from Crazyflie firmware)
```rust
Position P gains:  Kp = Vec3::new(7.0, 7.0, 7.0)     // Kpos_P
Velocity D gains:  Kv = Vec3::new(4.0, 4.0, 4.0)     // Kpos_D  
Attitude P gains:  KR = Vec3::new(0.007, 0.007, 0.008)   // KR
Angular vel D:     Kω = Vec3::new(0.00115, 0.00115, 0.002) // Komega
```

Source: [controller_lee.c lines 52-63](https://github.com/bitcraze/crazyflie-firmware/blob/main/src/modules/src/controller/controller_lee.c#L52-L63)

### Key Insight
The attitude gains needed to be **70x weaker** than we initially thought!
- Old KR: 0.5 → New KR: 0.007
- Old Kω: 0.1 → New Kω: 0.00115

This makes sense because:
1. Crazyflie has **extremely low inertia** (1.7e-5 kg⋅m²)
2. Small torques create large angular accelerations: α = τ/J
3. Too-strong gains cause oscillations that grow exponentially

## Results

### Before (wrong gains):
- Hover: ✅ 0.000 m error (worked because no attitude changes)
- Figure-8: ❌ 4.626 m error (massive divergence)
- Circle: ❌ 4.188 m error (divergence)

### After (official gains):
- Hover: ✅ 0.000 m error (perfect)
- Figure-8: ✅ Max 0.437 m, Avg 0.195 m (11x better!)
- Circle: ✅ Max 0.012 m, Avg 0.005 m (350x better!)

## Files Changed
1. `src/controller/mod.rs`: Updated default() with official gains
2. `src/bin/assignment2.rs`: Updated hardcoded gains to match
3. Tests: Updated gain verification test

## How We Found This
1. Systematic debugging revealed angular velocity oscillation
2. Checked Crazyflie firmware repository
3. Found Lee controller with identical SE(3) implementation
4. Used their tested/proven gains
5. **Instant success!**

## Verification
All 60 unit tests passing ✅

## Conclusion
**Always check official implementations for reference parameters!** The Crazyflie has been flying for years with thousands of units deployed - their gains are battle-tested.
