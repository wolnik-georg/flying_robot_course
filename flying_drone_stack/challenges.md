# Flight Control Challenges — Solutions & Learnings

A concise record of the real bugs we hit, what caused them, and how they were fixed.
Grouped by theme, not date.

---

## 1. HOVER_PWM too low → drone falls immediately on RPYT handoff

**What happened:** After switching from `setpoint_hover` to our RPYT control loop, the drone
immediately descended and landed instead of hovering.

**Root cause:** `HOVER_PWM = 42000` was too low for the actual battery charge on the day.
The thrust formula is `pwm = (desired_thrust / hover_thrust_N) * HOVER_PWM`. If `HOVER_PWM`
is below the true hover PWM, the controller can only ever command less thrust than gravity
requires. The integral compensates slowly but never overcomes the deficit.

**Fix:** Raised `HOVER_PWM = 50000`. Calibration method: run `my_hover`, read the `thr_pwm`
value in the logs once `ep.z ≈ 0` (drone stable). That value is the real hover PWM.
Copy it to `HOVER_PWM` and recompile.

**Rule:** If drone falls → `HOVER_PWM` too low. If drone rises → too high.

---

## 2. Stale log buffer causes max-thrust burst at takeoff

**What happened:** On the first flight after a 2–3 second connect delay, the RPYT loop
consumed stale packets at 2000 Hz and commanded maximum thrust before the drone was
ready. The drone shot up and crashed (flight 2026-03-10).

**Root cause:** Log streams start buffering from the moment `cf.log.create_block()` is
called — roughly 3.5 seconds before the control loop. At 100 Hz that is ~350 stale
packets per stream. With only 30 drain iterations we cleared ~30 packets; the remaining
320 were fed into the control loop instantly, producing wild sensor values and hence
wild thrust commands.

**Fix:** Drain 500 iterations before starting RPYT. At 10 ms per iteration that covers
5 seconds of buffered data and guarantees the control loop starts with real-time packets.

```rust
for _ in 0..500 {
    run_logging_step(...).await;
}
```

**Rule:** Drain iterations must cover at least the full time between connect and control
start. When in doubt, drain more.

---

## 3. RPYT pitch sign is inverted by the Crazyflie firmware

**What happened:** Any position error in X caused the drone to pitch away from the target
instead of toward it — positive feedback that diverged immediately.

**Root cause:** The Crazyflie RPYT commander negates pitch internally before converting to
a quaternion (`rpy2quat(roll, -pitch, yaw)`). Sending a positive pitch command actually
produces a negative (nose-up) attitude, which tilts thrust away from +X instead of toward it.

**Fix:** The controller's `pitch_d_raw` already has the correct sign for forward motion.
Send it un-negated — the firmware's own negation then produces the correct nose-down attitude.

```
ep.x > 0  →  pitch_d_raw > 0  →  firmware negates  →  actual nose-down  →  thrust toward +X  ✓
```

Roll does NOT have this inversion. Only pitch is negated in the RPYT path.

---

## 4. Yaw rate sign convention is opposite on the Crazyflie

**What happened:** The drone yawed away from the target heading instead of correcting toward it.

**Root cause:** `yaw_rate_cmd` returns `(ref_yaw − current_yaw) * kp`, which is positive
when the drone needs to turn CCW. But the Crazyflie RPYT protocol defines positive
`yaw_rate` as **clockwise from above** = decreasing EKF yaw. The signs are opposite.

**Fix:** Negate the output of `yaw_rate_cmd` before sending:

```rust
let yaw_rate_d = -yaw_rate_cmd(ref_yaw, current_yaw_deg, kp_yaw, max_deg_s);
```

---

## 5. EKF height is unreliable without Lighthouse — use range.zrange instead

**What happened:** When Lighthouse had not yet initialised, `stateEstimate.z` reported
~0 m (floor), even while the drone was physically at 0.3 m. The controller saw
`ep.z < 0` (drone "above" reference), wound the Z integral negative, and cut thrust
until the drone fell.

**Fix:** Use `range.zrange` (the ToF Z-ranger on the Flow Deck) for height feedback.
It is absolute, drift-free, and works without Lighthouse from the first tick.
Divide by 1000 because the firmware logs it in millimetres as a `uint16`.

```rust
entry.range_z = get_f32(d, "range.zrange") / 1000.0;
```

---

## 6. get_f32 silently returned 0.0 for integer log variables

**What happened:** `range.zrange` and `stabilizer.thrust` both logged as zero even though
the sensor was clearly working in cfclient.

**Root cause:** The helper used `f32::try_from(Value)` which only matches `Value::F32`.
Integer-typed variables like `range.zrange` (logged as `Value::U16`) never matched,
so the function silently returned `0.0` without any error.

**Fix:** Use `to_f64_lossy()` which handles every `Value` variant:

```rust
fn get_f32(map: &HashMap<String, Value>, key: &str) -> f32 {
    map.get(key).map(|v| v.to_f64_lossy() as f32).unwrap_or(0.0)
}
```

**Rule:** When a log variable reads as zero but the sensor clearly works, check the
Crazyflie firmware log type table — it may be integer-typed, not float.

---

## 7. XY position control without Lighthouse — velocity integration + soft rubber-band

**What happened:** Without Lighthouse, `stateEstimate.x/y` is always near zero (only
accelerometer dead-reckoning). Using it for position control produced large, noisy
corrections and the drone drifted.

**Fix:** Self-integrate EKF velocity to estimate XY displacement from the start position:

```rust
if ever_airborne && vel_xy < 0.2 {   // gate: freeze during fast drift to avoid runaway
    est_pos_x += vel_x * 0.01;       // 10 ms tick
    est_pos_y += vel_y * 0.01;
    est_pos_x = est_pos_x.clamp(-0.5, 0.5);
}
```

Feed a soft restoring force using a fraction of `kp` (effective `kp_xy = 0.5`) so
the drone drifts gently back to the start position without aggressive corrections.
Clamp to ±0.5 m so a velocity bias can't build an unbounded integral.

---

## 8. Position integral windup during saturation

**What happened:** When tilt or thrust hit its limit, the controller could not execute
the correction but the position integral kept growing. When saturation ended, the
accumulated integral caused an overcorrection spike.

**Fix:** Save the integral before each `compute_control` call. Revert it if tilt
or thrust saturated on that tick:

```rust
let i_pos_prev = controller.i_error_pos();
let control = controller.compute_control(...);

if tilt_saturated(...) || thrust_saturated(...) {
    controller.set_i_error_pos(i_pos_prev);
}
```

---

## 9. RPYT commander ignores non-zero thrust until it sees thrust=0

**What happened:** On the very first RPYT command, the drone did not respond to the
commanded thrust — it sat motionless on the ground.

**Root cause:** The Crazyflie RPYT commander requires a `thrust=0` command before it
will accept any non-zero thrust. This is a safety interlock in the firmware.

**Fix:** Send one explicit zero-thrust unlock before the control loop:

```rust
cf.commander.setpoint_rpyt(0.0, 0.0, 0.0, 0u16).await?;
// then start the loop
```

---

## Summary: what made my_hover reliable

| Fix | Impact |
|-----|--------|
| HOVER_PWM = 50000 | Drone no longer descends at RPYT handoff |
| 500-iter buffer drain | No max-thrust burst at startup |
| Pitch sign un-negated | Stable attitude control in X |
| Yaw rate negated | Correct heading hold |
| range.zrange for Z | Height works without Lighthouse |
| get_f32 via to_f64_lossy | range.zrange reads correctly |
| Velocity integration + soft kp | Lateral stability without Lighthouse |
| Anti-windup | No post-saturation overshoot |
| RPYT thrust=0 unlock | Motors actually respond on first tick |
