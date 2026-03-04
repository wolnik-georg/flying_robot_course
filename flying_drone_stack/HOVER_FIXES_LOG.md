# my_hover Controller — Fix History & Current Status

## Architecture reminder
`my_hover` uses a **geometric position controller** (Lee et al.) in **RPYT mode**:
- Outer loop (our code): computes desired roll/pitch angles + thrust PWM from position error
- Inner loop (CF firmware): attitude rate PIDs + motor mixing
- Command: `setpoint_rpyt(roll_deg, pitch_deg, yaw_rate_deg_s, thrust_u16)`
- `thrust_pwm = (control.thrust / hover_thrust_n) * HOVER_PWM`

---

## Fix history (oldest → newest)

### ✅ F1 — Axis-aware EKF reset detection (commit `a706d0e`)
**Problem:** Lighthouse re-initialisation fires a combined EKF jump in position + yaw.
Old code reset ALL position integrals (X, Y, Z) on any reset, discarding the Z
steady-state correction and causing altitude collapse.

**Fix:** Separate detection for XY jump, Z jump, and yaw jump with independent
re-anchoring. Z integral preserved when only XY resets.

**Status:** ✅ In source. Was NOT in the binary for the 2026-03-04 18:46 flight
(old binary was used — log format mismatch proved this).

---

### ✅ F2 — `run_logging_step` signature fix (this session)
**Problem:** `run_logging_step` was updated to read 3 log streams (stream3 = thrust/vbat/acc),
but most call sites (stabilize loop, `my_hover`, `hover`, `circle`, `figure8`,
`my_figure8`, fallback `_`) still passed only 5 args (missing `&stream3`).
This caused a **compile error** — the old binary was being used for every flight.

**Fix:** Updated all call sites to pass `&stream3`. Also fixed stabilize loop
which was creating `Instant::now()` each call (so `elapsed()` was always ~0).

**Status:** ✅ Fixed. Binary now compiles fresh.

---

### ✅ F3 — Z reference target: don't anchor to EKF z at hand-off (this session)
**Problem (root cause of 2026-03-04 flight):**
- During the `setpoint_hover` stabilize phase, **Lighthouse had not yet initialised**.
- EKF z reported `~0.007 m` (floor), even though the drone was physically at ~0.3 m.
  (Evidence: `thr=0` in all stabilize log lines — hover command wasn't being accepted,
  the drone was on the ground the entire stabilize phase.)
- We anchored `hover_ref_rpyt.position.z = anchor.pos_z = 0.007 m`.
- When RPYT started, the drone rose from floor → it was **above** the 0.007m reference
  → `ep.z < 0` → `i_pos.z` integrated negative → thrust dropped 42000→31500 → drone fell.

**Fix:** Set `hover_ref_rpyt.position.z = TARGET_HEIGHT = 0.3` always, regardless of
what EKF z reports at hand-off. The drone IS physically at ~0.3m (setpoint_hover put it
there), so targeting 0.3m is correct even if EKF z is wrong.

**Status:** ✅ Fixed.

---

### ✅ F4 — Re-anchor Z on EKF XY reset (this session)
**Problem:** When Lighthouse comes online for the first time mid-flight, it fires a large
XY jump reset (e.g. 1.49m). At that moment:
- The old XY-only re-anchor preserved the Z reference (still 0.3m from F3 fix).
- But EKF z also just became trustworthy for the first time.
- If the drone is physically at a different height than 0.3m, ep.z error begins.
- More importantly: the old code preserved the Z integral across the XY reset. If the
  integral had wound up from the pre-Lighthouse phase, it would cause a thrust spike.

**Fix:** On any XY reset, also re-anchor Z to current EKF z AND reset the full
position integral (all axes). Rationale: a Lighthouse-triggered XY reset means the
*entire* position estimate just re-initialised; Z is now trustworthy for the first time.

**Status:** ✅ Fixed.

---

## Current expected flow (post all fixes)

```
1. Drone on floor, Lighthouse not yet visible
2. Kalman reset, ramp up to 0.3m via setpoint_hover
3. Stabilize 3s at 0.3m — EKF z may still be wrong (Lighthouse not ready)
4. Drain log buffer 60 cycles
5. RPYT hand-off:
   - XY anchored to EKF xy (fine: XY from IMU dead-reckoning is ok short-term)
   - Z = 0.3m (TARGET_HEIGHT, not EKF z which may be bogus)
   - Yaw anchored to EKF yaw
   - Integral = 0
6. Control loop runs — ep.z = 0.3 - actual_z
   - If drone at 0.3m: ep.z ≈ 0 → stable
   - If drone at 0.29m: ep.z = +0.01 → small upward correction → correct
7. Lighthouse comes online → EKF XY reset fires (large XY jump)
   - Re-anchor XY to new EKF position
   - Re-anchor Z to new (now trustworthy) EKF z
   - Reset full position integral
   - ep recalculated → should be near zero if drone near 0.3m
8. Continue stable hover
```

---

## Remaining known issues / next steps

### ⚠️ HOVER_PWM may need re-tuning
The last *successful* hover was `thr_pwm ≈ 42000` but that was from a flight where
the EKF was working properly. Current value `HOVER_PWM = 42000`. If the next flight
shows ep.z settling positive (drone below 0.3m), increase HOVER_PWM. If negative, lower it.

### ⚠️ setpoint_hover may not work without Lighthouse
The stabilize phase uses `setpoint_hover` which requires the CF's hover controller
(uses flow deck / baro). If the drone sits on the floor during stabilize, the RPYT
hand-off won't happen at 0.3m. **Mitigation by F3**: Z is always targeted at 0.3m
regardless, so even if the drone is on the floor at hand-off, the controller will
drive it up. The risk is a sudden large positive ep.z at the start → fast climb.
**Future improvement**: Add a `while anchor.pos_z < 0.2` wait loop after the drain
that keeps sending `setpoint_hover` until EKF z is believable.

### ⚠️ ki_pos.z = 0.05 may be too weak to correct HOVER_PWM offset quickly
If HOVER_PWM is off by >5%, the integral takes 10+ seconds to correct the Z offset.
Consider either better HOVER_PWM calibration or increasing ki_pos.z to 0.1.

### ℹ️ my_circle: stream3 now required
`my_circle` already used 6-arg `run_logging_step` (which caused compile failures before F2).
Now consistent.

---

## Signal interpretation guide

```
ep=(+0.000,+0.000,+X)   → drone is BELOW reference by X meters (ep.z positive = need more thrust)
ep=(+0.000,+0.000,-X)   → drone is ABOVE reference by X meters (ep.z negative = need less thrust)
i_pos.z growing negative → integral pulling thrust DOWN (indicates drone was above ref)
i_pos.z growing positive → integral pulling thrust UP (indicates drone was below ref)
thr_pwm << 42000        → controller commanding less than hover thrust → drone will fall
thr_pwm >> 42000        → controller commanding more than hover thrust → drone will rise
```
