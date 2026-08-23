# Investigation — inverted `loop` (accel-pinned) commissioning, 2026-07-27

Bonus item 10b from `FINALIZE_PROJECT.md` ("a few more teardrop/inverted-loop attempts").
Turned into a full commissioning session for `loop`, plus a scoping pass on `teardrop`/
`teardrop_wide`. This doc is the single source of truth for what was tried, what broke, what
fixed it, and what's still open — pull numbers from here directly for the presentation instead
of re-deriving them.

## 1. Starting point

`loop`, `teardrop`, `teardrop_wide` all use **acceleration-pinned inversion**: instead of
authoring attitude explicitly (Mode 2), a hard equality constraint on world-frame acceleration
at the apex waypoint (`richter_with_accel_pins` / `plan_from_times_with_accel_pins`, in
`export_poly4d.rs`) forces the differential-flatness solution to produce `body_z = (0,0,-1)`
there — genuine inversion, without ever authoring a quaternion. `teardrop`/`teardrop_wide`
already had this working (single pin, automatic Richter timing, kt=0.05, flown 2026-07-21/22).
`loop` did not yet have a flight-validated pinned version going into this session.

## 2. Timeline

**Root cause #1 — automatic timing gives the pin no time.** First `loop` attempt used
automatic Richter timing (kt-driven). The pin got ~28ms regardless of kt, forcing a near-zero-
thrust singularity and an unachievable ~90 rad/s rotation rate. **Crashed** (not flown — caught
in simulation before flight). Fixed by switching to manual per-segment timing
(`plan_from_times_with_accel_pins`), entry=exit=0.33s, pin window=0.16s either side of the pin
waypoint, pin=1.4g.

**Root cause #2 — tilt clamp.** First real flights (`loop_mode1_kt0.15_2026-07-27_18-54-45`,
`...18-56-58`) crashed: `tau_y` pinned at exactly `-0.01400` (the old `tau_xy_max` clamp) —
the position loop's **tilt clamp** (`clamp_en` bit 2, `tilt_max_deg=30`) was capping the desired-
thrust-vector angle before it reached the attitude controller at all, structurally incompatible
with ~180° inversion. Fixed: `clamp_en: 15 -> 11` (tilt clamp off, tau/thrust clamps stay on) in
`crazyswarm2/crazyflie/config/crazyflies.yaml`. Also loosened `tau_xy_max: 0.014 -> 0.030` per
operator request to loosen clamps generally.

**Flights `loop_mode1_kt0.15_2026-07-27_19-06-56`, `...19-08-08`** (same entry=exit=0.33s
timing, new clamps): drone reached **roll=175.6°, near-full inversion** — the flip itself
worked — then tumbled recovering (gyro to -1997°/s, torque sign-flipping at the new ±0.03
clamp). Genuine overshoot, not an authority problem at that point. **Both crashed.**

**Detour — entry/exit shortened to 0.20s** (chasing "quicker pre-flip, harder post-flip
correction" per operator intent, misapplied). Flights `18-54-45`/`18-56-58`-adjacent retune
flown as `loop_mode1_kt0.15_2026-07-27_19-39-50`, `...19-47-00`: **worse**, not better — an
uncommanded roll kick right at the entry->pin handoff, sometimes derailing the whole climb,
sometimes surviving into a flip with zero recovery margin left. **Reverted** back to
entry=exit=0.33s (the config that actually produced the clean 175.6° flip).

**Root cause #3 (the real one) — hover height, not trajectory shape.** All four real crashes
so far were flown at hover height ~1.0-1.05m. Telemetry showed the drone reaching z≈1.87-1.90m
at the exact moment of max roll, every single time — the flight volume's real ceiling
(~0.3-1.7m usable), not a control failure. The loop's own z-span is `[0, +1.0]` relative to
hover (asymmetric — waypoint 0 is at the bottom, z=0, NOT the center). At hover=1.0m the apex
target is ~2.0m, over the ceiling, at *any* trajectory timing. Fixed: `--height 0.5` (apex at
~1.5m, safe margin both sides). **This was the actual reason every earlier crash reached
apex-region and failed — confirmed from CSV mocap z-trace across all 4 crash logs, not
inferred.**

**Separately found, unrelated bug — wind-up ramp excursion.** `wrap_rest_to_rest()` (connects
the periodic core lap to a rest-to-rest start/end) computed its entry/exit ramp duration from
lap *speed* only, ignoring lap-point *acceleration*. At the loop's lap point there's real
centripetal acceleration (bottom of a tight r=0.5m circle), and the ramp had to swing far in z
to satisfy that boundary condition smoothly. At the (already-reverted) 0.20s-entry timing this
reached **±1.79m** — enormous, would not fit in the flight envelope at any height. Even at the
original 0.33s timing it was ±0.34m — non-trivial but apparently below the threshold that
visibly affected those flights (they failed later, near the apex, not during entry). Fixed:
cap ramp duration using boundary acceleration, not just speed (empirical fit
`excursion ≈ 0.0254 * accel * t_e²`, budget 0.10m). Verified harmless/beneficial for other
trajectories too (circle, teardrop, teardrop_wide excursions all shrank slightly as a side
effect — see §4 below).

**First fully-fixed flight** (`loop_mode1_kt0.15_2026-07-27_19-55-26`, `...19-57-04`, at
`--height 0.5`; a third `...19-59-44` flown at `--height 0.9` per operator's own test of
whether more margin alone would fix recovery): **all three reached clean inversion (roll
163.6-178.2° right at the apex, apex height matching plan within margin) — the flip is now
solid and reproducible regardless of height.** Height 0.9 did NOT fix the remaining problem —
confirming it's not a height/ceiling issue for the *recovery* phase. Post-apex, all three still
crashed: z dropped from ~1.42m to ~-1.9m in 1.0s (freefall reference for 1s from rest is 4.9m —
so ~32% braking was happening, not zero, but nowhere near enough). `tau_x`/`tau_y` saturated at
the clamp *and* kept flipping sign (chattering, not converging), gyro rates hit 500-1000+ deg/s.
Read as an attitude-recovery authority/tracking problem, not a pure thrust-ceiling problem —
torque was already saturating and still couldn't converge.

**Note on `stabilizer.thrust`**: initially misread this CSV column as confirming "near-zero
thrust during the fall." Retracted — checked `firmware_app/src/lib.rs`
(`controllerOutOfTree`), and this log variable reads near-zero even during rock-steady hover in
the same logs, so it isn't usable as a real-thrust signal for this analysis. The z-position
freefall-rate argument (§ above) doesn't depend on it and stands on its own.

## 3. Three-pronged fix, applied together (2026-07-27 evening, not yet flown as of writing)

1. **Diagnostic resolution**: `attitude` and `gyro_acc` CRTP log topics bumped 20Hz -> 100Hz in
   `crazyswarm2/crazyflie/config/crazyflies.yaml` (matching `state`/`indi_state`, already at
   100Hz with zero packet gaps observed in the `19-55-26` log — real evidence this drone's
   radio link has headroom, not just an assumption). No SD/USD logging available on this
   drone, so this is the best resolution achievable. Goal: check whether the post-flip torque
   chatter correlates with EKF-attitude-estimate lag (a known open question for this INDI
   setup, see `investigation_indi_oscillation_2026-07-21.md` §16.4) vs. genuine authority
   limits.
2. **Recovery pin**: `build_loop()` now has a **second** acceleration pin at waypoint 6 (one
   full exit segment past the apex, +0.3g) forcing the flatness solution itself to include a
   real recovery acceleration, instead of leaving the post-apex path to an unconstrained
   min-jerk return. Required widening `exit_t` 0.33s -> 0.40s — every combination tried at
   0.33s was QP-infeasible for any pin magnitude/waypoint (tested exhaustively). Validated:
   max_thrust=0.750N (97% of 0.77N ceiling — tightest margin in this config, watch closely),
   max_omega=22.1 rad/s (65%), max_tau_xy=0.0147Nm (49% of the *new* 0.045 clamp, see below).
3. **Clamp loosened further**: `tau_xy_max: 0.030 -> 0.045` (still under the 0.052Nm
   blow-up-incident value that originally motivated this clamp). If the chatter persists
   *unchanged* at 0.045, that's evidence for the EKF-lag explanation over saturation — flagged
   in the yaml comment as a decision point for the next analysis pass.

**Explicitly rejected**: fully disabling `tau_xy_max`. The observed failure mode is torque
saturating *and oscillating* (sign-flipping), not consistently pinned wanting more in one
direction — removing the clamp risks reproducing the original blow-up incident and could make
chatter amplitude worse, not better. Tilt clamp (`clamp_en` bit 2) stays off — required for any
genuine inversion, non-negotiable for loop/teardrop/teardrop_wide alike.

## 4. Height calc, all three inverting trajectories (2026-07-27)

Different trajectories use **different height conventions** — confirmed from real telemetry,
not assumed:

| Trajectory | z-formula (rel. to hover) | z-range | wind-up (kt=0.15) | safe `--height` window | recommended |
|---|---|---|---|---|---|
| `loop` | `r*(1-cos θ)`, wp0=bottom | `[0, +1.0]` | ±0.11m | [0.41, 0.59] | **0.5** |
| `teardrop` | `-rz*cos θ`, wp0=bottom, **center**=hover | `[-0.5, +0.5]` | ±0.11m | [0.91, 1.09] | **1.0** |
| `teardrop_wide` | same, rz=0.48 | `[-0.48, +0.48]` | ±0.11m | [0.89, 1.11] | **1.0** |

`loop`'s convention differs from `teardrop`/`teardrop_wide` — `--height` sets loop's *bottom*
but teardrop's *center*. Confirmed empirically: a real teardrop flight
(`teardrop_mode1_kt0.05_2026-07-27_18-27-39`) hovered at z≈0.70m and reached apex z≈1.11m,
matching center+0.5 (not bottom+1.0). Getting this wrong was the single largest contributor to
every `loop` crash before the fix (§2) — worth double-checking per-trajectory before any new
inverted maneuver, not assuming one convention.

## 5. teardrop / teardrop_wide recovery pin — attempted, did NOT transfer (open)

Tried to replicate loop's two-pin recovery technique for `teardrop`. Extensive search: pin
placement (waypoint immediately after apex; one waypoint further out, mirroring loop's exact
topology); widening the segments around it individually and jointly; letting Richter's
redistribution optimizer rebalance (`n_iter` 0/4/8/16); recovery magnitude 0.05g-0.3g. **Every
combination either QP-fails outright or "solves" into something physically nonsensical** —
near-zero-thrust singularities (min_thrust down to 0.001-0.03N) paired with omega spikes from
70 up to 3500+ rad/s (limit 34) and thrust exceeding the 0.77N ceiling.

Why loop worked and this doesn't: loop's two pins have a full unconstrained buffer segment
between them (waypoint 5, no pin) — room for the polynomial to swing from "pointing down" to
"pointing up" smoothly. Teardrop's apex and any nearby recovery point are geometrically tighter
(rz=0.5m, faster v_hi=2.6 m/s) — pinning acceleration at two nearby points seems to force a
near-zero-thrust crossing regardless of spacing/magnitude tried. Would need a structural
redesign (e.g. an extra unconstrained waypoint inserted into `push_vloop`'s geometry), not a
tuning pass. **Left on the existing single-pin config** (already flight-validated at kt=0.05,
2026-07-21/22) — only the height fix (§4) and global clamp loosening (§3.3) applied, no
trajectory-shape changes.

**Decision**: get real flight data from `loop`'s three-pronged fix (§3) first — specifically
whether the 100Hz logs show saturation-chatter vs. EKF-lag — before investing more QP time into
teardrop/teardrop_wide's tighter geometry. If loop's recovery pin genuinely fixes the tumble,
the *principle* (explicit recovery-acceleration pin) is validated even though teardrop needs
its own from-scratch geometry work to use it.

## 6. Flight log inventory, 2026-07-27

| Log | Config | Outcome |
|---|---|---|
| `loop_mode1_kt0.15_18-32-36`, `...18-34-09` | early/exploratory | superseded |
| `loop_mode1_kt0.15_18-54-45`, `...18-56-58` | manual timing, `clamp_en=15` (tilt clamp still ON) | crashed — tau pinned at old 0.014 clamp |
| `loop_mode1_kt0.15_19-06-56`, `...19-08-08` | `clamp_en=11`, `tau_xy_max=0.030`, entry=exit=0.33s | **clean flip (roll 175.6°)**, crashed on recovery — later found to be the ceiling-height problem |
| `loop_mode1_kt0.15_19-18-38`, `...19-19-37`, `...19-25-58` | entry/exit=0.20s retune | worse — uncommanded roll kick at entry->pin handoff |
| `loop_mode1_kt0.15_19-39-50`, `...19-47-00` | entry/exit=0.20s (continued) | same failure mode, confirmed not viable, reverted |
| `loop_mode1_kt0.15_19-55-26`, `...19-57-04` | reverted to 0.33s, `--height 0.5` | **clean flip (roll 172.6-178.2°)**, height now correct, recovery still tumbles — root cause reframed as attitude-recovery authority/EKF-lag, not height |
| `loop_mode1_kt0.15_19-59-44` | same, `--height 0.9` (operator test) | confirms height 0.9 does NOT fix recovery — height isn't the remaining variable |
| `teardrop_mode1_kt0.05_18-27-39` | existing single-pin config | used to empirically confirm teardrop's height convention (§4) |
| `teardrop_wide_mode1_kt0.08_18-29-21`, `...18-30-25` | existing single-pin config | reference flights, not re-analyzed in depth this session |

## 7. Current state (as of this doc, not yet flown)

`loop`: entry=exit=0.33s, pin(apex, wp4)=-1.4g, recovery pin(wp6)=+0.3g, `--height 0.5`,
`clamp_en=11`, `tau_xy_max=0.045`, attitude/gyro_acc logging at 100Hz. **Next flight is the
first test of the three-pronged fix (§3) — not yet flown as of this doc's last edit.**

`teardrop`/`teardrop_wide`: unchanged trajectory shape (single pin, automatic timing),
`--height 1.0` (both), global clamp loosening inherited automatically. Not re-flown this
session.

## 7b. Second recovery pin (2026-07-28) + a real wind-up-ramp bug found and fixed

Flown: `loop_mode1_kt0.15_2026-07-28_17-11-12` (wrong height again, ~0.95m, discard),
`...17-13-14`, `...17-15-12` (correct height ~0.55m). Both good-height flights: the single
recovery pin (§3) genuinely worked for its intended window -- roll now recovers **monotonically**
from ~180 deg to near-level in ~0.35s (vs. the old multi-swing chatter). But a **second,
previously-hidden oscillation** starts right where the pin's influence ends (~t+0.4-0.5s
post-apex): torque saturates and flips sign again, roll swings back to 60-115 deg, both flights
crash 2-2.7s after the apex. 100Hz gyro-vs-EKF-roll-rate check in that window found **sustained
50-100ms sign-disagreement runs** -- real evidence for EKF-lag during the fast tumble (not pure
saturation), consistent with the leading hypothesis in
`investigation_indi_oscillation_2026-07-21.md` §16.4.

Fix: extended recovery coverage with a **third pin** at waypoint 7 (wp6 lowered 0.3g->0.1g, wp7
added at 0.2g) -- keeps the commanded recovery acceleration alive through the window where phase-2
previously began, instead of ending abruptly at wp6. Validated: max_thrust=0.726N (94%),
max_omega=21.4 rad/s (63%), max_tau_xy=0.0043Nm (10% of the 0.045 clamp -- much more margin than
the 2-pin version).

**Found and fixed a real bug in `wrap_rest_to_rest`'s timing logic while validating this**: the
existing accel-only excursion budget (from §2's wind-up fix) doesn't account for jerk, and the
3-pin config has low accel but high jerk at the lap point -- the position-only bisection shrank
`t_e` to 0.31s (fine for excursion) but produced a **dynamically infeasible ramp**: 1.12N thrust
(vs. 0.77N ceiling) and 39.6 rad/s omega (vs. 34 ceiling), invisible to the excursion metric alone.
Fixed properly: `wrap_rest_to_rest` now searches for the smallest `t_e` that is BOTH within the
excursion budget AND dynamically feasible (evaluates `compute_flatness` directly on the candidate
ramp, brushless-conservative limits at 90% margin), instead of trusting either metric alone.
Re-verified across all periodic trajectories -- loop, circle both clean.

**While doing that re-verification, found a separate, unrelated, and more serious problem**:
`teardrop`'s and `teardrop_wide`'s **core trajectory** (unrelated to any wind-up ramp, unrelated
to today's loop work) has a **near-zero-thrust singularity** -- `compute_flatness` swept across
the full core lap shows `min_thrust` down to 0.009-0.017N and **`max_omega` of 300-500 rad/s**
(9-15x the 34 rad/s ceiling), at their actual historically-flown kt values (kt=0.05 for teardrop,
kt=0.08 for teardrop_wide) -- not just at the kt=0.15 used for some scratch checks in this doc.
This is the same failure class as loop's original "automatic timing gives the pin no time"
problem (§2) -- differential flatness is ill-conditioned near zero thrust, where the required
attitude direction becomes extremely sensitive -- but was never caught for teardrop/teardrop_wide
because no full omega sweep had been run on their core trajectories before now (only thrust was
checked, early in this session). **This is very likely the real, previously-unexplained root
cause of the still-open `teardrop_wide` crash** ("crashed under plain geometric on both platforms,
never root-caused" -- FINALIZE_PROJECT.md §10 item 2, `project_final_presentation_2026-07-25.md`).
**Do not fly `teardrop`/`teardrop_wide` again until this is investigated** -- needs the same
treatment loop's automatic-timing singularity got (§2): either manual per-segment timing that
gives the pin-adjacent segment more room, or accept a lower kt and re-check. Not attempted this
session -- flagged here for the next one.

## 9. 2026-07-28 continuation — compressed timing, v5 (best result), and the exit-ramp wall

Long single-session continuation of §7b's open thread (post-flip recovery still crashing).
Summary of the version history, each one flight-tested before moving to the next:

### 9.1 v3 — compress recovery-pin timing to match the real fall budget

Root cause found for why §7b's pin (fired 0.56s post-apex) never fully worked: real telemetry
showed the drone hitting the floor **0.33-0.50s** after the apex — the pin was scheduled to fire
*after* the crash was already happening. Re-timed the post-apex segments so the recovery pin
fires at **0.38s post-apex** (redistributing time onto the final segment to keep the QP
feasible), and bumped its magnitude 0.20g→0.40g (the earlier-firing pin has more thrust margin:
84% of ceiling vs. 91%).

Flown (`loop_mode1_kt0.15_2026-07-28_19-09-02`/`19-11-20`, height=0.5m correct): **real, measured
improvement** — descent went from 115-143% of freefall (worse than freefall, from the §3 config)
to **50-91% of freefall**, genuinely arrested and *improving over time* as the pin takes hold.
Both flights grazed the floor gently instead of crashing. Committed as `8330c5f` ("best loop
thus far v3").

### 9.2 v4 — second pin, broke pre-flip, root-caused, reverted, permanent check added

Tried adding a second pin at wp7 (0.87s post-apex, +0.50g) to catch a residual wobble. The core
trajectory shifted only ~2cm pre-apex and every numeric feasibility check (thrust/omega/torque)
looked fine — but two real flights (`19-20-00`/`19-21-43`) **crashed pre-flip**, roll to 150+ deg
at t=0.51-0.54s, long before the apex was ever supposed to happen (planned 1.80s).

Root cause: the rest-to-rest wind-up **ramp** (built by `wrap_rest_to_rest`, connects hover to
the lap-start point) depends on the core's boundary derivatives at that point. The downstream pin
change shifted those derivatives enough to give the ramp's own acceleration profile a real
~50-55° commanded tilt during what should be a benign takeoff — verified by evaluating the
exported ramp coefficients directly through `compute_flatness`. The existing position-based
excursion check (§2) didn't catch it because the *position* deviation was still small (±0.11m);
only the *attitude* implied by that position's acceleration was corrupted.

**Fix, made permanent**: `export_poly4d` now runs a standing wind-up-ramp **attitude** check
(not just position) on every `--onboard` export — evaluates `min(body_z.z)` across the entry and
exit ramps and warns if it implies more than ~46° tilt (`min_bz < 0.7`). Verified: silent on the
known-safe v3 config, correctly fires on the reverted v4 config. This check is what caught every
subsequent bad idea below *before* flying it.

### 9.3 v5 — second pin done properly, best result of the whole investigation

Re-added the second pin (wp7, +0.20g, 0.87s post-apex) but validated against **four** criteria
before flying anything, per the v4 lesson: (1) core feasibility, (2) ramp position excursion,
(3) ramp **attitude** (the new check), (4) pre-apex core shift. Grid-searched timing/magnitude
specifically for ramp attitude staying close to v3's own baseline (0.71-0.73) rather than just
"not obviously broken." Validated: core max_thrust=0.650N (84%), max_omega=24.47 rad/s (72%),
max_tau_xy=0.0044Nm (10%); ramp min_bz=0.728 (not degraded from v3's 0.730/0.711); pre-apex shift
1cm.

Flown (`loop_mode1_kt0.15_2026-07-28_19-33-11` and others): **the best result of the entire
investigation.** Apex near-full inversion, first pin arrested the fall (descent stayed below
freefall rate), the drone **grazed the floor without crashing** (z_min=-0.028m), then **genuinely
recovered** — positive vz, climbed back to z=+0.42m, roll settled to single digits. A third
tumble then started at t=3.36-3.45s — inside the **auto-generated rest-to-rest exit ramp** (core
closes at t=3.17s per this trajectory's timing), which has zero pin coverage and just assumes
the drone already matches its boundary state by the time the core hands off to it. Real tracking
drift from the earlier recovery broke that assumption.

**Committed as `ba25316` ("best loop thus far v5") — this is the current, flight-validated code
state.** Flight log inventory for this round: 3 good-height flights close to `19-33-11`'s
outcome, several others at wrong heights (0.65-0.85m, operator testing the height hypothesis,
see 9.4) that reproduced the third-tumble pattern but with worse apex/recovery quality.

### 9.4 Height re-tested as a lever, found not to help (again)

Operator hypothesis: does more altitude margin give the recovery more time to complete before
the exit-ramp handoff? Re-tested at height=0.65-0.85m. Result: **no** — apex strength and
recovery quality were unaffected or worse (apex 122-157° vs. 0.5m's 163-178°), and the same
third-tumble pattern occurred regardless of height. Confirms height was never the right lever for
this specific failure (consistent with the original freefall-rate finding) — the exit-ramp
handoff problem is a *timing/state-matching* issue, not an altitude-budget issue.

### 9.5 v6 attempts — 4th pin (two placements), both hit the same structural wall

Two independent approaches tried to close the exit-ramp gap, both searched properly (not flown
blind) and both found to have no safe+useful solution — reverted, working tree returned to v5
exactly (verified numeric match) after each:

1. **Extend the core's own pin coverage**: inserted a new waypoint between wp7 and the close
   point, with a pin anchoring it, so the core (not the generic ramp) holds a real reference
   further into the recovery. Grid-searched requiring both ramp ends stay ≥0.70 (matching the
   standing check). Result: a pin early/strong enough to land near the real tumble time
   (~1.5-1.6s post-apex) always degrades ramp attitude below 0.70 (e.g. +0.30g at 1.47s
   post-apex → min_bz=0.693); the combinations that stay safe only exist weak (0.05-0.15g) and
   firing at 1.87-2.17s post-apex — already later than the real failure.
2. **Split the exit ramp itself with an internal pin**: added `wrap_rest_to_rest_pinned_exit`,
   splitting the ramp into two Hermite sub-segments at a chosen τ with a pinned acceleration at
   the junction (mid-junction position/velocity/jerk taken directly from the *original* ramp's
   own curve at that τ, only acceleration overridden — an earlier version that hand-picked
   linear-interpolated velocity and forced zero jerk was far more infeasible, omega 45-305 rad/s
   even at zero pin magnitude, and was discarded first). First validation pass had a real bug
   (extracted the wrong end of the core's last segment — `at_end=false` instead of the correct
   `at_end=true` — giving falsely optimistic numbers); after fixing the bug and re-validating
   properly: **even a zero-magnitude split already sits at the edge of safe attitude** (best
   case min_bz=0.708), because the exit ramp inherits substantial acceleration from the core
   right at its start (a=(2.78, 0, 2.72)) — splitting it *at all* concentrates that into a
   shorter window and degrades attitude, independent of any pin. No timing/magnitude combination
   found both safe and useful.

Both attempts point at the same underlying fact: the exit ramp is structurally near its own
attitude margin already, not merely under-explored. The function `wrap_rest_to_rest_pinned_exit`
and its wiring were fully implemented and validated-away, then reverted out of the code entirely
(not left in as dead code) — if revisited, rebuild from this doc's description rather than
searching from scratch.

### 9.6 A+B hypothesis (non-periodic rest-to-rest loop + tighter radius) — both refuted

Proposed direction: (A) make the loop non-periodic and self-terminating (core itself decelerates
to rest at hover, removing the un-pinnable exit ramp entirely) combined with (B) a tighter loop
radius (less energy to recover from). Both tested empirically before committing to either:

- **(A) refuted**: a genuinely rest-to-rest (v=a=0 at both ends) periodic-free loop hits a
  **zero-thrust singularity** near the flip — flatness is ill-conditioned when required thrust
  crosses zero, and a slow "flip in place" does exactly that. Confirmed directly: even generous
  timing gave omega up to 1500 rad/s and thrust magnitudes collapsing to ~0.001N. The loop
  fundamentally *needs* forward speed through the flip to keep thrust positive — periodicity
  (entering already at speed) isn't an accident of the current design, it's load-bearing.
- **(B) refuted**: tighter radius makes it *worse*, not better — ω = v/r, so a smaller loop at
  the same speed needs a *higher* rotation rate. Direct sweep: r=0.5m gives omega=24 rad/s
  (current, working); r=0.35m gives omega=92 rad/s; r=0.30m gives omega=140 rad/s. r=0.5m is
  close to the *best* radius for feasibility, not a compromise to shrink further.

Both ideas were reasonable first-principles guesses that didn't survive contact with the actual
QP/flatness math — recorded here specifically so a future session doesn't re-propose them without
first checking this section.

### 9.7 State at end of session

Code is at `ba25316` ("best loop thus far v5") — the version that produced flight `19-33-11`,
the best result of the whole investigation (genuine recovery, third tumble only in the un-pinned
exit ramp). No uncommitted changes. Verified via `cargo run --release --bin export_poly4d --
--trajectory loop --mode 1 --kt 0.15 --onboard`: wind-up excursion x±0.16/z±0.06m, thrust=0.650N
(84%), omega=24.47rad/s (72%), tau=0.0044Nm (10%), ramp attitude 0.774(entry)/0.728(exit), no
warning.

**Open problem, not solved this session**: the exit-ramp tumble. Every approach tried within the
current architecture (more pins, extended core coverage, split ramp, non-periodic redesign,
tighter radius) has been tested and found infeasible or unhelpful. What hasn't been tried:
a fundamentally different exit strategy that doesn't require the ramp to reproduce the exact
lap-speed boundary state (e.g. accepting a larger/asymmetric final settling region, or checking
whether the *entry* ramp's `wrap_rest_to_rest` reasoning generalizes differently for exit given
the observed real-flight state at handoff differs from planned). Next session should read this
entire §9 before proposing new pin placements — the search space very close to v5 has been
covered thoroughly.

## 8. For the presentation (superseded — see update below)

*(Original 2026-07-27 framing, kept for the timeline; §9.7 below has the current headline.)*
If this becomes a slide/section: the honest headline is "inverted loop flip achieved and
repeatable (3/3 clean inversions at 163-178° roll); post-flip recovery still open — genuine
control problem (torque saturating and chattering, not a height or trajectory-shape issue),
diagnostic instrumentation + a recovery-pin trajectory change now in place, next flight will
tell us if it worked." Good material for the "Real Fix Candidates" / limitations framing already
used elsewhere in the deck (see `FINALIZE_PROJECT.md` §4.6-4.7) — same honest-about-what's-
unresolved tone. The height-convention bug (§4) is a good concrete "root cause was operational,
not the trajectory math" example if the presentation wants one.

**Updated headline (2026-07-28, after §9)**: inverted loop flip achieved and repeatable, AND a
genuine post-flip recovery achieved for the first time — flight `19-33-11` inverted, arrested a
near-freefall, grazed the floor without crashing, and climbed back to positive altitude with
level attitude. The remaining failure (a third tumble in the auto-generated exit ramp) was
narrowed to a specific, well-understood mechanism through systematic search (four independent
fix attempts tried and ruled out with data, not guesswork — see §9.5-9.6), not an unexplained
crash. This is a stronger, more specific story for a limitations slide than the 07-27 version:
"we know exactly where and why it still fails, and we know four plausible-sounding fixes that
don't work and why" is a better demonstration of rigor than an open-ended "still investigating."
