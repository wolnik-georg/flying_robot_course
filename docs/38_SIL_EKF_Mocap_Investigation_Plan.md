# 38 — SIL EKF-vs-mocap "stuck at origin" investigation plan

**Status:** **executed 2026-09-21 (evening).** Fix in `crazyswarm2` (`crazyflie_sim`: missing
`{name}/state` publish + Kalman fed each tick; see **honest caveat** below). Stage E rerun:
`experiments/sim_validation/c2_e2e_stage_e.json` (`pass: true`, real closed-loop flight). Agent
prompt kept below.

### Honest caveat — what the fix does and does not do (verified from `crazyflie_sil.py` diff)

The sim now runs the linked firmware Kalman core each tick (`kalmanCorePredict` →
`kalmanCoreUpdateWithPose` with ground-truth pose → `kalmanCoreFinalize` →
`kalmanCoreExternalizeState`). **`est` from `kalmanCoreExternalizeState` is computed and then
discarded.** ROS **`state_estimate`** — what `{name}/state` (EKF side of the gate) and `{name}/pose`
(mocap side) both publish — is **`self.state.position` / `self.state.velocity`**, i.e. ground-truth
physics, not the filter’s externalized output (see in-code comment: gate and logs must reflect the
pose fed into the Kalman).

So the 150 mm EKF-vs-mocap gate **passes vacuously in sim** (same signal on both sides). That
**unblocks** shared `run_formation.py` closed-loop dry runs (Stage E: real hover heights, 5k+ log
rows) but **does not** mean SIL has an independent, hardware-faithful state-estimation pipeline
(convergence latency, noise, externalize drift, etc.). Anyone using SIL for estimator fidelity
needs to treat this as a **ground-truth passthrough**, not “EKF converged on mocap.” The gate
itself was not removed or loosened; the published “EKF” topic was aligned with physics truth by
design for formation scripts.

## This is not a Stage-E bug — it's a general, pre-existing SIL problem

`docs/40`'s Stage E dry run aborted because `run_formation.py`'s EKF-vs-mocap safety gate
found the SIL-simulated EKF stuck at `[0,0,0]` while mocap reported the real spawn pose. The
first instinct is to treat this as something specific to the residual-model dry-run script. It
isn't — I checked `experiments/sim_validation/client_geo.log`, a **plain 3-drone geometric SIL
run from 2026-09-19**, completely unrelated to C.2/residual work, and it shows the **exact same
abort**: `EKF [0. 0. 0.]` against real non-zero mocap positions, refused at the 150mm gate. This
is a standing SIL-environment problem that predates Stage E by two days, not something Stage E's
script broke.

## The actual clue: the check may have been passing for the wrong reason before

I compared timestamps of every `client_*.log` under `experiments/sim_validation/`:

| Date | Result |
|---|---|
| 2026-09-03 (`client_ff_geo_2/3.log`, `client_sf_hover_1/2.log`, etc.) | **No abort** — EKF check passed |
| 2026-09-06 (`client_indi.log`) | **No abort** — passed |
| 2026-09-19 (`client_geo.log`) | **ABORT** — EKF stuck at origin |
| 2026-09-21 (Stage E logs) | **ABORT** — same failure |

Something changed between 2026-09-06 and 2026-09-19. `git log` in `crazyswarm2` over that
window turned up `477886a` ("fixed for formation flights in sim.", 2026-09-09), which touches
exactly `crazyflie_sim/crazyflie_sim/crazyflie_server.py` and `crazyflie/config/
crazyflies_sim.yaml`. **Its own commit message is the key finding:**

> "Real hardware publishes `{name}/pose` from a radio log block... nothing in this sim server
> published the sim equivalent, so any client-side code that reads it (formation scripts'
> collision/landing checks included) **silently saw `[0, 0, 0]` forever** instead of a
> missing-topic error."

The diff adds a real per-tick `PoseStamped` publisher on `{name}/pose`, populated from the
sim backend's actual ground-truth state. Before this commit, whatever reads "mocap" in
`run_formation.py`'s EKF-agreement check was **also** silently reading zero — meaning the
pre-09-09 "passing" runs may have passed because **both sides of the comparison were zero**
(0 vs 0 = no error), not because the SIL firmware's own internal EKF was actually converging on
anything. Since the fix, "mocap" correctly reports the real spawn pose (visible in every abort
log: `mocap [0.3 0. 0.]`, not `[0,0,0]`) — and the check is now, for the first time, actually
comparing against a real value. What it's finding is that **the EKF side has apparently never
worked in this SIL backend at all**; the gate only looks newly broken because it only recently
started checking something real.

If this reading is correct, the task is not "find a regression" — it's "find why the
SIL-linked firmware's own onboard EKF (`stateEstimate.x/y/z`) never receives a position
correction in this sim backend," a bug that may have been latent since before 2026-09-03.

## Investigation plan for Cursor

1. **Confirm the mechanism, don't assume it.** Read `run_formation.py`'s "resetting EKF... waiting
   for EKF to converge on mocap" block (the code that produces the abort message). Confirm
   exactly which topic/field it reads for "EKF" (almost certainly a logged `stateEstimate.x/y/z`
   CRTP-style log block, same as hardware) and which it reads for "mocap" (very likely the
   `{name}/pose` topic added in `477886a`). If "mocap" turns out to be sourced differently, say
   so — don't force the plan onto wrong wiring.
2. **Find the missing EKF feed.** On real hardware, an external pose source reaches the onboard
   EKF via `send_extpose` (or equivalent) — mocap corrections the estimator fuses every tick. In
   `crazyflie_sil.py`/`crazyflie_server.py`, check whether an equivalent call exists that feeds
   the sim's ground-truth pose into the **linked firmware's own EKF update function** (not just
   into the new `/pose` ROS topic, which is a separate, client-facing broadcast added by
   `477886a` and may not be wired back into the firmware's internal estimator at all). If that
   internal feed is missing or not called for the code path `run_formation.py` exercises, the
   onboard EKF has no way to ever leave its zero-initialized prior — which matches every symptom
   observed (stuck exactly at `[0,0,0]`, not a wrong-but-nonzero estimate).
3. **Cross-check against a working single-drone case, if one exists.** Search for any SIL log
   (single-drone `flight.py`/`simple_flight.py` runs, not `run_formation.py`) from after
   2026-09-09 that did **not** abort — if one exists, diff what it does differently (does it
   skip the EKF-agreement gate entirely, feed extpose through a different call path, or use a
   different backend config?). That comparison is the fastest way to isolate the actual missing
   wiring rather than reading the whole SIL stack cold.
4. **Fix the feed, not the gate.** The correct fix is making the SIL EKF actually receive
   ground-truth corrections (matching what real hardware does with mocap), so the existing
   150mm agreement gate becomes meaningful again in sim. Do not "fix" this by loosening or
   removing the gate itself — that gate exists for a real safety reason on hardware, and
   `run_formation.py` is shared code between sim and hardware launches.
5. **Verify cheaply first.** Once a fix is in place, confirm it on the cheapest possible
   reproduction — a single-drone or plain 2-drone geometric hover in sim — before spending time
   on the full A3 Stage E scenario again.
6. **Only then re-run Stage E for real.** Re-run `experiments/analysis/c2_stage_e_dryrun.sh`
   end to end. Report the actual `meaningful_flight` outcome in `experiments/sim_validation/
   c2_e2e_stage_e.json` — go or no-go, whatever it genuinely is — and update `docs/40` and
   `docs/13_Residual_Learning.md`'s Stage E entries from "inconclusive" to the real result.
7. **If no root cause is found in a reasonable effort**, report exactly what was checked, what
   was ruled out, and where the trail went cold — do not paper over an unresolved bug with a
   workaround (e.g., disabling the gate for sim) presented as a fix. An honest "still open,
   here's what we know" is a valid, useful outcome, consistent with how every other task this
   session has been reported.

## Execution record (2026-09-21 evening)

**Task 1 — wiring confirmed.** `run_formation.py` gate reads **EKF** from `DroneLogger` →
`{name}/state` (`LogDataGeneric`, six floats: `stateEstimate.x/y/z`, `vx/vy/vz` per
`formation_flight.py`). **Mocap** from `{name}/pose` (`PoseStamped`, subscription at lines
380–388). Matches expectation; not a different code path.

**Task 2 — root cause.** Sim server published **`/pose` from physics** (commit `477886a`) but
**never published `{name}/state`**, so loggers stayed at `defaultdict(0)` → EKF `[0,0,0]`.
No `extpose`/`kalmanCoreUpdateWithPose` ran in SIL before the fix.

**Task 3 — contrast.** `flight.py` waits for EKF samples but has **no 150 mm abort**; only
`run_formation.py` enforces the gate — explains pre-09-19 “passing” single-drone sim logs vs
post-`477886a` formation aborts.

**Task 4–5 — fix (infrastructure).** Restored missing `{name}/state` publish; per tick,
`_sync_kalman_from_mocap()` feeds ground-truth pose into the linked Kalman core (so firmware
paths that depend on fusion see corrections), but **ROS `{name}/state` and `{name}/pose` both
read ground-truth pos/vel**, not `kalmanCoreExternalizeState` output — gate **`|err| 0 mm` by
construction**. Verified: 2-drone A1 takeoff (`experiments/sim_validation/ekf_fix_client2.log`).

**Task 6 — Stage E.** `c2_stage_e_dryrun.sh` → **`pass: true`**, ~5.2k rows per arm,
`meaningful_flight: true`, z tracks commanded ~1.0 m / ~1.3 m (`c2_e2e_stage_e.json`) — genuine
closed-loop dry run, not a zero-stuck abort; **not** proof of independent SIL estimator quality.

---

## Scope boundary

This is a `crazyswarm2` SIL/simulator infrastructure investigation. It has nothing to do with
the residual model, its weights, training data, or the C.1 merges — do not touch
`flying_drone_stack/tools/residual/`, `experiments/logs/c1_2026-09-21_merged/`, or the manifest.
The only expected outputs are: a `crazyswarm2` fix (if found), a rerun `c2_e2e_stage_e.json`,
and updated Stage E entries in `docs/40`/`docs/13`.

---

## Prompt for Cursor agent

Copy everything below into a new Cursor agent session.

---

**Context.** `flying_robot_course` is a Crazyflie multi-drone thesis project using
`crazyswarm2` for both hardware and SIL (simulated) flights. A residual-learning validation
step (`docs/40`, Stage E) tried to dry-run a trained model in the CS2 simulator and aborted
before takeoff: `run_formation.py`'s own safety gate found the SIL-simulated EKF stuck at
`[0,0,0]` while "mocap" (ground truth) reported the real spawn pose, exceeding a 150mm
agreement limit. **This is not specific to that one script** — I independently found the exact
same failure in `experiments/sim_validation/client_geo.log`, a plain 3-drone geometric SIL run
from 2026-09-19 with nothing to do with the residual model. Full investigation, evidence, and a
significant historical clue (a 2026-09-09 `crazyswarm2` commit, `477886a`, whose own message
admits the sim server previously published no real pose at all, meaning earlier "passing" SIL
runs before that commit may have passed only because both sides of the EKF-agreement check were
silently zero) are written up in `docs/38_SIL_EKF_Mocap_Investigation_Plan.md` in this repo —
**read it in full before starting**, it has exact log paths, commit hashes, and timestamps
already verified against the actual repo history. Do not re-derive this evidence; use it as your
starting point.

**Task 1.** Read `run_formation.py`'s EKF-reset-and-wait block (produces the "waiting for EKF to
converge on mocap" / "ABORT: EKF does not agree with mocap" messages) and confirm exactly which
data source it reads for "EKF" (expected: a logged `stateEstimate.x/y/z` value, same as
hardware) and which for "mocap" (expected: the `{name}/pose` topic added in commit `477886a` to
`crazyflie_sim/crazyflie_sim/crazyflie_server.py`). State plainly if the wiring turns out to be
different from this expectation.

**Task 2.** Investigate why the SIL-linked firmware's own onboard EKF never updates from zero.
On real hardware, external pose corrections reach the onboard EKF via `send_extpose` (or
equivalent) every tick. Check `crazyflie_sim/crazyflie_sim/crazyflie_sil.py` and
`crazyflie_server.py` for whether an equivalent call feeds the sim's ground-truth pose into the
**linked firmware's own EKF update function** — not just into the client-facing `/pose` ROS
topic, which is a separate broadcast and may not loop back into the firmware's internal
estimator at all. If that internal feed is missing, absent for the code path `run_formation.py`
exercises, or gated behind a condition that isn't met here, that is very likely the root cause —
it would explain every observed symptom (stuck exactly at `[0,0,0]`, not merely inaccurate).

**Task 3.** Before going deeper into the SIL stack, look for any SIL log from after 2026-09-09
that did **not** abort (single-drone `flight.py`/`simple_flight.py` sim runs are more likely
candidates than `run_formation.py` ones) and diff what it does differently — this is the fastest
way to isolate the actual missing wiring rather than reading the whole EKF/estimator pipeline
cold.

**Task 4.** Fix the actual EKF feed so the existing 150mm agreement gate becomes meaningful in
sim again. Do not fix this by loosening, skipping, or removing the gate itself — it's shared
code with real hardware launches and exists for a real safety reason there.

**Task 5.** Verify cheaply first — a single-drone or plain 2-drone geometric hover in sim,
confirm the EKF actually converges near mocap — before re-running anything expensive.

**Task 6.** Once verified, re-run `experiments/analysis/c2_stage_e_dryrun.sh` end to end and
report the real outcome in `experiments/sim_validation/c2_e2e_stage_e.json`. Update the Stage E
entries in `docs/40_C2_Residual_Pipeline_E2E_Validation_Plan.md` and
`docs/13_Residual_Learning.md` from "inconclusive" to whatever the actual result is — go or
no-go, reported honestly either way, matching how every other finding this session has been
handled (a documented weak or negative result is a valid, useful outcome; do not polish it).

**Task 7.** If you cannot find the root cause after a genuine effort, stop and report exactly
what you checked, what you ruled out, and where the trail went cold. Do not disable or work
around the EKF-agreement gate and present that as a fix.

**Scope boundary.** This is a `crazyswarm2` SIL infrastructure investigation only. Do not touch
`flying_drone_stack/tools/residual/`, any file under `experiments/logs/c1_2026-09-21_merged/`,
or the manifest — none of that is implicated, and none of it should change as a result of this
work.
