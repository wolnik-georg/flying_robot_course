# Trajectory upload paths — Mode D (onboard) vs Mode E (HLC)

**Status as of 2026-08-19.** Two upload/evaluation paths exist. Both are supported; neither
removes the other. Mode D remains the path all aggressive-maneuver results were flown on.

| | **Mode D — onboard** | **Mode E — HLC** |
|---|---|---|
| Select with | `--onboard` on both exporter and flight script | omit `--onboard` |
| CSV | `{label}_onboard.csv`, 28 cols, degree-8 in τ∈[0,1] | `{label}.csv`, 33 cols, degree-7 in physical time |
| Upload | `traj.ci`/`cv`/`cw` param poking, 3 blocking calls per float (~5 s) | stock `cf.uploadTrajectory()` |
| Start | `traj.start` param | stock `allcfs.startTrajectory()` |
| Evaluated by | our OOT controller, `eval_traj_onboard()`, 500 Hz | Crazyflie HLC, `poly4d_eval()`, 1 kHz |
| Segment cap | 14 (`traj_iface.c TRAJ_MAX_SEGS`) | 31 (4096 B / 132 B per piece) |
| Multi-robot | no — per-CF sequential uploads, shared `traj.*` namespace | **yes** — `uploadTrajectory` per drone, one broadcast start |

## Why both exist

The planner emits **9 coefficients per axis** (degree 8); Crazyswarm2 and the C firmware expect
**8** (degree 7, `pptraj.h PP_DEGREE`). `export_poly4d.rs::to_hermite_phys7` bridges this by
matching (p, v, a, j) at both segment ends, preserving C3 continuity.

That reduction is **exact when the planner does not use the degree-8 term**. Minimum-snap over a
degree-8 basis has a degree-7 optimum, so `c8 → 0` whenever the QP converges cleanly.

## Which trajectories may use Mode E

Verified offline by replaying both reference pipelines and diffing what `controller_step()`
receives — `(pd, vd, ad, ω_d, α_des)`:

**✅ Equivalent (α_des deviation < 0.005 rad/s²)** — `hover`, `figure8`, `circle`, `oval`,
`tilted_oval`. These are the tight-formation trajectories; use Mode E.

**❌ Mode D only:**
- *Degree-8 content* (`|c8|/|c_max|` ≈ 0.05–0.16) → α_des differs by 17–6700 rad/s²:
  `slalom`, `helix`, `corner`, `corkscrew`, `roller_coaster`, `loop_train`
- *Near/through zero thrust* → the two body-frame constructions diverge, Δω_d 49–691 rad/s:
  `loop`, `teardrop`, `teardrop_wide`
- *Explicit attitude polynomials* (planner Mode 2): `lib.rs` gates the attitude evaluator on
  `g_traj_mode == 1 || == 2`, and Poly4D has no attitude channel — so flip / immelmann / splits
  are structurally Mode D only.

## Rest-to-rest: why there is a wind-up ramp, and why it stays

Every closed-loop trajectory in the Mode E export starts and ends at **zero velocity**, so the
drone never receives a velocity step. That is achieved by wrapping the lap with a 0.65 s
entry/exit ramp.

**These two properties cannot both hold**: a lap flown at constant speed has non-zero velocity
everywhere on it, so starting from rest requires either (a) a ramp, or (b) a step at t=0. The
ramp buys the accelerating distance *outside* the lap, which is what keeps the lap itself exact.

The ramp is far gentler than "extra trajectory" suggests — it rolls backwards ~15 cm *along the
path* and then accelerates forward through the start point at exactly lap speed:

| circle, kt 0.1, lap-start `[0.75, 0, 0]` | position | \|v\| |
|---|---|---|
| t = 0.00 s | `[0.750, 0.000]` | 0.000 m/s |
| t = 0.33 s | `[0.737, −0.111]` | 0.594 m/s |
| t = 0.65 s | `[0.750, −0.000]` | **1.000 m/s** — enters the lap |

Measured deviation of the ramps from the flown path: **5.7 mm** (circle 1.0 m/s), 14.7 mm
(circle 1.6 m/s), 2.0 mm (oval). The lap itself is untouched — circle radius 0.7500 m with
**0.00 mm** spread.

### `--rest-to-rest` (on-path, opt-in) — measured worse, not recommended

An alternative was implemented that plans the ring **non-periodically** instead, so the drone
accelerates from rest *along* the path with no ramp at all (`spline.rs` already constrains
derivatives 1–4 to zero at both ends when `periodic = false`). It does exactly what it says, but
forcing accel and decel *inside* the lap makes the min-snap spline bow between waypoints and
overspeed:

| circle kt 0.1 | radial error vs the true circle | peak speed |
|---|---|---|
| **default (wind-up wrap)** | mean **0.4 mm**, max **5.7 mm** | **1.000 m/s** |
| `--rest-to-rest` | mean 54.5 mm, max **160.5 mm** | **2.102 m/s** |

The radius wanders 0.634 → 0.902 m on a 0.75 m circle. The flag is kept for completeness and
prints a warning; **the default wind-up wrap is the correct choice.**

### figure8 is now rest-to-rest on Mode E

`figure8` is a genuine closed loop (end-vs-start mismatch 2.3e-8 m) but was historically flown
unwrapped, starting and ending mid-lap at ~1.06 m/s. It is now wrapped in the HLC export via
`hlc_rest_wrap()`, a superset of `is_loop_safe()`.

The wrap is applied **only in the HLC branch**; the onboard export still uses `is_loop_safe()`,
so Mode D's figure8 is byte-identical (verified pre-change vs post-change). Wrapping costs
nothing in shape or speed:

| figure8 | lap shape deviation | start/end \|v\| | peak \|v\| |
|---|---|---|---|
| unwrapped (Mode D, unchanged) | — | 1.0586 m/s | 1.067 m/s |
| **wrapped (Mode E)** | **0.0 mm** | **0.0000 m/s** | 1.067 m/s |

## Continuous multi-lap flight — `--laps`

Mode D loops **only the core lap**: the firmware knows which segments are the entry/exit ramps
(`n_entry`/`n_exit` in the `.meta.json` sidecar) and wraps time within the core, giving
`ramp-up → N continuous laps → ramp-down` as one uninterrupted flight.

The HLC has no equivalent — `startTrajectory` replays whatever it was given. So on Mode E the
laps are baked into the export instead:

```bash
cargo run --release --bin export_poly4d -- --trajectory circle --mode 1 --kt 0.1 --laps 3
#   -> circle_mode1_kt0.1_laps3.csv   (26 pieces, 15.44 s, starts and ends at rest)
ros2 run crazyflie_examples formation_flight -- --trajectory circle --mode 1 --kt 0.1 --laps 3 ...
```

Concatenation happens **before** the rest-to-rest wrap, so the ramps end up on the outside of the
whole stack — `ramp-up → N laps → ramp-down` — exactly the Mode D shape. Repeating is a plain
segment copy: coefficients are normalised to τ∈[0,1] per segment with the duration carried
alongside, so there is no re-fit and no QP re-solve. Each lap seam *is* the core's own wraparound
junction, which the planner already made C3.

Measured on the generated files — lap seams are indistinguishable from ordinary junctions:

| | pieces | max junction jump (pos / vel) | start–end \|v\| |
|---|---|---|---|
| circle ×3 | 26 | 6.8e-7 m / 1.4e-5 m/s | 0.0000 → 0.0000 m/s |
| oval ×2 | 26 | 3.0e-6 m / 1.2e-5 m/s | 0.0000 → 0.0001 m/s |
| figure8 ×3 | 30 | 9.1e-7 m / 7.8e-6 m/s | 1.0586 → 1.0586 m/s |

**Only closed loops can be repeated.** The exporter checks the end-vs-start mismatch and refuses
otherwise, because repeating an open path would jump the reference from the end back to the
start. The separation is unambiguous:

| Repeatable (≈1e-7 m) | Refused (mismatch) |
|---|---|
| figure8, circle, oval, tilted_oval, helix, teardrop, teardrop_wide | slalom (4.4 m), roller_coaster (3.2 m), loop_train (1.4 m), corner (1.3 m), corkscrew (1.0 m) |

The 31-piece cap bounds the lap count; exceeding it is a clear error naming the limit:

| Trajectory | pieces/lap | max laps |
|---|---|---|
| figure8 | 10 (no ramps) | 3 |
| circle | 8 core + 2 ramps | 3 |
| oval / tilted_oval | 12 core + 2 ramps | 2 |

`--laps` is HLC-only; passing it with `--onboard` prints a note and is ignored, since Mode D
already repeats the core in firmware.

### `--laps` vs `--reps`

| | Meaning |
|---|---|
| `--laps N` (export + flight script) | **N continuous core laps in one flight**: ramp-up → N laps → ramp-down, one `startTrajectory` |
| `--reps N` (flight script) | **N separate runs**: full ramp-up/lap/ramp-down each time, with a pause. What the protocol's "repeat ≥ 5 times" asks for |

## Controller equivalence

The geometric and INDI laws are **identical between modes** — there is one `controller_step()`
call site (`firmware_app/src/lib.rs`), reached by both branches with the same gains, filters and
INDI state. Only the *source* of the reference differs. Full feedforward reaches the controller
in both modes because this firmware fork plumbs jerk and snap through `traj_eval`, `setpoint_t`
and `crtp_commander_high_level.c` (stock Crazyswarm2 cannot do this).

### Measured, as shipped

Replaying both reference pipelines over `figure8`, `circle`, `oval`, `tilted_oval`:

| Signal | Conversion only (same convention) | As shipped (D=Faessler, E=Mellinger) |
|---|---|---|
| position | 0.000008 m | 0.000008 m |
| velocity | 0.000075 m/s | 0.000075 m/s |
| acceleration | 0.000397 m/s² | 0.000397 m/s² |
| jerk | 0.0019 m/s³ | 0.0019 m/s³ |
| snap | 0.032 m/s⁴ | 0.032 m/s⁴ |
| **ω_d** | 0.00023 rad/s | **0.166 rad/s (2.1 %)** |
| **α_des** | 0.0049 rad/s² | **2.68 rad/s² (1.3 %)** |

Left column: the degree-8 → degree-7 CSV conversion is **lossless** for these trajectories.
Right column: the trajectory is identical, and the only remaining difference is the deliberate
frame-convention split — Mode D pinned to Faessler, Mode E on Mellinger.

**So Mode D and Mode E are no longer bit-identical, by design.** Expect ~1–2 % difference in
attitude feedforward when comparing them. That is far below the ~2.5 cm tracking noise floor, but
it means a Mode D vs Mode E flight comparison is a *behavioural* check, not an exact one.

## Two fixes made for Mode E (2026-08-19)

**1. Rest-to-rest wrap in the HLC export.** `wrap_rest_to_rest` previously ran only in the
`--onboard` branch. `startTrajectory(relative=True)` shifts *position* only, so periodic
trajectories were handed a velocity step at t=0 (measured: circle 1.00 m/s, oval 1.19 m/s,
tilted_oval 1.20 m/s). Now applied in both branches. Non-periodic output is byte-identical;
the core lap is untouched (max difference 0.000e+00 m). Escape hatch: `--no-rest-wrap`.

**2. `indi_gains.omega_src` (default 0 = unchanged).** The HLC builds its body frame
Mellinger-style (`y_B = z_B × x_C`); our controller uses Faessler-style (`x_B = y_C × z_B`).
They agree only at axis-aligned tilt, so in Mode E the setpoint's `attitudeRate` is inconsistent
with the `R_d` the controller derives itself (0.04–0.22 rad/s on level trajectories). Mode D
never had this because it computes ω_d locally.

- `omega_src = 0` — use `setpoint.attitudeRate` (default)
- `omega_src = 1` — recompute via `omega_desired()` from the HLC's jerk

Runtime-switchable, so the two can be A/B'd back-to-back without reflashing. Falls back to the
setpoint rate when jerk is negligible, so `cmdFullState` callers (hover keepalive, takeoff/land
streaming, legacy Mode B binaries) are unaffected under either setting. Largely superseded by
`frame_conv` below — with `frame_conv = 1` the local and HLC values agree, so `omega_src` no
longer changes anything.

**3. `indi_gains.frame_conv` (default 1 = Mellinger) — Mode E only; Mode D is pinned.**
z_B is unambiguous (the thrust direction); the only choice is where yaw sits around it, and the
two standard constructions agree only at axis-aligned tilt.

| Component | Convention |
|---|---|
| `desired_rot()` → R_d | Mellinger (always was) |
| official `controller_lee.c` — R_d **and** ω_des | Mellinger |
| official `controller_mellinger.c`, HLC `pptraj.c` | Mellinger |
| `omega_desired()` / `alpha_desired()` **before 2026-08-22** | **Faessler** (App. A) |

So the controller was internally **mixed**: R_d in one frame, ω_d/α_des in another. This was
never intentional — `desired_rot` came from the Lee/Mellinger lineage while ω_d/α_des were
written from Faessler et al. 2018. `controller_lee.c` derives its `omega_des` from the same
`xdes`/`ydes` it used for R_d; ours did not.

- `frame_conv = 1` — Mellinger everywhere. R_d, ω_d and α_des share one frame, α_des is a valid
  derivative of ω_d, and the locally computed ω_d equals the HLC's. **Default.**
- `frame_conv = 0` — legacy Faessler ω_d/α_des.

ω_d feeds the `KW` damping term of **both** the geometric and INDI attitude laws; α_des feeds
the INDI snap feedforward only. Measured effect of the switch on level trajectories: ω_d
0.002–0.17 rad/s, α_des ~1–1.5 %.

**Scope — Mode D is unaffected.** `frame_conv` governs the passthrough (Mode E / HLC) branch
only. Mode D is pinned to Faessler by `FRAME_CONV_MODE_D` in `lib.rs` and ignores the parameter
entirely, so the completed INDI project's gain blocks, kt ceilings and results stay exactly
reproducible. A full Mode D flight is unchanged end to end:

| Phase | ω_d | α_des | Changed? |
|---|---|---|---|
| Trajectory (Mode D branch) | Faessler (pinned) | Faessler (pinned) | no |
| Takeoff / land (HLC passthrough) | `sp.attitudeRate`, i.e. HLC Mellinger — as before | unused: the ramp runs `ctrl_mode = 0` (geometric), which has no snap feedforward | no |

The one way to disturb Mode D would be running the takeoff/landing ramp under INDI
(`ctrl_mode >= 2`); `flight.py` hardcodes `_RAMP_CTRL_MODE = 0`, so this does not arise.

## Running

```bash
# Mode E — official path (recommended for formation work)
cargo run --release --bin export_poly4d -- --trajectory figure8 --mode 1 --kt 0.05
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.05

# Mode D — legacy/aggressive maneuvers (unchanged)
cargo run --release --bin export_poly4d -- --trajectory loop --mode 1 --kt 0.15 --onboard
ros2 run crazyflie_examples flight -- --trajectory loop --mode 1 --kt 0.15 --onboard
```

## Before flying Mode E

1. The 132 periodic HLC CSVs in the CS2 data dir were regenerated on 2026-08-19 with the wrap.
   Any CSV older than that still contains the velocity step — re-export before flying it.
2. Re-validation ladder (Mode E has 180 hover flights but only **1** trajectory flight on full
   INDI): hover → figure8 → circle, geometric first then INDI, each against its Mode D baseline.
   `figure8` is non-periodic and unchanged by the wrap, so it is the cleanest first comparison.

## Regression cover

`src/bin/export_poly4d.rs` tests: boundary (p,v,a,j) preservation, exactness when `c8 == 0`,
rest-to-rest after wrapping, core-untouched, and periodic-classification agreement between the
two export branches.
