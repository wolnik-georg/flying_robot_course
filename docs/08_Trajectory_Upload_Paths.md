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

## Known limitation — `--reps` means something different in each mode

Mode D loops **only the core lap**: the firmware knows which segments are the entry/exit ramps
(`n_entry`/`n_exit` in the `.meta.json` sidecar) and wraps time within the core, giving
`ramp-up → N continuous laps → ramp-down` as one uninterrupted flight.

Mode E has no such concept — `startTrajectory` replays the *whole* uploaded trajectory, so
`--reps N` gives `(ramp-up → 1 lap → ramp-down)` repeated N times, with a pause between. Fine for
"repeat the run ≥5 times" as the experimental protocol asks, but it is **not** a way to get one
long continuous formation flight.

To fly continuous laps on Mode E the trajectory must be exported with the laps baked in. That is
not implemented (`export_poly4d` has no `--laps`), and the 31-piece cap bounds how far it could
go:

| Trajectory | pieces | max laps in one upload |
|---|---|---|
| figure8 | 10 (non-periodic) | 3 |
| circle | 8 core + 2 ramps | 3 |
| oval / tilted_oval | 12 core + 2 ramps | 2 |

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
