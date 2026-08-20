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

## Controller equivalence

The geometric and INDI laws are **identical between modes** — there is one `controller_step()`
call site (`firmware_app/src/lib.rs`), reached by both branches with the same gains, filters and
INDI state. Only the *source* of the reference differs. Full feedforward reaches the controller
in both modes because this firmware fork plumbs jerk and snap through `traj_eval`, `setpoint_t`
and `crtp_commander_high_level.c` (stock Crazyswarm2 cannot do this).

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

- `omega_src = 0` — use `setpoint.attitudeRate` (current, default)
- `omega_src = 1` — recompute via `omega_desired()` from the HLC's jerk, matching Mode D

Runtime-switchable, so the two can be A/B'd back-to-back without reflashing. Falls back to the
setpoint rate when jerk is negligible, so `cmdFullState` callers (hover keepalive, takeoff/land
streaming, legacy Mode B binaries) are unaffected under either setting.

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
