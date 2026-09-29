# Z-only position integral — new term (2026-09-29)

**Scope:** Desk-only. Separate **Z-only** integral on the geometric controller
(`controller_step` / `geometric_step_ref` in `lib.rs`), distinct from the existing joint
XY+Z path (`ENABLE_POSITION_INTEGRAL`, `KI_P`, `i_ep`) closed in `docs/50`.

**Status (2026-09-29):** **Implemented, default OFF.** SIL shows materially more Z
correction than the joint integral at the same disturbance sweep; still **not** a
credible full fix for **17–20 cm** A1-scale log bias without further gain/limit tuning
or addressing formation physics.

---

## Motivation

`docs/50` Task 3 concluded the joint integral adds at most **~0.6 mm** improvement at
**−200 mN** in hover SIL — useless for **2–20 cm** real Z gaps. Flight data (Task 1 there,
plus **2026-09-29 A1** geometric comparison in `docs/41` §9) show **0.17–0.20 m** mean
Z error under stacked hold at **dz = 0.30 m** command.

This pass adds a **new** accumulator `i_ez`, runtime gains `g_ki_z` / `g_ki_z_limit`
(`pos_gains` in `traj_iface.c`), and compile gate `ENABLE_Z_INTEGRAL` (**false** in
tree). Joint integral code paths are **unchanged** when `ENABLE_Z_INTEGRAL` is false.

---

## Implementation

| Item | Location |
|------|----------|
| Gate | `ENABLE_Z_INTEGRAL: bool = false` (`lib.rs`, v2 flags block) |
| State | `i_ez`, `prev_thrust_si`; reset in `State::reset()` and when `thrust < 0.05` (same as `i_ep`) |
| Gains (defaults) | `g_ki_z = 8.0`, `g_ki_z_limit = 1.5` (`traj_iface.c`) |
| Application | `KI_Z * i_ez` added to **Z** component of `f_d` only |
| Anti-windup | **Conditional integration (freeze, not zero):** no accumulation on ground (`prev_thrust_si ≤ 0.05`); when thrust clamp is active, freeze if saturated **against** the error sign (thrust at ceiling and `ep_z > 0`, or at floor and `ep_z < 0`). Chosen over back-calculation because takeoff/ground phase is the main windup risk and we need sustained authority in flight without resetting accumulated bias. |
| SWIG | `g_ki_z`, `g_ki_z_limit` in `cffirmware_bindings.patch` / live `cffirmware.i` |

**Out of scope:** duplicate joint-integral block inside INDI-only branches of
`controller_step()` — geometric / `ctrl_mode=0` path only for this change.

**Rebuild (host SIL):** `cargo build --release --target x86_64-unknown-linux-gnu` with
`DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort"`, then `make bindings_python` in
`crazyflie-firmware`. Verify **distinct** OFF/ON `.so` SHA256 before trusting A/B
(`docs/50` Task 2 gotcha).

---

## Task 1 — SIL A/B (Z integral OFF vs ON)

**Method:** `experiments/analysis/position_integral_z_only_sil_compare.py --full-suite`
(same fixture as `docs/50`: geometric `oot`, hover 1 m, np plant, stats **t > 8 s**,
constant downward `f_ext_z` after **t = 5 s**).

**Verified distinct binaries (2026-09-29):**

| Arm | SHA256 |
|-----|--------|
| OFF (`ENABLE_Z_INTEGRAL=false`) | `07d056b4569c0f868a2154b7d7e4fd8efb09801fc5f60876c9a53681a49eb234` |
| ON (`ENABLE_Z_INTEGRAL=true`, script patch) | `60d0a57119542608fd6baa510155ed2ec1400be251850efca1097d0d55910588` |

Full table: `experiments/analysis/out/position_integral_z_only_sil_2026-09-29.json`.

**ON vs OFF (Δ mean Z error = ON − OFF; positive = ON flies higher):**

| f_ext (N) | OFF Z err mean | ON Z err mean | Δ (mm) | Joint integral Δ (`docs/50`) |
|----------:|---------------:|--------------:|-------:|-----------------------------:|
| 0 | ≈ 0 | +0.92 | +0.92 | +0.03 |
| −0.008 | −4.06 | −0.67 | **+3.39** | +0.05 |
| −0.040 | −20.33 | −7.02 | **+13.30** | +0.15 |
| −0.120 | −60.98 | −22.90 | **+38.08** | +0.39 |
| −0.200 | −101.63 | −38.77 | **+62.86** | +0.64 |

At **−200 mN**, the Z-only term trims sag by **~63 mm** vs **~0.6 mm** for the joint
integral — **two orders of magnitude** more authority in this plant. **ROLL_MAX_ALL = 0°**
throughout (no attitude blow-up in this hover fixture).

With flag **OFF**, `lib.rs` is restored to `ENABLE_Z_INTEGRAL=false` after the suite;
default-tree behavior matches pre-change (zero `z_integral_force_term`).

---

## Task 2 — Real-data sizing sanity

**Question:** Do default `ki_z = 8`, `ki_z_limit = 1.5` plausibly scale to **17–20 cm**
A1 bias?

- **Max steady integral force** (if limit saturated): `8 × 1.5 = 12 N` — nominally
  enough to lift a ~42 g platform; the **observed** SIL correction at −200 mN stops at
  **~63 mm** improvement with **~39 mm** residual error, so the loop is **not** hitting
  the force cap alone — plant/PD balance and anti-windup during thrust transients limit
  how much bias is cancelled in **10 s** hover stats.
- **Mapping log bias to SIL:** `docs/50` notes **−200 mN** ≈ **~102 mm** OFF error in
  this plant (mid **A1** range, not full **17 cm**). Real **A1** **0.17–0.20 m** bias
  is **larger** than that SIL point; closing it would require **either** stronger
  effective integral action (higher `ki_z` / `ki_z_limit`, longer settle, or relaxed
  freeze rules after takeoff) **or** accepting that stacked **downwash / setpoint
  geometry** is not a constant **f_ext** in the np hover model.
- **Plain verdict:** This term is **worth SIL/hardware A/B** relative to the joint
  integral, but **default gains do not credibly close 17–20 cm** A1 gaps on back-of-envelope
  scaling — expect **partial** trim (order **few–6 cm** at disturbance levels tested),
  not full elimination of formation-scale error.

---

## Task 3 — Recommendation

**Do not enable `ENABLE_Z_INTEGRAL` on hardware yet** (compile flag remains **false**).

**Next steps if pursued:**

1. C.0 **solo C5** hover with flag on + param tune — confirm no takeoff windup with
   conditional integration.
2. **A1** geometric re-fly with integral on — compare mean `z − ctrltarget_z` to
   **2026-09-29** **0.17–0.20 m** baseline.
3. If still short, tune **`ki_z` / `ki_z_limit`** via `pos_gains` (not joint `KI_P`).

---

## Artifacts

| File | Role |
|------|------|
| `experiments/analysis/position_integral_z_only_sil_compare.py` | SIL harness (Z-only ON/OFF) |
| `experiments/analysis/out/position_integral_z_only_sil_2026-09-29.json` | Distinct `.so` + disturbance sweep |
| `flying_drone_stack/firmware_app/src/lib.rs` | `i_ez`, anti-windup, `ENABLE_Z_INTEGRAL` |
| `flying_drone_stack/firmware_app/traj_iface.c` | `g_ki_z`, `g_ki_z_limit` |
| `flying_drone_stack/firmware_app/host/cffirmware_bindings.patch` | SWIG externs |

**Related (controller=10 telemetry triage, not this integral):**
`experiments/analysis/check_omar_rust_telemetry.py`.
