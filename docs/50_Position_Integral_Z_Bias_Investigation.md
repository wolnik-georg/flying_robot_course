# Position integral (KI_P) — Z bias on real C.1 logs (2026-09-26)

**Scope:** Desk-only. Whether enabling the existing joint XY+Z position integral
(`ENABLE_POSITION_INTEGRAL`, `KI_P = 0.05`, `KI_LIMIT = 2.0` in `lib.rs`) would fix a
**steady-state Z tracking bias** seen in merged flight logs.

**Not the same investigation as** `docs/09_Simulation.md` § “KI_P — refuted” /
`docs/07` History (7), 2026-08-22: that time-boxed study asked whether integral explains
the **sim-vs-hardware XY stability-wall gap** (bit-identical with integral off in that
fixture). This pass asks whether **real hardware logs** show a **repeatable Z offset**
vs firmware `ctrltarget_z`.

**Attitude integral** (`ENABLE_ATTITUDE_INTEGRAL`) — out of scope (known pre-takeoff
windup risk).

---

## Task 1 — Flight data (merged C.1 CSVs)

**Method:** `experiments/analysis/position_integral_z_bias.py` on
`c1_2026-09-21_merged/` and `c1_2026-09-23_merged/` manifests. Error =
`z − ctrltarget_z` (logged position vs logged setpoint). Phases: **scenario + approach\***
only (excludes ramp/land). Summary artifact:
`experiments/analysis/out/position_integral_z_bias_2026-09-26.json`.

### Finding — **partial** (consistent sign, scenario-dependent magnitude)

| Cohort | n flights (cf5, scenario) | Mean of per-flight mean Z error | Std across flights |
|--------|--------------------------:|--------------------------------:|-------------------:|
| All merged C.1 | 27 | **−0.071 m** | **0.057 m** |
| Calm subset (&#124;vz&#124;&lt;0.05, &#124;d cmd_z/dt&#124;&lt;0.02) | 27 | **−0.068 m** | **0.059 m** |

**Sign:** **27/27** flights have mean Z error **&lt; −1 cm** (estimate **below** setpoint).
**None** above +1 cm. Same-sign fraction = **100%** — not scatter around zero.

**Magnitude is not one number:**

| Scenario (cf5, scenario phase) | Typical mean Z error |
|--------------------------------|---------------------:|
| **C5** solo hovers (6× 23 Sep) | **−0.020 … −0.025 m** (~2 cm), RMSE ≈ same |
| **A3** (4×) | **−0.032 … −0.037 m** (~3–4 cm) |
| **A2** circle (2×) | **−0.060 m** (~6 cm); RMSE ~8.5 cm |
| **A7** (3×) | **−0.096 … −0.099 m** (~10 cm) |
| **A1** hold-under-top (4×) | **−0.163 … −0.199 m** (~16–20 cm) |

So there **is** a repeatable **“fly low vs commanded z”** signature on **cf5**, but the
**size scales with scenario** (solo hover smallest; A1 stacked largest). That pattern fits
** sustained disturbance / geometry** (downwash, formation) at least as well as a fixed
controller bias integrator would remove everywhere.

### Flagged cross-checks

1. **A2 `top_align_rms_high` (~28 cm)** (`manifest_2026-09-23_c1.json`): **merge
   time-alignment RMS** on **cf_second**, not Z tracking error on cf5. For the same
   flights, **cf5** mean Z error ≈ **−6 cm** — do **not** read 28 cm as altitude bias.

2. **24 Sep stack attempts** (`docs/lab_sessions/2026-09-24_to_26.md`,
   `experiments/analysis/out/mocap_pose_proof_2026-09-24_batch.txt`): snapshot hovers
   often show **estimate z ≈ 0.73–0.86 m** vs **ctrltarget_z ≈ 1.00 m** (mean
   &#124;Δz&#124; ~**0.13–0.21 m**), with **small** per-step estimate jumps (not
   teleport-class). That is the **same sign** as merged **A1**, but these logs are
   **failed / aborted stack work** (geofence-height context, instability day) — **whole-file
   means**, not isolated steady-state holds. Treat as **consistent with “flies low under
   command”**, not as a clean solo-hover integral A/B.

---

## Task 2 — SIL (integral ON vs OFF)

**Method:** `experiments/analysis/position_integral_sil_compare.py --full-suite`
(geometric `oot`, hover 1 m, np plant, stats for **t &gt; 8 s**, constant downward
`f_ext_z` after **t = 5 s**). Host rebuild ( **`lib.rs` restored to `false`** after each
build):

```bash
cd flying_drone_stack/firmware_app
DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" cargo build --release --target x86_64-unknown-linux-gnu
cd ~/Desktop/crazyflie-firmware && make bindings_python
```

**First-pass bug (fixed 2026-09-26 evening):** the script called **`make bindings_python`
only**, without the **host `cargo` step** and without **`RUSTFLAGS="-C panic=abort"`**.
`cargo` then failed (`unwinding panics are not supported without std`) and bindings
relinked a **stale** `libcf_controller_rs.a` — OFF/ON `.so` were SHA256-identical
(tooling artifact).

**Verified distinct binaries (2026-09-26):**

| Arm | SHA256 |
|-----|--------|
| OFF | `9a167c55…b1f3521` |
| ON | `6ec51b5e…fd3ae6d` |

Full table: `experiments/analysis/out/position_integral_sil_2026-09-26.json`.

**Disturbance calibration (integral OFF):** in this SIL plant, steady **|Z error|** scales
roughly linearly with **|f_ext|** (~**5 mm per 10 mN**): **−40 mN → ~20 mm** (order of
**C5** log bias), **−120 mN → ~61 mm** (**A2/A3**), **−200 mN → ~102 mm** (mid **A1**
range, not full 17 cm).

**ON vs OFF (Δ = ON − OFF mean Z error, positive = ON flies higher):**

| f_ext (N) | OFF Z err mean | ON Z err mean | Δ (mm) | Roll max (off/on) |
|----------:|---------------:|--------------:|-------:|------------------:|
| 0 | ≈ 0 mm | ≈ 0 mm | **+0.03** | 0° / 0° |
| −0.008 | **−4.06 mm** | −4.01 mm | **+0.05** | 0° / 0° |
| −0.040 | **−20.33 mm** | −20.18 mm | **+0.15** | 0° / 0° |
| −0.120 | **−60.98 mm** | −60.58 mm | **+0.39** | 0° / 0° |
| −0.200 | **−101.63 mm** | −100.99 mm | **+0.64** | 0° / 0° |

Integral **ON** trims the sag by **sub-millimetre to &lt;1 mm** across this range — far
short of closing **2–20 cm** log biases. **No** roll/pitch growth or oscillation observed
(**ROLL_MAX_ALL = 0°** throughout); **KI_LIMIT = 2.0** did not show windup pathology in
this hover fixture (integral action is **weak**, not aggressively fighting the P loop).

---

## Task 3 — Recommendation

**Do not enable `ENABLE_POSITION_INTEGRAL` on hardware now.** (Unchanged headline; now
**supported by valid SIL**, not “inconclusive”.)

- **Flight data (Task 1):** real, scenario-scaled **below-setpoint** bias — still valid.
- **SIL (Task 2):** with **working ON/OFF binaries**, existing **KI_P = 0.05 / KI_LIMIT =
  2.0** integral **does not materially reduce** steady Z error at disturbance levels that
  reproduce **cm–decimetre** sag in sim — it adds at most **~0.6 mm** improvement at
  **−200 mN**. That is **not** a credible fix for **2 cm** C5 offsets, let alone **A1**
  downwash-scale gaps.
- **Joint XY+Z** gate remains a blunt coupling; **no windup** seen here, but also **no
  benefit** at the magnitudes that matter.

**If revisited:** do **not** flip the current gate expecting log bias to disappear — would
need **much larger Z-only integral authority** (and anti-windup design), or address
formation/downwash physics directly. C.0 **solo C5** could still **confirm** flight bias
isn't already explained by this weak integral path.

---

## Artifacts

| File | Role |
|------|------|
| `experiments/analysis/position_integral_z_bias.py` | Log analysis |
| `experiments/analysis/out/position_integral_z_bias_2026-09-26.json` | Per-flight table + summary |
| `experiments/analysis/position_integral_sil_compare.py` | SIL harness (desk) |
| `experiments/analysis/out/position_integral_sil_2026-09-26.json` | SIL ON/OFF suite (distinct `.so`, disturbance sweep) |
