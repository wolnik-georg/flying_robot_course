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

**Method:** `experiments/analysis/position_integral_sil_compare.py` + desk rebuild of
host `cffirmware` with `ENABLE_POSITION_INTEGRAL` toggled ( **`lib.rs` restored to
`false`** after runs; **not** a flight config change).

**Caveat:** Host `cargo build --target x86_64-unknown-linux-gnu` for
`libcf_controller_rs.a` **failed** in this environment (`unwinding panics are not
supported without std`). `make bindings_python` therefore **relinked an unchanged
archive** — OFF vs ON `.so` files were **SHA256-identical**. Reported ON/OFF metrics
match for that reason, not because the compiler proved the branch dead.

**Observed (identical binaries, geometric `oot`, hover 1 m, np plant):**

| Condition | Z error mean (t &gt; 8 s) | Roll RMS |
|-----------|-------------------------:|---------:|
| Nominal (no disturbance) | ≈ **0 mm** | **0°** |
| Constant **−8 mN** after t = 5 s | ≈ **−4.1 mm** | **0°** |

No takeoff/landing **instability** or attitude growth in this script; **cannot** claim
integral **helps or hurts** until a successful host Rust rebuild produces **distinct**
ON/OFF binaries. (Consistent with **08-22** “bit-identical” in the *other* SIL fixture.)

---

## Task 3 — Recommendation

**Do not enable `ENABLE_POSITION_INTEGRAL` on hardware now.**

- **Partial bias:** Logs show a **real, same-sign** “below ctrltarget” tendency on **cf5**,
  but **not** a single ~cm bias an integrator would uniformly erase (**2 cm** solo C5 vs
  **~17 cm** A1).
- **A1-scale offsets** are likely **downwash / formation physics**, not a missing **2 cm**
  trim — turning on **XY+Z** integral at **KI_P = 0.05** is a blunt tool and couples axes.
- **SIL A/B inconclusive** here due to stale host link; any future test needs a verified
  ON/OFF binary diff **plus** disturbances matched to log-scale biases (cm–decimeter), not
  only the stability-wall setup in `docs/09`.

**If revisited after lab unblock:** C.0-gated **solo C5 hover** first (smallest, most
integral-like offset); compare `rnn.en=0` logs before touching integral; consider **Z-only
or anti-windup** design rather than flipping the existing joint gate.

---

## Artifacts

| File | Role |
|------|------|
| `experiments/analysis/position_integral_z_bias.py` | Log analysis |
| `experiments/analysis/out/position_integral_z_bias_2026-09-26.json` | Per-flight table + summary |
| `experiments/analysis/position_integral_sil_compare.py` | SIL harness (desk) |
| `experiments/analysis/out/position_integral_sil_2026-09-26.json` | SIL metrics (stale-.a caveat) |
