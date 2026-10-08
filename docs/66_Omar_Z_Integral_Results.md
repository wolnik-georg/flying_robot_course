# 66 — Omar z integral investigation results (2026-10-08)

Plan: `docs/65_Omar_Z_Offset_Plan.md` (not edited). Follow-up supersedes Cursor step-1 H2/H3 wording.

---

## Corrected diagnosis

| Hypothesis | Verdict |
|------------|---------|
| **H3** IMU `acc_z` bias | **Refuted** — hardware steady `acc_z` ≈ 1.000–1.003 g; step-1 “−0.114 g → +16 cm” was **circular** (bisection target). |
| **H2** plant `kt` mismatch | **Refuted for diagnosis** — not comparable on `CrazyflieSIL` thrust chain; hardware shows `a_imu_fz ≈ a_rpm_fz ≈ 0` (INDI residual ≈ 0). |
| **H4** commanded vs delivered thrust | **Accepted** — hardware `thrustSi` ~12–15% below weight at hover; position error matches `(thrustSi/MASS − g)/kp_z`. SIL injection: **`cmd_gain`** = delivered/commanded thrust, **`rpm_plant = rpm_cmd × sqrt(cmd_gain)`**, measured RPM unchanged for INDI. |

---

## Step 1 (revised) — H4 reproduction

**Script:** `experiments/analysis/omar_z_integral_sil.py` (`h4` mode)  
**Artifact:** `experiments/analysis/out/omar_z_integral/h4_reproduce.json`

### Solo harness (direct Omar kt plant)

| cmd_gain | Mean z err [cm] | vs HW +16 cm |
|----------|-----------------|--------------|
| 1.00 | ≈ 0 | — |
| 1.10 | **+12.7** | low |
| **1.136** | **+16.8** | **match** |
| 1.14 | +17.2 | close |

### A8 + NS2 bank plant (symmetrized, `torque_c=0.0032`)

SIL path: after `executeController()`, scale **plant** RPM by `sqrt(cmd_gain)`; controller still reads **actual** plant RPM (`motors_rpm_meas`).

| cmd_gain | Mean z err [cm] | HW Omar ~+19…+22 cm |
|----------|-----------------|----------------------|
| 1.10 | +12.9 | low |
| 1.14 | +17.5 | high side of band |
| 1.17 | +20.7 | **in band** |

**Gate:** H4 reproduced in solo harness; A8 with NS2 reaches **+17…+21 cm** without retuning the bank (no claim of exact +22 at one gain).

---

## Steps 2–3 — Firmware (`kpos_iz`, default 0)

**Files changed**

- `flying_drone_stack/firmware_app/src/omar_indi_rust.rs` — runtime `g_oot5_kpos_iz`, z-only integral term, clamp when ≠0, reset `i_error_pos.z` outside position mode when ≠0.
- `flying_drone_stack/firmware_app/traj_iface.c` — `g_oot5_kpos_iz`, param `ctrlOot5.kpos_iz`, `omar_indi_rust_set_kpos_iz()`.
- `~/Desktop/crazyflie-firmware/.../controller_omar_indi.c` — clamp/reset when `Kpos_I.z ≠ 0` (z-only `i_term`).
- `cffirmware.i` + `cffirmware_bindings.patch` — `omar_indi_rust_set_kpos_iz`.
- `host/_omar_indi_rust_case_runner.py` — optional `kpos_iz` in cases.

**Rebuild**

```bash
cd flying_drone_stack/firmware_app && DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" \
  cargo build --release --target x86_64-unknown-linux-gnu
cd ~/Desktop/crazyflie-firmware && rm -f build/_cffirmware*.so && make bindings_python
```

**Default-off regression (after build)**

| Test | Result |
|------|--------|
| `test_omar_indi_rust_vs_c.py` @ `kpos_iz=0` | **7/7 PASS**, worst **7.15e-07** |
| Same @ `kpos_iz=0.5` | **7/7 PASS**, worst **7.15e-07** (≤ **1e-6**) |
| INDI replay CSV @ Iz=0 | **Not re-run** (replay uses logged thrust series; controller change is no-op at Iz=0) |

---

## Step 4 — SIL grid (`cmd_gain=1.14`, NS2 plant on A8/A1)

**Artifact:** `experiments/analysis/out/omar_z_integral/grid_summary.json`

| Scenario | `Kpos_Iz=0` mean z [cm] | `Kpos_Iz=1.0` mean z [cm] |
|----------|-------------------------|----------------------------|
| Solo hover | +17.2 | **+3.3** → tune **+1.1** with longer settle / **+1.2** @ Iz=1.5 solo |
| A8 + NS2 | +17.5 | **+1.1** |
| A1 + NS2 | — | **+1.6** |

**Robustness (`Kpos_Iz=1.0`, A8 mean z):** 1.10 → **+0.8**, 1.14 → **+1.1**, 1.17 → **+1.3** cm.

**Oscillation:** A8 gyro RMS @ cg=1.14: **38.3** (Iz=0) vs **38.2** (Iz=1.0) — no >20% increase.

**Crossing dips (A8, cg=1.14): NOT VALIDATED.** The stored `crossing_dip_a8_cg1.14_cm` numbers (17.47 / 2.23) are the *mean z error* (+17.5 → +2.2 cm; sign flipped in the earlier text), not per-crossing dips; with a +17 cm offset the 'min e_z per crossing' metric is meaningless. Dips relative to the steady level still have to be computed.

**Matched plant:** `cmd_gain=1.0`, `Kpos_Iz=1.0` solo → **+1.2 cm** (integral does not blow up on nominal plant).

**Recommended lab/SIL starting gain:** **`ctrlOot5.kpos_iz = 1.0`** (yaml) / **`ctrlOmarIndi.Kpos_Iz = 1.0`**. Consider **1.5** if solo hover still >3 cm after full takeoff segment check.

**Figure-8:** not in grid (script stub only); extend `omar_z_integral_sil.py` with `oot5_bounded_gain_sweep` trajectory if needed before lab.

**Takeoff/landing windup:** low-thrust / non-position-mode resets active when Iz≠0; **not** exercised in a dedicated climb/land episode here.

---

## Limits

- SIL H4 is a **delivered-thrust scaler**, not a identified ESC/PWM model.
- NS2 plant is **docs/62 bank + symmetrized torque**; not re-calibrated in this task.
- Hardware Rust A1 **−11 cm** not reproduced in this harness.

---

## Proposed lab sequence (after NS2 gate, `docs/58` abort rules)

1. **Omar exact** (`kpos_iz=0`) INDI hover @ 1 m — confirm H4-class offset still present.
2. **Omar + Iz** @ **1.0** — same hover, target |mean z err| < 3 cm.
3. **A8** then **A1** with Iz=1.0; abort on partner loss, tilt, or gyro RMS spike per `docs/58`.

---

## Commands

```bash
# H4 table
python3 experiments/analysis/omar_z_integral_sil.py h4

# Parity
python3 flying_drone_stack/firmware_app/host/test_omar_indi_rust_vs_c.py ~/Desktop/crazyflie-firmware/build
```


## Independent validation (Claude, 2026-10-08)
- **Default-off regression: bit-identical.** New build, Omar C and Omar Rust replayed on the A8 inputs (26 060 ticks) vs the committed pre-change replay CSVs: max |diff| = 0 for thrust and all three torques (both controllers). Rust-vs-C test 7/7, worst 7.15e-07 (float32 rounding at thrust ≈ 1; torque deltas ≤ 4e-9).
- **Code reviewed:** Rust: gain from `g_oot5_kpos_iz` (default 0), z-only, clamp `KPOS_I_LIMIT`, reset when not in position mode, x/y path unchanged (`KPOS_I` const = 0). C: same behaviour gated on `Kpos_I.z != 0`.
- **Defect found and fixed:** `flying_drone_stack/firmware_patches/controller_omar_indi.c.patch` had been overwritten with an EMPTY file (the C file is untracked in crazyflie-firmware, so the regeneration produced nothing). Regenerated from the source (`git diff --no-index`); it differs from the committed version only by the `Kpos_Iz` hunks, applies cleanly to an empty tree and reproduces the C file byte-for-byte.
- **Acceptance statement corrected:** at `Kpos_Iz = 1.0` solo hover is **+3.26 cm** (outside ±3 cm, runs 18 s); A8 +1.1 and A1 +1.6 are inside. `1.5` gives solo +1.24 cm. Matched plant (cmd_gain 1.0) at Iz = 1.0 leaves +1.2 cm (takeoff-transient integral, i.e. the windup question is real but small).
- **Closed afterwards by Claude (section below):** ARM build, windup/landing, real figure-8, A1 baseline, both controllers, hardware dips. Still open: the SIL does not reproduce Omar's hardware crossing dips; the cmd_gain ≈ 1.14 mechanism is inferred from logs, not traced to a firmware constant.


## Closure checks (Claude, 2026-10-08, second pass) — everything below was run by Claude
**1. ARM firmware (CF21BL) builds and links.** `make` in a scratch copy of `firmware_app` (so neither `firmware_app/build` nor the host bindings in `crazyflie-firmware/build` were touched): link OK, flash 402 372/1 032 192 B (39 %), RAM 75 %, CCM 85 %; ELF contains `g_oot5_kpos_iz` and `controllerOmarIndi`; the binary contains the parameter names `kpos_iz` (group `ctrlOot5`) and `Kpos_Iz` (group `ctrlOmarIndi`). Rust ARM library already up to date (`cargo build --release --target thumbv7em-none-eabihf`).
**2. Solo flights with takeoff, hold and landing (no downwash), `omar_z_integral_validate.py`, both controllers identical to the printed digits.** Hold-window z error [cm] (climb overshoot [cm] in brackets):
| cmd_gain | Iz 0 | 0.5 | 1.0 | 1.5 | 2.0 |
|---|---|---|---|---|---|
| 1.00 (matched) | −0.01 (13.5) | −0.04 (14.2) | −0.07 (14.9) | −0.07 (15.6) | −0.05 (16.2) |
| 1.14 | +17.2 (29.2) | +7.26 (28.7) | +2.84 (28.2) | +1.00 (27.8) | +0.31 (27.3) |
Windup cost on a matched plant: +1.4 cm climb overshoot at Iz 1.0, +2.1 cm at 1.5, +2.7 cm at 2.0. Landing (3 s ramp): matched plant touchdown speed −0.39 (Iz 0) … −0.43 m/s (Iz 2.0); at cmd_gain 1.14 and Iz 0 the drone does not reach the ground in the ramp (stays 12.6 cm up), with Iz ≥ 0.5 it lands at −0.26…−0.39 m/s. No windup problem found.
**3. Real figure-8 (2 laps, no downwash; Cursor's version was a hover stub).** xy error RMS 5.17 cm (matched) / 6.31 cm (cmd_gain 1.14), tilt peak 43.7° / 46.3° and gyro RMS 71.8 / 79.4 deg/s — **all identical for every Iz** (Omar-exact itself flies the 8 with ~44° peaks in the SIL). z error mean at cmd_gain 1.14: +13.9 (Iz 0) → 5.06 → 1.88 → 0.66 → 0.15 (Iz 2.0).
**4. Two-drone runs with the NS2 plant (symmetrized bank + torque 0.0032), both controllers (identical):** mean z error [cm] at cmd_gain 1.14: A8 17.5 / 4.46 / 1.13 / 0.32 / 0.08 and A1 17.5 / 5.32 / 1.59 / 0.47 / 0.13 for Iz 0 / 0.5 / 1.0 / 1.5 / 2.0; robustness at Iz 1.0 / 1.5: cmd_gain 1.10 → A8 0.82 / 0.23, A1 1.19 / 0.36; cmd_gain 1.17 → A8 1.27 / 0.35, A1 1.88 / 0.57. Gyro RMS A8 38.3 → 38.2 (Iz 1.0) → 38.1 (Iz 2.0); A1 ≈ 9. A1 at cmd_gain 1.0: −0.01 (Iz 0) vs +0.02 (Iz 1.5), i.e. no harm on a matched plant. **A8 with the matched thrust (cmd_gain 1.0) and the NS2 plant falls to the ground for Iz 0 and Iz 1.5 alike** (SIL property, independent of the integral; the real hardware flew with the extra ≈ 14 % delivered thrust).
**5. Crossing dips — important limit.** The SIL gives **no** dip for Omar relative to its own steady level (0.0 cm at Iz 0, a small positive integral transient with Iz), but the **hardware** Omar flights (10-02, A8, cmd z 0.5 m) show dips of **−11, −16, −15, −20 / −11, −21, −19, −24 cm (Omar C, 18-23-23 / 18-24-57) and −7, −16, −24, −26 / −13, −1, −23, −24 cm (Omar Rust)** relative to their steady offset of +22…+25 cm (ours with the NS2-validated plant: −6 cm). So the SIL does not capture Omar's response to the crossing, and **removing the offset with the integral will not remove those dips**: expect crossings ≈ 15–25 cm below the setpoint for Omar + Iz on hardware, unlike ours. This matters for the variant decision.
**6. Recommended gain.** `kpos_iz = 1.5` (|mean z| ≤ 1.0 cm in solo, fig-8, A8, A1 for cmd_gain 1.10–1.17, no harm on the matched plant, overshoot +2 cm); `1.0` as the cautious first step. Lab parameter names: `ctrlOot5.kpos_iz` (controller 10) and `ctrlOmarIndi.Kpos_Iz` (controller 9).

**7. Regression found and fixed during closure.** The host bindings had been rebuilt without `--features residual_nn` (broke all network-on NS2 SIL runs and `test_residual_nn.py`). Rebuilt with the documented recipe (see `docs/62` host-binding note); afterwards `test_residual_nn.py` passes, Omar Rust-vs-C 7/7 (worst 7.15e-07), Omar C and Omar Rust replays on the A8 inputs are bit-identical to the committed pre-change CSVs (26 060 ticks, thrust and 3 torques), `omar_indi_rust_set_kpos_iz` is exposed, NS2 cohorts re-run (docs/62).
