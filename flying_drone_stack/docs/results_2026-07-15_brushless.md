# Advanced Flying Robots — CF2.1 Brushless (CF21BL) INDI Commissioning

## Flight Experiment Results — starting 2026-07-15

**Drone:** Crazyflie 2.1 **Brushless** (CF21BL), `CONFIG_PLATFORM_CF21BL=y`, DShot bidirectional ESC telemetry
**Goal:** get full-INDI **hover** stable, then **figure-8**, on the brushless platform
**Source of truth for physics:** Busetto et al. 2025, *Nonlinear System Identification Nano-drone Benchmark* (`docs/brushless_system_identification.pdf`)
**Status:** ⛔ Full INDI not yet stable — hover detonates at the geometric→INDI switch. Root-cause hunt in progress.

---

## Platform parameters (from the paper) vs our config

| Quantity | Paper (Busetto 2025) | Our config | Match |
|----------|----------------------|-----------|-------|
| Inertia J | diag(2.3951e‑5, 2.3951e‑5, 3.2347e‑5) kg·m² | identical | ✅ |
| Arm L | 35.35 mm | ARM_M = 0.035355 m | ✅ |
| kF (thrust) | 3.72e‑8 Ns²/rad² | kt refit to eRPM (self-consistent) | ✅ |
| kM (drag) | 7.73e‑11 Nms²/rad² | — | ✅ |
| kM/kF (→ τ_z) | 0.002078 | TORQUE_RATIO = 0.002078 (= paper; firmware uses 0.00569) | ⚠️ paper ok / fw differs |
| Mass m | 45 g (with Flow + AI deck) | 41 g | ⚠️ 9% low — verify AUW |
| Controller | geometric SE(3) + integral, 500 Hz | identical | ✅ |
| ω bandwidth (Table 4) | ~18 Hz | INDI fc_bw = 30–70 Hz | ⚠️ over |
| Motor speed bandwidth | ~20 Hz | — | — |
| Motor-speed telemetry | DShot back-EMF, **documented cmd→actuation delay** | read live, no delay compensation | ⚠️ |

---

## Active configuration (as of last flight 2026-07-15 19:xx)

```yaml
# indi_gains
ctrl_mode: 3          # full INDI
kr: 100.0             # conservative (also tried 603)
kw: 30.0              # ζ=1.5 (also tried 90)
kr_z: 100.0
kw_z: 30.0
fc_bw: 30.0           # (also tried 70)
mass: 0.041           # AUW 41.0 g — paper says 45 g, verify
kt1: 4.2336e-10       # identified 2026-07-15 (15.6k eRPM hover)
kt2: 3.9865e-10
kt3: 4.0689e-10
kt4: 4.2210e-10
# pos_gains
kp_xy: 40.0
kp_z:  30.0
kv_xy: 8.0
kv_z:  10.0
```

---

## 1. Root-cause hypothesis matrix (hover crash) — **test plan for next session**

Full detail + evidence in `brushless_indi_crash_hypotheses.md`. Summary below.

### Established facts (data-backed)
- **Geometric hover stable** (roll/pitch < 5°, α std ≈ 7–8 rad/s²). **Full INDI never stable** —
  detonates to 180° within ~0.2–0.5 s of the `ctrl_mode 0→3` switch. Onset t ≈ 6.0 s = switch time, every flight.
- **Divergence only weakly gain-dependent**: kr=603 → λ≈15–17/s; kr=100 → λ≈8–14/s (6× kr cut → ~1.5–2× slower only).
- **tau_current baseline correct**: ≈0.0002 Nm level hover; roll/pitch mixer matches firmware & paper exactly.
- **kt/mass consistent** (Σthrust≈weight). DShot 0% invalid while flying (65535 sentinel only post-crash).
- **Physics params match the paper** (J, L, kF, kM). Paper flags a brushless motor-telemetry **delay**.
- **Structural asymmetry**: geometric never uses motor-speed telemetry; INDI's inner loop is built on it.
  The one uniquely-brushless subsystem (DShot back-EMF, delayed) is the one INDI depends on and geometric ignores.

### Hypotheses & experiments

| # | Hypothesis | Conf. | Experiment | Confirms if… | Result |
|---|-----------|-------|------------|--------------|--------|
| **H0** | Partition: attitude vs position loop | — | `ctrl_mode=2` then `=1` | `2` boom & `1` stable ⇒ attitude loop. **Run first** | ⬜ |
| **H1** | DShot telemetry delay misaligns `tau_current` vs `alpha_meas` | High | force `tau_current=tau_prev`; 500 Hz cmd+az log | feed-forward-free stabilizes / delay ≫1 ms | ⬜ |
| **H1b** | Optical RPM deck (like brushed) works; DShot fails | High | swap INDI+yaml to `rpm.*`, re-hover | optical-deck INDI stable ⇒ DShot is culprit | ⬜ |
| **H2** | α-chain under-filtered vs ~18 Hz ω bandwidth | Med | fc_bw DOWN: 18 / 12 / 8 Hz (runtime) | divergence slows toward ~18 Hz | ⬜ |
| **H3** | Gains too high (pure linear instability) | Low–Med | kr = 30 / 10 / 3 sweep | stabilizes at low kr ⇒ gains; boom at kr≈10 ⇒ not gains | ⬜ |
| **H4** | Inertia effectiveness overestimate (J too high) | Low | J_scale 1.0 / 0.5 / 0.33 (needs knob) | slows as J drops (J matches paper ⇒ unlikely) | ⬜ |
| **H5** | Mass 45 g paper vs 41 g config | Low | set mass=0.045; weigh drone | only if H0 implicates position | ⬜ |
| **H6** | Switch/init transient (cold filters) | Low–Med | start in `ctrl_mode=3` from takeoff | boom without switch ⇒ steady instability | ⬜ |
| **H7** | Yaw TORQUE_RATIO 0.002078 vs 0.00569 | Very low | set →0.00569 (reflash) | yaw only; won't fix roll/pitch | ⬜ |

**Recommended order:** H0 → H2 (cheapest) → H1b (strongest discriminator) → H1a → H3 → H6 → (H4/H5/H7).

**Firmware knobs still to add:** runtime `j_scale` (H4), runtime `tau_current=tau_prev` toggle (H1a).
fc_bw and ctrl_mode are already runtime params. Optical-deck path (H1b) = compile flag / yaml swap.

---

## 2. Hover results log

_(fill in as flights are run tomorrow)_

| Time | ctrl_mode | kr | kw | fc_bw | RPM src | Outcome | λ (1/s) | Notes |
|------|-----------|----|----|-------|---------|---------|---------|-------|
| 17:36 | 3 | 603 | 90 | 70 | DShot | 💥 crash | ~15 | old kt=3.16e-10 |
| 17:44 | 3 | 603 | 90 | 70 | DShot | 💥 crash | ~15 | |
| 19:11–19:22 | 3 | 100 | 30 | 30 | DShot | 💥 crash | ~8–14 | conservative gains didn't fix |

---

## 3. Figure-8 results log

_(pending stable hover — do not attempt figure-8 until hover is solid)_

| Time | kt | kr | kw | fc_bw | Phase XY RMSE | Roll err | Pitch err | Notes |
|------|----|----|----|-------|---------------|----------|-----------|-------|

---

## Reference baselines (other drones, for comparison)

- **Standard CF2.1 INDI:** 3.87 cm XY RMSE (June 20 2026, kr=1050) — `results_2026-06-20.md`
- **Upgraded CF2.1 INDI:** 3.7 cm mean XY RMSE, n=11 (July 15 2026, kr=603) — `results_2026-07-11_upgraded_drone.md`
