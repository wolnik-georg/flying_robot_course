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
| **H0** | Partition: attitude vs position loop | — | `ctrl_mode=2` then `=1` | `2` boom & `1` stable ⇒ attitude loop. **Run first** | ✅ **2026-07-16 CONFIRMED attitude loop** — cm=1 stable (2.3°), cm=2 & 3 blow up |
| **H1** | DShot telemetry delay misaligns `tau_current` vs `alpha_meas` | High | force `tau_current=tau_prev`; 500 Hz cmd+az log | feed-forward-free stabilizes / delay ≫1 ms | ⬜ |
| **H1b** | Optical RPM deck (like brushed) works; DShot fails | High | swap INDI+yaml to `rpm.*`, re-hover | optical-deck INDI stable ⇒ DShot is culprit | ✅ **2026-07-16 CONFIRMED — THE FIX.** Optical deck → INDI hover STABLE (7 Hz cycle gone). DShot telemetry delay was the culprit. Required: `app-config-bl CONFIG_DECK_FORCE="bcRpm"` (deck has vid/pid=0, no auto-detect) + `traj_iface.c`→`rpm.*` + `rm build/.config` rebuild |
| **H2** | α-chain under-filtered vs ~18 Hz ω bandwidth | Med | fc_bw DOWN: 18 / 12 / 8 Hz (runtime) | divergence slows toward ~18 Hz | ❌ **2026-07-16 REFUTED as primary** — tau_x osc freq PINNED at 6.6–7.9 Hz across fc_bw 70→10. Filter is not the dominant lag |
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
| 07-15 17:36 | 3 | 603 | 90 | 70 | DShot | 💥 crash | ~15 | old kt=3.16e-10 |
| 07-15 17:44 | 3 | 603 | 90 | 70 | DShot | 💥 crash | ~15 | |
| 07-15 19:11–19:22 | 3 | 100 | 30 | 30 | DShot | 💥 crash | ~8–14 | conservative gains didn't fix |
| 07-16 17:12 | **1** | 10 | 3 | 30 | DShot | ✅ **stable 2.3°** | — | **position INDI only — clean (H0)** |
| 07-16 17:13 | **2** | 10 | 3 | 30 | DShot | 🌀 osc→46° | — | attitude INDI only — 6.0 Hz α limit cycle |
| 07-16 17:15 | 3 | 10 | 3 | 30 | DShot | 🌀 osc→43° | — | 8.2 Hz α limit cycle |
| 07-16 17:17 | 3 | **5** | **1.5** | 30 | DShot | 🌀 **bounded osc ~30°** | — | 7.1 Hz α limit cycle; amplitude ↓ but NOT gone |

### 2026-07-16 key finding — attitude INDI limit cycle
- **H0 confirmed**: cm=1 (position INDI) stable; cm=2 (attitude INDI) is the failure.
- The blow-up is a **limit cycle in the α / τ chain at 6–8 Hz** (roll/pitch body slosh at 0.3–0.9 Hz is its envelope).
- **Oscillation frequency is ~gain-independent**: kr=10→8.2 Hz, kr=5→7.1 Hz (√2 gain change ⇒ only 1.17× freq change).
  A pure kr-set oscillation would scale as √kr. ⇒ **frequency is set by loop phase lag, not the gain.**
- Lowering kr/kw 10→5 cut amplitude (46°→30°, divergence→bounded) but did **not** remove the cycle.
- **Interpretation:** the attitude *incremental* loop `τ = τ_current + J·(α_ref_filt − α_meas)` is phase-lag limited.
  A ~7 Hz limit cycle ⇒ ~70 ms half-period of loop lag — consistent with **brushless motor/prop spin-up time
  constant (real rotor inertia, much slower than brushed coreless) + DShot telemetry delay** (H1), NOT just gains.
- **Consequence:** gain reduction alone → progressively smaller but sluggish, still-oscillating hover. Crisp INDI
  needs the *phase lag* addressed (fewer/faster α filters, or motor-lag/telemetry-delay compensation).
- **⚠️ fc_bw direction caution:** lowering the cutoff = MORE phase lag → likely makes the 6–8 Hz cycle **worse**,
  not better. Test: sweep fc_bw and watch whether the limit-cycle frequency moves.
  - freq moves with fc_bw ⇒ filter is inside the limit-cycle loop (can't filter your way out).
  - freq stays ~7 Hz regardless of fc_bw ⇒ it's actuator/telemetry delay (H1) → needs delay compensation.
- **Most effective gain knob at this frequency = KW (rate damping)** — sweep kw independently of kr.

### 2026-07-16 (b) — gyro pipeline is identical; sensor noise is NOT the difference
- **fc_bw matters enormously on brushless, ~irrelevant on brushed.** Higher fc_bw (70>60>30) = worse
  (drifts/climbs, faster onset); lower = bounded. On brushed drones fc_bw 50–80 barely changed RMSE.
- **Gyro pipeline is byte-identical on both platforms** (firmware verified): both use
  `CONFIG_SENSORS_BMI088_BMP3XX` → BMI088 gyro, HW **116 Hz BW @ 1000 Hz ODR**, then a **2nd-order SW LPF
  at 80 Hz** (`GYRO_LPF_CUTOFF_FREQ`), applied unconditionally, feeding both EKF and controller. Not raw,
  no `#ifdef cf21bl`. So the OOT controller gets the *same* gyro processing on every drone.
- **Sensor noise is comparable** (measured, geometric-phase alp_raw std): brushless ≈ 3–10 rad/s²,
  brushed ≈ 5–6 rad/s²; HF fraction >20 Hz ≈ 0.4–0.6 both. ⇒ the brushless is **not** meaningfully noisier
  at the gyro. Rules out "raw/noisier sensor" as the cause.
- **Correct interpretation:** the brushless INDI incremental loop runs with **far less stability margin**.
  fc_bw sets the loop's HF gain/phase; on the brushed drone the loop had comfortable margin (fc_bw
  irrelevant), on the brushless it sits near the stability limit so fc_bw is make-or-break. Higher fc_bw =
  more HF loop gain = tips to growth. The lost margin is the extra loop lag unique to the brushless:
  **heavier brushless motor+prop → slower thrust response than brushed coreless** (± DShot telemetry delay).
- **Hidden knob noted:** the firmware 80 Hz gyro SW-LPF sits *before* the INDI differentiation and is the
  same on both — could be lowered for the brushless, but the data says raw noise isn't the bottleneck.
- **Direction:** fc_bw **DOWN** from 30 (20, 15, 10) + lower kr/kw; the durable fix is actuator-lag
  compensation (align `tau_current` phase / motor model) — confirm the lag source via optical deck (H1b).

### 2026-07-16 (c) — fc_bw sweep: limit cycle is PLANT-lag limited (not filter, not gain)
- Swept fc_bw = 70/60/40/20/15/10 at kr=5/kw=1.5. **Commanded-torque (tau_x) oscillation frequency stayed
  6.6–7.9 Hz across the entire sweep** — does NOT track fc_bw. (Raw `alp_raw` freqs were contaminated by
  motor vibration + crash-broadband; `tau_x` is the clean loop signal.) max tilt scattered 7–67° = marginal.
- **Conclusion: cannot filter or gain your way out.** The ~7 Hz limit cycle (~71 ms half-period loop lag) is
  a fixed lag in the plant = **brushless motor+prop thrust-response dynamics** (real rotor inertia, ~tens of ms
  vs ~ms brushed coreless) ± DShot telemetry delay. INDI assumes ~instant actuation; geometric doesn't invert
  the actuator ⇒ geometric immune, INDI limit-cycles.
- **Next decisive test = split motor-mechanical-lag vs telemetry-delay:**
  - **H1a feed-forward-free** (`tau_current = tau_prev`, firmware toggle): 7 Hz persists ⇒ motor mechanical lag
    (deck won't help); 7 Hz vanishes ⇒ telemetry/tau_current path (deck would help).
  - **H1b optical RPM deck**: same discrimination, more setup.
- **Strategic note:** if it's motor mechanical lag (likely), the honest options are (1) accept a very
  low-bandwidth, sluggish INDI, (2) add a first-order actuator model to the INDI increment, or (3) **question
  whether INDI is worth it on brushless at all** — geometric already hovers at <5° and is lag-immune.

---

### 2026-07-16 (d) — ✅ STABLE INDI HOVER (optical deck). DShot delay was the root cause.
- Optical RPM deck fixed it. Root cause = **DShot bidirectional telemetry delay** desyncing `tau_current`
  from `alpha_meas` in the attitude incremental loop. Confirmed by: 7 Hz cycle pinned across fc_bw & gains
  (plant lag, not filter/gain), then eliminated purely by switching the RPM source.
- Deck RPM: balanced to 1.3%, matches DShot scale 1.00 (poles=12 correct), 0 invalid. kt ≈ unchanged (4.10e-10).
- Current stable hover (kr=603/kw=90/fc=70, deck): roll std 1.8–2.9°, pitch 1.5–3.2°, ±5–9°, gyro std 18–30°/s,
  z std 4.5 cm. Wiggles at ~1.4 Hz (slow body wobble) — now a **tuning** problem, not stability.

### Tuning ladder (in progress) — order: filters → attitude (KW,KR) → position (KV,KP) → figure-8
| Step | Knob | From | Goal / metric | Status |
|------|------|------|---------------|--------|
| 1 | fc_bw | 70 | **✅ 2026-07-16 hover sweep → fc_bw=60.** Higher is better (opposite of paper): flat optimum 50–80 (roll+pitch σ≈3.7°, gyro σ≈20°/s), cliff <40 (σ→7° at 20). With deck RPM clean, α-chain is lag-limited not noise-limited → less filtering = better. | ✅ =60 |
| 2 | KW (rate damping) | 90 | damp attitude rate (gyro std ↓), no osc | ⬜ |
| 3 | KR (att stiffness) | 603 | crisp attitude tracking, tau not chattering | ⬜ |
| 4 | KV (vel damping) | 8 | damp ~1 Hz position wobble (pos ζ≈0.63 now, underdamped) | ⬜ |
| 5 | KP (pos stiffness) | 40 | tighten xy hold | ⬜ |
| 6 | figure-8 + snap FF | — | once hover crisp | ⬜ |

## 3. Figure-8 tuning ladder (full — preserve & fill in)

**Trajectory:** figure-8, mode 1, kt=0.05 (onboard degree-8 poly). **RPM source:** optical deck.
**Metric:** `analyze_flight.py --type figure8` → phase-aligned XY RMSE (primary) + Roll/Pitch flatness err
+ along-track lag [ms]. **Baselines:** standard **3.9 cm**, upgraded **3.7 cm**.
**Fixed unless being swept:** mass 0.041, kt (deck, §2), fc_bw=60, kp_xy=40, kp_z=30, kv_xy=8, kv_z=10.

### Order: KV (hover wobble) → ATTITUDE (KR,KW) → POSITION (KP) → FILTER (fc_bw) refine-back

**Two separate problems:**
- **Hover wobble (~1.4 Hz)** = position loop underdamped (ζ_pos = KV/(2·√KP) = 8/(2·√40) = **0.63**).
  Fix: raise KV only (yaml change, no reflash). Damps hover; also helps figure-8 XY RMSE.
- **Figure-8 lag + flatness err** = attitude too slow (ζ_att = KW/(2·√KR) = 90/(2·√603) = **1.83**,
  τ_dom = 137 ms). Fix: raise KR while scaling KW to ζ ≈ 1.4 (KW = 2·1.4·√KR). ⚠️ DO NOT raise KW
  faster than KR — that increases ζ and makes attitude *slower*, not faster.

**Attitude math recap:**  ωₙ = √KR,  ζ = KW/(2·√KR),  τ_dom = 1/(|ζ - √(ζ²-1)|·ωₙ) for ζ>1.
Standard CF2.1 reference: kr=1050/kw=87 → ωₙ=32.4, ζ=1.34, τ_dom=69 ms.
Brushless motors faster → should reach or exceed that.

**Stop signals when pushing KR/KW:** gyro σ > 35 deg/s sustained, or tau_x std rising (HF chatter).

| Block | kr | kw | kp_xy | kv_xy | fc_bw | ζ_att | τ_dom | Phase XY RMSE | Geom RMSE | Roll err | Pitch err | lag ms | gyro_x σ | Notes |
|-------|----|----|-------|-------|-------|-------|-------|---------------|-----------|----------|-----------|--------|----------|-------|
| **BL-0** (baseline) | 603 | 90 | 40 | 8 | 60 | 1.83 | 137 ms | **4.3 cm** | 2.9 cm | 8.4° | 6.4° | 180 | 56 °/s | flown 18-51-38 (repeat of 18-29-14) |
| **BL-att1** ✅ | 900 | 84 | 40 | 8 | 60 | 1.40 | 80 ms | **3.9 cm** | 2.6 cm | 5.0° | 3.2° | 180 | 44 °/s | big jump: roll/pitch err −40%, gyro σ ↓ |
| **BL-att2** ✅ | 1200 | 97 | 40 | 8 | 60 | 1.40 | 71 ms | **3.9 cm** | 2.6 cm | 4.4° | 3.1° | 170 | 39 °/s | marginal improvement; gyro σ still ↓ |
| **BL-att3** ✅ | 1500 | 108 | 40 | 8 | 60 | 1.40 | 63 ms | **3.9 cm** | 2.5 cm | 4.8° | 2.7° | 170 | 44 °/s | plateau; tau_x σ uptick (+21%) |
| **BL-att4** ✅ | 1800 | 119 | 40 | 8 | 60 | 1.40 | 59 ms | **4.0 cm** | 2.7 cm | 4.1° | 2.7° | 170 | 39 °/s | RMSE flat/slight regress; attitude plateau confirmed |
| BL-5 (KV) | **1200** | **97** | 40 | **11** | 60 | — | — | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | — | ζ_pos 0.63→0.87; cut hover wobble + lag |
| BL-6 (KV↑) | 1200 | 97 | 40 | **13** | 60 | — | — | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | — | ζ_pos→1.03; critical pos damping |
| BL-7 (KP) | 1200 | 97 | **50** | *best* | 60 | — | — | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | — | tighter tracking once KV locked |
| BL-8 (filt) | 1200 | 97 | *best* | *best* | **50** | — | — | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | — | re-sweep fc_bw under dynamic load |
| BL-9 (filt) | 1200 | 97 | *best* | *best* | **70** | — | — | ⬜ | ⬜ | ⬜ | ⬜ | ⬜ | — | hover optimum was flat 50–80 |

**Stage guidance**
0. ~~**KV first (yaml-only, no reflash):** KV 8→11 to damp the ~1 Hz position wobble (ζ_pos 0.63→0.87). Fly
   one hover; if wobble persists try KV=13 (ζ≈1.0). Lock KV before touching KR/KW.~~
   **⚠️ Superseded 2026-07-18 — prediction was wrong.** Raising KV made attitude oscillation
   monotonically *worse* (roll/pitch σ 1.07/0.98°→3.53/6.32° as KV 6→13), not better. See
   §4b-results below for the full sweep and root-cause note. Locked config reverted to
   `kp_xy=40, kv_xy=8`.
1. **Attitude (KR,KW):** raise KR (603→900→1200→1500) scaling KW = 2·1.4·√KR each step. ζ drops from 1.83
   toward 1.40 → τ_dom 137→80→71→63 ms. Target: lag ↓ (from 170 ms), roll/pitch err ↓ (from 7.1°/5.5°).
   Stop signal: gyro σ climbing or tau_x chattering.
2. **Position KP:** with attitude locked at best KR/KW, raise KP (40→50) for tracking stiffness.
3. **Filter (fc_bw) refine-back:** re-sweep 50/60/70 with final gains — hover optimum (flat 50–80) may shift
   under dynamic load.

### Figure-8 results log
| Time | kr | kw | kp | kv | fc_bw | Phase XY RMSE | Geom RMSE | Roll err | Pitch err | lag ms | gyro_x σ | Notes |
|------|----|----|----|----|-------|---------------|-----------|----------|-----------|--------|----------|-------|
| 18-29-14 | 603 | 90 | 40 | 8 | 60 | 4.3 cm | 2.8 cm | 7.1° | 5.5° | 170 | — | BL-0 first baseline |
| 18-51-38 | 603 | 90 | 40 | 8 | 60 | 4.3 cm | 2.9 cm | 8.4° | 6.4° | 180 | 56 °/s | BL-0 repeat (confirms baseline) |
| 18-52-59 | 900 | 84 | 40 | 8 | 60 | 3.9 cm | 2.6 cm | 5.0° | 3.2° | 180 | 44 °/s | BL-att1 ✅ big jump |
| 18-54-09 | 1200 | 97 | 40 | 8 | 60 | 3.9 cm | 2.6 cm | 4.4° | 3.1° | 170 | 39 °/s | BL-att2 ✅ marginal gain |
| 18-55-22 | 1500 | 108 | 40 | 8 | 60 | 3.9 cm | 2.5 cm | 4.8° | 2.7° | 170 | 44 °/s | BL-att3 ✅ plateau |
| 18-56-23 | 1800 | 119 | 40 | 8 | 60 | 4.0 cm | 2.7 cm | 4.1° | 2.7° | 170 | 39 °/s | BL-att4 — RMSE flat, att plateau confirmed |

---

## 4. Next session plan — position loop (KV/KP), then speed sweep

### 4a. Diagnosis: attitude ladder is exhausted, position loop is next

Across the entire attitude ladder (kr 603→1800, τ_dom 137→59 ms), **lag stayed pinned at
170–180 ms** and Phase XY RMSE stayed flat at 3.9–4.0 cm. If attitude speed were still the
bottleneck, lag should have shrunk along with τ_dom — it didn't. **The position loop is now the
limiting factor, not attitude.** Current ζ_pos = kv_xy/(2·√kp_xy) = 8/(2·√40) = **0.63**
(underdamped) — the same underdamping pattern (ζ≈0.26–0.63) that caused conditional divergence
on the standard-drone session (`results_2026-06-20.md`, July 1 root-cause analysis).

### 4b. Ladder — execute in this order (already queued as BL-5..9 above; this is the how-to)

1. **KV first (BL-5/6), yaml-only, no reflash.** `kv_xy` 8→11→13.
   - Fly 1 hover to check the ~1.4 Hz wobble damps out, then 1 figure-8 per step.
   - **Efficiency note:** don't exhaustively repeat each KV like the attitude ladder did — if
     RMSE + wobble improve monotonically from 8→11→13, stop and lock the better point. Save
     repeat-flight statistics (n≥10) for the final locked config only, as in the upgraded-drone
     n=11 session.
   - Target ζ_pos ≈ 0.87 (kv=11) to ≈1.0 (kv=13).
2. **KP (BL-7).** Once KV is locked, raise `kp_xy` 40→50 for tighter tracking stiffness at the
   new damping ratio.
3. **fc_bw refine-back (BL-8/9).** Re-sweep 50/60/70 with the new KV/KP — the hover-only optimum
   (flat 50–80, found under static load) may shift once the position loop is retuned under the
   dynamic figure-8 load.
4. **Stop signal, same as attitude ladder:** gyro σ climbing or lag not improving ⇒ diminishing
   returns, lock current best and move to the speed sweep (4c) rather than continuing to chase RMSE.

### 4b-results. KV/KP sweep — flown 2026-07-18 (hover, kr=1300/kw=100/fc_bw=60)

**Result reverses the plan's prediction.** Position error is a non-issue everywhere (≤0.7 cm XY
RMSE, steady-state window, all logs). But **attitude oscillation gets monotonically worse as
`kv_xy` increases** — the opposite of what raising KV to damp the ~1.4 Hz wobble was supposed to
do — and raising `kp_xy` 40→50 also made it worse on its own.

| Log (16:xx) | kp_xy | kv_xy | Pos RMSE (xy) | Roll σ | Pitch σ | Roll p-p | Pitch p-p | Osc. freq | gyro_x/y σ | Notes |
|---|---|---|---|---|---|---|---|---|---|---|
| 30-24 | 40 | 6 | — | — | — | — | — | — | — | **ABORTED — only 0.3 s airborne** |
| **33-19** | **40** | **8** | **0.23 cm** | **0.83°** | **0.75°** | 4.96° | 4.98° | 2.8–3.9 Hz | **18.9 / 17.6 °/s** | **best — matches original BL-0 baseline** |
| 29-28 | 50 | 6 | 0.24 cm | 1.07° | 0.98° | 6.75° | 6.25° | 1.4–1.7 Hz | 20.7 / 19.1 °/s | good, but worse than 40/8 |
| 25-24 | 50 | 8 | 0.65 cm* | 1.16° | 1.22° | 6.30° | 7.96° | 2.5 Hz | 22.5 / 23.5 °/s | *first flight, tail of goTo transient inflates RMSE |
| 26-40 | 50 | 11 | 0.25 cm | 1.67° | 2.05° | 11.98° | 12.12° | 2.7–2.8 Hz | 34.5 / 40.0 °/s | clearly worse — trend reversing |
| 27-35 | 50 | 13 | 0.36 cm | 3.53° | 6.32° | 16.56° | **26.79°** | 3.1 Hz | 69.8 / **122.4 °/s** | **worst — approaching divergence** |

**Root-cause note — the velocity feedback filter is already on and didn't help.**
`ENABLE_VEL_FB_FILTER=true` (`firmware_app/src/lib.rs:473`) with `VEL_FB_FC_HZ=10 Hz` and the
pose-glitch clamp (`ENABLE_POSE_GLITCH_GUARD=true`, `EV_XY_MAX=0.6 m/s`) were already active for
every flight in this sweep — this is not the "raise KV without a filter" failure mode flagged in
`results_2026-06-20.md`. The observed oscillation sits at **1.4–3.9 Hz, below the 10 Hz filter
cutoff**, so the filter passes it through unattenuated. This points to a genuine gain-margin /
resonance effect (position-loop KV term coupling into the attitude demand at a frequency the
current attitude bandwidth can't fully reject) rather than raw velocity-estimate HF noise — a
different mechanism than the upgraded-drone case, and not fixable by the existing filter without
lowering `VEL_FB_FC_HZ` further (which adds lag and was not attempted here).

**Conclusion: revert to `kp_xy=40, kv_xy=8` (log 33-19) as the locked position config.** Do not
push `kv_xy`/`kp_xy` further on this platform at the current attitude gains (kr=1300/kw=100)
without first isolating whether lowering `VEL_FB_FC_HZ` (e.g. 10→6 Hz) removes the 1.4–3.9 Hz
oscillation — that is the next diagnostic step if KV needs to go higher, not a blind gain sweep.

### 4b-results-KW. KW sweep — flown 2026-07-18 (hover, kr=1300 fixed, kp_xy=40/kv_xy=8 locked)

| Log (16:xx) | kw | ζ_att | τ_dom | Pos RMSE (xy) | Roll σ | Pitch σ | Roll p-p | Pitch p-p | Angle osc. freq | gyro_x/y σ |
|---|---|---|---|---|---|---|---|---|---|---|
| 44-44 | 90 | 1.25 | 55 ms | 0.38 cm | 1.05° | 1.21° | 7.19° | 7.58° | 3.6–4.2 Hz | 23.2 / 26.6 °/s |
| 40-57 | 103 | 1.43 | 68 ms | 0.74 cm* | 1.04° | 0.96° | 7.20° | 5.90° | 2.5–2.6 Hz | 20.5 / 20.6 °/s |
| 41-53 | 106 | 1.47 | 71 ms | 0.31 cm | 0.98° | 1.03° | 5.53° | 6.06° | 2.6–2.7 Hz | 19.4 / 20.0 °/s |
| 42-51 | 110 | 1.53 | 74 ms | 0.32 cm | 0.82° | 1.00° | 4.67° | 5.48° | 1.8–2.4 Hz | 17.1 / 17.6 °/s |
| 43-45 | 120 | 1.66 | 83 ms | 0.32 cm | 0.81° | 0.91° | 4.51° | 5.42° | 1.5–2.1 Hz | 17.4 / 17.3 °/s |
| 45-40 | 200 | 2.77 | 149 ms | 0.33 cm | **0.62°** | **0.66°** | 3.52° | 3.42° | 1.4–1.5 Hz | 13.9 / 15.2 °/s |

*first flight, tail of goTo transient — same artifact as 25-24 in the KV sweep.

**On roll/pitch alone this looks like a clean win** (σ down monotonically 90→200, gyro σ down
23–27→14–15°/s). It is not — see below.

### 4b-diagnosis. Internal-signal analysis reveals TWO separate oscillations, not one

Roll/pitch is the wrong signal to judge this on. Checking `tau_x/y`, `alp_x/y`, `alp_raw_x/y`
(the INDI inner-loop torque/angular-accel chain, already logged every flight) against the same
three logs tells a different story:

| kw | Roll/pitch peak freq | gyro/tau/alp peak freq | Interpretation |
|----|----------------------|--------------------------|-----------------|
| 90 | 3.6–4.2 Hz | **same** 3.6–4.2 Hz — fully coherent | one coupled mode; attitude loop still fast enough to track whatever drives it |
| 110 | 1.8–3.3 Hz | starts splitting, tau/alp → ~5.5 Hz | the mode is separating into two |
| 200 | **1.4–1.5 Hz** (small, looks improved) | **tau_x/alp_x/gyro → 7.0–7.9 Hz** | **two distinct oscillations**, now cleanly separated |

Raising `kw` does not remove the oscillation — it splits one visible coupled mode into two, and
**hides one of them from the roll/pitch signal** while it remains fully present in
`tau_x`/`alp_x`/gyro. At kw=200 there is a small ~1.4 Hz wobble in the angle (position-loop-band,
what looked like "improvement") riding on top of a **~7–8 Hz limit cycle in the commanded torque
and angular-acceleration chain** — the same 6.6–7.9 Hz signature the 2026-07-16 root-cause hunt
pinned to **brushless motor/prop mechanical lag**, present before the optical-RPM-deck switch.
The deck fix removed the *divergent* version of that limit cycle; a small-amplitude residual
appears to have survived in `tau_x`/`alp_x` regardless of gain. Raising `kw` just makes the
attitude loop too sluggish to fully express that residual in the angle — that reads as "better"
on roll/pitch but is lag, not a cure.

**Why geometric hover looks clean by comparison**: geometric never closes an inner loop around
measured angular acceleration (no `tau_current`/`alpha_meas` incremental term), so it has no
mechanism to ring at the actuator's own mechanical-lag frequency. INDI does, by construction —
consistent with everything already established in §1.

### 4b-diagnosis-toggle. H1a toggle — checked missing 2026-07-18, implemented same day

H1a (`tau_current = tau_prev`, feed-forward-free) was queued 2026-07-16 as the decisive test to
tell motor-mechanical-lag (persists) apart from telemetry/model desync (vanishes), but was never
run because the firmware knob was never built. Checked 2026-07-18 — confirmed true:
`firmware_app/src/lib.rs:1253-1258` only fell back to `tau_current = s.tau_prev`
**automatically** when `rpms_active` is false (RPM deck physically absent/not detected). There
was no runtime param to force feed-forward-free mode **while the deck is live and working**,
which is what H1a actually requires — you need to isolate whether the RPM-feedback path itself
is contributing to the 7–8 Hz content, not just observe what happens with no RPM feedback at all.

**✅ Implemented 2026-07-18** — new runtime param `indi_gains.ff_free` (uint8, default 0):
- `firmware_app/traj_iface.c`: `g_indi_ff_free` global (line ~153) + `PARAM_ADD(PARAM_UINT8,
  ff_free, &g_indi_ff_free)` in the `indi_gains` param group.
- `firmware_app/src/lib.rs`: new `extern "C" { static mut g_indi_ff_free: u8; }`, read each
  attitude-INDI tick (`let ff_free = unsafe { g_indi_ff_free } != 0;`) and OR'd into the existing
  `tau_current` selection: `if rpms_active && !ff_free { rpms_to_torque(...) } else { s.tau_prev }`.
  Reflash required (`make cload`) — this is compiled-in Rust logic, not yaml-only.
- No CS2 `flight.py` wiring added — this is a manual diagnostic-only knob (uint8, not part of the
  auto-pushed float `indi_gains` dict), set via cfclient Parameters tab or
  `cf.param.set_value('indi_gains.ff_free', '1')`, same convention as `ctrl_mode`.

**Test procedure**: fly hover at the locked config (kr=1300, kw=110–120, kp=40, kv=8), once with
`ff_free=0` (normal) and once with `ff_free=1` (forced feed-forward-free even though the deck is
active), diff the `tau_x`/`alp_x` FFT peak at ~7–8 Hz between the two.
- **Vanishes with ff_free=1** → the 7–8 Hz content depends on live RPM feedback → likely a
  telemetry/model desync, fixable in software (delay compensation on the RPM signal).
- **Persists with ff_free=1** → confirmed motor-mechanical lag, independent of the feedback
  path; only remaining options are (a) accept the residual, (b) add first-order actuator-lag
  compensation to the INDI increment, (c) reconsider INDI vs geometric for this platform's
  hardware (per the honest options already listed in §1, 2026-07-16 (c)).

**Also queued, lower priority:** re-sweep `fc_bw` at kr=1300/kw=110–120 (not kw=200 — that's
overdamped in the angle but doesn't touch the actual problem). fc_bw=60 passes 7–8 Hz freely;
try 30/20 to see if attenuating that band in the α-filter reduces `tau_x` amplitude, even though
the 2026-07-16 fc_bw sweep found the *frequency* pinned regardless of cutoff (that test predates
the current kr/kw and the deck).

### 4c. After gains are locked — kt speed sweep

Once 4b settles, repeat the standard-drone kt sweep (`results_2026-06-20.md` §5) at the locked
gains: kt = 0.01 → 0.03 → 0.05 → 0.10 → ... , tracking Phase XY RMSE + peak roll/pitch + lap time.
Expect the same shape — clean tracking up to some kt, then a wall where roll/pitch error climbs
sharply before an outright crash (standard drone: clean to kt=0.05, barely holding at kt=0.10,
crash by kt=0.6). Record the wall precisely; that kt is the speed ceiling for the locked gains.
Pushing past the wall requires retuning attitude/position for the new demanded bandwidth — don't
retune preemptively.

### 4d. After the speed ceiling is known — accel-pinned aggressive maneuvers (Phase 4, slide 20)

`SplineTrajectory::plan_with_accel_pins()` (see root `CLAUDE.md`) is implemented and explicitly
requires the brushless platform's higher thrust/ω ceiling — now unblocked since INDI hover +
figure-8 are stable on CF21BL. Sequence this **after** 4c: accel-pinned segments (e.g. teardrop
loop, a_z=-1.5g at apex) demand more thrust margin than any kt figure-8, so a characterized speed
ceiling should exist before adding pinned-attitude segments on top of it.

---

## Reference baselines (other drones, for comparison)

- **Standard CF2.1 INDI:** 3.87 cm XY RMSE (June 20 2026, kr=1050) — `results_2026-06-20.md`
- **Upgraded CF2.1 INDI:** 3.7 cm mean XY RMSE, n=11 (July 15 2026, kr=603) — `results_2026-07-11_upgraded_drone.md`
