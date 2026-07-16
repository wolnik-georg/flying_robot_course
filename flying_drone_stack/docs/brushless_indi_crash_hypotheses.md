# CF21BL Brushless — Full-INDI Crash: Hypothesis Matrix

Status as of 2026-07-15. Goal: find why `ctrl_mode=3` (full INDI) detonates on the
brushless while geometric (`ctrl_mode=0`) is rock-solid, and while INDI worked on the
standard + upgraded (brushed) drones.

## Established facts (data-backed, treat as ground truth)

- **Geometric hover stable** on brushless (roll/pitch < 5°, α std ≈ 7–8 rad/s²).
- **Full INDI has never been stable** on brushless. Explodes to 180° within ~0.2–0.5 s of
  the `ctrl_mode 0→3` switch. Onset t = 6.04–6.12 s in **every** flight (= the switch time).
- **Divergence is only weakly gain-dependent**: kr=603 → λ≈15–17/s; kr=100 → λ≈8–14/s
  (6× kr cut → only ~1.5–2× slower). tau_x doubling 40–90 ms.
- **tau_current baseline is correct**: rpms_to_torque ≈ 0.0002 Nm in level hover; roll/pitch
  mixer matches firmware AND paper exactly (arm = 0.707×0.050 = 0.035355 m = ARM_M).
- **kt/mass consistent**: Σthrust ≈ weight at hover eRPM. DShot telemetry 0% invalid during
  the flying phase (72% invalid was all post-crash spin-down).
- **Physics params match the paper** (Busetto 2025): J, L, kF, kM all correct. TORQUE_RATIO
  (0.002078 = kM/kF) matches the *paper*; firmware uses 0.00569 (yaw-only disagreement).
- **Paper documents a brushless-specific motor-telemetry delay** (§5.1): DShot back-EMF motor
  speeds needed cross-correlation alignment vs the IMU. ω spectral bandwidth ≈ 18 Hz,
  motor-speed ≈ 20 Hz (Table 4) — our INDI α-filter fc_bw = 30–70 Hz is well above these.
- **Key structural asymmetry**: geometric never uses motor-speed telemetry; INDI's inner loop
  is built entirely on it (`tau_current`). The one subsystem that is uniquely brushless
  (DShot back-EMF telemetry, with delay) is exactly the one INDI depends on and geometric ignores.

---

## Hypothesis matrix

| # | Hypothesis | Evidence FOR | Evidence AGAINST | Conf. | Experiment | Confirms if… |
|---|-----------|--------------|------------------|-------|------------|--------------|
| **H0** | (Partition) It is the **attitude** incremental loop, not position | Explosion is in roll/pitch; tau_y kicks first | — | — | Fly `ctrl_mode=2` (att INDI, pos geom) and `ctrl_mode=1` (pos INDI, att geom) separately | `2` explodes & `1` stable ⇒ attitude loop. Do this FIRST; it scopes all others |
| **H1** | **DShot telemetry delay**: `tau_current` (delayed) misaligned with `alpha_meas` (gyro) ⇒ RHP pole | Paper documents cmd→actuation delay needing alignment; geom(no telem)=stable, INDI(telem)=boom; weak gain-dep; λ≈14/s ≈ few-ms lag; brushless-only | none yet | **High** | (a) Force `tau_current = tau_prev` (feed-forward-free INDI). (b) **Swap RPM source to optical deck** `rpm.*` and compare. (c) Log motor-cmd + az at 500 Hz, cross-correlate | Any of: (a) stabilizes, or (b) optical-deck INDI stable while DShot explodes, or (c) delay ≫ 1 ms |
| **H1b** | **Optical RPM deck** behaves like brushed (works) vs DShot (fails) | brushed INDI used `rpm.*` and flew; DShot is the only new telemetry path | Optical deck may be absent/inactive on this drone (`rpm.*` read 0 in earlier logs) | **High** | Fit/enable optical RPM deck, point INDI + yaml at `rpm.m1..m4`, re-hover INDI | INDI stable on optical deck ⇒ DShot telemetry is the culprit |
| **H2** | **α-chain under-filtered** vs true ω bandwidth (~18 Hz) | Table 4: ω 18 Hz / motor 20 Hz; our fc_bw 30–70 passes noise | J tiny ⇒ torque noise small on its own | **Med** | fc_bw sweep DOWN: 18 → 12 → 8 Hz (runtime param, no reflash) | Divergence slows/stops as fc_bw drops toward ~18 Hz |
| **H3** | **Gains too high** (pure linear instability) — the "it should only be gains" thesis | Obvious knob; INDI is feedback | 6× kr cut barely changed λ; onset time unchanged | **Low–Med** | Drastic kr/kw sweep: kr=30, 10, 3 (kw ≈ 2√(kr·J) scaled). Keep fc_bw fixed | Stabilizes at some low kr ⇒ gains-driven. Still explodes at kr≈10 ⇒ NOT gains |
| **H4** | **Inertia effectiveness overestimate** (J_assumed/J_real > 1 ⇒ increment loop gain > 1) | INDI inverts J directly; gain-independent divergence | J matches paper exactly; paper 45 g vs our 41 g ⇒ real J likely *lower* not higher | **Low** | `J_scale` sweep 1.0 → 0.5 → 0.33 with gains fixed (needs param/knob) | Divergence slows as J_scale drops ⇒ effectiveness overestimate contributes |
| **H5** | **Mass mismatch** 45 g (paper) vs 41 g (config) skews position INDI a_model | 9 % off | geom hover fine w/ same mass; small; scoped out by H0 if attitude-only fails | **Low** | Set `mass=0.045`; confirm actual AUW on scale (AI-deck on?) | Only relevant if H0 shows position loop involved |
| **H6** | **Switch/init transient** (cold `bw_ref` filters, `tau_prev` seed) at 0→3 | Explodes exactly at the switch | Stable-then-explode also fits a steady instability | **Low–Med** | Start flight DIRECTLY in `ctrl_mode=3` from takeoff (no mid-air switch) | Explodes even without a switch ⇒ steady instability (rules H6 out). Stable-from-takeoff ⇒ transient |
| **H7** | **Yaw TORQUE_RATIO** paper 0.002078 vs firmware 0.00569 | 2.74× mismatch on τ_z | explosion is roll/pitch; brushed had a mismatch too & flew | **Very low** | Set `TORQUE_RATIO`→0.00569 (reflash); watch yaw drift only | Only affects yaw behaviour; not expected to fix roll/pitch |

---

## Recommended test order for tomorrow (max information per flight)

1. **H0 partition** — `ctrl_mode=2`, then `ctrl_mode=1`. One or two hovers. Scopes everything.
2. **H2 fc_bw sweep** — 18 / 12 / 8 Hz (runtime, cheap). Cheapest possible test.
3. **H1b optical-deck swap** — if the deck can be fitted/enabled; strongest single discriminator
   for the telemetry hypothesis (H1).
4. **H1a feed-forward-free** — force `tau_current = tau_prev` (needs a small firmware knob + reflash).
5. **H3 drastic gain sweep** — kr=30/10/3 to settle the "is it just gains" question definitively.
6. **H6 no-switch** — start in `ctrl_mode=3` from takeoff.
7. Low priority: H4 J_scale, H5 mass, H7 yaw.

## Knobs still needed (firmware, ask before implementing)

- Runtime `indi_gains.j_scale` (for H4) — avoids recompiling per J value.
- Runtime `indi_gains.tau_ff` on/off or a `tau_current` low-pass/delay-align (for H1a) — one param.
- fc_bw is already a runtime param (H2 needs no code change).
- Optical-deck path: point `rpm_get_all` + yaml at `rpm.*` (compile flag / yaml swap) for H1b.

## Notes / caveats

- Command→actuation delay CANNOT be measured from current CSVs (rpm + acc logged at 20 Hz;
  a few-ms lag needs ≥ 200 Hz onboard logging of motor cmd + az).
- Onset time = switch time by construction; "stable-then-explode" alone does not distinguish a
  switch transient (H6) from a steady instability — H6's no-switch test resolves this.
- Divergence-rate numbers (λ) are from tau_x envelope fits over the first 0.25 s after onset.
