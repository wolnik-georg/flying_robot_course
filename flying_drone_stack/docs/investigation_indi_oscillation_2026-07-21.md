# Investigation — INDI attitude oscillation + trajectory start/end jerks

**Opened:** 2026-07-21 · **Last updated:** 2026-07-22 (second session — deep offline re-investigation, §15)
**Platform:** CF2.1 Brushless (CF21BL), branch `brushless-port`, firmware ctrl_mode=3 (full INDI)
**Status:** OPEN. **Leading hypothesis (§15, H7): the actuator path (ESC+motor+rotor) delivers only
~23% of commanded differential torque with ~83° lag at the 6.3 Hz cycle amplitude** (harmonic-balance
measurement, §15.3) — most plausibly an amplitude-dependent ESC slew/acceleration limit (fast for
small signals, saturating at the cycle's ~200k RPM/s demand — consistent with fact 8).
§14 (RPM/gyro desync) is DEMOTED: onboard rpm-torque and gyro-α measured phase-aligned to ±10° (§15.4).
**2026-07-22 operator correction (fact 12):** motors are confirmed fast/strong, no braking
asymmetry, they follow commands — so the loss of modulation is NOT motor capability. Reframed
(§15.8): the STM32 command chain audited CLEAN (mixer, bat-comp, DShot bit-shift — no filtering,
no slew) ⇒ the measured ~90% modulation loss at 6.3 Hz is either **inside the ESC's throttle
processing (L1, leading)** or a coherent two-sensor measurement error (L2, no mechanism found).
**Next action:** bench open-loop chain measurement via `tools/bench_actuator_id.py` (motorPowerSet
chirp+steps, drone strapped down, NO flight) to locate the loss → then the USD flight log (§13).
**Sharing with another agent:** use `docs/handoff_prompt_indi_oscillation.md` (short entry-point
prompt) together with this doc.

This is a living timeline of what we observe, hypothesize, test, and rule out, so the reasoning
survives across sessions and we never re-run a refuted experiment. **Two problems:** A = INDI-only
attitude oscillation (leading cause §14); B = trajectory start/end jerk on BOTH controllers (§8, HB1).
See Appendix A (repos/paths), Appendix B (operator ground-truth facts), and the hypothesis table (§4).

---

## 1. Symptoms (observations, 2026-07-21 trajectory-library flights, kt=0.05)

Flew the full trajectory library (circle, oval, tilted_oval, figure8, helix, corkscrew, corner,
slalom, teardrop, teardrop_wide, loop) under BOTH geometric and full-INDI, at kt=0.05.

| # | Observation | Which controller |
|---|---|---|
| O1 | Geometric flies visibly smoother, esp. attitude; INDI oscillates | INDI worse |
| O2 | Aggressive attitude **jerk at start and end** of every trajectory | **BOTH** |
| O2b | Geometric settles after the start jerk → smooth; **INDI never settles** | INDI worse |
| O3 | Brushless INDI `tau` bigger / more aggressive than standard drone | INDI, gain-scaled |
| O4 | **All 3 platforms fail at the same kt≈0.2** speed ceiling | shared |

**Two independent problems** (O2 is separate from O1/O2b/O3 — see §2):
- **Issue A** — INDI-only attitude oscillation / limit cycle (O1, O2b, O3, O4).
- **Issue B** — start/end jerk on BOTH controllers (O2).

---

## 2. Why A and B must be separate bugs

Issue B (start/end jerk) appears under the **geometric** controller too. The INDI increment law
(the only thing unique to INDI) therefore cannot be its cause. B must live **upstream of the
controller** — in the reference/setpoint path shared by both controllers (trajectory arm/disarm
handoff). A is INDI-only and lives in the increment/attitude loop. Keeping them separate.

---

## 3. Evidence gathered (2026-07-21, offline from 100 Hz CSV logs)

Logs are 100 Hz (Nyquist 50 Hz); onboard RPM deck updates at only ~20 Hz effective.

### 3.1 The oscillation is a CONTROL artifact, not mechanical
- `tau`, `alpha_meas`, `alpha_raw` all peak at **6.2–6.4 Hz**. This ≈ the INDI attitude natural
  frequency ωn = √kr = √2400 = 49 rad/s = **7.8 Hz**. Mechanical/prop/frame resonances live at
  50–400 Hz. 6 Hz is a **control-loop** frequency.
- gyro PSD, same drone + same maneuver: **geometric rolls off cleanly above 5 Hz; INDI sits
  10–1000× higher across the entire 5–50 Hz band.** The elevated energy exists **only when the
  INDI incremental law is active.**

| maneuver | gyro_x σ GEOM | gyro_x σ INDI | ratio |
|---|---|---|---|
| figure8 | 36 °/s | 48 °/s | 1.3× |
| oval | 62 °/s | 244 °/s (motion) / 104 (mid) | up to ~4× |

⇒ **Refutes the earlier "broadband mechanical vibration / hardware ceiling" conclusion**
(results_2026-07-15_brushless.md §4b-spectrum). Mechanical vibration would show under geometric
too; it does not.

### 3.2 The commanded INDI torque is largely NOT realized
Actual motor torque reconstructed from RPM² vs. the logged commanded `tau_x`:

| maneuver | actual τ (RPM) | commanded τ (INDI) | correlation G |
|---|---|---|---|
| GEOM oval | 0.60 mNm | — | — |
| **INDI oval** | **1.24 mNm** | **25.0 mNm** | **G ≈ 0.00** |
| INDI figure8 | 0.53 mNm | 1.74 mNm | ≈ 0 |

⇒ INDI commands ~20× more torque than the motors deliver, uncorrelated with what's delivered. The
increment is generating 6–8 Hz torque faster/bigger than the actuator can follow; the motors
low-pass it away. The phase lag from the un-followed command sustains the limit cycle.

### 3.3 Bandwidth asymmetry (the core structural difference)
- Geometric attitude loop: ωn = √(KR/J) = √(0.010 / 23.95e-6) = **3.25 Hz**, ζ≈1.1.
- INDI attitude loop: ωn = √kr = √2400 = **7.8 Hz**, ζ=1.73 on paper.
- INDI is tuned to **2.4× the attitude bandwidth of geometric**, and limit-cycles at 6.3 Hz just
  below its ωn. Geometric stays well below the actuator-limited region → smooth.
- Note: INDI's effective attitude stiffness J·kr = 23.95e-6·2400 = 0.0575 Nm/rad is **5.7×**
  geometric's KR=0.010. kr was pushed this high chasing position-RMSE (2.2 cm), at the cost of
  attitude-loop stability.

### 3.4 Same ceiling across platforms (O4)
kt≈0.2 fails identically on standard / upgraded / brushless despite different mass, inertia, and
thrust. A physical limit would differ per platform. An identical numeric ceiling ⇒ the bottleneck
is **shared firmware** (the INDI loop), not hardware.

---

## 4. Hypotheses

| ID | Hypothesis | Status | Evidence |
|---|---|---|---|
| H0 | Broadband mechanical vibration / hardware ceiling | **RULED OUT** | Oscillation is INDI-only (§3.1); 6 Hz is control-freq not mechanical |
| H1 | INDI phase mismatch: unfiltered `tau_current` base | **RULED OUT** | `filt_tau=1` + `filt_order=1` already active in flown config; cycle persists |
| H2 | RPM-feedback path drives it (telemetry desync) | **RULED OUT** | ff_free A/B (2026-07-18): gyro σ + band unchanged with RPM base removed |
| H3 | fc_bw (filter) is the lever | **RULED OUT** | fc_bw 20–100 Hz flat; fc_bw=10 much worse (added phase lag) |
| H4 | **J (inertia) overestimated → increment loop gain >1** | **RULED OUT (2026-07-21)** | System-ID from τ_rpm vs dω/dt: **Jxx≈27–28e-6, Jyy≈26–32e-6** (r=0.6–0.93) vs model 23.95e-6 → J is if anything ~15% *under*, not over |
| **H5** | **INDI loop bandwidth (kr→ωn 7.8 Hz) exceeds actuator/sensing bandwidth → commands torque the motors can't follow → delay-limited limit cycle** | **superseded by H5′** | §3.2 (G≈0, 20× phantom torque), §3.3 (2.4× geometric BW), O3, O4 |
| **H5′** | **Unmodeled BRUSHLESS actuator lag.** Brushless ESC/DShot + larger-prop rotational inertia = slower thrust response than the tiny brushed motors. INDI assumes commanded torque appears instantly; on brushless it lags, so at the tracking-optimal kr the loop over-commands and limit-cycles. Brushed drones have near-instant actuators → same tuning procedure stays stable. **Explains why ONLY brushless shakes.** | **ACTIVE — leading** | §3.2 (G≈0), figure8 ~50 ms cmd→actual lag + coherence rising to 0.83 at 5–8 Hz (§7), tuning-history constraint (§7) |
| **H6** | **Position-INDI outer loop (ctrl_mode=3, `a_indi` from accel+stale RPM) injects noise into Rd → eR → ×kr=2400 → huge alpha_ref** | **ACTIVE — secondary** | Only INDI has `a_indi`; kr amplifies Rd jitter; brushless-specific only if its accel is noisier |
| **HB1** | **Issue B:** periodic (closed-loop) trajectories start/end MID-MOTION → velocity step at arm/disarm → jerk (both controllers) | **CONFIRMED (Issue B)** | Rest-to-rest table §8: circle v=0.857 m/s at both ends; oval/tilted_oval/loop/teardrop periodic |
| **HB2** | **Issue B:** hard `g_traj_mode=0` → hover-keepalive switch at end + any position offset at arm → setpoint step | **ACTIVE (Issue B)** | Residual jerk even on rest-to-rest figure8; firmware lib.rs §1489–1510, flight.py §1060–1067 |
| H8 (=§14) | RPM/gyro phase desync (optical RPM stale vs fresh gyro) breaks increment cancellation | **DEMOTED (2026-07-22, §15.4)** | Onboard rpm-τ vs gyro-α phase-aligned ±10° at 6.3 Hz (the measured 46° lag = telemetry S&H exactly); desync bound ≲5 ms; hover oscillates hard yet ff_free (no RPM in att loop) is null → RPM path not necessary for the cycle |
| **H7** | **Actuator path effective transfer ≈ 0.23∠−83° at 6.3 Hz at cycle amplitude** (INDI assumes 1∠0°). Candidate mechanism: amplitude-dependent ESC slew/accel limit (describing function = flat attenuation + up to −90° lag; small-signal stays fast → fact 8 preserved). Loop math with this P puts kr=2400/kw=170 at \|L\|≈1.2 @ −196° → sustained cycle; explains kr≈600 marginality AND upgraded@603 smooth (their P≈1∠small ⇒ ~70° margin) | **ACTIVE — LEADING (2026-07-22, §15.3/§15.6)** | Harmonic balance at the verified-stationary hover limit cycle; onset exactly at geometric→INDI switch; commanded slew ~200k RPM/s ≳2× plausible spool rate |

---

## 5. Next experiments (all RUNTIME params — no reflash unless noted)

Run on a trajectory with real angular excitation (oval or figure8), kt=0.05, brushless.

1. **ctrl_mode 3 → 2** (attitude INDI only, disable position INDI). Tests H6. Expect: if gyro σ
   drops toward geometric, `a_indi`→Rd→eR was the injector.
2. **kr reduction toward geometric-equivalent.** Set kr so J·kr ≈ KR_geo=0.010 → kr≈420
   (ωn=3.25 Hz, matching geometric). Sweep kr = 2400 → 1200 → 600 → 420. Tests H5. Expect: gyro σ
   and phantom-torque gap shrink monotonically as ωn drops into actuator bandwidth; watch whether
   position-RMSE degrades (the tradeoff that originally pushed kr up).
3. **j_scale down** (2400 fixed): 1.0 → 0.7 → 0.5. Scales the whole increment gain; a secondary
   check on H5 independent of kr (does throttling increment authority calm the cycle).
4. **Issue B:** inspect/adjust the arm/disarm handoff — pre-match velocity/accel continuity at
   traj start (goTo already matches position; check vel/accel), and ramp the end transition
   instead of the hard `g_traj_mode=0` switch. Flight-script + firmware, needs reflash for the
   firmware part.

**Decision rule:** the experiment that most reduces gyro σ / the commanded-vs-actual torque gap
identifies the dominant term. Permanent fix follows from which of H5/H6 wins.

---

## 7. Re-evaluation given the tuning history (2026-07-21)

**New constraint from operator:** all 3 platforms were tuned by the *same* procedure — start at
single-digit gains (smooth hover but catastrophic trajectory tracking), raise pos+att gains until
tracking error bottoms out, lock there. Standard→kr=1050, upgraded→kr=603, brushless→kr=2400.
**Only brushless shakes.** Lowering the gains is not an option — it destroys tracking (already
explored; that's the whole reason the gains are where they are).

**What this rules in/out:**
- "kr generically too high" (plain H5) is **not actionable** — the operator already swept kr; low
  kr = smooth but untrackable. And it doesn't explain why the *identical* procedure is fine on the
  brushed drones. The cause must be **brushless-specific**.
- INDI attitude ωn = √kr: brushless 7.8 Hz > standard 5.2 Hz > upgraded 3.9 Hz. Brushless is the
  only platform whose tracking-optimum pushed the bandwidth past the shake threshold.
- **Why brushless needs the highest kr AND is the only one that shakes points to one thing: its
  actuator is slower/laggier than the brushed motors, and our INDI does not model that lag.**
  Bigger props = more rotational inertia = slower thrust response; ESC/DShot adds transport delay
  (the "DShot delay" already flagged 2026-07-16). Proper INDI (Smeur/Tal) includes a first-order
  actuator model on the "current command" term; ours does not → the increment assumes instant
  response → over-commands (§3.2: 20× phantom torque, G≈0) → limit cycle. **This is a code gap,
  brushless-specific in effect. → H5′.**

Actuator-lag evidence (resolution-limited by 20 Hz RPM logging, so suggestive not conclusive):
figure8 commanded-vs-actual torque peak-corr lag ≈ **+50 ms**, coherence rising 0.60→0.83 across
1→8 Hz. A clean measurement needs high-rate motor/RPM logging or a bench step test.

## 8. Rest-to-rest audit (Issue B, 2026-07-21) — operator assumption was PARTLY WRONG

Planned speed at trajectory start/end, from the onboard degree-8 polynomial:

| trajectory | v_start | v_end | rest-to-rest? |
|---|---|---|---|
| figure8 | 0.000 | 0.000 | **YES** |
| circle | **0.857** | **0.857** | **NO — starts/ends moving** |
| oval, tilted_oval, loop, teardrop | — | — | **NO** (defined `periodic` in export_poly4d.rs) |
| corner, slalom, corkscrew, loop_train, roller_coaster | — | — | YES (defined non-periodic) |

The **periodic / closed-loop** trajectories (circle, oval, tilted_oval, loop, teardrop) are NOT
rest-to-rest by construction — they begin and end mid-lap. Arming from a hover (0 m/s) forces an
instant velocity step → the start jerk (HB1). The non-periodic ones ARE rest-to-rest; their
residual start jerk (e.g. figure8) is smaller and comes from position offset / mode-switch at arm
(HB2), not a velocity step.

## 9. Top-2 causes per issue + how to resolve

### Issue A (INDI oscillation)
| Rank | Cause | How to find out | Solution |
|---|---|---|---|
| A1 | **H5′ — unmodeled brushless actuator lag** | Bench motor step test / high-rate RPM logging → measure lag τ_act (expect ~30–80 ms brushless vs ~0 brushed) | Add first-order actuator model to the INDI increment (filter the current-command term by τ_act, per Smeur/Tal) so high kr stops shaking — **keeps tracking, no gain reduction** |
| A2 | **H6 — position-INDI noise ×kr** | ctrl_mode 3→2 (attitude-only INDI); if shake drops toward geometric, confirmed | Filter/limit `a_indi` injection, or run attitude-only INDI |

Note: "lower kr" is deliberately NOT a listed solution — the operator has shown it wrecks tracking.
The fix must let the high kr stay by removing what makes it shake (the actuator-lag gap).

### Issue B (start/end jerk, both controllers)
| Rank | Cause | How to find out | Solution |
|---|---|---|---|
| B1 | **HB1 — periodic trajectories start/end mid-motion** | DONE (§8 table) | Rest-to-rest lead-in/out (short entry arc that spins up to v_start; decel on exit), or plan periodic paths with a ramp from rest, or fly non-periodic variants |
| B2 | **HB2 — hard end mode-switch + arm position offset** | Quantify the setpoint position/velocity step at the arm and disarm ticks from the log | Blend/ramp the handoff: match velocity (not just position) at arm; ramp the `g_traj_mode=0` end transition instead of instant switch |

**FULL FIX — IMPLEMENTED 2026-07-22 (pending operator review + reflash + re-export; NOT flown):**
1. **Lead-in/lead-out (kills B1) — DONE.** export_poly4d.rs `wrap_rest_to_rest()`: one degree-7
   Hermite entry segment from rest AT the lap-start point to the lap's full (p,v,a,j) at phase 0,
   and a mirror exit segment back to rest at the same point (wind-up arc; excursion printed at
   export for flight-space checks — circle: y±0.33 m). Junctions C³-exact by construction.
   VERIFIED numerically on circle kt=0.05: start/end v,a,j ≤ 1e-4 (f32 roundoff), junction
   mismatch ≤ 6e-5, entry peak accel 2.5 m/s². meta.json gains n_entry/n_exit.
   Firmware timeline support: new `traj_locate()` in lib.rs — entry plays ONCE, core loops
   (traj.reps counts core laps), exit plays ONCE; self-stop at t_entry + reps·t_core + t_exit.
   New params `traj.n_entry`/`traj.n_exit` (default 0 = byte-identical legacy). To fit
   oval/tilted_oval (12-seg cores) + entry/exit: TRAJ_MAX_SEGS 12→14, `traj.ci` widened u8→u16
   (266 coefs). Satisfies Appendix B fact 10 exactly (first entry + last exit at rest, reps
   chain continuously). Builds clean (cf21bl + exporter).
2. **Arm handoff — mostly pre-existing, gap closed.** flight.py already pre-positions to the
   exact trajectory start AND switches ctrl_mode ~5 s before traj.start (correction: the switch
   was NOT on the same tick as arming — verified in code; the hover-log t≈6 s onset is the
   switch alone). Added `_wait_until_at()`: arm only after measured position error < 6 cm
   (was: fixed 2.5 s sleep). flight.py always writes n_entry/n_exit (no stale leak), hard-errors
   if a wrapped export meets pre-entry/exit firmware.
3. **End handoff — aligned by construction now.** Exit ends at rest at the lap-start point =
   keepalive_pos already streamed by flight.py; firmware self-stop connects two identical states
   (same position, zero velocity) → no-op.
**Re-export DONE (2026-07-22): 103/141 periodic combos re-exported with the wrap** — every kt
variant of circle, oval, tilted_oval, loop, teardrop, teardrop_wide, corner_loop (meta now
n_entry=1/n_exit=1). 38 combos NOT re-exported, old files left untouched (legacy meta → flight.py
flies them in legacy mode):
- slalom_loop (7), loop_train_loop (7), roller_coaster_loop (22): 21-segment cores + 2 > the
  14-seg cap. **Pre-existing discovery: these were already broken on the old firmware too** —
  TRAJ_MAX_SEGS was 12 and the firmware silently clamps nseg, so only the first 12 of 21
  segments ever played. Flying them correctly needs the ci-protocol capacity raised further
  (u16 ci already done; needs TRAJ_MAX_SEGS≈24 + RAM) — separate task, flagged not fixed.
- circle mode1 kt1.5 + kt2: Richter QP solver failure at extreme kt (pre-existing, unrelated).
Verification remaining (operator): reflash (`make cload` — timeline + clamps go together),
one flight per shape under GEOMETRIC first — logs must show no attitude transient at arm/disarm.
Side benefit: the start jerk is the "kick" that pushes marginal INDI into the oscillating
attractor (§15.6) — fixing B softens A's onset (but does NOT remove the cycle).

## 10. Code comparison vs official crazyflie-firmware `controller_indi.c` (2026-07-21)

Compared our Rust INDI (firmware_app/src/lib.rs) against the stock C
(`~/Desktop/crazyflie-firmware/src/modules/src/controller/controller_indi.c`) + brushless platform
defaults (`platform_defaults_cf21bl.h`). Real STRUCTURAL differences found:

| Aspect | Official C firmware | Ours | Implication |
|---|---|---|---|
| **Control effectiveness** | `du = (1/g1)·(α_ref − α_meas)`, **g1 empirically identified** per platform (G1_P=0.0066, G1_Q=0.0052) | `delta_tau = J·(α_ref − α_meas)`, **assumes commanded torque is realized exactly** (effectiveness ≡ 1/J) | If realized torque ≠ commanded, our loop gain is wrong. We never identify effectiveness; official does. |
| **Actuator model** | first-order `act_dyn` (≈63 ms) propagated on the command; base = filtered command through this model | none — base = measured RPM torque | Official reconstructs actuator state with a model at 500 Hz; ours samples RPM at ~20 Hz |
| **Filter cutoff** | 8 Hz (gyro + actuator) | 60 Hz | Our differentiated-gyro α_meas is far less filtered — but fc_bw sweep (§4 H3) showed 20–100 flat, so not the primary |
| **Loop bandwidth** | rate-loop REF_RATE=24 ⇒ ~3.8 Hz | kr=2400 ⇒ ωn 7.8 Hz | **We run ~2× the official's attitude bandwidth**, with the same-class sensing/actuation delay |
| **Feedforward** | pure feedback (α_ref = ref_err·att_err) | adds snap feedforward `alpha_des` (Tal & Karaman) | extra INDI-only term; smooth, unlikely the limit-cycle driver but INDI-only |
| **Mixer geometry** | ARM=0.050, THRUST2TORQUE=0.005693 | ARM_M=0.035355 (=√2/2·0.050) ✓, TORQUE_RATIO=0.005693 ✓ | **match** — geometry is not the bug |

**Unifying insight:** the official INDI **never assumes the commanded torque is perfectly realized**
— it identifies the true control effectiveness `g1` and models the actuator. **Ours assumes perfect
realization (torque = J·α, mixer exact).** The data contradicts that assumption: commanded torque
is ~20× the actual (RPM) torque with ~0 correlation (§3.2). So our effective loop gain is not the
"stable 0.89" that J_model/J_real would predict — the realization gap makes it oscillate. This is
the concrete code-level discrepancy, and it is what the official firmware explicitly guards against.

**Still open (honest):** these 100 Hz / 20 Hz-RPM logs cannot cleanly separate WHY commanded ≠
realized torque — candidates are (a) high-kr rate feedback (kw·e_omega) driving the loop past its
delay-limited margin (the 6.3 Hz cycle ≈ ωn), (b) mixer/thrust-curve effectiveness error, (c)
saturation, (d) RPM undersampling artifact. The `j_scale` sweep tests the aggregate effectiveness/
loop-gain directly (NOT just J — it scales the whole increment), so it stays the decisive cheap test
despite J itself being correct.

## 11. Retractions / corrections (2026-07-21)

- **"Brushless actuator is slower" (old A1) — WRONG.** Brushless motors react *faster* than brushed.
  Retracted. The real code-level issue is the effectiveness/realization assumption (§10), not slowness.
- **A2 (ctrl_mode 3→2, position-INDI) — operator already tried, no difference.** H6 demoted.
- **Rest-to-rest requirement (operator, Issue B):** ALL trajectories should be rest-to-rest at the
  overall start AND end (v=0); for multi-rep, only the first entry and last exit are rest-to-rest,
  and reps in between must chain *continuously* (no stop mid-sequence). The periodic exports
  (circle/oval/tilted_oval/loop/teardrop) currently violate this (start/end mid-lap) → the fix is a
  rest-to-rest lead-in/lead-out wrapper around the periodic core.

## 12. Operator corrections + mixer verification + honest current state (2026-07-22)

**Operator corrections (all accepted):**
- RPM-based tau_current is the *better* INDI variant — measured RPM intrinsically contains the
  actuator response, so `act_dyn` / command-base are NOT needed to be modeled. §10 diffs #2/#4 are
  design choices, not bugs. Withdrawn as defects.
- Filter cutoff: 60–70 Hz gives best tracking, 8 Hz crashed, smaller already tried. §10 #3 withdrawn.

**Mixer reconstruction VERIFIED CORRECT (ruled out):** inverted the stock brushless
`powerDistributionForceTorque` and it matches `rpms_to_torque()` exactly on all three axes
(torqueX=arm·(f2+f3−f0−f1), torqueY=arm·(f1+f2−f0−f3), torqueZ=THRUST2TORQUE·(f1+f3−f0−f2); our
ARM_M=0.7071·0.050 ✓, TORQUE_RATIO=THRUST2TORQUE ✓). Only unverifiable-offline residue: whether the
RPM deck's m1–m4 index the same physical motors as the mixer (a permutation hides at hover).

**Honest current state — no discrete bug found.** With J correct (§H4), mixer correct, RPM valid
(physically consistent r=0.93), filter maxed, position-INDI null (A2), and act_dyn unneeded, the
increment reduces to `tau ≈ J·(α_des − kr·eR − kw·eω)` — i.e. INDI is effectively a **stiff geometric
attitude law at ωn=√kr=7.8 Hz (2.4× geometric's 3.25 Hz; effective stiffness J·kr=0.0576=5.7×KR)**.
The 6.3 Hz limit cycle sits at that bandwidth. The weight of evidence now says brushless INDI is
tuned **past its stability edge** chasing the last 0.3 cm of tracking, while standard (5.2 Hz) and
upgraded (3.9 Hz) landed below it — a bandwidth/margin issue, not clearly a code defect. A hidden
brushless-specific loop delay lowering that edge remains possible but is unproven.

**DECISIVE next experiment (runtime, no reflash):** fly brushless INDI at **kr=1050** (= standard's
value → ωn 5.2 Hz).
- smooth like standard ⇒ pure bandwidth; fix = back kr off to the smooth-and-good-enough point.
- still shakes at 5.2 Hz while standard is smooth at 5.2 Hz ⇒ genuine brushless-specific delay/bug.

**Offline-analysis limit reached.** The 100 Hz logs (20 Hz RPM) alias and cannot resolve the
phase/delay setting the 6.3 Hz cycle. To find a delay-based bug we need **≥500 Hz logging of
gyro/tau/RPM + motor PWM**, or an **open-loop chirp/step frequency-response test** on the attitude
loop (measure where phase crosses 180° = the true margin).

## 13. Decisive fact + high-rate USD logging plan (2026-07-22)

**Decisive operator fact:** the shake appears on brushless even at **kr≈600** (= upgraded drone's
value), while the **upgraded drone is smooth at kr≈603**. Same gain, same bandwidth, same INDI
code → different result. **This RULES OUT bandwidth/tuning-edge (§12 retracted) and proves the
cause is brushless-specific.** Everything shared is now excluded (J, mixer, filter, gains,
position-INDI). The only element that differs between brushless@kr600 and upgraded@kr603 is the
**actuator — the brushless motor+ESC response** — which the 100 Hz / 20 Hz-RPM radio logs cannot
resolve.

**RPM source note:** cf21bl uses plain DShot (`CONFIG_MOTORS_ESC_PROTOCOL_DSHOT`, NOT
bidirectional) → no ESC RPM telemetry; the **optical deck (`rpm.mX`) is the only RPM source**, and
it's what our INDI reads for `tau_current`. Optional future test: enable
`CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL` to get native fast ESC RPM (`motor.mX_rpm`) and
compare/replace the optical deck.

**Plan — high-rate USD logging** (config ready: `flying_drone_stack/tools/usd_indi_diagnostic_config.txt`,
17 vars @ 500 Hz synchronous, 4096-byte buffer). Copy to SD card root as `config.txt`, fly a short
INDI hover + one oval, pull the file, parse with `crazyflie-firmware/tools/usdlog/cfusdlog.py`.

Measures the actuator + loop at full rate (no aliasing):
1. **Actuator response:** commanded PWM (`motor.mX`) → optical RPM (`rpm.mX`) → angular accel
   (d`gyro`/dt). Reveals brushless motor+ESC lag / resonance — the suspected brushless-specific
   element.
2. **True RPM rate:** is `rpm.mX` actually ~20 Hz (radio artifact) or fast? Determines if
   `tau_current` is stale.
3. **Loop phase at 6.3 Hz:** clean `tau` ↔ `alp` ↔ `gyro` phase → where the margin is lost.
4. **Reliability:** bigger buffer + trimmed 17-var list should stop the "works with luck" overruns.

Success criterion: identify a brushless-actuator phase lag / resonance around 6 Hz that INDI's
increment excites (and geometric doesn't, since geometric doesn't differentiate gyro / close the
increment loop). That would be the concrete brushless-specific root cause.

## 14. LEADING HYPOTHESIS — RPM/gyro phase desync on fast actuators (2026-07-22)

**Operator input:** tried DShot ESC-telemetry RPM → broke INDI; optical deck works; both give similar
RPM numbers but "similar phase lag that fucks the INDI loop." Uses optical deck (same as other drones).

**Mechanism (fits ALL constraints):** the INDI increment `tau = tau_current + J·(α_ref − α_meas)`
only cancels correctly if `tau_current` (RPM) and `J·α_meas` (gyro) are the **same instant**
(then `tau ≈ J·α_ref`). This needs RPM and gyro **phase-synchronized**.
- **Brushed (slow motors):** gyro responds slowly (motor-limited); optical RPM also lags → both lagged
  ~equally → consistent → cancel → smooth.
- **Brushless (fast motors):** gyro tracks the fast motor almost immediately, but the optical RPM
  sensor lag is fixed → gyro fresh, `tau_current` stale → cancellation breaks → residual
  `J·(α(t−τ) − α(t))` (~90° shifted) injected → destabilizing → 6.3 Hz limit cycle.

**Explains every constraint:** only-brushless (only there are motors fast enough to out-run the RPM
lag); present at kr=600 (residual is gain-independent); filt_tau/filt_order didn't fix it (they sync
the *Butterworth*, not the upstream RPM *sensor* lag); DShot also broke it (similar lag); ff_free null
at hover (no motion → no mismatch). This is Smeur's "actuator feedback must be time-synchronized with
the gyro" requirement — we synced the filters but not the sensor lag.

**Fix (code-level):** add a delay / matched low-pass to the **gyro→α_meas** path equal to the optical
RPM sensor lag, so `tau_current` and `J·α_meas` realign. The high-rate USD log (§13) measures that
lag directly (rpm.mX vs d gyro/dt phase) → sets the delay. Alternative to test: reduce reliance on the
RPM base during fast transients.

**Status: leading, not yet confirmed.** Confirmation = USD log shows rpm.mX lagging the gyro-derived
α by a consistent τ, and adding that τ to the α_meas path removes the cycle.

## 15. Deep offline re-investigation (2026-07-22, second session) — §14 demoted, new leading H7

Independent from-scratch pass over the 07-21 logs + firmware sources. Analysis scripts in
scratchpad (`phase1.py`, `phase2_tf.py`); new tools committed to `flying_drone_stack/tools/`.

### 15.1 The limit cycle is fully present at HOVER — and onsets exactly at the INDI switch
`hover_mode0_17-56-44` (yaml ctrl_mode=3; takeoff runs geometric, switch at trajectory start —
per-log meta `takeoff_ctrl_mode=0` / `trajectory_ctrl_mode=3`):
- gyro σ **24.7/27.8 °/s (INDI hover)** vs **4.9/4.1 (geometric hover)** — 5–7×.
- α spectrum peaks at **6.25 Hz**, 63% of α energy in the 4–9 Hz band; tau_x σ 10.4 mNm at hover.
- Band-passed (5–8 Hz) α envelope: **~1 rad/s² during the geometric takeoff/hover (t≈3–4.5 s),
  jumps to ~16 rad/s² within ≲1.5 s of the ctrl_mode 0→3 switch (t≈6 s), then stays 8–20 rad/s²
  (CV 0.31) for the remaining 20 s.** Sustained, quasi-constant-amplitude, narrowband =
  a genuine limit cycle, not amplified noise. Implied gyro amplitude 16/39.6 = 23 °/s ✓ matches σ.
- Consequence: §14's reconciliation of the ff_free null ("no motion at hover → no mismatch") fails —
  hover has α σ ≈ 18 rad/s² of self-sustained motion, so a desync mechanism WOULD be active there,
  yet ff_free (tau_prev base, no RPM in the attitude law) changed nothing (H2). Together with §15.4
  this demotes §14.

### 15.2 Closed-loop identification caveat — §3.2's "20× phantom torque, G≈0" reframed
Every offline correlation between in-loop signals (tau↔alp↔rpm) in a noise/cycle-driven closed loop
converges to the **controller inverse −1/C, not the plant**. Verified numerically: the measured
tau_cmd→J·alp transfer (coherence 0.95–0.99!) matches −1/C = −1/[J(kr/s²+kw/s+1)] across 5–12 Hz
(e.g. 6.3 Hz: predicted 0.23∠−83°, measured 0.25–0.28∠−80..−87°; 8 Hz: predicted 0.30∠−91°,
measured 0.25∠−91°). So §3.2's raw-σ ratio ("20×", G≈0) was additionally inflated by aliasing
(100 Hz log of a 500 Hz signal) — the honest statement is §15.3 below. **Offline in-flight data can
NEVER identify the actuator; only open-loop excitation can (hence the bench test).**

### 15.3 What survives the caveat: harmonic balance at the cycle frequency
BECAUSE the hover tone is a verified stationary limit cycle (§15.1), the oscillation condition
P·C ≈ −1 must hold at 6.25 Hz ⇒ **the true operative plant (mixer→ESC→motor→rotor→gyro, normalized
so ideal INDI = 1∠0°) is ≈ 0.23∠−83° at 6.3 Hz at the cycle amplitude.** The motors realize ~¼ of
the commanded differential torque, ~35 ms late. Known filter chain accounts for only ~11° of it.

### 15.4 RPM vs gyro are phase-SYNCHRONIZED onboard (kills the §14 mechanism)
Cross-spectral phase (sign convention verified on synthetic data): rpm-reconstructed τ lags logged
α by **46° at 6.3 Hz** — but the 20 Hz telemetry sample-hold of the rpm columns alone predicts
~45° (25 ms hold-lag minus 5 ms for the 100 Hz α columns). **Onboard desync bound: ≲5 ms (±10°).**
The optical-deck driver (rpm.c) is interrupt-driven input capture updating per blade-pass
(~2–5 ms intrinsic latency at 10–20k RPM); the ~20 Hz seen in radio logs is pure telemetry
decimation (measured: exactly 1-in-5 samples change at 100 Hz, all four motors, zero dropouts).
Also: J·α_meas σ (0.43 mNm) ≈ τ_rpm σ (0.44 mNm) — the two sensors agree in magnitude too.

### 15.5 Firmware/code audit results (all offline-verifiable items closed)
- **No timer conflict:** CF21BL motors all on TIM2 (PA1/PB11/PA15/PB10, motorMapCF21Brushless);
  RPM deck uses TIM3/TIM5 input capture — clean.
- **Battery-thrust compensation IS active in the flown build** (`firmware_app/build/.config`,
  built 17:52, flown 17:56–18:48: `CONFIG_ENABLE_THRUST_BAT_COMPENSATED=y`, MAX_THRUST=200 mN).
  The force→duty map is the cubic thrust↔voltage inversion (motors.c:181), hover sits mid-scale
  (u₀ = 0.1 N / 0.2 N = 0.5). A naive "linear force→PWM slope = 2u₀" theory was checked and is
  DEAD — the static small-signal slope is nominally correct.
- **§13 correction: the flown build has `CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL=y`**
  (app-config-bl overrides the defconfig) → DShot ESC RPM telemetry (`motor.mX_rpm`) IS available
  as a second, independent RPM sensor. Added `motor.m1_rpm` to the USD config for cross-check.
  `CONFIG_MOTORS_DEFAULT_IDLE_THRUST=7000` (~11%).
- **Oval 18-01-26 is a blow-up flight, exclude it:** tau_x railed at exactly ±51.93 mNm =
  J·kr·0.903 ≈ J·kr·|eR|max — the attitude error was saturated (≥90° off) for >39% of samples.
  §3.2's "25 mNm commanded" row came from this flight. Use oval 18-16-21 instead (partially railed
  too). Fig8 17-57-32 and hover 17-56-44 are clean (rail-frac 0.00–0.04).

### 15.6 Loop margins with the measured P — explains every platform fact quantitatively
L(jω) = P(ω)·(kr + jω·kw)/(jω)² (+known filters). At 6.3 Hz the PD zero (kr/kw = 14.1 rad/s)
contributes +70° lead — **exactly cancelled by the −83° actuator lag** — that is WHY the cycle sits
at 6.3 Hz. With P = 0.23∠−83°: kr=2400/kw=170 → |L| ≈ 1.2 @ ≈ −196° → unstable → limit cycle ✓.
kr≈600 (kw ~85) → |L| ≈ 0.55 @ −186° → linearly marginal; any transient (start jerk, mode switch)
kicks it into the large-amplitude attractor ✓ (matches "shakes even at kr=600", O2b "never settles").
Upgraded @ kr=603 with a fast actuator (P≈1∠−15°) → ~70° phase margin → smooth ✓ (§13 decisive fact).
Geometric on brushless: crossover far below 6 Hz (KR=0.010 = J·420 equiv) where the actuator lag is
small → smooth ✓. **No remaining unexplained platform fact.**

### 15.7 Candidate physical mechanisms for P ≈ 0.23∠−83° (to be discriminated on the bench)
1. **ESC slew/acceleration limiting (leading):** amplitude-dependent; describing function gives flat
   attenuation + up to −90° lag; the cycle demands ±~2600 RPM at 6.3 Hz ≈ 2×10⁵ RPM/s — ≳2× a
   plausible spool rate. Small signals stay fast → consistent with fact 8 ("brushless reacts faster").
   Predicts CONDITIONAL stability (needs a kick — supplied by the INDI engage transient / Issue B jerk).
2. Spool-down asymmetry (brushless deceleration is drag/brake-limited; brushed micro-motors are not).
3. A linear rotor/ESC pole — would need τ≈100 ms to fit; implausible and contradicts fact 8 at small
   signal. Only kept until the bench measures it.
4. Wrong local slope of the VMOTOR2THRUST cubic fit for these props — would be a flat gain error
   (phase unexplained); the bench staircase measures the true static curve.

### 15.8 Operator corrections (2026-07-22) + STM32 chain audit → H7 mechanism REFRAMED
**Operator ground truth added (Appendix B fact 12):** the motors are confirmed faster and stronger
than brushed, no braking asymmetry, they follow commands. So §15.7's mechanisms (1)-(3) as
*motor*-explanations are rejected. What SURVIVES is the measurement itself, restated precisely:

**Measured fact (two independent sensors):** during the 6.3 Hz cycle the commanded torque implies
±~2700 RPM of rotor-speed modulation; the optical deck shows only ±~250 RPM happened, and the
gyro's angular acceleration (J·alp σ 0.43 mNm vs τ_rpm σ 0.44 mNm) agrees with the deck, not with
the command. ~90% of the commanded MODULATION never appears at the shaft. This is not a claim
about motor capability — it is about where the modulation is lost.

**STM32-side command chain audited CLEAN (offline, this session):**
- powerDistributionForceTorque: linear force mixing, geometry verified (§12).
- Bat-comp cubic (motors.c:181): static, memoryless, slope self-consistent by construction
  (its inverse is the same fit); active in the flown build.
- powerDistributionCap: inactive at hover levels (headroom ~2× before any cap).
- DShot conversion (motorsPrepareDshot): a transparent bit-shift (ratio>>5 → 11-bit throttle),
  NO filtering, NO slew limit, new frame every 500 Hz tick.

**⇒ Only two places left for the loss:**
- **(L1) Inside the ESC** — its own throttle signal processing (see "ESC settings" below). An ESC
  input low-pass/smoothing passes sharp steps fast (motors feel fast ✓ fact 12) while attenuating a
  sustained 6 Hz sine. This is now the leading concrete location for H7's measured 0.23∠−83°.
- **(L2) The measurement is somehow wrong** — both sensors would have to be wrong *coherently*
  (optical RPM under-reporting modulation by ~10× at 6.3 Hz AND gyro-α matching that error).
  No mechanism found: the deck averages over 1 rev (~4 ms, negligible at 6.3 Hz), telemetry S&H
  costs only 16%, and gyro-α is an independent physical channel. Kept only for completeness.

**ESC settings (verified for CF21BL):** the ESCs run **Bitcraze's fork of Bluejay** (BLHeli_S-class
8-bit firmware; github.com/bitcraze/bluejay). Settings are read/written with the drone on USB via
Chrome at esc-configurator.com (pass-through; see
`crazyflie-firmware/docs/userguides/advanced-configuration/esc_configuration.md`). Configurable:
startup/rampup power, demag compensation, motor timing, PWM frequency, brake, etc. Bitcraze's own
doc warns "running on 24 kHz PWM frequency causes unstable flight behaviour" — direct precedent
that ESC-side settings affect flight stability on this platform. If L1 confirms, read the current
ESC settings first (screenshot them into this doc) before changing anything; the fix is either an
ESC setting or an actuator-model compensation in our INDI parameterized by the bench measurement.

**Speculative side-note (H9, weak — recorded for completeness):** kr landed at 2400 ≈ 4×
upgraded's 603, and 1/|P| ≈ 4.3 — IF the true CF21BL inertia were ~4× the Busetto value, the
tuning ladder would naturally have compensated a 4×-under J_model, and the increment's
base/α_meas cancellation would leave 0.75·J_t·α positive feedback (brushless-specific residual).
BUT this is contradicted by τ_rpm (J-independent) showing the delivered torque really is small,
and by the TLS sys-ID. Only resurrect if the bench shows the chain transparent AND the USD log
shows full RPM modulation in flight (which would invalidate the τ_rpm smallness).

### 15.10 Stock-C vs ours: clamping + logic-order re-audit (2026-07-22, operator question)
Re-read controller_indi.c specifically for clamps/ordering (complements §10):
- **Order/filters: already matched.** Stock filters gyro then differentiates (= our filt_order=1),
  filters the base with the same filter (= our filt_tau=1). Both ON in the flown config. Not it.
- **Explicit output clamp: stock HAS one, we have NONE.** `indi.u_in` clamped ±32000 motor units
  (runtime param `indi.bound_ctrl_input`) before output. Our tau is unclamped (oval's 52 mNm =
  natural eR saturation, not a designed bound). Mixer-level clamps (force<0→0, uniform over-limit
  reduction, idle floor 7000) exist in BOTH and are silent to the controller.
- **BUT clamping is NOT the cause:** hover-cycle torque needs ~0.066 N/motor vs ~0.1 N headroom —
  the cycle runs ~96% inside the clamp-free region (rail-frac 0.04). Adding a tau clamp = cheap
  robustness improvement (esp. against blow-ups like oval 18-01-26), not the fix for the cycle.
- **The behavioral difference that matters — the increment BASE:** stock base = previous COMMAND
  through clamp + act_dyn actuator model + filter → increment accumulates; empirically-identified
  g1 absorbs the whole chain gain (mixer/ESC/motor) → law never trusts realization. Ours =
  measured-RPM torque, which ≈ J·α_meas (verified ±2%) → cancels → collapses to τ = J·α_ref, a
  stiff SI-unit PD that implicitly assumes chain gain 1∠0° — the exact assumption the data breaks.
  CAVEAT: ff_free (τ_prev base, but NO act_dyn + NO clamp) was tested null → the base alone is not
  the switch; stock's robustness = base + act_dyn + clamp + identified g1 as a package.

**CLAMP PLAN (designed + IMPLEMENTED 2026-07-22, pending operator review + reflash; robustness,
NOT the cycle fix).** Code: `clamp_torque()` + tilt/thrust clamps in lib.rs `controller_step`
(tau clamped in BOTH controller branches, BEFORE tau_prev store/log; tilt clamp skipped when
rd_override is active or thrust_vec.z ≤ 0); params in traj_iface.c (`indi_gains.clamp_en` bitmask
+ `tau_xy_max`/`tau_z_max`/`tilt_max_deg`/`thrust_max`, default clamp_en=0 = byte-identical);
yaml brushless block sets clamp_en: 15 with values below. Builds clean (cf21bl, 2026-07-22).
| clamp | value (CF21BL) | rationale |
|---|---|---|
| tau_xy | ±14 mNm | physical headroom: ARM·4·ΔF_hover = 0.0354·4·0.1; normal flight ≤4 mNm, blow-up hit 52 |
| tau_z | ±2.5 mNm | yaw authority TORQUE_RATIO·4·ΔF ≈ 2.3 mNm |
| tilt (angle of desired thrust vector from vertical, pre-desired_rot) | ±30° | stock uses 10° but our kt=0.05 needs 15–22°; 30° stops runaway Rd |
| thrust | [0, 0.8 N] | 4×THRUST_MAX |

Implementation rules: (1) clamp BEFORE storing tau_prev (stock propagates post-clamp into its
base/act model); (2) runtime bitmask `indi_gains.clamp_en` (bit0 tau_xy, bit1 tau_z, bit2 tilt,
bit3 thrust) + runtime values (`tau_xy_max`, `tau_z_max`, `tilt_max_deg`), per-drone defaults in
yaml like the gains (standard ≈ ±7 mNm, upgraded ≈ ±10 mNm by the same headroom formula).
Per-bit gating is REQUIRED for Phase 4 inverted/loop flight: tilt clamp must be OFF there
(tilt >90° is intended) while tau clamps stay ON (motor physics is orientation-independent).

`clamp_en` bitmask cheat sheet (bits compose additively — no separate "all" flag needed):
| value | meaning |
|---|---|
| 0  | all off (firmware default — byte-identical behaviour) |
| 15 | all four on (brushless yaml setting) |
| 11 | all except tilt — Phase-4 inverted/loop preset |
| 3  | torque clamps only |
- **WARNING recorded:** if the ~4× chain-gain deficit confirms, do NOT compensate via j_scale≈4 —
  that raises loop gain into the phase deficit and worsens the cycle. The fix must address gain
  AND phase together (actuator model / ESC config), parameterized by the bench measurement.
- **Outer loop (position_controller_indi.c) also audited** (stock INDI spans two files): stock
  filters accel (Butterworth) — ours does too (bw_acc, lib.rs:1449). What stock has and we lack,
  again: **clamps** — commanded tilt clamped to ±10° (`posCtrlIndi.pq_clamp`, runtime param),
  thrust clamped [MIN,MAX], thrust base through act_dyn. Position-INDI is A/B-exonerated for the
  cycle (fact 7: ctrl_mode 2 vs 3 no difference), so these are not the cause either — but a tilt
  clamp on our Rd would have prevented the oval 18-01-26 blow-up (eR saturated ≥90°) and would
  soften Issue B start/end jerks. Candidate cheap robustness additions (post-fix): tau clamp +
  tilt clamp, both mirroring stock.

### 15.9 New decisive experiments (in order; scripts ready, no reflash)
1. **Bench open-loop chain measurement — NO FLIGHT** (`tools/bench_actuator_id.py` +
   `analyze_bench_actuator.py`): drone strapped/held down, props on; motorPowerSet staircase +
   ±10%/±30% steps + 1–15 Hz chirps at ±5% and ±25% around hover. Purpose per §15.8: locate WHERE
   the 6 Hz modulation is lost (per fact 12 it is NOT motor capability). motorPowerSet enters at
   motorsSetRatio (motors.c:698) → identical bat-comp + DShot path as flight; the optical deck is
   the same sensor as in flight. 100 Hz radio logging fully resolves ≤15 Hz. Outcomes:
   - chirp attenuated ~4× with ~−80° at 6.3 Hz → **L1 confirmed: ESC throttle processing** is
     eating the modulation → fix = ESC config or actuator-model compensation in the increment.
   - chirp tracks fully to 15 Hz on the optical RPM → chain transparent AND sensor fine → the
     flight measurement (§15.8) is contradicted → re-examine in-flight conditions (voltage sag,
     4-motor load) via the USD log; H9 becomes live again.
2. USD 500 Hz in-flight log (§13 config, now 18 vars incl. motor.m1_rpm) — confirms in flight and
   cross-checks optical vs DShot RPM phase.
3. Fix design follows the bench numbers — candidate directions (NOT yet chosen): compensate the
   measured actuator dynamics in the increment (Smeur-style actuator model on the base term, which
   with a LAGGING actuator is not equivalent to the RPM base — the RPM base self-cancels against
   α_meas and provides zero robustness, verified in the loop algebra); and/or raise the effective
   spool rate (ESC settings); and/or reshape kr/kw to restore phase margin at the measured P while
   keeping tracking (kr/kw ratio moves the PD zero — the CURRENT ratio wastes its lead exactly
   where the actuator eats it).

## 16. BENCH RESULT (2026-07-22, flight log bench_actuator_2026-07-22_17-12-22.csv) — ROOT CAUSE MEASURED

**The brushless command→rotor chain is a LINEAR first-order lag: fc ≈ 3.4 Hz, τ ≈ 47–60 ms.**
Open-loop motorPowerSet chirps (12.5k samples @ 101 Hz, coherence 0.88–1.00 up to 10 Hz):

| freq | \|H\|/\|H0\| (±5%) | \|H\|/\|H0\| (±25%) | phase (±25%) |
|---|---|---|---|
| 2 Hz | 0.88 | 0.89 | −33° |
| 4 Hz | 0.55 | 0.68 | −56° |
| 6.3 Hz | 0.45 | 0.48 | −68° |
| 10 Hz | 0.23 | 0.31 | −81° |

- **Both amplitudes identical** ⇒ NOT an amplitude-dependent slew/accel limit — §15.7 mechanism (1)
  REFUTED. It is a plain linear pole (§15.7 mechanism (3), previously dismissed — data wins).
- Steps: τ63 up 53–59 ms, down 70 ms → mild spool-down asymmetry (1.2–1.3×), secondary.
- Static curve: monotone, ~17.5k RPM per unit ratio around hover — no static anomaly.
- Fact 8 reconciliation: the motors have plenty of thrust *authority* (more + quicker than
  brushed in force terms); the ~50 ms is the ROTOR SPOOL response (prop rotational inertia vs
  aero drag) — both statements are true. Whether the brushed drones spool faster is now the one
  remaining platform question → same 5-min bench on the upgraded drone answers it.

**This closes the loop on Issue A quantitatively:** actuator −68° at 6.3 Hz + known filters −11°
= −79° ≈ the −83° the harmonic balance demanded (§15.3) — phase agreement within 5°. |L(6.3 Hz)|
= 0.46·4.56 ≈ 2.1 at ≈ −189° at kr=2400/kw=170 ⇒ linearly unstable ⇒ oscillation grows until
nonlinearity caps it at the observed amplitude (the HB magnitude 0.23 vs bench 0.46 gap = the
amplitude-dependent effective-gain reduction at the full cycle + 4-motor/voltage-sag conditions).
**The 6.3 Hz limit cycle is fully explained by a measured, linear, brushless-specific actuator
pole that our INDI does not model.**

**Why previous fixes failed, now explained by the same pole:** RPM base self-cancels against
α_meas (zero robustness, §15.10); ff_free's raw τ_prev base gives a 1/(dt·s) near-pure integrator
→ still unstable with a 47 ms plant pole; fc_bw sweeps never touched the actuator.

**~~THE FIX: act_dyn base~~ — PROPOSED, THEN REFUTED IN SIMULATION SAME DAY (§16.1).** Kept for
the record: the idea was that an actuator-model base inverts the pole (Smeur; stock ships
act_dyn≈63 ms). The simulator falsified it before any flight — see §16.1.

### 16.1 Simulator validation (tests/test_indi_actuator_lag.rs, 2026-07-22) — act_dyn ≢ fix

Replicated the firmware INDI chain (same Butterworth/filter order/gains, 500 Hz) on a 1-axis
plant with the bench-measured actuator (44 ms pole + ~4 ms dead time — the dead time is required:
the pure fitted pole predicts −60° at 6.3 Hz but −68° was measured). Findings (all encoded as
passing regression tests):

1. **RPM base at kr=2400 limit-cycles** (σ≈9.6 rad/s, ~5 Hz in this crude model) ✓ reproduces
   the flight failure from the bench measurement alone.
2. **act_dyn with the MATCHED model (44 ms) is EQUIVALENT to the RPM base** — identical
   instability, identical stable-kr ceiling at every dead-time setting. Obvious in hindsight: a
   correct model of the commanded torque IS the applied torque, which is exactly what the optical
   deck measures. **Our RPM base was never the deficiency — it already is an act_dyn with a
   perfect model.** The §15.10 "zero robustness" framing of the RPM base is hereby corrected:
   no base variant provides robustness to the actuator lag.
3. **The stable-kr ceiling is physical:** with the measured actuator, kr_max ≈ 800 (ζ=1.74
   ladder; swings 500–1300 for 2–6 ms dead time). Stable at kr=603, unstable at kr=1050 in this
   model. The flown kr=2400 is ~3× past the ceiling — that IS the shake.
4. **Model gap, stated honestly:** flight reportedly shakes even at kr≈600 (fact 4) while this
   model is stable there ⇒ in-flight carries extra lag the bench/model misses (candidates: EKF
   attitude lag in the eR path, position-loop coupling via Rd at kp=64, 4-motor voltage sag).
   The sim reproduces the mechanism and the ceiling's existence, not its exact in-flight value.
5. **Curiosity (NOT a validated fix):** act_dyn with a deliberately 2×-slow model (88 ms) leaves
   a lead-like residual that doubles kr_max to ~1700 in this NOISE-FREE sim. Real gyro noise
   pays for lead — do not deploy without a noise study.

**Implemented anyway as a hardware A/B knob (operator request): `indi_gains.act_tau`** (float s;
0.0 default = RPM base, byte-identical; >0 = act_dyn base with that time constant; state
`tau_act` fed from the CLAMPED previous command). Builds clean. Expected hardware result per
finding 2: act_tau=0.044 ≈ no change vs today — verifying that on hardware validates the model.

**Revised fix directions (in order of promise):**
- **(F1) Respect the ceiling + recover tracking via feedforward:** drop kr into the stable range
  (~600) and phase-advance the TRAJECTORY feedforward only (apply the inverse actuator model
  (1+τ_act·s) to the α_des/snap feedforward term) — feedforward inversion has NO stability cost
  and directly attacks the tracking lag that pushed kr up in the first place. The kr=2400
  "tracking optimum" was found WITH the limit cycle present; the landscape changes once the loop
  is stable and the FF is advanced. Needs: sim check of FF-inversion tracking benefit, then
  firmware param.
- **(F2) Find the extra in-flight lag** (finding 4) — EKF attitude phase at 5-7 Hz is measurable
  from the USD log (stabilizer.roll/pitch are in the config); position-loop decoupling test =
  fly attitude-INDI with soft position gains.
- **(F3) The 88 ms lead trick** — only after a noise study (cheap to test on hardware via the new
  act_tau knob at 0.088, since it's runtime — but treat as experiment, not fix).
- ESC-side (PWM freq etc.): cannot remove rotor inertia; minor at best. j_scale warning stands.

### 16.2 Upgraded-drone bench (2026-07-22, bench_actuator_2026-07-22_17-38-01.csv) — confirms
### AND breaks the story: single-motor bench misses a brushless-specific effect

Fit: τ ≈ **71 ms** (fc ≈ 2.2 Hz), ~2–3 ms extra dead time — a genuinely SLOWER actuator than
brushless's 44 ms. **Confirms fact 8** (brushless is the faster actuator) — no contradiction.

Fed each drone's own bench-measured actuator into its own flown gains in the validated simulator
(dead=2ms, the physically likely value from the higher-frequency fit points):
| Gains (drone) | Actuator used | Result | Matches reality? |
|---|---|---|---|
| kr=603/kw=90 (upgraded) | 71 ms (upgraded's own) | **stable** (σ=0.0002) | ✓ upgraded flies smooth |
| kr=2400/kw=170 (brushless) | 44 ms (brushless's own) | **unstable** (σ=6.1) | ✓ brushless shakes |

Good corroboration at each drone's ACTUAL operating point. **But the decisive cross-check breaks
it:** operator fact 4 says brushless STILL shakes at kr≈600 (same as upgraded's value). Testing
that exactly — brushless's own 44 ms actuator at kr=603/kw=90 — the simulator says **stable**
(σ=0.0000). **The bench-measured single-motor actuator lag does NOT explain the low-kr brushless
shake.** This is the same "model gap" flagged in finding 4 (§16.1), now sharpened: it is not a
generic missing lag, it is specifically NOT reproduced by a single-motor open-loop bench.

**Leading suspect:** the bench chirped ONE motor with the other three held at hover; a real
roll/pitch command moves TWO DIAGONAL motors up and two down SIMULTANEOUSLY. On a static bench
(recirculating downwash, no forward airspeed) that differential pattern can drive prop-to-prop
aerodynamic interaction (accelerating prop's wake impinging on the neighbour's decelerating disc)
that a lone motor never exercises — plausibly brushless-specific if its prop/frame spacing or
disk loading differs from the brushed platforms.

**Next decisive bench experiment:** repeat the chirp DIFFERENTIALLY (m1,m2 command = hover+chirp;
m3,m4 = hover−chirp, mimicking an actual roll command) instead of single-motor. If differential
shows materially more attenuation/lag than single-motor at 5-8 Hz → confirms a differential-mode
aerodynamic effect (brushless-specific, testable qualitatively vs upgraded's own differential
run too). If it looks the same as single-motor → the gap is NOT aerodynamic-differential; revert
to F2 (in-flight EKF/position-loop/voltage-sag lag hunt).

### 16.3 Differential-chirp bench, BOTH drones (2026-07-22) — INCONCLUSIVE, mount-rigidity artifact

Runs: upgraded `bench_actuator_2026-07-22_17-47-46.csv`, brushless `bench_actuator_2026-07-22_17-55-01.csv`.

**Single-motor fits CONFIRMED, cross-session repeatable** (different bench session, same drone):
upgraded fc≈2.26 Hz → τ≈70 ms (was 71 ms); brushless fc≈3.87 Hz → τ≈41 ms (was 44 ms). Solid,
reproducible measurements — §16/§16.2 numbers stand.

**Differential-chirp result on BOTH drones: breaks down at 6-8 Hz with a resonance signature, not
a lag signature.** Both track the single-motor roll-off cleanly to ~4-5 Hz (coherence 0.9+), then
at 6-8 Hz: gain rises ABOVE 1 (upgraded diff_25%: |H|=0.744 at 6.28 Hz vs 0.387 at 5.01 Hz),
phase FLIPS SIGN (upgraded +77°; brushless −122° after leading at 5 Hz), coherence collapses
(0.12–0.31). Gain>1 + phase-sign-flip + coherence collapse is the signature of exciting an
UNDERDAMPED RESONANCE that dominates the output (decorrelating it from the input chirp) — not a
smooth aerodynamic lag, which would stay causal and monotonic like the single-motor case did.

**Why this is a MOUNT artifact, not a drone/aero finding:** (1) single-motor chirps (pure thrust,
zero net torque) never show it, on either drone, at any frequency tested — only DIFFERENTIAL
commands (which command net ROLL torque) do; (2) it appears in nearly the same 6-8 Hz band on two
physically different airframes — a genuine prop-to-prop aerodynamic or frame-structural effect
would be expected to differ between platforms with different prop/frame geometry, whereas a hand-
or tape-mount's own rotational compliance is a property of the HOLD, not the drone, and plausibly
sits in this range for both. Conclusion: commanding differential thrust tries to rotate the whole
airframe; the bench hold (hand/tape) is not rotationally rigid, so the drone+mount system rocks at
its own resonance and that corrupts the RPM signal near 6-8 Hz.

**Verdict: THIS EXPERIMENT DOES NOT CONFIRM OR REFUTE the prop-interaction hypothesis** — it is a
bench-methodology artifact, coincidentally near the flight shake frequency (which is itself
suspicious but unconfirmable without better data — a genuinely rigid rotational fixture, bolting
the frame so it truly cannot rotate, would be needed to get a clean differential-mode
measurement; not attempted, deprioritized below).

**Recommendation: stop pursuing differential-bench testing without a rigid fixture — it will keep
hitting this wall.** Redirect effort to **F2 (in-flight extra-lag hunt)**, the piece still needed
to explain fact 4 (brushless shakes at kr≈600 despite its OWN measured actuator predicting
stability there, §16.2): a USD 500 Hz log during a low-kr brushless hover, comparing
`stabilizer.roll/pitch` phase against `gyro` to look for EKF-attitude lag in the eR path, and/or
an attitude-INDI-only hover (ctrl_mode=2) with position gains soft vs. tight to probe position-
loop coupling via Rd.

## 16.4 The kr≈600 gap — kw-independence rules out "wrong test point," implicates EKF attitude lag

Operator confirmed (2026-07-22): the shake was present at every kr tried from ~500-600 upward,
**regardless of which kw was paired with it** (170, 90, and intermediate values all tried).
A quick sim sweep at kr=600 with the brushless's own measured 44ms actuator shows a WIDE stable
window for kw≈50-120 (only unstable at the extremes, kw≤40 or kw≥150) — so kw=90 specifically
should have been clean per the actuator-lag-alone model. It wasn't. **This rules out "the earlier
sim just tested the wrong kw" and confirms the kr≈600 gap is real, not an artifact.**

The kw-independence is itself the clue: varying kw only changes the rate/damping term
(`kw·e_omega`); if it doesn't matter at all across a wide range, the missing phase lives in the
OTHER term — `kr·eR`, the attitude error. Checked in code (`firmware_app/src/lib.rs:1496-1500`):
`eR` is built from `st.attitudeQuaternion` — **the EKF's estimated attitude**, not raw gyro. A
fixed phase lag in the EKF attitude output would be invisible to any kw sweep (matches
observation) and would explain why geometric (much lower bandwidth, ~5.7× less stiff even than
our lowest tested INDI point) stays smooth on the same drone with the same EKF — enough margin
to absorb a lag that INDI's higher bandwidth cannot.

**New leading candidate for the residual gap: EKF attitude-estimate phase lag, not actuator lag,
not kw/rate-loop related.** Sharper than the prior generic candidate list (§4 fact 4). Next test:
500 Hz USD log comparing `stabilizer.roll/pitch` (the same signal feeding `eR`) against gyro-
integrated short-window dead-reckoning during hover — would give a direct phase/delay number at
~6 Hz. Not yet done. Handoff prompt and this doc both need updating with this sharpened lead.

Operator additionally confirmed: at gains low enough ("super low"), the shake disappears but
tracking error becomes bad — consistent with, not contradicting, the picture above (matches the
original tuning-history fact, Appendix B #3, that low gains give smooth-but-bad tracking on every
platform). Full shape: smooth-but-bad below some threshold → shake reappears somewhere before
kr≈500-600 regardless of kw → worse with rising kr. Consistent with a fixed EKF-lag phase penalty
that only bites once bandwidth crosses a threshold — low enough bandwidth never reaches it.

## 16.5 USD (SD-card) logging session — prepared, checklist of what it must answer

Config verified 2026-07-22: `tools/usd_indi_diagnostic_config.txt` — all 20 variables confirmed
to exist in the firmware source (gyro.x/y/z, indi.tau_x/y, indi.alp_x/y, **indi.alp_raw_x/y
(added — was missing, needed for the filter-aliasing question below)**, motor.m1-4, rpm.m1-4,
motor.m1_rpm, stabilizer.roll/pitch), syntax matches the reference config exactly, byte budget
(~70 B/sample × 500 Hz ≈ 35 KB/s) trivial for the 4096 B buffer.

**Practical procedure:**
1. Copy the config file to the SD card root as `config.txt` (physical access to the card needed —
   the deck doesn't expose it over radio; requires a card reader).
2. **Fly WITHOUT `--onboard`** for this diagnostic session. Checked: `flight.py` auto-enables/
   disables `usd.logging` only in the non-onboard (HLC) path (line ~1161/1131) — the `--onboard`
   Mode D path never toggles it, so a manual cfclient param flip would otherwise be needed. Using
   plain HLC playback avoids that; it doesn't need the rest-to-rest wrap for a diagnostic flight.
3. Fly a hover + one maneuver that reliably shows the shake. **Ideally two short sessions**: one
   at the flown kr=2400/kw=170, and — if time allows — one at kr≈600 with kw ALSO at 170 (not
   scaled down), since that's the exact pairing confirmed shaky in §16.4 and is the one most
   informative for the EKF-lag test.
4. Physically retrieve the log file from the SD card afterward (card reader again).
5. Decode with `tools/decode_usd_log.py <file>` — converts the raw binary (firmware variable
   names, ms timestamps) into a numpy dict matching our usual CSV naming convention (this format
   difference is real: USD logs are NOT structurally the same as the radio-telemetry CSVs, just
   the same underlying physical signals at a different rate/format).

**What this one session must answer (do not repeat without addressing all three):**
1. **Filter aliasing (raised 2026-07-22):** compare `alp_raw` vs `alp_x` (filtered) at the true
   500 Hz rate, no decimation ambiguity — does the filter meaningfully attenuate the 6-7 Hz
   content once seen without alias-fold risk, contradicting the 100 Hz-log finding of filt/raw≈1?
2. **EKF attitude-estimate phase lag (§16.4, the current leading candidate for the kr≈600 gap):**
   compare `stabilizer.roll/pitch` (the exact signal feeding `eR`) against gyro-integrated short-
   window dead-reckoning — direct phase/delay number at ~6 Hz.
3. **In-flight actuator confirmation, trap-free (§16, §16.3):** `motor.mX` (commanded, an output)
   vs `rpm.mX` (measured, a sensor) — NOT subject to the closed-loop −1/C bias (§15.2) since
   neither is derived from inside the running control loop. Does the bench-measured 44 ms/linear
   result (single motor, static rig) hold under real 4-motor differential load and aerodynamics?
   Bonus: cross-check `motor.m1_rpm` (DShot) against `rpm.m1` (optical) at full rate.
4. True RPM sensor update rate, directly (was inferred at ~20 Hz from 100 Hz-log decimation
   patterns, §15.4) — confirm or correct.

## 17. Verification plan for the clamps + Issue B fix (2026-07-22, brushless, not yet flown)

Issue A (the 6.3 Hz cycle) is intentionally left OPEN per §16.3 — nothing below is expected to
touch it. This section verifies the two SHIPPED fixes: output clamps (§15.10) and the rest-to-
rest entry/exit timeline (§9), which are independent of the actuator root cause.

**Ordered steps** (tools/check_new_params.py added for step 2):
1. `cd firmware_app && make cload` — reflashes clamps + traj_locate timeline + act_tau together.
2. `check_new_params.py` (no motors, no ROS) — confirms the new params exist on the flashed
   firmware and reads their live values before risking a flight on a stale flash.
3. Restart the CS2 server so it re-reads `crazyflies.yaml` (clamp_en=15 etc.) and reconnects to
   the new firmware's param TOC — required because CS2 pushes `firmware_params` once at connect.
4. **Hover sanity flight** (new code — buffers, widened `traj.ci`, clamp math — de-risk before
   trajectories): `ros2 run crazyflie_examples flight -- --trajectory hover --duration 15
   --brushless`. Confirms normal arm/hover/land with the new firmware; clamps are live here too
   (clamp_torque runs in both controller branches) but should never engage at hover.
5. **Issue B isolation flight (geometric):** temporarily set the active CF21BL block's
   `indi_gains.ctrl_mode: 0` in crazyflies.yaml, restart the server, fly one re-exported periodic
   trajectory: `ros2 run crazyflie_examples flight -- --trajectory oval --mode 1 --kt 0.05
   --onboard --reps 2 --brushless`. Isolates the rest-to-rest fix from INDI's noise. Revert
   `ctrl_mode: 3` and restart the server afterward.
6. **Combined flight (real target config):** same command as step 5 but with the yaml reverted
   to `ctrl_mode: 3` — exercises clamps + timeline + INDI together, the actual intended state.
7. **Analyze both logs** (`Controls/analyze_flight.py` + a manual look at `gyro_x/y`,
   `roll_deg/pitch_deg` in the first/last ~1 s):
   - Issue B PASS: no large single-tick transient at the arm tick or the disarm tick — only the
     gentle multi-second wind-up/wind-down ramp from `wrap_rest_to_rest`.
   - Issue A regression check (expected UNCHANGED): 6.3 Hz gyro oscillation still present
     mid-flight in the INDI log — confirms nothing about the actuator was touched.
   - Clamp sanity: `tau_x/y` never railed at ±14 mNm / tilt never railed at 30° during normal
     flight (rail-fraction ≈ 0 like the clean fig8/hover logs, unlike the railed oval blow-up).
8. Update this doc + memory with pass/fail per item.

## 18. First verification flights, brushless (2026-07-22 evening, 21 logs) — real results

Flights across circle/corner/figure8/helix/hover/oval/slalom/teardrop_wide/tilted_oval, mixing
geometric (Issue-B isolation), our full INDI, and stock firmware INDI. Full numeric table +
scripts in scratchpad `batch_analyze.py`. Four operator-raised points, addressed with data:

### 18.1 Rest-to-rest entry — confirmed working, but via a "swing-out" not a straight departure
Traced oval kt=0.05 (geometric) at 100 Hz through the actual entry segment (isolated by the
known `_log_t0` reset at `traj.start`, not by a speed threshold — the pre-position `goTo` looks
identical to real motion and must be excluded). Measured: drone rests at the lap-start point,
**dips 0.41 m in −y, then swings back through the exact same point** with vx≈0, vy≈+1.0 m/s
matching the core's true initial state — confirms the predicted (§9) wind-up excursion
(circle/oval calc gave ~0.33 m; measured 0.41 m, same mechanism, right ballpark).
**Root cause of the "special approach":** the entry Hermite segment is built with p0 = p1 (same
position at both ends, only velocity/accel/jerk differ 0→lap-value) so the drone returns to
exactly the pre-position point — deliberate, so a single `goTo` target serves both entry-start
and exit-end. But matching equal boundary POSITIONS with very different boundary VELOCITIES
forces a degree-7 polynomial through a net-zero-displacement S-curve — it must "pay back" the
implied forward drift by first moving backward. This is real and reproducible, not a bug, but
it costs extra space and looks non-intuitive.
**Concrete alternative for later, if this needs to change:** stop pinning entry start = exit end
to the SAME point. Place the rest point *behind* the lap-start along the reverse tangent
direction (offset ≈ `v_lap · t_e / 2`) and solve entry+exit as a proper 2-waypoint QP segment
(not a single hand-built Hermite blend) into/out of the core. That gives a straight, monotonic
lead-in — no swing-out — at the cost of the pre-position `goTo` target needing to become that
offset point instead of the literal lap-start coordinate, and a QP solve replacing the closed-form
Hermite. Not implemented; flag for a future session if the swing-out proves operationally annoying
(room/space) rather than just visually odd.

### 18.2 Full-INDI kt ceiling on oval — genuinely pushed further, real crash found at kt=0.7
| kt | ctrl | gyro σ | peak Hz | 3–9 Hz band | outcome |
|---|---|---|---|---|---|
| 0.05 | geometric | 16.8 | 3.3 | 8% | clean |
| 0.2 | INDI (ours) | 31.3 | 7.0 | 49% | completed |
| 0.3 | INDI (ours) | 30.4 | 6.8 | 34% | completed |
| 0.5 | INDI (ours) | 70.6 | 7.0 | 44% | completed (shake rising) |
| 0.5 | geometric (retry) | 82.4 | 4.3 | 5% | completed — high σ is real maneuvering, not oscillation |
| 0.7 | INDI (ours) | 180.5 | 4.1* | 56% | **CRASH** (confirmed, §18.5) |

Band fraction (the oscillation signature, not raw σ) climbs 34→44% from kt=0.3→0.5, exactly the
loop-margin prediction (§16.3/§15.6): higher kt needs tighter tracking → effectively pushes
further past the actuator-limited stability margin → bigger cycle → eventual crash. **Working
ceiling for oval on this drone/gain set: ~kt 0.3–0.5; failure at 0.7.** This IS an improvement
over the original investigation's "kt≈0.2 shared ceiling" fact (O4) — plausibly because tonight's
clamps prevented some earlier-blocking saturation/blow-up dynamics — but the ceiling is still
set by Issue A, not removed by it.

### 18.3 Shake persists — confirmed, unchanged in kind, exactly as expected
Every INDI run (hover and oval, all kt) shows the 6.3–7.0 Hz band signature; every geometric run
(including corner/slalom's much higher raw σ from real aggressive maneuvering) shows <10% band
fraction — the diagnostic separating "real motion" from "the cycle" continues to work cleanly.
Nothing about tonight's fixes was expected to touch Issue A, and nothing did — consistent.

### 18.4 Stock firmware INDI (controller=3) — genuinely calm, but NOT yet a clean comparison
Hover: gyro σ 3.4, peak 3.5 Hz, band 34% (vs our INDI hover: σ 22.0, peak 7.0 Hz, band 67%) —
clearly calmer. Figure8 kt0.05: σ 12.6, band 5%. Oval kt=1: σ 15.3, band 9%. **No 6–7 Hz signature
anywhere under stock INDI in this batch.**
**But this cannot yet confirm "good tracking + no shake is achievable"** — two open confounds:
1. **Stock's own gains are unlogged and unknown.** `ctrlINDI`/`posCtrlIndi` are a completely
   separate param group from our `indi_gains`; the CSV meta only records OUR params (kr=2400 etc,
   irrelevant when controller=3). We do not know whether stock ran at firmware-default (gentle,
   generic CF2.1) gains or anything tuned for this drone — very likely the untuned default.
2. **The one tracking check available is confounded by different kt.** Oval kt=1 (stock) spans
   only 3.01 m in Y vs the expected 4.4 m (68%, corners cut) — but kt=1 is also simply a much
   faster/more demanding lap than kt=0.3 (which our INDI traced to 4.35 m, 99%). Cannot separate
   "worse tracking" from "just a harder trajectory" without matching kt.
**Exactly the operator-proposed next experiment stands as the resolving test:** raise stock
INDI's own gains (`ctrlINDI.ref_err_p/q` ≈ our kr, `ctrlINDI.ref_rate_p/q` ≈ our kw, `g1_p/q` is
its effectiveness estimate) to comparable aggressiveness, refly the SAME kt as an our-INDI run
that worked (e.g. oval kt=0.3), and see whether the 6–7 Hz band appears. Outcome interpretation:
appears → supports "any INDI hits this at matched aggressiveness, actuator-fundamental" (strengthens
§16 across the board); stays clean at matched tracking → real evidence something in OUR increment
structure (not just gain magnitude) is the differentiator — reopens the code-vs-official question.

### 18.4b Stock-INDI matched-gain test result (2026-07-22 later) — different failure, inconclusive
Raised stock's `ref_rate_p/q: 24→170`, `ref_err_p/q: 5→14.12` (§18.4 plan; edited permanently into
`controller_indi.h`, rebuilt/reflashed — `g1`/`g2`/`act_dyn`/filter cutoff deliberately untouched).
Flew hover 3× (2 pre-reflash confirming old defaults still calm: gyro σ 3.1–8.1, no 3–9 Hz peak;
1 post-reflash confirmed-raised):

| run | gyro σ x/y | roll/pitch range | peak Hz | 0.5–3 Hz band | 3–9 Hz band |
|---|---|---|---|---|---|
| old gains (×2) | 3.1–8.1 / 3.2–3.9 | ±2° | 3.0–5.3 | 27–50% | 29–45% |
| **raised gains** | **73.3 / 64.2** | **−11.5..+16.6° / −7.8..+11.4°** | **1.75** | **69%** | **12%** |

**Not our shake.** Raised-gain hover is 10–20× larger in gyro σ, with attitude swinging into the
tens of degrees during plain hover — but the energy sits at **1.75 Hz, low-frequency dominated**,
the OPPOSITE spectral signature from our 6.3–7 Hz narrowband cycle (3–9 Hz band actually LOWER
than the calm baseline). This is a different, more basic failure: raising `ref_rate`/`ref_err`
alone increases the DEMANDED torque per unit attitude error, while `g1` (stock's effectiveness
constant, still at its brushed-CF2.1-calibrated value) still converts that into motor command at
the OLD scale — the combination overdrives real authority, likely compounded by the position
outer loop (`posCtrlIndi`, also untouched, no longer tuned for what's now a much tighter inner
loop). **Conclusion: this test does not yet answer whether stock INDI hits our 6.3 Hz mechanism
at matched aggressiveness** — it hit a different, more basic gain/effectiveness mismatch first.
**To do this properly:** `g1` must be scaled too (convert our `J`-based torque law into stock's
motor-command-unit effectiveness via the mixer/thrust-curve constants), or step `ref_rate` up
gradually (24→50→80→120→170) watching specifically for the 3–9 Hz signature's onset before any
larger low-frequency mode can dominate. **Safety note:** reverted/flagged to revert
`stabilizer.controller` to 6 or the `controller_indi.h` edit before further hover/flight tests —
a ±16° hover excursion is a near-loss-of-control event, not to be repeated without a plan.

### 18.4c Full-match reflash (2026-07-22 later) — g1/g2/act_dyn/filt_cutoff also matched

Operator wanted stock INDI pushed as close as possible to our gains to properly test whether it
hits the same 6.3 Hz mechanism, not stop at the confound found in §18.4b. Discovered stock INDI's
output is NOT the ForceTorque path we verified earlier — `controller_indi.c:150` sets
`control->controlMode = controlModeLegacy`, so `u_in` goes through `powerDistributionLegacy`
(the older, non-SI-calibrated differential mixer: `m1=thrust-r+p+yaw` etc., `r=roll/2`), not the
ARM-geometry ForceTorque decomposition. `g1` is therefore NOT directly comparable to our `J` —
it needed converting through this different mixer using real bench data:

- `dRPM/d(raw command)` = 0.573 (from the brushless static curve, `bench_actuator_2026-07-22_17-12-22.csv`)
- `dF/d(raw)` = 2·kt·RPM_hover · dRPM/d(raw) = 6.76e-6 N/unit (kt=4.10e-10, RPM_hover≈14400)
- roll/pitch: legacy mixer moves 2 motors ±raw/2 → d(torque)/d(cmd) = ARM_M·2·dF/d(raw) = 4.78e-7 Nm/unit
  → **g1_p = g1_q = 4.78e-7 / J_xy(23.951e-6) = 0.01997** (stock default 0.0066146/0.0052125, ~3.0× larger)
- yaw: legacy mixer moves all 4 motors ± full raw (diagonal) → d(torque_z)/d(cmd) = TORQUE_RATIO·4·dF/d(raw) = 1.54e-7 Nm/unit
  → **(g1_r+g2) = 1.54e-7 / J_z(32.347e-6) = 0.00476** → g1_r=0.00463, g2=0.000134 (~3.1× stock)

Also matched: `act_dyn_p/q/r` — discovered this stores the filter coefficient α=dt/(dt+τ) at
500 Hz, NOT τ directly (stock default 0.03149 → τ=61.5 ms, coincidentally close to our bench
number but for the brushed CF2.1, not this drone). Set to our bench-measured **τ=44 ms** →
α=0.04348. `filt_cutoff` (8→60 Hz) matched to our `fc_bw`. All edited into
`controller_indi.h` (originals commented above each, easy revert), rebuilt, clean.

**Honest caveat carried forward:** the legacy-mixer conversion is less certain than our earlier
*verified* ForceTorque mixer reconstruction (§12) — Bitcraze's own comment in
`stabilizer_types.h` calls the legacy scheme cruder ("SI units...is probably a better option").
Treat g1/g2 as a best-effort physically-grounded starting point, not an exact calibration.
**Given §18.4b's ±16° wobble from a smaller change, fly this cautiously: short low hover first,
abort-ready, before any trajectory.** `check_stock_indi_gains.py` updated with all expected
post-reflash values for a pre-flight sanity check.

### 18.4d Full-match reflash FLOWN (2026-07-22, hover_mode0_2026-07-22_20-01-20.csv) — INSTANT CRASH

Config: full §18.4c match (ref_err/rate incl. yaw, g1/g2 legacy-mixer-derived, act_dyn=44ms,
filt_cutoff=60Hz), plain hover, controller 6→3 switch mid-flight (standard takeoff pattern).

**Timeline (unambiguous in the log):** t=0–6.0s normal geometric hover (±3° roll/pitch, gyro
<30°/s). **t=6.2s: controller switches to stock INDI (3). gyro_y immediately jumps to −29.6.**
t=6.4s: gyro_y=−159.6. t=6.6s: roll=+14.3° pitch=−31.3° gx=+354.5 gy=+661.6. **t=7.0s:
gx=+1168.7°/s** (free tumble). t=7.2–7.4s: roll sweeps 103°→174° (physically flips). From
t=7.6s: roll≈178°, pitch≈−10°, gyro≈0, thrust=0 — **inverted on the floor, motors cut.**
**Under 1.4 s from stable hover to fully crashed**, starting the instant the controller switched.

**This is qualitatively different from §18.4b's wobble** (27 s survived, growing then bounded
oscillation) and from our own 6.3 Hz limit cycle (bounded, narrowband) — a step-like explosive
divergence is the signature of POSITIVE FEEDBACK, not "gains too aggressive" in the usual sense.
Supporting evidence this isn't just gain magnitude: this config's `g1` was RAISED (~3×), which
*decreases* torque-per-error (1/g1 dropped 151→50) relative to §18.4b's config (g1 untouched,
1/g1 stayed at 151, MORE torque per error) — yet §18.4b only wobbled for 27 s while this flipped
instantly. Raw torque-per-error magnitude doesn't explain the ordering.

**Leading hypothesis: a sign/convention mismatch, unmasked by gain, not caused by it.** Stock's
own code comment (`controller_indi.c:209`) flags a documented convention subtlety: *"no sign
conversion as CF coords use left hand for positive pitch."* This HLC→stock-INDI setpoint path
has only ever been flight-tested against OUR geometric controller, never at real authority
against stock INDI's own internal frame assumptions. At low/default gain (or even §18.4b's
config, which never touched g1), a wrong-sign contribution is small enough to be invisible or
read as mere noise/wobble. Raising BOTH bandwidth (ref_rate/ref_err) AND effectiveness (g1)
together crosses a threshold where the same wrong-sign term becomes genuine positive feedback →
runaway in under a second. Consistent with gyro_y (pitch axis — the one with the flagged
convention comment) diverging first, roll following.

**STOPPED for the night. `stabilizer.controller` left at 6.** This is not a "reduce gains and
retry" situation — a latent sign/convention issue would still be present at any sufficiently
high gain. Before any further stock-INDI-authority testing: verify the setpoint sign convention
INTO stock INDI (bench/code review, not another flight) — check whether `setpoint->attitude.roll
/.pitch` as fed by our HLC path matches stock's assumed convention, independent of gain level.

### 18.5 Two real crashes found in this batch, and one false alarm — triaged from z/gyro tails
- **oval kt=0.7 (our INDI): REAL CRASH.** gyro_x σ in the last 2 s of flight = 271.6 (was ~30-70
  earlier) — a violent tumble right before impact, consistent with §18.2's growing cycle. Post-
  impact z free-drifts smoothly to −8.66 m while gyro flatlines ≈0 (stationary on the floor,
  EKF free-integrating with no new fix) — the classic post-crash EKF signature, not a live event.
- **teardrop_wide kt=0.05 GEOMETRIC (baseline, NOT INDI!): REAL CRASH, separate + unexplained.**
  gyro_x spikes to −297 deg/s at t≈9.4 s (violent), then the same post-crash flat-gyro/free-drift-z
  signature. This is NOT an Issue-A/INDI event — it happened under geometric, at the mildest kt.
  **Flagging as a new, separate, unresolved problem** — no root-cause data gathered tonight
  (could be mocap occlusion during the loop's steep pitch, a flight-space boundary hit, physical
  strike, or a geometric-controller issue specific to this vertical-loop shape). Needs its own
  investigation before teardrop_wide is trusted again.
- **oval kt=0.5 geometric retry: FALSE ALARM, not a crash.** z_end=−1.14 looked alarming, but
  gyro_x stays ≈0 through the entire tail and z merely OSCILLATES noisily between 0 and −2.4 m
  after a normal landing (~t=13.5 s, matching the expected duration) — post-landing EKF z jitter
  while sitting/being handled, not a control failure. Safe to disregard.
- **corner/slalom (geometric) high raw gyro σ (131.8/83.0) is REAL, not a red flag:** band
  fraction is low (7%/6%) — this is genuine aggressive banked-turn/weave maneuvering the shape
  demands, correctly distinguished from the Issue-A oscillation signature by the band-fraction
  metric. No action needed.

## 19. Optional stage-2 notch filter (2026-07-28) — implemented, not yet flight-tested

Full writeup: `docs/results_2026-07-15_brushless.md` §11. Direct response to §16.6/§8's finding that
the stage-1 Butterworth (fc_bw=60-70) does nothing at 7.22 Hz and that lowering it into that band
makes things worse (§4b-fc-bw-10). Added an optional, runtime-tunable band-reject (notch) biquad
targeting only the diagnosed 5-10 Hz band (default center 7.2 Hz, bandwidth 5 Hz), applied
symmetrically to `alpha_meas`, `alpha_ref`, and `tau_current` (all three, verified to always match
filter depth — see §11 for the full consistency table) so no phase mismatch is reintroduced between
the terms of the INDI increment law. Gated by `indi_gains.notch_en` (default 0 = byte-identical to
today). Logged on both radio (`indi_alp_notch` topic) and 500 Hz SD card (required raising
`MAX_USD_LOG_VARIABLES_PER_EVENT` 20→40 in `crazyflie-firmware`, since the SD config was already
full). Build-verified clean; next step is `make cload` + an A/B hover/figure-8 flight with
`notch_en` toggled, checking whether `tau_x/tau_y` visibly calm down at ~7 Hz. Does not address the
actuator-lag phase-loss term (82° at 7.22 Hz, the larger of the two measured contributors) — a
separate, unaddressed mechanism if the notch alone turns out insufficient.

## 6. Timeline

- **2026-07-21** — Flew full trajectory library (kt=0.05) both controllers. Symptoms O1–O4 logged.
- **2026-07-21** — Offline analysis: confirmed INDI-only 6.3 Hz limit cycle + broadband 5–50 Hz
  elevation (H0 ruled out); confirmed filt_tau/filt_order already on (H1 ruled out); **system-ID
  J ≈ 27e-6 ≈ model (H4 ruled out)**; found commanded torque ~20× not realized, G≈0 (H5).
- **2026-07-21 (re-eval)** — Operator supplied tuning history: same procedure all platforms, only
  brushless shakes, can't lower gains (wrecks tracking). Rest-to-rest audit (§8): periodic
  trajectories (circle/oval/tilted_oval/loop/teardrop) are NOT rest-to-rest (circle v=0.857) →
  confirms HB1 for Issue B.
- **2026-07-22** — Compared vs official crazyflie-firmware controller_indi.c + brushless mixer
  (§10): mixer reconstruction VERIFIED correct. Operator: RPM-based INDI is the better variant
  (act_dyn/command-base not needed), filter can't be lowered (8 crashed). "Brushless slower" (H5′)
  RETRACTED — brushless is faster. **Decisive operator fact: shake present at kr≈600 (=upgraded's
  value) while upgraded smooth at kr≈603 → NOT bandwidth (§12 retracted); brushless-specific.**
- **2026-07-22** — Operator: tried DShot ESC-telem RPM → broke INDI; optical deck works; similar
  numbers + similar phase lag. → **LEADING HYPOTHESIS (§14): RPM/gyro phase desync on fast brushless
  actuators** breaks the INDI increment cancellation. Prepared high-rate USD logging (§13) to measure
  the rpm-vs-gyro lag and confirm. Next: pull a USD log, quantify τ, add matching delay to α_meas.
- **2026-07-22 (session 2)** — Deep offline re-investigation (§15). Limit cycle confirmed AT HOVER,
  onset exactly at the geometric→INDI switch, stationary envelope (§15.1). Closed-loop-ID caveat
  identified: in-flight tau→alp transfer ≡ −1/C, §3.2's "20×/G≈0" reframed (§15.2). Harmonic
  balance ⇒ operative plant ≈ 0.23∠−83° at 6.3 Hz (§15.3). Onboard rpm-vs-gyro measured
  SYNCHRONIZED (±10°); telemetry S&H explains the apparent lag → **§14 demoted (H8)**; **new leading
  H7: amplitude-dependent actuator slew/lag** (§15.7). Firmware audit: bat-comp cubic active,
  DSHOT_BIDIRECTIONAL=y (§13 corrected), no timer conflict, oval 18-01-26 was a railed/blow-up
  flight (§15.5). Loop margins with measured P explain all platform facts (§15.6). Wrote bench
  open-loop actuator-ID scripts (tools/) + added motor.m1_rpm to USD config. Operator: RPM guard
  stays disabled (tried → crashes; stock C has none) → Appendix B fact 11.
- **2026-07-22 (session 2, cont.)** — Operator fact 12 (motors confirmed fast/strong, no braking
  asymmetry, follow commands) → H7 loss location reframed to ESC throttle processing (L1) after
  STM32 chain audited clean (§15.8); ESCs identified: Bitcraze Bluejay fork, configurable via
  esc-configurator.com passthrough. Stock-vs-ours clamp/order re-audit incl. the outer-loop file
  (§15.10): order/filters matched, stock clamps everything / we clamped nothing, clamps NOT the
  cycle cause. **Clamps implemented** (lib.rs + traj_iface.c + yaml, clamp_en bitmask so tilt
  clamp can be dropped for Phase-4 inverted flight; default off, brushless yaml arms all four;
  builds clean) — pending operator review. j_scale≈4 "compensation" explicitly warned against.
- **2026-07-22 (session 2, cont.)** — **Issue B fix implemented end-to-end** (§9): exporter
  rest-to-rest wrap (Hermite entry/exit, verified C³-exact numerically), firmware entry/core/exit
  timeline (`traj_locate()`, traj.n_entry/n_exit, TRAJ_MAX_SEGS 14, ci→u16), flight.py
  settle-on-error arming + always-write n_entry/n_exit. Confirmed the expected behaviour matches
  the planner's own semantics: overall start AND end at rest for any rep count, reps chained
  continuously — the planner was already correct; only the deployment wrapper was missing.
  Correction: flight.py's ctrl_mode switch was already ~5 s before traj start (not same-tick).
  All builds clean; pending review + reflash + re-export + geometric-first verification flight.

---

## Appendix A — Repositories, environments & key file paths

All three repos live under `/home/georg/Desktop/`. Investigation spans all of them.

| Repo / path | Role | Key files for this investigation |
|---|---|---|
| **flying_robot_course** `/home/georg/Desktop/flying_robot_course` (branch `brushless-port`) | The course repo — our controller, motion planning, analysis, docs | `flying_drone_stack/firmware_app/src/lib.rs` (OOT controller: geometric + INDI, `controller_step`), `firmware_app/traj_iface.c` (RPM bridge `rpm_get_all`, `indi` log group, CRTP params), `firmware_app/src/bin/export_poly4d.rs` (trajectory defs, periodic vs rest-to-rest), `Controls/analyze_flight.py` (analysis), `Controls/logs/*_2026-07-21_*.csv` (flight logs), this doc |
| **crazyswarm2** `/home/georg/Desktop/crazyswarm2` | CS2 flight stack (Mode E) | `crazyflie_examples/crazyflie_examples/flight.py` (flight script: arm/disarm handoff, goTo pre-position, keepalive), `crazyflie/config/crazyflies.yaml` (flown INDI gains + `ctrl_mode`, `stabilizer.controller`) |
| **crazyflie-firmware** `/home/georg/Desktop/crazyflie-firmware` | Official Bitcraze firmware — reference for comparison + build base | `src/modules/src/controller/controller_indi.c` + `.h` (stock INDI: g1/g2/act_dyn/filt), `src/modules/src/power_distribution_quadrotor.c` (mixer), `src/platform/interface/platform_defaults_cf21bl.h` (brushless thrust curve, ARM, THRUST2TORQUE), `configs/cf21bl_defconfig` (DShot config), `src/deck/drivers/src/rpm.c` (optical RPM deck), `src/drivers/src/motors.c` (motor PWM + DShot telem logs), `tools/usdlog/{config.txt,cfusdlog.py}` (USD logging) |
| **imrc-crazyflie-firmware** `/home/georg/Desktop/imrc-crazyflie-firmware` | IMRC firmware variant (secondary reference) | same layout as crazyflie-firmware; cross-check if configs differ |

Environments: Python `~/.pyenv/versions/flying_robots/bin/python`; firmware build `cd firmware_app && make cload` (DRONE=bl default); flight via CS2 `ros2 run crazyflie_examples flight -- ...`. USD diagnostic config: `flying_drone_stack/tools/usd_indi_diagnostic_config.txt`.

## Appendix B — Operator-provided ground-truth facts (do not re-derive; treat as given)

1. **Premise:** the fault is in **our software/firmware** (controller or flight script), NOT hardware
   or architecture — the firmware's *other* stock controllers fly smooth on the same drone.
2. **Only the brushless shakes.** Standard + upgraded (brushed) drones are smooth with the same INDI.
3. **Tuning was identical across platforms:** start at single-digit pos+att gains (smooth hover but
   catastrophic trajectory tracking) → raise until tracking-error bottoms out → lock. Landed at
   kr=2400 (brushless) / 1050 (standard) / 603 (upgraded).
4. **Shake is NOT bandwidth:** present on brushless at **kr≈600** (= upgraded's value) while upgraded
   is smooth at kr≈603. Lowering gains does not remove it (and wrecks tracking).
5. **RPM:** DShot ESC-telemetry RPM **broke** INDI; optical deck works; both give similar numbers but
   similar phase lag. Optical deck used on all platforms.
6. **Filter:** fc_bw 60–70 = best tracking; 8 = crashed; smaller already tried.
7. **ctrl_mode 3→2 (position-INDI off):** operator tried, no difference.
8. **Brushless motors react FASTER than brushed** (do not claim otherwise).
9. **Two issues:** A = INDI-only oscillation; B = start/end jerk on BOTH controllers (separate bug).
10. **Trajectory requirement:** ALL trajectories must be rest-to-rest at the overall start AND end
    (v=0); for multi-rep, only first entry + last exit are rest-to-rest and reps chain continuously
    (no stop mid-sequence). Periodic exports currently violate this.
11. **RPM guard stays disabled** (`ENABLE_RPM_GUARD=false`): enabling it was tried and led to
    crashes; the stock C firmware has no equivalent and INDI works on the brushed drones without it.
    Do not re-propose it. (Data agrees: zero dropouts/zeros in all 07-21 logs.)
12. **The brushless motors are CONFIRMED to produce much more thrust much more quickly than the
    brushed motors** (operator, 2026-07-22). There is definitely NO braking/spool-down asymmetry,
    and the motors are not "slow" in any sense — they follow commands. Any claim that the physical
    motor/rotor can't keep up is wrong. If commanded modulation is not appearing at the rotor
    (§15.9), the loss is in the signal path (ESC throttle processing) or the measurement — not in
    motor capability. The brushless is also better than brushed on tracking; the shake (and possibly
    the shared kt≈0.2 speed ceiling) is what holds it back.

## Appendix C — key facts / constants (brushless, flown config)
- INDI gains: kr=2400, kw=170 (kr_z/kw_z same), fc_bw=60, filt_order=1, filt_tau=1, j_scale=1.0,
  ff_free=0, ctrl_mode=3, mass=0.041, kt≈4.1e-10.
- Geometric attitude gains (firmware const): KR=0.010, KW=0.0011.
- Inertia model (brushless, Busetto 2025 sim default): JXX=JYY=23.951e-6, JZZ=32.347e-6.
- System-ID'd inertia (2026-07-21, from RPM² torque vs dω/dt): Jxx≈27e-6, Jyy≈28e-6.
- Analysis scripts: ad-hoc (scratchpad); logs in `Controls/logs/*_2026-07-21_*.csv`.
