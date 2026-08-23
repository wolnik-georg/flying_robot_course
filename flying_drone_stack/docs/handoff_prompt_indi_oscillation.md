# Prompt — help find the root cause of a brushless INDI limit cycle

Copy everything below into the agent, together with the full investigation doc
(`investigation_indi_oscillation_2026-07-21.md`, now ~18 sections/§16-18 are the most recent
work). The doc is authoritative — this prompt is only the entry point.

---

## The problem

A Crazyflie 2.1 **Brushless** (CF21BL) running our out-of-tree Rust INDI attitude controller
oscillates at a stationary **6.3 Hz limit cycle** — even in plain hover (gyro σ 25 °/s vs 5 °/s
under our geometric controller on the same drone). The cycle starts the instant INDI engages.
Two brushed Crazyflies run the **identical INDI code** and are smooth at their own flown gains.
Goal: find and fix the brushless-specific root cause **without lowering gains** (destroys
tracking — already exhausted) — OR reach a well-evidenced conclusion that it can't be fixed in
software and the practical mitigation is the ceiling found in §18.2.

## What is MEASURED (not hypothesized) — do not re-derive

1. It is a genuine control limit cycle, not mechanical vibration/noise (§3.1, §15.1).
2. **Open-loop bench identification (§16, §16.2) — the actuator IS the root cause, measured
   directly, no longer inferred:** the brushless command→rotor chain is a clean **linear
   first-order lag, τ≈44 ms** (fc≈3.6 Hz), reproduced across two independent bench sessions. The
   thrust-upgraded (brushed) drone's chain is τ≈71 ms — genuinely *slower*, confirming the
   operator's ground truth that brushless motors are faster in absolute terms; the brushless
   drone's problem is that its *tuned gains* (kr=2400) are far more aggressive relative to its
   own (still nonzero) actuator lag than the brushed platforms' gains are relative to theirs.
3. **Simulator validation (§16.1, `tests/test_indi_actuator_lag.rs`) confirms the mechanism AND
   refutes the obvious fix:** feeding the measured 44 ms lag + our exact filter chain into a
   closed-loop sim reproduces the cycle at kr=2400. But an "actuator-model" (act_dyn-style)
   increment base with a *correctly matched* model is mathematically **equivalent** to our
   existing RPM-feedback base (both converge to the true applied torque) — so switching increment
   base variants is NOT a fix. The measured actuator imposes a **physical stable-kr ceiling
   (~kr 800 in the simplified 1-axis model)**; kr=2400 is ~3× past it.
4. **Known gap, SHARPENED (not resolved) — 2026-07-22 late session:** operator confirms the
   shake was present at every kr tried from ~500-600 upward **regardless of which kw was paired
   with it** (170, 90, and intermediate values all tried and all shook). A sim sweep shows kr=600
   with the brushless's own measured 44ms actuator has a WIDE stable window for kw≈50-120 — so
   kw=90 specifically should have been clean per the actuator-lag-alone model, and wasn't. This
   rules out "wrong kw tested" and confirms a real second effect beyond the bench-measured
   actuator. **The kw-independence is itself the clue:** varying kw only changes the rate/damping
   term; if it never helps, the missing phase lives in the OTHER term, `kr·eR` (attitude error).
   Checked in code: `eR` is built from `st.attitudeQuaternion` (`firmware_app/src/lib.rs:1496-
   1500`) — the EKF's ESTIMATED attitude, not raw gyro. A fixed EKF attitude-estimate phase lag
   would be invisible to any kw sweep (matches observation) and explains why geometric (much
   lower bandwidth on the same drone/EKF) stays smooth — enough margin to absorb a lag INDI's
   higher bandwidth cannot. Operator also confirms "super low" gains ARE smooth but track badly
   — consistent (not contradicting): smooth-but-bad below some threshold → shake reappears before
   kr≈500-600 regardless of kw → worse with rising kr, consistent with a FIXED lag that only bites
   once bandwidth crosses a threshold. **Leading candidate is now specifically EKF attitude-
   estimate phase lag — NOT actuator lag (already explains kr=2400 fine on its own), NOT kw/rate-
   loop-related (ruled out by the kw-independence).** Not yet measured. See doc §16.4.
5. **Differential-motor bench test (§16.3) — INCONCLUSIVE, not evidence either way.** Chirping
   two motors up / two down together (mimicking a real roll command) broke down above ~5 Hz on
   BOTH drones with a resonance signature (gain>1, phase-sign flip, coherence collapse) — almost
   certainly the bench mount's own rotational compliance (hand/tape lets the frame rock under a
   real torque command), not a drone property. Would need a genuinely rigid rotational fixture to
   resolve; not attempted.
6. **Methodological trap that already cost us once:** any transfer function estimated from
   in-flight closed-loop logs equals the controller inverse −1/C, not the plant (§15.2). Only
   open-loop bench excitation identifies the actuator. Do not propose flight-log-only analyses.
7. Ruled out with data (§4 table — do not re-run): mechanical resonance, filter order/cutoff, RPM-
   feedback path (ff_free null), J overestimate, bandwidth/tuning-edge (pre-bench belief),
   mixer geometry, position-INDI outer loop, RPM/gyro phase desync (§14→H8, demoted), linear
   force→PWM slope, timer conflicts, RPM dropouts, "add an actuator model" as a fix (§16.1,
   mathematically equivalent to existing RPM base), clamping (implemented, robustness only, does
   not touch the cycle — §15.10).

## Real-flight confirmation of the ceiling (§18.2, §18.5) — the cycle has a practical cost, not just an academic one

With clamps+rest-to-rest fixed (see next section), flew oval at a kt sweep under full INDI:
kt=0.2→0.3→0.5 completed with the oscillation band-fraction climbing 34%→44% (exactly the
loop-margin prediction — more aggressive tracking pushes further past the actuator-limited
margin); **kt=0.7 produced a REAL, confirmed crash** — verified via the gyro tail (σ=271.6 in
the last 2 s, a violent tumble) followed by the classic post-crash flat-gyro/free-falling-z
signature. This is a genuine improvement over the investigation's original "kt≈0.2 shared
ceiling" fact, but the ceiling is still set by this cycle, not removed by anything done so far.

**Separate, unrelated crash found in the same flight batch — do not confuse with this
investigation:** `teardrop_wide` at kt=0.05 under plain **geometric** (not INDI, the mildest
possible config) also crashed for real (gyro spike to −297°/s at t≈9.4s). This has nothing to do
with the INDI oscillation and has no root-cause data gathered — flagged as its own open problem,
out of scope for this handoff.

## Separately: Issue B (start/end jerk) — SOLVED, verified in flight, not open

Rest-to-rest entry/exit segments (§9), a firmware timeline (`traj_locate`), and output clamps are
implemented AND flight-verified (§18.1, §18.5). Not part of this handoff — mentioned only so a
fresh agent doesn't waste time on it. One residual note: the entry/exit mechanism produces a
visible "swing-out" (0.41 m measured) rather than a straight departure — cosmetic/spatial, not a
correctness bug; §18.1 has a proposed refinement if it matters later.

## New this session: an attempt to prove/disprove "it's not just our code" — inconclusive AND
## produced a real safety incident (§18.4b–d)

Tried running the drone under Bitcraze's **stock firmware INDI** (`stabilizer.controller=3`) at
gains matched to ours, to see if the same 6.3 Hz mechanism appears there too:
- At stock defaults: genuinely calm, no 6–7 Hz signature (but gains were gentle/generic-CF2.1,
  not comparable to ours — confound, not evidence).
- Raising only the bandwidth-equivalent gains (`ref_rate`/`ref_err`) while leaving stock's
  effectiveness constant (`g1`) untouched → a DIFFERENT failure: a large, low-frequency (1.75 Hz)
  wobble (±16° attitude) — a g1/effectiveness mismatch, not our cycle. Inconclusive.
- Deriving `g1`/`g2` properly through stock's actual output path (discovered it uses the OLDER
  `controlModeLegacy` mixer, not the ForceTorque path we'd verified — required a fresh physical
  derivation via bench data + mixer geometry, §18.4c) and reflashing with the full match
  (`g1`/`g2`/`act_dyn`=44ms/filter cutoff matched) → **flown, and it crashed within 1.4 seconds of
  the controller switch** — an instant, step-like, explosive divergence (gyro to >1000°/s),
  qualitatively different from the earlier wobble and from our own bounded 6.3 Hz cycle. Leading
  hypothesis: a **sign/coordinate-convention mismatch** in the HLC→stock-INDI setpoint path
  (stock's own code flags a pitch-sign subtlety, `controller_indi.c:209`, never previously
  exercised at real authority), unmasked by gain rather than caused by it. **This line is
  STOPPED — do not propose re-flying raised stock-INDI gains without first resolving the sign/
  convention question by code review, not by another flight.**

## What we want from you

Read the full doc first (§16–18, §16.4 especially — most recent and most load-bearing). Then:
(a) The core open question is item 4 above, now sharpened to a specific candidate: **is the
residual kr≈500-600+ shake (present regardless of kw) explained by phase lag in the EKF attitude
estimate feeding `eR`?** Propose how to measure this directly (a 500 Hz USD log comparing
`stabilizer.roll/pitch` against gyro-integrated dead-reckoning is the planned test, not yet
run) — prefer runtime-param/no-reflash experiments, and respect the closed-loop-ID trap (item 6).
(b) If you want to revisit the differential/multi-motor bench question (item 5), propose how to
build a genuinely rigid test fixture rather than repeating the same inconclusive hand/tape setup.
(c) The stock-INDI sign-convention question (last section) is a separate, self-contained code-
review task if you want to pursue it — it does not require flight to make progress.
Distinguish measured fact from hypothesis throughout; "need more data, here is exactly which" is
an acceptable conclusion. Do not re-run anything in the ruled-out list (item 7).
