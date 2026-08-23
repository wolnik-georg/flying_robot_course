# Handoff prompt — deep re-investigation of the brushless INDI oscillation

Copy everything below the line into a fresh agent session. It is written to be model-agnostic and
to drive a rigorous, from-scratch re-investigation without repeating dead ends.

---

## Situation (short)

A Crazyflie 2.1 **Brushless** (CF21BL) running our out-of-tree Rust controller has two open flight
problems:

- **Issue A (primary):** with **full INDI** attitude control, the drone attitude **oscillates/limit-
  cycles at ~6.3 Hz** the entire flight and never settles. The **geometric** controller on the *same
  drone/trajectory is smooth*. The two brushed drones (standard, upgraded) run the *same* INDI code
  and are smooth. **Only the brushless shakes.**
- **Issue B (secondary):** an aggressive attitude **jerk at the start and end of every trajectory**,
  under **both** controllers (so it is upstream of the controller).

We have investigated extensively and **not yet confirmed a root cause**. A detailed living document
records everything — symptoms, all hypotheses with ruled-out status and evidence, the code comparison
against the official firmware, operator ground-truth facts, and the current leading hypothesis:

**READ FIRST (authoritative, detailed):**
`flying_drone_stack/docs/investigation_indi_oscillation_2026-07-21.md`

That doc's **Appendix A** lists every relevant repo and file path (our course repo, crazyswarm2,
and the official crazyflie-firmware). Its **Appendix B** lists operator-provided facts that are
GROUND TRUTH — do not re-derive or contradict them (e.g. brushless motors are *faster* than brushed;
the shake persists at low gains; it is a software/firmware issue, not hardware). Its **§4** is the
hypothesis table — **do not re-run any experiment already marked RULED OUT.**

**Current leading hypothesis (§14):** the INDI increment `τ = τ_current + J·(α_ref − α_meas)` only
cancels correctly if `τ_current` (from RPM) and `J·α_meas` (from differentiated gyro) are phase-
synchronized. On the *fast* brushless actuators the gyro is fresh but the optical-RPM sensor lag is
fixed, so `τ_current` is stale → the cancellation breaks → a ~90°-shifted residual torque drives the
limit cycle. On slow brushed motors gyro and RPM lag together, so it stays consistent. Not yet
confirmed. The immediate next step is a **high-rate (500 Hz) USD log** to measure the rpm-vs-gyro
phase lag (config ready: `flying_drone_stack/tools/usd_indi_diagnostic_config.txt`).

Your job: **re-investigate deeply and independently.** Confirm or refute the leading hypothesis with
data, and either fix it or find the true cause. Keep the living doc updated. **Do not commit to git.**

---

## Investigation pipeline (best-practice software + robotics)

Follow this as a disciplined loop, not a checklist to rush. Update the living doc as you go.

### Phase 0 — Ingest & orient (read before touching anything)
- Read the investigation doc end to end, plus Appendix A's key files: our `firmware_app/src/lib.rs`
  (`controller_step`, the INDI branch, filters) and `traj_iface.c`; the official
  `crazyflie-firmware/src/modules/src/controller/controller_indi.c`; the brushless mixer
  `power_distribution_quadrotor.c` and `platform_defaults_cf21bl.h`; the flown `crazyflies.yaml`;
  and `Controls/analyze_flight.py`.
- Restate the problem and the operator ground-truth facts in your own words. If any plan step would
  contradict a ground-truth fact, discard it.

### Phase 1 — Reproduce & characterize from data (no assumptions)
- Re-derive the symptoms yourself from `Controls/logs/*_2026-07-21_*.csv` (100 Hz, ~20 Hz RPM —
  aliasing-limited; treat frequencies >~30 Hz with suspicion). Compare INDI vs geometric on matched
  trajectories: gyro/tau/α spectra, time series, the 6.3 Hz cycle, the start/end transients.
- Quantify, don't eyeball. State every number with its window and its uncertainty.

### Phase 2 — Model from first principles (theory before more experiments)
- Write the INDI inner-loop as a block diagram: reference → attitude/rate error → α_ref → increment
  (τ_current + J·(α_ref − α_meas)) → mixer → actuator → plant → gyro → (filter, differentiate) →
  α_meas. Mark every filter, every differentiation, every sensor, and the **delay/phase each adds**.
- Compute a phase/delay budget: at 6.3 Hz, 180° = 79 ms. Where does the phase margin go? Which
  elements differ between brushless and brushed at the *same* gains?
- Derive what the increment reduces to when terms cancel, and what residual appears when `τ_current`
  and `α_meas` are misaligned in time. Predict the signature each candidate cause would leave in data.

### Phase 3 — Compare against references (known-good)
- Diff our INDI structure against the official `controller_indi.c` (effectiveness g1 vs our J;
  actuator model act_dyn; filter cutoffs; base term) AND against the source papers (Tal & Karaman
  2020; Smeur et al. INDI — esp. the requirement that actuator feedback be *time-synchronized* with
  the gyro). Note deliberate, justified deviations (e.g. RPM-based τ_current is legitimately better
  *if* synchronized) vs. accidental ones.
- Use the *other platforms* as references: what is identical (code, mixer, filter, gains at a matched
  kr) vs. different (the physical actuator). The cause must live in a *difference*.

### Phase 4 — Targeted data collection (get the data the 100 Hz logs can't)
- Use high-rate **USD logging** (config `flying_drone_stack/tools/usd_indi_diagnostic_config.txt`,
  17 vars @ 500 Hz; copy to SD root as `config.txt`; parse with
  `crazyflie-firmware/tools/usdlog/cfusdlog.py`). Ask the operator to fly a short INDI hover + one
  oval, both at the current gains and (if safe) at kr≈600.
- Primary measurement: the **phase lag between `rpm.mX` and d`gyro`/dt** (and between commanded PWM
  `motor.mX` and `rpm.mX`, and `motor.mX` → d`gyro`/dt) — i.e. system-ID the actuator and the sensor
  chain. Also verify the true RPM update rate. This confirms/refutes the leading hypothesis and sizes
  any fix.

### Phase 5 — Controlled experiments (change ONE variable at a time)
- **Runtime params first (no reflash):** anything settable via `crazyflies.yaml` / cfclient (gains,
  ctrl_mode, ff_free, j_scale, fc_bw). Only escalate to firmware reflash when a runtime test can't
  isolate it. Re-confirm each result against the hypothesis table before concluding.
- Keep a strict experiment log (config, prediction, result, verdict) in the doc. Falsify aggressively.

### Phase 6 — Fix, verify, document
- Implement the smallest change the evidence supports (e.g. a matched delay/low-pass on the α_meas
  path to phase-align it with τ_current). Predict the expected data change *before* flying.
- Verify with a fresh high-rate log: did the 6.3 Hz cycle drop, and did tracking survive? Update the
  hypothesis table, the timeline, and the status header. If it didn't work, say so and why.

### Guardrails (non-negotiable)
- **Safety:** these are real flights that can crash. Prefer runtime tests; make one change at a time;
  keep gains within known-safe ranges; flag anything that could destabilize before proposing a flight.
- **Honesty:** distinguish measured fact from hypothesis. If the data can't resolve something, say
  what data would. Do not invent a root cause to look decisive. It is fine to conclude "need more data."
- **No git commits.** The operator commits. Update the living doc (it is the durable memory).
- **Don't re-run ruled-out experiments** (§4). Don't contradict Appendix B ground-truth facts.
- **Keep the doc current** after every meaningful step — a future session (or model) must be able to
  resume cold from it.

### Definition of done
Either: (a) a confirmed root cause backed by high-rate data + a fix that removes the 6.3 Hz cycle
while preserving tracking, verified in flight; or (b) a crisp, evidence-backed narrowing with the
exact next measurement/experiment specified — both fully written into the living doc.
