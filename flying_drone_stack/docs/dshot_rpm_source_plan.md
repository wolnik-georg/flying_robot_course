# Revisiting DShot as an RPM source for brushless INDI — plan

**Status 2026-09-03: planning only, nothing implemented yet.** Written while lab access is
blocked (until Monday); no firmware changes made. Goal: give DShot telemetry a fair,
correctly-scoped re-test as an INDI RPM source, without disturbing the optical-deck path that
is currently locked and working. **Optical RPM deck stays the default in every step below.**

## Why this is worth revisiting — the history, precisely

This was tried once already (2026-07-15/16/18, `results_2026-07-15_brushless.md` §1/§4b) and the
story is more nuanced than "DShot crashed, deck fixed it":

1. **2026-07-15**: full INDI (`ctrl_mode=3`) on DShot bidirectional telemetry crashed repeatedly,
   at both aggressive (kr=603/kw=90) and conservative (kr=100/kw=30, "conservative gains didn't
   fix") gain points. `H0` partition test: position-INDI-only (`ctrl_mode=1`) was stable on
   DShot; attitude-INDI (`ctrl_mode=2`/`3`) diverged — the problem was in the attitude
   incremental loop specifically, not position.
2. **2026-07-16**: switched the RPM source to the optical deck (`CONFIG_DECK_FORCE="bcRpm"`,
   `app-config-bl` line 15) → INDI hover became stable. Working hypothesis at the time: "DShot
   bidirectional telemetry has a documented cmd→actuation delay that desyncs `tau_current` from
   `alpha_meas`."
3. **2026-07-18, H1a test**: built `indi_gains.ff_free` (runtime uint8, forces
   `tau_current = tau_prev`, bypassing whichever RPM source is active — `firmware_app/src/lib.rs`
   line 1696, `traj_iface.c` line 232) specifically to test whether RPM feedback itself was
   driving a residual ~5–6 Hz oscillation seen even on the by-then-stable deck setup. Result:
   **oscillation unchanged with `ff_free=0` vs `1`** — same amplitude, same frequency band, gyro
   data identical either way (`results_2026-07-15_brushless.md` §4b-h1a-result, table at line 442).
   This ruled out "the RPM-feedback path itself" as the driver of *that* oscillation.
4. **Same day, retraction**: the initial "motor-mechanical lag, hardware ceiling" conclusion from
   step 3 was retracted after comparing against a **standard (non-brushless) drone's** own INDI
   logs — the same oscillation exists there too, at ~1/3 the amplitude. Not brushless-specific;
   an intrinsic property of the INDI incremental law (`tau_current + J·(alpha_ref_filt −
   alpha_meas)` differentiates noisy gyro data by construction) that gets amplified by high `kr`.
   Corrected diagnosis: **gain amplification, not any hardware or RPM-source effect.** Re-sweeping
   `kr`/`kw` on the deck found `kr=1500, kw=180` as a better operating point.

**The gap this leaves**: step 4's correction (gain amplification, not RPM source) was discovered
*and only ever tested* on the deck-based setup. **DShot was never re-tried at the corrected,
lower-oscillation gain point.** The original 2026-07-15 crashes might have been "flying with
gains too aggressive for any RPM source" — coincidentally diagnosed and fixed while testing on
the deck, not because the deck is inherently required. That is a real, well-motivated open
question, not yet closed either way.

## What this plan is NOT trying to do

- Not claiming DShot is proven safe — it genuinely might not be; the point is the question was
  never re-asked after the gains that actually mattered were found.
- Not replacing the deck as default. The deck's current config (locked, flying, 2.48 cm RMSE
  best-of-platform result per `results_2026-07-15_brushless.md`) is untouched by any of this.
- Not urgent, not blocking C.0/C.1. This is background prep for a future session, explicitly
  because lab access is unavailable right now and this needed no hardware to write.

## What already exists and doesn't need to be built

- **DShot telemetry itself is already always live**, independent of the deck: `app-config-bl`
  sets `CONFIG_MOTORS_ESC_PROTOCOL_DSHOT_BIDIRECTIONAL=y` unconditionally (this is how the
  brushless ESCs are driven at all) — `CONFIG_DECK_FORCE="bcRpm"` only additionally force-enables
  the *separate* optical deck driver on top. `flying_drone_stack/tools/bench_actuator_id.py`
  already logs DShot's `motor_m1` alongside the deck's `rpm_m1` **simultaneously** in the same
  CSV (confirmed 0.0 ms lag between the two value streams, `results_2026-07-15_brushless.md`
  line 1141-1144) — so both sources are readable concurrently today, at least for logging.
- **The runtime-toggle pattern is already established** three times over (`ff_free`, `filt_tau`,
  `notch_en` — all `PARAM_UINT8` in the `indi_gains` group, `traj_iface.c` lines 229-249, default
  off = byte-identical behaviour). Adding one more of the same shape is a known-safe pattern in
  this codebase, not a new kind of change.

## Proposed plan, in order

### Step 1 — find out whether tau_current's RPM read is source-selectable at runtime already, or compile-time only

Currently `CONFIG_DECK_FORCE` is a build-time flag (`app-config-bl`), and the comment on it says
"To return to DShot telemetry: delete CONFIG_DECK_FORCE below" — implying today it's an
either/or rebuild, not a runtime choice. Confirm this by reading `rpms_active`/`rpms_to_torque`'s
actual source selection in `lib.rs` (~line 1691-1702) end to end — is the deck vs. DShot choice
baked in by which deck driver responds, or is there already a mux point this could hook into.

### Step 2 — add `indi_gains.rpm_source` (runtime uint8, default 0 = deck)

Same shape as `ff_free`: `0` = optical deck (current default, unchanged behaviour), `1` = DShot.
No CS2 `flight.py` wiring needed at first (manual diagnostic knob via cfclient, same convention
as `ff_free`). Byte-identical to today when left at 0. This is the actual "keep deck as default,
DShot as an option" ask — a runtime switch, not a rebuild, so both can be A/B tested in one
flight session without reflashing between them.

### Step 3 — re-run the H0 partition test on DShot, at the CURRENT locked gains

Not the 2026-07-15 gains (`kr=603` or the conservative `kr=100` — both predate the gain-
amplification finding). Use whatever is the current production locked config
(`firmware_app/CLAUDE.md`'s active gains block) and fly `ctrl_mode=1` (position INDI only) then
`ctrl_mode=2`/`3` (attitude INDI) on `rpm_source=1` (DShot). This directly answers the open
question: does the attitude loop still diverge on DShot now that the gains aren't the
2026-07-15-era aggressive ones.

### Step 4 — if stable, A/B against the deck at identical gains

Same hover, same locked gains, `rpm_source=0` then `1`, back to back (same pattern as the H1a
A/B test) — compare gyro σ, `tau_x` FFT peak, position RMSE. If DShot is statistically
indistinguishable from the deck, DShot becomes a real option (no deck hardware dependency for
future drones). If it's measurably worse but not crashing, that's useful data on its own,
independent of whether it's ever adopted.

### Abort / stop conditions

- Any divergence at `ctrl_mode=2`/`3` on DShot at the current locked gains → same failure mode
  as 2026-07-15, not fixed by the intervening gain correction. Revert to `rpm_source=0`, park
  this investigation, note the result and move on — do not gain-sweep on DShot chasing stability,
  that road was already partially walked in 07-15 without success.
- This is explicitly lab-only from Step 3 onward. Steps 1-2 (reading existing code, adding the
  runtime param) can be prepared and even implemented+bench-tested (`host/test_residual_nn.py`-
  style, no drone needed) ahead of lab access, but nothing here should be flashed and flown
  without going through C.0 first, same as everything else currently queued.

## Where this sits relative to C.0-C.4

Not on the critical path. `docs/07_Thesis_Progress_Checklist.md`'s own framing already states
"nothing on the critical path is software any more" — this doesn't change that. It's parked
here as a ready-to-pick-up investigation for spare lab time, not a blocker or a new requirement
for the residual-learning campaign, which uses whichever RPM source is active (currently, and by
default going forward, the deck) without caring which one it is.
