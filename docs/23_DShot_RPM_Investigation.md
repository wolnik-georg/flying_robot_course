# DShot bidirectional RPM as an alternative to the optical deck

**Status 2026-09-11: prep only, nothing flown.** New firmware and tooling added and verified
at the desk; the actual re-test is queued for **after** the controller-validation card and gain
freeze, **before** two-drone flights — the same slot the operator asked for.

**Why this matters for the comparative study:** dropping the optical RPM deck removes reflective
markers from the airframe, which (a) is one less piece of hardware to fit per drone, (b) removes
a documented concern about markers changing the vehicle's aerodynamic surface and therefore the
interaction forces this thesis measures, and (c) DShot bidirectional telemetry is *already
running unconditionally* on every CF21BL — it costs nothing extra to read.

---

## 1. What actually happened in July — read from the primary sources, not restated from memory

Full detail: `results_2026-07-15_brushless.md` §1, §2, §4b; `dshot_rpm_source_plan.md` (written
2026-09-03, planning only, never executed until today).

**The crash (2026-07-15).** Full attitude INDI (`ctrl_mode=3`) on DShot diverged to ~180° within
0.2–0.5 s of the `ctrl_mode 0→3` switch, at **both** an aggressive gain point (`kr=603/kw=90`)
and a conservative one (`kr=100/kw=30`) — "conservative gains didn't fix it" is in the log
verbatim. Geometric hover on the same airframe was clean (<5°).

**H0 partition (2026-07-16).** `ctrl_mode=1` (position INDI only) was stable on DShot;
`ctrl_mode=2`/`3` (attitude INDI) diverged. **The attitude loop specifically, not position.**

**H1b — the fix that actually worked (2026-07-16).** Switching the RPM source from DShot to the
optical deck (`app-config-bl`: `CONFIG_DECK_FORCE="bcRpm"`) made INDI hover **stable** — the 7 Hz
divergent cycle was gone. This is real and not in question; something about the deck fixed a
genuine crash.

**But the mechanism explanation attached to that fix was revised twice, entirely on deck data:**

1. **2026-07-16/18, fc_bw + kw sweeps (still on the deck):** a *smaller*, **bounded** (not
   divergent) 5–8 Hz limit cycle remained in `tau_x`/`alp_x`/gyro even after the deck fix, pinned
   in frequency regardless of `fc_bw`. Working theory: brushless motor/prop **mechanical** lag
   (real rotor inertia, slower than brushed coreless), a hardware ceiling.
2. **2026-07-18, H1a (`indi_gains.ff_free`, still on the deck):** forced `tau_current = tau_prev`
   (RPM-feedback-free) while the deck was live and working. Gyro σ and oscillation band were
   **essentially unchanged** with feedback on vs. off (17.0 vs 17.7 °/s, same ~5–6 Hz band) —
   ruling out "RPM feedback path itself" as the driver of *this* bounded residual. Conclusion at
   the time: "motor-mechanical lag, hardware ceiling."
3. **2026-07-18, retraction (still on the deck):** that conclusion did not survive a check that
   should have been done first — the same FFT run on a **standard** (non-brushless) drone's own
   INDI log found the **same** oscillation, at ~1/3 the amplitude, not qualitatively different.
   Also, brushless tolerated `kr` up to 1800 without full divergence while the standard drone
   *crashed outright* at `kr`=1100–1300 — the opposite of what "brushless motors are laggier"
   predicts. **Corrected diagnosis: ordinary gain amplification of gyro-differentiation noise,
   present on every platform this project has flown INDI on, not a brushless- or RPM-source-
   specific hardware effect.** Gains were re-swept down to `kr=1500/kw=180` on the deck.

## 2. The gap this leaves — precisely what is still open

**The corrected diagnosis (step 3) was derived, and only ever tested, on the deck.** DShot was
never re-tried at the gains that diagnosis actually produced. Two genuinely different stories fit
the 2026-07-15 data equally well right now:

- **Story A (the original H1/H1b framing):** DShot's telemetry genuinely desyncs
  `tau_current` from `alpha_meas` badly enough to cause outright divergence, independent of
  gains. The deck fixes this because it has no such delay.
- **Story B (consistent with the July 18 retraction):** the 2026-07-15 crashes were "gains too
  aggressive for *any* RPM source" (`kr=603` is well above the eventual `kr=1500/kw=180` lock's
  *effective* damping — note `kw=90` there against the locked `kw=180`, i.e. ζ was much lower).
  Switching to the deck and re-tuning happened together; the tuning may be what actually fixed
  it, and DShot might be perfectly usable at the current locked gains.

**This has not been decided.** Whichever it is has real consequences: if A, the deck stays
required for INDI on this platform indefinitely. If B, DShot is a genuine option for every future
drone, with the marker/hardware benefits above.

## 3. What was added today, 2026-09-11 — all default-off, verified byte-identical

### 3a. Runtime RPM-source switch (previously compile-time only)

Before today, switching sources meant editing `traj_iface.c`'s `rpm_get_all()` and reflashing —
the 07-16 fix comment literally says "swap the two lines below back." That made a same-session
A/B impossible without stopping to rebuild between every attempt.

**New:** `indi_gains.rpm_source`, `PARAM_UINT8`, **default 0 = optical deck** (today's flight-
proven behaviour, byte-identical). `1` = DShot bidirectional telemetry (log group `motor`,
`m1_rpm`..`m4_rpm`). Two independently-cached `logVarId` arrays, so switching at runtime needs no
cache-invalidation logic — same pattern already used for `res_sign`/`filt_dt_us`/`filt_prewarp`
this session.

`flying_drone_stack/firmware_app/traj_iface.c` — see the block above `rpm_get_all()` for the full
reasoning inline. Verified: firmware builds clean for **both** `DRONE=bl` and `DRONE=std`
(the `#ifdef CONFIG_PLATFORM_CF21BL` branch this replaced only mattered on brushless; on
`DRONE=std` builds `rpm_source=1` would look for a `motor.m*_rpm` log var that plain DShot-bidir
brushed builds don't declare, and `rpm_get_all()` already guards a missing/invalid `logVarId` by
returning 0 — same "absent, fall back" behaviour as an unfitted deck, not a new failure mode).
Host build also clean (`rpm_get_all` is renamed away and stubbed for the simulator regardless,
so this change cannot affect sim behaviour at all — confirmed by reading `oot_host.c`).

### 3b. Logging — both sources readable concurrently, regardless of which one feeds INDI

`usd_dshot_investigation_config.txt` — a **separate** uSD config (35/40 variables, the standing
`usd_thesis_config.txt` is untouched and stays the default for real data collection) that logs
`rpm.m1..4` (deck) **and** `motor.m1_rpm..m4_rpm` (DShot) in the same file at 500 Hz, alongside
`tau_*`/`alp_*` (the "clean loop signal" per the 07-16 finding), raw gyro/accel, `e_r_norm`, and
`indi.dt_us` (this session's separate filter-rate finding, `docs/22` §2f — worth checking on any
new log regardless of what it's for).

`decode_usd_log.py`'s `RENAME` table was completed — it already had `rpm.m1..4` and
`motor.m1_rpm` mapped (from earlier, unfinished prep) but was missing `motor.m2_rpm`,
`motor.m3_rpm`, `motor.m4_rpm`, and `indi.dt_us`. Fixed; would otherwise have printed the
"unmapped channel" warning and silently dropped those columns.

### 3c. Analysis script

`flying_drone_stack/tools/investigate_dshot_rpm.py` — reports exactly the four numbers the July
investigation actually used, from a decoded log:

1. Gyro σ + FFT peak per axis (the physical oscillation)
2. `tau_x/y/z` σ + FFT peak (the "clean loop signal")
3. Deck-vs-DShot agreement: mean bias, %, and **measured lag in ms via cross-correlation** —
   this directly quantifies the "documented cmd→actuation delay" the CF21BL paper flags, instead
   of assuming a number
4. `dt_us` sanity (this session's unrelated filter-rate finding — free to check on any log)

Plus a 4-panel PNG (gyro, torque, deck-vs-DShot RPM overlay, `|e_R|`).

**Verified against synthetic data with a known, injected 4 ms delay and a known 7.2 Hz tone**
before trusting it: the script recovered 7.16 Hz (close enough given FFT bin resolution at 10 s /
500 Hz) and **exactly** the injected lag. A real sign-convention bug was caught and fixed during
this test — `np.correlate`'s native lag sign is the *opposite* of "does b lag a", and the first
draft reported −4 ms for a signal delayed by +4 ms. Do not trust the lag number from any tool
without this kind of check; a silently-flipped sign here would have meant reading DShot's lag as
a lead, or vice versa.

---

## 4. The actual test — lab only, after C.0's gain freeze, before two-drone flights

Do not run this until the controller-validation card has passed and gains are frozen — this
investigation is explicitly parked until then, per the operator's own sequencing.

1. **Install** `usd_dshot_investigation_config.txt` as `config.txt` on the card (same procedure
   as `README_usd_thesis_logging.md`, different file).
2. **Bench check**: confirm `indi_gains.rpm_source` reads **0** and is settable; read it back
   after setting `1` to confirm the write actually landed (this project has been burned by
   silent param-write failures before — verify, don't assume).
3. **H0 partition on DShot, at the CURRENT locked gains** — not the 2026-07-15 ones. Fly
   `rpm_source=1`, `ctrl_mode=1` (position INDI only), then `2`, then `3`. This is the test that
   was never run: does the attitude loop still diverge on DShot now that the gains are not
   2026-07-15-era aggressive ones.
   - **Diverges at `ctrl_mode≥2`, same as 2026-07-15** → confirms Story A (genuine DShot delay),
     independent of the gain-amplification finding. Deck stays required.
   - **Stable** → consistent with Story B. Proceed to step 4.
4. **A/B at identical gains, back-to-back**, `rpm_source=0` then `1`, same hover, same config —
   the same pattern as the 07-18 `ff_free` A/B. Run `investigate_dshot_rpm.py` on both logs,
   compare gyro σ, `tau_x` FFT peak, and the deck-vs-DShot lag number directly (both sources are
   logged in every flight with this config, regardless of which one fed INDI).

**Abort condition, same as the original plan:** any divergence at `ctrl_mode≥2` on DShot at the
current locked gains → do not gain-sweep chasing stability on DShot, that road was partially
walked in July without success. Revert to `rpm_source=0`, record the result, stop.

**If DShot proves stable and statistically indistinguishable from the deck**, it becomes a real
option for every future drone in the comparative study — genuinely useful given the operator's
stated concern about marker reliability and interaction-force contamination in tight multi-drone
formations, which is exactly where this thesis's measurements matter most.
