# Audit findings — 2026-09-09 crash investigation review

Independent pass over the claims in `REVIEW_REQUEST_2026-09-09.md`, executed against the
repositories, git history, logs and the compiled controller. Verdicts use the requested
scale; every verdict names its evidence. Two new defects and one unlisted frozen-delta were
found; three claims survive with sharpened evidence, one is downgraded.

---

## Per-claim verdicts

### Claim A — geometric ran INDI's position gains → **CONFIRMED**, with stronger evidence than the brief cited

- `a_indi` unreachable at `ctrl_mode=0`: **proven by execution, all four modes.** Toggling
  `res_sign` on the compiled controller (RPM injected so `a_res ≠ 0`): mode 0 → 0/100
  samples differ, mode 1 → 100/100, mode 2 → 0/100, mode 3 → 100/100. Exactly the
  `mode & 1` gate at `lib.rs:1525`. The residual term cannot have caused the geometric
  crashes.
- ζ arithmetic checks out: `8/(2√40)=0.632`, `5/(2√64)=0.3125`.
- **Historical corroboration the brief missed:** `_RAMP_POS_GAINS` was introduced on
  2026-07-18 (`dc6a2df`) with a comment stating that high `kp_xy/kv_xy` destabilises the
  geometric loop, citing the July 1 divergence incidents. I.e. the project already *knew*
  geometric diverges at INDI-scale position gains — that is why the ramp was pinned to
  40/8. Geometric at 64/5 was never flown before 09-07 *because the ramp pin deliberately
  prevented it*.
- 40/8 is genuinely pre-existing, not invented: it is the yaml's own named fallback and the
  ramp config of every flight since 07-18.
- Fix verified in code: the trajectory-phase guard tuple
  `(6, 0, {40/30/8/10}) == (ramp)` compares equal for geometric → **no push at t≈6 s at
  all**; INDI's tuple differs → push, i.e. the July-proven behaviour, unchanged.

### Claim B — INDI failed because of the residual sign flip → **downgraded to UNPROVEN (attribution); the FIX is nevertheless complete**

The `f_d → thrust_vec → desired_rot() → Rd → eR` chain exists as claimed (verified at the
`rd_flatness = desired_rot(thrust_vec, yaw_d)` site), and the sign measurably moves the
command (0.033 N on 0.40 N hover ≈ 8 %).

But the 09-07 flash carried **three** unflown firmware deltas simultaneously, all live in
`ctrl_mode=3`: (1) the sign flip, (2) `rnn_predict()` + peer scan every 500 Hz tick,
(3) the RPM sanity guard running in all modes (different `rpm_prev` state at handover).
Five crashes cannot be attributed to (1) specifically — any of the three, or an
interaction, fits the data equally well. No log distinguishes them: `a_res` is absent from
the radio CSV, and **no uSD data for these flights exists in either repo** (only the
un-retrieved cards can settle it).

Why this doesn't change the action: commits `f25f35a` + `c519d54` reverted **all three**
deltas, so the fix-set is complete regardless of which member was the killer. The
`res_sign` A/B param is also exactly the right instrument to establish attribution later.

### Claim C — stock INDI is a third, unrelated failure → **CONFIRMED**

- `controller_indi.h` is locally modified (present in `git status`; `LOCAL_MODIFICATIONS.md`
  documents the CF21BL `g1`/`g2`/filter re-derivation).
- `controller_lee.c` is **clean** — `git diff HEAD` empty. Lee is genuinely untouched
  bitcraze code.
- Sharpening: the stock-INDI crash **reproduces a documented July result** — the
  oscillation investigation's "FULL-MATCH → FLOWN, INSTANT CRASH within 1.4 s of switch"
  (doc §18). This is a *known, previously observed* failure of the locally-matched stock
  INDI path, not a new phenomenon. It was unresolved then and remains unresolved.
- "Vehicle healthy from one Lee flight" is acceptable here because the crashes bracket the
  Lee flight in time (17:31…18:45 crash, 18:51 clean, 18:56 crash) — an environmental or
  hardware cause would have had to switch off and on again within minutes.

### Claim D — the handover is NOT the cause (retraction) → **CONFIRMED, with one correction to its evidence base**

- `flight.py` byte-identical: same blob `4eaa7f6` in working tree, `main`, and frozen.
- **Correction:** the frozen branch's *active* yaml block is the **upgraded CF2.1 at
  kp 40/8** — on frozen, ramp gains equal trajectory gains, so frozen itself never
  exercised a pos-gain change at handover. The real evidence that the 0→3 +
  40/8→64/5 handover flew successfully is the July 18–29 **brushless-active commit
  window** (`525b099` … `df52fdd`: ctrl_mode 3, kr 2400, mass 0.041, kp 64/kv 5), whose
  yaml comments record the successful kv=5 lock flights (RMSE 2.3–2.4 cm). The retraction
  stands; cite the commit window, not the frozen branch.

### Claim E — landing bug → **CONFIRMED**

No `cmdFullState`, `_stream_hover_hold` or `notify_setpoints_stop` call remains in
`simple_flight.py` (only comments). Everything it issues is HLC (`takeoff/goTo/
startTrajectory/land`), so it never enters low-level mode and there was never any
low-level state to hand back — the removed machinery was inapplicable, exactly as argued.
The `_firmware_idle_reset` in the `finally` block runs after land+disarm, on the ground;
harmless.

### Claims F/G — latent unit bugs → **both CONFIRMED**

- `g_indi_kr_geo` default now `0.010f` with corrected `[Nm]` unit comments
  (`traj_iface.c:234`); yaml carries the same values, so the default only matters on a
  failed param write — which is the correct failure posture.
- Gain selection `mode & 2 == 0` now matches the consuming branch (`mode & 2 != 0 … else`)
  for every mode 0–3. At the shipped defaults, modes 0 **and 1** receive exactly frozen's
  compile-time constants, which is frozen's effective behaviour. The mode-1 probe above
  (100/100 differ on `res_sign`) also confirms mode 1 executes the position-INDI +
  geometric-attitude combination as designed.

---

## New findings (not in the brief)

### N1 — `alpha_ref` warm-up delta at the handover: an **unlisted frozen-difference**, still present

`alpha_ref = alpha_des − kr_xy·eR − kw_xy·e_omega` uses the *selected* gain pair
(`lib.rs:1745`). On frozen, `kr_xy` was `g_indi_kr` (2400) in **every** mode, so during
the geometric ramp the passively-computed `alpha_ref` and the `bw_ref_*` Butterworth
states carried 2400-scale history into a 0→3 switch. On current, modes 0/1 use
`kr_geo = 0.010`, so those filters are warmed with near-`alpha_des` values and see a step
to 2400-scale at the switch — a transient of roughly 10–20 ms (fc_bw = 60 Hz), order
0.5 mNm at hover-typical eR. Also changes the *meaning* of the logged `alp_*` filter-char
signals in geometric mode. **Small, bounded, and not a plausible crash mechanism — but it
sits at exactly the handover instant and contradicts the "all 26 lines
reachable-equivalent" claim.** One-line fix if wanted: compute `alpha_ref` from
`g_indi_kr/kw` unconditionally (its Tal & Karaman definition), independent of the torque
gain selection.

### N2 — metadata self-contradiction in future logs (**data-integrity defect, introduced by the fix**)

For a geometric flight, `# meta:pos_kv_xy` records the *selected* 8.0 (via
`_yaml_pos_gains`, updated at `simple_flight.py:319`), while the appended
`# meta:full_pos_gains_kv_xy` records the yaml's 5.0 (dump of the installed file). The
same CSV will carry two contradictory statements about the flown position gains — exactly
the ambiguity class that cost this session two flights. Fix: `_append_full_gains_meta`
must also write the selected set (e.g. `# meta:full_selected_pos_*`), or overwrite the
`pos_gains` sub-block with the selection before dumping.

### N3 — the rnn gating revert **eliminates the predict-and-log-only state** (thesis regression, latent)

Frozen-thesis design (`firmware_app/CLAUDE.md`): `rnn.en=0` → predict and log only;
`en=1` → feed control. As committed (`lib.rs:1503`), the prediction is computed only when
`en && ready` — so there is **no configuration in which predictions are logged without
also feeding the control loop**. That kills the core evaluation instrument (predicted-vs-
measured residual on flights where it is *not* used, e.g. geometric-only training/eval
flights for Geometric+NN). Harmless for the immediate C.0 flights (`ready=0` keeps frozen
tick cost), but must be corrected before C.2/C.3. One-line fix: gate the *computation* on
`ready` alone (cost then exists only when weights were deliberately uploaded), gate `a_nn`
on `en` as now.

### Missed-cause analysis (deliverable 2)

A single mechanism for both controllers is **excluded** by the stock-INDI data point: it
ignores both `pos_gains.*` and `res_sign` (OOT params) yet crashed — so at least two
mechanisms are required, and stock-INDI's is a third (Claim C, the known July FULL-MATCH
crash). The similar magnitudes (std 16–32°) are what any attitude-loop divergence on this
airframe looks like once it saturates against the actuator/clamp limits — the 18-11/18-28
flights (clamp_en=0) reaching 177–180° while clamped flights peak lower is consistent with
saturation setting the amplitude, not with a shared cause.

---

## Verdict on the reverts (deliverable 4)

"Frozen-equivalent at the shipped defaults" is **achieved for the torque/thrust command
path** (spot-checks: residual term, gain selection, frame convention, windup guard, rnn
tick cost, RPM-guard state evolution) **except** for N1 (`alpha_ref`/`bw_ref_*` warm-up in
modes 0/1) and the two accepted, intended log-sink additions. N1 is the only unintended
behavioural difference found; severity low.

## Dangerous-fix check (deliverable 3)

Nothing found that would crash on the next flight. The two live defects (N2, N3) corrupt
*interpretation* and *future thesis data*, not flight behaviour. The `--pin-controller`
flag is correctly opt-in; taking off directly in INDI remains unflown and correctly not
the default.

## Safest next hardware test (deliverable 5)

1. **Retrieve the uSD cards before flying anything** — if 09-07/09-09 data is on them,
   Claim B's attribution may be resolvable at the desk (`a_res` sign/magnitude at the
   divergence), for free.
2. Geometric hover (`ctrl_mode=0`). No handover push now occurs; every element of this
   configuration has flown before. Pass: roll std single digits, bounded.
3. INDI hover (`ctrl_mode=3`) — the restored July path. If it still diverges, the cause
   pre-dates all of this session's deltas and the H0 partition applies: fly `ctrl_mode=2`
   (attitude INDI, residual-free by the mode-2 probe result) vs `ctrl_mode=1`.
4. Only then the `res_sign=-1` A/B, single flight, from hover.

---

*Method note: all four-mode `res_sign` probes run against the actual compiled SIL
controller with RPM injected; git evidence from blob hashes and `-S` pickaxe on both
repos; no claim accepted from the brief without an independent check. The numerical
frozen-vs-current A/B remains impossible (frozen predates the `host/` layer) — N1 was
found by line review, which is exactly the method's residual risk.*
