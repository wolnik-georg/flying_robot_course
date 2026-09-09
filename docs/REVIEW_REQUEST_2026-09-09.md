# Independent review request — 2026-09-09 crash investigation

You are auditing a debugging session on a Crazyflie 2.1 **Brushless (CF21BL)** research
platform. Six hover flights crashed in one lab session. Three separate root causes were
claimed and fixed. **Your job is to falsify those claims, not confirm them.** A wrong
conclusion here costs crashed hardware and corrupted thesis data, so treat every claim below
as unproven until you have checked it against the code and logs yourself.

Work from the repositories and logs directly. Do not trust the narrative in this document —
it is the hypothesis under test.

---

## 1. System under test

| Thing | Path |
|---|---|
| Main repo (firmware source, logs, docs) | `~/Desktop/flying_robot_course` |
| ROS 2 / Crazyswarm2 (flight scripts, yaml) | `~/Desktop/crazyswarm2` |
| Bitcraze firmware (upstream, **deliberately not forked**) | `~/Desktop/crazyflie-firmware` |
| Out-of-tree controller (Rust, no_std, 500 Hz) | `flying_drone_stack/firmware_app/src/lib.rs` |
| Params / log groups / C bridges | `flying_drone_stack/firmware_app/traj_iface.c` |
| Flight script actually used | `crazyswarm2/crazyflie_examples/crazyflie_examples/simple_flight.py` |
| Legacy flight script (frozen; still imported for internals) | `.../flight.py` |
| Hardware config | `crazyswarm2/crazyflie/config/crazyflies.yaml` |
| Flight logs | `flying_robot_course/Controls/logs/*.csv` |

**The known-good reference is the branch `finalized-version-for-INDI-project`, in both
repos.** It is the state in which geometric and INDI both flew correctly for weeks (July
2026). "Frozen" below always means that branch.

### Controller selection

- `stabilizer.controller`: `1`=PID, `2`=Mellinger, `3`=stock INDI, `5`=stock Lee, `6`=our OOT
- `indi_gains.ctrl_mode` (only read when `controller==6`): `0`=geometric SE(3),
  `1`=position INDI, `2`=attitude INDI, `3`=full INDI

### Critical unit trap (verify this is understood correctly in the code)

- Geometric computes `torque = -kr_geo*eR - kw_geo*eOmega` **directly in Nm**
- INDI's `kr`/`kw` produce an **angular acceleration** `[1/s²]`, later multiplied by
  `J ≈ 24e-6` to become torque

A value from one used in the other is wrong by ~1/J (~40000×). Several bugs below are
instances of exactly this.

---

## 2. The evidence base

All logs in `Controls/logs/`. Newer ones carry `# meta:full_*` lines with the complete
`firmware_params` block; older ones only carry a narrower `# meta:` set. Attitude columns are
`roll_deg`/`pitch_deg`, **degrees**, from `stabilizer.roll/pitch`.

Summary produced during the session (verify independently):

| Log | Controller | roll std | peak | Verdict |
|---|---|---|---|---|
| `hover_..._2026-09-09_18-51-42` | stock Lee `5` | **0.74°** | 5.2° | clean |
| `hover_..._2026-09-09_18-56-52` | stock INDI `3` | 17.97° | 132.7° | crashed |
| `hover_..._2026-09-07_17-56-32` | ours `6`/m0 geometric | 28.23° | 84.2° | crashed |
| `hover_..._2026-09-07_18-15-47` | ours `6`/m0 geometric | 15.86° | 32.8° | crashed |
| `hover_..._2026-09-09_18-11-34` | ours `6`/m0 geometric | 29.83° | 177.5° | crashed |
| `hover_..._2026-09-09_18-28-25` | ours `6`/m0 geometric | 31.77° | 179.6° | crashed |
| `hover_..._2026-09-07_17-47-37` | ours `6`/m3 full INDI | 19.46° | 84.3° | crashed |
| `hover_..._2026-09-07_18-18-22` | ours `6`/m3 full INDI | 17.72° | 51.2° | crashed |
| `hover_..._2026-09-09_17-34-06` | ours `6`/m3 full INDI | 16.07° | 67.6° | crashed |
| `hover_..._2026-09-09_18-31-42` | ours `6`/m3 full INDI | 23.70° | 107.3° | crashed |
| `hover_..._2026-09-09_18-45-44` | ours `6`/m3 full INDI | 28.28° | 106.9° | crashed |

Additional observations claimed:

- All flights hover cleanly (|roll| 0.3–1.8°) for the first ~6 s, then diverge at **t ≈ 6.06 s**
- t≈6.06 s is when `simple_flight` switches from the ramp phase to the trajectory phase
- RPM shows **0 % zero-samples until after** each divergence (so the RPM deck is not the cause)
- `indi.a_res_*` is **not** in the radio CSV — `flight.py` never subscribed to that topic, so
  the residual's actual magnitude is **not measurable from these logs**

---

## 3. Claims to audit

### Claim A — Geometric failed because it ran INDI's position gains

`crazyflies.yaml`'s `pos_gains` (`kp_xy=64, kv_xy=5`) were tuned and locked 2026-07-19 for
**INDI** at `kr=2400/kw=170`. Geometric's attitude loop is much slower (`kr_geo=0.010`, a
value tuned for the 27 g CF2.1, on a 41 g brushless airframe), so the cascade separation
those gains assume does not hold. Damping ratio `ζ = kv/(2√kp)` drops 0.63 → 0.31 while
stiffness rises 60 %, and roll rings up ~1.45×/s.

Supporting: `a_indi` is identically zero when `ctrl_mode==0` (`mode & 1 == 0`), so the
residual sign cannot be geometric's cause. Both 09-07 geometric flights record
`# meta:pos_kv_xy=5.0`.

**Fix:** `_select_pos_gains()` in `simple_flight.py` selects `kp_xy=40, kp_z=30, kv_xy=8,
kv_z=10` for `ctrl_mode==0`; INDI keeps the yaml block; stock controllers pass through
(they ignore `pos_gains.*` entirely).

**Verify:** that `a_indi` really is unreachable at `ctrl_mode=0`; that `40/8` is genuinely a
previously flown configuration and not invented; that the ζ arithmetic is right; that
selecting gains in the script rather than the yaml cannot desynchronise from what the log
records.

### Claim B — INDI failed because of the residual sign flip

Commit `fa2131a` (2026-08-23) changed the position loop from `.add(a_indi)` to
`.sub(a_indi)`. Every `ctrl_mode=3` flight since (5 of them) crashed; the frozen `.add`
version flew for weeks.

Claimed mechanism: this is not only a thrust-magnitude term. `f_d` → `thrust_vec = f_d*mass`
→ `desired_rot(thrust_vec, yaw_d)` → `Rd` → `eR` → the attitude law. So the sign changes the
**commanded attitude**, matching the observed roll divergence. `a_res` is RPM-derived and
lags by a measured 44–71 ms actuator τ, so subtracting a lagged image of the vehicle's own
response is a plausible oscillation mechanism.

Counter-consideration explicitly noted: the derivation for `.sub` is believed correct
(`m*a = f_thrust + f_res + m*g`), and simulation measured the `.add` version reinforcing a
known 20 mN disturbance at exactly 2.00×.

**Fix:** runtime param `indi_gains.res_sign`, **default `+1`** (frozen behaviour). The
derivation-correct `-1` is preserved and selectable without a reflash.

**Verify:** the `f_d → Rd → eR` chain actually exists as claimed; whether the lag argument is
sound or hand-waving; whether some *other* post-frozen change better explains the INDI
failures; whether `.add` vs `.sub` can be distinguished in the existing logs at all (note
`a_res` is absent from the radio CSV — is it on the uSD logs?).

### Claim C — Stock INDI is a third, unrelated failure

`stabilizer.controller=3` crashed too, but it is **not** stock: `LOCAL_MODIFICATIONS.md`
records that `crazyflie-firmware/src/modules/interface/controller/controller_indi.h` has
`g1`/`g2` and filter cutoff re-derived locally for CF21BL. Stock Lee (`5`), which is
untouched bitcraze code, hovered cleanly — taken as proof the airframe/motors/mocap/EKF are
healthy.

**Verify:** that `controller_indi.h` really is modified; that Lee is genuinely untouched;
that "vehicle is healthy" follows from one clean Lee flight.

### Claim D — The mid-air handover is NOT the cause (an earlier conclusion, retracted)

Initially the ramp→trajectory controller handover at t≈6.06 s was blamed. This was
**retracted** after finding `flight.py` is **byte-identical** between `main` and frozen, and
that the brushless flew the same `ctrl_mode 0→3` handover with the same `40/8 → 64/5` gain
change through late July successfully.

**Verify this retraction is correct** — it is load-bearing. If the handover *is* the cause,
Claims A and B are both wrong. Check `git diff finalized-version-for-INDI-project..main --
crazyswarm2/crazyflie_examples/crazyflie_examples/flight.py`, and check which yaml block was
active in July (note the yaml contains several commented-out per-airframe blocks; the
*active* one on frozen is the **upgraded CF2.1**, not the brushless).

### Claim E — Landing bug

`simple_flight` streamed 2.5 s of `cmdFullState` (`_stream_hover_hold`) then called
`allcfs.land()`. `flight.py`'s own docstring on `_onboard_stream_land` states: *"cmdFullState
forces low-level mode; HLC land()/goTo() do not work afterward."* So the vehicle never
descended and `arm(False)` cut motors from ~0.7 m. Logging was also stopped *before*
`land()`, so no descent was ever recorded.

**Fix:** removed the streaming and the stop-notify; sequence is now the plain `figure8.py`
one. Logging runs through descent.

**Verify:** that simple_flight is genuinely Mode E only (never enters low-level mode), so
removing the handback is safe.

### Claims F/G — Two latent bugs found by inspection

- `traj_iface.c`: `g_indi_kr_geo` default was `100.0` where the law needs `0.010` Nm —
  10,000× too large. Only mattered once the geometric torque law started reading the param
  (commit `1f7829e`) instead of compile-time `KR_X`.
- `lib.rs`: gain selection used `if mode == 0` while the branch consuming it is
  `if mode & 2 != 0` … `else`. So `ctrl_mode=1` (position INDI, which keeps a **geometric**
  attitude loop) would have received INDI's `kr=2400` in an Nm-unit law — ~240,000× torque.
  Never flown at mode 1; would have crashed instantly. Now `mode & 2 == 0`.

**Verify both**, especially that `mode & 2 == 0` is the correct condition and matches frozen's
effective behaviour at every mode 0–3.

---

## 4. Reverts applied to reach "frozen-equivalent"

| Item | Frozen | Was on main | Now |
|---|---|---|---|
| Residual sign | `.add(a_indi)` | `.sub(a_indi)` | `res_sign=+1` → `.add` |
| `frame_conv` | Faessler (unconditional) | `1` = Mellinger | `0` = Faessler |
| Residual net | did not exist | ran every tick | gated on `rnn.en && rnn.ready` |
| RPM guard | `mode != 0` | ran in all modes | `mode != 0` |
| `kr_geo` default | n/a (compile-time `KR_X=0.010`) | `100.0` | `0.010` |
| Gain selection | compile-time constants | `mode == 0` | `mode & 2 == 0` |

**Claimed:** all 26 code lines frozen has that current does not are *reachable-equivalent* at
the shipped defaults. **Audit that claim line by line** — it is the core assertion.

---

## 5. What must NOT be broken (thesis-critical)

These are additive instrumentation and must survive any correction you propose:

- `indi.a_res_*` — `a_meas − a_model = f_res/m`, **the** thesis measurement (downwash force)
- `indi.e_r_*` — geometric attitude error, logged under all ctrl_modes
- `rnn.*` — onboard residual network + weight upload (default off)
- `kr_geo/kw_geo` — geometric's independently tunable attitude gains
- `res_sign` — the sign fix, preserved as a runtime A/B
- SIL simulator bindings (`crazyflie-firmware/bindings/`, patch-preserved in
  `firmware_app/host/cffirmware_bindings.patch` — that repo tracks upstream and is not forked)
- Full `firmware_params` block appended into every flight CSV as `# meta:full_*`

Claimed proof they are outside the control path: toggling `res_sign` changes **0/100**
samples at `ctrl_mode=0` and **100/100** at `ctrl_mode=3`. **Re-derive this yourself.**

---

## 6. Known gaps in the verification (do not paper over these)

1. **No numerical frozen-vs-current A/B was possible.** The frozen branch has **no `host/`
   directory** — SIL bindings postdate it — so the frozen controller cannot be compiled for
   the host. Equivalence rests on line-by-line review, not measured output. *If you can
   devise a sound way to do this comparison, that is the single highest-value contribution.*
2. **Simulation cannot validate stability.** A sim hover produced roll/pitch of *exactly*
   0.000 — noiseless and undisturbed, so nothing excites the instability. `docs/09_Simulation.md`
   also records that sim needs ~2× the position damping of hardware for reasons unknown.
3. **`a_res` magnitude is unmeasured on hardware.** Absent from the radio CSV. Check whether
   uSD logs exist for these flights (`flying_drone_stack/tools/usd_thesis_config.txt`,
   `decode_usd_log.py`, `merge_usd_logs.py`) — if so, that is direct evidence for Claim B.
4. **Nothing has been re-flown.** Every fix is unvalidated on hardware.

---

## 7. What to deliver

1. **Per claim (A–G): CONFIRMED / REFUTED / UNPROVEN**, each with the specific file+line or
   log evidence you used. Refutations are more valuable than confirmations.
2. **Any cause that was missed.** Particularly: is there a single mechanism explaining *both*
   geometric and INDI failures that was wrongly split into two? Note geometric and INDI failed
   with similar magnitude (std 16–32°) — argue whether that similarity is coincidence.
3. **Any fix that is wrong or dangerous**, especially anything that could crash on the next
   flight.
4. **A verdict on the reverts** — is "frozen-equivalent" actually achieved, or is something
   still different?
5. **The safest next hardware test**, given a real vehicle and a person in the lab.

Prioritise: *what would crash the drone on the next flight* > *what corrupts thesis data* >
*what is merely untidy*.

Useful commits: main repo `5d206ac`, `f25f35a`, `c519d54`; crazyswarm2 `b456870`, `2124de4`,
`1f5825d`, `c83e928`, `835965c`. Sign flip originated in `fa2131a`. Compare everything
against `finalized-version-for-INDI-project`.
