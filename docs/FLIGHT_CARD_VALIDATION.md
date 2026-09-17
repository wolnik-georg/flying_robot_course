# Flight card — validate both controllers

> **⚠️ Bench-check table below is HISTORICAL (09-09→09-13).** As of 2026-09-16,
> `indi_gains.rpm_source` defaults to **1 (DShot)**, not 0 — the optical deck was found to
> silently drop 2 of 4 motors' RPM in flight and DShot is now the standing source. See
> `docs/lab_sessions/2026-09-16.md`. Everything else on this card (gains, filter params) is
> unaffected.

> **One page. Take this to the lab.** **Stage 1: three geometric flights, then ONE INDI
> flight on the restored config only.** `check_flight.py` after every single one, stop on the
> first FAIL. **Stage 2** (the new fixes, one variable per flight) begins only once stage 1
> passes.
>
> Why this exists: 2026-09-09, six hover flights crashed under both controllers. Three
> separate root causes were found and fixed and re-flown clean 2026-09-11 (stage 1). Stage 2
> and the H0 partition ran 09-11/09-12 — see the results below the tick sheet. Background:
> [`lab_sessions/2026-09-09.md`](lab_sessions/2026-09-09.md) · audit:
> [`REVIEW_FINDINGS_2026-09-09.md`](REVIEW_FINDINGS_2026-09-09.md)

---

## 0 · Once, before anything

**Pull the uSD cards from the 09-07/09-09 flights first** — free evidence, lost once overwritten.

```bash
cd ~/Desktop/flying_robot_course && git pull
cd flying_drone_stack/firmware_app && make cload

cd ~/Desktop/crazyswarm2 && git pull
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select crazyflie crazyflie_examples
. install/local_setup.bash
```

**Bench check — cfclient → Parameters:**

| Param | Must read |
|---|---|
| `indi_gains.res_sign` | **1** |
| `indi_gains.frame_conv` | **0** |
| `indi_gains.kr_geo` | **0.010** |
| `indi_gains.clamp_en` | **11** |
| `stabilizer.controller` | **6** |
| `indi_gains.res_fc` | **0** |
| `indi_gains.res_clamp` | **0** |
| `indi_gains.filt_dt_us` | **2000** |
| `indi_gains.filt_prewarp` | **0** |
| `indi_gains.rpm_source` | **0** |

The last five are new switches — **all must read their defaults for stage 1.**
`rpm_source` stays at **0 (deck)** for this entire session — DShot is a separate
investigation, see the closing note at the bottom of this card.

---

## 1 · GEOMETRIC — set `ctrl_mode: 0`

Edit `~/Desktop/crazyswarm2/crazyflie/config/crazyflies.yaml` → `all.firmware_params.indi_gains.ctrl_mode: 0`, then:

```bash
cd ~/Desktop/crazyswarm2
colcon build --symlink-install --packages-select crazyflie && . install/local_setup.bash

# terminal 1 — leave running
ros2 launch crazyflie launch.py
```

Confirm this line at takeoff: `position gains: {'kp_xy': 40.0, ...} <- GEOMETRIC_POS_GAINS`

```bash
# terminal 2 — one at a time
ros2 run crazyflie_examples simple_flight -- --trajectory hover   --duration 15
ros2 run crazyflie_examples simple_flight -- --trajectory circle  --kt 0.1
ros2 run crazyflie_examples simple_flight -- --trajectory figure8 --kt 0.008
```

## 2 · INDI — **ONE flight, restored config only** (`ctrl_mode: 3`)

Same edit, rebuild `crazyflie`, **Ctrl-C and relaunch the server**, confirm `ctrl_mode`=3 in cfclient. Expect `pos_gains 64/5` here — correct, that's INDI's locked block.

**Change nothing else.** INDI crashed 5/5. The restored config is the July one that flew for
weeks; this single flight answers whether the restoration worked, and only if it is the only
variable.

```bash
ros2 run crazyflie_examples simple_flight -- --trajectory hover --duration 15
```

**PASS** → stage 2. **FAIL** → stop; the cause pre-dates every fix → H0 partition
(`ctrl_mode=2`, then `1`).

## 2b · Stage 2 — one variable per flight, only if stage 1 passed

All runtime params; no reflash between them.

| # | Set | Tests |
|---|---|---|
| a | `res_fc=80`, `res_clamp=10` | conditioning the residual — the thing NA-INDI does and we never did |
| b | `filt_prewarp=1`, `filt_dt_us=1000`, `fc_bw=206` | **deliberate no-op** — same filtering, but `fc_bw` now means what it says |
| c | `fc_bw < 206` | only after (b); tuning is meaningful for the first time |
| d | `notch_en=1`, `notch_f0=6.9` | only after (b). The notch sat at 13.8 Hz — **it has never actually been tested** |
| e | `kr=987`, `kw=109` → then `632`/`87` | **the shake-band test.** Our ω_n is **7.80 Hz**, the shake is 6.3–7.9 Hz. Moves ω_n to 5.0 then 4.0 Hz at the same ζ=1.74. Shake follows → it's the attitude loop. Shake stays → `kr` exonerated. Expect worse RMSE — that's the trade |

⚠️ **Do not copy NA-INDI's frequency numbers.** Their filters carry the same 2× sample-rate
error *and* a different discretisation, so their "80 Hz" is neither our 80 nor their own.
Copy the *ratios*, not the labels.

⚠️ **Same for gains.** Converted to a common basis ours is **8.2× stiffer** than theirs
(0.0575 vs 0.0070 Nm/rad), ω_n **7.80 Hz** vs their 3.28 Hz. A different airframe justifies
some of that, not 8×. Ours were tuned for figure-8 tracking against the hover limit; theirs to
be comfortable. See `docs/22` §2h.

## 3 · After every single flight

```bash
cd ~/Desktop/flying_robot_course && python3 experiments/analysis/check_flight.py
```

**PASS** = `bounded`, std single digits. **FAIL** = `GROWING` or std > 10 → **stop, don't fly the next one.**

---

## Abort rules

| Symptom | Meaning |
|---|---|
| Amplitude visibly growing | kill it — that's the failure mode under test |
| Geometric hover fails | diagnosis wrong; stop the session |
| INDI fails, geometric passed | cause pre-dates all fixes → H0 partition: `ctrl_mode=2`, then `1` |
| Log ends above 0.20 m | landing fix didn't take |

**Stage 1 clean → stage 2, one variable per flight.** Retuning and multi-drone come after that.

---

## Tick sheet — RUN 2026-09-11, results filled in

Full account: [`lab_sessions/2026-09-11.md`](lab_sessions/2026-09-11.md).

| # | Mode | Trajectory | PASS / FAIL | roll std | peak | Notes |
|---|---|---|---|---|---|---|
| 1 | stage 1 | geo · hover | PASS | ~1.1° | — | clean |
| 2 | stage 1 | geo · circle | PASS* | — | — | *first attempt hit the ramp-ringing bug (±53°, fixed same session, see history 21); clean after the fix |
| 3 | stage 1 | geo · figure8 | PASS* | — | — | same ramp fix applied |
| 4 | stage 1 | INDI · hover, restored only | PASS | ~1.1-1.3° | — | clean |
| 5 | stage 2a | res_fc 80 + res_clamp 10 | PASS | — | — | clean, matches stage-1 baseline |
| 6 | stage 2b | prewarp/dt/fc_bw=206 (no-op) | PASS | 1.13° | — | confirmed genuine no-op |
| – | stage 2c | fc_bw=100 — tune down | **FAIL** | 6.82° | 17.3° | hover regressed PASS→FAIL; circle peak 27.5°→69.2°; reverted to 206 |
| – | stage 2d | notch_en=1, notch_f0=6.9 | **CRASHED** | — | 88.7°/66.6° | real impact within 0.7s of a stable hover; root cause unknown; `notch_en` reverted to 0 |
| 7 | stage 2e attempt 1 | kr=987 kw=109 (ω_n 5 Hz), pos_gains unchanged | aborted | — | — | manual kill ~2s in, diverging; cascade/pos_gains mismatch |
| 7b | stage 2e attempt 2 | kr=987 kw=109 + matched pos_gains (26/3.2) | PASS (hover) | 0.99° | — | best hover of the day; figure8 unchanged (17.6° vs 17.7° baseline) |
| 7c | stage 2e attempt 3 | kr=483 kw=76 + matched pos_gains (13/2.2) | FAIL | 1.67° | 11.4° | attitude peak improved; position holding broke (z overshoot/drift) |
| 7d | stage 2e attempt 4 | kr=483 kw=76 + geometric pos_gains (40/8/30/10) | FAIL | 2.33° | 6.7° | position holding fixed; small new hover growing trend |
| — | session close | reverted to kr=2400/kw=170, pos_gains 64/5/48/7 | — | — | — | last confirmed-clean, flight-proven config |

**Not reached**: `kr=632`/`kw=87` (ω_n 4 Hz) rung, and the card's own H0 partition
(`ctrl_mode=2` then `1`) — recommended as the next diagnostic given 4 straight gain attempts
converged on the same trajectory-tracking shake without a clean pass.

Push the logs when done — `git add Controls/logs && git commit && git push`.

---

## 2026-09-12, part 1 — H0 partition — DONE, inconclusive

`ctrl_mode=2` (attitude INDI alone) crashed sharply: roll peak **119.7°**, `z` to -0.017m,
onset exactly at the geometric→INDI handover tick (t=6.58s). `ctrl_mode=1` (position INDI
alone) also crashed, more gradually: roll peak **179.3°**, `z` to **-1.457m** (real impact),
onset ~t=11.2s (the `rpm_m1` dropout at t=19.2s is a symptom of the tumble, not its cause).
**Both far worse than full INDI** — the mildest config all session. Full INDI (`ctrl_mode=3`)
re-flown as a sanity check afterward: unchanged, clean hover, same figure8 shake magnitude.

**Conclusion: the H0 partition does not cleanly localize the shake.** Both sub-loops are
individually worse alone than the combined system — itself the useful negative result. The
`ctrl_mode=1` divergence has an explanation on file (pos_gains/attitude-bandwidth cascade
mismatch); the `ctrl_mode=2` sharp, handover-timed divergence does not yet have a second
concrete cause the way the notch crash does — left open.

**The stage 2d notch crash was root-caused offline the same day, separately**: the 2026-07-29
fix that hoisted the `alpha_meas`/`alpha_ref` notch-filter warm-up out of the mode-gated branch
missed a third chain — `tau_current`'s own Butterworth pre-filter (`bw_tau_x/y/z`), which only
started updating at the same tick as the controller handover. Fixed in
`firmware_app/src/lib.rs` (`bc7190c`), builds clean on both platforms — **not yet
reflashed/flown**, needs a real `notch_en=1` test next session.

## 2026-09-12, part 2 — first 2-drone flight — DONE, 6 bugs fixed

`cf_second` enabled for the first time as a standard CF2.1 (not brushless yet, to validate the
infra cheaply). Running `formation_flight.py` for real surfaced six genuine, durable bugs — all
fixed: missing `pos_gains` selection for `ctrl_mode=0` (the exact 2026-09-09 root cause, never
ported here); a mid-air controller-*identity*-switch risk; no hover support at all; arming
gated behind a removed `--brushless` flag; `LOG_DIR` hardcoded to a dev-machine path (crashed
the whole flight before landing on a different lab machine); a landing collision for vertical
(stacked) formations. Full detail: `docs/lab_sessions/2026-09-12.md`.

**First fully clean flight** (stock Lee, both drones): landed and logged correctly, no
exceptions. Real signal: `cf231_active` (lower, in the downwash) roll std 17.9°/peak 38.2° vs
`cf_second` (upper) 0.75°/peak 5.8° — stock Lee has no interaction-force awareness. `a_res`
exactly 0 for both, as expected (only computed under our own controller).

**Then our OOT geometric controller on both drones — both crashed.** `cf_second` oscillated
almost the whole flight (roll to 179.5°); `cf231_active` crashed within ~2s, coinciding with a
real `a_res_z` spike of 8.1 m/s². **Root cause, corrected twice on the way to the real answer**:
first suspected `kr_geo`/`kw_geo` as brushless-only — wrong, checked via git (bit-identical to a
2026-07-02 clean flight on standard). Real cause: the shared `all:` firmware config has drifted
brushless-only for two months, compounded by a bigger bug found later the same day —
`apply()`'s broadcast also carries `indi_gains.mass`/`kt1-4`, silently wiping `cf_second`'s
per-robot mass/`kt` override every call, including before takeoff — so it almost certainly flew
on brushless's thrust model (~2.8× off on `kt`), not its own. **This explains `cf_second`
(standard)'s own instability, not `cf231_active` (brushless)'s crash directly** — brushless's
config was never touched by this bug; its crash reads as collateral damage from the standard
drone thrashing directly above it, not a fault of its own. Fixed generally:
`load_per_robot_overrides()` now re-pushes any per-robot key after every broadcast
(`crazyswarm2` `fdfc640`).

## Architecture decision, 2026-09-13

`cf_second` is now **permanently pinned to stock Lee** (`crazyswarm2` `bb88e73`), regardless of
platform — only `cf231_active` is ever the controller under study. This removes the whole
cross-platform gain-sharing risk by construction (stock Lee never reads
`indi_gains`/`pos_gains`), and means **the brushless swap for `cf_second` is no longer needed**
for that reason — it can stay standard CF2.1 indefinitely.

## 2026-09-13 — notch test flown, CRASHED

The `notch_en=1` test was flown for real (`stabilizer.controller=6`, `ctrl_mode=3` full INDI,
matching the config the original crash happened under). **It crashed — worse than the
2026-09-11 crash — with zero log data saved.** The `bc7190c` fix does **not** resolve the
problem. `notch_en` reverted to 0 (`crazyswarm2` `3958a4e`). Drone inspected, no damage.

Most of the session before this was lost to a separate, genuine hardware/tooling problem: the
drone had been flashed with the wrong firmware platform (standard onto the brushless
airframe), compounded by a diagnostic command that accidentally forced it into bootloader
mode. Resolved with an explicit `make DRONE=bl` reflash. Full account:
`docs/lab_sessions/2026-09-13.md`.

## 2026-09-13 — offline notch analysis, CONCLUSIVE: abandon the notch approach

`experiments/analysis/notch_filter_bode.py` computes the real discrete frequency response of
the `Butterworth2`→`NotchFilter` chain at the true 1 kHz tick rate. **The attitude loop's own
natural frequency (`ωₙ = √kr = 7.80 Hz` at `kr=2400`) sits almost exactly inside the notch's
rejection band** (`f0=6.9`, `bw=3.0`, ~5.4–8.4 Hz) — phase swings from −89° to +85° in under
0.1 Hz right through that band, and even at 7.80 Hz the notch still applies +58° phase and
halves the signal. **The "shake" is likely the loop's own dynamics, not an external
resonance** — notching it corrupts the loop's own feedback at its operating point, which is a
structural problem no `notch_f0`/`notch_bw` retune can fix while `kr`/`kw` keep `ωₙ` at 7.8 Hz.
**Decision: the notch-filter approach is abandoned.** Continue the `kr`/`kw` + `pos_gains`
coordinated retuning line instead (best so far: 2026-09-11 attempt 4, `483/76` + geometric
`pos_gains`, peak 6.7°).

## Next steps — 2026-09-14 plan

**Neither our own geometric nor INDI has actually worked in a 2-drone scenario yet** — the
only clean 2-drone flight remains stock Lee (`a_res=0`, no real data); the one attempt at our
own geometric on both drones is what crashed on 2026-09-12.

1. **Geometric, 2 drones, A1 hover** (`cf231_active`=our geometric, `cf_second`=stock Lee) —
   closes C.0 flight 5 and *is* the first real C.1 data point (A1 under geometric).
2. **Full INDI, 2 drones, A1 hover** — deliberately ahead of C.0's strict required order
   (figure-8 under corrected INDI hasn't reached a clean pass) since A1 is pure hover and
   doesn't exercise the open trajectory-tracking shake — an informed decision, not an
   oversight.
3. ~~**Validate uSD logging + sync on both drones**~~ ✅ **done 2026-09-15** — chain validated end to end, first usable two-drone dataset, sync ~20–30 ms. See `docs/lab_sessions/2026-09-15.md`.
4. **If clean: move to A3.**

The `kr`/`kw` + `pos_gains` coordinated retuning line (the actual fix for the trajectory-
tracking shake, not the notch) continues separately and does not block this plan. Brushless
swap for `cf_second` explicitly deferred to its own session.

---

## DShot RPM — NOT today

`indi_gains.rpm_source` stays at **0 (deck)** through both stage 1 and stage 2. DShot is a
separate investigation (`docs/23_DShot_RPM_Investigation.md`, own tooling), queued **after**
this card is fully clean and gains are frozen — the one point where trying an alternate RPM
source can't confound anything being tested today. If there's time left at the very end:
switch the uSD config to `usd_dshot_investigation_config.txt` and follow `docs/23` §4. Not
required to close out this session.
