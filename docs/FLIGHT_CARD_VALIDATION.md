# Flight card — validate both controllers

> **One page. Take this to the lab.** **Stage 1: three geometric flights, then ONE INDI
> flight on the restored config only.** `check_flight.py` after every single one, stop on the
> first FAIL. **Stage 2** (the new fixes, one variable per flight) begins only once stage 1
> passes.
>
> Why this exists: 2026-09-09, six hover flights crashed under both controllers. Three
> separate root causes were found and fixed; **none has been re-flown.** Background:
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

## Tomorrow, part 1 — H0 partition

Never actually run this session. Config at start: the confirmed-clean locked point
(`kr=2400`/`kw=170`, `pos_gains` 64/5/48/7, `notch_en=0`). Root-cause the stage 2d notch crash
offline in parallel, whenever — separate, doesn't block this.

1. `crazyflies.yaml` → `indi_gains.ctrl_mode: 2` (attitude INDI only, position loop geometric).
   Rebuild `crazyflie`, relaunch server, confirm in cfclient.
2. Fly hover → `check_flight.py` → if clean, circle + figure8 at kt=0.05. Compare peak
   roll/pitch against the full-INDI baseline (27.5°/18.1° circle, 17.7°/14.1° figure8) — clean
   or meaningfully better here means the shake is in **position**-INDI.
3. Repeat with `ctrl_mode: 1` (position INDI only, attitude geometric). Clean here instead
   means the shake is in **attitude**-INDI.
4. Same abort rules as every flight this session: kill on visible growth, `check_flight.py`
   after every single one, don't fly the next config on a FAIL.

If neither partition is clean, that's itself a useful negative result — proceed to part 2 on
whichever controller (geometric, or INDI if a partition came back clean) is confirmed-good.

## Tomorrow, part 2 — first 2-drone flight

**uSD logging first, both drones, before anything else.** Radio can't carry two drones' worth
of data without dropping packets, and dropped packets in a residual-force dataset are silently
corrupt training data. uSD is the actual dataset; radio is only for live monitoring.

1. Confirm the uSD deck is physically installed on **both** drones.
2. Copy the config to **both** cards, named exactly `config.txt`:
   `cp flying_drone_stack/tools/usd_thesis_config.txt /media/<sd>/config.txt`
3. Power-cycle each drone, confirm on each: `python3 flying_drone_stack/tools/check_usd_deck.py`
4. Logs 39/40 vars at 500Hz per drone — position/velocity, attitude, gyro/accel, **`indi.a_res_*`**
   (the thesis signal), INDI internals, tracking error, motor effort. Already deliberately
   chosen and documented (`flying_drone_stack/tools/README_usd_thesis_logging.md`) — no config
   changes needed, just confirm both cards actually have it installed.

**Second drone setup in `crazyflies.yaml`** — still placeholder as of today:
`cf_second.enabled: false→true`, `cf_second.uri` (real radio address), `cf_second.initial_position`
(real physical takeoff spot).

**First flight — safety-first, rigid formation, geometric controller** (fully validated all
session; INDI multi-drone waits for part 1's result):

```bash
ros2 run crazyflie_examples formation_flight -- --formation vertical --trajectory hover --brushless --dry-run
```

Run with `--dry-run` first (prints the plan, doesn't fly), drop the flag once it looks right.
`vertical` is the downwash-coupled case — one drone in the other's wash, exactly what tomorrow's
data collection needs. `usd.logging` is toggled automatically for every drone in the formation.

Once hover is confirmed safe, progress toward the thesis's actual C.1 data-collection
requirement — a scenario that excites **lateral** relative motion, not just vertical (formation
library's A4/A7), not only this rigid-offset vertical check.

After each flight: pull both uSD cards, `python3 flying_drone_stack/tools/decode_usd_log.py <file>`
per drone, then `python3 experiments/analysis/run_analysis.py` for RMSE/a_res numbers and
`~/.pyenv/versions/flying_robots/bin/python experiments/analysis/plot_flight.py` for the
dashboard PNG (needs the pyenv, not system Python).

---

## DShot RPM — NOT today

`indi_gains.rpm_source` stays at **0 (deck)** through both stage 1 and stage 2. DShot is a
separate investigation (`docs/23_DShot_RPM_Investigation.md`, own tooling), queued **after**
this card is fully clean and gains are frozen — the one point where trying an alternate RPM
source can't confound anything being tested today. If there's time left at the very end:
switch the uSD config to `usd_dshot_investigation_config.txt` and follow `docs/23` §4. Not
required to close out this session.
