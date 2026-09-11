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

## Tick sheet

| # | Mode | Trajectory | PASS / FAIL | roll std | peak | Notes |
|---|---|---|---|---|---|---|
| 1 | stage 1 | geo · hover | | | | |
| 2 | stage 1 | geo · circle | | | | |
| 3 | stage 1 | geo · figure8 | | | | |
| 4 | stage 1 | INDI · hover, restored only | | | | |
| 5 | stage 2a | res_fc 80 + res_clamp 10 | | | | |
| 6 | stage 2b | prewarp/dt/fc_bw=206 (no-op) | | | | |
| – | stage 2c | fc_bw < 206 — tune down | | | | |
| – | stage 2d | notch_en=1, notch_f0=6.9 | | | | |
| 7 | stage 2e | kr=987 kw=109 (ω_n 5 Hz) | | | | |
| 8 | stage 2e | kr=632 kw=87 (ω_n 4 Hz) | | | | |

Push the logs when done — `git add Controls/logs && git commit && git push`.

---

## DShot RPM — NOT today

`indi_gains.rpm_source` stays at **0 (deck)** through both stage 1 and stage 2. DShot is a
separate investigation (`docs/23_DShot_RPM_Investigation.md`, own tooling), queued **after**
this card is fully clean and gains are frozen — the one point where trying an alternate RPM
source can't confound anything being tested today. If there's time left at the very end:
switch the uSD config to `usd_dshot_investigation_config.txt` and follow `docs/23` §4. Not
required to close out this session.
