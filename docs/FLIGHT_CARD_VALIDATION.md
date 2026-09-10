# Flight card — validate both controllers

> **One page. Take this to the lab.** Six flights, `check_flight.py` after every one, stop
> on the first FAIL. Nothing past this card runs until all six pass.
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

## 2 · INDI — set `ctrl_mode: 3`

Same edit, rebuild `crazyflie`, **Ctrl-C and relaunch the server**, confirm `ctrl_mode`=3 in cfclient. Expect `pos_gains 64/5` here — correct, that's INDI's locked block.

```bash
ros2 run crazyflie_examples simple_flight -- --trajectory hover   --duration 15
ros2 run crazyflie_examples simple_flight -- --trajectory circle  --kt 0.1
ros2 run crazyflie_examples simple_flight -- --trajectory figure8 --kt 0.008
```

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

**Only after all six PASS** → gain retuning, then multi-drone.

---

## Tick sheet

| # | Mode | Trajectory | PASS / FAIL | roll std | peak | Notes |
|---|---|---|---|---|---|---|
| 1 | 0 geometric | hover | | | | |
| 2 | 0 geometric | circle | | | | |
| 3 | 0 geometric | figure8 | | | | |
| 4 | 3 full INDI | hover | | | | |
| 5 | 3 full INDI | circle | | | | |
| 6 | 3 full INDI | figure8 | | | | |

Push the logs when done — `git add Controls/logs && git commit && git push`.
