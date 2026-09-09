# Formation-library sim commands — 2026-09-08

Every command needed to run the full 16-scenario formation library in the CS2 simulator,
grouped by robot count. Server commands are given once per roster/controller combination;
restart the server (Ctrl+C, relaunch) between scenario runs — repeated runs on the same
unrestarted server were found this session to accumulate controller/planner state that shows
up as a growing attitude wiggle by the third run (see `docs/lab_sessions/2026-09-07.md`).

Status column: **run** = executed and confirmed working this session (2026-09-07/08),
**untested** = command is correct per the scenario library but not yet run.

## Server commands

```bash
# common preamble, every terminal
cd ~/Desktop/crazyswarm2
source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}
```

| Roster | Robots | Controller yaml | Launch |
|---|---|---|---|
| `crazyflies_sim1.yaml` | 1 (`cf231_active`) | `server_sim_geo.yaml` / `server_sim_indi.yaml` | `ros2 launch crazyflie launch.py backend:=sim crazyflies_yaml_file:=crazyflie/config/crazyflies_sim1.yaml server_yaml_file:=crazyflie/config/server_sim_geo.yaml gui:=True rviz:=True` |
| `crazyflies_sim.yaml` | 2 (`cf231_active`, `cf_second`) | `server_sim_geo.yaml` / `server_sim_indi.yaml` | `ros2 launch crazyflie launch.py backend:=sim crazyflies_yaml_file:=crazyflie/config/crazyflies_sim.yaml server_yaml_file:=crazyflie/config/server_sim_geo.yaml gui:=True rviz:=True` |
| `crazyflies_sim3.yaml` | 3 (`cf231_active`, `cf_second`, `cf5`) | `server_sim_geo.yaml` / `server_sim_indi.yaml` | `ros2 launch crazyflie launch.py backend:=sim crazyflies_yaml_file:=crazyflie/config/crazyflies_sim3.yaml server_yaml_file:=crazyflie/config/server_sim_geo.yaml gui:=True rviz:=True` |

Swap `server_sim_geo.yaml` for `server_sim_indi.yaml` for a full-INDI run (`ctrl_mode=3`).

Client preamble, every scenario run:
```bash
cd ~/Desktop/crazyswarm2
source /opt/ros/humble/setup.bash && source install/setup.bash
```

## 2-drone scenarios (`crazyflies_sim.yaml`)

| Scenario | Command | Status |
|---|---|---|
| A1 | `ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 --height 0.85 --auto-center --yes --ros-args -p use_sim_time:=true` | run (geometric, repeat) |
| A2 | `ros2 run crazyflie_examples run_formation --scenario A2 --dz 0.30 --speed 0.4 --auto-center --yes --ros-args -p use_sim_time:=true` | run (INDI) |
| A3 | `ros2 run crazyflie_examples run_formation --scenario A3 --dz 0.30 --speed 0.3 --passes 1 --auto-center --yes --ros-args -p use_sim_time:=true` | run (geometric, repeat) |
| A4 | `ros2 run crazyflie_examples run_formation --scenario A4 --dz 0.60 --offset 0.10 --auto-center --yes --ros-args -p use_sim_time:=true` | run (geometric) |
| A5 | `ros2 run crazyflie_examples run_formation --scenario A5 --dz 0.50 --laps 1 --auto-center --yes --ros-args -p use_sim_time:=true` | run (geometric, 28mm) |
| A6 | `ros2 run crazyflie_examples run_formation --scenario A6 --dz 0.10 --passes 1 --allow-extreme --auto-center --yes --ros-args -p use_sim_time:=true` | run (INDI) |
| A7 | `ros2 run crazyflie_examples run_formation --scenario A7 --allow-extreme --height 0.5 --auto-center --yes --ros-args -p use_sim_time:=true` | run (geometric) |
| A8 | `ros2 run crazyflie_examples run_formation --scenario A8 --dz 0.25 --auto-center --yes --ros-args -p use_sim_time:=true` | run (geometric) |
| C4 | `ros2 run crazyflie_examples run_formation --scenario C4 --allow-extreme --auto-center --yes --ros-args -p use_sim_time:=true` | run (geometric) |

## 3-drone scenarios (`crazyflies_sim3.yaml`)

| Scenario | Command | Status |
|---|---|---|
| B1 | `ros2 run crazyflie_examples run_formation --scenario B1 --dz1 0.20 --dz2 0.30 --auto-center --yes --ros-args -p use_sim_time:=true` | untested |
| B2 | `ros2 run crazyflie_examples run_formation --scenario B2 --dz1 0.20 --dz2 0.30 --r 0.10 --auto-center --yes --ros-args -p use_sim_time:=true` | untested |
| B3 | `ros2 run crazyflie_examples run_formation --scenario B3 --dz 0.22 --span 0.55 --auto-center --yes --ros-args -p use_sim_time:=true` | untested |
| C1 | `ros2 run crazyflie_examples run_formation --scenario C1 --sep 0.30 --auto-center --yes --ros-args -p use_sim_time:=true` | untested |
| C2 | `ros2 run crazyflie_examples run_formation --scenario C2 --gap 0.50 --speed 0.4 --auto-center --yes --ros-args -p use_sim_time:=true` | untested |
| C3 | `ros2 run crazyflie_examples run_formation --scenario C3 --side 0.50 --auto-center --yes --ros-args -p use_sim_time:=true` | untested |

## 1-drone scenario (`crazyflies_sim1.yaml`)

| Scenario | Command | Status |
|---|---|---|
| C5 | `ros2 run crazyflie_examples run_formation --scenario C5 --z 0.15 --speed 0.25 --auto-center --yes --ros-args -p use_sim_time:=true` | untested |

## Notes

- All the vertical-stack scenarios above (A1/A2/A5/A6/A7/A8, C4, and by construction B1/B2)
  should print `landing XY overlap detected -- spreading robots sideways first` on landing —
  this is the fix from 2026-09-07/08 (`run_formation.py`), not a fault.
- A3/A4 never share XY, so they should **not** print that line.
- `--allow-extreme` is required whenever the scenario is tagged `extreme` (A6, C4 with
  `dz_end < 0.15`) — the runner refuses to fly otherwise.
- Restart the server between every run listed above; do not chain scenarios on one
  unrestarted server (see the wiggle note at the top).
