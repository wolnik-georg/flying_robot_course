# Next lab session — plan (prepared 2026-10-13, date open) — PLANNED

## 1. Goals (decided 2026-10-13)
1. Check the firmware on **both** drones.
2. Restore the study SD config on the cf5 card.
3. Fly **ours with Omar's values** and compare with ours frozen (`docs/77`) and Omar C.
4. Fly **Tal & Karaman with Omar's values**.
Not in this session (dropped): `--study` chain run, never-flown settings, NS2 thin spots, flights per hour.

## 2. What the closed-loop simulation says (`experiments/analysis/indi_variants_sil_check.py`, one drone, airborne start, noise on, 44 ms motor lag, real vehicle as plant)
| variant | hover | A8 bottom path | circle (A2) |
|---|---|---|---|
| Omar C (z integral 0 / 1.5) | stable: xy 0.4 cm, z −0.3 / 0.0 cm | stable: 0.4 cm, −0.3 / 0.0 | stable |
| Tal & Karaman native / with Omar's values | stable: 0.2 / 0.5 cm, tilt ≤ 3° | stable | stable |
| **ours with Omar's values, ours' own filters (206 Hz)** (`ours_omar_values_keepfilters`) | **stable: 0.5 cm, z −1.5 cm, tilt ≤ 2°** | **stable: 0.4 cm, −1.5 cm** | **stable** |
| same with z integral 1.5 (`…_keepfilters_iz`) | stable: 0.5 cm, z 0.0 cm | stable | stable |
| ours with Omar's values **and his 30 Hz filters** (`ours_omar_values_30hz`) | diverges (tilt > 30°) | diverges | diverges |
| ours frozen (as flown) | falls after about 1–5 s | falls | falls |
- Reading: with Omar's mass, position gains, residual sign and attitude gains, **but ours' own filters**, ours behaves like Omar C in closed loop (within about 1 cm, a z offset of −1.5 cm without the integral, 0.0 with it). Lowering ours' filter cutoff to 30, 60 or 100 Hz destabilises it (the reference is low-passed too, which adds lag in the feedback path; Omar's reference is unfiltered). Hence the first flight uses ours' filters; the 30 Hz variant is expected to oscillate and goes last, if at all.
- Limits: ours frozen also falls in the simulation although it flies on the drone (27 s), so the simulation is harsher than the hardware for ours; only the ordering is used. No downwash, no estimator delay, no ground contact (takeoff from the ground is not covered: the simulation starts airborne).

## 3. Setup (once)
Pull both repos on the lab PC (`git pull` in `flying_robot_course` and `crazyswarm2`), CS2 stopped for the checks.
**Firmware check, both drones** (the console must print the line for 10, 11 and 12; ground, motors off):
```bash
python3 flying_drone_stack/tools/check_active_controller.py radio://0/80/2M/E7E7E7BB02 6 10 11 12 6   # cf5
python3 flying_drone_stack/tools/check_active_controller.py radio://0/80/2M/E7E7E7E7E9 6 10 11 12 6   # cf_second
```
Expected: "Using OutOfTree5 (10)", "Using OutOfTree6 (11)", "Using OutOfTree7 (12)", "Using OutOfTree (6)". If a drone prints nothing for 11 / 12 it carries the other image: flash `flying_drone_stack/firmware_app/build_artifacts/cf21bl_study_usd96_rpmf_rnn_iz_sentinel_ON_tk.bin` (sha256 `cb38f3f4981d8fa6b9c85c9e26d5645b5535040328d4ef99926a1c267037f8ca`, manifest) with `cfloader flash <file> stm32-fw -w <uri>`, power-cycle, check again.
**SD config on the cf5 card** (the card currently holds the Tal & Karaman debug config; the study config is restored, the old one is kept as a dated backup on the card):
```bash
python3 flying_drone_stack/tools/restore_study_sd_config.py /media/georg/<cf5 card>
```
The cf_second card already has the study config (check with the same command: it says "already in place").

## 4. Flights
Abort rule (protocol §4): |roll| or |pitch| > 25° for 0.5 s, or z < 0.25 m; stop a block after 2 crashes; after a crash toggle `stabilizer.estimator` 1→2. Always from a clean yaml and relaunch CS2 after every yaml change:
```bash
cd ~/Desktop/crazyswarm2 && git pull && git checkout crazyflie/config/crazyflies.yaml && git apply crazyflie/config/study/<patch>
ros2 launch crazyflie launch.py
```
**Block 1 — ours with Omar's values** (patch `ours_omar_values_keepfilters.patch`: `ctrl_mode 3`, mass 0.0427, k_t 4.29e-10, position gains 7 / 4, `res_sign −1`, attitude gains 421 / 69 (z 273 / 68), `j_scale 0.693`, filters as flown, `ki_z 0`):
1. Solo hover, cf_second off (`enabled: false` in its block): `ros2 run crazyflie_examples simple_flight -- --trajectory hover --duration 15` (no `--pin-controller`). `res_sign −1` diverged in hover on 2026-09-09 with other settings: look at the first seconds.
2. If the hover is clean: both drones, A8 as usual: `ros2 run crazyflie_examples run_formation --scenario A8 --dz 0.5 --height 0.5 --passes 4 --auto-center --yes` (two flights).
3. Then `ours_omar_values_keepfilters_iz.patch` (z integral 1.5, `ki_z_limit 2.0`, Omar C + Iz 1.5 is the study variant): hover, A8.
4. Optional, last: `ours_omar_values_30hz.patch` (Omar's 30 Hz filters too, predicted unstable): solo hover only.
Compare with: ours frozen A8 dip −9.4 cm, steady offset +1.5 / +1.7 cm; Omar C + Iz dip −9.1 cm, offset +0.3 cm; geometric −5.3 cm.
**Block 2 — Tal & Karaman with Omar's values** (patch `TK_omar_values_handover.patch`: takeoff on the geometric ramp, switch to controller 11 at altitude; position gains 7 / 4, no acceleration feedback, torque-equivalent attitude gains 292 / 48 (z 247 / 62), `jrz 0`):
1. Solo hover, cf_second off, same `simple_flight` command; before takeoff the firmware check above must have printed "Using OutOfTree6 (11)"; the radio log meta must read takeoff controller 6 and trajectory controller 11.
2. If it hovers: A8 as above. If it flips or oscillates: stop; the patch without the handover (`TK_omar_values.patch`, takeoff directly on controller 11) flipped at takeoff in the first attempts, so it is not the one to try.

## 5. After the flights
```bash
python3 flying_drone_stack/tools/copy_usd_log.py /media/georg/<cf_second card> cf_second --file thesisNN --tag <name>   # cf_second first, one call per file
python3 flying_drone_stack/tools/copy_usd_log.py /media/georg/<cf5 card> cf5 --file thesisNN --tag <name>
python3 flying_drone_stack/tools/check_usd_log_quality.py experiments/logs/usd_raw/*<today>*
```
Crossing-averaged plot of the new groups: `experiments/analysis/a8_crossing_average.py` (add the new flight names to `GROUPS`).
