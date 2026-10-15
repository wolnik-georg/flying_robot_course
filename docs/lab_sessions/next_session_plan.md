# Next lab session — Omar C tree with NS2 (Phase 7 bench + Phase 8 validation) — PLANNED (date open)

Plan: `docs/plan_omar_tree_ns2_2026-10-14.md`. Spec: `docs/ns2_c_module_spec.md`. Replaces the earlier reduced-shakedown plan (controller 6, obsolete).

## 0. Goal and what must NOT be skipped
Validate the unified controller tree on the drones: **S0 Omar geometric, S1 Omar full INDI, S2 Omar geometric + NS2, S3 Omar full INDI + NS2**, both drones on Omar's controller (controller 9). Nothing else is changed: controller 6 / ours / TK are untouched.
Desk gates passed before this session (details in the plan doc §Phases): C module = Rust flight code bit-identical (Phase 4), NS2-off outputs byte-identical to the old Omar C on four logged flights (Phase 3), closed-loop SIL direction/sign (Phase 5), image built from the patch list (Phase 6).

## 1. Image and manifest (Phase 6 result)
- Image: `flying_drone_stack/firmware_app/build_artifacts/cf21bl_study_usd96_rpmf_rnn_iz_sentinel_ON_tk_ns2c.bin`, sha256 `dfac761a16d115c9a20ef29e320f7d9ff5a76300b85e475548b285fac24fc2d5` (532 960 B; flash 52 %, RAM 75 %). Candidate manifest: `docs/frozen/omar_ns2c_tree/study_manifest_ns2c_candidate.json` — **not active**; copy its `cf5` / `cf_second` blocks into `crazyswarm2/crazyflie/config/study_manifest.json` only at the flash step (below).
- The previous study image (`…_ON_tk.bin`, `cb38f3f4…`) stays as the fallback; flashing it returns to the old tree.

## 2. Setup (no motors)
```bash
cd ~/Desktop/flying_robot_course && git pull && cd ~/Desktop/crazyswarm2 && git pull     # lab PC, CS2 stopped
# flash BOTH drones (cf5 E7E7E7BB02, cf_second E7E7E7E7E9), power-cycle each afterwards
cfloader flash ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build_artifacts/cf21bl_study_usd96_rpmf_rnn_iz_sentinel_ON_tk_ns2c.bin stm32-fw -w radio://0/80/2M/E7E7E7BB02
cfloader flash ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build_artifacts/cf21bl_study_usd96_rpmf_rnn_iz_sentinel_ON_tk_ns2c.bin stm32-fw -w radio://0/80/2M/E7E7E7E7E9
# Phase 7 bench check (static, sends NO setpoint), once per drone
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/ns2c_bench_check.py radio://0/80/2M/E7E7E7BB02
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/ns2c_bench_check.py radio://0/80/2M/E7E7E7E7E9
# SD config on the cf5 card (it may still hold the TK debug config) and the cf_second card
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/restore_study_sd_config.py /media/georg/<card>
```
Bench expectations: console `Using OutOfTree4 (9)`, `rnn.ready 1`, `rnn.div 10`, `rnn.en 0`, Omar mass 0.0427, `Kpos_P 7`, `Kpos_D 4`, all log variables present. **Any FAIL: stop and report; do not fly.** Then copy the candidate manifest blocks into `study_manifest.json`, commit and push both repos.

## 3. Flights (fresh batteries; abort rule protocol §4: |roll| or |pitch| > 25° for 0.5 s, or z < 0.25 m; stop a block after 2 crashes; after a crash toggle `stabilizer.estimator` 1→2; relaunch CS2 after every yaml change)
Yaml per strategy (clean yaml first):
```bash
cd ~/Desktop/crazyswarm2 && git checkout crazyflie/config/crazyflies.yaml && git apply crazyflie/config/study/tree_S0.patch   # S0 | S1 | S2 | S3
ros2 launch crazyflie launch.py
```
Both drones are pinned to controller 9 by the patch, so takeoff and landing also run Omar's controller (no `--pin-controller` needed).
| # | Strategy | Flight | Command | Look at |
|---|---|---|---|---|
| 1 | S0 | solo hover (set `enabled: false` for cf_second AND remove it from the volume) | `ros2 run crazyflie_examples simple_flight -- --trajectory hover --duration 15` | clean takeoff and landing on Omar geometric, z offset ≈ 0 (Iz 1.5) |
| 2 | S0 | A8, 2 drones | `ros2 run crazyflie_examples run_formation --scenario A8 --dz 0.5 --height 0.5 --passes 4 --auto-center --yes --study --strategy S0 --session tree_validation` | crossing dip of the baseline on the new tree |
| 3 | S2 | A1 stack hold (NS2 active: the neighbour is inside the gate) | `ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.5 --hold 15.0 --height 0.5 --auto-center --yes --study --strategy S2 --session tree_validation` | `rnn.pred_*` non-zero, no oscillation |
| 4 | S2 | A8, 2 drones | same as #2 with `--strategy S2` | dip vs #2 |
| 5 | S1 | solo hover (cf_second off) | as #1 | **takeoff from the ground in INDI mode (indi 3) is new**: liftoff, no flip |
| 6 | S1 | A8 | as #2 with `--strategy S1` | compare with Omar C + Iz history (dip ≈ −9 cm, offset +0.3 cm) |
| 7 | S3 | A1 hold, then A8 | as #3 / #2 with `--strategy S3` | first INDI + NS2 flight; watch for double compensation (oscillation) |
Do not skip the order: S0 first (it is the same tree without NS2 and without INDI), S1 only after S0 is clean.

## 4. After each flight / at the end
```bash
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/ns2c_bench_check.py radio://0/80/2M/E7E7E7BB02 --post-flight   # cf5, CS2 stopped, drone still powered: rnn.us_max (single NS2 evaluation, µs)
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/copy_usd_log.py /media/georg/<cf_second card> cf_second --file thesisNN --tag tree_<S>   # cf_second first
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/copy_usd_log.py /media/georg/<cf5 card> cf5 --file thesisNN --tag tree_<S>
python3 ~/Desktop/flying_robot_course/flying_drone_stack/tools/check_usd_log_quality.py experiments/logs/usd_raw/*<today>*
```
Archive both SD cards to `~/Desktop/sd_backups/<date>_lab/` before reusing them.

## 5. Known risks (from the desk phases)
- **S3 (INDI + NS2) as built adds the two terms (plain sum): in this INDI formulation the measured residual does not shrink when the prediction raises the thrust, so the prediction acts as an offset on the acceleration reference (over-compensation during a crossing, removed in steady state only by the vertical integral). DECISION D-12 (`thesis_revision/round2/round2_open_items.md`): keep (what is flashed), or change the hybrid so the INDI term only handles the remaining mismatch (`a_d − a_res,f − (â − â_f)`, one filter and a few lines in the NS2 hook, rebuild + re-flash). Decide with the supervisor BEFORE the S3 flights; S0, S1, S2 are unaffected.**
- **INDI takeoff from the ground** (S1, S3): never flown with Omar's controller (all earlier Omar INDI flights took off on controller 6 and switched at altitude). If #5 flips or oscillates at liftoff: stop, do not repeat; report. A framework change (takeoff with `indi 0`, switch to 3 at altitude) is the fallback.
- **NS2 timing** is measured only now (`rnn.us_max`); the desk estimate is 0.3–0.45 ms per evaluation inside Omar's 2 ms slot; the limit you define.
- **Flight framework** (`flight.py`, `run_formation.py`) still has `_RAMP_CONTROLLER = 6` as the shared default; the per-robot pin overrides it for both drones (this is how earlier "cf5@9" shakedowns worked). Verify in the first flight's console that both drones report `controller=9` in the takeoff phase.
- **Simulation limits:** the SIL shows no dip for Omar INDI (perfect RPM information) and cannot say whether S3 helps; hardware decides.
- **Old data** (controller 6 S0/S2, `L0–L2`) is history, not study data.
