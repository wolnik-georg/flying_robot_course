# F2.2 / D4: control rate of the lab FBL firmware and why frsim's odd-tick call does not advance the state

18 Nov 2026 desk read, nothing run on hardware. Result: **lab = 500 Hz like frsim.**

Sources (read only):
- `~/Desktop/fbl_v2b_firmware/fw/src/modules/interface/stabilizer_types.h:383-389`: `RATE_MAIN_LOOP RATE_1000_HZ`, `ATTITUDE_RATE RATE_500_HZ`, `RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)`, i.e. a 2-tick period.
- `~/Desktop/fbl_v2b_firmware/fw/src/modules/src/stabilizer.c:303,350,376`: the stabilizer loop waits `F2T(RATE_MAIN_LOOP)` (1 kHz), calls `controller(&control, &setpoint, &sensorData, &state, stabilizerStep)` every step and increments `stabilizerStep`.
- `~/Desktop/fbl_v2b_firmware/snap/src/modules/src/controller/controller_omar_indi.c:352-358`: `controllerOmarIndi(...)` begins with `if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) { return; }`, before the FBL branch (`:414`, `controllerOmarFbl`). So the FBL branch runs on every second 1 kHz step, 500 Hz, with `dt = 1/ATTITUDE_RATE` = 2 ms for the stock PIDs (`:322` `attitudeControllerInit(1.0f / ATTITUDE_RATE)`, `:364` `dt`). The FBL law's own dt is the real elapsed time between FBL calls from `usecTimestamp()` (`controllerOmarFbl`, `:220-222` in the simulator copy), nominally 2 ms.
- The same gate is in the simulator's firmware copy: `~/Desktop/flying_robot_course_fbl/simulator/_build/crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c:359`.

Why frsim's odd-tick call advances nothing: frsim calls the firmware every 1 ms tick with `tick = i` (`simulator/frsim/episode.py:167-168`); on odd `i` the gate above returns immediately, so no law, PID or clock call happens. The host clock `usecTimestamp()` (`simulator/firmware/host/oot_host.c:337-341`, +2000 us per call) is therefore only advanced on even ticks, which gives dt = 2 ms on every executed call. This is the "even ticks, dt 2 ms" structure the model uses (`fbl_model.py`, F1 report section 4). The F1 inventory remark that the host source "does not explain it" is resolved by the gate.

Limits (stated, not decided):
- `~/Desktop/fbl_v2b_firmware/snap/.../controller_omar_indi.c` is the 25 Oct source (hand-over, tilt latch and rate limit still in the code); the simulator build is the 4 Nov authors-aligned version (116 differing lines in `diff`). Which of them the image on the drones was built from is not stated in the files read (`docs/fbl/LAB_PLAN.md:10` names the image `study_tree_v2_fbl_stockinner_nn4_full_candidate.bin` and its sha256, not its source tree). Both have the 500 Hz gate, so the rate result does not depend on it.
- On hardware the law dt is measured time, so it carries jitter; the model and frsim use exactly 2 ms.
