# FBL gain tuning in simulation (2026-10-20) — desk step 1 of 5

![gain tuning](../../experiments/fbl/gain_tuning.png)

- **Plant:** the real firmware controller (C-in-the-loop) with command delay 4 ms, motor lag 44 ms (our identified value, `docs/09`), sensor noise; A8 swap.
- **Authors' gains are unstable on our vehicle:** position loop at full strength diverges (tilt > 60°, latch trips) once the motor lag is ≥ 15 ms (with 4 ms delay). Delay alone (up to 6 ms) or noise alone is harmless. The authors' vehicle and loop differ; their hardware gains will tell.
- **Working region:** position gains (kp, kv, ki) at ≤ 0.25× the authors' values; the rate gain kr matters little (0.12–0.5× all fine at 0.25× position gains). Chosen: **position 0.25×, kr 0.25×** (the best cell, 0.35×, sits next to the unstable region, so no margin).
- **Result in this plant (A8, dip, 0.1 / 0.3 / 0.5 m/s):** Omar geometric −23.8 / −11.6 / −7.7 cm, FBL without downwash model −14.6 / −8.1 / −5.5, FBL with it −2.8 / −1.5 / −2.1. Stable for motor lag 20–60 ms. **Simulation only**; the plant has no thrust-map error, ground effect or battery sag.
- **Weak point: thrust-model error.** FBL's z integral is small (ki_z 3). If only 90 % of the commanded thrust arrives, z RMS goes 1.5 → 4.6 cm (ki_z 3) or 3.1 cm (ki_z 8). Proposal: ki_z 8.
- **In the firmware (patch `fbl_gain_presets.patch`, applied after `fbl_integration.patch`):** parameter `ctrlFbl.preset`. **0 = the authors' gains exactly as in their code (kept for reference and comparison)**, **1 = ours (default)**: kp 14/11.25, kv 9.5/5.4, ki 12/8, kr 59.5/35. Individual `ctrlFbl.kp_xy` ... still override after the preset. Checked: defaults read back as preset 1; preset 0 reproduces the authors' unstable result in the plant (z RMS 15.2 cm, tripped), preset 1 gives z RMS 0.9 cm, dip −0.2 cm (A8, 0.3 m/s); G4 byte-identical at `en = 0` (4 of 4); ARM candidate `study_tree_v2_fbl_candidate_presets.bin` (sha256 `a354eee3…`, flash 56 %, local only) builds clean. Not flashed.
- **Platform note:** our vehicle is a brushless Crazyflie with ~44 ms motor lag and 42.7 g, close to but not the authors' platform; both gain sets stay available and the authors' hardware gains will be added as a third preset when they arrive.

![handover test](../../experiments/fbl/handover_test.png)

- **Handover (takeoff from 0.02 m, hold at 0.5 m, landing), same plant, no neighbour:** FBL took over at 0.15 m and handed back below 0.10 m without a jump; tilt stayed below 1°, the latch never tripped. Both controllers show a height swing after takeoff (FBL −10/+5 cm, Omar −19 cm; most likely integrator wind-up from the takeoff ramp, not investigated yet). Not tested yet: the latch on a forced tilt fault, non-finite inputs.
- **Next:** (handover done, see above) at 0.15/0.10 m and tilt latch in the plant; FBL on A1 and A4; explore further what suits our platform (e.g. a lag-aware tuning, ki_z, estimator gains).

Reproduce: `experiments/fbl/stage3_port/gain_tuning.py --stage scale|grid|robust|eff`, plot `gain_tuning_plot.py`, outputs `experiments/fbl/out/stage3/gain_tuning_*.json`, plant `c_in_the_loop_real.py`.
