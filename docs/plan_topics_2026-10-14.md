# Plan topics collected after the 2026-10-14 meeting (decisions so far; work not started)

**Detailed plan for topics 1 and 2: `docs/plan_omar_tree_ns2_2026-10-14.md` (Phase 0 desk reading done).**

Three topics. Topics 1 and 2 are decided in outline; topic 3 is the FBL follow-up. Status: **collecting; no work started** (author's instruction 2026-10-14). Replaces the "reduced shakedown" in `lab_sessions/next_session_plan.md` (obsolete, see the banner there).

## Topic 1 — One controller tree: Omar's C controller (controller 9) with NS2 on top
**Decided (author, 2026-10-14):**
- Study strategies: **S0 Omar geometric** (`indi=0`), **S1 Omar full INDI** (`indi=3`, Pure INDI), **S2 Omar geometric + NS2**, **S3 Omar full INDI + NS2** (hybrid, in the comparative study). FBL moves to **S4** (late tier or documented limitation).
- Both drones (cf5 and cf_second) run Omar's controller. Omar's geometric and INDI code is **not modified**; only NS2 is added. Our controller 6 (`lib.rs`) stays untouched, frozen (`docs/77`) and supplementary.
- Shared parameters: position gains identical in all four (whatever the code has; to be read when porting); the z integral (`Kpos_Iz` 1.5) in all four; everything else as in Omar's code (mass 0.0427, his k_t, 30 Hz filters, deck RPM).
- NS2 enters as a plain sum with the measured residual, gain 1, same sign convention as Omar's subtracted residual (check sign by replay/SIL before flight). Same network architecture as v1 (19,297 weights, layout from CS2).
- NS2 is ported to **C inside the Omar tree** (no Rust dependency). Validation: same inputs through our working Rust NS2 and the C port, outputs equal to ~1e-6; then replay/SIL with Omar's controller; then hover, A8.
- Logging stays exactly as now (same 96-variable chain), now also from controller 9, including `rnn_pred_*`.
- All baselines are re-flown on the new tree; thesis draft follows after (Ch1 scope, Ch4 table and fairness text, D-01/D-02/D-06 reframed).

**To establish when working (desk, from the code):** how peer positions are reached from `controller_omar_indi.c`; weight upload and verification path; timing budget of a 100 Hz forward pass in Omar's loop; which Omar parameters are shared between `indi=0` and `indi=3`; whether Omar geometric was ever flown as a baseline.

## Topic 2 — NS2 retrain with diverse data (on the new tree)
**Decided / agreed:**
- Retrain is needed anyway (new tree, new baseline, new residual definition = Omar's: mass 0.0427, his k_t, 30 Hz filters, deck RPM; label `a_imu_f − a_rpm_f`). Same architecture, so the C port is written once and the new weights upload.
- Training data must cover the study's combinations of **speed and vertical separation dz** (and lateral offset), not only dz 0.5 / speed 0.4. Known thin spots of v1 (`docs/ns2_retrain_options.md`): dz 0.75 (0.5 % of the bank), no A8.
- Data flights are flown **on the new tree**, so the S0 baseline re-flights double as training data. Order: tree → baseline flights and diverse data → retrain → S2/S3.
- The network only acts inside the gate (|dx| < 0.2 m, |dy| < 0.2 m), so data must be biased toward stacked positions to be efficient.
- Two sources: (a) structured grid from the formation library (dz × speed × offset), (b) random-waypoint flights.

**The random-waypoint script (validated 2026-10-14):** `cvmrs_ros/cvmrs_ros/random_flight.py` in https://github.com/IMRCLab/dataset-cv-rel-pos (IMRCLab; Moldagalieva and Hönig, arXiv 2303.03898; built on Crazyswarm2 `crazyflie_py`). Source read in full (204 lines; copy in the session scratchpad only).
- Per drone, one thread: goals `np.random.uniform(bbox_min, bbox_max)` in a **box** (default x,y ±0.6 m, z 0.3–1.2 m; per-drone override), speed `uniform(0.1, 0.5)` m/s per leg, leg time `max(dist/speed, 1.5)` s, `goTo` sent 3× per leg, `total_time = 30` s, then back to the start and land.
- Collision avoidance = firmware module `colAv` (buffered Voronoi cell, `collision_avoidance.c`, compiled into our firmware: `Kbuild obj-y += collision_avoidance.o`, hooked in `stabilizer.c:344`, parameters `colAv.enable`, `ellipsoidX/Y/Z`, `bboxMin/Max`); needs the peer positions (already used by NS2). Script sets ellipsoid 0.15 m, enables after takeoff.
- Works with our CS2 API (`Crazyswarm()`, `allcfs.crazyfliesById`, `setParam`, `goTo`, `initialPosition` all exist in `crazyflie_py`).
- **Changes needed before use:** remove the camera / AI-deck parts (subprocess image collection, `cvmrs.*` params, calibration, hard-coded `/home/whoenig/...` paths, `hlCommander.yawrate`, `stabilizer.controller 4`); two drones only; our own uSD logging start sequence from `run_formation.py` (note its comment on `usd.logging`) and the `usec.reset` rule (ground only); our geofence/arena limits instead of the fixed box; yaml selects the controller.
- **Separation limit:** in the BVC the centre distance is at least twice the ellipsoid radius per axis (stretched metric). With `ellipsoidZ` 0.15 the vertical separation is ≥ 0.30 m, so **dz 0.25 is not reached**; use `ellipsoidZ` ≈ 0.12 (≥ 0.24 m). Firmware default is 0.3/0.3/0.9 (downwash-aware, far too large).
- **Yield:** two independent goals in a 1.2 × 1.2 m box put the pair inside the NS2 gate only ~9 % of the time (before the vertical condition); bias one drone's goals toward the other's xy, or restrict the box.
- Positive: z range 0.4–1.1 gives a continuous spread of dz for free; speed is randomised.

**Open:** which (dz, speed, offset) grid is the core; how many random flights; label definition details (decided: Omar's); retrain pipeline script (`run_c2_lab_retrain.py` is the old pipeline, to be adapted to Omar's labels).

## Topic 3 — FBL controller follow-up (Hsieh et al. 2026; `docs/73`)
**Decided 2026-10-14 (author): ask the authors only for the training script (+ why: retrain on our platform). The meeting doc question is reduced to that; licence and singularity handling are not asked, the rest is figured out by us.**
**Position (to confirm with the supervisor):**
- What we have: inference side only (114-parameter downwash residual with analytic derivatives, FBL control mapping, acceleration estimator, a sample *simulation* checkpoint). Verified against the paper (`docs/73`).
- **Needed for our platform: the training script** (KNODE loss, ODE solver, data format), because the residual must be retrained on our vehicle and data; the bundle's constants are hard-coded for a 30 g Crazyflie 2.x and the sample net fits one specific simulated plant.
- **Not needed if we retrain:** the hardware checkpoint, hardware gains and control rate (we tune on our platform; the rate gain that is unstable at 100 Hz is fine at our 500–1000 Hz).
- **Worth one line in the same message:** (a) licence / permission to use and adapt the code in a thesis (the bundle has no licence header); (b) how the near-aligned singularity was handled (residual jerk blows up when the horizontal offset goes to zero, exactly our A1/A2/A8 stack passes). We could regularise it ourselves, so (b) is optional.
- Everything else in the draft e-mail (C_d 0.55 vs 0.7, mass, integral first-step `dt`, `_vee_skew` sign, yaw handling) is ours to handle and not needed from the authors when the sample checkpoint is not reused.
- Fallback if no script arrives: the bundle contains the differentiable nominal model; write our own training (integrated-state loss, or simpler supervised regression on our measured residual labels). Extra weeks, not equivalent to their KNODE training; decide with the supervisor.
- Fit with the new tree: FBL is its own cascade (position PID → acceleration, jerk by differentiation, rate command, torque), not Omar's geometric law; it would be a separate controller (S4), 4–6 weeks of port work after the authors' answer (estimate `docs/73` §9). Study entry: late tier or documented limitation.
