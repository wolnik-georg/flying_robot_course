# Prompt for an independent reviewer (Opus) — validate the "S3 = INDI + NS2: Version A vs Version B" analysis before it goes to the supervisor

> **Closed 2026-10-18:** Version B was removed and S3 decided (`docs/s3_indi_ns2_decision_2026-10-18.md`); kept as history.


You are an experienced control/estimation engineer reviewing a colleague's analysis. Be adversarial: the goal is to find out where we are **wrong**, not to confirm. Separate **mechanism** (is the reasoning right?) from **magnitude** (does it explain the numbers we measured?). Write your result to `docs/validation_s3_versions_2026-10-18.md` (new file). Do NOT modify firmware, the controller, yaml, the meeting document or any existing doc; do NOT push; no co-author trailers. Repository: `~/Desktop/flying_robot_course` (branch `main`); firmware tree `~/Desktop/crazyflie-firmware` (patched, local); python for analysis: `~/.pyenv/versions/flying_robots/bin/python` (host bindings/SIL need the system `python3` 3.10 and `CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build`).

## 1. System in one page
Crazyflie 2.1 Brushless (41-43 g), mocap at 100 Hz, onboard "Omar C" controller (`controller_omar_indi.c`, controller slot 9) as ONE controller with switches: `indi` bit 0 = INDI in the **position loop**, bit 1 = INDI on the moments, plus the NS2 hook. Strategies: S0 geometric (indi 0), S1 INDI (indi 3), S2 geometric + NS2 (indi 0, `rnn.en 1`), S3 INDI + NS2 (indi 3, `rnn.en 1`). Shared gains Kpos_P 7, Kpos_D 4, Kpos_Iz 1.5 (z integral), mass 0.0427. Controller step every 2 ms (500 Hz) inside a 1 kHz stabilizer loop, 30 Hz 2nd-order Butterworth filters on the INDI accelerations; NS2 (Neural-Swarm2 Deep Sets, 11 461 weights) is evaluated every 10th tick (100 Hz), zero-order hold in between, 526 µs worst case, z-component only, clamp ±8 m/s².
Problem physics: the lower of two stacked/crossing drones feels the upper one's downwash, a vertical force. `a_res_z = a_imu_f - a_rpm_f` (filtered measured acceleration minus acceleration implied by the motor RPM) is our "residual" label and is typically −1.0 … −1.4 m/s² inside the NS2 gate, peaks −3 … −5.

Code that matters (read the real file, lines ~237-300; do not trust this excerpt):
```c
a_d = acc_d + Kpos_D*vel_err + Kpos_P*pos_err + i_term;        // position PD (+ z integral)
if (ns2_active()) a_d = a_d - a_nn;                              // NS2 hold, z only; a_nn ~ residual (negative = push down)
if ((indi & 1) && rpm_deck_available) {                          // INDI position term
   a_rpm = (f_rpm/m) * R*e3 - g*e3;   a_imu = 9.81*acc;          // filtered by filter_acc_rpm / filter_acc_imu (30 Hz)
   a_indi = a_rpm_filtered - a_imu_filtered;                     // ~ -(unmodelled acceleration)
   // NEW 2026-10-17, only if ns2_mode == 1 (Version B):  a_indi += filter30(a_nn)   // default ns2_mode = 0
}
thrustSi = m * dot(a_d + a_indi, R*e3);   F_d = a_d + a_indi;    // desired attitude from F_d
```
Files: live firmware `src/modules/src/controller/controller_omar_indi.c` and `src/modules/src/ns2_residual.c` in the firmware tree; the same in the repo as patches `flying_drone_stack/firmware_patches/controller_omar_indi.{c,h}.patch`, `ns2_residual_module.patch`, snapshot `flying_drone_stack/firmware_app/host/controller_omar_indi.c.snapshot`; NS2 spec `docs/ns2_c_module_spec.md` (§6 sign convention).

## 2. The two versions under review
- **Version A (flown, current S3):** plain sum of the two switches, exactly the code above with `ns2_mode = 0`.
- **Version B (implemented 2026-10-17, tested in SIL only, never flown):** `ns2_mode = 1`: `a_indi_B = (a_rpm_f - a_imu_f) + filter30(a_nn)` while `a_d -= a_nn` stays. Intended meaning: "INDI works on what NS2 left over"; algebraically `a_d - (a_nn - a_nn_f) + a_indi`, i.e. NS2 contributes only its part above the 30 Hz filter band. Only active when INDI position term and NS2 are both on (S2 unchanged, tested identical).
The supervisor asked for "simple on/off switches, no complex reinvention".

## 3. Our claims (each needs a verdict: CORRECT / PARTLY / WRONG, with evidence)
C1 (mechanism). The INDI term measures the unmodelled **force/acceleration** (IMU minus RPM-implied), not the remaining tracking error. Raising the thrust raises `a_rpm` and `a_imu` equally, so `a_rpm_f - a_imu_f` stays at about minus the push. Consequence: with INDI active the vehicle acceleration is driven to `a_d` (+ the filter-lag error); an additive NS2 term `-a_nn` inside `a_d` is therefore not a feed-forward disturbance compensation but an extra upward acceleration target of size ≈ |a_nn| ("double counting": thrust compensation ≈ 2×push, net acceleration bias ≈ +1× push).
C2 (data). In the flights of 2026-10-15 the INDI-measured push inside the NS2 gate is the same in S3 as in S1: A1 −2.86 vs −2.89 m/s², A8 −1.22/−1.31 vs −1.05/−1.13/−1.17, while NS2 predicts −1.0 … −2.3 (S3). (Gate: top vehicle above, |dx|,|dy| < 0.2 m. Source `experiments/results/tree_validation_2026-10-15/tree_S{1,3}_A{1,8}_*.csv`, columns `bottom_omar_a_rpm_fz`, `bottom_omar_a_imu_fz`, `bottom_rnn_pred_z`; raw 500 Hz uSD logs in `experiments/logs/usd_raw/cf5_tree_S*_*.bin`, decoder `flying_drone_stack/tools/decode_usd_log.py`.)
C3 (hardware result, n = 2-3, same session). A8 crossing dip / A1 z error: S0 −15.7 cm / −19.8; S1 −10.4 / −0.6; S2 −5.5 / −13.7; S3 −1.9 / +2.6. S3 shows a small upward overshoot (A8 up to +5.7 cm, A1 +2.6 cm vs −0.6 for S1).
C4 (Version B algebra). In steady state B's NS2 contribution vanishes and INDI alone provides the compensation (each force counted once); at a fast onset NS2's high-passed part cancels INDI's filter-lag error; with a biased NS2 (e.g. 20 % too high) B is self-correcting while A is not.
C5 (SIL, `experiments/analysis/omar_ns2_closed_loop_sil.py --mode-matrix 0.7,1.0,1.3`, results `experiments/analysis/out/omar_ns2_closed_loop_sil/results_modes.json`). B reproduces INDI-alone; A adds an upward bias (bump +1.5 … +2.0 vs +1.2 cm, offset 0.16-0.21 vs 0.08 cm); with `indi 0` the two modes are identical. The SIL's INDI-alone has no crossing dip, so B's expected benefit over INDI alone cannot show there.
C6 (implementation). With `ns2_mode = 0` the new build is byte-identical to the old (`flying_drone_stack/firmware_app/host/test_omar_ns2_off_identical.py`, 4 replays, hashes a300021ed3b0 / 404a9d4373d3 / 9466b5a18dea / 81e66789889c); ARM image builds clean.
C7 (the user's own mental model, to be judged fairly). "Stack the controllers: geometric < geometric+NS2 < INDI+NS2, INDI should only fix what NS2 left over, just like NS2 on top of geometric." Version B is our attempt to implement exactly that.

## 4. What I already suspect is NOT consistent (start here)
(a) **Magnitude gap.** If C1 held with an ideal INDI, the extra target acceleration `-a_nn` (S3 flights: pred_z down to −3.4 m/s², gate mean ≈ −1.1) fed through the position PD (Kp 7, Kd 4, e'' + 4e' + 7e = −a_nn) would give an upward excursion of roughly **+12 cm on A8** (and ≈ +35 cm on A1 before the integral term acts), using the logged `bottom_rnn_pred_z` as input. Measured max upward z error in S3: **+3.5 … +6.0 cm (A8), +8.1 cm (A1)**, vs S1 +3.1 … +4.5 / +3.7. So either the mechanism is much weaker than C1 says, or something (INDI effectiveness, integral, saturation, delay) removes most of it. The SIL also shows only ≈ +0.3 … +0.8 cm of A-minus-B bump.
(b) **INDI is far from an ideal disturbance canceller in S1.** A pure 30 Hz filter lag on a ~3 m/s² / 0.2 s push explains < 1 cm of dip, yet INDI alone still dips −10.4 cm (geometric −15.7). Why? (thrust-model mismatch ≈ 1.14, motor/RPM delay, deck RPM quality, only z force considered while the downwash also tilts/torques the vehicle, saturation, the integral...). If INDI only removes ~1/3 of the push in practice, "A counts it twice" is the wrong picture and "NS2 complements a weak INDI" may be closer to the truth.
(c) Is the observed dip reduction S1 → S3 (−10.4 → −1.9) explained by NS2's extra target acceleration (A-like) or by lag compensation (B-like)? Both predict a smaller dip, so the dip cannot discriminate; what can?
(d) Version B injects `a_nn - a_nn_f` where `a_nn` is a **100 Hz zero-order-hold staircase** and the filter runs at 500 Hz: the high-pass of a staircase is a train of decaying pulses at 100 Hz. Quantify their size on the logged `rnn_pred_z` (or the SIL) and say whether B can excite the attitude/thrust loop or the INDI delay sensitivity (INDI was delay-knife-edge in earlier tests, `docs/56`).
(e) Alignment: is `filter30(a_nn)` really time-aligned with `a_rpm_f - a_imu_f` (same Butterworth, same 500 Hz update; but NS2 itself uses 100 Hz peer packets and its own ≈ 0.5 ms compute)?

## 5. What to do
1. Read the controller code and derive the closed-loop relation yourself (vehicle acceleration as a function of a_d, a_nn, r, filters, thrust-to-RPM/IMU dynamics) for A, B, INDI-alone, NS2-alone. Check every sign (NS2 hold, a_rpm, a_imu, `a_d - a_nn`, `a_indi`). Decide on C1 and C4 as mechanism.
2. Recompute C2 from the data with at least two different gate definitions and with the 500 Hz uSD logs; report means, medians, per-flight values, and whether S1 and S3 are comparable (same dz, speed, battery, time in session).
3. Reconcile (a)-(c) quantitatively: simulate the position PD + a simple vehicle/INDI model (and/or use the existing SIL / the replay tooling `experiments/analysis/indi_replay_equal_worker.py`, `docs/64_INDI_Input_Replay_Results.md`) with the real logged `a_nn`, and say what INDI effectiveness (fraction of the push actually cancelled, delay) makes the model reproduce S1 (dip −10.4) AND S3 (dip −1.9, overshoot ≤ +5.7). Is "double counting" the right description at all?
4. Run extra SIL cases (reuse `omar_ns2_closed_loop_sil.py`, add what you need in a copy, not the original): plant scale 0 and −1 (NS2 predicts, nothing there / opposite) for modes 0 and 1; a case with a deliberately delayed or noisy INDI measurement; B's staircase artefact. State which comparisons are informative and which the SIL cannot answer (perfect-prediction plant, INDI has no dip there).
5. Review Version B's code and the patch for mistakes (initial state of the filter, reset, ns2 inactive case, `rnn.div` ≠ 10, runtime switching, the fact that NS2 is evaluated under every strategy).
6. Judge C7 and the supervisor's "simple switches" wish: is A simply fine? is B justified? is there a simpler/better third option (e.g. NS2 only on the torque or only when INDI is off, scaling NS2 down, using NS2 as a prior inside the INDI measurement, adding the NS2 term to the INDI filter input instead)?
7. Design the discriminating lab test: what to fly, how many flights, which signals (`a_rpm_f`, `a_imu_f`, `rnn.pred_z`, z error, motor commands), which statistic separates A from B given our noise (n = 2-3 so far, dip std ≈ 1-4 cm), and what result would prove us wrong.

## 6. Deliverable (`docs/validation_s3_versions_2026-10-18.md`, ≤ 3 pages + appendix)
- Verdict table C1-C7 (correct / partly / wrong) with one-line evidence each.
- What is solid, what is only plausible, what is wrong, in plain words a student can follow, plus a corrected 5-line explanation we can show the supervisor.
- Recommended wording of the supervisor question and a recommended default (A, B or both) with the reason.
- The extra analyses/figures (scripts under `experiments/analysis/` with names starting `s3_validation_`), and the exact commands to reproduce.
- A list of anything in `docs/meetings/2026-10-22.md` (Questions section) or `docs/ns2_s3_versions_A_B_2026-10-17.md` that you consider incorrect or overstated, quoted, with the replacement text. Do not edit those files.
