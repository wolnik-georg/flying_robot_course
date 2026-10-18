# S3 = INDI + NS2: decision (closed 2026-10-18)

**Decision: S3 is INDI and NS2 as two plain switches. A second variant ("Version B": INDI only on what NS2 left over, switch `ctrlOmarIndi.ns2_mode`) was built on 2026-10-17, reviewed on 2026-10-18 and removed again; it was never flashed or flown.** The controller tree is S0 geometric, S1 INDI, S2 geometric + NS2, S3 INDI + NS2, exactly as validated on 2026-10-15; the only change for the study is the retrained network `ns2_v2` in S2 and S3.

## How S3 is built (Omar C controller, `controller_omar_indi.c`, position loop)
```c
a_d = acc_d + Kpos_D*vel_err + Kpos_P*pos_err + i_term;     // position PD (+ z integral 1.5)

if (ns2_active())                                            // switch C, rnn.en = 1 (S2, S3)
    a_d = a_d - a_nn;                                        // NS2 prediction of the push, subtracted like a measured residual

struct vec a_indi = vzero();
if ((indi & 1) && rpm_deck_available) {                      // switch A, indi bit 0 (S1, S3)
    a_indi = a_rpm_filtered - a_imu_filtered;                // INDI: measured push (30 Hz), with the opposite sign
}
thrustSi = mass * dot(a_d + a_indi, R*e3);                   // both enter the thrust additively
F_d      = a_d + a_indi;                                     // and the desired attitude
```
S0: neither switch; S1: INDI only (`indi 3`); S2: NS2 only (`indi 0`, `rnn.en 1`); S3: both (`indi 3`, `rnn.en 1`). NS2 is evaluated and logged under every strategy and applied only when `rnn.en = 1`.

## The correct understanding of S3
- **NS2 and INDI both act on the same downwash push:** NS2 predicts it (feed-forward), INDI measures it. Both are added to the thrust command.
- **INDI measures the force on the vehicle, not the remaining error.** It therefore reads the same push whatever compensates it: in the A8 gate (offline reconstruction with the controller's definition) S0 −1.20/−1.07, S1 −1.03/−1.16/−1.13, S2 −1.23/−1.23, S3 −1.18/−1.27 m/s², against a baseline of −0.06.
- **With an ideal INDI that would be a double correction** (the sum would push the vehicle up by about the size of the push). **Our INDI is not ideal:** at the crossings only 64–72 % of the thrust rise it commands arrives (S1 64/64/72 %, S3 71/70 %; S0 and S2 96–109 %), INDI alone removes only about a third of the dip, NS2 about two thirds. The two corrections therefore complement each other instead of doubling: A8 crossing dip S0 −15.7, S1 −10.6, S2 −5.5, **S3 −1.9 cm** (n = 2–3), upward peak in S3 +1.5 cm on average (max +5.7).
- **Likely cause of the weak INDI (plausible, not proven):** motor headroom. With INDI the busiest motor sits at a median of 0.74–0.85 of full range (0.58–0.62 without), 0.3–8.8 % of the samples above 0.95; the power distribution then cuts collective thrust to keep the torques. Both measurements reproduced independently on 2026-10-18.
- **Why Version B was dropped:** B would let INDI handle "the rest" and keep only NS2's part above about 30 Hz (0.04 m/s² RMS on the logged prediction). With the INDI as flown that predicts a dip like INDI alone (−10 … −11 cm), i.e. most of the gain of S3 would be lost. The reasoning that B is better rested on a strong INDI, which the hardware contradicts. The earlier statement "the measured push in S3 equals S1, therefore the push is corrected twice" was wrong: the equality holds in all four strategies.

## What was removed and how it was checked (2026-10-18)
- Firmware: `controller_omar_indi.{c,h}` restored to the version before `ns2_mode`; regenerating the patches reproduces the previously committed patches byte for byte (`firmware_patches/`, snapshots in `firmware_app/host/`); `ns2c_bench_check.py` and `omar_ns2_closed_loop_sil.py` restored; yaml variant S3B deleted.
- Host build rebuilt: `ns2_mode` no longer exists; the four logged replays are byte-identical to the pre-switch build (sha256 a300021ed3b0, 404a9d4373d3, 9466b5a18dea, 81e66789889c).
- ARM image `study_tree_v2.bin` rebuilt: 532 952 B (v1: 532 960 B), sha256 `3ae7304a31d0f01ea1287c6e4012cfab97ea5d1126bfde462664751572215795`; contains the `ns2_v2` weights and not the v1 weights, no `ns2_mode`, no `a_nn_fz`; NOT yet flashed.
- Yaml variants for the lab (apply on the current main yaml = S0): `crazyswarm2/crazyflie/config/study/from_main_S1.patch`, `from_main_S2.patch`, `from_main_S3.patch`.

## Open points (none blocks the study)
- The S1 flights of 2026-10-15 ran partly at 3.44–3.49 V (S1 dip −11 cm at low battery, −9.2 cm at 3.86 V): re-fly S1 on fresh batteries (≥ 3.8 V) in the validation session, on the new bottom drone.
- If the new drone or a controller change makes INDI effective, S3 would start to overshoot (an ideal INDI plus NS2 predicts about +8 cm): look at the upward peak of S3 against S1 in the validation flights.
- Optional diagnostic of the headroom explanation (not needed for the study): S1 with `indi = 1` (position INDI only, no moment term) should dip less than `indi = 3` with less motor saturation. Changing the study's INDI definition would be a separate decision.

## Record
Review with all numbers and reproduction commands: `docs/validation_s3_versions_2026-10-18.md`, scripts `experiments/analysis/validation_s3_*.py`, results `experiments/analysis/out/validation_s3/`. The earlier write-up `docs/ns2_s3_versions_A_B_2026-10-17.md` and the SIL runs with `ns2_mode` (`experiments/analysis/out/omar_ns2_closed_loop_sil/*modes*`, `*validation_2026-10-18*`) are kept as history; they describe the removed variant, and the SIL has a near-ideal INDI (dip 0.75 cm, hardware 10.6 cm), so it cannot rank A and B for the hardware.
