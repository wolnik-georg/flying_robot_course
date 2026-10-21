# FBL vs the controller tree — simulation only, one plant (2026-10-20, updated 2026-10-21)

![sim comparison](../../experiments/fbl/sim_compare.png)

- **Plant:** `c_in_the_loop_real.py` (motor lag 44 ms, delay 4 ms, noise; truth downwash = NS2 network). Controllers = the real firmware C code (Omar tree and FBL branch, tuned preset). A8 at 0.5 m/s, A1 dz 0.5, A4 line 0.5 m/s. Script `experiments/fbl/stage3_port/sim_compare.py`, data `out/stage3/sim_compare*.json`.
- **S1 and S3 (INDI) are not in this plant:** they need the RPM-based plant of the two-drone harness, where INDI looks perfect (`docs/75` §3), so a simulation ranking of INDI would be meaningless anyway.
- **Bottom row = most realistic plant:** thrust as measured from our hover logs (97.7 % of the command arrives, `ROBUSTNESS_2026-10-21.md`) and the downwash 1.3× the prediction (the factor that reproduces the hardware S0 dip in the two-drone harness). Black diamonds = real hardware flights. A8 and A4 of S0 and S2 land close to the diamonds (A8 S0 13.1 vs 12.5, S2 6.9 vs 5.6; A4 0.05 S0 15.1 vs 17.2); the simulated A1 sag is larger than on the hardware (S0 36.8 vs 17.7).
- **Result in that plant:** FBL + downwash model A8 dip 1.0 cm, A1 0.2 cm, A4 2.8 / 2.2 / 2.6 cm RMS; S2 6.9 / 14.6 / 6.2 / 5.4 / 5.0; S0 13.1 / 36.8 / 15.1 / 12.6 / 10.9.
- **Ideal plant (top row):** S2 is as good as or better than FBL because the NS2 prediction is exactly the plant's downwash (upper bound); FBL's physics model is only an approximation (fit R² 0.4 against NS2's 0.8). With realistic thrust and downwash the picture flips in FBL's favour.
- **Reading:** in simulation FBL is clearly better than S0 and S2. Still a simulation: the downwash truth is the NS2 network the lookup table was fitted to, and the plant has no ground effect or battery sag. Whether FBL's physics model or NS2's network wins on the real downwash is what the lab flights decide.
