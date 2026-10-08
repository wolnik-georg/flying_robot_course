# Cursor prompt (follow-up) — Omar z integral: steps 2–5 with the corrected diagnosis (2026-10-08)

Same rules as `docs/cursor_prompt_omar_z_integral_2026-10-08.md` (default-off bit-identical, tests after every code change, no commits, allowed files, stop rule). Read `docs/65_Omar_Z_Offset_Plan.md` section "Independent validation of Cursor step 1" first; it supersedes your step-1 conclusions.

## Corrections to step 1 (accept, do not argue; fix in your doc)
- The `acc_z` bias row is circular (bias bisected to +16.0 cm) and hardware `acc_z` is 1.000–1.003 g in steady hover: **H3 is refuted**; remove "matches" wording in `docs/66`.
- The A8 harness (`CrazyflieSIL`) routes thrust through the SIL's own thrust→PWM→RPM chain, so plant `kt` changes do not act the same way as in your solo harness; the A8 rows (kt ×0.92 → 0 cm vs solo +12.2) are not comparable. Do not use plant-kt for the diagnosis.
- Hardware shows `a_imu_fz ≈ a_rpm_fz ≈ 0` in the Omar hover flights (`cf5_thesis58/59_2026-10-02`), so INDI sees no residual, and the controller's `thrustSi` (0.354/0.343 N) is 12–15 % below the weight while hovering; position error matches `(thrustSi/MASS − g)/kp` (−0.217 m predicted, −0.214 m logged). Diagnosis H4 = commanded-vs-delivered thrust DC mismatch (factor ≈ 1.14).

## Steps
1. **Reproduce H4 in both harnesses.** Add an actuator `cmd_gain` (delivered thrust / commanded thrust, implemented by scaling the commanded RPM by sqrt(cmd_gain) with RPM→force kept consistent so INDI's `a_rpm` still equals the IMU) to (a) your solo harness (scratch result: 1.136 → +16.8 cm, 1.10 → +12.7 cm; confirm) and (b) the A8 two-drone harness: for the `CrazyflieSIL` path scale the plant output consistently (state exactly how; the injected error must not change the RPM→force relation the controller sees). Add the NS2 downwash plant from `docs/62` (symmetrized bank + torque, c = 0.0032) to the A8 case and report A8 mean z error vs hardware (Omar C +22, Rust +19 cm) for cmd_gain ∈ {1.10, 1.14, 1.17}. If A8 with downwash still cannot reach +19…+22 cm, say so with numbers; no tuning of the NS2 plant.
2. **Rust runtime parameter** `kpos_iz` (as in the original prompt step 2) — z only, default 0, clamp with `KPOS_I_LIMIT`, reset outside position mode and at takeoff/landing. Gate: default-off bit-identical (Rust-vs-C 7/7 ~1e-9, Rust host tests, replay at gain 0 unchanged).
3. **Omar C**: existing `ctrlOmarIndi.Kpos_Iz`, add the missing clamp/reset only active when `Kpos_Iz != 0`. C ≡ Rust target ≤ 1e-6 with a nonzero gain.
4. **SIL gain grid** `Kpos_Iz` ∈ {0, 0.25, 0.5, 1, 2} × {solo hover, A8 (with NS2 plant), A1 (with NS2 plant), figure-8} at cmd_gain = 1.14, plus a robustness row at 1.10 and 1.17 for the best gain (a gain must not rely on one exact mismatch). Metrics and acceptance as in the original prompt (mean z error within ±3 cm, gyro RMS not > 20 % above gain 0, crossing dips not deeper, no ringing). Also run cmd_gain = 1.00 at the chosen gain (integral must not hurt a correctly matched plant) and the hardware-motivated check that the integral does not wind up during takeoff/landing.
5. **Update `docs/66_Omar_Z_Integral_Results.md`**: corrected diagnosis (H3 refuted, H2-as-kt refuted, H4), diffs summary, default-off regression, grid, recommended gain or "none satisfies", limits, proposed lab sequence (INDI hover → A8 → A1, abort criteria as `docs/58`). Do not edit docs/65.

## Report back
Per step: files, commands, numbers, pass/fail vs gates; default-off regression after each code change; what you could not verify.
