# INDI input replay across the three controllers — plan (2026-10-05)

**Origin:** meeting 2026-10-05, next steps: "take input data for INDI from different flights and let all 3 controllers run exactly the same input numbers and compare output numbers; theoretically all should agree, if not there are differences in code / gains / filters."
**Status:** plan + Cursor prompt (`cursor_prompt_indi_input_replay_2026-10-05.md`). Desk only, no lab, no firmware change. Feeds the "which INDI variant is the thesis Pure INDI" decision; runs after the NS2 sign-test lab session is planned, in parallel with it.

## Question
Given the **same measured inputs per tick** (state, gyro, acceleration, commanded setpoint, per-motor RPM), do **ours** (controller 6, `ctrl_mode=3`), **Omar C** (controller 9) and **Omar Rust** (controller 10) produce the same thrust and torque outputs? If not: which difference (gains, filters, structure) explains it?

## What exists already (reuse, do not rebuild)
- Host bindings that run the same source as the drone: `flying_drone_stack/firmware_app/host/README.md` (build, `PYTHONPATH=~/Desktop/crazyflie-firmware/build`, one process per controller because controller `State` persists across runs).
- C-vs-Rust per-tick comparison of Omar's two ports (~1e-9, `docs/41`): `host/_omar_indi_case_runner.py`, `_omar_indi_rust_case_runner.py`, `test_omar_indi_rust_vs_c.py`. **Omar C ≡ Omar Rust is therefore already known; the new part is ours vs Omar.**
- Our INDI in the loop harnesses: `flying_drone_stack/tests/indi_*_harness.rs`, `indi_harness_bisect.rs`; SIL pipeline `experiments/analysis/indi_sil_*`; delay/loop-rate ledger `docs/53`, `docs/54`; structure/motor-model comparison `docs/55`, `docs/41`.
- Flight data: uSD logs (500 Hz, 54 vars) in `experiments/logs/usd_raw/`, with `usd_run_tag` in the matching `*.meta.json`; decoder `flying_drone_stack/tools/decode_usd_log.py`. 2026-10-02 holds the 3-way comparison flights (cf5: Omar C → Omar Rust → ours `ctrl_mode=3`, A8 and A1).

## Design
1. **Input flight(s).** (Corrected after review: use A8 `19-09-54` / `cf5_thesis69` and A1 `19-15-47` / `cf5_thesis72` from 2026-10-02; `A8 18-59-17` is a pre-fix tumble. Host bindings are the Python **3.10** extension: run with system `python3`, not the 3.12 pyenv.) One clean A8 flight with RPM available (primary, cf5 bottom drone, ours `ctrl_mode=3`, from 2026-10-02; pick by meta + run tag) and one A1 flight (second). Inputs are replayed **open loop**: the logged measured values go in, the controllers' outputs are compared; no plant, no feedback. (The replayed flight's own controller was one of the three; that is fine and noted.)
2. **Per-tick input vector:** position, velocity, attitude (from logged roll/pitch/yaw deg), gyro (deg/s), acceleration, setpoint pos/vel/acc/yaw, four RPM. **Gaps to close first:** logged channels contain only `ctrltarget_x/y/z` for the setpoint, so setpoint velocity/acceleration must be rebuilt from the scenario trajectory (`meta.json` params; same generator as `run_formation`). Log rate 500 Hz vs controller 1 kHz: hold each sample twice (document).
3. **Warm-up:** each controller carries filter and history state; start replay at a steady hover segment and discard the first N ticks (state it).
4. **Compared outputs (common currency):** total thrust [N], body torque [N·m] or angular-acceleration command, and the intermediate desired force vector f_d and attitude error where each exposes it. Where a controller does not expose an intermediate, say so; do not infer it.
5. **Three comparison levels** (to attribute differences):
   - **L0 native:** each controller with its own default gains/filters.
   - **L1 harmonized gains:** same position/attitude gains where the structure allows (document what cannot be mapped).
   - **L2 harmonized filters:** same gyro LPF / RPM filter settings where switchable (our config exposes `filt_dt_us`, notch, prewarp; Omar's filtered-gyro path is fixed, see `docs/41`).
6. **Metrics:** per-tick difference (RMS and max), correlation, and gain/phase of output vs output over frequency (shows delay/filter differences, not only offsets); per-axis. Plot as SVG (matplotlib-free pattern already used in `indi_*_plot_svg.py`).
7. **Attribution table:** expected structural differences from `docs/41` (no gyroscopic term in Omar's, per-motor `kt1..kt4` vs single `MOTORRPM2FORCE`, filtered gyro, PWM-fallback RPM) vs measured differences. Anything unexplained is a finding, not something to tune away.

## Pitfalls to state in the report
- Controller `State` persists within a process: one fresh process per controller per run.
- Units: logged gyro deg/s and angles deg; controllers expect rad (see the case runners).
- Open-loop replay is **off-policy** for two of the three controllers: equal inputs, but not the inputs they would have produced themselves. That is the point of the test, and also its limit; do not read output differences as flight-performance differences.
- RPM source: logged deck RPM vs DShot differ by < 500 RPM typically; use one source for all three and say which. Spikes (> 2000 RPM apart) are removed as in `meeting_simple_rpm_deck_dshot.py`.

## Success criteria
- A table for A8 and A1: L0/L1/L2 output differences for thrust and torque, with the largest unexplained residual named.
- Our-vs-Omar differences attributed to gains, filters or structure with evidence, or explicitly left unexplained.
- Omar C ≡ Omar Rust re-confirmed on the same inputs (sanity check of the harness).
- Written up as `docs/64_INDI_Input_Replay_Results.md`; one paragraph for the INDI-variant decision (what the replay does and does not say).

## Not in scope
Closed-loop SIL (separate, `docs/58`–`61`), gain tuning, firmware changes, new flights.
