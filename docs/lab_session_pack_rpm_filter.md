# Lab pack — RPM-filter confirmation on OUR INDI (DShot) — next lab session (prepared 2026-10-08)

**Question:** does the sentinel hold-last-good fix (`RPM_FILTER_HOLD_SENTINEL=1`) change anything — good or bad — on our INDI (`ctrl_mode 3`, `rpm_source 1` DShot)? **Who uses what:** ours and Omar Rust read DShot via `rpm_get_all()` (filter active); Omar C reads the optical deck (no filter).
**Already known (desk, 2026-10-08):** motor output at DShot-sentinel instants is indistinguishable from random instants in legacy flights (Rust 10-02 ratio 1.05, ours 10-02 1.03) AND with the fix on (Rust 10-08, 6 flights, ratio 1.01); sentinel = 0.057 % of samples. Unit tests pass. Missing: our INDI with the fix on, same-day OFF/ON pair.

## Firmware (flash only cf5, from the laptop; stop CS2 first; power-cycle after)
| file | flag | sha256 |
|---|---|---|
| `build_artifacts/cf21bl_omar_iz_legacy.bin` (OFF) | `RPM_FILTER_HOLD_SENTINEL` unset (= 10-05 behaviour) | `004872db40ccbc72ab66d0d7d461819c44119421ea8d2047e316e98409a4f372` |
| `build_artifacts/cf21bl_omar_iz_sentinel.bin` (ON) | `=1` | `c24de1f10b3561d5f5629645831498350818a84cea941fd5f3bb488d5557953e` |
Both contain `kpos_iz`; neither contains the NS2 network (reflash `cf21bl_rnn_100hz.bin` afterwards for NS2). The `.bin` files are gitignored — they exist on the laptop only; rebuild: `cd flying_drone_stack/firmware_app && make DRONE=bl [RPM_FILTER_HOLD_SENTINEL=1]` then copy `build/cf21bl.bin`.
```bash
cfloader flash <file> stm32-fw -w radio://0/80/2M/E7E7E7BB02
```

## cf5 yaml (TEMPORARY — push by Claude; restore afterwards)
```yaml
stabilizer: {controller: 6}
indi_gains: {ctrl_mode: 3, rpm_source: 1, res_sign: 1}   # res_sign +1 with ctrl_mode 3 (rule: study_parameter_decisions.md)
pos_gains: {ki_z: 0.0}                                    # NEVER 16 with ctrl_mode 3
rnn: {en: 0}
```
(The exact commit used on 10-08 evening is crazyswarm2 `7029bdf`; defaults restored in the following commit.)

## Plan
- **Option A (recommended, ~15 min):** firmware ON (already what cf5 would carry from the 10-08 block, otherwise flash it) → A8 ×2. Compare with the 10-02 legacy flights of ours (ctrl_mode 3, ki_z 0: +3.4 cm mean, ≈ 8 cm max).
- **Option B (strict, ~45–60 min):** flash OFF → A8 ×2 → flash ON → A8 ×2 (same day, same batteries).
- Flights: `run_formation --scenario A8 --dz 0.5 --height 0.5 --passes 4 --auto-center --yes`, pose bag per flight, fresh batteries, abort at roll/pitch > 25° for 0.5 s or z < 0.25 m. A8 only (our INDI oscillates on A1).

## Pass criteria
1. No crash / no new oscillation (gyro_x sd not above the 10-02 level; ours ON vs OFF within flight-to-flight spread).
2. Steady z error within ±5 cm; ON vs OFF difference < 2 cm (B) or comparable to 10-02 (A).
3. Sentinel-instant / random-instant motor-output ratio ≈ 1 for ON (script: the inline analysis of 2026-10-08, see `docs/67` in-flight section — to be saved as `experiments/analysis/rpm_sentinel_motor_response.py` after the lab).
4. Count of raw sentinels per flight ≈ 20–40 (0.05–0.1 %).
**Verdict rule:** all four hold → keep the fix ON as the study default; any new oscillation or a z shift → switch back to the legacy build and document.

## After the session
Restore cf5 yaml (`ctrl_mode 0`, `res_sign -1`, `ki_z 16`), reflash `cf21bl_rnn_100hz.bin` if NS2 flights follow, process cards as usual (cf_second first).

## Added 2026-10-09 — prepared material
- **Unified study firmware (network + `kpos_iz` + filter), laptop only (`build_artifacts/`):** `cf21bl_study_rnn_iz_sentinel_ON.bin` sha256 `989e5150f4d68690d319fa9320c1f947e36f7996caca329db669a512d3079121` (`RPM_FILTER_HOLD_SENTINEL=1`), `cf21bl_study_rnn_iz_sentinel_OFF.bin` sha256 `9dafe3489c15a584874b9da37786a1405e0354fe582e384101e2c013d1c83e1d`. Flash 484 676 / 484 668 B (47 %), RAM 76 %. NOT yet bench-validated: after flashing run `read_rnn_timing.py` (expect `us_max` ≪ 1000 µs as before) and one hover, then one NS2 sanity flight (A8, `res_sign -1`, expect ≈ −3 cm) before using it for data. Build: `make DRONE=bl all-rnn-flash [RPM_FILTER_HOLD_SENTINEL=1]`.
- **Yaml patch (temporary INDI config), applies cleanly:** `cd ~/georg/ros2_ws/src/crazyswarm2 && git apply ~/georg/flying_robot_course/docs/lab_prep_ours_indi_filter_yaml.patch` (and `git checkout crazyflie/config/crazyflies.yaml` to undo). Claude can also push it as a commit on request.
- **Analysis after the flights:** `~/.pyenv/versions/flying_robots/bin/python experiments/analysis/rpm_sentinel_motor_response.py --label "ours ON" experiments/logs/usd_raw/cf5_A8_thesis0*_<date>_<stamp>.bin` (reference: Rust ON 10-08 → ratio 1.01; legacy Rust 10-02 1.05; ours legacy 10-02 1.03).
