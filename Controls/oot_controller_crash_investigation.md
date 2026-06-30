# OOT Controller Crash Investigation (June 2026)

## Summary

After a stable period (up to ~June 20), the OOT Rust geometric controller began crashing on every flight while the built-in Lee controller remained stable. Three different drones and multiple hardware swaps were tested over ~1 week. Root cause traced to a **firmware build inconsistency** introduced by uncommitted C-side changes.

## Symptom

- Geometric mode (ctrl_mode=0): drone lifts, oscillates, crashes within 2–5 seconds
- Lee/Mellinger (built-in): stable hover and trajectories
- Consistent across 3 drones, multiple propeller sets, both with and without reflective tape on RPM deck

## Timeline

| Date | Event |
|------|-------|
| June 20 | Last known stable commit (`66a3d49` + CS2 `ba0ff4a7`). `built-in.o` was compiled from **uncommitted** traj_iface.c that had `indi_log_write` + `LOG_GROUP(indi)` + `PARAM_GROUP(pos_gains)` |
| June 20+ | Firmware changes broke the implicit C-Rust interface. Binary inconsistency. Crashes begin. |
| June 30 | Checked out both repos to June 20 commits; applied missing `indi_log_write` bridge to traj_iface.c; rebuilt clean; flights stable. |

## Hypotheses

| # | Hypothesis | Evidence | Status |
|---|-----------|----------|--------|
| H1 | Changed CS2 params (KP/KV/mass) destabilise outer loop | CS2 YAML identical to June 20 at ba0ff4a7 | Ruled out |
| H2 | KT_MOTOR values wrong for new drone | Same KT values used during stable period | Possible but not root cause |
| H3 | Rust firmware binary inconsistency (Rust compiled against stale C headers) | `cargo clean` + `make` restored stability | **Root cause** |
| H4 | INDI gains wrong (KR/KW asymmetry) | Not activated in mode 0 geometric | Ruled out (mode 0 only) |
| H5 | Position integral windup | KI_P=0.05, KI_LIMIT=2.0 — small | Unlikely |
| H6 | CS2 motor shutdown bug mid-trajectory | Observed, separate issue in flight.py | Separate issue (not crash cause) |
| H7 | RPM deck reflective tape interfering with propeller aerodynamics | Same crash without tape | Ruled out |
| H8 | Motor or propeller damage | Third drone with new motors also crashed | Ruled out |
| H9 | Reflective tape on propellers causing imbalance | Third drone: NO tape, still crashed | Definitively ruled out |

## Root Cause

At commit `66a3d49`, the committed `built-in.o` was built from an **uncommitted version** of `traj_iface.c` that included:
- `indi_log_write()` and `indi_tau_write()` C functions (called by Rust via FFI)
- `LOG_GROUP(indi)` — required by CS2 YAML `indi_state`/`indi_alp_raw` log topics
- `PARAM_GROUP(pos_gains)` — required by CS2 YAML `pos_gains` firmware params

The committed `traj_iface.c` at that commit did NOT have these. So:
1. Checking out `66a3d49` and building from source → linker error (undefined `indi_log_write`)
2. Any post-June-20 changes that modified traj_iface.c → broke the implicit interface
3. CS2 tried to subscribe to `indi.alp_raw_x` → "not found in log TOC" → CS2 crash or partial flight

## Fix Applied (on stable-jun30-working branch)

Added the missing bridge to `traj_iface.c` (commit `4940f53`):

```c
static float log_alp_raw_x, log_alp_raw_y, log_alp_raw_z;
static float log_alp_x, log_alp_y, log_alp_z;
static float log_tau_x, log_tau_y, log_tau_z;

void indi_log_write(float arx, float ary, float arz, float ax, float ay, float az) {
    log_alp_raw_x = arx; log_alp_raw_y = ary; log_alp_raw_z = arz;
    log_alp_x = ax; log_alp_y = ay; log_alp_z = az;
}
void indi_tau_write(float tx, float ty, float tz) {
    log_tau_x = tx; log_tau_y = ty; log_tau_z = tz;
}
LOG_GROUP_START(indi)
  LOG_ADD(LOG_FLOAT, alp_raw_x, &log_alp_raw_x)
  LOG_ADD(LOG_FLOAT, alp_raw_y, &log_alp_raw_y)
  LOG_ADD(LOG_FLOAT, alp_raw_z, &log_alp_raw_z)
  LOG_ADD(LOG_FLOAT, alp_x, &log_alp_x)
  LOG_ADD(LOG_FLOAT, alp_y, &log_alp_y)
  LOG_ADD(LOG_FLOAT, alp_z, &log_alp_z)
  LOG_ADD(LOG_FLOAT, tau_x, &log_tau_x)
  LOG_ADD(LOG_FLOAT, tau_y, &log_tau_y)
  LOG_ADD(LOG_FLOAT, tau_z, &log_tau_z)
LOG_GROUP_STOP(indi)
```

Also: always run `cargo clean` before `make` when changing C-side or Rust-side code.

## Verification

After fix: geometric hover ✅, figure-8 ✅ (multiple repetitions on June 30, 2026).

## Open Items

1. **INDI hover** — still unstable. Requires recomputing `kt1`–`kt4` for current drone (use a stable Lee hover log from June 30 flights, quick mode, `compute_kt_motor.py`).
2. **CS2 motor shutdown bug** — motors stop mid-trajectory in some flight.py configurations. Separate from the crash issue; investigate CS2 stop condition.
3. **M3/M4 RPM imbalance** — telemetry shows M3/M4 consistently lower RPM than M1/M2. Worth investigating after INDI is stable (could affect KT calibration).
