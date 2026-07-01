# OOT Controller Crash Investigation (June 2026)

## Summary

After a stable period (up to ~June 20), the OOT Rust geometric controller began crashing on every flight while the built-in Lee controller remained stable. Three different drones and multiple hardware swaps were tested over ~1 week. Investigation revealed **two independent issues occurring simultaneously**, which made diagnosis significantly harder — each issue masked the other and symptoms overlapped.

**Issue 1 (firmware)** — root cause of geometric crashes: a firmware build inconsistency introduced by uncommitted C-side changes. Fixed June 30. Geometric now stable.

**Issue 2 (hardware)** — RPM deck readings intermittent and unreliable due to reflective sticker problems. Not the cause of geometric crashes (geometric doesn't use RPM), but the direct cause of INDI instability: INDI's torque model depends on accurate per-motor RPM; bad readings corrupt the control law. Still open.

## Symptom

- Geometric mode (ctrl_mode=0): drone lifts, oscillates, crashes within 2–5 seconds
- Lee/Mellinger (built-in): stable hover and trajectories
- Consistent across 3 drones, multiple propeller sets, both with and without reflective tape on RPM deck

## Timeline

| Date | Event |
|------|-------|
| June 20 | Last known stable commit (`66a3d49` + CS2 `ba0ff4a7`). `built-in.o` was compiled from **uncommitted** traj_iface.c that had `indi_log_write` + `LOG_GROUP(indi)` + `PARAM_GROUP(pos_gains)` |
| June 20+ | Firmware changes broke the implicit C-Rust interface. Binary inconsistency. Geometric crashes begin. INDI also unstable — at this point believed to be same root cause but actually a separate hardware issue co-occurring. |
| June 20–30 | ~1 week of debugging across 3 drones. Both issues active simultaneously. Hardware swaps (new motors, no reflective tape) ruled out aerodynamic/mechanical causes but could not isolate the firmware inconsistency because RPM deck was also unreliable. |
| June 30 | Checked out both repos to June 20 commits; applied missing `indi_log_write` bridge to traj_iface.c; rebuilt clean. Geometric hover + figure-8 stable. INDI still unstable — now understood to be the RPM deck issue, not firmware. |
| June 30 (analysis) | Per-flight RPM classification of all June 30 logs revealed: 13/19 flights had all-zero RPM (deck not reading); 3 flights had M3 reading ~18k–20k RPM instead of expected ~25k (sticker misaligned); only 3 flights had clean balanced readings. |

## Hypotheses

### Issue 1 — Geometric controller crashes

| # | Hypothesis | Evidence | Status |
|---|-----------|----------|--------|
| H1 | Changed CS2 params (KP/KV/mass) destabilise outer loop | CS2 YAML identical to June 20 at ba0ff4a7 | Ruled out |
| H2 | KT_MOTOR values wrong for new drone | Same KT values used during stable period | Possible but not root cause |
| H3 | Rust firmware binary inconsistency (Rust compiled against stale C headers) | `cargo clean` + `make` restored stability | **Root cause ✅ confirmed** |
| H4 | INDI gains wrong (KR/KW asymmetry) | Not activated in mode 0 geometric | Ruled out (mode 0 only) |
| H5 | Position integral windup | KI_P=0.05, KI_LIMIT=2.0 — small | Ruled out |
| H6 | CS2 motor shutdown bug mid-trajectory | Observed, separate issue in flight.py | Separate issue, not crash cause |
| H7 | RPM deck reflective tape interfering with propeller aerodynamics | Same crash without tape | Ruled out |
| H8 | Motor or propeller damage | Third drone with new motors also crashed | Ruled out |
| H9 | Reflective tape on propellers causing imbalance | Third drone: NO tape, still crashed | Definitively ruled out |

### Issue 2 — INDI instability (independent of geometric crashes)

| # | Hypothesis | Evidence | Status |
|---|-----------|----------|--------|
| H10 | RPM deck intermittently not reading (all zeros) | 13/19 June 30 flights: all motors zero. INDI falls back to τ_prev — open-loop torque, no RPM feedback → unstable | **Confirmed** |
| H11 | Reflective sticker misaligned on M3 | M3 reads 18k–20k RPM (vs ~25k expected) in 3 flights; M1/M2/M4 correct; no code change between those flights and the good ones | **Confirmed** |
| H12 | RPM deck hardware failure | No evidence — same deck gives good readings when stickers are properly attached | Unlikely, still open |
| H13 | KT_MOTOR values wrong for current drone | Old kt1–4 from June 18 drone; recomputed June 30 from clean Lee hover: kt1=1.4193e-10, kt2=1.4534e-10, kt3=1.4997e-10, kt4=1.4655e-10 | **Fixed** |

**Why both issues co-occurring made debugging so hard**: geometric crashes (Issue 1) looked like they could be caused by RPM deck noise affecting the control loop. INDI instability (Issue 2) looked like it could be the same firmware bug. Neither issue had a clean independent signature until geometric was fixed and RPM logs were analysed per-flight.

## Root Causes

### Issue 1 — Firmware build inconsistency (geometric crashes)

At commit `66a3d49`, the committed `built-in.o` was built from an **uncommitted version** of `traj_iface.c` that included:
- `indi_log_write()` and `indi_tau_write()` C functions (called by Rust via FFI)
- `LOG_GROUP(indi)` — required by CS2 YAML `indi_state`/`indi_alp_raw` log topics
- `PARAM_GROUP(pos_gains)` — required by CS2 YAML `pos_gains` firmware params

The committed `traj_iface.c` at that commit did NOT have these. So:
1. Checking out `66a3d49` and building from source → linker error (undefined `indi_log_write`)
2. Any post-June-20 changes that modified traj_iface.c → broke the implicit interface
3. CS2 tried to subscribe to `indi.alp_raw_x` → "not found in log TOC" → CS2 crash or partial flight

### Issue 2 — RPM deck / reflective sticker problem (INDI instability)

The RPM deck has no 1-Wire ID and requires `deck.bcRpm: 1` (set in CS2 YAML) to be force-enabled at boot. **No firmware or code changes affect how the deck reads RPM** — the IR sensor driver is entirely in crazyflie-firmware which was not modified.

June 30 per-flight analysis (flights from after stickers were first applied):

| Flights | Time | RPM reading | Assessment |
|---------|------|-------------|------------|
| 8 | 20:04 | All motors ~23.5k–24.2k ✓ | Stickers just applied, reading OK |
| 9–14 | 20:09–20:18 | All zero | Deck lost signal — sticker fell off or deck connector loose |
| 15–17 | 20:32–20:33 | M3 = 18k–20k, others ~25k | M3 sticker misaligned — 4,600–6,700 RPM below expected |
| 18–19 | 20:38–20:40 | All motors 23.9k–25.1k ✓ | Sticker re-seated or self-corrected |

**Impact on INDI**: with M3 reading 18k instead of 25k, INDI computes `KT × 18000²` = 52% of the correct M3 thrust. The torque model is wrong by ~50% on one motor → roll instability guaranteed. With all-zero RPM, INDI falls back to `τ_prev` entirely (open-loop torque) → always unstable.

**Lee and geometric are unaffected** by RPM deck readings — they compute torque purely from attitude error and do not use motor RPM.

### Cross-session RPM analysis — confirming sticker root cause (June 30 analysis)

To determine whether the issue was **RPM deck hardware** (fixed IR sensor failure) or **reflective sticker placement** (varies with each application), all available logs from June 20–27 were analysed from the Trash (these sessions had RPM logging active but the files were deleted after review).

Mean RPM per motor across sessions:

| Session | M1 | M2 | M3 | M4 | Flagged motor | Notes |
|---------|-----|-----|-----|-----|--------------|-------|
| Jun 20 (5 flights) | ~24.5k | ~23.7k | ~24.1k | ~22.5k | **M4** slightly low | Consistent across all 5 |
| Jun 22 (4 flights) | ~24.1k | ~23.9k | ~23.9k | ~22.7k | **M4** slightly low | Same M4 pattern |
| Jun 24 (3 flights) | ~24.3k | ~23.7k | ~23.8k | ~22.9k | **M4** slightly low | Same M4 pattern |
| Jun 25 (many flights) | varies | 9k–25k | varies | 22k–24k | **M2** intermittent | M2 drops to 9–18k in some flights, OK in others |
| Jun 27 (9 flights) | ~24k | **6k–15k** | ~24k | ~23k | **M2** catastrophic | M2 reads 6k–14k in 8 consecutive flights |
| Jun 30 (flights 15–17) | ~25k | ~25k | **18k–20k** | ~25k | **M3** low | After re-applying stickers |

**Key finding**: the motor that reads badly changed across sessions — M4 (Jun 20–24) → M2 (Jun 25–27) → M3 (Jun 30). If the RPM deck hardware had a failed IR sensor, the same physical motor position would fail every time, because the deck and its sensors are fixed to the frame. The fact that the bad motor changed each time stickers were reapplied **confirms the sticker is the root cause**, not the deck electronics.

Most likely mechanism: the reflective sticker must pass directly in front of its motor's IR sensor during rotation. A sticker that is off-centre, too high/low on the hub, or poorly adhered (peeling, folded edge) causes the sensor to miss reflections or read at wrong angular positions → anomalously low RPM count. Even a small misalignment can drop readings by 30–50%.

**Diagnostic rule**: after any sticker application or propeller swap, always fly one geometric hover and check the `_rpm_balance.png` plot before attempting INDI. Do not proceed with INDI if any motor reads more than ~2k RPM below the others.

**Sticker application guide for reliable readings**:
- Place sticker on the very outer edge of the propeller hub (not the centre, where linear velocity is too low)
- Same angular position on all 4 motors (e.g. aligned with one arm of the hub)
- Press firmly; ensure no lifted edges
- After application: hover geometric → inspect RPM balance plot → all 4 motors should read within ±2k of each other at hover thrust

## Fixes Applied

### Issue 1 — Firmware (commit `4940f53` on stable-jun30-working)

Added the missing bridge to `traj_iface.c`:

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

1. **INDI hover** — not yet confirmed stable. KT values recomputed June 30 from clean Lee hover (kt1=1.4193e-10 … kt4=1.4655e-10, updated in crazyflies.yaml). Fresh stickers applied June 30 with new propellers. Next step: fly geometric hover → verify RPM balance plot shows all motors within ±2k → attempt INDI hover.
2. **CS2 motor shutdown bug** — motors stop mid-trajectory in some flight.py configurations. Separate from the crash issue; investigate CS2 stop condition.
3. **Sticker long-term reliability** — even with correct initial placement, stickers have come loose or shifted within a session (Jun 30 flights 9–14 went all-zero after an initial good reading). Consider whether stronger adhesive, a different sticker material, or a conformal coating would help.
